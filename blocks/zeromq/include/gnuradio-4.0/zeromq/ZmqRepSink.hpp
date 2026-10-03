#pragma once

#include "detail/ZmqCommon.hpp"
#include "detail/ZmqTagHeaders.hpp"
#include "trait_helpers.hpp"

#include <algorithm>
#include <cstring>
#include <expected>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>
#include <gnuradio-4.0/zeromq/detail/ZmqPmtCodec.hpp>
#include <optional>
#include <vector>

namespace gr::blocks::zeromq {

template<typename T>
concept ZmqRepSinkAcceptableTypes = std::is_same_v<T, std::vector<typename T::value_type, typename T::allocator_type>> || std::is_same_v<T, std::complex<typename T::value_type>> || std::is_integral<T>::value || std::is_floating_point<T>::value || std::is_same_v<T, gr::pmt::Value>;

namespace detail {

inline std::vector<ZmqTagHeaderRecord> select_pending_tags(const std::vector<ZmqTagHeaderRecord>& pending, std::size_t count) {
    std::vector<ZmqTagHeaderRecord> selected;
    for (const auto& tag : pending) {
        if (tag.offset < count) {
            selected.push_back(tag);
        }
    }
    return selected;
}

inline void commit_pending_tags(std::vector<ZmqTagHeaderRecord>& pending, std::size_t consumed) {
    std::vector<ZmqTagHeaderRecord> remaining;
    remaining.reserve(pending.size());
    for (auto& tag : pending) {
        if (tag.offset >= consumed) {
            tag.offset -= consumed;
            remaining.push_back(std::move(tag));
        }
    }
    pending = std::move(remaining);
}

} // namespace detail

template<ZmqRepSinkAcceptableTypes T>
class ZmqRepSink : public gr::Block<ZmqRepSink<T>> {
public:
    using Description = Doc<R""(
@brief ZMQ REP Sink.

This block receives stream items and replies to REQ requests using a REP socket.

)"">;

    gr::PortIn<T> in;
    std::string   endpoint        = "tcp://*:5555";
    int           timeout         = 100;
    bool          bind            = true;
    bool          pass_tags       = false;
    PmtWireFormat pmt_wire_format = PmtWireFormat::GR4_YAML_V1;
    int           linger          = 1000;
    int           hwm             = -1;
    // Covers ZMTP handshake metadata, identities and correlated REQ envelopes,
    // while bounding allocation before the four-byte body validation below.
    std::int64_t max_message_size = 4096;

    detail::ZmqSocketTransport _transport{zmq::socket_type::rep};
    detail::ZmqSendCounters    _send_counters;

    // A REP socket must retain its reply between work calls after accepting a request.
    // This is deliberately the only cross-call stream storage in this block.
    std::optional<zmq::message_t> _pending_reply;
    std::size_t                   _pending_reply_items = 0;

    GR_MAKE_REFLECTABLE(ZmqRepSink, in, endpoint, timeout, bind, pass_tags, pmt_wire_format, linger, hwm, max_message_size);

    // processBulk() waits on the socket for up to `timeout`. stop() ends that wait from another thread.
    [[nodiscard]] constexpr bool isBlocking() const noexcept { return true; }

    void start() {
        detail::throw_on_error(detail::ZmqSocketTransport::require_nonnegative_timeout(timeout));
        clear_run_state();
        _send_counters.reset();
        detail::throw_on_error(_transport.open(endpoint, bind, linger, hwm, true, max_message_size));
    }

    void stop() {
        _transport.close();
        clear_run_state();
    }

    void reset() { stop(); }

    [[nodiscard]] std::expected<std::string, gr::Error> last_endpoint() const { return _transport.last_endpoint(); }
    [[nodiscard]] detail::ZmqSendStatistics             send_statistics() const noexcept { return _send_counters.snapshot(); }

    static std::size_t parse_request_count(const zmq::message_t& request) {
        uint32_t n = 0;
        std::memcpy(&n, request.data(), sizeof(n));
        return static_cast<std::size_t>(n);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inData) {
        auto                             operation = _transport.acquire_operation_guard();
        const detail::InvocationDeadline deadline{timeout};
        auto                             cancelled = detail::cancellation_check(*this);
        std::size_t                      messages  = 0;

        auto fail = [&](const gr::Error& error) {
            std::ignore = inData.consume(0);
            this->emitErrorMessage("processBulk", error);
            return gr::work::Status::ERROR;
        };

        auto make_reply = [](const detail::EncodedBytes& header, const void* payload, std::size_t payload_size) -> std::expected<zmq::message_t, gr::Error> {
            if (!header) {
                return std::unexpected(header.error());
            }
            zmq::message_t reply(header->size() + payload_size);
            if (!header->empty()) {
                std::memcpy(reply.data(), header->data(), header->size());
            }
            if (payload_size != 0) {
                std::memcpy(static_cast<std::uint8_t*>(reply.data()) + header->size(), payload, payload_size);
            }
            return reply;
        };

        auto tag_header = [&](std::size_t count) -> detail::EncodedBytes {
            if (!pass_tags) {
                return {};
            }
            const auto all_tags = detail::collect_tag_records(inData);
            return detail::serialize_tag_header(0, detail::select_pending_tags(all_tags, count), pmt_wire_format);
        };

        auto make_empty_reply = [&]() -> std::expected<zmq::message_t, gr::Error> {
            const auto header = pass_tags ? detail::serialize_tag_header(0, {}, pmt_wire_format) : detail::EncodedBytes{};
            if constexpr (std::is_same_v<T, gr::pmt::Value>) {
                const auto serialized = detail::serialize_pmt(gr::pmt::Value{}, pmt_wire_format);
                if (!serialized) {
                    return std::unexpected(serialized.error());
                }
                return make_reply(header, serialized->data(), serialized->size());
            } else {
                return make_reply(header, nullptr, 0);
            }
        };

        auto stage_reply = [&](std::size_t requested, bool malformed) -> std::expected<void, gr::Error> {
            std::expected<zmq::message_t, gr::Error> reply;
            std::size_t                              items = 0;
            if (malformed || requested == 0 || inData.empty()) {
                reply = make_empty_reply();
            } else if constexpr (is_arithmetic_or_complex_v<T>) {
                items = std::min(requested, inData.size());
                reply = make_reply(tag_header(items), inData.data(), items * sizeof(T));
            } else if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
                const auto& vec = inData.front();
                items           = 1;
                reply           = make_reply(tag_header(items), vec.data(), vec.size() * sizeof(typename T::value_type));
            } else {
                const auto serialized = detail::serialize_pmt(inData.front(), pmt_wire_format);
                if (!serialized) {
                    return std::unexpected(serialized.error());
                }
                items = 1;
                reply = make_reply(tag_header(items), serialized->data(), serialized->size());
            }
            if (!reply) {
                return std::unexpected(reply.error());
            }
            _pending_reply       = std::move(*reply);
            _pending_reply_items = items;
            return {};
        };

        while (messages < detail::kMaxMessagesPerWork) {
            if (_pending_reply.has_value()) {
                const auto writable = _transport.wait_writable(deadline, true, cancelled);
                if (!writable) {
                    return fail(writable.error());
                }
                if (!*writable) {
                    if (!cancelled()) {
                        _send_counters.record_refused();
                    }
                    break;
                }
                auto socket = _transport.socket();
                if (!socket) {
                    return fail(socket.error());
                }
                if (!socket->get().send(*_pending_reply, zmq::send_flags::dontwait)) {
                    _send_counters.record_refused();
                    break;
                }

                const std::size_t committed = _pending_reply_items;
                _send_counters.record_accepted(committed);
                _pending_reply.reset();
                _pending_reply_items = 0;
                ++messages;
                std::ignore = inData.consume(committed);
                // Return after every completed REP transaction. In particular, an empty reply
                // cannot drive an unlimited request/reply loop in one scheduler invocation.
                return gr::work::Status::OK;
            }

            const auto readable = _transport.wait_readable(deadline, messages == 0, cancelled);
            if (!readable) {
                return fail(readable.error());
            }
            if (!*readable) {
                break;
            }
            auto socket = _transport.socket();
            if (!socket) {
                return fail(socket.error());
            }
            auto received = detail::receive_all_parts(socket->get(), 1);
            if (!received) {
                return fail(received.error());
            }
            auto& request = *received;
            if (!request) {
                break;
            }

            const bool        malformed = request->has_extra_parts || request->parts.size() != 1 || request->parts.front().size() != sizeof(uint32_t);
            const std::size_t requested = malformed ? 0 : parse_request_count(request->parts.front());
            if (auto staged = stage_reply(requested, malformed); !staged) {
                return fail(staged.error());
            }
        }

        std::ignore = inData.consume(0);
        return gr::work::Status::OK;
    }

private:
    void clear_run_state() {
        _pending_reply.reset();
        _pending_reply_items = 0;
    }
};

} // namespace gr::blocks::zeromq

GR_REGISTER_BLOCK("gr::blocks::zeromq::ZmqRepSink", gr::blocks::zeromq::ZmqRepSink, ([T]), [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::vector<float>, std::vector<std::complex<float>>, gr::pmt::Value ])
