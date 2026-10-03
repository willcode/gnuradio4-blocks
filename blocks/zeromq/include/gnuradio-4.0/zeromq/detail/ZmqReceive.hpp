#pragma once
#include "../trait_helpers.hpp"
#include "ZmqCommon.hpp"
#include "ZmqTagHeaders.hpp"

#include <expected>

namespace gr::blocks::zeromq::detail {
struct DecodedFrame {
    std::size_t                     consumed_bytes = 0;
    std::vector<ZmqTagHeaderRecord> tags;
    gr::pmt::Value                  pmt;
};

// This boundary covers only header/payload validation and decoding. In
// particular, neither socket I/O nor output/tag publication belongs here.
template<typename T>
std::expected<DecodedFrame, DecodeError> decode_message(const zmq::message_t& msg, bool pass_tags, PmtWireFormat format) {
    const auto*  bytes = static_cast<const std::uint8_t*>(msg.data());
    DecodedFrame decoded;
    if (pass_tags) {
        auto header = parse_tag_header(bytes, msg.size(), format);
        if (!header) {
            return std::unexpected(header.error());
        }
        decoded.consumed_bytes = header->consumed;
        decoded.tags           = std::move(header->tags);
        for (auto& tag : decoded.tags) {
            if (tag.offset >= header->offset) {
                tag.offset -= header->offset;
            }
        }
    }
    const auto size = msg.size() - decoded.consumed_bytes;
    if constexpr (std::is_same_v<T, gr::pmt::Value>) {
        auto pmt = deserialize_pmt(bytes + decoded.consumed_bytes, size, format);
        if (!pmt) {
            return std::unexpected(pmt.error());
        }
        decoded.pmt = std::move(*pmt);
    } else if constexpr (is_vector_of_arithmetic_or_complex_v<T>) {
        if (!ZmqSocketTransport::is_multiple_of(size, sizeof(typename T::value_type))) {
            return std::unexpected(DecodeError::item_size_mismatch);
        }
    } else if (!ZmqSocketTransport::is_multiple_of(size, sizeof(T))) {
        return std::unexpected(DecodeError::item_size_mismatch);
    }
    return decoded;
}

// Decodes one message and counts it as refused when it does not decode.
template<typename T>
std::expected<DecodedFrame, DecodeError> decode_frame(const zmq::message_t& msg, bool pass_tags, PmtWireFormat format, ZmqReceiveCounters& counters) {
    auto decoded = decode_message<T>(msg, pass_tags, format);
    if (!decoded) {
        counters.record_refused();
    }
    return decoded;
}
} // namespace gr::blocks::zeromq::detail
