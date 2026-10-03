#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/LifeCycle.hpp>
#include <zmq.hpp>

namespace gr::blocks::zeromq::detail {

inline constexpr std::size_t kMaxMessagesPerWork = 8;

struct ZmqReceiveStatistics {
    std::uint64_t messages         = 0;
    std::uint64_t items            = 0;
    std::uint64_t refused_messages = 0;
};

class ZmqReceiveCounters {
public:
    void reset() noexcept {
        _messages.store(0, std::memory_order_relaxed);
        _items.store(0, std::memory_order_relaxed);
        _refused_messages.store(0, std::memory_order_relaxed);
    }

    void record_accepted(std::size_t items) noexcept {
        _messages.fetch_add(1, std::memory_order_relaxed);
        _items.fetch_add(static_cast<std::uint64_t>(items), std::memory_order_relaxed);
    }

    void record_refused() noexcept {
        _messages.fetch_add(1, std::memory_order_relaxed);
        _refused_messages.fetch_add(1, std::memory_order_relaxed);
    }

    [[nodiscard]] ZmqReceiveStatistics snapshot() const noexcept {
        return {
            .messages         = _messages.load(std::memory_order_relaxed),
            .items            = _items.load(std::memory_order_relaxed),
            .refused_messages = _refused_messages.load(std::memory_order_relaxed),
        };
    }

private:
    std::atomic<std::uint64_t> _messages{0};
    std::atomic<std::uint64_t> _items{0};
    std::atomic<std::uint64_t> _refused_messages{0};
};

struct ZmqSendStatistics {
    std::uint64_t messages         = 0;
    std::uint64_t items            = 0;
    std::uint64_t refused_messages = 0;
    std::uint64_t dropped_messages = 0;
    std::uint64_t dropped_items    = 0;
};

class ZmqSendCounters {
public:
    void reset() noexcept {
        _messages.store(0, std::memory_order_relaxed);
        _items.store(0, std::memory_order_relaxed);
        _refused_messages.store(0, std::memory_order_relaxed);
        _dropped_messages.store(0, std::memory_order_relaxed);
        _dropped_items.store(0, std::memory_order_relaxed);
    }

    void record_accepted(std::size_t items) noexcept {
        _messages.fetch_add(1, std::memory_order_relaxed);
        _items.fetch_add(static_cast<std::uint64_t>(items), std::memory_order_relaxed);
    }

    void record_refused() noexcept { _refused_messages.fetch_add(1, std::memory_order_relaxed); }

    void record_dropped(std::size_t messages, std::size_t items) noexcept {
        _dropped_messages.fetch_add(static_cast<std::uint64_t>(messages), std::memory_order_relaxed);
        _dropped_items.fetch_add(static_cast<std::uint64_t>(items), std::memory_order_relaxed);
    }

    [[nodiscard]] ZmqSendStatistics snapshot() const noexcept {
        return {
            .messages         = _messages.load(std::memory_order_relaxed),
            .items            = _items.load(std::memory_order_relaxed),
            .refused_messages = _refused_messages.load(std::memory_order_relaxed),
            .dropped_messages = _dropped_messages.load(std::memory_order_relaxed),
            .dropped_items    = _dropped_items.load(std::memory_order_relaxed),
        };
    }

private:
    std::atomic<std::uint64_t> _messages{0};
    std::atomic<std::uint64_t> _items{0};
    std::atomic<std::uint64_t> _refused_messages{0};
    std::atomic<std::uint64_t> _dropped_messages{0};
    std::atomic<std::uint64_t> _dropped_items{0};
};

class InvocationDeadline {
public:
    explicit InvocationDeadline(int timeout_ms) : _end(std::chrono::steady_clock::now() + std::chrono::milliseconds{timeout_ms}) {
        if (timeout_ms < 0) {
            throw gr::exception("ZMQ timeout must be non-negative");
        }
    }

    [[nodiscard]] std::chrono::steady_clock::time_point end() const noexcept { return _end; }

private:
    std::chrono::steady_clock::time_point _end;
};

class ZmqSocketTransport {
public:
    using OperationGuard = std::unique_lock<std::mutex>;

    explicit ZmqSocketTransport(zmq::socket_type type) : _type(type) {}

    ZmqSocketTransport(const ZmqSocketTransport&)            = delete;
    ZmqSocketTransport& operator=(const ZmqSocketTransport&) = delete;
    ZmqSocketTransport(ZmqSocketTransport&&)                 = delete;
    ZmqSocketTransport& operator=(ZmqSocketTransport&&)      = delete;

    ~ZmqSocketTransport() { close(); }

    [[nodiscard]] std::expected<void, gr::Error> open(const std::string& endpoint, bool bind, int linger, int hwm, bool sink, std::optional<std::int64_t> max_message_size = std::nullopt, std::optional<bool> pub_drop_on_hwm = std::nullopt) {
        std::lock_guard lock(_operation_mutex);
        _cancel_requested.store(false, std::memory_order_release);
        return open_unlocked(endpoint, bind, linger, hwm, sink, max_message_size, pub_drop_on_hwm);
    }

    [[nodiscard]] OperationGuard acquire_operation_guard() { return OperationGuard{_operation_mutex}; }

    void               request_cancel() noexcept { _cancel_requested.store(true, std::memory_order_release); }
    [[nodiscard]] bool cancel_requested() const noexcept { return _cancel_requested.load(std::memory_order_acquire); }

    // The wait functions return whether the socket became ready, or the error of a closed socket.
    template<typename Cancelled>
    [[nodiscard]] std::expected<bool, gr::Error> wait_readable(const InvocationDeadline& deadline, bool allow_block, Cancelled&& cancelled) {
        return poll_until_ready(ZMQ_POLLIN, deadline, allow_block, std::forward<Cancelled>(cancelled));
    }

    template<typename Cancelled>
    [[nodiscard]] std::expected<bool, gr::Error> wait_writable(const InvocationDeadline& deadline, bool allow_block, Cancelled&& cancelled) {
        return poll_until_ready(ZMQ_POLLOUT, deadline, allow_block, std::forward<Cancelled>(cancelled));
    }

    // Retained for focused transport tests and simple callers.
    [[nodiscard]] std::expected<bool, gr::Error> wait_readable(int timeout_ms) {
        return require_nonnegative_timeout(timeout_ms).and_then([&] { return wait_readable(InvocationDeadline{timeout_ms}, true, [] { return false; }); });
    }

    [[nodiscard]] std::expected<bool, gr::Error> wait_writable(int timeout_ms) {
        return require_nonnegative_timeout(timeout_ms).and_then([&] { return wait_writable(InvocationDeadline{timeout_ms}, true, [] { return false; }); });
    }

    [[nodiscard]] static std::expected<void, gr::Error> require_nonnegative_timeout(int timeout_ms) {
        if (timeout_ms < 0) {
            return std::unexpected(gr::Error("ZMQ timeout must be non-negative"));
        }
        return {};
    }

    [[nodiscard]] static std::expected<void, gr::Error> require_positive_max_message_size(std::int64_t max_message_size) {
        if (max_message_size <= 0) {
            return std::unexpected(gr::Error("ZMQ maximum message size must be positive"));
        }
        return {};
    }

    [[nodiscard]] std::expected<std::string, gr::Error> last_endpoint() const {
        std::lock_guard lock(_metadata_mutex);
        if (_last_endpoint.empty()) {
            return std::unexpected(gr::Error("ZMQ socket is not open"));
        }
        return _last_endpoint;
    }

    // socket() is for use while the caller owns the operation guard (or during start()).
    [[nodiscard]] std::expected<std::reference_wrapper<zmq::socket_t>, gr::Error>       socket() { return socket_unlocked(); }
    [[nodiscard]] std::expected<std::reference_wrapper<const zmq::socket_t>, gr::Error> socket() const { return socket_unlocked(); }

    static bool is_multiple_of(std::size_t byte_count, std::size_t item_size) { return item_size != 0 && byte_count % item_size == 0; }

    void close() noexcept {
        request_cancel();
        std::lock_guard lock(_operation_mutex);
        close_unlocked();
    }

private:
    // Allows deterministic inproc peers in the focused transport tests.
    friend struct ZmqTransportTestAccess;

    std::expected<void, gr::Error> open_unlocked(const std::string& endpoint, bool bind, int linger, int hwm, bool sink, std::optional<std::int64_t> max_message_size, std::optional<bool> pub_drop_on_hwm) {
        if (max_message_size.has_value()) {
            if (auto valid = require_positive_max_message_size(*max_message_size); !valid) {
                return valid;
            }
        }
        close_unlocked();
        // PUB inherits XPUB_NODROP support without exposing subscription notifications.
        _socket.emplace(_context, _type);
        _socket->set(zmq::sockopt::linger, linger);
        if (_type == zmq::socket_type::req) {
            // Do not queue a request to a not-yet-connected peer. Otherwise short reply
            // timeouts repeatedly retry during peer startup. Relaxed/correlated mode permits
            // that retry without recreating a bound socket and rejects stale replies.
            _socket->set(zmq::sockopt::immediate, 1);
            _socket->set(zmq::sockopt::req_relaxed, 1);
            _socket->set(zmq::sockopt::req_correlate, 1);
        }
        if (hwm >= 0) {
            if (sink) {
                _socket->set(zmq::sockopt::sndhwm, hwm);
            } else {
                _socket->set(zmq::sockopt::rcvhwm, hwm);
            }
        }
        if (max_message_size.has_value()) {
            _socket->set(zmq::sockopt::maxmsgsize, *max_message_size);
        }
        if (pub_drop_on_hwm.has_value() && !*pub_drop_on_hwm) {
            _socket->set(zmq::sockopt::xpub_nodrop, 1);
        }

        if (bind) {
            _socket->bind(endpoint);
            const auto resolved_endpoint = _socket->get(zmq::sockopt::last_endpoint);
            _bound_endpoint              = resolved_endpoint;
            std::lock_guard lock(_metadata_mutex);
            _last_endpoint = resolved_endpoint;
        } else {
            _socket->connect(endpoint);
            std::lock_guard lock(_metadata_mutex);
            _last_endpoint = _socket->get(zmq::sockopt::last_endpoint);
        }
        return {};
    }

    void close_unlocked() noexcept {
        if (!_socket.has_value()) {
            std::lock_guard lock(_metadata_mutex);
            _last_endpoint.clear();
            return;
        }
        if (_bound_endpoint.has_value()) {
            try {
                _socket->unbind(*_bound_endpoint);
            } catch (...) {
            }
            _bound_endpoint.reset();
        }
        try {
            _socket->close();
        } catch (...) {
        }
        _socket.reset();
        std::lock_guard lock(_metadata_mutex);
        _last_endpoint.clear();
    }

    [[nodiscard]] std::expected<std::reference_wrapper<zmq::socket_t>, gr::Error> socket_unlocked() {
        if (!_socket.has_value()) {
            return std::unexpected(gr::Error("ZMQ socket is not open"));
        }
        return std::ref(*_socket);
    }

    [[nodiscard]] std::expected<std::reference_wrapper<const zmq::socket_t>, gr::Error> socket_unlocked() const {
        if (!_socket.has_value()) {
            return std::unexpected(gr::Error("ZMQ socket is not open"));
        }
        return std::cref(*_socket);
    }

    template<typename Cancelled>
    [[nodiscard]] std::expected<bool, gr::Error> poll_until_ready(short event, const InvocationDeadline& deadline, bool allow_block, Cancelled&& cancelled) {
        if (cancel_requested() || cancelled()) {
            return false;
        }

        auto socket = socket_unlocked();
        if (!socket) {
            return std::unexpected(socket.error());
        }
        zmq::pollitem_t items[] = {{static_cast<void*>(socket->get()), 0, event, 0}};
        // Every wait performs one readiness check, including timeout=0 calls.
        zmq::poll(items, 1, std::chrono::milliseconds{0});
        if ((items[0].revents & event) != 0) {
            return true;
        }
        if (!allow_block) {
            return false;
        }

        constexpr auto max_poll_slice = std::chrono::milliseconds{10};
        while (true) {
            if (cancel_requested() || cancelled()) {
                return false;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline.end()) {
                return false;
            }
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline.end() - now);
            if (remaining.count() == 0) {
                remaining = std::chrono::milliseconds{1};
            }
            items[0].revents = 0;
            zmq::poll(items, 1, std::min(remaining, max_poll_slice));
            if ((items[0].revents & event) != 0) {
                return true;
            }
        }
    }

    zmq::context_t               _context{1};
    zmq::socket_type             _type;
    std::optional<zmq::socket_t> _socket;
    std::optional<std::string>   _bound_endpoint;
    mutable std::mutex           _operation_mutex;
    mutable std::mutex           _metadata_mutex;
    std::string                  _last_endpoint;
    std::atomic<bool>            _cancel_requested{false};
};

// Throws the error of a start-up step as gr::exception.
inline void throw_on_error(const std::expected<void, gr::Error>& result) {
    if (!result) {
        throw gr::exception(result.error().message, result.error().sourceLocation);
    }
}

template<typename Block>
[[nodiscard]] auto cancellation_check(Block& block) {
    return [&block] { return block._transport.cancel_requested() || gr::lifecycle::isShuttingDown(block.state()); };
}

template<typename T>
inline void copy_items_from_bytes(const std::uint8_t* source, std::size_t count, T* destination) {
    static_assert(std::is_trivially_copyable_v<T>);
    for (std::size_t i = 0; i < count; ++i) {
        std::memcpy(destination + i, source + i * sizeof(T), sizeof(T));
    }
}

struct ReceivedParts {
    std::vector<zmq::message_t> parts;
    bool                        has_extra_parts = false;
};

// Returns the parts of one message, no value when none is queued, or the error of a zero part limit.
inline std::expected<std::optional<ReceivedParts>, gr::Error> receive_all_parts(zmq::socket_t& socket, std::size_t max_retained_parts) {
    if (max_retained_parts == 0) {
        return std::unexpected(gr::Error("At least one multipart frame must be retained"));
    }

    ReceivedParts received;
    received.parts.reserve(max_retained_parts);
    while (true) {
        zmq::message_t part;
        if (!bool(socket.recv(part, zmq::recv_flags::dontwait))) {
            return std::optional<ReceivedParts>{};
        }
        const bool more = socket.get(zmq::sockopt::rcvmore);
        if (received.parts.size() < max_retained_parts) {
            received.parts.push_back(std::move(part));
        } else {
            received.has_extra_parts = true;
        }
        if (!more) {
            return std::optional<ReceivedParts>{std::move(received)};
        }
    }
}

} // namespace gr::blocks::zeromq::detail
