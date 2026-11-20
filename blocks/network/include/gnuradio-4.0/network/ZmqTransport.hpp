#ifndef GNURADIO_NETWORK_ZMQTRANSPORT_HPP
#define GNURADIO_NETWORK_ZMQTRANSPORT_HPP

#include <cstdint>
#include <format>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Message.hpp>

/**
 * @brief The socket vocabulary the module's ZeroMQ blocks share: the four patterns, the two names each role
 * accepts, and the one endpoint diagnostic worth stating.
 *
 * It sits in a header of its own because the alternative is a second copy of one table and one piece of
 * user-facing text — the drift these blocks exist to avoid — and because a block carrying a raw sample stream has
 * no business including the packet envelope to learn what 'sub' means.
 */
namespace gr::blocks::network {

/// @brief The `pattern` of a sink: pub fans out, never blocks and drops in mute state; push deals round-robin to one
/// consumer at a time, blocks in mute state and discards nothing.
enum class SendPattern : std::uint8_t { pub, push };

/// @brief The `pattern` of a source: sub subscribes to a prefix, pull fair-queues.
enum class ReceivePattern : std::uint8_t { sub, pull };

} // namespace gr::blocks::network

namespace gr::blocks::network::detail::zmqio {

/// @brief The socket pattern the transport opens, whichever role the block has.
enum class Pattern : std::uint8_t { Pub, Push, Sub, Pull };

[[nodiscard]] constexpr Pattern socketPattern(SendPattern pattern) noexcept { return pattern == SendPattern::pub ? Pattern::Pub : Pattern::Push; }
[[nodiscard]] constexpr Pattern socketPattern(ReceivePattern pattern) noexcept { return pattern == ReceivePattern::sub ? Pattern::Sub : Pattern::Pull; }

/// @brief The one endpoint diagnostic worth stating rather than leaving to be rediscovered.
///
/// libzmq answers an endpoint with no transport prefix with a bare `EINVAL`, and that is the common typo.
[[nodiscard]] inline std::string endpointHint(std::string_view endpoint) { return endpoint.find("://") == std::string_view::npos ? std::format(" — '{}' names no transport; a libzmq endpoint begins with a prefix such as tcp://, ipc:// or inproc://", endpoint) : std::string{}; }

} // namespace gr::blocks::network::detail::zmqio

#endif // GNURADIO_NETWORK_ZMQTRANSPORT_HPP
