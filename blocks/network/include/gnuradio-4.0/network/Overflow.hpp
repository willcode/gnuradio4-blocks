#ifndef GNURADIO_NETWORK_OVERFLOW_HPP
#define GNURADIO_NETWORK_OVERFLOW_HPP

#include <cstdint>

namespace gr::blocks::network {

/// @brief The `overflow` setting of the module's in-process queues, which decides what a full queue does: drop_oldest
/// sheds the stalest entry and counts it, backpressure takes no more until there is room.
enum class Overflow : std::uint8_t { drop_oldest, backpressure };

} // namespace gr::blocks::network

#endif // GNURADIO_NETWORK_OVERFLOW_HPP
