#ifndef GNURADIO_ANALOG_DETAIL_AVERAGER_HPP
#define GNURADIO_ANALOG_DETAIL_AVERAGER_HPP

#include <cmath>

namespace gr::blocks::analog::detail {

/// @brief The averager coefficient: @p alpha when @p averagingTimeSeconds is 0 or less, else the pole of that time
/// constant at @p sampleRate. A time constant of more than about 1e16 samples gives a pole that rounds to 0, and an
/// average with it never moves.
[[nodiscard]] inline double averagerCoefficient(double alpha, double averagingTimeSeconds, double sampleRate) {
    if (averagingTimeSeconds <= 0.0) {
        return alpha;
    }
    return 1.0 - std::exp(-1.0 / (sampleRate * averagingTimeSeconds));
}

} // namespace gr::blocks::analog::detail

#endif // GNURADIO_ANALOG_DETAIL_AVERAGER_HPP
