#ifndef GNURADIO_TIMING_RATIONAL_RATE_HPP
#define GNURADIO_TIMING_RATIONAL_RATE_HPP

#include <cmath>
#include <cstdint>
#include <format>
#include <string_view>
#include <utility>

#include <gnuradio-4.0/Block.hpp>

#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

namespace gr::blocks::timing::detail {

/// The rational grid a stated rate is snapped to: one microhertz. It is a whole number of hertz for every rate a
/// device states, and it expresses the trims a clock is specified in — 44100 Hz trimmed by one part per million is
/// 44100044100/1000000 exactly, which no `double` rate holds.
inline constexpr std::uint64_t kRateDenominator = 1'000'000ULL;

/// The largest rate the grid can carry: `rate_num` must stay inside the clock's exact domain.
inline constexpr double kMaxRateHz = static_cast<double>(gr::timing::SampleClock::kMaxRateNum / kRateDenominator);

/**
 * @brief The exact rational a stated `double` rate becomes, or a refusal naming the setting.
 *
 * `SampleClock` maps sample index to nanoseconds in integers and takes an exact rational rate; a block's settings are
 * `double`, so the conversion happens once, here, rather than per sample. The grid is a microhertz, so the residual is
 * at most half of one — 1.1e-14 relative at 44.1 kHz, four orders below the part-per-billion a disciplined oscillator
 * is specified to. A rate too small to land on the grid, or too large for the clock's domain, is refused naming the
 * setting rather than silently rounded to zero or wrapped.
 */
[[nodiscard]] inline std::pair<std::uint64_t, std::uint64_t> rationalRate(double rateHz, std::string_view block, std::string_view setting) {
    if (!std::isfinite(rateHz) || rateHz <= 0.) {
        throw gr::exception(std::format("{}: '{}' must be a positive finite rate, got {}", block, setting, rateHz));
    }
    if (rateHz < 1. / static_cast<double>(kRateDenominator) || rateHz > kMaxRateHz) {
        throw gr::exception(std::format("{}: '{}' is {} Hz, outside the exact rational domain [{:g}, {:g}] Hz the microhertz grid and gr::timing::SampleClock allow", block, setting, rateHz, 1. / static_cast<double>(kRateDenominator), kMaxRateHz));
    }
    return {static_cast<std::uint64_t>(std::llround(rateHz * static_cast<double>(kRateDenominator))), kRateDenominator};
}

} // namespace gr::blocks::timing::detail

#endif // GNURADIO_TIMING_RATIONAL_RATE_HPP
