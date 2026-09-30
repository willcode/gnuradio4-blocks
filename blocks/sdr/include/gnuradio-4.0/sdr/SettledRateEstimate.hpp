#ifndef GNURADIO_SDR_SETTLED_RATE_ESTIMATE_HPP
#define GNURADIO_SDR_SETTLED_RATE_ESTIMATE_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

#include <gnuradio-4.0/algorithm/SampleRateEstimator.hpp>

namespace gr::blocks::sdr {

/**
 * @brief A sample-rate estimate from timed chunk arrivals that answers only once it has settled.
 *
 * `algorithm::SampleRateEstimator` low-passes the period per sample measured between two chunk arrivals. Its reset
 * seeds each direct-form filter section with the configured period itself. A section in steady state holds that
 * period divided by A(1), the sum of its feedback coefficients. A(1) falls as the square of the corner over the update
 * rate. The seeded filter therefore starts its output near zero and climbs along its step response. At 0.1 Hz on
 * 250 kS/s in 64 KiB reads, the first second reads 6.8 times the configured rate. This estimate feeds the reset
 * filter the configured period until the output holds it. Every section is then in its steady state.
 *
 * An estimate is settled once it has stayed within `kMaxOffsetPpm` of the configured rate for `settleSeconds()` of
 * observation time. That time is five time constants of the filter corner, and at least one second. `settledRate()`
 * and `settledPpm()` are empty before that and while the estimate is outside the bound. A reset or a new corner
 * starts the wait again. Times are seconds on any clock that does not step.
 */
struct SettledRateEstimate {
    static constexpr double      kMaxOffsetPpm        = 1000.0; // a crystal's offset stays below about 100 ppm
    static constexpr double      kSettleTimeConstants = 5.0;
    static constexpr double      kMinSettleSeconds    = 1.0;
    static constexpr double      kPrimingTolerance    = 1e-12; // relative, on the filter output
    static constexpr std::size_t kMaxPrimingSteps     = 1'000'000UZ;

    algorithm::SampleRateEstimator estimator;
    bool                           _withinBound     = false;
    double                         _withinSince     = 0.0; // observation time the estimate entered the bound
    double                         _lastObservation = 0.0;

    void reset(double nominalRate, double expectedUpdateRateHz, float cutoffHz) {
        estimator.filter_cutoff_hz = cutoffHz;
        estimator.reset(nominalRate, expectedUpdateRateHz);
        primeFilter();
        _withinBound = false;
    }

    /// A new corner keeps the current estimate and starts the settle wait again.
    void setCutoff(float cutoffHz) {
        estimator.filter_cutoff_hz = cutoffHz;
        estimator.rebuildFilter();
        primeFilter();
        _withinBound = false;
    }

    /// Drops the previous arrival time, and the next chunk measures no interval. The filter and the wait are kept.
    void resetPhase() { estimator.resetPhase(); }

    void update(double tObsSeconds, std::size_t nSamples) {
        estimator.update(tObsSeconds, nSamples);
        _lastObservation  = tObsSeconds;
        const bool within = std::abs(offsetPpm()) <= kMaxOffsetPpm;
        if (within && !_withinBound) {
            _withinSince = tObsSeconds;
        }
        _withinBound = within;
    }

    [[nodiscard]] double nominalRate() const noexcept { return estimator._nominalRate; }

    /// The offset of the current estimate from the configured rate, settled or not.
    [[nodiscard]] double offsetPpm() const noexcept {
        const double nominal = estimator._nominalRate;
        return nominal > 0.0 ? (estimator.estimatedRate() - nominal) / nominal * 1e6 : 0.0;
    }

    [[nodiscard]] double settleSeconds() const noexcept {
        const double cutoff = static_cast<double>(estimator.filter_cutoff_hz);
        return cutoff > 0.0 ? std::max(kMinSettleSeconds, kSettleTimeConstants / (2.0 * std::numbers::pi * cutoff)) : kMinSettleSeconds;
    }

    [[nodiscard]] bool settled() const noexcept { return estimator._initialised && estimator._nominalRate > 0.0 && _withinBound && _lastObservation - _withinSince >= settleSeconds(); }

    [[nodiscard]] std::optional<double> settledRate() const noexcept { return settled() ? std::optional<double>(estimator.estimatedRate()) : std::nullopt; }

    [[nodiscard]] std::optional<float> settledPpm() const noexcept { return settled() ? std::optional<float>(estimator.estimatedPpm()) : std::nullopt; }

    /// Feeds the filter its current period until two outputs in a row hold it.
    void primeFilter() {
        const double period = estimator._periodEst;
        if (period <= 0.0) {
            return;
        }
        const double tolerance    = kPrimingTolerance * period;
        bool         previousHeld = false;
        for (std::size_t step = 0UZ; step < kMaxPrimingSteps; ++step) {
            const bool held = std::abs(estimator._lpFilter.processOne(period) - period) <= tolerance;
            if (held && previousHeld) {
                return;
            }
            previousHeld = held;
        }
    }
};

} // namespace gr::blocks::sdr

#endif // GNURADIO_SDR_SETTLED_RATE_ESTIMATE_HPP
