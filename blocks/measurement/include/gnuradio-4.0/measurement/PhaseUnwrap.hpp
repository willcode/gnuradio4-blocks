#ifndef GNURADIO_MEASUREMENT_PHASE_UNWRAP_HPP
#define GNURADIO_MEASUREMENT_PHASE_UNWRAP_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/measurement/PhaseUnwrap.hpp>

namespace gr::blocks::measurement {

GR_REGISTER_BLOCK(gr::blocks::measurement::PhaseUnwrap)

/**
 * @brief Unwrapped instantaneous phase as an exact `int64` cycle count beside a separate `float` residual.
 *
 * A 1:1 block that wraps `gr::measurement::CycleUnwrapper`. `cycles` counts whole turns since the origin exactly.
 * `phase` is the residual on `[-pi, pi)` at a `float`'s own spacing. The residual does not grow, because the count
 * carries the turns. The block does not add the two. Forming `2*pi*cycles + phase` needs a floating type. At the
 * block's output that sum would bring back the resolution loss of a `float` or `double` accumulator, and the lost
 * resolution could not be recovered after the output.
 *
 * `max_step_fraction` is an observability hook and not a detector. The unwrap is correct only when the true phase
 * advances by less than `pi` per sample (`|f| < fs/2`). A step whose magnitude exceeds `max_step_fraction * pi` is
 * counted as a sign that the bound was approached. The setting is live. It is evaluated against the cycle and phase
 * the kernel has produced, and changing it does not disturb the count.
 *
 * With `reset_on_discontinuity` set, a `n_dropped_samples` tag resets the cycle count and re-takes the origin. The
 * turns during a gap were not seen. A continued count would look exact and be wrong.
 */
struct PhaseUnwrap : Block<PhaseUnwrap> {
    using Description = Doc<R""(
@brief Unwrapped phase as an exact int64 cycle count beside a float residual on [-pi, pi).

A 1:1 block. The ports carry `cycles` and `phase` separately and never their sum. A consumer that wants one number
forms the sum in a type of its choice. `nSuspectSteps()` counts the steps that exceed `max_step_fraction * pi`, near
Nyquist. The count is an observability hook and not a detector. `reset_on_discontinuity` resets the count at a
`n_dropped_samples` tag.
)"">;

    PortIn<std::complex<float>> in;
    PortOut<std::int64_t>       cycles;
    PortOut<float>              phase;

    Annotated<std::string, "origin", Visible, Doc<"'first_sample' (cycles=0 at the first sample) or 'zero' (also subtracts its phase)">> origin                 = std::string("first_sample");
    Annotated<double, "max_step_fraction", Visible, Doc<"fraction of pi a suspect step exceeds, in (0, 1]">>                             max_step_fraction      = 0.9;
    Annotated<bool, "reset_on_discontinuity", Visible, Doc<"reset the cycle count at a n_dropped_samples tag">>                          reset_on_discontinuity = true;

    GR_MAKE_REFLECTABLE(PhaseUnwrap, in, cycles, phase, origin, max_step_fraction, reset_on_discontinuity);

    static constexpr double kPi    = std::numbers::pi_v<double>;
    static constexpr double kTwoPi = 2. * std::numbers::pi_v<double>;

    gr::measurement::CycleUnwrapper _unwrapper{};
    gr::measurement::UnwrapOrigin   _origin{gr::measurement::UnwrapOrigin::first_sample};
    bool                            _configured{false};

    bool          _havePrev{false};
    double        _prevCycles{0.};
    double        _prevPhase{0.};
    std::uint64_t _streamAt{0ULL};

    std::atomic<std::uint64_t> _nSuspect{0ULL};
    std::atomic<std::uint64_t> _nResets{0ULL};
    std::atomic<std::int64_t>  _lastSuspect{-1LL}; ///< -1 means "none yet"

    bool _taggedUnits{false}; ///< the signal_unit tags for cycles/phase have been published once

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto parsedOrigin = gr::measurement::unwrapOriginFrom(origin.value);
        if (!parsedOrigin.has_value()) {
            throw gr::exception(std::format("origin must be 'first_sample' or 'zero', got '{}'", origin.value));
        }
        if (!(max_step_fraction.value > 0.) || !(max_step_fraction.value <= 1.)) {
            throw gr::exception(std::format("max_step_fraction must lie in (0, 1], got {}", max_step_fraction.value));
        }

        if (_configured && newSettings.contains("origin") && *parsedOrigin != _origin) {
            // The origin of the count has changed, and the old count does not describe this stream. The count returns
            // to zero, and the origin is re-taken at the next sample.
            _origin    = *parsedOrigin;
            _unwrapper = gr::measurement::CycleUnwrapper(_origin, 1.0);
            _havePrev  = false;
            _nResets.fetch_add(1ULL, std::memory_order_relaxed);
        } else {
            _origin = *parsedOrigin;
        }
        _configured = true;
    }

    void start() {
        // The kernel's own maxStepFraction is fixed at 1.0. A step is bounded by pi, so the kernel's suspect threshold
        // in effect does not fire. The block tests its own live `max_step_fraction` against the cycle and phase the
        // kernel produces. The setting can then change without disturbing the cycle count.
        _unwrapper = gr::measurement::CycleUnwrapper(_origin, 1.0);
        _havePrev  = false;
        _streamAt  = 0ULL;
        _nSuspect.store(0ULL, std::memory_order_relaxed);
        _nResets.store(0ULL, std::memory_order_relaxed);
        _lastSuspect.store(-1LL, std::memory_order_relaxed);
        _taggedUnits = false;
        // A block built entirely from defaults stages nothing, and `settingsChanged` may not have run yet. From here
        // on every settings change is a change to a running block, and the reset rule applies to it.
        _configured = true;
    }

    // `cycles` and `phase` name the two output ports. The block has no methods of the same names for the per-sample
    // readings. `_unwrapper.cycles()` and `.phase()` serve a caller on the owning thread that holds the block. Every
    // other reader reads the two streams.

    /// @brief `2*pi*cycles + phase` for the current sample only, with spacing `9.54e-7 rad` at `10^9` cycles. Owning thread.
    [[nodiscard]] double unwrappedRadians() const noexcept { return _unwrapper.unwrappedRadians(); }

    [[nodiscard]] std::uint64_t                nSuspectSteps() const noexcept { return _nSuspect.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t                nResets() const noexcept { return _nResets.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t                nSaturations() const noexcept { return _unwrapper.nSaturations(); }
    [[nodiscard]] std::optional<std::uint64_t> lastSuspectIndex() const noexcept {
        const std::int64_t value = _lastSuspect.load(std::memory_order_relaxed);
        return value < 0 ? std::nullopt : std::optional<std::uint64_t>(static_cast<std::uint64_t>(value));
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& cyclesSpan, OutputSpanLike auto& phaseSpan) {
        const std::size_t n = std::min({inSpan.size(), cyclesSpan.size(), phaseSpan.size()});
        if (n == 0UZ) {
            cyclesSpan.publish(0UZ);
            phaseSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
        }

        if (!_taggedUnits) {
            cyclesSpan.publishTag(property_map{{std::pmr::string(gr::tag::SIGNAL_UNIT.shortKey()), pmt::Value(std::string(""))}}, 0UZ);
            phaseSpan.publishTag(property_map{{std::pmr::string(gr::tag::SIGNAL_UNIT.shortKey()), pmt::Value(std::string("rad"))}}, 0UZ);
            _taggedUnits = true;
        }

        const std::uint64_t base = _streamAt;

        std::vector<std::size_t> resetOffsets;
        if (reset_on_discontinuity.value) {
            for (const gr::Tag& tag : inSpan.rawTags) {
                const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
                if (at < base || at >= base + n) {
                    continue;
                }
                if (tag.map.find(gr::tag::N_DROPPED_SAMPLES.shortKey()) == tag.map.end()) {
                    continue;
                }
                resetOffsets.push_back(at - base);
            }
            std::ranges::sort(resetOffsets);
            resetOffsets.erase(std::ranges::unique(resetOffsets).begin(), resetOffsets.end());
        }

        std::span<const std::complex<float>> input(inSpan.data(), n);
        std::span<std::int64_t>              cyclesOut(cyclesSpan.data(), n);
        std::span<float>                     phaseOut(phaseSpan.data(), n);

        std::size_t at = 0UZ;
        for (const std::size_t offset : resetOffsets) {
            if (offset > at) {
                processSlice(input.subspan(at, offset - at), cyclesOut.subspan(at, offset - at), phaseOut.subspan(at, offset - at), base + at);
                at = offset;
            }
            _unwrapper.markDiscontinuity();
            _nResets.fetch_add(1ULL, std::memory_order_relaxed);
            _havePrev = false;
        }
        if (at < n) {
            processSlice(input.subspan(at), cyclesOut.subspan(at), phaseOut.subspan(at), base + at);
        }

        _streamAt += n;
        cyclesSpan.publish(n);
        phaseSpan.publish(n);
        std::ignore = inSpan.consume(n);
        return work::Status::OK;
    }

private:
    void processSlice(std::span<const std::complex<float>> input, std::span<std::int64_t> cyclesOut, std::span<float> phaseOut, std::uint64_t baseIndex) {
        _unwrapper.process(input, cyclesOut, phaseOut);

        const double threshold = max_step_fraction.value * kPi;
        for (std::size_t k = 0UZ; k < input.size(); ++k) {
            const double curCycles = static_cast<double>(cyclesOut[k]);
            const double curPhase  = static_cast<double>(phaseOut[k]);
            if (_havePrev) {
                const double step = kTwoPi * (curCycles - _prevCycles) + (curPhase - _prevPhase);
                if (std::abs(step) > threshold) {
                    _nSuspect.fetch_add(1ULL, std::memory_order_relaxed);
                    _lastSuspect.store(static_cast<std::int64_t>(baseIndex + k), std::memory_order_relaxed);
                }
            }
            _havePrev   = true;
            _prevCycles = curCycles;
            _prevPhase  = curPhase;
        }
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_PHASE_UNWRAP_HPP
