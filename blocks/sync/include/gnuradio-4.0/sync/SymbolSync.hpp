#ifndef GNURADIO_SYNC_SYMBOL_SYNC_HPP
#define GNURADIO_SYNC_SYMBOL_SYNC_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/sync/ControlLoop.hpp>
#include <gnuradio-4.0/algorithm/sync/MmseInterpolator.hpp>
#include <gnuradio-4.0/algorithm/sync/TimingErrorDetector.hpp>

#include <gnuradio-4.0/sync/LoopCommon.hpp>

namespace gr::blocks::sync {

/// @brief Stream tag: the optimum sampling instant of the next symbol, in input samples from the tagged sample, in `[-1, 1]`.
inline constexpr std::string_view kTimeEstKey = "time_est";
/// @brief Stream tag: a two-element real tensor `{offset, period}` — `time_est`, plus the average period in input samples per symbol.
inline constexpr std::string_view kClockEstKey = "clock_est";

/// @brief The slicer the decision-directed detectors use: this block's own `bpsk`, `qpsk` and `pam4`, or the shared
/// `psk` with `arity`, which the constellation-carrying blocks use.
enum class SlicerConstellation : std::uint8_t { bpsk, qpsk, pam4, psk };

/// @brief How SymbolSync interpolates between input samples: a Farrow MMSE filter of 8, 6 or 12 taps, or the polyphase
/// matched-filter bank.
enum class TimingInterpolator : std::uint8_t { mmse8, mmse6, mmse12, polyphase };

/// @brief The polyphase partition this bank runs, under the name the timing machinery knows it by.
[[nodiscard]] inline std::vector<float> partitionBranchMajor(std::span<const float> prototype, std::size_t arms) { return gr::filter::fir::design::polyphasePartition(prototype, arms); }

namespace detail {

/**
 * @brief The timing loop: the shared kernel in `double`, its phase left to run.
 *
 * `double` and not `float`, which is the one place this block departs from its neighbors: at a period of 4 samples
 * and a `Bn*T` of 0.01 the integrator's dead band in `float` is 100 ppm, twenty times the steady-state period error
 * this block has to reach. The phase is not reduced, because the interpolator's position is not derived from it — it
 * is a diagnostic no port carries, and reducing it modulo a varying period would cost a `std::remainder` per symbol.
 */
using TimingLoop = gr::sync::ControlLoop<double, gr::sync::PhaseWrap::None>;

/// @brief The interpolator's tap count, 0 for the polyphase matched-filter path.
[[nodiscard]] inline int interpolatorTaps(TimingInterpolator interpolator) noexcept {
    switch (interpolator) {
    case TimingInterpolator::mmse8: return 8;
    case TimingInterpolator::mmse6: return 6;
    case TimingInterpolator::mmse12: return 12;
    case TimingInterpolator::polyphase: break;
    }
    return 0;
}

/// @brief A `time_est` or `clock_est` payload that passed validation, waiting for the position to reach it.
struct TimingTag {
    std::uint64_t offset  = 0ULL; /// absolute input offset of the tagged sample
    double        instant = 0.0;  /// input samples from `offset` to the requested symbol instant
    double        period  = 0.0;  /// input samples per symbol; 0 leaves the tracked period alone
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::sync::SymbolSync, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct SymbolSync : Block<SymbolSync<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Symbol timing recovery: estimates the transmitter's symbol clock from the signal and resamples onto it.

The receiver knows roughly how many samples there are per symbol, but not the transmitter's exact clock nor where the
symbol boundaries fall, and both drift. This block estimates the symbol instants from the signal itself and emits
`outputs_per_symbol` samples per symbol taken at those instants. The interpolator's position is advanced by the loop's
period estimate and not derived from its phase.

An AGC is a precondition, as for CostasLoop and FllBandEdge. `detector_gain` is per symbol period and the block
divides by `samples_per_symbol` itself; 0 takes the detector's own gain on a raised-cosine channel of the stated
`rolloff`, which is the right value whenever the transmitter shaped with a matching root-raised cosine. Gardner,
early-late and signal times slope have no gain at all at zero rolloff, and a graph that asks for one is refused
rather than run with a loop that cannot converge. `noise_bandwidth` is normalized to the symbol rate here, the loop
stepping once per symbol, where the carrier loops normalize theirs to the sample rate.

Gardner and early-late are blind and work before the carrier loop has locked; Mueller & Muller, the modified M&M and
zero crossing need decisions, hence a locked carrier and the `constellation` setting. `mmse8` is a clean interpolator
for `sps >= 2.5`; below that use `mmse12` or `polyphase`, which also matched-filters and so must not follow another
matched filter. Output sample `k` estimates the input at a continuous position that is not a fixed multiple of the
output index; `instant_period` and `average_period` carry the clock for a consumer that needs it.

The input port's minimum is held at the count that guarantees one more output, so at end of stream the block ends
as soon as fewer samples than that remain: the trailing samples are dropped, an interpolation without its future
samples being fabrication, and a finite graph terminates instead of waiting on input that cannot arrive.

A `time_est` or `clock_est` tag presets the position, and `clock_est` the period with it, which on a burst link is what
removes the acquisition the loop has too few symbols to do. A preset re-times a sample rather than removing one, so the
stream leaving the block carries one sample per symbol across a tag as it does anywhere else. `trigger` names which
producer's tags to honor and empty honors any; `trigger_presets` turns presetting off altogether. A payload out of
range, one carrying another producer's name, and one whose instant the position has already gone past are each ignored
and counted, and leave the block in the state it would have been in with no tag at all.

A NaN or infinite input sample makes the timing error non-finite for each symbol whose window or history holds it.
Such a symbol leaves the loop's period estimate (`average_period`) and phase unchanged, puts 0 on the `error` port and
counts in `non_finite_errors`; an output sample whose window holds the NaN is NaN. A finite timing error beyond plus
or minus 16 is clipped to that limit and counted in `clipped_errors`; a noiseless unit-power signal shaped with
rolloff 0.2 or more stays below it. A period above twice the upper clamp is reset to `samples_per_symbol` and counted
in `period_resets`. The upper clamp is `samples_per_symbol` plus `max_deviation`, or one and a half times
`samples_per_symbol` when `max_deviation` is 0.

`samples_per_symbol` need not be an integer. `outputs_per_symbol` is fixed at design time as the internal clock's
modulus. `detector_gain` is Kted in error units per symbol period; 0 selects the raised-cosine gain of the detector in
use. `noise_bandwidth` is normalized to the symbol rate, Bn*Tsym, and the loop is critically damped by default;
under-damped timing loops are not useful. `polyphase_arms` is rounded up to a power of two, and `matched_filter_taps` is
the polyphase prototype at arms * samples_per_symbol. `rolloff` also sets the M&M closed-form gain. `trigger` names the
gr:trigger_name a timing tag carries for its payload to be honored; with `trigger_presets` off, every time_est and
clock_est is counted and ignored. `clipped_errors` counts timing errors clipped to +/-16.
)"">;

    PortIn<T>                in;
    PortOut<T>               out;
    PortOut<float, Optional> error;
    PortOut<float, Optional> instant_period;
    PortOut<float, Optional> average_period;

    struct Settings {
        Annotated<double, Doc<"samples per symbol">, Limits<Over<1.0>>>                                samples_per_symbol = 4.0;
        Annotated<double, Unit<"samples">, Doc<"period deviation limit (0 = half)">, Limits<Min<0.0>>> max_deviation      = 0.0;
        Annotated<gr::Size_t, Doc<"outputs per symbol">, Limits<Min<1U>>>                              outputs_per_symbol = 1U;
        Annotated<gr::sync::TimingDetector, Doc<"timing error detector">>                              detector           = gr::sync::TimingDetector::mueller_muller;
        Annotated<SlicerConstellation, Doc<"decision slicer">>                                         constellation      = std::same_as<T, float> ? SlicerConstellation::bpsk : SlicerConstellation::qpsk;
        Annotated<gr::Size_t, Unit<"symbols">, Doc<"psk order">>                                       arity              = 4U;
        Annotated<double, Doc<"detector gain (0 = derived)">, Limits<Min<0.0>>>                        detector_gain      = 0.0;
        Annotated<double, Doc<"loop noise bandwidth">, Limits<Over<0.0>>>                              noise_bandwidth    = 0.01;
        Annotated<double, Doc<"damping factor">, Limits<Over<0.0>>>                                    damping            = 1.0;
        Annotated<TimingInterpolator, Doc<"interpolator form">>                                        interpolator       = TimingInterpolator::mmse8;
        Annotated<gr::Size_t, Doc<"polyphase arm count">>                                              polyphase_arms     = 128U;
        Annotated<std::vector<float>, Doc<"matched filter taps (empty = designed)">>                   matched_filter_taps{};
        Annotated<double, Doc<"excess bandwidth">, Limits<Min<0.0>, Max<1.0>>>                         rolloff = 0.35;
        Annotated<std::string, Doc<"timing tag trigger name (empty = any)">>                           trigger{};
        Annotated<bool, Doc<"timing tag presets">>                                                     trigger_presets = true;

        GR_MAKE_REFLECTABLE(Settings, samples_per_symbol, max_deviation, outputs_per_symbol, detector, constellation, arity, detector_gain, noise_bandwidth, damping, interpolator, polyphase_arms, matched_filter_taps, rolloff, trigger, trigger_presets);
    } settings;

    struct Status {
        Annotated<gr::Size_t, Doc<"non-finite errors">> non_finite_errors = 0U;
        Annotated<gr::Size_t, Doc<"period resets">>     period_resets     = 0U;
        Annotated<gr::Size_t, Doc<"clipped errors">>    clipped_errors    = 0U;

        GR_MAKE_REFLECTABLE(Status, non_finite_errors, period_resets, clipped_errors);
    } status;

    GR_MAKE_REFLECTABLE(SymbolSync, in, out, error, instant_period, average_period, settings, status);

    detail::TimingLoop _loop{0.01, 1.0, 1.0, 1.0, 8.0};
    // Tap count follows `interpolator` (mmse6/8/12); the 128 phase steps and the 0.25 design band are
    // fixed here and are not settings. MmseInterpolator documents what each costs.
    gr::sync::MmseInterpolatorBank _bank{8, 128, 0.25, false};
    gr::sync::TimingDetector       _kind = gr::sync::TimingDetector::mueller_muller;
    gr::sync::TimingDetectorTraits _traits{};

    std::vector<float> _branches{}; /// the polyphase bank, branch-major and reversed
    std::size_t        _arms         = 0UZ;
    std::size_t        _windowLength = 8UZ;  /// `L`: the input samples one interpolation reads
    std::uint64_t      _anchorDelay  = 3ULL; /// `L/2 - 1`: the integer part of the interpolated instant's lead over `base`
    bool               _polyphase    = false;
    bool               _qpsk         = false;
    bool               _pam4         = false;

    static constexpr std::uint8_t kDetectorStep = 1U;
    static constexpr std::uint8_t kOutputStep   = 2U;

    /// @brief Which of the interpolator's instants a timing tag names, and so which side of a step its snap sits on.
    enum class Preset : std::uint8_t { ThisInstant, TheNextInstant };

    int                       _interpolationsPerSymbol = 1;
    double                    _cycleLength             = 1.0; /// the same count as a double, for the per-symbol divide
    int                       _counter                 = 0;
    std::vector<std::uint8_t> _schedule{kDetectorStep | kOutputStep}; /// which of the three clocks each phase of the cycle carries

    std::uint64_t _base       = 0ULL; /// absolute input offset of the interpolation window
    std::uint64_t _baseLead   = 0ULL; /// the base's lead over the point the last call consumed to
    double        _mu         = 0.0;  /// fraction of an input sample, in `[0, 1)`
    bool          _positioned = false;

    double _instantPeriod       = 4.0;
    double _interpolationPeriod = 4.0;
    float  _error               = 0.f;

    std::array<T, 3UZ> _y{};        /// interpolated samples at the last three symbol instants
    std::array<T, 3UZ> _a{};        /// their decisions, when the detector needs them
    T                  _midpoint{}; /// the half-symbol sample the two-input detectors read
    T                  _slope{};    /// the derivative per symbol at the current instant

    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags{};
    std::vector<detail::TimingTag>                      _timingTags{};
    std::uint64_t                                       _tagsSeenThrough = 0ULL;
    std::uint64_t                                       _ignoredTags     = 0ULL;

    /// The events of the work call under way, which it adds to the three counters once at its end.
    struct CallEvents {
        gr::Size_t nonFinite = 0U;
        gr::Size_t resets    = 0U;
        gr::Size_t clipped   = 0U;
    };
    CallEvents _events{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const double sps = s.samples_per_symbol.value;
        if (s.max_deviation.value >= sps) {
            return gr::Refused(s.max_deviation, "must be < samples_per_symbol");
        }
        const std::string_view shape = shapeOf(s.constellation.value, s.arity.value);
        if (shape.empty()) {
            return gr::Refused(s.arity, "must be 2 or 4 when constellation is psk");
        }
        if (std::same_as<T, float> && shape == "qpsk") {
            return gr::Refused(s.constellation, "qpsk requires a complex stream");
        }
        if (!std::same_as<T, float> && shape == "pam4") {
            return gr::Refused(s.constellation, "pam4 requires a real stream");
        }
        if (detectorGainOf(s.detector.value, s.detector_gain.value, s.rolloff.value, shape) <= 0.0) {
            return gr::Refused(s.rolloff, "must be > 0 when detector_gain is 0");
        }
        if (s.interpolator.value == TimingInterpolator::polyphase && gr::sync::traitsOf(s.detector.value).needsDerivative) {
            return gr::Refused(s.interpolator, "polyphase requires a derivative-free detector");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        const double deviation = s.max_deviation > 0.0 ? s.max_deviation.value : 0.5 * s.samples_per_symbol;

        _kind   = s.detector;
        _traits = gr::sync::traitsOf(_kind);
        _qpsk   = constellationShape() == "qpsk";
        _pam4   = constellationShape() == "pam4";

        _loop.setFrequencyLimits(s.samples_per_symbol - deviation, s.samples_per_symbol + deviation);
        _loop.setDamping(s.damping);
        _loop.setNoiseBandwidth(s.noise_bandwidth);
        _loop.setDetectorGain(gr::sync::detectorGainPerSample(effectiveDetectorGain(_kind), s.samples_per_symbol));

        // these settings shape the interpolator and the clock, which start again
        const bool reshape = s.samples_per_symbol.changed() || s.outputs_per_symbol.changed() || s.detector.changed() || s.constellation.changed() || s.arity.changed() || s.interpolator.changed() || s.polyphase_arms.changed() || s.matched_filter_taps.changed() || s.rolloff.changed();
        if (reshape) {
            buildInterpolator(detail::interpolatorTaps(s.interpolator.value));
            restart();
        }

        // The input port's minimum is the loop's own production condition at the position the block currently
        // holds: the base's lead over the point the last call consumed to, plus the furthest the detector reads
        // past the base, plus the window. A port minimum is also a refusal -- the scheduler withholds a shorter
        // span -- so any slack in it is input the loop could have used, and at the end of a stream refused input
        // is gone. How many samples are left over there would then be set by where the call boundaries fell,
        // making the symbol count a property of the scheduler's chunking rather than of the recording. Every call
        // rewrites the value from the position it leaves; this one carries a settings change into the same form.
        in.min_samples = static_cast<gr::Size_t>(_baseLead + readAhead(_traits.needsLookahead, _polyphase) + _windowLength);
    }

    void reset() {
        restart();
        _pendingTags.clear();
        _timingTags.clear();
        _tagsSeenThrough = 0ULL;
        _ignoredTags     = 0ULL;
        _base            = 0ULL;
        _baseLead        = 0ULL;
        _positioned      = false;

        _events                  = {};
        status.non_finite_errors = 0U;
        status.period_resets     = 0U;
        status.clipped_errors    = 0U;
    }

    /// @brief The decision shape the constellation selects, empty for a `psk` arity this slicer cannot carry.
    ///
    /// The detectors slice a binary or quadrature decision, so `psk` is carried at arity 2 and 4 and refused above
    /// that rather than sliced wrongly.
    [[nodiscard]] std::string_view constellationShape() const noexcept { return shapeOf(settings.constellation.value, settings.arity.value); }

    /// @brief The decision shape @p constellation with @p size selects, empty when none.
    [[nodiscard]] static std::string_view shapeOf(SlicerConstellation constellation, gr::Size_t size) noexcept {
        switch (constellation) {
        case SlicerConstellation::bpsk: return "bpsk";
        case SlicerConstellation::qpsk: return "qpsk";
        case SlicerConstellation::pam4: return "pam4";
        case SlicerConstellation::psk: break;
        }
        if (size == 2U) {
            return "bpsk";
        }
        return size == 4U ? "qpsk" : "";
    }

    /// @brief The average of `E[|Re{a}|]` and `E[|Im{a}|]` for the sliced constellation at unit power.
    [[nodiscard]] double meanAxisMagnitude() const noexcept { return meanAxisOf(constellationShape()); }

    [[nodiscard]] static double meanAxisOf(std::string_view shape) noexcept {
        if (shape == "qpsk") {
            return std::numbers::sqrt2 / 2.0;
        }
        if (shape == "pam4") {
            return 0.4472135954999579; // 1/sqrt(5): the mean of {1, 3}/sqrt(5)
        }
        return 0.5; // one axis carries the whole symbol and the kernel still halves
    }

    /// @brief `Kted` per symbol: what `detector_gain` says, or the raised-cosine value for the detector in use.
    [[nodiscard]] double effectiveDetectorGain(gr::sync::TimingDetector kind) const { return detectorGainOf(kind, settings.detector_gain.value, settings.rolloff.value, constellationShape()); }

    [[nodiscard]] static double detectorGainOf(gr::sync::TimingDetector kind, double detectorGain, double excessBandwidth, std::string_view shape) {
        if (detectorGain > 0.0) {
            return detectorGain;
        }
        switch (kind) {
        case gr::sync::TimingDetector::mueller_muller: return gr::sync::muellerMullerGain(excessBandwidth);
        case gr::sync::TimingDetector::modified_mueller_muller: return gr::sync::modifiedMuellerMullerGain(excessBandwidth);
        case gr::sync::TimingDetector::zero_crossing: return gr::sync::zeroCrossingGain(excessBandwidth);
        case gr::sync::TimingDetector::gardner: return gr::sync::gardnerGain(excessBandwidth);
        case gr::sync::TimingDetector::early_late: return gr::sync::earlyLateGain(excessBandwidth);
        case gr::sync::TimingDetector::signal_times_slope_ml: return gr::sync::signalTimesSlopeGain(excessBandwidth);
        case gr::sync::TimingDetector::signum_times_slope_ml: return gr::sync::signumTimesSlopeGain(excessBandwidth, meanAxisOf(shape));
        }
        return 0.0;
    }

    [[nodiscard]] std::uint64_t             ignoredTagPayloads() const noexcept { return _ignoredTags; }
    [[nodiscard]] const detail::TimingLoop& loop() const noexcept { return _loop; }
    [[nodiscard]] int                       interpolationsPerSymbol() const noexcept { return _interpolationsPerSymbol; }
    [[nodiscard]] std::size_t               windowLength() const noexcept { return _windowLength; }
    [[nodiscard]] std::span<const float>    branches() const noexcept { return _branches; }

    /// @brief The detector and the interpolator are chosen once per call, so the loop below has neither in it.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        const work::Status result = sweepFor(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        status.non_finite_errors.value += _events.nonFinite;
        status.period_resets.value += _events.resets;
        status.clipped_errors.value += _events.clipped;
        _events = {};
        return result;
    }

private:
    [[nodiscard]] work::Status sweepFor(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        using gr::sync::TimingDetector;
        switch (_kind) {
        case TimingDetector::mueller_muller: return dispatch<TimingDetector::mueller_muller>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::modified_mueller_muller: return dispatch<TimingDetector::modified_mueller_muller>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::zero_crossing: return dispatch<TimingDetector::zero_crossing>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::gardner: return dispatch<TimingDetector::gardner>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::early_late: return dispatch<TimingDetector::early_late>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        case TimingDetector::signal_times_slope_ml: return dispatch<TimingDetector::signal_times_slope_ml>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        default: return dispatch<TimingDetector::signum_times_slope_ml>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
        }
    }

    template<gr::sync::TimingDetector D>
    [[nodiscard]] work::Status dispatch(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        if constexpr (!gr::sync::traitsOf(D).needsDerivative) {
            if (_polyphase) {
                return sweep<D, true>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
            }
        }
        return sweep<D, false>(inSpan, outSpan, errorSpan, instantSpan, averageSpan);
    }

    /// @brief The furthest the detector reads past the base for the next interpolation, in input samples.
    ///
    /// It is `_mu` plus, where the detector reads a late sample, ONE INTERPOLATION period -- early-late
    /// interpolates twice a symbol, so its late sample stands half a symbol on and not a whole one -- rounded up,
    /// with one more branch step on the polyphase path. The loop's production condition and the input port
    /// minimum are the same expression, so both are read from here.
    [[nodiscard]] std::uint64_t readAhead(bool needsLookahead, bool polyphase) const noexcept {
        const double reach = needsLookahead ? _mu + _interpolationPeriod : _mu;
        return static_cast<std::uint64_t>(reach) + (reach > 0.0 ? 1ULL : 0ULL) + (polyphase ? 1ULL : 0ULL);
    }

    /**
     * @brief The largest timing error magnitude the loop steps on, in error units.
     *
     * The modified M&M and the two slope forms clip at 1 themselves. Each other detector multiplies a difference of two
     * samples or decisions by a third sample. At unit average symbol power, the outer 4-PAM level, 3/sqrt(5), is the
     * largest constellation magnitude. A raised-cosine pulse with rolloff 0.2 or more raises a sample to at most 2.08
     * times that level, the sum of the pulse's magnitudes at half-symbol offsets. A noiseless unit-power signal thus
     * gives an error of at most 2 * 9/5 * 2.08^2, about 15.6, at any timing offset, and at most about 8.7 for BPSK and
     * QPSK. The limit of 16 lies just above the 4-PAM bound and at 1.8 times the BPSK and QPSK bound.
     */
    static constexpr float kErrorLimit = 16.f;

    /**
     * @brief Steps the loop on `_error` and returns the symbol's period in input samples: finite, positive and at most
     * twice the clamp's upper bound. The caller turns it into a sample advance by an integer conversion, undefined for
     * a NaN or a value past the integer's range.
     */
    [[nodiscard]] double nextPeriod() noexcept {
        double instant = _loop.frequency();
        if (std::isfinite(_error)) [[likely]] {
            if (std::abs(_error) > kErrorLimit) [[unlikely]] {
                _error = std::copysign(kErrorLimit, _error);
                ++_events.clipped;
            }
            instant = _loop.step(static_cast<double>(_error));
        } else {
            _error = 0.f;
            ++_events.nonFinite;
        }
        const double period = instant > 0.0 ? instant : _loop.frequency();
        if (period <= 2.0 * _loop.maxFrequency()) [[likely]] {
            return period;
        }
        ++_events.resets;
        return settings.samples_per_symbol;
    }

    /// @brief A 64-bit sample offset narrowed to an index; a template, so the cast stands where `std::size_t` is 32
    /// bits and is not a useless cast where the two are the same 64-bit type.
    template<std::integral To, std::integral From>
    [[nodiscard]] static constexpr To narrowIndex(From value) noexcept {
        return static_cast<To>(value);
    }

    template<gr::sync::TimingDetector D, bool Polyphase>
    [[nodiscard]] work::Status sweep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& errorSpan, OutputSpanLike auto& instantSpan, OutputSpanLike auto& averageSpan) {
        constexpr gr::sync::TimingDetectorTraits kTraits = gr::sync::traitsOf(D);

        const std::uint64_t windowStart = static_cast<std::uint64_t>(inSpan.streamIndex);
        if (!_positioned) {
            _base            = windowStart;
            _tagsSeenThrough = windowStart;
            _positioned      = true;
        }

        std::size_t room = outSpan.size();
        room             = detail::syncCount(room, errorSpan);
        room             = detail::syncCount(room, instantSpan);
        room             = detail::syncCount(room, averageSpan);

        const bool          wantError   = errorSpan.isConnected;
        const bool          wantInstant = instantSpan.isConnected;
        const bool          wantAverage = averageSpan.isConnected;
        const std::uint64_t inputEnd    = windowStart + inSpan.size();

        collectTags(inSpan, inputEnd);

        const T* const x    = inSpan.data();
        std::size_t    made = 0UZ;
        while (made < room) {
            std::ignore = applyTimingTags(Preset::ThisInstant);
            if (_base + readAhead(kTraits.needsLookahead, Polyphase) + _windowLength - 1UZ >= inputEnd) {
                break;
            }

            _counter                  = _counter + 1 == _interpolationsPerSymbol ? 0 : _counter + 1;
            const T* const     window = x + (_base - windowStart);
            const T            y      = interpolate<Polyphase>(window, _mu);
            const std::uint8_t due    = _schedule[static_cast<std::size_t>(_counter)];

            if (_counter == 0) {
                _y[2] = _y[1];
                _y[1] = _y[0];
                _y[0] = y;
                if constexpr (kTraits.needsDecisions) {
                    _a[2] = _a[1];
                    _a[1] = _a[0];
                    _a[0] = decide(y);
                }
                if constexpr (kTraits.needsDerivative) {
                    // The taps differentiate per input sample; a detector's Kted is per symbol, so the period scales it.
                    _slope = scale(_bank.differentiate(window, quantize(_mu, _bank.steps())), static_cast<float>(_instantPeriod));
                }
                const T late = kTraits.needsLookahead ? lookahead<Polyphase>(x, windowStart) : T{};

                _error               = errorOf<D>(late);
                _instantPeriod       = nextPeriod();
                _interpolationPeriod = _instantPeriod / _cycleLength;
            } else if ((due & kDetectorStep) != 0U) {
                _midpoint = y;
            }

            if ((due & kOutputStep) != 0U) {
                outSpan[made] = y;
                if (wantError) {
                    errorSpan[made] = _error;
                }
                if (wantInstant) {
                    instantSpan[made] = static_cast<float>(_instantPeriod);
                }
                if (wantAverage) {
                    averageSpan[made] = static_cast<float>(_loop.frequency());
                }
                releaseTags(_base + _anchorDelay, made, outSpan, errorSpan, instantSpan, averageSpan);
                ++made;
            }

            if (!applyTimingTags(Preset::TheNextInstant)) {
                // `_interpolationPeriod` is positive and bounded here. Truncation is then `floor` and is one
                // instruction; `std::floor` is a dozen.
                const double        advanced = _mu + _interpolationPeriod;
                const std::uint64_t whole    = static_cast<std::uint64_t>(advanced);
                _mu                          = advanced - static_cast<double>(whole);
                _base += whole;
            }
        }

        // Half a window of input is held back: a timing tag first seen at the very end of this window asks the
        // interpolator to sit `L/2 - 1` samples before the sample it names, and that history has to still be there.
        const std::uint64_t reach     = _windowLength / 2UZ;
        const std::uint64_t keepFrom  = inputEnd > reach ? inputEnd - reach : 0ULL;
        const std::uint64_t consumeTo = std::max(windowStart, std::min(_base, keepFrom));

        std::ignore = inSpan.consume(narrowIndex<std::size_t>(consumeTo - windowStart));

        // What the next call needs, measured from where this consume leaves the reader: the loop's own production
        // condition and nothing more, so the block is offered a span exactly when one more output is computable.
        _baseLead      = _base - consumeTo;
        in.min_samples = static_cast<gr::Size_t>(_baseLead + readAhead(kTraits.needsLookahead, Polyphase) + _windowLength);

        outSpan.publish(made);
        errorSpan.publish(wantError ? made : 0UZ);
        instantSpan.publish(wantInstant ? made : 0UZ);
        averageSpan.publish(wantAverage ? made : 0UZ);

        if (made == 0UZ && consumeTo == windowStart) {
            return room == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    void restart() {
        _interpolationsPerSymbol         = std::lcm(_traits.inputsPerSymbol, static_cast<int>(settings.outputs_per_symbol.value));
        const std::size_t detectorStride = static_cast<std::size_t>(_interpolationsPerSymbol / _traits.inputsPerSymbol);
        const std::size_t outputStride   = static_cast<std::size_t>(_interpolationsPerSymbol / static_cast<int>(settings.outputs_per_symbol.value));

        // The three clocks are a modulo of an integer counter, but a modulo by
        // a value the compiler cannot see is a hardware divide, three of them per interpolation, so the whole cycle is
        // tabulated once per settings change instead.
        _schedule.assign(static_cast<std::size_t>(_interpolationsPerSymbol), 0U);
        for (std::size_t phase = 0UZ; phase < _schedule.size(); ++phase) {
            _schedule[phase] = static_cast<std::uint8_t>((phase % detectorStride == 0UZ ? kDetectorStep : 0U) | (phase % outputStride == 0UZ ? kOutputStep : 0U));
        }
        _counter = _interpolationsPerSymbol - 1;

        _cycleLength = static_cast<double>(_interpolationsPerSymbol);
        _loop.reset(0.0, settings.samples_per_symbol);
        _instantPeriod       = settings.samples_per_symbol;
        _interpolationPeriod = _instantPeriod / _cycleLength;
        _mu                  = 0.0;
        _error               = 0.f;
        _y.fill(T{});
        _a.fill(T{});
        _midpoint = T{};
        _slope    = T{};
    }

    void buildInterpolator(int nTaps) {
        _polyphase = nTaps == 0;
        if (!_polyphase) {
            _bank = gr::sync::MmseInterpolatorBank(nTaps, 128, 0.25, _traits.needsDerivative);
            _branches.clear();
            _arms         = 0UZ;
            _windowLength = _bank.size();
            _anchorDelay  = _bank.delay();
            return;
        }
        std::size_t arms = 1UZ;
        while (arms < static_cast<std::size_t>(settings.polyphase_arms.value)) {
            arms <<= 1U;
        }

        std::vector<float> prototype = settings.matched_filter_taps;
        if (prototype.empty()) {
            // The design returns an odd tap count, so ask for one short of the bank and let the partition pad the
            // last branch: the prototype stays centered to within one interpolated sample, 1/arms of an input sample.
            // Eleven symbols of span is a conventional root-raised-cosine truncation: long enough that the
            // matched-filter residue is well inside the timing error the loop already carries at any usable rolloff.
            std::size_t branch = static_cast<std::size_t>(std::ceil(11.0 * settings.samples_per_symbol));
            branch += branch % 2UZ;
            prototype = gr::filter::fir::design::rootRaisedCosine(branch * arms - 1UZ, static_cast<double>(arms) * settings.samples_per_symbol, settings.rolloff, static_cast<double>(arms));
        }

        _arms         = arms;
        _branches     = partitionBranchMajor(prototype, arms);
        _windowLength = _branches.size() / arms;
        _anchorDelay  = _windowLength / 2UZ - 1UZ;
    }

    /**
     * @brief The row or branch a fraction in `[0, 1)` selects, without a library rounding call.
     *
     * `MmseInterpolatorBank::row` states the rule and reaches `std::nearbyint` for it, which the compiler will not
     * inline at any `-march`, since it has to respect the dynamic rounding mode, so it is a libm call on every
     * interpolation. Truncating `x + 0.5` is the same selection for every input except an exact half, where this
     * rounds up and `nearbyint` rounds to even: one row of a 129-row bank, 1/256 of a sample.
     */
    [[nodiscard]] static std::size_t quantize(double fraction, std::size_t steps) noexcept {
        const std::size_t index = static_cast<std::size_t>(fraction * static_cast<double>(steps) + 0.5);
        return index < steps ? index : steps;
    }

    template<bool Polyphase>
    [[nodiscard]] T interpolate(const T* window, double fraction) const noexcept {
        if constexpr (!Polyphase) {
            return _bank.interpolate(window, quantize(fraction, _bank.steps()));
        } else {
            std::size_t branch = quantize(fraction, _arms);
            if (branch >= _arms) {
                branch = 0UZ;
                ++window;
            }
            const float* const taps = _branches.data() + branch * _windowLength;
            T                  sum{};
            for (std::size_t k = 0UZ; k < _windowLength; ++k) {
                sum += taps[k] * window[k];
            }
            return sum;
        }
    }

    /// @brief Early-late's sample after the symbol instant, one interpolation ahead at the period this symbol still holds.
    template<bool Polyphase>
    [[nodiscard]] T lookahead(const T* x, std::uint64_t windowStart) const noexcept {
        const double        advanced = _mu + _interpolationPeriod;
        const std::uint64_t whole    = static_cast<std::uint64_t>(advanced);
        return interpolate<Polyphase>(x + (_base + whole - windowStart), advanced - static_cast<double>(whole));
    }

    [[nodiscard]] static T scale(T value, float factor) noexcept { return value * factor; }

    /// @brief The unit-power hard decision: `+/-1` on a real stream (`pam4`: the nearest of `{+/-1, +/-3}/sqrt(5)`),
    /// the nearest of four at `(+/-1 +/- j)/sqrt(2)` on a complex one.
    [[nodiscard]] T decide(T y) const noexcept {
        if constexpr (std::same_as<T, float>) {
            if (_pam4) {
                constexpr float kInner = 0.4472136f; // 1/sqrt(5): unit average power over the four levels
                const float     level  = (y > 2.f * kInner || y < -2.f * kInner) ? 3.f * kInner : kInner;
                return y > 0.f ? level : -level;
            }
            return y > 0.f ? 1.f : -1.f;
        } else {
            const float re = y.real() > 0.f ? 1.f : -1.f;
            if (!_qpsk) {
                return T{re, 0.f};
            }
            const float half = std::numbers::sqrt2_v<float> / 2.f;
            return T{half * re, half * (y.imag() > 0.f ? 1.f : -1.f)};
        }
    }

    /// @brief The detector's formula, resolved at compile time — no switch and no function pointer inside the loop.
    template<gr::sync::TimingDetector D>
    [[nodiscard]] float errorOf(T late) const noexcept {
        using gr::sync::TimingDetector;
        if constexpr (D == TimingDetector::mueller_muller) {
            return gr::sync::muellerMullerError(_y[0], _y[1], _a[0], _a[1]);
        } else if constexpr (D == TimingDetector::modified_mueller_muller) {
            return gr::sync::modifiedMuellerMullerError(_y[0], _y[1], _y[2], _a[0], _a[1], _a[2]);
        } else if constexpr (D == TimingDetector::zero_crossing) {
            return gr::sync::zeroCrossingError(_midpoint, _a[0], _a[1]);
        } else if constexpr (D == TimingDetector::gardner) {
            return gr::sync::gardnerError(_y[0], _y[1], _midpoint);
        } else if constexpr (D == TimingDetector::early_late) {
            return gr::sync::earlyLateError(_y[0], _midpoint, late);
        } else if constexpr (D == TimingDetector::signal_times_slope_ml) {
            return gr::sync::signalTimesSlopeError(_y[0], _slope);
        } else {
            return gr::sync::signumTimesSlopeError(_y[0], _slope);
        }
    }

    [[nodiscard]] static std::optional<double> finiteNumber(const pmt::Value& value) noexcept {
        if (const std::optional<float> real = detail::finiteReal(value); real.has_value()) {
            return static_cast<double>(*real);
        }
        return std::nullopt;
    }

    /**
     * @brief Whether a timing tag's `gr:trigger_name` is the one this block was told to act on.
     *
     * An empty `trigger` honors a timing tag whatever name it carries or does not, which is the behavior a stream with
     * one timing producer on it wants. A non-empty one is a filter, so a graph carrying two producers — a preamble
     * estimator and a correlator, say — can send each block's presets to the block that asked for them.
     */
    [[nodiscard]] bool namedForThisBlock(const property_map& map) const noexcept {
        if (settings.trigger.value.empty()) {
            return true;
        }
        const auto named = map.find(property_map::key_type(gr::tag::TRIGGER_NAME.key()));
        if (named == map.end()) {
            return false;
        }
        const auto* label = named->second.get_if<std::pmr::string>();
        return label != nullptr && std::string_view(label->data(), label->size()) == std::string_view(settings.trigger.value);
    }

    /// @brief Read `clock_est`'s `{offset, period}` out of a two-element real tensor.
    [[nodiscard]] static std::optional<std::pair<double, double>> clockPayload(const pmt::Value& value) noexcept {
        if (const auto* wide = value.get_if<gr::Tensor<double>>(); wide != nullptr && wide->size() == 2UZ) {
            return std::pair{(*wide)[0], (*wide)[1]};
        }
        if (const auto* narrow = value.get_if<gr::Tensor<float>>(); narrow != nullptr && narrow->size() == 2UZ) {
            return std::pair{static_cast<double>((*narrow)[0]), static_cast<double>((*narrow)[1])};
        }
        return std::nullopt;
    }

    /**
     * @brief Take every tag in the window exactly once, whether or not this call reaches it.
     *
     * A tag is collected as soon as it is visible and held until an output anchor passes it, so placement depends on
     * the anchor sequence alone and not on where the scheduler put its call boundaries. `_tagsSeenThrough` is what
     * keeps a re-presented tag — anything past the point this call consumes to — from being collected twice.
     */
    void collectTags(const auto& inSpan, std::uint64_t inputEnd) {
        const property_map::key_type timeKey{kTimeEstKey};
        const property_map::key_type clockKey{kClockEstKey};

        for (const gr::Tag& tag : inSpan.rawTags) {
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < _tagsSeenThrough || at >= inputEnd) {
                continue;
            }

            const auto time  = tag.map.find(timeKey);
            const auto clock = tag.map.find(clockKey);
            if (time == tag.map.end() && clock == tag.map.end()) {
                _pendingTags.emplace_back(at, tag.map);
                continue;
            }

            // clock_est wins where both appear, and a payload that fails any part of its validation takes the whole
            // tag with it: half of a repositioning leaves a state the caller cannot reproduce.
            std::optional<double> instant;
            double                period = 0.0;
            if (clock != tag.map.end()) {
                if (const auto payload = clockPayload(clock->second); payload.has_value()) {
                    instant = payload->first;
                    period  = payload->second;
                }
            } else {
                instant = finiteNumber(time->second);
            }

            const bool usable = settings.trigger_presets && namedForThisBlock(tag.map) && instant.has_value() && std::abs(*instant) <= 1.0 && (period == 0.0 || (period >= _loop.minFrequency() && period <= _loop.maxFrequency()));
            if (usable) {
                _timingTags.push_back({at, *instant, period});
            } else {
                ++_ignoredTags;
            }

            property_map rest = tag.map;
            rest.erase(timeKey);
            rest.erase(clockKey);
            if (!rest.empty()) {
                _pendingTags.emplace_back(at, std::move(rest));
            }
        }
        _tagsSeenThrough = std::max(_tagsSeenThrough, inputEnd);
    }

    /**
     * @brief Snap the position onto a measured instant, at the step whose own instant that measurement names.
     *
     * The position only ever moves forward, so a target the anchor has already gone past cannot be reached at all and
     * is ignored and counted rather than applied with the advance clamped at zero — which would force `mu` to zero and
     * clear the history at a phase nobody asked for, destroying a position the loop already holds to honor a request
     * it cannot serve. Which step carries the snap is what keeps the count of samples intact. A target within half an
     * interpolation period of the position names the instant this step is about to take, and the snap moves that
     * instant onto it before the step runs. A target further ahead names the instant *after* this one, and the snap
     * replaces the step's own advance instead, so the sample this step carries is still taken at the position the
     * loop holds. A snap taken at a step whose instant it does not name swallows that step's sample, which on a burst
     * link deletes a symbol from the stream at the very boundary the tag marks.
     *
     * @return whether the position now sits on a target, so that the caller advances it no further.
     */
    [[nodiscard]] bool applyTimingTags(Preset names) {
        while (!_timingTags.empty()) {
            const detail::TimingTag& wanted   = _timingTags.front();
            const double             offset   = wanted.offset >= _base ? static_cast<double>(wanted.offset - _base) : -static_cast<double>(_base - wanted.offset);
            const double             target   = offset + wanted.instant;
            const double             position = static_cast<double>(_anchorDelay) + _mu;
            const double             horizon  = names == Preset::ThisInstant ? 0.5 * _interpolationPeriod : _interpolationPeriod;
            if (position + horizon <= target) {
                return false;
            }

            const double advance = target - static_cast<double>(_anchorDelay);
            if (advance < 0.0) {
                ++_ignoredTags;
                _timingTags.erase(_timingTags.begin());
                continue;
            }

            const std::uint64_t whole = static_cast<std::uint64_t>(advance);
            _base += whole;
            _mu = advance - static_cast<double>(whole);

            // The loop's integrator is its period estimate, so presetting the period is resetting the filter; the
            // phase goes with it, a diagnostic the position never reads, so that it restarts with the burst.
            if (wanted.period > 0.0) {
                _loop.reset(0.0, wanted.period);
            } else {
                _loop.setPhase(0.0);
            }
            _counter             = _interpolationsPerSymbol - 1;
            _instantPeriod       = _loop.frequency();
            _interpolationPeriod = _instantPeriod / _cycleLength;
            _error               = 0.f;
            _y.fill(T{});
            _a.fill(T{});
            _midpoint = T{};
            _slope    = T{};

            _timingTags.erase(_timingTags.begin());
            return true;
        }
        return false;
    }

    /// @brief Publish every held tag whose input offset the anchor has now reached, in input order, on every wired port.
    void releaseTags(std::uint64_t anchor, std::size_t at, auto& outSpan, auto& errorSpan, auto& instantSpan, auto& averageSpan) {
        std::size_t due = 0UZ;
        while (due < _pendingTags.size() && _pendingTags[due].first <= anchor) {
            ++due;
        }
        if (due == 0UZ) {
            return;
        }
        for (std::size_t i = 0UZ; i < due; ++i) {
            const property_map& forwarded = _pendingTags[i].second;
            outSpan.publishTag(forwarded, at);
            errorSpan.publishTag(forwarded, at);
            instantSpan.publishTag(forwarded, at);
            averageSpan.publishTag(forwarded, at);
        }
        _pendingTags.erase(_pendingTags.begin(), _pendingTags.begin() + static_cast<std::ptrdiff_t>(due));
    }
};

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_SYMBOL_SYNC_HPP
