#ifndef GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP
#define GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>

namespace gr::blocks::measurement {

namespace detail {

/// @brief What one `processBulk` or `processEpilogue` call got through, with the phase port's own count beside it.
struct AcfProgress {
    std::size_t taken      = 0UZ;   ///< input samples moved into the undecided tail
    std::size_t made       = 0UZ;   ///< records written to the correlation output span
    std::size_t madePhase  = 0UZ;   ///< records written to the phase output span
    bool        outputFull = false; ///< a window was ready and the record it might complete had nowhere to go
};

/// @brief The record this block emits, on the module's conventions with the axis relabeled to lag in seconds.
[[nodiscard]] inline DataSet<float> makeAcfRecord(std::span<const float> values, float sampleRate, std::string_view signalName, std::string_view quantity, std::string_view unit, const property_map& meta) {
    DataSet<float>    ds;
    const std::size_t lags = values.size();

    ds.extents = {static_cast<std::int32_t>(lags)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Lag"};
    ds.axis_units = {"s"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(lags);
    for (std::size_t k = 0UZ; k < lags; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) / sampleRate;
    }

    ds.signal_names      = {std::string(signalName)};
    ds.signal_quantities = {std::string(quantity)};
    ds.signal_units      = {std::string(unit)};
    ds.signal_values.assign(values.begin(), values.end());
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = meta;
    ds.timing_events.resize(1UZ);
    ds.timestamp = 0;
    return ds;
}

/// @brief Reject a window length the estimator cannot plan a transform for, naming the value that arrived.
inline void requireAcfWindowLength(gr::Size_t length) {
    if (length < 16U || length > 1048576U) {
        throw gr::exception(std::format("window_length must lie in [16, 1048576] samples, got {}", length));
    }
}

/// @brief Reject a lag geometry the estimator cannot measure, naming both values and the reason.
inline void requireAcfLag(gr::Size_t maxLag, gr::Size_t windowLength) {
    if (maxLag == 0U) {
        throw gr::exception("max_lag is the longest published lag and must be at least 1 sample");
    }
    if (maxLag >= windowLength) {
        throw gr::exception(std::format("max_lag ({}) must be below window_length ({}): at a lag of a whole window there is not one sample pair separated by it inside a window, so the unbiased normalization would divide by zero and the biased one would publish a structural zero as a measurement", maxLag, windowLength));
    }
}

/// @brief Reject a per-record false-alarm rate that is not a probability.
inline void requireFalseAlarmRate(double rate) {
    if (!(rate > 0.) || !(rate < 1.)) {
        throw gr::exception(std::format("false_alarm_rate is a probability per published record and must lie in (0, 1), got {}", rate));
    }
}

/// @brief The refusal a staged-restart key gives when it is moved under a running graph.
[[nodiscard]] inline gr::exception runningAcfKeyRefusal(std::string_view key) { return gr::exception(std::format("{} is a staged-restart setting: it changes the transform plan or the meaning of every lag in an accumulation already in flight, with nothing downstream to say so - stop the graph to change it", key)); }

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::Autocorrelation, [T], [ float, std::complex<float> ])

/**
 * @brief What a signal repeats at: one autocorrelation record per averaged group of windows, with the scatter and
 * the detection threshold that decide whether a peak in it is a peak.
 *
 * `kind` selects the instrument. `complex` estimates `R(tau) = E[x(t) x*(t - tau)]` on the samples as they arrive
 * and is coherent, so its phase at a known lag is a frequency estimate and a carrier offset leaves its magnitude
 * alone; `envelope` estimates the autocorrelation of the mean-removed `|x|^2` and is immune to oscillator phase
 * noise. They see different things: an OFDM cyclic prefix shows in both, a shaped linear modulation's symbol rate
 * only in the envelope, and a constant-modulus waveform has no envelope autocorrelation at all — such an input
 * publishes no record and is counted in `nDegenerate()` rather than emitting a curve of `0/0`.
 *
 * `normalization` selects the divisor. `unbiased` divides lag `tau` by the taper's own autocorrelation `r_w(tau)`
 * and estimates `R(tau)` itself; `biased` divides every lag by `r_w(0)` and estimates `R(tau) r_w(tau)/r_w(0)`, a
 * triangularly-windowed correlation whose variance falls with the lag and which is positive-semidefinite. A height
 * compared against a closed form is a statement about `R(tau)` and therefore about `unbiased`, which is the default.
 *
 * The record carries `signal_power`, `scatter`, `detection_threshold`, `false_alarm_rate` and `peak_threshold_db`,
 * which is the whole of what a consumer needs to decide whether a peak is a peak without reconstructing the
 * estimator's statistics. `peak_threshold_db` is that height expressed over the record's own median, which is
 * the form a median-referenced peak detector takes as its threshold; on a lag axis a detection's frequency
 * field is a lag in seconds, and a parabolic interpolation of it refines a period below the sample grid.
 *
 * Both geometry settings are in samples and the record's axis is in seconds, so a `sample_rate` tag moves the axis
 * without re-planning the transform. `window_length`, `max_lag`, `kind`, `normalization`, `overlap`, `window` and
 * `remove_mean` are staged-restart and refused while the graph runs; `n_averages`, `false_alarm_rate`,
 * `emit_phase`, `signal_name` and `sample_rate` are live. A `sample_rate` change discards the window in progress —
 * it straddles two rates and no single lag axis describes it — and flushes the accumulation so far as a short
 * record carrying its own averaged count, distinct-sample count, scatter and threshold, with the old rate on its
 * axis, so a short average is honest rather than misleading.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Autocorrelation : Block<Autocorrelation<T>, NoTagPropagation> {
    using Description = Doc<"Windowed averaged autocorrelation: one DataSet record of |R(tau)|/R(0) over lags 0..max_lag per n_averages windows, with the scatter and the detection threshold the record is read against. kind selects the complex or the mean-removed envelope instrument; a constant-modulus input publishes no record and is counted degenerate. window_length, max_lag, kind, normalization, overlap, window and remove_mean are staged-restart; n_averages, false_alarm_rate, emit_phase, signal_name and sample_rate are live">;
    using Real        = float;

    PortIn<T>                               in;
    PortOut<DataSet<Real>, Async>           out;   ///< `|R(tau)|/R(0)` over lags 0..max_lag
    PortOut<DataSet<Real>, Async, Optional> phase; ///< `arg R(tau)`, fed only when `emit_phase` and `kind` is complex

    Annotated<gr::Size_t, "window_length", Visible, Doc<"samples per window, in [16, 1048576]; staged-restart">>                          window_length    = 4096U;
    Annotated<gr::Size_t, "max_lag", Visible, Doc<"longest published lag in samples, below window_length; staged-restart">>               max_lag          = 1024U;
    Annotated<std::string, "kind", Visible, Doc<"complex (the samples themselves) or envelope (the mean-removed |x|^2)">>                 kind             = std::string("envelope");
    Annotated<std::string, "normalization", Visible, Doc<"unbiased (divide lag tau by r_w(tau)) or biased (divide every lag by r_w(0))">> normalization    = std::string("unbiased");
    Annotated<gr::Size_t, "n_averages", Visible, Doc<"windows per emitted record">>                                                       n_averages       = 16U;
    Annotated<double, "overlap", Visible, Doc<"fraction of a window shared with the next, in [0, 1); it does not reduce the scatter">>    overlap          = 0.5;
    Annotated<bool, "remove_mean", Visible, Doc<"remove each window's own mean before correlating; staged-restart">>                      remove_mean      = true;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                                      window           = std::string("Rectangular");
    Annotated<double, "false_alarm_rate", Visible, Doc<"probability per record that noise alone puts a lag over the threshold">>          false_alarm_rate = 1e-3;
    Annotated<bool, "emit_phase", Visible, Doc<"publish arg R(tau) on the phase port; complex kind only">>                                emit_phase       = false;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate, which sets the record's lag axis">>                      sample_rate      = 1.f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name; the phase record appends _phase">>                       signal_name      = std::string("acf");

    GR_MAKE_REFLECTABLE(Autocorrelation, in, out, phase, window_length, max_lag, kind, normalization, n_averages, overlap, remove_mean, window, false_alarm_rate, emit_phase, sample_rate, signal_name);

    gr::analysis::Autocorrelation<T> _kernel{};
    std::vector<T>                   _pending{}; ///< the undecided tail: samples taken but not yet covered by a whole window
    gr::analysis::AcfConfig          _built{};   ///< the configuration the kernel currently holds
    bool                             _configured = false;
    bool                             _running    = false;
    bool                             _flushed    = false;
    float                            _rate       = 1.f;   ///< the rate in force for the windows in the accumulation
    float                            _nextRate   = 1.f;   ///< the rate a tag moved to, which takes effect at the next window
    bool                             _rateMoved  = false; //
    std::uint64_t                    _sequence   = 0ULL;

    std::atomic<std::uint64_t> _nSamples{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nRateRefused{0ULL};
    std::atomic<std::uint64_t> _nTailDropped{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        detail::requireAcfWindowLength(window_length);
        detail::requireAcfLag(max_lag, window_length);
        detail::requireOverlap(overlap);
        detail::requireFalseAlarmRate(false_alarm_rate);
        if (n_averages.value == 0U) {
            throw gr::exception("n_averages counts the windows a record is made of and must be at least 1");
        }
        const auto parsedKind = gr::analysis::acfKindFrom(kind.value);
        if (!parsedKind.has_value()) {
            throw gr::exception(std::format("kind must be 'complex' or 'envelope', got '{}'", kind.value));
        }
        const auto parsedNormalization = gr::analysis::acfNormalizationFrom(normalization.value);
        if (!parsedNormalization.has_value()) {
            throw gr::exception(std::format("normalization must be 'biased' or 'unbiased', got '{}'", normalization.value));
        }
        const auto windowType = detail::requireWindow(window.value);

        // A tagged rate the estimator cannot put on an axis leaves the previous rate standing and is counted, because
        // a stream tag is a statement about the stream and not a request a graph can be stopped over; the same value
        // arriving as a settings change on a stopped block is refused by name.
        if (!(sample_rate.value > 0.f) || !std::isfinite(sample_rate.value)) {
            if (_running) {
                _nRateRefused.fetch_add(1ULL, std::memory_order_relaxed);
                sample_rate = _rate;
            } else {
                detail::requireSampleRate(sample_rate);
            }
        }

        // A staged-restart key is refused on the value it would move to and not on its mere presence: the framework
        // re-applies a block's whole settings map, so refusing the key would refuse every apply after the first.
        if (_running && _configured) {
            const auto refuse = [](bool moved, std::string_view key) {
                if (moved) {
                    throw detail::runningAcfKeyRefusal(key);
                }
            };
            refuse(static_cast<std::size_t>(window_length.value) != _built.windowLength, "window_length");
            refuse(static_cast<std::size_t>(max_lag.value) != _built.maxLag, "max_lag");
            refuse(*parsedKind != _built.kind, "kind");
            refuse(*parsedNormalization != _built.normalization, "normalization");
            refuse(overlap.value != _built.overlap, "overlap");
            refuse(windowType != _built.window, "window");
            refuse(remove_mean.value != _built.removeMean, "remove_mean");

            if (sample_rate.value != _rate) {
                _nextRate  = sample_rate;
                _rateMoved = true;
            }
            _built.nAverages      = static_cast<std::size_t>(n_averages.value);
            _built.falseAlarmRate = false_alarm_rate;
            _kernel.setAveraging(_built.nAverages);
            _kernel.setFalseAlarmRate(_built.falseAlarmRate);
            return;
        }

        const gr::analysis::AcfConfig config{
            .windowLength   = static_cast<std::size_t>(window_length.value),
            .maxLag         = static_cast<std::size_t>(max_lag.value),
            .kind           = *parsedKind,
            .normalization  = *parsedNormalization,
            .overlap        = overlap,
            .nAverages      = static_cast<std::size_t>(n_averages.value),
            .removeMean     = remove_mean,
            .window         = windowType,
            .falseAlarmRate = false_alarm_rate,
        };
        _kernel.prepare(config);
        _built      = config;
        _configured = true;
        _rate       = sample_rate;
        _nextRate   = sample_rate;
        _pending.clear();
    }

    void start() {
        _kernel.reset();
        _pending.clear();
        _sequence  = 0ULL;
        _flushed   = false;
        _rateMoved = false;
        _rate      = sample_rate;
        _nextRate  = sample_rate;
        _running   = true;
        _nSamples.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nRateRefused.store(0ULL, std::memory_order_relaxed);
        _nTailDropped.store(0ULL, std::memory_order_relaxed);
        // The framework runs `processEpilogue` only over a non-empty trailing span, and that epilogue is what flushes
        // a partial accumulation at end of stream. `processBulk` therefore always leaves one sample unconsumed, and
        // asking for two keeps that from stalling the steady state.
        in.min_samples = 2UZ;
    }

    void stop() {
        _nTailDropped.fetch_add(static_cast<std::uint64_t>(_pending.size()), std::memory_order_relaxed);
        _running = false;
    }

    /// @brief Input samples this block has taken. Callable from any thread.
    [[nodiscard]] std::uint64_t nSamples() const noexcept { return _nSamples.load(std::memory_order_relaxed); }

    /// @brief Windows the grid produced, degenerate ones included.
    [[nodiscard]] std::uint64_t nWindows() const noexcept { return static_cast<std::uint64_t>(_kernel.nWindows); }

    /// @brief Records made of a full `n_averages` windows.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return static_cast<std::uint64_t>(_kernel.nRecords); }

    /// @brief Windows in progress discarded because the rate under them moved. Callable from any thread.
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }

    /// @brief Tagged rates that could not scale an axis and were ignored. Callable from any thread.
    [[nodiscard]] std::uint64_t nRateRefused() const noexcept { return _nRateRefused.load(std::memory_order_relaxed); }

    /// @brief Samples never covered by a whole window at end of stream. Callable from any thread.
    [[nodiscard]] std::uint64_t nTailDropped() const noexcept { return _nTailDropped.load(std::memory_order_relaxed); }

    /// @brief Records published with fewer than `n_averages` windows behind them.
    [[nodiscard]] std::uint64_t nPartialFlushes() const noexcept { return static_cast<std::uint64_t>(_kernel.nPartialFlushes); }

    /// @brief Windows those partial records were made of, which closes the window count's identity.
    [[nodiscard]] std::uint64_t nPartialWindows() const noexcept { return static_cast<std::uint64_t>(_kernel.nPartialWindows); }

    /// @brief Windows with no fluctuation to correlate, which publish nothing.
    [[nodiscard]] std::uint64_t nDegenerate() const noexcept { return static_cast<std::uint64_t>(_kernel.nDegenerate); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        const detail::AcfProgress progress = fold(inSpan, inSpan.size() > 0UZ ? inSpan.size() - 1UZ : 0UZ, outSpan, phaseSpan);
        if (progress.outputFull && progress.made == 0UZ && progress.taken == 0UZ) {
            outSpan.publish(0UZ);
            phaseSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan.publish(progress.made);
        phaseSpan.publish(progress.madePhase);
        std::ignore = inSpan.consume(progress.taken);
        return work::Status::OK;
    }

    /// @brief End of stream: fold the trailing samples, then emit the accumulation in progress marked with the window
    /// count that actually reached it. The framework consumes the trailing span itself.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        detail::AcfProgress progress = fold(inSpan, inSpan.size(), outSpan, phaseSpan);
        if (!_flushed && _kernel.windowsPending() > 0UZ && progress.made < outSpan.size()) {
            std::ignore = _kernel.flush([&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
            _flushed    = true;
        }
        outSpan.publish(progress.made);
        phaseSpan.publish(progress.madePhase);
        return work::Status::OK;
    }

private:
    [[nodiscard]] bool wantsPhase(const auto& phaseSpan) const noexcept { return emit_phase.value && _built.kind == gr::analysis::AcfKind::Complex && phaseSpan.isConnected; }

    [[nodiscard]] detail::AcfProgress fold(InputSpanLike auto& inSpan, std::size_t offer, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        detail::AcfProgress progress{};
        if (!_configured) {
            return progress;
        }
        const std::size_t length = _built.windowLength;

        // The rate under the stream has moved: the window in progress straddles two rates and no single lag axis
        // describes it, so it goes; the accumulation is published as the short record it honestly is, on the old
        // axis, and the next window starts at the sample the tag arrived on.
        if (_rateMoved) {
            if (_kernel.windowsPending() > 0UZ && progress.made >= outSpan.size()) {
                progress.outputFull = true; // the short record has nowhere to go yet, so the change waits for room
                return progress;
            }
            _rateMoved                    = false;
            const std::uint64_t restartAt = _kernel.streamAt() + static_cast<std::uint64_t>(_pending.size());
            if (!_pending.empty()) {
                // only a window that had samples in it is discarded; a rate arriving on a window boundary, the
                // stream's own first sample included, disturbs nothing and is counted as nothing
                _nWindowResets.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _pending.clear();
            std::ignore = _kernel.flush([&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
            _kernel.seek(restartAt);
            _rate = _nextRate;
        }

        for (;;) {
            if (_pending.size() >= length) {
                const bool mayComplete = _kernel.windowsPending() + 1UZ >= _built.nAverages;
                if (mayComplete && (progress.made == outSpan.size() || (wantsPhase(phaseSpan) && progress.madePhase == phaseSpan.size()))) {
                    progress.outputFull = true;
                    return progress;
                }
                const std::size_t used = _kernel.process(std::span<const T>(_pending).first(length), [&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
                if (used == 0UZ) {
                    return progress;
                }
                _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(used));
                continue;
            }
            if (progress.taken == offer) {
                return progress;
            }
            const std::size_t take  = std::min(length - _pending.size(), offer - progress.taken);
            const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(progress.taken);
            _pending.insert(_pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
            progress.taken += take;
            _nSamples.fetch_add(static_cast<std::uint64_t>(take), std::memory_order_relaxed);
        }
    }

    void emit(const gr::analysis::AcfResult& result, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, detail::AcfProgress& progress) {
        if (progress.made >= outSpan.size()) {
            return;
        }
        const property_map meta = metaOf(result);
        outSpan[progress.made]  = detail::makeAcfRecord(result.magnitude, _rate, signal_name.value, "Autocorrelation", "", meta);
        ++progress.made;
        ++_sequence;
        if (wantsPhase(phaseSpan) && !result.phase.empty() && progress.madePhase < phaseSpan.size()) {
            phaseSpan[progress.madePhase] = detail::makeAcfRecord(result.phase, _rate, signal_name.value + "_phase", "Phase", "rad", meta);
            ++progress.madePhase;
        }
    }

    [[nodiscard]] property_map metaOf(const gr::analysis::AcfResult& result) const {
        return property_map{
            {std::pmr::string("sample_rate"), pmt::Value(_rate)},
            {std::pmr::string("sample_start"), pmt::Value(result.sampleStart)},
            {std::pmr::string("sequence"), pmt::Value(_sequence)},
            {std::pmr::string("kind"), pmt::Value(std::string(gr::analysis::acfKindName(_built.kind)))},
            {std::pmr::string("normalization"), pmt::Value(std::string(gr::analysis::acfNormalizationName(_built.normalization)))},
            {std::pmr::string("window_length"), pmt::Value(static_cast<std::uint64_t>(_built.windowLength))},
            {std::pmr::string("max_lag"), pmt::Value(static_cast<std::uint64_t>(_built.maxLag))},
            {std::pmr::string("hop"), pmt::Value(static_cast<std::uint64_t>(_kernel.hop()))},
            {std::pmr::string("overlap"), pmt::Value(_built.overlap)},
            {std::pmr::string("window"), pmt::Value(window.value)},
            {std::pmr::string("n_averaged"), pmt::Value(static_cast<std::uint64_t>(result.nAveraged))},
            {std::pmr::string("n_samples"), pmt::Value(static_cast<std::uint64_t>(result.nDistinctSamples))},
            // the independent sample products behind the worst published lag, which is what the scatter and the
            // threshold are computed from and what a consumer recomputes them from
            {std::pmr::string("n_pairs"), pmt::Value(static_cast<std::uint64_t>(result.nPairs))},
            {std::pmr::string("mean_removed"), pmt::Value(_built.removeMean)},
            // the envelope kind squares magnitudes and the complex kind of a real input correlates real samples, so
            // in both the estimate is real and its tail is folded normal rather than Rayleigh; the two thresholds
            // below are computed for whichever tail this record's estimate actually has
            {std::pmr::string("real_valued"), pmt::Value(result.realValued)},
            // R(0) before normalization: the published ratio throws the scale away and this is where it is kept
            {std::pmr::string("signal_power"), pmt::Value(static_cast<float>(result.power))},
            {std::pmr::string("scatter"), pmt::Value(result.scatter)},
            {std::pmr::string("detection_threshold"), pmt::Value(result.threshold)},
            {std::pmr::string("false_alarm_rate"), pmt::Value(_built.falseAlarmRate)},
            {std::pmr::string("peak_threshold_db"), pmt::Value(result.peakThresholdDb)},
            {std::pmr::string("transform_length"), pmt::Value(static_cast<std::uint64_t>(_kernel.transformLength()))},
        };
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP
