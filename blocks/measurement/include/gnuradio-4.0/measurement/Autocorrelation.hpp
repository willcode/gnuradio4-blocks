#ifndef GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP
#define GNURADIO_MEASUREMENT_AUTOCORRELATION_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
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

/// @brief Progress of one `processBulk` or `processEpilogue` call, with a separate record count for the phase port.
struct AcfProgress {
    std::size_t taken      = 0UZ;   ///< input samples moved into the pending window
    std::size_t made       = 0UZ;   ///< records written to the correlation output span
    std::size_t madePhase  = 0UZ;   ///< records written to the phase output span
    bool        outputFull = false; ///< a ready window could complete a record, and the output had no room for it
};

/// @brief Builds one record on the module's conventions, with a lag axis in seconds.
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

/// @brief Rejects a window length outside [16, 1048576], naming the value.
inline void requireAcfWindowLength(gr::Size_t length) {
    if (length < 16U || length > 1048576U) {
        throw gr::exception(std::format("window_length must lie in [16, 1048576] samples, got {}", length));
    }
}

/// @brief Rejects a `max_lag` of zero or of at least `window_length`, naming both values.
inline void requireAcfLag(gr::Size_t maxLag, gr::Size_t windowLength) {
    if (maxLag == 0U) {
        throw gr::exception("max_lag is the longest published lag and must be at least 1 sample");
    }
    if (maxLag >= windowLength) {
        throw gr::exception(std::format("max_lag ({}) must be below window_length ({})", maxLag, windowLength));
    }
}

/// @brief Rejects a per-record false-alarm rate outside (0, 1).
inline void requireFalseAlarmRate(double rate) {
    if (!(rate > 0.) || !(rate < 1.)) {
        throw gr::exception(std::format("false_alarm_rate is a probability per published record and must lie in (0, 1), got {}", rate));
    }
}

/// @brief Rejects an `n_averages` of zero.
inline void requireAcfAverages(gr::Size_t averages) {
    if (averages == 0U) {
        throw gr::exception("n_averages counts the windows a record is made of and must be at least 1");
    }
}

/// @brief The setting values the hot path reads, as the last accepted application left them.
struct AcfAccepted {
    std::string window{};
    bool        emitPhase{};
    std::string signalName{};
};

/// @brief The refusal of a staged-restart key that changes while the block runs.
[[nodiscard]] inline gr::exception runningAcfKeyRefusal(std::string_view key) { return gr::exception(std::format("{} is a staged-restart setting; stop the block to change it", key)); }

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::Autocorrelation, [T], [ float, std::complex<float> ])

/**
 * @brief Publishes one autocorrelation record per averaged group of windows, with the scatter and detection threshold.
 *
 * The record shows the lags at which a signal repeats. `kind` selects the estimator. `complex` estimates
 * `R(tau) = E[x(t) x*(t - tau)]` on the input samples. It is coherent. Its phase at a known lag is a frequency
 * estimate, and a carrier offset leaves its magnitude unchanged. `envelope` estimates the autocorrelation of the
 * mean-removed `|x|^2`. Oscillator phase noise does not change it. The two kinds show different features. An OFDM
 * cyclic prefix shows in both. A shaped linear modulation's symbol period shows only in the envelope. Under the
 * envelope kind a constant-modulus window has nothing to correlate. Such an input publishes no record, and
 * `nDegenerate()` counts its windows. `nNonFinite()` counts the windows that hold a NaN or an infinity.
 *
 * `normalization` selects the divisor. `unbiased`, the default, divides lag `tau` by the taper's own autocorrelation
 * `r_w(tau)` and estimates `R(tau)`. `biased` divides every lag by `r_w(0)` and estimates `R(tau) r_w(tau)/r_w(0)`.
 * Under a rectangular taper that is a triangularly windowed correlation. The biased estimate's variance falls with
 * the lag, and the estimate is positive-semidefinite.
 *
 * The record carries `zero_lag`, `scatter`, `scatter_lag`, `detection_threshold`, `false_alarm_rate` and
 * `peak_threshold_db`. `zero_lag` is `R(0)` of the correlated sequence: in the input's units squared under the
 * complex kind and to the fourth power under the envelope kind. `scatter` is the standard deviation of a noise
 * record at its noisiest lag, `scatter_lag`. Noise alone clears `detection_threshold` at some lag in a fraction
 * `false_alarm_rate` of records. `peak_threshold_db` is a height in decibels over the median of a noise record. It
 * applies the noisiest lag's threshold at every lag. Noise alone clears it in at most a fraction `false_alarm_rate` of
 * records.
 *
 * Both geometry settings are in samples, and the record's axis is in seconds. A `sample_rate` tag moves the axis
 * without re-planning the transform. A record averages `n_averages` windows, covers `N + (n_averages-1) H` distinct
 * samples and goes out every `n_averages` hops. `window_length`, `max_lag`, `kind`, `normalization`, `overlap`,
 * `window` and `remove_mean` are staged-restart settings, refused while the block runs. `n_averages`,
 * `false_alarm_rate`, `emit_phase`, `signal_name` and `sample_rate` are live. The block computes on the values of the
 * last accepted application, and the records state them. A refused value stays in its member. Later live changes
 * still apply, and the next `start()` builds the block from the members.
 *
 * A `sample_rate` change takes effect at the sample its tag sits on. It discards the window in progress, which spans
 * two rates, and flushes the accumulation as a short record with the old rate on its axis. The short record carries
 * its own averaged count, distinct-sample count, scatter and threshold. While the block runs, a rate that is not
 * positive and finite leaves the previous rate in force, and `nRateRefused()` counts it. At end of stream the block
 * publishes the accumulation in progress as a short record, and `nTailDropped()` counts the samples no window
 * covered. `nRecordsDropped()` counts the records the end of the stream found no output room for.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Autocorrelation : Block<Autocorrelation<T>, NoTagPropagation> {
    using Description = Doc<"Computes a windowed, averaged autocorrelation. Each DataSet record holds |R(tau)|/R(0) over lags 0..max_lag from n_averages windows, with its scatter and detection threshold. kind selects the complex or the mean-removed envelope estimator. Under the envelope kind a constant-modulus input publishes no record and is counted as degenerate. window_length, max_lag, kind, normalization, overlap, window and remove_mean are staged-restart settings. n_averages, false_alarm_rate, emit_phase, signal_name and sample_rate are live.">;
    using Real        = float;

    PortIn<T>                               in;
    PortOut<DataSet<Real>, Async>           out;   ///< `|R(tau)|/R(0)` over lags 0..max_lag
    PortOut<DataSet<Real>, Async, Optional> phase; ///< `arg R(tau)`, fed only when `emit_phase` is set and `kind` is complex

    Annotated<gr::Size_t, "window_length", Visible, Doc<"samples per window, in [16, 1048576]">>                        window_length    = 4096U;
    Annotated<gr::Size_t, "max_lag", Visible, Doc<"longest published lag in samples, below window_length">>             max_lag          = 1024U;
    Annotated<std::string, "kind", Visible, Doc<"estimator kind, 'complex' or 'envelope'">>                             kind             = std::string("envelope");
    Annotated<std::string, "normalization", Visible, Doc<"lag divisor, 'unbiased' or 'biased'">>                        normalization    = std::string("unbiased");
    Annotated<gr::Size_t, "n_averages", Visible, Doc<"windows per emitted record">>                                     n_averages       = 16U;
    Annotated<double, "overlap", Visible, Doc<"fraction of a window shared with the next, in [0, 1)">>                  overlap          = 0.5;
    Annotated<bool, "remove_mean", Visible, Doc<"remove each window's mean before correlating">>                        remove_mean      = true;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                    window           = std::string("Rectangular");
    Annotated<double, "false_alarm_rate", Visible, Doc<"probability per record that noise alone clears the threshold">> false_alarm_rate = 1e-3;
    Annotated<bool, "emit_phase", Visible, Doc<"publish arg R(tau) on the phase port, complex kind only">>              emit_phase       = false;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate">>                                      sample_rate      = 1.f;
    Annotated<std::string, "signal_name", Doc<"signal name of the emitted record, with _phase on the phase record">>    signal_name      = std::string("acf");

    GR_MAKE_REFLECTABLE(Autocorrelation, in, out, phase, window_length, max_lag, kind, normalization, n_averages, overlap, remove_mean, window, false_alarm_rate, emit_phase, sample_rate, signal_name);

    gr::analysis::Autocorrelation<T> _kernel{};
    std::vector<T>                   _pending{};  ///< samples taken and not yet covered by a whole window
    gr::analysis::AcfConfig          _built{};    ///< the configuration the kernel holds
    detail::AcfAccepted              _accepted{}; ///< the values the hot path reads
    bool                             _configured = false;
    bool                             _running    = false;
    bool                             _flushed    = false;
    float                            _rate       = 1.f; ///< the rate in force for the windows in the accumulation
    float                            _nextRate   = 1.f; ///< the rate a tag moved to, in force from the next window
    bool                             _rateMoved  = false;
    std::uint64_t                    _sequence   = 0ULL;

    std::atomic<std::uint64_t> _nSamples{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nRateRefused{0ULL};
    std::atomic<std::uint64_t> _nTailDropped{0ULL};
    std::atomic<std::uint64_t> _nRecordsDropped{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (_running) {
            applyMoved(newSettings);
        } else {
            applySettings();
        }
    }

    /// @brief Checks every member and builds the kernel from the members. `start()` runs it too. A construction that
    /// changes no value calls no `settingsChanged()`.
    void applySettings() {
        detail::requireAcfWindowLength(window_length);
        detail::requireAcfLag(max_lag, window_length);
        detail::requireOverlap(overlap);
        detail::requireFalseAlarmRate(false_alarm_rate);
        detail::requireAcfAverages(n_averages);
        detail::requireSampleRate(sample_rate);
        const auto parsedKind = gr::analysis::acfKindFrom(kind.value);
        if (!parsedKind.has_value()) {
            throw gr::exception(std::format("kind must be 'complex' or 'envelope', got '{}'", kind.value));
        }
        const auto parsedNormalization = gr::analysis::acfNormalizationFrom(normalization.value);
        if (!parsedNormalization.has_value()) {
            throw gr::exception(std::format("normalization must be 'biased' or 'unbiased', got '{}'", normalization.value));
        }
        const auto windowType = detail::requireWindow(window.value);

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
        _accepted = detail::AcfAccepted{.window = window.value, .emitPhase = emit_phase.value, .signalName = signal_name.value};
    }

    /// @brief Applies the keys an application moved while the block runs. The core names only those keys. The block
    /// refuses the whole application when a moved staged-restart key differs from the built configuration, or when a
    /// moved `n_averages` or `false_alarm_rate` lies out of range. A refused application changes nothing the block
    /// computes. A rate that is not positive and finite is counted, and the last accepted rate stays.
    void applyMoved(const property_map& moved) {
        const auto named       = [&moved](std::string_view key) { return moved.contains(std::pmr::string(key)); };
        const auto refuseMoved = [&named](std::string_view key, bool differs) {
            if (named(key) && differs) {
                throw detail::runningAcfKeyRefusal(key);
            }
        };
        refuseMoved("window_length", static_cast<std::size_t>(window_length.value) != _built.windowLength);
        refuseMoved("max_lag", static_cast<std::size_t>(max_lag.value) != _built.maxLag);
        refuseMoved("kind", gr::analysis::acfKindFrom(kind.value) != _built.kind);
        refuseMoved("normalization", gr::analysis::acfNormalizationFrom(normalization.value) != _built.normalization);
        refuseMoved("overlap", overlap.value != _built.overlap);
        refuseMoved("window", window.value != _accepted.window);
        refuseMoved("remove_mean", remove_mean.value != _built.removeMean);
        if (named("n_averages")) {
            detail::requireAcfAverages(n_averages);
        }
        if (named("false_alarm_rate")) {
            detail::requireFalseAlarmRate(false_alarm_rate);
        }

        if (named("n_averages")) {
            _built.nAverages = static_cast<std::size_t>(n_averages.value);
            _kernel.setAveraging(_built.nAverages);
        }
        if (named("false_alarm_rate")) {
            _built.falseAlarmRate = false_alarm_rate;
            _kernel.setFalseAlarmRate(_built.falseAlarmRate);
        }
        if (named("emit_phase")) {
            _accepted.emitPhase = emit_phase.value;
        }
        if (named("signal_name")) {
            _accepted.signalName = signal_name.value;
        }
        if (named("sample_rate")) {
            if (!(sample_rate.value > 0.f) || !std::isfinite(sample_rate.value)) {
                _nRateRefused.fetch_add(1ULL, std::memory_order_relaxed);
                sample_rate = _nextRate; // the last accepted rate
            } else if (sample_rate.value != _nextRate) {
                _nextRate  = sample_rate;
                _rateMoved = true;
            }
        }
    }

    void start() {
        applySettings(); // from the members, a value refused while the block ran included
        _sequence  = 0ULL;
        _flushed   = false;
        _rateMoved = false;
        _running   = true;
        _nSamples.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nRateRefused.store(0ULL, std::memory_order_relaxed);
        _nTailDropped.store(0ULL, std::memory_order_relaxed);
        _nRecordsDropped.store(0ULL, std::memory_order_relaxed);
    }

    void stop() {
        // the pending samples past the end of the last window
        const std::uint64_t pendingEnd = _kernel.streamAt() + static_cast<std::uint64_t>(_pending.size());
        const std::uint64_t covered    = std::max(_kernel.coveredUntil(), _kernel.streamAt());
        _nTailDropped.fetch_add(pendingEnd > covered ? pendingEnd - covered : 0ULL, std::memory_order_relaxed);
        _running = false;
    }

    /// @brief Input samples this block has taken. Callable from any thread.
    [[nodiscard]] std::uint64_t nSamples() const noexcept { return _nSamples.load(std::memory_order_relaxed); }

    /// @brief Windows the grid produced, degenerate ones included.
    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _kernel.nWindows; }

    /// @brief Records made of a full `n_averages` windows.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _kernel.nRecords; }

    /// @brief Windows in progress discarded because the rate under them moved. Callable from any thread.
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }

    /// @brief Tagged rates that could not scale an axis and were ignored. Callable from any thread.
    [[nodiscard]] std::uint64_t nRateRefused() const noexcept { return _nRateRefused.load(std::memory_order_relaxed); }

    /// @brief Samples no window covered at end of stream. Callable from any thread.
    [[nodiscard]] std::uint64_t nTailDropped() const noexcept { return _nTailDropped.load(std::memory_order_relaxed); }

    /// @brief Records published with fewer than `n_averages` windows behind them.
    [[nodiscard]] std::uint64_t nPartialFlushes() const noexcept { return _kernel.nPartialFlushes; }

    /// @brief Windows behind those partial records.
    [[nodiscard]] std::uint64_t nPartialWindows() const noexcept { return _kernel.nPartialWindows; }

    /// @brief Windows with no fluctuation to correlate. They publish nothing.
    [[nodiscard]] std::uint64_t nDegenerate() const noexcept { return _kernel.nDegenerate; }

    /// @brief Windows holding a NaN or an infinity. They publish nothing.
    [[nodiscard]] std::uint64_t nNonFinite() const noexcept { return _kernel.nNonFinite; }

    /// @brief Records the end of stream found no output room for, on either port. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecordsDropped() const noexcept { return _nRecordsDropped.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        const detail::AcfProgress progress = fold(inSpan, outSpan, phaseSpan, false);
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

    /// @brief At end of stream, folds in the trailing samples and emits the accumulation in progress. The record
    /// states how many windows it holds. A record the outputs have no room for is counted in `nRecordsDropped()`.
    /// The framework consumes the trailing span itself.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan) {
        detail::AcfProgress progress = fold(inSpan, outSpan, phaseSpan, true);
        if (!_flushed) {
            std::ignore = _kernel.flush([&](const gr::analysis::AcfResult& result) { emit(result, outSpan, phaseSpan, progress); });
            _flushed    = true;
        }
        outSpan.publish(progress.made);
        phaseSpan.publish(progress.madePhase);
        return work::Status::OK;
    }

private:
    [[nodiscard]] bool wantsPhase(const auto& phaseSpan) const noexcept { return _accepted.emitPhase && _built.kind == gr::analysis::AcfKind::Complex && phaseSpan.isConnected; }

    [[nodiscard]] bool hasRoom(const auto& outSpan, const auto& phaseSpan, const detail::AcfProgress& progress) const noexcept { return progress.made < outSpan.size() && (!wantsPhase(phaseSpan) || progress.madePhase < phaseSpan.size()); }

    /// @brief Takes input into whole windows and runs them. Before a window that may complete a record it waits for
    /// room on both outputs, except at end of stream, where @p atEnd is set and nothing follows.
    [[nodiscard]] detail::AcfProgress fold(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, bool atEnd) {
        detail::AcfProgress progress{};
        if (!_configured) {
            return progress;
        }
        const std::size_t length = _built.windowLength;

        // A rate tag takes effect before the span that starts on its sample. The grid restarts on that sample.
        if (_rateMoved) {
            if (!atEnd && _kernel.windowsPending() > 0UZ && !hasRoom(outSpan, phaseSpan, progress)) {
                progress.outputFull = true; // the change waits until both outputs have room for the short record
                return progress;
            }
            _rateMoved                    = false;
            const std::uint64_t restartAt = _kernel.streamAt() + static_cast<std::uint64_t>(_pending.size());
            if (!_pending.empty()) {
                // Only a window holding samples is discarded and counted. Without overlap a tag on a window boundary,
                // the stream's first sample included, leaves none.
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
                if (mayComplete && !atEnd && !hasRoom(outSpan, phaseSpan, progress)) {
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
            if (progress.taken == inSpan.size()) {
                return progress;
            }
            const std::size_t take  = std::min(length - _pending.size(), inSpan.size() - progress.taken);
            const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(progress.taken);
            _pending.insert(_pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
            progress.taken += take;
            _nSamples.fetch_add(static_cast<std::uint64_t>(take), std::memory_order_relaxed);
        }
    }

    void emit(const gr::analysis::AcfResult& result, OutputSpanLike auto& outSpan, OutputSpanLike auto& phaseSpan, detail::AcfProgress& progress) {
        const property_map meta = metaOf(result);
        if (progress.made < outSpan.size()) {
            outSpan[progress.made] = detail::makeAcfRecord(result.magnitude, _rate, _accepted.signalName, "Autocorrelation", "", meta);
            ++progress.made;
        } else {
            _nRecordsDropped.fetch_add(1ULL, std::memory_order_relaxed);
        }
        if (wantsPhase(phaseSpan) && !result.phase.empty()) {
            if (progress.madePhase < phaseSpan.size()) {
                phaseSpan[progress.madePhase] = detail::makeAcfRecord(result.phase, _rate, _accepted.signalName + "_phase", "Phase", "rad", meta);
                ++progress.madePhase;
            } else {
                _nRecordsDropped.fetch_add(1ULL, std::memory_order_relaxed);
            }
        }
        ++_sequence;
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
            {std::pmr::string("window"), pmt::Value(_accepted.window)},
            {std::pmr::string("n_averaged"), pmt::Value(static_cast<std::uint64_t>(result.nAveraged))},
            {std::pmr::string("n_samples"), pmt::Value(static_cast<std::uint64_t>(result.nDistinctSamples))},
            // the effective independent sample products behind lag scatter_lag
            {std::pmr::string("n_pairs"), pmt::Value(result.nPairs)},
            {std::pmr::string("scatter_lag"), pmt::Value(static_cast<std::uint64_t>(result.scatterLag))},
            {std::pmr::string("mean_removed"), pmt::Value(_built.removeMean)},
            // true for a real estimate, whose thresholds use the folded normal tail
            {std::pmr::string("real_valued"), pmt::Value(result.realValued)},
            {std::pmr::string("zero_lag"), pmt::Value(result.zeroLag)},
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
