#ifndef GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP
#define GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/fourier/SpectralCalibration.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <magic_enum.hpp>

namespace gr::blocks::measurement {

namespace detail {

/// @brief The segmenting, windowing and accumulation both spectral blocks run on.
///
/// The window grid is anchored at the stream start: the first segment begins at sample zero and each next one a hop
/// later, so a record's `sample_start` is a stream-absolute fact and the same input split differently yields the same
/// records. Segments are accumulated in power, and the emitted record is the density the calibration kernel defines.
template<typename T>
struct SegmentAccumulator {
    using Real     = float;
    using Spectrum = std::complex<Real>;

    static constexpr bool kRealInput = std::is_same_v<T, Real>;

    /// A real stream goes through the real-to-complex transform, which takes real samples: the windowed segment is
    /// therefore the input's own element type, not the spectrum's.
    using Windowed = std::conditional_t<kRealInput, Real, Spectrum>;

    std::size_t fftSize   = 1024UZ;
    std::size_t hop       = 512UZ;
    std::size_t nAverages = 16UZ;
    bool        maxHold   = false;

    std::vector<Real>                       window{};
    gr::algorithm::fft::SpectralScale<Real> scale{};
    gr::algorithm::FFT<Windowed, Spectrum>  transform{};

    std::vector<T>        pending{};  ///< the tail of the stream not yet covered by a whole segment
    std::vector<Windowed> windowed{}; ///< one segment, windowed, handed to the transform
    std::vector<Spectrum> spectrum{}; ///< the transform's output
    std::vector<Real>     accumulator{};
    std::vector<Real>     hold{}; ///< one segment's density, kept apart so max_hold can compare rather than sum

    std::size_t   segments      = 0UZ;  ///< segments in the accumulator, which is what `n_averaged` reports
    std::uint64_t streamAt      = 0ULL; ///< absolute index of `pending`'s first sample
    std::uint64_t recordStartAt = 0ULL; ///< absolute index of the first segment in the accumulator

    [[nodiscard]] std::size_t bins() const noexcept { return kRealInput ? fftSize / 2UZ + 1UZ : fftSize; }

    /// @brief Build for a transform length. The stream position goes back to zero: a different length is a different
    /// window grid, and there is no honest way to continue the old grid on the new one.
    void configure(std::size_t size, std::size_t hopSize, std::size_t averages, bool holdMode, gr::algorithm::window::Type windowType, Real sampleRate) {
        fftSize   = size;
        transform = gr::algorithm::FFT<Windowed, Spectrum>{};
        windowed.assign(fftSize, Windowed{});
        spectrum.assign(fftSize, Spectrum{});
        pending.clear();
        streamAt = 0ULL;
        reconfigure(hopSize, averages, holdMode, windowType, sampleRate);
    }

    /// @brief Set what may move while the block runs. The accumulation in progress restarts — a spectrum averaged
    /// half under one window and half under another states nothing — but `pending` and `streamAt` survive, so the
    /// grid stays anchored where the stream anchored it and no buffered sample is dropped.
    void reconfigure(std::size_t hopSize, std::size_t averages, bool holdMode, gr::algorithm::window::Type windowType, Real sampleRate) {
        hop       = hopSize;
        nAverages = averages;
        maxHold   = holdMode;
        window    = gr::algorithm::window::create<Real>(windowType, fftSize);
        scale     = gr::algorithm::fft::spectralScale(std::span<const Real>(window), sampleRate);
        hold.assign(bins(), Real{0});
        restart();
    }

    /// @brief Drop the accumulation in progress, keeping the stream position.
    void restart() {
        accumulator.assign(bins(), Real{0});
        segments      = 0UZ;
        recordStartAt = streamAt;
    }

    /// @brief Drop everything, the stream position included. What a fresh run starts from.
    void reset() {
        pending.clear();
        streamAt = 0ULL;
        restart();
    }

    /// @brief Fold one whole segment out of `pending` into the accumulator.
    void accumulateFront() {
        if (segments == 0UZ) {
            recordStartAt = streamAt;
        }
        for (std::size_t k = 0UZ; k < fftSize; ++k) {
            if constexpr (kRealInput) {
                windowed[k] = pending[k] * window[k];
            } else {
                windowed[k] = Spectrum(pending[k].real() * window[k], pending[k].imag() * window[k]);
            }
        }
        transform.compute(windowed, std::span<Spectrum>(spectrum));

        if (maxHold) {
            std::ranges::fill(hold, Real{0});
            gr::algorithm::fft::accumulatePowerSpectrum(std::span<const Spectrum>(spectrum), scale.density, kRealInput, std::span<Real>(hold));
            for (std::size_t k = 0UZ; k < accumulator.size(); ++k) {
                accumulator[k] = segments == 0UZ ? hold[k] : std::max(accumulator[k], hold[k]);
            }
        } else {
            gr::algorithm::fft::accumulatePowerSpectrum(std::span<const Spectrum>(spectrum), scale.density, kRealInput, std::span<Real>(accumulator));
        }
        ++segments;

        const std::size_t advance = std::min(hop, pending.size());
        pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(advance));
        streamAt += advance;
    }

    /// @brief The record the accumulator currently holds, averaged by the segments that actually contributed.
    [[nodiscard]] std::vector<Real> take() {
        std::vector<Real> values = accumulator;
        if (!maxHold && segments > 1UZ) {
            const Real divisor = static_cast<Real>(segments);
            for (Real& value : values) {
                value /= divisor;
            }
        }
        if constexpr (!kRealInput) { // a two-sided axis runs negative to positive, so the halves swap
            std::rotate(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2UZ), values.end());
        }
        accumulator.assign(bins(), Real{0});
        segments = 0UZ;
        return values;
    }
};

/// @brief What one `processBulk` or `processEpilogue` call got through.
struct Progress {
    std::size_t taken      = 0UZ;   ///< input samples moved into the accumulator's pending buffer
    std::size_t made       = 0UZ;   ///< records written to the output span
    bool        outputFull = false; ///< a segment was ready and the record it completes had nowhere to go
};

/// @brief The record every block in this module emits, on the tier's §0 conventions.
template<typename Real>
[[nodiscard]] DataSet<Real> makeSpectralRecord(std::vector<Real> values, Real sampleRate, std::size_t fftSize, bool oneSided, std::uint64_t sampleStart, std::size_t nAveraged, double overlap, Real enbwBins, std::string_view windowName, std::string_view signalName) {
    DataSet<Real> ds;
    const auto    bins = values.size();

    ds.extents = {static_cast<std::int32_t>(bins)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Frequency"};
    ds.axis_units = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(bins);

    const Real binWidth = sampleRate / static_cast<Real>(fftSize);
    const Real offset   = oneSided ? Real{0} : static_cast<Real>(bins / 2UZ) * binWidth;
    for (std::size_t k = 0UZ; k < bins; ++k) {
        ds.axis_values[0UZ][k] = static_cast<Real>(k) * binWidth - offset;
    }

    ds.signal_names      = {std::string(signalName)};
    ds.signal_quantities = {"PowerSpectralDensity"};
    // Linear power per hertz, referred to a full-scale sine, so `10*log10(value)` is dBFS/Hz. The stored form is
    // linear because that is the form the arithmetic downstream needs: a band integral sums it, a detector compares
    // ratios of it, and a further average weights it. A consumer that wants decibels takes the logarithm once, at
    // the point it displays.
    ds.signal_units  = {"1/Hz"};
    ds.signal_values = std::move(values);
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = property_map{
        {std::pmr::string("sample_rate"), pmt::Value(sampleRate)},
        {std::pmr::string("sample_start"), pmt::Value(sampleStart)},
        {std::pmr::string("n_averaged"), pmt::Value(static_cast<std::uint64_t>(nAveraged))},
        // the fraction each averaged segment shared with the next. A consumer that averages these records further, or
        // states a variance for one, needs it: overlapping segments are correlated, so `n_averaged` alone does not say
        // how many independent looks the estimate holds, and a shape that cannot be averaged is refused on this key.
        {std::pmr::string("overlap"), pmt::Value(overlap)},
        {std::pmr::string("enbw_bins"), pmt::Value(enbwBins)},
        {std::pmr::string("window"), pmt::Value(std::string(windowName))},
        {std::pmr::string("fft_size"), pmt::Value(static_cast<std::uint64_t>(fftSize))},
        {std::pmr::string("one_sided"), pmt::Value(oneSided)},
        {std::pmr::string("level_reference"), pmt::Value(std::string("full-scale sine"))},
    };
    ds.timing_events.resize(1UZ);
    ds.timestamp = 0;
    return ds;
}

/// The transform lengths both blocks accept, stated once because both validate against them.
inline constexpr std::size_t kMinFftSize = 64UZ;
inline constexpr std::size_t kMaxFftSize = 65536UZ;

/// @brief The hop `overlap` implies for a transform of `size` samples, never less than one sample.
[[nodiscard]] inline std::size_t hopFor(std::size_t size, double overlap) noexcept { return std::max(1UZ, static_cast<std::size_t>(std::llround(static_cast<double>(size) * (1.0 - overlap)))); }

/// @brief Reject a transform length the segmenting cannot use, naming the value that arrived.
inline void requireFftSize(gr::Size_t size) {
    const auto value = static_cast<std::size_t>(size);
    if (value < kMinFftSize || value > kMaxFftSize || (value & (value - 1UZ)) != 0UZ) {
        throw gr::exception(std::format("fft_size must be a power of two in [{}, {}], got {}", kMinFftSize, kMaxFftSize, size));
    }
}

/// @brief Reject an overlap that does not name a fraction of a segment.
inline void requireOverlap(double overlap) {
    if (!(overlap >= 0.0) || !(overlap < 1.0)) {
        throw gr::exception(std::format("overlap is the fraction a segment shares with the next and must lie in [0, 1), got {}", overlap));
    }
}

/// @brief Reject a sample rate that cannot scale an axis.
inline void requireSampleRate(float sampleRate) {
    if (!(sampleRate > 0.f)) {
        throw gr::exception(std::format("sample_rate must be positive, got {}", sampleRate));
    }
}

/// @brief The window type a name selects, or a refusal naming the vocabulary.
[[nodiscard]] inline gr::algorithm::window::Type requireWindow(const std::string& name) {
    const auto windowType = magic_enum::enum_cast<gr::algorithm::window::Type>(name);
    if (!windowType.has_value()) {
        throw gr::exception(std::format("window must be one of {}, got '{}'", std::string_view(gr::algorithm::window::TypeNames), name));
    }
    return *windowType;
}

/// @brief The refusal both blocks give when `fft_size` is moved under a running graph.
[[nodiscard]] inline gr::exception runningFftSizeRefusal(gr::Size_t was, gr::Size_t asked) { return gr::exception(std::format("fft_size is a staged-restart setting: moving it from {} to {} would rebuild the transform and re-anchor the window grid underneath a running graph, with nothing downstream to say so - stop the graph to change it", was, asked)); }

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::WelchPsd, [T], [ float, std::complex<float> ])

/**
 * @brief Welch's averaged periodogram: overlapping windowed segments, accumulated in power, emitted as one calibrated
 * density record per `n_averages` segments.
 *
 * The estimate trades variance against frequency resolution in the way Welch's method always has: `n_averages`
 * segments reduce the estimator's variance by about that factor while the resolution stays the transform's own.
 * `mode` selects what the accumulation means — `mean` is the estimate, `max_hold` keeps the largest density each bin
 * has reached, which is what finds an intermittent emitter rather than describing a stationary one.
 *
 * Record values are linear power per hertz referred to a full-scale sine, so `10*log10(value)` is dBFS/Hz. The
 * record's `enbw_bins` is what a consumer multiplies a peak density by, along with the bin width, to read a tone's
 * own power rather than a noise density.
 *
 * `fft_size` is staged-restart: changing it rebuilds the transform and re-anchors the window grid, so it is refused
 * while the graph runs. `window`, `overlap`, `n_averages`, `mode` and `sample_rate` are live — they restart the
 * accumulation in progress, since half a spectrum under one window and half under another estimates nothing, but
 * they keep the stream position and every buffered sample.
 *
 * A record states the segment count that actually went into it. A stream that ends mid-accumulation flushes what it
 * has, marked with that count, rather than discarding it or padding it to look complete.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct WelchPsd : Block<WelchPsd<T>, NoTagPropagation> {
    using Description = Doc<"Welch averaged power spectral density: overlapping windowed segments accumulated in power, one calibrated DataSet record per n_averages segments. Values are linear power per hertz referred to a full-scale sine. fft_size is staged-restart; window, overlap, n_averages, mode and sample_rate are live and restart the accumulation in progress">;
    using Real        = float;

    PortIn<T>                     in;
    PortOut<DataSet<Real>, Async> out;

    Annotated<gr::Size_t, "fft_size", Visible, Doc<"transform length, a power of two in [64, 65536]; staged-restart">> fft_size    = 1024U;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                   window      = std::string("Hann");
    Annotated<double, "overlap", Visible, Doc<"fraction of a segment shared with the next, in [0, 1)">>                overlap     = 0.5;
    Annotated<gr::Size_t, "n_averages", Visible, Doc<"segments per emitted record; 1 is a bare periodogram">>          n_averages  = 16U;
    Annotated<std::string, "mode", Visible, Doc<"mean or max_hold">>                                                   mode        = std::string("mean");
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate, which sets the record's axis">>       sample_rate = 1.f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                     signal_name = std::string("psd");

    GR_MAKE_REFLECTABLE(WelchPsd, in, out, fft_size, window, overlap, n_averages, mode, sample_rate, signal_name);

    detail::SegmentAccumulator<T> _core{};
    bool                          _flushed      = false;
    bool                          _running      = false;
    gr::Size_t                    _builtFftSize = 0U; ///< the length the current transform and window were built for

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        // Only these change what the accumulator is; `signal_name` on its own must not restart an average in progress.
        static constexpr std::array kRebuildKeys{"fft_size", "window", "overlap", "n_averages", "mode", "sample_rate"};
        const bool                  built = !_core.window.empty();
        if (built && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }

        detail::requireFftSize(fft_size);
        detail::requireOverlap(overlap);
        detail::requireSampleRate(sample_rate);
        if (n_averages.value == 0U) {
            throw gr::exception("n_averages counts the segments an estimate is made of and must be at least 1");
        }
        if (mode.value != "mean" && mode.value != "max_hold") {
            throw gr::exception(std::format("mode must be 'mean' or 'max_hold', got '{}'", mode.value));
        }
        const auto windowType = detail::requireWindow(window.value);

        // A refused change stays staged and is written again on the next apply, so the block keeps refusing until it
        // is stopped and the value put back. That is the framework's behavior for any throwing settings change.
        if (_running && built && fft_size.value != _builtFftSize) {
            throw detail::runningFftSizeRefusal(_builtFftSize, fft_size);
        }

        const std::size_t size = static_cast<std::size_t>(fft_size.value);
        const std::size_t hop  = detail::hopFor(size, overlap);
        if (built && fft_size.value == _builtFftSize) {
            _core.reconfigure(hop, static_cast<std::size_t>(n_averages.value), mode.value == "max_hold", windowType, sample_rate);
        } else {
            _core.configure(size, hop, static_cast<std::size_t>(n_averages.value), mode.value == "max_hold", windowType, sample_rate);
            _flushed = false;
        }
        _builtFftSize = fft_size;
    }

    void start() {
        _core.reset();
        _flushed = false;
        _running = true;
        // The framework runs `processEpilogue` only over a non-empty trailing span, and that epilogue is what flushes
        // a partial accumulation at end of stream. `processBulk` therefore always leaves one sample unconsumed, and
        // asking for two keeps that from stalling the steady state.
        in.min_samples = 2UZ;
    }

    void stop() { _running = false; }

    /**
     * @brief The no-record-lost invariant: a sample is consumed only once it has been moved into the accumulator, and
     * a segment is folded only when the output span still has room for the record it might complete.
     *
     * When the output fills, the call publishes what it made, consumes what it took, and reports that it stopped for
     * want of output room, so a slow consumer stalls the input rather than growing a backlog inside this block;
     * `pending` therefore never holds more than one segment.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = fold(inSpan, inSpan.size() > 0UZ ? inSpan.size() - 1UZ : 0UZ, outSpan);
        if (progress.outputFull && progress.made == 0UZ && progress.taken == 0UZ) {
            outSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan.publish(progress.made);
        std::ignore = inSpan.consume(progress.taken);
        return work::Status::OK;
    }

    /// @brief End of stream: fold the trailing samples, then emit the accumulation in progress marked with the segment
    /// count that actually reached it. The framework consumes the trailing span itself.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        detail::Progress progress = fold(inSpan, inSpan.size(), outSpan);
        if (!_flushed && _core.segments > 0UZ && progress.made < outSpan.size()) {
            outSpan[progress.made] = emit();
            ++progress.made;
            _flushed = true;
        }
        outSpan.publish(progress.made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] detail::Progress fold(InputSpanLike auto& inSpan, std::size_t offer, OutputSpanLike auto& outSpan) {
        detail::Progress progress{};
        for (;;) {
            if (_core.pending.size() >= _core.fftSize) {
                if (_core.segments + 1UZ >= _core.nAverages && progress.made == outSpan.size()) {
                    progress.outputFull = true;
                    return progress;
                }
                _core.accumulateFront();
                if (_core.segments >= _core.nAverages) {
                    outSpan[progress.made] = emit();
                    ++progress.made;
                }
                continue;
            }
            if (progress.taken == offer) {
                return progress;
            }
            const std::size_t take  = std::min(_core.fftSize - _core.pending.size(), offer - progress.taken);
            const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(progress.taken);
            _core.pending.insert(_core.pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
            progress.taken += take;
        }
    }

    [[nodiscard]] DataSet<Real> emit() {
        const std::size_t   averaged = _core.segments;
        const std::uint64_t startAt  = _core.recordStartAt;
        return detail::makeSpectralRecord<Real>(_core.take(), sample_rate, _core.fftSize, detail::SegmentAccumulator<T>::kRealInput, startAt, averaged, overlap, _core.scale.enbwBins, window.value, signal_name.value);
    }
};

GR_REGISTER_BLOCK(gr::blocks::measurement::Spectrogram, [T], [ float, std::complex<float> ])

/**
 * @brief One calibrated density record per windowed transform, at hop `fft_size * (1 - overlap)`.
 *
 * The same machinery as `WelchPsd` with the averaging removed, and a separate registration because the consumer
 * contract differs: this is a stream of rows in time, not a settled estimate. Each record's `sample_start` is the
 * absolute index of the hop it was taken from, so a consumer stacks rows on a real time axis rather than on arrival
 * order. Values are linear power per hertz referred to a full-scale sine, as `WelchPsd`'s are.
 *
 * `fft_size` is staged-restart and refused while the graph runs; `window`, `overlap` and `sample_rate` are live.
 * A row is one whole transform, so there is no partial record to flush: the stream stops between rows.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Spectrogram : Block<Spectrogram<T>, NoTagPropagation> {
    using Description = Doc<"Spectrogram: one calibrated power spectral density record per windowed transform, timestamped by the hop it came from. Values are linear power per hertz referred to a full-scale sine. fft_size is staged-restart; window, overlap and sample_rate are live">;
    using Real        = float;

    PortIn<T>                     in;
    PortOut<DataSet<Real>, Async> out;

    Annotated<gr::Size_t, "fft_size", Visible, Doc<"transform length, a power of two in [64, 65536]; staged-restart">> fft_size    = 1024U;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                   window      = std::string("Hann");
    Annotated<double, "overlap", Visible, Doc<"fraction of a segment shared with the next, in [0, 1)">>                overlap     = 0.5;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate, which sets the record's axis">>       sample_rate = 1.f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                     signal_name = std::string("spectrogram");

    GR_MAKE_REFLECTABLE(Spectrogram, in, out, fft_size, window, overlap, sample_rate, signal_name);

    detail::SegmentAccumulator<T> _core{};
    bool                          _running      = false;
    gr::Size_t                    _builtFftSize = 0U; ///< the length the current transform and window were built for

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_size", "window", "overlap", "sample_rate"};
        const bool                  built = !_core.window.empty();
        if (built && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }

        detail::requireFftSize(fft_size);
        detail::requireOverlap(overlap);
        detail::requireSampleRate(sample_rate);
        const auto windowType = detail::requireWindow(window.value);

        if (_running && built && fft_size.value != _builtFftSize) {
            throw detail::runningFftSizeRefusal(_builtFftSize, fft_size);
        }

        const std::size_t size = static_cast<std::size_t>(fft_size.value);
        const std::size_t hop  = detail::hopFor(size, overlap);
        if (built && fft_size.value == _builtFftSize) {
            _core.reconfigure(hop, 1UZ, false, windowType, sample_rate);
        } else {
            _core.configure(size, hop, 1UZ, false, windowType, sample_rate);
        }
        _builtFftSize = fft_size;
    }

    void start() {
        _core.reset();
        _running       = true;
        in.min_samples = 2UZ; // as WelchPsd: a sample is held back so the end-of-stream epilogue has a span to run on
    }

    void stop() { _running = false; }

    /// @brief The no-record-lost invariant `WelchPsd` states, with every folded segment completing a record.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = fold(inSpan, inSpan.size() > 0UZ ? inSpan.size() - 1UZ : 0UZ, outSpan);
        if (progress.outputFull && progress.made == 0UZ && progress.taken == 0UZ) {
            outSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan.publish(progress.made);
        std::ignore = inSpan.consume(progress.taken);
        return work::Status::OK;
    }

    /// @brief End of stream: fold whatever whole segments the trailing samples still complete.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = fold(inSpan, inSpan.size(), outSpan);
        outSpan.publish(progress.made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] detail::Progress fold(InputSpanLike auto& inSpan, std::size_t offer, OutputSpanLike auto& outSpan) {
        detail::Progress progress{};
        for (;;) {
            if (_core.pending.size() >= _core.fftSize) {
                if (progress.made == outSpan.size()) {
                    progress.outputFull = true;
                    return progress;
                }
                const std::uint64_t startAt = _core.streamAt;
                _core.accumulateFront();
                outSpan[progress.made] = detail::makeSpectralRecord<Real>(_core.take(), sample_rate, _core.fftSize, detail::SegmentAccumulator<T>::kRealInput, startAt, 1UZ, overlap, _core.scale.enbwBins, window.value, signal_name.value);
                ++progress.made;
                continue;
            }
            if (progress.taken == offer) {
                return progress;
            }
            const std::size_t take  = std::min(_core.fftSize - _core.pending.size(), offer - progress.taken);
            const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(progress.taken);
            _core.pending.insert(_core.pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
            progress.taken += take;
        }
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP
