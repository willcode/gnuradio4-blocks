#ifndef GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP
#define GNURADIO_MEASUREMENT_SPECTRAL_ESTIMATE_HPP

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <stdexcept>
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
/// The window grid is anchored at the stream start. The first segment begins at sample zero and each next one a hop
/// later. A record's `sample_start` is therefore a stream-absolute fact, and the same input split differently yields
/// the same records. Segments are accumulated in power. The emitted record is the density the calibration kernel
/// defines.
///
/// A hop larger than the transform leaves a gap between segments. Those samples count toward the stream position and
/// are not copied. The grid stays stream-absolute while the transform runs on a stated fraction of the stream. A
/// display's cost then follows its frame rate and not its sample rate.
template<typename T>
struct SegmentAccumulator {
    using Real     = float;
    using Spectrum = std::complex<Real>;

    static constexpr bool kRealInput = std::is_same_v<T, Real>;

    /// A real stream goes through the real-to-complex transform, which takes real samples. The windowed segment
    /// therefore has the input's own element type, not the spectrum's.
    using Windowed = std::conditional_t<kRealInput, Real, Spectrum>;

    std::size_t fftSize   = 1024UZ;
    std::size_t hop       = 512UZ;
    std::size_t nAverages = 16UZ;
    bool        maxHold   = false;
    std::size_t threads   = 1UZ;                                    ///< threads one transform may use, kept here because `resize` builds a fresh transform
    Real        shape     = std::numeric_limits<Real>::quiet_NaN(); ///< the parameter the window was built with, which the record states

    std::vector<Real>                       window{};
    gr::algorithm::fft::SpectralScale<Real> scale{};
    gr::algorithm::FFT<Windowed, Spectrum>  transform{};

    std::vector<T>        pending{};  ///< the tail of the stream not yet covered by a whole segment
    std::vector<Windowed> windowed{}; ///< one windowed segment, the transform's input
    std::vector<Spectrum> spectrum{}; ///< the transform's output
    std::vector<Real>     accumulator{};
    std::vector<Real>     hold{}; ///< one segment's density, kept apart for the max_hold comparison

    std::size_t   segments      = 0UZ;  ///< segments in the accumulator, reported as `n_averaged`
    std::size_t   skipping      = 0UZ;  ///< samples still to be dropped before the next segment starts, when hop > fftSize
    std::uint64_t streamAt      = 0ULL; ///< absolute index of `pending`'s first sample
    std::uint64_t recordStartAt = 0ULL; ///< absolute index of the first segment in the accumulator
    std::uint64_t gridStart     = 0ULL; ///< absolute index where the window grid in force is anchored, stated in the record

    /// Wall-clock time the record in progress has cost so far, summed over its folded segments and measured on
    /// `steady_clock`. A transform spread over a thread pool the caller waits on is counted whole. The time the block
    /// spent waiting for input is excluded. That time reflects the stream's rate and not the block's cost.
    double costSeconds = 0.;

    [[nodiscard]] std::size_t bins() const noexcept { return kRealInput ? fftSize / 2UZ + 1UZ : fftSize; }

    /// @brief Builds the transform, the window and the buffers for a length. The caller decides whether the stream
    /// position survives. `configure` starts a stream, and `rebuild` moves the grid under a running one.
    void resize(std::size_t size, std::size_t hopSize, std::size_t averages, bool holdMode, gr::algorithm::window::Type windowType, Real windowParam, Real sampleRate) {
        fftSize           = size;
        transform         = gr::algorithm::FFT<Windowed, Spectrum>{};
        transform.threads = threads; // a fresh transform starts at one thread, so the setting in force is re-applied
        windowed.assign(fftSize, Windowed{});
        spectrum.assign(fftSize, Spectrum{});
        reconfigure(hopSize, averages, holdMode, windowType, windowParam, sampleRate);
    }

    /// @brief Builds for a transform length at a stream's start, with the stream position and the grid anchor at zero.
    void configure(std::size_t size, std::size_t hopSize, std::size_t averages, bool holdMode, gr::algorithm::window::Type windowType, Real windowParam, Real sampleRate) {
        pending.clear();
        streamAt  = 0ULL;
        gridStart = 0ULL;
        resize(size, hopSize, averages, holdMode, windowType, windowParam, sampleRate);
    }

    /// @brief Moves to a new transform length under a stream that is already running.
    ///
    /// `pending` and the stream position survive. No buffered sample is dropped, and `sample_start` keeps counting the
    /// same stream. The accumulation does not survive. Half an estimate at one resolution and half at another is not
    /// an estimate. The new grid is anchored at the current stream position and not at the stream's origin. The old
    /// grid's positions are not positions of the new one. `gridStart` records the anchor. A consumer cannot recover it
    /// from a record whose length has changed.
    void rebuild(std::size_t size, std::size_t hopSize, std::size_t averages, bool holdMode, gr::algorithm::window::Type windowType, Real windowParam, Real sampleRate) {
        gridStart = streamAt;
        resize(size, hopSize, averages, holdMode, windowType, windowParam, sampleRate);
    }

    /// @brief Sets the parameters that may change while the block runs. The accumulation in progress restarts,
    /// because a spectrum averaged half under one window and half under another is not an estimate. `pending` and
    /// `streamAt` survive. The grid stays where the stream anchored it, and no buffered sample is dropped.
    void reconfigure(std::size_t hopSize, std::size_t averages, bool holdMode, gr::algorithm::window::Type windowType, Real windowParam, Real sampleRate) {
        if (hopSize != hop) {
            // A new hop is a new grid, as a new length is. The outstanding gap was measured against the replaced hop.
            // The old hop's positions are not positions of the new one. The grid is re-anchored at the current stream
            // position, and `grid_start` reports it. An unchanged hop keeps the gap and the anchor. A live window or
            // averaging change then leaves every segment where it would have fallen.
            skipping  = 0UZ;
            gridStart = streamAt;
        }
        hop       = hopSize;
        nAverages = averages;
        maxHold   = holdMode;
        shape     = windowParam;
        window    = gr::algorithm::window::create<Real>(windowType, fftSize, windowParam);
        scale     = gr::algorithm::fft::spectralScale(std::span<const Real>(window), sampleRate);
        hold.assign(bins(), Real{0});
        restart();
    }

    /// @brief Sets how many threads one transform may use. The setting is live on its own. It changes what a
    /// transform costs and not what it computes. The accumulation in progress, the buffered samples and the grid stay.
    void setThreads(std::size_t count) {
        threads           = count;
        transform.threads = count;
    }

    /// @brief Drops the accumulation in progress and keeps the stream position and the grid. A gap still to be skipped
    /// belongs to the grid and survives. It marks where the next segment starts.
    void restart() {
        accumulator.assign(bins(), Real{0});
        segments      = 0UZ;
        recordStartAt = streamAt;
    }

    /// @brief Drops everything, the stream position and the grid anchor included. A fresh run starts from this state.
    void reset() {
        pending.clear();
        streamAt  = 0ULL;
        gridStart = 0ULL;
        skipping  = 0UZ;
        restart();
    }

    /// @brief Folds one whole segment out of `pending` into the accumulator and adds its cost to `costSeconds`.
    void accumulateFront() {
        const auto started = std::chrono::steady_clock::now();
        if (segments == 0UZ) {
            recordStartAt = streamAt;
            costSeconds   = 0.;
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

        // The hop advances the grid. The part past the buffered samples is a gap the caller consumes without copying.
        // A hop above `fftSize` then advances the stream position without a transform or a memory move.
        const std::size_t advance = std::min(hop, pending.size());
        pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(advance));
        streamAt += advance;
        skipping = hop - advance;

        costSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    }

    /// @brief The record the accumulator holds, averaged over the segments that contributed.
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

/// @brief The fraction a segment shares with the next at a stated hop. It is zero once the hop reaches the whole
/// transform. It is also zero for a hop that leaves a gap, because disjoint segments share nothing.
[[nodiscard]] inline double overlapAt(std::size_t fftSize, std::size_t hop) noexcept { return hop >= fftSize ? 0.0 : 1.0 - static_cast<double>(hop) / static_cast<double>(fftSize); }

/// @brief Seconds of wall clock elapsed since a steady-clock reading.
[[nodiscard]] inline double secondsSince(std::chrono::steady_clock::time_point since) noexcept { return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count(); }

/// @brief The record every block in this module emits.
template<typename Real>
[[nodiscard]] DataSet<Real> makeSpectralRecord(std::vector<Real> values, Real sampleRate, double centerFrequency, std::size_t fftSize, bool oneSided, std::uint64_t sampleStart, std::size_t nAveraged, std::size_t hop, Real enbwBins, std::string_view windowName, Real windowParam, std::uint64_t gridStart, std::string_view signalName) {
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
    // Linear power per hertz, referred to a full-scale sine. `10*log10(value)` is dBFS/Hz. The stored form is linear
    // because later arithmetic needs it. A band integral sums it, a detector compares its ratios, and a further
    // average weights it. A consumer that wants decibels takes the logarithm once, where it displays.
    ds.signal_units  = {"1/Hz"};
    ds.signal_values = std::move(values);
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = property_map{
        {std::pmr::string("sample_rate"), pmt::Value(sampleRate)},
        // The frequency the axis is centered on, in hertz. A consumer places the record on an absolute axis without
        // parsing a name. The axis itself stays baseband, as in the two-sided convention. This key says where zero
        // sits. A receiver that is not tuned leaves it at zero.
        {std::pmr::string("center_frequency"), pmt::Value(centerFrequency)},
        {std::pmr::string("sample_start"), pmt::Value(sampleStart)},
        {std::pmr::string("n_averaged"), pmt::Value(static_cast<std::uint64_t>(nAveraged))},
        // The fraction each averaged segment shared with the next. Overlapping segments are correlated, and
        // `n_averaged` alone does not give the number of independent looks in the estimate. A consumer that averages
        // these records further, or states a variance for one, needs this key. A shape that cannot be averaged is
        // refused on this key.
        {std::pmr::string("overlap"), pmt::Value(overlapAt(fftSize, hop))},
        // The samples the grid advanced between the segments of this record. Above `fft_size` the record describes a
        // stated fraction of the stream and not all of it. A consumer needs this before it reads a duty-cycled
        // estimate as an average over its whole interval.
        {std::pmr::string("hop"), pmt::Value(static_cast<std::uint64_t>(hop))},
        {std::pmr::string("enbw_bins"), pmt::Value(enbwBins)},
        {std::pmr::string("window"), pmt::Value(std::string(windowName))},
        // The shape parameter the window was built with. The window's own default applies where the setting named
        // none. Kaiser at beta 1.6 and Kaiser at beta 6.76 are different windows, with different sidelobes and a
        // different noise bandwidth. A record that names only "Kaiser" does not say which one it is.
        {std::pmr::string("window_param"), pmt::Value(windowParam)},
        {std::pmr::string("fft_size"), pmt::Value(static_cast<std::uint64_t>(fftSize))},
        // The anchor of the window grid in force. It is zero for a whole run whose transform length does not
        // change. A change to `fft_size` under a running graph re-anchors the grid at the position the change landed
        // on. `sample_start` and `fft_size` are per-record and stay correct either way. They do not tell a consumer
        // that the records on either side lie on two grids. This key does.
        {std::pmr::string("grid_start"), pmt::Value(gridStart)},
        {std::pmr::string("one_sided"), pmt::Value(oneSided)},
        {std::pmr::string("level_reference"), pmt::Value(std::string("full-scale sine"))},
    };
    ds.timing_events.resize(1UZ);
    ds.timestamp = 0;
    return ds;
}

/// @brief States a built record's computing cost, in seconds of wall clock, under the key `compute_seconds`.
///
/// The figure covers the windowing, the transform, the power accumulation and the assembly of the record, summed over
/// every segment of the record. It is wall-clock time and not processor time. A transform given more than one thread
/// runs on a pool the calling thread waits on. Only a wall-clock reading around the whole computation covers the
/// record's cost. Time the block spent waiting for input is excluded. The figure states the block's own cost and not
/// the rate of the stream. It is written after the record is built, because building the record is part of the cost.
/// It is a linear `double` in seconds, like every other figure on these records.
template<typename Real>
inline void stateComputeSeconds(DataSet<Real>& ds, double seconds) {
    ds.meta_information[0UZ].insert_or_assign(std::pmr::string("compute_seconds"), pmt::Value(seconds));
}

/// The transform lengths both blocks accept, stated once for both validations. A record carries its values and its
/// frequency axis, `bins` floats each. A record is then `8 * bins` bytes, 64 KiB at 8192 bins and 32 MiB at the
/// ceiling. The accumulator's own working set is about `36 * fft_size` bytes for a complex stream. A consumer at these
/// sizes sizes its record ring in records and expects to drop. `PollerConfig::dataSetDepth` sets that ring's depth.
inline constexpr std::size_t kMinFftSize = 64UZ;
inline constexpr std::size_t kMaxFftSize = 4194304UZ;

/// @brief The hop `overlap` implies for a transform of `size` samples, never less than one sample.
[[nodiscard]] inline std::size_t hopFor(std::size_t size, double overlap) noexcept { return std::max(1UZ, static_cast<std::size_t>(std::llround(static_cast<double>(size) * (1.0 - overlap)))); }

/// @brief The hop in force. A non-zero `hop` setting gives it directly. A zero setting takes the hop `overlap`
/// implies. A stated hop above `size` leaves a gap between segments. A stated hop below `size` is an overlap in
/// samples. It can express a hop that no fraction expresses.
[[nodiscard]] inline std::size_t hopFrom(std::size_t size, double overlap, gr::Size_t hopSetting) noexcept { return hopSetting == 0U ? hopFor(size, overlap) : static_cast<std::size_t>(hopSetting); }

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

/// @brief The shape parameter the window is built with. The setting supplies it where it names one. The window's
/// own default applies otherwise. A value outside what the window accepts is refused with the window named.
///
/// Zero and NaN both select the default. A graph description that leaves the key out, one that writes 0 and one that
/// writes a NaN then mean the same. A window whose parameter is legitimately zero is named by its own type instead.
/// Kaiser beta 0, for example, is the rectangular window. The default is read from the window library and is not
/// repeated here. The record states the parameter in force, and a second copy of the number could go stale.
[[nodiscard]] inline float windowParamFor(gr::algorithm::window::Type windowType, float param) {
    const float shape = (param == 0.f || std::isnan(param)) ? gr::algorithm::window::detail::defaultParameter<float>(windowType) : param;
    try {
        gr::algorithm::window::detail::validateParameter<float>(windowType, shape);
    } catch (const std::invalid_argument& refusal) {
        throw gr::exception(std::format("window_param: {}", refusal.what()));
    }
    return shape;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::WelchPsd, [T], [ float, std::complex<float> ])

/**
 * @brief Welch's averaged periodogram, one calibrated density record per `n_averages` windowed segments.
 *
 * The block accumulates overlapping windowed segments in power. The estimate trades variance against frequency
 * resolution, as in Welch's method. `n_averages` segments reduce the estimator's variance by about that factor. The
 * resolution stays the transform's own. `mode` selects what the accumulation means. `mean` gives the estimate.
 * `max_hold` keeps the largest density each bin has reached. It finds an intermittent emitter, where `mean`
 * describes a stationary one.
 *
 * Record values are linear power per hertz referred to a full-scale sine, so `10*log10(value)` is dBFS/Hz. To read a
 * tone's own power and not a noise density, a consumer multiplies a peak density by the record's `enbw_bins` and the
 * bin width.
 *
 * `hop` states in samples how far the grid advances between segments. At zero the hop follows `overlap`. Above
 * `fft_size` the block runs on a duty cycle and is not a continuous estimator. The samples between one segment and the
 * next are consumed and are not copied or transformed. The cost falls with the duty, while `sample_start` keeps
 * counting the whole stream. A display's cost then follows its frame rate and not its sample rate.
 *
 * `window_param` is the shape parameter of the parameterized windows. It is Kaiser beta, Tukey alpha, Gaussian sigma
 * or Exponential decay in decibels. Other windows ignore it. Zero or NaN selects the window library's own default.
 * The parameter sets the sidelobe level and with it the noise bandwidth. The record states the value in force beside
 * the window's name, because "Kaiser" alone does not say which Kaiser.
 *
 * Every setting is live. `window`, `window_param`, `overlap`, `hop`, `n_averages`, `mode` and `sample_rate` restart the
 * accumulation in progress. Half a spectrum under one window and half under another is not an estimate. The restart
 * keeps the stream position and every buffered sample. `fft_size` does the same and also rebuilds the transform, the
 * window and the buffers. A partial average is dropped and not emitted. The old grid's positions are not positions of
 * the new one. The new grid is anchored at the current stream position, and the record's `grid_start` states where. A
 * consumer that ignores `grid_start` still reads every record correctly, because `sample_start` and `fft_size` are
 * per-record facts. A consumer that stacks records on a common grid needs `grid_start`.
 *
 * A record states the segment count that went into it. A stream that ends mid-accumulation flushes what it has,
 * marked with that count. The block does not discard the partial record or pad it to look complete.
 *
 * Two settings do not affect the accumulation. A change to either alone leaves the average in progress standing.
 * `threads` sets how many threads one transform may use, clamped to the machine. At 1 the transform runs on the
 * calling thread. Above 1 the transform splits a long power-of-two length over the threads and produces the same
 * record a single thread produces. The threads cost while a transform runs and nothing between transforms. Raising
 * `threads` helps only where the transform dominates the block's cost, at a long length with a hop that keeps the
 * machine busy. `center_frequency` states in hertz where the zero of the record's baseband axis sits. The record
 * carries it beside `sample_rate`, and a consumer places the record on an absolute axis without parsing a name. A
 * receiver may retune mid-record and keep the record.
 *
 * Every record states its computing cost in wall-clock seconds as `compute_seconds`, without the wait for input. A
 * consumer that paces itself by the block's cost reads this figure and not the stream rate.
 *
 * `one_record_per_call` holds the input span to the effective hop, the least stream advance one record needs. A work
 * call then emits at most one record, and a record consumer receives records one at a time and not in bursts. The
 * setting is on by default and costs one scheduler visit per record. That cost is worth paying where a record costs
 * more than a visit, and not where the hop is a few hundred samples. With the setting off, a call emits as many
 * records as its span and the output allow, with fewer scheduler visits at a small hop.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct WelchPsd : Block<WelchPsd<T>, NoTagPropagation> {
    using Description = Doc<"Computes Welch's averaged power spectral density. Overlapping windowed segments are accumulated in power, and each calibrated DataSet record averages n_averages segments. Values are linear power per hertz referred to a full-scale sine. A hop above fft_size transforms one segment in every hop samples and consumes the rest untouched. Every setting is live. A change to fft_size rebuilds the transform and re-anchors the grid, and the record's grid_start states the anchor. A change to a setting the accumulation depends on restarts the accumulation in progress. Every record states its computing cost in wall-clock seconds as compute_seconds. one_record_per_call holds the input span to one hop, and a work call then emits at most one record.">;
    using Real        = float;

    PortIn<T>                     in;
    PortOut<DataSet<Real>, Async> out;

    Annotated<gr::Size_t, "fft_size", Visible, Doc<"transform length, a power of two in [64, 4194304]">>                        fft_size            = 1024U;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                            window              = std::string("Hann");
    Annotated<float, "window_param", Visible, Doc<"window shape parameter, 0 or NaN for the window's default">>                 window_param        = 0.f;
    Annotated<double, "overlap", Visible, Doc<"fraction of a segment shared with the next, in [0, 1)">>                         overlap             = 0.5;
    Annotated<gr::Size_t, "hop", Visible, Unit<"samples">, Doc<"samples the grid advances per transform, 0 to follow overlap">> hop                 = 0U;
    Annotated<gr::Size_t, "n_averages", Visible, Doc<"segments per emitted record, 1 for a bare periodogram">>                  n_averages          = 16U;
    Annotated<std::string, "mode", Visible, Doc<"mean or max_hold">>                                                            mode                = std::string("mean");
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate setting the record's axis scale">>              sample_rate         = 1.f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                              signal_name         = std::string("psd");
    Annotated<double, "center_frequency", Visible, Unit<"Hz">, Doc<"center frequency of the record's baseband axis">>           center_frequency    = 0.;
    Annotated<gr::Size_t, "threads", Visible, Doc<"threads one transform may use, 1 for the calling thread">>                   threads             = 1U;
    Annotated<bool, "one_record_per_call", Visible, Doc<"emit at most one record per work call">>                               one_record_per_call = true;

    GR_MAKE_REFLECTABLE(WelchPsd, in, out, fft_size, window, window_param, overlap, hop, n_averages, mode, sample_rate, signal_name, threads, center_frequency, one_record_per_call);

    detail::SegmentAccumulator<T> _core{};
    bool                          _flushed      = false;
    gr::Size_t                    _builtFftSize = 0U; ///< the length the current transform and window were built for

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        // `threads` changes what a transform costs and not what it computes. It is applied on its own path and is left
        // out of the rebuild keys below. A change to it must not restart an average in progress.
        if (threads.value == 0U) {
            throw gr::exception("threads counts the threads one transform may use and must be at least 1");
        }
        _core.setThreads(static_cast<std::size_t>(threads.value));

        // `center_frequency` goes into the record, and nothing else reads it. Like `signal_name`, it is not a rebuild
        // key. A receiver retuning mid-stream must not lose the estimate it is part way through.
        if (!std::isfinite(center_frequency.value)) {
            throw gr::exception(std::format("center_frequency must be a finite number of hertz, got {}", center_frequency.value));
        }

        // Only these keys change the accumulator. `signal_name` alone must not restart an average in progress.
        static constexpr std::array kRebuildKeys{"fft_size", "window", "window_param", "overlap", "hop", "n_averages", "mode", "sample_rate"};
        if (_core.window.empty() || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            buildCore();
        }
        capInput();
    }

    /// @brief Fits the accumulator to the members. A first build configures, a new length rebuilds, and any other
    /// change re-fits. The result depends on the members and not on a key's presence, so `start()` can run it.
    void buildCore() {
        const bool built = !_core.window.empty();
        detail::requireFftSize(fft_size);
        detail::requireOverlap(overlap);
        detail::requireSampleRate(sample_rate);
        if (n_averages.value == 0U) {
            throw gr::exception("n_averages counts the segments an estimate is made of and must be at least 1");
        }
        if (mode.value != "mean" && mode.value != "max_hold") {
            throw gr::exception(std::format("mode must be 'mean' or 'max_hold', got '{}'", mode.value));
        }
        const auto windowType  = detail::requireWindow(window.value);
        const auto windowShape = detail::windowParamFor(windowType, window_param);

        const std::size_t size       = static_cast<std::size_t>(fft_size.value);
        const std::size_t hopSamples = detail::hopFrom(size, overlap, hop);
        const bool        holdMode   = mode.value == "max_hold";
        if (!built) {
            _core.configure(size, hopSamples, static_cast<std::size_t>(n_averages.value), holdMode, windowType, windowShape, sample_rate);
            _flushed = false;
        } else if (fft_size.value != _builtFftSize) {
            // A new transform length under a running stream. The transform, the window and the buffers are rebuilt at
            // the next segment boundary, where the block stands between calls. The accumulation in progress is dropped
            // and not finished at two resolutions. `pending` and the stream position survive, and no sample is lost.
            // The record's `grid_start` states where the new grid was anchored.
            _core.rebuild(size, hopSamples, static_cast<std::size_t>(n_averages.value), holdMode, windowType, windowShape, sample_rate);
        } else {
            _core.reconfigure(hopSamples, static_cast<std::size_t>(n_averages.value), holdMode, windowType, windowShape, sample_rate);
        }
        _builtFftSize = fft_size;
    }

    /// @brief Holds the input span to the effective hop, the least stream advance one record needs. A work call then
    /// folds at most one record. Uncapped, the port takes whatever the edge offers.
    void capInput() { in.max_samples = one_record_per_call ? std::max(_core.hop, 2UZ) : std::numeric_limits<std::size_t>::max(); }

    void start() {
        if (_core.window.empty()) { // a construction that matched every default called no settingsChanged()
            buildCore();
        }
        _core.reset();
        _flushed = false;
        capInput();
        // The framework runs `processEpilogue` only over a non-empty trailing span. That epilogue flushes a partial
        // accumulation at end of stream. `processBulk` leaves one sample unconsumed on every call. Asking for two
        // samples keeps that from stalling the steady state.
        in.min_samples = 2UZ;
    }

    /**
     * @brief Keeps the no-record-lost invariant.
     *
     * A sample is consumed only once it is in the accumulator. A segment is folded only while the output span has room
     * for the record it might complete. When the output fills, the call publishes what it made and consumes what it
     * took. A slow consumer then stalls the input, and no backlog grows inside this block. `pending` holds at most one
     * segment.
     *
     * A call that moves nothing reports why. `in.min_samples` does not stop the framework from calling the block with
     * an empty span. The framework calls the block without input because the output is asynchronous. Those calls let
     * the accumulation flush. A call that moves nothing returns `INSUFFICIENT_OUTPUT_ITEMS` when the output is full and
     * `INSUFFICIENT_INPUT_ITEMS` otherwise. An `OK` would report progress and leave the scheduler spinning on the
     * block.
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
        return progress.made == 0UZ && progress.taken == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::OK;
    }

    /// @brief At end of stream, folds in the trailing samples and emits the accumulation in progress. The record
    /// states the segment count that reached it. The framework consumes the trailing span itself.
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
            if (_core.skipping > 0UZ) {
                // The gap a hop above `fftSize` leaves. It is consumed through the port and counted into the stream
                // position, and it is not copied. This is the whole saving of a duty cycle.
                const std::size_t drop = std::min(_core.skipping, offer - progress.taken);
                if (drop == 0UZ) {
                    return progress;
                }
                _core.skipping -= drop;
                _core.streamAt += drop;
                progress.taken += drop;
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
        const auto          assembly = std::chrono::steady_clock::now();
        const double        folded   = _core.costSeconds;
        const std::size_t   averaged = _core.segments;
        const std::uint64_t startAt  = _core.recordStartAt;

        DataSet<Real> record = detail::makeSpectralRecord<Real>(_core.take(), sample_rate, center_frequency, _core.fftSize, detail::SegmentAccumulator<T>::kRealInput, startAt, averaged, _core.hop, _core.scale.enbwBins, window.value, _core.shape, _core.gridStart, signal_name.value);
        detail::stateComputeSeconds(record, folded + detail::secondsSince(assembly));
        return record;
    }
};

GR_REGISTER_BLOCK(gr::blocks::measurement::Spectrogram, [T], [ float, std::complex<float> ])

/**
 * @brief One calibrated density record per windowed transform, at hop `fft_size * (1 - overlap)`.
 *
 * The block runs the machinery of `WelchPsd` without the averaging. It has its own registration because its output
 * contract differs. It emits a stream of rows in time, not a settled estimate. Each record's `sample_start` is the
 * absolute index of the hop it was taken from. A consumer stacks rows on a real time axis and not in arrival order.
 * Values are linear power per hertz referred to a full-scale sine, as in `WelchPsd`.
 *
 * `hop` states the advance in samples, and at zero it follows `overlap`. Above `fft_size` it leaves a gap between
 * rows. A waterfall then sets its own row rate and does not follow the sample rate. The samples between rows are
 * consumed and are not copied or transformed. `sample_start` keeps counting the whole stream, and the rows stay on a
 * true time axis.
 *
 * `window_param` is the shape parameter of the parameterized windows, as in `WelchPsd`. The record carries the value
 * in force.
 *
 * Every setting is live, `fft_size` included. A change to `fft_size` rebuilds the transform, the window and the
 * buffers. It re-anchors the grid at the current stream position, and the record's `grid_start` states the anchor. A
 * row is one whole transform. No partial record is lost across the change, and none is flushed at the end. The stream
 * stops between rows.
 *
 * `threads` sets how many threads one transform may use, as in `WelchPsd`. The rows equal the rows a single thread
 * produces. The setting re-applies without disturbing the grid or the buffered samples. `center_frequency` states in
 * hertz where the zero of the baseband axis sits. The record carries it beside `sample_rate`. Neither setting affects
 * a row, and neither re-anchors the grid.
 *
 * A row states its computing cost as `compute_seconds`, defined as for `WelchPsd`. `one_record_per_call` holds the
 * input span to one hop on the same terms. A work call then emits at most one row.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Spectrogram : Block<Spectrogram<T>, NoTagPropagation> {
    using Description = Doc<"Computes a spectrogram, one calibrated power spectral density record per windowed transform. Each record is timestamped by the hop it came from. Values are linear power per hertz referred to a full-scale sine. A hop above fft_size transforms one row in every hop samples and consumes the rest untouched. Every setting is live. A change to fft_size rebuilds the transform and re-anchors the grid, and the record's grid_start states the anchor. Every record states its computing cost in wall-clock seconds as compute_seconds. one_record_per_call holds the input span to one hop, and a work call then emits at most one record.">;
    using Real        = float;

    PortIn<T>                     in;
    PortOut<DataSet<Real>, Async> out;

    Annotated<gr::Size_t, "fft_size", Visible, Doc<"transform length, a power of two in [64, 4194304]">>                  fft_size            = 1024U;
    Annotated<std::string, "window", Visible, Doc<gr::algorithm::window::TypeNames>>                                      window              = std::string("Hann");
    Annotated<float, "window_param", Visible, Doc<"window shape parameter, 0 or NaN for the window's default">>           window_param        = 0.f;
    Annotated<double, "overlap", Visible, Doc<"fraction of a segment shared with the next, in [0, 1)">>                   overlap             = 0.5;
    Annotated<gr::Size_t, "hop", Visible, Unit<"samples">, Doc<"samples the grid advances per row, 0 to follow overlap">> hop                 = 0U;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate setting the record's axis scale">>        sample_rate         = 1.f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                        signal_name         = std::string("spectrogram");
    Annotated<double, "center_frequency", Visible, Unit<"Hz">, Doc<"center frequency of the record's baseband axis">>     center_frequency    = 0.;
    Annotated<gr::Size_t, "threads", Visible, Doc<"threads one transform may use, 1 for the calling thread">>             threads             = 1U;
    Annotated<bool, "one_record_per_call", Visible, Doc<"emit at most one row per work call">>                            one_record_per_call = true;

    GR_MAKE_REFLECTABLE(Spectrogram, in, out, fft_size, window, window_param, overlap, hop, sample_rate, signal_name, threads, center_frequency, one_record_per_call);

    detail::SegmentAccumulator<T> _core{};
    gr::Size_t                    _builtFftSize = 0U; ///< the length the current transform and window were built for

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        // As in WelchPsd, `threads` is applied on its own path and is left out of the rebuild keys.
        if (threads.value == 0U) {
            throw gr::exception("threads counts the threads one transform may use and must be at least 1");
        }
        _core.setThreads(static_cast<std::size_t>(threads.value));

        // As in WelchPsd, `center_frequency` is not a rebuild key.
        if (!std::isfinite(center_frequency.value)) {
            throw gr::exception(std::format("center_frequency must be a finite number of hertz, got {}", center_frequency.value));
        }

        static constexpr std::array kRebuildKeys{"fft_size", "window", "window_param", "overlap", "hop", "sample_rate"};
        if (_core.window.empty() || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            buildCore();
        }
        capInput();
    }

    /// @brief Fits the accumulator to the members, as `WelchPsd::buildCore` does, so `start()` can run it.
    void buildCore() {
        const bool built = !_core.window.empty();
        detail::requireFftSize(fft_size);
        detail::requireOverlap(overlap);
        detail::requireSampleRate(sample_rate);
        const auto windowType  = detail::requireWindow(window.value);
        const auto windowShape = detail::windowParamFor(windowType, window_param);

        const std::size_t size       = static_cast<std::size_t>(fft_size.value);
        const std::size_t hopSamples = detail::hopFrom(size, overlap, hop);
        if (!built) {
            _core.configure(size, hopSamples, 1UZ, false, windowType, windowShape, sample_rate);
        } else if (fft_size.value != _builtFftSize) {
            // A row is one whole transform, and no partial estimate is lost here. A live length change moves the grid
            // anchor, which the record states as `grid_start`.
            _core.rebuild(size, hopSamples, 1UZ, false, windowType, windowShape, sample_rate);
        } else {
            _core.reconfigure(hopSamples, 1UZ, false, windowType, windowShape, sample_rate);
        }
        _builtFftSize = fft_size;
    }

    /// @brief Holds the input span to the effective hop, as `WelchPsd` does. A row is one hop of stream, and a capped
    /// work call folds at most one row.
    void capInput() { in.max_samples = one_record_per_call ? std::max(_core.hop, 2UZ) : std::numeric_limits<std::size_t>::max(); }

    void start() {
        if (_core.window.empty()) { // as in WelchPsd
            buildCore();
        }
        _core.reset();
        capInput();
        in.min_samples = 2UZ; // as in WelchPsd, a sample is held back for the end-of-stream epilogue
    }

    /// @brief Keeps the no-record-lost invariant of `WelchPsd`, where every folded segment completes a record. A call
    /// that moves nothing reports why, as in `WelchPsd`.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = fold(inSpan, inSpan.size() > 0UZ ? inSpan.size() - 1UZ : 0UZ, outSpan);
        if (progress.outputFull && progress.made == 0UZ && progress.taken == 0UZ) {
            outSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan.publish(progress.made);
        std::ignore = inSpan.consume(progress.taken);
        return progress.made == 0UZ && progress.taken == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::OK;
    }

    /// @brief At end of stream, folds the whole segments the trailing samples still complete.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const detail::Progress progress = fold(inSpan, inSpan.size(), outSpan);
        outSpan.publish(progress.made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] DataSet<Real> emit(std::uint64_t startAt) {
        const auto   assembly = std::chrono::steady_clock::now();
        const double folded   = _core.costSeconds;

        DataSet<Real> record = detail::makeSpectralRecord<Real>(_core.take(), sample_rate, center_frequency, _core.fftSize, detail::SegmentAccumulator<T>::kRealInput, startAt, 1UZ, _core.hop, _core.scale.enbwBins, window.value, _core.shape, _core.gridStart, signal_name.value);
        detail::stateComputeSeconds(record, folded + detail::secondsSince(assembly));
        return record;
    }

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
                outSpan[progress.made] = emit(startAt);
                ++progress.made;
                continue;
            }
            if (_core.skipping > 0UZ) {
                // The gap a hop above `fftSize` leaves between rows. It is consumed and counted into the stream
                // position, and it is not copied. A row's `sample_start` still names its true place in the stream.
                const std::size_t drop = std::min(_core.skipping, offer - progress.taken);
                if (drop == 0UZ) {
                    return progress;
                }
                _core.skipping -= drop;
                _core.streamAt += drop;
                progress.taken += drop;
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
