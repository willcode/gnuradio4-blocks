#ifndef GNURADIO_POWER_METER_HPP
#define GNURADIO_POWER_METER_HPP

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>

#include <gnuradio-4.0/analog/NamespaceCompatibility.hpp>

namespace gr::blocks::analog {

GR_REGISTER_BLOCK(gr::blocks::analog::PowerMeter, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct PowerMeter : Block<PowerMeter<T>> {
    using Description = Doc<R""(
@brief An S-meter: the mean power of the last `window_time` seconds, in dBFS, readable from another thread.

A sink. 0 dBFS is a mean power of exactly 1.0, and the window is a boxcar rather than a pole. `level()` is that
reading in dBFS, `linear_power()` the same reading without the logarithm, and `coverage()` the fraction of the nominal
window accumulated so far - a partial window is reported rather than suppressed.

The optional `records` port carries the same reading as one `DataSet<float>` per completed window, stamped with the
window's first input sample, for a consumer outside C++; a stream that ends mid-window emits a final record covering
what it had.
)"">;

    PortIn<T> in;
    /// One record per completed window, for a consumer outside C++. The port is optional: leaving it unconnected
    /// costs nothing and the readers below remain the whole interface for a graph that polls.
    PortOut<DataSet<float>, Async, Optional> records;

    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate; a change rebuilds the window">>                   sample_rate     = 96000.f;
    Annotated<double, "window_time", Unit<"s">, Doc<"length of the rolling average; a change rebuilds the window">> window_time     = 0.100;
    Annotated<gr::Size_t, "segments", Doc<"pieces the window is cut into; the reading refreshes once per piece">>   segments        = 16U;
    Annotated<double, "floor_db", Unit<"dBFS">, Doc<"what level() returns at or below 10^(floor_db/10)">>           floor_db        = -200.0;
    Annotated<gr::Size_t, "segment_samples", Doc<"observable: the realized segment length, at least one sample">>   segment_samples = 600U;
    Annotated<gr::Size_t, "window_samples", Doc<"observable: segments * segment_samples, the realized window">>     window_samples  = 9600U;

    GR_MAKE_REFLECTABLE(PowerMeter, in, records, sample_rate, window_time, segments, floor_db, segment_samples, window_samples);

    std::array<double, 8UZ> _lanes{};
    std::vector<double>     _segments{};
    std::size_t             _segmentSamples = 600UZ;
    std::size_t             _index          = 0UZ; ///< position within the current segment
    std::size_t             _completed      = 0UZ;

    // Everything a reader touches is atomic, so a settings change beside a poll is not a race.
    gr::measurement::MeasurementSlot<1UZ> _slot{};

    std::vector<DataSet<float>> _pending{};             ///< records built at a window's close and not yet published
    std::size_t                 _sinceRecord   = 0UZ;   ///< segments closed since the last record was published
    std::uint64_t               _streamAt      = 0ULL;  ///< absolute index of the next input sample
    std::uint64_t               _windowStartAt = 0ULL;  ///< absolute index of the first sample of the window now filling
    bool                        _flushed       = false; ///< the end-of-stream record has gone out
    std::atomic<std::uint32_t>  _windowSegments{16U};
    std::atomic<double>         _floorLinear{1e-20};
    std::atomic<double>         _floorDb{-200.0};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        _floorDb.store(floor_db, std::memory_order_relaxed);
        _floorLinear.store(std::pow(10.0, floor_db / 10.0), std::memory_order_relaxed);

        static constexpr std::array kRebuildKeys{"sample_rate", "window_time", "segments"};
        if (!_segments.empty() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return; // floor_db alone changes the clamp and nothing else
        }
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!(window_time > 0.0) || !std::isfinite(window_time)) {
            throw gr::exception(std::format("window_time must be positive and finite, got {}", window_time.value));
        }
        if (segments < 1U) {
            throw gr::exception("segments must be at least one");
        }

        _segmentSamples = std::max(1UZ, static_cast<std::size_t>(std::llround(static_cast<double>(sample_rate) * window_time / static_cast<double>(segments.value))));
        segment_samples = static_cast<gr::Size_t>(_segmentSamples);
        window_samples  = static_cast<gr::Size_t>(_segmentSamples * static_cast<std::size_t>(segments.value));
        _segments.assign(static_cast<std::size_t>(segments.value), 0.0);
        _windowSegments.store(static_cast<std::uint32_t>(segments.value), std::memory_order_relaxed);
        reset();
    }

    void start() {
        reset();
        // The framework runs `processEpilogue` only over a non-empty trailing span, and that epilogue is what emits
        // the final partial window. `processBulk` therefore leaves the last sample of a call unconsumed, and asking
        // for two keeps that from stalling the steady state.
        in.min_samples = 2UZ;
    }

    /// @brief Zero the window. Not thread-safe against a running scheduler: for the owning thread between stop() and start().
    void reset() {
        _lanes.fill(0.0);
        std::ranges::fill(_segments, 0.0);
        _index         = 0UZ;
        _completed     = 0ULL;
        _sinceRecord   = 0UZ;
        _streamAt      = 0ULL;
        _windowStartAt = 0ULL;
        _flushed       = false;
        _pending.clear();
        publish(0.0, 0U);
    }

    /// @brief Mean power over the accumulated window, in dBFS, clamped at floor_db. Callable from any thread.
    [[nodiscard]] float level() const noexcept {
        const auto [power, filled] = read();
        return clampedLevel(power, filled);
    }

    /// @brief Fraction of the nominal window the reading covers, 0 to 1. Callable from any thread.
    [[nodiscard]] float coverage() const noexcept { return coverageOf(read().second); }

    /// @brief The same reading without the logarithm, for a caller that wants a ratio rather than decibels.
    [[nodiscard]] double linear_power() const noexcept { return read().first; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input(inSpan);
        std::size_t              made = drain(outSpan, 0UZ);

        // The last sample of a call is held back so the end-of-stream epilogue always has a span to run on. A call
        // carrying a single sample is one a caller drove by hand rather than one the framework composed, and takes it.
        const std::size_t offer = input.size() >= 2UZ ? input.size() - 1UZ : input.size();
        const std::size_t take  = std::min(offer, roomFor(outSpan, made));

        accumulate(input.first(take), outSpan.isConnected);
        made += drain(outSpan, made);

        outSpan.publish(made);
        std::ignore = inSpan.consume(take);
        return take == 0UZ && made == 0UZ && !input.empty() ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief End of stream: fold the trailing samples, then emit what has accumulated since the last record. That
    /// span is shorter than a window, and `sample_start` with the stream's end is what delimits it.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        accumulate(std::span<const T>(inSpan), outSpan.isConnected);
        std::size_t made = drain(outSpan, 0UZ);

        if (!_flushed && outSpan.isConnected && _completed > 0UZ && _streamAt > _windowStartAt && made < outSpan.size()) {
            const auto [power, filled] = read();
            outSpan[made]              = makeRecord(power, filled);
            ++made;
            _flushed = true;
        }
        outSpan.publish(made);
        return work::Status::OK;
    }

    /// @brief The reading as a record, on the tier's measurement conventions: the power the window closed at, in the
    /// clamped form `level()` returns, beside the raw ratio and the coverage.
    [[nodiscard]] DataSet<float> makeRecord(double power, std::uint32_t filled) const {
        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"power", "Power", "dBFS", clampedLevel(power, filled)},
            {"power_linear", "Power", "1", static_cast<float>(power)},
            {"coverage", "Coverage", "1", coverageOf(filled)},
        }};
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), sample_rate, _windowStartAt, property_map{{std::pmr::string("n_segments"), pmt::Value(static_cast<std::uint64_t>(filled))}, {std::pmr::string("window_samples"), pmt::Value(static_cast<std::uint64_t>(window_samples.value))}});
    }

private:
    [[nodiscard]] static constexpr double squaredMagnitude(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return static_cast<double>(sample) * static_cast<double>(sample);
        } else {
            return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
    }

    /// @brief The reading in dBFS with the floor applied, which is the one form `level()` and a record both state.
    [[nodiscard]] float clampedLevel(double power, std::uint32_t filled) const noexcept {
        if (filled == 0U || power <= _floorLinear.load(std::memory_order_relaxed)) {
            return static_cast<float>(_floorDb.load(std::memory_order_relaxed));
        }
        return static_cast<float>(10.0 * std::log10(power));
    }

    [[nodiscard]] float coverageOf(std::uint32_t filled) const noexcept { return std::min(1.f, static_cast<float>(filled) / static_cast<float>(_windowSegments.load(std::memory_order_relaxed))); }

    /// @brief How many input samples may be taken before a record would have nowhere to go. A record closes every
    /// `window_samples` samples, so it is that many per free output slot, less what the current window already holds.
    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& outSpan, std::size_t made) const {
        if (!outSpan.isConnected) {
            return std::numeric_limits<std::size_t>::max();
        }
        const std::size_t free          = outSpan.size() > made + _pending.size() ? outSpan.size() - made - _pending.size() : 0UZ;
        const std::size_t windowSamples = _segments.size() * _segmentSamples;
        const std::size_t into          = _sinceRecord * _segmentSamples + _index;
        const std::size_t untilClose    = windowSamples > into ? windowSamples - into : 0UZ;
        if (free == 0UZ) {
            return untilClose > 0UZ ? untilClose - 1UZ : 0UZ;
        }
        return untilClose + (free - 1UZ) * windowSamples;
    }

    /// @brief Move what has been built into the output span from @p at, and report how many went.
    [[nodiscard]] std::size_t drain(OutputSpanLike auto& outSpan, std::size_t at) {
        if (_pending.empty() || !outSpan.isConnected || at >= outSpan.size()) {
            return 0UZ;
        }
        const std::size_t made = std::min(_pending.size(), outSpan.size() - at);
        for (std::size_t k = 0UZ; k < made; ++k) {
            outSpan[at + k] = std::move(_pending[k]);
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        return made;
    }

    void accumulate(std::span<const T> input, bool wantRecords) {
        std::array<double, 8UZ> lanes = _lanes;

        for (std::size_t at = 0UZ; at < input.size();) {
            const std::size_t run = std::min(input.size() - at, _segmentSamples - _index);
            std::size_t       k   = 0UZ;

            for (; k < run && ((_index + k) & 7UZ) != 0UZ; ++k) { // reach a lane boundary, so the body below is a fixed eight
                lanes[(_index + k) & 7UZ] += squaredMagnitude(input[at + k]);
            }
            for (; k + 8UZ <= run; k += 8UZ) {
                for (std::size_t lane = 0UZ; lane < 8UZ; ++lane) {
                    lanes[lane] += squaredMagnitude(input[at + k + lane]);
                }
            }
            for (; k < run; ++k) {
                lanes[(_index + k) & 7UZ] += squaredMagnitude(input[at + k]);
            }

            _index += run;
            at += run;
            _streamAt += run;
            if (_index == _segmentSamples) {
                closeSegment(lanes, wantRecords);
            }
        }

        _lanes = lanes;
    }

    void closeSegment(std::array<double, 8UZ>& lanes, bool wantRecords) {
        double total = 0.0;
        for (const double lane : lanes) { // index order, fixed, so the answer is a function of the input alone
            total += lane;
        }
        lanes.fill(0.0);
        _index = 0UZ;

        _segments[_completed % _segments.size()] = total / static_cast<double>(_segmentSamples);
        ++_completed;

        const std::size_t filled = std::min(_completed, _segments.size());
        double            mean   = 0.0;
        for (std::size_t slot = 0UZ; slot < filled; ++slot) { // slot order, fixed for the same reason
            mean += _segments[slot];
        }
        mean /= static_cast<double>(filled);
        publish(mean, static_cast<std::uint32_t>(filled));

        // A record is one completed window, so the rate is one per `window_samples` rather than one per segment, and
        // the record states the reading of the window that closed rather than whatever the next chunk goes on to make.
        if (++_sinceRecord >= _segments.size()) {
            _sinceRecord = 0UZ;
            if (wantRecords) {
                _pending.push_back(makeRecord(mean, static_cast<std::uint32_t>(filled)));
            }
            _windowStartAt = _streamAt;
        }
    }

    /// The one reading and the segments behind it, carried across the thread boundary by the seqlock.
    void publish(double power, std::uint32_t filled) noexcept { _slot.publish({power}, static_cast<std::uint64_t>(filled)); }

    [[nodiscard]] std::pair<double, std::uint32_t> read() const noexcept {
        const auto [values, filled] = _slot.read();
        return {values[0], static_cast<std::uint32_t>(filled)};
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_POWER_METER_HPP
