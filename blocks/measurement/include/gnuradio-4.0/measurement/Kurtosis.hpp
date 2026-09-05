#ifndef GNURADIO_MEASUREMENT_KURTOSIS_HPP
#define GNURADIO_MEASUREMENT_KURTOSIS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>

namespace gr::blocks::measurement {

namespace detail {
inline constexpr gr::Size_t kMinKurtosisWindow = 8U;
inline constexpr gr::Size_t kMaxKurtosisWindow = 1U << 24U;
} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::Kurtosis, [T], [ float, std::complex<float> ])

/**
 * @brief The normalized fourth moment over a stream-absolute window, as excess kurtosis and its un-shifted form.
 *
 * A sink with a record port, not a 1:1 block: the stream is consumed and nothing passed through. Each window's
 * `gr::measurement::KurtosisAccumulator<T>` figures are published to a pollable slot at the window's close, and one
 * `DataSet<float>` record is emitted when `emit_records` is set. `input_domain` rides in the record because the two
 * domains have different Gaussian references (2 complex, 3 real) and a reader of `excess_kurtosis` alone cannot tell
 * which reference applies.
 *
 * A window whose accumulated `sum|x|^2` is exactly zero is degenerate: both kurtosis channels are reported as `0.f`
 * and `valid` is written false, counted in `nDegenerateWindows()`, rather than a NaN a consumer would have to test
 * for. `window` below eight samples is refused: at that floor the estimator's own spread already exceeds the range
 * of values it is meant to resolve, so a shorter window is not a measurement.
 */
template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct Kurtosis : Block<Kurtosis<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Excess kurtosis and the un-shifted normalized fourth moment over a stream-absolute window.

A sink. Every `window` samples the block reports `excess_kurtosis` (0 for a Gaussian of the input's own domain),
`normalized_fourth_moment` (`m4/m2^2` itself) and `mean_power`, both through a pollable slot and, when `emit_records`,
one `DataSet<float>` record. An all-zero window is reported with `valid = false` rather than a NaN.
)"">;

    static constexpr bool kComplexDomain = std::same_as<T, std::complex<float>>;

    PortIn<T>                                in;
    PortOut<DataSet<float>, Async, Optional> measurements;

    Annotated<gr::Size_t, "window", Visible, Doc<"samples per reading; below 8 or above 2^24 is refused; a change restarts the window">> window       = 4096U;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate stated in every record; must be positive and finite">>                  sample_rate  = 96000.f;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per completed window">>                              emit_records = true;
    Annotated<std::string, "signal_name", Doc<"producer label carried in the record's extra metadata">>                                  signal_name  = std::string("kurtosis");

    GR_MAKE_REFLECTABLE(Kurtosis, in, measurements, window, sample_rate, emit_records, signal_name);

    gr::measurement::KurtosisAccumulator<T> _acc{};
    gr::measurement::MeasurementSlot<3UZ>   _slot{}; ///< excess_kurtosis, normalized_fourth_moment, mean_power

    /// @brief One `n_dropped_samples` tag: where it sits on the stream, and how many samples it says went missing.
    struct DroppedTag {
        std::uint64_t at;
        std::uint64_t count;
    };

    std::atomic<std::uint64_t> _nWindows{0ULL};
    std::atomic<std::uint64_t> _nDegenerateWindows{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nDroppedSampleTags{0ULL};

    std::uint64_t               _streamAt{0ULL};        ///< absolute index of the next input sample
    std::uint64_t               _windowStartAt{0ULL};   ///< absolute index of the first sample of the window now filling
    bool                        _flushed{false};        ///< the end-of-stream record has gone out
    bool                        _configured{false};     ///< a later settings change is a change rather than the first configuration
    std::vector<DroppedTag>     _droppedTags{};         ///< the dropped-sample tags sitting on the samples this call takes
    std::uint64_t               _droppedInWindow{0ULL}; ///< their summed count over the window now filling
    std::vector<DataSet<float>> _pending{};             ///< records built at a window's close and not yet published

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (window.value < detail::kMinKurtosisWindow || window.value > detail::kMaxKurtosisWindow) {
            throw gr::exception(std::format("window must lie in [{}, {}], got {}", detail::kMinKurtosisWindow, detail::kMaxKurtosisWindow, window.value));
        }
        if (!std::isfinite(sample_rate.value) || !(sample_rate.value > 0.f)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (_configured && newSettings.contains("window")) {
            if (_acc.count() > 0UZ) {
                _nWindowResets.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _acc.reset();
            _windowStartAt = _streamAt;
        }
        _configured = true;
    }

    void start() {
        _acc.reset();
        _nWindows.store(0ULL, std::memory_order_relaxed);
        _nDegenerateWindows.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nDroppedSampleTags.store(0ULL, std::memory_order_relaxed);
        _streamAt        = 0ULL;
        _windowStartAt   = 0ULL;
        _flushed         = false;
        _droppedInWindow = 0ULL;
        _droppedTags.clear();
        _pending.clear();
        _slot.publish({0., 0., 0.}, 0ULL);
        // A block built entirely from defaults stages nothing, so `settingsChanged` has not necessarily run by now.
        // From here on every settings change is a change to a running block, and the window reset rule applies to it.
        _configured = true;
        // The framework runs `processEpilogue` only over a non-empty trailing span, and that epilogue is what flushes
        // a partial window at end of stream. `processBulk` therefore leaves the last sample of a call unconsumed, and
        // asking for two keeps that from stalling the steady state.
        in.min_samples = 2UZ;
    }

    /// @brief Excess kurtosis of the last completed (or flushed) window. Callable from any thread.
    [[nodiscard]] double excessKurtosis() const noexcept { return _slot.read().first[0]; }
    /// @brief `m4/m2^2` of the same window, un-shifted. Callable from any thread.
    [[nodiscard]] double normalizedFourthMoment() const noexcept { return _slot.read().first[1]; }
    /// @brief Mean power of the same window, against the same full-scale reference the tier's spectral records use.
    [[nodiscard]] double meanPower() const noexcept { return _slot.read().first[2]; }
    /// @brief Fraction of `window` the last reading covers, 0 before the first window closes. Any thread.
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_slot.read().second) / static_cast<double>(std::max(window.value, 1U))); }

    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _nWindows.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDegenerateWindows() const noexcept { return _nDegenerateWindows.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }
    /// @brief `n_dropped_samples` tags seen on the input stream. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSampleTags() const noexcept { return _nDroppedSampleTags.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input(inSpan);
        const bool               wantRecords = emit_records.value && outSpan.isConnected;
        std::size_t              made        = drain(outSpan, 0UZ);

        // The last sample of a call is held back so the end-of-stream epilogue always has a span to run on.
        const std::size_t offer = input.size() >= 2UZ ? input.size() - 1UZ : input.size();
        const std::size_t take  = std::min(offer, wantRecords ? roomFor(outSpan, made) : std::numeric_limits<std::size_t>::max());

        scanDroppedTags(inSpan, take);
        accumulate(input.first(take), wantRecords);
        made += drain(outSpan, made);

        outSpan.publish(made);
        std::ignore = inSpan.consume(take);
        return take == 0UZ && made == 0UZ && !input.empty() ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief End of stream: fold the trailing samples, then flush the partial window with the count it actually
    /// covers, reported and never suppressed.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool wantRecords = emit_records.value && outSpan.isConnected;
        scanDroppedTags(inSpan, inSpan.size());
        accumulate(std::span<const T>(inSpan), wantRecords);
        std::size_t made = drain(outSpan, 0UZ);

        if (!_flushed && _acc.count() > 0UZ) {
            const bool degenerate = _acc.degenerate();
            if (degenerate) {
                _nDegenerateWindows.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _slot.publish({_acc.excess(), _acc.normalized(), _acc.meanPower()}, static_cast<std::uint64_t>(_acc.count()));
            if (wantRecords && made < outSpan.size()) {
                outSpan[made] = makeRecord(degenerate, _acc.count());
                ++made;
            }
            _flushed = true;
        }
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    /// @brief The `n_dropped_samples` tags sitting on the @p count samples this call takes, bounded so that a tag
    /// belonging to a later call is neither counted twice nor attributed to the window now filling.
    void scanDroppedTags(InputSpanLike auto& inSpan, std::size_t count) {
        _droppedTags.clear();
        const std::uint64_t base = _streamAt;
        for (const gr::Tag& tag : inSpan.rawTags) {
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < base || at >= base + count) {
                continue;
            }
            const auto it = tag.map.find(gr::tag::N_DROPPED_SAMPLES.shortKey());
            if (it == tag.map.end()) {
                continue;
            }
            _droppedTags.push_back({at, droppedCount(it->second)});
            _nDroppedSampleTags.fetch_add(1ULL, std::memory_order_relaxed);
        }
    }

    /// @brief The reserved key's own `gr::Size_t`, and the wider integer a producer outside the tree might write.
    [[nodiscard]] static std::uint64_t droppedCount(const pmt::Value& value) {
        if (const auto* asSize = value.get_if<gr::Size_t>(); asSize != nullptr) {
            return static_cast<std::uint64_t>(*asSize);
        }
        if (const auto* asWide = value.get_if<std::uint64_t>(); asWide != nullptr) {
            return *asWide;
        }
        return 0ULL;
    }

    /// @brief Samples the producer reports missing over `[from, from + count)`, from this call's own scan.
    [[nodiscard]] std::uint64_t droppedWithin(std::uint64_t from, std::size_t count) const {
        std::uint64_t total = 0ULL;
        for (const DroppedTag& tag : _droppedTags) {
            if (tag.at >= from && tag.at < from + count) {
                total += tag.count;
            }
        }
        return total;
    }

    void accumulate(std::span<const T> input, bool wantRecords) {
        std::size_t at = 0UZ;
        while (at < input.size()) {
            const std::size_t remaining = static_cast<std::size_t>(window.value) - _acc.count();
            const std::size_t take      = std::min(remaining, input.size() - at);
            _acc.add(input.subspan(at, take));
            _droppedInWindow += droppedWithin(_streamAt, take);
            at += take;
            _streamAt += take;
            if (_acc.count() == static_cast<std::size_t>(window.value)) {
                closeWindow(wantRecords);
            }
        }
    }

    void closeWindow(bool wantRecords) {
        const bool degenerate = _acc.degenerate();
        if (degenerate) {
            _nDegenerateWindows.fetch_add(1ULL, std::memory_order_relaxed);
        }
        _slot.publish({_acc.excess(), _acc.normalized(), _acc.meanPower()}, static_cast<std::uint64_t>(window.value));
        _nWindows.fetch_add(1ULL, std::memory_order_relaxed);
        if (wantRecords) {
            _pending.push_back(makeRecord(degenerate, static_cast<std::size_t>(window.value)));
        }
        _acc.reset();
        _windowStartAt   = _streamAt;
        _droppedInWindow = 0ULL;
    }

    [[nodiscard]] DataSet<float> makeRecord(bool degenerate, std::size_t nSamples) const {
        const float excess     = degenerate ? 0.f : static_cast<float>(_acc.excess());
        const float normalized = degenerate ? 0.f : static_cast<float>(_acc.normalized());
        const float meanPow    = degenerate ? 0.f : static_cast<float>(_acc.meanPower());

        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"excess_kurtosis", "kurtosis", "", excess},
            {"normalized_fourth_moment", "kurtosis", "", normalized},
            {"mean_power", "power", "", meanPow},
        }};
        property_map                                          extra{
                                                     {std::pmr::string("n_samples"), pmt::Value(static_cast<std::uint64_t>(nSamples))},
                                                     {std::pmr::string("window"), pmt::Value(static_cast<std::uint64_t>(window.value))},
                                                     {std::pmr::string("input_domain"), pmt::Value(std::string(kComplexDomain ? "complex" : "real"))},
                                                     {std::pmr::string("valid"), pmt::Value(!degenerate)},
                                                     {std::pmr::string("signal_name"), pmt::Value(signal_name.value)},
        };
        if (_droppedInWindow > 0ULL) {
            extra.insert_or_assign(std::pmr::string("n_dropped_samples"), pmt::Value(_droppedInWindow));
        }
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), sample_rate.value, _windowStartAt, std::move(extra));
    }

    /// @brief How many input samples may be taken before a record would have nowhere to go.
    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& outSpan, std::size_t made) const {
        if (!outSpan.isConnected) {
            return std::numeric_limits<std::size_t>::max();
        }
        const std::size_t free       = outSpan.size() > made + _pending.size() ? outSpan.size() - made - _pending.size() : 0UZ;
        const std::size_t windowSize = static_cast<std::size_t>(window.value);
        const std::size_t untilClose = windowSize > _acc.count() ? windowSize - _acc.count() : 0UZ;
        if (free == 0UZ) {
            return untilClose > 0UZ ? untilClose - 1UZ : 0UZ;
        }
        return untilClose + (free - 1UZ) * windowSize;
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
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_KURTOSIS_HPP
