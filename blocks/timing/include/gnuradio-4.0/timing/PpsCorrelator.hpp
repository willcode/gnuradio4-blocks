#ifndef GNURADIO_TIMING_PPS_CORRELATOR_HPP
#define GNURADIO_TIMING_PPS_CORRELATOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>

namespace gr::blocks::timing {

/// Where an edge comes from: the samples themselves, or the trigger tags a PPS source already publishes.
enum class PpsEdgeSource : std::uint8_t { Threshold = 0, TriggerTag };

namespace detail {

[[nodiscard]] inline PpsEdgeSource parseEdgeSource(std::string_view name) {
    if (name == "threshold") {
        return PpsEdgeSource::Threshold;
    }
    if (name == "trigger_tag") {
        return PpsEdgeSource::TriggerTag;
    }
    throw gr::exception(std::format("PpsCorrelator: 'edge_source' must be 'threshold' or 'trigger_tag', got '{}'", name));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::timing::PpsCorrelator, [T], [ float, std::complex<float>, std::uint8_t ])

/**
 * @brief The sampling clock's frequency error, measured against a one-pulse-per-second reference.
 *
 * Two clocks never agree. A receiver counting `N_k` samples between two PPS edges that are one second apart by
 * definition has measured its own rate: `rate_error_ppm = (N_k / nominal_interval - 1) * 1e6`, where
 * `nominal_interval = sample_rate * pps_interval`. That number is exactly the `ppm` a `gr::recipes::SampleClockOffset`
 * corrects with, which is what makes the loop closable and what the QA closes.
 *
 * Two edge sources, one edge convention. `threshold` reads a hardware pulse on the stream: the magnitude is taken, an
 * edge is the first sample at or above `threshold` while armed, and the sub-sample position is the two-point linear
 * interpolation across the crossing,
 *
 *     position = (k - 1) + (threshold - m[k-1]) / (m[k] - m[k-1])
 *
 * with `m[k-1] < threshold <= m[k]` guaranteed by the arming, so the denominator is positive. The detector re-arms
 * when the magnitude falls back below `threshold`. A stream that starts already above it is not a rise anybody saw,
 * so it disarms without an edge. The interpolation is exact for a linear edge and carries the shape's own curvature
 * error otherwise, which is the caller's pulse rather than the block's arithmetic.
 *
 * `trigger_tag` reads what `gr::blocks::timing::PpsSource` already publishes, rather than inventing a second edge
 * convention: that block emits `std::uint8_t` samples that are all zero and puts `trigger_name`, `trigger_time` and
 * `trigger_offset` on a tag at the first sample of each second — one sample per second in `ppsOnly` mode,
 * `sample_rate` samples per second in `clock` mode. The edge is the tag's own stream offset, moved by
 * `trigger_offset`, which is a delay in **seconds** per the reserved key's declaration and so is scaled by the
 * nominal rate. `trigger_filter` restricts which trigger names count; empty takes every tag carrying a
 * `trigger_time`. Note that a `ppsOnly` stream advances one sample per second, so its nominal interval is one
 * sample: `sample_rate` must be the rate of the stream the block is reading, not the rate of some other stream.
 *
 * `sample_rate` is a `double` here rather than the `float` the reserved tag carries, because a part-per-million
 * measurement against a rate rounded to 24 bits would already be carrying 0.06 ppm of the rate itself above 16.8 MS/s.
 * A `float` `sample_rate` tag still updates it: the settings machinery converts numerically into the wider type.
 *
 * Honesty about missing pulses. An interval outside `+/-tolerance_ppm` is not a rate measurement, it is a lost or a
 * spurious edge, and averaging it in would move the figure by orders of magnitude. A long interval is counted as
 * `round(N_k / nominal) - 1` missed edges and a short one as one extra edge; either way the interval is excluded from
 * the smoothed figure and its record says so. That is why deleting one pulse leaves the reported rate unchanged.
 *
 * Records. One per closed interval on the tier's measurement conventions, with `sample_start` the whole sample the
 * interval opened on. An interval still open at the end of a stream has no length and so no measurement; the edge
 * counters are what report it. The record port is optional, and the readers are the whole interface for a graph that
 * polls: the seqlock carries the smoothed figure and the four counts together, so a poll sees one consistent account.
 */
template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>> || std::same_as<T, std::uint8_t>)
struct PpsCorrelator : Block<PpsCorrelator<T>> {
    using Description = Doc<R""(
@brief Measures the sampling clock's error in ppm from the interval between PPS edges.

`rate_error_ppm = (N_k / (sample_rate * pps_interval) - 1) * 1e6` per interval, from a threshold crossing with
two-point linear sub-sample interpolation, or from the trigger tags a PPS source publishes. An interval outside
`tolerance_ppm` is counted as a missed or extra edge and excluded from the smoothed figure. One record per interval.
)"">;

    PortIn<T> in;
    /// One record per closed interval, for a consumer outside C++. Optional: leaving it unconnected costs nothing.
    PortOut<DataSet<float>, Async, Optional> records;

    Annotated<double, "sample_rate", Visible, Unit<"Hz">, Doc<"nominal rate of the stream being read">>                              sample_rate    = 1.0;
    Annotated<double, "pps_interval", Visible, Unit<"s">, Doc<"nominal seconds between reference edges">>                            pps_interval   = 1.0;
    Annotated<std::string, "edge_source", Visible, Doc<"'threshold' reads a pulse on the stream, 'trigger_tag' reads trigger tags">> edge_source    = std::string("threshold");
    Annotated<double, "threshold", Visible, Doc<"magnitude a rising edge crosses; 'threshold' source only">>                         threshold      = 0.5;
    Annotated<std::string, "trigger_filter", Doc<"only trigger names starting with this count; empty takes every trigger_time">>     trigger_filter = std::string("");
    Annotated<double, "tolerance_ppm", Visible, Doc<"how far an interval may be from nominal and still be a measurement">>           tolerance_ppm  = 10'000.0;
    Annotated<gr::Size_t, "n_intervals", Visible, Doc<"accepted intervals the reported figure is averaged over">>                    n_intervals    = 16U;

    GR_MAKE_REFLECTABLE(PpsCorrelator, in, records, sample_rate, pps_interval, edge_source, threshold, trigger_filter, tolerance_ppm, n_intervals);

    /// The published account: the smoothed figure, the last interval's own, and the four counts.
    static constexpr std::size_t kSmoothedAt = 0UZ;
    static constexpr std::size_t kLastAt     = 1UZ;
    static constexpr std::size_t kEdgesAt    = 2UZ;
    static constexpr std::size_t kMissedAt   = 3UZ;
    static constexpr std::size_t kExtraAt    = 4UZ;
    static constexpr std::size_t kAcceptedAt = 5UZ;
    static constexpr std::size_t kSlotValues = 6UZ;

    gr::measurement::MeasurementSlot<kSlotValues> _slot{};

    PpsEdgeSource                   _source{PpsEdgeSource::Threshold};
    double                          _nominalInterval{1.};
    std::array<double, kSlotValues> _values{};
    std::uint64_t                   _intervals{0ULL};

    std::vector<double> _window{};        ///< the last accepted figures, a ring
    std::size_t         _windowAt{0UZ};   ///< where the next accepted figure lands
    std::size_t         _windowFill{0UZ}; ///< how much of the ring is written

    double        _previousEdge{0.}; ///< position of the edge that opened the current interval
    bool          _haveEdge{false};
    double        _lastMagnitude{0.};
    bool          _havePrevious{false}; ///< a sample before this call's first has been seen
    bool          _armed{false};
    std::uint64_t _tagCursor{0ULL}; ///< absolute index past the last tag already read

    std::vector<DataSet<float>> _pending{}; ///< records made but not yet published, drained as the port offers room

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!std::isfinite(sample_rate) || sample_rate <= 0.0) {
            throw gr::exception(std::format("PpsCorrelator: 'sample_rate' must be positive and finite, got {}", sample_rate.value));
        }
        if (!std::isfinite(pps_interval) || pps_interval <= 0.0) {
            throw gr::exception(std::format("PpsCorrelator: 'pps_interval' must be positive and finite, got {}", pps_interval.value));
        }
        if (!std::isfinite(threshold) || threshold <= 0.0) {
            throw gr::exception(std::format("PpsCorrelator: 'threshold' must be positive and finite, got {}", threshold.value));
        }
        if (!std::isfinite(tolerance_ppm) || tolerance_ppm <= 0.0) {
            throw gr::exception(std::format("PpsCorrelator: 'tolerance_ppm' must be positive and finite, got {}", tolerance_ppm.value));
        }
        if (n_intervals < 1U) {
            throw gr::exception("PpsCorrelator: 'n_intervals' must be at least one");
        }
        _source          = detail::parseEdgeSource(edge_source);
        _nominalInterval = sample_rate * pps_interval;
        _window.assign(static_cast<std::size_t>(n_intervals.value), 0.);
        reset();
    }

    void start() { reset(); }

    /// @brief Forget every edge and every count. For the owning thread between stop() and start().
    void reset() {
        _values.fill(0.);
        _intervals  = 0ULL;
        _windowAt   = 0UZ;
        _windowFill = 0UZ;
        std::ranges::fill(_window, 0.);
        _previousEdge  = 0.;
        _haveEdge      = false;
        _lastMagnitude = 0.;
        _havePrevious  = false;
        _armed         = false;
        _tagCursor     = 0ULL;
        _pending.clear();
        publish();
    }

    /// @brief The clock error in parts per million, averaged over the last `n_intervals` accepted intervals. Any thread.
    [[nodiscard]] double rateErrorPpm() const noexcept { return _slot.read().first[kSmoothedAt]; }

    /// @brief The last accepted interval's own figure, unaveraged. Callable from any thread.
    [[nodiscard]] double lastRateErrorPpm() const noexcept { return _slot.read().first[kLastAt]; }

    /// @brief Edges located. Callable from any thread.
    [[nodiscard]] std::uint64_t nEdges() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kEdgesAt]); }

    /// @brief Reference edges the intervals imply were lost. Callable from any thread.
    [[nodiscard]] std::uint64_t nMissedEdges() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kMissedAt]); }

    /// @brief Intervals too short to be one period, each one spurious edge. Callable from any thread.
    [[nodiscard]] std::uint64_t nExtraEdges() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kExtraAt]); }

    /// @brief Intervals closed, of which `nAcceptedIntervals()` were inside the tolerance. Callable from any thread.
    [[nodiscard]] std::uint64_t nIntervals() const noexcept { return _slot.read().second; }

    [[nodiscard]] std::uint64_t nAcceptedIntervals() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kAcceptedAt]); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input = std::span<const T>(inSpan);
        const std::uint64_t      base  = static_cast<std::uint64_t>(inSpan.streamIndex);

        if (_source == PpsEdgeSource::Threshold) {
            scanThreshold(input, base);
        } else {
            scanTags(inSpan, base + static_cast<std::uint64_t>(input.size()));
        }
        std::size_t made = 0UZ;
        if (outSpan.isConnected) {
            const std::size_t take = std::min(_pending.size(), outSpan.size());
            for (; made < take; ++made) {
                outSpan[made] = std::move(_pending[made]);
            }
            _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        } else {
            _pending.clear(); // an unconnected port is not a backlog
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(input.size());
        return work::Status::OK;
    }

private:
    /// The magnitude, so a bipolar pulse is one edge rather than two and a complex one needs no phase.
    [[nodiscard]] static double magnitude(T sample) noexcept {
        if constexpr (std::same_as<T, std::complex<float>>) {
            return std::sqrt(static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag()));
        } else if constexpr (std::same_as<T, float>) {
            return std::abs(static_cast<double>(sample));
        } else {
            return static_cast<double>(sample);
        }
    }

    void scanThreshold(std::span<const T> input, std::uint64_t base) {
        const double level    = threshold;
        double       previous = _lastMagnitude;
        bool         armed    = _armed;
        bool         have     = _havePrevious;

        for (std::size_t k = 0UZ; k < input.size(); ++k) {
            const double m = magnitude(input[k]);
            if (!have) {
                // the stream's own first sample: whether it is high says only what state the detector starts in
                armed = m < level;
                have  = true;
            } else if (armed && m >= level) {
                // previous < level <= m by the arming, so the denominator is positive and the crossing is between them
                const double fraction = (level - previous) / (m - previous);
                closeEdge(static_cast<double>(base + static_cast<std::uint64_t>(k)) - 1. + fraction);
                armed = false;
            } else if (!armed && m < level) {
                armed = true;
            }
            previous = m;
        }

        _lastMagnitude = previous;
        _armed         = armed;
        _havePrevious  = have;
    }

    void scanTags(const auto& inSpan, std::uint64_t end) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < _tagCursor || at >= end) {
                continue; // a tag the framework could not retire returns in the next window; count it exactly once
            }
            const auto found = tag.map.find(gr::tag::TRIGGER_TIME.shortKey());
            if (found == tag.map.end() || found->second.template get_if<std::uint64_t>() == nullptr) {
                continue;
            }
            if (!trigger_filter.value.empty()) {
                const auto named = tag.map.find(gr::tag::TRIGGER_NAME.shortKey());
                if (named == tag.map.end()) {
                    continue;
                }
                const std::pmr::string* name = named->second.template get_if<std::pmr::string>();
                if (name == nullptr || !std::string_view(name->data(), name->size()).starts_with(std::string_view(trigger_filter.value))) {
                    continue;
                }
            }
            // trigger_offset is a delay in seconds against the tagged sample, so it becomes samples through the rate
            double offset = 0.;
            if (const auto delay = tag.map.find(gr::tag::TRIGGER_OFFSET.shortKey()); delay != tag.map.end()) {
                if (const float* seconds = delay->second.template get_if<float>(); seconds != nullptr) {
                    offset = static_cast<double>(*seconds) * sample_rate;
                }
            }
            closeEdge(static_cast<double>(at) + offset);
        }
        _tagCursor = std::max(_tagCursor, end);
    }

    /// @brief Take one located edge: count it, and where it closes an interval, measure and report that interval.
    void closeEdge(double position) {
        _values[kEdgesAt] += 1.;
        if (!_haveEdge) {
            _previousEdge = position;
            _haveEdge     = true;
            publish();
            return;
        }

        const double span     = position - _previousEdge;
        const double ppm      = (span / _nominalInterval - 1.) * 1e6;
        const bool   accepted = std::abs(ppm) <= tolerance_ppm;

        std::uint64_t missed = 0ULL;
        if (!accepted) {
            if (span > _nominalInterval) {
                // a long interval is whole periods that went unseen, and one too long to accept lost at least one
                const std::int64_t periods = std::max<std::int64_t>(2, std::llround(span / _nominalInterval));
                missed                     = static_cast<std::uint64_t>(periods) - 1ULL;
                _values[kMissedAt] += static_cast<double>(missed);
            } else {
                _values[kExtraAt] += 1.;
            }
        } else {
            _window[_windowAt] = ppm;
            _windowAt          = (_windowAt + 1UZ) % _window.size();
            _windowFill        = std::min(_windowFill + 1UZ, _window.size());
            double mean        = 0.;
            for (std::size_t i = 0UZ; i < _windowFill; ++i) { // ring order, fixed, so the mean is a function of the input alone
                mean += _window[i];
            }
            _values[kSmoothedAt] = mean / static_cast<double>(_windowFill);
            _values[kLastAt]     = ppm;
            _values[kAcceptedAt] += 1.;
        }

        emitRecord(_intervals, position, span, ppm, accepted, missed);
        ++_intervals;
        _previousEdge = position;
        publish();
    }

    void emitRecord(std::uint64_t index, double position, double span, double ppm, bool accepted, std::uint64_t missed) {
        const double                                          whole = std::floor(position);
        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"rate_error_ppm", "Frequency error", "ppm", static_cast<float>(ppm)},
            {"edge_fraction", "Position", "1", static_cast<float>(position - whole)},
            {"accepted", "Flag", "1", accepted ? 1.f : 0.f},
        }};
        // The absolute sample index and the interval length are exact integers a float channel would round, so they
        // travel as metadata; `sample_start` is the whole sample the interval opened on, per the tier's conventions.
        property_map extra{
            {property_map::key_type("interval_index"), pmt::Value(index)},
            {property_map::key_type("edge_index"), pmt::Value(static_cast<std::uint64_t>(whole))},
            {property_map::key_type("interval_samples"), pmt::Value(span)},
            {property_map::key_type("n_missed_edges"), pmt::Value(missed)},
        };
        _pending.push_back(gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), static_cast<float>(sample_rate), static_cast<std::uint64_t>(std::floor(_previousEdge)), std::move(extra)));
    }

    void publish() noexcept { _slot.publish(_values, _intervals); }
};

} // namespace gr::blocks::timing

#endif // GNURADIO_TIMING_PPS_CORRELATOR_HPP
