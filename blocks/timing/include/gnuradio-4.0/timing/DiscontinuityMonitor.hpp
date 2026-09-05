#ifndef GNURADIO_TIMING_DISCONTINUITY_MONITOR_HPP
#define GNURADIO_TIMING_DISCONTINUITY_MONITOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>

#include <gnuradio-4.0/timing/RationalRate.hpp>

namespace gr::blocks::timing {

/// @brief The causes a `discontinuity` tag names, as the tree's one producer writes them.
///
/// `gr::blocks::basic::DataSetToStream` builds the key as a comma-separated list drawn from exactly this vocabulary,
/// and it is the only block in the tree that puts the key on a stream. `Other` is the bucket for a cause a future
/// producer states that this list does not know, so an unrecognized name is counted rather than dropped.
enum class DiscontinuityCause : std::uint8_t { SampleRate = 0, SignalName, SignalQuantity, SignalUnit, SignalRange, Gap, Other };

inline constexpr std::size_t kDiscontinuityCauses = 7UZ;

inline constexpr std::array<std::string_view, kDiscontinuityCauses> kDiscontinuityCauseNames{"sample_rate", "signal_name", "signal_quantity", "signal_unit", "signal_range", "gap", "other"};

[[nodiscard]] inline constexpr DiscontinuityCause discontinuityCauseFrom(std::string_view name) noexcept {
    for (std::size_t i = 0UZ; i + 1UZ < kDiscontinuityCauses; ++i) {
        if (kDiscontinuityCauseNames[i] == name) {
            return static_cast<DiscontinuityCause>(i);
        }
    }
    return DiscontinuityCause::Other;
}

GR_REGISTER_BLOCK(gr::blocks::timing::DiscontinuityMonitor, [T], [ float, std::complex<float>, std::uint8_t ])

/**
 * @brief A passthrough that accounts the gaps in a stream: how much time is missing, and why.
 *
 * Every consumer that cares about timing re-derives the same three things from the same tags — how many samples were
 * dropped, what called them dropped, and what sample index now means what wall-clock time. This block is the one place
 * a graph asks, so the answer is stated once and every consumer reads the same one.
 *
 * It watches the metadata already flowing and invents none of its own: `n_dropped_samples` and `sample_rate` are
 * reserved keys of `gr::tag`, and `discontinuity` is the record vocabulary's cause list, written on a stream by
 * `gr::blocks::basic::DataSetToStream` as a comma-separated selection from `sample_rate`, `signal_name`,
 * `signal_quantity`, `signal_unit`, `signal_range` and `gap`. Nothing here reads the parked tag design.
 *
 * The samples pass through untouched and so do the tags: the block declares `UnfilteredTagPropagation`, so a key the
 * framework's default filter would drop — `discontinuity` is exactly such a key, being no reserved tag — leaves at the
 * offset it arrived at. That is load-bearing rather than incidental: a monitor that ate the evidence on the way past
 * would leave the next monitor downstream with nothing to account.
 *
 * The clock. A `SampleClock` maps sample index to nanoseconds exactly, and a rate change is a new anchor at the sample
 * it happens on rather than a smear across it, so the block re-anchors at each accepted `sample_rate` tag and both
 * epochs convert exactly. `nominal_rate` is what a stated rate is checked against: one disagreeing by more than
 * `rate_tolerance` is refused — counted, and named by `lastRefusedRate()` — and the clock keeps the rate it had. A
 * `nominal_rate` of zero turns the check off and every stated rate is adopted.
 *
 * Readers. The whole account is published through one seqlock, so a poll from another thread reads a whole account or
 * retries, never half of two. `SampleClock` itself is not part of that slot: it is five 64-bit fields and a slot of
 * doubles cannot carry a nanosecond count near the epoch without rounding it, which is the one thing the kernel exists
 * to prevent. `clock()` is therefore an owning-thread read.
 *
 * Cost. The sample path is a copy. The tag path runs once per tag, never per sample, and the bench measures the whole
 * block against a bare copy of the same stream.
 */
template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>> || std::same_as<T, std::uint8_t>)
struct DiscontinuityMonitor : Block<DiscontinuityMonitor<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Accounts a stream's gaps: dropped samples, discontinuity events by cause, and the sample-to-time map.

A 1:1 passthrough. It reads `n_dropped_samples`, `sample_rate` and the record vocabulary's `discontinuity` cause list,
totals them, and tracks a `SampleClock` re-anchored at each accepted rate change. Samples and tags pass through
unchanged. `nominal_rate` refuses a stated rate disagreeing by more than `rate_tolerance`, counted rather than fatal.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<double, "nominal_rate", Visible, Unit<"Hz">, Doc<"the rate the stream is expected to state; 0 accepts whatever it states">>          nominal_rate   = 0.0;
    Annotated<double, "rate_tolerance", Visible, Doc<"relative disagreement with nominal_rate that is still accepted">>                            rate_tolerance = 1e-6;
    Annotated<std::uint64_t, "anchor_index", Doc<"the sample index anchor_ns belongs to">>                                                         anchor_index   = 0ULL;
    Annotated<std::int64_t, "anchor_ns", Unit<"ns">, Doc<"nanoseconds since the Unix epoch at anchor_index; 0 makes the clock read elapsed time">> anchor_ns      = 0LL;

    GR_MAKE_REFLECTABLE(DiscontinuityMonitor, in, out, nominal_rate, rate_tolerance, anchor_index, anchor_ns);

    /// The published account. `filled` carries the dropped-sample total, which is the one number that can pass 2^53
    /// over a long run and so may not be a double; the event counts cannot, one tag being at most one event.
    static constexpr std::size_t kAccountValues = kDiscontinuityCauses + 3UZ;
    static constexpr std::size_t kEventsAt      = kDiscontinuityCauses;
    static constexpr std::size_t kRateChangesAt = kDiscontinuityCauses + 1UZ;
    static constexpr std::size_t kMismatchesAt  = kDiscontinuityCauses + 2UZ;

    gr::measurement::MeasurementSlot<kAccountValues> _slot{};

    std::array<double, kAccountValues> _account{};
    std::uint64_t                      _dropped{0ULL};
    std::uint64_t                      _tagCursor{0ULL}; ///< absolute index past the last tag already accounted
    gr::timing::SampleClock            _clock{};
    bool                               _haveRate{false};
    double                             _rateHz{0.}; ///< the rate the clock currently runs at, as stated
    double                             _refusedRate{std::numeric_limits<double>::quiet_NaN()};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!std::isfinite(nominal_rate) || nominal_rate < 0.0) {
            throw gr::exception(std::format("DiscontinuityMonitor: 'nominal_rate' must be zero or a positive finite rate, got {}", nominal_rate.value));
        }
        if (!std::isfinite(rate_tolerance) || rate_tolerance < 0.0) {
            throw gr::exception(std::format("DiscontinuityMonitor: 'rate_tolerance' must be zero or a positive finite fraction, got {}", rate_tolerance.value));
        }
        if (nominal_rate > 0.0) { // refuse an unrepresentable nominal at staging rather than at the first tag
            std::ignore = detail::rationalRate(nominal_rate, "DiscontinuityMonitor", "nominal_rate");
        }
        if (newSettings.contains("nominal_rate") || newSettings.contains("anchor_index") || newSettings.contains("anchor_ns")) {
            anchorClock(nominal_rate > 0.0 ? nominal_rate.value : 0.0);
        }
    }

    void start() { reset(); }

    /// @brief Zero the account and re-anchor the clock. For the owning thread between stop() and start().
    void reset() {
        _account.fill(0.);
        _dropped     = 0ULL;
        _tagCursor   = 0ULL;
        _refusedRate = std::numeric_limits<double>::quiet_NaN();
        anchorClock(nominal_rate > 0.0 ? nominal_rate.value : 0.0);
        publish();
    }

    /// @brief Total samples the stream said were dropped. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSamples() const noexcept { return _slot.read().second; }

    /// @brief Discontinuity events seen, one per `discontinuity` tag however many causes it names. Any thread.
    [[nodiscard]] std::uint64_t nEvents() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kEventsAt]); }

    /// @brief Causes counted, one per name in each event's list; an event naming three causes counts in three. Any thread.
    [[nodiscard]] std::uint64_t nEvents(DiscontinuityCause cause) const noexcept { return static_cast<std::uint64_t>(_slot.read().first[static_cast<std::size_t>(cause)]); }

    /// @brief The whole account in one coherent read: the per-cause counts, then events, rate changes and mismatches.
    [[nodiscard]] std::array<std::uint64_t, kAccountValues> account() const noexcept {
        const auto                                values = _slot.read().first;
        std::array<std::uint64_t, kAccountValues> counts{};
        for (std::size_t i = 0UZ; i < kAccountValues; ++i) {
            counts[i] = static_cast<std::uint64_t>(values[i]);
        }
        return counts;
    }

    /// @brief Rate changes the clock was re-anchored at. Callable from any thread.
    [[nodiscard]] std::uint64_t nRateChanges() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kRateChangesAt]); }

    /// @brief Stated rates refused for disagreeing with `nominal_rate` beyond `rate_tolerance`. Any thread.
    [[nodiscard]] std::uint64_t nRateMismatches() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kMismatchesAt]); }

    /// @brief The last refused rate, or NaN if none was. Owning thread.
    [[nodiscard]] double lastRefusedRate() const noexcept { return _refusedRate; }

    /// @brief Whether a rate has been established, by `nominal_rate` or by an accepted tag. Owning thread.
    [[nodiscard]] bool hasClock() const noexcept { return _haveRate; }

    /// @brief The current sample-to-time map, re-anchored at the last accepted rate change. Owning thread.
    [[nodiscard]] const gr::timing::SampleClock& clock() const noexcept { return _clock; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input    = std::span<const T>(inSpan);
        const std::size_t        nSamples = std::min(input.size(), outSpan.size());
        const std::uint64_t      base     = static_cast<std::uint64_t>(inSpan.streamIndex);
        const std::uint64_t      end      = base + static_cast<std::uint64_t>(nSamples);

        bool changed = false;
        for (const gr::Tag& tag : inSpan.rawTags) {
            // A tag the framework could not retire with its chunk returns in the next window; the cursor is what makes
            // each one countable exactly once, whatever window it arrives in.
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < _tagCursor || at >= end) {
                continue;
            }
            changed = account(tag.map, at) || changed;
        }
        _tagCursor = std::max(_tagCursor, end);

        std::ranges::copy(input.first(nSamples), outSpan.begin());

        if (changed) {
            publish();
        }
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

private:
    void anchorClock(double rateHz) {
        _haveRate = rateHz > 0.0;
        _rateHz   = _haveRate ? rateHz : 0.0;
        if (_haveRate) {
            const auto [num, den] = detail::rationalRate(rateHz, "DiscontinuityMonitor", "sample_rate");
            _clock                = gr::timing::SampleClock(anchor_index.value, anchor_ns.value, num, den);
        } else {
            _clock = gr::timing::SampleClock{};
        }
    }

    /// @brief Take one tag's metadata into the account; true when anything the readers show has moved.
    [[nodiscard]] bool account(const property_map& map, std::uint64_t at) {
        // the map's hash and equality are transparent, so a lookup by the reserved key's own characters copies nothing
        bool changed = false;
        if (const auto it = map.find(gr::tag::N_DROPPED_SAMPLES.shortKey()); it != map.end()) {
            if (const gr::Size_t* dropped = it->second.template get_if<gr::Size_t>(); dropped != nullptr) {
                _dropped += static_cast<std::uint64_t>(*dropped);
                changed = true;
            }
        }
        if (const auto it = map.find(std::string_view("discontinuity")); it != map.end()) {
            if (const std::pmr::string* causes = it->second.template get_if<std::pmr::string>(); causes != nullptr) {
                countCauses(std::string_view(causes->data(), causes->size()));
                _account[kEventsAt] += 1.;
                changed = true;
            }
        }
        if (const auto it = map.find(gr::tag::SAMPLE_RATE.shortKey()); it != map.end()) {
            if (const float* rate = it->second.template get_if<float>(); rate != nullptr) {
                changed = adoptRate(static_cast<double>(*rate), at) || changed;
            }
        }
        return changed;
    }

    void countCauses(std::string_view causes) noexcept {
        while (!causes.empty()) {
            const std::size_t      comma = causes.find(',');
            const std::string_view name  = causes.substr(0UZ, comma);
            if (!name.empty()) {
                _account[static_cast<std::size_t>(discontinuityCauseFrom(name))] += 1.;
            }
            if (comma == std::string_view::npos) {
                return;
            }
            causes.remove_prefix(comma + 1UZ);
        }
    }

    /// @brief Adopt a stated rate as a new epoch, or refuse it against `nominal_rate` and say so.
    [[nodiscard]] bool adoptRate(double rateHz, std::uint64_t at) {
        if (!std::isfinite(rateHz) || rateHz <= 0.0 || (nominal_rate > 0.0 && std::abs(rateHz - nominal_rate) > rate_tolerance * nominal_rate)) {
            _refusedRate = rateHz;
            _account[kMismatchesAt] += 1.;
            return true;
        }
        if (_haveRate && rateHz == _rateHz) {
            return false; // the stream restating the rate it already runs at is not a discontinuity
        }
        const auto [num, den] = detail::rationalRate(rateHz, "DiscontinuityMonitor", "sample_rate");
        // A rate change is a new anchor at the sample it happens on. Before the first rate there is nothing to carry,
        // so the clock is built at the configured anchor rather than moved from a map that does not exist yet.
        _clock    = _haveRate ? _clock.withRate(num, den, at) : gr::timing::SampleClock(anchor_index.value, anchor_ns.value, num, den);
        _rateHz   = rateHz;
        _haveRate = true;
        _account[kRateChangesAt] += 1.;
        return true;
    }

    void publish() noexcept { _slot.publish(_account, _dropped); }
};

} // namespace gr::blocks::timing

#endif // GNURADIO_TIMING_DISCONTINUITY_MONITOR_HPP
