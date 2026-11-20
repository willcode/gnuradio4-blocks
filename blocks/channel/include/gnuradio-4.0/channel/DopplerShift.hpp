#ifndef GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP
#define GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>
#include <gnuradio-4.0/algorithm/timing/ScheduleAnchor.hpp>
#include <gnuradio-4.0/algorithm/timing/TrajectoryFile.hpp>

namespace gr::blocks::channel {

/// Where the next sample stands against the schedule's time span. `Unarmed` means a trigger anchor has not seen
/// its trigger.
enum class SchedulePosition : std::uint8_t { Before = 0, Inside, After, Unarmed };

namespace detail {

/// The direction of the schedule. `apply` puts the trajectory's shift on a clean signal. `correct` removes it.
enum class DopplerDirection : std::uint8_t { Apply = 0, Correct };

/// The limits of `anchor_ns`, the bound `gr::timing::ScheduleAnchor` holds.
using AnchorNsLimits = Limits<Min<-gr::timing::ScheduleAnchor::kMaxAnchorNs>, Max<gr::timing::ScheduleAnchor::kMaxAnchorNs>>;

/// The `Doc` texts of the settings that blocks reading a trajectory table share.
inline constexpr gr::meta::fixed_string kScheduleTimesDoc      = "strictly increasing knot times on the SampleClock axis, two or more";
inline constexpr gr::meta::fixed_string kDirectionDoc          = "schedule direction, 'apply' or 'correct'";
inline constexpr gr::meta::fixed_string kSampleRateDoc         = "stream sample rate";
inline constexpr gr::meta::fixed_string kAnchorIndexDoc        = "the stream sample that anchor_ns belongs to, under anchor_source 'setting'";
inline constexpr gr::meta::fixed_string kAnchorNsDoc           = "schedule time of anchor_index under anchor_source 'setting'";
inline constexpr gr::meta::fixed_string kAnchorSourceDoc       = "'setting', 'first_trigger' or 'every_trigger'";
inline constexpr gr::meta::fixed_string kHonorTriggerOffsetDoc = "move a trigger anchor by the tag's own gr:trigger_offset seconds";

/// A trigger tag's time and offset, read with the reserved keys' types. A tag without a time is no anchor event.
struct TriggerRead {
    std::uint64_t timeNs{0ULL};
    float         offsetSeconds{0.f};
    bool          present{false};  ///< the tag carries a `gr:trigger_time` key
    bool          readable{false}; ///< the key, and `gr:trigger_offset` when honored, carry the reserved types
};

/// @brief Reads a tag's `gr:trigger_time` and, when @p honorOffset is set, its `gr:trigger_offset`. A key of a type
/// other than the reserved one leaves `readable` false.
[[nodiscard]] inline TriggerRead readTrigger(const property_map& map, bool honorOffset) {
    TriggerRead read;
    const auto  time = map.find(property_map::key_type(gr::tag::TRIGGER_TIME.key()));
    if (time == map.end()) {
        return read;
    }
    read.present                = true;
    const std::uint64_t* timeNs = time->second.get_if<std::uint64_t>();
    if (timeNs == nullptr) {
        return read;
    }
    read.timeNs   = *timeNs;
    read.readable = true;
    if (honorOffset) {
        if (const auto offset = map.find(property_map::key_type(gr::tag::TRIGGER_OFFSET.key())); offset != map.end()) {
            const float* seconds = offset->second.get_if<float>();
            if (seconds == nullptr) {
                read.readable = false;
                return read;
            }
            read.offsetSeconds = *seconds;
        }
    }
    return read;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::channel::DopplerShift, [T], [std::complex<float>])

template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct DopplerShift : gr::Block<DopplerShift<T>> {
    using Description = Doc<R""(
@brief Applies a trajectory's Doppler frequency shift to a complex stream, or removes it.

In a channel model the block puts a moving source's carrier shift on a clean signal. Ahead of a demodulator it takes
the shift off. Each sample is mixed with a phasor whose phase increment is the integral of a frequency schedule over
that sample's interval. The schedule is linear between its `(time, offset)` knots and holds its end values outside
them. `gr::timing::offsetFor(v_radial, f_carrier)` converts a radial velocity to an offset, and a closing source
reads high. One sample comes out for each sample in.

The knots come from `schedule_times_ns` and `schedule_offsets_hz`, or from the `offset_hz` or `range_rate_m_s` column of
a `#!gr4-trajectory 1` file named in `schedule_file`. Setting both is refused. With neither, the stream passes through.
Every offset must stay under half of `sample_rate` in magnitude, and the refusal names the knot. `direction` 'apply'
adds the offsets, and 'correct' subtracts them. A new table, direction or rate takes effect at the next sample and keeps
the phase. A new rate re-times the whole stream from the anchor.

`anchor_source` selects what ties the schedule's time axis to the stream. Under 'setting', sample `anchor_index` is at
time `anchor_ns`. Under 'first_trigger', the first `gr:trigger_time` tag sets the anchor at its own sample, and later
tags are ignored. Under 'every_trigger', each tag moves the anchor. Every tag that arms or moves the anchor restarts the
phase at zero. `honor_trigger_offset` adds the tag's `gr:trigger_offset` seconds to its time. Until a trigger anchor is
armed, the stream passes through unshifted. A change to one of these four settings rebuilds the anchor and clears its
counters, and a trigger anchor then waits for its next tag. Every other change keeps the anchor.
`gr::timing::ScheduleAnchor` holds these rules.

A `gr:frequency` tag passes unchanged. `currentOffsetHz()` reads the offset at the next sample, `schedulePosition()`
where that sample stands against the knots, and `position()` its index from the stream's start. `nIgnoredAnchors()`
counts the trigger tags the mode ignores, `nReanchors()` the moves under 'every_trigger', and `nRefusedAnchors()` the
tags whose time is unreadable or past the anchor's bound. Each reads from any thread.

`schedule_offsets_hz` holds one offset per time, and the end values hold outside the schedule. `schedule_file` names a
'#!gr4-trajectory 1' file with a frequency column.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<std::vector<std::int64_t>, Visible, Unit<"ns">, Doc<detail::kScheduleTimesDoc>> schedule_times_ns{};
        Annotated<std::vector<double>, Visible, Unit<"Hz">, Doc<"scheduled frequency offsets">>   schedule_offsets_hz{};
        Annotated<std::string, Doc<"frequency trajectory file">>                                  schedule_file{};
        Annotated<detail::DopplerDirection, Visible, Doc<detail::kDirectionDoc>>                  direction            = detail::DopplerDirection::Apply;
        Annotated<float, Visible, Unit<"Hz">, Doc<detail::kSampleRateDoc>>                        sample_rate          = 1.f;
        Annotated<std::uint64_t, Doc<detail::kAnchorIndexDoc>>                                    anchor_index         = 0ULL;
        Annotated<std::int64_t, Unit<"ns">, Doc<detail::kAnchorNsDoc>, detail::AnchorNsLimits>    anchor_ns            = 0LL;
        Annotated<gr::timing::AnchorSource, Doc<detail::kAnchorSourceDoc>>                        anchor_source        = gr::timing::AnchorSource::setting;
        Annotated<bool, Doc<detail::kHonorTriggerOffsetDoc>>                                      honor_trigger_offset = true;

        GR_MAKE_REFLECTABLE(Settings, schedule_times_ns, schedule_offsets_hz, schedule_file, direction, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset);
    } settings;

    GR_MAKE_REFLECTABLE(DopplerShift, in, out, settings);

    std::shared_ptr<const gr::timing::FrequencySchedule> _table{};    ///< the knots as configured, before the direction's sign
    std::shared_ptr<const gr::timing::FrequencySchedule> _schedule{}; ///< the knots in the direction's sign

    gr::timing::SampleClock    _clock{};
    gr::timing::ScheduleAnchor _anchor{};
    gr::signal::Phasor<float>  _phasor{};
    std::vector<double>        _increments{};   ///< increments for one call, resized only upward
    std::uint64_t              _position{0ULL}; ///< index of the next sample, counted from the stream's own start

    // The observables, copied for a reader on another thread. The doubles go through the slot. A nanosecond count
    // since the epoch is past the exact integer range of a double, and the anchor time has its own atomic.
    gr::measurement::MeasurementSlot<2UZ> _slot{}; ///< offset in Hz and schedule position, with the stream position as fill count
    std::atomic<bool>                     _armed{true};
    std::atomic<std::uint64_t>            _anchorIndex{0ULL};
    std::atomic<std::int64_t>             _anchorNs{0LL};
    std::atomic<std::uint64_t>            _nIgnoredAnchors{0ULL};
    std::atomic<std::uint64_t>            _nReanchors{0ULL};
    std::atomic<std::uint64_t>            _nRefusedAnchors{0ULL};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const bool tableGiven = !s.schedule_times_ns.value.empty() || !s.schedule_offsets_hz.value.empty();
        if (!s.schedule_file.value.empty() && tableGiven) {
            return gr::Refused("schedule_file is incompatible with schedule lists");
        }
        if (tableGiven) {
            std::ignore = gr::timing::FrequencySchedule(s.schedule_times_ns.value, s.schedule_offsets_hz.value);
            if (const auto knot = knotAboveNyquist(s.schedule_offsets_hz.value, s.sample_rate)) {
                return gr::Refused(s.schedule_offsets_hz, *knot);
            }
        }
        // a trigger anchor is placed by the stream, so its clock is checked when its tag arrives
        const bool fromSetting = s.anchor_source == gr::timing::AnchorSource::setting;
        const auto clock       = clockAt(s.sample_rate, fromSetting ? s.anchor_index.value : 0ULL, fromSetting ? s.anchor_ns.value : 0LL);
        if (!clock) {
            return gr::Refused(s.sample_rate, clock.error().message);
        }
        return {};
    }

    void applySettings(const Settings& s) {
        const bool                       anchorMoved = s.anchor_source.changed() || s.anchor_index.changed() || s.anchor_ns.changed() || s.honor_trigger_offset.changed();
        const gr::timing::ScheduleAnchor anchor      = anchorMoved ? gr::timing::ScheduleAnchor(s.anchor_source, s.anchor_index, s.anchor_ns, s.honor_trigger_offset) : _anchor;
        // an anchor a trigger armed keeps its place when the rate moves, and the check cannot see that place
        const auto clock = clockAt(s.sample_rate, anchor.armed() ? anchor.anchorIndex() : 0ULL, anchor.armed() ? anchor.anchorNs() : 0LL);
        if (!clock) {
            this->fail(s.sample_rate, clock.error().message);
            return;
        }
        const bool tableMoved = s.schedule_times_ns.changed() || s.schedule_offsets_hz.changed() || s.schedule_file.changed();
        const auto table      = tableMoved ? loadTable(s) : _table;
        if (!table) {
            this->fail(s.schedule_file, table.error()); // only a file fails here: the check refuses a table of the vectors
            return;
        }
        if (*table && !s.schedule_file.value.empty()) { // a file's knots are known only once it is read
            if (const auto knot = knotAboveNyquist((*table)->offsets(), s.sample_rate)) {
                this->fail(s.schedule_file, *knot);
                return;
            }
        }

        if (tableMoved || s.direction.changed()) {
            _schedule = signedSchedule(*table, s.direction);
        }
        _table  = *table;
        _anchor = anchor;
        if (anchor.armed()) {
            _clock = *clock;
        }
        publishMeasurements();
    }

    void start() {
        _position = 0ULL;
        _phasor.setPhase(0.);
        _anchor.reset(); // a trigger anchor waits for the new stream's trigger
        publishMeasurements();
    }

    /// @brief The offset in Hz the block applies to the stream now, negated in `correct`. Any thread.
    [[nodiscard]] double currentOffsetHz() const noexcept { return _slot.read().first[0]; }

    /// @brief The position of the next sample against the schedule's span. `Before` and `After` hold an end value.
    [[nodiscard]] SchedulePosition schedulePosition() const noexcept { return static_cast<SchedulePosition>(static_cast<std::uint8_t>(_slot.read().first[1])); }

    /// @brief The next sample's index counted from the stream's start, which the anchor places in time.
    [[nodiscard]] std::uint64_t position() const noexcept { return _slot.read().second; }

    [[nodiscard]] bool          anchorArmed() const noexcept { return _armed.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t anchorIndex() const noexcept { return _anchorIndex.load(std::memory_order_relaxed); }
    [[nodiscard]] std::int64_t  anchorNs() const noexcept { return _anchorNs.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nIgnoredAnchors() const noexcept { return _nIgnoredAnchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nReanchors() const noexcept { return _nReanchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedAnchors() const noexcept { return _nRefusedAnchors.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t  nSamples = std::min(input.size(), output.size());
        const std::span<T> written  = output.first(nSamples);

        if (!_schedule || !_anchor.armed()) {
            std::ranges::copy(input.first(nSamples), written.begin());
            _position += nSamples;
            publishMeasurements();
            return work::Status::OK;
        }

        // The buffer grows to the largest chunk and keeps that size. A call allocates only when its chunk is
        // larger than every earlier one.
        if (_increments.size() < nSamples) {
            _increments.resize(nSamples);
        }
        const std::span<double> increments(_increments.data(), nSamples);
        _schedule->phaseIncrementsFor(_clock, _position, increments);
        _phasor.mixModulated(std::span<const double>(increments), input.first(nSamples), written);

        _position += nSamples;
        publishMeasurements();
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());

        // Every mode reads the trigger tags, and the anchor counts the ones it ignores.
        std::size_t done = 0UZ;
        for (const auto& [relIndex, mapRef] : inSpan.tags(nSamples)) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= nSamples) {
                continue;
            }
            const detail::TriggerRead read = detail::readTrigger(mapRef.get(), _anchor.honorTriggerOffset());
            if (!read.present) {
                continue;
            }
            const std::size_t at = static_cast<std::size_t>(relIndex);
            if (at > done) { // the call splits at the trigger, and the samples before it keep their anchor
                std::ignore = processBulk(std::span<const T>(inSpan.data() + done, at - done), std::span<T>(outSpan.data() + done, at - done));
                done        = at;
            }

            const gr::timing::ScheduleAnchor::Response response = read.readable ? _anchor.onTrigger(_position, read.timeNs, read.offsetSeconds) : _anchor.onUnreadableTrigger();
            if (response == gr::timing::ScheduleAnchor::Response::armed || response == gr::timing::ScheduleAnchor::Response::reanchored) {
                const auto clock = clockAt(settings.sample_rate, _anchor.anchorIndex(), _anchor.anchorNs());
                if (!clock) {
                    std::ignore = inSpan.consume(done);
                    outSpan.publish(done);
                    this->emitErrorMessage("processBulk", clock.error());
                    return work::Status::ERROR;
                }
                _clock = *clock;
                // Each anchor restarts the phase at zero. The output after a trigger then does not depend on the
                // stream before it.
                _phasor.setPhase(0.);
            }
        }
        if (done < nSamples) {
            std::ignore = processBulk(std::span<const T>(inSpan.data() + done, nSamples - done), std::span<T>(outSpan.data() + done, nSamples - done));
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        publishMeasurements(); // a call whose last segment ended on a tag has published nothing since that tag
        return work::Status::OK;
    }

private:
    /// The schedule's time axis at @p rate, with sample @p index at @p timeNs.
    [[nodiscard]] static std::expected<gr::timing::SampleClock, gr::Error> clockAt(float rate, std::uint64_t index, std::int64_t timeNs) { return gr::timing::clockForRateHz(static_cast<double>(rate), index, timeNs); }

    /// The table @p s names, before the direction's sign, or none for a stream that passes through; the reason when
    /// the file does not give one.
    [[nodiscard]] static std::expected<std::shared_ptr<const gr::timing::FrequencySchedule>, std::string> loadTable(const Settings& s) {
        const std::vector<std::int64_t>& times   = s.schedule_times_ns.value;
        const std::vector<double>&       offsets = s.schedule_offsets_hz.value;
        if (!s.schedule_file.value.empty()) {
            auto loaded = gr::timing::loadTrajectoryFile(s.schedule_file.value);
            if (!loaded) {
                return std::unexpected(loaded.error().message);
            }
            if (!loaded->frequency.has_value()) {
                return std::unexpected("must name a file with a frequency column");
            }
            return std::make_shared<const gr::timing::FrequencySchedule>(std::move(*loaded->frequency));
        }
        if (times.empty() && offsets.empty()) {
            return nullptr;
        }
        return std::make_shared<const gr::timing::FrequencySchedule>(std::span<const std::int64_t>(times), std::span<const double>(offsets));
    }

    /// The refusal of an offset at or past half of @p rate, naming its knot.
    [[nodiscard]] static std::optional<std::string> knotAboveNyquist(std::span<const double> offsets, float rate) {
        const double limit = 0.5 * static_cast<double>(rate);
        for (std::size_t i = 0UZ; i < offsets.size(); ++i) {
            if (std::abs(offsets[i]) >= limit) {
                return std::format("knot {}: must be < sample_rate / 2", i);
            }
        }
        return std::nullopt;
    }

    /// @p table in the sign of @p seat. 'correct' negates it once here, and the sample path is the same in both.
    [[nodiscard]] static std::shared_ptr<const gr::timing::FrequencySchedule> signedSchedule(const std::shared_ptr<const gr::timing::FrequencySchedule>& table, detail::DopplerDirection seat) {
        if (!table || seat == detail::DopplerDirection::Apply) {
            return table;
        }
        std::vector<double> negated(table->offsets().begin(), table->offsets().end());
        for (double& value : negated) {
            value = -value;
        }
        return std::make_shared<const gr::timing::FrequencySchedule>(table->times(), std::span<const double>(negated));
    }

    /// The schedule's offset at sample @p index, in the direction's sign. An unarmed block or one without a table
    /// shifts nothing.
    [[nodiscard]] double offsetAt(std::uint64_t index) const noexcept { return _schedule && _anchor.armed() ? _schedule->offsetAt(_clock.timeOf(index)) : 0.; }

    /// Where sample @p index stands against the schedule's span.
    [[nodiscard]] SchedulePosition positionAt(std::uint64_t index) const noexcept {
        if (!_anchor.armed()) {
            return SchedulePosition::Unarmed;
        }
        if (!_schedule) {
            return SchedulePosition::After;
        }
        const std::int64_t now = _clock.timeOf(index);
        if (now < _schedule->firstTime()) {
            return SchedulePosition::Before;
        }
        if (now > _schedule->lastTime()) {
            return SchedulePosition::After;
        }
        return SchedulePosition::Inside;
    }

    /// Publishes the block's current state for a reader on another thread. Runs once per call.
    void publishMeasurements() noexcept {
        _armed.store(_anchor.armed(), std::memory_order_relaxed);
        _anchorIndex.store(_anchor.anchorIndex(), std::memory_order_relaxed);
        _anchorNs.store(_anchor.anchorNs(), std::memory_order_relaxed);
        _nIgnoredAnchors.store(_anchor.nIgnoredAnchors(), std::memory_order_relaxed);
        _nReanchors.store(_anchor.nReanchors(), std::memory_order_relaxed);
        _nRefusedAnchors.store(_anchor.nRefusedAnchors(), std::memory_order_relaxed);
        _slot.publish({offsetAt(_position), static_cast<double>(static_cast<std::uint8_t>(positionAt(_position)))}, _position);
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP
