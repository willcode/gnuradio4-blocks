#ifndef GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP
#define GNURADIO_CHANNEL_DOPPLER_SHIFT_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
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

/// Where the stream stands against the schedule's own time span. `Unarmed` is a trigger-anchored
/// block that has not seen its trigger yet: the stream has no time, so it has no position either.
enum class SchedulePosition : std::uint8_t { Before = 0, Inside, After, Unarmed };

namespace detail {

/// Which way the schedule is taken: `apply` puts the trajectory's shift on a clean signal, `correct` removes it.
enum class DopplerDirection : std::uint8_t { Apply = 0, Correct };

/// The seat @p name selects, with the refusal opening on the @p block that raised it.
[[nodiscard]] inline DopplerDirection parseDopplerDirection(std::string_view name, std::string_view block) {
    if (name == "apply") {
        return DopplerDirection::Apply;
    }
    if (name == "correct") {
        return DopplerDirection::Correct;
    }
    throw gr::exception(std::format("{}: 'direction' must be 'apply' or 'correct', got '{}'", block, name));
}

/// The anchor mode @p name selects, with the refusal naming the three there are.
[[nodiscard]] inline gr::timing::AnchorSource parseAnchorSource(std::string_view name, std::string_view block) {
    const std::optional<gr::timing::AnchorSource> source = gr::timing::anchorSourceFrom(name);
    if (!source.has_value()) {
        throw gr::exception(std::format("{}: 'anchor_source' must be 'setting', 'first_trigger' or 'every_trigger', got '{}'", block, name));
    }
    return *source;
}

/// A trigger tag's time and offset, read with the reserved keys' own types; no time means no anchor event.
struct TriggerRead {
    std::uint64_t timeNs{0ULL};
    float         offsetSeconds{0.f};
    bool          present{false};  ///< the tag carries a `trigger_time` key
    bool          readable{false}; ///< and that key, with `trigger_offset` where it is honored, carries the reserved type
};

/// @brief Read a tag's anchor event with the reserved keys' own types, never through a substituted default.
///
/// A `trigger_time` that holds something other than a `std::uint64_t` states a time this block cannot read.
/// Taking it as zero would anchor the pass at the Unix epoch and translate the whole schedule silently, so the
/// key is reported present and unreadable and the caller counts it as the refusal it is.
[[nodiscard]] inline TriggerRead readTrigger(const property_map& map, bool honorOffset) {
    TriggerRead read;
    const auto  time = map.find(property_map::key_type(gr::tag::TRIGGER_TIME.shortKey()));
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
        if (const auto offset = map.find(property_map::key_type(gr::tag::TRIGGER_OFFSET.shortKey())); offset != map.end()) {
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

/**
 * @brief The frequency shift a trajectory puts on a stream, or the same shift taken back off it.
 *
 * A satellite pass arrives as a handful of `(time, offset)` points and has to become one phase increment per sample.
 * `gr::timing::FrequencySchedule` is that map — piecewise linear between knots, holding the end values outside them,
 * and integrated across each sample's own interval so the accumulated phase is the schedule's true integral rather
 * than a sampled approximation. A coherent demodulator downstream rides the phase, not the frequency, which is why
 * that distinction is the kernel's contract and not a refinement.
 *
 * Orbit propagation stays outside. TLEs, SGP4 and station geometry belong to whatever produced the trajectory;
 * `gr::timing::offsetFor(v_radial, f_carrier)` is the one map from the common physical statement to a schedule knot,
 * and a closing pass reads high.
 *
 * One block, both seats. `apply` models propagation and `correct` removes it — the same table with the sign flipped,
 * which is the argument `IqImbalance` makes for the transmitter and receiver seats of one impairment. The negation
 * happens once, when the table is built, so `correct` costs exactly what `apply` costs and the two are each other's
 * inverse by construction.
 *
 * Like `FrequencyOffset`, a passing `gr::tag::FREQUENCY` is forwarded **untouched** in both directions: applying a
 * pass's Doppler models propagation and correcting it removes that model, and neither changes what the stream is
 * nominally tuned to. `Rotator` retunes the tag because retuning on purpose is what it is for; these two do not.
 *
 * Validation refuses what cannot be meant: an offset at or past `+/-sample_rate/2` would alias rather than shift, and
 * it is refused at staging with the offending knot named rather than wrapped silently. The kernel refuses the rest —
 * unpaired vectors, non-monotonic times, non-finite offsets — and names the knot too.
 *
 * A schedule replacement is a staged settings change: the new table is built and swapped in whole, and the phasor is
 * never re-anchored, so the accumulated phase runs straight through the switch. What changes at the switch is the
 * increment, which is what a frequency change is; a block that reset the phase would put a step discontinuity in the
 * middle of the stream instead.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct DopplerShift : gr::Block<DopplerShift<T>> {
    using Description = Doc<R""(
@brief Applies or corrects a Doppler frequency schedule: piecewise-linear knots in, one phase increment per sample.

The table arrives as the paired `schedule_times_ns` / `schedule_offsets_hz` vectors, or as `schedule_file` naming a
`#!gr4-trajectory 1` file whose `offset_hz` or `range_rate_m_s` column supplies the same knots — two spellings of one
input, so supplying both refuses. `direction` is 'apply' or 'correct'; an offset at or past `sample_rate/2` is
refused naming the knot.

The schedule's time axis meets the stream at an anchor. `anchor_source = 'setting'` reads it from `anchor_index` /
`anchor_ns`; 'first_trigger' arms it from the first `trigger_time` tag that passes — until then the stream passes
through unshifted and `schedulePosition()` reads Unarmed — and 'every_trigger' re-arms on each one, restarting the
accumulated phase at the new anchor, because a re-anchor moves the phase by an unbounded amount and continuing it
would be a step pretending to be a curve. A second tag under 'first_trigger' is ignored and counted; a
`trigger_time` past the nanosecond axis is refused and counted. `trigger_offset` moves the anchor by its own seconds
when `honor_trigger_offset` is set. A passing `frequency` tag is forwarded unchanged: the shift moves the signal
within the band, not the band.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::vector<std::int64_t>, "schedule_times_ns", Visible, Unit<"ns">, Doc<"knot times on the SampleClock axis; strictly increasing, at least two">>          schedule_times_ns{};
    Annotated<std::vector<double>, "schedule_offsets_hz", Visible, Unit<"Hz">, Doc<"knot offsets, one per time; held at the end values outside the table">>               schedule_offsets_hz{};
    Annotated<std::string, "schedule_file", Doc<"a #!gr4-trajectory 1 file carrying a frequency column; an alternative to the paired vectors, and staging both refuses">> schedule_file{};
    Annotated<std::string, "direction", Visible, Doc<"'apply' puts the schedule on the stream, 'correct' takes it off">>                                                  direction            = std::string("apply");
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"stream sample rate">>                                                                                       sample_rate          = 1.f;
    Annotated<std::uint64_t, "anchor_index", Doc<"the stream sample that anchor_ns belongs to, under anchor_source 'setting'">>                                           anchor_index         = 0ULL;
    Annotated<std::int64_t, "anchor_ns", Unit<"ns">, Doc<"the schedule-axis time of anchor_index, under anchor_source 'setting'">>                                        anchor_ns            = 0LL;
    Annotated<std::string, "anchor_source", Doc<"'setting' (the pair above), 'first_trigger' (armed once by a trigger_time tag) or 'every_trigger' (re-armed by each)">>  anchor_source        = std::string("setting");
    Annotated<bool, "honor_trigger_offset", Doc<"move a trigger anchor by the tag's own trigger_offset seconds">>                                                         honor_trigger_offset = true;

    GR_MAKE_REFLECTABLE(DopplerShift, in, out, schedule_times_ns, schedule_offsets_hz, schedule_file, direction, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset);

    std::shared_ptr<const gr::timing::FrequencySchedule> _schedule{}; ///< swapped whole, so a replacement is atomic
    gr::timing::SampleClock                              _clock{};
    gr::timing::ScheduleAnchor                           _anchor{};
    gr::signal::Phasor<float>                            _phasor{};
    std::vector<double>                                  _increments{};   ///< one call's worth, grown rather than rebuilt
    std::uint64_t                                        _position{0ULL}; ///< index of the next sample, counted from the stream's own start
    detail::DopplerDirection                             _direction{detail::DopplerDirection::Apply};

    // The observables a caller polls belong to whatever thread does the polling, while the scheduler thread writes
    // them; the doubles cross that boundary through the seqlock and the integers through their own atomics. An
    // anchor time is nanoseconds since the epoch — around 1.7e18 today, past the 2^53 a double holds exactly — so it
    // is an integer rather than a slot value, which would round the anchor by hundreds of nanoseconds.
    gr::measurement::MeasurementSlot<2UZ> _slot{}; ///< offset Hz, schedule position; the fill count is the stream position
    std::atomic<bool>                     _armed{true};
    std::atomic<std::uint64_t>            _anchorIndex{0ULL};
    std::atomic<std::int64_t>             _anchorNs{0LL};
    std::atomic<std::uint64_t>            _nIgnoredAnchors{0ULL};
    std::atomic<std::uint64_t>            _nReanchors{0ULL};
    std::atomic<std::uint64_t>            _nRefusedAnchors{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("DopplerShift: 'sample_rate' must be positive and finite, got {}", sample_rate.value));
        }
        _direction = detail::parseDopplerDirection(direction, "DopplerShift");
        _anchor    = gr::timing::ScheduleAnchor(detail::parseAnchorSource(anchor_source.value, "DopplerShift"), anchor_index.value, anchor_ns.value, honor_trigger_offset.value);
        if (_anchor.armed()) {
            _clock = clockAt(_anchor.anchorIndex(), _anchor.anchorNs());
        } else {
            std::ignore = clockAt(0ULL, 0LL); // the rate is validated even while the anchor waits for its trigger
        }

        const std::vector<std::int64_t>& times   = schedule_times_ns.value;
        const std::vector<double>&       offsets = schedule_offsets_hz.value;
        if (!schedule_file.value.empty() && !(times.empty() && offsets.empty())) {
            throw gr::exception("DopplerShift: 'schedule_file' and the paired 'schedule_times_ns'/'schedule_offsets_hz' are two spellings of one table, and staging both leaves two answers to one question");
        }

        if (!schedule_file.value.empty()) {
            gr::timing::Trajectory trajectory;
            try {
                trajectory = gr::timing::loadTrajectoryFile(schedule_file.value);
            } catch (const std::invalid_argument& refusal) {
                throw gr::exception(std::format("DopplerShift: 'schedule_file': {}", refusal.what()));
            }
            if (!trajectory.frequency.has_value()) {
                throw gr::exception(std::format("DopplerShift: '{}' carries no frequency — its columns name neither offset_hz nor range_rate_m_s, and a delay table cannot be differentiated into one", schedule_file.value));
            }
            stageSchedule(trajectory.frequency->times(), trajectory.frequency->offsets());
        } else if (times.empty() && offsets.empty()) {
            _schedule.reset(); // no table is a passthrough, not an error: a graph may stage the pass later
        } else {
            stageSchedule(std::span<const std::int64_t>(times), std::span<const double>(offsets));
        }

        // Neither the phase nor the stream position is touched by a settings change: a new table changes the
        // increment, which is what a frequency change is, and re-anchoring either would put a step in the middle of
        // the stream. The anchor machinery itself is rebuilt, so a trigger-armed anchor waits for its trigger again.
        publishMeasurements();
    }

    void start() {
        _position = 0ULL;
        _phasor.setPhase(0.);
        _anchor.reset(); // a new stream carries no history of the last one's triggers, and a trigger anchor waits again
        publishMeasurements();
    }

    /// @brief The offset the block is applying to the stream right now, in Hz — negated in `correct`. Any thread.
    [[nodiscard]] double currentOffsetHz() const noexcept { return _slot.read().first[0]; }

    /// @brief Where the next sample stands against the schedule's span; `Before`/`After` are where the ends hold.
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

        // Grown, not rebuilt: the buffer reaches the largest chunk the scheduler ever hands over and stays there, so
        // no call after the first of a given size allocates and no sample ever does.
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

        // Every mode reads the trigger tags, because ignoring one is a counted event and a graph wired with a
        // tagged source against a hand-set anchor can only be seen to be wired that way if the count happens.
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
            if (at > done) { // the samples before the trigger run under the anchor they arrived under, so the call is cut before the rule is fed
                std::ignore = processBulk(std::span<const T>(inSpan.data() + done, at - done), std::span<T>(outSpan.data() + done, at - done));
                done        = at;
            }

            const gr::timing::ScheduleAnchor::Response response = read.readable ? _anchor.onTrigger(_position, read.timeNs, read.offsetSeconds) : _anchor.onUnreadableTrigger();
            if (response == gr::timing::ScheduleAnchor::Response::armed || response == gr::timing::ScheduleAnchor::Response::reanchored) {
                _clock = clockAt(_anchor.anchorIndex(), _anchor.anchorNs());
                _phasor.setPhase(0.); // the schedule's time jumped, and continuing the phase would be a step pretending to be a curve
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
    /// The schedule's exact time axis at this block's rate, anchored where the anchor machinery says.
    [[nodiscard]] gr::timing::SampleClock clockAt(std::uint64_t index, std::int64_t timeNs) const {
        try {
            return gr::timing::clockForRateHz(static_cast<double>(sample_rate.value), index, timeNs);
        } catch (const std::invalid_argument& refusal) {
            throw gr::exception(std::format("DopplerShift: 'sample_rate': {}", refusal.what()));
        }
    }

    void stageSchedule(std::span<const std::int64_t> times, std::span<const double> offsets) {
        const double limit = 0.5 * static_cast<double>(sample_rate);
        for (std::size_t i = 0UZ; i < offsets.size(); ++i) {
            if (std::abs(offsets[i]) >= limit) {
                throw gr::exception(std::format("DopplerShift: knot {} offsets {} Hz, at or past the +/-{} Hz a {} Hz stream can carry — that is an alias, not a shift", i, offsets[i], limit, static_cast<double>(sample_rate)));
            }
        }
        // `correct` is the same table with the sign flipped, negated once here so the sample path is identical in
        // both seats and the two are exact inverses of one another
        std::vector<double> applied(offsets.begin(), offsets.end());
        if (_direction == detail::DopplerDirection::Correct) {
            for (double& value : applied) {
                value = -value;
            }
        }
        _schedule = std::make_shared<const gr::timing::FrequencySchedule>(times, std::span<const double>(applied));
    }

    /// The offset the schedule states at sample @p index, in the seat's own sign; an unarmed or tableless block shifts nothing.
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

    /// Once per call, never per sample: what the block is doing right now, for a reader on another thread.
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
