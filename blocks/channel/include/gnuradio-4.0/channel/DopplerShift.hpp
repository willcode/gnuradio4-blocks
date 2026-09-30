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

/// The position of the stream against the schedule's time span. `Unarmed` means a trigger-anchored
/// block has not seen its trigger yet. The stream then has no time and no position.
enum class SchedulePosition : std::uint8_t { Before = 0, Inside, After, Unarmed };

namespace detail {

/// The direction of the schedule. `apply` puts the trajectory's shift on a clean signal. `correct` removes it.
enum class DopplerDirection : std::uint8_t { Apply = 0, Correct };

/// The direction @p name selects. A refusal message starts with @p block.
[[nodiscard]] inline DopplerDirection parseDopplerDirection(std::string_view name, std::string_view block) {
    if (name == "apply") {
        return DopplerDirection::Apply;
    }
    if (name == "correct") {
        return DopplerDirection::Correct;
    }
    throw gr::exception(std::format("{}: 'direction' must be 'apply' or 'correct', got '{}'", block, name));
}

/// The anchor mode @p name selects. A refusal names the three modes.
[[nodiscard]] inline gr::timing::AnchorSource parseAnchorSource(std::string_view name, std::string_view block) {
    const std::optional<gr::timing::AnchorSource> source = gr::timing::anchorSourceFrom(name);
    if (!source.has_value()) {
        throw gr::exception(std::format("{}: 'anchor_source' must be 'setting', 'first_trigger' or 'every_trigger', got '{}'", block, name));
    }
    return *source;
}

/// A trigger tag's time and offset, read with the reserved keys' types. A tag without a time is no anchor event.
struct TriggerRead {
    std::uint64_t timeNs{0ULL};
    float         offsetSeconds{0.f};
    bool          present{false};  ///< the tag carries a `trigger_time` key
    bool          readable{false}; ///< the key, and `trigger_offset` when honored, carry the reserved types
};

/// @brief Reads a tag's anchor event with the reserved keys' types and substitutes no default.
///
/// A `trigger_time` that is not a `std::uint64_t` is a time the block cannot read. Zero in its place would
/// anchor the pass at the Unix epoch and move the whole schedule silently. The key is reported present and
/// unreadable, and the caller counts it as a refusal.
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
 * @brief Applies a trajectory's Doppler frequency shift to a stream, or removes it.
 *
 * A satellite pass arrives as a few `(time, offset)` points. The block turns them into one phase increment per
 * sample. `gr::timing::FrequencySchedule` does that mapping. It is piecewise linear between knots and holds the end
 * values outside them. It integrates across each sample's interval, and the accumulated phase is the exact integral
 * of the schedule. A coherent demodulator tracks the phase. The exact phase is therefore the kernel's contract.
 *
 * The block does no orbit propagation. TLEs, SGP4 and station geometry are the job of the trajectory's producer.
 * `gr::timing::offsetFor(v_radial, f_carrier)` converts a radial velocity to a schedule offset. A closing pass reads
 * high.
 *
 * `apply` models propagation. `correct` removes it with the same table and the sign flipped. The negation happens
 * once, when the table is built. `correct` costs the same as `apply`, and the two are exact inverses.
 *
 * A passing `gr::tag::FREQUENCY` tag is forwarded unchanged in both directions. A Doppler shift does not change the
 * frequency the stream is tuned to.
 *
 * Staging refuses an offset at or past `+/-sample_rate/2`, since such an offset aliases. The refusal names the knot.
 * The kernel refuses unpaired vectors, non-monotonic times and non-finite offsets, and names the knot too.
 *
 * A schedule replacement is a staged settings change. The block builds the new table and swaps it in whole. The
 * phasor keeps its phase, and the accumulated phase continues across the switch. Only the phase increment changes.
 * A phase reset would put a step in the middle of the stream.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct DopplerShift : gr::Block<DopplerShift<T>> {
    using Description = Doc<R""(
@brief Applies or removes a Doppler frequency schedule with one phase increment per sample.

The knots arrive as the paired `schedule_times_ns` and `schedule_offsets_hz` vectors. They can also come from
`schedule_file`, a `#!gr4-trajectory 1` file with an `offset_hz` or `range_rate_m_s` column. Supplying both refuses.
`direction` is 'apply' or 'correct'. An offset at or past `sample_rate/2` is refused, and the refusal names the knot.

An anchor places the schedule's time axis on the stream. With `anchor_source = 'setting'`, `anchor_index` and
`anchor_ns` give the anchor. With 'first_trigger', the first `trigger_time` tag arms it. Until then the stream
passes through unshifted and `schedulePosition()` reads Unarmed. With 'every_trigger', each trigger tag re-arms the
anchor and restarts the accumulated phase. A re-anchor moves the phase by an unbounded amount. A continued phase
would put a step in the signal. Under 'first_trigger', a second tag is ignored and counted. A `trigger_time` past
the nanosecond axis is refused and counted. With `honor_trigger_offset` set, `trigger_offset` moves the anchor by
its seconds. A passing `frequency` tag is forwarded unchanged. The shift moves the signal within the band and
leaves the tuning as it is.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::vector<std::int64_t>, "schedule_times_ns", Visible, Unit<"ns">, Doc<"strictly increasing knot times on the SampleClock axis, two or more">> schedule_times_ns{};
    Annotated<std::vector<double>, "schedule_offsets_hz", Visible, Unit<"Hz">, Doc<"knot offsets, one per time, held at the end values outside">>              schedule_offsets_hz{};
    Annotated<std::string, "schedule_file", Doc<"#!gr4-trajectory 1 file with a frequency column">>                                                            schedule_file{};
    Annotated<std::string, "direction", Visible, Doc<"'apply' puts the schedule on the stream, 'correct' takes it off">>                                       direction            = std::string("apply");
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"stream sample rate">>                                                                            sample_rate          = 1.f;
    Annotated<std::uint64_t, "anchor_index", Doc<"the stream sample that anchor_ns belongs to, under anchor_source 'setting'">>                                anchor_index         = 0ULL;
    Annotated<std::int64_t, "anchor_ns", Unit<"ns">, Doc<"the schedule-axis time of anchor_index, under anchor_source 'setting'">>                             anchor_ns            = 0LL;
    Annotated<std::string, "anchor_source", Doc<"'setting', 'first_trigger' or 'every_trigger'">>                                                              anchor_source        = std::string("setting");
    Annotated<bool, "honor_trigger_offset", Doc<"move a trigger anchor by the tag's own trigger_offset seconds">>                                              honor_trigger_offset = true;

    GR_MAKE_REFLECTABLE(DopplerShift, in, out, schedule_times_ns, schedule_offsets_hz, schedule_file, direction, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset);

    std::shared_ptr<const gr::timing::FrequencySchedule> _schedule{}; ///< swapped whole, so a replacement is atomic
    gr::timing::SampleClock                              _clock{};
    gr::timing::ScheduleAnchor                           _anchor{};
    gr::signal::Phasor<float>                            _phasor{};
    std::vector<double>                                  _increments{};   ///< increments for one call, resized only upward
    std::uint64_t                                        _position{0ULL}; ///< index of the next sample, counted from the stream's own start
    detail::DopplerDirection                             _direction{detail::DopplerDirection::Apply};

    // The scheduler thread writes the observables, and a caller reads them from its own thread. The doubles cross
    // through the seqlock and the integers through their own atomics. An anchor time is nanoseconds since the epoch,
    // around 1.7e18 today. That is past 2^53, the largest integer a double holds exactly. The anchor is therefore an
    // integer atomic. A slot value would round it by hundreds of nanoseconds.
    gr::measurement::MeasurementSlot<2UZ> _slot{}; ///< offset in Hz and schedule position, with the stream position as fill count
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
            _schedule.reset(); // without a table the stream passes through, and a table can be staged later
        } else {
            stageSchedule(std::span<const std::int64_t>(times), std::span<const double>(offsets));
        }

        // A settings change keeps the phase and the stream position. The anchor is rebuilt, and a trigger-armed
        // anchor waits for its trigger again.
        publishMeasurements();
    }

    void start() {
        _position = 0ULL;
        _phasor.setPhase(0.);
        _anchor.reset(); // a new stream carries no history of the last one's triggers, and a trigger anchor waits again
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

        // Every mode reads the trigger tags. An ignored trigger is counted. The count shows a tagged source wired
        // against a hand-set anchor.
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
                _clock = clockAt(_anchor.anchorIndex(), _anchor.anchorNs());
                _phasor.setPhase(0.); // the schedule time jumped, and a continued phase would put a step in the signal
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
        // `correct` negates the table once here. The sample path is the same in both directions.
        std::vector<double> applied(offsets.begin(), offsets.end());
        if (_direction == detail::DopplerDirection::Correct) {
            for (double& value : applied) {
                value = -value;
            }
        }
        _schedule = std::make_shared<const gr::timing::FrequencySchedule>(times, std::span<const double>(applied));
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
