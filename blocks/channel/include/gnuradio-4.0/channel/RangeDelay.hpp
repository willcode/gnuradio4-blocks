#ifndef GNURADIO_CHANNEL_RANGE_DELAY_HPP
#define GNURADIO_CHANNEL_RANGE_DELAY_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/filter/FractionalDelay.hpp>
#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>
#include <gnuradio-4.0/algorithm/timing/SampleClock.hpp>
#include <gnuradio-4.0/algorithm/timing/ScheduleAnchor.hpp>
#include <gnuradio-4.0/algorithm/timing/TrajectoryFile.hpp>

#include <gnuradio-4.0/channel/DopplerShift.hpp>

namespace gr::blocks::channel {

GR_REGISTER_BLOCK(gr::blocks::channel::RangeDelay, [T], [ std::complex<float>, float ])

/**
 * @brief The envelope delay a trajectory puts on a stream, or the same delay taken back off it.
 *
 * A receiver at slant range `R(t)` sees the signal `R(t)/c` seconds late, and that envelope delay is the second
 * factor of the same propagation whose first factor is the Doppler shift `DopplerShift` carries: the baseband
 * equivalent of a moving source is `x(t - tau(t)) * exp(-j 2 pi f_c tau(t))`, and this block is the `x(t - tau(t))`
 * half. `DopplerShift` and `RangeDelay` fed from one trajectory are consistent by construction and compose in that
 * order; a ten-minute low-orbit pass moves the delay by about fourteen milliseconds, hundreds of samples at a
 * baseband rate, which is why a chain that models the shift without the delay is missing a real effect rather than
 * a refinement.
 *
 * The table is a `(time, delay)` schedule — `schedule_times_ns` with `schedule_delays_s`, or a `#!gr4-trajectory 1`
 * file whose `range_m` column supplies `range/c` per knot. It is piecewise linear between knots and held at the
 * ends, and the loader never differentiates a delay table into a frequency one or the other way: those are separate
 * inputs to separate blocks.
 *
 * A delay line is causal, so it can only reach into the past. `apply` puts the trajectory's delay on a clean signal
 * and `correct` removes it, and `correct` — which would command a negative delay where the trajectory delayed the
 * signal least — is made causal by a `bias_s` that shifts the whole schedule into the reachable past; `apply`
 * defaults it to zero, `correct` to the schedule's own maximum. A commanded delay outside the fractional-delay
 * line's reach, or a schedule whose slope reaches one sample per sample, is refused at staging naming the knot.
 *
 * The anchor and the trigger machinery are `DopplerShift`'s: the schedule's time meets the stream at
 * `anchor_source` in {setting, first_trigger, every_trigger}, honoring `trigger_offset` where set, ignoring a
 * second first-trigger tag and counting it, refusing a trigger time past the nanosecond axis. Until the anchor is
 * armed the stream passes through unmodified and the line is not fed, so a trigger-anchored block starts its delay
 * from silence and its first `historySamples()` outputs are the filter's transient over that silence.
 *
 * A tag rides with the sample it marks: the reserved stream vocabulary stays at the index it arrived on, and every
 * other tag is emitted on the first output sample whose whole read position has reached the tag's input index, in
 * the same fixed point the read cursor itself is kept in.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>> || std::is_same_v<T, float>
struct RangeDelay : gr::Block<RangeDelay<T>, gr::NoTagPropagation> {
    using Description = Doc<R""(
@brief Applies or corrects a range-delay schedule: piecewise-linear (time, delay) knots in, a fractionally delayed
stream out, one sample per sample.

The delay is a `SampleClock`-timed schedule, `schedule_times_ns` with `schedule_delays_s`, or a #!gr4-trajectory 1
`schedule_file` carrying a range column. 'apply' delays the signal, 'correct' advances it; a delay line is causal,
so 'correct' biases the schedule into the past by its own maximum unless `bias_s` says otherwise. A slope of one
sample per sample, or a delay past the line's reach, is refused naming the knot. A tag that is not part of the
reserved stream vocabulary moves with the sample it marks.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::vector<std::int64_t>, "schedule_times_ns", Visible, Unit<"ns">, Doc<"knot times on the SampleClock axis; strictly increasing, at least two">>                        schedule_times_ns{};
    Annotated<std::vector<double>, "schedule_delays_s", Visible, Unit<"s">, Doc<"knot delays in seconds, one per time; held at the end values outside the table">>                      schedule_delays_s{};
    Annotated<std::string, "schedule_file", Doc<"a #!gr4-trajectory 1 file carrying a range column; an alternative to the paired vectors, and staging both refuses">>                   schedule_file{};
    Annotated<std::string, "direction", Visible, Doc<"'apply' delays the signal by the schedule, 'correct' advances it by the schedule">>                                               direction            = std::string("apply");
    Annotated<double, "bias_s", Unit<"s">, Doc<"a constant delay added to the schedule to keep the commanded delay non-negative; NaN derives it (0 for apply, max delay for correct)">> bias_s               = std::numeric_limits<double>::quiet_NaN();
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"stream sample rate">>                                                                                                     sample_rate          = 1.f;
    Annotated<std::uint64_t, "anchor_index", Doc<"the stream sample that anchor_ns belongs to, under anchor_source 'setting'">>                                                         anchor_index         = 0ULL;
    Annotated<std::int64_t, "anchor_ns", Unit<"ns">, Doc<"the schedule-axis time of anchor_index, under anchor_source 'setting'">>                                                      anchor_ns            = 0LL;
    Annotated<std::string, "anchor_source", Doc<"'setting', 'first_trigger' or 'every_trigger'">>                                                                                       anchor_source        = std::string("setting");
    Annotated<bool, "honor_trigger_offset", Doc<"move a trigger anchor by the tag's own trigger_offset seconds">>                                                                       honor_trigger_offset = true;
    Annotated<gr::Size_t, "bank_size", Doc<"polyphase arms; 0 derives it from attenuation_db, otherwise a power of two up to 65536">>                                                   bank_size            = 0U;
    Annotated<int, "order", Doc<"interpolation order between arms: 0, 1 or 3">>                                                                                                         order                = 1;
    Annotated<double, "attenuation_db", Unit<"dB">, Doc<"stopband attenuation the derived bank meets, in [20, 140]">>                                                                   attenuation_db       = 60.0;
    Annotated<double, "rolloff", Doc<"the design's excess bandwidth, in (0, 1)">>                                                                                                       rolloff              = 0.2;

    GR_MAKE_REFLECTABLE(RangeDelay, in, out, schedule_times_ns, schedule_delays_s, schedule_file, direction, bias_s, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset, bank_size, order, attenuation_db, rolloff);

    std::optional<gr::timing::DelaySchedule>            _schedule{};
    std::optional<gr::filter::FractionalDelayLine<T>>   _line{};
    gr::timing::SampleClock                             _clock{};
    gr::timing::ScheduleAnchor                          _anchor{};
    std::uint64_t                                       _position{0ULL};
    detail::DopplerDirection                            _direction{detail::DopplerDirection::Apply};
    double                                              _bias{0.};
    std::int64_t                                        _latencyQ32{0LL}; ///< the line's own lag in the fixed point the tag map reads
    std::vector<double>                                 _delays{};        ///< one call's worth of commanded delays, grown not rebuilt
    std::vector<std::uint64_t>                          _delaysQ32{};
    std::vector<std::pair<std::uint64_t, property_map>> _tagsMoving{}; ///< by absolute input index, ascending: their output sample is not here yet
    std::vector<std::pair<std::uint64_t, property_map>> _tagsHeld{};   ///< the same, for the tags that leave on the sample they arrived on

    /// The bank the line was cut for. A table, a rate or an anchor leaves it alone and keeps the history the line holds.
    std::size_t _builtBank{0UZ};
    int         _builtOrder{0};
    double      _builtRolloff{0.};
    double      _builtAttenuationDb{0.};

    gr::measurement::MeasurementSlot<6UZ> _slot{}; ///< delay s, delay samples, bias s, latency, group delay, position
    std::atomic<std::uint64_t>            _nTagsCoalesced{0ULL};
    std::atomic<std::uint64_t>            _nBankRebuilds{0ULL};
    std::atomic<std::uint64_t>            _nDiscontinuities{0ULL};
    std::atomic<std::uint64_t>            _nRateDisagreements{0ULL};
    // The anchor's state, mirrored for a reader on another thread: an anchor time is a nanosecond count past what a
    // double holds exactly, so it travels as an integer beside the slot rather than inside it.
    std::atomic<bool>          _armed{false};
    std::atomic<std::uint64_t> _anchorIndex{0ULL};
    std::atomic<std::int64_t>  _anchorNs{0LL};
    std::atomic<std::uint64_t> _nIgnoredAnchors{0ULL};
    std::atomic<std::uint64_t> _nReanchors{0ULL};
    std::atomic<std::uint64_t> _nRefusedAnchors{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("RangeDelay: 'sample_rate' must be positive and finite, got {}", sample_rate.value));
        }
        if (order.value != 0 && order.value != 1 && order.value != 3) {
            throw gr::exception(std::format("RangeDelay: 'order' is the interpolation order and must be 0, 1 or 3, got {}", order.value));
        }
        if (!(attenuation_db.value >= 20.0 && attenuation_db.value <= 140.0)) {
            throw gr::exception(std::format("RangeDelay: 'attenuation_db' must be in [20, 140], got {}", attenuation_db.value));
        }
        if (!(rolloff.value > 0.0 && rolloff.value < 1.0)) {
            throw gr::exception(std::format("RangeDelay: 'rolloff' must be in (0, 1), got {}", rolloff.value));
        }

        // Everything is built and validated into locals first, and installed only once nothing can still refuse:
        // a change the block turns down leaves the schedule, the bank and the history it was already running on.
        const detail::DopplerDirection   seat = detail::parseDopplerDirection(direction.value, "RangeDelay");
        const gr::timing::ScheduleAnchor anchor(detail::parseAnchorSource(anchor_source.value, "RangeDelay"), anchor_index.value, anchor_ns.value, honor_trigger_offset.value);
        const gr::timing::SampleClock    clock  = clockAt(anchor.armed() ? anchor.anchorIndex() : 0ULL, anchor.armed() ? anchor.anchorNs() : 0LL);
        const std::size_t                bank   = resolveBankSize();
        Staged                           staged = stage(seat);

        const bool held    = _line.has_value();
        const bool rebuild = !held || bank != _builtBank || order.value != _builtOrder || rolloff.value != _builtRolloff || attenuation_db.value != _builtAttenuationDb;
        if (rebuild) {
            // The window length follows the prototype, so samples the line holds would not line up with the taps
            // that read them; the bank is the one change that costs the history, and the count says how often.
            const gr::filter::ResamplerDesign  design = gr::filter::designFractionalDelay(bank, rolloff.value, attenuation_db.value);
            gr::filter::FractionalDelayLine<T> built(bank, order.value, std::span<const float>(design.taps), staged.wholeDelaySamples);
            _line               = std::move(built);
            _builtBank          = bank;
            _builtOrder         = order.value;
            _builtRolloff       = rolloff.value;
            _builtAttenuationDb = attenuation_db.value;
            if (held) { // cutting the first bank is not a rebuild: there was no history to lose
                _nBankRebuilds.fetch_add(1ULL, std::memory_order_relaxed);
            }
        } else {
            _line->growMaxDelay(staged.wholeDelaySamples);
        }

        _schedule   = std::move(staged.schedule);
        _bias       = staged.bias;
        _direction  = seat;
        _anchor     = anchor;
        _clock      = clock;
        _latencyQ32 = static_cast<std::int64_t>(std::llround(_line->latencySamples() * static_cast<double>(gr::filter::kArbitraryOne)));
        publishMeasurements();
    }

    void start() {
        _position = 0ULL;
        _anchor.reset();
        if (_line) {
            _line->reset();
        }
        _tagsMoving.clear();
        _tagsHeld.clear();
        _nTagsCoalesced.store(0ULL, std::memory_order_relaxed);
        _nDiscontinuities.store(0ULL, std::memory_order_relaxed);
        _nRateDisagreements.store(0ULL, std::memory_order_relaxed);
        publishMeasurements();
    }

    /// @brief The delay commanded right now, in seconds — the schedule plus the bias, negated in `correct`. Any thread.
    [[nodiscard]] double currentDelaySeconds() const noexcept { return _slot.read().first[0]; }
    /// @brief The same delay in samples at the block's own rate. Any thread.
    [[nodiscard]] double currentDelaySamples() const noexcept { return _slot.read().first[1]; }
    /// @brief The constant the schedule is shifted by, derived under `correct` unless `bias_s` states it. Any thread.
    [[nodiscard]] double biasSeconds() const noexcept { return _slot.read().first[2]; }
    /// @brief The output's lag behind the input at a commanded delay of zero: the prototype's own plus the bank's one sample.
    [[nodiscard]] double latencySamples() const noexcept { return _slot.read().first[3]; }
    /// @brief `(N-1)/(2L)` input samples, the prototype's own, which `latencySamples()` carries one more than.
    [[nodiscard]] double groupDelaySamples() const noexcept { return _slot.read().first[4]; }
    /// @brief Where the next sample stands against the schedule's span; `Before`/`After` are where the ends hold.
    [[nodiscard]] SchedulePosition schedulePosition() const noexcept { return static_cast<SchedulePosition>(static_cast<std::uint8_t>(_slot.read().first[5])); }
    /// @brief The samples the line holds, which is the transient a fresh or re-anchored line emits over silence.
    [[nodiscard]] std::size_t historySamples() const noexcept { return _line ? _line->historySamples() : 0UZ; }

    /// @brief The next sample's index counted from the stream's start, which the anchor places in time.
    [[nodiscard]] std::uint64_t position() const noexcept { return _position; }
    [[nodiscard]] bool          anchorArmed() const noexcept { return _armed.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t anchorIndex() const noexcept { return _anchorIndex.load(std::memory_order_relaxed); }
    [[nodiscard]] std::int64_t  anchorNs() const noexcept { return _anchorNs.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nIgnoredAnchors() const noexcept { return _nIgnoredAnchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nReanchors() const noexcept { return _nReanchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedAnchors() const noexcept { return _nRefusedAnchors.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nTagsCoalesced() const noexcept { return _nTagsCoalesced.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nBankRebuilds() const noexcept { return _nBankRebuilds.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDiscontinuities() const noexcept { return _nDiscontinuities.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRateDisagreements() const noexcept { return _nRateDisagreements.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        const std::size_t nSamples = std::min(input.size(), output.size());
        reserve(nSamples);
        std::ignore = runSegment(input.first(nSamples), output.first(nSamples), 0UZ);
        publishMeasurements();
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t   nSamples  = std::min(inSpan.size(), outSpan.size());
        const std::uint64_t callStart = _position;
        reserve(nSamples);

        // The reserved vocabulary describes the stream rather than a point in it, so it leaves on the sample it
        // arrived on; everything else leaves on the output sample that carries its input sample. Both go through
        // the same walk over the output, because that walk is what puts them out in the order they are in.
        _tagsHeld.clear();
        for (const auto& [relIndex, mapRef] : inSpan.tags(nSamples)) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= nSamples) {
                continue;
            }
            const property_map& map = mapRef.get();
            const std::size_t   at  = static_cast<std::size_t>(relIndex);
            countStreamKeys(map);
            if (namesReservedKey(map)) {
                _tagsHeld.emplace_back(callStart + at, map);
            } else {
                _tagsMoving.emplace_back(callStart + at, map);
            }
        }

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
            advance(inSpan, outSpan, done, at); // the samples before the trigger run under the anchor they arrived under, so the call is cut before the rule is fed
            done = at;

            const gr::timing::ScheduleAnchor::Response response = read.readable ? _anchor.onTrigger(_position, read.timeNs, read.offsetSeconds) : _anchor.onUnreadableTrigger();
            if (response == gr::timing::ScheduleAnchor::Response::armed || response == gr::timing::ScheduleAnchor::Response::reanchored) {
                _clock = clockAt(_anchor.anchorIndex(), _anchor.anchorNs());
                if (response == gr::timing::ScheduleAnchor::Response::reanchored && _line) {
                    _line->reset(); // the schedule's time jumped, so what the line holds does not describe the stream that follows
                }
            }
        }
        advance(inSpan, outSpan, done, nSamples);

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        publishMeasurements();
        return work::Status::OK;
    }

private:
    /// A table and the constants that go with it, held apart from the block until every refusal has had its chance.
    struct Staged {
        std::optional<gr::timing::DelaySchedule> schedule{};
        double                                   bias{0.};
        std::uint64_t                            wholeDelaySamples{0ULL};
    };

    [[nodiscard]] gr::timing::SampleClock clockAt(std::uint64_t index, std::int64_t timeNs) const {
        try {
            return gr::timing::clockForRateHz(static_cast<double>(sample_rate.value), index, timeNs);
        } catch (const std::invalid_argument& refusal) {
            throw gr::exception(std::format("RangeDelay: 'sample_rate': {}", refusal.what()));
        }
    }

    [[nodiscard]] std::size_t resolveBankSize() const {
        if (bank_size.value == 0U) {
            return gr::filter::arbitraryBankSize(attenuation_db.value, rolloff.value, order.value);
        }
        const std::size_t bank = static_cast<std::size_t>(bank_size.value);
        if (bank > (1UZ << 16) || (bank & (bank - 1UZ)) != 0UZ) {
            throw gr::exception(std::format("RangeDelay: 'bank_size' must be a power of two up to 65536, got {}", bank_size.value));
        }
        return bank;
    }

    /// The largest delay the fixed point carries, in seconds at this rate, one sample short of it so that the
    /// history the line is sized to — the whole part of the delay, plus one — stays inside the same bound.
    [[nodiscard]] double reachSeconds() const noexcept { return static_cast<double>(gr::filter::kMaxFractionalDelaySamples - 1ULL) / static_cast<double>(sample_rate.value); }

    /// The whole samples of history a commanded delay of @p seconds needs: its whole part, plus the sample the
    /// fraction reaches into. Bounded by `reachSeconds()`, which every path checks before arriving here.
    [[nodiscard]] std::uint64_t wholeDelayFor(double seconds) const noexcept { return static_cast<std::uint64_t>(std::max(0., std::floor(seconds * static_cast<double>(sample_rate.value)) + 1.)); }

    [[nodiscard]] Staged stage(detail::DopplerDirection seat) const {
        const std::vector<std::int64_t>& times  = schedule_times_ns.value;
        const std::vector<double>&       delays = schedule_delays_s.value;
        if (!schedule_file.value.empty() && !(times.empty() && delays.empty())) {
            throw gr::exception("RangeDelay: 'schedule_file' and the paired 'schedule_times_ns'/'schedule_delays_s' are two spellings of one table, and staging both leaves two answers to one question");
        }

        if (!schedule_file.value.empty()) {
            gr::timing::Trajectory trajectory;
            try {
                trajectory = gr::timing::loadTrajectoryFile(schedule_file.value);
            } catch (const std::invalid_argument& refusal) {
                throw gr::exception(std::format("RangeDelay: 'schedule_file': {}", refusal.what()));
            }
            if (!trajectory.delay.has_value()) {
                throw gr::exception(std::format("RangeDelay: '{}' carries no delay — its columns name no range, and a frequency table cannot be integrated into one", schedule_file.value));
            }
            return adopt(std::move(*trajectory.delay), seat);
        }

        if (times.empty() && delays.empty()) {
            // No table is a constant delay of the bias alone, which the same reach and the same causality hold.
            Staged staged;
            staged.bias = std::isnan(bias_s.value) ? 0. : bias_s.value;
            if (!std::isfinite(staged.bias) || staged.bias < 0.) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s, and with no table it is the whole commanded delay, which a causal line takes non-negative and finite", staged.bias));
            }
            if (!(staged.bias < reachSeconds())) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s, at or past the {} s the fixed-point delay carries at {} Hz", staged.bias, reachSeconds(), static_cast<double>(sample_rate.value)));
            }
            staged.wholeDelaySamples = wholeDelayFor(staged.bias);
            return staged;
        }

        gr::timing::DelaySchedule schedule = [&] {
            try {
                return gr::timing::DelaySchedule(std::span<const std::int64_t>(times), std::span<const double>(delays));
            } catch (const std::invalid_argument& refusal) {
                throw gr::exception(std::format("RangeDelay: 'schedule_times_ns'/'schedule_delays_s': {}", refusal.what()));
            }
        }();
        return adopt(std::move(schedule), seat);
    }

    [[nodiscard]] Staged adopt(gr::timing::DelaySchedule schedule, detail::DopplerDirection seat) const {
        const double sign = seat == detail::DopplerDirection::Correct ? -1. : 1.;
        const double bias = resolveBias(schedule, seat);

        // the commanded delay is `sign * delay + bias`; it must stay in [0, the line's reach) at every knot, and its
        // slope must stay under one sample per sample or the read position would run backwards
        const double rate      = static_cast<double>(sample_rate.value);
        const double reach     = reachSeconds();
        const auto   knotTimes = schedule.times();
        const auto   knotVals  = schedule.values();
        for (std::size_t i = 0UZ; i < knotVals.size(); ++i) {
            const double commanded = sign * knotVals[i] + bias;
            if (!(commanded >= 0.)) {
                throw gr::exception(std::format("RangeDelay: knot {} commands a delay of {} s, which is negative — a delay line cannot reach the future; raise bias_s", i, commanded));
            }
            if (!(commanded < reach)) {
                throw gr::exception(std::format("RangeDelay: knot {} commands {} s, at or past the {} s the fixed-point delay carries at {} Hz", i, commanded, reach, rate));
            }
        }
        for (std::size_t i = 1UZ; i < knotVals.size(); ++i) {
            const double dtSeconds = static_cast<double>(knotTimes[i] - knotTimes[i - 1UZ]) * 1e-9;
            const double slope     = std::abs((knotVals[i] - knotVals[i - 1UZ]) / dtSeconds); // seconds of delay per second, dimensionless
            if (!(slope < 1.0)) {
                throw gr::exception(std::format("RangeDelay: segment {} changes delay by {} s per second, at or past one sample per sample — the read position would run backwards", i, slope));
            }
        }

        Staged staged;
        staged.bias = bias;
        // the largest delay this table ever commands, which is what the line must hold history for; a schedule
        // holds its end values, so that maximum is a property of the table rather than of the stream
        double worst = 0.;
        for (const double value : knotVals) {
            worst = std::max(worst, sign * value + bias);
        }
        staged.wholeDelaySamples = wholeDelayFor(worst);
        staged.schedule.emplace(std::move(schedule));
        return staged;
    }

    [[nodiscard]] double resolveBias(const gr::timing::DelaySchedule& schedule, detail::DopplerDirection seat) const {
        if (!std::isnan(bias_s.value)) {
            if (!std::isfinite(bias_s.value)) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s, which is not a delay", bias_s.value));
            }
            if (seat == detail::DopplerDirection::Apply && bias_s.value < 0.) {
                throw gr::exception(std::format("RangeDelay: 'bias_s' is {} s under 'apply', where the schedule is already non-negative and a negative bias only reaches the future", bias_s.value));
            }
            return bias_s.value;
        }
        // the default keeps the commanded delay causal: apply is already non-negative, so zero; correct commands
        // `-delay + bias`, least where the delay is greatest, so the schedule's maximum lifts that to zero
        return seat == detail::DopplerDirection::Correct ? schedule.maxValue() : 0.;
    }

    void reserve(std::size_t nSamples) {
        // Grown, not rebuilt: the buffers reach the largest chunk the scheduler ever hands over and stay there, so
        // no call after the first of a given size allocates and no sample ever does.
        if (_delays.size() < nSamples) {
            _delays.resize(nSamples);
            _delaysQ32.resize(nSamples);
        }
    }

    /// @brief Fill `[at, at + input.size())` of the call's delays and run the line over @p input; true where it filtered.
    [[nodiscard]] bool runSegment(std::span<const T> input, std::span<T> output, std::size_t at) {
        const std::size_t nSamples = input.size();
        if (nSamples == 0UZ) {
            return false;
        }
        if (!_line || !_anchor.armed()) {
            // An anchor that has not been armed places no sample in time, so there is no delay to command and
            // nothing the line could hold that would describe the stream: the input passes and the line waits.
            std::ranges::copy(input, output.begin());
            _position += nSamples;
            return false;
        }

        const std::span<double>        delays(_delays.data() + at, nSamples);
        const std::span<std::uint64_t> delaysQ32(_delaysQ32.data() + at, nSamples);
        if (_schedule) {
            _schedule->valuesFor(_clock, _position, delays);
            for (double& value : delays) {
                value = (_direction == detail::DopplerDirection::Correct ? -value : value) + _bias;
            }
        } else {
            std::ranges::fill(delays, _bias); // no table is a constant bias, which may be zero
        }
        gr::filter::fractionalDelayQ32(std::span<const double>(delays), static_cast<double>(sample_rate.value), delaysQ32);
        _line->process(input, std::span<const std::uint64_t>(delaysQ32), output);

        _position += nSamples;
        return true;
    }

    /// @brief Run `[from, to)` of the call and hand the tags it carried to the output samples that carry their samples.
    void advance(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, std::size_t from, std::size_t to) {
        if (to <= from) {
            return;
        }
        const std::uint64_t firstIndex = _position;
        const bool          filtered   = runSegment(std::span<const T>(inSpan.data() + from, to - from), std::span<T>(outSpan.data() + from, to - from), from);
        placeTags(outSpan, firstIndex, from, to - from, filtered);
    }

    /**
     * @brief Emit each waiting tag on the first output sample whose whole read position has reached its input index.
     *
     * The read position of output `k` is `k - lag`, with the lag the commanded delay plus the line's own, both in
     * the `Q32` the cursor is kept in: `floor((k << 32) - lag) >> 32 >= i` is `(k - i) << 32 >= lag`, which is the
     * comparison below and is integer throughout. The lag is non-negative, so the map is monotone and tags never
     * reorder; where two of them reach one output sample both are attached there, in input order.
     */
    void placeTags(OutputSpanLike auto& outSpan, std::uint64_t firstIndex, std::size_t outOffset, std::size_t nSamples, bool filtered) {
        if (_tagsMoving.empty() && _tagsHeld.empty()) {
            return;
        }
        std::size_t moved = 0UZ;
        std::size_t held  = 0UZ;
        for (std::size_t j = 0UZ; j < nSamples && (moved < _tagsMoving.size() || held < _tagsHeld.size()); ++j) {
            const std::uint64_t index = firstIndex + j;
            const std::int64_t  lag   = filtered ? (static_cast<std::int64_t>(_delaysQ32[outOffset + j]) + _latencyQ32) : 0LL;

            while (held < _tagsHeld.size() && _tagsHeld[held].first <= index) {
                outSpan.publishTag(_tagsHeld[held].second, outOffset + j);
                ++held;
            }

            std::size_t here = 0UZ;
            while (moved < _tagsMoving.size()) {
                const std::uint64_t marked = _tagsMoving[moved].first;
                if (index < marked) {
                    break;
                }
                const std::uint64_t ahead = index - marked;
                if (ahead < gr::filter::kMaxFractionalDelaySamples && (static_cast<std::int64_t>(ahead) << gr::filter::kArbitraryFractionBits) < lag) {
                    break;
                }
                outSpan.publishTag(_tagsMoving[moved].second, outOffset + j);
                ++moved;
                ++here;
            }
            if (here > 1UZ) {
                _nTagsCoalesced.fetch_add(1ULL, std::memory_order_relaxed);
            }
        }
        _tagsMoving.erase(_tagsMoving.begin(), _tagsMoving.begin() + static_cast<std::ptrdiff_t>(moved));
        _tagsHeld.erase(_tagsHeld.begin(), _tagsHeld.begin() + static_cast<std::ptrdiff_t>(held));
    }

    /// @brief A tag naming any of the reserved stream keys, which describe the stream rather than one place in it.
    [[nodiscard]] static bool namesReservedKey(const property_map& map) noexcept {
        for (const auto& entry : map) {
            if (std::ranges::find(gr::tag::kDefaultTags, std::string_view(entry.first)) != gr::tag::kDefaultTags.end()) {
                return true;
            }
        }
        return false;
    }

    /// @brief Count what the reserved keys say about the stream: a gap in it, and a rate that is not the staged one.
    void countStreamKeys(const property_map& map) noexcept {
        if (map.find(property_map::key_type(gr::tag::N_DROPPED_SAMPLES.shortKey())) != map.end()) {
            // The schedule maps a sample index to a time, so a gap makes that map wrong by exactly the gap; the
            // block does not bridge it, and the count is what says so.
            _nDiscontinuities.fetch_add(1ULL, std::memory_order_relaxed);
        }
        const auto rate = map.find(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()));
        if (rate == map.end()) {
            return;
        }
        // The staged rate is the one the schedule is read at, so a stream that says otherwise does not silently
        // retime a pass; the disagreement is counted instead.
        const float* tagged = rate->second.get_if<float>();
        const double staged = static_cast<double>(sample_rate.value);
        if (tagged == nullptr || !(std::abs(static_cast<double>(*tagged) - staged) <= 1e-9 * staged)) {
            _nRateDisagreements.fetch_add(1ULL, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] double delaySecondsAt(std::uint64_t index) const noexcept {
        if (!_schedule || !_anchor.armed()) {
            return _anchor.armed() ? _bias : 0.;
        }
        const double raw = _schedule->delayAt(_clock.timeOf(index));
        return (_direction == detail::DopplerDirection::Correct ? -raw : raw) + _bias;
    }

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
        const double seconds = delaySecondsAt(_position);
        _slot.publish({seconds, seconds * static_cast<double>(sample_rate.value), _bias, _line ? _line->latencySamples() : 0., _line ? _line->groupDelaySamples() : 0., static_cast<double>(static_cast<std::uint8_t>(positionAt(_position)))}, _position);
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_RANGE_DELAY_HPP
