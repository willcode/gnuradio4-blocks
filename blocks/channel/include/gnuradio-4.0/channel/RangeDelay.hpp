#ifndef GNURADIO_CHANNEL_RANGE_DELAY_HPP
#define GNURADIO_CHANNEL_RANGE_DELAY_HPP

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <complex>
#include <cstdint>
#include <expected>
#include <format>
#include <iterator>
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

namespace detail {

/// The refusal of a bank whose prototype would need more than the design allows, given before any design runs.
[[nodiscard]] inline std::string tooManyTaps(std::size_t bank) { return std::format("bank {} needs more than {} taps", bank, gr::filter::kFractionalDelayMaxTaps); }

/// The refusal of a design that missed its stopband or its ripple limit, or none where it met them.
[[nodiscard]] inline std::optional<std::string> designShortfall(std::size_t bank, const gr::filter::ResamplerDesign& design) {
    if (design.ok) {
        return std::nullopt;
    }
    return std::format("bank {} misses attenuation_db or the ripple limit", bank);
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::channel::RangeDelay, [T], [ std::complex<float>, float ])

template<typename T>
requires std::is_same_v<T, std::complex<float>> || std::is_same_v<T, float>
struct RangeDelay : gr::Block<RangeDelay<T>, gr::NoTagPropagation> {
    using Description = Doc<R""(
@brief Applies a trajectory's envelope delay to a stream, or removes it.

A source at slant range `R(t)` arrives `tau(t) = R(t)/c` seconds late. Its baseband equivalent is
`x(t - tau(t)) * exp(-j 2 pi f_c tau(t))`. This block carries the delay `x(t - tau(t))`, and `DopplerShift` carries
the phase factor. To apply one trajectory, run RangeDelay and then DopplerShift. To remove it, run DopplerShift's
correction and then RangeDelay's. The block outputs one sample per input sample through a
`gr::filter::FractionalDelayLine`.

The delays come from `schedule_times_ns` and `schedule_delays_s`, or from the `range_m` column of a
`#!gr4-trajectory 1` file named in `schedule_file`, read as `range/c`. Setting both is refused. The schedule is linear
between knots and holds its end values outside them. With neither, the commanded delay is the bias alone.

Under `direction` 'apply' the commanded delay is the schedule plus `bias_s`. Under 'correct' it is `bias_s` less the
schedule. A `bias_s` of 0, the default, derives the bias: zero under 'apply' and the schedule's maximum under 'correct'.
The commanded delay is then non-negative, and the corrected stream carries a constant delay of that maximum. A knot
whose commanded delay is negative or past the line's reach is refused, and so is a segment whose slope reaches one
sample per sample. The refusal names the knot or the segment.

`bank_size`, `interpolation`, `attenuation_db` and `rolloff` design the line. `bank_size` is 0 or a power of two, and 0
derives the bank from the other three. A bank whose estimated prototype needs more than 65536 taps is refused before it is designed. The estimate
follows `bank_size * (attenuation_db - 8) / rolloff`, and below 45 dB the ripple limit makes it longer, many times so
for a rolloff up to 0.85. From 45 dB up no rolloff below one holds more than 8192 arms. Below 45 dB a rolloff above 0.7
is estimated short, so the check can admit a bank the design cannot hold, and the block refuses it after the design. At
`rolloff` 0.2 with `bank_size` 0, `linear` designs up to 110 dB and `cubic` up to 140 dB. A design near the cap takes
tens of seconds to a few minutes inside the settings change, and longer in a Debug build. A change to any of the four
builds a new bank, drops the line's history and counts in `nBankRebuilds()`. A design that misses `attenuation_db` or
its ripple limit is refused too, and the refusal gives the stopband and ripple it delivers. A change of table,
direction, bias or rate takes effect at the next sample and keeps the history. A new rate re-times the whole stream from
the anchor.

The settings `anchor_source`, `anchor_index`, `anchor_ns` and `honor_trigger_offset` and the counters
`nIgnoredAnchors()`, `nReanchors()` and `nRefusedAnchors()` work as in `DopplerShift`. Until a trigger anchor is armed,
the stream passes through and the line takes nothing. An arm, or a move of an armed anchor to another index or time,
by a tag or by a settings change, clears the line's history. Until it fills, up to `historySamples()` outputs are the filter's response to silence.

A tag with a key in the `gr:` domain leaves on the sample it arrived on. Every other tag leaves on the first
output sample whose whole read position has reached the tag's input index. A tag still waiting when the stream ends
leaves at the end-of-stream index. `nTagsCoalesced()` counts the output samples that carry two or more such tags.
`nDiscontinuities()` counts `gr:n_dropped_samples` tags, a gap the schedule does not bridge. `nRateDisagreements()`
counts `gr:sample_rate` tags that differ from the accepted `sample_rate`. The schedule stays at that rate.

`schedule_delays_s` holds one delay per time. `schedule_file` names a '#!gr4-trajectory 1' file with a range column.
`bias_s` is a constant delay added to the schedule.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<std::vector<std::int64_t>, Visible, Unit<"ns">, Doc<detail::kScheduleTimesDoc>>           schedule_times_ns{};
        Annotated<std::vector<double>, Visible, Unit<"s">, Doc<"scheduled delays">>                         schedule_delays_s{};
        Annotated<std::string, Doc<"range trajectory file">>                                                schedule_file{};
        Annotated<detail::DopplerDirection, Visible, Doc<detail::kDirectionDoc>>                            direction            = detail::DopplerDirection::Apply;
        Annotated<double, Unit<"s">, Doc<"delay bias (0 = derived)">>                                       bias_s               = 0.0;
        Annotated<float, Visible, Unit<"Hz">, Doc<detail::kSampleRateDoc>>                                  sample_rate          = 1.f;
        Annotated<std::uint64_t, Doc<detail::kAnchorIndexDoc>>                                              anchor_index         = 0ULL;
        Annotated<std::int64_t, Unit<"ns">, Doc<detail::kAnchorNsDoc>, detail::AnchorNsLimits>              anchor_ns            = 0LL;
        Annotated<gr::timing::AnchorSource, Doc<detail::kAnchorSourceDoc>>                                  anchor_source        = gr::timing::AnchorSource::setting;
        Annotated<bool, Doc<detail::kHonorTriggerOffsetDoc>>                                                honor_trigger_offset = true;
        Annotated<gr::Size_t, Doc<"polyphase arm count (0 = derived)">, Limits<Max<8192U>>>                 bank_size            = 0U;
        Annotated<gr::filter::ArmInterpolation, Doc<"interpolation between arms">>                          interpolation        = gr::filter::ArmInterpolation::linear;
        Annotated<double, Unit<"dB">, Doc<"prototype stopband attenuation">, Limits<Min<20.0>, Max<140.0>>> attenuation_db       = 60.0;
        Annotated<double, Doc<"excess bandwidth">, Limits<Over<0.0>, Under<1.0>>>                           rolloff              = 0.2;

        GR_MAKE_REFLECTABLE(Settings, schedule_times_ns, schedule_delays_s, schedule_file, direction, bias_s, sample_rate, anchor_index, anchor_ns, anchor_source, honor_trigger_offset, bank_size, interpolation, attenuation_db, rolloff);
    } settings;

    GR_MAKE_REFLECTABLE(RangeDelay, in, out, settings);

    std::optional<gr::timing::DelaySchedule> _schedule{}; ///< the table as configured, before the direction's sign

    std::optional<gr::filter::FractionalDelayLine<T>>   _line{};
    gr::timing::SampleClock                             _clock{};
    gr::timing::ScheduleAnchor                          _anchor{};
    std::uint64_t                                       _position{0ULL};
    double                                              _bias{0.};        ///< the bias in force, derived when `bias_s` asks for it
    std::int64_t                                        _latencyQ32{0LL}; ///< the line's own lag in the fixed point the tag map reads
    std::vector<double>                                 _delays{};        ///< one call's worth of commanded delays, grown not rebuilt
    std::vector<std::uint64_t>                          _delaysQ32{};
    std::vector<std::pair<std::uint64_t, property_map>> _tagsMoving{}; ///< tags waiting for their output sample, by ascending absolute input index
    std::vector<std::pair<std::uint64_t, property_map>> _tagsHeld{};   ///< the same, for the tags that leave on the sample they arrived on

    /// The design the line's bank was built from.
    std::size_t                  _builtBank{0UZ};
    gr::filter::ArmInterpolation _builtInterpolation{gr::filter::ArmInterpolation::nearest};
    double                       _builtRolloff{0.};
    double                       _builtAttenuationDb{0.};

    gr::measurement::MeasurementSlot<7UZ> _slot{}; ///< delay s, delay samples, bias s, latency, group delay, schedule position, history, with the stream position as fill count
    std::atomic<std::uint64_t>            _nTagsCoalesced{0ULL};
    std::atomic<std::uint64_t>            _nBankRebuilds{0ULL};
    std::atomic<std::uint64_t>            _nDiscontinuities{0ULL};
    std::atomic<std::uint64_t>            _nRateDisagreements{0ULL};
    // The anchor state, copied for a reader on another thread. An anchor time is a nanosecond count beyond the
    // exact range of a double. It is stored as an integer beside the slot.
    std::atomic<bool>          _armed{false};
    std::atomic<std::uint64_t> _anchorIndex{0ULL};
    std::atomic<std::int64_t>  _anchorNs{0LL};
    std::atomic<std::uint64_t> _nIgnoredAnchors{0ULL};
    std::atomic<std::uint64_t> _nReanchors{0ULL};
    std::atomic<std::uint64_t> _nRefusedAnchors{0ULL};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const bool tableGiven = !s.schedule_times_ns.value.empty() || !s.schedule_delays_s.value.empty();
        if (!s.schedule_file.value.empty() && tableGiven) {
            return gr::Refused("schedule_file is incompatible with schedule lists");
        }
        // a trigger anchor's clock is checked when its tag arrives
        std::uint64_t anchorIndex = 0ULL;
        std::int64_t  anchorNs    = 0LL;
        if (s.anchor_source == gr::timing::AnchorSource::setting) {
            anchorIndex = s.anchor_index;
            anchorNs    = s.anchor_ns;
        }
        const auto clock = clockAt(s.sample_rate, anchorIndex, anchorNs);
        if (!clock) {
            return gr::Refused(s.sample_rate, clock.error().message);
        }
        if (s.schedule_file.value.empty()) { // a file's knots are known only once it is read
            const std::optional<gr::timing::DelaySchedule> table  = tableOf(s.schedule_times_ns.value, s.schedule_delays_s.value);
            const std::expected<Staged, std::string>       staged = stage(table, s.direction, s.bias_s, s.sample_rate);
            if (!staged) {
                return gr::Refused(staged.error());
            }
        }
        if (s.bank_size != 0U && !std::has_single_bit(s.bank_size.value)) {
            return gr::Refused(s.bank_size, "must be 0 or a power of 2");
        }
        const std::size_t bank = bankFor(s.bank_size, s.attenuation_db, s.rolloff, s.interpolation);
        if (!gr::filter::fractionalDelayReachable(bank, s.rolloff, s.attenuation_db)) {
            return gr::Refused(detail::tooManyTaps(bank));
        }
        return {};
    }

    void applySettings(const Settings& s) {
        const bool                 anchorMoved = s.anchor_source.changed() || s.anchor_index.changed() || s.anchor_ns.changed() || s.honor_trigger_offset.changed();
        gr::timing::ScheduleAnchor anchor      = _anchor;
        if (anchorMoved) {
            anchor = gr::timing::ScheduleAnchor(s.anchor_source, s.anchor_index, s.anchor_ns, s.honor_trigger_offset);
        }
        const bool                               tableMoved = s.schedule_times_ns.changed() || s.schedule_delays_s.changed() || s.schedule_file.changed();
        std::optional<gr::timing::DelaySchedule> table      = _schedule;
        if (tableMoved) {
            auto loaded = loadTable(s);
            if (!loaded) {
                this->fail(s.schedule_file, loaded.error());
                return;
            }
            table = std::move(*loaded);
        }
        if (const std::optional<Failure> failure = configure(s, std::move(table), anchor)) {
            this->fail(failure->key, failure->reason);
        }
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

    /// @brief The delay commanded now, in seconds. It is the schedule plus the bias, negated in `correct`. Any thread.
    [[nodiscard]] double currentDelaySeconds() const noexcept { return _slot.read().first[0]; }
    /// @brief The same delay in samples at the block's own rate. Any thread.
    [[nodiscard]] double currentDelaySamples() const noexcept { return _slot.read().first[1]; }
    /// @brief The constant the schedule is shifted by, derived under `correct` unless `bias_s` states it. Any thread.
    [[nodiscard]] double biasSeconds() const noexcept { return _slot.read().first[2]; }
    /// @brief The output's lag behind the input at a commanded delay of zero. It is the prototype's lag plus one sample.
    [[nodiscard]] double latencySamples() const noexcept { return _slot.read().first[3]; }
    /// @brief The prototype's own delay of `(N-1)/(2L)` input samples. `latencySamples()` is one sample more.
    [[nodiscard]] double groupDelaySamples() const noexcept { return _slot.read().first[4]; }
    /// @brief The next sample's place relative to the schedule's span. `Before` and `After` hold an end value.
    [[nodiscard]] SchedulePosition schedulePosition() const noexcept { return static_cast<SchedulePosition>(static_cast<std::uint8_t>(_slot.read().first[5])); }
    /// @brief The samples of history the line holds. Any thread.
    [[nodiscard]] std::size_t historySamples() const noexcept { return static_cast<std::size_t>(_slot.read().first[6]); }

    /// @brief The index of the next sample from the stream's start. The anchor places it in time. Any thread.
    [[nodiscard]] std::uint64_t position() const noexcept { return _slot.read().second; }
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
        const auto filtered = runSegment(input.first(nSamples), output.first(nSamples), 0UZ);
        publishMeasurements();
        if (!filtered) {
            this->emitErrorMessage("processBulk", filtered.error());
            return work::Status::ERROR;
        }
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t   nSamples  = std::min(inSpan.size(), outSpan.size());
        const std::uint64_t callStart = _position;
        reserve(nSamples);

        // A tag with a reserved key waits in `_tagsHeld`, and every other tag in `_tagsMoving`. One walk over the
        // output places both kinds in order.
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
            // The call splits at the trigger, and the samples before it keep their anchor.
            if (const auto advanced = advance(inSpan, outSpan, done, at); !advanced) {
                return refuse(inSpan, outSpan, done, advanced.error());
            }
            done = at;

            const gr::timing::ScheduleAnchor::Response response = read.readable ? _anchor.onTrigger(_position, read.timeNs, read.offsetSeconds) : _anchor.onUnreadableTrigger();
            if (response == gr::timing::ScheduleAnchor::Response::armed || response == gr::timing::ScheduleAnchor::Response::reanchored) {
                const auto clock = clockAt(settings.sample_rate, _anchor.anchorIndex(), _anchor.anchorNs());
                if (!clock) {
                    return refuse(inSpan, outSpan, done, clock.error());
                }
                _clock = *clock;
                // The line starts on silence at each anchor. Its history holds samples from before a jump in schedule
                // time, or from before a stretch the block passed through unarmed.
                if (_line) {
                    _line->reset();
                }
            }
        }
        if (const auto advanced = advance(inSpan, outSpan, done, nSamples); !advanced) {
            return refuse(inSpan, outSpan, done, advanced.error());
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        publishMeasurements();
        return work::Status::OK;
    }

    /// @brief At the end of the stream, publishes every tag still waiting, and every tag past the last input, at the
    /// end-of-stream index. `processBulk` places the tags of each chunk.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (processedIn != 0UZ) {
            return;
        }
        std::vector<std::pair<std::uint64_t, property_map>> waiting = std::move(_tagsHeld);
        waiting.insert(waiting.end(), std::make_move_iterator(_tagsMoving.begin()), std::make_move_iterator(_tagsMoving.end()));
        _tagsHeld.clear();
        _tagsMoving.clear();
        gr::for_each_reader_span(
            [&waiting, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
                for (const gr::Tag& tag : span.rawTags) {
                    if (tag.index < span.streamIndex) { // placed by an earlier call
                        continue;
                    }
                    property_map map = tag.map;
                    map.erase(endOfStream);
                    if (!map.empty()) {
                        countStreamKeys(map);
                        waiting.emplace_back(static_cast<std::uint64_t>(tag.index), std::move(map));
                    }
                }
            },
            inputSpans);
        if (waiting.empty()) {
            return;
        }
        std::ranges::stable_sort(waiting, [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
        gr::for_each_writer_span(
            [&waiting](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [index, map] : waiting) {
                    span.publishTag(map, 0UZ);
                }
            },
            outputSpans);
    }

private:
    /// The bias and the history a table needs, kept apart from the block until every check has passed.
    struct Staged {
        double        bias{0.};
        std::uint64_t wholeDelaySamples{0ULL};
    };

    /// A setting the block could not put in force, and why.
    struct Failure {
        std::string_view key;
        std::string      reason;
    };

    [[nodiscard]] static std::expected<gr::timing::SampleClock, gr::Error> clockAt(float rate, std::uint64_t index, std::int64_t timeNs) { return gr::timing::clockForRateHz(static_cast<double>(rate), index, timeNs); }

    /// The polyphase arms `bank_size` names, or derives at 0.
    [[nodiscard]] static std::size_t bankFor(gr::Size_t bankSize, double attenuationDb, double rolloff, gr::filter::ArmInterpolation interpolation) { return bankSize == 0U ? gr::filter::arbitraryBankSize(attenuationDb, rolloff, gr::filter::interpolationOrder(interpolation)) : static_cast<std::size_t>(bankSize); }

    /**
     * @brief Puts @p s in force with @p table and @p anchor, or returns the setting that failed.
     *
     * Every step that can fail runs before anything changes. A new bank is built only when a design value differs from
     * the line's. Otherwise the line keeps its history and grows to the reach the table needs.
     */
    [[nodiscard]] std::optional<Failure> configure(const Settings& s, std::optional<gr::timing::DelaySchedule> table, const gr::timing::ScheduleAnchor& anchor) {
        // the place a trigger gave an armed anchor is known here alone
        const auto clock = clockAt(s.sample_rate, anchor.armed() ? anchor.anchorIndex() : 0ULL, anchor.armed() ? anchor.anchorNs() : 0LL);
        if (!clock) {
            return Failure{"sample_rate", clock.error().message};
        }
        const std::string_view                   delayKey = s.schedule_file.value.empty() ? "schedule_delays_s" : "schedule_file"; // the source of the table
        const std::size_t                        bank     = bankFor(s.bank_size, s.attenuation_db, s.rolloff, s.interpolation);
        const std::expected<Staged, std::string> staged   = stage(table, s.direction, s.bias_s, s.sample_rate); // a file's knots are checked here
        if (!staged) {
            return Failure{delayKey, staged.error()};
        }

        const bool held    = _line.has_value();
        const bool cutBank = !held || bank != _builtBank || s.interpolation != _builtInterpolation || s.rolloff != _builtRolloff || s.attenuation_db != _builtAttenuationDb;
        if (cutBank) {
            // The held samples do not line up with a new prototype's taps, and a new bank starts on silence. The design
            // runs here and not in the check, as it can take minutes near the cap.
            const auto design = gr::filter::designFractionalDelay(bank, s.rolloff, s.attenuation_db);
            if (!design) {
                return Failure{"attenuation_db", design.error().message};
            }
            if (const std::optional<std::string> shortfall = detail::designShortfall(bank, *design)) {
                return Failure{"attenuation_db", *shortfall};
            }
            gr::filter::FractionalDelayLine<T> built(bank, gr::filter::interpolationOrder(s.interpolation.value), std::span<const float>(design->taps), staged->wholeDelaySamples);
            _line               = std::move(built);
            _builtBank          = bank;
            _builtInterpolation = s.interpolation;
            _builtRolloff       = s.rolloff;
            _builtAttenuationDb = s.attenuation_db;
            if (held) { // the first bank drops no history and is not counted
                _nBankRebuilds.fetch_add(1ULL, std::memory_order_relaxed);
            }
        } else {
            const std::expected<void, gr::Error> grown = _line->growMaxDelay(staged->wholeDelaySamples);
            if (!grown) {
                return Failure{delayKey, grown.error().message};
            }
        }

        // An arm, or an armed anchor moved to another index or time, starts the line on silence.
        if (anchor.armed() && (!_anchor.armed() || anchor.anchorIndex() != _anchor.anchorIndex() || anchor.anchorNs() != _anchor.anchorNs())) {
            _line->reset();
        }
        _schedule = std::move(table);
        _bias     = staged->bias;
        _anchor   = anchor;
        if (anchor.armed()) {
            _clock = *clock;
        }
        _latencyQ32 = static_cast<std::int64_t>(std::llround(_line->latencySamples() * static_cast<double>(gr::filter::kArbitraryOne)));
        publishMeasurements();
        return std::nullopt;
    }

    /// Publishes the samples before @p done and reports @p error as the call's result.
    [[nodiscard]] work::Status refuse(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, std::size_t done, const gr::Error& error) {
        std::ignore = inSpan.consume(done);
        outSpan.publish(done);
        publishMeasurements();
        this->emitErrorMessage("processBulk", error);
        return work::Status::ERROR;
    }

    /// The largest delay the fixed point carries at @p rate, in seconds, less one sample. The line's history is the
    /// whole part of the delay plus one. That history stays inside the same bound.
    [[nodiscard]] static double reachSeconds(float rate) noexcept { return static_cast<double>(gr::filter::kMaxFractionalDelaySamples - 1ULL) / static_cast<double>(rate); }

    /// The whole samples of history a commanded delay of @p seconds needs at @p rate. That is its whole part plus the
    /// sample the fraction reaches into. `reachSeconds()` bounds it, and every path checks that bound before this call.
    [[nodiscard]] static std::uint64_t wholeDelayFor(double seconds, float rate) noexcept { return static_cast<std::uint64_t>(std::max(0., std::floor(seconds * static_cast<double>(rate)) + 1.)); }

    /// The table @p s names, before the direction's sign, or none for a delay of the bias alone; the reason when the
    /// file does not give one.
    [[nodiscard]] static std::expected<std::optional<gr::timing::DelaySchedule>, std::string> loadTable(const Settings& s) {
        if (s.schedule_file.value.empty()) {
            return tableOf(s.schedule_times_ns.value, s.schedule_delays_s.value);
        }
        auto loaded = gr::timing::loadTrajectoryFile(s.schedule_file.value);
        if (!loaded) {
            return std::unexpected(loaded.error().message);
        }
        if (!loaded->delay.has_value()) {
            return std::unexpected("must name a file with a delay column");
        }
        return std::move(loaded->delay);
    }

    /// The table @p times and @p delays give, or none when both are empty. A table the schedule refuses throws.
    [[nodiscard]] static std::optional<gr::timing::DelaySchedule> tableOf(std::span<const std::int64_t> times, std::span<const double> delays) {
        if (times.empty() && delays.empty()) {
            return std::nullopt;
        }
        return gr::timing::DelaySchedule(times, delays);
    }

    /// The bias and the history @p table needs, or the refusal of a knot, a segment or the bias.
    [[nodiscard]] static std::expected<Staged, std::string> stage(const std::optional<gr::timing::DelaySchedule>& table, detail::DopplerDirection direction, double bias, float rate) {
        const double reach = reachSeconds(rate);
        if (!table) {
            // Without a table the delay is the bias alone. The same reach and causality limits apply.
            Staged staged;
            staged.bias = bias;
            if (staged.bias < 0.) {
                return std::unexpected("bias_s: must be >= 0");
            }
            if (!(staged.bias < reach)) {
                return std::unexpected(std::format("bias_s: must be < {} s", reach));
            }
            staged.wholeDelaySamples = wholeDelayFor(staged.bias, rate);
            return staged;
        }

        const double sign = direction == detail::DopplerDirection::Correct ? -1. : 1.;
        if (direction == detail::DopplerDirection::Apply && bias < 0.) {
            return std::unexpected("bias_s: must be >= 0 in apply mode");
        }
        const double resolved = resolveBias(*table, direction, bias);

        // The commanded delay is `sign * delay + bias`. It must stay in [0, the line's reach) at every knot. Its
        // slope must stay under one sample per sample, or the read position would run backwards.
        const auto knotTimes = table->times();
        const auto knotVals  = table->values();
        for (std::size_t i = 0UZ; i < knotVals.size(); ++i) {
            const double commanded = sign * knotVals[i] + resolved;
            if (!(commanded >= 0.)) {
                return std::unexpected(std::format("knot {}: delay must be >= 0", i));
            }
            if (!(commanded < reach)) {
                return std::unexpected(std::format("knot {}: delay must be < {} s", i, reach));
            }
        }
        for (std::size_t i = 1UZ; i < knotVals.size(); ++i) {
            const double dtSeconds = static_cast<double>(knotTimes[i] - knotTimes[i - 1UZ]) * 1e-9;
            const double slope     = std::abs((knotVals[i] - knotVals[i - 1UZ]) / dtSeconds); // seconds of delay per second, dimensionless
            if (!(slope < 1.0)) {
                return std::unexpected(std::format("segment {}: slope must be < 1", i));
            }
        }

        Staged staged;
        staged.bias = resolved;
        // The line holds history for the largest delay this table commands. A schedule holds its end values. The
        // maximum therefore depends on the table alone and not on the stream.
        double worst = 0.;
        for (const double value : knotVals) {
            worst = std::max(worst, sign * value + resolved);
        }
        staged.wholeDelaySamples = wholeDelayFor(worst, rate);
        return staged;
    }

    [[nodiscard]] static double resolveBias(const gr::timing::DelaySchedule& schedule, detail::DopplerDirection direction, double bias) {
        if (bias != 0.) {
            return bias;
        }
        // The default keeps the commanded delay causal. Apply is already non-negative and takes zero. Correct commands
        // `-delay + bias`, least where the delay is greatest. The schedule's maximum lifts that least value to zero.
        return direction == detail::DopplerDirection::Correct ? schedule.maxValue() : 0.;
    }

    void reserve(std::size_t nSamples) {
        // The buffers grow to the largest chunk and keep that size. A call allocates only for a chunk larger than
        // every earlier one.
        if (_delays.size() < nSamples) {
            _delays.resize(nSamples);
            _delaysQ32.resize(nSamples);
        }
    }

    /// @brief Fills `[at, at + input.size())` of the call's delays and runs the line over @p input.
    /// True where it filtered, or the line's error for a delay or span it refused.
    [[nodiscard]] std::expected<bool, gr::Error> runSegment(std::span<const T> input, std::span<T> output, std::size_t at) {
        const std::size_t nSamples = input.size();
        if (nSamples == 0UZ) {
            return false;
        }
        if (!_line || !_anchor.armed()) {
            // An unarmed anchor places no sample in time. The input passes through, and the line takes nothing.
            std::ranges::copy(input, output.begin());
            _position += nSamples;
            return false;
        }

        const std::span<double>        delays(_delays.data() + at, nSamples);
        const std::span<std::uint64_t> delaysQ32(_delaysQ32.data() + at, nSamples);
        if (_schedule) {
            _schedule->valuesFor(_clock, _position, delays);
            for (double& value : delays) {
                value = (settings.direction.value == detail::DopplerDirection::Correct ? -value : value) + _bias;
            }
        } else {
            std::ranges::fill(delays, _bias); // no table is a constant bias, which may be zero
        }
        if (const auto converted = gr::filter::fractionalDelayQ32(std::span<const double>(delays), static_cast<double>(settings.sample_rate.value), delaysQ32); !converted) {
            return std::unexpected(converted.error());
        }
        if (const auto ran = _line->process(input, std::span<const std::uint64_t>(delaysQ32), output); !ran) {
            return std::unexpected(ran.error());
        }

        _position += nSamples;
        return true;
    }

    /// @brief Runs `[from, to)` of the call and attaches its tags to the output samples that carry their samples.
    [[nodiscard]] std::expected<void, gr::Error> advance(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, std::size_t from, std::size_t to) {
        if (to <= from) {
            return {};
        }
        const std::uint64_t firstIndex = _position;
        const auto          filtered   = runSegment(std::span<const T>(inSpan.data() + from, to - from), std::span<T>(outSpan.data() + from, to - from), from);
        if (!filtered) {
            return std::unexpected(filtered.error());
        }
        placeTags(outSpan, firstIndex, from, to - from, *filtered);
        return {};
    }

    /**
     * @brief Emits each waiting tag on the first output sample whose whole read position has reached its input index.
     *
     * The read position of output `k` is `k - lag`. The lag is the commanded delay plus the line's own, both in the
     * `Q32` format of the cursor. `floor((k << 32) - lag) >> 32 >= i` equals `(k - i) << 32 >= lag`. The comparison
     * below uses that integer form. The lag is non-negative, and the map is monotone. Tags keep their order. Two
     * tags that reach one output sample are both attached there, in input order.
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

    /// @brief True for a tag with any key in the `gr:` domain.
    [[nodiscard]] static bool namesReservedKey(const property_map& map) noexcept {
        for (const auto& entry : map) {
            if (gr::tag::domainOf(std::string_view(entry.first)) == gr::tag::kGrDomain) {
                return true;
            }
        }
        return false;
    }

    /// @brief Counts the gaps and the rates other than the accepted one that the reserved keys report.
    void countStreamKeys(const property_map& map) noexcept {
        if (map.find(property_map::key_type(gr::tag::N_DROPPED_SAMPLES.key())) != map.end()) {
            _nDiscontinuities.fetch_add(1ULL, std::memory_order_relaxed);
        }
        const auto rate = map.find(property_map::key_type(gr::tag::SAMPLE_RATE.key()));
        if (rate == map.end()) {
            return;
        }
        const float* tagged   = rate->second.get_if<float>();
        const double accepted = static_cast<double>(settings.sample_rate.value);
        if (tagged == nullptr || !(std::abs(static_cast<double>(*tagged) - accepted) <= 1e-9 * accepted)) {
            _nRateDisagreements.fetch_add(1ULL, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] double delaySecondsAt(std::uint64_t index) const noexcept {
        if (!_schedule || !_anchor.armed()) {
            return _anchor.armed() ? _bias : 0.;
        }
        const double raw = _schedule->delayAt(_clock.timeOf(index));
        return (settings.direction.value == detail::DopplerDirection::Correct ? -raw : raw) + _bias;
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

    /// Publishes the current state for readers on other threads. Runs once per call.
    void publishMeasurements() noexcept {
        _armed.store(_anchor.armed(), std::memory_order_relaxed);
        _anchorIndex.store(_anchor.anchorIndex(), std::memory_order_relaxed);
        _anchorNs.store(_anchor.anchorNs(), std::memory_order_relaxed);
        _nIgnoredAnchors.store(_anchor.nIgnoredAnchors(), std::memory_order_relaxed);
        _nReanchors.store(_anchor.nReanchors(), std::memory_order_relaxed);
        _nRefusedAnchors.store(_anchor.nRefusedAnchors(), std::memory_order_relaxed);
        const double seconds = delaySecondsAt(_position);
        _slot.publish({seconds, seconds * static_cast<double>(settings.sample_rate.value), _bias, _line ? _line->latencySamples() : 0., _line ? _line->groupDelaySamples() : 0., static_cast<double>(static_cast<std::uint8_t>(positionAt(_position))), static_cast<double>(_line ? _line->historySamples() : 0UZ)}, _position);
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_RANGE_DELAY_HPP
