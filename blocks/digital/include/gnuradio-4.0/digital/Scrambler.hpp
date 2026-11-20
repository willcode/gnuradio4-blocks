#ifndef GNURADIO_DIGITAL_SCRAMBLER_HPP
#define GNURADIO_DIGITAL_SCRAMBLER_HPP

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <iterator>
#include <limits>
#include <optional>
#include <print>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/math/BitPacking.hpp>

#include <gnuradio-4.0/algorithm/digital/Scrambler.hpp>

namespace gr::blocks::digital {

/// @brief The additive generators a published agreement fixes, by the library's names; `none` names no profile.
enum class AdditiveProfile : std::uint8_t { none, ccsds131, ccsds131_17, pn9, si4463 };

/// @brief The self-synchronizing generators a published agreement fixes, by the library's names; `none` names no profile.
enum class MultiplicativeProfile : std::uint8_t { none, g3ruh };

namespace detail {

/// @brief The library's name of @p profile, which is the enumerator's own, and empty for `none`.
template<typename TProfile>
[[nodiscard]] constexpr std::string_view profileName(TProfile profile) noexcept {
    return profile == TProfile::none ? std::string_view{} : gr::meta::enumName(profile).value_or(std::string_view{});
}

/// The setting that names the tag key whose item starts a new epoch.
using ResetTagKey = Annotated<std::string, Doc<"reset tag key (empty = off)">>;

/// @brief The settings the generator is built from, which are exactly the ones a change to reconfigures and resets.
/// `reset_tag_key` and `reset_tag_value` are deliberately absent: they select an epoch boundary rather than move one.
struct ScramblerSettings {
    std::string_view   profile{};
    std::string        taps{};
    std::string        seed{};
    std::string        sequence{};
    bool               sequenceRepeat = true;
    gr::Size_t         bitsPerItem    = 8U;
    gr::math::BitOrder bitOrder       = gr::math::BitOrder::msb_first;
    gr::Size_t         resetPeriod    = 0U;
    bool               invertOutput   = false;
    gr::Size_t         forceAfter     = 0U;
    std::string        monitorDelays{};
};

/// @brief What a scrambler block holds: the configured generator, the epoch period, the tag that marks an epoch
/// boundary, and the items since the last reset of any cause.
struct ScramblerState {
    gr::digital::ScramblerConfig config{};
    std::uint64_t                resetPeriod = 0ULL;
    property_map::key_type       resetKey{};
    std::string                  resetValue{};
    std::uint64_t                sinceReset = 0ULL;
};

/**
 * @brief The fault in which of the mask sources the settings in @p n name, under the setting at fault, or nothing.
 *
 * Three mask sources are alternatives and at most one may be given: `taps` with `seed`, a named `profile`, or an
 * explicit `sequence`, which the additive scrambler alone carries. A profile fills `taps`, `seed`, `bits_per_item` and
 * `bit_order` outright, so a value given beside it would leave two answers to one question; `bits_per_item` and
 * `bit_order` carry defaults and cannot be told from an explicit one, so the profile's values simply win there and the
 * refusal covers `taps`, `seed` and `sequence`, which are the settings that are empty until somebody names them.
 */
template<typename TSettings>
[[nodiscard]] std::optional<gr::Refused> checkSources(const TSettings& s, gr::digital::ScramblerMode mode) {
    constexpr bool         kHasSequence = requires { &TSettings::sequence; };
    const std::string_view profile      = profileName(s.profile.value);
    const std::string      taps         = s.taps.value;
    const std::string      seed         = s.seed.value;
    std::string            sequence;
    if constexpr (kHasSequence) {
        sequence = s.sequence.value;
    }

    if (!profile.empty() && !taps.empty()) {
        return gr::Refused(s.taps, "is incompatible with profile");
    }
    if (!profile.empty() && !seed.empty()) {
        return gr::Refused(s.seed, "is incompatible with profile");
    }
    if constexpr (kHasSequence) {
        if (!profile.empty() && !sequence.empty()) {
            return gr::Refused(s.sequence, "is incompatible with profile");
        }
        if (!taps.empty() && !sequence.empty()) {
            return gr::Refused(s.sequence, "is incompatible with taps");
        }
        if (!sequence.empty()) {
            const auto bytes = gr::digital::readSequence(sequence);
            if (!bytes) {
                return gr::Refused(s.sequence, bytes.error());
            }
        }
    }
    if (profile.empty() && taps.empty() && sequence.empty()) {
        return gr::Refused(s.taps, "required");
    }
    if (mode == gr::digital::ScramblerMode::Additive && !taps.empty() && seed.empty()) {
        return gr::Refused(s.seed, "required");
    }
    if (!taps.empty()) {
        const std::expected<gr::digital::TapMask, std::string_view> mask = gr::digital::readDelayList(taps);
        if (!mask) {
            return gr::Refused(s.taps, mask.error());
        }
        if (!seed.empty()) {
            const std::expected<std::uint64_t, std::string_view> bits = gr::digital::readSeed(seed, gr::digital::degreeOf(*mask));
            if (!bits) {
                return gr::Refused(s.seed, bits.error());
            }
        }
    }
    if (mode == gr::digital::ScramblerMode::Additive && !taps.empty() && std::ranges::all_of(seed, [](char c) { return c == '0'; })) {
        return gr::Refused(s.seed, "must contain a 1");
    }
    return {};
}

/**
 * @brief The generator @p wanted names, built by the library's readers. Their refusal is rethrown under the setting's
 * name, since a throw names no setting.
 *
 * An empty multiplicative seed is an all-zero register, a legitimate starting history whose influence is gone `degree`
 * bits later.
 */
[[nodiscard]] inline gr::digital::ScramblerConfig buildScrambler(const ScramblerSettings& wanted, gr::digital::ScramblerMode mode) {
    const auto reading = [](std::string_view setting, auto&& read) {
        try {
            return read();
        } catch (const std::invalid_argument& reason) {
            throw gr::exception(std::format("setting {}: {}", setting, reason.what()));
        }
    };
    gr::digital::ScramblerConfig config{};
    if (!wanted.sequence.empty()) {
        reading("sequence", [&] {
            const std::vector<std::uint8_t> bytes = gr::digital::sequenceFromHex(wanted.sequence);
            gr::digital::configureExplicit(config, std::span<const std::uint8_t>(bytes), bytes.size() * 8UZ, wanted.bitsPerItem, wanted.bitOrder, wanted.sequenceRepeat);
        });
    } else if (!wanted.profile.empty()) {
        reading("profile", [&] { gr::digital::applyProfile(config, wanted.profile, mode); });
    } else {
        const gr::digital::TapMask taps = reading("taps", [&] { return gr::digital::tapsFromDelayList(wanted.taps); });
        const std::uint64_t        seed = wanted.seed.empty() ? 0ULL : reading("seed", [&] { return gr::digital::seedFromBitString(wanted.seed, gr::digital::degreeOf(taps)); });
        gr::digital::configureScrambler(config, taps, seed, mode, wanted.bitsPerItem, wanted.bitOrder);
    }
    // the forced-transition group is the multiplicative pair's alone and is applied last, because every generator call
    // above puts it back to disabled
    if (mode != gr::digital::ScramblerMode::Additive) {
        reading("the forced-transition group", [&] { gr::digital::configureMonitor(config, wanted.invertOutput, static_cast<std::uint32_t>(wanted.forceAfter), wanted.monitorDelays); });
    }
    return config;
}

/**
 * @brief Whether a periodic epoch outlasts an explicit sequence that does not repeat.
 *
 * `sequence_repeat` false leaves everything past the sequence's `S` bits unscrambled, and with a non-zero
 * `reset_period` the shortfall is arithmetic rather than a property of the stream: it is known before the graph runs.
 * The dynamic case -- a tag-driven epoch that outlasts the sequence -- cannot be known then;
 * `nUnscrambledItems` counts it.
 */
[[nodiscard]] inline bool epochOutlastsSequence(const gr::digital::ScramblerConfig& config, gr::Size_t resetPeriod) {
    if (config.source != gr::digital::SequenceSource::Explicit || config.sequenceRepeat || resetPeriod == 0U) {
        return false;
    }
    return static_cast<std::uint64_t>(resetPeriod) * static_cast<std::uint64_t>(config.bitsPerItem) > static_cast<std::uint64_t>(config.tablePeriod);
}

/// @brief Whether @p map marks an epoch boundary: it carries @p key and, when @p value is non-empty, a string equal to
/// it at that key. A value of any other type never matches a narrowing value, and any value matches an empty one.
[[nodiscard]] inline bool marksEpoch(const property_map& map, const property_map::key_type& key, std::string_view value) {
    const auto entry = map.find(key);
    if (entry == map.end()) {
        return false;
    }
    return value.empty() || entry->second.value_or(std::string_view{}) == value;
}

/**
 * @brief One call of a scrambler block: the items between epoch boundaries scrambled a run at a time.
 *
 * A reset applies before the item it lands on, so that item is the first of the new epoch and off by one here would
 * cost the whole frame. At most one reset happens at any item: a tag arriving exactly on a periodic boundary is one
 * reset, and because it zeroes the counter the next periodic boundary is `reset_period` items after the tag rather
 * than after the boundary it replaced. A tag is read and forwarded, never consumed, and one from before the window
 * carries a negative relative index and is skipped.
 */
template<typename TInput, typename TOutput>
[[nodiscard]] work::Status processScrambler(ScramblerState& state, TInput& inSpan, TOutput& outSpan) {
    constexpr std::size_t kNoBoundary = std::numeric_limits<std::size_t>::max();

    const std::size_t   nItems = std::min(inSpan.size(), outSpan.size());
    const std::uint64_t period = state.resetPeriod;

    auto       tagView = inSpan.tags();
    auto       tag     = std::ranges::begin(tagView);
    const auto tagEnd  = std::ranges::end(tagView);

    // the relative index of the next tag that marks a boundary, leaving the iterator on it so the caller can step past
    const auto nextBoundary = [&]() -> std::size_t {
        if (state.resetKey.empty()) { // the mechanism is disabled entirely, for a chain that must not follow a detector
            return kNoBoundary;
        }
        for (; tag != tagEnd; ++tag) {
            const auto& [relIndex, map] = *tag;
            if (relIndex >= static_cast<std::ptrdiff_t>(nItems)) {
                break;
            }
            if (relIndex >= 0 && marksEpoch(map.get(), state.resetKey, state.resetValue)) {
                return static_cast<std::size_t>(relIndex);
            }
        }
        return kNoBoundary;
    };

    std::size_t done     = 0UZ;
    std::size_t boundary = nextBoundary();
    while (done < nItems) {
        bool epoch = false;
        while (boundary != kNoBoundary && boundary <= done) {
            epoch = true;
            ++tag;
            boundary = nextBoundary();
        }
        if (period != 0ULL && state.sinceReset >= period) {
            epoch = true;
        }
        if (epoch) {
            gr::digital::resetScrambler(state.config);
            state.sinceReset = 0ULL;
        }

        std::size_t limit = boundary == kNoBoundary ? nItems : std::min(nItems, boundary);
        if (period != 0ULL) {
            limit = std::min(limit, done + static_cast<std::size_t>(period > state.sinceReset ? period - state.sinceReset : 0ULL));
        }

        const std::size_t count = limit - done;
        gr::digital::scramble(state.config, std::span<const std::uint8_t>(inSpan.data() + done, count), std::span<std::uint8_t>(outSpan.data() + done, count));
        state.sinceReset += count;
        done = limit;
    }

    std::ignore = inSpan.consume(nItems);
    outSpan.publish(nItems);
    return work::Status::OK;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::AdditiveScrambler, [T], [ std::uint8_t, gr::DataSet<std::uint8_t> ])

template<typename T>
requires std::same_as<T, std::uint8_t> || std::same_as<T, DataSet<std::uint8_t>>
struct AdditiveScrambler : Block<AdditiveScrambler<T>> {
    using Description = Doc<R""(
@brief XORs the stream with a free-running LFSR sequence, so that what goes on the air looks like noise whatever the data is.

A transmitter sending a long run of identical bits gives the clock recovery loop nothing to lock to, drifts the AGC,
feeds the DC blocker real signal and grows discrete lines where the spectrum should be flat. This block fixes all of it
in one place. The sequence is a function of `taps` and `seed` alone, so the whitening does not depend on the data and a
channel bit error costs exactly one output bit error, which is as well behaved as anything in a receive chain gets.

The price is that both ends must agree on the phase: one bit of phase error leaves the descrambled stream agreeing with
the truth at chance, and nothing inside the block can tell, because there is nothing to compare against. The detection
belongs downstream at a CRC. Three mechanisms reach the epoch boundary the two ends agree on — `start()` and any
setting that reconfigures the generator; a tag carrying `reset_tag_key`, which resets before the tagged item so that
item is the first of the new epoch; and `reset_period` items since the last reset of any cause. At most one reset
happens at any item, and a tag re-anchors the period. The receive side finds its boundary in a detector's
`gr:trigger_name` tag, which is the default key; the transmit side counts it, which is what `reset_period` is for.

XOR is an involution, so this block descrambles as well as it scrambles and there is no second name for the same code.

`taps` is a list of feedback delays rather than a polynomial's exponents, because a three-term polynomial is written
both ways and the standards disagree on which their spelling means: CCSDS's `x^8 + x^7 + x^5 + x^3 + 1` is `"1,3,5,8"`
while IEEE 802.11's `x^7 + x^4 + 1` is `"4,7"`, each confirmed by the sequence its own standard publishes. There is no
default polynomial and no default seed, and the block refuses to start without both, because a scrambler is an
agreement between two ends and an arbitrary default guarantees only that the other end disagrees.

Two other ways of naming the same mask are alternatives to `taps` and `seed`, and staging more than one of the three is
a refusal rather than a merge. `profile` names a published agreement -- `ccsds131`, `ccsds131_17`, `pn9` or `si4463` --
and fills `taps`, `seed`, `bits_per_item` and `bit_order` from it; it is `none` by default, and the multiplicative
`g3ruh` is not among its names, because a self-synchronizing agreement run additively is a silent wrong answer.
`sequence` supplies the mask outright as hexadecimal bytes, which is how about half the vendor deframers state theirs.
An explicit sequence of `S` bits either tiles the epoch, which is what a period-`S` recursion would do and cannot lose
whitening, or covers its first `S` bits and leaves the rest unscrambled -- every such item counted in
`nUnscrambledItems` and stated at `stop()`, since a 1:1 block cannot drop an item. Where a periodic epoch is known in
advance to outlast the sequence the shortfall is arithmetic and the check refuses it.

The block is 1:1, so every input tag key passes through at its own offset; the reset tag is read and forwarded, never
consumed, and `sample_rate` leaves with the value it arrived with.

The carrier is a bit stream or a bit record. On the `DataSet<std::uint8_t>` instantiation one record is one epoch —
the generator resets at the first item of every record, which is 10.4.3's own rule once extraction has made one
record one codeblock — so no tag can reset on the wrong detector and no period needs counting: `reset_tag_key`,
`reset_tag_value` and `reset_period` are inert there. The record's metadata, name and shape cross verbatim, XOR
being an involution with no status to report.

`taps` lists distinct feedback delays from 1 to 64, comma separated, such as "1,3,5,8". `seed` holds the first degree
bits of the sequence, leftmost first. `sequence` is an explicit mask of hexadecimal byte pairs, MSB first, such as "FF
E1", and `sequence_repeat` tiles it over the epoch. A tag at `reset_tag_key` starts a new epoch.
)"">;

    static constexpr bool kRecordCarrier = std::same_as<T, DataSet<std::uint8_t>>;

    std::conditional_t<kRecordCarrier, PortIn<T, Async>, PortIn<T>>   in;
    std::conditional_t<kRecordCarrier, PortOut<T, Async>, PortOut<T>> out;

    struct Settings {
        Annotated<AdditiveProfile, Doc<"published generator">, Visible>                    profile = AdditiveProfile::none;
        Annotated<std::string, Doc<"feedback delays">, Visible>                            taps{};
        Annotated<std::string, Doc<"initial sequence bits">, Visible>                      seed{};
        Annotated<std::string, Doc<"explicit mask">, Visible>                              sequence{};
        Annotated<bool, Doc<"sequence tiling">>                                            sequence_repeat = true;
        Annotated<gr::Size_t, Unit<"bit">, Doc<"bits per item">, Limits<Min<1U>, Max<8U>>> bits_per_item   = 8U;
        Annotated<gr::math::BitOrder, Doc<"bit order">>                                    bit_order       = gr::math::BitOrder::msb_first;
        detail::ResetTagKey                                                                reset_tag_key   = std::string(gr::tag::TRIGGER_NAME.key());
        Annotated<std::string, Doc<"reset tag value (empty = any)">>                       reset_tag_value{};
        Annotated<gr::Size_t, Unit<"items">, Doc<"reset period (0 = off)">>                reset_period = 0U;

        GR_MAKE_REFLECTABLE(Settings, profile, taps, seed, sequence, sequence_repeat, bits_per_item, bit_order, reset_tag_key, reset_tag_value, reset_period);
    } settings;

    GR_MAKE_REFLECTABLE(AdditiveScrambler, in, out, settings);

    detail::ScramblerState _state{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::optional<gr::Refused> refused = detail::checkSources(s, gr::digital::ScramblerMode::Additive);
        if (refused) {
            return refused;
        }
        const gr::digital::ScramblerConfig config = detail::buildScrambler(generator(s), gr::digital::ScramblerMode::Additive);
        if (detail::epochOutlastsSequence(config, s.reset_period)) {
            return gr::Refused(s.reset_period, std::format("must be 0 or <= {}, the items in the sequence, while sequence_repeat is false", config.tablePeriod / config.bitsPerItem));
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _state.resetKey   = property_map::key_type(s.reset_tag_key.value);
        _state.resetValue = s.reset_tag_value.value;
        // a register carried across a polynomial change would be meaningless, so a change that reconfigures is a
        // discontinuity: the new configuration takes effect at the first item of the call the change lands in
        const bool reconfigure = s.profile.changed() || s.taps.changed() || s.seed.changed() || s.sequence.changed() || s.sequence_repeat.changed() || s.bits_per_item.changed() || s.bit_order.changed() || s.reset_period.changed();
        if (!reconfigure) {
            return;
        }
        _state.config      = detail::buildScrambler(generator(s), gr::digital::ScramblerMode::Additive);
        _state.resetPeriod = s.reset_period;
        _state.sinceReset  = 0ULL;
    }

    void start() { reset(); }

    void reset() {
        gr::digital::resetScrambler(_state.config);
        _state.sinceReset = 0ULL;
    }

    /// @brief The generator @p n names.
    [[nodiscard]] detail::ScramblerSettings generator(const Settings& s) const { return {.profile = detail::profileName(s.profile.value), .taps = s.taps.value, .seed = s.seed.value, .sequence = s.sequence.value, .sequenceRepeat = s.sequence_repeat.value, .bitsPerItem = s.bits_per_item.value, .bitOrder = s.bit_order.value, .resetPeriod = s.reset_period.value}; }

    /// @brief Items a non-repeating explicit sequence could not cover, cumulative over the configuration's life.
    /// Structurally zero without one, since a recursion has no end to run past.
    [[nodiscard]] std::uint64_t nUnscrambledItems() const noexcept { return static_cast<std::uint64_t>(_state.config.nUnscrambledItems); }

    /// @brief Zero here: the forced-transition rule belongs to the multiplicative pair, which has feedback for it to
    /// act on. Read alongside its sibling so one reader shape serves all three blocks.
    [[nodiscard]] std::uint64_t nForcedTransitions() const noexcept { return static_cast<std::uint64_t>(_state.config.nForcedTransitions); }

    void stop() {
        if (_state.config.nUnscrambledItems != 0UZ) {
            std::println(stderr, "gr::blocks::digital::AdditiveScrambler '{}': {} item(s) past the end of the explicit sequence left unscrambled", this->name, _state.config.nUnscrambledItems);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if constexpr (!kRecordCarrier) {
            return detail::processScrambler(_state, inSpan, outSpan);
        } else {
            std::size_t consumed = 0UZ;
            std::size_t made     = 0UZ;
            for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
                const DataSet<std::uint8_t>& record = inSpan[consumed];
                DataSet<std::uint8_t>        masked = record; // the record's facts cross verbatim; only the values move
                gr::digital::resetScrambler(_state.config);   // one record is one epoch, at the record's first item
                gr::digital::scramble(_state.config, std::span<const std::uint8_t>(record.signal_values), std::span<std::uint8_t>(masked.signal_values));
                outSpan[made] = std::move(masked);
                ++made;
            }

            std::ignore = inSpan.consume(consumed);
            outSpan.publish(made);
            if (made == 0UZ && consumed == 0UZ) {
                return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
            }
            return work::Status::OK;
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::MultiplicativeScrambler)

struct MultiplicativeScrambler : Block<MultiplicativeScrambler> {
    using Description = Doc<R""(
@brief Feeds the transmitted bits back through the shift register: `out[k] = in[k] XOR f(out, k)`.

The transmit half of the self-synchronizing pair. Where an additive scrambler needs the two ends to agree on a phase,
this one needs no boundary agreement at all: MultiplicativeDescrambler's register holds only received bits, so its
output is correct `degree` bits after it starts, from any state and from any offset into the stream. That is what a
continuous link with no framing to lean on uses, and it is why the V-series modems and 802.3's 64B/66B scramble this
way while CCSDS, DVB and 802.11's OFDM PHY, all framed, scramble additively. The polynomial does not decide which
family a link is in: 802.11 uses the same `"4,7"` additively in its OFDM PHY and self-synchronizingly in its DSSS PHY.

It is paid for twice. One channel bit error becomes `1 + tapCount` errors at the descrambler, which is the number that
decides whether the scrambler goes inside or outside a forward error correcting code. And the whitening depends on the
data, unlike the additive family's: an all-zero register carrying an all-zero input is a fixed point that emits zeros
for ever, and for any seed there is an input -- one equal to the feedback at every step -- that holds the output at a
constant for as long as it continues. The V-series answer to that is `force_transition_after` with `monitor_delays`,
which both ends compute from the transmitted stream alone: the counter runs while `y[k-p] XOR y[k-q]` stays zero and a
complement is injected at the `N`-th such item. Because the monitor reads only what is on the wire, the descrambler
computes the same counter from the same bits and stays exactly this block's inverse -- so the rule is a setting after
all, not a protocol. What it cannot do is bound the output's run length: the pair is a bijection on bit streams, so for
every output, the constant one included, there is exactly one input producing it. What it does do is destroy the
lock-up: fed the plain recursion's own lock-up input, the output stops being constant within `N` items.
`nForcedTransitions` counts the firings, and a chain whose count is zero is a chain that could have set `N` to zero.

`seed` is the `degree` bits assumed to precede the stream, oldest first, and an empty seed means all zeros and is
legal, because those bits reach only the first `degree` bits of an epoch. Resetting therefore buys reproducibility
rather than correctness: the same data with the same seed at the same boundaries produces the same transmitted bits.

`profile` names a published agreement -- `g3ruh`, the 9600-baud packet-radio scrambler -- and fills `taps`, `seed`,
`bits_per_item` and `bit_order` from it, refusing to merge with any of them; the additive profiles are not among its
names. `taps`, the three reset mechanisms and the tag policy are AdditiveScrambler's, unchanged.

The block is 1:1, so every input tag key passes through at its own offset; the reset tag is read and forwarded, never
consumed, and `sample_rate` leaves with the value it arrived with.

`taps` lists distinct feedback delays from 1 to 64, comma separated, such as "18,23". `seed` holds the degree bits
assumed to precede the stream, as '0'/'1' oldest first; all zeros is a legitimate history here. `invert_output`
complements every bit of the scrambled stream, the 'v' the V-series family carries, and the descrambler carries the same
value. At the `force_transition_after`th consecutive item over which the monitored comparison stayed zero, a complement
is injected; `monitor_delays` names the two transmitted-stream delays 'p,q' it compares, distinct and each from 1 to 64.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    struct Settings {
        Annotated<MultiplicativeProfile, Doc<"published generator">, Visible>              profile = MultiplicativeProfile::none;
        Annotated<std::string, Doc<"feedback delays">, Visible>                            taps{};
        Annotated<std::string, Doc<"initial history bits (empty = zeros)">, Visible>       seed{};
        Annotated<gr::Size_t, Unit<"bit">, Doc<"bits per item">, Limits<Min<1U>, Max<8U>>> bits_per_item          = 8U;
        Annotated<gr::math::BitOrder, Doc<"bit order">>                                    bit_order              = gr::math::BitOrder::msb_first;
        Annotated<bool, Doc<"output inversion">>                                           invert_output          = false;
        Annotated<gr::Size_t, Unit<"items">, Doc<"run length limit (0 = off)">>            force_transition_after = 0U;
        Annotated<std::string, Doc<"monitor delays">>                                      monitor_delays{};
        detail::ResetTagKey                                                                reset_tag_key = std::string(gr::tag::TRIGGER_NAME.key());
        Annotated<std::string, Doc<"reset tag value (empty = any)">>                       reset_tag_value{};
        Annotated<gr::Size_t, Unit<"items">, Doc<"reset period (0 = off)">>                reset_period = 0U;

        GR_MAKE_REFLECTABLE(Settings, profile, taps, seed, bits_per_item, bit_order, invert_output, force_transition_after, monitor_delays, reset_tag_key, reset_tag_value, reset_period);
    } settings;

    GR_MAKE_REFLECTABLE(MultiplicativeScrambler, in, out, settings);

    detail::ScramblerState _state{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::optional<gr::Refused> refused = detail::checkSources(s, gr::digital::ScramblerMode::MultiplicativeScramble);
        if (refused) {
            return refused;
        }
        std::ignore = detail::buildScrambler(generator(s), gr::digital::ScramblerMode::MultiplicativeScramble);
        return {};
    }

    void applySettings(const Settings& s) {
        _state.resetKey        = property_map::key_type(s.reset_tag_key.value);
        _state.resetValue      = s.reset_tag_value.value;
        const bool reconfigure = s.profile.changed() || s.taps.changed() || s.seed.changed() || s.bits_per_item.changed() || s.bit_order.changed() || s.reset_period.changed() || s.invert_output.changed() || s.force_transition_after.changed() || s.monitor_delays.changed();
        if (!reconfigure) {
            return;
        }
        _state.config      = detail::buildScrambler(generator(s), gr::digital::ScramblerMode::MultiplicativeScramble);
        _state.resetPeriod = s.reset_period;
        _state.sinceReset  = 0ULL;
    }

    void start() { reset(); }

    void reset() {
        gr::digital::resetScrambler(_state.config);
        _state.sinceReset = 0ULL;
    }

    /// @brief The generator @p n names.
    [[nodiscard]] detail::ScramblerSettings generator(const Settings& s) const { return {.profile = detail::profileName(s.profile.value), .taps = s.taps.value, .seed = s.seed.value, .bitsPerItem = s.bits_per_item.value, .bitOrder = s.bit_order.value, .resetPeriod = s.reset_period.value, .invertOutput = s.invert_output.value, .forceAfter = s.force_transition_after.value, .monitorDelays = s.monitor_delays.value}; }

    /// @brief Zero here: an explicit sequence is the additive family's mask source, and a recursion never runs out.
    [[nodiscard]] std::uint64_t nUnscrambledItems() const noexcept { return static_cast<std::uint64_t>(_state.config.nUnscrambledItems); }

    /// @brief Times the forcing term fired, cumulative over the configuration's life. Not a failure: it is what says
    /// the rule is reachable in a given chain at all.
    [[nodiscard]] std::uint64_t nForcedTransitions() const noexcept { return static_cast<std::uint64_t>(_state.config.nForcedTransitions); }

    void stop() {
        if (_state.config.forceAfter != 0U) {
            std::println(stderr, "gr::blocks::digital::MultiplicativeScrambler '{}': the forcing term fired {} time(s)", this->name, _state.config.nForcedTransitions);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) { return detail::processScrambler(_state, inSpan, outSpan); }
};

GR_REGISTER_BLOCK(gr::blocks::digital::MultiplicativeDescrambler)

struct MultiplicativeDescrambler : Block<MultiplicativeDescrambler> {
    using Description = Doc<R""(
@brief Feeds the received bits forward through the shift register: `out[k] = in[k] XOR f(in, k)`.

The inverse of MultiplicativeScrambler, exact from item 0 when both carry the same seed. Its register holds nothing but
received bits, so once `degree` of them have arrived every bit in it is data and the output is correct from there on
whatever the register started as -- from a wrong seed, from a mid-stream join, from any offset at all. That is what
self-synchronizing means, and it is why a reset here is a convenience rather than a requirement: a reset changes at
most the first `degree` output bits of an epoch and nothing else. The contrast with the additive family is the whole
argument for this one, and it is stark: an additive descrambler one bit out of phase agrees with the truth at chance
for ever, with nothing inside the block able to tell.

What it costs is error multiplication. One channel bit error becomes `1 + tapCount` output errors -- three for a
two-tap polynomial -- so on a link with forward error correction this number is what decides whether the scrambler
goes inside the code or outside it.

`seed` is the `degree` bits assumed to precede the stream, oldest first, and an empty seed means all zeros and is
legal. `profile`, `invert_output`, `force_transition_after` and `monitor_delays` are MultiplicativeScrambler's and must
carry the same values it does: the monitor reads only the transmitted stream, so both ends compute the same counter
from the same bits and this block stays the exact inverse -- from item 0 with the same seed, and from item `degree`
with any other. `taps`, the three reset mechanisms and the tag policy are AdditiveScrambler's, unchanged.

The block is 1:1, so every input tag key passes through at its own offset; the reset tag is read and forwarded, never
consumed, and `sample_rate` leaves with the value it arrived with.

`seed` holds the degree bits preceding the stream, oldest first. `invert_output`, `force_transition_after` and
`monitor_delays` take the scrambler's values, and the inversion is undone here.
)"">;

    PortIn<std::uint8_t>  in;
    PortOut<std::uint8_t> out;

    struct Settings {
        Annotated<MultiplicativeProfile, Doc<"published generator">, Visible>              profile = MultiplicativeProfile::none;
        Annotated<std::string, Doc<"feedback delays">, Visible>                            taps{};
        Annotated<std::string, Doc<"initial history bits (empty = zeros)">, Visible>       seed{};
        Annotated<gr::Size_t, Unit<"bit">, Doc<"bits per item">, Limits<Min<1U>, Max<8U>>> bits_per_item          = 8U;
        Annotated<gr::math::BitOrder, Doc<"bit order">>                                    bit_order              = gr::math::BitOrder::msb_first;
        Annotated<bool, Doc<"output inversion">>                                           invert_output          = false;
        Annotated<gr::Size_t, Unit<"items">, Doc<"run length limit (0 = off)">>            force_transition_after = 0U;
        Annotated<std::string, Doc<"monitor delays">>                                      monitor_delays{};
        detail::ResetTagKey                                                                reset_tag_key = std::string(gr::tag::TRIGGER_NAME.key());
        Annotated<std::string, Doc<"reset tag value (empty = any)">>                       reset_tag_value{};
        Annotated<gr::Size_t, Unit<"items">, Doc<"reset period (0 = off)">>                reset_period = 0U;

        GR_MAKE_REFLECTABLE(Settings, profile, taps, seed, bits_per_item, bit_order, invert_output, force_transition_after, monitor_delays, reset_tag_key, reset_tag_value, reset_period);
    } settings;

    GR_MAKE_REFLECTABLE(MultiplicativeDescrambler, in, out, settings);

    detail::ScramblerState _state{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::optional<gr::Refused> refused = detail::checkSources(s, gr::digital::ScramblerMode::MultiplicativeDescramble);
        if (refused) {
            return refused;
        }
        std::ignore = detail::buildScrambler(generator(s), gr::digital::ScramblerMode::MultiplicativeDescramble);
        return {};
    }

    void applySettings(const Settings& s) {
        _state.resetKey        = property_map::key_type(s.reset_tag_key.value);
        _state.resetValue      = s.reset_tag_value.value;
        const bool reconfigure = s.profile.changed() || s.taps.changed() || s.seed.changed() || s.bits_per_item.changed() || s.bit_order.changed() || s.reset_period.changed() || s.invert_output.changed() || s.force_transition_after.changed() || s.monitor_delays.changed();
        if (!reconfigure) {
            return;
        }
        _state.config      = detail::buildScrambler(generator(s), gr::digital::ScramblerMode::MultiplicativeDescramble);
        _state.resetPeriod = s.reset_period;
        _state.sinceReset  = 0ULL;
    }

    void start() { reset(); }

    void reset() {
        gr::digital::resetScrambler(_state.config);
        _state.sinceReset = 0ULL;
    }

    /// @brief The generator @p n names.
    [[nodiscard]] detail::ScramblerSettings generator(const Settings& s) const { return {.profile = detail::profileName(s.profile.value), .taps = s.taps.value, .seed = s.seed.value, .bitsPerItem = s.bits_per_item.value, .bitOrder = s.bit_order.value, .resetPeriod = s.reset_period.value, .invertOutput = s.invert_output.value, .forceAfter = s.force_transition_after.value, .monitorDelays = s.monitor_delays.value}; }

    /// @brief Zero here: an explicit sequence is the additive family's mask source, and a recursion never runs out.
    [[nodiscard]] std::uint64_t nUnscrambledItems() const noexcept { return static_cast<std::uint64_t>(_state.config.nUnscrambledItems); }

    /// @brief Times the forcing term fired, cumulative over the configuration's life. It must equal the scrambler's
    /// over the same stream, which is what says the two ends computed the same counter.
    [[nodiscard]] std::uint64_t nForcedTransitions() const noexcept { return static_cast<std::uint64_t>(_state.config.nForcedTransitions); }

    void stop() {
        if (_state.config.forceAfter != 0U) {
            std::println(stderr, "gr::blocks::digital::MultiplicativeDescrambler '{}': the forcing term fired {} time(s)", this->name, _state.config.nForcedTransitions);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) { return detail::processScrambler(_state, inSpan, outSpan); }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_SCRAMBLER_HPP
