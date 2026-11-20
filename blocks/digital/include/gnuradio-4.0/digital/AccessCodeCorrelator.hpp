#ifndef GNURADIO_DIGITAL_ACCESS_CODE_CORRELATOR_HPP
#define GNURADIO_DIGITAL_ACCESS_CODE_CORRELATOR_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fec/Convolutional.hpp>

namespace gr::blocks::digital {

namespace syncword {
/// CCSDS 131.0-B-5 attached sync marker, 9.3.1 and figure 9-1; hexadecimal 1ACFFC1D.
/// Used with uncoded, convolutional, Reed-Solomon and concatenated coded data (9.3.5 note).
/// Peak 32, worst aperiodic sidelobe 9. An interoperability constant, never a default.
inline constexpr std::string_view ccsds_asm = "00011010110011111111110000011101";

/**
 * @brief The marker above encoded by the named inner convention, truncated to the channel symbols that do not
 * depend on the encoder's prior state — the word a symbol-domain correlator searches for.
 *
 * In a concatenated chain the marker is encoded by the inner code along with everything else, so the encoded
 * marker's leading symbols depend on whatever data preceded it. From input bit `K-1` onward the register holds
 * only the marker's own bits, so the last `n * (32 - (K - 1))` symbols are a genuine constant of the channel —
 * derived here by encoding the marker from the zero state and keeping those symbols. The string is '0'/'1'
 * characters in symbol order, ready for this block's `access_code`; under the rate-1/2 conventions it is 52
 * symbols. An unknown convention name is refused listing the ones there are.
 */
[[nodiscard]] inline std::string ccsdsEncodedAsm(std::string_view convention) {
    gr::fec::ConvolutionalCode code;
    if (!gr::fec::configureConvention(code, convention)) {
        std::string names;
        for (const gr::fec::ConvolutionalConvention& entry : gr::fec::kConvConventions) {
            std::format_to(std::back_inserter(names), "{}'{}'", names.empty() ? "" : ", ", entry.name);
        }
        throw gr::exception(std::format("convention must name one of {}, got '{}'", names, convention));
    }

    const std::size_t steps = ccsds_asm.size();
    const std::size_t n     = code.polynomialCount;
    const std::size_t keep  = (steps - (code.constraintLength - 1UZ)) * n;

    std::string   encoded;
    std::uint32_t state = 0U;
    for (const char bit : ccsds_asm) {
        const std::uint32_t input = bit == '1' ? 1U : 0U;
        const std::uint32_t out   = gr::fec::convolutionalBranch(code, state, input);
        for (std::size_t j = 0UZ; j < n; ++j) {
            encoded.push_back(((out >> j) & 1U) != 0U ? '1' : '0');
        }
        state = gr::fec::convolutionalNextState(code, state, input);
    }
    return encoded.substr(encoded.size() - keep);
}
} // namespace syncword

/// @brief The item types an access code is correlated over: hard bits, or soft bits sliced at zero.
template<typename T>
concept BitLike = std::same_as<T, std::uint8_t> || std::floating_point<T>;

namespace detail {

/// @brief One iff non-zero for a byte item, `x >= 0` for a soft one; a NaN slices to zero, every IEEE compare being false.
[[nodiscard]] inline constexpr bool sliceBit(std::uint8_t item) noexcept { return item != 0U; }

template<std::floating_point T>
[[nodiscard]] inline constexpr bool sliceBit(T item) noexcept {
    return item >= T{0};
}

/// @brief Two 64-bit limbs, so an access code may hold up to 128 items — the AO-40 long form's 65 among them.
/// The hot loop reads the limbs directly and a code within the low one never touches the high one, so only a
/// word past 64 bits pays for the second limb. Every shift distance is under 64: the shift-in moves at most
/// 8 bits and the symbol collapse at most 7, so no whole limb ever crosses in one step.
struct Word128 {
    std::uint64_t hi = 0ULL;
    std::uint64_t lo = 0ULL;

    friend constexpr Word128 operator&(Word128 a, Word128 b) noexcept { return {a.hi & b.hi, a.lo & b.lo}; }

    constexpr void shiftInLow(unsigned k, std::uint64_t bits) noexcept {
        hi = (hi << k) | (lo >> (64U - k));
        lo = (lo << k) | bits;
    }

    constexpr void setBit(std::size_t at) noexcept { (at < 64UZ ? lo : hi) |= 1ULL << (at & 63UZ); }
};

/// @brief The lowest @p bits set, for @p bits in [0, 128].
[[nodiscard]] inline constexpr Word128 lowMask(std::size_t bits) noexcept {
    if (bits >= 128UZ) {
        return {~0ULL, ~0ULL};
    }
    if (bits >= 64UZ) {
        return {bits == 64UZ ? 0ULL : (1ULL << (bits - 64UZ)) - 1ULL, ~0ULL};
    }
    return {0ULL, bits == 0UZ ? 0ULL : (1ULL << bits) - 1ULL};
}

} // namespace detail

/// @brief Which item AccessCodeCorrelator's tag names: the one after the code, or the code's first.
enum class TagPosition : std::uint8_t { payload_start, code_start };

GR_REGISTER_BLOCK(gr::blocks::digital::AccessCodeCorrelator, [T], [ std::uint8_t, float ])

template<BitLike T>
struct AccessCodeCorrelator : Block<AccessCodeCorrelator<T>> {
    using Description = Doc<R""(
@brief Finds a known bit pattern in the stream and tags where the payload starts. The stream itself passes through.

The detector half of a framing chain. The stream passes through unchanged, the tags carry the framework's reserved
`trigger_*` keys, and `StreamToDataSet` cuts packets out of this block's output with no adapter in between.

There is no default access code: a block with none set is not ready rather than detecting something arbitrary. `max_errors`
is an inclusive bound on the Hamming distance rather than a count of matching bits, so 0 is an exact match. The tag
sits at the first item after the code, which is the first payload item, and the Hamming distance travels in
`gr:trigger_meta_info` as `sync_errors` rather than as the tag's key.

The block is 1:1, so every input tag key passes through at its own offset beside the tags it emits. The label the
emitted tags carry under `gr:trigger_name` is the `trigger_label` setting.

An item may carry more than one bit: at `bits_per_item` above 1 each byte item's low bits enter the register together
and the item is one SYMBOL — the distance `max_errors` bounds is then the count of mismatched symbols, a symbol wrong
in every one of its bits costing one, which is the distance framed protocols over multilevel modulations state their
sync thresholds in (P25's C4FM sync is 24 dibits, its threshold symbol-wise). At 1 bit per item a byte item is one
iff non-zero and a soft item slices at zero, exactly as before; a soft stream carries one bit per item by nature, so
a wider setting on a float stream is refused.

`code_stride` spaces the code's items `s` apart instead of packing them together, which is the construction the AO-40 FEC
beacon uses: a code of `n` items at stride `s` is exactly one row of an `s` by `n` block interleaver, so after
deinterleaving it is the frame's first `n` items and the coded block is everything behind it. Only the comparison is
strided; the frame of `n*s` items under it is contiguous, so the extraction downstream is an ordinary window and no
block below here learns about strides. Every item a strided comparison reads has the same residue modulo `s`, so the
block holds `s` independent registers and the per-item work is what it was plus one modular index. The register is
two 64-bit limbs, so a code may carry up to 128 items and the AO-40 long form's 65 fit with room behind them.

`tag_at` says which item the tag names. `payload_start` is the item after the code's last, which is where the tag has
always sat and is what a contiguous chain wants. `code_start` is the code's FIRST item, which is where a strided frame
begins, and it gives the block an output lag of `D = (n-1)*s + 1` items on `PreambleCorrelator`'s stated precedent:
`out[j] == in[j-D]`, the first `D` output items are zero, and the tag index does not move -- only the stream under it
does. An extractor after `code_start` is a start-only filter with no history at all. A tag arriving from upstream keeps
its input offset. Under `code_start` it stands `D` items ahead of the sample it marked. Only the tags this block emits
carry the lag.

`access_code` is written as '0'/'1' characters. `max_errors` is the largest distance accepted, counted in symbols of
`bits_per_item` bits. `trigger_label` is written under gr:trigger_name of the emitted tags, which an extractor filters
on.
)"">;

    /// @brief The largest `code_stride`, at sixteen bytes of register each: 64 KB of state at the cap, and the AO-40
    /// geometries' 80 and 51 stay comfortably L1. The bounded-state rule the family works under is what fixes it.
    static constexpr gr::Size_t kMaxStride = 4096U;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<std::string, Doc<"access code bits">, Visible>                                                                  access_code{};
        Annotated<gr::Size_t, Doc<"error limit (0 = exact)">>                                                                     max_errors    = 0U;
        Annotated<std::string, Doc<"emitted trigger name">>                                                                       trigger_label = std::string("access_code");
        Annotated<gr::Size_t, Doc<"bits per item">, Limits<Min<1U>, Max<8U>>>                                                     bits_per_item = 1U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"code item stride (1 = contiguous)">, Visible, Limits<Min<1U>, Max<kMaxStride>>> code_stride   = 1U;
        Annotated<TagPosition, Doc<"tag position">>                                                                               tag_at        = TagPosition::payload_start;

        GR_MAKE_REFLECTABLE(Settings, access_code, max_errors, trigger_label, bits_per_item, code_stride, tag_at);
    } settings;

    GR_MAKE_REFLECTABLE(AccessCodeCorrelator, in, out, settings);

    /// @brief Tags emitted over the block's life. The binomial false-alarm rate of the word's length and `max_errors`
    /// turns it into an expected value, and a count far above that says `max_errors` is too generous.
    std::uint64_t nDetections = 0ULL;

    detail::Word128              _code{};
    detail::Word128              _mask{};
    detail::Word128              _groupLsb{};       /// one bit per symbol, at each symbol's lowest position
    std::vector<detail::Word128> _registers{};      /// one register per residue class modulo `code_stride`
    std::size_t                  _codeLength = 0UZ; /// the code in bits
    std::size_t                  _symbols    = 0UZ; /// the code in items, which is what `code_stride` spaces apart
    std::size_t                  _firstAt    = 0UZ; /// `(n-1)*code_stride + 1`, the first item a whole comparison exists at
    std::size_t                  _seen       = 0UZ; /// items shifted in since the last reset, capped at `_firstAt`
    std::size_t                  _phase      = 0UZ; /// the residue of the item about to be shifted in
    std::vector<T>               _delay{};          /// the output lag's ring, empty unless `tag_at` is 'code_start'
    std::size_t                  _delayAt = 0UZ;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::string code  = s.access_code;
        const std::size_t width = s.bits_per_item;
        if (code.empty()) {
            return gr::Refused(s.access_code, "required");
        }
        if (code.size() > 128UZ) {
            return gr::Refused(s.access_code, "must be <= 128 bits");
        }
        if (std::ranges::any_of(code, [](char c) { return c != '0' && c != '1'; })) {
            return gr::Refused(s.access_code, "must contain only 0s and 1s");
        }
        if (std::floating_point<T> && width != 1UZ) {
            return gr::Refused(s.bits_per_item, "must be 1 on a floating-point stream");
        }
        if (code.size() % width != 0UZ) {
            return gr::Refused(s.access_code, "must be a multiple of bits_per_item");
        }
        if (s.max_errors > code.size() / width) {
            return gr::Refused(s.max_errors, "must be <= the symbols in access_code");
        }
        return {};
    }

    void applySettings(const Settings& s) { rebuild(s); }

    // a restart counts its own detections and compares against an empty register
    void start() {
        nDetections = 0ULL;
        reset();
    }

    void stop() {
        if (nDetections != 0ULL) {
            std::println(stderr, "gr::blocks::digital::AccessCodeCorrelator '{}': {} detection(s)", this->name, nDetections);
        }
    }

    void reset() {
        std::ranges::fill(_registers, detail::Word128{});
        std::ranges::fill(_delay, T{});
        _seen    = 0UZ;
        _phase   = 0UZ;
        _delayAt = 0UZ;
    }

    void rebuild(const Settings& s) {
        const std::string_view code(s.access_code.value);
        const std::size_t      width   = s.bits_per_item;
        const std::size_t      spacing = s.code_stride;
        detail::Word128        word{};
        for (const char character : code) {
            word.shiftInLow(1U, character == '1' ? 1ULL : 0ULL);
        }

        _codeLength = code.size();
        _symbols    = code.size() / width;
        _mask       = detail::lowMask(_codeLength);
        _code       = word & _mask;
        _groupLsb   = detail::Word128{};
        for (std::size_t at = 0UZ; at < _codeLength; at += width) {
            _groupLsb.setBit(at);
        }
        // the oldest item a comparison reads is `i - 1 - (n-1)*code_stride`, so no whole comparison exists before that
        // index is non-negative. At stride 1 this is `i = n`, which is the contiguous rule, so one counter serves both
        _firstAt = (_symbols - 1UZ) * spacing + 1UZ;

        // `_symbols` is at most 128, the register's width, and `code_stride` at most 4096, so the lag is at most
        // 127*4096 + 1 = 520193 items — still under the bounded-state line the "D above 2^20 throws" check would
        // draw, so the two caps remain the check and no second one exists.
        _registers.assign(spacing, detail::Word128{});
        _delay.assign(s.tag_at == TagPosition::code_start ? _firstAt : 0UZ, T{});
        nDetections = 0ULL;
        reset(); // a new code compares against its own bits, never against the previous code's
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        // a code within the low limb never reads or writes the high one, so a code up to 64 bits costs one limb
        // and only a longer word pays for the second
        if (_mask.hi == 0ULL) {
            scan<false>(inSpan, outSpan, nSamples);
        } else {
            scan<true>(inSpan, outSpan, nSamples);
        }
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

    template<bool kWide>
    void scan(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, std::size_t nSamples) {
        const detail::Word128 code     = _code;
        const detail::Word128 mask     = _mask;
        const detail::Word128 groupLsb = _groupLsb;
        const std::size_t     length   = _codeLength;
        const unsigned        width    = settings.bits_per_item;
        const std::size_t     bound    = settings.max_errors;
        const std::size_t     spacing  = _registers.size();
        const std::size_t     firstAt  = _firstAt;

        // hoisted out of the loop: the registers and the lag's ring are vectors, and the inner loop must not pay for
        // that on every item
        detail::Word128* const registers = _registers.data();
        T* const               delay     = _delay.empty() ? nullptr : _delay.data();
        const std::size_t      lag       = _delay.size();

        std::size_t seen    = _seen;
        std::size_t phase   = _phase;
        std::size_t delayAt = _delayAt;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            // the comparison at item `i` reads items i-1, i-1-s, ..., i-1-(n-1)s, which all share the residue
            // (i-1) mod s, so one register holds all of them and the index is a decrement rather than a division
            const std::size_t testPhase = phase == 0UZ ? spacing - 1UZ : phase - 1UZ;
            if (seen >= firstAt) {
                // A symbol is wrong when any of its bits is: the differences collapse onto each
                // symbol's lowest bit, so the popcount counts symbols. One bit per item makes the
                // collapse loop empty and the group mask the whole mask — the bit distance as before.
                const std::uint64_t diffLo      = (registers[testPhase].lo ^ code.lo) & mask.lo;
                std::uint64_t       collapsedLo = diffLo;
                std::size_t         wrong;
                if constexpr (kWide) {
                    const std::uint64_t diffHi      = (registers[testPhase].hi ^ code.hi) & mask.hi;
                    std::uint64_t       collapsedHi = diffHi;
                    for (unsigned s = 1U; s < width; ++s) { // a 128-bit shift: bits leaving the high limb enter the low one
                        collapsedHi |= diffHi >> s;
                        collapsedLo |= (diffLo >> s) | (diffHi << (64U - s));
                    }
                    wrong = static_cast<std::size_t>(std::popcount(collapsedLo & groupLsb.lo) + std::popcount(collapsedHi & groupLsb.hi));
                } else {
                    for (unsigned s = 1U; s < width; ++s) {
                        collapsedLo |= diffLo >> s;
                    }
                    wrong = static_cast<std::size_t>(std::popcount(collapsedLo & groupLsb.lo));
                }
                if (wrong <= bound) {
                    // the code's first item is at input `i - lag`, which the lag puts at output `i`; without a lag the
                    // tag names the item after the code's last, which is also `i`. The index does not move either way.
                    outSpan.publishTag(property_map{{gr::tag::TRIGGER_NAME.key(), settings.trigger_label.value}, //
                                           {gr::tag::TRIGGER_OFFSET.key(), 0.0f},                                //
                                           {gr::tag::TRIGGER_META_INFO.key(), property_map{{"sync_errors", static_cast<gr::Size_t>(wrong)}, {"code_length", static_cast<gr::Size_t>(length)}}}},
                        i);
                    ++nDetections;
                }
            } else {
                ++seen;
            }
            std::uint64_t bits;
            if constexpr (std::floating_point<T>) {
                bits = detail::sliceBit(inSpan[i]) ? 1ULL : 0ULL;
            } else {
                bits = width == 1U ? (detail::sliceBit(inSpan[i]) ? 1ULL : 0ULL) : (static_cast<std::uint64_t>(inSpan[i]) & ((1ULL << width) - 1ULL));
            }
            if constexpr (kWide) {
                registers[phase].hi = (registers[phase].hi << width) | (registers[phase].lo >> (64U - width));
            }
            registers[phase].lo = (registers[phase].lo << width) | bits;
            phase               = phase + 1UZ == spacing ? 0UZ : phase + 1UZ;

            if (delay == nullptr) {
                outSpan[i] = inSpan[i];
            } else {
                outSpan[i]     = delay[delayAt];
                delay[delayAt] = inSpan[i];
                delayAt        = delayAt + 1UZ == lag ? 0UZ : delayAt + 1UZ;
            }
        }

        _seen    = seen;
        _phase   = phase;
        _delayAt = delayAt;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_ACCESS_CODE_CORRELATOR_HPP
