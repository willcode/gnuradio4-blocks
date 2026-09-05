#ifndef GNURADIO_P25_IMBE_FRAME_HPP
#define GNURADIO_P25_IMBE_FRAME_HPP

// The voice codeword of P25 Phase 1, per TIA-102.BAAA: 144 transmitted bits carrying the 88
// bits of IMBE speech parameters that a vocoder turns into 20 ms of audio.
//
// A voice frame carries nine of these. `PayloadBits.hpp` owns the frame's geometry and
// says where each one begins in information bits; this file owns what is inside one.
//
// Three layers sit between the air and the parameters, and they are undone in this order:
//
// 1. The interleave. The codeword's 144 bits are not transmitted consecutively. Write them
//    into a matrix of six rows by twenty-four columns, filling it a row at a time; the frame
//    carries the matrix a column at a time, so codeword bit 24r+c lands six bits into the
//    frame for every column, at 6c+r. On odd-numbered columns the two bits of each
//    transmitted dibit are exchanged, which is the whole of the difference between the two
//    column parities and turns the row index r into r^1. Six rows read three dibits, so one
//    column is three dibits and the twenty-four columns are the codeword's 72.
//
//    The effect is that a burst of channel errors, which lands on consecutive transmitted
//    bits, is spread across the codeword six bits apart -- one bit into each of six different
//    error-correcting codewords instead of six bits into one.
//
// 2. The scrambling. Everything but the first Golay codeword is exclusive-ored with a
//    pseudorandom sequence generated from the first codeword's decoded information bits, so
//    the sequence can only be reproduced by a receiver that has already corrected u0. Its
//    generator is the linear congruence p <- (173p + 13849) mod 65536 seeded with u0 shifted
//    up four places, and each output bit is the top bit of the new state, taken most
//    significant first. The generator runs continuously across the six scrambled codewords
//    rather than restarting for each.
//
//    This is what makes u0 load-bearing: an error in it that the Golay code cannot repair
//    does not merely corrupt one parameter, it desynchronizes the sequence and destroys the
//    other six. u0 also carries the harmonic count, on which every later field's length
//    depends, so the standard protects it twice -- first by not scrambling it, then by
//    seeding everything else from it.
//
// 3. The error correction, unequal by design and matched to how much each parameter matters.
//    u0 through u3 are Golay(23,12,7), correcting three bits in twelve; u4 through u6 are
//    Hamming(15,11,3), correcting one in eleven; u7's seven bits are carried bare, being the
//    least significant bits of parameters whose high bits are already protected.
//
//      codeword bits   0..22   23..45  46..68  69..91   92..106  107..121 122..136 137..143
//      field           u0      u1      u2      u3       u4       u5       u6       u7
//      code            Golay23 Golay23 Golay23 Golay23  Ham15    Ham15    Ham15    none
//      scrambled       no      yes     yes     yes      yes      yes      yes      no
//
//    That comes to 88 information bits in 144 transmitted ones.

#include <array>
#include <cstddef>
#include <cstdint>

#include <gnuradio-4.0/algorithm/fec/Golay.hpp>
#include <gnuradio-4.0/algorithm/fec/Hamming.hpp>
#include <gnuradio-4.0/p25/PayloadBits.hpp>

namespace gr::p25 {

//! One codeword's transmitted bits, and the parameter words it carries.
inline constexpr std::size_t kImbeCodewordBits   = 144U;
inline constexpr std::size_t kImbeParameterWords = 8U;

//! Voice codewords in a voice frame, and the audio samples each one becomes.
inline constexpr std::size_t kImbeCodewordsPerFrame  = 9U;
inline constexpr std::size_t kImbeSamplesPerCodeword = 160U;

//! The interleaver's matrix, filled a row at a time and transmitted a column at a time.
inline constexpr std::size_t kImbeInterleaveRows    = 6U;
inline constexpr std::size_t kImbeInterleaveColumns = 24U;

//! The low speed data field, which sits between the eighth and ninth voice codewords.
inline constexpr std::size_t kLduLowSpeedDataBits   = 32U;
inline constexpr std::size_t kLduLowSpeedDataOrigin = kLduFirstGroupOrigin + 5U * kLduGroupStride + kLduCodewordsPerGroup * kLduCodewordBits + kLduVoiceCodewordBits;

//! Where each of the nine voice codewords begins, in information bits from the frame sync.
//!
//! The first two are adjacent; from there each is one interleaving stride of 184 bits past
//! the last, except the ninth, which the thirty-two bits of low speed data push further out.
inline constexpr std::array<std::size_t, kImbeCodewordsPerFrame> kImbeCodewordOrigin{
    kFrameHeaderInfoBits,
    kFrameHeaderInfoBits + kLduVoiceCodewordBits,
    kLduFirstGroupOrigin + kLduCodewordsPerGroup* kLduCodewordBits + 0U * kLduGroupStride,
    kLduFirstGroupOrigin + kLduCodewordsPerGroup* kLduCodewordBits + 1U * kLduGroupStride,
    kLduFirstGroupOrigin + kLduCodewordsPerGroup* kLduCodewordBits + 2U * kLduGroupStride,
    kLduFirstGroupOrigin + kLduCodewordsPerGroup* kLduCodewordBits + 3U * kLduGroupStride,
    kLduFirstGroupOrigin + kLduCodewordsPerGroup* kLduCodewordBits + 4U * kLduGroupStride,
    kLduFirstGroupOrigin + kLduCodewordsPerGroup* kLduCodewordBits + 5U * kLduGroupStride,
    kLduLowSpeedDataOrigin + kLduLowSpeedDataBits,
};

//! Where codeword bit `index` sits, counted in information bits from the codeword's own start.
[[nodiscard]] inline constexpr std::size_t imbeInterleavedBit(std::size_t index) noexcept {
    const std::size_t row    = index / kImbeInterleaveColumns;
    const std::size_t column = index % kImbeInterleaveColumns;
    return kImbeInterleaveRows * column + ((column & 1U) != 0U ? (row ^ 1U) : row);
}

//! The scrambling sequence, seeded from the decoded first codeword.
struct ImbeScrambler {
    std::uint32_t state{0U};

    //! Start the sequence for a codeword whose first Golay word decoded to `u0`.
    explicit constexpr ImbeScrambler(std::uint16_t u0) noexcept : state(static_cast<std::uint32_t>(u0) << 4U) {}

    //! The next `bits` of the sequence, the first generated the most significant.
    [[nodiscard]] constexpr std::uint32_t next(unsigned bits) noexcept {
        std::uint32_t word = 0U;
        for (unsigned i = 0U; i < bits; ++i) {
            state = (173U * state + 13849U) & 0xFFFFU;
            word  = (word << 1) | ((state >> 15) & 1U);
        }
        return word;
    }
};

//! One codeword's worth of speech parameters, with what the codes had to do to reach them.
struct ImbeParameters {
    //! u0 through u7: four of twelve bits, three of eleven, then seven carried bare.
    std::array<std::uint16_t, kImbeParameterWords> u{};

    unsigned u0_errors{0U}; //!< bits the Golay code repaired in u0, on which all the rest rests
    unsigned errors{0U};    //!< bits repaired across every coded word
};

//! The width of each parameter word, in bits.
inline constexpr std::array<unsigned, kImbeParameterWords> kImbeParameterBits{12U, 12U, 12U, 12U, 11U, 11U, 11U, 7U};

/*!
 * \brief The parameter words as one bit string — the u-vector — and the bytes it occupies.
 *
 * The eight widths sum to 88, which is eleven whole bytes with nothing left over, so a
 * sequence of u-vectors packs end to end with no padding between them and the nth one is at
 * a multiplication rather than a search.
 */
inline constexpr std::size_t kImbeUVectorBits  = 88U;
inline constexpr std::size_t kImbeUVectorBytes = kImbeUVectorBits / 8U;

/*!
 * \brief Pack u0 through u7 into \a bytes.
 *
 * u0 through u7 are packed in the order the codeword carries them: u0 first and u7 last, each
 * word's most significant bit first, the bits running from the top of the first byte down.
 * That is the same order \ref imbeDecodeCodeword lifts them out of the air in, so the packing
 * adds no convention of its own to remember.
 */
inline void imbePackUVector(const ImbeParameters& p, std::uint8_t* bytes) noexcept {
    for (std::size_t i = 0U; i < kImbeUVectorBytes; ++i) {
        bytes[i] = 0U;
    }
    std::size_t at = 0U;
    for (std::size_t w = 0U; w < kImbeParameterWords; ++w) {
        const unsigned width = kImbeParameterBits[w];
        for (unsigned b = 0U; b < width; ++b, ++at) {
            const unsigned bit = (static_cast<unsigned>(p.u[w]) >> (width - 1U - b)) & 1U;
            bytes[at / 8U] |= static_cast<std::uint8_t>(bit << (7U - (at % 8U)));
        }
    }
}

//! The parameter words a packed u-vector carries. The error counts are not part of the
//! packing — they describe what the codes had to do to reach the words, not the words — and
//! read zero.
[[nodiscard]] inline ImbeParameters imbeUnpackUVector(const std::uint8_t* bytes) noexcept {
    ImbeParameters p;
    std::size_t    at = 0U;
    for (std::size_t w = 0U; w < kImbeParameterWords; ++w) {
        const unsigned width = kImbeParameterBits[w];
        std::uint16_t  value = 0U;
        for (unsigned b = 0U; b < width; ++b, ++at) {
            const unsigned bit = (bytes[at / 8U] >> (7U - (at % 8U))) & 1U;
            value              = static_cast<std::uint16_t>((value << 1U) | bit);
        }
        p.u[w] = value;
    }
    return p;
}

//! Lift one codeword's 144 transmitted bits out of a frame's raw dibits, undoing the
//! interleave. `raw` must begin at the frame sync and `origin` is the codeword's first
//! information bit.
inline void gatherImbeCodeword(const std::uint8_t* raw, std::size_t origin, std::array<std::uint8_t, kImbeCodewordBits>& bits) noexcept {
    for (std::size_t i = 0U; i < kImbeCodewordBits; ++i) {
        bits[i] = informationBit(raw, origin + imbeInterleavedBit(i));
    }
}

//! Gather `count` bits into one word, the first the most significant.
[[nodiscard]] inline constexpr std::uint32_t imbeBitsToWord(const std::uint8_t* bits, std::size_t count) noexcept {
    std::uint32_t w = 0U;
    for (std::size_t i = 0U; i < count; ++i) {
        w = (w << 1) | (bits[i] & 1U);
    }
    return w;
}

//! Descramble and error-correct one deinterleaved codeword into speech parameters.
[[nodiscard]] inline ImbeParameters imbeDecodeCodeword(const std::array<std::uint8_t, kImbeCodewordBits>& bits) noexcept {
    ImbeParameters p;

    const fec::GolayResult g0 = fec::golay23Decode(imbeBitsToWord(bits.data(), 23U));
    p.u[0]                    = g0.info;
    p.u0_errors               = g0.errors;
    p.errors                  = g0.errors;

    ImbeScrambler pn{p.u[0]};

    // The three remaining Golay words, each twenty-three bits long and each descrambled with
    // the next twenty-three bits of the sequence before its own errors are counted.
    for (std::size_t k = 1U; k < 4U; ++k) {
        const std::uint32_t    received = imbeBitsToWord(bits.data() + 23U * k, 23U) ^ pn.next(23U);
        const fec::GolayResult g        = fec::golay23Decode(received);
        p.u[k]                          = g.info;
        p.errors += g.errors;
    }

    // The three Hamming words, fifteen bits each, in the same running sequence.
    for (std::size_t k = 0U; k < 3U; ++k) {
        const std::uint32_t      received = imbeBitsToWord(bits.data() + 92U + 15U * k, 15U) ^ pn.next(15U);
        const fec::HammingResult h        = fec::hamming1511Decode(static_cast<std::uint16_t>(received));
        p.u[4U + k]                       = h.info;
        p.errors += h.errors;
    }

    p.u[7] = static_cast<std::uint16_t>(imbeBitsToWord(bits.data() + 137U, 7U));
    return p;
}

//! Decode the `index`th voice codeword of a voice frame whose raw dibits begin at its sync.
[[nodiscard]] inline ImbeParameters imbeDecodeVoiceCodeword(const std::uint8_t* raw, std::size_t index) noexcept {
    std::array<std::uint8_t, kImbeCodewordBits> bits{};
    gatherImbeCodeword(raw, kImbeCodewordOrigin[index], bits);
    return imbeDecodeCodeword(bits);
}

//! Build the 144 transmitted bits carrying one set of speech parameters: the inverse of
//! `imbeDecodeCodeword`, and what a transmitter does.
inline void imbeEncodeCodeword(const ImbeParameters& p, std::array<std::uint8_t, kImbeCodewordBits>& bits) noexcept {
    const auto store = [&bits](std::size_t at, std::size_t count, std::uint32_t word) {
        for (std::size_t i = 0U; i < count; ++i) {
            bits[at + i] = static_cast<std::uint8_t>((word >> (count - 1U - i)) & 1U);
        }
    };

    store(0U, 23U, fec::golay23Encode(static_cast<std::uint16_t>(p.u[0] & 0x0FFFU)));

    ImbeScrambler pn{static_cast<std::uint16_t>(p.u[0] & 0x0FFFU)};
    for (std::size_t k = 1U; k < 4U; ++k) {
        store(23U * k, 23U, fec::golay23Encode(static_cast<std::uint16_t>(p.u[k] & 0x0FFFU)) ^ pn.next(23U));
    }
    for (std::size_t k = 0U; k < 3U; ++k) {
        store(92U + 15U * k, 15U, fec::hamming1511Encode(static_cast<std::uint16_t>(p.u[4U + k] & 0x07FFU)) ^ pn.next(15U));
    }
    store(137U, 7U, p.u[7] & 0x7FU);
}

//! Write one codeword into a voice frame's raw transmitted dibits, applying the interleave.
inline void scatterImbeCodeword(std::uint8_t* raw, std::size_t origin, const std::array<std::uint8_t, kImbeCodewordBits>& bits) noexcept {
    for (std::size_t i = 0U; i < kImbeCodewordBits; ++i) {
        setInformationBit(raw, origin + imbeInterleavedBit(i), bits[i]);
    }
}

//! Place one set of speech parameters into the `index`th voice codeword of a voice frame.
inline void imbeEncodeVoiceCodeword(std::uint8_t* raw, std::size_t index, const ImbeParameters& p) noexcept {
    std::array<std::uint8_t, kImbeCodewordBits> bits{};
    imbeEncodeCodeword(p, bits);
    scatterImbeCodeword(raw, kImbeCodewordOrigin[index], bits);
}

} // namespace gr::p25

#endif // GNURADIO_P25_IMBE_FRAME_HPP
