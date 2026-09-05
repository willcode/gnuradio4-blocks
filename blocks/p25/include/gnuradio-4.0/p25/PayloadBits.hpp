#ifndef GNURADIO_P25_PAYLOAD_BITS_HPP
#define GNURADIO_P25_PAYLOAD_BITS_HPP

// Where a P25 Phase 1 frame keeps its fields, per TIA-102.BAAA, and how they are lifted out.
//
// A frame is described here in information bits -- transmitted symbols with the status
// symbols taken out -- because that is the coordinate system the standard's frame layouts are
// written in and the only one in which the fields sit at fixed offsets. `StatusSymbol.hpp`
// owns the conversion between the two.
//
// Every frame opens the same way: 48 bits of sync, then the 64-bit network identifier, so the
// first 112 information bits are accounted for before any payload begins.
//
// The header, which opens a transmission and is 396 transmitted symbols long:
//
//   information bits 112 .. 759   36 codewords of Golay(18,6,8), 18 bits each, in order
//   information bits 760 .. 769   reserved
//
// The 36 hexbits they carry are a Reed-Solomon RS(36,20,17) codeword, and its 20 information
// hexbits are the message indicator, the manufacturer, the algorithm and key identifiers and
// the talkgroup.
//
// The voice frames, both 864 transmitted symbols long, interleave their 144-bit voice
// codewords with the field the frame exists to carry:
//
//   information bits  112       nine voice codewords of 144 bits, and between them
//   information bits  400 + 184k   for k = 0..5, forty bits each: four codewords of
//                                  Hamming(10,6,3) carrying four hexbits
//   information bits 1504 .. 1535  low speed data
//
// The 24 hexbits are a Reed-Solomon codeword whose length depends on which voice frame this
// is: RS(24,12,13) carrying Link Control in the first, RS(24,16,9) carrying the Encryption
// Sync in the second. The two are the same 240 bits in the same places and differ only in how
// many of the hexbits are parity.
//
// The stride of 184 is not arbitrary and is worth keeping in view: it is one voice codeword
// of 144 bits plus one group of 40, which is the whole of the interleaving pattern.

#include <array>
#include <cstddef>
#include <cstdint>

#include <gnuradio-4.0/algorithm/fec/Golay.hpp>
#include <gnuradio-4.0/algorithm/fec/Hamming.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

namespace gr::p25 {

//! Information bits consumed by the sync and the network identifier, before any payload.
inline constexpr std::size_t kFrameHeaderInfoBits = 112U;

//! The header's Golay codewords.
inline constexpr std::size_t kHduCodewords     = 36U;
inline constexpr std::size_t kHduCodewordBits  = 18U;
inline constexpr std::size_t kHduPayloadOrigin = kFrameHeaderInfoBits;

//! The voice frames' Hamming codewords, and where the groups of four sit.
inline constexpr std::size_t kLduCodewords         = 24U;
inline constexpr std::size_t kLduCodewordBits      = 10U;
inline constexpr std::size_t kLduGroups            = 6U;
inline constexpr std::size_t kLduCodewordsPerGroup = 4U;
inline constexpr std::size_t kLduFirstGroupOrigin  = 400U;
inline constexpr std::size_t kLduVoiceCodewordBits = 144U;
inline constexpr std::size_t kLduGroupStride       = kLduVoiceCodewordBits + kLduCodewordsPerGroup * kLduCodewordBits;

//! Where the Reed-Solomon block's information hexbits begin once the shortening is restored.
inline constexpr std::size_t kHduRsPad = 27U; //!< 63 - 36
inline constexpr std::size_t kLduRsPad = 39U; //!< 63 - 24

//! The transmitted symbol carrying information symbol `index`, counted from a frame sync.
[[nodiscard]] inline constexpr std::size_t transmittedIndexOf(std::size_t index) noexcept { return index + index / kInformationPerPeriod; }

//! One information bit out of a frame's raw transmitted dibits, which must begin at the sync.
[[nodiscard]] inline constexpr std::uint8_t informationBit(const std::uint8_t* raw, std::size_t bit) noexcept {
    const std::uint8_t dibit = raw[transmittedIndexOf(bit / 2U)];
    return static_cast<std::uint8_t>((dibit >> (1U - (bit & 1U))) & 1U);
}

//! Gather `bits` information bits starting at `origin` into one word, first bit most significant.
[[nodiscard]] inline constexpr std::uint32_t informationWord(const std::uint8_t* raw, std::size_t origin, std::size_t bits) noexcept {
    std::uint32_t w = 0U;
    for (std::size_t i = 0U; i < bits; ++i) {
        w = (w << 1) | informationBit(raw, origin + i);
    }
    return w;
}

//! Write one information bit into a frame's raw transmitted dibits.
inline constexpr void setInformationBit(std::uint8_t* raw, std::size_t bit, std::uint8_t value) noexcept {
    const std::size_t  index = transmittedIndexOf(bit / 2U);
    const unsigned     shift = 1U - static_cast<unsigned>(bit & 1U);
    const std::uint8_t mask  = static_cast<std::uint8_t>(1U << shift);
    raw[index]               = static_cast<std::uint8_t>((raw[index] & ~mask) | ((value & 1U) << shift));
}

//! Write `bits` information bits starting at `origin`, the most significant first.
inline constexpr void setInformationWord(std::uint8_t* raw, std::size_t origin, std::size_t bits, std::uint32_t word) noexcept {
    for (std::size_t i = 0U; i < bits; ++i) {
        setInformationBit(raw, origin + i, static_cast<std::uint8_t>((word >> (bits - 1U - i)) & 1U));
    }
}

//! What the inner code made of a frame's codewords, alongside the hexbits themselves.
struct P25HexbitBlock {
    unsigned corrected{0U}; //!< codewords the inner code repaired
    unsigned refused{0U};   //!< codewords the inner code could not place inside its own code
};

//! Lift the header's 36 hexbits, decoding each Golay(18,6,8) codeword.
inline P25HexbitBlock gatherHduHexbits(const std::uint8_t* raw, std::array<std::uint8_t, kHduCodewords>& hexbits) noexcept {
    P25HexbitBlock block;
    for (std::size_t i = 0U; i < kHduCodewords; ++i) {
        const std::uint32_t    cw = informationWord(raw, kHduPayloadOrigin + i * kHduCodewordBits, kHduCodewordBits);
        const fec::GolayResult r  = fec::golay18Decode(cw);
        hexbits[i]                = static_cast<std::uint8_t>(r.info);
        if (!r.valid) {
            ++block.refused;
        } else if (r.errors != 0U) {
            ++block.corrected;
        }
    }
    return block;
}

//! Lift a voice frame's 24 hexbits, decoding each Hamming(10,6,3) codeword.
inline P25HexbitBlock gatherLduHexbits(const std::uint8_t* raw, std::array<std::uint8_t, kLduCodewords>& hexbits) noexcept {
    P25HexbitBlock block;
    for (std::size_t g = 0U; g < kLduGroups; ++g) {
        const std::size_t origin = kLduFirstGroupOrigin + g * kLduGroupStride;
        for (std::size_t c = 0U; c < kLduCodewordsPerGroup; ++c) {
            const std::uint32_t      cw            = informationWord(raw, origin + c * kLduCodewordBits, kLduCodewordBits);
            const fec::HammingResult r             = fec::hamming1063Decode(static_cast<std::uint16_t>(cw));
            hexbits[g * kLduCodewordsPerGroup + c] = static_cast<std::uint8_t>(r.info);
            if (!r.valid) {
                ++block.refused;
            } else if (r.errors != 0U) {
                ++block.corrected;
            }
        }
    }
    return block;
}

//! Concatenate six-bit symbols into a byte stream, the first symbol most significant. Four
//! hexbits make three bytes exactly, which is why every field length the standard fixes for
//! these messages is a multiple of 24 bits.
inline void packHexbits(const std::uint8_t* hexbits, std::size_t count, std::uint8_t* bytes) noexcept {
    std::uint32_t acc  = 0U;
    unsigned      held = 0U;
    std::size_t   out  = 0U;
    for (std::size_t i = 0U; i < count; ++i) {
        acc = (acc << 6) | (hexbits[i] & 0x3FU);
        held += 6U;
        while (held >= 8U) {
            held -= 8U;
            bytes[out++] = static_cast<std::uint8_t>((acc >> held) & 0xFFU);
        }
    }
}

//! Split a byte stream into six-bit symbols, the inverse of the packing above. The byte count
//! must divide into whole hexbits, which every field length the standard fixes here does.
inline void unpackHexbits(const std::uint8_t* bytes, std::size_t octets, std::uint8_t* hexbits) noexcept {
    std::uint32_t acc  = 0U;
    unsigned      held = 0U;
    std::size_t   out  = 0U;
    for (std::size_t i = 0U; i < octets; ++i) {
        acc = (acc << 8) | bytes[i];
        held += 8U;
        while (held >= 6U) {
            held -= 6U;
            hexbits[out++] = static_cast<std::uint8_t>((acc >> held) & 0x3FU);
        }
    }
}

} // namespace gr::p25

#endif // GNURADIO_P25_PAYLOAD_BITS_HPP
