#ifndef GNURADIO_P25_PAYLOAD_GEN_HPP
#define GNURADIO_P25_PAYLOAD_GEN_HPP

// P25 Phase 1 frames built with a payload that was chosen, per TIA-102.BAAA.
//
// The frame generator alongside this one fills payloads with pseudorandom filler, which is
// what a layer that only identifies frames wants. A layer that reads them needs the opposite:
// a frame whose talkgroup, source unit, algorithm and key are known before any decoding
// happens, so that what comes out can be compared against a value nothing measured. Every
// encoder here is the exact inverse of the decoder it will be checked against -- the same bit
// positions, the same inner code, the same Reed-Solomon shortening -- run in the other
// direction, so a round trip through the pair exercises the layout, both codes and the field
// packing at once.
//
// Voice codewords are filler here. This produces frames for a payload decoder, not audio.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/algorithm/fec/Golay.hpp>
#include <gnuradio-4.0/algorithm/fec/Hamming.hpp>
#include <gnuradio-4.0/p25/EncryptionSync.hpp>
#include <gnuradio-4.0/p25/FrameGen.hpp>
#include <gnuradio-4.0/p25/LinkControl.hpp>
#include <gnuradio-4.0/p25/PayloadBits.hpp>
#include <gnuradio-4.0/p25/PayloadLayer.hpp>

namespace gr::p25 {

//! What a header carries beyond the encryption state it shares with the voice frames.
struct P25HeaderMessage {
    P25EncryptionSync encryption_sync{};
    std::uint8_t      mfid{0U};
    std::uint16_t     talkgroup{0U};
};

namespace detail {

//! One frame's worth of transmitted dibits: sync, network identifier, status symbols on the
//! period, and pseudorandom filler everywhere a payload has not been written yet.
inline std::vector<std::uint8_t> frameSkeleton(std::uint16_t nac, std::uint8_t duid, std::uint64_t& rng) {
    std::vector<std::uint8_t> raw;
    appendP25Frame(raw, nac, duid, rng);
    return raw;
}

//! Encode 20, 16 or 12 information hexbits into a full Reed-Solomon block.
template<typename Code>
inline typename Code::Block encodeRsBlock(const std::uint8_t* hexbits, std::size_t count, std::size_t pad) {
    typename Code::Block block{};
    for (std::size_t i = 0U; i < count; ++i) {
        block[pad + i] = static_cast<std::uint8_t>(hexbits[i] & 0x3FU);
    }
    Code::encode(block, pad);
    return block;
}

} // namespace detail

//! A header frame carrying a chosen encryption state, manufacturer and talkgroup.
[[nodiscard]] inline std::vector<std::uint8_t> buildHduFrame(std::uint16_t nac, const P25HeaderMessage& message, std::uint64_t& rng) {
    std::array<std::uint8_t, 15U> octets{};
    for (std::size_t i = 0U; i < kMessageIndicatorOctets; ++i) {
        octets[i] = message.encryption_sync.message_indicator[i];
    }
    octets[9]  = message.mfid;
    octets[10] = message.encryption_sync.algid;
    octets[11] = static_cast<std::uint8_t>(message.encryption_sync.keyid >> 8);
    octets[12] = static_cast<std::uint8_t>(message.encryption_sync.keyid & 0xFFU);
    octets[13] = static_cast<std::uint8_t>(message.talkgroup >> 8);
    octets[14] = static_cast<std::uint8_t>(message.talkgroup & 0xFFU);

    std::array<std::uint8_t, 20U> hexbits{};
    unpackHexbits(octets.data(), octets.size(), hexbits.data());

    const RsHeader::Block block = detail::encodeRsBlock<RsHeader>(hexbits.data(), hexbits.size(), kHduRsPad);

    std::vector<std::uint8_t> raw = detail::frameSkeleton(nac, static_cast<std::uint8_t>(P25Duid::Hdu), rng);
    for (std::size_t i = 0U; i < kHduCodewords; ++i) {
        const std::uint32_t cw = fec::golay18Encode(block[kHduRsPad + i]);
        setInformationWord(raw.data(), kHduPayloadOrigin + i * kHduCodewordBits, kHduCodewordBits, cw);
    }
    return raw;
}

namespace detail {

//! Lay 24 hexbits into a voice frame's six groups of four Hamming codewords.
inline void writeLduHexbits(std::vector<std::uint8_t>& raw, const std::uint8_t* hexbits) {
    for (std::size_t g = 0U; g < kLduGroups; ++g) {
        const std::size_t origin = kLduFirstGroupOrigin + g * kLduGroupStride;
        for (std::size_t c = 0U; c < kLduCodewordsPerGroup; ++c) {
            const std::uint16_t cw = fec::hamming1063Encode(hexbits[g * kLduCodewordsPerGroup + c]);
            setInformationWord(raw.data(), origin + c * kLduCodewordBits, kLduCodewordBits, cw);
        }
    }
}

} // namespace detail

//! A first voice frame carrying a chosen Link Control word.
[[nodiscard]] inline std::vector<std::uint8_t> buildLdu1Frame(std::uint16_t nac, const std::array<std::uint8_t, kLinkControlOctets>& octets, std::uint64_t& rng) {
    std::array<std::uint8_t, 12U> hexbits{};
    unpackHexbits(octets.data(), octets.size(), hexbits.data());

    const RsLinkControl::Block block = detail::encodeRsBlock<RsLinkControl>(hexbits.data(), hexbits.size(), kLduRsPad);

    std::vector<std::uint8_t> raw = detail::frameSkeleton(nac, static_cast<std::uint8_t>(P25Duid::Ldu1), rng);
    detail::writeLduHexbits(raw, &block[kLduRsPad]);
    return raw;
}

//! A second voice frame carrying a chosen encryption state.
[[nodiscard]] inline std::vector<std::uint8_t> buildLdu2Frame(std::uint16_t nac, const P25EncryptionSync& es, std::uint64_t& rng) {
    std::array<std::uint8_t, 12U> octets{};
    for (std::size_t i = 0U; i < kMessageIndicatorOctets; ++i) {
        octets[i] = es.message_indicator[i];
    }
    octets[9]  = es.algid;
    octets[10] = static_cast<std::uint8_t>(es.keyid >> 8);
    octets[11] = static_cast<std::uint8_t>(es.keyid & 0xFFU);

    std::array<std::uint8_t, 16U> hexbits{};
    unpackHexbits(octets.data(), octets.size(), hexbits.data());

    const RsEncryptionSync::Block block = detail::encodeRsBlock<RsEncryptionSync>(hexbits.data(), hexbits.size(), kLduRsPad);

    std::vector<std::uint8_t> raw = detail::frameSkeleton(nac, static_cast<std::uint8_t>(P25Duid::Ldu2), rng);
    detail::writeLduHexbits(raw, &block[kLduRsPad]);
    return raw;
}

//! Nine Link Control octets for a group voice call, the message a conventional repeater sends.
[[nodiscard]] inline std::array<std::uint8_t, kLinkControlOctets> groupVoiceLinkControl(std::uint16_t talkgroup, std::uint32_t sourceUnit, std::uint8_t mfid = 0U, std::uint8_t serviceOptions = 0U) {
    std::array<std::uint8_t, kLinkControlOctets> octets{};
    octets[0] = static_cast<std::uint8_t>(P25Lco::GroupVoiceChannelUser);
    octets[1] = mfid;
    octets[2] = serviceOptions;
    octets[3] = 0U;
    octets[4] = static_cast<std::uint8_t>(talkgroup >> 8);
    octets[5] = static_cast<std::uint8_t>(talkgroup & 0xFFU);
    octets[6] = static_cast<std::uint8_t>((sourceUnit >> 16) & 0xFFU);
    octets[7] = static_cast<std::uint8_t>((sourceUnit >> 8) & 0xFFU);
    octets[8] = static_cast<std::uint8_t>(sourceUnit & 0xFFU);
    return octets;
}

} // namespace gr::p25

#endif // GNURADIO_P25_PAYLOAD_GEN_HPP
