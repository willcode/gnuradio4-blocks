#ifndef GNURADIO_P25_ENCRYPTION_SYNC_HPP
#define GNURADIO_P25_ENCRYPTION_SYNC_HPP

// The P25 Phase 1 Encryption Sync, per TIA-102.BAAA and TIA-102.AAAD.
//
// Two frames carry it and they carry the same three things: a 72-bit message indicator, an
// 8-bit algorithm identifier and a 16-bit key identifier. The header that opens a
// transmission carries them alongside the manufacturer and the talkgroup, protected by
// RS(36,20,17); the second of each pair of voice frames repeats them alone, protected by
// RS(24,16,9), so a receiver joining a transmission late still learns them within 360 ms.
//
//   header, 20 information hexbits = 15 octets
//     octets 0..8   message indicator
//     octet  9      manufacturer
//     octet  10     algorithm identifier
//     octets 11..12 key identifier
//     octets 13..14 talkgroup
//
//   voice frame, 16 information hexbits = 12 octets
//     octets 0..8   message indicator
//     octet  9      algorithm identifier
//     octets 10..11 key identifier
//
// Algorithm identifier 0x80 signals the absence of encryption rather than a cipher, and it is
// the only value for which the voice can be decoded. Every other value names a cipher and is
// reported as such: the identifier and its key are carried upward unchanged. Reducing that to
// a single encrypted flag would discard the two things a listener actually wants — which
// algorithm, and which key — and they cost nothing to keep.
//
// The two paths are independent all the way down: different frames, different inner codes,
// different Reed-Solomon lengths. Where both decode they must agree, and a receiver that sees
// them disagree has learned something real about its own decoding rather than about the air.

#include <array>
#include <cstddef>
#include <cstdint>

namespace gr::p25 {

//! Octets of message indicator.
inline constexpr std::size_t kMessageIndicatorOctets = 9U;

//! The algorithm identifier standing for unencrypted traffic.
inline constexpr std::uint8_t kAlgidClear = 0x80U;

struct P25EncryptionSync {
    std::array<std::uint8_t, kMessageIndicatorOctets> message_indicator{};
    std::uint8_t                                      algid{kAlgidClear};
    std::uint16_t                                     keyid{0U};

    [[nodiscard]] bool encrypted() const noexcept { return algid != kAlgidClear; }
};

//! Interpret the header's 15 information octets. The talkgroup and manufacturer the header
//! also carries are returned through the out parameters, which is where they belong: they are
//! not part of the encryption state and only share a Reed-Solomon block with it.
[[nodiscard]] inline P25EncryptionSync parseHeaderEncryptionSync(const std::uint8_t* octets, std::uint8_t& mfid, std::uint16_t& talkgroup) noexcept {
    P25EncryptionSync es;
    for (std::size_t i = 0U; i < kMessageIndicatorOctets; ++i) {
        es.message_indicator[i] = octets[i];
    }
    mfid      = octets[9];
    es.algid  = octets[10];
    es.keyid  = static_cast<std::uint16_t>((static_cast<std::uint16_t>(octets[11]) << 8) | octets[12]);
    talkgroup = static_cast<std::uint16_t>((static_cast<std::uint16_t>(octets[13]) << 8) | octets[14]);
    return es;
}

//! Interpret a voice frame's 12 information octets.
[[nodiscard]] inline P25EncryptionSync parseVoiceEncryptionSync(const std::uint8_t* octets) noexcept {
    P25EncryptionSync es;
    for (std::size_t i = 0U; i < kMessageIndicatorOctets; ++i) {
        es.message_indicator[i] = octets[i];
    }
    es.algid = octets[9];
    es.keyid = static_cast<std::uint16_t>((static_cast<std::uint16_t>(octets[10]) << 8) | octets[11]);
    return es;
}

//! The name TIA-102 gives an algorithm identifier, or a null pointer where it names none.
[[nodiscard]] inline constexpr const char* algidName(std::uint8_t algid) noexcept {
    switch (algid) {
    case 0x80U: return "clear";
    case 0x81U: return "DES-OFB";
    case 0x83U: return "TDEA-3key";
    case 0x84U: return "AES-256";
    case 0x85U: return "AES-128";
    case 0xAAU: return "ADP/RC4";
    default: return nullptr;
    }
}

} // namespace gr::p25

#endif // GNURADIO_P25_ENCRYPTION_SYNC_HPP
