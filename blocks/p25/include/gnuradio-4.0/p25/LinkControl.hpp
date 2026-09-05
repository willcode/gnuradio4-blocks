#ifndef GNURADIO_P25_LINK_CONTROL_HPP
#define GNURADIO_P25_LINK_CONTROL_HPP

// The P25 Phase 1 Link Control word, per TIA-102.BAAA and TIA-102.AABF.
//
// Nine octets ride in the first of each pair of voice frames, protected by RS(24,12,13) over
// hexbits. The first octet is shared out three ways and the rest depend on what it says:
//
//   octet 0   bit 7 protected, bit 6 implicit manufacturer, bits 5..0 link control opcode
//   octet 1   manufacturer, meaningful only where the implicit bit says the format is not
//   octet 2   service options
//   octet 3   reserved
//   octets 4..8  interpreted by the opcode
//
// Opcode 0 is the group voice channel user, which is what a conventional repeater sends and
// the only one whose octets 4..8 are read here: a 16-bit talkgroup and a 24-bit source unit.
// The remaining opcodes are carried as raw octets rather than guessed at -- a talkgroup read
// out of a message that does not contain one is worse than no talkgroup at all.

#include <array>
#include <cstddef>
#include <cstdint>

namespace gr::p25 {

//! Octets a Link Control word occupies.
inline constexpr std::size_t kLinkControlOctets = 9U;

//! Link control opcodes this decoder reads fields out of.
enum class P25Lco : std::uint8_t {
    GroupVoiceChannelUser = 0x00U,
};

struct P25LinkControl {
    std::array<std::uint8_t, kLinkControlOctets> octets{};

    bool         protected_flag{false}; //!< the message itself is encrypted
    bool         implicit_mfid{false};  //!< the format is the standard's, not a manufacturer's
    std::uint8_t lco{0U};
    std::uint8_t mfid{0U};
    std::uint8_t service_options{0U};

    bool          has_addresses{false}; //!< the opcode is one whose address fields are defined
    std::uint16_t talkgroup{0U};
    std::uint32_t source_unit{0U};

    [[nodiscard]] bool isGroupVoice() const noexcept { return lco == static_cast<std::uint8_t>(P25Lco::GroupVoiceChannelUser); }
};

//! Interpret nine Link Control octets.
[[nodiscard]] inline P25LinkControl parseLinkControl(const std::uint8_t* octets) noexcept {
    P25LinkControl lc;
    for (std::size_t i = 0U; i < kLinkControlOctets; ++i) {
        lc.octets[i] = octets[i];
    }
    lc.protected_flag  = ((octets[0] & 0x80U) != 0U);
    lc.implicit_mfid   = ((octets[0] & 0x40U) != 0U);
    lc.lco             = static_cast<std::uint8_t>(octets[0] & 0x3FU);
    lc.mfid            = octets[1];
    lc.service_options = octets[2];

    if (lc.isGroupVoice()) {
        lc.has_addresses = true;
        lc.talkgroup     = static_cast<std::uint16_t>((static_cast<std::uint16_t>(octets[4]) << 8) | octets[5]);
        lc.source_unit   = (static_cast<std::uint32_t>(octets[6]) << 16) | (static_cast<std::uint32_t>(octets[7]) << 8) | static_cast<std::uint32_t>(octets[8]);
    }
    return lc;
}

} // namespace gr::p25

#endif // GNURADIO_P25_LINK_CONTROL_HPP
