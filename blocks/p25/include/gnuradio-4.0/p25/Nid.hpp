#ifndef GNURADIO_P25_NID_HPP
#define GNURADIO_P25_NID_HPP

// The P25 Phase 1 network identifier and the data unit identifiers it carries, per
// TIA-102.BAAA.
//
// Every frame opens with a 48-bit sync and then a 64-bit network identifier. The identifier is
// a BCH(63,16,23) codeword plus one trailing bit:
//
//   bits 63..52   network access code, 12 bits, identifying the system
//   bits 51..48   data unit identifier, 4 bits, saying what kind of frame this is
//   bits 47..1    the BCH parity over those 16 bits
//   bit  0        a parity bit outside the BCH codeword
//
// numbering from the first bit transmitted. Because the identifier is transmitted in the
// symbol stream rather than in frame content, a status symbol falls inside it -- at transmitted
// symbol 35, counted from the frame sync -- so the 32 symbols of the identifier occupy 33
// transmitted symbols and the status symbol must be removed before the 64 bits mean anything.
//
// The trailing bit is a linear function of the information bits, and it reduces to the
// exclusive-or of the two least significant bits of the data unit identifier: it is 0 for the
// header, terminator, trunking and data identifiers and 1 for the two voice ones. It is not
// covered by the BCH parity, so a single wrong symbol can break it while the codeword itself
// still decodes perfectly. That makes it a corroborating check rather than a gate -- but it is
// a genuinely independent one, since it is the only place the identifier says anything about
// itself twice.

#include <cstddef>
#include <cstdint>

#include <gnuradio-4.0/algorithm/fec/Bch.hpp>
#include <gnuradio-4.0/p25/FrameSync.hpp>

namespace gr::p25 {

//! Symbols of network identifier, and the transmitted symbols they occupy once the status
//! symbol inside them is counted.
inline constexpr std::size_t kNidDibits            = 32U;
inline constexpr std::size_t kNidTransmittedDibits = 33U;

//! Where the status symbol falls inside those 33, counting from the first identifier symbol.
inline constexpr std::size_t kNidStatusOffset = 11U;

//! Transmitted symbols from the start of a frame sync to the end of its network identifier.
inline constexpr std::size_t kFrameHeaderDibits = kFrameSyncDibits + kNidTransmittedDibits;

//! What kind of frame follows the identifier.
enum class P25Duid : std::uint8_t {
    Hdu   = 0x0U, //!< header, opening a voice transmission
    Tdu   = 0x3U, //!< terminator without link control
    Ldu1  = 0x5U, //!< voice frame carrying link control
    Tsbk  = 0x7U, //!< trunking signaling block
    Ldu2  = 0xAU, //!< voice frame carrying the encryption sync
    Pdu   = 0xCU, //!< packet data
    Tdu15 = 0xFU, //!< terminator with link control
};

//! The coarse classification a consumer dispatches on.
enum class P25FrameKind : std::uint8_t { Unknown, VoiceHeader, Voice, Terminator, Control, Data };

//! Whether TIA-102 defines this identifier at all. The nine undefined values are the frame
//! layer's cheapest rejection of a network identifier that decoded to a codeword but cannot
//! have come from a transmitter.
[[nodiscard]] inline constexpr bool duidIsDefined(std::uint8_t duid) noexcept {
    switch (duid) {
    case static_cast<std::uint8_t>(P25Duid::Hdu):
    case static_cast<std::uint8_t>(P25Duid::Tdu):
    case static_cast<std::uint8_t>(P25Duid::Ldu1):
    case static_cast<std::uint8_t>(P25Duid::Tsbk):
    case static_cast<std::uint8_t>(P25Duid::Ldu2):
    case static_cast<std::uint8_t>(P25Duid::Pdu):
    case static_cast<std::uint8_t>(P25Duid::Tdu15): return true;
    default: return false;
    }
}

//! The identifier's trailing parity bit.
[[nodiscard]] inline constexpr std::uint8_t duidParityBit(std::uint8_t duid) noexcept { return static_cast<std::uint8_t>(((duid >> 1U) ^ duid) & 0x1U); }

[[nodiscard]] inline constexpr P25FrameKind duidKind(std::uint8_t duid) noexcept {
    switch (duid) {
    case static_cast<std::uint8_t>(P25Duid::Hdu): return P25FrameKind::VoiceHeader;
    case static_cast<std::uint8_t>(P25Duid::Ldu1):
    case static_cast<std::uint8_t>(P25Duid::Ldu2): return P25FrameKind::Voice;
    case static_cast<std::uint8_t>(P25Duid::Tdu):
    case static_cast<std::uint8_t>(P25Duid::Tdu15): return P25FrameKind::Terminator;
    case static_cast<std::uint8_t>(P25Duid::Tsbk): return P25FrameKind::Control;
    case static_cast<std::uint8_t>(P25Duid::Pdu): return P25FrameKind::Data;
    default: return P25FrameKind::Unknown;
    }
}

[[nodiscard]] inline constexpr const char* duidName(std::uint8_t duid) noexcept {
    switch (duid) {
    case static_cast<std::uint8_t>(P25Duid::Hdu): return "HDU";
    case static_cast<std::uint8_t>(P25Duid::Tdu): return "TDU";
    case static_cast<std::uint8_t>(P25Duid::Ldu1): return "LDU1";
    case static_cast<std::uint8_t>(P25Duid::Tsbk): return "TSBK";
    case static_cast<std::uint8_t>(P25Duid::Ldu2): return "LDU2";
    case static_cast<std::uint8_t>(P25Duid::Pdu): return "PDU";
    case static_cast<std::uint8_t>(P25Duid::Tdu15): return "TDU15";
    default: return "?";
    }
}

//! Transmitted symbols a frame of this kind occupies, status symbols included, or zero where
//! TIA-102 does not fix the length. Every fixed length is a whole multiple of the 36-symbol
//! status period, which is what lets the status phase be re-derived at each frame sync. The
//! two variable ones carry a block count in their payload: a trunking frame runs 180 symbols
//! per block up to three blocks, and packet data is longer still.
[[nodiscard]] inline constexpr std::size_t duidTransmittedDibits(std::uint8_t duid) noexcept {
    switch (duid) {
    case static_cast<std::uint8_t>(P25Duid::Hdu): return 396U;
    case static_cast<std::uint8_t>(P25Duid::Tdu): return 72U;
    case static_cast<std::uint8_t>(P25Duid::Ldu1):
    case static_cast<std::uint8_t>(P25Duid::Ldu2): return 864U;
    case static_cast<std::uint8_t>(P25Duid::Tdu15): return 216U;
    default: return 0U;
    }
}

struct P25Nid {
    bool          valid{false};     //!< the BCH decode succeeded inside the accepted distance
    std::uint16_t nac{0U};          //!< network access code
    std::uint8_t  duid{0U};         //!< data unit identifier
    unsigned      bch_errors{0U};   //!< bits corrected, or the distance found when rejected
    bool          parity_ok{false}; //!< the trailing bit agreed with the recovered identifier
    bool          duid_defined{false};
};

//! Decode a 64-bit network identifier, its first transmitted bit in bit 63.
[[nodiscard]] inline P25Nid decodeNid(std::uint64_t nid, unsigned maxErrors = fec::kBch63Correctable) noexcept {
    const fec::Bch63Result bch = fec::bch63Decode(nid >> 1U, maxErrors);

    P25Nid n;
    n.bch_errors   = bch.errors;
    n.valid        = bch.valid;
    n.nac          = static_cast<std::uint16_t>((bch.info >> 4U) & 0x0FFFU);
    n.duid         = static_cast<std::uint8_t>(bch.info & 0x0FU);
    n.parity_ok    = (static_cast<std::uint8_t>(nid & 0x1U) == duidParityBit(n.duid));
    n.duid_defined = duidIsDefined(n.duid);
    return n;
}

} // namespace gr::p25

#endif // GNURADIO_P25_NID_HPP
