#ifndef GNURADIO_P25_FRAME_GEN_HPP
#define GNURADIO_P25_FRAME_GEN_HPP

// A P25 Phase 1 transmitted dibit stream, built from frames whose identity is chosen rather
// than measured, per TIA-102.BAAA.
//
// This is the frame-layer counterpart of the C4FM modulator: it produces exactly what a
// transmitter puts on the air at the symbol level -- frame sync, network identifier with its
// BCH parity, payload, and status symbols interleaved on the 36-symbol period -- so a decoder
// can be checked against the network access code and data unit identifier the stream was
// written with, not against a second decoder's opinion of them. The payload is filler: the
// frame layer identifies frames and does not read their content, and a payload of known
// pseudorandom dibits is a better test of the layer than a payload of zeros, which cannot
// produce a sync-like run by accident.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/p25/FrameSync.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

namespace gr::p25 {

//! The status symbol a repeater emits when its inbound channel is idle.
inline constexpr std::uint8_t kStatusSymbolIdle = 2U;

//! The 64-bit network identifier for a network access code and a data unit identifier: the
//! BCH codeword over the two of them, followed by the identifier's trailing parity bit.
[[nodiscard]] inline std::uint64_t buildNid(std::uint16_t nac, std::uint8_t duid) noexcept {
    const auto info = static_cast<std::uint16_t>(((nac & 0x0FFFU) << 4U) | (duid & 0x0FU));
    return (fec::bch63Encode(info) << 1U) | static_cast<std::uint64_t>(duidParityBit(duid));
}

//! Append one frame's transmitted dibits, status symbols included. `transmitted` overrides the
//! length for the identifiers whose length TIA-102 leaves to the payload; it must be a whole
//! multiple of the status period.
inline void appendP25Frame(std::vector<std::uint8_t>& out, std::uint16_t nac, std::uint8_t duid, std::uint64_t& rng, std::size_t transmitted = 0U) {
    if (transmitted == 0U) {
        transmitted = duidTransmittedDibits(duid);
    }
    if (transmitted == 0U) {
        transmitted = 180U; // one trunking block, the shortest frame whose length is not fixed
    }

    std::vector<std::uint8_t> info;
    info.reserve(informationSpan(transmitted));

    const auto sync = frameSyncDibits();
    for (const std::uint8_t d : sync) {
        info.push_back(d);
    }

    const std::uint64_t nid = buildNid(nac, duid);
    for (std::size_t i = 0U; i < kNidDibits; ++i) {
        const unsigned shift = static_cast<unsigned>(2U * (kNidDibits - 1U - i));
        info.push_back(static_cast<std::uint8_t>((nid >> shift) & 0x3ULL));
    }

    while (info.size() < informationSpan(transmitted)) {
        rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
        info.push_back(static_cast<std::uint8_t>((rng >> 33U) & 0x3ULL));
    }

    // Walk the transmitted positions rather than the information symbols: a frame whose length
    // is a whole multiple of the status period ends on a status symbol, after its last
    // information symbol, so the information alone does not reach the end of the frame. The
    // two counts meet exactly: the positions that are not status positions number
    // informationSpan(transmitted), which is what `info` was filled to.
    std::size_t taken = 0U;
    for (std::size_t i = 0U; i < transmitted; ++i) {
        out.push_back(isStatusSymbol(i) ? kStatusSymbolIdle : info[taken++]);
    }
}

//! A stream of frames cycling through `duids`, all carrying the same network access code.
[[nodiscard]] inline std::vector<std::uint8_t> p25FrameStream(std::uint16_t nac, const std::vector<std::uint8_t>& duids, std::uint64_t seed = 1234567U) {
    std::vector<std::uint8_t> out;
    std::uint64_t             rng = seed;
    for (const std::uint8_t duid : duids) {
        appendP25Frame(out, nac, duid, rng);
    }
    return out;
}

} // namespace gr::p25

#endif // GNURADIO_P25_FRAME_GEN_HPP
