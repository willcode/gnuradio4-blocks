#ifndef GNURADIO_P25_PAYLOAD_LAYER_HPP
#define GNURADIO_P25_PAYLOAD_LAYER_HPP

// The P25 Phase 1 payload layer: dibits in, decoded messages out, per TIA-102.BAAA.
//
// The frame layer below identifies a frame from its first 57 transmitted symbols and says so
// at once, which is 57 symbols before the shortest frame ends and 807 before the longest
// does. So this layer keeps the dibit stream itself, holds the identification until the whole
// frame has arrived, and only then lifts the payload out and reports both together. A message
// is therefore always complete when a consumer sees it, and the consumer never has to
// correlate an identification with a payload that turns up later.
//
// Only one frame is pending at a time, and a newly identified frame supersedes it. Every frame
// length TIA-102 fixes leaves the next frame's sync exactly at the end of this one, so in
// ordinary traffic a pending frame completes before the next is identified and the two never
// overlap. An overlap
// means the stream broke: either the pending frame's length was not what its identifier said,
// or a sync was found inside it. Either way the newer identification is the better evidence,
// so the pending frame is abandoned, reported as truncated, and the new one takes its place.
// Messages leave this layer in stream order whichever path they took.
//
// A frame whose length TIA-102 does not fix -- the trunking and packet-data identifiers,
// whose block counts live in payloads this layer does not read -- is reported immediately with
// no payload, rather than guessed at. Nothing is inferred from a length that is not known.
//
// The ring holds more symbols than the longest frame, so a pending frame's whole span is
// still in hand at the moment it completes.

#include <array>
#include <cstddef>
#include <cstdint>

#include <gnuradio-4.0/algorithm/fec/ReedSolomon.hpp>
#include <gnuradio-4.0/p25/EncryptionSync.hpp>
#include <gnuradio-4.0/p25/FrameLayer.hpp>
#include <gnuradio-4.0/p25/LinkControl.hpp>
#include <gnuradio-4.0/p25/PayloadBits.hpp>

namespace gr::p25 {

//! Reed-Solomon over GF(64) as P25 Phase 1 uses it: one field and one algorithm serve all
//! three codes, which are fixed entirely by how many parity symbols each carries. These
//! aliases are protocol vocabulary onto the library's shortened GF(64) kernel,
//! `gr::fec::ReedSolomon6`, and they keep their original Roots values and names:
//!
//!   RsLinkControl    RS(24,12,13)  R = 12, corrects 6   Link Control
//!   RsEncryptionSync RS(24,16,9)   R =  8, corrects 4   Encryption Sync
//!   RsHeader         RS(36,20,17)  R = 16, corrects 8   the header that opens a transmission
using RsResult         = gr::fec::RsResult;
using RsLinkControl    = gr::fec::ReedSolomon6<12U>;
using RsEncryptionSync = gr::fec::ReedSolomon6<8U>;
using RsHeader         = gr::fec::ReedSolomon6<16U>;

//! Transmitted symbols the layer retains, the longest frame rounded up to a power of two.
inline constexpr std::size_t kPayloadRingDibits = 1024U;

//! Why a frame carries no decoded payload.
enum class P25PayloadState : std::uint8_t {
    Decoded,       //!< the field-protecting code accepted the frame
    NoPayload,     //!< the identifier carries nothing this layer reads
    LengthUnknown, //!< TIA-102 does not fix this identifier's length
    Truncated,     //!< a later frame arrived before this one finished
    FecFailed,     //!< the Reed-Solomon block was beyond correction
};

struct P25Message {
    P25Frame        frame{};
    P25PayloadState state{P25PayloadState::NoPayload};

    unsigned inner_corrected{0U}; //!< Golay or Hamming codewords the inner code repaired
    unsigned inner_refused{0U};   //!< codewords the inner code placed outside its own code
    unsigned rs_errors{0U};       //!< hexbits the Reed-Solomon decoder corrected

    bool              has_link_control{false};
    P25LinkControl    link_control{};
    bool              has_encryption_sync{false};
    P25EncryptionSync encryption_sync{};

    //! Present only in the header, which shares its Reed-Solomon block with them.
    bool          has_header_fields{false};
    std::uint8_t  header_mfid{0U};
    std::uint16_t header_talkgroup{0U};

    [[nodiscard]] bool decoded() const noexcept { return state == P25PayloadState::Decoded; }
};

//! Whether this layer lifts a payload out of frames bearing this identifier.
[[nodiscard]] inline constexpr bool duidCarriesReadablePayload(std::uint8_t duid) noexcept {
    switch (duid) {
    case static_cast<std::uint8_t>(P25Duid::Hdu):
    case static_cast<std::uint8_t>(P25Duid::Ldu1):
    case static_cast<std::uint8_t>(P25Duid::Ldu2): return true;
    default: return false;
    }
}

struct P25PayloadLayer {
    P25FrameLayer frames{};

    std::uint64_t messages{0U};
    std::uint64_t decoded{0U};
    std::uint64_t fec_failed{0U};
    std::uint64_t truncated{0U};
    std::uint64_t length_unknown{0U};
    //! Reed-Solomon blocks whose padding a correction landed in, which the shortening forbids.
    std::uint64_t rs_pad_corrupted{0U};

    //! Discard everything held about the stream so far, as a tuner having moved requires.
    //!
    //! The absorbed count deliberately survives, because it is a position in the dibit stream
    //! and must keep agreeing with the frame layer's own -- which also keeps counting across a
    //! reset. A frame's reported position is in that shared coordinate system, and two
    //! counters that disagree would have the layer reading a frame's payload out of the wrong
    //! part of its own ring.
    void reset() noexcept {
        frames.reset();
        pending = false;
    }

    //! Absorb one dibit. `handler` is called with a `const P25Message&` for every frame the
    //! layer resolves, in stream order, and returns a `P25FrameAction`.
    template<typename Handler>
    void push(std::uint8_t dibit, Handler&& handler) {
        ring[absorbed % kPayloadRingDibits] = static_cast<std::uint8_t>(dibit & 0x3U);
        ++absorbed;

        bool     arrived = false;
        P25Frame frame{};
        frames.push(dibit, [&arrived, &frame](const P25Frame& f) {
            arrived = true;
            frame   = f;
            return P25FrameAction::Continue;
        });

        if (arrived) {
            if (pending) {
                P25Message message;
                message.frame = pending_frame;
                message.state = P25PayloadState::Truncated;
                ++truncated;
                pending = false;
                if (emit(message, handler)) {
                    return;
                }
            }
            const std::size_t length = duidTransmittedDibits(frame.duid);
            if (duidCarriesReadablePayload(frame.duid) && length != 0U) {
                pending       = true;
                pending_frame = frame;
                pending_end   = frame.dibit_index + length;
            } else {
                P25Message message;
                message.frame = frame;
                if (length == 0U) {
                    message.state = P25PayloadState::LengthUnknown;
                    ++length_unknown;
                } else {
                    message.state = P25PayloadState::NoPayload;
                }
                if (emit(message, handler)) {
                    return;
                }
            }
        }

        if (pending && absorbed >= pending_end) {
            std::array<std::uint8_t, kPayloadRingDibits> span{};
            const std::size_t                            length = duidTransmittedDibits(pending_frame.duid);
            for (std::size_t i = 0U; i < length; ++i) {
                span[i] = ring[(pending_frame.dibit_index + i) % kPayloadRingDibits];
            }
            P25Message message = decodePayload(pending_frame, span.data());
            pending            = false;
            if (emit(message, handler)) {
                return;
            }
        }
    }

    //! Decode one frame's payload from its raw transmitted dibits, which begin at the sync.
    //! Exposed so a consumer holding a whole frame can decode it without running the stream
    //! through the layer.
    [[nodiscard]] P25Message decodePayload(const P25Frame& frame, const std::uint8_t* raw) noexcept {
        P25Message message;
        message.frame = frame;

        if (frame.duid == static_cast<std::uint8_t>(P25Duid::Hdu)) {
            std::array<std::uint8_t, kHduCodewords> hexbits{};
            const P25HexbitBlock                    inner = gatherHduHexbits(raw, hexbits);
            message.inner_corrected                       = inner.corrected;
            message.inner_refused                         = inner.refused;

            RsHeader::Block block{};
            for (std::size_t i = 0U; i < kHduCodewords; ++i) {
                block[kHduRsPad + i] = hexbits[i];
            }
            const RsResult rs = RsHeader::decode(block, kHduRsPad);
            message.rs_errors = rs.errors;
            if (rs.pad_corrupted) {
                ++rs_pad_corrupted;
            }
            if (!rs.valid) {
                message.state = P25PayloadState::FecFailed;
                return message;
            }

            std::array<std::uint8_t, 15U> octets{};
            packHexbits(&block[kHduRsPad], 20U, octets.data());
            message.encryption_sync     = parseHeaderEncryptionSync(octets.data(), message.header_mfid, message.header_talkgroup);
            message.has_encryption_sync = true;
            message.has_header_fields   = true;
            message.state               = P25PayloadState::Decoded;
            return message;
        }

        std::array<std::uint8_t, kLduCodewords> hexbits{};
        const P25HexbitBlock                    inner = gatherLduHexbits(raw, hexbits);
        message.inner_corrected                       = inner.corrected;
        message.inner_refused                         = inner.refused;

        if (frame.duid == static_cast<std::uint8_t>(P25Duid::Ldu1)) {
            RsLinkControl::Block block{};
            for (std::size_t i = 0U; i < kLduCodewords; ++i) {
                block[kLduRsPad + i] = hexbits[i];
            }
            const RsResult rs = RsLinkControl::decode(block, kLduRsPad);
            message.rs_errors = rs.errors;
            if (rs.pad_corrupted) {
                ++rs_pad_corrupted;
            }
            if (!rs.valid) {
                message.state = P25PayloadState::FecFailed;
                return message;
            }
            std::array<std::uint8_t, kLinkControlOctets> octets{};
            packHexbits(&block[kLduRsPad], 12U, octets.data());
            message.link_control     = parseLinkControl(octets.data());
            message.has_link_control = true;
            message.state            = P25PayloadState::Decoded;
            return message;
        }

        RsEncryptionSync::Block block{};
        for (std::size_t i = 0U; i < kLduCodewords; ++i) {
            block[kLduRsPad + i] = hexbits[i];
        }
        const RsResult rs = RsEncryptionSync::decode(block, kLduRsPad);
        message.rs_errors = rs.errors;
        if (rs.pad_corrupted) {
            ++rs_pad_corrupted;
        }
        if (!rs.valid) {
            message.state = P25PayloadState::FecFailed;
            return message;
        }
        std::array<std::uint8_t, 12U> octets{};
        packHexbits(&block[kLduRsPad], 16U, octets.data());
        message.encryption_sync     = parseVoiceEncryptionSync(octets.data());
        message.has_encryption_sync = true;
        message.state               = P25PayloadState::Decoded;
        return message;
    }

private:
    //! Count one message, hand it to the consumer, and say whether the layer was reset under it.
    template<typename Handler>
    bool emit(const P25Message& message, Handler&& handler) {
        ++messages;
        switch (message.state) {
        case P25PayloadState::Decoded: ++decoded; break;
        case P25PayloadState::FecFailed: ++fec_failed; break;
        default: break;
        }
        if (handler(message) == P25FrameAction::Retune) {
            reset();
            return true;
        }
        return false;
    }

    std::array<std::uint8_t, kPayloadRingDibits> ring{};
    std::uint64_t                                absorbed{0U};
    bool                                         pending{false};
    P25Frame                                     pending_frame{};
    std::uint64_t                                pending_end{0U};
};

} // namespace gr::p25

#endif // GNURADIO_P25_PAYLOAD_LAYER_HPP
