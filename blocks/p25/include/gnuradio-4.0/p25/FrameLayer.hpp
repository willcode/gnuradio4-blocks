#ifndef GNURADIO_P25_FRAME_LAYER_HPP
#define GNURADIO_P25_FRAME_LAYER_HPP

// The P25 Phase 1 frame layer: dibits in, identified frames out, per TIA-102.BAAA.
//
// The layer turns a continuous dibit stream into frames by finding the 48-bit sync, removing
// the status symbol that falls inside the network identifier, and decoding that identifier's
// BCH codeword to recover the network access code and the data unit identifier.
//
// A frame sync match is a candidate rather than a confirmed frame. The sync pattern uses only
// the two outer deviations, so a demodulator whose level tracking has widened in noise can
// produce outer-symbol runs that match it by accident, and a frame layer that reported one
// frame per sync would report traffic on an empty channel. What settles it is the network
// identifier: BCH(63,16,23) admits 65536 codewords out of 2^63 words, so a 63-bit field that
// was never transmitted has almost no chance of lying within the code's correcting radius of
// any of them. Every sync is therefore carried to a decode and only a decode promotes it to a
// frame. Three further checks cost nothing and are applied for the same reason: the trailing
// parity bit must agree with the recovered identifier, the identifier must be one TIA-102
// actually defines, and the correction distance must be inside the configured limit.
//
// The search runs at every dibit position and never skips ahead over a frame it has already
// identified. Skipping would be cheaper and would suppress a sync-like pattern occurring
// inside a payload, but it would also make the layer's recovery from a lost frame depend on
// the length of the frame it last believed in. Running the correlator continuously means the
// layer reacquires on the first undamaged sync after any dropout, which is what a signal that
// arrives in bursts with dead air between them needs. A candidate at a neighboring offset is
// not a problem to be deduplicated: its identifier field is shifted with it, so it fails the
// decode like any other pattern that was not a frame.
//
// The layer's own delay is what lets the correlator be a shift register rather than a search.
// It keeps the last 33 dibits -- exactly the identifier's transmitted span -- and correlates a
// window that ends 33 dibits in the past, so at the moment a sync is recognized its identifier
// is already in hand and nothing has to be remembered per candidate.

#include <array>
#include <cstddef>
#include <cstdint>

#include <gnuradio-4.0/p25/FrameSync.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

namespace gr::p25 {

struct P25Frame {
    std::uint64_t dibit_index{0U}; //!< stream position of the frame sync's first dibit
    std::uint16_t nac{0U};
    std::uint8_t  duid{0U};
    P25FrameKind  kind{P25FrameKind::Unknown};
    unsigned      sync_errors{0U}; //!< dibits of the 24 that differed from the sync pattern
    unsigned      bch_errors{0U};  //!< bits the identifier's BCH decode corrected
    bool          parity_ok{false};
};

//! What a frame consumer asks the layer to do once it has looked at a frame.
//!
//! `Retune` exists so that a consumer which needs the receiver to move -- following a channel
//! grant out of a trunking frame is the case that wants it -- has somewhere to say so without
//! the layer's interface changing shape. The layer's own response to it is the part that is
//! its business: it discards the dibits it is holding, because after a tuner moves they are
//! from the old channel and a sync correlated across the discontinuity would pair a sync from
//! one channel with an identifier from another.
enum class P25FrameAction : std::uint8_t { Continue, Retune };

struct P25FrameLayer {
    //! Dibits of the 24 in a sync that may differ and still leave a candidate worth decoding.
    unsigned max_sync_errors{4U};
    //! Bits the identifier's BCH decode may correct.
    //!
    //! The code corrects 11, and the default stops two short of that on purpose. The chance
    //! that a 63-bit field which was never transmitted happens to lie within r bits of one of
    //! the 65536 codewords is the volume of a Hamming ball of radius r over 2^63, and that
    //! volume grows by roughly five times with every step of r -- so the last two radii carry
    //! most of the total risk while recovering the fewest frames. Against outer-symbol noise --
    //! the stream a widened level tracker produces and the most favorable one a false sync can
    //! be drawn from -- only a small number of candidates reach a codeword at radius 11, fewer
    //! at 10, and none at 9 or below. The frames given up are the ones that needed ten or
    //! eleven corrections, well under one per cent even on a marginal signal.
    unsigned max_bch_errors{9U};
    //! Whether the trailing parity bit and a defined identifier are required, not just recorded.
    bool require_parity{true};
    bool require_defined_duid{true};

    std::uint64_t                 dibits{0U};     //!< dibits absorbed
    std::uint64_t                 candidates{0U}; //!< syncs inside tolerance
    std::uint64_t                 frames{0U};     //!< candidates promoted to frames
    std::uint64_t                 rejected_bch{0U};
    std::uint64_t                 rejected_parity{0U};
    std::uint64_t                 rejected_duid{0U};
    std::uint64_t                 retunes{0U};
    std::array<std::uint64_t, 16> duid_census{};
    P25Frame                      last{};

    void reset() noexcept {
        ring.fill(0U);
        head = 0U;
        held = 0U;
        sync.reset();
    }

    //! Candidates the identifier refused, whatever the reason.
    [[nodiscard]] std::uint64_t rejected() const noexcept { return candidates - frames; }

    //! Absorb one dibit. `handler` is called with a `const P25Frame&` for every candidate the
    //! identifier confirms, and returns a `P25FrameAction`.
    template<typename Handler>
    void push(std::uint8_t dibit, Handler&& handler) {
        const std::uint8_t evicted = ring[head];
        ring[head]                 = static_cast<std::uint8_t>(dibit & 0x3U);
        head                       = (head + 1U) % kNidTransmittedDibits;
        ++dibits;

        if (held < kNidTransmittedDibits) {
            ++held;
            return; // nothing has left the delay line yet
        }

        sync.max_dibit_errors = max_sync_errors;
        unsigned syncErrors   = 0U;
        if (!sync.push(evicted, syncErrors)) {
            return;
        }
        ++candidates;

        // The delay line holds the identifier's 33 transmitted dibits, oldest first. Dropping
        // the status symbol among them leaves the 64 bits the decoder wants.
        std::uint64_t nid   = 0U;
        std::size_t   index = head;
        for (std::size_t j = 0U; j < kNidTransmittedDibits; ++j) {
            if (j != kNidStatusOffset) {
                nid = (nid << 2) | static_cast<std::uint64_t>(ring[index]);
            }
            index = (index + 1U) % kNidTransmittedDibits;
        }

        const P25Nid decoded = decodeNid(nid, max_bch_errors);
        if (!decoded.valid) {
            ++rejected_bch;
            return;
        }
        if (!decoded.duid_defined) {
            ++rejected_duid;
            if (require_defined_duid) {
                return;
            }
        }
        if (!decoded.parity_ok) {
            ++rejected_parity;
            if (require_parity) {
                return;
            }
        }

        P25Frame frame;
        frame.dibit_index = dibits - (kFrameSyncDibits + kNidTransmittedDibits);
        frame.nac         = decoded.nac;
        frame.duid        = decoded.duid;
        frame.kind        = duidKind(decoded.duid);
        frame.sync_errors = syncErrors;
        frame.bch_errors  = decoded.bch_errors;
        frame.parity_ok   = decoded.parity_ok;

        ++frames;
        ++duid_census[decoded.duid & 0x0FU];
        last = frame;

        if (handler(static_cast<const P25Frame&>(frame)) == P25FrameAction::Retune) {
            ++retunes;
            reset();
        }
    }

private:
    std::array<std::uint8_t, kNidTransmittedDibits> ring{};
    std::size_t                                     head{0U};
    std::size_t                                     held{0U};
    P25FrameSyncSearch                              sync{};
};

} // namespace gr::p25

#endif // GNURADIO_P25_FRAME_LAYER_HPP
