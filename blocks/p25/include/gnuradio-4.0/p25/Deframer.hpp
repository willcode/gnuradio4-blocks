#ifndef GNURADIO_P25_DEFRAMER_HPP
#define GNURADIO_P25_DEFRAMER_HPP

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/p25/FrameLayer.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>

namespace gr::blocks::p25 {

GR_REGISTER_BLOCK(gr::blocks::p25::P25Deframer)

/*!
@brief P25 Phase 1 deframer: a dibit stream in, whole-frame `DataSet` records out.

The sync search and the promotion law belong to the frame layer and run in place. A frame sync
match is a candidate rather than a confirmed frame, and only a network-identifier decode
promotes it — BCH(63,16,23) within the configured correction limit, the trailing parity bit, a
defined data unit identifier. The layer's own delay line is the correlator, running at every
dibit position so the block reacquires on the first undamaged sync after any dropout. Candidates
are found in place rather than carried as stream tags: a channel in hang time produces a sync
candidate every fifteen milliseconds, and a candidate must not depend on surviving inter-block
transport between the moment it is found and the moment it is promoted or dropped.

Only one frame is pending at a time, and a newly identified frame supersedes it. Every frame
length TIA-102 fixes leaves the next frame's sync exactly at the end of this one, so in ordinary
traffic a pending frame completes before the next is identified. An overlap means the stream
broke, and the newer identification is the better evidence: the pending frame is emitted as
truncated and the new one takes its place. A frame whose length TIA-102 does not fix — the
trunking and packet-data identifiers, whose block counts live in payloads this block does not
read — is emitted at once with no payload rather than guessed at.

Records leave in stream order. A complete frame's record carries the frame's transmitted dibits
from its sync's first dibit (`duidTransmittedDibits` of them) as data; truncated,
length-unknown and no-payload frames carry metadata only. Every record's metadata holds
`protocol` "p25", `sample_start` (the sync's first dibit on the absorbed-dibit clock), `state`
("complete", "no_payload", "length_unknown", "truncated"), the identifier's facts (`nac`,
`duid`, `parity_ok`), what the codes did (`sync_errors` in symbols, `corrected_errors` in BCH
bits), `sequence`, and the layer's census at the moment of emission (`census_dibits`,
`census_candidates`, `census_frames`, `census_rejected`) — values settled when the record is
built. The block's live input position is separately readable through `dibitsAbsorbed()` for
status polling.
*/
struct P25Deframer : Block<P25Deframer> {
    using Description = Doc<"P25 Phase 1 deframer: dibits to whole-frame records; the frame layer's correlator and identifier decode run in place, and only a decode promotes a candidate">;

    PortIn<std::uint8_t>                  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "max_sync_errors", Doc<"dibits of the 24 in a sync that may differ and still leave a candidate worth decoding">> max_sync_errors = 4U;
    //! The code's correction radius is 11 bits; the default stops two short of it, trading
    //! away the frames that needed ten or eleven corrections — well under one per cent even on
    //! a marginal signal — against the disproportionate share of false frames the last two
    //! radii admit.
    Annotated<gr::Size_t, "max_bch_errors", Doc<"bits the identifier's BCH decode may correct; 11 is the code's radius, 9 trades off the highest-risk radii">> max_bch_errors       = 9U;
    Annotated<bool, "require_parity", Doc<"whether the trailing parity bit must agree, not just be recorded">>                                                 require_parity       = true;
    Annotated<bool, "require_defined_duid", Doc<"whether the identifier must be one TIA-102 defines, not just be recorded">>                                   require_defined_duid = true;

    GR_MAKE_REFLECTABLE(P25Deframer, in, out, max_sync_errors, max_bch_errors, require_parity, require_defined_duid);

    //! Transmitted symbols retained: the longest frame plus its header, rounded to a power of
    //! two so the wrap is a mask rather than a division.
    static constexpr std::size_t kRingDibits = 1024UZ;

    gr::p25::P25FrameLayer    _layer{};
    std::vector<std::uint8_t> _ring          = std::vector<std::uint8_t>(kRingDibits, 0U);
    std::uint64_t             _absorbed      = 0ULL;
    alignas(8) std::uint64_t _absorbedShared = 0ULL; //!< published copy of _absorbed, read via dibitsAbsorbed()

    bool              _pending = false;
    gr::p25::P25Frame _pendingFrame{};
    std::uint64_t     _pendingEnd = 0ULL;

    std::uint64_t _sequence  = 0ULL;
    std::uint64_t _truncated = 0ULL;

    //! The block's input progress in dibits, safe to read from any thread. This is the
    //! clock a status reader compares a record's `sample_start` against: both are positions
    //! on the same stream, so frame recency does not depend on the progress of any other
    //! graph branch, and it keeps advancing when a quiet channel produces no records.
    [[nodiscard]] std::uint64_t dibitsAbsorbed() const noexcept { return std::atomic_ref<const std::uint64_t>(_absorbedShared).load(std::memory_order_relaxed); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        _layer.max_sync_errors      = max_sync_errors.value;
        _layer.max_bch_errors       = max_bch_errors.value;
        _layer.require_parity       = require_parity.value;
        _layer.require_defined_duid = require_defined_duid.value;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        // A dibit can resolve at most two records: a superseded pending frame and an
        // immediately reported one. Stop while two slots remain rather than overrun.
        for (; consumed < inSpan.size() && made + 2UZ <= outSpan.size(); ++consumed) {
            _ring[_absorbed % kRingDibits] = static_cast<std::uint8_t>(inSpan[consumed] & 0x3U);
            ++_absorbed;

            _layer.push(inSpan[consumed], [this, &outSpan, &made](const gr::p25::P25Frame& frame) {
                resolveFrame(frame, outSpan, made);
                return gr::p25::P25FrameAction::Continue;
            });

            if (_pending && _absorbed >= _pendingEnd) {
                outSpan[made] = frameRecord(_pendingFrame, "complete", gr::p25::duidTransmittedDibits(_pendingFrame.duid));
                ++made;
                _pending = false;
            }
        }

        std::atomic_ref<std::uint64_t>(_absorbedShared).store(_absorbed, std::memory_order_relaxed);
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() < 2UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    //! Take one confirmed frame from the layer: supersede a pending frame, then either hold
    //! this one until its last dibit lands or report it at once.
    void resolveFrame(const gr::p25::P25Frame& frame, OutputSpanLike auto& outSpan, std::size_t& made) {
        if (_pending) {
            outSpan[made] = frameRecord(_pendingFrame, "truncated", 0UZ);
            ++made;
            ++_truncated;
            _pending = false;
        }

        const std::size_t length   = gr::p25::duidTransmittedDibits(frame.duid);
        const bool        readable = frame.duid == static_cast<std::uint8_t>(gr::p25::P25Duid::Hdu) //
                              || frame.duid == static_cast<std::uint8_t>(gr::p25::P25Duid::Ldu1)    //
                              || frame.duid == static_cast<std::uint8_t>(gr::p25::P25Duid::Ldu2);
        if (readable && length != 0UZ) {
            _pending      = true;
            _pendingFrame = frame;
            _pendingEnd   = frame.dibit_index + length;
        } else {
            outSpan[made] = frameRecord(frame, length == 0UZ ? "length_unknown" : "no_payload", 0UZ);
            ++made;
        }
    }

    //! One frame as a record. `length` nonzero copies that many transmitted dibits out of the
    //! ring, starting at the frame sync's first dibit; zero leaves the record metadata-only.
    [[nodiscard]] DataSet<std::uint8_t> frameRecord(const gr::p25::P25Frame& frame, const char* state, std::size_t length) {
        DataSet<std::uint8_t> record;
        record.signal_values.resize(length);
        for (std::size_t i = 0UZ; i < length; ++i) {
            record.signal_values[i] = _ring[(frame.dibit_index + i) % kRingDibits];
        }
        record.extents.push_back(static_cast<std::int32_t>(length));
        record.signal_names.emplace_back("p25");
        record.timing_events.resize(1UZ);

        record.meta_information.resize(1UZ);
        property_map& map       = record.meta_information[0UZ];
        map["protocol"]         = std::string("p25");
        map["sample_start"]     = frame.dibit_index;
        map["state"]            = std::string(state);
        map["nac"]              = static_cast<gr::Size_t>(frame.nac);
        map["duid"]             = static_cast<gr::Size_t>(frame.duid);
        map["sync_errors"]      = gr::Size_t{frame.sync_errors};
        map["corrected_errors"] = gr::Size_t{frame.bch_errors};
        map["parity_ok"]        = frame.parity_ok;
        map["sequence"]         = _sequence++;
        // The layer's census at the moment of emission, so a record consumer holds the
        // stream-side counters without reaching into the block across threads. Values are
        // settled at emission and therefore reproduce on any schedule.
        map["census_dibits"]     = _layer.dibits;
        map["census_candidates"] = _layer.candidates;
        map["census_frames"]     = _layer.frames;
        map["census_rejected"]   = _layer.rejected();
        return record;
    }
};

} // namespace gr::blocks::p25

#endif // GNURADIO_P25_DEFRAMER_HPP
