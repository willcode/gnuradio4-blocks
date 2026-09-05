#ifndef GNURADIO_CCSDS_SPACE_PACKETS_HPP
#define GNURADIO_CCSDS_SPACE_PACKETS_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <format>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/ccsds/PacketExtractor.hpp>
#include <gnuradio-4.0/algorithm/ccsds/SpacePacket.hpp>
#include <gnuradio-4.0/algorithm/ccsds/TransferFrame.hpp>
#include <gnuradio-4.0/ccsds/RecordHelpers.hpp>

/**
 * @brief The space packet extraction machine as a block, the packet decoder, and both transmit-side blocks.
 *
 * 133.0-B-2's packets are laid end to end through a virtual channel's data fields; `SpacePacketExtract` is
 * `gr::ccsds::PacketExtractor` wearing ports, one instance per virtual channel as 132.0-B-3 4.3.2.1's NOTE
 * requires and this block enforces via a required `virtual_channel` setting. `SpacePacketDecode` reads a whole
 * packet's primary header into metadata; `SpacePacketEncode` and `SpacePacketSegment` are the transmit side.
 */
namespace gr::blocks::ccsds {

namespace packets_detail {

/// @brief Add one set of extraction counters into a running total, so a rebuilt kernel's history survives.
inline void accumulate(gr::ccsds::PacketExtractor::Counters& total, const gr::ccsds::PacketExtractor::Counters& add) noexcept {
    total.packets += add.packets;
    total.idle_packets += add.idle_packets;
    total.idle_frames += add.idle_frames;
    total.frames_lost += add.frames_lost;
    total.duplicate_frames += add.duplicate_frames;
    total.fragments_dropped += add.fragments_dropped;
    total.pointer_mismatch += add.pointer_mismatch;
    total.bad_pointer += add.bad_pointer;
    total.orphan_octets += add.orphan_octets;
    total.oversize_dropped += add.oversize_dropped;
}

} // namespace packets_detail

GR_REGISTER_BLOCK(gr::blocks::ccsds::SpacePacketExtract)

/*!
@brief One virtual channel's packet extraction, `gr::ccsds::PacketExtractor` behind ports, 132.0-B-3 4.3.2.

One zone per input record, zero to many whole packets out. `virtual_channel` has no default: 4.3.2.1's NOTE
requires one instance per virtual channel, and a default would let two channels' packets interleave into one
reassembly state silently. There is no `fail` or `idle` output port — a dropped fragment is octets of unknown
extent and an idle packet is padding, and the counters below say more than either would.
*/
struct SpacePacketExtract : Block<SpacePacketExtract> {
    using Description = Doc<"Space packet extraction for one virtual channel: zones in, whole space packets out, recovering a boundary from the first header pointer after any loss (132.0-B-3 4.3.2)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "virtual_channel", Doc<"the VCID this instance serves; required, one instance per channel">, Visible>    virtual_channel{detail::kUnset};
    Annotated<gr::Size_t, "count_modulus", Doc<"256 for TM, 16777216 or 268435456 for AOS; must be a power of two">>               count_modulus{gr::ccsds::kTmCountModulus};
    Annotated<gr::Size_t, "max_packet_length", Doc<"the reassembly bound; refused above the sixteen-bit field's derived maximum">> max_packet_length{static_cast<gr::Size_t>(gr::ccsds::kMaxPacketOctets)};

    GR_MAKE_REFLECTABLE(SpacePacketExtract, in, out, virtual_channel, count_modulus, max_packet_length);

    std::uint64_t packets           = 0ULL;
    std::uint64_t idle_packets      = 0ULL;
    std::uint64_t idle_frames       = 0ULL;
    std::uint64_t frames_lost       = 0ULL;
    std::uint64_t duplicate_frames  = 0ULL;
    std::uint64_t fragments_dropped = 0ULL;
    std::uint64_t pointer_mismatch  = 0ULL;
    std::uint64_t bad_pointer       = 0ULL;
    std::uint64_t orphan_octets     = 0ULL;
    std::uint64_t oversize_dropped  = 0ULL;
    std::uint64_t nWrongChannel     = 0ULL;
    std::uint64_t nMissingKey       = 0ULL;
    std::uint64_t nSyncFlagSet      = 0ULL;
    std::uint64_t nUndelivered      = 0ULL; //!< whole packets still queued when the stream ended
    std::uint64_t nDiscardedPending = 0ULL; //!< whole packets thrown away by a configuration change

    bool                                 _configured = false;
    gr::ccsds::PacketExtractor           _extractor{};
    gr::ccsds::PacketExtractor::Counters _carried{};
    std::deque<DataSet<std::uint8_t>>    _pending{};
    std::uint64_t                        _pendingGap = 0ULL; // frames lost since the last packet that carried the cause

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (virtual_channel.value == detail::kUnset) {
            throw gr::exception("virtual_channel is required and has no default: one instance must serve exactly one channel");
        }
        if (count_modulus.value == 0U || (count_modulus.value & (count_modulus.value - 1U)) != 0U) {
            throw gr::exception(std::format("count_modulus must be a power of two, got {}", count_modulus.value));
        }
        if (max_packet_length.value > gr::ccsds::kMaxPacketOctets) {
            throw gr::exception(std::format("max_packet_length must not exceed the derived bound of {} octets, got {}", gr::ccsds::kMaxPacketOctets, max_packet_length.value));
        }
        gr::ccsds::PacketExtractor::Config config{};
        config.max_packet_length = max_packet_length.value;
        config.count_modulus     = count_modulus.value;
        // The reconfigured extractor starts with an empty partial and no frame count, which is what a new
        // configuration requires; its counters are a history of the stream and are carried across instead.
        packets_detail::accumulate(_carried, _extractor.counters());
        _extractor = gr::ccsds::PacketExtractor(config);
        nDiscardedPending += _pending.size();
        _pending.clear();
        _pendingGap = 0ULL;
        syncCounters();
        _configured = true;
    }

    void syncCounters() noexcept {
        gr::ccsds::PacketExtractor::Counters total = _carried;
        packets_detail::accumulate(total, _extractor.counters());
        packets           = total.packets;
        idle_packets      = total.idle_packets;
        idle_frames       = total.idle_frames;
        frames_lost       = total.frames_lost;
        duplicate_frames  = total.duplicate_frames;
        fragments_dropped = total.fragments_dropped;
        pointer_mismatch  = total.pointer_mismatch;
        bad_pointer       = total.bad_pointer;
        orphan_octets     = total.orphan_octets;
        oversize_dropped  = total.oversize_dropped;
    }

    void stop() {
        if (!_extractor.fragment().empty()) { // a fragment is not a packet, so the end of the stream drops it
            ++_carried.fragments_dropped;
        }
        nUndelivered += _pending.size();
        syncCounters();
        detail::reportCounters(*this, "SpacePacketExtract", {{"packets", packets}, {"idle packets", idle_packets}, {"idle frames", idle_frames}, {"frames lost", frames_lost}, {"duplicate frames", duplicate_frames}, {"fragments dropped", fragments_dropped}, {"pointer mismatch", pointer_mismatch}, {"bad pointer", bad_pointer}, {"orphan octets", orphan_octets}, {"oversize dropped", oversize_dropped}, {"wrong channel", nWrongChannel}, {"missing key", nMissingKey}, {"sync flag set", nSyncFlagSet}, {"undelivered packets", nUndelivered}, {"pending packets discarded", nDiscardedPending}});
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t made = 0UZ;
        while (!_pending.empty() && made < outSpan.size()) {
            outSpan[made] = std::move(_pending.front());
            _pending.pop_front();
            ++made;
        }

        std::size_t consumed = 0UZ;
        if (_pending.empty()) {
            while (consumed < inSpan.size() && made < outSpan.size()) {
                const DataSet<std::uint8_t>& record = inSpan[consumed];
                ++consumed;
                const property_map* meta = detail::metaOf(record);

                const std::optional<gr::Size_t> vcid = detail::readSize(meta, "ccsds_vcid");
                if (!vcid.has_value()) {
                    ++nMissingKey;
                    continue;
                }
                if (*vcid != virtual_channel.value) {
                    ++nWrongChannel;
                    continue;
                }
                if (const std::optional<bool> sync = detail::readBool(meta, "ccsds_sync_flag"); sync.has_value() && *sync) {
                    ++nSyncFlagSet;
                    continue;
                }
                const std::optional<gr::Size_t> fhp   = detail::readSize(meta, "ccsds_first_header_pointer");
                const std::optional<gr::Size_t> count = detail::readSize(meta, "ccsds_vc_frame_count");
                if (!fhp.has_value() || !count.has_value()) {
                    ++nMissingKey;
                    continue;
                }

                const std::uint64_t                 beforeLost    = _extractor.counters().frames_lost;
                const std::size_t                   pendingBefore = _pending.size();
                const std::span<const std::uint8_t> zone(record.signal_values);
                _extractor.feed(zone, static_cast<std::uint16_t>(*fhp), *count, [&](std::span<const std::uint8_t> packet) {
                    // A gap is a property of the boundary between two zones, so it belongs on one record: the
                    // zone's own cause rides its first packet and is taken off the rest.
                    const bool            firstOfZone = _pending.size() == pendingBefore;
                    DataSet<std::uint8_t> packetRecord;
                    packetRecord.signal_values.assign(packet.begin(), packet.end());
                    detail::startRecord(record, packetRecord, "space_packet");
                    property_map& map = packetRecord.meta_information[0UZ];
                    map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("ccsds/space_packet")));
                    if (!firstOfZone) {
                        detail::removeDiscontinuity(map, "frame_gap");
                        map.erase(property_map::key_type("ccsds_frames_lost"));
                    }
                    _pending.push_back(std::move(packetRecord));
                });

                // The gap is detected before any packet of this zone is emitted, so the count is settled here; a
                // zone that detects one and completes nothing holds it for the next packet to come out.
                _pendingGap += _extractor.counters().frames_lost - beforeLost;
                if (_pendingGap > 0ULL && _pending.size() > pendingBefore) {
                    property_map& map = _pending[pendingBefore].meta_information[0UZ];
                    map.insert_or_assign(property_map::key_type("ccsds_frames_lost"), pmt::Value(static_cast<gr::Size_t>(_pendingGap)));
                    detail::appendDiscontinuity(map, "frame_gap");
                    _pendingGap = 0ULL;
                }

                while (!_pending.empty() && made < outSpan.size()) {
                    outSpan[made] = std::move(_pending.front());
                    _pending.pop_front();
                    ++made;
                }
            }
        }

        syncCounters();
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::SpacePacketDecode)

/*!
@brief One whole space packet per record becomes its packet data field, with the primary header in metadata, 133.0-B-2 4.1.4.

The secondary header, if the flag says one is present, is not split from the user data field: 4.1.4.2.1.4 says its
contents are managed and there is no length field, so there is nothing structural left to parse. A packet whose
declared length disagrees with the record's own length is refused rather than trimmed, because trimming would
publish a payload nobody vouches for.
*/
struct SpacePacketDecode : Block<SpacePacketDecode> {
    using Description = Doc<"Space packet decode: one whole packet per record becomes its packet data field, with the primary header written to metadata (133.0-B-2 4.1.4)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<bool, "strip_primary_header", Doc<"remove the six-octet primary header from the published record">> strip_primary_header = true;

    GR_MAKE_REFLECTABLE(SpacePacketDecode, in, out, strip_primary_header);

    std::uint64_t nPackets        = 0ULL;
    std::uint64_t nPayloadOctets  = 0ULL;
    std::uint64_t nRefusedShort   = 0ULL;
    std::uint64_t nWrongVersion   = 0ULL;
    std::uint64_t nIdlePackets    = 0ULL;
    std::uint64_t nLengthMismatch = 0ULL;

    void stop() { detail::reportCounters(*this, "SpacePacketDecode", {{"packets", nPackets}, {"payload octets", nPayloadOctets}, {"short packets", nRefusedShort}, {"wrong version", nWrongVersion}, {"idle packets", nIdlePackets}, {"length mismatch", nLengthMismatch}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>&        record = inSpan[consumed];
            const std::span<const std::uint8_t> bytes(record.signal_values);
            if (bytes.size() < gr::ccsds::kSpacePacketHeaderSize) {
                ++nRefusedShort;
                continue;
            }
            gr::ccsds::SpacePacketHeader header{};
            const gr::ccsds::ParseStatus status = gr::ccsds::parseSpacePacketHeader(bytes, header);
            if (status == gr::ccsds::ParseStatus::bad_version) {
                ++nWrongVersion;
                continue;
            }
            const std::size_t total = gr::ccsds::totalPacketOctets(header);
            if (total != bytes.size()) {
                ++nLengthMismatch;
                continue;
            }
            if (gr::ccsds::isIdlePacket(header)) {
                ++nIdlePackets;
                continue;
            }

            DataSet<std::uint8_t> payload;
            if (strip_primary_header.value) {
                payload.signal_values.assign(bytes.begin() + static_cast<std::ptrdiff_t>(gr::ccsds::kSpacePacketHeaderSize), bytes.end());
            } else {
                payload.signal_values.assign(bytes.begin(), bytes.end());
            }
            detail::startRecord(record, payload, "space_packet");
            property_map& map = payload.meta_information[0UZ];
            map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("ccsds/space_packet")));
            map.insert_or_assign(property_map::key_type("ccsds_packet_version"), pmt::Value(gr::Size_t{header.version}));
            map.insert_or_assign(property_map::key_type("ccsds_packet_type"), pmt::Value(header.type));
            map.insert_or_assign(property_map::key_type("ccsds_sec_hdr_flag"), pmt::Value(header.secondary_header));
            map.insert_or_assign(property_map::key_type("ccsds_apid"), pmt::Value(gr::Size_t{header.apid}));
            map.insert_or_assign(property_map::key_type("ccsds_sequence_flags"), pmt::Value(gr::Size_t{header.sequence_flags}));
            map.insert_or_assign(property_map::key_type("ccsds_packet_sequence_count"), pmt::Value(gr::Size_t{header.sequence_count}));
            map.insert_or_assign(property_map::key_type("ccsds_packet_data_length"), pmt::Value(gr::Size_t{header.data_length}));

            ++nPackets;
            nPayloadOctets += payload.signal_values.size();
            outSpan[made] = std::move(payload);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::SpacePacketEncode)

/*!
@brief One record of user data per record in, one whole space packet out, 133.0-B-2 4.1.

The sequence count is the one counter this module's encoders own outright rather than leaving to a setting:
4.1.3.4.3.3 makes it the sequential count of every packet a user application emits, one counter per APID,
continuous modulo 16384 — a plain counter with a one-sentence increment rule, unlike AX.25's window-driven `N(S)`.
*/
struct SpacePacketEncode : Block<SpacePacketEncode> {
    using Description = Doc<"Space packet encode: one record of user data becomes a whole space packet, with a per-APID sequence count (133.0-B-2 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "apid", Doc<"0 to 2046; required, and 2047 is refused because it is reserved for the idle packets that fill a zone">, Visible> apid{detail::kUnset};
    Annotated<bool, "packet_type", Doc<"false telemetry, true telecommand, 4.1.3.3.2.3">>                                                                packet_type           = false;
    Annotated<bool, "secondary_header_flag", Doc<"4.1.3.3.3.2">>                                                                                         secondary_header_flag = false;
    Annotated<gr::Size_t, "sequence_flags", Doc<"defaults to 3, unsegmented ('11'), the only value a non-segmenting block can honestly write">>          sequence_flags{3U};

    GR_MAKE_REFLECTABLE(SpacePacketEncode, in, out, apid, packet_type, secondary_header_flag, sequence_flags);

    std::uint64_t nPackets         = 0ULL;
    std::uint64_t nPayloadOctets   = 0ULL;
    std::uint64_t nRefusedEmpty    = 0ULL;
    std::uint64_t nRefusedOversize = 0ULL;
    std::uint64_t nRefusedOverride = 0ULL;
    std::uint64_t nRefusedHeader   = 0ULL; //!< a header the kernel would not build or write

    bool                                            _configured = false;
    std::array<std::uint16_t, gr::ccsds::kIdleApid> _sequenceCounters{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    // 4.1.3.4.3.3's count is the sequential count of the packets one application has produced, so it spans a
    // reconfiguration of how they are labeled and restarts only when the block itself does.
    void start() {
        rebuild();
        _sequenceCounters.fill(0U);
    }
    void reset() { _sequenceCounters.fill(0U); }

    void rebuild() {
        _configured = false;
        if (apid.value == detail::kUnset || apid.value >= gr::ccsds::kIdleApid) {
            throw gr::exception(std::format("apid is required and must be 0 to {}; {} is reserved for idle packets", gr::ccsds::kIdleApid - 1U, gr::ccsds::kIdleApid));
        }
        if (sequence_flags.value > 3U) {
            throw gr::exception(std::format("sequence_flags must be 0 to 3, got {}", sequence_flags.value));
        }
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "SpacePacketEncode", {{"packets", nPackets}, {"payload octets", nPayloadOctets}, {"empty payloads refused", nRefusedEmpty}, {"oversize payloads refused", nRefusedOversize}, {"overrides refused", nRefusedOverride}, {"headers refused", nRefusedHeader}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const property_map*          meta   = detail::metaOf(record);

            gr::Size_t resolvedApid     = apid.value;
            bool       resolvedType     = packet_type.value;
            bool       resolvedSecHdr   = secondary_header_flag.value;
            gr::Size_t resolvedSeqFlags = sequence_flags.value;
            bool       refused          = false;

            if (const std::optional<gr::Size_t> v = detail::readSize(meta, "ccsds_apid"); v.has_value()) {
                if (*v >= gr::ccsds::kIdleApid) {
                    refused = true;
                } else {
                    resolvedApid = *v;
                }
            }
            if (const std::optional<bool> v = detail::readBool(meta, "ccsds_packet_type"); v.has_value()) {
                resolvedType = *v;
            }
            if (const std::optional<bool> v = detail::readBool(meta, "ccsds_sec_hdr_flag"); v.has_value()) {
                resolvedSecHdr = *v;
            }
            if (const std::optional<gr::Size_t> v = detail::readSize(meta, "ccsds_sequence_flags"); v.has_value()) {
                if (*v > 3U) {
                    refused = true;
                } else {
                    resolvedSeqFlags = *v;
                }
            }
            if (refused) {
                ++nRefusedOverride;
                continue;
            }

            const std::size_t payloadOctets = record.signal_values.size();
            if (payloadOctets == 0UZ) {
                ++nRefusedEmpty;
                continue;
            }
            if (payloadOctets > gr::ccsds::kMaxPacketDataOctets) {
                ++nRefusedOversize;
                continue;
            }

            const std::uint16_t seqCount = _sequenceCounters[resolvedApid];

            gr::ccsds::SpacePacketHeader header{};
            gr::ccsds::WriteStatus       status = gr::ccsds::headerForPayload(static_cast<std::uint16_t>(resolvedApid), resolvedType, resolvedSecHdr, static_cast<std::uint8_t>(resolvedSeqFlags), seqCount, payloadOctets, header);

            DataSet<std::uint8_t> packet;
            packet.signal_values.resize(gr::ccsds::kSpacePacketHeaderSize + payloadOctets);
            if (status == gr::ccsds::WriteStatus::ok) {
                status = gr::ccsds::writeSpacePacketHeader(header, std::span<std::uint8_t>(packet.signal_values));
            }
            if (status != gr::ccsds::WriteStatus::ok) { // publishing the all-zero header would claim a packet nobody built
                ++nRefusedHeader;
                continue;
            }
            // the count advances per packet emitted, so a refused record leaves the next one's number unchanged
            _sequenceCounters[resolvedApid] = static_cast<std::uint16_t>((seqCount + 1U) % 16384U);
            std::ranges::copy(record.signal_values, packet.signal_values.begin() + static_cast<std::ptrdiff_t>(gr::ccsds::kSpacePacketHeaderSize));
            detail::startRecord(record, packet, "space_packet");

            ++nPackets;
            nPayloadOctets += payloadOctets;
            outSpan[made] = std::move(packet);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::SpacePacketSegment)

/*!
@brief Space packets in, fixed-length zones out with their first header pointers — the transmit half of 132.0-B-3 4.3.2.

Packets accumulate; whenever `zone_length` octets are available one zone is emitted with the pointer set to the
offset of the first packet that starts in it, or `kFhpNoPacketStart` when the whole zone continues one already
begun. 132.0-B-3 4.1.4.6's fill trigger is release time, a scheduling property this tree's data-driven graphs do
not have, so a padded or idle zone is emitted only where `flush` asks for one, and never on a timer of the
block's own. Such a zone pads whatever is buffered out to `zone_length`; with nothing buffered it is a whole zone
of fill — under `fill = "oid"` the PN sequence under the reserved pointer of 4.1.4.6's Only Idle Data frame, and
under `fill = "idle_packet"` one idle packet filling the zone from its first octet, whose pointer is therefore 0.
The two are different objects and the pointer is what tells them apart: the receiver discards an Only Idle Data
zone whole, and parses the idle packet and discards it by its APID.

`flush` asks twice, and the two are different questions. Raising it while the stream runs is an edge: the buffer
goes out at the next call and once only, so a setting left at `true` does not turn every call into a padded zone.
And while it is set, the end of the stream takes whatever is still buffered — the framework's end-of-stream hook
runs over a span the block deliberately left unconsumed, which is why a call keeps the last record of its input
while `flush` is set and asks for two records at a time so that keeping one cannot stall the steady state.

The octets survive a settings change. A `zone_length` or `fill` change is a property of the link the zones are
going onto, not of the packets already handed over, and the accumulated starts are offsets into the buffer that
any zone length in the validated range addresses — so the change takes effect at the next zone boundary and the
packets in hand are still sent.

`zone_length` is bounded at 2046 octets because the pointer is eleven bits with two reserved values and so names
positions 0 to 2045: a longer zone has octets no pointer can point at, and a packet starting in one of them would
be announced by a reserved value that means the opposite.
*/
struct SpacePacketSegment : Block<SpacePacketSegment> {
    using Description = Doc<"Space packet segmentation: whole packets accumulate into fixed-length zones with a computed first header pointer, padded with idle fill on an explicit flush and at end of stream (132.0-B-3 4.3.2, transmit side)">;

    // `in` is synchronous because the framework offers its end-of-stream hook a span only where a synchronous
    // input still holds items, and that hook is what sends the last zone.
    PortIn<DataSet<std::uint8_t>>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "zone_length", Doc<"octets per emitted data field or packet zone; required, 1 to 2046">, Visible>                                                                                                                    zone_length{0U};
    Annotated<gr::Size_t, "idle_apid", Doc<"the APID for a generated idle packet, 0 to 2047, 4.1.3.3.4.4">>                                                                                                                                    idle_apid{static_cast<gr::Size_t>(gr::ccsds::kIdleApid)};
    Annotated<std::string, "fill", Doc<"what a flush with nothing buffered emits: 'oid' (the 4.1.4.6.2 PN sequence) or 'idle_packet' (one whole idle packet); a short leftover is always padded with the PN sequence">>                        fill{std::string("oid")};
    Annotated<bool, "flush", Doc<"raising this emits the buffer once as a padded or idle zone, and while it is set the end of the stream emits what is left; the stand-in for the release-time trigger a data-driven graph has no clock for">> flush = false;

    GR_MAKE_REFLECTABLE(SpacePacketSegment, in, out, zone_length, idle_apid, fill, flush);

    std::uint64_t nPacketsIn     = 0ULL;
    std::uint64_t nOctetsIn      = 0ULL;
    std::uint64_t nZonesEmitted  = 0ULL;
    std::uint64_t nFlushZones    = 0ULL;
    std::uint64_t nRefusedHeader = 0ULL; //!< an idle packet the kernel would not build or write, filled with the PN sequence instead

    bool                      _configured = false;
    std::vector<std::uint8_t> _buffer{};
    std::deque<std::size_t>   _starts{}; // offsets into _buffer where an accumulated packet begins
    gr::ccsds::OidFill        _oidFill{};
    bool                      _flushWas   = false; //!< `flush` as the last rebuild saw it, so a rebuild can find its edge
    bool                      _flushArmed = false; //!< a raised `flush` whose zone has not gone out yet

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }

    void start() {
        rebuild();
        _buffer.clear();
        _starts.clear();
        _oidFill.reset();
        // A run that begins with `flush` already set owes its zone to the end of its stream, not to its first
        // call: there is nothing buffered yet, and an idle zone in front of the first packet announces idle time
        // the link never had.
        _flushArmed = false;
    }

    void rebuild() {
        _configured = false;
        if (zone_length.value == 0U) {
            throw gr::exception("zone_length is required and has no default");
        }
        if (zone_length.value > gr::ccsds::kFhpOnlyIdleData) {
            // eleven pointer bits less the two reserved values name positions 0 to 2045, so 2046 octets is the
            // longest zone whose every position a pointer can hold
            throw gr::exception(std::format("zone_length must not exceed {} octets, got {}: the first header pointer cannot name a position beyond {}", gr::ccsds::kFhpOnlyIdleData, zone_length.value, gr::ccsds::kFhpOnlyIdleData - 1U));
        }
        if (idle_apid.value > gr::ccsds::kIdleApid) {
            throw gr::exception(std::format("idle_apid must be 0 to {}, got {}", gr::ccsds::kIdleApid, idle_apid.value));
        }
        if (fill.value != "oid" && fill.value != "idle_packet") {
            throw gr::exception(std::format("fill must be 'oid' or 'idle_packet', got '{}'", fill.value));
        }
        // A raised `flush` arms one zone; an unrelated change while it is still armed leaves it armed, and
        // clearing `flush` before that zone went out withdraws the request.
        _flushArmed = flush.value && (_flushArmed || !_flushWas);
        _flushWas   = flush.value;
        // Two at a time, so that keeping the last record of a call back for the end-of-stream hook cannot stall a
        // stream the framework would otherwise hand over one record per call.
        in.min_samples = flush.value ? 2UZ : 1UZ;
        // The buffer and its starts are octets a caller has already handed over, and they cross the rebuild: the
        // starts are offsets into the buffer, and every zone length this accepts can address them.
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "SpacePacketSegment", {{"packets in", nPacketsIn}, {"octets in", nOctetsIn}, {"zones emitted", nZonesEmitted}, {"flush zones", nFlushZones}, {"idle headers refused", nRefusedHeader}}); }

    [[nodiscard]] gr::Size_t pointerFor() const noexcept {
        if (!_starts.empty() && _starts.front() < zone_length.value) {
            return static_cast<gr::Size_t>(_starts.front());
        }
        return gr::ccsds::kFhpNoPacketStart;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        // While `flush` is set the last record of a call stays where it is, so that the end-of-stream hook has a
        // span to run on and the zone it holds is sent. A call carrying a single record is one a caller drove by
        // hand rather than one the framework composed, and takes it.
        const std::size_t offer    = flush.value && inSpan.size() >= 2UZ ? inSpan.size() - 1UZ : inSpan.size();
        std::size_t       consumed = 0UZ;
        std::size_t       made     = 0UZ;
        accumulate(inSpan, offer, outSpan, consumed, made);

        if (_flushArmed && made < outSpan.size()) {
            emitFlushZone(outSpan, made);
            _flushArmed = false;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief End of stream: take the records held back, then send what is buffered as one padded zone.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        accumulate(inSpan, inSpan.size(), outSpan, consumed, made);

        // An empty buffer here has already gone out whole; a zone of nothing but fill behind it would announce
        // idle time on a link that has ended.
        if (flush.value && !_buffer.empty() && made < outSpan.size()) {
            emitFlushZone(outSpan, made);
            _flushArmed = false;
        }

        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    /// @brief Take the first @p offer records of @p inSpan into the buffer, emitting a zone whenever one fills.
    void accumulate(InputSpanLike auto& inSpan, std::size_t offer, OutputSpanLike auto& outSpan, std::size_t& consumed, std::size_t& made) {
        while (consumed < offer && made < outSpan.size()) {
            if (_buffer.size() >= zone_length.value) {
                emitZone(outSpan, made);
                continue;
            }
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            ++consumed;
            ++nPacketsIn;
            nOctetsIn += record.signal_values.size();
            _starts.push_back(_buffer.size());
            _buffer.insert(_buffer.end(), record.signal_values.begin(), record.signal_values.end());
        }
        while (_buffer.size() >= zone_length.value && made < outSpan.size()) {
            emitZone(outSpan, made);
        }
    }

    void emitZone(OutputSpanLike auto& outSpan, std::size_t& made) {
        const gr::Size_t pointer = pointerFor();

        DataSet<std::uint8_t> zone;
        zone.signal_values.assign(_buffer.begin(), _buffer.begin() + static_cast<std::ptrdiff_t>(zone_length.value));
        detail::freshRecord(zone, "ccsds");
        zone.meta_information[0UZ].insert_or_assign(property_map::key_type("ccsds_first_header_pointer"), pmt::Value(pointer));

        _buffer.erase(_buffer.begin(), _buffer.begin() + static_cast<std::ptrdiff_t>(zone_length.value));
        while (!_starts.empty() && _starts.front() < zone_length.value) {
            _starts.pop_front();
        }
        for (std::size_t& s : _starts) {
            s -= zone_length.value;
        }

        ++nZonesEmitted;
        outSpan[made] = std::move(zone);
        ++made;
    }

    /// @brief The buffer padded out to `zone_length`, or a whole zone of fill when nothing is buffered.
    void emitFlushZone(OutputSpanLike auto& outSpan, std::size_t& made) {
        const bool        wasEmpty  = _buffer.empty();
        const std::size_t padNeeded = zone_length.value - _buffer.size();

        bool filledWithIdlePacket = false;
        if (wasEmpty && fill.value == "idle_packet" && zone_length.value >= gr::ccsds::kSpacePacketHeaderSize) {
            gr::ccsds::SpacePacketHeader header{};
            std::vector<std::uint8_t>    idlePacket(zone_length.value, std::uint8_t{0U});
            gr::ccsds::WriteStatus       status = gr::ccsds::headerForPayload(static_cast<std::uint16_t>(idle_apid.value), false, false, 3U, 0U, zone_length.value - gr::ccsds::kSpacePacketHeaderSize, header);
            if (status == gr::ccsds::WriteStatus::ok) {
                status = gr::ccsds::writeSpacePacketHeader(header, std::span<std::uint8_t>(idlePacket));
            }
            if (status == gr::ccsds::WriteStatus::ok) {
                _buffer.insert(_buffer.end(), idlePacket.begin(), idlePacket.end());
                filledWithIdlePacket = true;
            } else { // a zone too short to hold a packet takes the sequence instead of an unwritten header
                ++nRefusedHeader;
            }
        }
        if (!filledWithIdlePacket) {
            std::vector<std::uint8_t> pad(padNeeded);
            _oidFill.next(std::span<std::uint8_t>(pad));
            _buffer.insert(_buffer.end(), pad.begin(), pad.end());
        }

        // A zone of PN fill holds no packet at all, which is what 4.1.2.7.6.5's reserved value announces. A zone
        // filled with an idle packet holds one, starting at its first octet: the pointer says 0 and the receiver
        // parses the packet and discards it by its APID (4.1.3.3.4.4), which is the discard the standard names.
        // Anything buffered is a packet or the tail of one, and `pointerFor` says which.
        const gr::Size_t      pointer = filledWithIdlePacket ? gr::Size_t{0U} : (wasEmpty ? gr::ccsds::kFhpOnlyIdleData : pointerFor());
        DataSet<std::uint8_t> zone;
        zone.signal_values.assign(_buffer.begin(), _buffer.begin() + static_cast<std::ptrdiff_t>(zone_length.value));
        detail::freshRecord(zone, "ccsds");
        zone.meta_information[0UZ].insert_or_assign(property_map::key_type("ccsds_first_header_pointer"), pmt::Value(pointer));

        _buffer.clear();
        _starts.clear();

        ++nZonesEmitted;
        ++nFlushZones;
        outSpan[made] = std::move(zone);
        ++made;
    }
};

} // namespace gr::blocks::ccsds

#endif // GNURADIO_CCSDS_SPACE_PACKETS_HPP
