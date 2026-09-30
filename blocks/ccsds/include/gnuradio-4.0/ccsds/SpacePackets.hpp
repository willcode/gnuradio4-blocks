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
 * @brief The space packet extraction block, the packet decoder, and both transmit-side blocks.
 *
 * 133.0-B-2 packets lie end to end through the data fields of a virtual channel. `SpacePacketExtract` runs one
 * `gr::ccsds::PacketExtractor` per virtual channel (132.0-B-3 4.3.2.1 NOTE). Its required `virtual_channel` setting
 * enforces one instance per channel. `SpacePacketDecode` reads the primary header of a whole packet into metadata.
 * `SpacePacketEncode` and `SpacePacketSegment` form the transmit side.
 */
namespace gr::blocks::ccsds {

namespace packets_detail {

/// @brief Adds one set of extraction counters into a running total. A rebuilt kernel's history then survives.
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
@brief Extracts the packets of one virtual channel with `gr::ccsds::PacketExtractor`, 132.0-B-3 4.3.2.

The block takes one zone per input record and emits zero or more whole packets. `virtual_channel` has no default.
4.3.2.1's NOTE requires one instance per virtual channel. A default would let the packets of two channels interleave
silently in one reassembly state. There is no `fail` or `idle` output port. A dropped fragment is octets of unknown
extent, and an idle packet is padding. The counters report more than either port would.
*/
struct SpacePacketExtract : Block<SpacePacketExtract> {
    using Description = Doc<"Extracts whole space packets from the zones of one virtual channel and recovers a boundary from the first header pointer after any loss (132.0-B-3 4.3.2)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "virtual_channel", Doc<"required VCID of this instance, one instance per channel">, Visible>   virtual_channel{detail::kUnset};
    Annotated<gr::Size_t, "count_modulus", Doc<"power of two, 256 for TM, 16777216 or 268435456 for AOS">>               count_modulus{gr::ccsds::kTmCountModulus};
    Annotated<gr::Size_t, "max_packet_length", Doc<"reassembly bound, at most the sixteen-bit field's derived maximum">> max_packet_length{static_cast<gr::Size_t>(gr::ccsds::kMaxPacketOctets)};

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
        // The reconfigured extractor starts with an empty partial and no frame count, as a new configuration
        // requires. Its counters are a history of the stream and carry across.
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
                    // A gap is a property of the boundary between two zones and belongs on one record. The zone's
                    // own cause goes on its first packet and is removed from the rest.
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

                // The gap is detected before any packet of this zone is emitted, and the count is settled here. A
                // zone that detects a gap and completes no packet holds it for the next packet out.
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
@brief Decodes one whole space packet per record into its data field and primary header metadata, 133.0-B-2 4.1.4.

The block does not split a secondary header from the user data field. 4.1.4.2.1.4 says its contents are managed
and gives no length field. Nothing structural remains to parse. The block refuses a packet whose declared length
differs from the record's length. Trimming it would publish an unverified payload.
*/
struct SpacePacketDecode : Block<SpacePacketDecode> {
    using Description = Doc<"Decodes one whole space packet per record into its packet data field and writes the primary header to metadata (133.0-B-2 4.1.4)">;

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
@brief Encodes each record of user data as one whole space packet, 133.0-B-2 4.1.

The block owns the sequence count and does not take it from a setting. 4.1.3.4.3.3 makes it the sequential count of
every packet a user application emits. There is one counter per APID, continuous modulo 16384. It is a plain counter
with a one-sentence increment rule, unlike the window-driven `N(S)` of AX.25. The block does not segment user data.
`sequence_flags` therefore defaults to 3, unsegmented ('11').
*/
struct SpacePacketEncode : Block<SpacePacketEncode> {
    using Description = Doc<"Encodes one record of user data as a whole space packet with a per-APID sequence count (133.0-B-2 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "apid", Doc<"required APID, 0 to 2046, 2047 being reserved for idle packets">, Visible> apid{detail::kUnset};
    Annotated<bool, "packet_type", Doc<"false telemetry, true telecommand, 4.1.3.3.2.3">>                         packet_type           = false;
    Annotated<bool, "secondary_header_flag", Doc<"4.1.3.3.3.2">>                                                  secondary_header_flag = false;
    Annotated<gr::Size_t, "sequence_flags", Doc<"sequence flags, default 3 for unsegmented ('11')">>              sequence_flags{3U};

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
    // The count of 4.1.3.4.3.3 is the sequential count of the packets one application has produced. It spans a
    // reconfiguration of their labels and restarts only when the block itself restarts.
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
            if (status != gr::ccsds::WriteStatus::ok) { // an all-zero header would describe a packet that was never built
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
@brief Packs space packets into fixed-length zones with first header pointers, the transmit half of 132.0-B-3 4.3.2.

Packets accumulate in a buffer. Whenever `zone_length` octets are available, the block emits one zone. Its pointer
is the offset of the first packet that starts in it. The pointer is `kFhpNoPacketStart` when the whole zone
continues a packet already begun. The fill trigger of 132.0-B-3 4.1.4.6 is release time, a scheduling property that
a data-driven graph does not have. The block therefore emits a padded or idle zone only when `flush` asks for one.
It never emits one on a timer of its own. Such a zone pads the buffered octets out to `zone_length` with the PN
sequence. With nothing buffered, it is a whole zone of fill. Under `fill = "oid"` the fill is the PN sequence,
under the reserved pointer of the Only Idle Data frame of 4.1.4.6. Under `fill = "idle_packet"` the fill is one idle
packet that starts at the zone's first octet, and the pointer is 0. The two are different objects, and the pointer
tells them apart. The receiver discards an Only Idle Data zone whole. It parses the idle packet and discards it by
its APID.

`flush` acts in two ways. Raising it while the stream runs is an edge. The buffer goes out once, at the next call.
A setting left at `true` does not turn every call into a padded zone. While `flush` is set, the end of the stream
also sends whatever is still buffered. The framework's end-of-stream hook runs over a span the block left
unconsumed. A call therefore keeps the last record of its input while `flush` is set. It asks for two records at a
time, and keeping one back cannot stall the steady state.

The buffered octets survive a settings change. A `zone_length` or `fill` change is a property of the link, not of
the packets already received. The accumulated starts are offsets into the buffer, and every zone length in the
validated range can address them. The change takes effect at the next zone boundary, and the buffered packets are
still sent.

`zone_length` is at most 2046 octets. The pointer has eleven bits with two reserved values and names positions 0 to
2045. A longer zone has octets no pointer can name. A packet starting in one of them would be announced by a
reserved value that means the opposite.
*/
struct SpacePacketSegment : Block<SpacePacketSegment> {
    using Description = Doc<"Packs whole space packets into fixed-length zones with a computed first header pointer. It pads with idle fill on an explicit flush and at end of stream (132.0-B-3 4.3.2, transmit side)">;

    // `in` is synchronous because the framework offers its end-of-stream hook a span only where a synchronous
    // input still holds items, and that hook is what sends the last zone.
    PortIn<DataSet<std::uint8_t>>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "zone_length", Doc<"required octets per emitted data field or packet zone, 1 to 2046">, Visible> zone_length{0U};
    Annotated<gr::Size_t, "idle_apid", Doc<"the APID for a generated idle packet, 0 to 2047, 4.1.3.3.4.4">>                idle_apid{static_cast<gr::Size_t>(gr::ccsds::kIdleApid)};
    Annotated<std::string, "fill", Doc<"empty-buffer flush output, 'oid' (4.1.4.6.2 PN sequence) or 'idle_packet'">>       fill{std::string("oid")};
    Annotated<bool, "flush", Doc<"on raise, emit the buffer once, and at stream end while set">>                           flush = false;

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
        // A run that begins with `flush` already set sends its zone at the end of its stream, not at its first call.
        // Nothing is buffered yet. An idle zone in front of the first packet would announce idle time the link never
        // had.
        _flushArmed = false;
    }

    void rebuild() {
        _configured = false;
        if (zone_length.value == 0U) {
            throw gr::exception("zone_length is required and has no default");
        }
        if (zone_length.value > gr::ccsds::kFhpOnlyIdleData) {
            // eleven pointer bits less the two reserved values name positions 0 to 2045. 2046 octets is the longest
            // zone whose every position a pointer can hold.
            throw gr::exception(std::format("zone_length must not exceed {} octets, got {}: the first header pointer cannot name a position beyond {}", gr::ccsds::kFhpOnlyIdleData, zone_length.value, gr::ccsds::kFhpOnlyIdleData - 1U));
        }
        if (idle_apid.value > gr::ccsds::kIdleApid) {
            throw gr::exception(std::format("idle_apid must be 0 to {}, got {}", gr::ccsds::kIdleApid, idle_apid.value));
        }
        if (fill.value != "oid" && fill.value != "idle_packet") {
            throw gr::exception(std::format("fill must be 'oid' or 'idle_packet', got '{}'", fill.value));
        }
        // A raised `flush` arms one zone. An unrelated change leaves an armed zone armed. Clearing `flush` before
        // that zone goes out withdraws the request.
        _flushArmed = flush.value && (_flushArmed || !_flushWas);
        _flushWas   = flush.value;
        // Two at a time. Keeping the last record of a call back for the end-of-stream hook then cannot stall a stream
        // that the framework would otherwise deliver one record per call.
        in.min_samples = flush.value ? 2UZ : 1UZ;
        // The buffer and its starts hold octets already received, and they carry across the rebuild. The starts are
        // offsets into the buffer, and every zone length this accepts can address them.
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

        // While `flush` is set, the last record of a call stays in place. The end-of-stream hook then has a span to
        // run on, and the zone it holds is sent. A call with a single record comes from a caller driving the block by
        // hand, not from the framework, and the block takes that record.
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

    /// @brief At the end of the stream, takes the records held back and sends what is buffered as one padded zone.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        accumulate(inSpan, inSpan.size(), outSpan, consumed, made);

        // An empty buffer here has already gone out whole. A zone of fill alone after it would announce idle time
        // on a link that has ended.
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

        // A zone of PN fill holds no packet, and the reserved value of 4.1.2.7.6.5 announces that. A zone filled
        // with an idle packet holds one, starting at its first octet. The pointer says 0. The receiver parses the
        // packet and discards it by its APID (4.1.3.3.4.4), the discard the standard names. Anything buffered is a
        // packet or the tail of one, and `pointerFor` says which.
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
