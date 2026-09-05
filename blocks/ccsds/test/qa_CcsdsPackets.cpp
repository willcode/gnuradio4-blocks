#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/ccsds/SpacePackets.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

using Record = gr::DataSet<std::uint8_t>;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

namespace {

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] Record recordOf(std::vector<std::uint8_t> bytes, gr::property_map meta = {}) {
    Record record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.meta_information.push_back(std::move(meta));
    record.timing_events.emplace_back();
    return record;
}

[[nodiscard]] const gr::property_map& metaOf(const Record& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

[[nodiscard]] bool metaHas(const Record& record, std::string_view key) { return metaOf(record).find(gr::property_map::key_type(key)) != metaOf(record).end(); }

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? gr::Size_t{0xFFFFFFFFU} : entry->second.value_or(gr::Size_t{0xFFFFFFFFU});
}

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

/// One-in, one-out drive over a whole input vector at once, room sized generously.
template<typename TBlock>
[[nodiscard]] std::vector<Record> drive1(TBlock& block, std::span<const Record> in, std::size_t room = 64UZ) {
    std::vector<Record> scratch(room);
    InputSpan<Record>   inSpan(in);
    OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
    std::ignore = block.processBulk(inSpan, outSpan);
    scratch.resize(outSpan.count);
    return scratch;
}

/// A whole space packet of exactly @p octets octets, its payload a pattern that depends on both length and index.
[[nodiscard]] std::vector<std::uint8_t> packetOf(std::size_t octets, std::uint16_t apid, std::uint16_t sequence) {
    gr::ccsds::SpacePacketHeader header{};
    boost::ut::expect(gr::ccsds::headerForPayload(apid, false, false, 3U, sequence, octets - gr::ccsds::kSpacePacketHeaderSize, header) == gr::ccsds::WriteStatus::ok);
    std::vector<std::uint8_t> packet(octets, 0U);
    std::ignore = gr::ccsds::writeSpacePacketHeader(header, packet);
    for (std::size_t i = gr::ccsds::kSpacePacketHeaderSize; i < octets; ++i) {
        packet[i] = static_cast<std::uint8_t>((i * 31UZ + octets + sequence) & 0xFFUZ);
    }
    return packet;
}

/// The ten extraction counters plus the three admission counters, so a scene can assert all thirteen at once.
struct Tally {
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
    std::uint64_t wrong_channel     = 0ULL;
    std::uint64_t missing_key       = 0ULL;
    std::uint64_t sync_flag_set     = 0ULL;

    [[nodiscard]] bool operator==(const Tally&) const noexcept = default;
};

[[nodiscard]] Tally tallyOf(const gr::blocks::ccsds::SpacePacketExtract& block) {
    return Tally{block.packets, block.idle_packets, block.idle_frames, block.frames_lost, block.duplicate_frames, block.fragments_dropped, //
        block.pointer_mismatch, block.bad_pointer, block.orphan_octets, block.oversize_dropped, block.nWrongChannel, block.nMissingKey, block.nSyncFlagSet};
}

/// One hand-built zone as extractor input: the pointer, frame count and channel it would have arrived with.
[[nodiscard]] Record zoneRecord(std::vector<std::uint8_t> octets, gr::Size_t fhp, gr::Size_t count = 0U, gr::Size_t vcid = 0U) { return recordOf(std::move(octets), {{"ccsds_first_header_pointer", fhp}, {"ccsds_vc_frame_count", count}, {"ccsds_vcid", vcid}}); }

/// Turn segmented zones into extractor input: the pointer the segmenter computed, a continuous frame count, one channel.
[[nodiscard]] std::vector<Record> zonesForExtraction(std::span<const Record> zones, gr::Size_t vcid = 0U) {
    std::vector<Record> out;
    out.reserve(zones.size());
    for (std::size_t i = 0UZ; i < zones.size(); ++i) {
        const gr::property_map meta{{"ccsds_first_header_pointer", metaSize(zones[i], "ccsds_first_header_pointer")}, {"ccsds_vc_frame_count", static_cast<gr::Size_t>(i)}, {"ccsds_vcid", vcid}};
        out.push_back(recordOf(zones[i].signal_values, meta));
    }
    return out;
}

/// Segment @p lengths' worth of packets at @p zoneLength and extract them again, with one trailing packet sized so the
/// last zone comes out full: nothing is left in the segmenter, so the round trip is asserted over every packet sent.
struct RoundTrip {
    std::vector<std::vector<std::uint8_t>> sent{};
    std::vector<Record>                    zones{};
    std::vector<Record>                    got{};
    Tally                                  tally{};
};

[[nodiscard]] RoundTrip roundTrip(gr::Size_t zoneLength, std::span<const std::size_t> lengths) {
    using namespace gr::blocks::ccsds;
    RoundTrip   result;
    std::size_t total = 0UZ;
    for (std::size_t i = 0UZ; i < lengths.size(); ++i) {
        result.sent.push_back(packetOf(lengths[i], static_cast<std::uint16_t>(100U + i), static_cast<std::uint16_t>(i)));
        total += lengths[i];
    }
    std::size_t tail = (zoneLength - (total % zoneLength)) % zoneLength;
    while (tail < gr::ccsds::kSpacePacketHeaderSize + 1UZ) { // a packet is seven octets at the shortest
        tail += zoneLength;
    }
    result.sent.push_back(packetOf(tail, 200U, 0U));

    std::vector<Record> in;
    for (const std::vector<std::uint8_t>& packet : result.sent) {
        in.push_back(recordOf(packet));
    }

    auto segment = make<SpacePacketSegment>({{"zone_length", zoneLength}});
    result.zones = drive1(segment, in, 256UZ);

    auto extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
    result.got   = drive1(extract, zonesForExtraction(result.zones), 256UZ);
    result.tally = tallyOf(extract);
    return result;
}

/// The 4.1.4.6.2 fill sequence from its start, whose first ten octets 132.0-B-3 publishes.
constexpr std::array<std::uint8_t, 10> kOidFirstTen{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x6DU, 0xB6U, 0xD8U, 0x61U, 0x45U, 0x1FU};

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Record> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const Record& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// Source, segmenter, sink under the scheduler; the zones the sink collected come back.
[[nodiscard]] std::vector<Record> segmentUnderScheduler(std::span<const std::vector<std::uint8_t>> packets, gr::Size_t zoneLength, bool flush) {
    gr::Graph flow;
    auto&     source  = flow.emplaceBlock<RecordSource>();
    auto&     segment = flow.emplaceBlock<gr::blocks::ccsds::SpacePacketSegment>(gr::property_map{{"zone_length", zoneLength}, {"flush", flush}});
    auto&     sink    = flow.emplaceBlock<RecordSink>();
    for (const std::vector<std::uint8_t>& packet : packets) {
        source._records.push_back(recordOf(packet));
    }
    boost::ut::expect(flow.connect<"out", "in">(source, segment).has_value());
    boost::ut::expect(flow.connect<"out", "in">(segment, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!done.load()) {
        scheduler.requestStop();
        boost::ut::expect(false) << "the graph did not finish within thirty seconds";
    }
    runner.join();
    return sink._records;
}

/// Every zone's octets laid end to end, which is what the receiver's virtual channel carries.
[[nodiscard]] std::vector<std::uint8_t> concatenated(std::span<const Record> zones) {
    std::vector<std::uint8_t> all;
    for (const Record& zone : zones) {
        all.insert(all.end(), zone.signal_values.begin(), zone.signal_values.end());
    }
    return all;
}

} // namespace

const boost::ut::suite<"CcsdsPackets"> ccsdsPacketTests = [] {
    using namespace boost::ut;
    using namespace gr::ccsds;
    using namespace gr::blocks::ccsds;

    "criterion 5: segment then extract is the identity"_test = [] {
        for (const gr::Size_t zoneLength : {gr::Size_t{223}, gr::Size_t{1115}, gr::Size_t{2046}}) {
            const std::vector<std::size_t> lengths{7UZ, 8UZ, 100UZ, 1000UZ, 2047UZ, 4096UZ};
            const RoundTrip                trip = roundTrip(zoneLength, std::span<const std::size_t>(lengths));

            expect(eq(trip.got.size(), trip.sent.size())) << "every packet segmented at " << zoneLength << " comes back out";
            for (const Record& zone : trip.zones) {
                expect(eq(zone.signal_values.size(), std::size_t{zoneLength}));
            }
            for (std::size_t i = 0UZ; i < trip.sent.size() && i < trip.got.size(); ++i) {
                expect(that % (trip.got[i].signal_values == trip.sent[i])) << "packet " << i << " round trips byte for byte at zone length " << zoneLength;
            }
            Tally expected{};
            expected.packets = trip.sent.size();
            expect(that % (trip.tally == expected)) << "packets counted, every other counter still zero, at zone length " << zoneLength;
        }
    };

    "criterion 5: a packet boundary on a zone boundary and one octet either side"_test = [] {
        for (const gr::Size_t zoneLength : {gr::Size_t{223}, gr::Size_t{1115}, gr::Size_t{2046}}) {
            for (const std::ptrdiff_t offset : {std::ptrdiff_t{-1}, std::ptrdiff_t{0}, std::ptrdiff_t{1}}) {
                const std::size_t              first = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(zoneLength) + offset);
                const std::vector<std::size_t> lengths{first, 40UZ, 7UZ};
                const RoundTrip                trip = roundTrip(zoneLength, std::span<const std::size_t>(lengths));

                expect(eq(trip.got.size(), trip.sent.size())) << "zone length " << zoneLength << ", first packet " << first;
                for (std::size_t i = 0UZ; i < trip.sent.size() && i < trip.got.size(); ++i) {
                    expect(that % (trip.got[i].signal_values == trip.sent[i])) << "packet " << i << " at zone length " << zoneLength << ", first packet " << first;
                }
                Tally expected{};
                expected.packets = trip.sent.size();
                expect(that % (trip.tally == expected)) << "zone length " << zoneLength << ", first packet " << first;
            }
        }
    };

    "criterion 7: the negative control, an extractor that ignores the pointer"_test = [] {
        // one packet spanning three 50-octet zones, then two more behind it
        const std::vector<std::uint8_t> firstPacket  = packetOf(136UZ, 11U, 0U);
        const std::vector<std::uint8_t> secondPacket = packetOf(26UZ, 12U, 0U);
        const std::vector<std::uint8_t> thirdPacket  = packetOf(60UZ, 13U, 0U);

        std::vector<std::uint8_t> stream = firstPacket;
        stream.insert(stream.end(), secondPacket.begin(), secondPacket.end());
        stream.insert(stream.end(), thirdPacket.begin(), thirdPacket.end()); // 222 octets in all
        const std::vector<std::uint8_t> zone0(stream.begin(), stream.begin() + 50);
        const std::vector<std::uint8_t> zone2(stream.begin() + 100, stream.begin() + 150);
        const std::vector<std::uint8_t> zone3(stream.begin() + 150, stream.begin() + 200);
        const std::vector<std::uint8_t> zone4(stream.begin() + 200, stream.end());
        // zone1 [50,100) is lost with its frame count. firstPacket ends at 136, so zone2's first packet start is at
        // 136 - 100 = 36; secondPacket ends at 162, so zone3's is at 162 - 150 = 12; thirdPacket ends the stream, so
        // zone4 starts inside it and carries the reserved "no packet starts here".
        constexpr std::uint16_t kZone2Fhp = 36U;
        constexpr std::uint16_t kZone3Fhp = 12U;

        std::vector<std::vector<std::uint8_t>> recovered;
        {
            PacketExtractor extractor;
            const auto      emit = [&recovered](std::span<const std::uint8_t> p) { recovered.emplace_back(p.begin(), p.end()); };
            extractor.feed(zone0, 0U, 0U, emit);
            extractor.feed(zone2, kZone2Fhp, 2U, emit);
            extractor.feed(zone3, kZone3Fhp, 3U, emit);
            extractor.feed(zone4, kFhpNoPacketStart, 4U, emit);
            expect(eq(extractor.counters().frames_lost, std::uint64_t{1}));
            expect(eq(extractor.counters().fragments_dropped, std::uint64_t{1}));
            expect(eq(recovered.size(), 2UZ)) << "the packet that spanned the loss is gone; both behind it are recovered";
            if (recovered.size() == 2UZ) {
                expect(that % (recovered[0] == secondPacket));
                expect(that % (recovered[1] == thirdPacket));
            }
        }

        // The surveyed behavior: lay the surviving zones end to end and split them by the declared lengths alone. The
        // pointer is what the arms differ by, so this arm gets the same octets and none of the pointers.
        std::vector<std::vector<std::uint8_t>> naive;
        {
            std::vector<std::uint8_t> buffer;
            for (const std::vector<std::uint8_t>* zone : {&zone0, &zone2, &zone3, &zone4}) {
                buffer.insert(buffer.end(), zone->begin(), zone->end());
            }
            std::size_t at = 0UZ;
            while (at + kSpacePacketHeaderSize <= buffer.size()) {
                SpacePacketHeader header{};
                std::ignore             = parseSpacePacketHeader(std::span<const std::uint8_t>(buffer).subspan(at, kSpacePacketHeaderSize), header);
                const std::size_t total = totalPacketOctets(header);
                if (at + total > buffer.size()) {
                    break;
                }
                naive.emplace_back(buffer.begin() + static_cast<std::ptrdiff_t>(at), buffer.begin() + static_cast<std::ptrdiff_t>(at + total));
                at += total;
            }
        }
        expect(gt(naive.size(), 0UZ)) << "the surveyed extractor does produce packets - that is what makes it dangerous";
        if (!naive.empty()) {
            expect(eq(naive[0].size(), firstPacket.size())) << "and one of the right length";
            expect(that % (naive[0] != firstPacket)) << "built from the wrong octets, since the frame between them was lost";
        }
        const bool naiveFoundSecond = std::ranges::any_of(naive, [&secondPacket](const std::vector<std::uint8_t>& p) { return p == secondPacket; });
        expect(!naiveFoundSecond) << "and the second packet is never recovered, which is exactly what the pointer buys";
    };

    "criterion 3: the minus-one's second half, through the blocks"_test = [] {
        // Three packets built with the surveyed convention, data_length = payload_octets: 106 octets on the wire, each
        // claiming 107. Header octets for APID 100, unsegmented, count 0: 00 64 C0 00 00 64.
        const auto wrongPacket = [] {
            const std::array<std::uint8_t, 6UZ> header{0x00U, 0x64U, 0xC0U, 0x00U, 0x00U, 0x64U};
            std::vector<std::uint8_t>           packet(106UZ, 0xAAU);
            std::ranges::copy(header, packet.begin());
            return packet;
        };
        const auto rightPacket = [](std::uint16_t sequence) { return packetOf(106UZ, 100U, sequence); };

        { // §4.5: the record is one octet short of what its own header claims, so it is refused and not trimmed
            auto                      decode = make<SpacePacketDecode>({});
            const std::vector<Record> in{recordOf(wrongPacket()), recordOf(wrongPacket()), recordOf(wrongPacket())};
            const std::vector<Record> out = drive1(decode, in);
            expect(eq(out.size(), 0UZ));
            expect(eq(decode.nLengthMismatch, std::uint64_t{3}));
            expect(eq(decode.nPackets, std::uint64_t{0}));
        }
        { // the same three laid end to end in one zone: the first "packet" eats the second's first octet and the walk
            // never finds a boundary again, so one wrong record comes out where three right ones should have
            std::vector<std::uint8_t> zone;
            for (int i = 0; i < 3; ++i) {
                const std::vector<std::uint8_t> packet = wrongPacket();
                zone.insert(zone.end(), packet.begin(), packet.end());
            }
            auto                      extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
            const std::vector<Record> zones{zoneRecord(zone, 0U)};
            const std::vector<Record> out = drive1(extract, zones);
            expect(eq(out.size(), 1UZ)) << "one record out of a zone that holds three packets' worth of octets";
            if (out.size() == 1UZ) {
                expect(eq(out[0].signal_values.size(), std::size_t{107})) << "107 octets: the whole first packet and one octet of the second";
                expect(that % (out[0].signal_values != wrongPacket()));
            }
            expect(eq(extract.packets, std::uint64_t{1}));
            expect(!extract._extractor.fragment().empty()) << "the rest of the zone is a fragment against a length read out of a payload";
        }
        { // the same scene with the convention right: three packets in, three packets out, byte for byte
            std::vector<std::uint8_t> zone;
            for (std::uint16_t i = 0U; i < 3U; ++i) {
                const std::vector<std::uint8_t> packet = rightPacket(i);
                zone.insert(zone.end(), packet.begin(), packet.end());
            }
            auto                      extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
            const std::vector<Record> zones{zoneRecord(zone, 0U)};
            const std::vector<Record> out = drive1(extract, zones);
            expect(eq(out.size(), 3UZ));
            for (std::uint16_t i = 0U; i < 3U && i < out.size(); ++i) {
                expect(that % (out[i].signal_values == rightPacket(i)));
            }
            expect(eq(extract.packets, std::uint64_t{3}));
        }
    };

    "SpacePacketSegment: the flush pads a leftover and says so"_test = [] {
        auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"flush", true}});
        const std::vector<Record> in{recordOf(packetOf(30UZ, 5U, 0U))};
        const std::vector<Record> zones = drive1(segment, in, 8UZ);

        expect(eq(zones.size(), 2UZ)) << "one full zone and the flushed leftover";
        if (zones.size() != 2UZ) {
            return;
        }
        expect(eq(metaSize(zones[0], "ccsds_first_header_pointer"), gr::Size_t{0}));
        expect(eq(metaSize(zones[1], "ccsds_first_header_pointer"), gr::Size_t{kFhpNoPacketStart})) << "the leftover continues a packet begun in the zone before it";
        expect(eq(zones[1].signal_values.size(), std::size_t{20}));

        const std::vector<std::uint8_t> packet = packetOf(30UZ, 5U, 0U);
        expect(that % (std::vector<std::uint8_t>(zones[1].signal_values.begin(), zones[1].signal_values.begin() + 10) == std::vector<std::uint8_t>(packet.begin() + 20, packet.end())));
        // the pad is the 4.1.4.6.2 sequence from its start, whose first ten octets 132.0-B-3 publishes
        expect(that % (std::vector<std::uint8_t>(zones[1].signal_values.begin() + 10, zones[1].signal_values.end()) == std::vector<std::uint8_t>{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x6DU, 0xB6U, 0xD8U, 0x61U, 0x45U, 0x1FU}));

        // and the label is the one that gets the packet back: 2046 would have the receiver discard the zone as fill
        auto                      extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
        const std::vector<Record> out     = drive1(extract, zonesForExtraction(zones));
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(that % (out[0].signal_values == packet));
        }
        expect(eq(extract.idle_frames, std::uint64_t{0}));
    };

    "SpacePacketSegment: a flush with nothing buffered is an idle zone"_test = [] {
        {
            auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"flush", true}});
            const std::vector<Record> zones   = drive1(segment, std::span<const Record>{}, 4UZ);
            expect(eq(zones.size(), 1UZ));
            if (zones.size() == 1UZ) {
                expect(eq(metaSize(zones[0], "ccsds_first_header_pointer"), gr::Size_t{kFhpOnlyIdleData}));
                expect(eq(zones[0].signal_values.size(), std::size_t{20}));
                expect(that % (std::vector<std::uint8_t>(zones[0].signal_values.begin(), zones[0].signal_values.begin() + 10) == std::vector<std::uint8_t>{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x6DU, 0xB6U, 0xD8U, 0x61U, 0x45U, 0x1FU}));
            }
            expect(eq(segment.nFlushZones, std::uint64_t{1}));
        }
        {
            auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"flush", true}, {"fill", std::string("idle_packet")}, {"idle_apid", gr::Size_t{1234}}});
            const std::vector<Record> zones   = drive1(segment, std::span<const Record>{}, 4UZ);
            expect(eq(zones.size(), 1UZ));
            if (zones.size() == 1UZ) {
                expect(eq(metaSize(zones[0], "ccsds_first_header_pointer"), gr::Size_t{0})) << "an idle packet is a packet, and it starts at the zone's first octet";
                SpacePacketHeader header{};
                expect(parseSpacePacketHeader(zones[0].signal_values, header) == ParseStatus::ok);
                expect(eq(header.apid, std::uint16_t{1234})) << "idle_apid is the APID written";
                expect(!header.secondary_header) << "4.1.3.3.3.4";
                expect(eq(totalPacketOctets(header), std::size_t{20})) << "the packet fills the zone exactly";
            }
            expect(eq(segment.nRefusedHeader, std::uint64_t{0}));
        }
        { // and at the reserved APID the receiver parses it, counts it idle and publishes nothing
            auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"flush", true}, {"fill", std::string("idle_packet")}});
            const std::vector<Record> zones   = drive1(segment, std::span<const Record>{}, 4UZ);
            expect(eq(zones.size(), 1UZ));
            expect(eq(metaSize(zones[0], "ccsds_first_header_pointer"), gr::Size_t{0}));

            auto                      extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
            const std::vector<Record> out     = drive1(extract, zonesForExtraction(zones));
            expect(eq(out.size(), 0UZ)) << "nothing reaches the consumer";
            expect(eq(extract.idle_packets, std::uint64_t{1})) << "the packet is recognized and discarded by APID";
            expect(eq(extract.idle_frames, std::uint64_t{0})) << "the zone is not an only-idle-data frame: it carries a packet";
            expect(eq(extract.packets, std::uint64_t{0}));
        }
        { // a zone with no room for a packet takes the sequence instead, and says the header was refused
            auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{6}}, {"flush", true}, {"fill", std::string("idle_packet")}});
            const std::vector<Record> zones   = drive1(segment, std::span<const Record>{}, 4UZ);
            expect(eq(zones.size(), 1UZ));
            expect(eq(segment.nRefusedHeader, std::uint64_t{1}));
            if (zones.size() == 1UZ) {
                expect(that % (zones[0].signal_values == std::vector<std::uint8_t>{0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x6DU, 0xB6U}));
            }
        }
    };

    "SpacePacketSegment: configuration refusals"_test = [] {
        // one unrelated setting is staged in each case, because a block staged with nothing at all reaches its
        // configuration check at start() rather than here
        expect(throws([] { std::ignore = make<SpacePacketSegment>({{"fill", std::string("idle_packet")}}); })) << "zone_length is required";
        expect(throws([] { std::ignore = make<SpacePacketSegment>({{"zone_length", gr::Size_t{2047}}}); })) << "a zone the pointer cannot address is refused";
        expect(nothrow([] { std::ignore = make<SpacePacketSegment>({{"zone_length", gr::Size_t{2046}}}); })) << "2046 is the longest addressable zone";
        expect(throws([] { std::ignore = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"idle_apid", gr::Size_t{2048}}}); })) << "an APID above eleven bits is refused";
        expect(nothrow([] { std::ignore = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"idle_apid", gr::Size_t{2047}}}); }));
        expect(throws([] { std::ignore = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}, {"fill", std::string("zeros")}}); })) << "an unrecognized fill is refused";
    };

    "SpacePacketSegment: a settings change keeps the octets already handed over"_test = [] {
        const std::vector<std::uint8_t> first  = packetOf(30UZ, 5U, 0U);
        const std::vector<std::uint8_t> second = packetOf(30UZ, 6U, 1U);

        { // a setting that touches no geometry leaves the held tail exactly where it sits
            auto segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}});
            expect(eq(drive1(segment, std::vector<Record>{recordOf(first)}, 8UZ).size(), 1UZ));
            expect(eq(segment._buffer.size(), 10UZ)) << "the tail of the packet waits for the next zone";

            std::ignore = segment.settings().setStaged({{"idle_apid", gr::Size_t{100}}});
            std::ignore = segment.settings().applyStagedParameters();
            expect(eq(segment._buffer.size(), 10UZ)) << "the rebuild keeps them";

            const std::vector<Record> zones = drive1(segment, std::vector<Record>{recordOf(second)}, 8UZ);
            expect(eq(zones.size(), 2UZ));
            if (zones.size() == 2UZ) {
                std::vector<std::uint8_t> expected(first.begin() + 20, first.end());
                expected.insert(expected.end(), second.begin(), second.begin() + 10);
                expect(that % (zones[0].signal_values == expected)) << "the held tail leads the zone the next packet completes";
                expect(eq(metaSize(zones[0], "ccsds_first_header_pointer"), gr::Size_t{10})) << "and the pointer names where that packet starts";
                expect(that % (zones[1].signal_values == std::vector<std::uint8_t>(second.begin() + 10, second.end())));
                expect(eq(metaSize(zones[1], "ccsds_first_header_pointer"), gr::Size_t{kFhpNoPacketStart}));
            }
        }

        { // a zone length that cuts the held octets into several zones addresses every start it holds
            auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{100}}});
            const std::vector<Record> in{recordOf(packetOf(25UZ, 7U, 0U)), recordOf(packetOf(30UZ, 8U, 1U))};
            expect(eq(drive1(segment, in, 8UZ).size(), 0UZ)) << "fifty-five octets do not fill a hundred";
            expect(eq(segment._buffer.size(), 55UZ));

            std::ignore = segment.settings().setStaged({{"zone_length", gr::Size_t{10}}});
            std::ignore = segment.settings().applyStagedParameters();
            expect(eq(segment._buffer.size(), 55UZ)) << "shortening the zone does not throw the packets away";

            const std::vector<Record> zones = drive1(segment, std::span<const Record>{}, 8UZ);
            // the second packet begins at octet 25, which is octet 5 of the third zone of ten; the zones before
            // and after it carry no start and say so with the reserved value
            expect(eq(zones.size(), 5UZ)) << "fifty-five octets are five whole zones of ten and a five-octet remainder";
            const std::array<gr::Size_t, 5> expectedPointers{gr::Size_t{0}, gr::Size_t{kFhpNoPacketStart}, gr::Size_t{5}, gr::Size_t{kFhpNoPacketStart}, gr::Size_t{kFhpNoPacketStart}};
            for (std::size_t i = 0UZ; i < zones.size() && i < expectedPointers.size(); ++i) {
                expect(eq(zones[i].signal_values.size(), 10UZ));
                expect(eq(metaSize(zones[i], "ccsds_first_header_pointer"), expectedPointers[i])) << "zone " << i;
            }
            std::vector<std::uint8_t>       sent = packetOf(25UZ, 7U, 0U);
            const std::vector<std::uint8_t> tail = packetOf(30UZ, 8U, 1U);
            sent.insert(sent.end(), tail.begin(), tail.end());
            std::vector<std::uint8_t> got = concatenated(zones);
            got.insert(got.end(), segment._buffer.begin(), segment._buffer.end());
            expect(that % (got == sent)) << "and not one octet moved";
        }
    };

    "SpacePacketSegment: a raised flush is an edge, not a level"_test = [] {
        { // a leftover goes out once, and a call that follows adds nothing
            auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}});
            const std::vector<Record> in{recordOf(packetOf(30UZ, 5U, 0U))};
            expect(eq(drive1(segment, in, 8UZ).size(), 1UZ));
            expect(eq(segment.nFlushZones, std::uint64_t{0})) << "nothing asked for a flush yet";

            std::ignore = segment.settings().setStaged({{"flush", true}});
            std::ignore = segment.settings().applyStagedParameters();

            const std::vector<Record> flushed = drive1(segment, std::span<const Record>{}, 8UZ);
            expect(eq(flushed.size(), 1UZ)) << "the raised flush emits what is buffered";
            if (flushed.size() == 1UZ) {
                expect(eq(flushed[0].signal_values.size(), 20UZ));
                expect(eq(metaSize(flushed[0], "ccsds_first_header_pointer"), gr::Size_t{kFhpNoPacketStart})) << "a packet tail is a continuation, not idle data";
                expect(that % (std::vector<std::uint8_t>(flushed[0].signal_values.begin() + 10, flushed[0].signal_values.end()) == std::vector<std::uint8_t>(kOidFirstTen.begin(), kOidFirstTen.end())));
            }
            expect(eq(drive1(segment, std::span<const Record>{}, 8UZ).size(), 0UZ)) << "and the setting left standing does not emit again";
            expect(eq(drive1(segment, std::span<const Record>{}, 8UZ).size(), 0UZ));
            expect(eq(segment.nFlushZones, std::uint64_t{1}));
        }

        { // with nothing buffered the raised flush is one idle zone, and one only
            auto segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}});
            expect(eq(drive1(segment, std::span<const Record>{}, 8UZ).size(), 0UZ));

            std::ignore = segment.settings().setStaged({{"flush", true}});
            std::ignore = segment.settings().applyStagedParameters();

            const std::vector<Record> idle = drive1(segment, std::span<const Record>{}, 8UZ);
            expect(eq(idle.size(), 1UZ));
            if (idle.size() == 1UZ) {
                expect(eq(metaSize(idle[0], "ccsds_first_header_pointer"), gr::Size_t{kFhpOnlyIdleData}));
                expect(that % (std::vector<std::uint8_t>(idle[0].signal_values.begin(), idle[0].signal_values.begin() + 10) == std::vector<std::uint8_t>(kOidFirstTen.begin(), kOidFirstTen.end())));
            }
            expect(eq(drive1(segment, std::span<const Record>{}, 8UZ).size(), 0UZ)) << "one idle zone per raised flush, not one per call";
            expect(eq(segment.nFlushZones, std::uint64_t{1}));

            // lowering and raising it again asks a second time
            std::ignore = segment.settings().setStaged({{"flush", false}});
            std::ignore = segment.settings().applyStagedParameters();
            std::ignore = segment.settings().setStaged({{"flush", true}});
            std::ignore = segment.settings().applyStagedParameters();
            expect(eq(drive1(segment, std::span<const Record>{}, 8UZ).size(), 1UZ));
            expect(eq(segment.nFlushZones, std::uint64_t{2}));
        }
    };

    "SpacePacketSegment: the end of the stream sends the last zone"_test = [] {
        // five packets of thirty octets are 150, which is three whole zones of forty and a thirty-octet
        // remainder: the run that flushes emits four zones, the run that does not emits three
        std::vector<std::vector<std::uint8_t>> packets;
        std::vector<std::uint8_t>              sent;
        for (std::uint16_t i = 0U; i < 5U; ++i) {
            packets.push_back(packetOf(30UZ, static_cast<std::uint16_t>(40U + i), i));
            sent.insert(sent.end(), packets.back().begin(), packets.back().end());
        }

        const std::vector<Record> flushed = segmentUnderScheduler(packets, gr::Size_t{40}, true);
        const std::vector<Record> held    = segmentUnderScheduler(packets, gr::Size_t{40}, false);
        expect(eq(flushed.size(), 4UZ)) << "the buffered remainder goes out padded";
        expect(eq(held.size(), 3UZ)) << "and without the flush it does not go out at all";
        expect(neq(flushed.size(), held.size())) << "the two arms really differ";

        if (flushed.size() == 4UZ) {
            for (const Record& zone : flushed) {
                expect(eq(zone.signal_values.size(), 40UZ));
            }
            // the starts sit at 0, 30, 60, 90 and 120, so the zone boundaries at 40, 80 and 120 put the first
            // start of each zone at 0, 20, 10 and 0
            const std::array<gr::Size_t, 4> expectedPointers{gr::Size_t{0}, gr::Size_t{20}, gr::Size_t{10}, gr::Size_t{0}};
            for (std::size_t i = 0UZ; i < 4UZ; ++i) {
                expect(eq(metaSize(flushed[i], "ccsds_first_header_pointer"), expectedPointers[i])) << "zone " << i;
            }
            const std::vector<std::uint8_t> all = concatenated(flushed);
            expect(eq(all.size(), 160UZ));
            expect(that % (std::vector<std::uint8_t>(all.begin(), all.begin() + 150) == sent)) << "every octet sent is in the zones, in order";
            expect(that % (std::vector<std::uint8_t>(all.begin() + 150, all.end()) == std::vector<std::uint8_t>(kOidFirstTen.begin(), kOidFirstTen.end()))) << "and the ten octets behind them are the fill";
        }
        if (held.size() == 3UZ) {
            expect(that % (concatenated(held) == std::vector<std::uint8_t>(sent.begin(), sent.begin() + 120)));
        }
    };

    "SpacePacketDecode: every refusal counted, and the next record decodes"_test = [] {
        auto decode = make<SpacePacketDecode>({});

        std::vector<std::uint8_t> shortRecord(5UZ, 0U);
        std::vector<std::uint8_t> wrongVersion = packetOf(7UZ, 100U, 0U);
        wrongVersion[0]                        = static_cast<std::uint8_t>(wrongVersion[0] | 0x20U); // version '001', 4.1.3.2.2 says '000'
        std::vector<std::uint8_t> idlePacket   = packetOf(10UZ, kIdleApid, 0U);
        std::vector<std::uint8_t> tooShort     = packetOf(20UZ, 100U, 0U);
        tooShort.pop_back(); // 19 octets against a header that claims 20
        const std::vector<std::uint8_t> good = packetOf(20UZ, 300U, 7U);

        const std::vector<Record> in{recordOf(shortRecord), recordOf(wrongVersion), recordOf(idlePacket), recordOf(tooShort), recordOf(good)};
        const std::vector<Record> out = drive1(decode, in);

        expect(eq(decode.nRefusedShort, std::uint64_t{1}));
        expect(eq(decode.nWrongVersion, std::uint64_t{1}));
        expect(eq(decode.nIdlePackets, std::uint64_t{1}));
        expect(eq(decode.nLengthMismatch, std::uint64_t{1}));
        expect(eq(decode.nPackets, std::uint64_t{1}));
        expect(eq(out.size(), 1UZ)) << "the record behind four refusals decodes normally";
        if (out.size() == 1UZ) {
            expect(that % (out[0].signal_values == std::vector<std::uint8_t>(good.begin() + 6, good.end()))) << "the packet data field, primary header stripped";
            expect(eq(metaSize(out[0], "ccsds_apid"), gr::Size_t{300}));
            expect(eq(metaSize(out[0], "ccsds_packet_sequence_count"), gr::Size_t{7}));
            expect(eq(metaSize(out[0], "ccsds_packet_data_length"), gr::Size_t{13})) << "the field as transmitted: 14 octets of data field less one";
            expect(eq(metaSize(out[0], "ccsds_packet_version"), gr::Size_t{0}));
            expect(eq(metaSize(out[0], "ccsds_sequence_flags"), gr::Size_t{3}));
            expect(eq(metaString(out[0], "protocol"), std::string("ccsds/space_packet")));
            expect(eq(decode.nPayloadOctets, std::uint64_t{14}));
        }

        auto                      whole = make<SpacePacketDecode>({{"strip_primary_header", false}});
        const std::vector<Record> wholeIn{recordOf(good)};
        const std::vector<Record> kept = drive1(whole, wholeIn);
        expect(eq(kept.size(), 1UZ));
        if (kept.size() == 1UZ) {
            expect(that % (kept[0].signal_values == good)) << "the packet passes through whole and only the metadata is written";
        }
    };

    "SpacePacketEncode: one count per APID, modulo 16384"_test = [] {
        auto encode = make<SpacePacketEncode>({{"apid", gr::Size_t{10}}});

        const auto payload  = [](std::uint8_t value) { return recordOf(std::vector<std::uint8_t>(4UZ, value)); };
        const auto withApid = [&payload](std::uint8_t value, gr::Size_t apid) {
            Record record = payload(value);
            record.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("ccsds_apid"), gr::pmt::Value(apid));
            return record;
        };

        const std::vector<Record> in{payload(1U), withApid(2U, gr::Size_t{20}), payload(3U), withApid(4U, gr::Size_t{20}), withApid(5U, gr::Size_t{30})};
        const std::vector<Record> out = drive1(encode, in);
        expect(eq(out.size(), 5UZ));
        const std::vector<std::pair<std::uint16_t, std::uint16_t>> expected{{10U, 0U}, {20U, 0U}, {10U, 1U}, {20U, 1U}, {30U, 0U}};
        for (std::size_t i = 0UZ; i < out.size() && i < expected.size(); ++i) {
            SpacePacketHeader header{};
            expect(parseSpacePacketHeader(out[i].signal_values, header) == ParseStatus::ok);
            expect(eq(header.apid, expected[i].first)) << "record " << i;
            expect(eq(header.sequence_count, expected[i].second)) << "each APID counts its own packets, record " << i;
            expect(eq(totalPacketOctets(header), std::size_t{10}));
        }

        encode._sequenceCounters[10] = 16383U; // 4.1.3.4.3.4's modulus, reached without emitting 16383 packets
        const std::vector<Record> wrapIn{payload(6U), payload(7U)};
        const std::vector<Record> wrapped = drive1(encode, wrapIn);
        expect(eq(wrapped.size(), 2UZ));
        if (wrapped.size() == 2UZ) {
            SpacePacketHeader last{};
            SpacePacketHeader next{};
            expect(parseSpacePacketHeader(wrapped[0].signal_values, last) == ParseStatus::ok);
            expect(parseSpacePacketHeader(wrapped[1].signal_values, next) == ParseStatus::ok);
            expect(eq(last.sequence_count, std::uint16_t{16383}));
            expect(eq(next.sequence_count, std::uint16_t{0})) << "continuous modulo 16384";
        }

        encode.reset();
        expect(eq(encode._sequenceCounters[10], std::uint16_t{0}));
        expect(eq(encode._sequenceCounters[20], std::uint16_t{0}));
    };

    "SpacePacketEncode: a settings change relabels, it does not renumber"_test = [] {
        auto                      encode = make<SpacePacketEncode>({{"apid", gr::Size_t{10}}});
        const std::vector<Record> in{recordOf(std::vector<std::uint8_t>(4UZ, 1U)), recordOf(std::vector<std::uint8_t>(4UZ, 2U))};
        expect(eq(drive1(encode, in).size(), 2UZ));

        std::ignore = encode.settings().setStaged({{"packet_type", true}});
        std::ignore = encode.settings().applyStagedParameters();

        const std::vector<Record> afterIn{recordOf(std::vector<std::uint8_t>(4UZ, 3U))};
        const std::vector<Record> after = drive1(encode, afterIn);
        expect(eq(after.size(), 1UZ));
        if (after.size() == 1UZ) {
            SpacePacketHeader header{};
            expect(parseSpacePacketHeader(after[0].signal_values, header) == ParseStatus::ok);
            expect(eq(header.sequence_count, std::uint16_t{2})) << "the count is the application's, and it did not restart";
            expect(header.type) << "the new label took effect";
        }
    };

    "SpacePacketEncode: three refusals, none of them numbered"_test = [] {
        auto encode = make<SpacePacketEncode>({{"apid", gr::Size_t{10}}});

        Record badApid = recordOf(std::vector<std::uint8_t>(4UZ, 1U));
        badApid.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("ccsds_apid"), gr::pmt::Value(gr::Size_t{kIdleApid}));
        Record badFlags = recordOf(std::vector<std::uint8_t>(4UZ, 2U));
        badFlags.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("ccsds_sequence_flags"), gr::pmt::Value(gr::Size_t{4}));

        const std::vector<Record> in{recordOf(std::vector<std::uint8_t>{}), recordOf(std::vector<std::uint8_t>(kMaxPacketDataOctets + 1UZ, 3U)), badApid, badFlags, recordOf(std::vector<std::uint8_t>(4UZ, 4U))};
        const std::vector<Record> out = drive1(encode, in);

        expect(eq(encode.nRefusedEmpty, std::uint64_t{1})) << "4.1.4.1.2: there is no encoding for an empty data field";
        expect(eq(encode.nRefusedOversize, std::uint64_t{1}));
        expect(eq(encode.nRefusedOverride, std::uint64_t{2})) << "the idle APID and a sequence flag above three";
        expect(eq(encode.nRefusedHeader, std::uint64_t{0}));
        expect(eq(out.size(), 1UZ)) << "the record behind four refusals is encoded";
        if (out.size() == 1UZ) {
            SpacePacketHeader header{};
            expect(parseSpacePacketHeader(out[0].signal_values, header) == ParseStatus::ok);
            expect(eq(header.sequence_count, std::uint16_t{0})) << "a refused record spends no sequence number";
        }
    };

    "SpacePacketExtract: the three admission counters and the three refusals"_test = [] {
        expect(throws([] { std::ignore = make<SpacePacketExtract>({{"count_modulus", gr::Size_t{16777216}}}); })) << "virtual_channel is required";
        expect(throws([] { std::ignore = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}, {"count_modulus", gr::Size_t{100}}}); })) << "a modulus that is not a power of two is refused";
        expect(throws([] { std::ignore = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}, {"max_packet_length", gr::Size_t{65543}}}); })) << "above the derived bound is refused";
        expect(nothrow([] { std::ignore = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}, {"max_packet_length", gr::Size_t{65542}}}); }));

        auto                            extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{3}}});
        const std::vector<std::uint8_t> packet  = packetOf(20UZ, 42U, 0U);

        const auto          zoneWith = [&packet](gr::property_map meta) { return recordOf(packet, std::move(meta)); };
        std::vector<Record> in;
        in.push_back(zoneWith({{"ccsds_vcid", gr::Size_t{4}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{0}}}));
        in.push_back(zoneWith({{"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{0}}}));
        in.push_back(zoneWith({{"ccsds_vcid", gr::Size_t{3}}, {"ccsds_vc_frame_count", gr::Size_t{0}}}));
        in.push_back(zoneWith({{"ccsds_vcid", gr::Size_t{3}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{0}}, {"ccsds_sync_flag", true}}));
        in.push_back(zoneWith({{"ccsds_vcid", gr::Size_t{3}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{0}}}));

        const std::vector<Record> out = drive1(extract, in);
        expect(eq(extract.nWrongChannel, std::uint64_t{1}));
        expect(eq(extract.nMissingKey, std::uint64_t{2})) << "a missing vcid and a missing pointer are both refusals";
        expect(eq(extract.nSyncFlagSet, std::uint64_t{1})) << "a VCA_SDU carries no packets and its pointer has no value";
        expect(eq(out.size(), 1UZ)) << "the record behind four refusals is extracted";
        if (out.size() == 1UZ) {
            expect(that % (out[0].signal_values == packet));
        }
    };

    "SpacePacketExtract: what is held at the end of the stream is counted"_test = [] {
        auto extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});

        std::vector<std::uint8_t> zone;
        for (std::uint16_t i = 0U; i < 5U; ++i) {
            const std::vector<std::uint8_t> packet = packetOf(20UZ, 50U, i);
            zone.insert(zone.end(), packet.begin(), packet.end());
        }
        const std::vector<std::uint8_t> spilling = packetOf(40UZ, 50U, 5U);
        zone.insert(zone.end(), spilling.begin(), spilling.begin() + 10); // a sixth packet's head, whose rest never arrives

        const std::vector<Record> in{zoneRecord(zone, 0U)};
        const std::vector<Record> out = drive1(extract, in, 2UZ); // room for two of the five packets
        expect(eq(out.size(), 2UZ));
        expect(eq(extract.packets, std::uint64_t{5})) << "the kernel completed five; three are queued behind the output port";
        expect(eq(extract._pending.size(), 3UZ));
        expect(eq(extract.fragments_dropped, std::uint64_t{0}));

        extract.stop();
        expect(eq(extract.fragments_dropped, std::uint64_t{1})) << "the held fragment is dropped and counted, never published";
        expect(eq(extract.nUndelivered, std::uint64_t{3})) << "and the packets nobody took are counted too";
    };

    "SpacePacketExtract: a settings change keeps the history and says what it threw away"_test = [] {
        auto extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});

        std::vector<std::uint8_t> zone;
        for (std::uint16_t i = 0U; i < 4U; ++i) {
            const std::vector<std::uint8_t> packet = packetOf(20UZ, 60U, i);
            zone.insert(zone.end(), packet.begin(), packet.end());
        }
        const std::vector<Record> in{zoneRecord(zone, 0U)};
        expect(eq(drive1(extract, in, 1UZ).size(), 1UZ));
        expect(eq(extract.packets, std::uint64_t{4}));
        expect(eq(extract._pending.size(), 3UZ));

        std::ignore = extract.settings().setStaged({{"max_packet_length", gr::Size_t{2048}}});
        std::ignore = extract.settings().applyStagedParameters();

        expect(eq(extract.packets, std::uint64_t{4})) << "the counters are a history of the stream and survive the rebuild";
        expect(eq(extract.nDiscardedPending, std::uint64_t{3})) << "the packets the rebuild threw away are counted";
        expect(eq(extract._pending.size(), 0UZ));
        expect(eq(extract._extractor.config().max_packet_length, std::size_t{2048})) << "and the new configuration is in force";
    };

    "criterion 20: a gap is one cause on one record"_test = [] {
        auto extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});

        const auto zoneOf = [](std::uint16_t first, gr::property_map meta) {
            std::vector<std::uint8_t> octets;
            for (std::uint16_t i = 0U; i < 2U; ++i) {
                const std::vector<std::uint8_t> packet = packetOf(20UZ, 70U, static_cast<std::uint16_t>(first + i));
                octets.insert(octets.end(), packet.begin(), packet.end());
            }
            return recordOf(octets, std::move(meta));
        };

        std::vector<Record> in;
        in.push_back(zoneOf(0U, {{"ccsds_vcid", gr::Size_t{0}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{0}}, {"unrelated_key", std::string("kept")}}));
        // the frame count skips one, and the decode upstream already wrote its own view of the same gap
        in.push_back(zoneOf(2U, {{"ccsds_vcid", gr::Size_t{0}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{2}}, {"unrelated_key", std::string("kept")}, {"discontinuity", std::string("rate_change,frame_gap")}, {"ccsds_frames_lost", gr::Size_t{1}}}));

        const std::vector<Record> out = drive1(extract, in);
        expect(eq(out.size(), 4UZ));
        expect(eq(extract.frames_lost, std::uint64_t{1}));
        if (out.size() != 4UZ) {
            return;
        }
        expect(eq(metaString(out[0], "discontinuity"), std::string{}));
        expect(eq(metaString(out[1], "discontinuity"), std::string{}));
        expect(eq(metaString(out[2], "discontinuity"), std::string("rate_change,frame_gap"))) << "appended once, not twice";
        expect(eq(metaSize(out[2], "ccsds_frames_lost"), gr::Size_t{1}));
        expect(eq(metaString(out[3], "discontinuity"), std::string("rate_change"))) << "the cause belongs to the first record after the gap and to no other";
        expect(!metaHas(out[3], "ccsds_frames_lost"));
        for (const Record& record : out) {
            expect(eq(metaString(record, "unrelated_key"), std::string("kept")));
        }
    };

    "criterion 20: a gap the zone does not resolve rides the next packet"_test = [] {
        auto extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});

        // A packet spanning three zones, with the frame between the first two lost: the zone that detects the gap is a
        // pure continuation and completes nothing, so there is no record in it to carry the cause.
        const std::vector<std::uint8_t> spanning = packetOf(60UZ, 80U, 0U);
        const std::vector<std::uint8_t> behind   = packetOf(20UZ, 80U, 1U);

        std::vector<Record> in;
        in.push_back(recordOf(std::vector<std::uint8_t>(spanning.begin(), spanning.begin() + 40), {{"ccsds_vcid", gr::Size_t{0}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{0}}}));
        in.push_back(recordOf(std::vector<std::uint8_t>(spanning.begin() + 40, spanning.end()), {{"ccsds_vcid", gr::Size_t{0}}, {"ccsds_first_header_pointer", gr::Size_t{gr::ccsds::kFhpNoPacketStart}}, {"ccsds_vc_frame_count", gr::Size_t{2}}}));
        in.push_back(recordOf(behind, {{"ccsds_vcid", gr::Size_t{0}}, {"ccsds_first_header_pointer", gr::Size_t{0}}, {"ccsds_vc_frame_count", gr::Size_t{3}}}));

        const std::vector<Record> out = drive1(extract, in);
        expect(eq(extract.frames_lost, std::uint64_t{1}));
        expect(eq(extract.fragments_dropped, std::uint64_t{1})) << "the spanning packet is lost with the frame between its halves";
        expect(eq(extract.orphan_octets, std::uint64_t{20})) << "and its tail has nothing left to complete";
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(that % (out[0].signal_values == behind));
            expect(eq(metaString(out[0], "discontinuity"), std::string("frame_gap"))) << "the gap waits for a packet to carry it";
            expect(eq(metaSize(out[0], "ccsds_frames_lost"), gr::Size_t{1}));
        }
    };

    "criterion 21: the packet blocks pass every other key through"_test = [] {
        const gr::property_map meta{{"unrelated_key", std::string("kept")}, {"discontinuity", std::string("rate_change")}, {"crc_ok", true}};

        auto                      encode = make<SpacePacketEncode>({{"apid", gr::Size_t{10}}});
        const std::vector<Record> encodeIn{recordOf(std::vector<std::uint8_t>(8UZ, 0xA5U), meta)};
        const std::vector<Record> packets = drive1(encode, encodeIn);
        expect(eq(packets.size(), 1UZ));
        if (packets.size() != 1UZ) {
            return;
        }
        expect(eq(metaString(packets[0], "unrelated_key"), std::string("kept")));
        expect(eq(metaString(packets[0], "discontinuity"), std::string("rate_change"))) << "untouched: this block detects no discontinuity";
        expect(metaBool(packets[0], "crc_ok"));
        expect(!metaHas(packets[0], "sequence")) << "sequence is a local producer's key and no block here writes it";

        auto             extract  = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
        gr::property_map zoneMeta = meta;
        zoneMeta.insert_or_assign(gr::property_map::key_type("ccsds_vcid"), gr::pmt::Value(gr::Size_t{0}));
        zoneMeta.insert_or_assign(gr::property_map::key_type("ccsds_first_header_pointer"), gr::pmt::Value(gr::Size_t{0}));
        zoneMeta.insert_or_assign(gr::property_map::key_type("ccsds_vc_frame_count"), gr::pmt::Value(gr::Size_t{0}));
        const std::vector<Record> zone{recordOf(packets[0].signal_values, zoneMeta)};
        const std::vector<Record> extracted = drive1(extract, zone);
        expect(eq(extracted.size(), 1UZ));
        if (extracted.size() != 1UZ) {
            return;
        }
        expect(eq(metaString(extracted[0], "unrelated_key"), std::string("kept")));
        expect(eq(metaString(extracted[0], "discontinuity"), std::string("rate_change")));
        expect(metaBool(extracted[0], "crc_ok"));
        expect(eq(metaString(extracted[0], "protocol"), std::string("ccsds/space_packet")));
        expect(!metaHas(extracted[0], "sequence"));

        auto                      decode  = make<SpacePacketDecode>({});
        const std::vector<Record> decoded = drive1(decode, extracted);
        expect(eq(decoded.size(), 1UZ));
        if (decoded.size() == 1UZ) {
            expect(eq(metaString(decoded[0], "unrelated_key"), std::string("kept")));
            expect(eq(metaString(decoded[0], "discontinuity"), std::string("rate_change")));
            expect(metaBool(decoded[0], "crc_ok")) << "crc_ok, written by CrcCheck upstream, survives to here";
            expect(!metaHas(decoded[0], "sequence"));
        }

        // SpacePacketSegment builds its zones from the octets of many records and carries none of their keys: a zone is
        // not any one packet's record, and a spanning packet would otherwise leave its keys on an arbitrary zone.
        auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{20}}});
        const std::vector<Record> segmentIn{recordOf(std::vector<std::uint8_t>(20UZ, 1U), meta), recordOf(std::vector<std::uint8_t>(20UZ, 2U), meta)};
        const std::vector<Record> zones = drive1(segment, segmentIn);
        expect(eq(zones.size(), 2UZ));
        if (zones.size() == 2UZ) {
            expect(!metaHas(zones[0], "unrelated_key"));
            expect(!metaHas(zones[0], "discontinuity"));
            expect(metaHas(zones[0], "ccsds_first_header_pointer"));
        }
    };

    "criterion 22: every packet block's record is admissible at both boundaries"_test = [] {
        auto toStream = make<gr::blocks::basic::DataSetToStream<std::uint8_t>>({});
        auto toPacket = make<gr::blocks::basic::DataSetToPacket<std::uint8_t>>({});

        const auto admitted = [&toStream, &toPacket](const Record& record, std::string_view label) {
            const std::vector<Record> in{record};

            std::vector<std::uint8_t> streamBuf(record.signal_values.size() + 8UZ);
            std::vector<Record>       streamReject(4UZ);
            InputSpan<Record>         streamIn{std::span<const Record>(in)};
            OutputSpan<std::uint8_t>  streamOut{std::span<std::uint8_t>(streamBuf)};
            OutputSpan<Record>        streamRejectSpan{std::span<Record>(streamReject)};
            std::ignore = toStream.processBulk(streamIn, streamOut, streamRejectSpan);
            expect(eq(streamRejectSpan.count, std::size_t{0})) << label << ": A1-A5 pass";
            expect(eq(streamOut.count, record.signal_values.size())) << label << ": the accepted path ran, every octet published";

            std::vector<gr::Packet<std::uint8_t>> packetBuf(4UZ);
            std::vector<Record>                   packetReject(4UZ);
            InputSpan<Record>                     packetIn{std::span<const Record>(in)};
            OutputSpan<gr::Packet<std::uint8_t>>  packetOut{std::span<gr::Packet<std::uint8_t>>(packetBuf)};
            OutputSpan<Record>                    packetRejectSpan{std::span<Record>(packetReject)};
            std::ignore = toPacket.processBulk(packetIn, packetOut, packetRejectSpan);
            expect(eq(packetRejectSpan.count, std::size_t{0})) << label << ": P1-P6 pass";
            expect(eq(packetOut.count, std::size_t{1})) << label << ": the accepted path ran, one packet published";
        };

        auto                      encode = make<SpacePacketEncode>({{"apid", gr::Size_t{10}}});
        const std::vector<Record> encodeIn{recordOf(std::vector<std::uint8_t>(8UZ, 0x5AU))};
        const std::vector<Record> encoded = drive1(encode, encodeIn);
        expect(eq(encoded.size(), 1UZ));

        auto                      segment = make<SpacePacketSegment>({{"zone_length", gr::Size_t{14}}});
        const std::vector<Record> zones   = drive1(segment, encoded);
        expect(eq(zones.size(), 1UZ));

        auto                      extract   = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});
        const std::vector<Record> extracted = drive1(extract, zonesForExtraction(zones));
        expect(eq(extracted.size(), 1UZ));

        auto                      decode  = make<SpacePacketDecode>({});
        const std::vector<Record> decoded = drive1(decode, extracted);
        expect(eq(decoded.size(), 1UZ));

        if (encoded.size() == 1UZ && zones.size() == 1UZ && extracted.size() == 1UZ && decoded.size() == 1UZ) {
            admitted(encoded[0], "SpacePacketEncode");
            admitted(zones[0], "SpacePacketSegment");
            admitted(extracted[0], "SpacePacketExtract");
            admitted(decoded[0], "SpacePacketDecode");
        }
    };
};

int main() { /* not needed for UT */ }
