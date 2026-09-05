#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <thread>
#include <vector>

using namespace std::string_literals;

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/digital/Crc.hpp>
#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/ccsds/FieldRouter.hpp>
#include <gnuradio-4.0/ccsds/Frames.hpp>
#include <gnuradio-4.0/ccsds/SpacePackets.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
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

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

[[nodiscard]] bool metaHas(const Record& record, std::string_view key) { return metaOf(record).find(gr::property_map::key_type(key)) != metaOf(record).end(); }

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? gr::Size_t{0xFFFFFFFFU} : entry->second.value_or(gr::Size_t{0xFFFFFFFFU});
}

/// CRC-16/IBM-3740, the CCSDS frame error control field's parameter set (132.0-B-3 4.1.6.2.2).
[[nodiscard]] gr::property_map fecf() {
    return {{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, {"final_xor", std::uint64_t{0x0000}}, //
        {"input_reflected", false}, {"result_reflected", false}, {"crc_byte_order", std::string("big")}};
}

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos   = 0UZ;
    std::size_t         _chunk = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        std::size_t room = outSpan.size();
        if (_chunk != 0UZ) {
            room = std::min(room, _chunk);
        }
        const std::size_t n = std::min(room, _records.size() - _pos);
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

/// Drops the frame at zero-based index `_dropAt` from the wire, so a lost-frame scenario can be built without a
/// captured recording: every other frame passes unchanged.
struct DropNth : gr::Block<DropNth> {
    gr::PortIn<Record, gr::Async>  in;
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(DropNth, in, out);
    std::ptrdiff_t _dropAt = -1;
    std::ptrdiff_t _index  = 0;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed, ++_index) {
            if (_index == _dropAt) {
                continue;
            }
            outSpan[made] = inSpan[consumed];
            ++made;
        }
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        return made == 0UZ && consumed == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

template<typename TCollect>
void runGraph(gr::Graph flow, TCollect&& collect) {
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
    collect();
}

[[nodiscard]] std::vector<Record> seededPayloads(int n, std::span<const gr::Size_t> apids) {
    std::vector<Record> payloads;
    std::uint32_t       seed = 12345U;
    const auto          rnd  = [&seed] {
        seed = seed * 1103515245U + 12345U;
        return seed;
    };
    for (int i = 0; i < n; ++i) {
        const std::size_t         length = 4UZ + (rnd() % 60UZ);
        std::vector<std::uint8_t> bytes(length);
        for (auto& b : bytes) {
            b = static_cast<std::uint8_t>(rnd());
        }
        gr::property_map meta{{"ccsds_apid", apids[static_cast<std::size_t>(i) % apids.size()]}};
        payloads.push_back(recordOf(bytes, meta));
    }
    return payloads;
}

} // namespace

const boost::ut::suite<"CcsdsChain"> ccsdsChainTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::ccsds;
    using gr::blocks::digital::CrcAppend;
    using gr::blocks::digital::CrcCheck;

    static constexpr gr::Size_t                kZoneLength  = 242U; // 248 (frame_length) - 6 (TM primary header)
    static constexpr gr::Size_t                kFrameLength = 248U; // pre-CRC size; CrcAppend grows the wire frame to 250
    static constexpr std::array<gr::Size_t, 3> kApids{gr::Size_t{10}, gr::Size_t{20}, gr::Size_t{30}};

    "the receive chain, end to end, under the scheduler"_test = [] {
        // The segmenter holds octets back until a zone fills, so the run closes with one packet sized to fill the last
        // zone exactly. Every packet sent is then a packet received, and the comparison is over the whole stream.
        const std::vector<Record> payloads = [] {
            std::vector<Record> records = seededPayloads(30, std::span<const gr::Size_t>(kApids));
            std::size_t         octets  = 0UZ;
            for (const Record& record : records) {
                octets += gr::ccsds::kSpacePacketHeaderSize + record.signal_values.size();
            }
            std::size_t closing = (kZoneLength - (octets + gr::ccsds::kSpacePacketHeaderSize) % kZoneLength) % kZoneLength;
            if (closing == 0UZ) {
                closing = kZoneLength;
            }
            std::vector<std::uint8_t> bytes(closing);
            for (std::size_t i = 0UZ; i < closing; ++i) {
                bytes[i] = static_cast<std::uint8_t>((i * 13UZ + 5UZ) & 0xFFUZ);
            }
            records.push_back(recordOf(bytes, {{"ccsds_apid", kApids[records.size() % kApids.size()]}}));
            return records;
        }();

        std::vector<std::vector<Record>> perChunk;
        for (const std::size_t chunk : {std::size_t{1}, std::size_t{17}, std::size_t{4096}}) {
            gr::Graph flow;
            auto&     source             = flow.emplaceBlock<RecordSource>();
            source._records              = payloads;
            source._chunk                = chunk;
            auto&            pktEncode   = flow.emplaceBlock<SpacePacketEncode>(gr::property_map{{"apid", kApids[0]}});
            auto&            segment     = flow.emplaceBlock<SpacePacketSegment>(gr::property_map{{"zone_length", kZoneLength}});
            auto&            frameEncode = flow.emplaceBlock<TmFrameEncode>(gr::property_map{{"frame_length", kFrameLength}, {"spacecraft_id", gr::Size_t{7}}, {"virtual_channel", gr::Size_t{0}}});
            auto&            crcAppend   = flow.emplaceBlock<CrcAppend>(fecf());
            gr::property_map checking    = fecf();
            checking["discard_crc"]      = true;
            auto& crcCheck               = flow.emplaceBlock<CrcCheck>(checking);
            auto& frameDecode            = flow.emplaceBlock<TmFrameDecode>(gr::property_map{{"frame_length", kFrameLength}});
            // FieldRouter sits where 132.0-B-3's own service model puts it: after the frame decode, routing by the
            // field the frame decode wrote (ccsds_vcid), before per-channel packet extraction. The three APIDs are
            // multiplexed within this one virtual channel and survive to SpacePacketDecode's own ccsds_apid key.
            auto& router    = flow.emplaceBlock<FieldRouter>(gr::property_map{{"field", std::string("virtual_channel")}, {"values", std::vector<gr::Size_t>{gr::Size_t{0}}}});
            auto& extract   = flow.emplaceBlock<SpacePacketExtract>(gr::property_map{{"virtual_channel", gr::Size_t{0}}});
            auto& pktDecode = flow.emplaceBlock<SpacePacketDecode>();
            auto& sink      = flow.emplaceBlock<RecordSink>();

            expect(flow.connect<"out", "in">(source, pktEncode).has_value());
            expect(flow.connect<"out", "in">(pktEncode, segment).has_value());
            expect(flow.connect<"out", "in">(segment, frameEncode).has_value());
            expect(flow.connect<"out", "in">(frameEncode, crcAppend).has_value());
            expect(flow.connect<"out", "in">(crcAppend, crcCheck).has_value());
            expect(flow.connect<"ok", "in">(crcCheck, frameDecode).has_value());
            expect(flow.connect<"out", "in">(frameDecode, router).has_value());
            expect(flow.connect(router, "outputs#0"s, extract, "in"s).has_value());
            expect(flow.connect<"out", "in">(extract, pktDecode).has_value());
            expect(flow.connect<"out", "in">(pktDecode, sink).has_value());

            std::vector<Record> received;
            runGraph(std::move(flow), [&received, &sink] { received = sink._records; });

            expect(eq(received.size(), payloads.size())) << "every packet arrives, chunk size " << chunk;
            std::array<gr::Size_t, 3> counts{}; // the encoder's own per-APID sequence count, one per APID it emitted
            for (std::size_t i = 0UZ; i < received.size() && i < payloads.size(); ++i) {
                const gr::Size_t  apid = metaSize(payloads[i], "ccsds_apid");
                const std::size_t slot = i % kApids.size();
                expect(that % (received[i].signal_values == payloads[i].signal_values)) << "packet " << i << " byte for byte, in order, chunk size " << chunk;
                expect(eq(metaSize(received[i], "ccsds_apid"), apid)) << "packet " << i << " reached the port its APID says, chunk size " << chunk;
                expect(eq(metaSize(received[i], "ccsds_packet_sequence_count"), counts[slot])) << "packet " << i << " carries the encoder's own count, chunk size " << chunk;
                expect(eq(metaSize(received[i], "ccsds_packet_data_length"), static_cast<gr::Size_t>(payloads[i].signal_values.size() - 1UZ))) << "the field as transmitted, packet " << i;
                expect(eq(metaSize(received[i], "ccsds_packet_version"), gr::Size_t{0}));
                expect(eq(metaSize(received[i], "ccsds_sequence_flags"), gr::Size_t{3})) << "unsegmented, the encoder's default";
                expect(eq(metaSize(received[i], "ccsds_vcid"), gr::Size_t{0})) << "the frame decode's key crossed both packet blocks";
                expect(eq(metaString(received[i], "protocol"), std::string("ccsds/space_packet")));
                expect(metaHas(received[i], "crc_ok")) << "written by CrcCheck upstream and never touched here";
                counts[slot] += 1U;
            }
            perChunk.push_back(std::move(received));
        }

        expect(eq(perChunk.size(), 3UZ));
        for (std::size_t arm = 1UZ; arm < perChunk.size(); ++arm) {
            expect(eq(perChunk[arm].size(), perChunk[0].size())) << "the chunk sizes agree on how many packets there were";
            for (std::size_t i = 0UZ; i < perChunk[arm].size() && i < perChunk[0].size(); ++i) {
                expect(that % (perChunk[arm][i].signal_values == perChunk[0][i].signal_values)) << "record " << i << " is identical across chunk sizes";
                expect(eq(metaSize(perChunk[arm][i], "ccsds_apid"), metaSize(perChunk[0][i], "ccsds_apid")));
                expect(eq(metaSize(perChunk[arm][i], "ccsds_packet_sequence_count"), metaSize(perChunk[0][i], "ccsds_packet_sequence_count")));
            }
        }
    };

    "the AOS chain, m_pdu data unit"_test = [] {
        const std::vector<Record> payloads = seededPayloads(20, std::span<const gr::Size_t>(kApids));

        gr::Graph flow;
        auto&     source             = flow.emplaceBlock<RecordSource>();
        source._records              = payloads;
        auto&            pktEncode   = flow.emplaceBlock<SpacePacketEncode>(gr::property_map{{"apid", kApids[0]}});
        auto&            segment     = flow.emplaceBlock<SpacePacketSegment>(gr::property_map{{"zone_length", gr::Size_t{242}}}); // 250 (frame) - 6 - 2 (M_PDU header)
        auto&            frameEncode = flow.emplaceBlock<AosFrameEncode>(gr::property_map{{"frame_length", gr::Size_t{250}}, {"spacecraft_id", gr::Size_t{7}}, {"virtual_channel", gr::Size_t{0}}, {"data_unit", std::string("m_pdu")}});
        auto&            crcAppend   = flow.emplaceBlock<CrcAppend>(fecf());
        gr::property_map checking    = fecf();
        checking["discard_crc"]      = true;
        auto& crcCheck               = flow.emplaceBlock<CrcCheck>(checking);
        auto& frameDecode            = flow.emplaceBlock<AosFrameDecode>(gr::property_map{{"frame_length", gr::Size_t{250}}, {"data_unit", std::string("m_pdu")}});
        auto& extract                = flow.emplaceBlock<SpacePacketExtract>(gr::property_map{{"virtual_channel", gr::Size_t{0}}});
        auto& pktDecode              = flow.emplaceBlock<SpacePacketDecode>();
        auto& sink                   = flow.emplaceBlock<RecordSink>();

        expect(flow.connect<"out", "in">(source, pktEncode).has_value());
        expect(flow.connect<"out", "in">(pktEncode, segment).has_value());
        expect(flow.connect<"out", "in">(segment, frameEncode).has_value());
        expect(flow.connect<"out", "in">(frameEncode, crcAppend).has_value());
        expect(flow.connect<"out", "in">(crcAppend, crcCheck).has_value());
        expect(flow.connect<"ok", "in">(crcCheck, frameDecode).has_value());
        expect(flow.connect<"out", "in">(frameDecode, extract).has_value());
        expect(flow.connect<"out", "in">(extract, pktDecode).has_value());
        expect(flow.connect<"out", "in">(pktDecode, sink).has_value());

        std::vector<Record> received;
        runGraph(std::move(flow), [&received, &sink] { received = sink._records; });
        expect(gt(received.size(), 0UZ));

        // the pointer read is the M_PDU's, and has_fhec grows the primary header by two octets
        auto                        aosDecode = make<AosFrameDecode>({{"frame_length", gr::Size_t{250}}, {"data_unit", std::string("m_pdu")}, {"has_fhec", true}});
        Record                      raw       = recordOf(std::vector<std::uint8_t>(250UZ, 0U));
        gr::ccsds::AosPrimaryHeader header{.version = 1, .spacecraft_id = 1, .virtual_channel = 0, .vc_frame_count = 0, .replay = false, .vc_count_cycle_used = false, .reserved = 0, .vc_count_cycle = 0, .has_fhec = true, .frame_header_error_control = 0};
        std::ignore = gr::ccsds::writeAosPrimaryHeader(header, raw.signal_values);
        std::vector<Record>       scratch(4UZ);
        std::vector<Record>       ocfScratch(4UZ);
        const std::vector<Record> rawIn{raw};
        InputSpan<Record>         inSpan{std::span<const Record>(rawIn)};
        OutputSpan<Record>        outSpan{std::span<Record>(scratch)};
        OutputSpan<Record>        ocfSpan{std::span<Record>(ocfScratch)};
        std::ignore = aosDecode.processBulk(inSpan, outSpan, ocfSpan);
        expect(eq(outSpan.count, std::size_t{1}));
        if (outSpan.count == 1UZ) {
            expect(eq(scratch[0].signal_values.size(), std::size_t{250 - 8 - 2})) << "8-octet primary header with FHEC, minus the 2-octet M_PDU header";
        }
    };

    "a lost frame in the full chain"_test = [] {
        const std::vector<Record> payloads = seededPayloads(30, std::span<const gr::Size_t>(kApids));

        gr::Graph flow;
        auto&     source           = flow.emplaceBlock<RecordSource>();
        source._records            = payloads;
        auto& pktEncode            = flow.emplaceBlock<SpacePacketEncode>(gr::property_map{{"apid", kApids[0]}});
        auto& segment              = flow.emplaceBlock<SpacePacketSegment>(gr::property_map{{"zone_length", gr::Size_t{60}}});
        auto& frameEncode          = flow.emplaceBlock<TmFrameEncode>(gr::property_map{{"frame_length", gr::Size_t{66}}, {"spacecraft_id", gr::Size_t{7}}, {"virtual_channel", gr::Size_t{0}}});
        auto& dropper              = flow.emplaceBlock<DropNth>();
        dropper._dropAt            = 3;
        auto&            crcAppend = flow.emplaceBlock<CrcAppend>(fecf());
        gr::property_map checking  = fecf();
        checking["discard_crc"]    = true;
        auto& crcCheck             = flow.emplaceBlock<CrcCheck>(checking);
        auto& frameDecode          = flow.emplaceBlock<TmFrameDecode>(gr::property_map{{"frame_length", gr::Size_t{66}}});
        auto& extract              = flow.emplaceBlock<SpacePacketExtract>(gr::property_map{{"virtual_channel", gr::Size_t{0}}});
        auto& sink                 = flow.emplaceBlock<RecordSink>();

        expect(flow.connect<"out", "in">(source, pktEncode).has_value());
        expect(flow.connect<"out", "in">(pktEncode, segment).has_value());
        expect(flow.connect<"out", "in">(segment, frameEncode).has_value());
        expect(flow.connect<"out", "in">(frameEncode, dropper).has_value());
        expect(flow.connect<"out", "in">(dropper, crcAppend).has_value());
        expect(flow.connect<"out", "in">(crcAppend, crcCheck).has_value());
        expect(flow.connect<"ok", "in">(crcCheck, frameDecode).has_value());
        expect(flow.connect<"out", "in">(frameDecode, extract).has_value());
        expect(flow.connect<"out", "in">(extract, sink).has_value());

        std::vector<Record> received;
        runGraph(std::move(flow), [&received, &sink] { received = sink._records; });

        expect(gt(frameDecode.nFramesLost, std::uint64_t{0}));
        expect(eq(extract.frames_lost, std::uint64_t{1})) << "one frame deleted from the wire is one frame lost";
        expect(gt(extract.fragments_dropped, std::uint64_t{0}));
        std::size_t gapRecords = 0UZ;
        for (const Record& record : received) {
            const std::string causes = metaString(record, "discontinuity");
            if (causes.find("frame_gap") == std::string::npos) {
                continue;
            }
            ++gapRecords;
            expect(eq(causes, std::string("frame_gap"))) << "the cause is appended once, not once per block that saw the gap";
            expect(eq(metaSize(record, "ccsds_frames_lost"), gr::Size_t{1}));
        }
        expect(eq(gapRecords, std::size_t{1})) << "the discontinuity key is present on exactly one recovered record";
        expect(gt(received.size(), 0UZ)) << "the packet stream resumes";
    };

    "metadata conformance"_test = [] {
        auto frameDecode = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});

        std::vector<std::uint8_t>  frame1(20UZ, 0U);
        gr::ccsds::TmPrimaryHeader h1{.version = 0, .spacecraft_id = 1, .virtual_channel = 0, .ocf_present = false, .master_frame_count = 0, .vc_frame_count = 0, .secondary_header = false, .sync_flag = false, .packet_order = false, .segment_length_id = 3, .first_header_pointer = 0};
        std::ignore                       = gr::ccsds::writeTmPrimaryHeader(h1, frame1);
        std::vector<std::uint8_t>  frame2 = frame1;
        gr::ccsds::TmPrimaryHeader h2     = h1;
        h2.vc_frame_count                 = 5; // a gap of five - four missing
        h2.master_frame_count             = 5;
        std::ignore                       = gr::ccsds::writeTmPrimaryHeader(h2, frame2);

        gr::property_map          meta{{"discontinuity", std::string("rate_change")}, {"unrelated_key", std::string("kept")}};
        const std::vector<Record> in{recordOf(frame1), recordOf(frame2, meta)};

        std::vector<Record> scratch(4UZ), shScratch(4UZ), ocfScratch(4UZ);
        InputSpan<Record>   inSpan{std::span<const Record>(in)};
        OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
        OutputSpan<Record>  shSpan{std::span<Record>(shScratch)};
        OutputSpan<Record>  ocfSpan{std::span<Record>(ocfScratch)};
        std::ignore = frameDecode.processBulk(inSpan, outSpan, shSpan, ocfSpan);

        expect(eq(outSpan.count, std::size_t{2}));
        if (outSpan.count == 2UZ) {
            expect(eq(metaString(scratch[1], "discontinuity"), std::string("rate_change,frame_gap"))) << "appended, not replaced";
            expect(eq(metaString(scratch[1], "unrelated_key"), std::string("kept")));
        }

        if (outSpan.count == 2UZ) {
            expect(!metaHas(scratch[0], "sequence")) << "sequence is never written by any block in this module";
            expect(!metaHas(scratch[1], "sequence"));
        }

        // crc_ok, written upstream by CrcCheck, survives verbatim through the decode this module owns
        auto                      crcCheck = make<CrcCheck>(fecf());
        std::vector<std::uint8_t> checked  = frame1;
        const gr::digital::Crc    crc(16U, 0x1021ULL, 0xFFFFULL, 0x0000ULL, false, false);
        const std::uint64_t       value = crc.compute(checked);
        checked.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
        checked.push_back(static_cast<std::uint8_t>(value & 0xFFU));

        std::vector<Record>       okBuf(4UZ), failBuf(4UZ);
        const std::vector<Record> checkedIn{recordOf(checked)};
        InputSpan<Record>         crcIn{std::span<const Record>(checkedIn)};
        OutputSpan<Record>        okSpan{std::span<Record>(okBuf)};
        OutputSpan<Record>        failSpan{std::span<Record>(failBuf)};
        std::ignore = crcCheck.processBulk(crcIn, okSpan, failSpan);
        expect(eq(okSpan.count, std::size_t{1}));
        if (okSpan.count == 1UZ) {
            expect(metaHas(okBuf[0], "crc_ok"));

            auto                      frameDecode2 = make<TmFrameDecode>({{"frame_length", gr::Size_t{22}}, {"has_fecf", true}});
            std::vector<Record>       outBuf2(4UZ), shBuf2(4UZ), ocfBuf2(4UZ);
            const std::vector<Record> okIn{okBuf[0]};
            InputSpan<Record>         in2{std::span<const Record>(okIn)};
            OutputSpan<Record>        out2{std::span<Record>(outBuf2)};
            OutputSpan<Record>        sh2{std::span<Record>(shBuf2)};
            OutputSpan<Record>        ocf2{std::span<Record>(ocfBuf2)};
            std::ignore = frameDecode2.processBulk(in2, out2, sh2, ocf2);
            expect(eq(out2.count, std::size_t{1}));
            if (out2.count == 1UZ) {
                expect(metaHas(outBuf2[0], "crc_ok")) << "crc_ok crosses the decode verbatim";
            }
        }
    };

    "admission at both carrier boundaries"_test = [] {
        auto                       frameDecode = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});
        std::vector<std::uint8_t>  frame(20UZ, 0U);
        gr::ccsds::TmPrimaryHeader h{.version = 0, .spacecraft_id = 1, .virtual_channel = 0, .ocf_present = false, .master_frame_count = 0, .vc_frame_count = 0, .secondary_header = false, .sync_flag = false, .packet_order = false, .segment_length_id = 3, .first_header_pointer = 0};
        std::ignore = gr::ccsds::writeTmPrimaryHeader(h, frame);

        std::vector<Record>       outBuf(4UZ), shBuf(4UZ), ocfBuf(4UZ);
        const std::vector<Record> frameIn{recordOf(frame)};
        InputSpan<Record>         inSpan{std::span<const Record>(frameIn)};
        OutputSpan<Record>        outSpan{std::span<Record>(outBuf)};
        OutputSpan<Record>        shSpan{std::span<Record>(shBuf)};
        OutputSpan<Record>        ocfSpan{std::span<Record>(ocfBuf)};
        std::ignore = frameDecode.processBulk(inSpan, outSpan, shSpan, ocfSpan);
        expect(eq(outSpan.count, std::size_t{1}));
        if (outSpan.count != 1UZ) {
            return;
        }

        auto toStream = make<gr::blocks::basic::DataSetToStream<std::uint8_t>>({});
        auto toPacket = make<gr::blocks::basic::DataSetToPacket<std::uint8_t>>({});

        std::vector<std::uint8_t> streamBuf(64UZ);
        std::vector<Record>       streamReject(4UZ);
        const std::vector<Record> admissionIn{outBuf[0]};
        InputSpan<Record>         streamIn{std::span<const Record>(admissionIn)};
        OutputSpan<std::uint8_t>  streamOut{std::span<std::uint8_t>(streamBuf)};
        OutputSpan<Record>        streamRejectSpan{std::span<Record>(streamReject)};
        std::ignore = toStream.processBulk(streamIn, streamOut, streamRejectSpan);
        expect(eq(streamRejectSpan.count, std::size_t{0})) << "A1-A5 pass: the record is not rejected at the stream boundary";
        expect(eq(streamOut.count, outBuf[0].signal_values.size())) << "and it left by the accepted path, every octet published";

        std::vector<gr::Packet<std::uint8_t>> packetBuf(4UZ);
        std::vector<Record>                   packetReject(4UZ);
        InputSpan<Record>                     packetIn{std::span<const Record>(admissionIn)};
        OutputSpan<gr::Packet<std::uint8_t>>  packetOut{std::span<gr::Packet<std::uint8_t>>(packetBuf)};
        OutputSpan<Record>                    packetRejectSpan{std::span<Record>(packetReject)};
        std::ignore = toPacket.processBulk(packetIn, packetOut, packetRejectSpan);
        expect(eq(packetRejectSpan.count, std::size_t{0})) << "P1-P6 pass: the record is not rejected at the packet boundary";
        expect(eq(packetOut.count, std::size_t{1})) << "and it left by the accepted path, one packet published";
    };
};

int main() { /* not needed for UT */ }
