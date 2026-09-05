#include <boost/ut.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/algorithm/digital/Crc.hpp>
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

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? gr::Size_t{0xFFFFFFFFU} : entry->second.value_or(gr::Size_t{0xFFFFFFFFU});
}

[[nodiscard]] bool metaHas(const Record& record, std::string_view key) { return metaOf(record).find(gr::property_map::key_type(key)) != metaOf(record).end(); }

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
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

/// What a TM decode published on each of its three ports in one call.
struct TmPorts {
    std::vector<Record> out{};
    std::vector<Record> sh{};
    std::vector<Record> ocf{};
};

/// Drive a TM decode with the two optional ports connected or not, an unconnected port being the case that has to
/// publish nothing and count everything.
template<typename TBlock>
[[nodiscard]] TmPorts driveTm(TBlock& block, std::span<const Record> in, bool shConnected = true, bool ocfConnected = true, std::size_t room = 64UZ) {
    std::vector<Record> outBuf(room);
    std::vector<Record> shBuf(room);
    std::vector<Record> ocfBuf(room);
    InputSpan<Record>   inSpan(in);
    OutputSpan<Record>  outSpan{std::span<Record>(outBuf)};
    OutputSpan<Record>  shSpan{std::span<Record>(shBuf), 0UZ, nullptr, shConnected};
    OutputSpan<Record>  ocfSpan{std::span<Record>(ocfBuf), 0UZ, nullptr, ocfConnected};
    std::ignore = block.processBulk(inSpan, outSpan, shSpan, ocfSpan);

    TmPorts ports;
    ports.out.assign(outBuf.begin(), outBuf.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    ports.sh.assign(shBuf.begin(), shBuf.begin() + static_cast<std::ptrdiff_t>(shSpan.count));
    ports.ocf.assign(ocfBuf.begin(), ocfBuf.begin() + static_cast<std::ptrdiff_t>(ocfSpan.count));
    return ports;
}

/// The TM primary header the refusal, gap and channel scenes each vary one field of.
[[nodiscard]] gr::ccsds::TmPrimaryHeader tmHeader(std::uint8_t vcid, std::uint8_t mcCount, std::uint8_t vcCount) {
    return gr::ccsds::TmPrimaryHeader{.version = 0,
        .spacecraft_id                         = 1,
        .virtual_channel                       = vcid,
        .ocf_present                           = false,
        .master_frame_count                    = mcCount,
        .vc_frame_count                        = vcCount, //
        .secondary_header                      = false,
        .sync_flag                             = false,
        .packet_order                          = false,
        .segment_length_id                     = 3,
        .first_header_pointer                  = 0};
}

/// A TM frame of `frameLength` octets: the header written over a zero-filled record.
[[nodiscard]] std::vector<std::uint8_t> tmFrame(const gr::ccsds::TmPrimaryHeader& header, std::size_t frameLength) {
    std::vector<std::uint8_t> frame(frameLength, 0U);
    std::ignore = gr::ccsds::writeTmPrimaryHeader(header, frame);
    return frame;
}

/// CRC-16/IBM-3740, the frame error control field's parameter set (132.0-B-3 4.1.6.2.2).
[[nodiscard]] gr::property_map fecfSettings() {
    return {{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, {"final_xor", std::uint64_t{0x0000}}, //
        {"input_reflected", false}, {"result_reflected", false}, {"crc_byte_order", std::string("big")}};
}

} // namespace

const boost::ut::suite<"CcsdsFrames"> ccsdsFramesTests = [] {
    using namespace boost::ut;
    using namespace gr::ccsds;
    using namespace gr::blocks::ccsds;

    "worked octet examples from the standards"_test = [] {
        // TM primary header: TFVN 00, SCID 42, VCID 1, OCF set, MC 200, VC 17, TFSH clear, sync clear,
        // packet order clear, segment length '11', FHP 0 -> 02 A3 C8 11 18 00 (132.0-B-3 4.1.2)
        {
            const std::vector<std::uint8_t> octets{0x02, 0xA3, 0xC8, 0x11, 0x18, 0x00};
            TmPrimaryHeader                 header{};
            expect(parseTmPrimaryHeader(octets, header) == ParseStatus::ok);
            expect(eq(header.version, std::uint8_t{0}));
            expect(eq(header.spacecraft_id, std::uint16_t{42}));
            expect(eq(header.virtual_channel, std::uint8_t{1}));
            expect(header.ocf_present);
            expect(eq(header.master_frame_count, std::uint8_t{200}));
            expect(eq(header.vc_frame_count, std::uint8_t{17}));
            expect(!header.secondary_header);
            expect(!header.sync_flag);
            expect(!header.packet_order);
            expect(eq(header.segment_length_id, std::uint8_t{3}));
            expect(eq(header.first_header_pointer, std::uint16_t{0}));
            std::vector<std::uint8_t> written(6UZ, 0U);
            expect(writeTmPrimaryHeader(header, written) == WriteStatus::ok);
            expect(that % (written == octets));
        }
        // same header, FHP 2047 and FHP 2046 -> 02 A3 C8 11 1F FF and 02 A3 C8 11 1F FE
        {
            TmPrimaryHeader           header{.version = 0, .spacecraft_id = 42, .virtual_channel = 1, .ocf_present = true, .master_frame_count = 200, .vc_frame_count = 17, .secondary_header = false, .sync_flag = false, .packet_order = false, .segment_length_id = 3, .first_header_pointer = kFhpNoPacketStart};
            std::vector<std::uint8_t> written(6UZ, 0U);
            expect(writeTmPrimaryHeader(header, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x02, 0xA3, 0xC8, 0x11, 0x1F, 0xFF}));
            header.first_header_pointer = kFhpOnlyIdleData;
            expect(writeTmPrimaryHeader(header, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x02, 0xA3, 0xC8, 0x11, 0x1F, 0xFE}));
        }
        // AOS primary header: TFVN 01, SCID 42, VCID 63, VC count 0xFFFFFF, replay set, cycle use set,
        // cycle 15 -> 4A BF FF FF FF CF
        {
            AosPrimaryHeader          header{.version = 1, .spacecraft_id = 42, .virtual_channel = 63, .vc_frame_count = 0xFFFFFFU, .replay = true, .vc_count_cycle_used = true, .reserved = 0, .vc_count_cycle = 15, .has_fhec = false, .frame_header_error_control = 0};
            std::vector<std::uint8_t> written(6UZ, 0U);
            expect(writeAosPrimaryHeader(header, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x4A, 0xBF, 0xFF, 0xFF, 0xFF, 0xCF})) << "the same SCID of 42 gives 02 in TM and 4A in AOS";
            AosPrimaryHeader parsed{};
            expect(parseAosPrimaryHeader(written, false, parsed) == ParseStatus::ok);
            expect(that % (parsed == header));
        }
        // M_PDU header, FHP 2047 -> 07 FF
        {
            MpduHeader                mpdu{.reserved = 0, .first_header_pointer = kFhpNoPacketStart};
            std::vector<std::uint8_t> written(2UZ, 0U);
            expect(writeMpduHeader(mpdu, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x07, 0xFF}));
        }
        // TC primary header: SCID 42, VCID 1, frame length field 999, sequence 7 -> 00 2A 07 E7 07
        {
            TcPrimaryHeader           header{.version = 0, .bypass = false, .control_command = false, .reserved = 0, .spacecraft_id = 42, .virtual_channel = 1, .frame_length = 999, .sequence_number = 7};
            std::vector<std::uint8_t> written(5UZ, 0U);
            expect(writeTcPrimaryHeader(header, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x00, 0x2A, 0x07, 0xE7, 0x07}));
            expect(eq(totalTcFrameOctets(header), std::size_t{1000}));
        }
        // TM secondary header id, version 00, length 63 -> 3F
        {
            TmSecondaryHeaderId       id{.version = 0, .length = 63};
            std::vector<std::uint8_t> written(1UZ, 0U);
            expect(writeTmSecondaryHeaderId(id, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x3F}));
            expect(eq(tmSecondaryHeaderOctets(id), std::size_t{64}));
        }
        // CLCW: COP in effect 1, VCID 1, every flag set, FARM-B 3, report value 255 -> 01 04 FE FF
        {
            Clcw                      clcw{.control_word_type = false, .version = 0, .status = 0, .cop_in_effect = 1, .virtual_channel = 1, .reserved = 0, .no_rf_available = true, .no_bit_lock = true, .lockout = true, .wait = true, .retransmit = true, .farm_b_counter = 3, .reserved_bit = 0, .report_value = 255};
            std::vector<std::uint8_t> written(4UZ, 0U);
            expect(writeClcw(clcw, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x01, 0x04, 0xFE, 0xFF}));
        }
        // space packet primary header: APID 100, sequence flags 11, sequence count 5, data length 0 -> 00 64 C0 05 00 00
        {
            SpacePacketHeader         header{.version = 0, .type = false, .secondary_header = false, .apid = 100, .sequence_flags = 3, .sequence_count = 5, .data_length = 0};
            std::vector<std::uint8_t> written(6UZ, 0U);
            expect(writeSpacePacketHeader(header, written) == WriteStatus::ok);
            expect(that % (written == std::vector<std::uint8_t>{0x00, 0x64, 0xC0, 0x05, 0x00, 0x00}));
            expect(eq(totalPacketOctets(header), std::size_t{7}));
        }
    };

    "the minus-one, in numbers"_test = [] {
        SpacePacketHeader h1{};
        expect(headerForPayload(1, false, false, 3, 0, 1UZ, h1) == WriteStatus::ok);
        expect(eq(h1.data_length, std::uint16_t{0}));
        expect(eq(totalPacketOctets(h1), std::size_t{7}));

        SpacePacketHeader h256{};
        expect(headerForPayload(1, false, false, 3, 0, 256UZ, h256) == WriteStatus::ok);
        expect(eq(h256.data_length, std::uint16_t{255}));
        expect(eq(totalPacketOctets(h256), std::size_t{262}));

        SpacePacketHeader hMax{};
        expect(headerForPayload(1, false, false, 3, 0, 65536UZ, hMax) == WriteStatus::ok);
        expect(eq(hMax.data_length, std::uint16_t{65535}));
        expect(eq(totalPacketOctets(hMax), std::size_t{65542}));

        // the surveyed convention: data_length = payload_octets produces a total one octet short
        SpacePacketHeader wrong{.version = 0, .type = false, .secondary_header = false, .apid = 1, .sequence_flags = 3, .sequence_count = 0, .data_length = 100};
        expect(eq(totalPacketOctets(wrong), std::size_t{107})) << "a header built with data_length = payload_octets claims a total of 107 octets for a packet that occupies 106 on the wire";
    };

    "the OID fill anchor"_test = [] {
        OidFill                   fill;
        std::vector<std::uint8_t> out(10UZ, 0U);
        fill.next(out);
        expect(that % (out == std::vector<std::uint8_t>{0xFF, 0xFF, 0xFF, 0xFF, 0x6D, 0xB6, 0xD8, 0x61, 0x45, 0x1F}));
    };

    "the FECF, CRC-16/IBM-3740"_test = [] {
        const gr::digital::Crc          crc(16U, 0x1021ULL, 0xFFFFULL, 0x0000ULL, false, false);
        const std::vector<std::uint8_t> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        expect(eq(crc.compute(check), std::uint64_t{0x29B1}));

        std::vector<std::uint8_t> frame{0x02, 0xA3, 0xC8, 0x11, 0x18, 0x00, 0xAA, 0xBB, 0xCC};
        const std::uint64_t       fecf = crc.compute(frame);
        frame.push_back(static_cast<std::uint8_t>((fecf >> 8U) & 0xFFU));
        frame.push_back(static_cast<std::uint8_t>(fecf & 0xFFU));
        expect(eq(crc.compute(frame), std::uint64_t{0x0000})) << "the syndrome over the whole frame including the field is zero";
        frame[3] ^= 0x01U;
        expect(crc.compute(frame) != 0ULL) << "one flipped octet anywhere changes the syndrome";
    };

    "frame count gaps and the wrap"_test = [] {
        expect(eq(frameGap(1, 0, kTmCountModulus).lost, gr::Size_t{0}));
        expect(frameGap(1, 0, kTmCountModulus).continuous);
        expect(eq(frameGap(5, 0, kTmCountModulus).lost, gr::Size_t{4}));
        expect(frameGap(0, 0, kTmCountModulus).duplicate);
        expect(eq(frameGap(3, 254, kTmCountModulus).lost, gr::Size_t{4})) << "the wrap: (3 - 254) mod 256 = 5, four frames lost";
        expect(frameGap(0U, 0xFFFFFFU, kAosCountModulus).continuous) << "the AOS wrap: 0xFFFFFF then 0 is continuous at modulus 2^24";
        expect(eq(frameGap(0x000000, 0xFFFFFEU, kAosCountModulus).lost, gr::Size_t{1}));

        // the four-bit cycle widens the count to 28 bits: (cycle 0, 0xFFFFFF) then (cycle 1, 0) is continuous
        expect(frameGap(1U << 24U, 0xFFFFFFU, kAosCycleCountModulus).continuous);
        // and (cycle 0, 0xFFFFFF) then (cycle 2, 0) is a gap of exactly 2^24, the number the widening exists to produce
        expect(eq(frameGap(2U << 24U, 0xFFFFFFU, kAosCycleCountModulus).lost, gr::Size_t{1U << 24U}));
    };

    // ---- criterion 14: every decode refusal under its own counter, each followed by a record that decodes ----

    "TmFrameDecode refuses each way and recovers on the next record"_test = [] {
        auto block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});

        const std::vector<std::uint8_t> tooShort(10UZ, 0U);
        std::vector<std::uint8_t>       wrongVersion = tmFrame(tmHeader(0U, 0U, 0U), 20UZ);
        wrongVersion[0] |= 0x40U; // sets bit 1 of the version field, making TFVN '01'

        // a secondary header length field of 0 describes a one-octet header with no data field, which 4.1.3.2.3.2
        // against 4.1.3.1.3 makes structurally impossible
        TmPrimaryHeader shHeader        = tmHeader(0U, 0U, 0U);
        shHeader.secondary_header       = true;
        std::vector<std::uint8_t> badSh = tmFrame(shHeader, 20UZ);
        expect(writeTmSecondaryHeaderId(TmSecondaryHeaderId{.version = 0, .length = 0}, std::span<std::uint8_t>(badSh).subspan(kTmPrimaryHeaderSize)) == WriteStatus::ok);

        // 6 primary + 14 secondary + 4 OCF is 24 octets of overhead in a 20-octet frame, so no data field survives
        TmPrimaryHeader geometryHeader        = tmHeader(0U, 0U, 0U);
        geometryHeader.secondary_header       = true;
        geometryHeader.ocf_present            = true;
        std::vector<std::uint8_t> badGeometry = tmFrame(geometryHeader, 20UZ);
        expect(writeTmSecondaryHeaderId(TmSecondaryHeaderId{.version = 0, .length = 13}, std::span<std::uint8_t>(badGeometry).subspan(kTmPrimaryHeaderSize)) == WriteStatus::ok);

        // '11' is the only segment length identifier 4.1.2.7.5.2 admits under a zero sync flag: reported, refusing nothing
        TmPrimaryHeader reservedHeader           = tmHeader(0U, 0U, 0U);
        reservedHeader.segment_length_id         = 1U;
        const std::vector<std::uint8_t> reserved = tmFrame(reservedHeader, 20UZ);
        const std::vector<std::uint8_t> good     = tmFrame(tmHeader(0U, 1U, 1U), 20UZ);

        const std::vector<Record> records{recordOf(tooShort), recordOf(wrongVersion), recordOf(badSh), recordOf(badGeometry), recordOf(reserved), recordOf(good)};
        const TmPorts             ports = driveTm(block, records, false, false);

        expect(eq(ports.out.size(), 2UZ)) << "the reserved-violation frame and the well-formed one publish; the four refusals publish nothing";
        expect(eq(block.nRefusedShort, std::uint64_t{1}));
        expect(eq(block.nWrongVersion, std::uint64_t{1}));
        expect(eq(block.nBadSecondaryHeader, std::uint64_t{1}));
        expect(eq(block.nBadGeometry, std::uint64_t{1}));
        expect(eq(block.nReservedViolations, std::uint64_t{1}));
        expect(eq(block.nFrames, std::uint64_t{2}));
        expect(eq(block.nDuplicateFrames, std::uint64_t{0}));
        expect(eq(block.nFramesLost, std::uint64_t{0})) << "a refused frame never enters the gap state, so the two published frames are continuous";
    };

    "the spacecraft filter and the crc gate are counted drops"_test = [] {
        auto block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}, {"spacecraft_id", gr::Size_t{1}}, {"require_crc_ok", true}});

        TmPrimaryHeader elsewhere = tmHeader(0U, 0U, 0U);
        elsewhere.spacecraft_id   = 2U;

        const std::vector<Record> records{
            recordOf(tmFrame(elsewhere, 20UZ)),                                 //
            recordOf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ), {{"crc_ok", false}}), //
            recordOf(tmFrame(tmHeader(0U, 1U, 1U), 20UZ), {{"crc_ok", true}}),  //
            recordOf(tmFrame(tmHeader(0U, 2U, 2U), 20UZ)),                      // no crc_ok key: no check was claimed, so it passes
        };
        const TmPorts ports = driveTm(block, records, false, false);

        expect(eq(ports.out.size(), 2UZ));
        expect(eq(block.nFilteredScid, std::uint64_t{1}));
        expect(eq(block.nCrcFailed, std::uint64_t{1}));
        expect(eq(block.nFrames, std::uint64_t{2})) << "the record with no crc_ok key decodes";
    };

    "AosFrameDecode refuses a truncated record and TM's version number"_test = [] {
        auto block = make<AosFrameDecode>({{"frame_length", gr::Size_t{30}}, {"data_unit", std::string("m_pdu")}});

        std::vector<std::uint8_t> good(30UZ, 0U);
        const AosPrimaryHeader    header{.version = 1, .spacecraft_id = 7, .virtual_channel = 3, .vc_frame_count = 0, .replay = false, .vc_count_cycle_used = false, .reserved = 0, .vc_count_cycle = 0, .has_fhec = false, .frame_header_error_control = 0};
        expect(writeAosPrimaryHeader(header, good) == WriteStatus::ok);
        expect(writeMpduHeader(MpduHeader{.reserved = 0, .first_header_pointer = 0}, std::span<std::uint8_t>(good).subspan(kAosPrimaryHeaderSize)) == WriteStatus::ok);

        const std::vector<std::uint8_t> truncated(good.begin(), good.begin() + 20);
        std::vector<std::uint8_t>       tmVersion = good;
        tmVersion[0] &= 0x3FU; // clears the version field to '00', which is TM's and not AOS's

        const std::vector<Record> records{recordOf(truncated), recordOf(tmVersion), recordOf(good)};
        std::vector<Record>       outBuf(8UZ), ocfBuf(8UZ);
        InputSpan<Record>         inSpan{std::span<const Record>(records)};
        OutputSpan<Record>        outSpan{std::span<Record>(outBuf)};
        OutputSpan<Record>        ocfSpan{std::span<Record>(ocfBuf), 0UZ, nullptr, false};
        std::ignore = block.processBulk(inSpan, outSpan, ocfSpan);

        expect(eq(outSpan.count, std::size_t{1})) << "only the well-formed frame publishes";
        expect(eq(block.nRefusedShort, std::uint64_t{1}));
        expect(eq(block.nWrongVersion, std::uint64_t{1}));
        expect(eq(block.nFrames, std::uint64_t{1}));
    };

    "TcFrameDecode refuses a short record, a length past its end and a frame with no data field"_test = [] {
        TcFrameDecode block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();

        const auto frameOf = [](std::uint16_t lengthField, std::size_t octets) {
            const TcPrimaryHeader     header{.version = 0, .bypass = false, .control_command = false, .reserved = 0, .spacecraft_id = 1, .virtual_channel = 1, .frame_length = lengthField, .sequence_number = 0};
            std::vector<std::uint8_t> frame(octets, 0U);
            std::ignore = writeTcPrimaryHeader(header, frame);
            return frame;
        };

        const std::vector<std::uint8_t> tooShort(3UZ, 0U);                 // fewer octets than the five-octet header
        const std::vector<std::uint8_t> claimsMore   = frameOf(29U, 10UZ); // a declared total of 30 octets in a 10-octet record
        const std::vector<std::uint8_t> noData       = frameOf(4U, 6UZ);   // a declared total of 5 octets is the header alone
        std::vector<std::uint8_t>       wrongVersion = frameOf(9U, 10UZ);
        wrongVersion[0] |= 0x40U;
        const std::vector<std::uint8_t> good = frameOf(9U, 10UZ); // a declared total of 10 octets: five of data field

        const std::vector<Record> out = drive1(block, std::vector<Record>{recordOf(tooShort), recordOf(claimsMore), recordOf(noData), recordOf(wrongVersion), recordOf(good)});

        expect(eq(out.size(), 1UZ));
        expect(eq(block.nRefusedShort, std::uint64_t{2})) << "the three-octet record and the one whose declared length runs past its end";
        expect(eq(block.nBadGeometry, std::uint64_t{1}));
        expect(eq(block.nWrongVersion, std::uint64_t{1}));
        if (out.size() == 1UZ) {
            expect(eq(out[0].signal_values.size(), std::size_t{5}));
        }
    };

    "AosFrameDecode reads the M_PDU pointer and shrinks the zone by two octets"_test = [] {
        auto block = make<AosFrameDecode>({{"frame_length", gr::Size_t{30}}, {"data_unit", std::string("m_pdu")}});

        std::vector<std::uint8_t> frame(30UZ, 0U);
        AosPrimaryHeader          header{.version = 1, .spacecraft_id = 7, .virtual_channel = 3, .vc_frame_count = 0, .replay = false, .vc_count_cycle_used = false, .reserved = 0, .vc_count_cycle = 0, .has_fhec = false, .frame_header_error_control = 0};
        expect(writeAosPrimaryHeader(header, frame) == WriteStatus::ok);
        MpduHeader mpdu{.reserved = 0, .first_header_pointer = 5};
        expect(writeMpduHeader(mpdu, std::span<std::uint8_t>(frame).subspan(kAosPrimaryHeaderSize)) == WriteStatus::ok);

        const std::vector<Record> aosRecords{recordOf(frame)};
        std::vector<Record>       outBuf(4UZ), ocfBuf(4UZ);
        InputSpan<Record>         inSpan{std::span<const Record>(aosRecords)};
        OutputSpan<Record>        outSpan{std::span<Record>(outBuf)};
        OutputSpan<Record>        ocfSpan{std::span<Record>(ocfBuf), 0UZ, nullptr, false};
        std::ignore = block.processBulk(inSpan, outSpan, ocfSpan);
        const std::vector<Record> out(outBuf.begin(), outBuf.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0].signal_values.size(), std::size_t{30 - 6 - 2}));
            expect(eq(metaSize(out[0], "ccsds_first_header_pointer"), gr::Size_t{5}));
        }
    };

    "TcFrameDecode is self-describing and counts trailing octets"_test = [] {
        TcFrameDecode block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();

        TcPrimaryHeader           header{.version = 0, .bypass = false, .control_command = false, .reserved = 0, .spacecraft_id = 1, .virtual_channel = 1, .frame_length = 9, .sequence_number = 0}; // total 10 octets
        std::vector<std::uint8_t> frame(15UZ, 0xAAU);                                                                                                                                                // five octets of trailing surplus
        expect(writeTcPrimaryHeader(header, frame) == WriteStatus::ok);

        const std::vector<Record> out = drive1(block, std::vector<Record>{recordOf(frame)});
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0].signal_values.size(), std::size_t{5})); // 10 - 5 (header)
        }
        expect(eq(block.nTrailingOctets, std::uint64_t{5}));
    };

    "the secondary header and OCF ports"_test = [] {
        auto block = make<TmFrameDecode>({{"frame_length", gr::Size_t{6 + 20 + 10 + 4}}});

        std::vector<std::uint8_t> frame(6 + 20 + 10 + 4, 0U);
        TmPrimaryHeader           header{.version = 0, .spacecraft_id = 1, .virtual_channel = 0, .ocf_present = true, .master_frame_count = 0, .vc_frame_count = 0, .secondary_header = true, .sync_flag = false, .packet_order = false, .segment_length_id = 3, .first_header_pointer = 0};
        expect(writeTmPrimaryHeader(header, frame) == WriteStatus::ok);
        TmSecondaryHeaderId shId{.version = 0, .length = 19}; // 20-octet secondary header: 19 octets of data behind the id octet
        expect(writeTmSecondaryHeaderId(shId, std::span<std::uint8_t>(frame).subspan(kTmPrimaryHeaderSize)) == WriteStatus::ok);
        Clcw clcw{.control_word_type = false, .version = 0, .status = 0, .cop_in_effect = 0, .virtual_channel = 0, .reserved = 0, .no_rf_available = false, .no_bit_lock = false, .lockout = false, .wait = false, .retransmit = false, .farm_b_counter = 0, .reserved_bit = 0, .report_value = 0};
        expect(writeClcw(clcw, std::span<std::uint8_t>(frame).last(4UZ)) == WriteStatus::ok);

        std::vector<Record>       outBuf(4UZ);
        std::vector<Record>       shBuf(4UZ);
        std::vector<Record>       ocfBuf(4UZ);
        const std::vector<Record> in{recordOf(frame)};
        InputSpan<Record>         inSpan{std::span<const Record>(in)};
        OutputSpan<Record>        outSpan{std::span<Record>(outBuf)};
        OutputSpan<Record>        shSpan{std::span<Record>(shBuf)};
        OutputSpan<Record>        ocfSpan{std::span<Record>(ocfBuf)};
        std::ignore = block.processBulk(inSpan, outSpan, shSpan, ocfSpan);

        expect(eq(outSpan.count, std::size_t{1}));
        expect(eq(shSpan.count, std::size_t{1}));
        expect(eq(ocfSpan.count, std::size_t{1}));
        if (outSpan.count == 1UZ) {
            expect(eq(outBuf[0].signal_values.size(), std::size_t{10}));
        }
        if (shSpan.count == 1UZ) {
            expect(eq(shBuf[0].signal_values.size(), std::size_t{19})) << "20 - 1 octets of secondary header data";
        }
        if (ocfSpan.count == 1UZ) {
            expect(eq(metaSize(ocfBuf[0], "ccsds_ocf_type"), gr::Size_t{0}));
            expect(metaHas(ocfBuf[0], "ccsds_clcw_version"));
            expect(eq(metaString(ocfBuf[0], "protocol"), std::string("ccsds/clcw")));
        }
    };

    "both optional ports unconnected, and the Type-2 reports"_test = [] {
        constexpr gr::Size_t kFrameLength = 6U + 20U + 10U + 4U;

        const auto frameWithOcf = [](std::span<const std::uint8_t> ocfOctets) {
            std::vector<std::uint8_t> frame(kFrameLength, 0U);
            const TmPrimaryHeader     header{.version = 0, .spacecraft_id = 1, .virtual_channel = 0, .ocf_present = true, .master_frame_count = 0, .vc_frame_count = 0, .secondary_header = true, .sync_flag = false, .packet_order = false, .segment_length_id = 3, .first_header_pointer = 0};
            std::ignore = writeTmPrimaryHeader(header, frame);
            std::ignore = writeTmSecondaryHeaderId(TmSecondaryHeaderId{.version = 0, .length = 19}, std::span<std::uint8_t>(frame).subspan(kTmPrimaryHeaderSize));
            std::ranges::copy(ocfOctets, frame.end() - static_cast<std::ptrdiff_t>(kOcfSize));
            return frame;
        };

        // an unconnected optional port publishes nothing and costs nothing, and the counters still say what was there
        {
            auto                            block = make<TmFrameDecode>({{"frame_length", kFrameLength}});
            const std::vector<std::uint8_t> clcw{0x00U, 0x00U, 0x00U, 0x00U};
            const TmPorts                   ports = driveTm(block, std::vector<Record>{recordOf(frameWithOcf(clcw))}, false, false);
            expect(eq(ports.out.size(), 1UZ));
            expect(eq(ports.sh.size(), 0UZ));
            expect(eq(ports.ocf.size(), 0UZ));
            expect(eq(block.nSecondaryHeaders, std::uint64_t{1})) << "counted although no port carried it";
            expect(eq(block.nOcfRecords, std::uint64_t{1}));
            expect(eq(block.nFrames, std::uint64_t{1}));
        }

        // bit 0 set makes the field a Type-2 report and bit 1 then says whether it is the SDLS one, 4.1.5.4 and 4.1.5.5
        {
            auto                            block = make<TmFrameDecode>({{"frame_length", kFrameLength}});
            const std::vector<std::uint8_t> sdls{0xC0U, 0x11U, 0x22U, 0x33U};
            const TmPorts                   ports = driveTm(block, std::vector<Record>{recordOf(frameWithOcf(sdls))});
            expect(eq(ports.ocf.size(), 1UZ));
            if (ports.ocf.size() == 1UZ) {
                expect(eq(metaSize(ports.ocf[0], "ccsds_ocf_type"), gr::Size_t{1}));
                expect(eq(metaString(ports.ocf[0], "protocol"), std::string("ccsds/ocf"))) << "a Type-2 report is not a control link word and is not labeled one";
                expect(metaHas(ports.ocf[0], "ccsds_ocf_sdls"));
                expect(metaBool(ports.ocf[0], "ccsds_ocf_sdls"));
                expect(!metaHas(ports.ocf[0], "ccsds_clcw_version")) << "none of the eleven CLCW keys is written for a Type-2 report";
                expect(!metaHas(ports.ocf[0], "ccsds_clcw_report_value"));
                expect(that % (ports.ocf[0].signal_values == sdls)) << "the four octets cross whole, the report's contents being the SDLS protocol's";
            }
        }
        {
            auto                            block = make<TmFrameDecode>({{"frame_length", kFrameLength}});
            const std::vector<std::uint8_t> project{0x80U, 0x11U, 0x22U, 0x33U};
            const TmPorts                   ports = driveTm(block, std::vector<Record>{recordOf(frameWithOcf(project))});
            expect(eq(ports.ocf.size(), 1UZ));
            if (ports.ocf.size() == 1UZ) {
                expect(eq(metaSize(ports.ocf[0], "ccsds_ocf_type"), gr::Size_t{1}));
                expect(metaHas(ports.ocf[0], "ccsds_ocf_sdls")) << "the bool is written for every Type-2 report, true or false";
                expect(!metaBool(ports.ocf[0], "ccsds_ocf_sdls"));
            }
        }
    };

    "a full OCF port holds the whole frame back"_test = [] {
        {
            auto            block  = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});
            TmPrimaryHeader first  = tmHeader(0U, 0U, 0U);
            first.ocf_present      = true;
            TmPrimaryHeader second = tmHeader(0U, 1U, 1U);
            second.ocf_present     = true;
            const std::vector<Record> records{recordOf(tmFrame(first, 20UZ)), recordOf(tmFrame(second, 20UZ))};

            std::vector<Record> outBuf(4UZ), shBuf(4UZ), ocfBuf(1UZ); // room on the OCF port for one record only
            InputSpan<Record>   inSpan{std::span<const Record>(records)};
            OutputSpan<Record>  outSpan{std::span<Record>(outBuf)};
            OutputSpan<Record>  shSpan{std::span<Record>(shBuf), 0UZ, nullptr, false};
            OutputSpan<Record>  ocfSpan{std::span<Record>(ocfBuf)};
            std::ignore = block.processBulk(inSpan, outSpan, shSpan, ocfSpan);

            expect(eq(outSpan.count, std::size_t{1})) << "the second frame publishes nothing at all, not even its data field";
            expect(eq(ocfSpan.count, std::size_t{1}));
            expect(eq(inSpan.consumed, std::size_t{1})) << "the held-back frame stays unconsumed";
            expect(eq(block.nFrames, std::uint64_t{1}));
            expect(eq(block.nOcfRecords, std::uint64_t{1}));

            // the same frame presented again, with room this time
            const TmPorts ports = driveTm(block, std::span<const Record>(records).subspan(1UZ));
            expect(eq(ports.out.size(), 1UZ));
            expect(eq(ports.ocf.size(), 1UZ));
            expect(eq(block.nFrames, std::uint64_t{2}));
            expect(eq(block.nDuplicateFrames, std::uint64_t{0})) << "the frame enters the gap state once, on the call that publishes it";
        }
        {
            auto       block    = make<AosFrameDecode>({{"frame_length", gr::Size_t{30}}, {"data_unit", std::string("m_pdu")}, {"has_ocf", true}});
            const auto aosFrame = [](std::uint32_t count) {
                std::vector<std::uint8_t> frame(30UZ, 0U);
                const AosPrimaryHeader    header{.version = 1, .spacecraft_id = 7, .virtual_channel = 3, .vc_frame_count = count, .replay = false, .vc_count_cycle_used = false, .reserved = 0, .vc_count_cycle = 0, .has_fhec = false, .frame_header_error_control = 0};
                std::ignore = writeAosPrimaryHeader(header, frame);
                return frame;
            };
            const std::vector<Record> records{recordOf(aosFrame(0U)), recordOf(aosFrame(1U))};

            std::vector<Record> outBuf(4UZ), ocfBuf(1UZ);
            InputSpan<Record>   inSpan{std::span<const Record>(records)};
            OutputSpan<Record>  outSpan{std::span<Record>(outBuf)};
            OutputSpan<Record>  ocfSpan{std::span<Record>(ocfBuf)};
            std::ignore = block.processBulk(inSpan, outSpan, ocfSpan);

            expect(eq(outSpan.count, std::size_t{1}));
            expect(eq(inSpan.consumed, std::size_t{1}));
            expect(eq(block.nFrames, std::uint64_t{1}));

            std::vector<Record> outBuf2(4UZ), ocfBuf2(4UZ);
            InputSpan<Record>   inSpan2{std::span<const Record>(records).subspan(1UZ)};
            OutputSpan<Record>  outSpan2{std::span<Record>(outBuf2)};
            OutputSpan<Record>  ocfSpan2{std::span<Record>(ocfBuf2)};
            std::ignore = block.processBulk(inSpan2, outSpan2, ocfSpan2);

            expect(eq(outSpan2.count, std::size_t{1}));
            expect(eq(block.nFrames, std::uint64_t{2}));
            expect(eq(block.nDuplicateFrames, std::uint64_t{0}));
        }
    };

    // ---- criterion 8 at the block: the gap values on the record, the counters, and the report cap ----

    "frame count gaps at the block, and the report cap"_test = [] {
        {
            auto block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});
            // counts 0, 1, 5, 5 on one channel: three frames lost across the jump, then a repeat
            const std::vector<Record> records{recordOf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ)), recordOf(tmFrame(tmHeader(0U, 1U, 1U), 20UZ)), //
                recordOf(tmFrame(tmHeader(0U, 5U, 5U), 20UZ)), recordOf(tmFrame(tmHeader(0U, 6U, 5U), 20UZ))};
            const TmPorts             ports = driveTm(block, records, false, false);

            expect(eq(ports.out.size(), 4UZ));
            expect(!metaHas(ports.out[1], "ccsds_frames_lost")) << "a continuous record carries no gap key";
            expect(eq(metaSize(ports.out[2], "ccsds_frames_lost"), gr::Size_t{3}));
            expect(eq(metaSize(ports.out[2], "ccsds_mc_frames_lost"), gr::Size_t{3}));
            expect(metaString(ports.out[2], "discontinuity").contains("frame_gap"));
            expect(!metaHas(ports.out[3], "ccsds_frames_lost")) << "a repeated count is a duplicate, not a gap";
            expect(eq(block.nFramesLost, std::uint64_t{3}));
            expect(eq(block.nMcFramesLost, std::uint64_t{3}));
            expect(eq(block.nDuplicateFrames, std::uint64_t{1}));
        }
        {
            // the virtual channel's count is continuous while the master channel's jumps: frames of another
            // virtual channel went missing, which is what the two counters are separate to say
            auto                      block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});
            const std::vector<Record> records{recordOf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ)), recordOf(tmFrame(tmHeader(0U, 3U, 1U), 20UZ))};
            const TmPorts             ports = driveTm(block, records, false, false);

            expect(eq(ports.out.size(), 2UZ));
            expect(eq(metaSize(ports.out[1], "ccsds_mc_frames_lost"), gr::Size_t{2}));
            expect(eq(metaSize(ports.out[1], "ccsds_frames_lost"), gr::Size_t{0}));
            expect(eq(block.nMcFramesLost, std::uint64_t{2}));
            expect(eq(block.nFramesLost, std::uint64_t{0}));
        }
        {
            // max_frames_lost_report caps the total, not the report: the record still carries the gap it saw
            auto                      block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}, {"max_frames_lost_report", gr::Size_t{2}}});
            const std::vector<Record> records{recordOf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ)), recordOf(tmFrame(tmHeader(0U, 5U, 5U), 20UZ))};
            const TmPorts             ports = driveTm(block, records, false, false);

            expect(eq(ports.out.size(), 2UZ));
            expect(eq(metaSize(ports.out[1], "ccsds_frames_lost"), gr::Size_t{4}));
            expect(metaString(ports.out[1], "discontinuity").contains("frame_gap"));
            expect(eq(block.nFramesLost, std::uint64_t{0})) << "a gap of four past a cap of two is reported and left out of the total";
            expect(eq(block.nMcFramesLost, std::uint64_t{0}));
        }
    };

    "two virtual channels through one decoder keep their own frame counts"_test = [] {
        {
            auto block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});
            // interleaved channels, each continuous in its own count, the master count running across both
            const std::vector<Record> records{recordOf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ)), recordOf(tmFrame(tmHeader(1U, 1U, 100U), 20UZ)), //
                recordOf(tmFrame(tmHeader(0U, 2U, 1U), 20UZ)), recordOf(tmFrame(tmHeader(1U, 3U, 101U), 20UZ))};
            const TmPorts             ports = driveTm(block, records, false, false);

            expect(eq(ports.out.size(), 4UZ));
            expect(eq(block.nFramesLost, std::uint64_t{0})) << "one register per channel, so channel 1's count is no gap in channel 0's";
            expect(eq(block.nDuplicateFrames, std::uint64_t{0}));
            expect(eq(block.nMcFramesLost, std::uint64_t{0}));
            for (const Record& record : ports.out) {
                expect(!metaHas(record, "ccsds_frames_lost"));
            }
        }
        {
            // the same interleave with channel 0 losing three frames: the register that sees the gap is that channel's
            auto                      block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}});
            const std::vector<Record> records{recordOf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ)), recordOf(tmFrame(tmHeader(1U, 1U, 100U), 20UZ)), //
                recordOf(tmFrame(tmHeader(0U, 2U, 4U), 20UZ)), recordOf(tmFrame(tmHeader(1U, 3U, 101U), 20UZ))};
            const TmPorts             ports = driveTm(block, records, false, false);

            expect(eq(ports.out.size(), 4UZ));
            expect(eq(block.nFramesLost, std::uint64_t{3}));
            expect(eq(metaSize(ports.out[2], "ccsds_frames_lost"), gr::Size_t{3}));
            expect(!metaHas(ports.out[3], "ccsds_frames_lost")) << "channel 1 is undisturbed by channel 0's gap";
        }
    };

    "a flipped octet lands on CrcCheck's fail port and never reaches the decode"_test = [] {
        const gr::digital::Crc crc(16U, 0x1021ULL, 0xFFFFULL, 0x0000ULL, false, false);
        const auto             withFecf = [&crc](std::vector<std::uint8_t> body) {
            const std::uint64_t fecf = crc.compute(body);
            body.push_back(static_cast<std::uint8_t>((fecf >> 8U) & 0xFFU));
            body.push_back(static_cast<std::uint8_t>(fecf & 0xFFU));
            return body;
        };

        const std::vector<std::uint8_t> intact  = withFecf(tmFrame(tmHeader(0U, 0U, 0U), 20UZ));
        std::vector<std::uint8_t>       flipped = intact;
        flipped[7] ^= 0x01U; // one bit of the data field, so the header still parses and only the check can tell

        gr::property_map checking = fecfSettings();
        checking["discard_crc"]   = true;
        auto check                = make<gr::blocks::digital::CrcCheck>(checking);

        const std::vector<Record> records{recordOf(intact), recordOf(flipped)};
        std::vector<Record>       okBuf(4UZ), failBuf(4UZ);
        InputSpan<Record>         inSpan{std::span<const Record>(records)};
        OutputSpan<Record>        okSpan{std::span<Record>(okBuf)};
        OutputSpan<Record>        failSpan{std::span<Record>(failBuf)};
        std::ignore = check.processBulk(inSpan, okSpan, failSpan);

        expect(eq(okSpan.count, std::size_t{1}));
        expect(eq(failSpan.count, std::size_t{1})) << "the flipped octet is routed, never dropped";
        expect(metaBool(okBuf[0], "crc_ok"));
        expect(!metaBool(failBuf[0], "crc_ok"));
        expect(eq(okBuf[0].signal_values.size(), std::size_t{20})) << "discard_crc strips the field before the decode sees the frame";

        // the decode behind the ok port gates on what CrcCheck wrote, so a graph that wires the fail port to it anyway
        // still drops the frame under a named counter
        auto          block = make<TmFrameDecode>({{"frame_length", gr::Size_t{20}}, {"require_crc_ok", true}});
        const TmPorts ports = driveTm(block, std::vector<Record>{okBuf[0], failBuf[0]}, false, false);
        expect(eq(ports.out.size(), 1UZ));
        expect(eq(block.nCrcFailed, std::uint64_t{1}));
    };

    // ---- the encoders: their counts, their fill, and what they refuse ----

    "TmFrameEncode wraps both counts at 256"_test = [] {
        auto block = make<TmFrameEncode>({{"frame_length", gr::Size_t{20}}, {"spacecraft_id", gr::Size_t{42}}, {"virtual_channel", gr::Size_t{1}}});

        std::vector<Record> in;
        for (std::size_t i = 0UZ; i < 258UZ; ++i) {
            in.push_back(recordOf(std::vector<std::uint8_t>(14UZ, 0xA5U))); // 20 - 6 octets: the data field exactly
        }
        const std::vector<Record> frames = drive1(block, in, 300UZ);
        expect(eq(frames.size(), 258UZ));

        const auto headerAt = [&frames](std::size_t index) {
            TmPrimaryHeader header{};
            std::ignore = parseTmPrimaryHeader(frames[index].signal_values, header);
            return header;
        };
        expect(eq(headerAt(0UZ).vc_frame_count, std::uint8_t{0}));
        expect(eq(headerAt(255UZ).vc_frame_count, std::uint8_t{255}));
        expect(eq(headerAt(256UZ).vc_frame_count, std::uint8_t{0})) << "the virtual channel count wraps modulo 256, 4.1.2.6.2";
        expect(eq(headerAt(257UZ).vc_frame_count, std::uint8_t{1}));
        expect(eq(headerAt(0UZ).master_frame_count, std::uint8_t{0}));
        expect(eq(headerAt(255UZ).master_frame_count, std::uint8_t{255}));
        expect(eq(headerAt(256UZ).master_frame_count, std::uint8_t{0})) << "and the master channel count with it, 4.1.2.5.2";
        expect(eq(headerAt(257UZ).master_frame_count, std::uint8_t{1}));
    };

    "AosFrameEncode carries the cycle when the 24-bit count wraps"_test = [] {
        auto block = make<AosFrameEncode>({{"frame_length", gr::Size_t{30}}, {"spacecraft_id", gr::Size_t{7}}, {"virtual_channel", gr::Size_t{3}}, {"data_unit", std::string("m_pdu")}, {"vcfc_cycle_use", true}});
        // 2^24 frames is more than a unit test can emit, so the count is seeded one short of the wrap and the two
        // frames either side of it are the ones asserted
        block._vcCount = kAosCountModulus - 1U;

        const std::vector<Record> frames = drive1(block, std::vector<Record>{recordOf(std::vector<std::uint8_t>(22UZ, 0U)), recordOf(std::vector<std::uint8_t>(22UZ, 0U))});
        expect(eq(frames.size(), 2UZ));

        AosPrimaryHeader before{}, after{};
        expect(parseAosPrimaryHeader(frames[0].signal_values, false, before) == ParseStatus::ok);
        expect(parseAosPrimaryHeader(frames[1].signal_values, false, after) == ParseStatus::ok);
        expect(eq(before.vc_frame_count, std::uint32_t{0xFFFFFF}));
        expect(eq(before.vc_count_cycle, std::uint8_t{0}));
        expect(eq(after.vc_frame_count, std::uint32_t{0}));
        expect(eq(after.vc_count_cycle, std::uint8_t{1})) << "the cycle advances on the wrap, 4.1.2.5.5.2";
        expect(frameGap(aosWidenedFrameCount(after), aosWidenedFrameCount(before), aosCountModulus(after)).continuous) << "so the widened counts stay continuous across it";
    };

    "a short data field is padded with the OID fill, which runs on across frames"_test = [] {
        auto block = make<TmFrameEncode>({{"frame_length", gr::Size_t{20}}, {"spacecraft_id", gr::Size_t{42}}, {"virtual_channel", gr::Size_t{1}}});

        const std::vector<Record> in{recordOf(std::vector<std::uint8_t>(4UZ, 0x11U), {{"ccsds_first_header_pointer", gr::Size_t{3}}}), recordOf(std::vector<std::uint8_t>(4UZ, 0x11U))};
        const std::vector<Record> frames = drive1(block, in);
        expect(eq(frames.size(), 2UZ));

        // 6 octets of header and 4 of payload leave ten of fill, which is the standard's own published anchor for the
        // sequence's first ten octets (132.0-B-3 NOTE to 4.1.4.6.2.2)
        const std::vector<std::uint8_t> firstPad(frames[0].signal_values.begin() + 10, frames[0].signal_values.end());
        const std::vector<std::uint8_t> secondPad(frames[1].signal_values.begin() + 10, frames[1].signal_values.end());
        expect(that % (firstPad == std::vector<std::uint8_t>{0xFF, 0xFF, 0xFF, 0xFF, 0x6D, 0xB6, 0xD8, 0x61, 0x45, 0x1F}));
        expect(that % (secondPad != firstPad)) << "the generator carries across frames rather than restarting, 4.1.4.6.2.1";

        TmPrimaryHeader header{};
        expect(parseTmPrimaryHeader(frames[0].signal_values, header) == ParseStatus::ok);
        expect(eq(header.first_header_pointer, std::uint16_t{3})) << "the pointer is written as the metadata gave it, the fill notwithstanding";
        expect(parseTmPrimaryHeader(frames[1].signal_values, header) == ParseStatus::ok);
        expect(eq(header.first_header_pointer, kFhpNoPacketStart)) << "a record with no pointer key claims nothing about its zone";
    };

    "the encoders refuse what they cannot write"_test = [] {
        expect(throws([] { std::ignore = make<TmFrameEncode>({{"frame_length", gr::Size_t{20}}, {"spacecraft_id", gr::Size_t{1024}}, {"virtual_channel", gr::Size_t{1}}}); })) << "a TM spacecraft identifier past ten bits";
        expect(throws([] { std::ignore = make<AosFrameEncode>({{"frame_length", gr::Size_t{30}}, {"spacecraft_id", gr::Size_t{256}}, {"virtual_channel", gr::Size_t{1}}, {"data_unit", std::string("m_pdu")}}); })) << "an AOS spacecraft identifier past eight bits";
        expect(nothrow([] { std::ignore = make<AosFrameEncode>({{"frame_length", gr::Size_t{30}}, {"spacecraft_id", gr::Size_t{255}}, {"virtual_channel", gr::Size_t{1}}, {"data_unit", std::string("m_pdu")}}); })) << "255 is the widest an eight-bit field holds";

        auto                      tm     = make<TmFrameEncode>({{"frame_length", gr::Size_t{20}}, {"spacecraft_id", gr::Size_t{42}}, {"virtual_channel", gr::Size_t{1}}});
        const std::vector<Record> frames = drive1(tm, std::vector<Record>{recordOf(std::vector<std::uint8_t>(14UZ, 0U), {{"ccsds_first_header_pointer", gr::Size_t{5000}}})});
        expect(eq(frames.size(), 1UZ)) << "a pointer key the eleven-bit field cannot hold reads as absent, and the frame is still built";
        expect(eq(tm.nBadPointerKey, std::uint64_t{1}));
        expect(eq(tm.nRefusedHeader, std::uint64_t{0}));
        if (frames.size() == 1UZ) {
            TmPrimaryHeader header{};
            expect(parseTmPrimaryHeader(frames[0].signal_values, header) == ParseStatus::ok);
            expect(eq(header.first_header_pointer, kFhpNoPacketStart));
        }

        std::ignore = drive1(tm, std::vector<Record>{recordOf(std::vector<std::uint8_t>(15UZ, 0U))});
        expect(eq(tm.nRefusedOversize, std::uint64_t{1})) << "one octet more than the data field holds";

        auto                      tc    = make<TcFrameEncode>({{"spacecraft_id", gr::Size_t{42}}, {"virtual_channel", gr::Size_t{1}}});
        const std::vector<Record> tcOut = drive1(tc, std::vector<Record>{recordOf(std::vector<std::uint8_t>{}), recordOf(std::vector<std::uint8_t>(3UZ, 0x7EU))});
        expect(eq(tcOut.size(), 1UZ));
        expect(eq(tc.nRefusedEmpty, std::uint64_t{1})) << "4.1.1.1 b) makes the data field mandatory, so an empty payload has no encoding";
        if (tcOut.size() == 1UZ) {
            expect(eq(tcOut[0].signal_values.size(), std::size_t{8}));
        }
    };

    // ---- PacketExtractor kernel: idle, the lost-frame recovery, gaps, split headers, bad pointers, bounds ----

    "idle frames and idle packets are counted and silent"_test = [] {
        PacketExtractor     extractor;
        std::vector<Record> emitted;
        const auto          emit = [&emitted](std::span<const std::uint8_t> packet) { emitted.push_back(recordOf(std::vector<std::uint8_t>(packet.begin(), packet.end()))); };

        std::vector<std::uint8_t> oidZone(223UZ, 0U);
        OidFill                   fill;
        fill.next(oidZone);
        for (std::uint32_t i = 0U; i < 100U; ++i) {
            extractor.feed(oidZone, kFhpOnlyIdleData, i, emit);
        }
        expect(eq(extractor.counters().idle_frames, std::uint64_t{100}));
        expect(eq(emitted.size(), 0UZ));

        extractor.reset();
        emitted.clear();
        std::vector<std::uint8_t> zone;
        SpacePacketHeader         real{};
        for (int i = 0; i < 3; ++i) {
            expect(headerForPayload(10U, false, false, 3U, static_cast<std::uint16_t>(i), 4UZ, real) == WriteStatus::ok);
            std::vector<std::uint8_t> packet(10UZ, 0U);
            std::ignore = writeSpacePacketHeader(real, packet);
            zone.insert(zone.end(), packet.begin(), packet.end());
        }
        SpacePacketHeader idle{};
        for (int i = 0; i < 2; ++i) {
            expect(headerForPayload(kIdleApid, false, false, 3U, 0U, 4UZ, idle) == WriteStatus::ok);
            std::vector<std::uint8_t> packet(10UZ, 0U);
            std::ignore = writeSpacePacketHeader(idle, packet);
            zone.insert(zone.end(), packet.begin(), packet.end());
        }
        extractor.feed(zone, 0U, 0U, emit);
        expect(eq(emitted.size(), 3UZ));
        expect(eq(extractor.counters().idle_packets, std::uint64_t{2}));
    };

    "the lost frame between two spanning fragments"_test = [] {
        // build a packet spanning three 50-octet zones, and a second packet immediately behind it
        SpacePacketHeader big{};
        expect(headerForPayload(11U, false, false, 3U, 0U, 130UZ, big) == WriteStatus::ok); // 136 octets total, spans 3 zones of 50
        std::vector<std::uint8_t> firstPacket(6UZ, 0U);
        std::ignore = writeSpacePacketHeader(big, firstPacket);
        firstPacket.resize(136UZ);
        for (std::size_t i = 6UZ; i < firstPacket.size(); ++i) {
            firstPacket[i] = static_cast<std::uint8_t>(i);
        }
        SpacePacketHeader small{};
        expect(headerForPayload(12U, false, false, 3U, 0U, 20UZ, small) == WriteStatus::ok);
        std::vector<std::uint8_t> secondPacket(26UZ, 0U);
        std::ignore = writeSpacePacketHeader(small, secondPacket);
        for (std::size_t i = 6UZ; i < secondPacket.size(); ++i) {
            secondPacket[i] = static_cast<std::uint8_t>(0x80U + i);
        }

        std::vector<std::uint8_t> stream = firstPacket;
        stream.insert(stream.end(), secondPacket.begin(), secondPacket.end());
        // stream is 162 octets (136 + 26); zones: zone0 [0,50) fhp 0 (firstPacket starts here); zone1 [50,100)
        // is a pure continuation of firstPacket and is the one dropped below; zone2 [100,150) holds firstPacket's
        // last 36 octets [100,136) followed by secondPacket's first 14 octets [136,150), so its fhp is 36, the
        // local offset where secondPacket starts; zone3 [150,162) is secondPacket's remaining 12 octets, a pure
        // continuation, sized exactly (no padding, so no zero-filled octets can misparse as further packets).
        std::vector<std::uint8_t> zone0(stream.begin(), stream.begin() + 50);
        std::vector<std::uint8_t> zone2(stream.begin() + 100, stream.begin() + 150);
        std::vector<std::uint8_t> zone3(stream.begin() + 150, stream.end());
        constexpr std::uint16_t   kZone2Fhp = 36U; // 136 (firstPacket's end) - 100 (zone2's start)

        {
            PacketExtractor     extractor;
            std::vector<Record> emitted;
            const auto          emit = [&emitted](std::span<const std::uint8_t> packet) { emitted.push_back(recordOf(std::vector<std::uint8_t>(packet.begin(), packet.end()))); };
            extractor.feed(zone0, 0U, 0U, emit);
            // zone1 dropped, its frame count with it
            extractor.feed(zone2, kZone2Fhp, 2U, emit);
            extractor.feed(zone3, kFhpNoPacketStart, 3U, emit);

            expect(eq(extractor.counters().frames_lost, std::uint64_t{1}));
            expect(eq(extractor.counters().fragments_dropped, std::uint64_t{1}));
            expect(eq(emitted.size(), 1UZ)) << "the first packet is not recovered, the second is";
            if (emitted.size() == 1UZ) {
                expect(that % (emitted[0].signal_values == secondPacket));
            }
        }
        {
            // the count not dropped: zone1's slot filled by a differently-sized span of garbage, so the count is
            // continuous but the residue length disagrees with the pointer once zone2's 36 octets are appended.
            PacketExtractor           extractor;
            std::vector<Record>       emitted;
            const auto                emit = [&emitted](std::span<const std::uint8_t> packet) { emitted.push_back(recordOf(std::vector<std::uint8_t>(packet.begin(), packet.end()))); };
            std::vector<std::uint8_t> garbage(30UZ, 0xEEU);
            extractor.feed(zone0, 0U, 0U, emit);
            extractor.feed(garbage, kFhpNoPacketStart, 1U, emit);
            extractor.feed(zone2, kZone2Fhp, 2U, emit);
            extractor.feed(zone3, kFhpNoPacketStart, 3U, emit);
            expect(eq(extractor.counters().pointer_mismatch, std::uint64_t{1}));
            expect(eq(emitted.size(), 1UZ));
            if (emitted.size() == 1UZ) {
                expect(that % (emitted[0].signal_values == secondPacket));
            }
        }
    };

    "split headers reassemble exactly"_test = [] {
        SpacePacketHeader header{};
        expect(headerForPayload(5U, false, false, 3U, 0U, 20UZ, header) == WriteStatus::ok);
        std::vector<std::uint8_t> packet(26UZ, 0U);
        std::ignore = writeSpacePacketHeader(header, packet);
        for (std::size_t i = 6UZ; i < packet.size(); ++i) {
            packet[i] = static_cast<std::uint8_t>(i);
        }
        for (std::size_t split = 1UZ; split <= 5UZ; ++split) {
            PacketExtractor           extractor;
            std::vector<Record>       emitted;
            const auto                emit = [&emitted](std::span<const std::uint8_t> p) { emitted.push_back(recordOf(std::vector<std::uint8_t>(p.begin(), p.end()))); };
            std::vector<std::uint8_t> zone0(packet.begin(), packet.begin() + static_cast<std::ptrdiff_t>(split));
            std::vector<std::uint8_t> zone1(packet.begin() + static_cast<std::ptrdiff_t>(split), packet.end());
            extractor.feed(zone0, 0U, 0U, emit);
            expect(le(extractor.fragment().size(), std::size_t{5})) << "never more than five octets before the header completes";
            extractor.feed(zone1, kFhpNoPacketStart, 1U, emit);
            expect(eq(emitted.size(), 1UZ));
            if (emitted.size() == 1UZ) {
                expect(that % (emitted[0].signal_values == packet));
            }
        }
    };

    "bad pointers and orphans"_test = [] {
        std::vector<std::uint8_t> zone(30UZ, 0U);
        {
            PacketExtractor     extractor;
            std::vector<Record> emitted;
            const auto          emit = [&emitted](std::span<const std::uint8_t> p) { emitted.push_back(recordOf(std::vector<std::uint8_t>(p.begin(), p.end()))); };
            extractor.feed(zone, static_cast<std::uint16_t>(zone.size()), 0U, emit);
            expect(eq(extractor.counters().bad_pointer, std::uint64_t{1}));
            expect(eq(emitted.size(), 0UZ));
        }
        {
            PacketExtractor     extractor;
            std::vector<Record> emitted;
            const auto          emit = [&emitted](std::span<const std::uint8_t> p) { emitted.push_back(recordOf(std::vector<std::uint8_t>(p.begin(), p.end()))); };
            SpacePacketHeader   header{};
            expect(headerForPayload(9U, false, false, 3U, 0U, 4UZ, header) == WriteStatus::ok);
            std::vector<std::uint8_t> packet(10UZ, 0U);
            std::ignore = writeSpacePacketHeader(header, packet);
            std::vector<std::uint8_t> withOrphan;
            withOrphan.reserve(5UZ + packet.size());
            withOrphan.assign(5UZ, 0xAAU);
            withOrphan.insert(withOrphan.end(), packet.begin(), packet.end());
            extractor.feed(withOrphan, 5U, 0U, emit);
            expect(eq(extractor.counters().orphan_octets, std::uint64_t{5}));
            expect(eq(emitted.size(), 1UZ));
        }
    };

    "the reassembly bound is arithmetic"_test = [] {
        SpacePacketHeader header{};
        header.version        = 0;
        header.apid           = 13U;
        header.sequence_flags = 3U;
        header.data_length    = 0xFFFF;
        std::vector<std::uint8_t> zone(6UZ, 0U);
        std::ignore = writeSpacePacketHeader(header, zone);
        zone.resize(kMaxPacketOctets, 0U);

        {
            PacketExtractor     extractor;
            std::vector<Record> emitted;
            const auto          emit = [&emitted](std::span<const std::uint8_t> p) { emitted.push_back(recordOf(std::vector<std::uint8_t>(p.begin(), p.end()))); };
            extractor.feed(zone, 0U, 0U, emit);
            expect(eq(emitted.size(), 1UZ));
            if (emitted.size() == 1UZ) {
                expect(eq(emitted[0].signal_values.size(), kMaxPacketOctets));
            }
        }
        {
            PacketExtractor::Config config{.max_packet_length = 1024UZ, .count_modulus = kTmCountModulus};
            PacketExtractor         extractor(config);
            std::vector<Record>     emitted;
            const auto              emit = [&emitted](std::span<const std::uint8_t> p) { emitted.push_back(recordOf(std::vector<std::uint8_t>(p.begin(), p.end()))); };
            extractor.feed(zone, 0U, 0U, emit);
            expect(eq(extractor.counters().oversize_dropped, std::uint64_t{1}));
            expect(eq(emitted.size(), 0UZ));
        }
    };

    "segment then extract is the identity"_test = [] {
        // 2046 rather than 2048: eleven pointer bits less the two reserved values address positions 0 to 2045, so a
        // zone of 2048 octets has two positions no first header pointer can name.
        for (const gr::Size_t zoneLen : {gr::Size_t{223}, gr::Size_t{1115}, gr::Size_t{2046}}) {
            auto segment = make<SpacePacketSegment>({{"zone_length", zoneLen}});
            auto extract = make<SpacePacketExtract>({{"virtual_channel", gr::Size_t{0}}});

            std::vector<Record> packets;
            std::uint16_t       seq = 0U;
            for (const std::size_t length : {7UZ, 8UZ, 100UZ, 1000UZ}) {
                SpacePacketHeader header{};
                expect(headerForPayload(3U, false, false, 3U, seq++, length - 6UZ, header) == WriteStatus::ok);
                std::vector<std::uint8_t> packet(length, 0U);
                std::ignore = writeSpacePacketHeader(header, packet);
                for (std::size_t i = 6UZ; i < length; ++i) {
                    packet[i] = static_cast<std::uint8_t>((i * 7UZ + length) & 0xFFUZ);
                }
                packets.push_back(recordOf(packet));
            }
            // enough trailing filler so every real packet is forced through a whole zone without needing a flush
            for (int i = 0; i < 8; ++i) {
                std::vector<std::uint8_t> filler(1000UZ, static_cast<std::uint8_t>(i));
                SpacePacketHeader         header{};
                expect(headerForPayload(3U, false, false, 3U, seq++, filler.size() - 6UZ, header) == WriteStatus::ok);
                std::ignore = writeSpacePacketHeader(header, filler);
                packets.push_back(recordOf(filler));
            }

            const std::vector<Record> zones = drive1(segment, packets, 64UZ);
            expect(zones.size() > 0UZ);
            for (const Record& zone : zones) {
                expect(eq(zone.signal_values.size(), std::size_t{zoneLen}));
            }

            std::vector<Record> zonesWithMeta;
            for (const Record& zone : zones) {
                gr::property_map meta;
                const auto       it                = zone.meta_information[0].find(gr::property_map::key_type("ccsds_first_header_pointer"));
                meta["ccsds_first_header_pointer"] = it->second;
                meta["ccsds_vc_frame_count"]       = gr::Size_t{static_cast<gr::Size_t>(&zone - &zones[0])};
                meta["ccsds_vcid"]                 = gr::Size_t{0};
                zonesWithMeta.push_back(recordOf(zone.signal_values, meta));
            }
            const std::vector<Record> out = drive1(extract, zonesWithMeta, 64UZ);

            expect(ge(out.size(), 4UZ));
            for (std::size_t i = 0UZ; i < 4UZ && i < out.size(); ++i) {
                expect(that % (out[i].signal_values == packets[i].signal_values)) << "packet " << i << " round trips byte for byte";
            }
            expect(eq(extract.bad_pointer, std::uint64_t{0}));
            expect(eq(extract.pointer_mismatch, std::uint64_t{0}));
            expect(eq(extract.orphan_octets, std::uint64_t{0}));
        }
    };
};

int main() { /* not needed for UT */ }
