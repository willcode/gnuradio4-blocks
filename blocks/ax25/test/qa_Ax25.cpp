#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/ax25/Ax25.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>

#include "TestSpans.hpp"

/*
 * The frame these blocks build and read is a fixed byte layout, so the tests pin it where it is checkable by hand and
 * pin the composition where it has to hold end to end.
 *
 * The anchors come first: a twenty-one byte UI frame and a repeated hop, both written out as literal bytes rather than
 * computed, because bytes computed the way the block computes them would only agree with themselves. The control table
 * follows, one frame per named type. The refusals prove that a malformed record costs one record rather than the
 * stream, and the chain proves that what leaves an encoder over a bit-stuffed link with a frame check sequence in
 * front of it comes back the same frame.
 */
namespace {

using gr::blocks::ax25::Ax25AddressFilter;
using gr::blocks::ax25::Ax25Decode;
using gr::blocks::ax25::Ax25Encode;
using gr::blocks::ax25::test::InputSpan;
using gr::blocks::ax25::test::OutputSpan;
using gr::blocks::digital::CrcAppend;
using gr::blocks::digital::CrcCheck;
using gr::blocks::digital::DelimiterExtractor;
using gr::blocks::digital::DelimiterFramer;

using Record = gr::DataSet<std::uint8_t>;

/// Anchor A: destination APRS, source N0CALL, no repeaters, UI, PID 0xF0, command, poll/final clear, info ":TEST".
constexpr std::array<std::uint8_t, 21UZ> kAnchorA{{0x82U, 0xA0U, 0xA4U, 0xA6U, 0x40U, 0x40U, 0xE0U, //
    0x9CU, 0x60U, 0x86U, 0x82U, 0x98U, 0x98U, 0x61U,                                                //
    0x03U, 0xF0U,                                                                                   //
    0x3AU, 0x54U, 0x45U, 0x53U, 0x54U}};

/// Anchor B: the repeater subfield WIDE1-1 with its H bit set and the address field ending on it.
constexpr std::array<std::uint8_t, 7UZ> kAnchorB{{0xAEU, 0x92U, 0x88U, 0x8AU, 0x62U, 0x40U, 0xE3U}};

constexpr std::array<std::uint8_t, 5UZ> kInfo{{0x3AU, 0x54U, 0x45U, 0x53U, 0x54U}};

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// One record as the chain carries it: a flat byte array with its extent, its name and its own metadata.
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

[[nodiscard]] bool metaHas(const Record& record, std::string_view key) {
    const auto& map = metaOf(record);
    return map.find(gr::property_map::key_type(key)) != map.end();
}

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

[[nodiscard]] gr::Size_t metaSize(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? gr::Size_t{0xFFFFU} : entry->second.value_or(gr::Size_t{0xFFFFU});
}

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

/// What one direct drive of a block produced.
struct Driven {
    std::vector<Record> out{};
    std::size_t         consumed = 0UZ;
};

/// Drives @p block over @p records in windows of @p chunk records, with @p room records of output space.
template<typename TBlock>
[[nodiscard]] Driven feed(TBlock& block, std::span<const Record> records, std::size_t chunk = 0UZ, std::size_t room = 64UZ) {
    Driven              result;
    const std::size_t   stride = chunk == 0UZ ? std::max(records.size(), 1UZ) : chunk;
    std::vector<Record> scratch(room);

    for (std::size_t base = 0UZ; base < records.size();) {
        const std::size_t  count = std::min(stride, records.size() - base);
        InputSpan<Record>  inSpan(records.subspan(base, count), base);
        OutputSpan<Record> outSpan{std::span<Record>(scratch)};

        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.out.push_back(std::move(scratch[k]));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/// One frame decoded, with the block that decoded it left where its counters can be read.
[[nodiscard]] std::vector<Record> decoded(std::span<const Record> frames, Ax25Decode& block) { return feed(block, frames).out; }

/// What one direct drive of Ax25AddressFilter produced, its two output ports kept apart.
struct Filtered {
    std::vector<Record> ok{};
    std::vector<Record> failed{};
};

[[nodiscard]] Filtered runFilter(Ax25AddressFilter& filter, std::span<const Record> records) {
    std::vector<Record> okBuf(records.size() + 1UZ);
    std::vector<Record> failBuf(records.size() + 1UZ);
    InputSpan<Record>   inSpan(records, 0UZ);
    OutputSpan<Record>  okSpan{std::span<Record>(okBuf)};
    OutputSpan<Record>  failSpan{std::span<Record>(failBuf)};
    std::ignore = filter.processBulk(inSpan, okSpan, failSpan);

    Filtered result;
    result.ok.assign(okBuf.begin(), okBuf.begin() + static_cast<std::ptrdiff_t>(okSpan.count));
    result.failed.assign(failBuf.begin(), failBuf.begin() + static_cast<std::ptrdiff_t>(failSpan.count));
    return result;
}

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

/**
 * @brief Flattens each framed record onto a stream, keeping a copy of what it flattened.
 *
 * The copies are what lets one graph run be read from both ends. `_flipAt` inverts one wire item on the way past,
 * which is the corrupted frame of the chain test.
 */
struct RecordToStream : gr::Block<RecordToStream> {
    gr::PortIn<Record, gr::Async>        in;
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordToStream, in, out);

    std::vector<Record>       _seen{};
    std::vector<std::uint8_t> _pending{};
    std::size_t               _sent   = 0UZ;
    std::size_t               _index  = 0UZ;
    std::ptrdiff_t            _flipAt = -1;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        if (_sent == _pending.size() && inSpan.size() > 0UZ) {
            _seen.push_back(inSpan[0UZ]);
            _pending = inSpan[0UZ].signal_values;
            _sent    = 0UZ;
            consumed = 1UZ;
        }

        const std::size_t n = std::min(_pending.size() - _sent, outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            const std::uint8_t item = _pending[_sent + i];
            outSpan[i]              = (_flipAt >= 0 && static_cast<std::size_t>(_flipAt) == _index + i) ? static_cast<std::uint8_t>(item != 0U ? 0U : 1U) : item;
        }
        _sent += n;
        _index += n;

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(n);
        return consumed == 0UZ && n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
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

/**
 * @brief Runs a graph to completion under the simple scheduler, stopping it rather than hanging if it wedges.
 *
 * @p collect runs while the scheduler still owns the graph, because the references `emplaceBlock` returned point into
 * blocks the scheduler destroys with itself.
 */
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

/// The HDLC profile of the delimiter blocks: the flag, bit stuffing, and each payload byte unpacked least significant bit first.
[[nodiscard]] gr::property_map hdlcFraming() {
    return {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"stuff_after_ones", gr::Size_t{5}}, {"abort_ones", gr::Size_t{7}}, //
        {"bits_per_item", gr::Size_t{1}}, {"max_payload_items", gr::Size_t{8192}}, {"payload_pack_bits", gr::Size_t{8}}, {"payload_bit_order", std::string("lsb_first")}};
}

/// CRC-16/IBM-SDLC, the AX.25 frame check sequence, with its two bytes least significant first on the wire.
[[nodiscard]] gr::property_map fcs() {
    return {{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, {"final_xor", std::uint64_t{0xFFFF}}, //
        {"input_reflected", true}, {"result_reflected", true}, {"crc_byte_order", std::string("little")}};
}

/// What one run of the HDLC chain produced.
struct Chain {
    std::vector<Record> framed{};   ///< the wire frames the framer wrote
    std::vector<Record> received{}; ///< the records that reached the sink behind Ax25Decode
    std::vector<Record> failed{};   ///< the records that left CrcCheck by its fail port
};

/// `records` through Ax25Encode, CrcAppend, DelimiterFramer, DelimiterExtractor, CrcCheck and Ax25Decode in one graph.
[[nodiscard]] Chain hdlcChain(const gr::property_map& addressing, std::vector<Record> records, std::ptrdiff_t flipAt = -1) {
    gr::Graph flow;
    auto&     source           = flow.emplaceBlock<RecordSource>();
    source._records            = std::move(records);
    auto& encode               = flow.emplaceBlock<Ax25Encode>(addressing);
    auto& append               = flow.emplaceBlock<CrcAppend>(fcs());
    auto& framer               = flow.emplaceBlock<DelimiterFramer>(hdlcFraming());
    auto& wire                 = flow.emplaceBlock<RecordToStream>();
    wire._flipAt               = flipAt;
    auto&            extractor = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlcFraming());
    gr::property_map checking  = fcs();
    checking["discard_crc"]    = true;
    auto& check                = flow.emplaceBlock<CrcCheck>(checking);
    auto& decode               = flow.emplaceBlock<Ax25Decode>();
    auto& passed               = flow.emplaceBlock<RecordSink>();
    auto& corrupt              = flow.emplaceBlock<RecordSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, encode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(encode, append).has_value());
    boost::ut::expect(flow.connect<"out", "in">(append, framer).has_value());
    boost::ut::expect(flow.connect<"out", "in">(framer, wire).has_value());
    boost::ut::expect(flow.connect<"out", "in">(wire, extractor).has_value());
    boost::ut::expect(flow.connect<"out", "in">(extractor, check).has_value());
    boost::ut::expect(flow.connect<"ok", "in">(check, decode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(decode, passed).has_value());
    boost::ut::expect(flow.connect<"fail", "in">(check, corrupt).has_value());

    Chain result;
    runGraph(std::move(flow), [&result, &wire, &passed, &corrupt] {
        result.framed   = wire._seen;
        result.received = passed._records;
        result.failed   = corrupt._records;
    });
    return result;
}

} // namespace

const boost::ut::suite<"ax25"> ax25Tests = [] {
    using namespace boost::ut;

    "anchor A decodes to its stated facts and encodes back byte for byte"_test = [] {
        const std::vector<Record> frames{recordOf(std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()))};
        Ax25Decode                block;
        const std::vector<Record> back = decoded(std::span<const Record>(frames), block);

        expect(eq(back.size(), 1UZ));
        if (back.size() != 1UZ) {
            return;
        }
        expect(that % (back[0UZ].signal_values == std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))) << "the record's items are the information field";
        expect(eq(metaString(back[0UZ], "ax25_destination"), std::string("APRS")));
        expect(eq(metaString(back[0UZ], "ax25_source"), std::string("N0CALL")));
        expect(eq(metaString(back[0UZ], "ax25_via"), std::string("")));
        expect(eq(metaString(back[0UZ], "ax25_type"), std::string("UI")));
        expect(eq(metaSize(back[0UZ], "ax25_pid"), gr::Size_t{0xF0}));
        expect(!metaBool(back[0UZ], "ax25_poll_final"));
        expect(metaBool(back[0UZ], "ax25_command")) << "the C bits are 1 and 0, the version 2.2 command pair";
        expect(!metaHas(back[0UZ], "ax25_nr")) << "a UI frame numbers nothing";
        expect(!metaHas(back[0UZ], "ax25_ns"));
        expect(!metaHas(back[0UZ], "ax25_control")) << "the control byte is recorded only when the type has no name";
        expect(eq(block.nRecords, 1ULL));
        expect(eq(block.nInfoBytes, 5ULL));

        const gr::property_map    addressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}, {"control_type", std::string("UI")}, //
               {"pid", gr::Size_t{0xF0}}, {"command", true}, {"poll_final", false}};
        const std::vector<Record> payloads{recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))};
        Ax25Encode                encoder = make<Ax25Encode>(addressing);
        const Driven              built   = feed(encoder, std::span<const Record>(payloads));

        expect(eq(built.out.size(), 1UZ));
        if (built.out.size() == 1UZ) {
            expect(that % (built.out[0UZ].signal_values == std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()))) << "the twenty-one bytes, against the anchor";
            expect(eq(static_cast<std::size_t>(built.out[0UZ].extents.at(0)), 21UZ));
        }
        expect(eq(encoder.nRecords, 1ULL));
        expect(eq(encoder.nFrameBytes, 21ULL));
    };

    "anchor B round trips a repeated hop, H bit included"_test = [] {
        // anchor A with the source's extension bit cleared and the repeater subfield behind it
        std::vector<std::uint8_t> bytes(kAnchorA.begin(), kAnchorA.begin() + 14);
        bytes[13UZ] = 0x60U; // C = 0, reserved 11, SSID 0, extension clear because a repeater follows
        bytes.insert(bytes.end(), kAnchorB.begin(), kAnchorB.end());
        bytes.insert(bytes.end(), kAnchorA.begin() + 14, kAnchorA.end());

        const std::vector<Record> frames{recordOf(bytes)};
        Ax25Decode                block;
        const std::vector<Record> back = decoded(std::span<const Record>(frames), block);

        expect(eq(back.size(), 1UZ));
        if (back.size() == 1UZ) {
            expect(eq(metaString(back[0UZ], "ax25_via"), std::string("WIDE1-1*")));
            expect(eq(metaString(back[0UZ], "ax25_destination"), std::string("APRS")));
            expect(that % (back[0UZ].signal_values == std::vector<std::uint8_t>(kInfo.begin(), kInfo.end())));
        }

        const gr::property_map    addressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}, {"via", std::string("WIDE1-1*")}};
        const std::vector<Record> payloads{recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))};
        Ax25Encode                encoder = make<Ax25Encode>(addressing);
        const Driven              built   = feed(encoder, std::span<const Record>(payloads));
        expect(eq(built.out.size(), 1UZ));
        if (built.out.size() == 1UZ) {
            expect(that % (built.out[0UZ].signal_values == bytes)) << "the whole frame, the repeater subfield among it";
        }

        // the same hop without the marker leaves the H bit clear, which is what a transmitter of new frames sends
        Ax25Encode   plain = make<Ax25Encode>({{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}, {"via", std::string("WIDE1-1")}});
        const Driven quiet = feed(plain, std::span<const Record>(payloads));
        expect(eq(quiet.out.size(), 1UZ));
        if (quiet.out.size() == 1UZ) {
            expect(eq(static_cast<unsigned>(quiet.out[0UZ].signal_values.at(20UZ)), 0x63U)) << "H clear, reserved 11, SSID 1, extension set";
        }
    };

    "every named control value decodes to its type, its numbers and its PID presence"_test = [] {
        struct Case {
            std::uint8_t     control;
            std::string_view type;
            bool             pollFinal;
            bool             pid;
            bool             nr;
            bool             ns;
            gr::Size_t       nrValue;
            gr::Size_t       nsValue;
        };
        // an I frame with N(S) = 3, N(R) = 5 and the poll bit set, then one supervisory frame of each kind, then the
        // nine unnumbered modifiers with their poll/final bits clear
        const std::vector<Case> cases{{0xB6U, "I", true, true, true, true, 5U, 3U}, //
            {0x41U, "RR", false, false, true, false, 2U, 0U}, {0x05U, "RNR", false, false, true, false, 0U, 0U}, {0x09U, "REJ", false, false, true, false, 0U, 0U}, {0x0DU, "SREJ", false, false, true, false, 0U, 0U}, {0x03U, "UI", false, true, false, false, 0U, 0U}, {0x0FU, "DM", false, false, false, false, 0U, 0U}, {0x2FU, "SABM", false, false, false, false, 0U, 0U}, {0x43U, "DISC", false, false, false, false, 0U, 0U}, {0x63U, "UA", false, false, false, false, 0U, 0U}, {0x6FU, "SABME", false, false, false, false, 0U, 0U}, {0x87U, "FRMR", false, false, false, false, 0U, 0U}, {0xAFU, "XID", false, false, false, false, 0U, 0U}, {0xE3U, "TEST", false, false, false, false, 0U, 0U}};

        for (const Case& one : cases) {
            std::vector<std::uint8_t> bytes(kAnchorA.begin(), kAnchorA.begin() + 14);
            bytes.push_back(one.control);
            if (one.pid) {
                bytes.push_back(0xF0U);
            }
            bytes.insert(bytes.end(), kInfo.begin(), kInfo.end());

            const std::vector<Record> frames{recordOf(bytes)};
            Ax25Decode                block;
            const std::vector<Record> back = decoded(std::span<const Record>(frames), block);
            expect(eq(back.size(), 1UZ)) << one.type;
            if (back.size() != 1UZ) {
                continue;
            }
            expect(eq(metaString(back[0UZ], "ax25_type"), std::string(one.type)));
            expect(metaBool(back[0UZ], "ax25_poll_final") == one.pollFinal) << one.type;
            expect(metaHas(back[0UZ], "ax25_pid") == one.pid) << std::format("{}: the PID byte belongs to the I and UI frames alone", one.type);
            expect(metaHas(back[0UZ], "ax25_nr") == one.nr) << one.type;
            expect(metaHas(back[0UZ], "ax25_ns") == one.ns) << one.type;
            expect(!metaHas(back[0UZ], "ax25_control")) << std::format("{}: a named type does not carry the byte", one.type);
            if (one.nr) {
                expect(eq(metaSize(back[0UZ], "ax25_nr"), one.nrValue)) << one.type;
            }
            if (one.ns) {
                expect(eq(metaSize(back[0UZ], "ax25_ns"), one.nsValue)) << one.type;
            }
            expect(that % (back[0UZ].signal_values == std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))) << std::format("{}: what follows the header is the information field", one.type);
        }

        // an unlisted unnumbered modifier is still a frame: type "U" with the poll/final-masked byte recorded
        for (const std::uint8_t control : {std::uint8_t{0x23U}, std::uint8_t{0x33U}}) {
            std::vector<std::uint8_t> bytes(kAnchorA.begin(), kAnchorA.begin() + 14);
            bytes.push_back(control);
            bytes.insert(bytes.end(), kInfo.begin(), kInfo.end());

            const std::vector<Record> frames{recordOf(bytes)};
            Ax25Decode                block;
            const std::vector<Record> back = decoded(std::span<const Record>(frames), block);
            expect(eq(back.size(), 1UZ));
            if (back.size() == 1UZ) {
                expect(eq(metaString(back[0UZ], "ax25_type"), std::string("U")));
                expect(eq(metaSize(back[0UZ], "ax25_control"), gr::Size_t{0x23}));
                expect(metaBool(back[0UZ], "ax25_poll_final") == (control == 0x33U));
                expect(!metaHas(back[0UZ], "ax25_pid")) << "an unknown unnumbered frame is not told to have one";
            }
        }
    };

    "the structural refusals are counted, and the record after each of them decodes"_test = [] {
        // an address field ending at the record's end with no control byte behind it
        const std::vector<std::uint8_t> noControl(kAnchorA.begin(), kAnchorA.begin() + 14);
        // a UI frame ending at its control byte, with no protocol identifier behind it
        const std::vector<std::uint8_t> noPid(kAnchorA.begin(), kAnchorA.begin() + 15);
        // ten subfields whose extension bits are all clear, so the address field never closes
        std::vector<std::uint8_t> unclosed(70UZ, 0x40U);
        for (std::size_t i = 6UZ; i < unclosed.size(); i += 7UZ) {
            unclosed[i] = 0x60U;
        }
        const std::vector<std::uint8_t> good(kAnchorA.begin(), kAnchorA.end());

        const std::vector<Record> frames{recordOf(noControl), recordOf(good), recordOf(noPid), recordOf(unclosed), recordOf(good)};
        Ax25Decode                block;
        const Driven              run = feed(block, std::span<const Record>(frames));

        expect(eq(run.out.size(), 2UZ)) << "the two well-formed frames, and nothing published for the three refusals";
        expect(eq(block.nRefusedShort, 2ULL));
        expect(eq(block.nRefusedAddress, 1ULL));
        expect(eq(block.nRecords, 2ULL));
        expect(eq(run.consumed, frames.size())) << "a dropped record is consumed rather than left in the buffer";
        for (const Record& record : run.out) {
            expect(eq(metaString(record, "ax25_source"), std::string("N0CALL")));
        }
        expect(nothrow([&block] { block.stop(); })) << "the report is written once, at stop()";
    };

    "the address grammar is refused at staging by name, and a bad override costs one record"_test = [] {
        const auto staged = [](gr::property_map settings) { std::ignore = make<Ax25Encode>(std::move(settings)); };
        const auto with   = [](const char* key, std::string value) {
            gr::property_map settings{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}};
            settings[gr::property_map::key_type(key)] = std::move(value);
            return settings;
        };

        expect(nothrow([&staged, &with] { staged(with("via", std::string("WIDE1-1,WIDE2-2*"))); })) << "two hops, one of them already repeated";
        expect(throws([&staged, &with] { staged(with("source", std::string("n0call"))); })) << "a lowercase callsign";
        expect(throws([&staged, &with] { staged(with("destination", std::string("ABCDEFG"))); })) << "seven callsign characters";
        expect(throws([&staged, &with] { staged(with("source", std::string("N0CALL-16"))); })) << "an SSID above fifteen";
        expect(throws([&staged, &with] { staged(with("via", std::string("A1,A2,A3,A4,A5,A6,A7,A8,A9"))); })) << "a ninth hop";
        expect(throws([&staged, &with] { staged(with("control_type", std::string("I"))); })) << "the sequenced types need numbers no setting supplies";
        expect(throws([&staged, &with] { staged(with("control_type", std::string("RR"))); }));
        expect(throws([&staged, &with] { staged(with("control_type", std::string("U"))); })) << "the unnamed catch-all is a decode outcome, not a frame to build";
        expect(throws([] {
            Ax25Encode block = make<Ax25Encode>({{"source", std::string("N0CALL")}});
            block.start();
        })) << "the destination is required and has no default";

        // one record overrides the destination, the next names one the grammar refuses, the third falls back to the settings
        const std::vector<std::uint8_t> info(kInfo.begin(), kInfo.end());
        const std::vector<Record>       records{recordOf(info, {{"ax25_destination", std::string("CQ")}}), recordOf(info, {{"ax25_source", std::string("N0CALL-99")}}), recordOf(info)};
        Ax25Encode                      encoder = make<Ax25Encode>({{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}});
        const Driven                    built   = feed(encoder, std::span<const Record>(records));

        expect(eq(built.out.size(), 2UZ));
        expect(eq(encoder.nRefusedOverride, 1ULL));
        expect(eq(built.consumed, records.size()));
        if (built.out.size() == 2UZ) {
            Ax25Decode                readBack = Ax25Decode{};
            const std::vector<Record> back     = decoded(std::span<const Record>(built.out), readBack);
            expect(eq(back.size(), 2UZ));
            if (back.size() == 2UZ) {
                expect(eq(metaString(back[0UZ], "ax25_destination"), std::string("CQ"))) << "the override reaches the frame it rode on";
                expect(eq(metaString(back[1UZ], "ax25_destination"), std::string("APRS"))) << "and the record after the refused one is built from the settings";
            }
        }
    };

    "the HDLC chain returns every frame, and one flipped wire bit fails its check sequence"_test = [] {
        const gr::property_map addressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL-7")}, {"via", std::string("WIDE1-1,WIDE2-2")}};

        Rng                                    rng;
        std::vector<std::vector<std::uint8_t>> messages;
        for (const std::size_t length : {1UZ, 3UZ, 17UZ, 40UZ, 64UZ}) {
            std::vector<std::uint8_t> bytes(length);
            for (std::uint8_t& byte : bytes) {
                byte = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
            }
            messages.push_back(std::move(bytes));
        }

        // the records carry no keys of their own here: a byte stream is where a sender's metadata stops, so what the
        // far end can be held to is the frame's own facts and what the receiving stages wrote about it
        const auto build = [&messages] {
            std::vector<Record> records;
            for (const std::vector<std::uint8_t>& message : messages) {
                records.push_back(recordOf(message));
            }
            return records;
        };

        const Chain clean = hdlcChain(addressing, build());
        expect(eq(clean.received.size(), messages.size())) << "every frame survives the link";
        expect(eq(clean.failed.size(), 0UZ));
        for (std::size_t which = 0UZ; which < std::min(clean.received.size(), messages.size()); ++which) {
            expect(that % (clean.received[which].signal_values == messages[which])) << std::format("frame {}: the information field is byte identical", which);
            expect(eq(metaString(clean.received[which], "ax25_destination"), std::string("APRS"))) << std::format("frame {}", which);
            expect(eq(metaString(clean.received[which], "ax25_source"), std::string("N0CALL-7"))) << std::format("frame {}", which);
            expect(eq(metaString(clean.received[which], "ax25_via"), std::string("WIDE1-1,WIDE2-2"))) << std::format("frame {}", which);
            expect(eq(metaString(clean.received[which], "ax25_type"), std::string("UI")));
            expect(metaBool(clean.received[which], "ax25_command"));
            expect(metaBool(clean.received[which], "crc_ok")) << std::format("frame {}: what CrcCheck wrote about the frame crosses the decoder verbatim", which);
        }

        // one flipped wire bit, chosen inside the third frame and inside a run of ones short enough that no stuffing
        // decision either side of it changes: the frame keeps its length and fails only its check sequence
        expect(ge(clean.framed.size(), 3UZ));
        if (clean.framed.size() < 3UZ) {
            return;
        }
        const std::size_t                earlier = clean.framed[0UZ].signal_values.size() + clean.framed[1UZ].signal_values.size();
        const std::vector<std::uint8_t>& target  = clean.framed[2UZ].signal_values;
        std::ptrdiff_t                   flipAt  = -1;
        for (std::size_t i = 9UZ; i + 9UZ < target.size(); ++i) {
            if (target[i] == 0U) {
                continue;
            }
            std::size_t run = 1UZ;
            for (std::size_t back = i; back > 0UZ && target[back - 1UZ] != 0U; --back) {
                ++run;
            }
            for (std::size_t ahead = i + 1UZ; ahead < target.size() && target[ahead] != 0U; ++ahead) {
                ++run;
            }
            if (run <= 4UZ) {
                flipAt = static_cast<std::ptrdiff_t>(earlier + i);
                break;
            }
        }
        expect(gt(flipAt, std::ptrdiff_t{0})) << "a wire item to corrupt was found in the third frame";

        const Chain damaged = hdlcChain(addressing, build(), flipAt);
        expect(eq(damaged.failed.size(), 1UZ)) << "the corrupted frame leaves by the fail port";
        expect(eq(damaged.received.size(), messages.size() - 1UZ)) << "and never reaches the decoder, which sees the frames either side of it";
    };

    "the adapters conform: metadata, counters and independence from the record window"_test = [] {
        const gr::property_map    addressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}};
        const std::vector<Record> payloads{recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()), {{"packet", gr::Size_t{7}}, {"origin", std::string("qa")}}), //
            recordOf({0x11U, 0x22U}, {{"packet", gr::Size_t{8}}}), recordOf({})};

        // the same frames come out whatever the window is, because a record is assembled whole or not at all
        std::vector<std::vector<std::uint8_t>> reference;
        for (const std::size_t chunk : {1UZ, 2UZ, 0UZ}) {
            Ax25Encode   block = make<Ax25Encode>(addressing);
            const Driven built = feed(block, std::span<const Record>(payloads), chunk);
            expect(eq(built.out.size(), payloads.size())) << std::format("chunk {}", chunk);
            std::vector<std::vector<std::uint8_t>> produced;
            for (const Record& record : built.out) {
                produced.push_back(record.signal_values);
            }
            if (reference.empty()) {
                reference = produced;
            } else {
                expect(that % (produced == reference)) << std::format("chunk {}", chunk);
            }
        }

        Ax25Encode   encoder = make<Ax25Encode>(addressing);
        const Driven built   = feed(encoder, std::span<const Record>(payloads));
        expect(eq(built.out.size(), 3UZ));
        if (built.out.size() != 3UZ) {
            return;
        }
        expect(eq(metaSize(built.out[0UZ], "packet"), gr::Size_t{7})) << "the record's own keys cross the encoder verbatim";
        expect(eq(metaString(built.out[0UZ], "origin"), std::string("qa")));
        expect(eq(built.out[2UZ].signal_values.size(), 16UZ)) << "an empty information field still makes a frame";
        expect(eq(encoder.nRecords, 3ULL));
        expect(nothrow([&encoder] { encoder.stop(); }));

        Ax25Decode                decoder;
        const std::vector<Record> back = decoded(std::span<const Record>(built.out), decoder);
        expect(eq(back.size(), 3UZ));
        if (back.size() == 3UZ) {
            expect(eq(metaSize(back[0UZ], "packet"), gr::Size_t{7})) << "and the decoder's keys are written over them rather than instead of them";
            expect(eq(metaString(back[0UZ], "origin"), std::string("qa")));
            expect(eq(metaString(back[0UZ], "ax25_source"), std::string("N0CALL")));
            expect(eq(back[2UZ].signal_values.size(), 0UZ)) << "a frame with no information field decodes to a record with no items";
            expect(eq(back[0UZ].signal_names.size(), 1UZ)) << "the record's signal name follows it through";
            expect(eq(back[0UZ].timing_events.size(), 1UZ));
        }
        expect(eq(decoder.nRecords, 3ULL));
        expect(nothrow([&decoder] { decoder.stop(); }));

        // room for fewer records than arrived: the rest stay in the buffer for the next call
        Ax25Encode          narrow = make<Ax25Encode>(addressing);
        std::vector<Record> scratch(2UZ);
        InputSpan<Record>   inSpan(std::span<const Record>(payloads), 0UZ);
        OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
        std::ignore = narrow.processBulk(inSpan, outSpan);
        expect(eq(outSpan.count, 2UZ));
        expect(eq(inSpan.consumed, 2UZ)) << "only the records the port had room for were consumed";
    };

    "the address filter matches by address, direction and an optional digipeater, over Ax25Decode's own keys"_test = [] {
        // anchor A, decoded for real: destination APRS, source N0CALL, SSID unstated (0), no repeaters
        const std::vector<Record> anchorAFrames{recordOf(std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()))};
        Ax25Decode                anchorADecoder;
        const std::vector<Record> anchorA = decoded(std::span<const Record>(anchorAFrames), anchorADecoder);
        expect(eq(anchorA.size(), 1UZ));

        // a two-hop path, one hop repeated and one not, built through Ax25Encode/Ax25Decode rather than by hand
        const gr::property_map    twoHopAddressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}, {"via", std::string("WIDE1-1*,WIDE2-2")}};
        const std::vector<Record> twoHopPayload{recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))};
        Ax25Encode                twoHopEncoder = make<Ax25Encode>(twoHopAddressing);
        const Driven              twoHopBuilt   = feed(twoHopEncoder, std::span<const Record>(twoHopPayload));
        expect(eq(twoHopBuilt.out.size(), 1UZ));
        Ax25Decode                twoHopDecoder;
        const std::vector<Record> twoHop = decoded(std::span<const Record>(twoHopBuilt.out), twoHopDecoder);
        expect(eq(twoHop.size(), 1UZ));
        if (anchorA.empty() || twoHop.empty()) {
            return;
        }
        expect(eq(metaString(twoHop[0UZ], "ax25_via"), std::string("WIDE1-1*,WIDE2-2")));

        const auto matches = [](gr::property_map settings, const Record& record) {
            Ax25AddressFilter filter = make<Ax25AddressFilter>(std::move(settings));
            const Filtered    result = runFilter(filter, std::span<const Record>(std::addressof(record), 1UZ));
            return result.ok.size() == 1UZ;
        };

        expect(matches({{"address", std::string("APRS")}, {"direction", std::string("destination")}}, anchorA[0UZ])) << "APRS as destination matches";
        expect(!matches({{"address", std::string("APRS")}, {"direction", std::string("source")}}, anchorA[0UZ])) << "APRS is not the source";
        expect(matches({{"address", std::string("N0CALL")}, {"direction", std::string("source")}}, anchorA[0UZ])) << "N0CALL as source matches";
        expect(!matches({{"address", std::string("N0CALL-3")}, {"direction", std::string("source")}}, anchorA[0UZ])) << "SSID 3 is stated and different from anchor A's unstated SSID";
        expect(matches({{"address", std::string("N0CALL")}, {"direction", std::string("either")}}, anchorA[0UZ])) << "no SSID stated matches any, including anchor A's SSID 0";

        expect(matches({{"address", std::string("NOBODY")}, {"direction", std::string("either")}, {"digipeater", std::string("WIDE1-1")}}, twoHop[0UZ])) << "the repeated hop's '*' marker satisfies digipeater";
        expect(!matches({{"address", std::string("NOBODY")}, {"direction", std::string("either")}, {"digipeater", std::string("WIDE2-2")}}, twoHop[0UZ])) << "the same via without '*' does not";

        // a record missing ax25_source is counted, not thrown, and the block keeps working on the record after it
        Record noSource = anchorA[0UZ];
        noSource.meta_information[0UZ].erase(gr::property_map::key_type("ax25_source"));
        Ax25AddressFilter         filter = make<Ax25AddressFilter>({{"address", std::string("N0CALL")}, {"direction", std::string("source")}});
        const std::vector<Record> twoRecords{noSource, anchorA[0UZ]};
        const Filtered            result = runFilter(filter, std::span<const Record>(twoRecords));
        expect(eq(result.ok.size(), 1UZ)) << "the second record still matches";
        expect(eq(result.failed.size(), 1UZ));
        expect(eq(filter.nMissingKey, std::uint64_t{1ULL}));
        expect(eq(filter.nFailed, std::uint64_t{1ULL}));
        expect(eq(filter.nRecords, std::uint64_t{1ULL}));
        expect(nothrow([&filter] { filter.stop(); }));

        // the required settings are refused at start() when empty, and a bad grammar is refused when staged
        expect(throws([] {
            Ax25AddressFilter block = make<Ax25AddressFilter>({{"direction", std::string("destination")}});
            block.start();
        })) << "address is required";
        expect(throws([] {
            Ax25AddressFilter block = make<Ax25AddressFilter>({{"address", std::string("APRS")}});
            block.start();
        })) << "direction is required";
        expect(throws([] { std::ignore = make<Ax25AddressFilter>({{"address", std::string("APRS")}, {"direction", std::string("sideways")}}); })) << "direction must name one of the three values";
        expect(throws([] { std::ignore = make<Ax25AddressFilter>({{"address", std::string("n0call")}, {"direction", std::string("source")}}); })) << "the address grammar is checked the same way Ax25Encode checks it";

        // digipeater is read under the same grammar as address: refused when staged if it is not a callsign, and an
        // SSID left unstated matches any, exactly as it does for the primary address
        expect(throws([] { std::ignore = make<Ax25AddressFilter>({{"address", std::string("APRS")}, {"direction", std::string("either")}, {"digipeater", std::string("wide1-1")}}); })) << "a lowercase callsign is no callsign";
        expect(throws([] { std::ignore = make<Ax25AddressFilter>({{"address", std::string("APRS")}, {"direction", std::string("either")}, {"digipeater", std::string("WIDE1-1*")}}); })) << "the repeated marker belongs to the frame, not to the setting";
        expect(throws([] { std::ignore = make<Ax25AddressFilter>({{"address", std::string("APRS")}, {"direction", std::string("either")}, {"digipeater", std::string("WIDE1-99")}}); })) << "99 is no SSID";
        expect(matches({{"address", std::string("NOBODY")}, {"direction", std::string("either")}, {"digipeater", std::string("WIDE1")}}, twoHop[0UZ])) << "an SSID unstated matches the repeated WIDE1-1 hop";
        expect(!matches({{"address", std::string("NOBODY")}, {"direction", std::string("either")}, {"digipeater", std::string("WIDE1-2")}}, twoHop[0UZ])) << "an SSID stated and different does not";
    };

    "the address filter's failures are counted whether or not `fail` is connected, and never dropped when it is"_test = [] {
        const std::vector<Record> anchorAFrames{recordOf(std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()))};
        Ax25Decode                decoder;
        const std::vector<Record> anchorA = decoded(std::span<const Record>(anchorAFrames), decoder);
        expect(eq(anchorA.size(), 1UZ));
        if (anchorA.empty()) {
            return;
        }

        // criterion 5: with `fail` unconnected a non-matching record is a counted stated drop and nothing stalls
        {
            Ax25AddressFilter         filter = make<Ax25AddressFilter>({{"address", std::string("NOBODY")}, {"direction", std::string("either")}});
            const std::vector<Record> three{anchorA[0UZ], anchorA[0UZ], anchorA[0UZ]};
            std::vector<Record>       okBuf(4UZ);
            InputSpan<Record>         inSpan(std::span<const Record>(three), 0UZ);
            OutputSpan<Record>        okSpan{std::span<Record>(okBuf)};
            OutputSpan<Record>        failSpan{std::span<Record>{}, false};
            std::ignore = filter.processBulk(inSpan, okSpan, failSpan);
            expect(eq(okSpan.count, 0UZ));
            expect(eq(inSpan.consumed, 3UZ)) << "every record is consumed rather than left in the buffer";
            expect(eq(filter.nFailed, std::uint64_t{3ULL})) << "the count is all an unconnected port leaves behind";
        }

        // a connected `fail` with room for one refusal: the rest wait for the next call rather than being lost
        {
            Ax25AddressFilter         filter = make<Ax25AddressFilter>({{"address", std::string("NOBODY")}, {"direction", std::string("either")}});
            const std::vector<Record> three{anchorA[0UZ], anchorA[0UZ], anchorA[0UZ]};
            std::vector<Record>       okBuf(4UZ);
            std::vector<Record>       failBuf(1UZ);

            const auto call = [&](std::span<const Record> offered) {
                InputSpan<Record>  inSpan(offered, 0UZ);
                OutputSpan<Record> okSpan{std::span<Record>(okBuf)};
                OutputSpan<Record> failSpan{std::span<Record>(failBuf)};
                std::ignore = filter.processBulk(inSpan, okSpan, failSpan);
                return std::pair{inSpan.consumed, failSpan.count};
            };

            const auto [consumedFirst, refusedFirst] = call(std::span<const Record>(three));
            expect(eq(refusedFirst, 1UZ));
            expect(eq(consumedFirst, 1UZ)) << "only the record the port had room for was consumed";
            expect(eq(filter.nFailed, std::uint64_t{1ULL})) << "nothing is counted that was not written";

            const auto [consumedSecond, refusedSecond] = call(std::span<const Record>(three).subspan(1UZ));
            expect(eq(refusedSecond, 1UZ)) << "the block resumes where it stopped";
            expect(eq(consumedSecond, 1UZ));
            expect(eq(filter.nFailed, std::uint64_t{2ULL}));
        }
    };

    "direction 'either' needs both keys absent to be a missing key, and both ports carry the same metadata"_test = [] {
        const std::vector<Record> anchorAFrames{recordOf(std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()))};
        Ax25Decode                decoder;
        const std::vector<Record> anchorA = decoded(std::span<const Record>(anchorAFrames), decoder);
        expect(eq(anchorA.size(), 1UZ));
        if (anchorA.empty()) {
            return;
        }

        Record noSource = anchorA[0UZ];
        noSource.meta_information[0UZ].erase(gr::property_map::key_type("ax25_source"));
        Record neither = noSource;
        neither.meta_information[0UZ].erase(gr::property_map::key_type("ax25_destination"));

        // the destination is still there and still says APRS, so the frame did say who it was addressed to
        Ax25AddressFilter matching = make<Ax25AddressFilter>({{"address", std::string("APRS")}, {"direction", std::string("either")}});
        const Filtered    matched  = runFilter(matching, std::span<const Record>(std::vector<Record>{noSource}));
        expect(eq(matched.ok.size(), 1UZ)) << "one key of the two is enough to match on";
        expect(eq(matching.nMissingKey, std::uint64_t{0ULL}));

        Ax25AddressFilter other  = make<Ax25AddressFilter>({{"address", std::string("NOBODY")}, {"direction", std::string("either")}});
        const Filtered    failed = runFilter(other, std::span<const Record>(std::vector<Record>{noSource, neither}));
        expect(eq(failed.failed.size(), 2UZ));
        expect(eq(other.nFailed, std::uint64_t{2ULL}));
        expect(eq(other.nMissingKey, std::uint64_t{1ULL})) << "only the record carrying neither key is a missing key";

        // metadata is identical on both ports: the frame's facts were the decode's to write
        Ax25AddressFilter split  = make<Ax25AddressFilter>({{"address", std::string("APRS")}, {"direction", std::string("destination")}});
        const Filtered    routed = runFilter(split, std::span<const Record>(std::vector<Record>{anchorA[0UZ], neither}));
        expect(eq(routed.ok.size(), 1UZ));
        expect(eq(routed.failed.size(), 1UZ));
        if (routed.ok.size() == 1UZ && routed.failed.size() == 1UZ) {
            expect(eq(routed.ok[0UZ].meta_information[0UZ].size(), anchorA[0UZ].meta_information[0UZ].size())) << "nothing added on the way to ok";
            expect(eq(routed.failed[0UZ].meta_information[0UZ].size(), neither.meta_information[0UZ].size())) << "nor on the way to fail";
            expect(eq(metaString(routed.ok[0UZ], "ax25_destination"), std::string("APRS")));
            expect(eq(metaString(routed.ok[0UZ], "ax25_type"), metaString(anchorA[0UZ], "ax25_type")));
            expect(eq(metaString(routed.failed[0UZ], "ax25_type"), metaString(anchorA[0UZ], "ax25_type"))) << "the same keys with the same values on the other port";
            expect(!metaHas(routed.ok[0UZ], "matched")) << "which port a record left by is not written into it";
            expect(!metaHas(routed.failed[0UZ], "matched"));
        }
    };
};

int main() { /* not needed for UT */ }
