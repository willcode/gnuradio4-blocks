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
#include <gnuradio-4.0/ax25/Kiss.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>

#include "TestSpans.hpp"

/*
 * KISS is one byte, so the tests are about what that byte decides. The anchor pins the byte and the SLIP wire form it
 * sits inside; the chain proves that an AX.25 frame handed to a terminal node controller and read back off one comes
 * home with its port intact; and the parameter frame proves that a controller announcing its own timing costs a count
 * rather than a frame either side of it.
 */
namespace {

using gr::blocks::ax25::Ax25Decode;
using gr::blocks::ax25::Ax25Encode;
using gr::blocks::ax25::KissDecode;
using gr::blocks::ax25::KissEncode;
using gr::blocks::ax25::test::InputSpan;
using gr::blocks::ax25::test::OutputSpan;
using gr::blocks::digital::DelimiterExtractor;
using gr::blocks::digital::DelimiterFramer;

using Record = gr::DataSet<std::uint8_t>;

/// Anchor A: the twenty-one byte UI frame the KISS anchor wraps.
constexpr std::array<std::uint8_t, 21UZ> kAnchorA{{0x82U, 0xA0U, 0xA4U, 0xA6U, 0x40U, 0x40U, 0xE0U, //
    0x9CU, 0x60U, 0x86U, 0x82U, 0x98U, 0x98U, 0x61U,                                                //
    0x03U, 0xF0U,                                                                                   //
    0x3AU, 0x54U, 0x45U, 0x53U, 0x54U}};

constexpr std::array<std::uint8_t, 5UZ> kInfo{{0x3AU, 0x54U, 0x45U, 0x53U, 0x54U}};

/// A terminal node controller's TXDELAY parameter frame: port 2, command 1, one parameter byte.
constexpr std::array<std::uint8_t, 2UZ> kTxDelay{{0x21U, 0x19U}};

struct Rng {
    std::uint64_t state = 0x2545f4914f6cdd1dULL;

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

[[nodiscard]] Record recordOf(std::vector<std::uint8_t> bytes, gr::property_map meta = {}) {
    Record record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.meta_information.push_back(std::move(meta));
    record.timing_events.emplace_back();
    return record;
}

/// The nine bytes of a command-9 timestamp frame: the command byte, then @p milliseconds big-endian.
[[nodiscard]] std::vector<std::uint8_t> stampFrame(std::uint64_t milliseconds) {
    std::vector<std::uint8_t> bytes(9UZ, 0x00U);
    bytes[0UZ] = 0x09U;
    for (std::size_t i = 0UZ; i < 8UZ; ++i) {
        bytes[1UZ + i] = static_cast<std::uint8_t>((milliseconds >> (56U - 8U * i)) & 0xFFULL);
    }
    return bytes;
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
 * @brief Flattens each framed record onto the wire, splicing `_inject` in behind the record at `_injectAfter`.
 *
 * The injection is how a terminal node controller's own parameter frame reaches the chain: it arrives on the wire
 * between two data frames, already delimited, exactly as a controller sends one.
 */
struct RecordToStream : gr::Block<RecordToStream> {
    gr::PortIn<Record, gr::Async>        in;
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordToStream, in, out);

    std::vector<Record>       _seen{};
    std::vector<std::uint8_t> _pending{};
    std::vector<std::uint8_t> _inject{};
    std::size_t               _injectAfter = 0UZ;
    std::size_t               _sent        = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        if (_sent == _pending.size() && inSpan.size() > 0UZ) {
            _seen.push_back(inSpan[0UZ]);
            _pending = inSpan[0UZ].signal_values;
            if (!_inject.empty() && _seen.size() == _injectAfter) {
                _pending.insert(_pending.end(), _inject.begin(), _inject.end());
            }
            _sent    = 0UZ;
            consumed = 1UZ;
        }

        const std::size_t n = std::min(_pending.size() - _sent, outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _pending[_sent + i];
        }
        _sent += n;

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

/// SLIP framing, which serves KISS byte for byte: 0xC0 between frames, 0xDB the introducer and its two escapes.
[[nodiscard]] gr::property_map slipFraming() {
    return {{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8}}, {"transparency", std::string("byte_escape")}, //
        {"escape_item", gr::Size_t{0xDB}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", gr::Size_t{1024}}};
}

/// What one run of the KISS chain produced.
struct Chain {
    std::vector<Record> received{};
    std::uint64_t       controlFrames = 0ULL;
    std::uint64_t       emptyRecords  = 0ULL;
};

/// `records` through Ax25Encode, KissEncode, the SLIP wire, KissDecode and Ax25Decode in one graph.
[[nodiscard]] Chain kissChain(const gr::property_map& addressing, gr::Size_t port, std::vector<Record> records, std::vector<std::uint8_t> inject = {}, std::size_t injectAfter = 0UZ) {
    gr::Graph flow;
    auto&     source  = flow.emplaceBlock<RecordSource>();
    source._records   = std::move(records);
    auto& encode      = flow.emplaceBlock<Ax25Encode>(addressing);
    auto& kiss        = flow.emplaceBlock<KissEncode>({{"kiss_port", port}});
    auto& framer      = flow.emplaceBlock<DelimiterFramer>(slipFraming());
    auto& wire        = flow.emplaceBlock<RecordToStream>();
    wire._inject      = std::move(inject);
    wire._injectAfter = injectAfter;
    auto& extractor   = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(slipFraming());
    auto& unkiss      = flow.emplaceBlock<KissDecode>();
    auto& decode      = flow.emplaceBlock<Ax25Decode>();
    auto& sink        = flow.emplaceBlock<RecordSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, encode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(encode, kiss).has_value());
    boost::ut::expect(flow.connect<"out", "in">(kiss, framer).has_value());
    boost::ut::expect(flow.connect<"out", "in">(framer, wire).has_value());
    boost::ut::expect(flow.connect<"out", "in">(wire, extractor).has_value());
    boost::ut::expect(flow.connect<"out", "in">(extractor, unkiss).has_value());
    boost::ut::expect(flow.connect<"out", "in">(unkiss, decode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(decode, sink).has_value());

    Chain result;
    runGraph(std::move(flow), [&result, &unkiss, &sink] {
        result.received      = sink._records;
        result.controlFrames = unkiss.nControlFrames;
        result.emptyRecords  = unkiss.nRefusedEmpty;
    });
    return result;
}

} // namespace

const boost::ut::suite<"kiss"> kissTests = [] {
    using namespace boost::ut;

    "anchor C: the port 2 data frame, on the record and on the wire"_test = [] {
        const std::vector<Record> frames{recordOf(std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()))};

        KissEncode   encoder = make<KissEncode>({{"kiss_port", gr::Size_t{2}}});
        const Driven built   = feed(encoder, std::span<const Record>(frames));
        expect(eq(built.out.size(), 1UZ));
        if (built.out.size() != 1UZ) {
            return;
        }
        std::vector<std::uint8_t> onRecord{0x20U};
        onRecord.insert(onRecord.end(), kAnchorA.begin(), kAnchorA.end());
        expect(that % (built.out[0UZ].signal_values == onRecord)) << "the command byte is the port in the high nibble and the data command in the low one";
        expect(eq(built.out[0UZ].signal_values.size(), 22UZ));

        DelimiterFramer     framer = make<DelimiterFramer>(slipFraming());
        std::vector<Record> scratch(4UZ);
        InputSpan<Record>   inSpan(std::span<const Record>(built.out), 0UZ);
        OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
        std::ignore = framer.processBulk(inSpan, outSpan);
        expect(eq(outSpan.count, 1UZ));
        if (outSpan.count == 1UZ) {
            std::vector<std::uint8_t> onWire{0xC0U};
            onWire.insert(onWire.end(), onRecord.begin(), onRecord.end());
            onWire.push_back(0xC0U);
            expect(that % (scratch[0UZ].signal_values == onWire)) << "no byte of the frame is 0xC0 or 0xDB, so nothing is escaped";
            expect(eq(scratch[0UZ].signal_values.size(), 24UZ));
        }

        KissDecode   decoder;
        const Driven back = feed(decoder, std::span<const Record>(built.out));
        expect(eq(back.out.size(), 1UZ));
        if (back.out.size() == 1UZ) {
            expect(that % (back.out[0UZ].signal_values == std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end())));
            expect(eq(metaSize(back.out[0UZ], "kiss_port"), gr::Size_t{2}));
        }
        expect(eq(decoder.nRecords, 1ULL));
        expect(eq(decoder.nPayloadBytes, 21ULL));
    };

    "a parameter frame is counted, an empty record separately, and the frames around them decode"_test = [] {
        std::vector<std::uint8_t> data{0x00U};
        data.insert(data.end(), kAnchorA.begin(), kAnchorA.end());
        const std::vector<Record> records{recordOf(data), recordOf(std::vector<std::uint8_t>(kTxDelay.begin(), kTxDelay.end())), recordOf({}), recordOf({0xFFU, 0x11U}), recordOf(data)};

        KissDecode   decoder;
        const Driven back = feed(decoder, std::span<const Record>(records));
        expect(eq(back.out.size(), 2UZ)) << "the two data frames, and nothing published for the rest";
        expect(eq(decoder.nControlFrames, 2ULL)) << "TXDELAY and the return command, counted once each rather than by the byte";
        expect(eq(decoder.nRefusedEmpty, 1ULL));
        expect(eq(back.consumed, records.size())) << "a dropped record is consumed rather than left in the buffer";
        for (const Record& record : back.out) {
            expect(eq(metaSize(record, "kiss_port"), gr::Size_t{0}));
        }
        expect(nothrow([&decoder] { decoder.stop(); })) << "the report is written once, at stop()";
    };

    "the port setting is bounded, and a record may name its own"_test = [] {
        expect(throws([] { std::ignore = make<KissEncode>({{"kiss_port", gr::Size_t{16}}}); })) << "sixteen does not fit the command byte's high nibble";
        expect(nothrow([] { std::ignore = make<KissEncode>({{"kiss_port", gr::Size_t{15}}}); }));

        const std::vector<std::uint8_t> frame(kAnchorA.begin(), kAnchorA.end());
        const std::vector<Record>       records{recordOf(frame, {{"kiss_port", gr::Size_t{5}}}), recordOf(frame, {{"kiss_port", gr::Size_t{99}}}), recordOf(frame)};

        KissEncode   encoder = make<KissEncode>({{"kiss_port", gr::Size_t{2}}});
        const Driven built   = feed(encoder, std::span<const Record>(records));
        expect(eq(built.out.size(), 2UZ));
        expect(eq(encoder.nRefusedOverride, 1ULL));
        expect(eq(built.consumed, records.size()));
        if (built.out.size() == 2UZ) {
            expect(eq(static_cast<unsigned>(built.out[0UZ].signal_values.at(0UZ)), 0x50U)) << "the record's own port";
            expect(eq(static_cast<unsigned>(built.out[1UZ].signal_values.at(0UZ)), 0x20U)) << "and the record after the refused one takes the setting again";
        }
        expect(nothrow([&encoder] { encoder.stop(); }));
    };

    "the KISS chain returns every frame with its port, past a parameter frame on the wire"_test = [] {
        const gr::property_map addressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL-3")}, {"via", std::string("WIDE2-1")}};

        Rng                                    rng;
        std::vector<std::vector<std::uint8_t>> messages;
        for (const std::size_t length : {1UZ, 5UZ, 24UZ, 61UZ}) {
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

        const auto check = [&messages](const Chain& run, std::uint64_t controlFrames) {
            expect(eq(run.received.size(), messages.size()));
            expect(eq(run.controlFrames, controlFrames));
            for (std::size_t which = 0UZ; which < std::min(run.received.size(), messages.size()); ++which) {
                expect(that % (run.received[which].signal_values == messages[which])) << std::format("frame {}: the information field is byte identical", which);
                expect(eq(metaString(run.received[which], "ax25_destination"), std::string("APRS"))) << std::format("frame {}", which);
                expect(eq(metaString(run.received[which], "ax25_source"), std::string("N0CALL-3"))) << std::format("frame {}", which);
                expect(eq(metaString(run.received[which], "ax25_via"), std::string("WIDE2-1"))) << std::format("frame {}", which);
                expect(eq(metaSize(run.received[which], "kiss_port"), gr::Size_t{2})) << std::format("frame {}: the port carries the whole way", which);
                expect(metaHas(run.received[which], "stuffing_removed")) << std::format("frame {}: what the extractor wrote crosses both decoders verbatim", which);
            }
        };

        check(kissChain(addressing, 2U, build()), 0ULL);

        // the same run with a TXDELAY frame spliced onto the wire behind the second data frame, delimited as a
        // controller sends one; neither of its two parameter bytes is 0xC0 or 0xDB, so nothing about it needs escaping
        const std::vector<std::uint8_t> parameter{0xC0U, kTxDelay[0UZ], kTxDelay[1UZ], 0xC0U};
        check(kissChain(addressing, 2U, build(), parameter, 2UZ), 1ULL);
    };

    "the KISS adapters conform: metadata, counters and independence from the record window"_test = [] {
        const std::vector<Record> records{recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()), {{"packet", gr::Size_t{7}}, {"origin", std::string("qa")}}), //
            recordOf({0xC0U, 0xDBU}, {{"packet", gr::Size_t{8}}}), recordOf({0x33U})};

        std::vector<std::vector<std::uint8_t>> reference;
        for (const std::size_t chunk : {1UZ, 2UZ, 0UZ}) {
            KissEncode   block = make<KissEncode>({{"kiss_port", gr::Size_t{4}}});
            const Driven built = feed(block, std::span<const Record>(records), chunk);
            expect(eq(built.out.size(), records.size())) << std::format("chunk {}", chunk);
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

        KissEncode   encoder = make<KissEncode>({{"kiss_port", gr::Size_t{4}}});
        const Driven built   = feed(encoder, std::span<const Record>(records));
        expect(eq(built.out.size(), 3UZ));
        if (built.out.size() != 3UZ) {
            return;
        }
        expect(eq(metaSize(built.out[0UZ], "packet"), gr::Size_t{7})) << "the record's own keys cross the encoder verbatim";
        expect(eq(metaString(built.out[0UZ], "origin"), std::string("qa")));
        expect(eq(encoder.nRecords, 3ULL));

        KissDecode   decoder;
        const Driven back = feed(decoder, std::span<const Record>(built.out));
        expect(eq(back.out.size(), 3UZ));
        if (back.out.size() == 3UZ) {
            expect(that % (back.out[0UZ].signal_values == std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))) << "the round trip is the identity on the payload";
            expect(eq(metaSize(back.out[0UZ], "packet"), gr::Size_t{7})) << "and the port key is written over the record's keys rather than instead of them";
            expect(eq(metaString(back.out[0UZ], "origin"), std::string("qa")));
            expect(eq(metaSize(back.out[0UZ], "kiss_port"), gr::Size_t{4}));
            expect(eq(back.out[0UZ].signal_names.size(), 1UZ)) << "the record's signal name follows it through";
            expect(eq(back.out[0UZ].timing_events.size(), 1UZ));
        }
        expect(eq(decoder.nRecords, 3ULL));

        // room for fewer records than arrived: the rest stay in the buffer for the next call
        KissEncode          narrow = make<KissEncode>({{"kiss_port", gr::Size_t{4}}});
        std::vector<Record> scratch(2UZ);
        InputSpan<Record>   inSpan(std::span<const Record>(records), 0UZ);
        OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
        std::ignore = narrow.processBulk(inSpan, outSpan);
        expect(eq(outSpan.count, 2UZ));
        expect(eq(inSpan.consumed, 2UZ)) << "only the records the port had room for were consumed";
    };

    // The timestamp record, both directions, exactly.
    "the timestamp record: KissEncode emits it, KissDecode reads it back, exactly"_test = [] {
        constexpr std::int64_t  kTimestampNs = 1'500'000'000'123'456'789LL;
        constexpr std::uint64_t kExpectedMs  = 1'500'000'000'123ULL;

        Record stamped    = recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()));
        stamped.timestamp = kTimestampNs;
        Record unstamped  = recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end())); // timestamp defaults to 0, the field's unstated value

        KissEncode   encoder = make<KissEncode>({{"kiss_port", gr::Size_t{2}}, {"emit_timestamp", true}});
        const Driven built   = feed(encoder, std::span<const Record>(std::vector<Record>{stamped, unstamped}));
        expect(eq(built.out.size(), 3UZ)) << "a stamp frame ahead of the first record, then both data frames";
        expect(eq(encoder.nTimestampsUnavailable, std::uint64_t{1ULL})) << "the second record's timestamp is 0";
        if (built.out.size() != 3UZ) {
            return;
        }

        const Record& stampFrame = built.out[0UZ];
        expect(eq(stampFrame.signal_values.size(), 9UZ));
        expect(eq(static_cast<unsigned>(stampFrame.signal_values[0UZ]), 0x09U)) << "the command byte, unassigned by Chepponis and Karn";
        std::uint64_t decodedMs = 0ULL;
        for (std::size_t i = 1UZ; i < 9UZ; ++i) {
            decodedMs = (decodedMs << 8U) | stampFrame.signal_values[i];
        }
        expect(eq(decodedMs, kExpectedMs)) << "1 500 000 000 123 456 789 ns truncates to 1 500 000 000 123 ms";
        std::array<std::uint8_t, 8UZ> expectedBytes{};
        for (std::size_t i = 0UZ; i < 8UZ; ++i) {
            expectedBytes[i] = static_cast<std::uint8_t>((kExpectedMs >> (56U - 8U * i)) & 0xFFU);
        }
        expect(that % std::equal(stampFrame.signal_values.begin() + 1, stampFrame.signal_values.end(), expectedBytes.begin())) << "the eight bytes, big-endian";
        std::vector<std::uint8_t> expectedDataFrame{0x20U}; // kiss_port 2, data command
        expectedDataFrame.insert(expectedDataFrame.end(), kInfo.begin(), kInfo.end());
        expect(that % (built.out[1UZ].signal_values == expectedDataFrame)) << "the stamp frame precedes the record it stamps rather than replacing it";

        KissDecode   decoder = make<KissDecode>({{"read_timestamp", true}});
        const Driven back    = feed(decoder, std::span<const Record>(built.out));
        expect(eq(back.out.size(), 2UZ)) << "the stamp frame is consumed into the pending value, not published";
        expect(eq(decoder.nTimestampsRead, std::uint64_t{1ULL}));
        expect(eq(decoder.nTimestampsMalformed, std::uint64_t{0ULL}));
        if (back.out.size() == 2UZ) {
            expect(eq(back.out[0UZ].timestamp, std::int64_t{1'500'000'000'123'000'000LL})) << "the original truncated to the millisecond, asserted as that exact number";
            expect(eq(back.out[1UZ].timestamp, std::int64_t{0LL})) << "no stamp frame preceded the second record";
        }
        expect(eq(decoder.nTimestampsUnused, std::uint64_t{0ULL})) << "the one stamp read was consumed by the record right after it";
        expect(nothrow([&decoder] { decoder.stop(); }));
    };

    "a second timestamp frame before any data frame supersedes the first, and the first is counted unused"_test = [] {
        std::vector<std::uint8_t> data{0x00U};
        data.insert(data.end(), kInfo.begin(), kInfo.end());
        const std::vector<Record> records{recordOf(stampFrame(1'000ULL)), recordOf(stampFrame(2'000ULL)), recordOf(data)};

        KissDecode   decoder = make<KissDecode>({{"read_timestamp", true}});
        const Driven back    = feed(decoder, std::span<const Record>(records));
        expect(eq(back.out.size(), 1UZ)) << "one data frame, and neither stamp frame published";
        expect(eq(decoder.nTimestampsRead, std::uint64_t{2ULL}));
        expect(eq(decoder.nTimestampsUnused, std::uint64_t{1ULL})) << "the first stamp was superseded before any data frame took it";
        if (back.out.size() == 1UZ) {
            expect(eq(back.out[0UZ].timestamp, std::int64_t{2'000'000'000LL})) << "2 000 ms in nanoseconds: the newer stamp is the one carried";
        }
    };

    "a stamp naming more milliseconds than nanoseconds hold is malformed, at the exact boundary"_test = [] {
        // 2^63 - 1 nanoseconds is 9 223 372 036 854 775 807, so 9 223 372 036 854 ms is the last value that fits
        constexpr std::uint64_t kLastFitting = 9'223'372'036'854ULL;

        KissDecode   decoder = make<KissDecode>({{"read_timestamp", true}});
        const Driven fitting = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(stampFrame(kLastFitting))}));
        expect(eq(fitting.out.size(), 0UZ));
        expect(eq(decoder.nTimestampsRead, std::uint64_t{1ULL}));
        expect(eq(decoder.nTimestampsMalformed, std::uint64_t{0ULL}));

        std::vector<std::uint8_t> data{0x00U};
        data.insert(data.end(), kInfo.begin(), kInfo.end());
        const Driven carried = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(data)}));
        expect(eq(carried.out.size(), 1UZ));
        if (carried.out.size() == 1UZ) {
            expect(eq(carried.out[0UZ].timestamp, std::int64_t{9'223'372'036'854'000'000LL}));
        }

        KissDecode   past = make<KissDecode>({{"read_timestamp", true}});
        const Driven over = feed(past, std::span<const Record>(std::vector<Record>{recordOf(stampFrame(kLastFitting + 1ULL)), recordOf(stampFrame(0xFFFFFFFFFFFFFFFFULL)), recordOf(data)}));
        expect(eq(over.out.size(), 1UZ)) << "the data frame behind the refused stamps still decodes";
        expect(eq(past.nTimestampsMalformed, std::uint64_t{2ULL})) << "one past the boundary and the all-ones frame";
        expect(eq(past.nTimestampsRead, std::uint64_t{0ULL}));
        expect(eq(past.nTimestampsUnused, std::uint64_t{0ULL})) << "a refused stamp is never held, so there is nothing to leave unused";
        if (over.out.size() == 1UZ) {
            expect(eq(over.out[0UZ].timestamp, std::int64_t{0LL})) << "nothing is invented for a time the carrier cannot express";
        }
    };

    "a pending stamp goes only when read_timestamp itself changes"_test = [] {
        std::vector<std::uint8_t> data{0x00U};
        data.insert(data.end(), kInfo.begin(), kInfo.end());

        KissDecode decoder = make<KissDecode>({{"read_timestamp", true}});
        std::ignore        = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(stampFrame(1'234ULL))}));
        expect(eq(decoder.nTimestampsRead, std::uint64_t{1ULL}));

        expect(decoder.settings().setStaged({{"read_timestamp", true}}).empty());
        std::ignore = decoder.settings().applyStagedParameters();
        expect(eq(decoder.nTimestampsUnused, std::uint64_t{0ULL})) << "staging the value it already has is not a change";

        const Driven carried = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(data)}));
        expect(eq(carried.out.size(), 1UZ));
        if (carried.out.size() == 1UZ) {
            expect(eq(carried.out[0UZ].timestamp, std::int64_t{1'234'000'000LL})) << "the stamp that survived the staging is the one the record carries";
        }

        std::ignore = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(stampFrame(5'678ULL))}));
        expect(eq(decoder.nTimestampsRead, std::uint64_t{2ULL}));
        expect(decoder.settings().setStaged({{"read_timestamp", false}}).empty());
        std::ignore = decoder.settings().applyStagedParameters();
        expect(eq(decoder.nTimestampsUnused, std::uint64_t{1ULL})) << "the stamp read under the old setting has no frame to belong to";

        const Driven after = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(data)}));
        expect(eq(after.out.size(), 1UZ));
        if (after.out.size() == 1UZ) {
            expect(eq(after.out[0UZ].timestamp, std::int64_t{0LL}));
        }
    };

    "a stamped record needs two output slots, and one is a shortage of output"_test = [] {
        Record stamped    = recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()));
        stamped.timestamp = 1'500'000'000'123'456'789LL;
        const std::vector<Record> one{stamped};

        KissEncode             encoder = make<KissEncode>({{"kiss_port", gr::Size_t{2}}, {"emit_timestamp", true}});
        std::vector<Record>    scratch(1UZ);
        InputSpan<Record>      inSpan(std::span<const Record>(one), 0UZ);
        OutputSpan<Record>     outSpan{std::span<Record>(scratch)};
        const gr::work::Status status = encoder.processBulk(inSpan, outSpan);
        expect(eq(outSpan.count, 0UZ)) << "the stamp frame is not published without the frame it stamps";
        expect(eq(inSpan.consumed, 0UZ)) << "and the record waits rather than being encoded without its stamp";
        expect(that % (status == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS)) << "one free slot is a shortage of output, whatever the input port holds";

        std::vector<Record> room(2UZ);
        InputSpan<Record>   retry(std::span<const Record>(one), 0UZ);
        OutputSpan<Record>  wider{std::span<Record>(room)};
        std::ignore = encoder.processBulk(retry, wider);
        expect(eq(wider.count, 2UZ)) << "two slots take the stamp frame and the data frame together";
        expect(eq(retry.consumed, 1UZ));
        expect(eq(static_cast<unsigned>(room[0UZ].signal_values.at(0UZ)), 0x09U));
    };

    "a malformed command-9 frame is counted and a stamp still pending at stop() is counted unused"_test = [] {
        std::vector<std::uint8_t> shortStamp{0x09U, 0x01U, 0x02U, 0x03U}; // not the nine bytes a timestamp frame needs
        const std::vector<Record> records{recordOf(shortStamp)};

        KissDecode   decoder = make<KissDecode>({{"read_timestamp", true}});
        const Driven back    = feed(decoder, std::span<const Record>(records));
        expect(eq(back.out.size(), 0UZ));
        expect(eq(decoder.nTimestampsMalformed, std::uint64_t{1ULL}));
        expect(eq(decoder.nTimestampsRead, std::uint64_t{0ULL}));

        // a second, well-formed stamp with no data frame behind it is still held when the block stops
        std::vector<std::uint8_t> stamp(9UZ, 0x00U);
        stamp[0UZ]        = 0x09U;
        const Driven held = feed(decoder, std::span<const Record>(std::vector<Record>{recordOf(stamp)}));
        expect(eq(held.out.size(), 0UZ));
        expect(eq(decoder.nTimestampsRead, std::uint64_t{1ULL}));
        expect(eq(decoder.nTimestampsUnused, std::uint64_t{0ULL})) << "not yet: it has not been superseded or stopped past";
        decoder.stop();
        expect(eq(decoder.nTimestampsUnused, std::uint64_t{1ULL})) << "held at stop() with no data frame to carry it";
    };

    // Criterion 7: with the two settings false, each block's output is what a chain that sets neither gets.
    "read_timestamp false leaves KISS decode alone, and a command-9 frame counts as a control frame"_test = [] {
        std::vector<std::uint8_t> data{0x00U};
        data.insert(data.end(), kAnchorA.begin(), kAnchorA.end());
        std::vector<std::uint8_t> commandNine(9UZ, 0x00U);
        commandNine[0UZ] = 0x09U; // a well-formed nine-byte timestamp frame, but read_timestamp is false here
        const std::vector<Record> records{recordOf(data), recordOf(commandNine), recordOf(data)};

        KissDecode   decoder = make<KissDecode>({{"read_timestamp", false}});
        const Driven back    = feed(decoder, std::span<const Record>(records));
        expect(eq(back.out.size(), 2UZ)) << "the two data frames, unaffected by a setting that is off";
        expect(eq(decoder.nControlFrames, std::uint64_t{1ULL})) << "the command-9 frame is a plain control frame when read_timestamp is false";
        expect(eq(decoder.nTimestampsRead, std::uint64_t{0ULL}));
        expect(eq(decoder.nTimestampsMalformed, std::uint64_t{0ULL}));
        for (const Record& record : back.out) {
            expect(eq(record.timestamp, std::int64_t{0LL})) << "nothing is invented for a stamp that was never read";
        }
    };

    "emit_timestamp false leaves KISS encode alone"_test = [] {
        Record withTimestamp    = recordOf(std::vector<std::uint8_t>(kAnchorA.begin(), kAnchorA.end()));
        withTimestamp.timestamp = 123456789LL; // present, but emit_timestamp is false, so it is not read

        KissEncode   encoder = make<KissEncode>({{"kiss_port", gr::Size_t{2}}, {"emit_timestamp", false}});
        const Driven built   = feed(encoder, std::span<const Record>(std::vector<Record>{withTimestamp}));
        expect(eq(built.out.size(), 1UZ)) << "no stamp frame from a setting that is off";
        expect(eq(encoder.nTimestampsUnavailable, std::uint64_t{0ULL})) << "the counter is emit_timestamp's own, and it is off";
        if (built.out.size() == 1UZ) {
            std::vector<std::uint8_t> expected{0x20U};
            expected.insert(expected.end(), kAnchorA.begin(), kAnchorA.end());
            expect(that % (built.out[0UZ].signal_values == expected));
        }
    };
};

int main() { /* not needed for UT */ }
