#include <boost/ut.hpp>

#include <algorithm>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/ccsds/SpacePackets.hpp>
#include <gnuradio-4.0/ccsds/TimeCodes.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <atomic>
#include <chrono>
#include <thread>

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

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

/// Every key the record's single map holds, sorted, so a key set can be asserted whole rather than
/// one membership at a time.
[[nodiscard]] std::vector<std::string> keysOf(const Record& record) {
    std::vector<std::string> keys;
    for (const auto& [key, value] : metaOf(record)) {
        keys.emplace_back(std::string_view(key));
    }
    std::ranges::sort(keys);
    return keys;
}

template<typename TBlock>
[[nodiscard]] std::vector<Record> drive1(TBlock& block, std::span<const Record> in, std::size_t room = 64UZ) {
    std::vector<Record> scratch(room);
    InputSpan<Record>   inSpan(in);
    OutputSpan<Record>  outSpan{std::span<Record>(scratch)};
    std::ignore = block.processBulk(inSpan, outSpan);
    scratch.resize(outSpan.count);
    return scratch;
}

/// The same drive, keeping the status the block returned and the count it published.
template<typename TBlock>
[[nodiscard]] gr::work::Status driveStatus(TBlock& block, std::span<const Record> in, std::size_t& published) {
    std::vector<Record>    scratch(8UZ);
    InputSpan<Record>      inSpan(in);
    OutputSpan<Record>     outSpan{std::span<Record>(scratch)};
    const gr::work::Status status = block.processBulk(inSpan, outSpan);
    published                     = outSpan.count;
    return status;
}

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos   = 0UZ;
    std::size_t         _chunk = 0UZ; // 0 means unbounded (limited only by outSpan room)

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

} // namespace

const boost::ut::suite<"CcsdsTimeCodeBlocks"> ccsdsTimeCodeBlocksTests = [] {
    using namespace boost::ut;
    using gr::blocks::ccsds::TimeCodeDecode;
    using gr::blocks::ccsds::TimeCodeEncode;

    static constexpr std::int64_t kWorkedInstantNs = 569'524'843'123'456'000LL;

    "the block writes the carrier field and no metadata time key"_test = [] {
        auto encoder = make<TimeCodeEncode>({{"code", std::string("cds")}, {"day_octets", gr::Size_t{2}}, {"submillisecond_octets", gr::Size_t{2}}, {"offset", gr::Size_t{8}}});
        auto decoder = make<TimeCodeDecode>({{"code", std::string("cds")}, {"offset", gr::Size_t{8}}});

        Record source                     = recordOf(std::vector<std::uint8_t>(8UZ, 0xAAU), gr::property_map{{"probe", std::string("carried")}});
        source.timestamp                  = kWorkedInstantNs;
        const std::vector<Record> encoded = drive1(encoder, std::vector<Record>{source});
        expect(eq(encoded.size(), 1UZ));
        if (encoded.size() != 1UZ) {
            return;
        }

        const std::vector<Record> decoded = drive1(decoder, encoded);
        expect(eq(decoded.size(), 1UZ));
        if (decoded.size() != 1UZ) {
            return;
        }
        expect(eq(decoded[0].timestamp, kWorkedInstantNs));
        expect(!metaHas(decoded[0], "timestamp"));
        expect(!metaHas(decoded[0], "time"));
        expect(eq(metaString(decoded[0], "ccsds_time_code"), std::string("cds")));
        expect(eq(metaString(decoded[0], "ccsds_time_scale"), std::string("utc")));

        // The whole key set, enumerated: the two keys a plain decode always writes, the producer's own
        // key crossing verbatim, and nothing else. A key naming a time would show up here.
        const std::vector<std::string> plain{"ccsds_time_code", "ccsds_time_scale", "probe"};
        expect(std::ranges::equal(keysOf(decoded[0]), plain)) << "exactly the two written keys and the one that crossed";

        // The residue below a nanosecond: a CDS code with four submillisecond octets counts picoseconds
        // of millisecond, so the encoder reads the producer's key back out of the metadata and the
        // decoder writes it again. The key appears only because the value is nonzero.
        auto   wideEncoder = make<TimeCodeEncode>({{"code", std::string("cds")}, {"day_octets", gr::Size_t{2}}, {"submillisecond_octets", gr::Size_t{4}}});
        auto   wideDecoder = make<TimeCodeDecode>({{"code", std::string("cds")}});
        Record subNs       = recordOf(std::vector<std::uint8_t>{}, gr::property_map{{"ccsds_time_sub_ns", gr::Size_t{456}}});
        subNs.timestamp    = kWorkedInstantNs + 789LL;

        const std::vector<Record> subNsOut = drive1(wideDecoder, drive1(wideEncoder, std::vector<Record>{subNs}));
        expect(eq(subNsOut.size(), 1UZ));
        if (subNsOut.size() == 1UZ) {
            expect(eq(subNsOut[0].timestamp, kWorkedInstantNs + 789LL)) << "the nanoseconds are exact";
            const std::vector<std::string> withSubNs{"ccsds_time_code", "ccsds_time_scale", "ccsds_time_sub_ns"};
            expect(std::ranges::equal(keysOf(subNsOut[0]), withSubNs));
            const auto entry = metaOf(subNsOut[0]).find(gr::property_map::key_type("ccsds_time_sub_ns"));
            expect(entry != metaOf(subNsOut[0]).end());
            if (entry != metaOf(subNsOut[0]).end()) {
                const gr::Size_t* value = entry->second.get_if<gr::Size_t>();
                expect(value != nullptr) << "a gr::Size_t, which is the type the key is stated in";
                if (value != nullptr) {
                    expect(eq(*value, gr::Size_t{456})) << "the picoseconds under a nanosecond, carried out and back";
                }
            }
            expect(eq(wideDecoder.nSubNanosecondTruncated, std::uint64_t{1}));
        }

        // A leap second, off the wire rather than through the encoder, which has no way to name one: a
        // CDS millisecond of day of 86 400 123 is 23:59:60.123, and the axis has no room for it.
        auto                      leapDecoder = make<TimeCodeDecode>({{"code", std::string("cds")}});
        const std::vector<Record> leapIn{recordOf({0x40U, 0x2AU, 0xDEU, 0x05U, 0x26U, 0x5CU, 0x7BU})};
        const std::vector<Record> leapOut = drive1(leapDecoder, leapIn);
        expect(eq(leapOut.size(), 1UZ));
        if (leapOut.size() == 1UZ) {
            const std::vector<std::string> withLeap{"ccsds_time_code", "ccsds_time_leap_second", "ccsds_time_scale"};
            expect(std::ranges::equal(keysOf(leapOut[0]), withLeap));
            const auto entry = metaOf(leapOut[0]).find(gr::property_map::key_type("ccsds_time_leap_second"));
            expect(entry != metaOf(leapOut[0]).end());
            if (entry != metaOf(leapOut[0]).end()) {
                const bool* flag = entry->second.get_if<bool>();
                expect(flag != nullptr && *flag) << "a bool, and true";
            }
            expect(eq(leapDecoder.nLeapSecondValues, std::uint64_t{1}));
        }
    };

    "the standard's own worked instant, as CDS and as CUC"_test = [] {
        // The CUC arm, pinned to the number rather than to itself: a P-field declaring four coarse
        // octets and no fractional ones, then the coarse count 948 216 043 s from the 1958 epoch, which
        // is 569 524 843 s from the Unix epoch. Nothing here comes back out of the encoder.
        {
            auto                      decoder = make<TimeCodeDecode>({{"code", std::string("cuc")}});
            const std::vector<Record> wire{recordOf({0x1CU, 0x38U, 0x84U, 0xA0U, 0xEBU})};
            const std::vector<Record> decoded = drive1(decoder, wire);
            expect(eq(decoded.size(), 1UZ));
            if (decoded.size() == 1UZ) {
                expect(eq(decoded[0].timestamp, std::int64_t{569'524'843'000'000'000}));
                expect(eq(metaString(decoded[0], "ccsds_time_scale"), std::string("tai"))) << "the code is TAI-based and the offset is zero, so the scale is named TAI";
                expect(eq(metaString(decoded[0], "ccsds_time_code"), std::string("cuc")));
            }
            expect(eq(decoder.nDecoded, std::uint64_t{1}));
        }
        // CDS with the 1958 epoch: day 10974, ms 62 443 123, microsecond 456
        {
            auto   encoder = make<TimeCodeEncode>({{"code", std::string("cds")}, {"day_octets", gr::Size_t{2}}, {"submillisecond_octets", gr::Size_t{2}}});
            auto   decoder = make<TimeCodeDecode>({{"code", std::string("cds")}});
            Record source;
            source.timestamp = kWorkedInstantNs;
            source.signal_values.clear();
            const std::vector<Record> encoded = drive1(encoder, std::vector<Record>{source});
            expect(eq(encoded.size(), 1UZ));
            if (encoded.size() == 1UZ) {
                const std::vector<Record> decoded = drive1(decoder, encoded);
                expect(eq(decoded.size(), 1UZ));
                if (decoded.size() == 1UZ) {
                    expect(eq(decoded[0].timestamp, kWorkedInstantNs));
                }
            }
        }
        // CUC: coarse count 948 216 043 seconds from the 1958 epoch, TAI scale (tai_utc_offset_s = 0)
        {
            auto   encoder = make<TimeCodeEncode>({{"code", std::string("cuc")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}});
            auto   decoder = make<TimeCodeDecode>({{"code", std::string("cuc")}});
            Record source;
            source.timestamp                  = 948'216'043LL * 1'000'000'000LL - gr::ccsds::kEpoch1958Ns;
            const std::vector<Record> encoded = drive1(encoder, std::vector<Record>{source});
            expect(eq(encoded.size(), 1UZ));
            if (encoded.size() == 1UZ) {
                const std::vector<Record> decoded = drive1(decoder, encoded);
                expect(eq(decoded.size(), 1UZ));
                if (decoded.size() == 1UZ) {
                    expect(eq(decoded[0].timestamp, source.timestamp));
                    expect(eq(metaString(decoded[0], "ccsds_time_scale"), std::string("tai")));
                }
            }
        }
    };

    "settings refused rather than ignored"_test = [] {
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cds")}, {"tai_utc_offset_s", std::int32_t{1}}}); }));
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("ascii_a")}, {"p_field", std::string("implicit")}}); }));
        // A P-field name staged for an ASCII code is refused even where it names the harmless default:
        // 3.5.2 gives ASCII no P-field to be explicit about either, and the setting distinguishes "never
        // staged" from "staged as 'explicit'" so the two cases are not both silently accepted.
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("ascii_a")}, {"p_field", std::string("explicit")}}); }));
        expect(nothrow([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("ascii_a")}}); })) << "an ASCII code that never names p_field decodes under the unstaged default";
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("tai1958")}}); })) << "code is required and must be one of the five names";
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cds")}, {"p_field", std::string("implicit")}, {"fine_octets", gr::Size_t{2}}}); }));
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"p_field", std::string("implicit")}, {"coarse_octets", gr::Size_t{8}}, {"fine_octets", gr::Size_t{0}}}); }));
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("ccs")}, {"p_field", std::string("implicit")}, {"subsecond_octets", gr::Size_t{7}}}); }));
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cds")}, {"epoch", std::string("custom")}}); })) << "epoch_ns is required when epoch == custom";

        // epoch_ns given where the epoch is the recommended one is a contradiction and says so.
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"epoch_ns", std::int64_t{-631'152'000'000'000'000}}}); }));
        expect(throws([] { std::ignore = make<TimeCodeEncode>({{"code", std::string("cds")}, {"day_octets", gr::Size_t{2}}, {"submillisecond_octets", gr::Size_t{0}}, {"epoch_ns", std::int64_t{1}}}); }));

        // Zero is a legal custom epoch -- it is the Unix epoch -- so "unset" is a value outside the
        // axis and not a value on it. A block that read zero as absence could not express this.
        expect(nothrow([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cds")}, {"epoch", std::string("custom")}, {"epoch_ns", std::int64_t{0}}}); }));
        {
            auto                      unixEpoch = make<TimeCodeDecode>({{"code", std::string("cds")}, {"epoch", std::string("custom")}, {"epoch_ns", std::int64_t{0}}});
            const std::vector<Record> wire{recordOf({0x48U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U})};
            const std::vector<Record> decoded = drive1(unixEpoch, wire);
            expect(eq(decoded.size(), 1UZ));
            if (decoded.size() == 1UZ) {
                expect(eq(decoded[0].timestamp, std::int64_t{86'400'000'000'000})) << "day 1 of a code whose epoch is the Unix epoch";
            }
        }

        // A setting that names a fact the chosen code does not have is refused by name, never ignored.
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"day_of_year", true}}); })) << "day_of_year is the calendar code's variation";
        expect(throws([] { std::ignore = make<TimeCodeEncode>({{"code", std::string("cuc")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}, {"day_of_year", true}}); }));
        expect(throws([] { std::ignore = make<TimeCodeEncode>({{"code", std::string("cuc")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}, {"fraction_digits", gr::Size_t{3}}}); })) << "only the ASCII codes write a decimal fraction";
        expect(throws([] { std::ignore = make<TimeCodeEncode>({{"code", std::string("cuc")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}, {"terminator", false}}); })) << "only the ASCII codes have the trailing Z";
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("ccs")}, {"epoch", std::string("custom")}, {"epoch_ns", std::int64_t{5}}}); })) << "a calendar code counts from no epoch";
        expect(throws([] { std::ignore = make<TimeCodeDecode>({{"code", std::string("ascii_a")}, {"epoch_ns", std::int64_t{5}}}); }));
        expect(throws([] { std::ignore = make<TimeCodeEncode>({{"code", std::string("ascii_b")}, {"epoch", std::string("custom")}, {"epoch_ns", std::int64_t{5}}}); }));

        // A block that never reached a configuration is inert rather than wrong: it consumes nothing,
        // publishes nothing and says so.
        {
            TimeCodeDecode            decoder;
            std::size_t               published = 1UZ;
            const std::vector<Record> one{recordOf({0x1CU, 0x00U, 0x00U, 0x00U, 0x00U})};
            expect(driveStatus(decoder, one, published) == gr::work::Status::ERROR);
            expect(eq(published, 0UZ));
            expect(eq(decoder.nDecoded, std::uint64_t{0}));
        }
        {
            TimeCodeEncode            encoder;
            std::size_t               published = 1UZ;
            const std::vector<Record> one{recordOf({0x00U})};
            expect(driveStatus(encoder, one, published) == gr::work::Status::ERROR);
            expect(eq(published, 0UZ));
            expect(eq(encoder.nEncoded, std::uint64_t{0}));
        }
    };

    "a wire P-field that contradicts the settings is refused and counted"_test = [] {
        // The settings say CUC and the octet on the wire says CDS. Decoding under the wire's reading
        // would leave `ccsds_time_code` naming a code the instant did not come out of, and decoding
        // under the setting's would read a field of one width as a field of another.
        auto   wrongKind                  = make<TimeCodeDecode>({{"code", std::string("cuc")}});
        Record cds                        = recordOf({0x40U, 0x2AU, 0xDEU, 0x03U, 0xB8U, 0x67U, 0xB3U});
        cds.timestamp                     = kWorkedInstantNs;
        const std::vector<Record> kindOut = drive1(wrongKind, std::vector<Record>{cds});
        expect(eq(kindOut.size(), 1UZ)) << "published, because require_time is false";
        expect(eq(wrongKind.nPFieldMismatch, std::uint64_t{1}));
        expect(eq(wrongKind.nDecoded, std::uint64_t{0}));
        if (kindOut.size() == 1UZ) {
            expect(eq(kindOut[0].timestamp, kWorkedInstantNs)) << "untouched, because nothing decoded";
            expect(!metaHas(kindOut[0], "ccsds_time_code"));
        }

        // And the epoch: identification 010 is the agency-defined epoch, and the settings name the
        // recommended one. Decoding it against 1958 would be wrong by however far the two epochs are.
        auto                      wrongEpoch = make<TimeCodeDecode>({{"code", std::string("cuc")}});
        const std::vector<Record> agency{recordOf({0x2CU, 0x38U, 0x84U, 0xA0U, 0xEBU})};
        const std::vector<Record> epochOut = drive1(wrongEpoch, agency);
        expect(eq(epochOut.size(), 1UZ));
        expect(eq(wrongEpoch.nPFieldMismatch, std::uint64_t{1}));
        expect(eq(wrongEpoch.nDecoded, std::uint64_t{0}));

        // The same octet with the epoch the graph actually configured decodes, so the refusal is about
        // the disagreement and not about the identification.
        auto                      agreed    = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"epoch", std::string("custom")}, {"epoch_ns", std::int64_t{0}}});
        const std::vector<Record> agreedOut = drive1(agreed, agency);
        expect(eq(agreed.nPFieldMismatch, std::uint64_t{0}));
        expect(eq(agreed.nDecoded, std::uint64_t{1}));
        if (agreedOut.size() == 1UZ) {
            expect(eq(agreedOut[0].timestamp, 948'216'043LL * 1'000'000'000LL)) << "counted from the Unix epoch, which is what the graph said";
        }

        // A record too short to hold the P-field it declares is short of the code, not short of a field
        // that was read out of one.
        auto                      shortPField = make<TimeCodeDecode>({{"code", std::string("cuc")}});
        const std::vector<Record> stub{recordOf({0x9CU})};
        expect(eq(drive1(shortPField, stub).size(), 1UZ));
        expect(eq(shortPField.nShortRecord, std::uint64_t{1}));
        expect(eq(shortPField.nShortField, std::uint64_t{0}));
    };

    "strip removes the code and nothing beyond it"_test = [] {
        // The ASCII codes have no declared length, so the code ends at the NUL that closes it or at the
        // end of the payload where there is none. A NUL belongs to the code it terminates.
        const std::string text{"1988-01-18T17:20:43Z"};
        expect(eq(text.size(), 20UZ));

        std::vector<std::uint8_t> padded(text.begin(), text.end());
        padded.push_back(0x00U);
        padded.push_back(0xDEU);
        padded.push_back(0xADU);

        auto                      stripper = make<TimeCodeDecode>({{"code", std::string("ascii_a")}, {"strip", true}});
        const std::vector<Record> out      = drive1(stripper, std::vector<Record>{recordOf(padded)});
        expect(eq(out.size(), 1UZ));
        if (out.size() == 1UZ) {
            expect(eq(out[0].timestamp, std::int64_t{569'524'843'000'000'000}));
            expect(eq(out[0].signal_values.size(), 2UZ)) << "the twenty characters and the NUL that closed them, and no more";
            if (out[0].signal_values.size() == 2UZ) {
                expect(eq(static_cast<unsigned>(out[0].signal_values[0]), 0xDEU));
                expect(eq(static_cast<unsigned>(out[0].signal_values[1]), 0xADU));
            }
        }

        // With no NUL the code is the whole remainder, and stripping it leaves an empty payload.
        auto                      exact = make<TimeCodeDecode>({{"code", std::string("ascii_a")}, {"strip", true}});
        const std::vector<Record> bare  = drive1(exact, std::vector<Record>{recordOf(std::vector<std::uint8_t>(text.begin(), text.end()))});
        expect(eq(bare.size(), 1UZ));
        if (bare.size() == 1UZ) {
            expect(eq(bare[0].signal_values.size(), 0UZ));
        }

        // A binary code's length is declared, so what strip removes is the P-field and the T-field.
        auto                      binary = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"offset", gr::Size_t{2}}, {"strip", true}});
        const std::vector<Record> framed = drive1(binary, std::vector<Record>{recordOf({0x11U, 0x22U, 0x1CU, 0x38U, 0x84U, 0xA0U, 0xEBU, 0x33U})});
        expect(eq(framed.size(), 1UZ));
        if (framed.size() == 1UZ) {
            expect(eq(framed[0].timestamp, std::int64_t{569'524'843'000'000'000}));
            const std::vector<std::uint8_t> remainder{0x11U, 0x22U, 0x33U};
            expect(std::ranges::equal(framed[0].signal_values, remainder)) << "the octets before the offset and the octets after the code";
        }
    };

    "require_time both ways, and the zero timestamp"_test = [] {
        // An implicit layout declares four octets and the record holds two, so the payload does not
        // reach `offset + t_field_octets`: the record is short of the code, which is nShortRecord.
        auto lenient = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"p_field", std::string("implicit")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}, {"require_time", false}});
        auto strict  = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"p_field", std::string("implicit")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}, {"require_time", true}});

        static constexpr std::int64_t kUntouched = 4'242'424'242LL;
        Record                        shortOne   = recordOf(std::vector<std::uint8_t>{0x01, 0x02}, gr::property_map{{"probe", std::string("carried")}});
        shortOne.timestamp                       = kUntouched;
        const std::vector<Record> shortRecords{shortOne};

        const std::vector<Record> lenientOut = drive1(lenient, shortRecords);
        expect(eq(lenientOut.size(), 1UZ)) << "published even though the code did not decode";
        expect(eq(lenient.nShortRecord, std::uint64_t{1}));
        expect(eq(lenient.nShortField, std::uint64_t{0})) << "the length was known before any field was read";
        if (lenientOut.size() == 1UZ) {
            expect(eq(lenientOut[0].timestamp, kUntouched)) << "data rides, status judges: the carrier field is left alone";
            expect(!metaHas(lenientOut[0], "ccsds_time_code")) << "and no provenance is claimed for an instant that was never read";
            expect(eq(metaString(lenientOut[0], "probe"), std::string("carried")));
        }

        const std::vector<Record> strictOut = drive1(strict, shortRecords);
        expect(eq(strictOut.size(), 0UZ)) << "a counted drop rather than a pass-through";
        expect(eq(strict.nShortRecord, std::uint64_t{1}));

        // A record that follows a dropped one is still processed, so the drop is a drop and not a stop.
        auto                      mixed = make<TimeCodeDecode>({{"code", std::string("cuc")}, {"require_time", true}});
        const std::vector<Record> pair{recordOf({0x1CU, 0x38U}), recordOf({0x1CU, 0x38U, 0x84U, 0xA0U, 0xEBU})};
        const std::vector<Record> mixedOut = drive1(mixed, pair);
        expect(eq(mixedOut.size(), 1UZ));
        expect(eq(mixed.nShortRecord, std::uint64_t{1}));
        expect(eq(mixed.nDecoded, std::uint64_t{1}));
        if (mixedOut.size() == 1UZ) {
            expect(eq(mixedOut[0].timestamp, std::int64_t{569'524'843'000'000'000}));
        }

        // A timestamp of zero is the Unix epoch, a legal instant: it is encoded, not skipped, and the
        // octets it produces are the 1958 epoch's own distance from it, 378 691 200 s.
        auto   encoder = make<TimeCodeEncode>({{"code", std::string("cuc")}, {"p_field", std::string("implicit")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}});
        Record zero;
        zero.timestamp                = 0;
        const std::vector<Record> out = drive1(encoder, std::vector<Record>{zero});
        expect(eq(out.size(), 1UZ)) << "a zero timestamp is the Unix epoch, a legal instant, and is not skipped";
        if (out.size() == 1UZ) {
            const std::vector<std::uint8_t> epochOctets{0x16U, 0x92U, 0x5EU, 0x80U};
            expect(std::ranges::equal(out[0].signal_values, epochOctets)) << "378 691 200 seconds, big-endian, and not an empty or skipped field";
            expect(eq(out[0].timestamp, std::int64_t{0}));
        }
        expect(eq(encoder.nEncoded, std::uint64_t{1}));
    };

    "the chain under the scheduler, five codes at three chunk sizes"_test = [] {
        using gr::blocks::ccsds::SpacePacketDecode;
        using gr::blocks::ccsds::SpacePacketEncode;

        struct Arm {
            const char*      what;
            gr::property_map encode;
            gr::property_map decode;
            bool             ownsPayload; // an ASCII code has no declared length, so it runs to the end
            bool             stripped;
        };

        // Every layout below carries whole seconds, and the seeded instants are whole seconds, so each
        // code reproduces the timestamp exactly and no arm is measuring its own truncation. Three of
        // the five put the P-field on the wire, where the decoder reads the layout back out of it.
        const std::vector<Arm> arms{
            Arm{"cuc, P-field on the wire", gr::property_map{{"code", std::string("cuc")}, {"p_field", std::string("explicit")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}}, gr::property_map{{"code", std::string("cuc")}, {"p_field", std::string("explicit")}}, false, false},
            Arm{"cuc, layout from settings", gr::property_map{{"code", std::string("cuc")}, {"p_field", std::string("implicit")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}}, gr::property_map{{"code", std::string("cuc")}, {"p_field", std::string("implicit")}, {"coarse_octets", gr::Size_t{4}}, {"fine_octets", gr::Size_t{0}}}, false, false},
            Arm{"cds, P-field on the wire, stripped on the way out", gr::property_map{{"code", std::string("cds")}, {"day_octets", gr::Size_t{2}}, {"submillisecond_octets", gr::Size_t{0}}}, gr::property_map{{"code", std::string("cds")}, {"strip", true}}, false, true},
            Arm{"ccs, P-field on the wire", gr::property_map{{"code", std::string("ccs")}, {"subsecond_octets", gr::Size_t{0}}}, gr::property_map{{"code", std::string("ccs")}}, false, false},
            Arm{"ascii_a", gr::property_map{{"code", std::string("ascii_a")}}, gr::property_map{{"code", std::string("ascii_a")}}, true, false},
            Arm{"ascii_b", gr::property_map{{"code", std::string("ascii_b")}}, gr::property_map{{"code", std::string("ascii_b")}}, true, false},
        };

        const std::int64_t kWholeSecondBase = (kWorkedInstantNs / 1'000'000'000LL) * 1'000'000'000LL;
        for (const Arm& arm : arms) {
            for (const std::size_t chunk : {std::size_t{1}, std::size_t{17}, std::size_t{4096}}) {
                std::vector<Record> payloads;
                for (int i = 0; i < 12; ++i) {
                    std::vector<std::uint8_t> bytes(arm.ownsPayload ? 0UZ : 8UZ, static_cast<std::uint8_t>(i));
                    Record                    record = recordOf(bytes, gr::property_map{{"probe", std::string("carried")}});
                    record.timestamp                 = kWholeSecondBase + static_cast<std::int64_t>(i) * 1'000'000'000LL;
                    payloads.push_back(record);
                }

                gr::Graph flow;
                auto&     source = flow.emplaceBlock<RecordSource>();
                source._records  = payloads;
                source._chunk    = chunk;
                auto& timeEncode = flow.emplaceBlock<TimeCodeEncode>(arm.encode);
                auto& pktEncode  = flow.emplaceBlock<SpacePacketEncode>(gr::property_map{{"apid", gr::Size_t{42}}});
                auto& pktDecode  = flow.emplaceBlock<SpacePacketDecode>();
                auto& timeDecode = flow.emplaceBlock<TimeCodeDecode>(arm.decode);
                auto& sink       = flow.emplaceBlock<RecordSink>();

                expect(flow.connect<"out", "in">(source, timeEncode).has_value());
                expect(flow.connect<"out", "in">(timeEncode, pktEncode).has_value());
                expect(flow.connect<"out", "in">(pktEncode, pktDecode).has_value());
                expect(flow.connect<"out", "in">(pktDecode, timeDecode).has_value());
                expect(flow.connect<"out", "in">(timeDecode, sink).has_value());

                std::vector<Record> received;
                runGraph(std::move(flow), [&received, &sink] { received = sink._records; });

                expect(eq(received.size(), payloads.size())) << arm.what << " at chunk " << chunk;
                for (std::size_t i = 0UZ; i < received.size() && i < payloads.size(); ++i) {
                    expect(eq(received[i].timestamp, payloads[i].timestamp)) << arm.what << " record " << i << " at chunk " << chunk;
                    expect(eq(metaString(received[i], "probe"), std::string("carried"))) << arm.what << ": an unrelated key crosses both blocks verbatim";
                    if (arm.stripped) {
                        expect(std::ranges::equal(received[i].signal_values, payloads[i].signal_values)) << arm.what << " record " << i << ": strip leaves exactly what arrived";
                    }
                }
            }
        }
    };
};

int main() { /* not needed for UT */ }
