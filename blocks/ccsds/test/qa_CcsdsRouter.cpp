#include <boost/ut.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/ccsds/FieldRouter.hpp>
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

[[nodiscard]] Record recordOf(gr::property_map meta = {}) {
    Record record;
    record.signal_values = {0x01U};
    record.extents.push_back(1);
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

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

} // namespace

const boost::ut::suite<"CcsdsRouter"> ccsdsRouterTests = [] {
    using namespace boost::ut;
    using gr::blocks::ccsds::FieldRouter;

    "the router routes and does not invent"_test = [] {
        auto router = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{100U, 200U}}});

        const std::vector<Record> records{
            recordOf({{"ccsds_apid", gr::Size_t{100}}}), recordOf({{"ccsds_apid", gr::Size_t{200}}}), recordOf({{"ccsds_apid", gr::Size_t{300}}}), recordOf({}), // no key at all
            recordOf({{"ccsds_apid", std::string("nope")}}),                                                                                                     // wrong type
        };

        std::vector<Record>             out0(4UZ);
        std::vector<Record>             out1(4UZ);
        std::vector<Record>             otherBuf(4UZ);
        InputSpan<Record>               inSpan{std::span<const Record>(records)};
        std::vector<OutputSpan<Record>> outs{OutputSpan<Record>{std::span<Record>(out0)}, OutputSpan<Record>{std::span<Record>(out1)}};
        std::span<OutputSpan<Record>>   outsSpan(outs);
        OutputSpan<Record>              otherSpan{std::span<Record>(otherBuf)};

        std::ignore = router.processBulk(inSpan, outsSpan, otherSpan);

        expect(eq(outs[0].count, std::size_t{1}));
        expect(eq(outs[1].count, std::size_t{1}));
        expect(eq(otherSpan.count, std::size_t{3}));
        expect(eq(router.nRouted[0], std::uint64_t{1}));
        expect(eq(router.nRouted[1], std::uint64_t{1}));
        expect(eq(router.nOther, std::uint64_t{1}));
        expect(eq(router.nMissingKey, std::uint64_t{2})) << "the absent key and the wrong-typed key both read as absent";
    };

    "an unmatched record is a counted drop when other is unconnected"_test = [] {
        auto router = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{100U}}});

        const std::vector<Record> records{recordOf({{"ccsds_apid", gr::Size_t{100}}}), recordOf({{"ccsds_apid", gr::Size_t{999}}})};

        std::vector<Record>             out0(4UZ);
        std::vector<Record>             otherBuf(4UZ);
        InputSpan<Record>               inSpan{std::span<const Record>(records)};
        std::vector<OutputSpan<Record>> outs{OutputSpan<Record>{std::span<Record>(out0)}};
        std::span<OutputSpan<Record>>   outsSpan(outs);
        OutputSpan<Record>              otherSpan{std::span<Record>(otherBuf), 0UZ, nullptr, false}; // unconnected

        std::ignore = router.processBulk(inSpan, outsSpan, otherSpan);

        expect(eq(outs[0].count, std::size_t{1}));
        expect(eq(otherSpan.count, std::size_t{0}));
        expect(eq(router.nOther, std::uint64_t{1})) << "still counted even though it vanishes";
    };

    "virtual_channel routes on ccsds_vcid"_test = [] {
        auto router = make<FieldRouter>({{"field", std::string("virtual_channel")}, {"values", std::vector<gr::Size_t>{0U, 3U}}});

        const std::vector<Record>       records{recordOf({{"ccsds_vcid", gr::Size_t{3}}})};
        std::vector<Record>             out0(4UZ);
        std::vector<Record>             out1(4UZ);
        std::vector<Record>             otherBuf(4UZ);
        InputSpan<Record>               inSpan{std::span<const Record>(records)};
        std::vector<OutputSpan<Record>> outs{OutputSpan<Record>{std::span<Record>(out0)}, OutputSpan<Record>{std::span<Record>(out1)}};
        std::span<OutputSpan<Record>>   outsSpan(outs);
        OutputSpan<Record>              otherSpan{std::span<Record>(otherBuf)};
        std::ignore = router.processBulk(inSpan, outsSpan, otherSpan);
        expect(eq(outs[1].count, std::size_t{1}));
        expect(eq(outs[0].count, std::size_t{0}));
    };

    "a routed record crosses whole and gains nothing"_test = [] {
        auto router = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{100U}}});

        const gr::property_map    matched{{"ccsds_apid", gr::Size_t{100}}, {"protocol", std::string("ccsds/space_packet")}, {"unrelated_key", std::string("kept")}};
        const gr::property_map    unmatched{{"ccsds_apid", gr::Size_t{7}}, {"unrelated_key", std::string("kept too")}};
        const gr::property_map    keyless{{"unrelated_key", std::string("kept as well")}};
        const std::vector<Record> records{recordOf(matched), recordOf(unmatched), recordOf(keyless)};

        std::vector<Record>             out0(4UZ);
        std::vector<Record>             otherBuf(4UZ);
        InputSpan<Record>               inSpan{std::span<const Record>(records)};
        std::vector<OutputSpan<Record>> outs{OutputSpan<Record>{std::span<Record>(out0)}};
        std::span<OutputSpan<Record>>   outsSpan(outs);
        OutputSpan<Record>              otherSpan{std::span<Record>(otherBuf)};

        std::ignore = router.processBulk(inSpan, outsSpan, otherSpan);

        expect(eq(outs[0].count, std::size_t{1}));
        expect(eq(otherSpan.count, std::size_t{2}));
        expect(eq(metaOf(out0[0]).size(), matched.size())) << "the router writes no key of its own";
        expect(eq(metaSize(out0[0], "ccsds_apid"), gr::Size_t{100}));
        expect(eq(metaString(out0[0], "protocol"), std::string("ccsds/space_packet")));
        expect(eq(metaString(out0[0], "unrelated_key"), std::string("kept")));
        expect(that % (out0[0].signal_values == records[0].signal_values));
        expect(eq(metaString(otherBuf[0], "unrelated_key"), std::string("kept too")));
        expect(eq(metaOf(otherBuf[1]).size(), keyless.size()));
        expect(!metaHas(otherBuf[1], "ccsds_apid")) << "a record whose key is absent is never assigned one";
        expect(eq(router.nMissingKey, std::uint64_t{1}));
    };

    "a full output port holds a record back without counting it twice"_test = [] {
        auto router = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{100U}}});

        const std::vector<Record> records{recordOf({{"ccsds_apid", gr::Size_t{300}}}), recordOf({}), recordOf({{"ccsds_apid", gr::Size_t{100}}})};

        std::vector<Record>             out0(4UZ);
        std::vector<Record>             otherBuf(1UZ); // room for one of the two records bound for `other`
        InputSpan<Record>               inSpan{std::span<const Record>(records)};
        std::vector<OutputSpan<Record>> outs{OutputSpan<Record>{std::span<Record>(out0)}};
        std::span<OutputSpan<Record>>   outsSpan(outs);
        OutputSpan<Record>              otherSpan{std::span<Record>(otherBuf)};
        std::ignore = router.processBulk(inSpan, outsSpan, otherSpan);

        expect(eq(otherSpan.count, std::size_t{1}));
        expect(eq(inSpan.consumed, std::size_t{1})) << "the record with no room stays unconsumed";
        expect(eq(router.nOther, std::uint64_t{1}));
        expect(eq(router.nMissingKey, std::uint64_t{0})) << "the held-back record is counted on the call that routes it, not on the one that could not";

        // the same span again, with room this time: the two held-back records go through and are counted once
        std::vector<Record>             otherBuf2(4UZ);
        InputSpan<Record>               inSpan2{std::span<const Record>(records).subspan(1UZ)};
        std::vector<OutputSpan<Record>> outs2{OutputSpan<Record>{std::span<Record>(out0)}};
        std::span<OutputSpan<Record>>   outsSpan2(outs2);
        OutputSpan<Record>              otherSpan2{std::span<Record>(otherBuf2)};
        std::ignore = router.processBulk(inSpan2, outsSpan2, otherSpan2);

        expect(eq(outs2[0].count, std::size_t{1}));
        expect(eq(otherSpan2.count, std::size_t{1}));
        expect(eq(router.nMissingKey, std::uint64_t{1}));
        expect(eq(router.nOther, std::uint64_t{1}));
        expect(eq(router.nRouted[0], std::uint64_t{1}));
    };

    "router configuration refusals"_test = [] {
        expect(throws([] { std::ignore = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{}}}); })) << "an empty values is refused";
        expect(throws([] { std::ignore = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{1U, 1U}}}); })) << "a duplicated value is refused";
        expect(throws([] { std::ignore = make<FieldRouter>({{"field", std::string("apid")}, {"values", std::vector<gr::Size_t>{2048U}}}); })) << "a value above 2047 under apid is refused";
        expect(throws([] { std::ignore = make<FieldRouter>({{"field", std::string("virtual_channel")}, {"values", std::vector<gr::Size_t>{64U}}}); })) << "a value above 63 under virtual_channel is refused";
        expect(throws([] { std::ignore = make<FieldRouter>({{"field", std::string("bogus")}, {"values", std::vector<gr::Size_t>{1U}}}); })) << "an unrecognized field is refused";
        expect(throws([] { std::ignore = make<FieldRouter>({{"values", std::vector<gr::Size_t>{1U}}}); })) << "an unset field is refused";
    };
};

int main() { /* not needed for UT */ }
