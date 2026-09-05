#include <boost/ut.hpp>

#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>

#include <gnuradio-4.0/basic/RecordUtils.hpp>

/*
 * The two record utilities own an item span and a routing decision and nothing else, so the tests
 * drive processBulk directly: every record's fate — trimmed, passed, or rejected with its stated
 * reason — is asserted by value, and the counters are read off the block afterwards.
 */
namespace {

using gr::blocks::basic::RecordLengthFilter;
using gr::blocks::basic::RecordTrim;
using Record = gr::DataSet<std::uint8_t>;

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

using TagView = std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>;

template<typename T>
struct InputSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    bool          isConnected = true;
    bool          isSync      = true;

    explicit InputSpan(std::span<const T> items) : std::span<const T>(items) {}
    constexpr bool consume(std::size_t n) noexcept {
        consumed = n;
        return true;
    }
    constexpr void                     consumeTags(std::size_t) noexcept {}
    [[nodiscard]] std::vector<TagView> tags() const { return {}; }
    [[nodiscard]] std::vector<TagView> tags(std::size_t) const { return {}; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
    constexpr void publish(std::size_t) const noexcept {}
};

template<typename T>
struct OutputSpan : std::span<T> {
    using value_type = T;

    TagWriterSpan tags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   count       = 0UZ;
    bool          isConnected = true;
    bool          isSync      = false;

    explicit OutputSpan(std::span<T> items, bool connected = true) : std::span<T>(items), isConnected(connected) {}
    constexpr void publish(std::size_t n) noexcept { count = n; }
    void           publishTag(const gr::property_map&, std::size_t = 0UZ) {}
};

[[nodiscard]] Record record(std::size_t items) {
    Record r;
    r.signal_values.resize(items);
    for (std::size_t i = 0UZ; i < items; ++i) {
        r.signal_values[i] = static_cast<std::uint8_t>((i * 37UZ + 11UZ) & 0xFFUZ);
    }
    r.extents.push_back(static_cast<std::int32_t>(items));
    r.signal_names.emplace_back("basic");
    r.timing_events.resize(1UZ);
    r.meta_information.resize(1UZ);
    r.meta_information[0UZ]["origin"] = std::string("qa");
    return r;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Fate {
    std::vector<Record> out{};
    std::vector<Record> rejected{};
};

template<typename TBlock>
[[nodiscard]] Fate through(TBlock& block, const std::vector<Record>& records, bool rejectConnected = true) {
    std::vector<Record> outBuf(records.size() + 1UZ);
    std::vector<Record> rejectBuf(records.size() + 1UZ);
    InputSpan<Record>   inSpan{std::span<const Record>(records)};
    OutputSpan<Record>  outSpan{std::span<Record>(outBuf)};
    OutputSpan<Record>  rejectSpan{rejectConnected ? std::span<Record>(rejectBuf) : std::span<Record>{}, rejectConnected};
    std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);

    Fate fate;
    fate.out.assign(outBuf.begin(), outBuf.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    fate.rejected.assign(rejectBuf.begin(), rejectBuf.begin() + static_cast<std::ptrdiff_t>(rejectSpan.count));
    return fate;
}

[[nodiscard]] std::string reasonOf(const Record& r) {
    if (r.meta_information.empty()) {
        return {};
    }
    const auto entry = r.meta_information[0UZ].find(gr::property_map::key_type("discard_reason"));
    return entry == r.meta_information[0UZ].end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

} // namespace

int main() {
    using namespace boost::ut;

    "the trim keeps items [drop_head, n - drop_tail), exactly"_test = [] {
        struct Case {
            std::size_t items;
            gr::Size_t  head;
            gr::Size_t  tail;
        };
        for (const Case c : {Case{20UZ, 3U, 5U}, Case{20UZ, 0U, 7U}, Case{20UZ, 9U, 0U}, Case{20UZ, 0U, 0U}, Case{12UZ, 6U, 6U}}) {
            RecordTrim block = make<RecordTrim>({{"drop_head", c.head}, {"drop_tail", c.tail}});
            const auto in    = record(c.items);
            const Fate fate  = through(block, {in});
            expect(eq(fate.out.size(), 1UZ)) << c.items << c.head << c.tail;
            if (fate.out.empty()) {
                continue;
            }
            const std::size_t kept = c.items - c.head - c.tail;
            expect(eq(fate.out[0UZ].signal_values.size(), kept));
            expect(that % std::equal(fate.out[0UZ].signal_values.begin(), fate.out[0UZ].signal_values.end(), in.signal_values.begin() + static_cast<std::ptrdiff_t>(c.head))) << "the kept span is the middle, byte for byte";
            expect(eq(fate.out[0UZ].extents[0UZ], static_cast<std::int32_t>(kept))) << "the extent names the trimmed length";
            const auto origin = fate.out[0UZ].meta_information[0UZ].find(gr::property_map::key_type("origin"));
            expect(that % (origin != fate.out[0UZ].meta_information[0UZ].end())) << "the record's facts cross verbatim";
        }
    };

    "a record shorter than the trims is a counted, stated drop and the next one trims"_test = [] {
        RecordTrim block = make<RecordTrim>({{"drop_head", gr::Size_t{8U}}, {"drop_tail", gr::Size_t{5U}}});
        const Fate fate  = through(block, {record(12UZ), record(13UZ), record(20UZ)});
        expect(eq(fate.out.size(), 2UZ)) << "13 items trims to an empty record, which is published";
        if (fate.out.size() == 2UZ) {
            expect(eq(fate.out[0UZ].signal_values.size(), 0UZ)) << "exactly as long as the trims is empty, not refused";
            expect(eq(fate.out[1UZ].signal_values.size(), 7UZ));
        }
        expect(eq(fate.rejected.size(), 1UZ));
        if (!fate.rejected.empty()) {
            expect(eq(reasonOf(fate.rejected[0UZ]), std::string("shorter_than_trim")));
        }
        expect(eq(block.nRefusedShort, std::uint64_t{1ULL}));
        expect(eq(block.nRecords, std::uint64_t{2ULL}));
    };

    "the length filter's bounds are inclusive on both sides"_test = [] {
        RecordLengthFilter block = make<RecordLengthFilter>({{"min_items", gr::Size_t{4U}}, {"max_items", gr::Size_t{8U}}});
        const Fate         fate  = through(block, {record(3UZ), record(4UZ), record(8UZ), record(9UZ)});
        expect(eq(fate.out.size(), 2UZ));
        if (fate.out.size() == 2UZ) {
            expect(eq(fate.out[0UZ].signal_values.size(), 4UZ));
            expect(eq(fate.out[1UZ].signal_values.size(), 8UZ));
        }
        expect(eq(fate.rejected.size(), 2UZ));
        if (fate.rejected.size() == 2UZ) {
            expect(eq(reasonOf(fate.rejected[0UZ]), std::string("length_below_min")));
            expect(eq(reasonOf(fate.rejected[1UZ]), std::string("length_above_max")));
        }
        expect(eq(block.nRefusedShort, std::uint64_t{1ULL}));
        expect(eq(block.nRefusedLong, std::uint64_t{1ULL}));

        // an unconnected reject keeps the counts and stalls nothing
        RecordLengthFilter quiet     = make<RecordLengthFilter>({{"min_items", gr::Size_t{4U}}, {"max_items", gr::Size_t{8U}}});
        const Fate         discarded = through(quiet, {record(3UZ), record(5UZ)}, false);
        expect(eq(discarded.out.size(), 1UZ));
        expect(eq(discarded.rejected.size(), 0UZ));
        expect(eq(quiet.nRefusedShort, std::uint64_t{1ULL}));
    };

    "a refusal with no room on a connected reject waits for the next call instead of being lost"_test = [] {
        RecordTrim                block = make<RecordTrim>({{"drop_head", gr::Size_t{8U}}, {"drop_tail", gr::Size_t{5U}}});
        const std::vector<Record> records{record(4UZ), record(6UZ), record(20UZ)};

        std::vector<Record> outBuf(4UZ);
        std::vector<Record> rejectBuf(1UZ); // room for one of the two refusals

        const auto call = [&](std::span<const Record> offered) {
            InputSpan<Record>  inSpan{offered};
            OutputSpan<Record> outSpan{std::span<Record>(outBuf)};
            OutputSpan<Record> rejectSpan{std::span<Record>(rejectBuf)};
            std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
            return std::tuple{inSpan.consumed, outSpan.count, rejectSpan.count};
        };

        const auto [consumedFirst, madeFirst, refusedFirst] = call(std::span<const Record>(records));
        expect(eq(refusedFirst, 1UZ)) << "the port held one";
        expect(eq(consumedFirst, 1UZ)) << "and only the record it had room for was consumed";
        expect(eq(madeFirst, 0UZ));
        expect(eq(block.nRefusedShort, std::uint64_t{1ULL})) << "the second refusal is not counted before it is written";

        const auto [consumedSecond, madeSecond, refusedSecond] = call(std::span<const Record>(records).subspan(1UZ));
        expect(eq(refusedSecond, 1UZ)) << "the record that waited is refused on the next call";
        expect(eq(consumedSecond, 1UZ));
        expect(eq(block.nRefusedShort, std::uint64_t{2ULL}));

        const auto [consumedThird, madeThird, refusedThird] = call(std::span<const Record>(records).subspan(2UZ));
        expect(eq(madeThird, 1UZ)) << "and the long record behind them trims normally";
        expect(eq(consumedThird, 1UZ));
        expect(eq(refusedThird, 0UZ));
        expect(eq(block.nRecords, std::uint64_t{1ULL}));
    };

    "the length filter holds a rejection its port has no room for"_test = [] {
        RecordLengthFilter        block = make<RecordLengthFilter>({{"min_items", gr::Size_t{4U}}, {"max_items", gr::Size_t{8U}}});
        const std::vector<Record> records{record(2UZ), record(20UZ), record(5UZ)};

        std::vector<Record> outBuf(4UZ);
        std::vector<Record> rejectBuf(1UZ);
        InputSpan<Record>   inSpan{std::span<const Record>(records)};
        OutputSpan<Record>  outSpan{std::span<Record>(outBuf)};
        OutputSpan<Record>  rejectSpan{std::span<Record>(rejectBuf)};
        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
        expect(eq(rejectSpan.count, 1UZ));
        expect(eq(inSpan.consumed, 1UZ)) << "the over-long record stays in the input buffer";
        expect(eq(block.nRefusedShort, std::uint64_t{1ULL}));
        expect(eq(block.nRefusedLong, std::uint64_t{0ULL})) << "nothing is counted that was not written";

        const Fate rest = through(block, {records[1UZ], records[2UZ]});
        expect(eq(rest.rejected.size(), 1UZ)) << "the record that waited is refused with room to say so";
        expect(eq(rest.out.size(), 1UZ)) << "and the admissible record behind it passes";
        expect(eq(block.nRefusedLong, std::uint64_t{1ULL}));
    };

    "the filter's bound pair is checked before anything runs"_test = [] {
        expect(throws([] { std::ignore = make<RecordLengthFilter>({{"min_items", gr::Size_t{2U}}}); })) << "a ceiling of zero admits nothing and is the unset value";
        expect(throws([] { std::ignore = make<RecordLengthFilter>({{"min_items", gr::Size_t{9U}}, {"max_items", gr::Size_t{8U}}}); })) << "a pair that admits nothing names both bounds";
        expect(nothrow([] { std::ignore = make<RecordLengthFilter>({{"min_items", gr::Size_t{8U}}, {"max_items", gr::Size_t{8U}}}); })) << "one admissible length is a filter";
    };

    return 0;
}
