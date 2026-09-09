#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>

#include <gnuradio-4.0/basic/RecordMerge.hpp>

/*
 * The block owns an order and a set of counts and nothing else, so most of the assertions are about what did not
 * happen: no record was dropped, none was reordered within its input, and an input that offers nothing — because it
 * is silent or because it is not connected at all — neither stalls the block nor disturbs another input's order. The
 * mock spans drive processBulk directly, so a full output span is reachable on purpose rather than by luck, and one
 * graph test runs the same properties through the scheduler and real ports.
 */
namespace {

using gr::blocks::basic::RecordMerge;
using Record = gr::DataSet<std::uint8_t>;

using TagView = std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>;

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr TagReaderSpan(std::span<const gr::Tag> tags) : std::span<const gr::Tag>(tags) {}
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
    constexpr TagWriterSpan(std::span<gr::Tag> tags) : std::span<gr::Tag>(tags) {}
    constexpr void publish(std::size_t) const noexcept {}
};

template<typename T>
struct InputSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    bool          isConnected = true;
    bool          isSync      = false;

    InputSpan() = default;
    explicit InputSpan(std::span<const T> items, std::span<const gr::Tag> incoming = {}, bool connected = true) : std::span<const T>(items), rawTags(incoming), isConnected(connected) {}

    constexpr bool consume(std::size_t n) noexcept {
        consumed += n;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::vector<TagView> tags() const {
        std::vector<TagView> view;
        for (const gr::Tag& tag : rawTags) {
            view.emplace_back(static_cast<std::ptrdiff_t>(tag.index) - static_cast<std::ptrdiff_t>(streamIndex), std::cref(tag.map));
        }
        return view;
    }
    [[nodiscard]] std::vector<TagView> tags(std::size_t) const { return tags(); }
};

template<typename T>
struct OutputSpan : std::span<T> {
    using value_type = T;

    TagWriterSpan         tags{};
    std::vector<gr::Tag>* published   = nullptr;
    std::size_t           streamIndex = 0UZ;
    std::size_t           count       = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = false;

    explicit OutputSpan(std::span<T> items, std::vector<gr::Tag>* sink = nullptr, bool connected = true) : std::span<T>(items), published(sink), isConnected(connected) {}

    constexpr void publish(std::size_t n) noexcept { count = n; }
    void           publishTag(const gr::property_map& map, std::size_t offset = 0UZ) {
        if (published != nullptr) {
            published->push_back(gr::Tag{streamIndex + offset, map});
        }
    }
};

/// @brief A one-signal record whose single item is @p mark, which is how a test names it.
[[nodiscard]] Record record(std::uint8_t mark) {
    Record r;
    r.signal_values.push_back(mark);
    r.extents.push_back(1);
    r.signal_names.emplace_back("basic");
    r.timing_events.resize(1UZ);
    r.meta_information.resize(1UZ);
    return r;
}

[[nodiscard]] std::vector<Record> records(std::span<const std::uint8_t> marks) {
    std::vector<Record> out;
    out.reserve(marks.size());
    for (const std::uint8_t mark : marks) {
        out.push_back(record(mark));
    }
    return out;
}

/// @brief The single item of every record of @p list, which is the order the assertions are written in.
[[nodiscard]] std::vector<std::uint8_t> marksOf(std::span<const Record> list) {
    std::vector<std::uint8_t> marks;
    marks.reserve(list.size());
    for (const Record& r : list) {
        marks.push_back(r.signal_values.empty() ? std::uint8_t{0U} : r.signal_values.front());
    }
    return marks;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// @brief Everything one `processBulk` call published, with what each input span consumed.
struct Call {
    std::vector<Record>      out{};
    std::vector<gr::Tag>     tags{};
    std::vector<std::size_t> consumed{};
    gr::work::Status         status = gr::work::Status::OK;
};

/// @brief One call over @p ins, offering @p outRoom records of output room; 0 offers room for all of them.
[[nodiscard]] Call call(RecordMerge<std::uint8_t>& block, std::vector<InputSpan<Record>>& ins, std::size_t outRoom) {
    std::size_t offered = 0UZ;
    for (const auto& in : ins) {
        offered += in.size();
    }
    std::vector<Record>          outBuf(outRoom == 0UZ ? offered + 1UZ : outRoom);
    std::span<InputSpan<Record>> inSpans(ins);
    Call                         result;
    OutputSpan<Record>           outSpan{std::span<Record>(outBuf), &result.tags};
    result.status = block.processBulk(inSpans, outSpan);
    result.out.assign(outBuf.begin(), outBuf.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    for (const auto& in : ins) {
        result.consumed.push_back(in.consumed);
    }
    return result;
}

// ─── graph-side blocks, so that the same properties are asserted through real ports and the scheduler ──────────────

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);

    std::vector<Record> _records{};
    std::size_t         _emitted = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _emitted);
        for (std::size_t k = 0UZ; k < n; ++k) {
            outSpan[k] = _records[_emitted + k];
        }
        _emitted += n;
        outSpan.publish(n);
        return _emitted >= _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordCollector : gr::Block<RecordCollector> {
    gr::PortIn<Record> in;
    GR_MAKE_REFLECTABLE(RecordCollector, in);

    std::vector<Record> _records{};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _records.push_back(inSpan[k]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

} // namespace

int main() {
    using namespace boost::ut;
    using namespace gr;
    using namespace std::string_literals;

    "three inputs reach one output, each input's own order intact"_test = [] {
        auto block = make<RecordMerge<std::uint8_t>>({{"n_inputs", gr::Size_t{3U}}});

        const std::vector<std::uint8_t> firstMarks{10U, 11U, 12U, 13U};
        const std::vector<std::uint8_t> secondMarks{20U};
        const std::vector<std::uint8_t> thirdMarks{30U, 31U};
        const std::vector<Record>       first  = records(firstMarks);
        const std::vector<Record>       second = records(secondMarks);
        const std::vector<Record>       third  = records(thirdMarks);

        std::vector<InputSpan<Record>> ins;
        ins.emplace_back(std::span<const Record>(first));
        ins.emplace_back(std::span<const Record>(second));
        ins.emplace_back(std::span<const Record>(third));

        const Call result = call(block, ins, 0UZ);
        expect(eq(result.out.size(), 7UZ)) << "every record offered is published, none dropped";
        expect(eq(marksOf(result.out), std::vector<std::uint8_t>{10U, 11U, 12U, 13U, 20U, 30U, 31U})) << "port order across inputs, arrival order within one";
        expect(eq(result.consumed, std::vector<std::size_t>{4UZ, 1UZ, 2UZ})) << "each input is consumed exactly as far as it was published";
        expect(eq(block.n_records.value, std::vector<gr::Size_t>{4U, 1U, 2U})) << "one count per input, in port order";
    };

    "a full output span costs the call and never a record"_test = [] {
        auto block = make<RecordMerge<std::uint8_t>>({{"n_inputs", gr::Size_t{3U}}});

        const std::vector<std::uint8_t> firstMarks{10U, 11U, 12U, 13U};
        const std::vector<std::uint8_t> secondMarks{20U, 21U};
        const std::vector<Record>       first  = records(firstMarks);
        const std::vector<Record>       second = records(secondMarks);

        std::vector<InputSpan<Record>> ins;
        ins.emplace_back(std::span<const Record>(first));
        ins.emplace_back(std::span<const Record>(second));
        ins.emplace_back(std::span<const Record>{}); // declared, offering nothing

        const Call firstCall = call(block, ins, 3UZ);
        expect(eq(firstCall.out.size(), 3UZ));
        expect(eq(marksOf(firstCall.out), std::vector<std::uint8_t>{10U, 11U, 12U}));
        expect(eq(firstCall.consumed, std::vector<std::size_t>{3UZ, 0UZ, 0UZ})) << "what did not fit was not consumed, so it is not lost";
        expect(eq(block.n_records.value, std::vector<gr::Size_t>{3U, 0U, 0U}));

        // the framework offers the unconsumed records again; the second call is that offer
        std::vector<InputSpan<Record>> rest;
        rest.emplace_back(std::span<const Record>(first).subspan(3UZ));
        rest.emplace_back(std::span<const Record>(second));
        rest.emplace_back(std::span<const Record>{});
        const Call secondCall = call(block, rest, 3UZ);
        expect(eq(marksOf(secondCall.out), std::vector<std::uint8_t>{13U, 20U, 21U})) << "the first input resumes where it stopped, still in order";
        expect(eq(block.n_records.value, std::vector<gr::Size_t>{4U, 2U, 0U}));
    };

    // An unconnected input port offers nothing and reports nothing available. The ports are asynchronous, so the
    // framework's availability test is satisfied by any one input that has records rather than by all of them, and
    // the block never waits on a port that has nothing to offer.
    "an input that offers nothing does not stall the block"_test = [] {
        auto block = make<RecordMerge<std::uint8_t>>({{"n_inputs", gr::Size_t{3U}}});

        const std::vector<std::uint8_t> marks{40U, 41U};
        const std::vector<Record>       only = records(marks);

        std::vector<InputSpan<Record>> ins;
        ins.emplace_back(std::span<const Record>{}, std::span<const gr::Tag>{}, false); // unconnected
        ins.emplace_back(std::span<const Record>(only));
        ins.emplace_back(std::span<const Record>{}, std::span<const gr::Tag>{}, false); // unconnected

        const Call result = call(block, ins, 0UZ);
        expect(eq(marksOf(result.out), std::vector<std::uint8_t>{40U, 41U})) << "the connected input is published on its own";
        expect(result.status == work::Status::OK) << "and the call is progress, not a stall";
        expect(eq(block.n_records.value, std::vector<gr::Size_t>{0U, 2U, 0U})) << "a silent input is a zero count, not an absent one";

        // nothing offered at all is the framework's own idle case, and it is not an error either
        std::vector<InputSpan<Record>> idle;
        idle.emplace_back(std::span<const Record>{}, std::span<const gr::Tag>{}, false);
        idle.emplace_back(std::span<const Record>{});
        idle.emplace_back(std::span<const Record>{}, std::span<const gr::Tag>{}, false);
        const Call quiet = call(block, idle, 0UZ);
        expect(eq(quiet.out.size(), 0UZ));
        expect(quiet.status == work::Status::INSUFFICIENT_INPUT_ITEMS);
        expect(eq(block.n_records.value, std::vector<gr::Size_t>{0U, 2U, 0U})) << "and it changes no count";
    };

    // A record states its outcome in its own metadata, but a stage that publishes its reason as a tag — the record
    // to packet boundary does — must not have it deleted here: the merge is the block that carries the failure
    // ports, and a deleted annotation is the evidence the chain was told to keep.
    "a tag on a record travels with that record"_test = [] {
        auto block = make<RecordMerge<std::uint8_t>>({{"n_inputs", gr::Size_t{2U}}});

        const std::vector<std::uint8_t> firstMarks{50U, 51U};
        const std::vector<std::uint8_t> secondMarks{60U};
        const std::vector<Record>       first  = records(firstMarks);
        const std::vector<Record>       second = records(secondMarks);

        const std::vector<gr::Tag> firstTags{gr::Tag{1UZ, gr::property_map{{"discard_reason", std::string("crc_failed")}}}};
        const std::vector<gr::Tag> secondTags{gr::Tag{0UZ, gr::property_map{{"discard_reason", std::string("no_frame")}}}};

        std::vector<InputSpan<Record>> ins;
        ins.emplace_back(std::span<const Record>(first), std::span<const gr::Tag>(firstTags));
        ins.emplace_back(std::span<const Record>(second), std::span<const gr::Tag>(secondTags));

        const Call result = call(block, ins, 0UZ);
        expect(eq(result.out.size(), 3UZ));
        expect(eq(result.tags.size(), 2UZ)) << "one tag per annotated record, and no more";
        if (result.tags.size() == 2UZ) {
            expect(eq(result.tags[0UZ].index, 1UZ)) << "record 1 of input 0 leaves at offset 1";
            expect(eq(result.tags[1UZ].index, 2UZ)) << "record 0 of input 1 leaves after input 0's two";
        }
    };

    "n_inputs is fixed once a port is connected"_test = [] {
        auto block = make<RecordMerge<std::uint8_t>>({{"n_inputs", gr::Size_t{2U}}});
        expect(eq(block.inputs.size(), 2UZ));
        expect(eq(block.n_records.value.size(), 2UZ));
        expect(nothrow([&block] {
            block.n_inputs = gr::Size_t{4U};
            block.rebuild();
        })) << "unconnected, the count still moves";
        expect(eq(block.inputs.size(), 4UZ));
        expect(eq(block.n_records.value.size(), 4UZ));
    };

    // The same properties through the scheduler and real ports: three sources of different lengths, one output, and
    // the counts read off the block afterwards.
    "three sources through the scheduler"_test = [] {
        const std::vector<std::uint8_t> firstMarks{1U, 2U, 3U, 4U, 5U};
        const std::vector<std::uint8_t> secondMarks{11U, 12U};
        const std::vector<std::uint8_t> thirdMarks{21U, 22U, 23U};

        gr::Graph graph;
        auto&     sourceA = graph.emplaceBlock<RecordSource>();
        auto&     sourceB = graph.emplaceBlock<RecordSource>();
        auto&     sourceC = graph.emplaceBlock<RecordSource>();
        sourceA._records  = records(firstMarks);
        sourceB._records  = records(secondMarks);
        sourceC._records  = records(thirdMarks);
        auto& merge       = graph.emplaceBlock<RecordMerge<std::uint8_t>>({{"n_inputs", gr::Size_t{3U}}});
        auto& collector   = graph.emplaceBlock<RecordCollector>();

        expect(graph.connect(sourceA, "out"s, merge, "inputs#0"s).has_value());
        expect(graph.connect(sourceB, "out"s, merge, "inputs#1"s).has_value());
        expect(graph.connect(sourceC, "out"s, merge, "inputs#2"s).has_value());
        expect(graph.connect<"out", "in">(merge, collector).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        const auto& merged = collector._records;
        expect(eq(merged.size(), 10UZ)) << "every record of every source arrives, none dropped";
        const std::vector<std::uint8_t> arrived = marksOf(merged);
        for (const auto& source : {firstMarks, secondMarks, thirdMarks}) {
            std::vector<std::uint8_t> seen;
            for (const std::uint8_t mark : arrived) {
                if (std::ranges::find(source, mark) != source.end()) {
                    seen.push_back(mark);
                }
            }
            expect(eq(seen, source)) << "one input's records keep their own order, whatever the interleaving";
        }
        expect(eq(merge.n_records.value, std::vector<gr::Size_t>{5U, 2U, 3U})) << "one count per input, in port order";
    };
}
