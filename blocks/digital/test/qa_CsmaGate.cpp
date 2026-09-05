#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <format>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/digital/CsmaGate.hpp>

#include "TestSpans.hpp"

/*
 * The gate's whole state is one boolean and one counter, so the tests drive processBulk directly for the exact
 * counts the hold/release contract asks for, then hand the same gate to the scheduler for the chunk-size
 * independence and the held-gate cost measurement, both of which are properties of a real run rather than of one
 * call. The last test in the file is that measurement and is the one place here that reads a clock: what it is
 * about is what a held gate costs a machine, which no synthetic count can stand in for.
 */
namespace {

using gr::blocks::digital::CsmaGate;
using gr::blocks::digital::test::InputSpan;
using gr::blocks::digital::test::OutputSpan;

using Record = gr::DataSet<std::uint8_t>;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] Record record(std::uint8_t tag) {
    Record r;
    r.signal_values = {tag};
    r.extents.push_back(1);
    r.signal_names.emplace_back("csma");
    r.timing_events.resize(1UZ);
    r.meta_information.resize(1UZ);
    r.meta_information[0UZ]["origin"] = std::string("qa");
    return r;
}

struct Driven {
    std::vector<Record> out{};
    std::size_t         consumed = 0UZ;
};

/// One direct call to processBulk, with @p room records of output space.
[[nodiscard]] Driven drive(CsmaGate& block, std::span<const Record> records, std::span<const std::uint8_t> sense, std::size_t room = 32UZ) {
    std::vector<Record>     scratch(room);
    InputSpan<Record>       inSpan(records);
    InputSpan<std::uint8_t> senseSpan(sense);
    OutputSpan<Record>      outSpan{std::span<Record>(scratch)};
    std::ignore = block.processBulk(inSpan, senseSpan, outSpan);

    Driven result;
    result.out.assign(scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    result.consumed = inSpan.consumed;
    return result;
}

struct ChunkedRecordSource : gr::Block<ChunkedRecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(ChunkedRecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos   = 0UZ;
    std::size_t         _chunk = 1UZ; ///< records offered per call at most, the "input chunk size" criterion 4 asks about

    // idle rather than done once exhausted: this graph carries a second source (the sense line), and the test owns
    // the teardown through an explicit stop() rather than relying on a multi-source graph's own completion
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_pos >= _records.size()) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const std::size_t n = std::min({outSpan.size(), _records.size() - _pos, _chunk});
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }
};

/// Publishes one sense item, `_value`, on its first call and is done: the gate keeps that value as its newest until
/// something else overwrites it, which is what lets a graph exercise "sense stays clear/busy" with a finite source.
struct OneShotSense : gr::Block<OneShotSense> {
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(OneShotSense, out);
    std::uint8_t _value = 0U;
    bool         _sent  = false;

    // idle rather than done once the one item is sent: the test owns the teardown, and a DONE source here would
    // risk the scheduler treating the whole (multi-source) graph as finished before ChunkedRecordSource has
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_sent || outSpan.size() == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        outSpan[0UZ] = _value;
        outSpan.publish(1UZ);
        _sent = true;
        return gr::work::Status::OK;
    }
};

/// Offers `_limit` records in windows of `_chunk` and then goes idle, leaving a backlog the gate can decline call
/// after call. Bounded rather than endless so that what the measurement below times is the declining, not this
/// block's own record construction filling a buffer.
struct BackloggedRecordSource : gr::Block<BackloggedRecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(BackloggedRecordSource, out);
    std::size_t _limit = 64UZ;
    std::size_t _chunk = 8UZ;
    std::size_t _made  = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min({outSpan.size(), _limit - _made, _chunk});
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = record(0U);
        }
        outSpan.publish(n);
        _made += n;
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

/// A sense line that publishes a busy item on every call: the sense source performs work every traversal, which is
/// the arm that says what a co-runnable productive block does to a scheduler's idle back-off.
struct EndlessBusySense : gr::Block<EndlessBusySense> {
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(EndlessBusySense, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = outSpan.size();
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = 1U;
        }
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    mutable std::mutex  _mutex;
    std::vector<Record> _records{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _records.size();
    }

    [[nodiscard]] std::vector<Record> take() const {
        std::lock_guard lock(_mutex);
        return _records;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (const Record& r : inSpan) {
                _records.push_back(r);
            }
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// Runs a graph in a background thread until explicitly stopped, which is what a graph carrying more than one source
/// that never reports DONE on its own needs: the test owns the teardown.
struct GraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;

    explicit GraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] { std::ignore = scheduler.runAndWait(); });
    }

    GraphRunner(const GraphRunner&)            = delete;
    GraphRunner& operator=(const GraphRunner&) = delete;

    void stop() {
        scheduler.requestStop();
        if (worker.joinable()) {
            worker.join();
        }
    }

    ~GraphRunner() { stop(); }
};

template<typename F>
[[nodiscard]] bool waitUntil(F&& ready, std::chrono::milliseconds deadline = std::chrono::milliseconds(8000)) {
    const auto until = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < until) {
        if (ready()) {
            return true;
        }
        std::this_thread::yield();
    }
    return ready();
}

/// The tags of the records that made it through a `chunk`-sized run of the gate, sense clear throughout.
[[nodiscard]] std::vector<std::uint8_t> runGateGraph(std::size_t chunk, std::size_t count, gr::Size_t burstRecords = 1U) {
    std::vector<Record> records;
    records.reserve(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        records.push_back(record(static_cast<std::uint8_t>(i & 0xFFUZ)));
    }

    gr::Graph flow;
    auto&     source   = flow.emplaceBlock<ChunkedRecordSource>();
    source._records    = records;
    source._chunk      = chunk;
    auto& senseSource  = flow.emplaceBlock<OneShotSense>();
    senseSource._value = 0U; // clear, and it stays the gate's newest value for the rest of the run
    auto& gate         = flow.emplaceBlock<CsmaGate>({{"burst_records", burstRecords}});
    auto& sink         = flow.emplaceBlock<RecordSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, gate).has_value());
    boost::ut::expect(flow.connect<"out", "sense">(senseSource, gate).has_value());
    boost::ut::expect(flow.connect<"out", "in">(gate, sink).has_value());

    GraphRunner runner(std::move(flow));
    boost::ut::expect(waitUntil([&sink, count] { return sink.count() >= count; })) << std::format("chunk {}: only {} of {} records crossed", chunk, sink.count(), count);
    runner.stop();

    std::vector<std::uint8_t> tags;
    for (const Record& r : sink.take()) {
        tags.push_back(r.signal_values.empty() ? 0xFFU : r.signal_values[0UZ]);
    }
    return tags;
}

/// What one held-gate arm cost, and what the gate did while it was held.
struct HeldRun {
    double        cpuSeconds  = 0.0;
    double        wallSeconds = 0.0;
    std::uint64_t busyCalls   = 0ULL;
    std::uint64_t recordsHeld = 0ULL;
    std::uint64_t passed      = 0ULL;
    std::size_t   released    = 0UZ;
};

/**
 * @brief A backlogged gate held busy for @p interval, with the sense line @p addSense wires in, measured.
 *
 * Process CPU time against wall time over the same interval is the number: a gate that declines its input should
 * cost approximately nothing, and a whole core says the scheduler came straight back in. The two arms differ only
 * in the sense source, so what the difference between them measures is that source's effect on the scheduler's own
 * idle back-off rather than anything the gate does.
 */
template<typename TAddSense>
[[nodiscard]] HeldRun runHeldGate(std::chrono::milliseconds interval, TAddSense&& addSense) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<BackloggedRecordSource>();
    auto&     gate   = flow.emplaceBlock<CsmaGate>();
    auto&     sink   = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(source, gate).has_value());
    boost::ut::expect(flow.connect<"out", "in">(gate, sink).has_value());
    addSense(flow, gate);

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    const auto  cpuStart  = std::clock();
    const auto  wallStart = std::chrono::steady_clock::now();
    std::thread runner([&scheduler] { std::ignore = scheduler.runAndWait(); });
    std::this_thread::sleep_for(interval);
    scheduler.requestStop();
    runner.join();

    HeldRun run;
    run.cpuSeconds  = static_cast<double>(std::clock() - cpuStart) / static_cast<double>(CLOCKS_PER_SEC);
    run.wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
    run.busyCalls   = gate.nBusyCalls;
    run.recordsHeld = gate.nRecordsHeld;
    run.passed      = gate.nRecordsPassed;
    run.released    = sink.count();
    return run;
}

} // namespace

const boost::ut::suite<"csma_gate"> csmaGateTests = [] {
    using namespace boost::ut;

    // Criterion 3: the gate holds and releases on the sense state, exactly.
    "ten records offered while sense is busy: nothing published, nothing consumed, both counted"_test = [] {
        CsmaGate                        block;
        std::vector<Record>             ten(10UZ, record(7U));
        const std::vector<std::uint8_t> busy{1U};

        const Driven held = drive(block, std::span<const Record>(ten), std::span<const std::uint8_t>(busy));
        expect(eq(held.out.size(), 0UZ)) << "nothing published";
        expect(eq(held.consumed, 0UZ)) << "the input span is not consumed";
        expect(eq(block.nRecordsHeld, std::uint64_t{10ULL}));
        expect(that % block.nBusyCalls > 0ULL);
        expect(eq(block.nRecordsPassed, std::uint64_t{0ULL}));

        // the same ten offered again and held again: ten records were delayed, not twenty
        const Driven heldAgain = drive(block, std::span<const Record>(ten), std::span<const std::uint8_t>(busy));
        expect(eq(heldAgain.consumed, 0UZ));
        expect(eq(block.nRecordsHeld, std::uint64_t{10ULL})) << "a record waiting through a second call is the same waiting record";
        expect(eq(block.nBusyCalls, std::uint64_t{2ULL})) << "the calls, though, are two";

        // one clear item pushed at burst_records = 1: exactly one record is released and the gate holds again
        const std::vector<std::uint8_t> clear{0U};
        const Driven                    released = drive(block, std::span<const Record>(ten), std::span<const std::uint8_t>(clear));
        expect(eq(released.out.size(), 1UZ));
        expect(eq(released.consumed, 1UZ));
        expect(eq(block.nRecordsPassed, std::uint64_t{1ULL}));
    };

    "one clear item at burst_records = 4 releases exactly four before it consults sense again"_test = [] {
        CsmaGate                        block = make<CsmaGate>({{"burst_records", gr::Size_t{4}}});
        std::vector<Record>             ten(10UZ, record(3U));
        const std::vector<std::uint8_t> clear{0U};

        const Driven released = drive(block, std::span<const Record>(ten), std::span<const std::uint8_t>(clear));
        expect(eq(released.out.size(), 4UZ));
        expect(eq(released.consumed, 4UZ));
        expect(eq(block.nRecordsPassed, std::uint64_t{4ULL}));
        expect(eq(block.nRecordsHeld, std::uint64_t{6ULL})) << "the six left waiting for the next consultation";
        expect(eq(block.nBusyCalls, std::uint64_t{0ULL})) << "sense itself was clear; the burst limit held the rest, not sense";
    };

    // Criterion 4: the gate reads only the newest sense item.
    "a sense span of busy, busy, clear releases; clear, clear, busy does not"_test = [] {
        CsmaGate                        releasing;
        std::vector<Record>             one{record(1U)};
        const std::vector<std::uint8_t> busyThenClear{1U, 1U, 0U};
        const Driven                    a = drive(releasing, std::span<const Record>(one), std::span<const std::uint8_t>(busyThenClear));
        expect(eq(a.out.size(), 1UZ)) << "the last item, clear, is what decides it";
        expect(eq(a.consumed, 1UZ));

        CsmaGate                        holding;
        const std::vector<std::uint8_t> clearThenBusy{0U, 0U, 1U};
        const Driven                    b = drive(holding, std::span<const Record>(one), std::span<const std::uint8_t>(clearThenBusy));
        expect(eq(b.out.size(), 0UZ)) << "the last item, busy, is what decides it";
        expect(eq(b.consumed, 0UZ));
        expect(eq(holding.nRecordsHeld, std::uint64_t{1ULL}));
    };

    "metadata crosses verbatim; the gate writes nothing"_test = [] {
        CsmaGate                        block;
        std::vector<Record>             one{record(9U)};
        const std::vector<std::uint8_t> clear{0U};
        const Driven                    out = drive(block, std::span<const Record>(one), std::span<const std::uint8_t>(clear));
        expect(eq(out.out.size(), 1UZ));
        if (out.out.size() == 1UZ) {
            const auto entry = out.out[0UZ].meta_information[0UZ].find(gr::property_map::key_type("origin"));
            expect(that % (entry != out.out[0UZ].meta_information[0UZ].end())) << "the record's facts cross verbatim";
        }
    };

    "the first sense item is a transition only when it disagrees with busy-until-proven-otherwise"_test = [] {
        std::vector<Record>             one{record(4U)};
        const std::vector<std::uint8_t> busy{1U};
        const std::vector<std::uint8_t> clear{0U};

        CsmaGate staysBusy;
        std::ignore = drive(staysBusy, std::span<const Record>(one), std::span<const std::uint8_t>(busy));
        expect(eq(staysBusy.nSenseTransitions, std::uint64_t{0ULL})) << "the gate starts busy, so a busy observation changes nothing";

        CsmaGate turnsClear;
        std::ignore = drive(turnsClear, std::span<const Record>(one), std::span<const std::uint8_t>(clear));
        expect(eq(turnsClear.nSenseTransitions, std::uint64_t{1ULL})) << "the first clear observation is the first transition";
        std::ignore = drive(turnsClear, std::span<const Record>(one), std::span<const std::uint8_t>(clear));
        expect(eq(turnsClear.nSenseTransitions, std::uint64_t{1ULL})) << "a second clear observation says the same thing";
        std::ignore = drive(turnsClear, std::span<const Record>(one), std::span<const std::uint8_t>(busy));
        expect(eq(turnsClear.nSenseTransitions, std::uint64_t{2ULL}));
    };

    "burst_records is refused at zero, and a settings change discards both the allowance and the sense reading"_test = [] {
        expect(throws([] { std::ignore = make<CsmaGate>({{"burst_records", gr::Size_t{0U}}}); })) << "zero releases nothing per consultation, which is not a burst";
        expect(nothrow([] { std::ignore = make<CsmaGate>({{"burst_records", gr::Size_t{1U}}}); }));

        CsmaGate                        block = make<CsmaGate>({{"burst_records", gr::Size_t{4}}});
        std::vector<Record>             ten(10UZ, record(2U));
        const std::vector<std::uint8_t> clear{0U};

        // room for only two of the granted four-record burst, so an allowance of two carries as a leftover
        {
            std::vector<Record>     scratch(2UZ);
            InputSpan<Record>       inSpan{std::span<const Record>(ten)};
            InputSpan<std::uint8_t> senseSpan{std::span<const std::uint8_t>(clear)};
            OutputSpan<Record>      outSpan{std::span<Record>(scratch)};
            std::ignore = block.processBulk(inSpan, senseSpan, outSpan);
            expect(eq(outSpan.count, 2UZ));
        }
        expect(eq(block.nRecordsPassed, std::uint64_t{2ULL}));

        // the new value staged the way a graph stages one, through settingsChanged rather than by calling rebuild()
        expect(block.settings().setStaged({{"burst_records", gr::Size_t{1U}}}).empty()) << "the value is accepted";
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.burst_records.value, gr::Size_t{1U}));
        expect(eq(block.nAllowanceDiscarded, std::uint64_t{2ULL})) << "the two releases still owed under burst_records = 4 are thrown away and counted";

        const Driven stillHeld = drive(block, std::span<const Record>(ten), std::span<const std::uint8_t>{});
        expect(eq(stillHeld.out.size(), 0UZ)) << "the sense reading taken under the old configuration is gone too: busy until an item says otherwise";

        const Driven after = drive(block, std::span<const Record>(ten), std::span<const std::uint8_t>(clear));
        expect(eq(after.out.size(), 1UZ)) << "burst_records is 1, and the old leftover of 2 was discarded rather than honored";
        expect(nothrow([&block] { block.stop(); }));
    };

    "a stop() with a release allowance still owed counts it"_test = [] {
        CsmaGate                        block = make<CsmaGate>({{"burst_records", gr::Size_t{4}}});
        std::vector<Record>             one{record(5U)};
        const std::vector<std::uint8_t> clear{0U};

        const Driven released = drive(block, std::span<const Record>(one), std::span<const std::uint8_t>(clear));
        expect(eq(released.out.size(), 1UZ)) << "one record was all there was to release of the four granted";
        block.stop();
        expect(eq(block.nAllowanceDiscarded, std::uint64_t{3ULL})) << "the three releases the channel granted and no record spent";
    };

    // F4 asks what a graph does with `sense` unconnected, the port being required rather than gr::Optional.
    "a graph with sense unconnected"_test = [] {
        gr::Graph flow;
        auto&     source = flow.emplaceBlock<BackloggedRecordSource>();
        source._limit    = 4UZ;
        auto& gate       = flow.emplaceBlock<CsmaGate>();
        auto& sink       = flow.emplaceBlock<RecordSink>();
        expect(flow.connect<"out", "in">(source, gate).has_value());
        expect(flow.connect<"out", "in">(gate, sink).has_value());

        gr::scheduler::Simple<> scheduler;
        // the port is required rather than gr::Optional, but nothing refuses the graph for it: connection checking
        // is on the output side, so a mandatory input nobody wired is accepted and the block simply never sees an
        // item on it
        expect(scheduler.exchange(std::move(flow)).has_value()) << "the graph is accepted with a required input port unconnected";
        std::thread runner([&scheduler] { std::ignore = scheduler.runAndWait(); });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        scheduler.requestStop();
        runner.join();
        std::println("gr::blocks::digital::CsmaGate qa: a graph with sense unconnected ran, releasing {} of {} records over {} busy calls", sink.count(), source._limit, gate.nBusyCalls);
        // nothing arrives on an unconnected sense port, so the gate never leaves the busy state it starts in: the
        // graph is a permanent silent stall, which is exactly what the port being required is meant to prevent
        expect(eq(sink.count(), 0UZ)) << "no record crosses a gate whose channel was never observed";
        expect(eq(gate.nRecordsPassed, std::uint64_t{0ULL}));
    };

    // Criterion 4: identical output at input chunk sizes {1, 3, 4096}, run under the scheduler.
    "identical output at input chunk sizes 1, 3 and 4096"_test = [] {
        constexpr std::size_t     kCount = 24UZ;
        std::vector<std::uint8_t> reference;
        for (const std::size_t chunk : {1UZ, 3UZ, 4096UZ}) {
            const std::vector<std::uint8_t> tags = runGateGraph(chunk, kCount);
            expect(eq(tags.size(), kCount)) << std::format("chunk {}", chunk);
            if (reference.empty()) {
                reference = tags;
            } else {
                expect(that % (tags == reference)) << std::format("chunk {}", chunk);
            }
        }
    };

    // Criterion 4: what a graph held busy costs. This is a performance measurement and reads the wall clock, which
    // is the one thing the tests here are otherwise built to avoid; the assertions on the gate's own behavior are
    // counts, and only the cost bound is a time.
    "a held gate releases nothing, and what it costs depends on what else is runnable"_test = [] {
        constexpr auto kInterval = std::chrono::milliseconds(150);

        // one busy observation and then an idle sense line: nothing in the graph performs work, and the scheduler's
        // idle back-off is free to engage
        const HeldRun quiet = runHeldGate(kInterval, [](gr::Graph& flow, CsmaGate& gate) {
            auto& sense  = flow.emplaceBlock<OneShotSense>();
            sense._value = 1U;
            boost::ut::expect(flow.connect<"out", "sense">(sense, gate).has_value());
        });

        // the same graph with a sense line that publishes on every call: that source performs work every traversal,
        // so the back-off never engages however little the gate itself does
        const HeldRun productive = runHeldGate(kInterval, [](gr::Graph& flow, CsmaGate& gate) {
            auto& sense = flow.emplaceBlock<EndlessBusySense>();
            boost::ut::expect(flow.connect<"out", "sense">(sense, gate).has_value());
        });

        const auto held = [](std::string_view label, const HeldRun& run) {
            std::println("gr::blocks::digital::CsmaGate qa: {} held for {:.3f} s wall: {:.3f} s CPU, nBusyCalls={}, nRecordsHeld={}", label, run.wallSeconds, run.cpuSeconds, run.busyCalls, run.recordsHeld);
            expect(eq(run.released, 0UZ)) << std::format("{}: a permanently busy channel releases nothing", label);
            expect(eq(run.passed, std::uint64_t{0ULL})) << label;
            expect(that % (run.busyCalls > 0ULL)) << std::format("{}: the scheduler did call back in while the gate was held", label);
            expect(that % (run.recordsHeld > 0ULL)) << std::format("{}: records were offered and declined rather than never arriving", label);
        };
        held("idle sense", quiet);
        held("publishing sense", productive);

        // The gate's own work is the same in both arms — it declines and returns — so the difference between them is
        // the scheduler's. With nothing else runnable it reaches its idle back-off and sleeps between traversals,
        // and the held gate costs about a tenth of a core, most of that the run's own setup; with one block
        // publishing on every call the back-off never engages and the run costs a whole core however little the
        // gate does. Measured here: 0.014 s of CPU against 0.150 s of wall idle, 0.151 s against 0.151 s
        // publishing. The bound is half a core, five times the idle arm's measured cost and half the publishing
        // arm's, so machine load does not decide it.
        expect(that % (quiet.cpuSeconds < 0.5 * quiet.wallSeconds)) << std::format("idle sense: cpu={:.3f}s wall={:.3f}s", quiet.cpuSeconds, quiet.wallSeconds);
        expect(that % (productive.cpuSeconds > 4.0 * quiet.cpuSeconds)) << std::format("the two arms must really differ: idle sense cpu={:.4f}s, publishing sense cpu={:.4f}s", quiet.cpuSeconds, productive.cpuSeconds);
    };
};

int main() { /* not needed for UT */ }
