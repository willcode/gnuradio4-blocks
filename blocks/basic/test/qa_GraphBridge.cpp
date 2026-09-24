#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/GraphBridge.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <format>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace gr::blocks::basic::graph_bridge_test {

using C = std::complex<float>;

C sample(int i) { return C{static_cast<float>(i), 0.0f}; }

std::vector<C> seq(int a, int b) { // [a, b)
    std::vector<C> v;
    for (int i = a; i < b; ++i) {
        v.push_back(sample(i));
    }
    return v;
}

bool matchesFrom(std::span<const C> got, int first) {
    for (std::size_t i = 0UZ; i < got.size(); ++i) {
        if (got[i] != sample(first + static_cast<int>(i))) {
            return false;
        }
    }
    return true;
}

struct CollectSink : Block<CollectSink> {
    PortIn<C> in;
    GR_MAKE_REFLECTABLE(CollectSink, in);

    std::vector<C> collected;

    work::Status processBulk(std::span<const C> input) {
        collected.insert(collected.end(), input.begin(), input.end());
        return work::Status::OK;
    }
};

using TestIn  = gr::blocks::testing::span::InputSpan<C>;
using TestOut = gr::blocks::testing::span::OutputSpan<C>;

/// Shared between a `StallingSource` and the test thread.
struct StallControl {
    std::atomic<std::size_t> calls{0UZ};
    std::atomic<bool>        release{false};
};

/// Publishes nothing and answers `OK` until released, then `DONE`, and counts its calls. The
/// framework reports a block that answers this way for a second.
struct StallingSource : Block<StallingSource> {
    PortOut<C> out;
    GR_MAKE_REFLECTABLE(StallingSource, out);

    std::shared_ptr<StallControl> control;

    work::Status processBulk(OutputSpanLike auto& outSpan) {
        control->calls.fetch_add(1UZ);
        outSpan.publish(0UZ);
        return control->release.load() ? work::Status::DONE : work::Status::OK;
    }
};

/// Sends file descriptor 2 to a temporary file until `release()`; `text()` reads what arrived.
class StderrCapture {
    std::FILE* _file  = std::tmpfile();
    int        _saved = -1;

public:
    StderrCapture() {
        if (_file != nullptr) {
            std::fflush(stderr);
            _saved = ::dup(STDERR_FILENO);
            ::dup2(::fileno(_file), STDERR_FILENO);
        }
    }
    StderrCapture(const StderrCapture&)            = delete;
    StderrCapture& operator=(const StderrCapture&) = delete;
    ~StderrCapture() {
        release();
        if (_file != nullptr) {
            std::fclose(_file);
        }
    }

    void release() {
        if (_saved >= 0) {
            std::fflush(stderr);
            ::dup2(_saved, STDERR_FILENO);
            ::close(_saved);
            _saved = -1;
        }
    }

    [[nodiscard]] std::string text() const {
        std::string out;
        if (_file == nullptr) {
            return out;
        }
        std::fflush(stderr);
        std::array<char, 4096UZ> chunk{};
        for (off_t at = 0;;) {
            const ssize_t n = ::pread(::fileno(_file), chunk.data(), chunk.size(), at);
            if (n <= 0) {
                return out;
            }
            out.append(chunk.data(), static_cast<std::size_t>(n));
            at += n;
        }
    }
};

} // namespace gr::blocks::basic::graph_bridge_test

const boost::ut::suite GraphBridgeTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;
    using namespace gr::blocks::basic::graph_bridge_test;

    "FIFO order preserved below capacity"_test = [] {
        BridgeState b;
        b.configure(8UZ);
        b.push(seq(0, 4));

        std::vector<C> out(4UZ);
        bool           eos = false;
        expect(eq(b.popOrWait(out, eos), 4UZ));
        expect(!eos);
        expect(matchesFrom(out, 0));
        expect(eq(b.overflows.load(), 0UZ));
    };

    "drop-oldest on partial overflow"_test = [] {
        BridgeState b;
        b.configure(8UZ);
        b.push(seq(0, 6));
        b.push(seq(6, 10)); // 10 samples into capacity 8: the oldest two go

        std::vector<C> out(8UZ);
        bool           eos = false;
        expect(eq(b.popOrWait(out, eos), 8UZ));
        expect(matchesFrom(out, 2));
        expect(eq(b.overflows.load(), 2UZ));
    };

    "single push larger than the ring keeps only its tail"_test = [] {
        BridgeState b;
        b.configure(8UZ);
        b.push(seq(0, 20)); // keeps 12..19

        std::vector<C> out(8UZ);
        bool           eos = false;
        expect(eq(b.popOrWait(out, eos), 8UZ));
        expect(matchesFrom(out, 12));
        expect(eq(b.overflows.load(), 12UZ));
    };

    "backpressure accepts only what fits and never drops"_test = [] {
        BridgeState b;
        b.configure(8UZ, /*block=*/true);
        expect(eq(b.sink(seq(0, 5)), 5UZ));  // fits entirely
        expect(eq(b.sink(seq(5, 20)), 3UZ)); // only the remaining space is taken
        expect(eq(b.count, 8UZ));
        expect(eq(b.overflows.load(), 0UZ)); // the rest stays upstream instead

        std::vector<C> out(8UZ);
        bool           eos = false;
        expect(eq(b.popOrWait(out, eos), 8UZ));
        expect(matchesFrom(out, 0));
    };

    "clear() empties the ring and resets EoS"_test = [] {
        BridgeState b;
        b.configure(8UZ);
        b.push(seq(0, 5));
        b.setEos();
        b.clear();
        expect(eq(b.count, 0UZ));
        expect(!b.eos);
    };

    "popOrWait reports EoS on a drained and latched ring"_test = [] {
        BridgeState b;
        b.configure(8UZ);
        b.setEos();

        std::vector<C> out(4UZ);
        bool           eos = false;
        expect(eq(b.popOrWait(out, eos), 0UZ));
        expect(eos);
    };

    "BridgeSource drains the ring in order, then emits DONE"_test = [] {
        constexpr std::size_t kSamples = 5000UZ;

        auto bridge = std::make_shared<BridgeState>();
        bridge->configure(1UZ << 16);
        bridge->push(seq(0, static_cast<int>(kSamples)));
        bridge->setEos(); // the producer finished: drain, then DONE

        gr::Graph flow;
        auto&     src = flow.emplaceBlock<BridgeSource>();
        src.bridge    = bridge;
        auto& sink    = flow.emplaceBlock<CollectSink>();
        expect(flow.connect<"out", "in">(src, sink).has_value());

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(sched.runAndWait().has_value()) << "scheduler must terminate on the propagated DONE";

        expect(eq(sink.collected.size(), kSamples));
        expect(matchesFrom(sink.collected, 0));
    };

    "BridgeSource answers INSUFFICIENT_INPUT_ITEMS on an empty ring, OK when it published"_test = [] {
        auto bridge = std::make_shared<BridgeState>();
        bridge->configure(8UZ);
        BridgeSource src;
        src.bridge = bridge;

        std::vector<C> room(8UZ);
        TestOut        idle{std::span<C>(room)};
        expect(src.processBulk(idle) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "an empty ring is a wait for input";
        expect(eq(idle.count, 0UZ));

        bridge->push(seq(0, 3));
        TestOut fed{std::span<C>(room)};
        expect(src.processBulk(fed) == gr::work::Status::OK);
        expect(eq(fed.count, 3UZ));
        expect(matchesFrom(std::span<const C>(room).first(3UZ), 0));

        bridge->setEos();
        TestOut drained{std::span<C>(room)};
        expect(src.processBulk(drained) == gr::work::Status::DONE);
        expect(eq(drained.count, 0UZ));
    };

    "an idle BridgeSource stays out of the zero-progress report"_test = [] {
        auto bridge = std::make_shared<BridgeState>();
        bridge->configure(64UZ);
        auto stall = std::make_shared<StallControl>();

        gr::Graph flow;
        auto&     src     = flow.emplaceBlock<BridgeSource>();
        src.bridge        = bridge;
        auto& sink        = flow.emplaceBlock<CollectSink>();
        auto& control     = flow.emplaceBlock<StallingSource>();
        control.control   = stall;
        auto& controlSink = flow.emplaceBlock<CollectSink>();
        expect(flow.connect<"out", "in">(src, sink).has_value());
        expect(flow.connect<"out", "in">(control, controlSink).has_value());
        const std::string bridgeLine  = std::format("'{}' keeps returning OK", src.unique_name);
        const std::string controlLine = std::format("'{}' keeps returning OK", control.unique_name);

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(flow)).has_value());

        StderrCapture capture;
        auto          finished = std::async(std::launch::async, [&sched] { return sched.runAndWait(); });

        // The control's report marks a second without progress. Two more of its calls complete a
        // pass in which the bridge has been idle at least as long.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (!capture.text().contains(controlLine) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const std::size_t reportedAt = stall->calls.load();
        while (stall->calls.load() < reportedAt + 2UZ && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        bridge->push(seq(0, 4));
        bridge->setEos();
        stall->release.store(true);
        const bool        ran  = finished.get().has_value();
        const std::string text = capture.text();
        capture.release();

        expect(ran);
        expect(text.contains(controlLine)) << "the capture must see the report for the stalling control";
        expect(!text.contains(bridgeLine)) << text;
        expect(eq(sink.collected.size(), 4UZ));
        expect(matchesFrom(sink.collected, 0));
    };

    "a full backpressured BridgeSink answers INSUFFICIENT_OUTPUT_ITEMS and consumes nothing"_test = [] {
        const std::vector<C> fill = seq(0, 8);
        const std::vector<C> more = seq(8, 12);

        auto backpressured = std::make_shared<BridgeState>();
        backpressured->configure(8UZ, /*block=*/true);
        BridgeSink held;
        held.bridge = backpressured;
        TestIn filling{std::span<const C>(fill)};
        expect(held.processBulk(filling) == gr::work::Status::OK);
        expect(eq(filling.consumed, 8UZ));
        TestIn refused{std::span<const C>(more)};
        expect(held.processBulk(refused) == gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS) << "a full ring is a wait for room";
        expect(eq(refused.consumed, 0UZ));
        expect(eq(backpressured->counters().available, 8UZ));

        auto dropping = std::make_shared<BridgeState>();
        dropping->configure(8UZ);
        BridgeSink lossy;
        lossy.bridge = dropping;
        TestIn full{std::span<const C>(fill)};
        expect(lossy.processBulk(full) == gr::work::Status::OK);
        TestIn overflowing{std::span<const C>(more)};
        expect(lossy.processBulk(overflowing) == gr::work::Status::OK) << "drop-oldest takes every sample";
        expect(eq(overflowing.consumed, 4UZ));
        expect(eq(dropping->overflows.load(), 4UZ));
    };
};

int main() { /* not needed for UT */ }
