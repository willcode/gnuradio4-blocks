#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/GraphBridge.hpp>

#include <complex>
#include <cstdint>
#include <span>
#include <vector>

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
};

int main() { /* not needed for UT */ }
