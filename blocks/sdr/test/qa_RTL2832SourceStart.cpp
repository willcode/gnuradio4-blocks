#include <boost/ut.hpp>

#include <atomic>
#include <chrono>
#include <complex>
#include <expected>
#include <optional>
#include <string>
#include <thread>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/sdr/RTL2832Source.hpp>

// Cases that never open a USB device, apart from qa_RTL2832Source, which opens the first dongle attached. A device index
// past any count a bus can hold is refused after the enumeration, which reads descriptors and opens nothing.

namespace {

using namespace std::chrono_literals;

// the run's result, or nothing when the run was still going after `bound` and had to be stopped
std::optional<std::expected<void, gr::Error>> runBounded(gr::scheduler::Simple<>& sched, std::chrono::milliseconds bound) {
    std::optional<std::expected<void, gr::Error>> result;
    std::atomic<bool>                             ended{false};
    auto                                          schedThread = std::thread([&sched, &result, &ended] {
        result = sched.runAndWait();
        ended.store(true, std::memory_order_release);
    });
    const auto                                    deadline    = std::chrono::steady_clock::now() + bound;
    while (!ended.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
    }
    const bool endedInTime = ended.load(std::memory_order_acquire);
    if (!endedInTime) {
        sched.requestStop();
    }
    schedThread.join();
    return endedInTime ? result : std::nullopt;
}

} // namespace

const boost::ut::suite<"RTL2832Source start"> _rtlStartTests = [] {
    using namespace boost::ut;

    // with a reader on the scheduler's messages, the scheduler forwards an error message and does not end the run on
    // it: the run ends only if the source's start fails it
    "an RTL2832Source whose device cannot be opened fails the run of a host that reads messages"_test = [] {
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::sdr::RTL2832Source<std::complex<float>>>({{"device_index", std::uint32_t{1000U}}});
        auto&     sink   = graph.emplaceBlock<gr::blocks::testing::TagSink<std::complex<float>, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>();
        expect(graph.connect<"out", "in">(source, sink).has_value());

        gr::MsgPortIn           fromScheduler;
        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(graph)).has_value());
        expect(sched.msgOut.connect(fromScheduler).has_value());
        // the io thread retries a refused open every 2 s for a device lost during a run, and in the browser build also for
        // the first open
        const auto result = runBounded(sched, 5s);
        expect(fatal(result.has_value())) << "the run must end by itself, not wait for a device that is not there";
        expect(fatal(!result->has_value())) << "a source that cannot open its device must fail the run";
        expect(result->error().message.contains("open failed")) << result->error().message;
    };
};

int main() { /* not needed for UT */ }
