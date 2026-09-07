#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>
#include <gnuradio-4.0/basic/GraphBridge.hpp>

// The two halves of one flow graph, in two files and two schedulers, joined by nothing but the bridge name they both
// carry. Neither half is reachable from here as a typed block: they arrive through the YAML importer as `gr::Graph`,
// which is the whole point of the registry -- an application that exported the graph gets its ring back without a cast
// and without including this header in its loader.

namespace {

using namespace std::chrono_literals;

constexpr auto        kDeliveryTimeout = 5s;
constexpr std::size_t kWanted          = 16384UZ;
constexpr std::size_t kCapacity        = 1UZ << 16U;

[[nodiscard]] std::string producerGraph(std::string_view bridgeName) {
    return std::format(R"(
blocks:
  - id: gr::blocks::basic::SignalGenerator<complex<float32>>
    parameters:
      name: iq_source
      sample_rate: !!float32 1000000.0
      signal_type: Sin
      tone_frequency: !!float32 10000.0
      amplitude: !!float32 1.0
      chunk_size: !!uint32 4096
  - id: gr::blocks::basic::BridgeSink
    parameters:
      name: bridge_out
      bridge_name: {}
connections:
  - [iq_source, out, bridge_out, in]
)",
        bridgeName);
}

[[nodiscard]] std::string consumerGraph(std::string_view bridgeName, std::string_view signalName) {
    return std::format(R"(
blocks:
  - id: gr::blocks::basic::BridgeSource
    parameters:
      name: bridge_in
      bridge_name: {}
  - id: gr::blocks::basic::Real<complex<float32>>
    parameters:
      name: real_part
  - id: gr::blocks::basic::DataSink<float32>
    parameters:
      name: level_sink
      signal_name: {}
connections:
  - [bridge_in, out, real_part, in]
  - [real_part, real, level_sink, in]
)",
        bridgeName, signalName);
}

/// @brief A scheduler running its graph on its own thread, stopped when the test is done with it.
struct Runner {
    gr::scheduler::Simple<>                     scheduler;
    std::future<std::expected<void, gr::Error>> finished;

    explicit Runner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
        finished = std::async(std::launch::async, [this] { return scheduler.runAndWait(); });
    }

    Runner(const Runner&)            = delete;
    Runner& operator=(const Runner&) = delete;

    void stop() {
        if (finished.valid()) {
            scheduler.requestStop();
            const auto result = finished.get();
            boost::ut::expect(result.has_value()) << (result.has_value() ? std::string{} : result.error().message);
        }
    }

    ~Runner() { stop(); }
};

} // namespace

const boost::ut::suite<"graph bridge registry"> bridgeRegistryTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;

    "two loaded graphs joined only by the bridge name carry the stream across"_test = [] {
        constexpr std::string_view kBridge = "qa_bridge_stream";
        constexpr std::string_view kSignal = "qa_bridge_level";

        auto ring = std::make_shared<BridgeState>();
        ring->configure(kCapacity);
        globalBridgeRegistry().publish(kBridge, ring);

        auto consumer = gr::loadGrc(gr::globalPluginLoader(), consumerGraph(kBridge, kSignal));
        auto producer = gr::loadGrc(gr::globalPluginLoader(), producerGraph(kBridge));

        Runner consuming(std::move(*consumer));
        Runner producing(std::move(*producer));

        std::shared_ptr<StreamingPoller<float>> poller;
        std::vector<float>                      samples;

        const auto deadline = std::chrono::steady_clock::now() + kDeliveryTimeout;
        bool       enough   = false;
        while (!enough && std::chrono::steady_clock::now() < deadline) {
            if (poller == nullptr) {
                poller = globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName(std::string{kSignal}), {.overflowPolicy = OverflowPolicy::Drop});
                std::this_thread::sleep_for(2ms);
                continue;
            }
            while (poller->process([&samples](std::span<const float> data) { samples.insert(samples.end(), data.begin(), data.end()); })) {
            }
            enough = samples.size() >= kWanted;
            if (!enough) {
                std::this_thread::sleep_for(2ms);
            }
        }
        producing.stop();
        consuming.stop();

        expect(enough) << std::format("{} of {} samples crossed the bridge", samples.size(), kWanted) << fatal;

        float highest = 0.0f;
        for (const float sample : samples) {
            expect(le(std::abs(sample), 1.0f + 1e-5f)) << std::format("a sample of {:.6f} arrived, past the generator's amplitude", sample);
            highest = std::max(highest, std::abs(sample));
        }
        expect(ge(highest, 0.5f)) << "every sample that crossed was near zero; the tone did not cross intact";

        const auto counters = globalBridgeRegistry().counters(kBridge);
        expect(counters.has_value()) << fatal;
        expect(eq(counters->capacity, kCapacity));

        globalBridgeRegistry().withdraw(kBridge);
        expect(globalBridgeRegistry().find(kBridge) == nullptr);
    };

    "a name carries one ring, and publishing it twice is refused"_test = [] {
        constexpr std::string_view kBridge = "qa_bridge_once";

        auto first = std::make_shared<BridgeState>();
        first->configure(64UZ);
        globalBridgeRegistry().publish(kBridge, first);

        expect(throws<gr::exception>([&] { globalBridgeRegistry().publish(kBridge, std::make_shared<BridgeState>()); }));
        expect(globalBridgeRegistry().find(kBridge) == first) << "the refused publish replaced the ring already there";

        globalBridgeRegistry().withdraw(kBridge);
        expect(globalBridgeRegistry().find(kBridge) == nullptr);
        expect(!globalBridgeRegistry().counters(kBridge).has_value());
    };

    "a block naming an unpublished bridge refuses, naming itself and the bridge"_test = [] {
        gr::property_map settings;
        settings["name"]        = std::string("consumer_end");
        settings["bridge_name"] = std::string("qa_bridge_absent");

        BridgeSource source(settings);
        source.settings().init();

        std::string refusal;
        try {
            std::ignore = source.settings().applyStagedParameters();
        } catch (const gr::exception& error) {
            refusal = error.message;
        }
        expect(!refusal.empty()) << "an unpublished bridge name was taken in silence, which a DONE cannot be told from";
        expect(refusal.find("consumer_end") != std::string::npos) << refusal;
        expect(refusal.find("qa_bridge_absent") != std::string::npos) << refusal;
        expect(source.bridge == nullptr);
    };

    "the counters are readable by name"_test = [] {
        constexpr std::string_view kBridge = "qa_bridge_counters";

        auto ring = std::make_shared<BridgeState>();
        ring->configure(8UZ);
        globalBridgeRegistry().publish(kBridge, ring);

        expect(globalBridgeRegistry().counters(kBridge).has_value()) << fatal;
        expect(eq(globalBridgeRegistry().counters(kBridge)->capacity, 8UZ));
        expect(eq(globalBridgeRegistry().counters(kBridge)->overflows, std::uint64_t{0}));

        ring->push(std::vector<std::complex<float>>(12UZ)); // four more than the ring holds
        const BridgeCounters after = globalBridgeRegistry().counters(kBridge).value();
        expect(eq(after.available, after.capacity));
        expect(eq(after.overflows, std::uint64_t{4})) << "the count of dropped samples is the excess over the capacity";
        expect(!after.eos);

        ring->setEos();
        expect(globalBridgeRegistry().counters(kBridge)->eos);

        globalBridgeRegistry().withdraw(kBridge);
    };
};

int main() { /* not needed for UT */ }
