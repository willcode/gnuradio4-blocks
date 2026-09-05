#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <ctime>
#include <format>
#include <limits>
#include <numeric>
#include <span>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/Throttle.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include "TestSpans.hpp"

namespace {

using gr::blocks::basic::Throttle;
namespace spans = gr::blocks::basic::test;

using Clock = std::chrono::steady_clock;

constexpr float  kRate    = 100000.f;
constexpr double kFullSet = 50000.0 / 100000.0; // half a second of paced samples

template<typename T>
[[nodiscard]] Throttle<T> makeThrottle(gr::property_map settings) {
    Throttle<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Pump {
    std::size_t consumed     = 0UZ;
    std::size_t produced     = 0UZ;
    std::size_t largestChunk = 0UZ;
    std::size_t calls        = 0UZ;
};

template<typename T, typename FHook>
Pump pump(Throttle<T>& block, std::span<const T> input, std::span<T> output, std::size_t chunkSize, bool connected, FHook&& perCall) {
    Pump result;
    while (result.consumed < input.size()) {
        const std::size_t    count = std::min(chunkSize, input.size() - result.consumed);
        spans::InputSpan<T>  inSpan{input.subspan(result.consumed, count), result.consumed};
        spans::OutputSpan<T> outSpan{output.subspan(result.produced, count), result.produced};
        outSpan.isConnected = connected;
        std::ignore         = block.processBulk(inSpan, outSpan);

        result.consumed += inSpan.consumed;
        result.produced += outSpan.count;
        result.largestChunk = std::max(result.largestChunk, inSpan.consumed);
        ++result.calls;
        perCall(result.calls);
        if (inSpan.consumed == 0UZ) {
            break;
        }
    }
    return result;
}

template<typename T>
Pump pump(Throttle<T>& block, std::span<const T> input, std::span<T> output, std::size_t chunkSize, bool connected = true) {
    return pump(block, input, output, chunkSize, connected, [](std::size_t) {});
}

template<typename T>
[[nodiscard]] std::vector<T> ramp(std::size_t nSamples) {
    std::vector<T> input(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        input[i] = static_cast<T>(i % 251UZ);
    }
    return input;
}

template<typename T, typename FBody>
[[nodiscard]] double timed(FBody&& body) {
    const auto start = Clock::now();
    body();
    return std::chrono::duration<double>(Clock::now() - start).count();
}

} // namespace

const boost::ut::suite<"Throttle"> throttleTests = [] {
    using namespace boost::ut;

    "every sample comes through untouched"_test = []<typename T>() {
        Throttle<T>          block = makeThrottle<T>({{"sample_rate", 1e9f}});
        const std::vector<T> input = ramp<T>(4096UZ);
        std::vector<T>       output(input.size());
        block.restart();

        const Pump result = pump<T>(block, std::span<const T>(input), std::span<T>(output), 333UZ);
        expect(eq(result.consumed, input.size()));
        expect(eq(result.produced, input.size())) << "connected, the block is exactly 1:1";
        expect(std::ranges::equal(output, input)) << "and copies";
    } | std::tuple<float, std::int16_t, std::uint8_t>{};

    "a complex stream comes through untouched too"_test = [] {
        using CF              = std::complex<float>;
        Throttle<CF>    block = makeThrottle<CF>({{"sample_rate", 1e9f}});
        std::vector<CF> input(2048UZ);
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            input[i] = CF{static_cast<float>(i), static_cast<float>(-static_cast<double>(i))};
        }
        std::vector<CF> output(input.size());
        block.restart();

        expect(eq(pump<CF>(block, std::span<const CF>(input), std::span<CF>(output), 512UZ).produced, input.size()));
        expect(std::ranges::equal(output, input));
    };

    "tags arrive at the offsets they left at"_test = [] {
        gr::Graph graph;
        auto&     source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 2048U}, {"mark_tag", false}});
        for (const std::size_t at : {7UZ, 64UZ, 1000UZ}) {
            source._tags.emplace_back(at, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::format("probe{}", at)}});
        }
        // a key outside the auto-forward set, which the block's pass-all policy carries all the same
        source._tags.emplace_back(300UZ, gr::property_map{{gr::property_map::key_type{"private_key"}, gr::pmt::Value(std::string("carried"))}});
        std::ranges::sort(source._tags, std::ranges::less{}, &gr::Tag::index);

        auto& throttle = graph.emplaceBlock<Throttle<float>>({{"sample_rate", 1e9f}});
        auto& sink     = graph.emplaceBlock<gr::blocks::testing::TagSink<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(graph.connect<"out", "in">(source, throttle).has_value());
        expect(graph.connect<"out", "in">(throttle, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        const gr::property_map::key_type probe{gr::tag::TRIGGER_NAME.shortKey()};
        const gr::property_map::key_type priv{"private_key"};
        std::vector<std::size_t>         seen;
        std::vector<std::size_t>         seenPrivate;
        for (const gr::Tag& tag : sink._tags) {
            if (tag.map.contains(probe)) {
                seen.push_back(tag.index);
            }
            if (const auto found = tag.map.find(priv); found != tag.map.end() && found->second == gr::pmt::Value(std::string("carried"))) {
                seenPrivate.push_back(tag.index);
            }
        }
        expect(that % (seen == std::vector<std::size_t>{7UZ, 64UZ, 1000UZ})) << "a 1:1 block moves no tag";
        expect(that % (seenPrivate == std::vector<std::size_t>{300UZ})) << "and a non-reserved key arrives once, at its own offset";
    };

    "the long-run average rate is the configured one"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", kRate}});
        const std::vector<float> input(50000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        const double elapsed = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 4096UZ); });
        expect(ge(elapsed, 0.9 * kFullSet)) << "measured " << elapsed << " s against " << kFullSet;
        expect(le(elapsed, 1.5 * kFullSet)) << "measured " << elapsed << " s against " << kFullSet;
    };

    "the schedule is absolute, so the chunk size does not change the total"_test = [] {
        std::vector<double>      elapsed;
        const std::vector<float> input(20000UZ, 0.f);
        std::vector<float>       output(input.size());

        for (const std::size_t chunkSize : {256UZ, 8192UZ}) {
            Throttle<float> block = makeThrottle<float>({{"sample_rate", kRate}});
            block.restart();
            elapsed.push_back(timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), chunkSize); }));
        }
        const double spread = std::abs(elapsed[0] - elapsed[1]) / std::min(elapsed[0], elapsed[1]);
        expect(lt(spread, 0.2)) << "256-sample chunks took " << elapsed[0] << " s, 8192-sample chunks " << elapsed[1] << " s";
    };

    "a stall is absorbed rather than repaid"_test = [] {
        constexpr double         kStall = 0.15;
        const std::vector<float> input(20000UZ, 0.f);
        std::vector<float>       output(input.size());

        Throttle<float> clean = makeThrottle<float>({{"sample_rate", kRate}});
        clean.restart();
        const double straight = timed<float>([&] { std::ignore = pump<float>(clean, std::span<const float>(input), std::span<float>(output), 1000UZ); });

        Throttle<float> stalled = makeThrottle<float>({{"sample_rate", kRate}});
        stalled.restart();
        const double interrupted = timed<float>([&] {
            std::ignore = pump<float>(stalled, std::span<const float>(input), std::span<float>(output), 1000UZ, true, [kStall](std::size_t call) {
                if (call == 10UZ) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(kStall));
                }
            });
        });

        expect(ge(interrupted, straight)) << "the stall cannot make the run shorter";
        expect(le(interrupted - straight, 1.5 * kStall)) << "the stall cost " << (interrupted - straight) << " s, which must be the stall and not a recovered backlog";
    };

    "max_items_per_chunk bounds what one invocation handles"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", 1e9f}, {"max_items_per_chunk", 64U}});
        const std::vector<float> input(2000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        const Pump result = pump<float>(block, std::span<const float>(input), std::span<float>(output), 1000UZ);
        expect(eq(result.largestChunk, 64UZ)) << "no invocation may take more than the cap";
        expect(eq(result.consumed, input.size())) << "and a short chunk is still handled rather than waited for";
    };

    "a rate change restarts the schedule"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", 10000.f}});
        const std::vector<float> input(10000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        std::ignore = pump<float>(block, std::span<const float>(input).first(1000UZ), std::span<float>(output), 1000UZ);
        expect(eq(block.paced(), 1000ULL));

        std::ignore = block.settings().setStaged({{"sample_rate", kRate}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.paced(), 0ULL)) << "the origin and the count both move to the moment of the change";

        const double elapsed = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 4096UZ); });
        expect(ge(elapsed, 0.07)) << "the second phase must be paced at the new rate, not released instantly: " << elapsed << " s";
        expect(le(elapsed, 0.25)) << "and it must not stall: " << elapsed << " s";
    };

    "an unconnected output still paces the consumption"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", kRate}});
        const std::vector<float> input(20000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        Pump         result;
        const double elapsed = timed<float>([&] { result = pump<float>(block, std::span<const float>(input), std::span<float>(output), 4096UZ, false); });
        expect(eq(result.consumed, input.size()));
        expect(eq(result.produced, 0UZ)) << "nothing is published when nothing is listening";
        expect(ge(elapsed, 0.9 * 0.2)) << "and the consumption is paced all the same: " << elapsed << " s";
    };

    "a stop reaches a sleeping block within one slice"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", 1.f}, {"max_sleep_s", 0.05}});
        const std::vector<float> input(100UZ, 0.f); // a hundred seconds of deadline
        std::vector<float>       output(input.size());
        block.restart();

        std::jthread stopper([&block] {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            block.requestStop();
        });
        const double elapsed = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 100UZ); });
        expect(lt(elapsed, 0.3)) << "a single uninterruptible sleep would have held this for a hundred seconds; took " << elapsed << " s";
    };

    "the waiting costs no processor time"_test = [] {
        Throttle<float>          block = makeThrottle<float>({{"sample_rate", kRate}});
        const std::vector<float> input(25000UZ, 0.f);
        std::vector<float>       output(input.size());
        block.restart();

        const std::clock_t startCpu = std::clock();
        const double       elapsed  = timed<float>([&] { std::ignore = pump<float>(block, std::span<const float>(input), std::span<float>(output), 512UZ); });
        const double       cpu      = static_cast<double>(std::clock() - startCpu) / static_cast<double>(CLOCKS_PER_SEC);
        expect(lt(cpu, 0.2 * elapsed)) << "used " << cpu << " s of processor time over " << elapsed << " s of wall clock: that is a spin loop";
    };

    "unusable parameters are rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", 0.f}}); })) << "zero rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", -1.f}}); })) << "negative rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", std::numeric_limits<float>::quiet_NaN()}}); })) << "NaN rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"sample_rate", std::numeric_limits<float>::infinity()}}); })) << "infinite rate";
        expect(throws([] { std::ignore = makeThrottle<float>({{"max_sleep_s", 0.0}}); })) << "zero slice";
        expect(throws([] { std::ignore = makeThrottle<float>({{"max_sleep_s", -1.0}}); })) << "negative slice";
    };
};

int main() { /* tests are automatically registered and run */ }
