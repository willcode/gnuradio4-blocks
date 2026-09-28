#include <boost/ut.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/Delay.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

const boost::ut::suite<"testing flow blocks"> testingFlowBlocks = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;
    constexpr auto kTestTypes = std::tuple<uint8_t, int16_t, int32_t, float>();

    "ConstantSource->Copy->CountingSink"_test = []<typename T>(const T&) {
        constexpr std::uint64_t                       N     = 10;
        constexpr typename ConstantSource<T>::value_t value = typename ConstantSource<T>::value_t(7);

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<ConstantSource<T>>(property_map{{"default_value", value}, {"n_samples_max", N}});
        auto&                 copy = test.emplace<Copy<T>>();
        auto&                 sink = test.emplace<CountingSink<T>>(property_map{{"n_samples_max", N}});

        expect(test.connect(src, "out", copy, "in").has_value());
        expect(test.connect(copy, "out", sink, "in").has_value());

        expect(test.run().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "CountingSource->HeadBlock->CountingSink"_test = []<typename T>(const T&) {
        constexpr std::uint64_t N_total     = 10;
        constexpr std::uint64_t N_head      = 4;
        constexpr T             start_value = T(5);

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<CountingSource<T>>(property_map{{"default_value", start_value}, {"n_samples_max", N_total}});
        auto&                 head = test.emplace<HeadBlock<T>>(property_map{{"n_samples_max", N_head}});
        auto&                 sink = test.emplace<CountingSink<T>>(property_map{{"n_samples_max", N_head}});

        expect(test.connect(src, "out", head, "in").has_value());
        expect(test.connect(head, "out", sink, "in").has_value());

        expect(test.run().has_value());
        expect(eq(sink.count, N_head));
    } | kTestTypes;

    "TagSource mark_tag publishes only 0 and 1"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 16;

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<TagSource<T, ProcessFunction::USE_PROCESS_BULK>>(property_map{{"n_samples_max", N}, {"mark_tag", true}});
        auto&                 sink = test.emplace<TagSink<T, ProcessFunction::USE_PROCESS_ONE>>(property_map{{"n_samples_expected", N}});
        src._tags                  = {{3UZ, {{"key", "a"}}}, {11UZ, {{"key", "b"}}}};

        expect(test.connect(src, "out", sink, "in").has_value());

        expect(test.run().has_value());

        expect(eq(sink._samples.size(), static_cast<std::size_t>(N)));
        std::vector<T> expected(N, T(0));
        expected[3]  = T(1);
        expected[11] = T(1);
        expect(std::ranges::equal(sink._samples, expected)) << std::format("mark_tag must publish 1 at tagged offsets and 0 everywhere else, got {}", sink._samples);
    } | kTestTypes;

    "TagMonitor reports the stream without rewriting it"_test = [] {
        using Monitor                   = TagMonitor<float, ProcessFunction::USE_PROCESS_ONE>;
        constexpr gr::Size_t N          = 16;
        constexpr float      streamRate = 4242.f;

        // the monitor owns none of the reserved keys, so it neither substitutes its own value into a forwarded tag
        // nor tracks a passing one
        const auto& writable = gr::CtxSettings<Monitor>::allWritableMembers();
        expect(!writable.contains(std::string(gr::tag::SAMPLE_RATE.shortKey()))) << "a monitor must not own the stream keys it observes";
        expect(!writable.contains(std::string(gr::tag::SIGNAL_NAME.shortKey()))) << "a monitor must not own the stream keys it observes";

        gr::test::RuntimeTest test;
        auto&                 src     = test.emplace<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>(property_map{{"n_samples_max", N}, {gr::tag::SAMPLE_RATE.shortKey(), 1000.f}});
        auto&                 monitor = test.emplace<Monitor>(property_map{{"n_samples_expected", N}});
        auto&                 sink    = test.emplace<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>(property_map{{"n_samples_expected", N}});
        src._tags                     = {{6UZ, {{gr::tag::SAMPLE_RATE.shortKey(), streamRate}}}};

        expect(test.connect(src, "out", monitor, "in").has_value());
        expect(test.connect(monitor, "out", sink, "in").has_value());

        expect(test.run().has_value());

        std::size_t nSeen = 0UZ;
        for (const auto& tag : sink._tags) {
            const auto it = tag.map.find(gr::tag::SAMPLE_RATE.shortKey());
            if (it == tag.map.end() || it->second.value_or(0.f) == 1000.f) {
                continue; // the settings-forward tag carries the source's own rate
            }
            ++nSeen;
            expect(eq(it->second.value_or(0.f), streamRate)) << "the monitor rewrote the sample rate it forwarded";
        }
        expect(eq(nSeen, 1UZ)) << std::format("the sample_rate tag did not cross the monitor, sink saw {} tags", sink._tags.size());
    };

    "Delay carries every tag key once, at its own offset"_test = [] {
        const gr::property_map::key_type key{"private_key"};
        const gr::pmt::Value             value{std::string("carried")};

        gr::test::RuntimeTest test;
        auto&                 src   = test.emplace<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>(property_map{{"n_samples_max", gr::Size_t(2048)}, {"mark_tag", false}});
        auto&                 delay = test.emplace<Delay<float>>(property_map{{"delay_ms", std::uint32_t(50)}});
        auto&                 sink  = test.emplace<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>(property_map{{"name", "TagSink"}});
        for (const std::size_t at : {0UZ, 7UZ, 300UZ}) {
            src._tags.emplace_back(at, property_map{{key, value}});
        }

        expect(test.connect(src, "out", delay, "in").has_value());
        expect(test.connect(delay, "out", sink, "in").has_value());

        expect(test.run().has_value());

        std::vector<std::size_t> offsets;
        for (const gr::Tag& tag : sink._tags) {
            if (const auto found = tag.map.find(key); found != tag.map.end() && found->second == value) {
                offsets.push_back(tag.index);
            }
        }
        // the tag at 0 is the one the hold would republish on every polled call, so the multiplicity is the assertion
        expect(that % (offsets == std::vector<std::size_t>{0UZ, 7UZ, 300UZ})) << "a pass-all pass-through neither drops, moves nor duplicates a private key";
    };

    // Tag::index is a std::size_t; a 32-bit target cannot record an index past 2^32
    if constexpr (sizeof(std::size_t) >= 8UZ) {
        "TagMonitor and TagSink record a tag past 2^32 at its index"_test = [] {
            using Monitor = TagMonitor<float, ProcessFunction::USE_PROCESS_ONE>;
            using Sink    = TagSink<float, ProcessFunction::USE_PROCESS_ONE>;
            // both counters start six below 2^32 and the tag arrives seven samples later; a 32-bit counter records it at 1
            constexpr std::uint64_t kStart     = 4'294'967'290U;
            constexpr gr::Size_t    N          = 10U;
            constexpr std::size_t   kTagOffset = 7UZ;
            static_assert(kStart + kTagOffset > std::numeric_limits<std::uint32_t>::max());

            Graph g;
            auto& src     = g.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>(property_map{{"n_samples_max", N}, {"mark_tag", false}});
            auto& monitor = g.emplaceBlock<Monitor>();
            auto& sink    = g.emplaceBlock<Sink>();
            src._tags     = {{kTagOffset, {{gr::tag::TRIGGER_NAME.shortKey(), "past"}}}};

            monitor._nSamplesProduced = kStart;
            sink._nSamplesProduced    = kStart;

            expect(g.connect<"out", "in">(src, monitor).has_value());
            expect(g.connect<"out", "in">(monitor, sink).has_value());

            gr::scheduler::Simple sch;
            if (auto ret = sch.exchange(std::move(g)); !ret) {
                throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
            }
            expect(sch.runAndWait().has_value());

            const auto hasKey = [](const Tag& tag) { return tag.map.contains(gr::tag::TRIGGER_NAME.shortKey()); };
            for (const auto* tags : {&monitor._tags, &sink._tags}) {
                const auto it = std::ranges::find_if(*tags, hasKey);
                expect(fatal(it != tags->end())) << "the tag did not arrive";
                expect(eq(it->index, kStart + kTagOffset));
            }
            expect(eq(monitor._nSamplesProduced, kStart + N));
            expect(eq(sink._nSamplesProduced, kStart + N));
        };
    }
};

int main() { /* not needed for UT */ }
