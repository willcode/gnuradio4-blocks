#include <boost/ut.hpp>

#include <algorithm>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/algorithm/ImChart.hpp>
#include <gnuradio-4.0/basic/ClockSource.hpp>
#include <gnuradio-4.0/basic/FunctionGenerator.hpp>
#include <gnuradio-4.0/basic/SignalGenerator.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

const boost::ut::suite TagTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;
    using namespace gr::blocks::testing;

    auto clockSourceTest = []<bool useIoThreadPool>(bool verbose = false) {
        // useIoThreadPool - true: scheduler/graph-provided thread, false: use user-provided call-back or thread
        if (verbose) {
            std::println("started ClockSource test w/ {}", useIoThreadPool ? "Graph/Block<T> provided-thread" : "user-provided thread");
        }
        constexpr gr::Size_t n_samples   = 1900;
        constexpr float      sample_rate = 2000.f;
        Graph                testGraph;
        auto&                src = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", n_samples}, {"name", "ClockSource"}, {"verbose_console", verbose}});
        src.tags                 = {
            {0, {{"key", "value@0"}}},       //
            {1, {{"key", "value@1"}}},       //
            {100, {{"key", "value@100"}}},   //
            {150, {{"key", "value@150"}}},   //
            {1000, {{"key", "value@1000"}}}, //
            {1001, {{"key", "value@1001"}}}, //
            {1002, {{"key", "value@1002"}}}, //
            {1023, {{"key", "value@1023"}}}  //
        };
        auto& sink1 = testGraph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink1"}});
        auto& sink2 = testGraph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_BULK>>({{"name", "TagSink2"}});
        expect(testGraph.connect<"out", "in">(src, sink1).has_value());
        expect(testGraph.connect<"out", "in">(src, sink2).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        if (verbose) {
            std::println("finished ClockSource sched.runAndWait() w/ {}", useIoThreadPool ? "Graph/Block<T> provided-thread" : "user-provided thread");
        }

        expect(eq(src.settings.n_samples_max, n_samples)) << "src did not accept require max input samples";
        expect(eq(src.n_samples_produced, n_samples)) << "src did not produce enough output samples";
        expect(eq(static_cast<gr::Size_t>(sink1._nSamplesProduced), n_samples)) << std::format("sink1 did not consume enough input samples ({} vs. {})", sink1._nSamplesProduced, n_samples);
        expect(eq(static_cast<gr::Size_t>(sink2._nSamplesProduced), n_samples)) << std::format("sink2 did not consume enough input samples ({} vs. {})", sink2._nSamplesProduced, n_samples);

        // TODO: last decimator/interpolator + stride addition seems to break the limiting the input samples to the min of available vs. n_samples-until next tags
        // expect(equal_tag_lists(src.tags, sink1._tags)) << "sink1 (USE_PROCESS_ONE) did not receive the required tags";
        // expect(equal_tag_lists(src.tags, sink2._tags)) << "sink2 (USE_PROCESS_BULK) did not receive the required tags";
        if (verbose) {
            std::println("finished ClockSource test w/ {}", useIoThreadPool ? "Graph/Block<T>-provided thread" : "user-provided thread");
        }
    };

    "ClockSource w/ Graph/Block<T> provided thread"_test = [&clockSourceTest] { clockSourceTest.template operator()<true>(false); };
    "ClockSource verbose console"_test                   = [&clockSourceTest] { clockSourceTest.template operator()<true>(true); };
    // "ClockSource w/ user-provided thread"_test           = [&clockSourceTest] { clockSourceTest.template operator()<false>(false); }; // TODO: check potential threading issue for user-provided threads

    "SignalGenerator test"_test = [] {
        const std::size_t        N      = 16; // test points
        const double             offset = 2.;
        std::vector<std::string> signals{"constant", "sin", "cos", "square", "saw", "triangle"};

        for (const auto& sig : signals) {
            SignalGenerator<double> signalGen({{"signal_type", sig}, {gr::tag::SAMPLE_RATE.name(), 2048.f}, {"tone_frequency", 256.}, {"amplitude", 1.}, {"offset", offset}, {"phase", std::numbers::pi / 4}});
            expect(signalGen.check({}).has_value());

            // expected values corresponds to sample_rate = 1024., tone_frequency = 128., amplitude = 1., offset = 0., phase = pi/4.
            std::map<std::string, std ::vector<double>> expResults = {{"constant", {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.}}, {"sin", {0.707106, 1., 0.707106, 0., -0.707106, -1., -0.707106, 0., 0.707106, 1., 0.707106, 0., -0.707106, -1., -0.707106, 0.}}, {"cos", {0.707106, 0., -0.707106, -1., -0.7071067, 0., 0.707106, 1., 0.707106, 0., -0.707106, -1., -0.707106, 0., 0.707106, 1.}}, {"square", {1., 1., 1., -1., -1., -1., -1., 1., 1., 1., 1., -1., -1., -1., -1., 1.}}, {"saw", {0.25, 0.5, 0.75, -1., -0.75, -0.5, -0.25, 0., 0.25, 0.5, 0.75, -1., -0.75, -0.5, -0.25, 0.}}, {"triangle", {0.5, 1., 0.5, 0., -0.5, -1., -0.5, 0., 0.5, 1., 0.5, 0., -0.5, -1., -0.5, 0.}}};

            for (std::size_t i = 0; i < N; i++) {
                const auto val = signalGen.generateSample();
                const auto exp = expResults[sig][i] + offset;
                expect(approx(exp, val, 1e-5)) << std::format("SignalGenerator for signal: {} and i: {} does not match.", sig, i);
            }
        }
    };

    "SignalGenerator FastSin/FastCos match Sin/Cos"_test = [] {
        constexpr std::size_t N = 200;
        for (const auto& [fast, precise] : std::vector<std::pair<std::string, std::string>>{{"fast_sin", "sin"}, {"fast_cos", "cos"}}) {
            SignalGenerator<double> genFast({{"signal_type", fast}, {gr::tag::SAMPLE_RATE.name(), 1000.f}, {"tone_frequency", 50.f}, {"amplitude", 1.f}, {"offset", 0.f}, {"phase", 0.f}});
            SignalGenerator<double> genPrecise({{"signal_type", precise}, {gr::tag::SAMPLE_RATE.name(), 1000.f}, {"tone_frequency", 50.f}, {"amplitude", 1.f}, {"offset", 0.f}, {"phase", 0.f}});
            expect(genFast.check({}).has_value());
            expect(genPrecise.check({}).has_value());
            for (std::size_t i = 0; i < N; ++i) {
                expect(approx(genPrecise.generateSample(), genFast.generateSample(), 1e-10)) << std::format("{} vs {} at sample {}", fast, precise, i);
            }
        }
    };

    "SignalGenerator noise types produce bounded output"_test = [] {
        constexpr std::size_t N = 10000;
        for (const auto& noiseType : {"uniform_noise", "triangular_noise", "gaussian_noise"}) {
            SignalGenerator<double> gen({{"signal_type", std::string(noiseType)}, {gr::tag::SAMPLE_RATE.name(), 1000.f}, {"amplitude", 2.f}, {"offset", 1.f}, {"seed", std::uint64_t(42)}});
            expect(gen.check({}).has_value());
            double sum = 0.;
            for (std::size_t i = 0; i < N; ++i) {
                const double v = gen.generateSample();
                sum += v;
                if (std::string_view(noiseType) != "gaussian_noise") {
                    expect(ge(v, -2. + 1.)) << std::format("{} sample {} out of range: {}", noiseType, i, v);
                    expect(le(v, 2. + 1.)) << std::format("{} sample {} out of range: {}", noiseType, i, v);
                }
            }
            const double mean = sum / static_cast<double>(N);
            expect(approx(1., mean, 0.2)) << std::format("{} mean {} far from offset 1.0", noiseType, mean);
        }
    };

    "SignalGenerator integer output clamps correctly"_test = [] {
        SignalGenerator<std::int16_t> gen({{"signal_type", "sin"}, {gr::tag::SAMPLE_RATE.name(), 1000.f}, {"tone_frequency", 50.f}, {"amplitude", 40000.f}, {"offset", 0.f}});
        expect(gen.check({}).has_value());
        bool hitMax = false, hitMin = false;
        for (std::size_t i = 0; i < 100; ++i) {
            const auto v = gen.generateSample();
            expect(ge(v, std::numeric_limits<std::int16_t>::min()));
            expect(le(v, std::numeric_limits<std::int16_t>::max()));
            if (v == std::numeric_limits<std::int16_t>::max()) {
                hitMax = true;
            }
            if (v == std::numeric_limits<std::int16_t>::min()) {
                hitMin = true;
            }
        }
        expect(hitMax) << "amplitude 40000 should clamp to int16 max";
        expect(hitMin) << "amplitude 40000 should clamp to int16 min";
    };

    "SignalGenerator complex output produces analytic signal"_test = [] {
        SignalGenerator<std::complex<double>> gen({{"signal_type", "sin"}, {gr::tag::SAMPLE_RATE.name(), 1000.f}, {"tone_frequency", 50.f}, {"amplitude", 1.f}, {"offset", 0.f}, {"phase", 0.f}});
        expect(gen.check({}).has_value());
        for (std::size_t i = 0; i < 50; ++i) {
            const auto   sample = gen.generateSample();
            const double mag    = std::abs(sample);
            expect(approx(1., mag, 1e-10)) << std::format("analytic signal magnitude at sample {} should be ~1.0, got {}", i, mag);
        }
    };

    "SignalGenerator ImChart test"_test = [] {
        const std::size_t        N = 512; // test points
        std::vector<std::string> signals{"constant", "sin", "cos", "square", "saw", "triangle"};
        for (const auto& sig : signals) {
            SignalGenerator<double> signalGen({{"signal_type", sig}, {gr::tag::SAMPLE_RATE.name(), 8192.f}, {"tone_frequency", 32.}, {"amplitude", 2.}, {"offset", 0.}, {"phase", std::numbers::pi / 4.}});
            expect(signalGen.check({}).has_value());

            std::vector<double> xValues(N), yValues(N);
            std::iota(xValues.begin(), xValues.end(), 0);
            std::ranges::generate(yValues, [&signalGen]() { return signalGen.generateSample(); });

            std::println("Chart {}\n\n", sig);
            auto chart = gr::graphs::ImChart<128, 16>({{0., static_cast<double>(N)}, {-2.6, 2.6}});
            chart.draw(xValues, yValues, sig);
            chart.draw();
        }
    };

    "SignalGenerator + ClockSource test"_test = [] {
        constexpr gr::Size_t n_samples   = 200;
        constexpr float      sample_rate = 1000.f;
        Graph                testGraph;
        auto&                clockSrc  = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", n_samples}, {"name", "ClockSource"}});
        auto&                signalGen = testGraph.emplaceBlock<SignalGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "SignalGenerator"}});
        auto&                sink      = testGraph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}, {"verbose_console", true}});

        expect(testGraph.connect<"out", "clk_in">(clockSrc, signalGen).has_value());
        expect(testGraph.connect<"out", "in">(signalGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());

        expect(eq(n_samples, static_cast<std::uint32_t>(sink._nSamplesProduced))) << "Number of samples does not match";
    };

    "FunctionGenerator ImChart test"_test = [] {
        using namespace function_generator;
        const std::size_t       N          = 128; // test points
        double                  startValue = 10.;
        double                  finalValue = 20.;
        std::vector<SignalType> signals{SignalType::constant, SignalType::linear_ramp, SignalType::parabolic_ramp, SignalType::cubic_spline, SignalType::impulse_response};

        property_map segments;
        for (const auto& sig : signals) {
            segments.insert_or_assign(toString(sig), property_map{createPropertyMapEntry(signal_type, sig), //
                                                         createPropertyMapEntry(start_value, startValue),   //
                                                         createPropertyMapEntry(final_value, finalValue),   //
                                                         createPropertyMapEntry(duration, 1.),              //
                                                         createPropertyMapEntry(round_off_time, .15),       //
                                                         createPropertyMapEntry(impulse_time0, .2),         //
                                                         createPropertyMapEntry(impulse_time1, .15)});
        }
        FunctionGenerator<double> funcGen({{gr::tag::SAMPLE_RATE.name(), 128.f}, {"segments", segments}});
        expect(funcGen.check({}).has_value());

        for (const auto& sig : signals) {
            expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segment", toString(sig)}}).empty()) << "a set of the selector puts the segment in force";
            expect(eq(funcGen._signal.signal_type, sig));

            std::vector<double> xValues(N), yValues(N);
            std::iota(xValues.begin(), xValues.end(), 0);
            std::ranges::generate(yValues, [&funcGen]() { return funcGen.generateSample(); });
            std::println("Chart {}\n\n", std::string_view(toString(sig)));
            auto chart = gr::graphs::ImChart<128, 32>({{0., static_cast<double>(N)}, {7., 22.}});
            chart.draw(xValues, yValues, toString(sig));
            chart.draw();
        }
    };

    "FunctionGenerator + ClockSource test"_test = [] {
        using namespace std::string_literals;
        using namespace function_generator;
        constexpr std::uint32_t N           = 1000;
        constexpr float         sample_rate = 1000.f;
        Graph                   testGraph;
        auto&                   clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});

        clockSrc.tags = {Tag(0, {{tag::CONTEXT.key(), "1"s}}), //
            Tag(100, {{tag::CONTEXT.key(), "2"s}}),            //
            Tag(300, {{tag::CONTEXT.key(), "3"s}}),            //
            Tag(350, {{tag::CONTEXT.key(), "4"s}}),            //
            Tag(550, {{tag::CONTEXT.key(), "5"s}}),            //
            Tag(650, {{tag::CONTEXT.key(), "6"s}}),            //
            Tag(800, {{tag::CONTEXT.key(), "7"s}}),            //
            Tag(850, {{tag::CONTEXT.key(), "8"s}})};

        const property_map segments{{"1", createConstPropertyMap(5.f)}, {"2", createLinearRampPropertyMap(5.f, 30.f, .2f)}, {"3", createConstPropertyMap(30.f)},      //
            {"4", createParabolicRampPropertyMap(30.f, 20.f, .1f, 0.02f)}, {"5", createConstPropertyMap(20.f)}, {"6", createCubicSplinePropertyMap(20.f, 10.f, .1f)}, //
            {"7", createConstPropertyMap(10.f)}, {"8", createImpulseResponsePropertyMap(10.f, 20.f, .02f, .06f)}};
        auto&              funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}, {"segments", segments}});
        // the context the clock's tags carry selects the segment
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}}).has_value());

        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(eq(N, static_cast<std::uint32_t>(sink._samples.size()))) << "Number of samples does not match";
        expect(eq(sink._tags.size(), clockSrc.tags.size())) << [&]() {
            std::string ret = std::format("DataSet nTags: {}\n", sink._tags.size());
            for (const auto& tag : sink._tags) {
                ret += std::format("tag.index: {} .map: {}\n", tag.index, tag.map);
            }
            return ret;
        }();
        expect(fatal(eq(sink._samples.size(), static_cast<std::size_t>(N))));
        expect(eq(sink._samples[50UZ], 5.f)) << "the first segment holds 5";
        expect(eq(sink._samples[325UZ], 30.f)) << "the third segment holds 30";
        expect(eq(sink._samples[600UZ], 20.f)) << "the fifth segment holds 20";

        std::println("\n\nChart FunctionGenerator + ClockSource test\n\n");
        std::vector<double> xValues(N);
        std::vector<double> yValues(sink._samples.begin(), sink._samples.end());
        std::iota(xValues.begin(), xValues.end(), 0);
        auto chart = gr::graphs::ImChart<128, 32>({{0., static_cast<double>(N)}, {0., 35.}});
        chart.draw(xValues, yValues, "Signal");
        chart.draw();
    };

    "FunctionGenerator + ClockSource FAIR test"_test = [] {
        using namespace function_generator;
        constexpr std::uint64_t lengthSeconds = 1;
        constexpr std::uint32_t N             = 1000 * lengthSeconds;
        constexpr float         sample_rate   = 1'000.f;

        Graph testGraph;
        auto& clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});

        std::vector<std::uint64_t> tagTimes;
        std::vector<std::string>   tagValues;
        auto                       addTimeTagEntry = [&tagTimes, &tagValues](auto&, std::uint64_t timeInNanoseconds, const std::string& value) {
            tagTimes.push_back(timeInNanoseconds);
            tagValues.push_back(value);
        };

        // all times are in nanoseconds
        constexpr std::uint64_t ms = 1'000'000;                                       // ms -> ns
        addTimeTagEntry(clockSrc, 10 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=1"); // <trigger_name>/<ctx>
        addTimeTagEntry(clockSrc, 100 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=2");
        addTimeTagEntry(clockSrc, 300 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=3");
        addTimeTagEntry(clockSrc, 350 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=4");
        addTimeTagEntry(clockSrc, 550 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=5");
        addTimeTagEntry(clockSrc, 650 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=6");
        addTimeTagEntry(clockSrc, 800 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=7");
        addTimeTagEntry(clockSrc, 850 * ms, "CMD_BP_START/FAIR.SELECTOR.C=1:S=1:P=8");
        // a block's settings change by a settings call, not by a write of its member
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(clockSrc), {{"tag_times", tagTimes}, {"tag_values", tagValues}, {"repeat_period", 5'000 * ms}, {"do_zero_order_hold", true}}).empty());

        // all times are in seconds
        const property_map segments{{"FAIR.SELECTOR.C=1:S=1:P=1", createConstPropertyMap(5.f)}, {"FAIR.SELECTOR.C=1:S=1:P=2", createLinearRampPropertyMap(5.f, 30.f, .2f)}, //
            {"FAIR.SELECTOR.C=1:S=1:P=3", createConstPropertyMap(30.f)}, {"FAIR.SELECTOR.C=1:S=1:P=4", createParabolicRampPropertyMap(30.f, 20.f, .1f, 0.02f)},             //
            {"FAIR.SELECTOR.C=1:S=1:P=5", createConstPropertyMap(20.f)}, {"FAIR.SELECTOR.C=1:S=1:P=6", createCubicSplinePropertyMap(20.f, 10.f, .1f)},                      //
            {"FAIR.SELECTOR.C=1:S=1:P=7", createConstPropertyMap(10.f)}, {"FAIR.SELECTOR.C=1:S=1:P=8", createImpulseResponsePropertyMap(10.f, 20.f, .02f, .06f)}};
        auto&              funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}});
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", segments}}).empty());
        // the context of the clock's timing events selects the segment
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}}).has_value());
        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(eq(N, static_cast<std::uint32_t>(sink._samples.size()))) << "Number of samples does not match";
        expect(ge(sink._tags.size(), 8UZ)) << [&]() {
            std::string ret = std::format("DataSet nTags: {}\n", sink._tags.size());
            for (const auto& tag : sink._tags) {
                ret += std::format("tag.index: {} .map: {}\n", tag.index, tag.map);
            }
            return ret;
        }();
        // the third segment holds 30 from its timing event at 300 ms to the next at 350 ms
        const std::size_t inThird = static_cast<std::size_t>(325.f * sample_rate / 1000.f);
        expect(fatal(lt(inThird, sink._samples.size())));
        expect(eq(sink._samples[inThird], 30.f)) << "the timing event's context selects its segment";

        std::println("\n\nFunctionGenerator + ClockSource FAIR test\n\n");
        std::vector<double> xValues(N);
        std::vector<double> yValues(sink._samples.begin(), sink._samples.end());
        std::iota(xValues.begin(), xValues.end(), 0);
        auto chart = gr::graphs::ImChart<128, 32>({{0., static_cast<double>(N)}, {0., 35.}});
        chart.draw(xValues, yValues, "Signal");
        chart.draw();
    };

    "ClockSource repeat_period emits tags across cycles"_test = [] {
        constexpr std::uint64_t ms          = 1'000'000; // ns
        constexpr float         sample_rate = 1'000.f;
        constexpr std::uint64_t repeatNs    = 500 * ms; // 500ms = 500 samples per cycle
        constexpr gr::Size_t    nCycles     = 3;
        constexpr gr::Size_t    N           = nCycles * 500; // 1500 samples

        Graph testGraph;
        auto& clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});

        std::vector<std::uint64_t> tagTimes;
        std::vector<std::string>   tagValues;
        auto                       addTimeTagEntry = [&tagTimes, &tagValues](auto&, std::uint64_t timeInNanoseconds, const std::string& value) {
            tagTimes.push_back(timeInNanoseconds);
            tagValues.push_back(value);
        };
        addTimeTagEntry(clockSrc, 10 * ms, "CMD_BP_START/ctx1");
        addTimeTagEntry(clockSrc, 100 * ms, "CMD_BP_START/ctx2");
        addTimeTagEntry(clockSrc, 200 * ms, "CMD_BP_START/ctx3");
        addTimeTagEntry(clockSrc, 400 * ms, "CMD_BP_START/ctx4");
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(clockSrc), {{"tag_times", tagTimes}, {"tag_values", tagValues}, {"repeat_period", repeatNs}}).empty());

        auto& sink = testGraph.emplaceBlock<TagSink<std::uint8_t, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "in">(clockSrc, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(eq(N, static_cast<gr::Size_t>(sink._nSamplesProduced))) << "sample count mismatch";

        constexpr std::size_t tagsPerCycle = 4;
        expect(ge(sink._tags.size(), nCycles * tagsPerCycle)) << [&]() {
            std::string ret = std::format("expected >= {} tags ({}x{}), got {}:\n", nCycles * tagsPerCycle, nCycles, tagsPerCycle, sink._tags.size());
            for (const auto& tag : sink._tags) {
                ret += std::format("  tag.index: {} .map: {}\n", tag.index, tag.map);
            }
            return ret;
        }();

        // verify tags appear in each cycle (each cycle = 500 samples)
        for (gr::Size_t cycle = 0; cycle < nCycles; ++cycle) {
            const auto cycleStart   = cycle * 500UZ;
            const auto cycleEnd     = cycleStart + 500UZ;
            const auto nTagsInCycle = static_cast<std::size_t>(std::ranges::count_if(sink._tags, [&](const auto& tag) { return tag.index >= cycleStart && tag.index < cycleEnd; }));
            expect(ge(nTagsInCycle, tagsPerCycle)) << std::format("cycle {}: expected >= {} tags in [{}, {}), got {}", cycle, tagsPerCycle, cycleStart, cycleEnd, nTagsInCycle);
        }
    };

    "FunctionGenerator tone types produce correct waveform"_test = [] {
        using namespace function_generator;
        constexpr float sampleRate = 1000.f;
        constexpr float freq       = 50.f;
        constexpr float amplitude  = 3.f;
        constexpr float offset     = 1.f;
        constexpr float phase      = 0.f;

        for (const auto& toneType : {SignalType::sin, SignalType::cos, SignalType::fast_sin, SignalType::fast_cos}) {
            FunctionGenerator<double> funcGen({{gr::tag::SAMPLE_RATE.name(), sampleRate}});
            expect(funcGen.check({}).has_value());

            property_map params{createPropertyMapEntry(signal_type, toneType), createPropertyMapEntry(ParameterType::tone_frequency, freq), createPropertyMapEntry(final_value, amplitude), createPropertyMapEntry(start_value, offset), createPropertyMapEntry(ParameterType::phase, phase), createPropertyMapEntry(duration, 0.f)};
            expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", params}}}, {"segment", "1"}}).empty()) << "the segment is stored and selected in one call";

            constexpr std::size_t N     = 20; // one full period at 50 Hz / 1000 Hz
            double                sumSq = 0.;
            for (std::size_t i = 0; i < N; ++i) {
                const double v = funcGen.generateSample();
                sumSq += (v - static_cast<double>(offset)) * (v - static_cast<double>(offset));
            }
            const double rms = std::sqrt(sumSq / static_cast<double>(N));
            expect(approx(static_cast<double>(amplitude) / std::numbers::sqrt2, rms, 0.15)) << std::format("tone {} RMS mismatch", toString(toneType));
        }
    };

    "FunctionGenerator tone duration expiry holds at offset"_test = [] {
        using namespace function_generator;
        constexpr float sampleRate = 1000.f;

        FunctionGenerator<double> funcGen({{gr::tag::SAMPLE_RATE.name(), sampleRate}});
        expect(funcGen.check({}).has_value());

        property_map params{createPropertyMapEntry(signal_type, SignalType::sin), createPropertyMapEntry(ParameterType::tone_frequency, 100.f), createPropertyMapEntry(final_value, 5.f), // amplitude
            createPropertyMapEntry(start_value, 2.f),                                                                                                                                     // offset
            createPropertyMapEntry(ParameterType::phase, 0.f), createPropertyMapEntry(duration, 0.01f)};                                                                                  // 10 ms = 10 samples
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", params}}}, {"segment", "1"}}).empty()) << "the segment is stored and selected in one call";

        // generate past the duration
        for (std::size_t i = 0; i < 15; ++i) {
            std::ignore = funcGen.generateSample();
        }
        // after expiry, output should hold at start_value (offset = 2.0)
        for (std::size_t i = 0; i < 10; ++i) {
            expect(approx(2., funcGen.generateSample(), 1e-10)) << std::format("sample {} after expiry should be offset", i);
        }
    };

    "FunctionGenerator noise types produce output"_test = [] {
        using namespace function_generator;
        constexpr float sampleRate = 1000.f;

        for (const auto& noiseType : {SignalType::uniform_noise, SignalType::triangular_noise, SignalType::gaussian_noise}) {
            FunctionGenerator<double> funcGen({{gr::tag::SAMPLE_RATE.name(), sampleRate}});
            expect(funcGen.check({}).has_value());

            property_map params{createPropertyMapEntry(signal_type, noiseType), createPropertyMapEntry(start_value, 2.f)};
            expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", params}}}, {"segment", "1"}}).empty()) << "the segment is stored and selected in one call";

            bool nonZero = false;
            for (std::size_t i = 0; i < 100; ++i) {
                if (funcGen.generateSample() != 0.) {
                    nonZero = true;
                }
            }
            expect(nonZero) << std::format("noise type {} should produce non-zero output", toString(noiseType));
        }
    };

    "FunctionGenerator complex output for tone type"_test = [] {
        using namespace function_generator;
        FunctionGenerator<std::complex<double>> funcGen({{gr::tag::SAMPLE_RATE.name(), 1000.f}});
        expect(funcGen.check({}).has_value());

        property_map params{createPropertyMapEntry(signal_type, SignalType::sin), createPropertyMapEntry(ParameterType::tone_frequency, 50.f), createPropertyMapEntry(final_value, 1.f), createPropertyMapEntry(start_value, 0.f), createPropertyMapEntry(ParameterType::phase, 0.f), createPropertyMapEntry(duration, 0.f)};
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", params}}}, {"segment", "1"}}).empty()) << "the segment is stored and selected in one call";

        for (std::size_t i = 0; i < 20; ++i) {
            const auto   sample = funcGen.generateSample();
            const double mag    = std::abs(sample);
            expect(approx(1., mag, 1e-10)) << std::format("analytic signal magnitude at sample {} should be ~1.0, got {}", i, mag);
        }
    };

    "FunctionGenerator integer output for tone type"_test = [] {
        using namespace function_generator;
        FunctionGenerator<std::int16_t> funcGen({{gr::tag::SAMPLE_RATE.name(), 1000.f}});
        expect(funcGen.check({}).has_value());

        property_map params{createPropertyMapEntry(signal_type, SignalType::sin), createPropertyMapEntry(ParameterType::tone_frequency, 50.f), createPropertyMapEntry(final_value, 40000.f), createPropertyMapEntry(start_value, 0.f), createPropertyMapEntry(ParameterType::phase, 0.f), createPropertyMapEntry(duration, 0.f)};
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", params}}}, {"segment", "1"}}).empty()) << "the segment is stored and selected in one call";

        bool hitMax = false, hitMin = false;
        for (std::size_t i = 0; i < 100; ++i) {
            const auto v = funcGen.generateSample();
            expect(ge(v, std::numeric_limits<std::int16_t>::min()));
            expect(le(v, std::numeric_limits<std::int16_t>::max()));
            if (v == std::numeric_limits<std::int16_t>::max()) {
                hitMax = true;
            }
            if (v == std::numeric_limits<std::int16_t>::min()) {
                hitMin = true;
            }
        }
        expect(hitMax) << "amplitude 40000 should clamp to int16 max";
        expect(hitMin) << "amplitude 40000 should clamp to int16 min";
    };

    "FunctionGenerator + ClockSource tone segment test"_test = [] {
        using namespace std::string_literals;
        using namespace function_generator;
        constexpr std::uint32_t N           = 500;
        constexpr float         sample_rate = 1000.f;
        Graph                   testGraph;
        auto&                   clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});

        clockSrc.tags = {Tag(0, {{tag::CONTEXT.key(), "1"s}}), Tag(100, {{tag::CONTEXT.key(), "2"s}}), Tag(300, {{tag::CONTEXT.key(), "3"s}})};

        auto& funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}});
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", createConstPropertyMap(0.f)}, {"2", createSinPropertyMap(50.f, 5.f, 0.f, 1.f, 0.f)}, {"3", createConstPropertyMap(1.f)}}}}).empty());
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}}).has_value());

        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(eq(N, static_cast<std::uint32_t>(sink._samples.size()))) << "number of samples does not match";

        // segment 1 (samples 0–99): Const = 0
        for (std::size_t i = 0; i < 100 && i < sink._samples.size(); ++i) {
            expect(approx(0.f, sink._samples[i], 1e-4f)) << std::format("const segment sample {}", i);
        }
        // segment 2 (samples 100–299): Sin with amplitude=5, offset=1 -> oscillating around 1.0
        double sumSq = 0.;
        for (std::size_t i = 100; i < 300 && i < sink._samples.size(); ++i) {
            sumSq += static_cast<double>((sink._samples[i] - 1.f) * (sink._samples[i] - 1.f));
        }
        const double rms = std::sqrt(sumSq / 200.);
        expect(approx(5. / std::numbers::sqrt2, rms, 0.3)) << std::format("sine segment RMS mismatch: {}", rms);
        // segment 3 (samples 300+): Const = 1
        for (std::size_t i = 300; i < sink._samples.size(); ++i) {
            expect(approx(1.f, sink._samples[i], 1e-4f)) << std::format("const-after-sine segment sample {}", i);
        }
    };

    // a generator driven by an external clock sits mid-stream, so the stream keys crossing it are not its own: the
    // waveform frequency is 'tone_frequency', and the reserved 'gr:frequency' key passes through in both directions
    static constexpr double kStreamFrequency = 100.0e6;

    // the scheduler owns the graph once it is exchanged in, so the blocks are read inside the callback
    const auto runClockedGenerator = []<typename TBlock, typename TCheck>(Graph& testGraph, TBlock& generator, TCheck&& check) {
        constexpr gr::Size_t nSamples = 128U;

        auto& clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), 10000.f}, {"n_samples_max", nSamples}, {"chunk_size", gr::Size_t(16)}, {"name", "ClockSource"}});
        // the rate the tag carries is unequal to the generator's: the generator forwards it as written, by the same
        // path the gr:frequency key takes
        clockSrc.tags = {Tag(32UZ, {{gr::tag::FREQUENCY.key(), kStreamFrequency}, {gr::tag::SAMPLE_RATE.key(), 250.f}})};

        auto& sink = testGraph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "clk_in">(clockSrc, generator).has_value());
        expect(testGraph.connect<"out", "in">(generator, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        check(generator, sink);
    };

    const auto expectStreamFrequencyIntact = [](const auto& sink) {
        std::size_t nSeen = 0UZ;
        for (const auto& tag : sink._tags) {
            const auto it = tag.map.find(gr::tag::FREQUENCY.key());
            if (it == tag.map.end()) {
                continue;
            }
            ++nSeen;
            const double* value = it->second.template get_if<double>();
            expect(value != nullptr) << std::format("the forwarded frequency key lost its type: {}", tag.map);
            if (value != nullptr) {
                expect(eq(*value, kStreamFrequency)) << "the generator rewrote the stream frequency it forwarded";
            }
            // the 250 Hz rate rode the same tag and leaves with it, although the generator runs at 10 kHz
            const auto rateIt = tag.map.find(gr::tag::SAMPLE_RATE.key());
            expect(rateIt != tag.map.end()) << "the sample_rate key rode the same tag and must still be there";
            if (rateIt != tag.map.end()) {
                expect(eq(rateIt->second.value_or(0.f), 250.f)) << "the generator forwards the rate key as written";
            }
        }
        expect(eq(nSeen, 1UZ)) << "the stream frequency tag did not cross the generator";
    };

    "SignalGenerator forwards the stream frequency key unchanged"_test = [&] {
        Graph testGraph;
        auto& gen = testGraph.emplaceBlock<SignalGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), 10000.f}, {"signal_type", "sin"}, {"tone_frequency", 250.f}, {"name", "SignalGenerator"}});
        runClockedGenerator(testGraph, gen, [&](const auto&, const auto& sink) { expectStreamFrequencyIntact(sink); });
    };

    "SignalGenerator keeps its tone across a stream frequency tag"_test = [&] {
        Graph testGraph;
        // the tone is left at its default, so it is still in the settings auto-update set
        auto& gen = testGraph.emplaceBlock<SignalGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), 10000.f}, {"signal_type", "sin"}, {"name", "SignalGenerator"}});
        runClockedGenerator(testGraph, gen, [](const auto& generator, const auto&) { expect(eq(generator.settings.tone_frequency.value, 1.f)) << "a passing frequency tag retuned the generated waveform"; });
    };

    "FunctionGenerator forwards the stream frequency key unchanged"_test = [&] {
        Graph testGraph;
        auto& gen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), 10000.f}, {"signal_type", "sin"}, {"tone_frequency", 250.f}, {"name", "FunctionGenerator"}});
        runClockedGenerator(testGraph, gen, [&](const auto&, const auto& sink) { expectStreamFrequencyIntact(sink); });
    };

    "FunctionGenerator keeps its tone across a stream frequency tag"_test = [&] {
        Graph testGraph;
        auto& gen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), 10000.f}, {"signal_type", "sin"}, {"name", "FunctionGenerator"}});
        runClockedGenerator(testGraph, gen, [](const auto& generator, const auto&) { expect(eq(generator.settings.tone_frequency.value, 0.f)) << "a passing frequency tag retuned the generated waveform"; });
    };

    "FunctionGenerator + ClockSource repeated ramp segment test"_test = [] {
        using namespace std::string_literals;
        using namespace function_generator;
        constexpr std::uint32_t N           = 400;
        constexpr float         sample_rate = 1000.f;
        Graph                   testGraph;
        auto&                   clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});

        clockSrc.tags = {Tag(0, {{tag::CONTEXT.key(), "1"s}}), Tag(200, {{tag::CONTEXT.key(), "2"s}})};

        // the two segments hold the same ramp, so only their selection tells the second from the first
        auto& funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}});
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", createLinearRampPropertyMap(20.f, 30.f, .1f)}, {"2", createLinearRampPropertyMap(20.f, 30.f, .1f)}}}}).empty());
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}}).has_value());

        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(fatal(eq(N, static_cast<std::uint32_t>(sink._samples.size())))) << "number of samples does not match";

        // segment 1: a 100-sample ramp from 20 to 30, held at 30 until the next activation
        expect(approx(25.f, sink._samples[50], .6f)) << std::format("first ramp mid-point: {}", sink._samples[50]);
        expect(approx(30.f, sink._samples[150], .5f)) << std::format("first ramp holds its final value: {}", sink._samples[150]);

        // segment 2: the same ramp again, so it starts from its start value rather than holding 30
        const float restart = *std::min_element(sink._samples.begin() + 200, sink._samples.begin() + 205);
        expect(approx(20.f, restart, .5f)) << std::format("second ramp restarts at its start value: {}", restart);
        expect(approx(25.f, sink._samples[250], .6f)) << std::format("second ramp mid-point: {}", sink._samples[250]);
        expect(approx(30.f, sink._samples[350], .5f)) << std::format("second ramp reaches its final value: {}", sink._samples[350]);
    };

    "FunctionGenerator + ClockSource repeated tone segment test"_test = [] {
        using namespace std::string_literals;
        using namespace function_generator;
        constexpr std::uint32_t N           = 400;
        constexpr float         sample_rate = 1000.f;
        Graph                   testGraph;
        auto&                   clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});

        // 50 Hz at 1 kHz is 20 samples per cycle: a phase carried into the second segment sits at a zero crossing
        // at sample 205, where a phase of the segment's own is at the cosine's peak
        clockSrc.tags = {Tag(0, {{tag::CONTEXT.key(), "1"s}}), Tag(205, {{tag::CONTEXT.key(), "2"s}})};

        auto& funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}});
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segments", property_map{{"1", createCosPropertyMap(50.f, 1.f, 0.f, 0.f, 0.f)}, {"2", createCosPropertyMap(50.f, 1.f, 0.f, 0.f, 0.f)}}}}).empty());
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}}).has_value());

        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(fatal(eq(N, static_cast<std::uint32_t>(sink._samples.size())))) << "number of samples does not match";

        const float firstPeak = *std::max_element(sink._samples.begin(), sink._samples.begin() + 4);
        expect(firstPeak > .9f) << std::format("first tone segment starts at the cosine's peak: {}", firstPeak);

        const float secondPeak = *std::max_element(sink._samples.begin() + 205, sink._samples.begin() + 209);
        expect(secondPeak > .9f) << std::format("second tone segment restarts its phase: {}", secondPeak);
    };

    "FunctionGenerator: a key the call names wins over the selected segment's value"_test = [] {
        using namespace function_generator;
        FunctionGenerator<double> funcGen({{"segments", property_map{{"low", createConstPropertyMap(5.f)}, {"high", createConstPropertyMap(9.f)}}}});
        expect(funcGen.check({}).has_value());

        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segment", "low"}}).empty());
        expect(eq(funcGen._signal.start_value, 5.f)) << "the selection alone puts the segment's value in force";
        expect(eq(funcGen.get<float>("start_value").value_or(-1.f), 0.f)) << "the host's start_value keeps the value the host wrote";

        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segment", "high"}, {"start_value", 7.f}}).empty());
        expect(eq(funcGen._signal.start_value, 7.f)) << "the call's own start_value wins over the segment's 9";
        expect(eq(funcGen._signal.signal_type, SignalType::constant)) << "the segment's other settings still go in force";
        expect(eq(funcGen.generateSample(), 7.)) << "the waveform runs on the value in force";

        const auto selected = funcGen.get("segment");
        expect(selected.has_value() && selected->value == pmt::Value(std::string("high"))) << "a read of the selector returns the name in force";
        const auto start = funcGen.get("start_value");
        expect(start.has_value() && start->value == pmt::Value(7.f)) << "a read of a signal setting returns the value the host wrote";
        const auto current = funcGen.get("current_segment");
        expect(fatal(current.has_value()));
        const property_map* report = current->value.get_if<property_map>();
        expect(fatal(report != nullptr)) << "current_segment is a map";
        expect(report->at(std::pmr::string("segment")) == pmt::Value(std::string("high"))) << "current_segment names the segment in force";
        expect(report->at(std::pmr::string("start_value")) == pmt::Value(7.f)) << "current_segment holds the value the waveform runs";
    };

    "FunctionGenerator: a caller's unknown segment name is refused and the segment in force stays"_test = [] {
        using namespace function_generator;
        FunctionGenerator<double> funcGen({{"segments", property_map{{"low", createConstPropertyMap(5.f)}}}, {"segment", "low"}});
        expect(funcGen.check({}).has_value());
        expect(eq(funcGen._signal.start_value, 5.f)) << "the segment given at construction goes in force at the initialization";

        const auto refused = funcGen.set({{"segment", "none"}});
        expect(!refused.has_value()) << "a name with no segment is refused";
        if (!refused.has_value()) {
            expect(fatal(eq(refused.error().size(), 1UZ)));
            expect(eq(refused.error()[0].key, std::string("segment")));
        }
        expect(eq(funcGen.settings.segment.value, std::string("low")));
        expect(eq(funcGen._signal.start_value, 5.f));
    };

    "FunctionGenerator: a segment that names no setting it may hold is refused"_test = [] {
        using namespace function_generator;
        FunctionGenerator<double> funcGen;
        expect(funcGen.check({}).has_value());

        const auto refusedKey = [&funcGen](property_map segment) -> std::string {
            const auto result = funcGen.set({{"segments", property_map{{"bad", std::move(segment)}}}});
            return result.has_value() || result.error().empty() ? std::string{} : result.error()[0].key;
        };
        expect(eq(refusedKey({{"amplitude", 1.f}}), std::string("segments"))) << "a key that is no setting";
        expect(eq(refusedKey({{"sample_rate", 1.f}}), std::string("segments"))) << "a setting a segment does not hold";
        expect(eq(refusedKey({{"segment", "bad"}}), std::string("segments"))) << "the selector";
        expect(eq(refusedKey({{"segments", property_map{}}}), std::string("segments"))) << "the table itself";
        expect(eq(refusedKey({{"signal_type", "square"}}), std::string("segments"))) << "a value its setting cannot take";
        expect(eq(funcGen.settings.segments.value.size(), 0UZ)) << "a refused table leaves the table in force";

        expect(gr::testing::initRefused<FunctionGenerator<double>>({{"segments", property_map{{"bad", property_map{{"amplitude", 1.f}}}}}})) << "a constructed table is checked at the initialization";
    };

    "FunctionGenerator: a segment's signal_type is read as the setting reads it, exactly as spelled"_test = [] {
        using namespace function_generator;
        FunctionGenerator<double> funcGen({{"segments", property_map{{"tone", property_map{{"signal_type", "sin"}}}, {"ramp", property_map{{"signal_type", "linear_ramp"}}}}}, {"segment", "tone"}});
        expect(fatal(funcGen.check({}).has_value())) << "the table with \"sin\" and \"linear_ramp\" is accepted";
        expect(funcGen._signal.signal_type == SignalType::sin) << "\"sin\" runs sin";
        expect(funcGen.set({{"segment", "ramp"}}).has_value());
        expect(funcGen._signal.signal_type == SignalType::linear_ramp) << "\"linear_ramp\" runs linear_ramp";
        expect(funcGen.set({{"signal_type", "cos"}}).has_value()) << "the setting itself takes the same spelling";
        expect(!funcGen.set({{"segments", property_map{{"ramp", property_map{{"signal_type", "LinearRamp"}}}}}}).has_value()) << "a segment's other spelling is refused, as the setting refuses it";
        expect(!funcGen.set({{"signal_type", "Cos"}}).has_value()) << "and the setting refuses it too";
    };

    "FunctionGenerator: a tag that names no segment leaves the segment in force"_test = [] {
        using namespace std::string_literals;
        using namespace function_generator;
        constexpr std::uint32_t N           = 300;
        constexpr float         sample_rate = 1000.f;
        Graph                   testGraph;
        auto&                   clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});
        clockSrc.tags                    = {Tag(0, {{tag::CONTEXT.key(), "one"s}}), Tag(100, {{tag::CONTEXT.key(), "unknown"s}}), Tag(200, {{tag::CONTEXT.key(), "two"s}})};

        auto& funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}, //
            {"segments", property_map{{"one", createConstPropertyMap(1.f)}, {"two", createConstPropertyMap(2.f)}}}});
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}}).has_value());
        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value()) << "an unknown name in the stream does not end the run";
        expect(fatal(eq(sink._samples.size(), static_cast<std::size_t>(N))));
        expect(eq(sink._samples[150UZ], 1.f)) << "the segment in force stays past the unknown name";
        expect(eq(sink._samples[250UZ], 2.f)) << "a later known name still switches";
        expect(eq(sink._tags.size(), clockSrc.tags.size())) << "the tag with the unknown name passes on downstream";
    };

    "FunctionGenerator: trigger restarts the segment in force at each set and reads false"_test = [] {
        using namespace function_generator;
        // a ramp from 0 to 10 over 1 s at 10 Hz rises by one per sample, so a sample is ten times its time since the restart
        FunctionGenerator<double> funcGen({{gr::tag::SAMPLE_RATE.name(), 10.f}, {"segments", property_map{{"ramp", createLinearRampPropertyMap(0.f, 10.f, 1.f)}}}, {"segment", "ramp"}});
        expect(funcGen.check({}).has_value());
        const auto advance = [&funcGen](std::size_t count) {
            double last = -1.;
            for (std::size_t i = 0UZ; i < count; ++i) {
                last = funcGen.generateSample();
            }
            return last;
        };
        const auto latched = [&funcGen] { return funcGen.get<bool>("trigger").value_or(true); };

        expect(approx(advance(4UZ), 3., 1e-6)) << "the ramp runs from the selection";
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"trigger", true}}).empty());
        expect(approx(funcGen.generateSample(), 0., 1e-6)) << "a true restarts the segment in force";
        expect(!latched()) << "a read after the set returns false";
        expect(!funcGen.settings.trigger.value) << "the hook assigned false";

        expect(approx(advance(3UZ), 3., 1e-6));
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"trigger", true}}).empty());
        expect(approx(funcGen.generateSample(), 0., 1e-6)) << "a second set of true restarts again";
        expect(!latched());

        expect(approx(advance(3UZ), 3., 1e-6));
        expect(funcGen.set({{"trigger", true}}).has_value()) << "a host's set";
        expect(approx(funcGen.generateSample(), 0., 1e-6)) << "a host's set of true restarts too";
        expect(!latched());

        expect(approx(advance(3UZ), 3., 1e-6));
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(funcGen), {{"segment", "ramp"}}).empty());
        expect(approx(funcGen.generateSample(), 4., 1e-6)) << "naming the segment in force again restarts nothing";
    };

    "FunctionGenerator: a tag coupled to trigger restarts the segment in force"_test = [] {
        using namespace std::string_literals;
        using namespace function_generator;
        constexpr std::uint32_t N           = 300;
        constexpr float         sample_rate = 1000.f;
        Graph                   testGraph;
        auto&                   clockSrc = testGraph.emplaceBlock<ClockSource<std::uint8_t>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"n_samples_max", N}, {"name", "ClockSource"}});
        // the ramp reaches 10 at sample 100 and holds it; each `restart` tag starts it again from 0
        clockSrc.tags = {Tag(0, {{tag::CONTEXT.key(), "ramp"s}}), Tag(150, {{"restart", true}}), Tag(250, {{"restart", true}})};

        auto& funcGen = testGraph.emplaceBlock<FunctionGenerator<float>>({{gr::tag::SAMPLE_RATE.name(), sample_rate}, {"name", "FunctionGenerator"}, //
            {"segments", property_map{{"ramp", createLinearRampPropertyMap(0.f, 10.f, .1f)}}}});
        expect(funcGen.setTagCouplings({{"segment", std::string(tag::CONTEXT.key())}, {"trigger", std::string("restart")}}).has_value());
        auto& sink = testGraph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(testGraph.connect<"out", "clk_in">(clockSrc, funcGen).has_value());
        expect(testGraph.connect<"out", "in">(funcGen, sink).has_value());

        gr::scheduler::Simple sched;
        if (auto ret = sched.exchange(std::move(testGraph)); !ret) {
            throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
        }
        expect(sched.runAndWait().has_value());
        expect(fatal(eq(sink._samples.size(), static_cast<std::size_t>(N))));

        expect(approx(10.f, sink._samples[140UZ], .1f)) << std::format("the ramp holds its final value: {}", sink._samples[140UZ]);
        const float first = *std::min_element(sink._samples.begin() + 150, sink._samples.begin() + 155);
        expect(approx(0.f, first, .5f)) << std::format("the first tag restarts the ramp: {}", first);
        expect(approx(5.f, sink._samples[200UZ], .6f)) << std::format("the restarted ramp's mid-point: {}", sink._samples[200UZ]);
        const float second = *std::min_element(sink._samples.begin() + 250, sink._samples.begin() + 255);
        expect(approx(0.f, second, .5f)) << std::format("the second tag, true again, restarts it again: {}", second);
        expect(!funcGen.get<bool>("trigger").value_or(true)) << "trigger reads false after the last tag";
    };

    "FunctionGenerator: a saved graph loads back with its segments and the selector's coupling"_test = [] {
        gr::BlockRegistry& registry = gr::globalBlockRegistry();
        std::ignore                 = registry.insert<FunctionGenerator<float>>("=gr::blocks::basic::FunctionGenerator<float32>");
        gr::PluginLoader& loader    = gr::globalPluginLoader();

        // the form tools/migrate_generator_settings.py writes for a graph that held one context per segment
        const std::string document = R"yaml(blocks:
  - id: gr::blocks::basic::FunctionGenerator<float32>
    parameters:
      name: ramp
      sample_rate: 1000
      segments:
        one:
          signal_type: constant
          start_value: 5
        two:
          signal_type: linear_ramp
          start_value: 5
          final_value: 30
          duration: 0.2
    tag_couplings:
      segment: gr:context
)yaml";

        const auto findGenerator = [](const gr::Graph& graph) -> gr::BlockModel* {
            for (const auto& block : graph.blocks()) {
                if (block->name() == "ramp") {
                    return block.get();
                }
            }
            return nullptr;
        };
        const auto expectSegments = [&findGenerator](const gr::Graph& graph, std::string_view what) {
            gr::BlockModel* generator = findGenerator(graph);
            expect(fatal(generator != nullptr)) << what;
            const auto segments = generator->get("segments");
            expect(fatal(segments.has_value())) << what;
            const property_map* table = segments->value.get_if<property_map>();
            expect(fatal(table != nullptr)) << what;
            expect(eq(table->size(), 2UZ)) << what;
            const auto two = table->find(std::string_view("two"));
            expect(fatal(two != table->end())) << what;
            const property_map* ramp = two->second.get_if<property_map>();
            expect(fatal(ramp != nullptr)) << what;
            expect(eq(ramp->size(), 4UZ)) << what;
            expect(eq(generator->tagCouplings().size(), 1UZ)) << std::format("{}: the selector keeps its coupling", what);
        };

        // the final_value the waveform runs, as the generator reports it
        const auto currentFinalValue = [](gr::BlockModel& generator) {
            const auto current = generator.get("current_segment");
            if (!current.has_value()) {
                return -1.f;
            }
            const property_map* report = current->value.get_if<property_map>();
            return report != nullptr && report->contains(std::pmr::string("final_value")) ? report->at(std::pmr::string("final_value")).value_or(-1.f) : -1.f;
        };

        auto loaded = gr::loadGrc(loader, document);
        expectSegments(*loaded, "the graph file as written");
        gr::BlockModel* generator = findGenerator(*loaded);
        expect(fatal(generator != nullptr));
        // a stopped graph's block holds the change until its start; the test applies it as the start would
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(*generator), {{"segment", "two"}}).empty()) << "a host selects a segment of the loaded table";
        expect(eq(currentFinalValue(*generator), 30.f)) << "the selected segment is in force";
        const std::string saved    = gr::saveGrc(loader, *loaded);
        auto              reloaded = gr::loadGrc(loader, saved);
        expectSegments(*reloaded, std::format("the graph saved and loaded again:\n{}", saved));
        gr::BlockModel* reloadedGenerator = findGenerator(*reloaded);
        expect(fatal(reloadedGenerator != nullptr));
        expect(eq(currentFinalValue(*reloadedGenerator), 30.f)) << std::format("the saved selection puts its segment in force at the load:\n{}", saved);
    };
};

int main() { /* not needed for UT */ }
