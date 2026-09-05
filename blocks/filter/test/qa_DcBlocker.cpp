#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/filter/DcBlocker.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::filter::DcBlocker;

using CF = std::complex<float>;

constexpr gr::Size_t kLength = 32U;
constexpr gr::Size_t kReseed = 4096U;

template<typename T>
[[nodiscard]] DcBlocker<T> makeBlock(gr::property_map settings) {
    DcBlocker<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
[[nodiscard]] std::vector<T> run(DcBlocker<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    std::vector<T>    output(input.size());
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<T>(output).subspan(base, count));
    }
    return output;
}

/// @brief |H(f)| measured, by running a complex exponential through and reading the settled magnitude.
[[nodiscard]] double response(bool longForm, double normalizedFrequency) {
    DcBlocker<CF>   block = makeBlock<CF>({{"length", kLength}, {"long_form", longForm}});
    std::vector<CF> input(8192UZ);
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        const double phase = 2.0 * std::numbers::pi * normalizedFrequency * static_cast<double>(i);
        input[i]           = CF{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    const std::vector<CF> output = run<CF>(block, std::span<const CF>(input));
    return static_cast<double>(std::abs(output.back()));
}

/// @brief The frequency at which the measured response crosses -3 dB, by bisection.
[[nodiscard]] double corner(bool longForm) {
    double low = 1e-4, high = 0.04;
    for (int step = 0; step < 24; ++step) {
        const double middle = 0.5 * (low + high);
        if (response(longForm, middle) < std::numbers::sqrt2 / 2.0) {
            low = middle;
        } else {
            high = middle;
        }
    }
    return 0.5 * (low + high);
}

[[nodiscard]] std::vector<float> pseudoRandom(std::size_t nSamples) {
    std::vector<float> input(nSamples);
    std::uint64_t      state = 0xB7E151628AED2A6BULL;
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        state    = state * 6364136223846793005ULL + 1442695040888963407ULL;
        input[i] = static_cast<float>(static_cast<double>(state >> 11U) / static_cast<double>(1ULL << 53U) - 0.5);
    }
    return input;
}

/// @brief The same pipeline in double, as the drift reference.
[[nodiscard]] std::vector<double> referenceBlocker(std::span<const float> input, std::size_t length, std::size_t stages) {
    const std::size_t   delay = stages / 2UZ * (length - 1UZ);
    std::vector<double> chain(input.begin(), input.end());
    for (std::size_t s = 0UZ; s < stages; ++s) {
        std::vector<double> next(chain.size());
        for (std::size_t n = 0UZ; n < chain.size(); ++n) {
            const std::size_t first = n + 1UZ >= length ? n + 1UZ - length : 0UZ;
            double            sum   = 0.0;
            for (std::size_t at = first; at <= n; ++at) {
                sum += chain[at];
            }
            next[n] = sum / static_cast<double>(length);
        }
        chain = std::move(next);
    }
    std::vector<double> output(input.size());
    for (std::size_t n = 0UZ; n < input.size(); ++n) {
        output[n] = (n >= delay ? static_cast<double>(input[n - delay]) : 0.0) - chain[n];
    }
    return output;
}

/// @brief The offsets at which a key outside `gr::tag::kDefaultTags` reaches the sink through @p TBlock.
template<typename TIn, typename TOut, typename TBlock>
[[nodiscard]] std::vector<std::size_t> privateTagOffsets(gr::property_map settings) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    const gr::property_map::key_type key{"private_key"};
    const gr::pmt::Value             value{std::string("carried")};

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<TIn, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t(2048)}, {"mark_tag", false}});
    for (const std::size_t at : {7UZ, 300UZ, 1000UZ}) {
        source._tags.emplace_back(at, gr::property_map{{key, value}});
    }
    auto& block = graph.emplaceBlock<TBlock>(std::move(settings));
    auto& sink  = graph.emplaceBlock<TagSink<TOut, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : sink._tags) {
        if (const auto found = tag.map.find(key); found != tag.map.end() && found->second == value) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

} // namespace

const boost::ut::suite<"DcBlocker"> dcBlockerTests = [] {
    using namespace boost::ut;

    "a constant is removed exactly"_test = [] {
        for (const bool longForm : {true, false}) {
            // the pipeline fills in stages*(D-1) samples, which is twice the group delay: the cascade's own impulse
            // response is stages*(D-1)+1 long and the group delay is its center, which is where the step transient is
            // exactly half way rather than where it ends
            const std::size_t        transient = (longForm ? 4UZ : 2UZ) * (kLength - 1UZ);
            DcBlocker<float>         block     = makeBlock<float>({{"length", kLength}, {"long_form", longForm}});
            const std::vector<float> input(2000UZ, 1.f);
            const std::vector<float> output = run<float>(block, std::span<const float>(input));
            for (std::size_t n = transient; n < output.size(); ++n) {
                expect(lt(std::abs(static_cast<double>(output[n])), 1e-6)) << (longForm ? "long" : "short") << " form at " << n;
            }

            DcBlocker<CF>         complexBlock = makeBlock<CF>({{"length", kLength}, {"long_form", longForm}});
            const std::vector<CF> complexInput(2000UZ, CF{1.f, 1.f});
            const std::vector<CF> complexOutput = run<CF>(complexBlock, std::span<const CF>(complexInput));
            for (std::size_t n = transient; n < complexOutput.size(); ++n) {
                expect(lt(static_cast<double>(std::abs(complexOutput[n])), 1e-6)) << "complex " << (longForm ? "long" : "short") << " at " << n;
            }
        }
    };

    "group_delay is stated and is where the step is half way"_test = [] {
        for (const bool longForm : {true, false}) {
            const std::size_t stages = longForm ? 4UZ : 2UZ;
            const std::size_t delay  = stages / 2UZ * (kLength - 1UZ);

            DcBlocker<float> block = makeBlock<float>({{"length", kLength}, {"long_form", longForm}});
            expect(eq(block.group_delay.value, static_cast<gr::Size_t>(delay)));

            const std::vector<float> input(2000UZ, 1.f);
            const std::vector<float> output = run<float>(block, std::span<const float>(input));
            expect(lt(std::abs(std::abs(static_cast<double>(output[delay])) - 0.5), 0.05)) << (longForm ? "long" : "short") << " form is half way through its transient at the group delay";
            expect(gt(std::abs(static_cast<double>(output[delay - 1UZ])), 0.1)) << "which is well inside it, not at its end";
        }
    };

    "the passband follows |1 - A(f)^stages|"_test = [] {
        struct Row {
            double frequency, longForm, shortForm;
        };
        constexpr Row kRows[] = {{0.005, 0.156098, 0.081358}, {0.010, 0.502113, 0.294389}, {0.015625, 0.835480, 0.594390}, {0.200, 0.999993, 0.997443}};

        for (const Row& row : kRows) {
            expect(lt(std::abs(response(true, row.frequency) - row.longForm) / row.longForm, 0.01)) << "long form at f/fs = " << row.frequency;
            expect(lt(std::abs(response(false, row.frequency) - row.shortForm) / row.shortForm, 0.01)) << "short form at f/fs = " << row.frequency;
        }
        expect(gt(response(true, 0.2), response(false, 0.2))) << "the long form is the flatter of the two, which is why it is the default";
    };

    "the long form has the narrower notch as well"_test = [] {
        const double longCorner  = corner(true);
        const double shortCorner = corner(false);
        expect(lt(std::abs(longCorner - 0.013088) / 0.013088, 0.05)) << "long form corner measured at " << longCorner;
        expect(lt(std::abs(shortCorner - 0.017908) / 0.017908, 0.05)) << "short form corner measured at " << shortCorner;
        expect(lt(longCorner, shortCorner)) << "the long form's corner is the lower of the two";

        expect(lt(std::abs(response(true, 1.0 / static_cast<double>(kLength)) - 1.0), 1e-3)) << "and the response is back at unity on the boxcar's own null";
    };

    "the output is bit-identical whatever the chunking"_test = [] {
        const std::vector<float> input = pseudoRandom(4UZ * kReseed);
        const gr::property_map   settings{{"length", kLength}, {"long_form", true}, {"reseed_interval", kReseed}};

        DcBlocker<float>         reference = makeBlock<float>(settings);
        const std::vector<float> want      = run<float>(reference, std::span<const float>(input));

        for (const std::size_t chunkSize : {1UZ, 7UZ, 32UZ, 4095UZ, 4096UZ, 4097UZ}) {
            DcBlocker<float>         block = makeBlock<float>(settings);
            const std::vector<float> got   = run<float>(block, std::span<const float>(input), chunkSize);
            expect(std::ranges::equal(got, want)) << "chunk size " << chunkSize;
        }
    };

    "the reseed keeps the drift bounded"_test = [] {
        const std::vector<float>  input = pseudoRandom(1000000UZ);
        const std::vector<double> want  = referenceBlocker(std::span<const float>(input), kLength, 2UZ);

        const auto drift = [&input, &want](gr::Size_t interval) {
            DcBlocker<float>         block = makeBlock<float>({{"length", kLength}, {"long_form", false}, {"reseed_interval", interval}});
            const std::vector<float> got   = run<float>(block, std::span<const float>(input), 997UZ);
            double                   worst = 0.0;
            for (std::size_t n = 0UZ; n < got.size(); ++n) {
                worst = std::max(worst, std::abs(static_cast<double>(got[n]) - want[n]));
            }
            return worst;
        };

        const double reseeded  = drift(kReseed);
        const double unbounded = drift(1000000000U); // the same run with no reseed inside a million samples
        std::println("DcBlocker short-form drift over 1e6 samples: {:.3e} at R = {}, {:.3e} with no reseed", reseeded, kReseed, unbounded);
        expect(lt(reseeded, 7e-7)) << "the bound is a factor of two above the drift this pipeline shows";
        expect(gt(unbounded, 2.0 * reseeded)) << "and the reseed is what holds it there";
    };

    "a length below two is rejected at construction"_test = [] {
        expect(throws([] { std::ignore = makeBlock<float>({{"length", 1U}}); }));
        expect(throws([] { std::ignore = makeBlock<float>({{"length", 0U}}); }));
    };

    "nanoseconds per sample"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        const std::vector<float> x = pseudoRandom(1UZ << 16);
        std::vector<float>       y(x.size());

        struct Arm {
            const char*      label;
            gr::property_map settings;
        };
        const Arm     kArms[]  = {{"DcBlocker<float> long D=32", {{"length", kLength}, {"long_form", true}}}, {"DcBlocker<float> short D=32", {{"length", kLength}, {"long_form", false}}}};
        constexpr int kRepeats = 7;

        std::vector<DcBlocker<float>> blocks;
        for (const Arm& arm : kArms) {
            blocks.push_back(makeBlock<float>(arm.settings));
        }

        std::vector<double> best(std::size(kArms), 1e30);
        std::vector<double> worst(std::size(kArms), 0.0);
        for (int repeat = 0; repeat < kRepeats; ++repeat) { // arms interleaved, so a thermal drift moves all of them
            for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
                const auto start = Clock::now();
                std::ignore      = blocks[a].processBulk(std::span<const float>(x), std::span<float>(y));
                const double ns  = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
                best[a]          = std::min(best[a], ns);
                worst[a]         = std::max(worst[a], ns);
            }
        }
        for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
            std::println("{}: best {:.3f} ns/sample, spread {:.3f} ns", kArms[a].label, best[a], worst[a] - best[a]);
        }
    };

    "a non-reserved tag key rides through at its own offset"_test = [] {
        const std::vector<std::size_t> offsets = privateTagOffsets<float, float, DcBlocker<float>>({{"length", kLength}});
        expect(that % (offsets == std::vector<std::size_t>{7UZ, 300UZ, 1000UZ})) << "the group delay moves the samples and not the annotation";
    };

    "length and long_form are refused while the block runs; reseed_interval is not"_test = [] {
        // A refused settings change stays staged and re-applies, so a block that has refused once goes on refusing:
        // each case gets its own block, the discipline qa_LinearEqualizer already records for the same reason.
        const auto running = [](gr::property_map settings) {
            auto block = std::make_unique<DcBlocker<float>>(std::move(settings));
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };
        const auto live = [](DcBlocker<float>& block, gr::property_map changes) {
            std::ignore = block.settings().set(std::move(changes));
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        };

        {
            auto block = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            expect(throws([&] { live(*block, {{"length", gr::Size_t(64)}}); })) << "a running block refuses a new length";
        }
        {
            auto block = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            expect(throws([&] { live(*block, {{"long_form", false}}); })) << "and a new form";
        }
        {
            // reseed_interval touches no pipeline state and stays live, which is what the documentation claims
            auto             block  = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            const gr::Size_t before = block->group_delay;
            expect(nothrow([&] { live(*block, {{"reseed_interval", gr::Size_t(2048)}}); }));
            expect(eq(block->reseed_interval.value, gr::Size_t(2048)));
            expect(eq(block->group_delay.value, before)) << "and moves nothing the refusals protect";
        }
        {
            // stopped, both move again: the contract is construction-time, not immutable
            auto             block  = running({{"length", gr::Size_t(32)}, {"long_form", true}});
            const gr::Size_t before = block->group_delay;
            block->stop();
            expect(nothrow([&] { live(*block, {{"length", gr::Size_t(64)}}); }));
            expect(eq(block->length.value, gr::Size_t(64)));
            expect(block->group_delay.value != before) << "and the rebuilt pipeline states its new delay";
        }
    };
};

int main() { /* tests are automatically registered and run */ }
