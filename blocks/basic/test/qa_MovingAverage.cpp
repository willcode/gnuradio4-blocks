#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/basic/MovingAverage.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>

namespace {

using gr::blocks::basic::MovingAverage;

constexpr gr::Size_t kReseed = 4096U;

template<typename T>
[[nodiscard]] MovingAverage<T> makeBlock(gr::property_map settings) {
    MovingAverage<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
[[nodiscard]] std::vector<T> run(MovingAverage<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    std::vector<T>    output(input.size());
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        const std::size_t count = std::min(stride, input.size() - base);
        std::ignore             = block.processBulk(input.subspan(base, count), std::span<T>(output).subspan(base, count));
    }
    return output;
}

/// @brief Deterministic N(0,1)-ish data, so the drift bound is the same number on every machine.
[[nodiscard]] std::vector<float> pseudoRandom(std::size_t nSamples) {
    std::vector<float> input(nSamples);
    std::uint64_t      state = 0x243F6A8885A308D3ULL;
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        double sum = 0.0;
        for (int k = 0; k < 4; ++k) { // four uniforms summed: close enough to normal for a rounding measurement
            state = state * 6364136223846793005ULL + 1442695040888963407ULL;
            sum += static_cast<double>(state >> 11U) / static_cast<double>(1ULL << 53U) - 0.5;
        }
        input[i] = static_cast<float>(sum);
    }
    return input;
}

[[nodiscard]] std::vector<double> referenceAverage(std::span<const float> input, std::size_t window) {
    std::vector<double> output(input.size());
    for (std::size_t n = 0UZ; n < input.size(); ++n) {
        const std::size_t first = n + 1UZ >= window ? n + 1UZ - window : 0UZ;
        double            sum   = 0.0;
        for (std::size_t at = first; at <= n; ++at) { // ascending, oldest first, in double
            sum += static_cast<double>(input[at]);
        }
        output[n] = sum;
    }
    return output;
}

} // namespace

const boost::ut::suite<"MovingAverage"> movingAverageTests = [] {
    using namespace boost::ut;

    "the output is bit-identical whatever the chunking"_test = [] {
        const std::vector<float> input = pseudoRandom(4UZ * kReseed);
        const gr::property_map   settings{{"length", 64U}, {"scale", 1.f / 64.f}, {"reseed_interval", kReseed}};

        MovingAverage<float>     reference = makeBlock<float>(settings);
        const std::vector<float> want      = run<float>(reference, std::span<const float>(input));

        for (const std::size_t chunkSize : {1UZ, 3UZ, 7UZ, 63UZ, 64UZ, 65UZ, 4095UZ, 4096UZ, 4097UZ}) {
            MovingAverage<float>     block = makeBlock<float>(settings);
            const std::vector<float> got   = run<float>(block, std::span<const float>(input), chunkSize);
            expect(std::ranges::equal(got, want)) << "chunk size " << chunkSize << " must be bit-identical, not merely close";
        }
    };

    "the startup window is partial and defined"_test = [] {
        MovingAverage<float>     block = makeBlock<float>({{"length", 4U}, {"scale", 1.f}});
        const std::vector<float> input{1.f, 2.f, 3.f, 4.f, 5.f};
        expect(that % (run<float>(block, std::span<const float>(input)) == std::vector<float>{1.f, 3.f, 6.f, 10.f, 14.f}));
    };

    "a constant comes out constant once the window has filled"_test = [] {
        constexpr std::size_t    kWindow = 64UZ;
        MovingAverage<float>     block   = makeBlock<float>({{"length", static_cast<gr::Size_t>(kWindow)}, {"scale", 1.f / static_cast<float>(kWindow)}});
        const std::vector<float> input(1000UZ, 0.375f);
        const std::vector<float> output = run<float>(block, std::span<const float>(input));

        for (std::size_t n = kWindow - 1UZ; n < output.size(); ++n) {
            expect(lt(std::abs(static_cast<double>(output[n]) - 0.375), 64.0 * static_cast<double>(std::numeric_limits<float>::epsilon()))) << "at " << n;
        }
    };

    "an impulse produces exactly length outputs"_test = [] {
        constexpr gr::Size_t kWindow = 8U;
        MovingAverage<float> block   = makeBlock<float>({{"length", kWindow}, {"scale", 0.25f}});
        std::vector<float>   input(40UZ, 0.f);
        input[3UZ] = 1.f;

        const std::vector<float> output = run<float>(block, std::span<const float>(input));
        for (std::size_t n = 0UZ; n < output.size(); ++n) {
            const bool inWindow = n >= 3UZ && n < 3UZ + kWindow;
            expect(eq(output[n], inWindow ? 0.25f : 0.f)) << "at " << n;
        }
    };

    "an integer window accumulates wider than it samples"_test = [] {
        constexpr gr::Size_t            kWindow = 100000U;
        MovingAverage<std::int16_t>     block   = makeBlock<std::int16_t>({{"length", kWindow}, {"scale", static_cast<std::int16_t>(1)}});
        const std::vector<std::int16_t> input(static_cast<std::size_t>(kWindow), std::numeric_limits<std::int16_t>::max());
        std::ignore = run<std::int16_t>(block, std::span<const std::int16_t>(input), 4096UZ);

        const std::int64_t expected = static_cast<std::int64_t>(kWindow) * static_cast<std::int64_t>(std::numeric_limits<std::int16_t>::max());
        expect(eq(block.windowSum(0UZ), expected)) << "3.3e9 needs more than the sample type has";
    };

    "each vector element gets its own average"_test = [] {
        constexpr std::size_t    kWidth = 3UZ;
        const std::vector<float> flat   = pseudoRandom(300UZ * kWidth);

        MovingAverage<float>     vectored = makeBlock<float>({{"length", 16U}, {"scale", 1.f / 16.f}, {"vlen", static_cast<gr::Size_t>(kWidth)}});
        const std::vector<float> woven    = run<float>(vectored, std::span<const float>(flat));

        for (std::size_t e = 0UZ; e < kWidth; ++e) {
            std::vector<float> lane(flat.size() / kWidth);
            for (std::size_t v = 0UZ; v < lane.size(); ++v) {
                lane[v] = flat[v * kWidth + e];
            }
            MovingAverage<float>     scalar = makeBlock<float>({{"length", 16U}, {"scale", 1.f / 16.f}});
            const std::vector<float> want   = run<float>(scalar, std::span<const float>(lane));
            for (std::size_t v = 0UZ; v < lane.size(); ++v) {
                expect(eq(woven[v * kWidth + e], want[v])) << "element " << e << " at " << v;
            }
        }
    };

    "a live length change takes effect at a defined sample and reseeds there"_test = [] {
        constexpr std::size_t    kSwitch = 1000UZ;
        const std::vector<float> input   = pseudoRandom(3000UZ);

        const auto drive = [&input](std::size_t chunkSize) {
            MovingAverage<float> block = makeBlock<float>({{"length", 8U}, {"scale", 1.f}});
            std::vector<float>   first = run<float>(block, std::span<const float>(input).first(kSwitch), chunkSize);
            std::ignore                = block.settings().setStaged({{"length", 4U}});
            std::ignore                = block.settings().applyStagedParameters();
            std::vector<float> second  = run<float>(block, std::span<const float>(input).subspan(kSwitch), chunkSize);
            first.insert(first.end(), second.begin(), second.end());
            return first;
        };

        const std::vector<float> want = drive(0UZ);
        double                   sum  = 0.0;
        for (std::size_t k = 0UZ; k < 4UZ; ++k) { // real history, summed fresh at the sample the change lands on
            sum += static_cast<double>(input[kSwitch - 3UZ + k]);
        }
        expect(lt(std::abs(static_cast<double>(want[kSwitch]) - sum), 1e-5)) << "the first output after the change is a full new window";
        expect(gt(std::abs(static_cast<double>(want[kSwitch - 1UZ]) - sum), 1e-5)) << "and the sample before it is still the old one";

        for (const std::size_t chunkSize : {1UZ, 7UZ, 4096UZ}) {
            expect(std::ranges::equal(drive(chunkSize), want)) << "chunk size " << chunkSize << " across the change";
        }
    };

    "the reseed keeps the drift bounded"_test = [] {
        constexpr std::size_t    kWindow = 64UZ;
        const std::vector<float> input   = pseudoRandom(1000000UZ);

        const std::vector<double> want = referenceAverage(std::span<const float>(input), kWindow);

        const auto drift = [&input, &want](gr::Size_t interval) {
            MovingAverage<float>     block = makeBlock<float>({{"length", static_cast<gr::Size_t>(kWindow)}, {"scale", 1.f}, {"reseed_interval", interval}});
            const std::vector<float> got   = run<float>(block, std::span<const float>(input), 997UZ);
            double                   worst = 0.0;
            for (std::size_t n = 0UZ; n < got.size(); ++n) {
                worst = std::max(worst, std::abs(static_cast<double>(got[n]) - want[n]));
            }
            return worst;
        };

        const double reseeded  = drift(kReseed);
        const double unbounded = drift(1000000000U); // no reseed inside a million samples
        std::println("MovingAverage drift over 1e6 samples: {:.3e} at R = {}, {:.3e} with no reseed", reseeded, kReseed, unbounded);
        expect(lt(reseeded, 5.2e-5)) << "the bound is a factor of two above the drift this accumulator shows";
        expect(gt(unbounded, 2.0 * reseeded)) << "and the reseed is what holds it there";
    };

    "unusable parameters are rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeBlock<float>({{"length", 0U}}); })) << "zero length";
        expect(throws([] { std::ignore = makeBlock<float>({{"reseed_interval", 0U}}); })) << "zero reseed_interval";
        expect(throws([] { std::ignore = makeBlock<float>({{"vlen", 0U}}); })) << "zero vlen";
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
        const Arm     kArms[]  = {{"MovingAverage<float> N=16", {{"length", 16U}, {"scale", 1.f / 16.f}}}, {"MovingAverage<float> N=1024", {{"length", 1024U}, {"scale", 1.f / 1024.f}}}};
        constexpr int kRepeats = 7;

        std::vector<MovingAverage<float>> blocks;
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
};

int main() { /* tests are automatically registered and run */ }
