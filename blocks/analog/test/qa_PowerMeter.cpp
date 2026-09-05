#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <iterator>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/analog/PowerMeter.hpp>

#include "TestSpans.hpp"
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {
/// @brief Drives the meter over `input` to exhaustion and returns the records it published, which most cases ignore.
///
/// The meter leaves the last sample of a call unconsumed so the framework's end-of-stream epilogue has a span to run
/// on, and takes at most as many samples as the output span has room for the records they close. Calling until the
/// span is spent is therefore what a scheduler does, and what makes one call here mean one stretch of stream.
template<typename TBlock, typename T>
std::vector<gr::DataSet<float>> drive(TBlock& block, std::span<const T> input) {
    namespace test = gr::blocks::analog::test;
    std::vector<gr::DataSet<float>> records;

    for (std::size_t base = 0UZ; base < input.size();) {
        std::vector<gr::DataSet<float>>      made(4UZ);
        test::InputSpan<T>                   inSpan(input.subspan(base));
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        std::ignore = block.processBulk(inSpan, outSpan);
        records.insert(records.end(), std::make_move_iterator(made.begin()), std::make_move_iterator(made.begin() + static_cast<std::ptrdiff_t>(outSpan.count)));
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return records;
}

/// @brief What the meter emits when the stream ends here: the trailing samples folded in, then the record covering
/// what has accumulated since the last one.
template<typename TBlock, typename T>
std::vector<gr::DataSet<float>> finish(TBlock& block, std::span<const T> trailing) {
    namespace test = gr::blocks::analog::test;
    std::vector<gr::DataSet<float>>      made(4UZ);
    test::InputSpan<T>                   inSpan(trailing);
    test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
    std::ignore = block.processEpilogue(inSpan, outSpan);
    made.resize(outSpan.count);
    return made;
}
} // namespace

namespace {

using gr::blocks::analog::PowerMeter;
using gr::testing::ProcessFunction;
using gr::testing::TagSource;

using CF = std::complex<float>;

constexpr double kPi   = std::numbers::pi;
constexpr float  kRate = 96000.f;

/// The block holds atomics and is therefore not movable, so it is built in place and initialized by reference.
template<typename T>
void init(PowerMeter<T>& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
}

template<typename T>
void apply(PowerMeter<T>& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

template<typename T>
void feed(PowerMeter<T>& block, std::span<const T> input, std::size_t chunkSize = 0UZ) {
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    for (std::size_t base = 0UZ; base < input.size(); base += stride) {
        std::ignore = drive(block, input.subspan(base, std::min(stride, input.size() - base)));
    }
}

/// 0.01 normalized: a whole number of periods in a 600-sample segment and in a 9600-sample window alike.
[[nodiscard]] std::vector<CF> complexTone(std::size_t count, double amplitude, double normalized = 0.01) {
    std::vector<CF> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        const double phase = 2.0 * kPi * normalized * static_cast<double>(i);
        samples[i]         = CF(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return samples;
}

[[nodiscard]] std::vector<float> realTone(std::size_t count, double amplitude, double normalized = 0.01) {
    std::vector<float> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        samples[i] = static_cast<float>(amplitude * std::cos(2.0 * kPi * normalized * static_cast<double>(i)));
    }
    return samples;
}

struct Random {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] double uniform() noexcept {
        state += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state;
        z               = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z               = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z ^= z >> 31;
        return static_cast<double>(z >> 11) * 0x1.0p-53;
    }

    [[nodiscard]] double gaussian() noexcept { return std::sqrt(-2.0 * std::log(uniform() + 1e-300)) * std::cos(2.0 * kPi * uniform()); }

    /// Circular complex Gaussian of unit mean power: each component has variance one half.
    [[nodiscard]] CF sample() noexcept { return CF(static_cast<float>(gaussian() * 0.7071067811865476), static_cast<float>(gaussian() * 0.7071067811865476)); }
};

[[nodiscard]] std::vector<CF> noise(std::size_t count, std::uint64_t seed) {
    Random          rng{seed};
    std::vector<CF> samples(count);
    for (CF& sample : samples) {
        sample = rng.sample();
    }
    return samples;
}

/// @brief Mean and standard deviation of `level()` over @p trials independent windows of @p length noise samples.
[[nodiscard]] std::pair<double, double> levelStatistics(std::size_t length, std::size_t trials, std::uint64_t seed) {
    PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", static_cast<double>(length) / static_cast<double>(kRate)}, {"segments", 1U}});
    init(block);
    Random          rng{seed};
    std::vector<CF> window(length);

    double total  = 0.0;
    double square = 0.0;
    for (std::size_t trial = 0UZ; trial < trials; ++trial) {
        for (CF& sample : window) {
            sample = rng.sample();
        }
        block.reset();
        std::ignore        = drive(block, std::span<const CF>(window));
        const double level = static_cast<double>(block.level());
        total += level;
        square += level * level;
    }
    const double mean = total / static_cast<double>(trials);
    return {mean, std::sqrt(std::max(0.0, square / static_cast<double>(trials) - mean * mean))};
}

[[nodiscard]] bool longTestsEnabled() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

} // namespace

const boost::ut::suite<"PowerMeter"> powerMeterTests = [] {
    using namespace boost::ut;

    "the dBFS anchors are exact"_test = [] {
        struct Case {
            const char* what;
            double      amplitude;
            bool        isTone;
            double      want;
        };
        constexpr Case kCases[]{
            {"complex tone A=1.0", 1.0, true, 0.0},
            {"complex tone A=0.5", 0.5, true, -6.0206},
            {"complex tone A=0.1", 0.1, true, -20.0},
            {"complex DC A=1", 1.0, false, 0.0},
            {"complex DC A=0.25", 0.25, false, -12.0412},
        };
        for (const auto& [what, amplitude, isTone, want] : kCases) {
            PowerMeter<CF> block({{"sample_rate", kRate}});
            init(block);
            const std::vector<CF> x = isTone ? complexTone(9600UZ, amplitude) : std::vector<CF>(9600UZ, CF(static_cast<float>(amplitude), 0.f));
            feed<CF>(block, std::span<const CF>(x), 997UZ);
            expect(approx(static_cast<double>(block.level()), want, 1e-4)) << what;
        }

        PowerMeter<CF> rails({{"sample_rate", kRate}});
        init(rails);
        feed<CF>(rails, std::span<const CF>(std::vector<CF>(9600UZ, CF(1.f, 1.f))), 997UZ);
        expect(approx(static_cast<double>(rails.level()), 3.0103, 1e-4)) << "both rails at once is |x|^2 = 2, and the meter reads above 0 dBFS rather than clamping at it";
    };

    "a real sinusoid reads 3.0103 dB below the complex one, and a real DC reads the same"_test = [] {
        for (const double amplitude : {1.0, 0.5}) {
            PowerMeter<float> real({{"sample_rate", kRate}});
            init(real);
            const std::vector<float> x = realTone(9600UZ, amplitude);
            feed<float>(real, std::span<const float>(x), 997UZ);
            expect(approx(static_cast<double>(real.level()), 20.0 * std::log10(amplitude) - 3.0103, 1e-4)) << std::format("real tone A={}", amplitude);

            PowerMeter<CF> promoted({{"sample_rate", kRate}});
            init(promoted);
            std::vector<CF> lifted(x.size());
            std::ranges::transform(x, lifted.begin(), [](float sample) { return CF(sample, 0.f); });
            feed<CF>(promoted, std::span<const CF>(lifted), 997UZ);
            expect(eq(promoted.level(), real.level())) << "the same waveform through the complex port, bit for bit: there is no stray factor of two";
        }

        PowerMeter<float> dc({{"sample_rate", kRate}});
        init(dc);
        feed<float>(dc, std::span<const float>(std::vector<float>(9600UZ, 0.5f)), 997UZ);
        expect(approx(static_cast<double>(dc.level()), 20.0 * std::log10(0.5), 1e-4)) << "a real DC reads what a complex DC of the same amplitude reads";
    };

    "the window fills segment by segment and is already correct at the first"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.1}, {"segments", 16U}});
        init(block);
        expect(eq(block.segment_samples.value, 600U));
        expect(eq(block.window_samples.value, 9600U));
        expect(eq(block.coverage(), 0.f)) << "nothing accumulated yet";
        expect(eq(block.level(), static_cast<float>(block.floor_db.value))) << "and the floor rather than 0 dBFS, which is full scale";

        const std::vector<CF> x = complexTone(9600UZ, 1.0);
        feed<CF>(block, std::span<const CF>(x).first(599UZ));
        expect(eq(block.coverage(), 0.f)) << "still short of the first segment";

        for (std::size_t k = 1UZ; k <= 16UZ; ++k) {
            feed<CF>(block, std::span<const CF>(x).subspan(k == 1UZ ? 599UZ : (k - 1UZ) * 600UZ, k == 1UZ ? 1UZ : 600UZ));
            expect(approx(static_cast<double>(block.coverage()), static_cast<double>(k) / 16.0, 1e-6)) << std::format("after {} segments", k);
            expect(approx(static_cast<double>(block.level()), 0.0, 0.01)) << "the partial window is already the right reading for a deterministic signal";
        }
        expect(eq(block.coverage(), 1.f));
    };

    "the window is a boxcar, not a pole"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        feed<CF>(block, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
        expect(approx(static_cast<double>(block.level()), 0.0, 1e-4));

        const std::vector<CF> silence(600UZ, CF{});
        for (std::size_t retired = 1UZ; retired <= 16UZ; ++retired) {
            feed<CF>(block, std::span<const CF>(silence));
            const double remaining = static_cast<double>(16UZ - retired);
            if (remaining > 0.0) {
                expect(approx(static_cast<double>(block.level()), 10.0 * std::log10(remaining / 16.0), 0.01)) << std::format("{} of 16 segments still hold signal", remaining);
            } else {
                expect(eq(block.level(), static_cast<float>(block.floor_db.value))) << "exactly window_samples later the reading is the floor; a single pole never gets there";
            }
        }
    };

    "the partial-window statistics are the closed form"_test = [] {
        struct Case {
            std::size_t length;
            double      bias;
            double      sigma;
        };
        std::vector<Case> cases{{100UZ, -0.022, 0.4354}};
        if (longTestsEnabled()) {
            cases.insert(cases.begin(), Case{10UZ, -0.221, 1.4084});
            cases.push_back(Case{1000UZ, -0.002, 0.1374});
        }
        constexpr std::size_t kTrials = 20000UZ;

        for (const auto& [length, bias, sigma] : cases) {
            const auto [measuredBias, measuredSigma] = levelStatistics(length, kTrials, 0x13198a2e03707344ULL + length);
            const double standardError               = sigma / std::sqrt(static_cast<double>(kTrials));
            expect(lt(std::abs(measuredBias - bias), 3.0 * standardError)) << std::format("N={}: bias {:.4f} against {:.4f}", length, measuredBias, bias);
            expect(lt(std::abs(measuredSigma - sigma), 3.0 * sigma / std::sqrt(2.0 * static_cast<double>(kTrials)))) << std::format("N={}: sigma {:.4f} against {:.4f}", length, measuredSigma, sigma);
        }
    };

    "the full-window figure is 0.0443 dB"_test = [] {
        constexpr std::size_t kTrials            = 400UZ;
        const auto [measuredBias, measuredSigma] = levelStatistics(9600UZ, kTrials, 0xa4093822299f31d0ULL);
        const double sigmaError                  = 0.0443 / std::sqrt(2.0 * static_cast<double>(kTrials));
        std::println("full window: bias {:.5f} dB, sigma {:.5f} dB over {} trials", measuredBias, measuredSigma, kTrials);
        expect(lt(std::abs(measuredSigma - 0.0443), 3.0 * sigmaError)) << "the number a user sees on a 100 ms window";
        expect(lt(std::abs(measuredBias), 3.0 * 0.0443 / std::sqrt(static_cast<double>(kTrials)))) << "and it is unbiased";
    };

    "the reading does not depend on the chunking"_test = [] {
        constexpr std::size_t kLength = 30000UZ;
        const std::vector<CF> x       = noise(kLength, 0x082efa98ec4e6c89ULL);

        PowerMeter<CF> reference({{"sample_rate", kRate}});
        init(reference);
        std::vector<double> want(kLength);
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            std::ignore = drive(reference, std::span<const CF>(x).subspan(i, 1UZ));
            want[i]     = reference.linear_power();
        }

        for (const std::size_t chunk : {3UZ, 17UZ, 600UZ, 601UZ, 4096UZ}) {
            PowerMeter<CF> block({{"sample_rate", kRate}});
            init(block);
            std::size_t fed   = 0UZ;
            bool        equal = true;
            while (fed < kLength) {
                const std::size_t count = std::min(chunk, kLength - fed);
                std::ignore             = drive(block, std::span<const CF>(x).subspan(fed, count));
                fed += count;
                equal = equal && block.linear_power() == want[fed - 1UZ];
            }
            expect(that % equal) << std::format("chunk {}: the segment boundary is an absolute offset, so the reading is bit-identical", chunk);
            expect(eq(block.level(), reference.level()));
        }
    };

    "no drift over a hundred million samples"_test = [] {
        constexpr std::size_t kBuffer = 600UZ * 1748UZ; // a whole number of segments, so repeated passes keep the segment boundaries aligned
        constexpr std::size_t kPasses = 96UZ;

        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        const std::vector<CF> x = noise(kBuffer, 0x2ff24fa2ee84e39cULL);
        for (std::size_t pass = 0UZ; pass < kPasses; ++pass) {
            feed<CF>(block, std::span<const CF>(x), 4096UZ);
        }

        double fresh = 0.0;
        for (const CF& sample : std::span<const CF>(x).last(9600UZ)) {
            fresh += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
        fresh /= 9600.0;
        std::println("after {} samples: linear_power {:.15g} against a fresh mean of {:.15g}", kBuffer * kPasses, block.linear_power(), fresh);
        expect(lt(std::abs(block.linear_power() - fresh) / fresh, 1e-12)) << "nothing is ever subtracted, so there is no residue to accumulate";

        // A single 9600-sample window of noise has a 0.0443 dB spread of its own, so the 0 dB check needs a signal
        // whose power is not a random variable; the drift is checked above.
        PowerMeter<CF> tone({{"sample_rate", kRate}});
        init(tone);
        const std::vector<CF> steady = complexTone(kBuffer, 1.0);
        for (std::size_t pass = 0UZ; pass < kPasses; ++pass) {
            feed<CF>(tone, std::span<const CF>(steady), 4096UZ);
        }
        expect(lt(std::abs(static_cast<double>(tone.level())), 0.01)) << "and a unit tone still reads 0.000 dBFS a hundred million samples later";
    };

    "the floor is a clamp and not an epsilon"_test = [] {
        const auto readAt = [](double power, double floorDb) {
            PowerMeter<CF> block({{"sample_rate", kRate}, {"floor_db", floorDb}});
            init(block);
            feed<CF>(block, std::span<const CF>(std::vector<CF>(9600UZ, CF(static_cast<float>(std::sqrt(power)), 0.f))), 600UZ);
            return static_cast<double>(block.level());
        };

        expect(approx(readAt(1e-19, -200.0), -190.0, 1e-3)) << "an additive epsilon of 1e-20 would read -189.586 instead";
        expect(approx(readAt(1e-18, -200.0), -180.0, 1e-3));
        expect(approx(readAt(1e-16, -140.0), -140.0, 1e-9)) << "and below the floor the answer is exactly the floor";

        PowerMeter<CF> silent({{"sample_rate", kRate}});
        init(silent);
        feed<CF>(silent, std::span<const CF>(std::vector<CF>(9600UZ, CF{})), 600UZ);
        expect(eq(silent.level(), -200.f)) << "a true power of zero is the floor rather than negative infinity";
    };

    "a settings change resets the window, and floor_db alone does not"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        feed<CF>(block, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
        expect(eq(block.coverage(), 1.f));

        for (const gr::property_map& change : {gr::property_map{{"sample_rate", 48000.f}}, gr::property_map{{"window_time", 0.05}}, gr::property_map{{"segments", 8U}}}) {
            PowerMeter<CF> filled({{"sample_rate", kRate}});
            init(filled);
            feed<CF>(filled, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
            apply<CF>(filled, change);
            expect(eq(filled.coverage(), 0.f)) << "a window rescaled into a different shape holds neither the old nor the new average";
            expect(eq(filled.level(), -200.f));
        }

        apply<CF>(block, {{"floor_db", -140.0}});
        expect(eq(block.coverage(), 1.f)) << "floor_db alone changes the clamp and nothing else";
        expect(approx(static_cast<double>(block.level()), 0.0, 1e-4));
    };

    "degenerate settings"_test = [] {
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", 0.f}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", -1.f}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"sample_rate", std::numeric_limits<float>::infinity()}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"window_time", 0.0}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"window_time", -0.1}});
            init(rejected);
        }));
        expect(throws([] {
            PowerMeter<CF> rejected({{"segments", 0U}});
            init(rejected);
        }));

        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        expect(throws([&block] { apply<CF>(block, {{"segments", 0U}}); })) << "on a live change too";

        PowerMeter<CF> tiny({{"sample_rate", kRate}, {"window_time", 1e-9}, {"segments", 16U}});
        init(tiny);
        expect(eq(tiny.segment_samples.value, 1U)) << "a window too short for a segment is clamped to one sample";
        expect(eq(tiny.window_samples.value, 16U)) << "and the realized window is reported rather than silently wrong";

        PowerMeter<CF> single({{"sample_rate", kRate}, {"segments", 1U}});
        init(single);
        feed<CF>(single, std::span<const CF>(complexTone(9600UZ, 1.0)), 600UZ);
        expect(approx(static_cast<double>(single.level()), 0.0, 1e-4)) << "one segment is an integrate-and-dump and is legal";
    };

    "tags arrive at the sink, change nothing and cause no error"_test = [] {
        const auto readingOf = [](bool withTags) {
            gr::Graph graph;
            auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 9600U}, {"mark_tag", false}});
            if (withTags) {
                source._tags.emplace_back(0UZ, gr::property_map{{gr::property_map::key_type{"trigger_name"}, gr::pmt::Value(std::string("alpha"))}});
                source._tags.emplace_back(1UZ, gr::property_map{{gr::property_map::key_type{"trigger_time"}, gr::pmt::Value(std::uint64_t{111})}});
                source._tags.emplace_back(1UZ, gr::property_map{{gr::property_map::key_type{"trigger_offset"}, gr::pmt::Value(0.5f)}});
                source._tags.emplace_back(37UZ, gr::property_map{{gr::property_map::key_type{"num_channels"}, gr::pmt::Value(gr::Size_t{3})}});
                source._tags.emplace_back(512UZ, gr::property_map{{gr::property_map::key_type{"rx_overflow"}, gr::pmt::Value(true)}});
                source._tags.emplace_back(900UZ, gr::property_map{{gr::property_map::key_type{"signal_name"}, gr::pmt::Value(std::string("iq"))}});
                source._tags.emplace_back(1200UZ, gr::property_map{{gr::property_map::key_type{"t0"}, gr::pmt::Value(std::string("private"))}});
            }

            auto& meter = graph.emplaceBlock<PowerMeter<CF>>({{"sample_rate", kRate}});
            boost::ut::expect(graph.connect<"out", "in">(source, meter).has_value());

            gr::scheduler::Simple scheduler;
            boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
            boost::ut::expect(scheduler.runAndWait().has_value());
            return std::pair{meter.level(), meter.coverage()};
        };

        const auto [taggedLevel, taggedCoverage] = readingOf(true);
        const auto [plainLevel, plainCoverage]   = readingOf(false);
        expect(eq(taggedLevel, plainLevel)) << "a sink receives tags, interprets none and publishes none, so the reading is the same";
        expect(eq(taggedCoverage, plainCoverage));
        expect(eq(taggedCoverage, 1.f)) << "and the run was long enough for the window to fill";
    };

    "a reader in another thread never sees a torn value"_test = [] {
        const double   seconds = longTestsEnabled() ? 10.0 : 0.5;
        PowerMeter<CF> block({{"sample_rate", kRate}});
        init(block);
        const std::vector<CF> loud  = complexTone(4096UZ, 1.0);
        const std::vector<CF> quiet = complexTone(4096UZ, 0.1);

        std::atomic<bool> running{true};
        std::atomic<int>  outOfRange{0};
        std::thread       reader([&block, &running, &outOfRange] {
            while (running.load(std::memory_order_relaxed)) {
                const float level = block.level();
                if (!(level >= -200.f && level <= 3.1f)) {
                    outOfRange.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });

        const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < until) {
            feed<CF>(block, std::span<const CF>(loud), 4096UZ);
            feed<CF>(block, std::span<const CF>(quiet), 4096UZ);
        }
        running.store(false, std::memory_order_relaxed);
        reader.join();
        expect(eq(outOfRange.load(), 0)) << "every value a reader sees is a mean of complete segment sets, never a torn double";
    };

    "the logarithm is not on the sample path"_test = [] {
        using Clock                   = std::chrono::steady_clock;
        constexpr std::size_t kLength = 1UZ << 22;
        constexpr std::size_t kChunk  = 4096UZ;
        constexpr int         kRuns   = 7;

        const std::vector<CF> x = noise(kLength, 0x243f6a8885a308d3ULL);

        PowerMeter<CF> narrow({{"sample_rate", kRate}});
        init(narrow); // 600-sample segments
        PowerMeter<CF> wide({{"sample_rate", 25e6f}});
        init(wide); // 156250-sample segments
        double sink = 0.0;

        const auto sweep = [&x, &sink](PowerMeter<CF>& meter, bool poll) {
            for (std::size_t base = 0UZ; base < kLength; base += kChunk) {
                std::ignore = drive(meter, std::span<const CF>(x).subspan(base, kChunk));
                if (poll && (base / kChunk) % 24UZ == 0UZ) { // 10 Hz against 4096-sample calls at 96 kHz
                    sink += static_cast<double>(meter.level());
                }
            }
        };

        const std::array<std::pair<const char*, std::function<void()>>, 4UZ> arms{{
            {"|x|^2 accumulation only",
                [&x, &sink] {
                    double total = 0.0;
                    for (const CF& sample : x) {
                        total += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
                    }
                    sink += total;
                }},
            {"PowerMeter, 96 kHz window", [&sweep, &narrow] { sweep(narrow, false); }},
            {"PowerMeter, 25 MS/s window", [&sweep, &wide] { sweep(wide, false); }},
            {"PowerMeter, 96 kHz, polled at 10 Hz", [&sweep, &narrow] { sweep(narrow, true); }},
        }};

        std::vector<double> best(arms.size(), 1e300);
        std::vector<double> worst(arms.size(), 0.0);
        for (int repeat = 0; repeat <= kRuns; ++repeat) {
            for (std::size_t arm = 0UZ; arm < arms.size(); ++arm) {
                const auto start = Clock::now();
                arms[arm].second();
                const double ns = std::chrono::duration<double, std::nano>(Clock::now() - start).count() / static_cast<double>(kLength);
                if (repeat > 0) {
                    best[arm]  = std::min(best[arm], ns);
                    worst[arm] = std::max(worst[arm], ns);
                }
            }
        }
        for (std::size_t arm = 0UZ; arm < arms.size(); ++arm) {
            std::println("{:<38} {:7.3f} ns/sample (spread {:.3f})", arms[arm].first, best[arm], worst[arm] - best[arm]);
        }
        std::println("[checksum {:g}]", sink);

        // The tight 1.5x bound holds only under ENABLE_BENCHMARK_TESTS, where the harness controls the run; an
        // ordinary ctest run measures the scheduler as much as this code, so the bound is 3.0 there. Either bound
        // still catches a per-sample log10, which costs tens of times the accumulation rather than tens of percent.
        const double bound = std::getenv("ENABLE_BENCHMARK_TESTS") != nullptr ? 1.5 : 3.0;

        expect(lt(best[1UZ] / best[0UZ], bound)) << "the 96 kHz window costs what accumulating |x|^2 costs";
        expect(lt(best[2UZ] / best[0UZ], bound)) << "and so does the 25 MS/s window: the memory is 16 doubles at either rate";
        expect(lt(best[3UZ] / best[1UZ], bound)) << "and polling level() at 10 Hz does not change it";
    };

    "the record port publishes a completed window, and an unconnected one publishes nothing"_test = [] {
        constexpr gr::Size_t kSegments = 4U;
        PowerMeter<CF>       block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();

        // two whole windows of a unit-power tone, so a record is due and its value is known
        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   x(2UZ * window, CF(1.f, 0.f));
        const auto        records = drive(block, std::span<const CF>(x));

        expect(!records.empty()) << "a completed window publishes a record";
        if (records.empty()) {
            return;
        }
        const auto& record = records.front();
        expect(eq(record.signal_names.size(), 3UZ));
        expect(eq(record.signal_names[0UZ], std::string("power")));
        expect(eq(record.signal_units[0UZ], std::string("dBFS")));
        expect(std::abs(record.signal_values[0UZ] - 0.f) < 1e-3f) << "a unit-power tone reads 0 dBFS";
        expect(std::abs(record.signal_values[1UZ] - 1.f) < 1e-3f) << "and 1 in linear power";
        const auto& meta = record.meta_information[0UZ];
        expect(meta.find(std::pmr::string("sample_rate")) != meta.end()) << "a sample-domain meter states its rate";
        expect(meta.find(std::pmr::string("sample_start")) != meta.end());

        PowerMeter<CF> bare({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        bare.settings().init();
        std::ignore = bare.settings().applyStagedParameters();
        {
            namespace test        = gr::blocks::analog::test;
            std::size_t published = 0UZ;
            for (std::size_t base = 0UZ; base < x.size();) {
                std::vector<gr::DataSet<float>>      made(4UZ);
                test::InputSpan<CF>                  inSpan{std::span<const CF>(x).subspan(base)};
                test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
                outSpan.isConnected = false; // this module's shim carries no constructor argument for it
                std::ignore         = bare.processBulk(inSpan, outSpan);
                published += outSpan.count;
                if (inSpan.consumed == 0UZ) {
                    break;
                }
                base += inSpan.consumed;
            }
            expect(eq(published, 0UZ)) << "an unconnected port has nothing published to it";
        }
        expect(eq(block.linear_power(), bare.linear_power())) << "and the reading does not depend on whether anyone is listening";
    };

    "several windows closing in one call are several records, each stamped at its own first sample"_test = [] {
        constexpr gr::Size_t  kSegments = 4U;
        constexpr std::size_t kWindows  = 8UZ;
        PowerMeter<CF>        block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        init(block);

        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   x(kWindows * window + 1UZ, CF(1.f, 0.f)); // one spare, since a call holds one sample back

        namespace test = gr::blocks::analog::test;
        std::vector<gr::DataSet<float>>      made(16UZ);
        test::InputSpan<CF>                  inSpan{std::span<const CF>(x)};
        test::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(made)};
        std::ignore = block.processBulk(inSpan, outSpan);

        expect(eq(outSpan.count, kWindows)) << "eight windows closed inside the one call and each is its own record";
        for (std::size_t w = 0UZ; w < std::min(outSpan.count, kWindows); ++w) {
            const auto& meta  = made[w].meta_information[0UZ];
            const auto  entry = meta.find(std::pmr::string("sample_start"));
            expect(entry != meta.end()) << "record " << w << " states no place in the stream";
            if (entry != meta.end()) {
                const auto* index = entry->second.get_if<std::uint64_t>();
                expect(index != nullptr && *index == w * window) << "record " << w << " is stamped at its own window's first sample";
            }
        }
    };

    "a stream ending mid-window emits a final record covering what it had"_test = [] {
        constexpr gr::Size_t kSegments = 4U;
        PowerMeter<CF>       block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", kSegments}});
        init(block);

        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   x(2UZ * window + window / 2UZ, CF(1.f, 0.f));

        const auto during = drive(block, std::span<const CF>(x));
        expect(eq(during.size(), 2UZ)) << "two whole windows closed while the stream ran";

        const auto last = finish(block, std::span<const CF>{});
        expect(eq(last.size(), 1UZ)) << "and what accumulated after them is reported rather than discarded";
        if (last.empty()) {
            return;
        }
        const auto& meta  = last.front().meta_information[0UZ];
        const auto  entry = meta.find(std::pmr::string("sample_start"));
        expect(entry != meta.end());
        if (entry != meta.end()) {
            const auto* index = entry->second.get_if<std::uint64_t>();
            expect(index != nullptr && *index == 2UZ * window) << "starting where the last whole window ended";
        }
        expect(eq(last.front().signal_values[0UZ], block.level())) << "and stating the reading a poller sees at the same moment";
    };

    "the record's level is clamped exactly where the reader's is"_test = [] {
        PowerMeter<CF> block({{"sample_rate", kRate}, {"window_time", 0.001}, {"segments", gr::Size_t{4U}}, {"floor_db", -60.0}});
        init(block);

        const std::size_t window = static_cast<std::size_t>(block.window_samples.value);
        std::vector<CF>   quiet(window + 1UZ, CF(1e-6f, 0.f)); // -120 dBFS, far under the floor

        const auto records = drive(block, std::span<const CF>(quiet));
        expect(!records.empty()) << "a window closed, so a record went out";
        if (records.empty()) {
            return;
        }
        expect(approx(static_cast<double>(block.level()), -60.0, 1e-6)) << "the reader clamps at floor_db";
        expect(eq(records.front().signal_values[0UZ], block.level())) << "and the record states the same number rather than an unclamped one";
        expect(records.front().signal_values[1UZ] < 1e-9f) << "while the linear channel carries the raw ratio, unclamped";
    };
};

int main() { /* not needed for UT */ }
