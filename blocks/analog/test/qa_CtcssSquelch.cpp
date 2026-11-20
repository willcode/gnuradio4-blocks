#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/analog/Squelch.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>

#include <gnuradio-4.0/testing/InitChecks.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::analog::CtcssSquelch;
namespace spans = gr::blocks::testing::span;

constexpr float  kRate = 8000.f;
constexpr double kTone = 100.0;

[[nodiscard]] CtcssSquelch makeSquelch(gr::property_map settings) {
    CtcssSquelch block(std::move(settings));
    boost::ut::expect(block.check({}).has_value());
    return block;
}

[[nodiscard]] std::vector<float> tone(std::size_t nSamples, double frequency, double amplitude = 1.0, std::size_t phaseOffset = 0UZ) {
    std::vector<float> signal(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        signal[i] = static_cast<float>(amplitude * std::cos(2.0 * std::numbers::pi * frequency * static_cast<double>(i + phaseOffset) / static_cast<double>(kRate)));
    }
    return signal;
}

/// @brief Deterministic noise of a stated RMS.
[[nodiscard]] std::vector<float> noise(std::size_t nSamples, double rms) {
    std::vector<float> signal(nSamples);
    std::uint64_t      state = 0x9E3779B97F4A7C15ULL;
    double             sum   = 0.0;
    for (std::size_t i = 0UZ; i < nSamples; ++i) {
        state          = state * 6364136223846793005ULL + 1442695040888963407ULL;
        const double u = static_cast<double>(state >> 11U) / static_cast<double>(1ULL << 53U) - 0.5;
        signal[i]      = static_cast<float>(u);
        sum += u * u;
    }
    const double scale = rms / std::sqrt(sum / static_cast<double>(nSamples));
    for (float& sample : signal) {
        sample = static_cast<float>(static_cast<double>(sample) * scale);
    }
    return signal;
}

[[nodiscard]] std::vector<char> observeUnmuted(CtcssSquelch& block, std::span<const float> input) {
    const gr::settings::detail::OwnThread ownThread(gr::settings::detail::settingsOf(block)); // the work function publishes `unmuted`
    std::vector<char>                     open(input.size());
    for (std::size_t i = 0UZ; i < input.size(); ++i) {
        std::ignore = spans::run<CtcssSquelch, float>(block, input.subspan(i, 1UZ));
        open[i]     = block.status.unmuted.value ? 1 : 0;
    }
    return open;
}

} // namespace

const boost::ut::suite<"CtcssSquelch"> ctcssSquelchTests = [] {
    using namespace boost::ut;

    "a tone exactly on the analysis frequency gives half its amplitude"_test = [] {
        CtcssSquelch             block  = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", 0.1}});
        const std::vector<float> signal = tone(800UZ, kTone);
        std::ignore                     = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal));

        expect(approx(block.lastMagnitude(1UZ), 0.5, 1e-6)) << "the 1/N normalization makes |X| independent of the integration length";
        expect(eq(block.integrationLength(), 800U));

        CtcssSquelch             shorter = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 400U}, {"level", 0.1}});
        const std::vector<float> half    = tone(400UZ, kTone);
        std::ignore                      = spans::run<CtcssSquelch, float>(shorter, std::span<const float>(half));
        expect(approx(shorter.lastMagnitude(1UZ), 0.5, 1e-6)) << "and half the length gives the same number";
    };

    "the neighbors sit only a few percent below the center"_test = [] {
        CtcssSquelch             block  = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", 0.1}});
        const std::vector<float> signal = tone(800UZ, kTone);
        std::ignore                     = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal));

        // The image term makes the two neighbors differ by about 1 percent, and which of them is lower follows the
        // tone's phase, so the tone here is a cosine and 0.4631 is the lower of the two.
        expect(approx(block.lastMagnitude(0UZ), 0.46303, 1e-3)) << "lower neighbor at 98 Hz";
        expect(approx(block.lastMagnitude(2UZ), 0.47237, 1e-3)) << "upper neighbor at 102 Hz";
        expect(lt(block.lastMagnitude(0UZ), block.lastMagnitude(1UZ)));
        expect(lt(block.lastMagnitude(2UZ), block.lastMagnitude(1UZ))) << "the center wins, but only by 6 percent";
    };

    "it opens on its tone and on nothing else"_test = [] {
        const gr::property_map settings = {{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", 0.1}};

        CtcssSquelch             onTone = makeSquelch(settings);
        const std::vector<float> match  = tone(1600UZ, kTone);
        std::ignore                     = spans::run<CtcssSquelch, float>(onTone, std::span<const float>(match));
        expect(onTone.status.unmuted.value) << "a tone at the analysis frequency opens the squelch";

        CtcssSquelch             onNoise = makeSquelch(settings);
        const std::vector<float> hiss    = noise(1600UZ, 1.0 / std::numbers::sqrt2);
        std::ignore                      = spans::run<CtcssSquelch, float>(onNoise, std::span<const float>(hiss));
        expect(!onNoise.status.unmuted.value) << "noise of the same RMS does not";

        CtcssSquelch             offTone   = makeSquelch(settings);
        const std::vector<float> displaced = tone(1600UZ, kTone + 30.0); // 3/T Hz away at T = 100 ms
        std::ignore                        = spans::run<CtcssSquelch, float>(offTone, std::span<const float>(displaced));
        expect(!offTone.status.unmuted.value) << "and neither does a tone 3/T Hz away";
    };

    "the decision changes only on a multiple of the integration length"_test = [] {
        CtcssSquelch             block  = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 400U}, {"level", 0.1}});
        const std::vector<float> signal = tone(805UZ, kTone);
        const std::vector<char>  open   = observeUnmuted(block, std::span<const float>(signal));

        for (std::size_t i = 0UZ; i < open.size(); ++i) {
            expect(eq(open[i] != 0, i >= 399UZ)) << "the decision at sample " << i << " must be the one taken at the last block boundary";
        }
    };

    "the level boundary is the center magnitude itself"_test = [] {
        const std::vector<float> signal = tone(800UZ, kTone);
        for (const double level : {0.49, 0.51}) {
            CtcssSquelch block = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", level}});
            std::ignore        = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal));
            expect(eq(block.status.unmuted.value, level < 0.5)) << "level " << level << " against a center magnitude of 0.5";
        }
    };

    "retuning restarts the integration and holds the standing decision"_test = [] {
        CtcssSquelch             block  = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", 0.1}});
        const std::vector<float> signal = tone(4000UZ, kTone);

        std::ignore = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal).first(1600UZ));
        expect(block.status.unmuted.value) << "open on its tone";

        std::ignore = gr::testing::setAndApply(gr::settings::detail::settingsOf(block), {{"tone_freq", 200.0}});

        const auto held = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal).subspan(1600UZ, 799UZ));
        expect(block.status.unmuted.value) << "the standing decision holds until the first new block completes";
        expect(std::ranges::all_of(held.samples, [](float sample) { return std::isfinite(sample); })) << "and nothing goes non-finite across the change";

        std::ignore = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal).subspan(2399UZ, 1UZ));
        expect(!block.status.unmuted.value) << "and then the 200 Hz detector closes on a 100 Hz tone";
    };

    "output and tag offsets do not depend on chunking"_test = [] {
        std::vector<float> signal = tone(2800UZ, kTone);
        std::ranges::for_each(std::span<float>(signal).first(400UZ), [](float& sample) { sample *= 0.02f; });
        std::ranges::fill(std::span<float>(signal).subspan(2000UZ), 0.f);

        const gr::property_map settings = {{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 400U}, {"level", 0.1}, {"ramp", 32U}};
        for (const bool gate : {false, true}) {
            gr::property_map armed = settings;
            armed["gate"]          = gate;

            CtcssSquelch reference = makeSquelch(armed);
            const auto   want      = spans::run<CtcssSquelch, float>(reference, std::span<const float>(signal));
            expect(gt(want.samples.size(), 0UZ)) << "the chunking test needs a burst to compare";

            for (const std::size_t chunkSize : {1UZ, 7UZ, 399UZ, 4096UZ}) {
                CtcssSquelch block = makeSquelch(armed);
                const auto   got   = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal), chunkSize);
                expect(std::ranges::equal(got.samples, want.samples)) << "gate=" << gate << " chunk " << chunkSize << " must be bit-identical";
                expect(eq(got.tags.size(), want.tags.size())) << "gate=" << gate << " chunk " << chunkSize;
            }
        }
    };

    "a passing gr:sample_rate leaves with the value it arrived with, private keys included"_test = [] {
        constexpr float kUpstream = 192000.f;

        const std::vector<float>   signal = tone(2000UZ, kTone);
        const std::vector<gr::Tag> tags{gr::Tag{500UZ, gr::property_map{{gr::tag::SAMPLE_RATE.key(), kUpstream}, {"test_probe", static_cast<gr::Size_t>(7)}}}};

        CtcssSquelch block = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 400U}, {"level", 0.1}});
        const auto   got   = spans::run<CtcssSquelch, float>(block, std::span<const float>(signal), 0UZ, std::span<const gr::Tag>(tags));

        std::vector<float> rates;
        for (const gr::Tag& forwarded : got.tags) {
            if (const auto found = forwarded.map.find(gr::property_map::key_type{gr::tag::SAMPLE_RATE.key()}); found != forwarded.map.end()) {
                rates.push_back(found->second.value_or(0.f));
            }
        }
        expect(that % (rates == std::vector<float>{kUpstream})) << "the block writes no rate of its own into a passing tag";
        expect(eq(got.offsetsOf("test_probe").size(), 1UZ)) << "and the private key passes beside it";
    };

    "unusable parameters are rejected at settings time"_test = [] {
        using gr::testing::initRefused;
        expect(initRefused<CtcssSquelch>({{"sample_rate", 0.f}})) << "zero sample_rate";
        expect(initRefused<CtcssSquelch>({{"epsilon", 0.0}})) << "zero epsilon";
        expect(initRefused<CtcssSquelch>({{"tone_freq", -1.0}})) << "negative tone_freq";
        expect(initRefused<CtcssSquelch>({{"sample_rate", kRate}, {"tone_freq", 3950.0}})) << "an upper neighbor above Nyquist";
    };

    "a refusal names the key the call changed"_test = [] {
        CtcssSquelch                 block   = makeSquelch({{"sample_rate", kRate}, {"tone_freq", kTone}});
        const gr::settings::Rejected refused = gr::testing::setAndApply(gr::settings::detail::settingsOf(block), {{"epsilon", 50.0}});
        expect(fatal(eq(refused.size(), 1UZ))) << "an upper neighbor above Nyquist";
        expect(eq(refused.front().key, std::string("epsilon"))) << "the tone the call left alone is not refused";
    };

    "nanoseconds per sample"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;

        const std::vector<float> x = tone(1UZ << 16, kTone);
        std::vector<float>       y(x.size());

        struct Arm {
            const char*      label;
            gr::property_map settings;
        };
        const Arm     kArms[]  = {{"CtcssSquelch open", {{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", 0.1}}}, {"CtcssSquelch ramping", {{"sample_rate", kRate}, {"tone_freq", kTone}, {"length", 800U}, {"level", 0.1}, {"ramp", 64U}}}};
        constexpr int kRepeats = 7;

        std::vector<CtcssSquelch> blocks;
        for (const Arm& arm : kArms) {
            blocks.push_back(makeSquelch(arm.settings));
        }

        std::vector<double> best(std::size(kArms), 1e30);
        std::vector<double> worst(std::size(kArms), 0.0);
        for (int repeat = 0; repeat < kRepeats; ++repeat) { // arms interleaved, so a thermal drift moves all of them
            for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
                spans::InputSpan<float>               inSpan{std::span<const float>(x)};
                spans::OutputSpan<float>              outSpan{std::span<float>(y)};
                const gr::settings::detail::OwnThread ownThread(gr::settings::detail::settingsOf(blocks[a]));
                const auto                            start = Clock::now();
                std::ignore                                 = blocks[a].processBulk(inSpan, outSpan);
                const double ns                             = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
                best[a]                                     = std::min(best[a], ns);
                worst[a]                                    = std::max(worst[a], ns);
            }
        }
        for (std::size_t a = 0UZ; a < std::size(kArms); ++a) {
            std::println("{}: best {:.3f} ns/sample, spread {:.3f} ns", kArms[a].label, best[a], worst[a] - best[a]);
        }
    };
};

const boost::ut::suite<"CtcssSquelch settings at the initialization"> ctcssSquelchInitTests = [] {
    using namespace boost::ut;

    "CtcssSquelch initializes at its defaults and with each setting off its default"_test = [] {
        const std::vector<std::string> failures = gr::testing::initFailures<gr::blocks::analog::CtcssSquelch>({
            {"sample_rate", 16000.f},
            {"tone_freq", 123.0},
            {"level", 0.05},
            {"length", gr::Size_t{1600U}},
            {"epsilon", 0.03},
            {"ramp", gr::Size_t{8U}},
            {"gate", true},
            {"level_min", 0.01},
            {"level_max", 0.9},
            {"level_step", 0.02},
        });
        expect(failures.empty()) << std::format("{}", failures);
    };
};

int main() { /* tests are automatically registered and run */ }
