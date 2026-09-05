#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/analog/NoiseBlanker.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include "TestSpans.hpp"

namespace {

using gr::blocks::analog::NoiseBlanker;
using gr::testing::ProcessFunction;
using gr::testing::TagSink;
using gr::testing::TagSource;

namespace test = gr::blocks::analog::test;

using CF = std::complex<float>;

constexpr double kPi   = std::numbers::pi;
constexpr float  kRate = 96000.f;

template<typename T>
[[nodiscard]] NoiseBlanker<T> make(gr::property_map settings = {}) {
    NoiseBlanker<T> block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename TBlock>
void apply(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
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

/// A constant-magnitude tone: |x|^2 never departs from its own mean, so a correct detector never fires on it.
[[nodiscard]] std::vector<CF> tone(std::size_t count, double amplitude, double normalized = 0.013) {
    std::vector<CF> samples(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        const double phase = 2.0 * kPi * normalized * static_cast<double>(i);
        samples[i]         = CF(static_cast<float>(amplitude * std::cos(phase)), static_cast<float>(amplitude * std::sin(phase)));
    }
    return samples;
}

/// @brief A signal band-limited to @p bandwidth of the sample rate, as a sum of random-phase bins inside it.
[[nodiscard]] std::vector<CF> bandLimited(std::size_t count, double bandwidth, std::uint64_t seed) {
    Random               rng{seed};
    const std::ptrdiff_t half = static_cast<std::ptrdiff_t>(bandwidth * 0.5 * static_cast<double>(count));
    std::vector<CF>      samples(count, CF{});
    std::vector<double>  phase(static_cast<std::size_t>(2 * half + 1));
    for (double& value : phase) {
        value = 2.0 * kPi * rng.uniform();
    }
    for (std::ptrdiff_t bin = -half; bin <= half; ++bin) {
        const double turn  = 2.0 * kPi * static_cast<double>(bin) / static_cast<double>(count);
        const double start = phase[static_cast<std::size_t>(bin + half)];
        for (std::size_t i = 0UZ; i < count; ++i) {
            const double angle = start + turn * static_cast<double>(i);
            samples[i] += CF(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)));
        }
    }
    const double scale = 1.0 / std::sqrt(static_cast<double>(2 * half + 1));
    for (CF& sample : samples) {
        sample *= static_cast<float>(scale);
    }
    return samples;
}

/// @brief Energy of @p signal inside `|f| <= bandwidth/2`, by a direct transform of the bins that matter.
[[nodiscard]] double inBandEnergy(std::span<const CF> signal, double bandwidth) {
    const std::ptrdiff_t half  = static_cast<std::ptrdiff_t>(bandwidth * 0.5 * static_cast<double>(signal.size()));
    double               total = 0.0;
    for (std::ptrdiff_t bin = -half; bin <= half; ++bin) {
        const double turn = -2.0 * kPi * static_cast<double>(bin) / static_cast<double>(signal.size());
        double       re   = 0.0;
        double       im   = 0.0;
        for (std::size_t i = 0UZ; i < signal.size(); ++i) {
            const double angle = turn * static_cast<double>(i);
            const double c     = std::cos(angle);
            const double s     = std::sin(angle);
            re += static_cast<double>(signal[i].real()) * c - static_cast<double>(signal[i].imag()) * s;
            im += static_cast<double>(signal[i].real()) * s + static_cast<double>(signal[i].imag()) * c;
        }
        total += (re * re + im * im) / static_cast<double>(signal.size() * signal.size());
    }
    return total;
}

[[nodiscard]] double meanPower(std::span<const CF> samples) {
    double total = 0.0;
    for (const CF& sample : samples) {
        total += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
    }
    return total / static_cast<double>(samples.size());
}

[[nodiscard]] bool longTestsEnabled() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

/// A magnitude threshold `k` carried into this block's power units: `T = (pi/4) k^2`, i.e. 20log10(k) - 1.0492 dB.
[[nodiscard]] double thresholdFromMagnitudeRatio(double k) { return 20.0 * std::log10(k) - 1.0492; }

/**
 * @brief Where a censored tracker settles on unit-power noise, and the run rate that follows.
 *
 * An exact reference gives exp(-T) for the trigger rate. A censored tracker is not exact: it never learns from the
 * samples it rejected, so it settles on E[p | p <= T*m] rather than on E[p], and the fixed point of that is below
 * one. At the default 9.32 dB the shift is 0.17 % and exp(-T) is the answer; at 6 dB it is 10 % and it is not,
 * which is a property of censoring rather than of this implementation.
 */
[[nodiscard]] std::pair<double, double> censoredRunRate(double thresholdDb, std::size_t blankSamples) {
    const double threshold = std::pow(10.0, thresholdDb / 10.0);
    double       mean      = 1.0;
    for (int step = 0; step < 200; ++step) { // m = E[p | p <= T*m] for a unit-mean exponential p
        const double u    = threshold * mean;
        const double tail = std::exp(-u);
        mean              = (1.0 - (1.0 + u) * tail) / (1.0 - tail);
    }
    const double perSample = std::exp(-threshold * mean);
    return {mean, perSample / (1.0 + perSample * static_cast<double>(blankSamples - 1UZ))};
}

} // namespace

const boost::ut::suite<"NoiseBlanker"> noiseBlankerTests = [] {
    using namespace boost::ut;

    "a clean signal comes out delayed and otherwise untouched"_test = [] {
        for (const char* rule : {"interpolate", "hold", "zero"}) {
            NoiseBlanker<CF>       block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}});
            const std::vector<CF>  x     = tone(40000UZ, 0.5);
            const test::Result<CF> got   = test::run(block, std::span<const CF>(x), 997UZ);

            expect(eq(got.samples.size(), x.size())) << "1:1";
            std::size_t firstDifference = got.samples.size();
            for (std::size_t i = 9UZ; i < x.size(); ++i) {
                if (got.samples[i] != x[i - 9UZ]) {
                    firstDifference = i;
                    break;
                }
            }
            expect(eq(firstDifference, got.samples.size())) << std::format("{}: nothing above the threshold, so the output is the input delayed by 9, bit for bit", rule);
            for (std::size_t i = 0UZ; i < 9UZ; ++i) {
                expect(eq(got.samples[i], CF{})) << "and the delay line starts zeroed";
            }
        }
    };

    "the delay is there when the block is off and does not move when it is toggled"_test = [] {
        NoiseBlanker<CF> off = make<CF>({{"sample_rate", kRate}});
        std::vector<CF>  ramp(4000UZ);
        for (std::size_t i = 0UZ; i < ramp.size(); ++i) {
            ramp[i] = CF(static_cast<float>(i), static_cast<float>(2UZ * i));
        }
        const test::Result<CF> passed = test::run(off, std::span<const CF>(ramp), 137UZ);
        expect(eq(passed.samples.size(), ramp.size()));
        bool exact = true;
        for (std::size_t i = 9UZ; i < ramp.size(); ++i) {
            exact = exact && passed.samples[i] == ramp[i - 9UZ];
        }
        expect(that % exact) << "disabled is the input delayed by delay_samples, bit for bit";

        // A mid-stream `enabled` change keeps the alignment: every sample comes out once, in order.
        NoiseBlanker<CF>  toggled = make<CF>({{"sample_rate", kRate}, {"threshold_db", 60.0}});
        std::size_t       calls   = 0UZ;
        std::vector<CF>   out;
        const std::size_t stride = 137UZ;
        for (std::size_t base = 0UZ; base < ramp.size(); base += stride) {
            if (calls == 5UZ) {
                apply(toggled, {{"enabled", true}});
            } else if (calls == 17UZ) {
                apply(toggled, {{"enabled", false}});
            }
            ++calls;
            const std::size_t      count = std::min(stride, ramp.size() - base);
            const test::Result<CF> piece = test::run(toggled, std::span<const CF>(ramp).subspan(base, count), count);
            out.insert(out.end(), piece.samples.begin(), piece.samples.end());
        }
        expect(eq(out.size(), ramp.size())) << "a toggle does not change the number of output samples";
        bool continuous = true;
        for (std::size_t i = 9UZ; i < ramp.size(); ++i) {
            continuous = continuous && out[i] == ramp[i - 9UZ];
        }
        expect(that % continuous) << "and the ramp is continuous across it";
    };

    "the threshold is a power ratio and the conversion from decibels is exact"_test = [] {
        for (const double db : {0.0, 3.0, 9.32, 20.0}) {
            NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", db}});
            expect(approx(block._threshold, std::pow(10.0, db / 10.0), std::pow(10.0, db / 10.0) * 1e-9)) << std::format("threshold_db {}", db);

            // The marked sample only reaches the output delay_samples later, so the probe is followed by enough
            // silence to flush the line; silence is below any threshold and opens nothing of its own.
            const auto fires = [db](double scale) {
                NoiseBlanker<CF> probe = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", db}, {"emit_tags", true}});
                probe._warmup          = 0UZ;
                probe._power           = 1.0;
                std::vector<CF> one(32UZ, CF{}); // the probe is the first sample, so p_bar is still exactly what was set
                one[0UZ] = CF(static_cast<float>(std::sqrt(probe._threshold * scale)), 0.f);
                return test::run(probe, std::span<const CF>(one), 32UZ).tags.size() == 1UZ;
            };
            expect(!fires(1.0 - 1e-5)) << std::format("threshold_db {}: just below T * p_bar does not trigger", db);
            expect(fires(1.0 + 1e-5)) << std::format("threshold_db {}: just above it does", db);
        }

        // Exactly at the threshold, in a case binary arithmetic represents exactly: T = 4, p_bar = 1, |x| = 2.
        NoiseBlanker<CF> exactly = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", 10.0 * std::log10(4.0)}, {"emit_tags", true}});
        exactly._warmup          = 0UZ;
        exactly._power           = 1.0;
        expect(eq(exactly._threshold, 4.0));
        std::vector<CF> atThreshold(32UZ, CF{});
        atThreshold[0UZ] = CF(2.f, 0.f);
        expect(eq(test::run(exactly, std::span<const CF>(atThreshold), 32UZ).tags.size(), 0UZ)) << "a sample at exactly T * p_bar does not trigger: the comparison is strict";

        // The conversion is exact; the two defaults are it, rounded to two decimals. 3.3 gives 9.32108, not 9.32000.
        expect(approx(thresholdFromMagnitudeRatio(3.3), 9.32, 2e-3)) << std::format("k=3.3 maps to {:.5f} dB", thresholdFromMagnitudeRatio(3.3));
        expect(approx(thresholdFromMagnitudeRatio(2.5), 6.91, 2e-3)) << std::format("k=2.5 maps to {:.5f} dB", thresholdFromMagnitudeRatio(2.5));
    };

    "the false-blank rate is the exponential tail"_test = [] {
        const std::size_t     samples    = longTestsEnabled() ? 4000000UZ : 400000UZ;
        std::vector<double>   thresholds = longTestsEnabled() ? std::vector<double>{6.0, 8.0, 9.32} : std::vector<double>{6.0};
        constexpr std::size_t kWarmUp    = 120000UZ;

        for (const double db : thresholds) {
            NoiseBlanker<CF>      block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 100000.0 / static_cast<double>(kRate)}, {"threshold_db", db}, {"emit_tags", true}});
            const std::vector<CF> warm  = noise(kWarmUp, 0x13198a2e03707344ULL);
            std::ignore                 = test::run(block, std::span<const CF>(warm), 4096UZ);
            for (std::size_t pass = 0UZ; pass < 5UZ; ++pass) { // let the censored tracker reach its own fixed point
                std::ignore = test::run(block, std::span<const CF>(noise(kWarmUp, 0x452821e638d01377ULL + pass)), 4096UZ);
            }

            const std::vector<CF>  x   = noise(samples, 0xa4093822299f31d0ULL + static_cast<std::uint64_t>(db));
            const test::Result<CF> got = test::run(block, std::span<const CF>(x), 4096UZ);

            // The tail is tested against the reference the block actually holds, which is the exact statement of
            // P(false) = exp(-T * p_bar). Where that reference sits is the second assertion, and it is below one:
            // a censored tracker never learns from the samples it rejected. Taking exp(-T) alone is right at the
            // default 9.32 dB (0.17 % apart) and 30 % out at 6 dB.
            // Two triggers less than four samples apart leave one contiguous replaced run and so one marker, which
            // is the first-order merge term below.
            const auto [modeled, modeledRate] = censoredRunRate(db, 7UZ);
            const double perSample            = std::exp(-std::pow(10.0, db / 10.0) * block._power);
            const double wantRate             = perSample / (1.0 + perSample * 6.0) * (1.0 - 3.0 * perSample);
            const double gotRate              = static_cast<double>(got.tags.size()) / static_cast<double>(samples);
            const double error                = std::sqrt(wantRate / static_cast<double>(samples));
            std::println("threshold {:.2f} dB: {} runs, rate {:.6e} against {:.6e} at a tracker of {:.4f} (model {:.4f}); exp(-T) alone is {:.6e}", db, got.tags.size(), gotRate, wantRate, block._power, modeled, std::exp(-std::pow(10.0, db / 10.0)));
            expect(lt(std::abs(gotRate - wantRate), 5.0 * error)) << std::format("threshold_db {}: the trigger rate is the exponential tail of |x|^2 against the tracked reference", db);
            expect(lt(block._power, 1.0)) << "and the reference sits below the true power, which is what censoring costs";
            expect(gt(block._power, 0.5)) << "but not far below it";
            std::ignore = modeledRate;
        }
    };

    "censoring keeps the reference where the noise is"_test = [] {
        constexpr std::size_t kSamples = 200000UZ;
        constexpr std::size_t kPeriod  = 50UZ;
        constexpr double      kInr     = 1000.0; // 30 dB

        std::vector<CF> x = noise(kSamples, 0x082efa98ec4e6c89ULL);
        for (std::size_t at = kPeriod; at < kSamples; at += kPeriod) {
            x[at] += CF(static_cast<float>(std::sqrt(kInr)), 0.f);
        }

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 1000.0 / static_cast<double>(kRate)}});
        std::ignore            = test::run(block, std::span<const CF>(x), 4096UZ);

        double       uncensored = 0.0; // the desensitizing order: update from every sample, then compare
        const double alpha      = block._alpha;
        for (const CF& sample : x) {
            const double power = static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
            uncensored += alpha * (power - uncensored);
        }

        std::println("period 50, INR 30 dB: censored {:.4f}, uncensored {:.4f}", block._power, uncensored);
        expect(approx(block._power, 1.0, 0.05)) << "the censored tracker settles on the noise, which is what the threshold is taken against";
        expect(gt(uncensored, 15.0)) << "where an uncensored one is dragged up by the events it is meant to detect";
    };

    "the warm-up blanks nothing and then settles on the budget"_test = [] {
        constexpr std::size_t kTau  = 1000UZ;
        NoiseBlanker<CF>      block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", static_cast<double>(kTau) / static_cast<double>(kRate)}, {"emit_tags", true}});

        std::vector<CF> loud = noise(kTau, 0x2ff24fa2ee84e39cULL);
        for (CF& sample : loud) {
            sample *= 100.f; // loud enough that a fixed initial reference of 1.0 would blank about a third of them
        }
        expect(eq(test::run(block, std::span<const CF>(loud), 128UZ).tags.size(), 0UZ)) << "not one blanked sample in the first time constant, whatever the level";

        // The tracker is still climbing for a few time constants after the warm-up ends, so the settled duty is read
        // after that rather than across it: the cost of the warm-up is stated as one time constant of no protection,
        // plus the further few during which the reference is still climbing.
        std::vector<CF> settling = noise(10UZ * kTau, 0x452821e638d01377ULL);
        std::vector<CF> after    = noise(10UZ * kTau, 0xbe5466cf34e90c6cULL);
        for (CF& sample : settling) {
            sample *= 100.f;
        }
        for (CF& sample : after) {
            sample *= 100.f;
        }
        std::ignore                    = test::run(block, std::span<const CF>(settling), 128UZ);
        const test::Result<CF> got     = test::run(block, std::span<const CF>(after), 128UZ);
        const double           blanked = static_cast<double>(got.tags.size()) * 9.0 / static_cast<double>(after.size());
        const double           want    = std::exp(-std::pow(10.0, 9.32 / 10.0)) * 9.0;
        std::println("after the warm-up: blanked {:.5f} of the stream against a budget of {:.5f}", blanked, want);
        expect(lt(blanked, 3.0 * want)) << "and afterwards the duty is the budget, within a factor of three";
    };

    "detection probability follows the non-central chi-square"_test = [] {
        struct Case {
            double inrDb;
            double want;
        };
        constexpr Case        kCases[]{{6.0, 0.12179}, {10.0, 0.67512}, {13.0, 0.98868}};
        const std::size_t     kImpulses = longTestsEnabled() ? 100000UZ : 30000UZ;
        constexpr std::size_t kSpacing  = 20UZ;

        for (const auto& [inrDb, want] : kCases) {
            const double      amplitude = std::sqrt(std::pow(10.0, inrDb / 10.0));
            const std::size_t count     = kImpulses * kSpacing;
            std::vector<CF>   x         = noise(count, 0x9216d5d98979fb1bULL + static_cast<std::uint64_t>(inrDb));
            for (std::size_t k = 1UZ; k <= kImpulses; ++k) {
                const std::size_t at = k * kSpacing - 1UZ;
                x[at] += CF(static_cast<float>(amplitude), 0.f);
            }

            // The reference is held exact: a 5 % duty of impulses would otherwise raise it and cost detection, which
            // is a property of the scene rather than of the statistic under test here.
            NoiseBlanker<CF> block     = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 1e8 / static_cast<double>(kRate)}, {"replacement", std::string("zero")}});
            block._warmup              = 0UZ;
            block._power               = 1.0;
            const test::Result<CF> got = test::run(block, std::span<const CF>(x), 4096UZ);

            // Counts whether the planted sample was replaced. Counting run starts instead
            // undercounts by every window that met its neighbor, which at 6 dB is a sixth of them.
            std::size_t caught = 0UZ;
            std::size_t direct = 0UZ;
            for (std::size_t k = 1UZ; k <= kImpulses; ++k) {
                const std::size_t at    = k * kSpacing - 1UZ;
                const double      power = static_cast<double>(x[at].real()) * static_cast<double>(x[at].real()) + static_cast<double>(x[at].imag()) * static_cast<double>(x[at].imag());
                caught += got.samples[at + 9UZ] == CF{} ? 1UZ : 0UZ;
                direct += power > block._threshold ? 1UZ : 0UZ;
            }
            const double measured = static_cast<double>(caught) / static_cast<double>(kImpulses);
            const double error    = std::sqrt(want * (1.0 - want) / static_cast<double>(kImpulses));
            std::println("INR {:.0f} dB: blanked {:.5f}, statistic over threshold {:.5f}, closed form {:.5f}", inrDb, measured, static_cast<double>(direct) / static_cast<double>(kImpulses), want);
            expect(lt(std::abs(measured - want), 4.0 * error)) << std::format("INR {} dB against the non-central chi-square", inrDb);
            expect(ge(caught, direct)) << "and every sample whose statistic exceeds the threshold is replaced";
        }
    };

    "interpolation beats holding beats zeroing, by the measured margins"_test = [] {
        constexpr std::size_t kLength    = 16384UZ;
        constexpr double      kBandwidth = 0.01;
        constexpr double      kIsr       = 1000.0; // 30 dB in power

        const std::vector<CF> clean = bandLimited(kLength, kBandwidth, 0x3f84d5b5b5470917ULL);
        const double          rms   = std::sqrt(meanPower(std::span<const CF>(clean)));

        std::vector<CF> dirty = clean;
        for (std::size_t at = 2048UZ; at < kLength; at += 4096UZ) {
            dirty[at] += CF(static_cast<float>(rms * std::sqrt(kIsr)), 0.f);
        }

        struct Arm {
            const char* rule;
            double      bound;
        };
        // The absolute residuals follow the generator, so what is asserted is the ordering of the three rules
        // and the 40 dB margin between the best of them and the worst.
        constexpr Arm       kArms[]{{"interpolate", -80.0}, {"hold", -55.0}, {"zero", -33.0}};
        std::vector<double> residual;

        for (const auto& [rule, bound] : kArms) {
            NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"averaging_time", 100000.0 / static_cast<double>(kRate)}, {"replacement", std::string(rule)}});
            block._warmup          = 0UZ;
            block._power           = rms * rms;

            const test::Result<CF> got = test::run(block, std::span<const CF>(dirty), 4096UZ);
            std::vector<CF>        error(kLength - 9UZ);
            for (std::size_t i = 0UZ; i < error.size(); ++i) {
                error[i] = got.samples[i + 9UZ] - clean[i];
            }
            const double db = 10.0 * std::log10(inBandEnergy(std::span<const CF>(error), kBandwidth) / inBandEnergy(std::span<const CF>(clean), kBandwidth));
            residual.push_back(db);
            std::println("{:<12} in-channel residual {:.2f} dB", rule, db);
            expect(lt(db, bound)) << std::format("{} leaves {:.2f} dB", rule, db);
        }
        expect(gt(residual[2UZ] - residual[0UZ], 40.0)) << "and interpolation beats zeroing by more than 40 dB";
    };

    "interpolation is exact on a straight line"_test = [] {
        constexpr std::size_t kLength = 400UZ;
        constexpr std::size_t kAt     = 200UZ;
        std::vector<CF>       x(kLength);
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            x[i] = CF(1.f + 0.01f * static_cast<float>(i), -2.f + 0.03f * static_cast<float>(i));
        }
        std::vector<CF> planted = x;
        planted[kAt] += CF(500.f, 0.f);

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}});
        block._warmup          = 0UZ;
        block._power           = 20.0;

        const test::Result<CF> got = test::run(block, std::span<const CF>(planted), 64UZ);
        for (std::size_t i = kAt - 2UZ; i <= kAt + 6UZ; ++i) {
            const CF     made     = got.samples[i + 9UZ];
            const double relative = std::abs(static_cast<double>(made.real()) - static_cast<double>(x[i].real())) / std::abs(static_cast<double>(x[i].real()));
            expect(lt(relative, 1e-6)) << std::format("input {}: interpolating across the gap reproduces the line exactly, so the (j+1)/(W+1) weights are right", i);
        }
    };

    "the replaced set is exactly the nine samples the window names"_test = [] {
        constexpr std::size_t kAt = 500UZ;
        std::vector<CF>       x   = tone(2000UZ, 1.0);
        x[kAt] += CF(200.f, 0.f);

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        block._warmup          = 0UZ;
        block._power           = 1.0;

        const test::Result<CF>   got = test::run(block, std::span<const CF>(x), 251UZ);
        std::vector<std::size_t> replaced;
        for (std::size_t i = 9UZ; i < got.samples.size(); ++i) {
            if (got.samples[i] == CF{}) {
                replaced.push_back(i - 9UZ); // back to the input offset it came from
            }
        }
        std::vector<std::size_t> want;
        for (std::size_t i = kAt - 2UZ; i <= kAt + 6UZ; ++i) {
            want.push_back(i);
        }
        expect(that % (replaced == want)) << "inputs q-2 .. q+6, nine of them, and no others";
        expect(eq(got.tags.size(), 1UZ)) << "one marker for the run";
        expect(eq(got.tags[0UZ].index, kAt - 2UZ + 9UZ)) << "at the run's first output sample";
    };

    "a retriggering window is bounded and reopens"_test = [] {
        constexpr std::size_t kLength = 2000UZ;
        constexpr std::size_t kStart  = 500UZ;
        constexpr std::size_t kBurst  = 200UZ;

        std::vector<CF> x = tone(kLength, 1.0);
        for (std::size_t i = kStart; i < kStart + kBurst; ++i) {
            x[i] += CF(200.f, 0.f);
        }
        x[1500UZ] += CF(200.f, 0.f); // one more, well after the burst

        NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"retrigger", true}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        block._warmup          = 0UZ;
        block._power           = 1.0;

        // Sample by sample, so the bound can be read at every point rather than inferred from the output.
        std::uint32_t highest = 0U;
        std::size_t   made    = 0UZ;
        for (std::size_t i = 0UZ; i < kLength; ++i) {
            made += test::run(block, std::span<const CF>(x).subspan(i, 1UZ), 1UZ).samples.size();
            highest = std::max(highest, block._windowLength);
        }
        expect(eq(made, kLength)) << "the block never stops producing";
        expect(eq(highest, 28U)) << "a retriggering window stops extending at exactly max_window_samples, 4 * blank_samples";

        NoiseBlanker<CF> again     = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"retrigger", true}, {"replacement", std::string("zero")}, {"emit_tags", true}});
        again._warmup              = 0UZ;
        again._power               = 1.0;
        const test::Result<CF> got = test::run(again, std::span<const CF>(x), 97UZ);
        expect(eq(got.tags.size(), 2UZ)) << "the burst is one replaced run and the later impulse opens a second: a closed window reopens on the next detection";

        std::size_t longest = 0UZ;
        std::size_t run     = 0UZ;
        for (std::size_t i = 9UZ; i < got.samples.size(); ++i) {
            run     = got.samples[i] == CF{} ? run + 1UZ : 0UZ;
            longest = std::max(longest, run);
        }
        expect(le(longest, kBurst + 9UZ)) << "and a sustained burst is replaced for as long as it lasts and no longer";
    };

    "the output does not depend on the chunking"_test = [] {
        std::vector<CF> x = noise(60000UZ, 0xba3bbe38fb4c55d9ULL);
        for (std::size_t at = 777UZ; at < x.size(); at += 1013UZ) {
            x[at] += CF(60.f, 20.f);
        }

        for (const char* rule : {"interpolate", "hold", "zero"}) {
            NoiseBlanker<CF>       reference = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}});
            const test::Result<CF> want      = test::run(reference, std::span<const CF>(x), 4096UZ);

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
                NoiseBlanker<CF> block = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"replacement", std::string(rule)}});
                expect(that % (test::run(block, std::span<const CF>(x), chunk).samples == want.samples)) << std::format("{}, chunk {}", rule, chunk);
            }
        }
    };

    "reserved keys ride through, the private one does not, and the block's own marker appears"_test = [] {
        struct Marker {
            const char*    key;
            std::size_t    at;
            gr::pmt::Value value;
        };
        const std::array<Marker, 7UZ> markers{{
            {"trigger_name", 0UZ, gr::pmt::Value(std::string("alpha"))},
            {"trigger_time", 1UZ, gr::pmt::Value(std::uint64_t{111})},
            {"trigger_offset", 1UZ, gr::pmt::Value(0.5f)},
            {"num_channels", 37UZ, gr::pmt::Value(gr::Size_t{3})},
            {"rx_overflow", 512UZ, gr::pmt::Value(true)},
            {"signal_name", 900UZ, gr::pmt::Value(std::string("nb"))},
            {"t0", 1200UZ, gr::pmt::Value(std::string("private"))},
        }};

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", 4000U}, {"mark_tag", false}});
        for (const Marker& marker : markers) {
            source._tags.emplace_back(marker.at, gr::property_map{{gr::property_map::key_type{marker.key}, marker.value}});
        }
        auto& block = graph.emplaceBlock<NoiseBlanker<CF>>({{"sample_rate", kRate}});
        auto& sink  = graph.emplaceBlock<TagSink<CF, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});
        expect(graph.connect<"out", "in">(source, block).has_value());
        expect(graph.connect<"out", "in">(block, sink).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value());
        expect(scheduler.runAndWait().has_value());

        constexpr std::size_t    kMissing = std::numeric_limits<std::size_t>::max();
        std::vector<std::size_t> offsets(markers.size(), kMissing);
        for (const gr::Tag& tag : sink._tags) {
            for (std::size_t which = 0UZ; which < markers.size(); ++which) {
                const auto found = tag.map.find(gr::property_map::key_type{markers[which].key});
                if (found != tag.map.end() && found->second == markers[which].value) {
                    offsets[which] = tag.index;
                }
            }
        }
        for (std::size_t which = 0UZ; which + 1UZ < markers.size(); ++which) {
            expect(eq(offsets[which], markers[which].at)) << std::format("{} rides through a 1:1 block unmoved, uncompensated for the delay", markers[which].key);
        }
        expect(eq(offsets[markers.size() - 1UZ], kMissing)) << "and the non-reserved key does not survive, which is the framework's rule and not this block's";
    };

    "degenerate settings"_test = [] {
        expect(throws([] { std::ignore = make<CF>({{"blank_samples", 0U}}); }));
        expect(throws([] { std::ignore = make<CF>({{"sample_rate", 0.f}}); }));
        expect(throws([] { std::ignore = make<CF>({{"sample_rate", -1.f}}); }));
        expect(throws([] { std::ignore = make<CF>({{"averaging_time", -1.0}}); }));
        expect(throws([] { std::ignore = make<CF>({{"replacement", std::string("taper")}}); })) << "'taper' is not one of the accepted replacement rules";

        NoiseBlanker<CF> block = make<CF>({{"sample_rate", kRate}});
        expect(throws([&block] { apply(block, {{"blank_samples", 0U}}); })) << "on a live change too";

        NoiseBlanker<CF>       silly = make<CF>({{"enabled", true}, {"sample_rate", kRate}, {"threshold_db", -60.0}});
        const std::vector<CF>  x     = noise(20000UZ, 0x21c66842f6e96c9aULL);
        const test::Result<CF> got   = test::run(silly, std::span<const CF>(x), 512UZ);
        expect(eq(got.samples.size(), x.size())) << "a threshold of -60 dB blanks essentially everything and the block still produces one output per input";

        NoiseBlanker<float>      real = make<float>({{"enabled", true}, {"sample_rate", kRate}});
        const std::vector<float> audio(5000UZ, 0.25f);
        expect(eq(test::run(real, std::span<const float>(audio), 333UZ).samples.size(), audio.size())) << "and the real port runs the same code on x^2";
    };

    "the per-sample cost stays inside the recorded budget"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock                   = std::chrono::steady_clock;
        constexpr std::size_t kLength = 1UZ << 22;
        constexpr std::size_t kChunk  = 4096UZ;
        constexpr int         kRuns   = 7;

        const std::vector<CF> x = noise(kLength, 0x24a19947b3916cf7ULL);
        std::vector<CF>       y(kLength);

        NoiseBlanker<CF> full = make<CF>({{"enabled", true}, {"sample_rate", kRate}});

        double best      = 1e300;
        double worst     = 0.0;
        double floorBest = 1e300;
        for (int repeat = 0; repeat <= kRuns; ++repeat) {
            const auto floorStart = Clock::now();
            std::copy_n(x.begin(), kLength, y.begin());
            const double floorNs = std::chrono::duration<double, std::nano>(Clock::now() - floorStart).count() / static_cast<double>(kLength);

            const auto start = Clock::now();
            for (std::size_t base = 0UZ; base < kLength; base += kChunk) {
                test::InputSpan<CF>  inSpan(std::span<const CF>(x).subspan(base, kChunk), base);
                test::OutputSpan<CF> outSpan(std::span<CF>(y).subspan(base, kChunk), base);
                std::ignore = full.processBulk(inSpan, outSpan);
            }
            const double ns = std::chrono::duration<double, std::nano>(Clock::now() - start).count() / static_cast<double>(kLength);
            if (repeat > 0) {
                best      = std::min(best, ns);
                worst     = std::max(worst, ns);
                floorBest = std::min(floorBest, floorNs);
            }
        }
        std::println("NoiseBlanker<complex<float>> {:.3f} ns/sample (spread {:.3f}), span copy {:.3f}", best, worst - best, floorBest);
        expect(lt(best, 9.0)) << std::format("the detector loop alone measures 3.94 at the baseline ISA and the whole block 7.8; this run reads {:.3f}", best);
    };
};

int main() { /* not needed for UT */ }
