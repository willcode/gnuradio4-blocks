#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <format>
#include <numeric>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/channel/NoisePower.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

namespace {

template<typename T>
[[nodiscard]] gr::blocks::channel::AwgnChannel<T> makeChannel(double noisePower, std::uint64_t seed) {
    gr::blocks::channel::AwgnChannel<T> block({{"noise_power", noisePower}, {"seed", seed}});
    boost::ut::expect(block.check({}).has_value());
    return block;
}

/// Runs a zero input through the block in `chunk`-sized calls, so the output IS the noise.
template<typename T>
[[nodiscard]] std::vector<T> noiseOnly(double noisePower, std::uint64_t seed, std::size_t nSamples, std::size_t chunk) {
    auto           block = makeChannel<T>(noisePower, seed);
    std::vector<T> zeros(nSamples, T{});
    std::vector<T> out(nSamples);
    for (std::size_t i = 0UZ; i < nSamples; i += chunk) {
        const std::size_t n = std::min(chunk, nSamples - i);
        std::ignore         = block.processBulk(std::span<const T>(zeros.data() + i, n), std::span<T>(out.data() + i, n));
    }
    return out;
}

[[nodiscard]] double meanPower(std::span<const float> x) {
    return std::transform_reduce(x.begin(), x.end(), 0.0, std::plus{}, [](float v) { return static_cast<double>(v) * static_cast<double>(v); }) / static_cast<double>(x.size());
}

[[nodiscard]] double meanPower(std::span<const std::complex<float>> x) {
    return std::transform_reduce(x.begin(), x.end(), 0.0, std::plus{}, [](std::complex<float> v) { return static_cast<double>(std::norm(v)); }) / static_cast<double>(x.size());
}

} // namespace

const boost::ut::suite<"AwgnChannel"> awgnTests = [] {
    using namespace boost::ut;
    using gr::blocks::channel::AwgnChannel;
    using gr::channel::noisePowerFor;
    using C = std::complex<float>;

    constexpr std::size_t kLong = 1'000'000UZ;

    // the setting is the mean power of what is added, and means the same for both sample types
    "measured noise power matches the setting"_test = [] {
        for (const double power : {0.01, 0.25, 1.0, 4.0}) {
            const auto real    = noiseOnly<float>(power, 7ULL, kLong, kLong);
            const auto complex = noiseOnly<C>(power, 7ULL, kLong, kLong);
            expect(lt(std::abs(meanPower(real) / power - 1.0), 0.01)) << std::format("real: wanted {}, measured {:g}", power, meanPower(real));
            expect(lt(std::abs(meanPower(complex) / power - 1.0), 0.01)) << std::format("complex: wanted {}, measured {:g}", power, meanPower(complex));
        }
    };

    // Option B means the two quadratures are independent, not one variate reused
    "complex quadratures are uncorrelated"_test = [] {
        const auto noise = noiseOnly<C>(1.0, 11ULL, kLong, kLong);
        double     sumII = 0., sumQQ = 0., sumIQ = 0.;
        for (const C v : noise) {
            const double re = static_cast<double>(v.real());
            const double im = static_cast<double>(v.imag());
            sumII += re * re;
            sumQQ += im * im;
            sumIQ += re * im;
        }
        const double rho = sumIQ / std::sqrt(sumII * sumQQ);
        expect(lt(std::abs(rho), 0.01)) << std::format("quadrature correlation {:g}", rho);
        // and each quadrature carries half the power, which is what makes E[|n|^2] the setting
        expect(lt(std::abs(sumII / sumQQ - 1.0), 0.02)) << "the quadratures must carry equal power";
    };

    // every seeded model in this family rests on this
    "output is bit-identical however the stream is split"_test = [] {
        constexpr std::size_t nSamples = 20'000UZ;
        const auto            realRef  = noiseOnly<float>(0.5, 99ULL, nSamples, nSamples);
        const auto            cplxRef  = noiseOnly<C>(0.5, 99ULL, nSamples, nSamples);
        for (const std::size_t chunk : {1UZ, 2UZ, 3UZ, 7UZ, 64UZ, 4095UZ}) {
            expect(noiseOnly<float>(0.5, 99ULL, nSamples, chunk) == realRef) << std::format("real, {}-sample calls", chunk);
            expect(noiseOnly<C>(0.5, 99ULL, nSamples, chunk) == cplxRef) << std::format("complex, {}-sample calls", chunk);
        }
    };

    // a disabled impairment must not perturb the stream at all, negative zero included
    "noise_power = 0 is bit-exact passthrough"_test = [] {
        constexpr std::size_t nSamples = 1024UZ;
        std::vector<C>        input(nSamples);
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            input[k] = C(static_cast<float>(std::cos(0.01 * static_cast<double>(k))), static_cast<float>(std::sin(0.01 * static_cast<double>(k))));
        }
        input[0] = C(-0.0f, -0.0f); // the case an add-zero would silently change

        auto           block = makeChannel<C>(0.0, 3ULL);
        std::vector<C> out(nSamples);
        std::ignore = block.processBulk(std::span<const C>(input), std::span<C>(out));

        expect(out == input) << "a zero-power channel must copy, not add";
        expect(eq(std::signbit(out[0].real()), true)) << "negative zero must survive";
        expect(eq(std::signbit(out[0].imag()), true)) << "negative zero must survive";
    };

    "a different seed is a different realization, the same seed reproduces"_test = [] {
        const auto a = noiseOnly<C>(1.0, 1ULL, 4096UZ, 4096UZ);
        const auto b = noiseOnly<C>(1.0, 1ULL, 4096UZ, 4096UZ);
        const auto c = noiseOnly<C>(1.0, 2ULL, 4096UZ, 4096UZ);
        expect(a == b) << "same seed must reproduce exactly";
        expect(a != c) << "a different seed must give a different realization";
    };

    "noise adds to the signal rather than replacing it"_test = [] {
        constexpr std::size_t nSamples = 100'000UZ;
        constexpr double      power    = 0.1;
        std::vector<C>        input(nSamples, C(1.f, 0.f));
        auto                  block = makeChannel<C>(power, 5ULL);
        std::vector<C>        out(nSamples);
        std::ignore = block.processBulk(std::span<const C>(input), std::span<C>(out));

        // E[|s + n|^2] = |s|^2 + noise_power for independent zero-mean noise
        expect(lt(std::abs(meanPower(out) / (1.0 + power) - 1.0), 0.02)) << std::format("measured {:g}, expected {:g}", meanPower(out), 1.0 + power);
    };

    // a running graph refuses a change of a StoppedOnly setting at the call
    "seed takes a change only while the graph is stopped"_test = [] {
        static_assert(decltype(gr::blocks::channel::AwgnChannel<C>::Settings::seed)::stoppedOnly(), "a mid-stream reseed would be a new realization");
        static_assert(!decltype(gr::blocks::channel::AwgnChannel<C>::Settings::noise_power)::stoppedOnly(), "noise power stays live-settable");
    };

    "noise power stays live-settable, and moves without disturbing the draw sequence"_test = [] {
        auto block = makeChannel<C>(1.0, 4ULL);
        block.start();
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(block), {{"noise_power", 2.0}}).empty()) << "noise power must stay live-settable";

        // the amplitude change must scale the same realization rather than restart it: one block run at
        // power 4 matches another run at power 1 scaled by two, sample for sample
        auto reference = makeChannel<C>(4.0, 4ULL);
        auto scaled    = makeChannel<C>(1.0, 4ULL);
        reference.start();
        scaled.start();
        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(scaled), {{"noise_power", 4.0}}).empty());

        constexpr std::size_t nSamples = 4096UZ;
        std::vector<C>        zeros(nSamples, C{});
        std::vector<C>        a(nSamples);
        std::vector<C>        b(nSamples);
        std::ignore = reference.processBulk(std::span<const C>(zeros), std::span<C>(a));
        std::ignore = scaled.processBulk(std::span<const C>(zeros), std::span<C>(b));
        expect(a == b) << "an amplitude change must not reseed";
    };

    "negative noise power is refused"_test = [] { expect(gr::testing::initRefused<gr::blocks::channel::AwgnChannel<float>>({{"noise_power", -1.0}, {"seed", std::uint64_t{0U}}})); };

    "noisePowerFor states the literature's operating point"_test = [] {
        // 0 dB Es/N0 at unit symbol energy and one sample per symbol is unit noise power
        expect(approx(noisePowerFor(0.f), 1.f, 1e-6f));
        expect(approx(noisePowerFor(10.f), 0.1f, 1e-6f));
        // noise added at 4x oversampling spreads over four samples per symbol
        expect(approx(noisePowerFor(10.f, 1.f, 4.f), 0.4f, 1e-6f));
    };
};

const boost::ut::suite<"AwgnChannel settings at the initialization"> awgnInitTests = [] {
    using namespace boost::ut;

    "AwgnChannel initializes at its defaults and with each setting off its default"_test = [] {
        const std::vector<std::string> failures = gr::testing::initFailures<gr::blocks::channel::AwgnChannel<std::complex<float>>>({
            {"noise_power", 0.5},
            {"seed", std::uint64_t{3U}},
        });
        expect(failures.empty()) << std::format("{}", failures);
    };
};

int main() { /* not needed for UT */ }
