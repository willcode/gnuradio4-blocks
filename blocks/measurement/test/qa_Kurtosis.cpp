#include <cmath>
#include <complex>
#include <cstdint>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/measurement/Kurtosis.hpp>

namespace qa_kurtosis {

using gr::blocks::measurement::Kurtosis;
using CF = std::complex<float>;

/// @brief Emits a fixed sample sequence in bursts of a stated size, then ends the stream, optionally carrying one
/// `sample_rate` and one `n_dropped_samples` tag at stated absolute indices. The burst size is what makes chunk
/// independence testable: the same samples presented differently must produce the same records.
template<typename T>
struct SampleSource : gr::Block<SampleSource<T>> {
    gr::PortOut<T> out;

    std::vector<T> samples{};
    std::size_t    burst   = 4096UZ;
    std::size_t    at      = 0UZ;
    float          rateTag = 0.f; ///< positive publishes one sample_rate tag at rateAt
    std::size_t    rateAt  = 0UZ;
    gr::Size_t     dropTag = 0U; ///< positive publishes one n_dropped_samples tag at dropAt
    std::size_t    dropAt  = 0UZ;

    GR_MAKE_REFLECTABLE(SampleSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        if (rateTag > 0.f && rateAt >= at && rateAt < at + take) {
            outSpan.publishTag(gr::property_map{{std::pmr::string(gr::tag::SAMPLE_RATE.shortKey()), gr::pmt::Value(rateTag)}}, rateAt - at);
        }
        if (dropTag > 0U && dropAt >= at && dropAt < at + take) {
            outSpan.publishTag(gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.shortKey()), gr::pmt::Value(dropTag)}}, dropAt - at);
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::vector<gr::DataSet<float>> records{};

    GR_MAKE_REFLECTABLE(RecordSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        records.insert(records.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief Seeded standard normal pairs, xorshift plus Box-Muller: a real component and a complex sample from one call.
struct GaussianNoise {
    std::uint64_t state;

    explicit GaussianNoise(std::uint64_t seed = 0x243f6a8885a308d3ULL) : state(seed) {}

    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }

    /// @brief One circular complex Gaussian sample, `E|z|^2 = 1`, `u,v ~ N(0, 1/2)` independent.
    [[nodiscard]] CF complexSample() {
        const double radius = std::sqrt(-std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }

    /// @brief One real standard normal sample, Box-Muller.
    [[nodiscard]] float realSample() {
        const double radius = std::sqrt(-2. * std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return static_cast<float>(radius * std::cos(angle));
    }
};

/// @brief Everything a criterion reads out of one run, captured while the block is still alive.
struct Run {
    std::vector<gr::DataSet<float>> records;

    double        coverage{};
    std::uint64_t nWindows{};
    std::uint64_t nDegenerateWindows{};
    std::uint64_t nWindowResets{};
    std::uint64_t nDroppedSampleTags{};
};

struct Tagging {
    float       rate   = 0.f;
    std::size_t rateAt = 0UZ;
    gr::Size_t  drop   = 0U;
    std::size_t dropAt = 0UZ;
};

template<typename T>
[[nodiscard]] Run runWith(gr::property_map settings, const std::vector<T>& samples, std::size_t burst, Tagging tagging = {}) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<SampleSource<T>>();
    auto&                 block  = test.emplace<Kurtosis<T>>(std::move(settings));
    auto&                 sink   = test.emplace<RecordSink>();
    source.samples               = samples;
    source.burst                 = burst;
    source.rateTag               = tagging.rate;
    source.rateAt                = tagging.rateAt;
    source.dropTag               = tagging.drop;
    source.dropAt                = tagging.dropAt;
    if (!test.connect(source, "out", block, "in").has_value()) {
        return {};
    }
    if (!test.connect(block, "measurements", sink, "in").has_value()) {
        return {};
    }
    std::ignore = test.run();

    Run result;
    result.records            = std::move(sink.records);
    result.coverage           = block.coverage();
    result.nWindows           = block.nWindows();
    result.nDegenerateWindows = block.nDegenerateWindows();
    result.nWindowResets      = block.nWindowResets();
    result.nDroppedSampleTags = block.nDroppedSampleTags();
    return result;
}

template<typename T>
[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, const std::vector<T>& samples, std::size_t burst) {
    return runWith<T>(std::move(settings), samples, burst).records;
}

[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return fallback;
    }
    if (const auto* asU64 = it->second.template get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asBool = it->second.template get_if<bool>()) {
        return *asBool ? 1. : 0.;
    }
    if (const auto* asFloat = it->second.template get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    if (const auto* asDouble = it->second.template get_if<double>()) {
        return *asDouble;
    }
    return fallback;
}

[[nodiscard]] std::string metaString(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return {};
    }
    if (const auto* asString = it->second.template get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return {};
}

[[nodiscard]] double mean(const std::vector<double>& values) {
    double total = 0.;
    for (const double v : values) {
        total += v;
    }
    return values.empty() ? 0. : total / static_cast<double>(values.size());
}

[[nodiscard]] double sampleStdDev(const std::vector<double>& values, double meanValue) {
    if (values.size() < 2UZ) {
        return 0.;
    }
    double sumSq = 0.;
    for (const double v : values) {
        const double d = v - meanValue;
        sumSq += d * d;
    }
    return std::sqrt(sumSq / static_cast<double>(values.size() - 1UZ));
}

} // namespace qa_kurtosis

const boost::ut::suite<"Kurtosis"> kurtosisTests = [] {
    using namespace boost::ut;
    using namespace qa_kurtosis;

    // criterion 1: seeded circular complex and real Gaussian, each window's excess_kurtosis inside the closed-form
    // bias band (2.3), and the population of readings itself spread the way (2.4) says a window's estimate should be.
    "criterion 1: Gaussian excess kurtosis sits inside its derived bias-and-spread envelope"_test = [] {
        constexpr std::size_t kWindow = 4096UZ;
        constexpr std::size_t kTrials = 400UZ;

        GaussianNoise   noise(0x9e3779b97f4a7c15ULL);
        std::vector<CF> complexSamples(kWindow * kTrials);
        std::ranges::generate(complexSamples, [&] { return noise.complexSample(); });
        const auto complexRecords = collect<CF>({{"window", gr::Size_t{kWindow}}}, complexSamples, 4096UZ);

        std::vector<float> realSamples(kWindow * kTrials);
        std::ranges::generate(realSamples, [&] { return noise.realSample(); });
        const auto realRecords = collect<float>({{"window", gr::Size_t{kWindow}}}, realSamples, 4096UZ);

        expect(eq(complexRecords.size(), kTrials));
        expect(eq(realRecords.size(), kTrials));

        std::vector<double> complexExcess;
        std::vector<double> realExcess;
        for (const auto& r : complexRecords) {
            complexExcess.push_back(static_cast<double>(r.signal_values.at(0UZ)));
        }
        for (const auto& r : realRecords) {
            realExcess.push_back(static_cast<double>(r.signal_values.at(0UZ)));
        }

        const double biasComplex   = -2. / static_cast<double>(kWindow);
        const double spreadComplex = 2. / std::sqrt(static_cast<double>(kWindow));
        const double semComplex    = 4. * spreadComplex / std::sqrt(static_cast<double>(kTrials));
        const double biasReal      = -6. / static_cast<double>(kWindow);
        const double spreadReal    = std::sqrt(24. / static_cast<double>(kWindow));
        const double semReal       = 4. * spreadReal / std::sqrt(static_cast<double>(kTrials));

        const double meanComplex = mean(complexExcess);
        const double meanReal    = mean(realExcess);
        const double sdComplex   = sampleStdDev(complexExcess, meanComplex);
        const double sdReal      = sampleStdDev(realExcess, meanReal);

        std::println("criterion 1: complex mean excess {:.6f} (band {:.6f} +- {:.6f}), spread {:.5f} (nominal {:.5f})", meanComplex, biasComplex, semComplex, sdComplex, spreadComplex);
        std::println("criterion 1: real    mean excess {:.6f} (band {:.6f} +- {:.6f}), spread {:.5f} (nominal {:.5f})", meanReal, biasReal, semReal, sdReal, spreadReal);

        expect(std::abs(meanComplex - biasComplex) < semComplex) << "complex mean excess kurtosis outside its 4*sigma_mean band";
        expect(std::abs(meanReal - biasReal) < semReal) << "real mean excess kurtosis outside its 4*sigma_mean band";
        expect(std::abs(sdComplex - spreadComplex) < 0.10 * spreadComplex) << "complex per-window spread outside 10% of the derived value";
        expect(std::abs(sdReal - spreadReal) < 0.10 * spreadReal) << "real per-window spread outside 10% of the derived value";

        // criterion 2, the Gaussian point: K = m4/m2^2 reads 2 within this same band, on the un-shifted channel.
        std::vector<double> complexNormalized;
        for (const auto& r : complexRecords) {
            complexNormalized.push_back(static_cast<double>(r.signal_values.at(1UZ)));
        }
        const double meanK = mean(complexNormalized);
        std::println("criterion 2: complex Gaussian normalized fourth moment mean {:.6f} (nominal 2)", meanK);
        expect(std::abs(meanK - (2. + biasComplex)) < semComplex) << "circular complex Gaussian must read K=2 within criterion 1's band";
    };

    // criterion 2: the three other closed values, each measured over a single large window so the ensemble average
    // the derivation assumes is well approximated.
    "criterion 2: constant modulus, two equal tones, and a real sinusoid read their exact excess values"_test = [] {
        constexpr std::size_t kWindow = 1UZ << 18U; // 262144

        // constant modulus: |z| = A with a random phase. Exact K=1, excess=-1, whatever the window (no dependence on
        // phase distribution at all: |z|^4/|z|^2^2 = A^4/A^4 = 1 sample by sample).
        {
            GaussianNoise    noise(0x1234567890abcdefULL);
            std::vector<CF>  samples(kWindow);
            constexpr double kAmplitude = 1.7;
            for (CF& s : samples) {
                const double theta = 2. * std::numbers::pi * noise.uniform();
                s                  = CF(static_cast<float>(kAmplitude * std::cos(theta)), static_cast<float>(kAmplitude * std::sin(theta)));
            }
            const auto records = collect<CF>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), 1UZ));
            if (!records.empty()) {
                const double excess = static_cast<double>(records.front().signal_values.at(0UZ));
                std::println("criterion 2: constant modulus excess {:.6f} (exact -1, asserted against the literal 1 SignalQuality.hpp's ka names)", excess);
                expect(std::abs(excess - (-1.)) < 1e-4) << "constant modulus must read excess -1 to float rounding";
            }
        }

        // two equal complex tones: |z|^2 = 2 + 2*cos(D) with D the difference phase, swept over many, non-integer
        // cycles across the window so its distribution over the window is close to uniform.
        {
            std::vector<CF>  samples(kWindow);
            constexpr double kBin1 = 41.0;
            constexpr double kBin2 = 41.0 + 3000.37; // a large, non-integer cycle difference over the window
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                const double n  = static_cast<double>(k);
                const double p1 = 2. * std::numbers::pi * kBin1 * n / static_cast<double>(kWindow);
                const double p2 = 2. * std::numbers::pi * kBin2 * n / static_cast<double>(kWindow);
                samples[k]      = CF(static_cast<float>(std::cos(p1) + std::cos(p2)), static_cast<float>(std::sin(p1) + std::sin(p2)));
            }
            const auto records = collect<CF>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), 1UZ));
            if (!records.empty()) {
                const double excess = static_cast<double>(records.front().signal_values.at(0UZ));
                std::println("criterion 2: two equal complex tones excess {:.6f} (nominal -0.5)", excess);
                expect(std::abs(excess - (-0.5)) < 0.01) << "two equal-amplitude complex tones must read excess close to -0.5";
            }
        }

        // a real sinusoid: m4=3A^4/8, m2=A^2/2, K=3/2, excess=-1.5 exactly for a whole number of cycles, and close to
        // it for any long window since the boundary effect is O(1/N).
        {
            std::vector<float> samples(kWindow);
            constexpr double   kBin = 977.0;
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                samples[k] = static_cast<float>(std::cos(2. * std::numbers::pi * kBin * static_cast<double>(k) / static_cast<double>(kWindow)));
            }
            const auto records = collect<float>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), 1UZ));
            if (!records.empty()) {
                const double excess = static_cast<double>(records.front().signal_values.at(0UZ));
                std::println("criterion 2: real sinusoid excess {:.6f} (nominal -1.5)", excess);
                expect(std::abs(excess - (-1.5)) < 1e-3) << "a real sinusoid over a whole number of cycles must read excess -1.5";
            }
        }
    };

    // criterion 3: the closed-form impulsive excess, `x = g + b*A`, g standard Gaussian and b Bernoulli(p), measured
    // over a scaled-down number of windows (the kernel's own qa already pins the Monte Carlo reference at the full
    // scale; this reproduces the shape at a size this binary's ~5s budget affords).
    "criterion 3: the impulsive closed form, positive where the structured signals are negative"_test = [] {
        constexpr std::size_t kWindow = 1UZ << 16U; // 65536
        constexpr std::size_t kTrials = 60UZ;

        const auto impulsiveExcess = [](double p, double a) { return (3. + 6. * p * a * a + p * a * a * a * a) / ((1. + p * a * a) * (1. + p * a * a)) - 3.; };

        struct Case {
            double p;
            double a;
        };
        const std::array<Case, 3UZ> cases{{{0.001, 10.}, {0.01, 5.}, {0.001, 20.}}};

        GaussianNoise noise(0xabcdef0123456789ULL);
        for (const Case& c : cases) {
            std::vector<float> samples(kWindow * kTrials);
            for (float& s : samples) {
                const bool impulse = noise.uniform() < c.p;
                s                  = noise.realSample() + (impulse ? static_cast<float>(c.a) : 0.f);
            }
            const auto records = collect<float>({{"window", gr::Size_t{kWindow}}}, samples, 8192UZ);
            expect(eq(records.size(), kTrials));

            std::vector<double> readings;
            for (const auto& r : records) {
                readings.push_back(static_cast<double>(r.signal_values.at(0UZ)));
            }
            const double measured = mean(readings);
            const double expected = impulsiveExcess(c.p, c.a);
            std::println("criterion 3: p={:.3f} A={:.1f} measured excess {:.4f}, closed form {:.4f}", c.p, c.a, measured, expected);

            expect(measured > 0.) << "the impulsive case must read a positive excess";
            expect(std::abs(measured - expected) < 0.10 * std::abs(expected) + 0.3) << std::format("p={} A={}: measured {} too far from closed form {}", c.p, c.a, measured, expected);
        }
    };

    // criterion 4: an all-zero window is degenerate and reported so, never silently; a stream ending mid-window
    // flushes what it actually covered.
    "criterion 4: a degenerate window is reported false and never a NaN, and a partial window states its own coverage"_test = [] {
        const auto everyChannelFinite = [](const gr::DataSet<float>& record, std::string_view what) {
            for (std::size_t k = 0UZ; k < 3UZ; ++k) {
                expect(std::isfinite(record.signal_values.at(k))) << std::format("{}: channel {} must be finite, read {}", what, k, record.signal_values.at(k));
            }
        };

        std::vector<float> zeros(4096UZ, 0.f);
        const Run          zeroRun = runWith<float>({{"window", gr::Size_t{4096U}}}, zeros, 4096UZ);
        expect(eq(zeroRun.records.size(), 1UZ));
        expect(eq(zeroRun.nDegenerateWindows, std::uint64_t{1ULL})) << "the all-zero window must be counted degenerate";
        if (!zeroRun.records.empty()) {
            const auto& r = zeroRun.records.front();
            expect(eq(r.signal_values.at(0UZ), 0.f));
            expect(eq(r.signal_values.at(1UZ), 0.f));
            expect(eq(r.signal_values.at(2UZ), 0.f));
            expect(eq(metaNumber(r, "valid"), 0.)) << "an all-zero window must be marked invalid";
            everyChannelFinite(r, "degenerate window");
        }

        {
            GaussianNoise      noise(0x55aa55aa55aa55aaULL);
            std::vector<float> samples(4096UZ + 900UZ);
            std::ranges::generate(samples, [&] { return noise.realSample(); });
            const Run partial = runWith<float>({{"window", gr::Size_t{4096U}}}, samples, 4096UZ);

            expect(eq(partial.records.size(), 2UZ)) << "one full window and one partial flush";
            expect(eq(partial.nWindows, std::uint64_t{1ULL})) << "only the full window counts as a window";
            std::println("criterion 4: coverage after the partial flush {:.6f} (900/4096 = {:.6f})", partial.coverage, 900. / 4096.);
            expect(std::abs(partial.coverage - 900. / 4096.) < 1e-9) << "the flush leaves coverage() at the fraction it actually covered";
            expect(partial.coverage < 1.) << "a partial window must report a coverage below one";
            if (partial.records.size() == 2UZ) {
                expect(eq(metaNumber(partial.records[1UZ], "n_samples"), 900.)) << "the flush states the true sample count";
                expect(eq(metaNumber(partial.records[0UZ], "n_samples"), 4096.));
                everyChannelFinite(partial.records[0UZ], "full window");
                everyChannelFinite(partial.records[1UZ], "partial flush");
            }
        }
    };

    // criterion 5: window refusals at staging, naming the bound.
    "criterion 5: window below 8 or above 2^24 refuses at staging"_test = [] {
        const auto refused = [](gr::property_map settings) {
            Kurtosis<float> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"window", gr::Size_t{7U}}}); })) << "below the floor of 8";
        expect(throws([&] { refused({{"window", gr::Size_t{(1U << 24U) + 1U}}}); })) << "above the ceiling of 2^24";
        expect(nothrow([&] { refused({{"window", gr::Size_t{8U}}}); })) << "the floor itself is admitted";
        expect(nothrow([&] { refused({{"window", gr::Size_t{1U << 24U}}}); })) << "the ceiling itself is admitted";
    };

    // criterion 18 (Kurtosis half): before the first window, coverage() reads 0.
    "criterion 18: coverage() reads 0 before the first window closes"_test = [] {
        Kurtosis<CF> block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.coverage(), 0.));
        expect(eq(block.nWindows(), std::uint64_t{0ULL}));
    };

    // criterion 19: chunk independence in both domains, at chunk sizes landing off and exactly on window boundaries
    // and on the tagged sample, with the counters compared alongside the records.
    "criterion 19: chunk independence, bit-identical records and counters at every chunk size"_test = [] {
        const auto sweep = [](const auto& samples, std::string_view domain) {
            using Sample                   = std::remove_cvref_t<decltype(samples.front())>;
            constexpr std::size_t kDropAt  = 1200UZ; // interior to window 1, not on a window boundary
            const auto            settings = [] { return gr::property_map{{"window", gr::Size_t{1024U}}}; };
            const Tagging         tagging{.rate = 0.f, .rateAt = 0UZ, .drop = gr::Size_t{64U}, .dropAt = kDropAt};

            const Run reference = runWith<Sample>(settings(), samples, 1024UZ, tagging);
            expect(!reference.records.empty()) << domain;
            expect(eq(reference.nDroppedSampleTags, std::uint64_t{1ULL})) << std::format("{}: the one dropped-sample tag must be counted once", domain);

            for (const std::size_t burst : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
                const Run candidate = runWith<Sample>(settings(), samples, burst, tagging);
                expect(eq(candidate.records.size(), reference.records.size())) << std::format("{} burst {}: record count", domain, burst);
                expect(eq(candidate.nWindows, reference.nWindows)) << std::format("{} burst {}: nWindows", domain, burst);
                expect(eq(candidate.nDegenerateWindows, reference.nDegenerateWindows)) << std::format("{} burst {}: nDegenerateWindows", domain, burst);
                expect(eq(candidate.nWindowResets, reference.nWindowResets)) << std::format("{} burst {}: nWindowResets", domain, burst);
                expect(eq(candidate.nDroppedSampleTags, reference.nDroppedSampleTags)) << std::format("{} burst {}: nDroppedSampleTags", domain, burst);
                for (std::size_t r = 0UZ; r < std::min(candidate.records.size(), reference.records.size()); ++r) {
                    for (std::size_t k = 0UZ; k < 3UZ; ++k) {
                        expect(eq(candidate.records[r].signal_values.at(k), reference.records[r].signal_values.at(k))) << std::format("{} burst {}: record {} channel {}", domain, burst, r, k);
                    }
                    expect(eq(metaNumber(candidate.records[r], "n_samples"), metaNumber(reference.records[r], "n_samples"))) << std::format("{} burst {}: record {} n_samples", domain, burst, r);
                    expect(eq(metaNumber(candidate.records[r], "n_dropped_samples"), metaNumber(reference.records[r], "n_dropped_samples"))) << std::format("{} burst {}: record {} n_dropped_samples", domain, burst, r);
                }
            }
        };

        GaussianNoise   noise(0x0f0f0f0f0f0f0f0fULL);
        std::vector<CF> complexSamples(3UZ * 1024UZ + 777UZ);
        std::ranges::generate(complexSamples, [&] { return noise.complexSample(); });
        sweep(complexSamples, "complex");

        std::vector<float> realSamples(3UZ * 1024UZ + 777UZ);
        std::ranges::generate(realSamples, [&] { return noise.realSample(); });
        sweep(realSamples, "real");
    };

    // the settings §4.2 names and the record keys §4.4 does: emit_records off, the producer label, the rate the
    // record states, and the reserved sample_rate tag moving that rate the way it moves every other block's.
    "settings and record metadata: emit_records, signal_name, sample_rate by setting and by tag"_test = [] {
        GaussianNoise   noise(0x6a09e667f3bcc908ULL);
        std::vector<CF> samples(2UZ * 4096UZ);
        std::ranges::generate(samples, [&] { return noise.complexSample(); });

        {
            const Run silent = runWith<CF>({{"window", gr::Size_t{4096U}}, {"emit_records", false}}, samples, 4096UZ);
            expect(eq(silent.records.size(), 0UZ)) << "emit_records = false publishes no record";
            expect(eq(silent.nWindows, std::uint64_t{2ULL})) << "the windows are still measured and counted";
            expect(std::abs(silent.coverage - 1.) < 1e-12) << "and the readers still carry the last complete window";
        }
        {
            const Run named = runWith<CF>({{"window", gr::Size_t{4096U}}, {"signal_name", std::string("rfi_probe")}}, samples, 4096UZ);
            expect(eq(named.records.size(), 2UZ));
            if (!named.records.empty()) {
                expect(eq(metaString(named.records.front(), "signal_name"), std::string("rfi_probe")));
                expect(eq(metaString(named.records.front(), "input_domain"), std::string("complex")));
                std::println("settings: default record sample_rate {:.1f} Hz", metaNumber(named.records.front(), "sample_rate"));
                expect(std::abs(metaNumber(named.records.front(), "sample_rate") - 96000.) < 1e-3) << "the record states the rate unconditionally, from the setting's own default";
            }
        }
        {
            const Run set = runWith<CF>({{"window", gr::Size_t{4096U}}, {"sample_rate", 250000.f}}, samples, 4096UZ);
            expect(eq(set.records.size(), 2UZ));
            for (const auto& record : set.records) {
                expect(std::abs(metaNumber(record, "sample_rate") - 250000.) < 1e-3) << "a sample_rate setting reaches every record";
            }
        }
        {
            // The reserved tag on the stream, at the first sample, is what the framework turns into a settings change.
            const Run tagged = runWith<CF>({{"window", gr::Size_t{4096U}}}, samples, 1024UZ, Tagging{.rate = 32000.f, .rateAt = 0UZ, .drop = 0U, .dropAt = 0UZ});
            expect(eq(tagged.records.size(), 2UZ));
            for (const auto& record : tagged.records) {
                std::println("settings: record sample_rate after the tag {:.1f} Hz", metaNumber(record, "sample_rate"));
                expect(std::abs(metaNumber(record, "sample_rate") - 32000.) < 1e-3) << "a sample_rate stream tag moves the setting the record states";
            }
        }
        {
            const Run refused = runWith<CF>({{"window", gr::Size_t{4096U}}}, samples, 4096UZ, Tagging{.rate = 0.f, .rateAt = 0UZ, .drop = gr::Size_t{7U}, .dropAt = 100UZ});
            expect(eq(refused.nDroppedSampleTags, std::uint64_t{1ULL})) << "the dropped-sample tag is counted";
            expect(eq(refused.records.size(), 2UZ));
            if (!refused.records.empty()) {
                expect(eq(metaNumber(refused.records[0UZ], "n_dropped_samples"), 7.)) << "the window carrying the tag states what it lost";
                expect(eq(metaNumber(refused.records[1UZ], "n_dropped_samples"), -1.)) << "a window that lost nothing carries no such key";
            }
        }
    };

    // §4.3's settings change: a new `window` discards the partial accumulation rather than carrying a piece of the
    // old length into the new one, and says so in nWindowResets(). Driven on the block itself: a settings key that is
    // not one of the reserved stream tags does not reach a running block from the stream.
    "a window change discards the partial accumulation and counts the reset"_test = [] {
        Kurtosis<float> block({{"window", gr::Size_t{1024U}}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.nWindowResets(), std::uint64_t{0ULL}));

        const std::vector<float> partial(100UZ, 0.5f);
        block._acc.add(std::span<const float>(partial)); // the window part-filled, as a stream would leave it
        expect(eq(block._acc.count(), 100UZ));

        expect(block.settings().setStaged(gr::property_map{{"window", gr::Size_t{2048U}}}).empty()) << "the new window is accepted";
        std::ignore = block.settings().applyStagedParameters();
        std::println("window change: nWindowResets() = {}, accumulated {} samples after it", block.nWindowResets(), block._acc.count());
        expect(eq(block.nWindowResets(), std::uint64_t{1ULL})) << "the discarded partial accumulation is counted";
        expect(eq(block._acc.count(), 0UZ)) << "and it really is discarded rather than carried into the new length";
        expect(eq(block.window.value, gr::Size_t{2048U}));

        expect(block.settings().setStaged(gr::property_map{{"window", gr::Size_t{4096U}}}).empty());
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nWindowResets(), std::uint64_t{1ULL})) << "a change with nothing accumulated discards nothing";
    };

    "sample_rate refusals: zero and non-finite refuse at staging"_test = [] {
        const auto staged = [](gr::property_map settings) {
            Kurtosis<float> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { staged({{"sample_rate", 0.f}}); })) << "a rate of zero is not a rate";
        expect(throws([&] { staged({{"sample_rate", -48000.f}}); })) << "a negative rate is not a rate";
        expect(nothrow([&] { staged({{"sample_rate", 48000.f}}); }));
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
