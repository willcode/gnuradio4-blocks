#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/measurement/PolarizationCombiner.hpp>

namespace qa_polcomb {

using gr::blocks::measurement::PolarizationCombiner;
using CF = std::complex<float>;

template<typename T>
struct SampleSource : gr::Block<SampleSource<T>> {
    gr::PortOut<T> out;

    std::vector<T> samples{};
    std::size_t    burst = 4096UZ;
    std::size_t    at    = 0UZ;

    GR_MAKE_REFLECTABLE(SampleSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

template<typename T>
struct Collector : gr::Block<Collector<T>> {
    gr::PortIn<T> in;

    std::vector<T> items{};

    GR_MAKE_REFLECTABLE(Collector, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        items.insert(items.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
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

struct Rng {
    std::uint64_t state;
    explicit Rng(std::uint64_t seed = 0xbf58476d1ce4e5b9ULL) : state(seed) {}
    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }
    [[nodiscard]] CF complexSample() {
        const double radius = std::sqrt(-std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }
};

struct Branches {
    std::vector<CF> r0;
    std::vector<CF> r1;
    std::vector<CF> s; ///< the transmitted signal, when the scene needs it back
};

/// @brief `r_i = h_i * s + n_i`, with `s` and both noises independent circular complex Gaussian; `noisePower` per
/// branch (equal). `signalPower` scales `s` before the channel.
[[nodiscard]] Branches twoBranch(Rng& rng, std::size_t n, CF h0, CF h1, double signalPower, double noisePower) {
    Branches b;
    b.r0.resize(n);
    b.r1.resize(n);
    b.s.resize(n);
    const float sigS = static_cast<float>(std::sqrt(signalPower));
    const float sigN = static_cast<float>(std::sqrt(noisePower));
    for (std::size_t k = 0UZ; k < n; ++k) {
        const CF s  = sigS * rng.complexSample();
        const CF n0 = sigN * rng.complexSample();
        const CF n1 = sigN * rng.complexSample();
        b.s[k]      = s;
        b.r0[k]     = h0 * s + n0;
        b.r1[k]     = h1 * s + n1;
    }
    return b;
}

/// @brief Everything a criterion needs out of one run, captured while the block is still alive (the `RuntimeTest` it
/// lives in is local to `run()`, so nothing here may be a pointer into it).
struct Run {
    std::vector<CF>                 out;
    std::vector<CF>                 ortho;
    std::vector<gr::DataSet<float>> records;

    double relativePhase{};
    double amplitudeRatio{};
    double branch0Db{};
    double branch1Db{};
    double combinedDb{};
    double combiningGainDb{};
    int    selectedBranch{};
    double coverage{};

    std::uint64_t nWindows{};
    std::uint64_t nWindowResets{};
    std::uint64_t nBranchSwitches{};
    std::uint64_t nSaturatedFigures{};
    std::uint64_t nDroppedSampleTags{};

    bool endedOnItsOwn{true};

    std::complex<double> weight0{}, weight1{}, ortho0{}, ortho1{}; ///< the weights in force when the run ended
};

[[nodiscard]] Run run(gr::property_map settings, const std::vector<CF>& r0, const std::vector<CF>& r1, std::size_t burst, bool connectOrtho = true) {
    gr::test::RuntimeTest test;
    auto&                 src0  = test.emplace<SampleSource<CF>>();
    auto&                 src1  = test.emplace<SampleSource<CF>>();
    auto&                 block = test.emplace<PolarizationCombiner>(std::move(settings));
    auto&                 outS  = test.emplace<Collector<CF>>();
    auto&                 recS  = test.emplace<RecordSink>();
    // A collector with nothing wired to its input would not stage, so the scene without an `ortho` consumer has no
    // collector at all rather than an idle one.
    Collector<CF>* orthoS = connectOrtho ? &test.emplace<Collector<CF>>() : nullptr;

    src0.samples = r0;
    src0.burst   = burst;
    src1.samples = r1;
    src1.burst   = burst;

    std::ignore = test.connect(src0, "out", block, "in0");
    std::ignore = test.connect(src1, "out", block, "in1");
    std::ignore = test.connect(block, "out", outS, "in");
    if (orthoS != nullptr) {
        std::ignore = test.connect(block, "ortho", *orthoS, "in");
    }
    std::ignore = test.connect(block, "measurements", recS, "in");
    // A bounded deadline rather than an unbounded wait: a graph that fails to end fails an assertion instead of
    // hanging the binary. Every scene here is a few tens of thousands of samples.
    const bool inTime = test.runWithin(std::chrono::seconds(20));

    Run result;
    result.endedOnItsOwn      = inTime;
    result.out                = std::move(outS.items);
    result.ortho              = orthoS != nullptr ? std::move(orthoS->items) : std::vector<CF>{};
    result.records            = std::move(recS.records);
    result.relativePhase      = block.relativePhase();
    result.amplitudeRatio     = block.amplitudeRatio();
    result.branch0Db          = block.branchSnrDb(0UZ);
    result.branch1Db          = block.branchSnrDb(1UZ);
    result.combinedDb         = block.combinedSnrDb();
    result.combiningGainDb    = block.combiningGainDb();
    result.selectedBranch     = block.selectedBranch();
    result.coverage           = block.coverage();
    result.nWindows           = block.nWindows();
    result.nWindowResets      = block.nWindowResets();
    result.nBranchSwitches    = block.nBranchSwitches();
    result.nSaturatedFigures  = block.nSaturatedFigures();
    result.nDroppedSampleTags = block.nDroppedSampleTags();
    result.weight0            = static_cast<std::complex<double>>(block._estimate.weight0);
    result.weight1            = static_cast<std::complex<double>>(block._estimate.weight1);
    result.ortho0             = static_cast<std::complex<double>>(block._estimate.ortho0);
    result.ortho1             = static_cast<std::complex<double>>(block._estimate.ortho1);
    return result;
}

[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto it = record.meta_information[0UZ].find(std::pmr::string(key));
    if (it == record.meta_information[0UZ].end()) {
        return fallback;
    }
    if (const auto* asI32 = it->second.template get_if<std::int32_t>()) {
        return static_cast<double>(*asI32);
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
    return fallback;
}

[[nodiscard]] std::string metaString(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto it = record.meta_information[0UZ].find(std::pmr::string(key));
    if (it == record.meta_information[0UZ].end()) {
        return {};
    }
    if (const auto* asString = it->second.template get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return {};
}

/// @brief The gain onto a known reference and the residual power beside it, both measured over one range.
struct Projection {
    std::complex<double> gain{};          ///< the least-squares complex gain of @p reference in @p measured
    double               signalPower{};   ///< the power that gain accounts for
    double               residualPower{}; ///< what is left, which under (7.1) is the noise
};

[[nodiscard]] Projection project(std::span<const CF> measured, std::span<const CF> reference, std::size_t from, std::size_t to) {
    std::complex<double> cross{};
    double               referencePower = 0.;
    for (std::size_t k = from; k < to; ++k) {
        const std::complex<double> r = static_cast<std::complex<double>>(reference[k]);
        cross += std::conj(r) * static_cast<std::complex<double>>(measured[k]);
        referencePower += std::norm(r);
    }
    Projection out;
    out.gain = referencePower > 0. ? cross / referencePower : std::complex<double>{};

    double residual = 0.;
    for (std::size_t k = from; k < to; ++k) {
        residual += std::norm(static_cast<std::complex<double>>(measured[k]) - out.gain * static_cast<std::complex<double>>(reference[k]));
    }
    const double n    = static_cast<double>(to - from);
    out.signalPower   = n > 0. ? std::norm(out.gain) * referencePower / n : 0.;
    out.residualPower = n > 0. ? residual / n : 0.;
    return out;
}

/// @brief The signal-to-noise ratio of @p measured against the known transmitted @p reference, in dB.
[[nodiscard]] double measuredSnrDb(std::span<const CF> measured, std::span<const CF> reference, std::size_t from, std::size_t to) {
    const Projection p = project(measured, reference, from, to);
    return (p.residualPower > 0.) ? 10. * std::log10(p.signalPower / p.residualPower) : std::numeric_limits<double>::infinity();
}

} // namespace qa_polcomb

const boost::ut::suite<"PolarizationCombiner"> polarizationCombinerTests = [] {
    using namespace boost::ut;
    using namespace qa_polcomb;

    // criterion 12: MRC gain over a branch, at equal branches (3.0103 dB, within 0.05 dB) and at -6 dB relative
    // (0.9691 dB), against the derived envelope: weight-estimation loss under 0.011 dB at M>=512, plus the
    // SNR-measurement spread 4.343/sqrt(N) dB, which at N = 65536 is 0.017 dB.
    "criterion 12: MRC gain over a branch, inside its derived envelope"_test = [] {
        constexpr std::size_t kN      = 65536UZ;
        constexpr gr::Size_t  kWindow = 8192U; // shorter than the stream, so `out` is really combined and not passed through
        constexpr std::size_t kFrom   = static_cast<std::size_t>(kWindow);
        Rng                   rng(0x243f6a8885a308d3ULL);

        // The measurement is made on the streams: the output's signal-to-noise ratio against the transmitted signal,
        // over the windows after the first (window 0 is the startup passthrough), against the better branch's over
        // the same range. The envelope is the spec's two named contributions — the weight-estimation loss, under
        // 0.011 dB at M >= 512, and the ratio-measurement spread 4.343/sqrt(N), which over 57344 samples is
        // 0.018 dB.
        const auto arm = [kWindow, kFrom](const Branches& b, double nominalGainDb, std::string_view what) {
            const Run r = run({{"window", kWindow}}, b.r0, b.r1, 8192UZ);
            expect(r.endedOnItsOwn) << what;
            expect(eq(r.out.size(), b.s.size())) << what;
            if (r.out.size() != b.s.size()) {
                return;
            }
            const double snrOut     = measuredSnrDb(r.out, b.s, kFrom, b.s.size());
            const double snrBranch0 = measuredSnrDb(b.r0, b.s, kFrom, b.s.size());
            const double snrBranch1 = measuredSnrDb(b.r1, b.s, kFrom, b.s.size());
            const double best       = std::max(snrBranch0, snrBranch1);
            const double gain       = snrOut - best;

            std::println("criterion 12 ({}): measured out {:.4f} dB, branches {:.4f} / {:.4f} dB, gain {:.4f} dB (nominal {:.4f}); block reports {:.4f} dB", what, snrOut, snrBranch0, snrBranch1, gain, nominalGainDb, r.combiningGainDb);
            expect(std::isfinite(snrOut) && std::isfinite(best)) << std::format("{}: the measured ratios must be finite", what);
            expect(std::abs(gain - nominalGainDb) < 0.05) << std::format("{}: the realized output gain is outside its 0.05 dB envelope", what);
            expect(std::abs(r.combiningGainDb - nominalGainDb) < 0.05) << std::format("{}: the reported gain is outside its 0.05 dB envelope", what);
        };

        arm(twoBranch(rng, kN, CF{1.f, 0.f}, CF(static_cast<float>(std::cos(0.7)), static_cast<float>(std::sin(0.7))), 1., 0.1), 3.0103, "equal branches");
        arm(twoBranch(rng, kN, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.))), 1., 0.1), 0.9691, "-6 dB relative branch");
    };

    // criterion 13: the estimator's internal identity, on arbitrary data with no assumed model: the two branch SNRs
    // sum in linear terms to the combined SNR, to 1e-9 relative — the algebraic identity (7.6) states.
    "criterion 13: branchSnrDb(0) and branchSnrDb(1) sum in linear terms to combinedSnrDb()"_test = [] {
        Rng             rng(0x9e3779b97f4a7c15ULL);
        std::vector<CF> r0(8192UZ);
        std::vector<CF> r1(8192UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{8192U}}}, r0, r1, 4096UZ);

        // The identity is only a test if the figures it relates are real numbers: an eigenvalue swap that drives a
        // ratio to zero reads -inf dB, which the block now reports as the finite placeholder 0, and 1 + 1 == ... is
        // not the identity. So each figure is asserted finite and strictly positive before it is used.
        std::println("criterion 13: branch0 {:.6f} dB, branch1 {:.6f} dB, combined {:.6f} dB", r.branch0Db, r.branch1Db, r.combinedDb);
        expect(std::isfinite(r.branch0Db) && std::isfinite(r.branch1Db) && std::isfinite(r.combinedDb)) << "no figure may be an infinity";
        expect(r.combinedDb > r.branch0Db && r.combinedDb > r.branch1Db) << "the combined figure must exceed either branch's";
        if (!r.records.empty()) {
            expect(eq(metaNumber(r.records.back(), "valid"), 1.)) << "and a window of ordinary data is a valid record";
        }

        const double linear0  = std::pow(10., r.branch0Db / 10.);
        const double linear1  = std::pow(10., r.branch1Db / 10.);
        const double sum      = linear0 + linear1;
        const double combined = std::pow(10., r.combinedDb / 10.);
        std::println("criterion 13: branch0 {:.6f} + branch1 {:.6f} = {:.6f}, combined {:.6f}", linear0, linear1, sum, combined);
        expect(linear0 > 0. && linear1 > 0. && combined > 0.) << "each ratio must be a positive number before the identity means anything";
        expect(std::abs(sum - combined) < 1e-9 * std::max(1., combined)) << "the additive identity must hold to 1e-9 relative on arbitrary data";
    };

    // criterion 14: the estimate recovers the channel, within the covariance's own relative standard error.
    "criterion 14: relativePhase and amplitudeRatio recover the channel, within 4/sqrt(M)"_test = [] {
        constexpr std::size_t kM = 262144UZ;
        Rng                   rng(0x1234567890abcdefULL);
        const Branches        b = twoBranch(rng, kM, CF{1.f, 0.f}, 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.))), 1., 0.1);

        const Run    r         = run({{"window", gr::Size_t{static_cast<gr::Size_t>(kM)}}}, b.r0, b.r1, 8192UZ);
        const double tolerance = 4. / std::sqrt(static_cast<double>(kM));
        std::println("criterion 14: relativePhase {:.5f} (nominal -2, tol {:.4f}), amplitudeRatio {:.5f} (nominal 2)", r.relativePhase, tolerance, r.amplitudeRatio);
        expect(std::abs(r.relativePhase - (-2.)) < tolerance) << "relativePhase must recover arg(h0)-arg(h1) = -2 within the covariance's own standard error";
        expect(std::abs(r.amplitudeRatio - 2.) < tolerance) << "amplitudeRatio must recover |h0|/|h1| = 2 within the same tolerance";
    };

    // criterion 15: the orthogonal output nulls the transmitted signal, below -40 dB of what `out` carries of it, and
    // the orthogonal weight vector nulls the *true* channel `h` the scene injected — which `v^H v_perp == 0` cannot
    // show, since `v_perp` is built from `v` and their inner product is zero however wrong `v` is.
    "criterion 15: the orthogonal output nulls, on the correlation and on the injected channel"_test = [] {
        constexpr std::size_t kWindow  = 8192UZ;
        constexpr std::size_t kWindows = 3UZ;
        Rng                   rng(0xd1b54a32d192ed03ULL);
        const CF              h0 = CF{1.f, 0.f};
        const CF              h1 = 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.)));
        const Branches        b  = twoBranch(rng, kWindow * kWindows, h0, h1, 1., 0.02);

        const Run r = run({{"window", gr::Size_t{kWindow}}}, b.r0, b.r1, 4096UZ);
        expect(eq(r.out.size(), b.s.size()));
        expect(eq(r.ortho.size(), b.s.size()));
        if (r.out.size() != b.s.size() || r.ortho.size() != b.s.size()) {
            return;
        }

        // window 0 is the passthrough startup; windows 1 and 2 carry weights solved from a real (stationary) channel.
        std::complex<double> corrOut{};
        std::complex<double> corrOrtho{};
        for (std::size_t k = kWindow; k < kWindow * kWindows; ++k) {
            corrOut += std::conj(static_cast<std::complex<double>>(b.s[k])) * static_cast<std::complex<double>>(r.out[k]);
            corrOrtho += std::conj(static_cast<std::complex<double>>(b.s[k])) * static_cast<std::complex<double>>(r.ortho[k]);
        }
        const double ratioDb = 20. * std::log10(std::abs(corrOrtho) / std::abs(corrOut));
        std::println("criterion 15: |corr(s,ortho)| / |corr(s,out)| = {:.2f} dB (must be below -40)", ratioDb);
        expect(ratioDb < -40.) << "the orthogonal output must carry the transmitted signal at least 40 dB below what out carries";

        // The channel the scene injected, which the block never saw: the orthogonal weights must annihilate it and
        // the combining weights must not. An estimate that has drifted away from the true polarization fails this
        // even though `v^H v_perp` would still be zero.
        const std::complex<double> trueH0       = static_cast<std::complex<double>>(h0);
        const std::complex<double> trueH1       = static_cast<std::complex<double>>(h1);
        const double               throughOut   = std::abs(std::conj(r.weight0) * trueH0 + std::conj(r.weight1) * trueH1);
        const double               throughOrtho = std::abs(std::conj(r.ortho0) * trueH0 + std::conj(r.ortho1) * trueH1);
        const double               nullDb       = 20. * std::log10(throughOrtho / throughOut);
        std::println("criterion 15: |u^H h| / |w^H h| = {:.2f} dB (out gain {:.5f}, ortho gain {:.3e})", nullDb, throughOut, throughOrtho);
        expect(throughOut > 0.5) << "the combining weights must pass the injected channel";
        expect(nullDb < -40.) << "the orthogonal weights must annihilate the channel the scene injected";
    };

    // criterion 16: selection mode follows the stronger branch across exactly one crossover; mrc on the same stream
    // reads a positive gain throughout.
    "criterion 16: selection mode switches exactly once at a branch crossover; mrc stays positive throughout"_test = [] {
        constexpr std::size_t kWindow  = 2048UZ;
        constexpr std::size_t kWindows = 4UZ;
        Rng                   rng(0x2545f4914f6cdd1dULL);

        std::vector<CF> r0(kWindow * kWindows);
        std::vector<CF> r1(kWindow * kWindows);
        for (std::size_t w = 0UZ; w < kWindows; ++w) {
            const bool branch0Louder = w < 2UZ;
            const CF   h0            = branch0Louder ? CF{2.f, 0.f} : CF{0.4f, 0.f};
            const CF   h1            = branch0Louder ? CF{0.4f, 0.f} : CF{2.f, 0.f};
            for (std::size_t k = 0UZ; k < kWindow; ++k) {
                const CF s          = rng.complexSample();
                const CF n0         = 0.2f * rng.complexSample();
                const CF n1         = 0.2f * rng.complexSample();
                r0[w * kWindow + k] = h0 * s + n0;
                r1[w * kWindow + k] = h1 * s + n1;
            }
        }

        const Run selRun = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("selection")}}, r0, r1, 4096UZ);
        std::println("criterion 16: selection nBranchSwitches() = {}", selRun.nBranchSwitches);
        expect(eq(selRun.nBranchSwitches, std::uint64_t{1ULL})) << "the branch power crosses over exactly once";

        const Run mrcRun = run({{"window", gr::Size_t{kWindow}}, {"mode", std::string("mrc")}, {"emit_records", true}}, r0, r1, 4096UZ);
        expect(ge(mrcRun.records.size(), 3UZ)) << "at least the three post-startup windows must have produced records";
        for (std::size_t i = 1UZ; i < mrcRun.records.size(); ++i) { // record 0 describes window 0's own solve; still a valid combine
            const float gainDb = mrcRun.records[i].signal_values.at(5UZ);
            std::println("criterion 16: mrc record {} combining_gain_db = {:.3f}", i, gainDb);
            expect(gainDb > 0.f) << std::format("record {}: mrc must show a positive gain over the stronger branch throughout", i);
        }
    };

    // criterion 17: saturation is counted, never silent, at both stated bounds.
    "criterion 17: saturation is counted at both bounds, and a degenerate ratio leaves branch 0 unchanged"_test = [] {
        // (a) a noiseless two-branch input: no noise in either branch, so lambda_- = 0 exactly (small integer
        // covariance entries, computed without rounding) and the SNR figures saturate at 60 dB / 1e6 linear.
        {
            constexpr std::size_t kWindow = 1024UZ;
            std::vector<CF>       r0(kWindow, CF{1.f, 0.f});
            std::vector<CF>       r1(kWindow, CF{2.f, 0.f});
            const Run             r = run({{"window", gr::Size_t{kWindow}}}, r0, r1, 4096UZ);
            std::println("criterion 17: noiseless branchSnrDb(0)={:.3f} combinedSnrDb={:.3f} nSaturatedFigures={}", r.branch0Db, r.combinedDb, r.nSaturatedFigures);
            expect(r.nSaturatedFigures > 0ULL) << "a noiseless input must saturate at least one figure";
            expect(std::abs(r.combinedDb - 60.) < 1e-6) << "the saturated figure must read exactly 60 dB (1e6 linear)";
        }

        // (b) branch 1 carries only independent structure (an exactly orthogonal deterministic sequence, so the
        // cross-covariance is exactly zero, not merely small): the amplitude ratio saturates and the weights
        // degenerate to branch 0 alone, checked by comparing `out` to branch 0 bit for bit.
        {
            constexpr std::size_t kWindow  = 1024UZ;
            constexpr std::size_t kWindows = 3UZ;
            std::vector<CF>       r0(kWindow * kWindows);
            std::vector<CF>       r1(kWindow * kWindows);
            for (std::size_t k = 0UZ; k < r0.size(); ++k) {
                r0[k] = (k % 2UZ == 0UZ) ? CF{1.f, 0.f} : CF{-1.f, 0.f};         // period 2
                r1[k] = ((k / 2UZ) % 2UZ == 0UZ) ? CF{1.f, 0.f} : CF{-1.f, 0.f}; // period 4, exactly orthogonal to r0 over any multiple of 4
            }
            const Run r = run({{"window", gr::Size_t{kWindow}}}, r0, r1, 4096UZ);
            expect(eq(r.out.size(), r0.size()));
            if (r.out.size() != r0.size()) {
                return;
            }
            std::println("criterion 17: orthogonal-branch-1 amplitudeRatio={:.3f} nSaturatedFigures={} branch0Db={:.3f} combiningGainDb={:.3f}", r.amplitudeRatio, r.nSaturatedFigures, r.branch0Db, r.combiningGainDb);
            expect(r.nSaturatedFigures > 0ULL) << "an exactly-uncorrelated branch 1 must saturate the amplitude ratio";
            expect(std::abs(r.amplitudeRatio - 1e6) < 1.) << "the saturated ratio must read the stated 1e6 linear bound";
            for (std::size_t k = kWindow; k < r0.size(); ++k) { // skip the startup window, which is passthrough by construction anyway
                expect(eq(r.out[k], r0[k])) << std::format("sample {}: with branch 1 uncorrelated, out must equal branch 0 bit for bit", k);
            }

            // This is also the scene in which every estimated signal power is exactly zero, so every decibel figure
            // would be an infinity and their difference a NaN. Nothing non-finite may reach a reader or a record.
            expect(std::isfinite(r.branch0Db) && std::isfinite(r.branch1Db) && std::isfinite(r.combinedDb)) << "the readers carry finite placeholders";
            expect(std::isfinite(r.combiningGainDb)) << "combiningGainDb() is a number, not a NaN";
            expect(!r.records.empty()) << "the windows produced records";
            for (std::size_t i = 0UZ; i < r.records.size(); ++i) {
                for (std::size_t c = 0UZ; c < 6UZ; ++c) {
                    expect(std::isfinite(r.records[i].signal_values.at(c))) << std::format("record {} channel {} must be finite, read {}", i, c, r.records[i].signal_values.at(c));
                }
                expect(eq(metaNumber(r.records[i], "valid"), 0.)) << std::format("record {}: a window whose figures had to be replaced is not valid", i);
            }
        }
    };

    // §7.6/§8: the record states the rate it was measured at, unconditionally, from the block's own setting.
    "the record states its sample_rate, and a non-positive one refuses at staging"_test = [] {
        Rng             rng(0x9e3779b97f4a7c15ULL);
        std::vector<CF> r0(4096UZ);
        std::vector<CF> r1(4096UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{2048U}}, {"sample_rate", 192000.f}}, r0, r1, 1024UZ);
        expect(!r.records.empty());
        for (const auto& record : r.records) {
            expect(std::abs(metaNumber(record, "sample_rate") - 192000.) < 1e-3) << "every record carries the key";
            expect(eq(metaString(record, "mode"), std::string("mrc")));
            expect(eq(metaString(record, "normalize"), std::string("unit_noise")));
            expect(eq(metaNumber(record, "window"), 2048.));
        }
    };

    "the settings refusals §7.4 names, each at staging"_test = [] {
        const auto staged = [](gr::property_map settings) {
            PolarizationCombiner block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { staged({{"window", gr::Size_t{63U}}}); })) << "below the floor of 64";
        expect(throws([&] { staged({{"window", gr::Size_t{(1U << 24U) + 1U}}}); })) << "above the ceiling of 2^24";
        expect(nothrow([&] { staged({{"window", gr::Size_t{64U}}}); })) << "the floor itself is admitted";
        expect(throws([&] { staged({{"mode", std::string("diversity")}}); })) << "mode takes two values and no others";
        expect(throws([&] { staged({{"normalize", std::string("unit_power")}}); })) << "normalize takes two values and no others";
        expect(throws([&] { staged({{"weight_smoothing", 1.0}}); })) << "weight_smoothing is [0, 1)";
        expect(throws([&] { staged({{"weight_smoothing", -0.1}}); }));
        expect(throws([&] { staged({{"noise_powers", std::vector<double>{1.}}}); })) << "two branches take two noise powers";
        expect(throws([&] { staged({{"noise_powers", std::vector<double>{1., 0.}}}); })) << "a noise power of zero is not a noise power";
        expect(nothrow([&] { staged({{"noise_powers", std::vector<double>{0.1, 0.5}}}); }));
        expect(throws([&] { staged({{"sample_rate", 0.f}}); })) << "a rate of zero is not a rate";
    };

    // §7.5's normalizations are two exact scalars, and they are measurably different on the same stream: unit_noise
    // keeps ||v|| = 1 so the output signal power is P|h|^2, unit_signal scales it to exactly 1.
    "normalize: unit_noise and unit_signal scale the output differently, each to its stated value"_test = [] {
        constexpr std::size_t kWindow  = 4096UZ;
        constexpr std::size_t kWindows = 4UZ;
        Rng                   rng(0x1234567890abcdefULL);
        const CF              h1 = 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.)));
        const Branches        b  = twoBranch(rng, kWindow * kWindows, CF{1.f, 0.f}, h1, 1., 0.05);

        const auto outputSignalPower = [&b](std::string_view normalize) {
            const Run r = run({{"window", gr::Size_t{kWindow}}, {"normalize", std::string(normalize)}}, b.r0, b.r1, 4096UZ);
            expect(eq(r.out.size(), b.s.size())) << normalize;
            return r.out.size() == b.s.size() ? project(r.out, b.s, kWindow, b.s.size()).signalPower : 0.;
        };

        const double noisePower  = outputSignalPower("unit_noise");
        const double signalPower = outputSignalPower("unit_signal");
        std::println("normalize: unit_noise output signal power {:.5f} (nominal |h|^2 = 1.25), unit_signal {:.5f} (nominal 1)", noisePower, signalPower);
        // The envelope is the measurement's own: the mean |s|^2 the projection divides by has a relative spread of
        // 1/sqrt(12288) = 0.9 % over the range measured, and the weights come from the window before each.
        expect(std::abs(noisePower - 1.25) < 0.05) << "unit_noise keeps ||v|| = 1, so the output signal power is P|h|^2 = 1.25";
        expect(std::abs(signalPower - 1.) < 0.05) << "unit_signal scales the output's signal power to 1";
        expect(std::abs(noisePower - signalPower) > 0.1) << "the two normalizations must really differ on this stream";
    };

    // §7.5's smoothing, under the unequal branch noise §7.2's whitening is for: the one-pole blend runs on the
    // whitened weights, so the orthogonal channel it rebuilds still nulls. Blending the unwhitened weights instead
    // leaves an orthogonal vector that carries the signal at a level this catches.
    "weight_smoothing under unequal branch noise still nulls the orthogonal channel"_test = [] {
        constexpr std::size_t kWindow  = 4096UZ;
        constexpr std::size_t kWindows = 5UZ;
        constexpr double      kNoise0  = 0.02;
        constexpr double      kNoise1  = 0.20; // a 10:1 imbalance, which is what makes whitening visible

        Rng             rng(0x2545f4914f6cdd1dULL);
        const CF        h0 = CF{1.f, 0.f};
        const CF        h1 = 0.5f * CF(static_cast<float>(std::cos(2.)), static_cast<float>(std::sin(2.)));
        std::vector<CF> r0(kWindow * kWindows);
        std::vector<CF> r1(kWindow * kWindows);
        std::vector<CF> s(kWindow * kWindows);
        for (std::size_t k = 0UZ; k < s.size(); ++k) {
            const CF sample = rng.complexSample();
            s[k]            = sample;
            r0[k]           = h0 * sample + static_cast<float>(std::sqrt(kNoise0)) * rng.complexSample();
            r1[k]           = h1 * sample + static_cast<float>(std::sqrt(kNoise1)) * rng.complexSample();
        }

        const Run r = run({{"window", gr::Size_t{kWindow}}, {"noise_powers", std::vector<double>{kNoise0, kNoise1}}, {"weight_smoothing", 0.5}}, r0, r1, 4096UZ);
        expect(r.endedOnItsOwn);
        expect(eq(r.out.size(), s.size()));
        if (r.out.size() != s.size()) {
            return;
        }

        // measured over the windows whose weights came through the blend (windows 2 onward)
        const double throughOut   = std::abs(project(r.out, s, 2UZ * kWindow, s.size()).gain);
        const double throughOrtho = std::abs(project(r.ortho, s, 2UZ * kWindow, s.size()).gain);
        const double nullDb       = 20. * std::log10(throughOrtho / throughOut);
        std::println("weight_smoothing: out gain {:.5f}, ortho gain {:.3e}, null {:.2f} dB", throughOut, throughOrtho, nullDb);
        expect(throughOut > 0.5) << "the combined output must still carry the signal";
        expect(nullDb < -30.) << "the smoothed orthogonal channel must still null the signal under unequal branch noise";

        // and the whitened combination is worth what §7.2 says it is: SNR_0/N_0 + SNR_1/N_1 rather than the
        // equal-noise answer. 1/0.02 + 0.25/0.2 = 51.25 linear, 17.10 dB.
        const double snrOut = measuredSnrDb(r.out, s, 2UZ * kWindow, s.size());
        std::println("weight_smoothing: measured output SNR {:.3f} dB (optimum 10*log10(1/0.02 + 0.25/0.2) = {:.3f} dB)", snrOut, 10. * std::log10(1. / kNoise0 + 0.25 / kNoise1));
        expect(std::abs(snrOut - 10. * std::log10(1. / kNoise0 + 0.25 / kNoise1)) < 0.3) << "the whitened combination must reach the unequal-noise optimum";
    };

    "emit_records = false measures without publishing"_test = [] {
        Rng             rng(0x5deece66dULL);
        std::vector<CF> r0(8192UZ);
        std::vector<CF> r1(8192UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{2048U}}, {"emit_records", false}}, r0, r1, 4096UZ);
        expect(eq(r.records.size(), 0UZ)) << "no record is published";
        expect(eq(r.nWindows, std::uint64_t{4ULL})) << "but the windows were measured and counted";
        expect(std::abs(r.coverage - 1.) < 1e-12) << "and the readers carry the last complete window";
        expect(eq(r.out.size(), r0.size())) << "the stream is unaffected";
    };

    // §7.3's `ortho` is optional in both directions: nothing connected to it, and the arithmetic switched off with
    // something connected. Neither may stall the combined output.
    "emit_orthogonal and the ortho connection each gate the second output without stalling the first"_test = [] {
        Rng             rng(0xbf58476d1ce4e5b9ULL);
        std::vector<CF> r0(8192UZ);
        std::vector<CF> r1(8192UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        {
            const Run r = run({{"window", gr::Size_t{2048U}}}, r0, r1, 4096UZ, false);
            expect(r.endedOnItsOwn) << "nothing connected to ortho must not stall the graph";
            expect(eq(r.out.size(), r0.size())) << "the combined output is complete with ortho unconnected";
            expect(eq(r.nWindows, std::uint64_t{4ULL}));
        }
        {
            const Run r = run({{"window", gr::Size_t{2048U}}, {"emit_orthogonal", false}}, r0, r1, 4096UZ, true);
            expect(r.endedOnItsOwn) << "emit_orthogonal = false with ortho connected must not stall the graph";
            expect(eq(r.out.size(), r0.size())) << "the combined output is complete";
            expect(eq(r.ortho.size(), 0UZ)) << "and the orthogonal port publishes nothing, as the setting says";
            expect(eq(r.nWindows, std::uint64_t{4ULL}));
        }
    };

    // criterion 18 (PolarizationCombiner half): before the first window, out/ortho pass branches 0/1 through
    // unchanged, and every estimate reads 0.
    "criterion 18: startup passes branch 0 and branch 1 through unchanged, and coverage() reads 0"_test = [] {
        // the readers, checked before any sample is processed at all: a run through a complete RuntimeTest graph
        // always reaches end of stream, and PolarizationCombiner's own end-of-stream flush (spec's own "the partial
        // window is published as a final record") updates coverage() and the estimates by the time the graph
        // returns — so the "before the first window" reading is checked on a freshly started block directly.
        PolarizationCombiner block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.coverage(), 0.));
        expect(eq(block.relativePhase(), 0.));
        expect(eq(block.amplitudeRatio(), 0.));
        expect(eq(block.nWindows(), std::uint64_t{0ULL}));

        // the passthrough itself, over an actual stream shorter than one window (so it never closes and the
        // end-of-stream flush the paragraph above works around never has real weights to publish over what streamed).
        Rng             rng(0x0f0f0f0f0f0f0f0fULL);
        std::vector<CF> r0(2000UZ);
        std::vector<CF> r1(2000UZ);
        std::ranges::generate(r0, [&] { return rng.complexSample(); });
        std::ranges::generate(r1, [&] { return rng.complexSample(); });

        const Run r = run({{"window", gr::Size_t{4096U}}}, r0, r1, 777UZ);
        expect(eq(r.out.size(), r0.size()));
        expect(eq(r.ortho.size(), r1.size()));
        for (std::size_t k = 0UZ; k < r0.size(); ++k) {
            expect(eq(r.out[k], r0[k])) << std::format("sample {}: out must equal branch 0 bit for bit before the first window closes", k);
            expect(eq(r.ortho[k], r1[k])) << std::format("sample {}: ortho must equal branch 1 bit for bit before the first window closes", k);
        }
    };

    // criterion 19: chunk independence, bit-identical out/ortho streams, records and counters at every chunk size.
    "criterion 19: chunk independence, bit-identical for every chunk size"_test = [] {
        Rng            rng(0x5deece66dULL);
        const Branches b         = twoBranch(rng, 3UZ * 1024UZ + 777UZ, CF{1.f, 0.f}, 0.6f * CF(static_cast<float>(std::cos(1.1)), static_cast<float>(std::sin(1.1))), 1., 0.15);
        const auto     settings  = gr::property_map{{"window", gr::Size_t{1024U}}, {"emit_records", true}};
        const Run      reference = run(settings, b.r0, b.r1, 1024UZ);

        for (const std::size_t burst : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            const Run candidate = run(settings, b.r0, b.r1, burst);
            expect(eq(candidate.out.size(), reference.out.size())) << std::format("burst {}: out size", burst);
            expect(std::ranges::equal(candidate.out, reference.out)) << std::format("burst {}: out stream differs", burst);
            expect(std::ranges::equal(candidate.ortho, reference.ortho)) << std::format("burst {}: ortho stream differs", burst);
            expect(eq(candidate.nWindows, reference.nWindows)) << std::format("burst {}: nWindows", burst);
            expect(eq(candidate.nWindowResets, reference.nWindowResets)) << std::format("burst {}: nWindowResets", burst);
            expect(eq(candidate.nBranchSwitches, reference.nBranchSwitches)) << std::format("burst {}: nBranchSwitches", burst);
            expect(eq(candidate.nSaturatedFigures, reference.nSaturatedFigures)) << std::format("burst {}: nSaturatedFigures", burst);
            expect(eq(candidate.nDroppedSampleTags, reference.nDroppedSampleTags)) << std::format("burst {}: nDroppedSampleTags", burst);
            expect(eq(candidate.combiningGainDb, reference.combiningGainDb)) << std::format("burst {}: combiningGainDb", burst);
            expect(eq(candidate.records.size(), reference.records.size())) << std::format("burst {}: record count", burst);
            for (std::size_t r = 0UZ; r < std::min(candidate.records.size(), reference.records.size()); ++r) {
                expect(std::ranges::equal(candidate.records[r].signal_values, reference.records[r].signal_values)) << std::format("burst {}: record {}", burst, r);
                expect(eq(metaNumber(candidate.records[r], "selected_branch"), metaNumber(reference.records[r], "selected_branch"))) << std::format("burst {}: record {}", burst, r);
            }
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
