#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <map>
#include <numbers>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/measurement/PhaseUnwrap.hpp>

namespace qa_unwrap {

using gr::blocks::measurement::PhaseUnwrap;
using CF = std::complex<float>;

/// @brief A complex tone at a stated cycles-per-sample rate and starting phase, in bursts of a stated size,
/// optionally carrying one `n_dropped_samples` tag at a stated absolute sample index.
struct ToneSource : gr::Block<ToneSource> {
    gr::PortOut<CF> out;

    double      cyclesPerSample = 0.1;
    double      phaseOffset     = 0.;
    std::size_t total           = 10000UZ;
    std::size_t burst           = 4096UZ;
    bool        tagDrop         = false;
    std::size_t dropAt          = 0UZ;
    std::size_t at              = 0UZ;

    GR_MAKE_REFLECTABLE(ToneSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= total) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, total - at, outSpan.size()});
        for (std::size_t k = 0UZ; k < take; ++k) {
            const double phase = phaseOffset + 2. * std::numbers::pi * cyclesPerSample * static_cast<double>(at + k);
            outSpan[k]         = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }
        if (tagDrop && dropAt >= at && dropAt < at + take) {
            outSpan.publishTag(gr::property_map{{std::pmr::string(gr::tag::N_DROPPED_SAMPLES.shortKey()), gr::pmt::Value(gr::Size_t{100U})}}, dropAt - at);
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

template<typename T>
struct Collector : gr::Block<Collector<T>> {
    gr::PortIn<T> in;

    std::vector<T>                            items{};
    std::map<std::uint64_t, gr::property_map> tags{};

    GR_MAKE_REFLECTABLE(Collector, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            tags.insert({static_cast<std::uint64_t>(tag.index), tag.map});
        }
        items.insert(items.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

struct Run {
    std::vector<std::int64_t> cycles;
    std::vector<float>        phase;

    std::map<std::uint64_t, gr::property_map> cycleTags;
    std::map<std::uint64_t, gr::property_map> phaseTags;

    std::uint64_t nSuspectSteps{};
    std::uint64_t nResets{};
    std::uint64_t nSaturations{};
    std::int64_t  lastSuspectIndex{-1};
};

struct Scene {
    double      cyclesPerSample = 0.1;
    double      phaseOffset     = 0.;
    std::size_t total           = 10000UZ;
    std::size_t burst           = 4096UZ;
    bool        tagDrop         = false;
    std::size_t dropAt          = 0UZ;
};

[[nodiscard]] Run runScene(gr::property_map settings, const Scene& scene) {
    gr::test::RuntimeTest test;
    auto&                 source     = test.emplace<ToneSource>();
    auto&                 block      = test.emplace<PhaseUnwrap>(std::move(settings));
    auto&                 cyclesSink = test.emplace<Collector<std::int64_t>>();
    auto&                 phaseSink  = test.emplace<Collector<float>>();

    source.cyclesPerSample = scene.cyclesPerSample;
    source.phaseOffset     = scene.phaseOffset;
    source.total           = scene.total;
    source.burst           = scene.burst;
    source.tagDrop         = scene.tagDrop;
    source.dropAt          = scene.dropAt;

    std::ignore = test.connect(source, "out", block, "in");
    std::ignore = test.connect(block, "cycles", cyclesSink, "in");
    std::ignore = test.connect(block, "phase", phaseSink, "in");
    std::ignore = test.run();

    Run result;
    result.cycles           = std::move(cyclesSink.items);
    result.phase            = std::move(phaseSink.items);
    result.cycleTags        = std::move(cyclesSink.tags);
    result.phaseTags        = std::move(phaseSink.tags);
    result.nSuspectSteps    = block.nSuspectSteps();
    result.nResets          = block.nResets();
    result.nSaturations     = block.nSaturations();
    result.lastSuspectIndex = block.lastSuspectIndex().has_value() ? static_cast<std::int64_t>(*block.lastSuspectIndex()) : -1LL;
    return result;
}

[[nodiscard]] Run run(gr::property_map settings, double cyclesPerSample, std::size_t total, std::size_t burst, bool tagDrop = false, std::size_t dropAt = 0UZ) { return runScene(std::move(settings), Scene{.cyclesPerSample = cyclesPerSample, .phaseOffset = 0., .total = total, .burst = burst, .tagDrop = tagDrop, .dropAt = dropAt}); }

[[nodiscard]] std::string tagString(const std::map<std::uint64_t, gr::property_map>& tags, std::uint64_t index, std::string_view key) {
    const auto at = tags.find(index);
    if (at == tags.end()) {
        return "<no tag>";
    }
    const auto it = at->second.find(std::pmr::string(key));
    if (it == at->second.end()) {
        return "<no key>";
    }
    if (const auto* asString = it->second.get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return "<not a string>";
}

/// @brief Wrap to `[-pi, pi)`.
[[nodiscard]] double wrap(double radians) {
    double out = std::fmod(radians + std::numbers::pi, 2. * std::numbers::pi);
    if (out < 0.) {
        out += 2. * std::numbers::pi;
    }
    return out - std::numbers::pi;
}

} // namespace qa_unwrap

const boost::ut::suite<"PhaseUnwrap"> phaseUnwrapTests = [] {
    using namespace boost::ut;
    using namespace qa_unwrap;

    // criterion 9: exact at a scale a float accumulator has already failed at (the kernel qa's own pinned scene:
    // 5,242,881 samples at 0.4 cycles/sample give 2,097,152 cycles exactly). The full 10^9-cycle scene this criterion
    // names takes on the order of 2*10^9 samples to reach — tens of seconds even at the kernel's own ns/sample bench
    // — so, as the kernel qa's own `ENABLE_LONG_TESTS` arm records, it is not part of the default ~5s run; this
    // reproduces the same demonstration at the scale the kernel qa already measured and pinned.
    "criterion 9: exact where a float accumulator has already stopped working"_test = [] {
        constexpr double      kCyclesPerSample = 0.4;
        constexpr std::size_t kTotal           = 5'242'881UZ;

        const Run run_ = run(gr::property_map{}, kCyclesPerSample, kTotal, 4096UZ);
        expect(eq(run_.cycles.size(), kTotal));
        if (run_.cycles.empty()) {
            return;
        }

        const std::int64_t measuredCycles = run_.cycles.back();
        const float        measuredPhase  = run_.phase.back();
        std::println("criterion 9: measured cycles()={} phase()={:.8f}", measuredCycles, measuredPhase);
        expect(eq(measuredCycles, std::int64_t{2'097'152LL})) << "the kernel qa's own pinned cycle count at this scene";

        const double trueRadians  = 2. * std::numbers::pi * kCyclesPerSample * static_cast<double>(kTotal - 1UZ);
        const double trueResidual = wrap(trueRadians);
        std::println("criterion 9: analytic residual {:.8f}, block residual {:.8f}", trueResidual, static_cast<double>(measuredPhase));
        expect(std::abs(static_cast<double>(measuredPhase) - trueResidual) < 4. * 2.38e-7) << "phase() must sit within 4 * a float's own pi-spacing of the analytic residual";

        const double unwrappedRadians = 2. * std::numbers::pi * static_cast<double>(measuredCycles) + static_cast<double>(measuredPhase);
        expect(std::abs(unwrappedRadians - trueRadians) < 1e-3) << "cycles and phase together must reconstruct the true unwrapped phase";

        // A float accumulator over the same ramp, run alongside: wrong by more than one whole cycle, its spacing
        // printed rather than asserted (that number is a fact about IEEE754 at this magnitude, not a block contract).
        float       floatAccumulator  = 0.f;
        double      doubleAccumulator = 0.;
        const float step              = static_cast<float>(2. * std::numbers::pi * kCyclesPerSample);
        for (std::size_t k = 1UZ; k < kTotal; ++k) {
            floatAccumulator += step;
            doubleAccumulator += 2. * std::numbers::pi * kCyclesPerSample;
        }
        const double floatError  = std::abs(static_cast<double>(floatAccumulator) - trueRadians);
        const double doubleError = std::abs(doubleAccumulator - trueRadians);
        const float  spacing     = std::nextafter(floatAccumulator, floatAccumulator * 2.f) - floatAccumulator;
        std::println("criterion 9: float accumulator off by {:.3f} rad ({:.1f} cycles), final ULP spacing {:.3f} rad; double accumulator off by {:.3e} rad (not asserted)", floatError, floatError / (2. * std::numbers::pi), spacing, doubleError);
        expect(floatError > 2. * std::numbers::pi) << "a float accumulator over this ramp must be wrong by more than one whole cycle";
    };

    // criterion 10: correctness below Nyquist, the aliased result at or above it, and (per F14) the suspect-step
    // hook's own threshold crossing, which the derivation places at 0.47 cycles/sample rather than at 0.6.
    "criterion 10: unwraps exactly below Nyquist, aliases at or above it, and the suspect hook fires below both"_test = [] {
        constexpr std::size_t kTotal = 1'000'000UZ;

        {
            const Run    run_        = run(gr::property_map{}, 0.4, kTotal, 8192UZ);
            const double trueRadians = 2. * std::numbers::pi * 0.4 * static_cast<double>(kTotal - 1UZ);
            const double unwrapped   = 2. * std::numbers::pi * static_cast<double>(run_.cycles.back()) + static_cast<double>(run_.phase.back());
            std::println("criterion 10: 0.4 fs unwrapped {:.3f}, analytic {:.3f}", unwrapped, trueRadians);
            expect(std::abs(unwrapped - trueRadians) < 1e-2) << "0.4 cycles/sample must unwrap exactly over 10^6 samples";
        }

        {
            // 0.6 cycles/sample aliases to -0.4: the wrapped step (1.2*pi wraps to -0.8*pi) is indistinguishable
            // from the one -0.4 cycles/sample produces, which is exactly the ambiguity the precondition names.
            const Run    run_         = run(gr::property_map{}, 0.6, kTotal, 8192UZ);
            const double aliasRadians = 2. * std::numbers::pi * (-0.4) * static_cast<double>(kTotal - 1UZ);
            const double unwrapped    = 2. * std::numbers::pi * static_cast<double>(run_.cycles.back()) + static_cast<double>(run_.phase.back());
            std::println("criterion 10: 0.6 fs unwrapped {:.3f}, aliased prediction (-0.4 fs) {:.3f}", unwrapped, aliasRadians);
            expect(std::abs(unwrapped - aliasRadians) < 1e-2) << "0.6 cycles/sample must alias to the -0.4 cycles/sample result, not the true 0.6";
        }

        {
            // the default max_step_fraction = 0.9 sets the threshold at 0.9*pi; at 0.6 fs the observed step is
            // 0.8*pi, under it, so the hook does not fire — recorded rather than asserted per criterion 10's original
            // text, since F14 found no fraction-of-pi threshold separates the two sides of Nyquist here.
            gr::test::RuntimeTest test;
            auto&                 source    = test.emplace<ToneSource>();
            auto&                 block     = test.emplace<PhaseUnwrap>();
            auto&                 sink      = test.emplace<Collector<std::int64_t>>();
            auto&                 phaseSink = test.emplace<Collector<float>>();
            source.cyclesPerSample          = 0.6;
            source.total                    = 2000UZ;
            source.burst                    = 4096UZ;
            std::ignore                     = test.connect(source, "out", block, "in");
            std::ignore                     = test.connect(block, "cycles", sink, "in");
            std::ignore                     = test.connect(block, "phase", phaseSink, "in");
            std::ignore                     = test.run();
            std::println("criterion 10 (F14): nSuspectSteps() at 0.6 fs = {} (the step there is 0.8*pi, under the default 0.9*pi threshold)", block.nSuspectSteps());
            expect(eq(block.nSuspectSteps(), std::uint64_t{0ULL})) << "F14: the hook does not fire at 0.6 fs, which is why the criterion moved to 0.47 fs";
        }
        {
            // 0.47 cycles/sample: the step is 0.94*pi, over the default 0.9*pi threshold, so this is where the
            // observability hook does fire, per F14's amendment to criterion 10.
            gr::test::RuntimeTest test;
            auto&                 source    = test.emplace<ToneSource>();
            auto&                 block     = test.emplace<PhaseUnwrap>();
            auto&                 sink      = test.emplace<Collector<std::int64_t>>();
            auto&                 phaseSink = test.emplace<Collector<float>>();
            source.cyclesPerSample          = 0.47;
            source.total                    = 2000UZ;
            source.burst                    = 4096UZ;
            std::ignore                     = test.connect(source, "out", block, "in");
            std::ignore                     = test.connect(block, "cycles", sink, "in");
            std::ignore                     = test.connect(block, "phase", phaseSink, "in");
            std::ignore                     = test.run();
            std::println("criterion 10 (F14): nSuspectSteps() at 0.47 fs = {}", block.nSuspectSteps());
            expect(block.nSuspectSteps() > 0ULL) << "0.47 cycles/sample must cross the default suspect threshold, per F14's amendment to criterion 10";
        }
    };

    // criterion 11: the reset rule, on the same stream with only the flag differing.
    "criterion 11: a n_dropped_samples tag resets the count exactly when reset_on_discontinuity is set"_test = [] {
        constexpr std::size_t kTotal  = 10000UZ;
        constexpr std::size_t kDropAt = 4000UZ;

        const Run withReset    = run(gr::property_map{{"reset_on_discontinuity", true}}, 0.1, kTotal, 777UZ, true, kDropAt);
        const Run withoutReset = run(gr::property_map{{"reset_on_discontinuity", false}}, 0.1, kTotal, 777UZ, true, kDropAt);

        expect(eq(withReset.cycles.size(), kTotal));
        expect(eq(withoutReset.cycles.size(), kTotal));
        if (withReset.cycles.size() != kTotal || withoutReset.cycles.size() != kTotal) {
            return;
        }

        for (std::size_t k = 0UZ; k < kDropAt; ++k) {
            expect(eq(withReset.cycles[k], withoutReset.cycles[k])) << std::format("sample {}: the two runs must agree before the drop", k);
        }
        std::println("criterion 11: cycles just before the drop {}, at the drop with reset {}, without reset {}", withReset.cycles[kDropAt - 1UZ], withReset.cycles[kDropAt], withoutReset.cycles[kDropAt]);
        expect(eq(withReset.cycles[kDropAt], std::int64_t{0LL})) << "with reset_on_discontinuity, the count returns to zero at the tagged sample";
        expect(withoutReset.cycles[kDropAt] > std::int64_t{100LL}) << "without reset_on_discontinuity, the count is nowhere near zero at the same sample: ~400 cycles have already accumulated at 0.1 cycles/sample";
        expect(std::abs(withoutReset.cycles[kDropAt] - withReset.cycles[kDropAt - 1UZ]) <= std::int64_t{1LL}) << "without the flag, this sample's cycle count is the stream's own continuous value, unaffected by the tag";

        {
            gr::test::RuntimeTest test;
            auto&                 source    = test.emplace<ToneSource>();
            auto&                 block     = test.emplace<PhaseUnwrap>(gr::property_map{{"reset_on_discontinuity", true}});
            auto&                 sink      = test.emplace<Collector<std::int64_t>>();
            auto&                 phaseSink = test.emplace<Collector<float>>();
            source.cyclesPerSample          = 0.1;
            source.total                    = kTotal;
            source.burst                    = 777UZ;
            source.tagDrop                  = true;
            source.dropAt                   = kDropAt;
            std::ignore                     = test.connect(source, "out", block, "in");
            std::ignore                     = test.connect(block, "cycles", sink, "in");
            std::ignore                     = test.connect(block, "phase", phaseSink, "in");
            std::ignore                     = test.run();
            expect(eq(block.nResets(), std::uint64_t{1ULL})) << "reset_on_discontinuity: one dropped-sample tag raises nResets() by one";
        }
        {
            gr::test::RuntimeTest test;
            auto&                 source    = test.emplace<ToneSource>();
            auto&                 block     = test.emplace<PhaseUnwrap>(gr::property_map{{"reset_on_discontinuity", false}});
            auto&                 sink      = test.emplace<Collector<std::int64_t>>();
            auto&                 phaseSink = test.emplace<Collector<float>>();
            source.cyclesPerSample          = 0.1;
            source.total                    = kTotal;
            source.burst                    = 777UZ;
            source.tagDrop                  = true;
            source.dropAt                   = kDropAt;
            std::ignore                     = test.connect(source, "out", block, "in");
            std::ignore                     = test.connect(block, "cycles", sink, "in");
            std::ignore                     = test.connect(block, "phase", phaseSink, "in");
            std::ignore                     = test.run();
            expect(eq(block.nResets(), std::uint64_t{0ULL})) << "with the flag false, nResets() stays zero";
        }
    };

    // criterion 18 (PhaseUnwrap half): before any sample, cycles() reads 0.
    "criterion 18: cycles() reads 0 before any sample is processed"_test = [] {
        PhaseUnwrap block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block._unwrapper.cycles(), std::int64_t{0LL}));
    };

    // §6.3's `origin`: `zero` subtracts the first sample's phase so the output starts at exactly zero, and neither
    // setting changes the differences, which is what a consumer usually reads.
    "origin: 'zero' starts at exactly zero and leaves every difference untouched"_test = [] {
        constexpr std::size_t kTotal  = 4000UZ;
        constexpr double      kRate   = 0.137;
        constexpr double      kOffset = 0.9; // the tone starts away from zero, so the two settings are distinguishable

        const Scene scene{.cyclesPerSample = kRate, .phaseOffset = kOffset, .total = kTotal, .burst = 777UZ, .tagDrop = false, .dropAt = 0UZ};

        const Run first = runScene({{"origin", std::string("first_sample")}}, scene);
        const Run zero  = runScene({{"origin", std::string("zero")}}, scene);
        expect(eq(first.cycles.size(), kTotal));
        expect(eq(zero.cycles.size(), kTotal));
        if (first.cycles.size() != kTotal || zero.cycles.size() != kTotal) {
            return;
        }

        std::println("origin: first_sample phase[0] = {:.6f}, zero phase[0] = {:.6f}", first.phase.front(), zero.phase.front());
        expect(std::abs(first.phase.front()) > 1e-9f) << "the tone's own first phase is not zero, so the two settings are distinguishable here";
        expect(eq(zero.phase.front(), 0.f)) << "origin = zero starts the output at exactly zero";
        expect(eq(first.cycles.front(), std::int64_t{0LL}));
        expect(eq(zero.cycles.front(), std::int64_t{0LL}));

        double worst = 0.;
        for (std::size_t k = 1UZ; k < kTotal; ++k) {
            const double stepFirst = 2. * std::numbers::pi * static_cast<double>(first.cycles[k] - first.cycles[k - 1UZ]) + static_cast<double>(first.phase[k] - first.phase[k - 1UZ]);
            const double stepZero  = 2. * std::numbers::pi * static_cast<double>(zero.cycles[k] - zero.cycles[k - 1UZ]) + static_cast<double>(zero.phase[k] - zero.phase[k - 1UZ]);
            worst                  = std::max(worst, std::abs(stepFirst - stepZero));
        }
        std::println("origin: worst difference between the two settings' per-sample steps {:.3e} rad", worst);
        expect(worst < 1e-5) << "neither origin changes the differences";
    };

    // §6.3's refusals, and `max_step_fraction` as a live setting: it moves the hook's threshold without disturbing
    // the count, so the same stream reads a different nSuspectSteps() at a different fraction and the same cycles.
    "max_step_fraction: refused outside (0, 1], live over the same stream, and never touching the count"_test = [] {
        const auto staged = [](gr::property_map settings) {
            PhaseUnwrap block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { staged({{"max_step_fraction", 0.0}}); })) << "zero is outside (0, 1]";
        expect(throws([&] { staged({{"max_step_fraction", 1.5}}); })) << "above one is outside (0, 1]";
        expect(nothrow([&] { staged({{"max_step_fraction", 1.0}}); })) << "one itself is admitted";
        expect(throws([&] { staged({{"origin", std::string("elsewhere")}}); })) << "origin takes two values and no others";

        // 0.42 cycles/sample steps by 0.84*pi: under the default 0.9 fraction, over a fraction of 0.8.
        constexpr std::size_t kTotal = 2000UZ;
        const Run             loose  = run({{"max_step_fraction", 0.9}}, 0.42, kTotal, 4096UZ);
        const Run             tight  = run({{"max_step_fraction", 0.8}}, 0.42, kTotal, 4096UZ);
        std::println("max_step_fraction: nSuspectSteps() at 0.9 = {}, at 0.8 = {} (the step is 0.84*pi)", loose.nSuspectSteps, tight.nSuspectSteps);
        expect(eq(loose.nSuspectSteps, std::uint64_t{0ULL})) << "0.84*pi does not cross a 0.9*pi threshold";
        expect(eq(tight.nSuspectSteps, std::uint64_t{kTotal - 1UZ})) << "it crosses a 0.8*pi threshold at every step after the first";
        expect(eq(tight.lastSuspectIndex, static_cast<std::int64_t>(kTotal - 1UZ))) << "and the last one recorded is the last sample";
        expect(eq(loose.lastSuspectIndex, std::int64_t{-1LL})) << "with no crossing there is no index to report";
        expect(std::ranges::equal(loose.cycles, tight.cycles)) << "the fraction is an observability hook and moves no count";
        expect(std::ranges::equal(loose.phase, tight.phase));
        expect(eq(loose.nSaturations, std::uint64_t{0ULL})) << "2^63 turns are not reachable from a seeded scene";
    };

    // §6.4's second reset case: a change of `origin` returns the count to zero and re-takes the origin. Driven on
    // the block itself: a settings key that is not one of the reserved stream tags does not reach a running block
    // from the stream.
    "an origin change resets the count and raises nResets()"_test = [] {
        PhaseUnwrap block;
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();

        constexpr std::size_t     kSamples = 5000UZ; // ~500 turns at 0.1 cycles/sample
        std::vector<CF>           in(kSamples);
        std::vector<std::int64_t> cycles(kSamples);
        std::vector<float>        phase(kSamples);
        for (std::size_t k = 0UZ; k < kSamples; ++k) {
            const double p = 0.9 + 2. * std::numbers::pi * 0.1 * static_cast<double>(k);
            in[k]          = CF(static_cast<float>(std::cos(p)), static_cast<float>(std::sin(p)));
        }
        block._unwrapper.process(in, cycles, phase);
        std::println("origin change: cycles before the change {}, nResets() {}", block._unwrapper.cycles(), block.nResets());
        expect(block._unwrapper.cycles() > std::int64_t{400LL}) << "about 500 turns have accumulated by then";
        expect(eq(block.nResets(), std::uint64_t{0ULL}));

        expect(block.settings().setStaged(gr::property_map{{"origin", std::string("zero")}}).empty()) << "the new origin is accepted";
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nResets(), std::uint64_t{1ULL})) << "an origin change is one reset";
        expect(eq(block._unwrapper.cycles(), std::int64_t{0LL})) << "and the count returns to zero";
        expect(!block._unwrapper.started()) << "with the origin waiting to be re-taken at the next sample";

        expect(block.settings().setStaged(gr::property_map{{"origin", std::string("zero")}}).empty());
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nResets(), std::uint64_t{1ULL})) << "re-stating the origin it already has is not a change";
    };

    // §8: the two output streams state their units, and nothing else is invented.
    "the two outputs carry their signal_unit, rad on phase and none on cycles"_test = [] {
        const Run r = run(gr::property_map{}, 0.1, 2000UZ, 4096UZ);
        std::println("signal_unit: cycles '{}', phase '{}'", tagString(r.cycleTags, 0ULL, "signal_unit"), tagString(r.phaseTags, 0ULL, "signal_unit"));
        expect(eq(tagString(r.cycleTags, 0ULL, "signal_unit"), std::string(""))) << "a whole-turn count is dimensionless";
        expect(eq(tagString(r.phaseTags, 0ULL, "signal_unit"), std::string("rad"))) << "the residual is in radians";
    };

    // criterion 19: chunk independence, bit-identical cycles and phase streams and counters, at every chunk size.
    "criterion 19: chunk independence, bit-identical for every chunk size"_test = [] {
        constexpr std::size_t kTotal  = 3UZ * 4096UZ + 777UZ;
        constexpr std::size_t kDropAt = 4096UZ; // a tagged sample landing exactly on a chunk boundary at burst 4096

        const Run reference = run(gr::property_map{}, 0.37, kTotal, 4096UZ, true, kDropAt);
        expect(eq(reference.cycles.size(), kTotal));
        expect(eq(reference.nResets, std::uint64_t{1ULL}));
        expect(eq(reference.nSuspectSteps, std::uint64_t{0ULL})) << "0.37 cycles/sample steps by 0.74*pi, and the step across the reset is not measured at all";

        for (const std::size_t burst : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            const Run candidate = run(gr::property_map{}, 0.37, kTotal, burst, true, kDropAt);
            expect(eq(candidate.cycles.size(), reference.cycles.size())) << std::format("burst {}", burst);
            expect(std::ranges::equal(candidate.cycles, reference.cycles)) << std::format("burst {}: cycles stream differs", burst);
            expect(std::ranges::equal(candidate.phase, reference.phase)) << std::format("burst {}: phase stream differs", burst);
            expect(eq(candidate.nResets, reference.nResets)) << std::format("burst {}: nResets", burst);
            expect(eq(candidate.nSuspectSteps, reference.nSuspectSteps)) << std::format("burst {}: nSuspectSteps", burst);
            expect(eq(candidate.nSaturations, reference.nSaturations)) << std::format("burst {}: nSaturations", burst);
            expect(eq(candidate.lastSuspectIndex, reference.lastSuspectIndex)) << std::format("burst {}: lastSuspectIndex", burst);
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
