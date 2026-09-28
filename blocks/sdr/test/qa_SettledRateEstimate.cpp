#include <boost/ut.hpp>

#include <gnuradio-4.0/sdr/SettledRateEstimate.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <numbers>
#include <print>

// The estimate is driven with sample counts and arrival times alone, as the RTL2832 source drives it: 32768 complex
// samples per 64 KiB read at 250 kS/s, a 0.1 Hz corner, so a read arrives every 131 ms and the estimate settles after
// 5 / (2 pi 0.1 Hz) = 7.96 s. No device is involved and no clock is read.
namespace {

using gr::blocks::sdr::SettledRateEstimate;

constexpr double      kNominalRate = 250'000.0;
constexpr std::size_t kChunk       = 32'768UZ;
constexpr double      kUpdateHz    = kNominalRate / static_cast<double>(kChunk);
constexpr float       kCutoffHz    = 0.1f;
constexpr double      kStart       = 1'000.0; // an arbitrary origin, so that no case depends on time zero

// a stream of whole reads from a clock `ppm` fast, delivered on time from `tFirst`; returns the next arrival time
double feed(SettledRateEstimate& estimate, double tFirst, double seconds, double ppm, auto&& onUpdate) {
    const double interval = static_cast<double>(kChunk) / (kNominalRate * (1.0 + ppm * 1e-6));
    double       t        = tFirst;
    while (t < tFirst + seconds) {
        estimate.update(t, kChunk);
        onUpdate(t);
        t += interval;
    }
    return t;
}

double feed(SettledRateEstimate& estimate, double tFirst, double seconds, double ppm) {
    return feed(estimate, tFirst, seconds, ppm, [](double) {});
}

SettledRateEstimate freshEstimate() {
    SettledRateEstimate estimate;
    estimate.reset(kNominalRate, kUpdateHz, kCutoffHz);
    return estimate;
}

} // namespace

const boost::ut::suite<"SettledRateEstimate"> _settledRateTests = [] {
    using namespace boost::ut;

    "the filter starts in its steady state at the configured rate"_test = [] {
        SettledRateEstimate estimate = freshEstimate();
        double              largest  = 0.0;
        std::size_t         nUpdates = 0UZ;
        feed(estimate, kStart, 20.0, 0.0, [&](double) {
            largest = std::max(largest, std::abs(estimate.offsetPpm()));
            ++nUpdates;
        });
        std::println("{} reads at the configured rate: largest offset of the estimate {:.3e} ppm", nUpdates, largest);
        expect(gt(nUpdates, 100UZ));
        expect(lt(largest, 1e-3)) << "a stream at exactly the configured rate moves the estimate";

        // the same filter without priming, printed for comparison: it reads several times the rate after one second
        gr::algorithm::SampleRateEstimator bare;
        bare.filter_cutoff_hz = kCutoffHz;
        bare.reset(kNominalRate, kUpdateHz);
        double tLast = kStart;
        for (std::size_t i = 0UZ; i < 9UZ; ++i) {
            tLast = kStart + static_cast<double>(i) * static_cast<double>(kChunk) / kNominalRate;
            bare.update(tLast, kChunk);
        }
        std::println("an unprimed filter after {:.2f} s reads {:.4f} times the configured rate", tLast - kStart, bare.estimatedRate() / kNominalRate);
    };

    "the estimate answers nothing until it has settled"_test = [] {
        SettledRateEstimate estimate = freshEstimate();
        expect(approx(estimate.settleSeconds(), 5.0 / (2.0 * std::numbers::pi * 0.1), 1e-6));
        std::size_t nEarlyAnswers = 0UZ;
        std::size_t nLateSilences = 0UZ;
        feed(estimate, kStart, 20.0, 50.0, [&](double t) {
            const bool answered = estimate.settledRate().has_value();
            if (t - kStart < estimate.settleSeconds()) {
                nEarlyAnswers += answered ? 1UZ : 0UZ;
            } else if (t - kStart >= estimate.settleSeconds() + 1.0 / kUpdateHz) {
                nLateSilences += answered ? 0UZ : 1UZ;
            }
            expect(eq(answered, estimate.settledPpm().has_value()));
        });
        expect(eq(nEarlyAnswers, 0UZ)) << "an answer before the settle time";
        expect(eq(nLateSilences, 0UZ)) << "no answer after the settle time";
        expect(fatal(estimate.settledPpm().has_value()));
        expect(lt(std::abs(static_cast<double>(*estimate.settledPpm()) - 50.0), 1.0)) << std::format("a 50 ppm clock reads {} ppm", *estimate.settledPpm());
        expect(lt(std::abs(*estimate.settledRate() / (kNominalRate * (1.0 + 50e-6)) - 1.0), 1e-6));
    };

    "an estimate outside 1000 ppm is not settled, and the wait restarts inside the bound"_test = [] {
        SettledRateEstimate estimate = freshEstimate();
        double              t        = feed(estimate, kStart, 20.0, 0.0);
        expect(fatal(estimate.settled()));

        // one read arrives two reads late, and the reads after it keep their pace
        t += 2.0 * static_cast<double>(kChunk) / kNominalRate;
        estimate.update(t, kChunk);
        expect(gt(std::abs(estimate.offsetPpm()), SettledRateEstimate::kMaxOffsetPpm)) << "the late read did not leave the bound, so the case tests nothing";
        expect(!estimate.settled());

        double      returned  = 0.0;
        std::size_t nOutside  = 0UZ;
        std::size_t nWrongful = 0UZ;
        feed(estimate, t + static_cast<double>(kChunk) / kNominalRate, 60.0, 0.0, [&](double now) {
            const bool outside = std::abs(estimate.offsetPpm()) > SettledRateEstimate::kMaxOffsetPpm;
            if (outside) {
                ++nOutside;
                returned = 0.0;
            } else if (returned == 0.0) {
                returned = now;
            }
            const bool due = !outside && now - returned >= estimate.settleSeconds();
            nWrongful += estimate.settled() != due ? 1UZ : 0UZ;
        });
        std::println("after one late read the estimate stayed outside the bound for {} reads", nOutside);
        expect(gt(nOutside, 1UZ));
        expect(eq(nWrongful, 0UZ)) << "the estimate settled outside the rule";
        expect(estimate.settled()) << "a minute after the late read";
    };

    "a phase reset keeps the settled state, a reset and a new corner restart the wait"_test = [] {
        SettledRateEstimate estimate = freshEstimate();
        double              t        = feed(estimate, kStart, 20.0, 50.0);
        expect(fatal(estimate.settled()));
        const double before = *estimate.settledRate();

        estimate.resetPhase();
        t += 5.0; // a pause that measures no interval
        estimate.update(t, kChunk);
        expect(estimate.settled());
        expect(eq(*estimate.settledRate(), before));

        estimate.setCutoff(0.2f);
        expect(!estimate.settled());
        expect(eq(estimate.estimator.estimatedRate(), before)) << "a new corner keeps the estimate";
        feed(estimate, t + static_cast<double>(kChunk) / (kNominalRate * (1.0 + 50e-6)), estimate.settleSeconds() + 1.0, 50.0);
        expect(estimate.settled());

        estimate.reset(kNominalRate, kUpdateHz, kCutoffHz);
        expect(!estimate.settled());
        expect(eq(estimate.offsetPpm(), 0.0));
    };

    "a zero configured rate never settles"_test = [] {
        SettledRateEstimate estimate;
        estimate.reset(0.0, kUpdateHz, kCutoffHz);
        feed(estimate, kStart, 20.0, 0.0);
        expect(!estimate.settled());
        expect(!estimate.settledRate().has_value());
    };
};

int main() { /* not needed for UT */ }
