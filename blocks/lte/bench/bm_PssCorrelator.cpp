#include <algorithm>
#include <array>
#include <chrono>
#include <complex>
#include <cstddef>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/algorithm/lte/SyncSignals.hpp>

#include "LteDownlinkScene.hpp"

/*
 * What one half-frame of cell search costs, per input sample, in the form the correlator is written in.
 *
 * The unit is one input sample of the 1.92 MS/s stream, because that is what the cost scales with: a half-frame is
 * 9600 of them and a real-time identifier has 5 ms to spend on each half-frame, so 521 ns/sample is one whole core
 * and anything below it is the fraction of a core the arm needs. The two hypothesis counts are the ends of the
 * range the block's `frequency_search_hz` spans: one hypothesis is the default and no frequency search at all,
 * thirteen is a +/-45 kHz search.
 *
 * The direct form is what is measured because it is what is implemented. Its arithmetic is 3*K*128 complex
 * multiply-accumulates per input sample; the transform form's is one shared forward transform of the window plus
 * 3*K inverse transforms of it, which is the cheaper of the two for a wide search and is why `search_interval`
 * exists for a consumer that wants one on a small box.
 *
 * Pin this to one P-core -- `taskset -c 2` on an i7-12700H, whose cpus 0-11 are the P-cores -- and hold the machine
 * lock while it runs. The arms are interleaved so a clock ramp or a neighbor's build lands on all of them alike.
 */
namespace {

using Complex = std::complex<float>;

struct Arm {
    std::string             label;
    std::function<double()> body;
};

/// Run every arm once per repeat in the same order, discard the first pass, and report the best with its spread.
void report(std::span<Arm> arms, std::size_t unitsPerCall, std::size_t repeats) {
    std::vector<double> best(arms.size(), 1e300);
    std::vector<double> worst(arms.size(), 0.);
    double              checksum = 0.;

    for (std::size_t repeat = 0UZ; repeat <= repeats; ++repeat) {
        for (std::size_t a = 0UZ; a < arms.size(); ++a) {
            const auto start = std::chrono::steady_clock::now();
            checksum += arms[a].body();
            const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / static_cast<double>(unitsPerCall);
            if (repeat > 0UZ) {
                best[a]  = std::min(best[a], ns);
                worst[a] = std::max(worst[a], ns);
            }
        }
    }

    std::println("{} samples per call, best of {} interleaved runs after one discarded warm-up", unitsPerCall, repeats);
    for (std::size_t a = 0UZ; a < arms.size(); ++a) {
        std::println("{:<48} {:9.2f} ns/sample  (spread {:8.2f}, {:5.1f}% of a core at 1.92 MS/s)", arms[a].label, best[a], worst[a] - best[a], 100. * best[a] / 520.83);
    }
    std::println("[checksum {:g}]", checksum);
}

} // namespace

int main() {
    gr::test::lte::SceneConfig config;
    config.nId1                      = 100U;
    config.nId2                      = 1U;
    config.frames                    = 1UZ;
    config.seed                      = 0xbe4cULL;
    const gr::test::lte::Scene scene = gr::test::lte::makeDownlink(config);

    constexpr std::size_t          window = gr::lte::kMaxSecondaryLookBehind + gr::lte::kHalfFrameSamples + gr::lte::kSymbolSamples - 1UZ;
    const std::span<const Complex> whole(scene.samples.data(), window);
    const std::span<const Complex> positions = whole.subspan(gr::lte::kMaxSecondaryLookBehind);

    gr::lte::PssCorrelator narrow(0.f);
    gr::lte::PssCorrelator wide(45'000.f);
    gr::lte::SssDecoder    decoder;

    const std::array<gr::lte::PssDetection, 3UZ> located = narrow.search(positions, gr::lte::kHalfFrameSamples);
    const gr::lte::PssDetection&                 primary = located[config.nId2];
    std::println("scene: primary symbol at {}, metric {:.1f}, {} and {} hypotheses under test", primary.position, primary.metric, narrow.hypotheses(), wide.hypotheses());

    std::array<Arm, 3UZ> arms{{
        {"primary search, 1 hypothesis", [&narrow, positions] { return static_cast<double>(narrow.search(positions, gr::lte::kHalfFrameSamples)[0UZ].metric); }},
        {"primary search, 13 hypotheses", [&wide, positions] { return static_cast<double>(wide.search(positions, gr::lte::kHalfFrameSamples)[0UZ].metric); }},
        {"secondary decode, once per half-frame", [&decoder, whole, &primary, &config] { return static_cast<double>(decoder.decode(whole, gr::lte::kMaxSecondaryLookBehind + primary.position, config.nId2, primary.frequencyHz).metric); }},
    }};
    report(arms, gr::lte::kHalfFrameSamples, 9UZ);
    return 0;
}
