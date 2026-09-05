#ifndef GNURADIO_FILTER_BENCH_INTERLEAVED_HPP
#define GNURADIO_FILTER_BENCH_INTERLEAVED_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <vector>

namespace gr::blocks::filter::bench {

/**
 * @brief One thing being measured: `body` processes `samples` input samples and returns a value derived from its
 * output, which is summed and printed so the optimizer cannot delete the work.
 *
 * The sample count is per arm rather than shared, because a filter's pass is sized by its own tap count — 11001 taps at
 * decimation one and 143 at sixty-four are four orders apart in cost per sample and one buffer cannot serve both.
 * `tapsPerSample` is `N/M`, the multiply-accumulates an input sample costs, and is what turns the headline figure into
 * a per-tap number comparable across shapes.
 */
struct Arm {
    std::string             label;
    std::size_t             samples;
    double                  tapsPerSample;
    std::function<double()> body;
};

/**
 * @brief Run every arm once per repeat, in the same order, and report the best with its spread.
 *
 * Interleaving is what makes the arms comparable: a clock ramp, a thermal excursion or a neighbor's build lands on all
 * of them alike rather than on whichever ran last. The first pass is the warm-up and is discarded. The figure is the
 * best of the rest, because the best is the one the machine is capable of and every larger number is something else
 * that happened; the spread is printed beside it so a run that was disturbed says so.
 *
 * Pin the run to one core: on a hybrid CPU the same binary reports figures a factor of three apart according to
 * which core type it lands on.
 */
inline void report(std::span<Arm> arms, std::size_t repeats) {
    std::vector<double> best(arms.size(), 1e300);
    std::vector<double> worst(arms.size(), 0.0);
    double              checksum = 0.0;

    for (std::size_t repeat = 0UZ; repeat <= repeats; ++repeat) {
        for (std::size_t a = 0UZ; a < arms.size(); ++a) {
            const auto start = std::chrono::steady_clock::now();
            checksum += arms[a].body();
            const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / static_cast<double>(arms[a].samples);
            if (repeat > 0UZ) {
                best[a]  = std::min(best[a], ns);
                worst[a] = std::max(worst[a], ns);
            }
        }
    }

    std::println("best of {} interleaved runs after one discarded warm-up", repeats);
    for (std::size_t a = 0UZ; a < arms.size(); ++a) {
        std::println("{:<40} {:10.3f} ns/input  (spread {:8.3f})  {:.4f} ns per tap and output", arms[a].label, best[a], worst[a] - best[a], arms[a].tapsPerSample > 0.0 ? best[a] / arms[a].tapsPerSample : 0.0);
    }
    std::println("[checksum {:g}]", checksum);
}

} // namespace gr::blocks::filter::bench

#endif // GNURADIO_FILTER_BENCH_INTERLEAVED_HPP
