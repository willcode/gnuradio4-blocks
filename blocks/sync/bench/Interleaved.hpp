#ifndef GNURADIO_SYNC_BENCH_INTERLEAVED_HPP
#define GNURADIO_SYNC_BENCH_INTERLEAVED_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <vector>

namespace gr::blocks::sync::bench {

/// @brief One thing being measured. `body` processes `samplesPerCall` samples and returns a value derived from its
/// output, which is summed and printed so the optimizer cannot delete the work.
struct Arm {
    std::string             label;
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
inline void report(std::span<Arm> arms, std::size_t samplesPerCall, std::size_t repeats) {
    std::vector<double> best(arms.size(), 1e300);
    std::vector<double> worst(arms.size(), 0.0);
    double              checksum = 0.0;

    for (std::size_t repeat = 0UZ; repeat <= repeats; ++repeat) {
        for (std::size_t a = 0UZ; a < arms.size(); ++a) {
            const auto start = std::chrono::steady_clock::now();
            checksum += arms[a].body();
            const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / static_cast<double>(samplesPerCall);
            if (repeat > 0UZ) {
                best[a]  = std::min(best[a], ns);
                worst[a] = std::max(worst[a], ns);
            }
        }
    }

    std::println("{} samples per call, best of {} interleaved runs after one discarded warm-up", samplesPerCall, repeats);
    for (std::size_t a = 0UZ; a < arms.size(); ++a) {
        std::println("{:<44} {:7.2f} ns/sample  (spread {:5.2f})  {:9.1f} Msample/s", arms[a].label, best[a], worst[a] - best[a], 1e3 / best[a]);
    }
    std::println("[checksum {:g}]", checksum);
}

} // namespace gr::blocks::sync::bench

#endif // GNURADIO_SYNC_BENCH_INTERLEAVED_HPP
