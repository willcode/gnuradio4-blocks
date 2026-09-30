#ifndef GNURADIO_AX25_BENCH_INTERLEAVED_HPP
#define GNURADIO_AX25_BENCH_INTERLEAVED_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace gr::blocks::ax25::bench {

/// @brief One measured arm.
///
/// `body` processes `unitsPerCall` units and returns a value derived from its output. The harness sums and prints
/// these values. The printed sum keeps the optimizer from deleting the work.
struct Arm {
    std::string             label;
    std::function<double()> body;
};

/**
 * @brief Run every arm once per repeat, in the same order, and report the best with its spread.
 *
 * Interleaving makes the arms comparable. A clock ramp, a thermal excursion or another build on the machine affects
 * every arm alike. The first pass is a warm-up and is discarded. The reported figure is the best of the remaining
 * passes. The best time shows what the machine can do. Every larger time includes some other delay. The spread is
 * printed beside the figure and shows when a run was disturbed.
 */
inline void report(std::span<Arm> arms, std::size_t unitsPerCall, std::size_t repeats, std::string_view unit) {
    std::vector<double> best(arms.size(), 1e300);
    std::vector<double> worst(arms.size(), 0.0);
    double              checksum = 0.0;

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

    std::println("{} {}s per call, best of {} interleaved runs after one discarded warm-up", unitsPerCall, unit, repeats);
    for (std::size_t a = 0UZ; a < arms.size(); ++a) {
        std::println("{:<44} {:9.1f} ns/{}  (spread {:9.1f})", arms[a].label, best[a], unit, worst[a] - best[a]);
    }
    std::println("[checksum {:g}]", checksum);
}

} // namespace gr::blocks::ax25::bench

#endif // GNURADIO_AX25_BENCH_INTERLEAVED_HPP
