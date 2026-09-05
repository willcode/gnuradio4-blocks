#ifndef GNURADIO_ANALOG_BENCH_THROUGHPUT_HPP
#define GNURADIO_ANALOG_BENCH_THROUGHPUT_HPP

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace gr::blocks::analog::bench {

// `body` processes `samplesPerCall` samples and returns a value derived from its output; the returned values are
// summed and printed, which is what keeps the optimizer from deleting the work being measured
template<typename FBody>
void report(std::string_view label, std::size_t samplesPerCall, std::size_t iterations, FBody&& body) {
    double checksum = body();

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0UZ; i < iterations; ++i) {
        checksum += body();
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const double samples = static_cast<double>(iterations * samplesPerCall);

    std::println("{:<34} {:7.2f} ns/sample {:9.1f} Msample/s   [checksum {:g}]", label, 1e9 * elapsed / samples, samples / elapsed / 1e6, checksum);
}

/// @brief One arm of a comparison: `body` processes `samples` samples and returns a value derived from its output.
struct Arm {
    std::string             label;
    std::size_t             samples;
    std::function<double()> body;
};

/**
 * @brief Run every arm once per repeat, in the same order, and report the best with its spread.
 *
 * Interleaving is what makes the arms comparable: a clock ramp, a thermal excursion or a neighbor's build lands on all
 * of them alike rather than on whichever ran last. The first pass is the warm-up and is discarded. The figure is the
 * best of the rest, because the best is what the machine is capable of and every larger number is something else that
 * happened; the spread beside it says whether the run was disturbed.
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
        std::println("{:<44} {:10.4f} ns/sample  (spread {:8.4f})", arms[a].label, best[a], worst[a] - best[a]);
    }
    std::println("[checksum {:g}]", checksum);
}

} // namespace gr::blocks::analog::bench

#endif // GNURADIO_ANALOG_BENCH_THROUGHPUT_HPP
