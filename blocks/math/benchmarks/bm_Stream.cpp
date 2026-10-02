#include <benchmark.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/math/Stream.hpp>

namespace {

constexpr std::size_t kSamples = 65536UZ;
constexpr std::size_t kRounds  = 200UZ;
constexpr std::size_t kWarmUp  = 50UZ;

template<typename T>
T sampleValue(std::size_t i) {
    // values between -10 and 10
    const float value = -10.f + 20.f * static_cast<float>((i * 7919UZ) % 1000UZ) / 1000.f;
    if constexpr (gr::meta::complex_like<T>) {
        return T(value, 1.f - value);
    } else {
        return static_cast<T>(value);
    }
}

template<typename T>
std::vector<T> generate(std::size_t count) {
    std::vector<T> values(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        values[i] = sampleValue<T>(i);
    }
    return values;
}

struct Arm {
    std::string               name;
    std::function<void()>     pass;
    std::vector<std::int64_t> times{};
};

// one pass sums the whole input, as one processBulk call with no reset tag does
template<typename T>
Arm cumulativeSumArm(std::string name) {
    struct State {
        std::vector<T>                     in  = generate<T>(kSamples);
        std::vector<T>                     out = std::vector<T>(kSamples);
        gr::blocks::math::CumulativeSum<T> block{};
    };
    auto state = std::make_shared<State>();
    return Arm{std::move(name), [state] {
                   state->block.accumulate(std::span<const T>(state->in), std::span<T>(state->out));
                   ::benchmark::force_to_memory(state->out);
               }};
}

// one pass multiplies every whole group of the input by the weights, as one processBulk call does
template<typename T>
Arm weightsArm(std::string name, std::size_t rows, std::size_t columns) {
    struct State {
        std::vector<T>                       in = generate<T>(kSamples);
        std::vector<T>                       out{};
        gr::blocks::math::stream::Weights<T> weights{};
    };
    auto state             = std::make_shared<State>();
    state->weights.rows    = rows;
    state->weights.columns = columns;
    state->weights.entries = generate<T>(rows * columns);
    state->out.resize(kSamples / columns * rows);
    return Arm{std::move(name), [state] {
                   state->weights.apply(std::span<const T>(state->in), std::span<T>(state->out));
                   ::benchmark::force_to_memory(state->out);
               }};
}

} // namespace

int main() {
    std::array arms{cumulativeSumArm<float>("CumulativeSum<float32>"), weightsArm<float>("MultiplyByMatrix<float32> 4 x 4", 4UZ, 4UZ), weightsArm<float>("MultiplyByMatrix<float32> 16 x 16", 16UZ, 16UZ), weightsArm<std::complex<float>>("DotProduct<complex<float32>> n 64", 1UZ, 64UZ)};

    // the arms interleaved pass by pass, the warm-up passes discarded
    for (std::size_t round = 0UZ; round < kWarmUp + kRounds; ++round) {
        for (Arm& arm : arms) {
            const auto start = std::chrono::steady_clock::now();
            arm.pass();
            const auto stop = std::chrono::steady_clock::now();
            if (round >= kWarmUp) {
                arm.times.push_back(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
            }
        }
    }

    std::printf("%-36s %12s %12s %12s   (ns per input sample over %zu samples, %zu passes)\n", "arm", "best", "median", "max", kSamples, kRounds);
    for (Arm& arm : arms) {
        std::ranges::sort(arm.times);
        const auto perSample = [](std::int64_t ns) { return static_cast<double>(ns) / static_cast<double>(kSamples); };
        std::printf("%-36s %12.3f %12.3f %12.3f\n", arm.name.c_str(), perSample(arm.times.front()), perSample(arm.times[arm.times.size() / 2UZ]), perSample(arm.times.back()));
    }
    std::fflush(stdout);
    std::_Exit(0);
}
