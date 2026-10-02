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

#include <gnuradio-4.0/math/Reduce.hpp>

namespace {

constexpr std::size_t kSamples = 65536UZ;
constexpr std::size_t kRounds  = 200UZ;
constexpr std::size_t kWarmUp  = 50UZ;

template<typename T>
std::vector<T> generateInput() {
    std::vector<T> input(kSamples);
    for (std::size_t i = 0UZ; i < kSamples; ++i) {
        // values between -10 and 10
        const float value = -10.f + 20.f * static_cast<float>((i * 7919UZ) % 1000UZ) / 1000.f;
        if constexpr (gr::meta::complex_like<T>) {
            input[i] = T(value, 1.f - value);
        } else {
            input[i] = static_cast<T>(value);
        }
    }
    return input;
}

struct Arm {
    std::string               name;
    std::function<void()>     pass;
    std::vector<std::int64_t> times{};
};

// one pass reduces the whole input in groups of n, as one processBulk call over whole groups does
template<template<typename> typename TAlias, typename T>
Arm makeArm(std::string name, std::size_t n) {
    using TBlock = TAlias<T>;
    using R      = typename TBlock::R;
    struct State {
        std::vector<T> in  = generateInput<T>();
        std::vector<R> out = std::vector<R>{};
        TBlock         block{};
    };
    auto state = std::make_shared<State>();
    state->out.resize(kSamples / n);
    return Arm{std::move(name), [state] {
                   state->block.reduceGroups(std::span<const T>(state->in), std::span<R>(state->out));
                   ::benchmark::force_to_memory(state->out);
               }};
}

} // namespace

int main() {
    using namespace gr::blocks::math;
    std::array arms{makeArm<Sum, float>("Sum<float32> n 4", 4UZ), makeArm<Sum, float>("Sum<float32> n 64", 64UZ), makeArm<Max, float>("Max<float32> n 4", 4UZ), makeArm<Max, float>("Max<float32> n 64", 64UZ), makeArm<Rms, std::complex<float>>("Rms<complex<float32>> n 64", 64UZ)};

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
