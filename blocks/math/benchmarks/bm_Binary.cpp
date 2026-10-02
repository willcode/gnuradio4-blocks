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

#include <gnuradio-4.0/math/Binary.hpp>

namespace {

constexpr std::size_t kSamples = 65536UZ;
constexpr std::size_t kRounds  = 200UZ;
constexpr std::size_t kWarmUp  = 50UZ;

template<typename T>
std::vector<T> generateInput(std::size_t stride) {
    std::vector<T> input(kSamples);
    for (std::size_t i = 0UZ; i < kSamples; ++i) {
        // values between -10 and 10, a and b drawn with different strides
        const float value = -10.f + 20.f * static_cast<float>((i * stride) % 1000UZ) / 1000.f;
        if constexpr (gr::meta::complex_like<T>) {
            input[i] = T(value, 1.f - value);
        } else {
            input[i] = static_cast<T>(value);
        }
    }
    return input;
}

// calls processOne as the framework does: over simd vectors where the block takes them, then the tail one pair at a time
template<typename TBlock, typename T, typename R>
void processSpan(const TBlock& block, std::span<const T> a, std::span<const T> b, std::span<R> output) {
    std::size_t i = 0UZ;
    if constexpr (TBlock::kSimdPath) {
        using V = vir::stdx::native_simd<T>;
        for (; i + V::size() <= a.size(); i += V::size()) {
            const V x(a.data() + i, vir::stdx::element_aligned);
            const V y(b.data() + i, vir::stdx::element_aligned);
            block.processOne(x, y).copy_to(output.data() + i, vir::stdx::element_aligned);
        }
    }
    for (; i < a.size(); ++i) {
        output[i] = block.processOne(a[i], b[i]);
    }
}

struct Arm {
    std::string               name;
    std::function<void()>     pass;
    std::vector<std::int64_t> times{};
};

template<template<typename> typename TAlias, typename T>
Arm makeArm(std::string name) {
    using TBlock = TAlias<T>;
    using R      = typename TBlock::R;
    struct State {
        std::vector<T> a   = generateInput<T>(7919UZ);
        std::vector<T> b   = generateInput<T>(104729UZ);
        std::vector<R> out = std::vector<R>(kSamples);
        TBlock         block{};
    };
    auto state = std::make_shared<State>();
    state->block.settings().init();
    std::ignore = state->block.settings().applyStagedParameters();
    return Arm{std::move(name), [state] {
                   processSpan(state->block, std::span<const T>(state->a), std::span<const T>(state->b), std::span<R>(state->out));
                   ::benchmark::force_to_memory(state->out);
               }};
}

} // namespace

int main() {
    using namespace gr::blocks::math;
    std::array arms{makeArm<Maximum, float>("Maximum<float32>"), makeArm<MultiplyConjugate, std::complex<float>>("MultiplyConjugate<complex<float32>>"), makeArm<Hypot, float>("Hypot<float32>")};

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

    std::printf("%-36s %12s %12s %12s   (ns per sample over %zu samples, %zu passes)\n", "arm", "best", "median", "max", kSamples, kRounds);
    for (Arm& arm : arms) {
        std::ranges::sort(arm.times);
        const auto perSample = [](std::int64_t ns) { return static_cast<double>(ns) / static_cast<double>(kSamples); };
        std::printf("%-36s %12.3f %12.3f %12.3f\n", arm.name.c_str(), perSample(arm.times.front()), perSample(arm.times[arm.times.size() / 2UZ]), perSample(arm.times.back()));
    }
    std::fflush(stdout);
    std::_Exit(0);
}
