#include "Interleaved.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/basic/IqCorrection.hpp>

namespace {

using gr::blocks::basic::DcOffsetCorrect;
using gr::blocks::basic::IqSwap;
using gr::blocks::basic::bench::Arm;

using CF = std::complex<float>;

constexpr std::size_t kSamples = 1UZ << 22;
constexpr std::size_t kRepeats = 7UZ;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

} // namespace

int main() {
    std::vector<CF> x(kSamples);
    for (std::size_t i = 0UZ; i < x.size(); ++i) { // an offset tone, so the tracker has something to track
        const double phase = 2.0 * std::numbers::pi * 0.031 * static_cast<double>(i);
        x[i]               = CF(static_cast<float>(0.7 * std::cos(phase) + 0.05), static_cast<float>(0.7 * std::sin(phase) - 0.02));
    }
    std::vector<CF> y(kSamples);

    IqSwap<CF>          swapOff = make<IqSwap<CF>>({});
    IqSwap<CF>          swapOn  = make<IqSwap<CF>>({{"enabled", true}});
    DcOffsetCorrect<CF> dcOff   = make<DcOffsetCorrect<CF>>({{"sample_rate", 25e6f}, {"tau", 1.0}});
    DcOffsetCorrect<CF> dcOn    = make<DcOffsetCorrect<CF>>({{"enabled", true}, {"sample_rate", 25e6f}, {"tau", 1.0}});

    const auto sweep = [&x, &y](auto& block) {
        for (std::size_t i = 0UZ; i < kSamples; ++i) {
            y[i] = block.processOne(x[i]);
        }
        return static_cast<double>(y[kSamples / 2UZ].real());
    };

    std::vector<Arm> arms;
    arms.emplace_back("plain span copy, the 1:1 floor", kSamples, [&x, &y] {
        std::copy_n(x.begin(), kSamples, y.begin());
        return static_cast<double>(y[kSamples / 2UZ].real());
    });
    arms.emplace_back("IqSwap disabled", kSamples, [&sweep, &swapOff] { return sweep(swapOff); });
    arms.emplace_back("IqSwap enabled", kSamples, [&sweep, &swapOn] { return sweep(swapOn); });
    arms.emplace_back("DcOffsetCorrect disabled", kSamples, [&sweep, &dcOff] { return sweep(dcOff); });
    arms.emplace_back("DcOffsetCorrect enabled", kSamples, [&sweep, &dcOn] { return sweep(dcOn); });

    gr::blocks::basic::bench::report(std::span<Arm>(arms), kRepeats);
}
