#ifndef GNURADIO_LTE_TEST_DOWNLINK_SCENE_HPP
#define GNURADIO_LTE_TEST_DOWNLINK_SCENE_HPP

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <vector>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/algorithm/lte/SyncSignals.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>

namespace gr::test::lte {

/**
 * @brief A synthetic E-UTRA downlink on the central six resource blocks at 1.92 MS/s, with nothing above the two
 * synchronization signals.
 *
 * Every OFDM symbol carries seeded random QPSK of unit power on all 72 subcarriers (-36..-1 and +1..+36, DC
 * empty), except that the primary and secondary synchronization symbols carry their sequences on the central 62
 * with the outer ten still QPSK. Cyclic prefixes follow the geometry asked for, so both frame structures and both
 * prefix types come out of the same code, and the mean per-sample power is exactly one whatever the structure —
 * which is what makes a stated per-sample signal-to-noise ratio a noise power a channel block can be set to
 * directly.
 *
 * A timing offset is a run of QPSK-filled symbols before the first frame, so the frame boundary falls anywhere in
 * the stream rather than only on a symbol edge of the generator's own making.
 */
struct SceneConfig {
    std::uint32_t         nId1{0U};
    std::uint32_t         nId2{0U};
    gr::lte::DuplexMode   duplex{gr::lte::DuplexMode::Fdd};
    gr::lte::CyclicPrefix cyclicPrefix{gr::lte::CyclicPrefix::Normal};
    std::size_t           frames{2UZ};       ///< 10 ms radio frames after the timing offset
    std::size_t           timingOffset{0UZ}; ///< samples of synchronization-free filler before the first frame
    std::uint64_t         seed{1ULL};
};

/// A generated stream together with what a correct identification of it says.
struct Scene {
    std::vector<std::complex<float>> samples;
    std::vector<std::size_t>         pss{};        ///< absolute first useful sample of each primary symbol
    std::vector<std::size_t>         frameStart{}; ///< absolute first sample of the frame each of those belongs to
    std::vector<std::uint32_t>       halfFrame{};  ///< 0 or 1 for each of those
    std::size_t                      origin{0UZ};  ///< absolute first sample of the first frame
};

namespace detail {

/// One OFDM symbol's worth of the scene, built in frequency and emitted with its cyclic prefix.
class SymbolWriter {
public:
    SymbolWriter(const SceneConfig& config, gr::lte::FrameGeometry geometry) : _geometry(geometry), _rng(config.seed), _pss(gr::lte::pssSequence(config.nId2)), _sss0(gr::lte::sssSequence(config.nId1, config.nId2, 0U)), _sss5(gr::lte::sssSequence(config.nId1, config.nId2, 5U)) {}

    /// @return the index of the symbol's first useful sample within @p out
    std::size_t append(std::vector<std::complex<float>>& out, std::size_t symbolInSlot, bool primary, bool secondary, std::uint32_t half) {
        std::vector<std::complex<float>> bins(gr::lte::kSymbolSamples, std::complex<float>(0.f, 0.f));
        for (std::size_t n = 0UZ; n < 72UZ; ++n) { // subcarriers -36..-1 then +1..+36
            bins[n < 36UZ ? n + 92UZ : n - 35UZ] = qpsk();
        }
        if (primary) {
            for (std::size_t n = 0UZ; n < gr::lte::kSignalLength; ++n) {
                bins[gr::lte::subcarrierBin(n)] = std::complex<float>(static_cast<float>(_pss[n].real()), static_cast<float>(_pss[n].imag()));
            }
        }
        if (secondary) {
            const std::array<float, gr::lte::kSignalLength>& values = half == 0U ? _sss0 : _sss5;
            for (std::size_t n = 0UZ; n < gr::lte::kSignalLength; ++n) {
                bins[gr::lte::subcarrierBin(n)] = std::complex<float>(values[n], 0.f);
            }
        }

        std::vector<std::complex<float>> useful(gr::lte::kSymbolSamples);
        _inverse.compute(bins, useful);
        // 72 unit-power subcarriers through an unnormalized inverse transform put 128*72 of energy into 128
        // samples, so this is what makes the mean per-sample power one.
        const float scale = 1.f / std::sqrt(72.f);
        for (std::complex<float>& sample : useful) {
            sample *= scale;
        }

        const std::size_t prefix = symbolInSlot == 0UZ ? _geometry.firstPrefix() : _geometry.otherPrefix();
        for (std::size_t n = 0UZ; n < prefix; ++n) {
            out.push_back(useful[gr::lte::kSymbolSamples - prefix + n]);
        }
        const std::size_t at = out.size();
        out.insert(out.end(), useful.begin(), useful.end());
        return at;
    }

private:
    [[nodiscard]] std::complex<float> qpsk() {
        const std::uint64_t bits  = _rng();
        const float         scale = 1.f / std::numbers::sqrt2_v<float>;
        return {(bits & 1ULL) != 0ULL ? scale : -scale, (bits & 2ULL) != 0ULL ? scale : -scale};
    }

    gr::lte::FrameGeometry                                                                           _geometry;
    gr::rng::Xoshiro256pp                                                                            _rng;
    gr::algorithm::FFT<std::complex<float>, std::complex<float>, gr::algorithm::Direction::Backward> _inverse{};
    std::array<std::complex<double>, gr::lte::kSignalLength>                                         _pss;
    std::array<float, gr::lte::kSignalLength>                                                        _sss0;
    std::array<float, gr::lte::kSignalLength>                                                        _sss5;
};

} // namespace detail

/// @brief Generate the scene @p config describes.
[[nodiscard]] inline Scene makeDownlink(const SceneConfig& config) {
    const gr::lte::FrameGeometry geometry{config.duplex, config.cyclicPrefix};
    const std::size_t            symbols = geometry.symbolsPerSlot();
    const bool                   tdd     = config.duplex == gr::lte::DuplexMode::Tdd;
    // Paired spectrum carries the primary signal in the last symbol of slots 0 and 10 and the secondary one in the
    // symbol before it; unpaired spectrum carries the primary signal in symbol 2 of subframes 1 and 6 and the
    // secondary one in the last symbol of the subframe before.
    const std::size_t primarySlot   = tdd ? 2UZ : 0UZ;
    const std::size_t primarySymbol = tdd ? 2UZ : symbols - 1UZ;
    const std::size_t secondarySlot = tdd ? 1UZ : 0UZ;
    const std::size_t lastSymbol    = symbols - 1UZ;

    detail::SymbolWriter writer(config, geometry);
    Scene                scene;

    std::vector<std::complex<float>> filler;
    while (filler.size() < config.timingOffset) { // whole slots, so the prefixes stay on the grid the frame uses
        for (std::size_t symbol = 0UZ; symbol < symbols; ++symbol) {
            std::ignore = writer.append(filler, symbol, false, false, 0U);
        }
    }
    scene.samples.assign(filler.end() - static_cast<std::ptrdiff_t>(config.timingOffset), filler.end());
    scene.origin = config.timingOffset;

    for (std::size_t frame = 0UZ; frame < config.frames; ++frame) {
        const std::size_t frameAt = scene.samples.size();
        for (std::size_t slot = 0UZ; slot < 20UZ; ++slot) {
            const std::uint32_t half = slot < 10UZ ? 0U : 1U;
            for (std::size_t symbol = 0UZ; symbol < symbols; ++symbol) {
                const bool        primary   = (slot % 10UZ) == primarySlot && symbol == primarySymbol;
                const bool        secondary = (slot % 10UZ) == secondarySlot && symbol == (tdd ? lastSymbol : symbols - 2UZ);
                const std::size_t at        = writer.append(scene.samples, symbol, primary, secondary, half);
                if (primary) {
                    scene.pss.push_back(at);
                    scene.frameStart.push_back(frameAt);
                    scene.halfFrame.push_back(half);
                }
            }
        }
    }
    return scene;
}

/// @brief @p first with @p second added @p delay samples later at @p amplitude, the two cells one receiver sees.
/// The result is as long as @p first; @p second is truncated or zero-padded to fit.
[[nodiscard]] inline std::vector<std::complex<float>> mix(const std::vector<std::complex<float>>& first, const std::vector<std::complex<float>>& second, std::size_t delay, float amplitude) {
    std::vector<std::complex<float>> out = first;
    for (std::size_t n = 0UZ; n + delay < out.size() && n < second.size(); ++n) {
        out[n + delay] += second[n] * amplitude;
    }
    return out;
}

} // namespace gr::test::lte

#endif // GNURADIO_LTE_TEST_DOWNLINK_SCENE_HPP
