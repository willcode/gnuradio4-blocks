#ifndef GNURADIO_TESTING_ARRAY_SOURCE_HPP
#define GNURADIO_TESTING_ARRAY_SOURCE_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK(gr::blocks::testing::ArraySource, [T], [ std::complex<float>, std::complex<double> ])

template<typename T>
struct ArraySource : Block<ArraySource<T>> {
    using Description = Doc<R""(@brief Emits the signals of plane waves arriving at a uniform linear array of N elements.

Element `k` is output `out#k`, and `num_elements` sets N. Sample `n` of element `k` is
`sum over waves w of amplitudes[w] * exp(j * (2*pi*frequency*n + 2*pi*spacing*k*sin(angles[w])))` plus noise.
`spacing` is the distance between adjacent elements in wavelengths, and `angles` are in degrees from the
array's broadside. A wave at a positive angle reaches element `k` with a phase lead of
`2*pi*spacing*k*sin(angle)` over element 0. `frequency` is the tone's frequency as a fraction of the sample
rate. The noise is Gaussian with standard deviation `noise` in each of the real and the imaginary part,
independent per element and sample, from a generator seeded with `seed`. Every output ends after
`n_samples` samples. An empty `amplitudes` gives every wave the amplitude 1, and a list of another length
than `angles` ends the run with an error.
)"">;

    std::vector<PortOut<T>> out = std::vector<PortOut<T>>(2UZ);

    Annotated<gr::Size_t, "num_elements", Visible, Doc<"number of array elements N">, Limits<1U, 1024U>> num_elements = 2U;
    Annotated<double, "spacing", Visible, Doc<"element spacing in wavelengths">>                         spacing      = 0.5;
    Annotated<std::vector<double>, "angles", Visible, Doc<"arrival angle per wave in degrees">>          angles{};
    Annotated<std::vector<double>, "amplitudes", Visible, Doc<"amplitude per wave">>                     amplitudes{};
    Annotated<double, "noise", Visible, Doc<"noise standard deviation per real component">>              noise     = 0.0;
    Annotated<std::uint64_t, "seed", Doc<"seed of the noise generator">>                                 seed      = 1U;
    Annotated<gr::Size_t, "n_samples", Visible, Doc<"samples per element">>                              n_samples = 1024U;
    Annotated<double, "frequency", Visible, Doc<"tone frequency as a fraction of the sample rate">>      frequency = 0.0;

    GR_MAKE_REFLECTABLE(ArraySource, out, num_elements, spacing, angles, amplitudes, noise, seed, n_samples, frequency);

    using real_type = gr::meta::fundamental_base_value_type_t<T>;

    bool                             _running = false;
    std::size_t                      _emitted = 0UZ;
    std::mt19937_64                  _engine{};
    std::normal_distribution<double> _normal{0.0, 1.0};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!_running && newSettings.contains("num_elements")) {
            out.resize(static_cast<std::size_t>(num_elements.value));
        }
    }

    void start() {
        _running = true;
        _emitted = 0UZ;
        _engine.seed(seed.value);
        _normal.reset();
    }

    void stop() { _running = false; }

    template<OutputSpanLike TOutSpan>
    [[nodiscard]] work::Status processBulk(std::span<TOutSpan>& outputs) {
        if (!amplitudes->empty() && amplitudes->size() != angles->size()) {
            this->emitErrorMessage("processBulk", std::format("amplitudes holds {} entries for {} angles", amplitudes->size(), angles->size()));
            return work::Status::ERROR;
        }
        const std::size_t total = static_cast<std::size_t>(n_samples.value);
        std::size_t       n     = total - _emitted;
        for (const auto& output : outputs) {
            n = std::min(n, output.size());
        }

        const std::size_t nWaves    = angles->size();
        const std::size_t nElements = outputs.size();
        const double      twoPi     = 2.0 * std::numbers::pi;
        for (std::size_t i = 0UZ; i < n; ++i) {
            const double toneArg = twoPi * frequency.value * static_cast<double>(_emitted + i);
            for (std::size_t k = 0UZ; k < nElements; ++k) {
                std::complex<double> sample{};
                for (std::size_t w = 0UZ; w < nWaves; ++w) {
                    const double amplitude = amplitudes->empty() ? 1.0 : amplitudes.value[w];
                    const double sinTheta  = std::sin(angles.value[w] * std::numbers::pi / 180.0);
                    sample += amplitude * std::polar(1.0, toneArg + twoPi * spacing.value * static_cast<double>(k) * sinTheta);
                }
                if (noise.value > 0.0) {
                    const double re = _normal(_engine);
                    const double im = _normal(_engine);
                    sample += noise.value * std::complex<double>(re, im);
                }
                outputs[k][i] = T{static_cast<real_type>(sample.real()), static_cast<real_type>(sample.imag())};
            }
        }
        for (auto& output : outputs) {
            output.publish(n);
        }
        _emitted += n;
        return _emitted >= total ? work::Status::DONE : work::Status::OK;
    }
};

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_ARRAY_SOURCE_HPP
