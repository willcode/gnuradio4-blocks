#ifndef GNURADIO_CHANNEL_CONVERTER_HPP
#define GNURADIO_CHANNEL_CONVERTER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <magic_enum.hpp>

namespace gr::blocks::channel {

enum class NonlinearityModel : int {
    Rapp,  ///< solid-state amplifier: soft envelope limiting, no AM/PM
    Saleh, ///< traveling-wave tube: envelope compression with an amplitude-dependent phase shift
};

GR_REGISTER_BLOCK(gr::blocks::channel::Nonlinearity, [T], [std::complex<float>])

/**
 * @brief Memoryless power-amplifier nonlinearity, Rapp or Saleh.
 *
 * Rapp (SSPA), phase untouched:
 *
 *     A_out = A / (1 + (A/A_sat)^(2p))^(1/(2p))
 *
 * Saleh (TWTA), which compresses the envelope and rotates by an amount that depends on it:
 *
 *     AM/AM = alpha_a*A / (1 + beta_a*A^2)      AM/PM = alpha_p*A^2 / (1 + beta_p*A^2)   [rad]
 *
 * Both are closed forms, so the equation is the oracle: QA checks the block against it at a table of
 * amplitudes. `input_backoff_db` pre-scales the input so an operating point states itself.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct Nonlinearity : gr::Block<Nonlinearity<T>> {
    using Description = Doc<R""(
@brief Memoryless amplifier nonlinearity (Rapp SSPA or Saleh TWTA).

Acts on the envelope only, so it is stateless and chunk safe by construction. `saturation` is in the same
units as the input amplitude; `input_backoff_db` pre-scales so an operating point can be stated directly.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<NonlinearityModel, "model", Visible, Doc<"rapp (SSPA, no AM/PM) or saleh (TWTA, with AM/PM)">>         model            = NonlinearityModel::Rapp;
    Annotated<double, "input_backoff_db", Visible, Unit<"dB">, Doc<"input pre-scale; positive backs the drive off">> input_backoff_db = 0.0;
    Annotated<double, "saturation", Visible, Doc<"Rapp saturation amplitude A_sat">>                                 saturation       = 1.0;
    Annotated<double, "smoothness", Visible, Doc<"Rapp knee sharpness p; larger is a harder limit">>                 smoothness       = 2.0;
    Annotated<double, "saleh_alpha_a", Visible, Doc<"Saleh AM/AM numerator">>                                        saleh_alpha_a    = 2.1587;
    Annotated<double, "saleh_beta_a", Visible, Doc<"Saleh AM/AM denominator">>                                       saleh_beta_a     = 1.1517;
    Annotated<double, "saleh_alpha_p", Visible, Unit<"rad">, Doc<"Saleh AM/PM numerator">>                           saleh_alpha_p    = 4.0033;
    Annotated<double, "saleh_beta_p", Visible, Doc<"Saleh AM/PM denominator">>                                       saleh_beta_p     = 9.1040;

    GR_MAKE_REFLECTABLE(Nonlinearity, in, out, model, input_backoff_db, saturation, smoothness, saleh_alpha_a, saleh_beta_a, saleh_alpha_p, saleh_beta_p);

    double _drive{1.};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (saturation <= 0.0) {
            throw gr::exception(std::format("Nonlinearity: 'saturation' must be positive, got {}", saturation.value));
        }
        if (smoothness <= 0.0) {
            throw gr::exception(std::format("Nonlinearity: 'smoothness' must be positive, got {}", smoothness.value));
        }
        _drive = std::pow(10., -input_backoff_db / 20.);
    }

    /// The model's own equation, exposed so QA and consumers read the same curve the block runs.
    [[nodiscard]] std::complex<double> transferAt(double amplitude) const noexcept {
        const double driven = amplitude * _drive;
        if (driven == 0.) {
            return {0., 0.};
        }
        if (model == NonlinearityModel::Rapp) {
            const double ratio = std::pow(driven / saturation, 2. * smoothness);
            return {driven / std::pow(1. + ratio, 1. / (2. * smoothness)), 0.};
        }
        const double squared = driven * driven;
        return {saleh_alpha_a * driven / (1. + saleh_beta_a * squared), saleh_alpha_p * squared / (1. + saleh_beta_p * squared)};
    }

    [[nodiscard]] T processOne(T sample) const noexcept {
        const double amplitude = std::hypot(static_cast<double>(sample.real()), static_cast<double>(sample.imag()));
        if (amplitude == 0.) {
            return T(0.f, 0.f); // a zero envelope has no phase to preserve, and passes through exactly
        }
        const std::complex<double> response = transferAt(amplitude);
        const double               gain     = response.real() / amplitude;
        if (response.imag() == 0.) {
            return T(static_cast<float>(static_cast<double>(sample.real()) * gain), static_cast<float>(static_cast<double>(sample.imag()) * gain));
        }
        const std::complex<double> rotated = std::polar(gain, response.imag()) * std::complex<double>(sample.real(), sample.imag());
        return T(static_cast<float>(rotated.real()), static_cast<float>(rotated.imag()));
    }
};

GR_REGISTER_BLOCK(gr::blocks::channel::Quantizer, [T], [ float, std::complex<float> ])

/**
 * @brief Mid-tread converter quantization with saturation.
 *
 *     y = clamp(round(x/delta), -2^(B-1), 2^(B-1) - 1) * delta,   delta = full_scale * 2^(1-B)
 *
 * Complex input quantizes each axis independently, which is what a pair of real converters does. Mid-tread
 * means zero is a code, so a zero input survives exactly. Idempotent by construction: quantizing an already
 * quantized value lands on the same code.
 */
template<typename T>
requires(std::is_same_v<T, float> || std::is_same_v<T, std::complex<float>>)
struct Quantizer : gr::Block<Quantizer<T>> {
    using Description = Doc<R""(
@brief Mid-tread quantization with saturation, one converter per axis.

`bits` counts the whole word including its sign, so 8 bits gives codes -128..127, and values beyond
`full_scale` land exactly on the rails.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "bits", Visible, Doc<"converter word length including the sign bit, 2-24">> bits       = 12U;
    Annotated<double, "full_scale", Visible, Doc<"amplitude mapping to the positive rail">>           full_scale = 1.0;

    GR_MAKE_REFLECTABLE(Quantizer, in, out, bits, full_scale);

    double _delta{1.};
    double _lowestCode{0.};
    double _highestCode{0.};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (bits < 2U || bits > 24U) {
            throw gr::exception(std::format("Quantizer: 'bits' must be in [2, 24], got {}", bits.value));
        }
        if (full_scale <= 0.0) {
            throw gr::exception(std::format("Quantizer: 'full_scale' must be positive, got {}", full_scale.value));
        }
        _delta       = full_scale * std::pow(2., 1. - static_cast<double>(bits.value));
        _highestCode = std::pow(2., static_cast<double>(bits.value) - 1.) - 1.;
        _lowestCode  = -std::pow(2., static_cast<double>(bits.value) - 1.);
    }

    [[nodiscard]] float quantize(float value) const noexcept {
        const double code = std::clamp(std::round(static_cast<double>(value) / _delta), _lowestCode, _highestCode);
        return static_cast<float>(code * _delta);
    }

    [[nodiscard]] T processOne(T sample) const noexcept {
        if constexpr (std::is_same_v<T, std::complex<float>>) {
            return T(quantize(sample.real()), quantize(sample.imag()));
        } else {
            return quantize(sample);
        }
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_CONVERTER_HPP
