#ifndef GNURADIO_CHANNEL_NONLINEARITY_HPP
#define GNURADIO_CHANNEL_NONLINEARITY_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

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
 *
 * The block drops the `phase_est` tag key. The Saleh AM/PM conversion rotates the carrier by an amount that depends on
 * the envelope.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct Nonlinearity : gr::Block<Nonlinearity<T>, gr::DroppedTagKeys<"phase_est">> {
    using Description = Doc<R""(
@brief Memoryless amplifier nonlinearity (Rapp SSPA or Saleh TWTA).

Acts on the envelope only, so it is stateless and chunk safe by construction. `saturation` is in the same
units as the input amplitude; `input_backoff_db` pre-scales so an operating point can be stated directly.

rapp models a solid-state amplifier and saleh a traveling-wave tube with AM/PM. A positive `input_backoff_db` backs the
drive off; a larger `smoothness` is a harder limit.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<NonlinearityModel, Visible, Doc<"amplifier model">>                   model            = NonlinearityModel::Rapp;
        Annotated<double, Visible, Unit<"dB">, Doc<"input backoff">>                    input_backoff_db = 0.0;
        Annotated<double, Visible, Doc<"Rapp saturation amplitude">, Limits<Over<0.0>>> saturation       = 1.0;
        Annotated<double, Visible, Doc<"Rapp knee sharpness">, Limits<Over<0.0>>>       smoothness       = 2.0;
        Annotated<double, Visible, Doc<"Saleh AM/AM numerator">>                        saleh_alpha_a    = 2.1587;
        Annotated<double, Visible, Doc<"Saleh AM/AM denominator">>                      saleh_beta_a     = 1.1517;
        Annotated<double, Visible, Unit<"rad">, Doc<"Saleh AM/PM numerator">>           saleh_alpha_p    = 4.0033;
        Annotated<double, Visible, Doc<"Saleh AM/PM denominator">>                      saleh_beta_p     = 9.1040;

        GR_MAKE_REFLECTABLE(Settings, model, input_backoff_db, saturation, smoothness, saleh_alpha_a, saleh_beta_a, saleh_alpha_p, saleh_beta_p);
    } settings;

    GR_MAKE_REFLECTABLE(Nonlinearity, in, out, settings);

    double _drive{};

    void applySettings(const Settings& s) { _drive = std::pow(10., -s.input_backoff_db / 20.); }

    /// The model's own equation, exposed so QA and consumers read the same curve the block runs.
    [[nodiscard]] std::complex<double> transferAt(double amplitude) const noexcept {
        const double driven = amplitude * _drive;
        if (driven == 0.) {
            return {0., 0.};
        }
        if (settings.model == NonlinearityModel::Rapp) {
            const double ratio = std::pow(driven / settings.saturation, 2. * settings.smoothness);
            return {driven / std::pow(1. + ratio, 1. / (2. * settings.smoothness)), 0.};
        }
        const double squared = driven * driven;
        return {settings.saleh_alpha_a * driven / (1. + settings.saleh_beta_a * squared), settings.saleh_alpha_p * squared / (1. + settings.saleh_beta_p * squared)};
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
        const std::complex<double> rotated = std::polar(gain, response.imag()) * std::complex<double>(sample);
        return T(static_cast<float>(rotated.real()), static_cast<float>(rotated.imag()));
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_NONLINEARITY_HPP
