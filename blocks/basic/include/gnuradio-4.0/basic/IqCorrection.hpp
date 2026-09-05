#ifndef GNURADIO_IQ_CORRECTION_HPP
#define GNURADIO_IQ_CORRECTION_HPP

#include <array>
#include <cmath>
#include <complex>
#include <format>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::IqSwap, [T], [std::complex<float>])

template<typename T>
requires(gr::meta::complex_like<T>)
struct IqSwap : Block<IqSwap<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Exchanges the real and imaginary components of every sample, mirroring the spectrum about DC.

A receiver whose I and Q have been exchanged somewhere in the hardware or the driver sees every signal at `-f` instead
of `+f`, and every sideband inverted. This puts them back. The swap is conjugation up to a constant phase,
`swap(x) = j * conj(x)`, and no downstream block in a receive chain can tell a fixed 90-degree rotation from any other
constant phase. The component exchange is chosen over the conjugation so that the output matches a swapped reference
sample for sample rather than up to a phase.

`enabled = false` is a bit-exact pass-through and is the default. The block stays in the graph either way: removing and
reinserting it on a toggle means a full receiver reconfiguration and a device restart. `processOne` is
`const`, the block writing no state, so it never has to be the member that ends a fused run.

The block is 1:1, so every input tag key passes through at its own offset. It declares no reserved key of its own and
so substitutes nothing.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"false is a bit-exact pass-through">, Visible> enabled = false;

    GR_MAKE_REFLECTABLE(IqSwap, in, out, enabled);

    [[nodiscard]] constexpr T processOne(T sample) const noexcept { return enabled ? T(sample.imag(), sample.real()) : sample; }
};

GR_REGISTER_BLOCK(gr::blocks::basic::DcOffsetCorrect, [T], [ std::complex<float>, float ])

template<typename T>
requires(gr::meta::complex_like<T> || std::floating_point<T>)
struct DcOffsetCorrect : Block<DcOffsetCorrect<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Subtracts a single-pole estimate of the mean, removing a converter's bias and an LO leaking into its own mixer.

On a real stream the same estimator removes a demodulated envelope's carrier level — the mean an AM detector's
magnitude carries by construction — tracking only the one component; `dcEstimate()` then reports it in the real part
with the imaginary part zero.

`gr::blocks::filter::DcBlocker` is the designed notch; this one tracks a property of the hardware. The estimate is
`double` while the ports are `float` because in `float32` the complement `1 - alpha` rounds to exactly `1.0f` at and
above `tau*fs = 2^25` and the recursion becomes a pure integrator. `processOne` is not `const`, so this block must be
the last member of any composed run it appears in.

The block is 1:1, so every input tag key passes through at its own offset, `sample_rate` carrying this block's value.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"false is a bit-exact pass-through; the estimate freezes and survives">, Visible>  enabled     = false;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate; this block substitutes it into a passing tag">>     sample_rate = 96000.f;
    Annotated<double, "tau", Unit<"s">, Doc<"time constant of the estimate; the corner is 1/(2*pi*tau) Hz">>          tau         = 1.0;
    Annotated<double, "alpha", Doc<"observable: 1/(1 + tau*sample_rate), re-derived on every settings change">>       alpha       = 0.0;
    Annotated<double, "corner_hz", Unit<"Hz">, Doc<"observable: the highpass -3 dB point, alpha*sample_rate/(2*pi)">> corner_hz   = 0.0;

    GR_MAKE_REFLECTABLE(DcOffsetCorrect, in, out, enabled, sample_rate, tau, alpha, corner_hz);

    double _alpha         = 0.0;
    double _oneMinusAlpha = 1.0;
    double _real          = 0.0;
    double _imag          = 0.0;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!(tau > 0.0) || !std::isfinite(tau)) {
            throw gr::exception(std::format("tau must be positive and finite, got {}", tau.value));
        }
        _alpha = 1.0 / (1.0 + tau * static_cast<double>(sample_rate));
        if (!(_alpha > 0.0) || _alpha > 1.0) {
            throw gr::exception(std::format("alpha must lie in (0, 1], got {} from tau {} and sample_rate {}", _alpha, tau.value, sample_rate.value));
        }
        _oneMinusAlpha = 1.0 - _alpha;
        alpha          = _alpha;
        corner_hz      = _alpha * static_cast<double>(sample_rate) / (2.0 * std::numbers::pi);
    }

    void reset() {
        _real = 0.0;
        _imag = 0.0;
    }

    /// @brief The tracked DC offset, for a caller to poll off the sample path.
    [[nodiscard]] std::complex<double> dcEstimate() const noexcept { return {_real, _imag}; }

    [[nodiscard]] constexpr T processOne(T sample) noexcept {
        if (!enabled) {
            return sample;
        }
        if constexpr (meta::complex_like<T>) {
            using Real        = typename T::value_type;
            const double real = static_cast<double>(sample.real());
            const double imag = static_cast<double>(sample.imag());
            _real             = _alpha * real + _oneMinusAlpha * _real;
            _imag             = _alpha * imag + _oneMinusAlpha * _imag;
            return T(static_cast<Real>(real - _real), static_cast<Real>(imag - _imag));
        } else {
            const double real = static_cast<double>(sample);
            _real             = _alpha * real + _oneMinusAlpha * _real;
            return static_cast<T>(real - _real);
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_IQ_CORRECTION_HPP
