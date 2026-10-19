#ifndef GNURADIO_IQ_CORRECTION_HPP
#define GNURADIO_IQ_CORRECTION_HPP

#include <array>
#include <cmath>
#include <complex>
#include <format>
#include <limits>
#include <numbers>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

namespace detail {

/// @brief The validator of `sample_rate`: true above 0.
inline constexpr auto kSampleRatePositive = [](float rate) { return rate > 0.f; };

/// @brief The validator of `tau`: true above 0.
inline constexpr auto kTauPositive = [](double seconds) { return seconds > 0.0; };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::IqSwap, [T], [std::complex<float>])

template<typename T>
requires(gr::meta::complex_like<T>)
struct IqSwap : Block<IqSwap<T>> {
    using Description = Doc<R""(
@brief Exchanges the real and imaginary components of every sample, mirroring the spectrum about DC.

A receiver whose I and Q are exchanged in the hardware or the driver sees every signal at `-f` instead of `+f`, with
every sideband inverted. This block puts them back. The swap equals conjugation up to a constant phase,
`swap(x) = j * conj(x)`. A receive chain cannot tell a fixed 90-degree rotation from any other constant phase. The block
exchanges the components and does not conjugate. The output then matches a swapped reference sample for sample, and
not only up to a phase.

`enabled = false` is a bit-exact pass-through and is the default. The block stays in the graph in both states.
`processOne` is `const` and writes no state.

Every input tag key passes through at its own offset.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"exchange I and Q">, Visible> enabled = false;

    GR_MAKE_REFLECTABLE(IqSwap, in, out, enabled);

    [[nodiscard]] constexpr T processOne(T sample) const noexcept { return enabled ? T(sample.imag(), sample.real()) : sample; }
};

GR_REGISTER_BLOCK(gr::blocks::basic::DcOffsetCorrect, [T], [ std::complex<float>, float ])

/**
 * `sample_rate` and `tau` take positive finite values. The framework refuses any other value, NaN included, before the
 * block sees it. It refuses that key alone and applies the rest of the application. The member and `settings().get()`
 * keep the accepted value. A refused value from a tag is reported in a plain message, and the run goes on. A refused
 * staged value or Set message is reported in an error message, which ends a run whose messages nothing reads.
 * `settings().set()` throws at the call and stores nothing.
 *
 * The block computes on the `enabled`, `sample_rate` and `tau` it last accepted, starting at the defaults, and keeps
 * them apart from its members. It refuses a product of `tau` and `sample_rate` that puts `alpha` outside (0, 1], and
 * checks the product with the accepted value of a setting the application leaves alone. This refusal covers the whole
 * application and changes nothing the block computes. The refused values stay in their members, and
 * `settings().get()` returns them. The block ignores each one until an application changes that setting again. The
 * framework drops a staged or tagged value equal to its member before the block sees it. A refused value sent again
 * alone therefore passes with no refusal and changes nothing.
 *
 * `alpha` and `corner_hz` follow from the accepted `tau` and `sample_rate`. The block compares a written `alpha` or
 * `corner_hz` with the value it derives from the application's `tau` and `sample_rate`. It accepts an equal value and
 * refuses the application on any other, as it refuses a product. It writes both again at each application it accepts.
 * Between a refused write and the next accepted application, `settings().get()` returns the written value.
 *
 * `reset()` sets the estimate to zero. The framework calls it at the lifecycle's reset and at a reset to defaults,
 * after the defaults are applied. After either one the block tracks the offset from zero, as at the start of a stream.
 */
template<typename T>
requires(gr::meta::complex_like<T> || std::floating_point<T>)
struct DcOffsetCorrect : Block<DcOffsetCorrect<T>> {
    using Description = Doc<R""(
@brief Subtracts a single-pole estimate of the mean, removing a converter's bias and an LO leaking into its own mixer.

On a real stream the estimator removes the carrier level of a demodulated envelope. An AM detector's magnitude carries
that mean. The block then tracks one component. `dcEstimate()` reports it in the real part, with the imaginary part
zero.

`gr::blocks::filter::DcBlocker` removes DC with a linear-phase notch and a delay of many samples. This block subtracts
a single-pole estimate from each sample as it arrives. The estimate is `double` while the ports are `float`. In
`float32` the complement `1 - alpha` rounds to exactly `1.0f` at and above `tau*fs = 2^25`, and the recursion becomes a
pure integrator.

A sample with a NaN or infinite component leaves the estimate unchanged. The block outputs that sample less the held
estimate.

The block is 1:1. Every input tag key passes through at its own offset, and `sample_rate` carries this block's value.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"subtract the tracked offset">, Visible>                                                                                                  enabled     = false;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate, written into a passing tag">, Limits<0.f, std::numeric_limits<float>::max(), detail::kSampleRatePositive>> sample_rate = 96000.f;
    Annotated<double, "tau", Unit<"s">, Doc<"time constant of the estimate, corner 1/(2*pi*tau) Hz">, Limits<0.0, std::numeric_limits<double>::max(), detail::kTauPositive>> tau         = 1.0;
    Annotated<double, "alpha", Doc<"derived smoothing coefficient 1/(1 + tau*sample_rate)">>                                                                                 alpha       = 0.0;
    Annotated<double, "corner_hz", Unit<"Hz">, Doc<"derived highpass -3 dB point, alpha*sample_rate/(2*pi)">>                                                                corner_hz   = 0.0;

    GR_MAKE_REFLECTABLE(DcOffsetCorrect, in, out, enabled, sample_rate, tau, alpha, corner_hz);

    bool   _enabled       = enabled.value;     ///< the accepted `enabled`
    float  _sampleRate    = sample_rate.value; ///< the accepted `sample_rate`
    double _tau           = tau.value;         ///< the accepted `tau`
    double _alpha         = 0.0;
    double _oneMinusAlpha = 1.0;
    double _real          = 0.0;
    double _imag          = 0.0;

    /// The framework calls settingsChanged() only for a batch that changes a value. The constructor derives `alpha`,
    /// the pole `1 - alpha` and `corner_hz` from the defaults. A block built at its declared defaults starts with the
    /// corner its settings describe.
    explicit DcOffsetCorrect(property_map init = {}) : Block<DcOffsetCorrect<T>>(std::move(init)) { accept(_enabled, _sampleRate, _tau, 1.0 / (1.0 + _tau * static_cast<double>(_sampleRate))); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const float  rate      = newSettings.contains("sample_rate") ? sample_rate.value : _sampleRate;
        const double seconds   = newSettings.contains("tau") ? tau.value : _tau;
        const double nextAlpha = 1.0 / (1.0 + seconds * static_cast<double>(rate));
        if (!(nextAlpha > 0.0) || nextAlpha > 1.0) {
            throw gr::exception(std::format("alpha must lie in (0, 1], got {} from tau {} and sample_rate {}", nextAlpha, seconds, rate));
        }
        const double nextCorner = cornerOf(nextAlpha, rate);
        if ((newSettings.contains("alpha") && alpha.value != nextAlpha) || (newSettings.contains("corner_hz") && corner_hz.value != nextCorner)) {
            throw gr::exception(std::format("alpha and corner_hz follow from tau and sample_rate, which give {} and {}", nextAlpha, nextCorner));
        }
        accept(newSettings.contains("enabled") ? enabled.value : _enabled, rate, seconds, nextAlpha);
    }

    void reset() {
        _real = 0.0;
        _imag = 0.0;
    }

    /// The tracked DC offset. Call it on the thread that runs the block, or while the block is stopped.
    [[nodiscard]] std::complex<double> dcEstimate() const noexcept { return {_real, _imag}; }

    /// Each component of the estimate runs its own recursion. A component that falls below the smallest normal
    /// `double` is set to zero. A silent input decays the estimate toward zero, and in the subnormal range each
    /// operation costs a microcode assist. A sample with a NaN or infinite component leaves both components unchanged.
    [[nodiscard]] constexpr T processOne(T sample) noexcept {
        if (!_enabled) {
            return sample;
        }
        if constexpr (meta::complex_like<T>) {
            using Real            = typename T::value_type;
            const double real     = static_cast<double>(sample.real());
            const double imag     = static_cast<double>(sample.imag());
            const double nextReal = _alpha * real + _oneMinusAlpha * _real;
            const double nextImag = _alpha * imag + _oneMinusAlpha * _imag;
            if (std::isfinite(nextReal) && std::isfinite(nextImag)) {
                _real = flushed(nextReal);
                _imag = flushed(nextImag);
            }
            return T(static_cast<Real>(real - _real), static_cast<Real>(imag - _imag));
        } else {
            const double real     = static_cast<double>(sample);
            const double nextReal = _alpha * real + _oneMinusAlpha * _real;
            if (std::isfinite(nextReal)) {
                _real = flushed(nextReal);
            }
            return static_cast<T>(real - _real);
        }
    }

private:
    [[nodiscard]] static constexpr double flushed(double value) noexcept { return std::abs(value) < std::numeric_limits<double>::min() ? 0.0 : value; }

    [[nodiscard]] static constexpr double cornerOf(double nextAlpha, float rate) noexcept { return nextAlpha * static_cast<double>(rate) / (2.0 * std::numbers::pi); }

    void accept(bool on, float rate, double seconds, double nextAlpha) {
        _enabled       = on;
        _sampleRate    = rate;
        _tau           = seconds;
        _alpha         = nextAlpha;
        _oneMinusAlpha = 1.0 - nextAlpha;
        alpha          = nextAlpha;
        corner_hz      = cornerOf(nextAlpha, rate);
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_IQ_CORRECTION_HPP
