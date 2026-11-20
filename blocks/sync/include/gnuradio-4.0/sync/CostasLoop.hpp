#ifndef GNURADIO_SYNC_COSTAS_LOOP_HPP
#define GNURADIO_SYNC_COSTAS_LOOP_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <numbers>
#include <optional>
#include <tuple>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/sync/ControlLoop.hpp>

#include <gnuradio-4.0/sync/LoopCommon.hpp>

namespace gr::blocks::sync {

/// @brief The 8PSK cross weight, `sqrt(2) - 1`, which is exactly `tan(pi/8)`.
inline constexpr double kOrder8CrossWeight = std::numbers::sqrt2 - 1.0;

/// @brief Hard decision: `+1` above zero, `-1` at zero and below.
template<std::floating_point T>
[[nodiscard]] inline constexpr T slice(T u) noexcept {
    return u > T{0} ? T{1} : T{-1};
}

/// @brief `tanh` to within `2.352e-2`, branch free and table free.
template<std::floating_point T>
[[nodiscard]] inline constexpr T softSlice(T u) noexcept {
    const T t = std::clamp(u, T{-4}, T{4});
    return t * (T{27} + t * t) / (T{27} + T{9} * t * t);
}

/**
 * @brief The decision-directed phase error for a constellation with `Order`-fold rotational symmetry.
 *
 * Write `y` for the derotated sample and `phi` for its residual phase. Each law removes the modulation by exploiting
 * the symmetry, so what is left is an odd function of `phi` with period `2*pi/Order`. Order 2 (BPSK) is
 * `e = Re{y} * Im{y}`; order 4 (QPSK) is `e = sgn(Re{y})*Im{y} - sgn(Im{y})*Re{y}`; order 8 (8PSK) is the same pair of
 * terms with the smaller of `|Re{y}|` and `|Im{y}|` weighted by `K = tan(pi/8) = sqrt(2) - 1`, which is what places a
 * zero on each of the eight symbols.
 *
 * `Kdet` is `A^2` at order 2 and `A*sqrt(2)` and `A*1.08239220` at orders 4 and 8, so the amplitude has to be known.
 * At order 8 the constellation must sit at odd multiples of `pi/8`: rotated onto the axes the detector is
 * discontinuous at zero error and has the wrong sign there, and the loop diverges rather than tracks.
 *
 * With `Soft`, each decision becomes `tanh(snr*u)` with `snr = |y|^2 / noisePower`, which weights an uncertain
 * decision down and costs the loop less self-noise at low signal-to-noise ratio; as the ratio rises it degenerates to
 * the law above.
 */
template<std::size_t Order, bool Soft = false, std::floating_point T>
[[nodiscard]] inline T costasError(std::complex<T> y, T snr = T{0}) noexcept {
    static_assert(Order == 2UZ || Order == 4UZ || Order == 8UZ, "the Costas laws here are for 2, 4 and 8-fold symmetry");
    const T re = y.real();
    const T im = y.imag();

    if constexpr (Order == 2UZ) {
        return (Soft ? softSlice(snr * re) : re) * im;
    } else {
        const T decidedRe = Soft ? softSlice(snr * re) : slice(re);
        const T decidedIm = Soft ? softSlice(snr * im) : slice(im);
        if constexpr (Order == 4UZ) {
            return decidedRe * im - decidedIm * re;
        } else {
            const T weight = static_cast<T>(kOrder8CrossWeight);
            return std::abs(re) >= std::abs(im) ? decidedRe * im - decidedIm * re * weight : decidedRe * im * weight - decidedIm * re;
        }
    }
}

GR_REGISTER_BLOCK(gr::blocks::sync::CostasLoop)

struct CostasLoop : Block<CostasLoop, NoTagPropagation> {
    using Description = Doc<R""(
@brief A decision-directed carrier loop for BPSK, QPSK and 8PSK: tracks a suppressed carrier through the modulation.

The loop removes the modulation from the phase error by exploiting the constellation's rotational symmetry, so it can
lock to a signal carrying no carrier. Its pull-in range is roughly its own loop bandwidth, which is why it is normally
preceded by a frequency-locked loop such as FllBandEdge. The output is the derotated sample the detector already
formed.

An AGC is a precondition: every error law scales with the input amplitude, so `detector_gain` is the S-curve slope at
the amplitude the block will actually see - 1, 1.41421356 and 1.08239220 for the three orders at unit amplitude.
`order` selects the law at design time, and at order 8 the constellation must sit at odd multiples of pi/8, off the
axes, or the loop diverges rather than tracks. Frequencies are radians per sample on every port and parameter, and
`noise_bandwidth` is `Bn*T`, the closed loop's normalized one-sided noise bandwidth.

`noise_bandwidth` is the closed-loop one-sided noise bandwidth Bn*T. `order` is the constellation's rotational symmetry:
2 for BPSK, 4 QPSK, 8 8PSK. `detector_gain` is the measured S-curve slope Kdet, 1 for order 2 at unit amplitude.
`error_limit` clips the error signal symmetrically and engages at unit-amplitude QPSK. With `soft_decisions`, each
decision is weighted by tanh(snr*u) instead of sliced, against the linear `noise_power`.
)"">;

    PortIn<std::complex<float>>  in;
    PortOut<std::complex<float>> out;
    PortOut<float, Optional>     freq;
    PortOut<float, Optional>     phase;
    PortOut<float, Optional>     error;

    struct Settings {
        Annotated<double, Unit<"cycles/sample">, Doc<"loop noise bandwidth">, Limits<Over<0.0>>> noise_bandwidth = 0.01;
        Annotated<double, Doc<"damping factor">, Limits<Over<0.0>>>                              damping         = std::numbers::sqrt2 / 2.0;
        Annotated<gr::Size_t, Doc<"constellation symmetry order">, Limits<OneOf<2U, 4U, 8U>>>    order           = 4U;
        Annotated<double, Doc<"detector gain">, Limits<Over<0.0>>>                               detector_gain   = 1.0;
        Annotated<float, Unit<"rad/sample">, Doc<"maximum loop frequency">>                      max_frequency   = 1.0f;
        Annotated<float, Unit<"rad/sample">, Doc<"minimum loop frequency">>                      min_frequency   = -1.0f;
        // The clip bounds the loop gain when the input runs hot. At 1.0 it engages within the intended
        // operating range -- an order-4 S-curve peaks at A*sqrt(2) -- so a caller who wants the detector's
        // full characteristic raises it to twice that peak; qa_CostasLoop measures Kdet with it raised.
        Annotated<double, Doc<"error clip">, Limits<Over<0.0>>>  error_limit    = 1.0;
        Annotated<bool, Doc<"soft decisions">>                   soft_decisions = false;
        Annotated<double, Doc<"noise power">, Limits<Over<0.0>>> noise_power    = 1.0;

        GR_MAKE_REFLECTABLE(Settings, noise_bandwidth, damping, order, detector_gain, max_frequency, min_frequency, error_limit, soft_decisions, noise_power);
    } settings;

    GR_MAKE_REFLECTABLE(CostasLoop, in, out, freq, phase, error, settings);

    detail::CarrierLoop _loop{0.01, std::numbers::sqrt2 / 2.0, 1.0};
    std::uint64_t       _ignoredTags = 0ULL;

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkCarrierSettings(s); }

    void applySettings(const Settings& s) {
        detail::applyCarrierSettings(_loop, s.noise_bandwidth, s.damping, s.min_frequency, s.max_frequency);
        _loop.setDetectorGain(s.detector_gain);
    }

    void reset() {
        _loop.reset();
        _ignoredTags = 0ULL;
    }

    [[nodiscard]] std::uint64_t ignoredTagPayloads() const noexcept { return _ignoredTags; }

    [[nodiscard]] const detail::CarrierLoop& loop() const noexcept { return _loop; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& freqSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& errorSpan) {
        std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        nSamples             = detail::syncCount(nSamples, freqSpan);
        nSamples             = detail::syncCount(nSamples, phaseSpan);
        nSamples             = detail::syncCount(nSamples, errorSpan);

        // The law is selected once per call. Six bodies, each with its decisions inlined and nothing left to branch on.
        if (settings.soft_decisions) {
            switch (settings.order.value) {
            case 2U: return sweep<2UZ, true>(inSpan, outSpan, freqSpan, phaseSpan, errorSpan, nSamples);
            case 4U: return sweep<4UZ, true>(inSpan, outSpan, freqSpan, phaseSpan, errorSpan, nSamples);
            default: return sweep<8UZ, true>(inSpan, outSpan, freqSpan, phaseSpan, errorSpan, nSamples);
            }
        }
        switch (settings.order.value) {
        case 2U: return sweep<2UZ, false>(inSpan, outSpan, freqSpan, phaseSpan, errorSpan, nSamples);
        case 4U: return sweep<4UZ, false>(inSpan, outSpan, freqSpan, phaseSpan, errorSpan, nSamples);
        default: return sweep<8UZ, false>(inSpan, outSpan, freqSpan, phaseSpan, errorSpan, nSamples);
        }
    }

private:
    template<std::size_t Order, bool Soft>
    [[nodiscard]] work::Status sweep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& freqSpan, OutputSpanLike auto& phaseSpan, OutputSpanLike auto& errorSpan, std::size_t nSamples) {
        const bool  wantFreq  = freqSpan.isConnected;
        const bool  wantPhase = phaseSpan.isConnected;
        const bool  wantError = errorSpan.isConnected;
        const float limit     = static_cast<float>(settings.error_limit);
        const float noise     = static_cast<float>(settings.noise_power);

        const auto&       rawTags = inSpan.rawTags;
        const std::size_t nTags   = rawTags.size();
        std::size_t       cursor  = 0UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            while (cursor < nTags && rawTags[cursor].index <= inSpan.streamIndex + i) {
                if (rawTags[cursor].index >= inSpan.streamIndex) {
                    detail::routeTag(rawTags[cursor].map, _loop, _ignoredTags, [&](const property_map& forwarded) {
                        outSpan.publishTag(forwarded, i);
                        freqSpan.publishTag(forwarded, i);
                        phaseSpan.publishTag(forwarded, i);
                        errorSpan.publishTag(forwarded, i);
                    });
                }
                ++cursor;
            }

            const float               loopPhase = _loop.phase();
            const std::complex<float> y         = inSpan[i] * detail::derotator(loopPhase);
            const float               raw       = costasError<Order, Soft>(y, Soft ? std::norm(y) / noise : 0.f);
            const float               clipped   = std::clamp(raw, -limit, limit);
            std::ignore                         = _loop.step(clipped);

            outSpan[i] = y;
            if (wantFreq) {
                freqSpan[i] = _loop.frequency();
            }
            if (wantPhase) {
                phaseSpan[i] = loopPhase;
            }
            if (wantError) {
                errorSpan[i] = clipped;
            }
        }

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        freqSpan.publish(wantFreq ? nSamples : 0UZ);
        phaseSpan.publish(wantPhase ? nSamples : 0UZ);
        errorSpan.publish(wantError ? nSamples : 0UZ);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::sync

#endif // GNURADIO_SYNC_COSTAS_LOOP_HPP
