#ifndef GNURADIO_AGC_HPP
#define GNURADIO_AGC_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <limits>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::analog {

namespace detail {

// std::clamp branches on its unordered comparisons; the hot loop needs the branch-free minss/maxss form
template<std::floating_point TValue>
[[nodiscard]] constexpr TValue clampOrdered(TValue value, TValue low, TValue high) noexcept {
    const TValue lifted = value < low ? low : value;
    return lifted > high ? high : lifted;
}

// log2 of a positive normal float: exponent field plus the atanh series for the mantissa, centered on
// [1/sqrt(2), sqrt(2)) so four terms hold the error below 5e-8, with no libm call and no table
[[nodiscard]] inline float fastLog2(float value) noexcept {
    const auto    bits     = std::bit_cast<std::uint32_t>(value);
    std::int32_t  exponent = static_cast<std::int32_t>(bits >> 23U) - 127;
    std::uint32_t mantissa = (bits & 0x007FFFFFU) | 0x3F800000U;
    if (mantissa >= 0x3FB504F3U) {
        mantissa -= 0x00800000U;
        ++exponent;
    }

    const float centered = std::bit_cast<float>(mantissa);
    const float z        = (centered - 1.0f) / (centered + 1.0f);
    const float z2       = z * z;
    return static_cast<float>(exponent) + z * (2.885390082f + z2 * (0.961796694f + z2 * (0.577078016f + z2 * 0.412198583f)));
}

// 2^value: the integer part is assembled straight into the exponent field, the fraction is a degree-5
// polynomial on [-0.5, 0.5]; evaluated in float the pair stays under 2e-6 relative
[[nodiscard]] inline float fastExp2(float value) noexcept {
    const float        clamped = clampOrdered(value, -126.0f, 126.0f);
    const std::int32_t whole   = static_cast<std::int32_t>(clamped + (clamped >= 0.0f ? 0.5f : -0.5f));
    const float        f       = clamped - static_cast<float>(whole);
    const float        poly    = 1.000000052f + f * (0.693147200f + f * (0.240222117f + f * (0.055503407f + f * (0.009670763f + f * 0.001339528f))));
    return std::bit_cast<float>(static_cast<std::uint32_t>(whole + 127) << 23U) * poly;
}

inline constexpr double kLog2PerDecibel        = 0.16609640474436813; // log2(10) / 20
inline constexpr double kDecibelPerHalfLog2    = 3.010299956639812;   // 10 / log2(10), i.e. dB from log2 of a squared magnitude
inline constexpr float  kMagnitudeFloorSquared = 1e-24f;              // squared form of the 1e-12 (-240 dB) level floor

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::Agc, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct Agc : Block<Agc<T>> {
    using Description = Doc<R""(
@brief Holds the output level of a stream near a target by adapting a gain, with attack and decay in seconds.

The loop runs in the logarithmic domain, so its settling time is independent of both the signal level and the sample
rate. The gain in force for a sample is the one this sample's update produced. `update_decimation` runs the loop once
per K samples against a stream-absolute phase, so the output does not depend on how the scheduler chunked the stream.

The block is 1:1. Every input tag leaves at its own offset with the values it arrived with.

`sample_rate` can follow gr:sample_rate. Disabled, the block freezes the gain and applies it as a manual gain. The
attack time constant applies while the gain decreases, the decay time constant while it increases. Input levels below
`gate_threshold_db` freeze the gain. The gain is updated every `update_decimation` samples; 1 is the normative choice.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>          sample_rate       = 48000.f;
        Annotated<bool, Doc<"loop enabled">>                                         enabled           = true;
        Annotated<double, Unit<"dB">, Doc<"output target level">>                    reference_db      = -6.0;
        Annotated<double, Unit<"s">, Doc<"attack time constant">, Limits<Over<0.0>>> attack_s          = 0.005;
        Annotated<double, Unit<"s">, Doc<"decay time constant">, Limits<Over<0.0>>>  decay_s           = 0.500;
        Annotated<double, Unit<"dB">, Doc<"maximum gain">>                           max_gain_db       = 60.0;
        Annotated<double, Unit<"dB">, Doc<"minimum gain">>                           min_gain_db       = -20.0;
        Annotated<double, Unit<"dB">, Doc<"gate threshold">>                         gate_threshold_db = -150.0;
        Annotated<double, Unit<"dB">, Doc<"initial gain">>                           gain_db           = 0.0;
        Annotated<gr::Size_t, Doc<"samples per gain update">, Limits<Min<1U>>>       update_decimation = 1U;

        GR_MAKE_REFLECTABLE(Settings, sample_rate, enabled, reference_db, attack_s, decay_s, max_gain_db, min_gain_db, gate_threshold_db, gain_db, update_decimation);
    } settings;

    struct Status {
        Annotated<double, Unit<"dB">, Doc<"applied gain">> current_gain_db = 0.0;

        GR_MAKE_REFLECTABLE(Status, current_gain_db);
    } status;

    GR_MAKE_REFLECTABLE(Agc, in, out, settings, status);

    double      _gainDb{};
    double      _alphaAttack{};
    double      _alphaDecay{};
    float       _manualGain{1.f};
    float       _appliedGain{1.f};
    std::size_t _updatePhase{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.max_gain_db < s.min_gain_db) {
            return gr::Refused(s.max_gain_db, "must be >= min_gain_db");
        }
        return {};
    }

    // `gain_db` is the loop's starting point: a changed value restarts the loop there, and every apply clamps the
    // loop's gain into the range in force
    void applySettings(const Settings& s) {
        _alphaAttack = loopCoefficient(s.attack_s);
        _alphaDecay  = loopCoefficient(s.decay_s);
        if (s.gain_db.changed()) {
            _gainDb = s.gain_db;
        }
        _gainDb                = std::clamp(_gainDb, s.min_gain_db.value, s.max_gain_db.value);
        status.current_gain_db = _gainDb;
        _manualGain            = static_cast<float>(std::pow(10.0, _gainDb / 20.0));
    }

    void reset() { _updatePhase = 0UZ; }

    static constexpr std::size_t kSubBlock = 512UZ;

    // Advances the gain in dB through `count` squared-magnitude levels given as log2, writing the exponent of the
    // linear gain for each. With g' = (1 - a) g + a (reference - level) both candidate updates are one fma, and the
    // sign of the error is a compare of g against reference - level, so the serial chain is fma, select, max, min;
    // the coefficients depend only on the level and are computed ahead without a dependence between iterations.
    [[nodiscard]] double runLoop(double gainDb, const float* level, float* exponent, std::size_t count) const noexcept {
        const double reference = settings.reference_db;
        const double gate      = settings.gate_threshold_db;
        const double minGain   = settings.min_gain_db;
        const double maxGain   = settings.max_gain_db;
        const double attack    = _alphaAttack;
        const double decay     = _alphaDecay;

        alignas(64) double keepAttack[kSubBlock];
        alignas(64) double pullAttack[kSubBlock];
        alignas(64) double keepDecay[kSubBlock];
        alignas(64) double pullDecay[kSubBlock];
        alignas(64) double threshold[kSubBlock];
        for (std::size_t i = 0UZ; i < count; ++i) {
            const double levelDb = detail::kDecibelPerHalfLog2 * static_cast<double>(level[i]);
            const double open    = levelDb < gate ? 0.0 : 1.0;
            const double target  = reference - levelDb;
            threshold[i]         = target;
            keepAttack[i]        = 1.0 - open * attack;
            pullAttack[i]        = open * attack * target;
            keepDecay[i]         = 1.0 - open * decay;
            pullDecay[i]         = open * decay * target;
        }
        for (std::size_t i = 0UZ; i < count; ++i) {
            const double attacked = std::fma(gainDb, keepAttack[i], pullAttack[i]);
            const double decayed  = std::fma(gainDb, keepDecay[i], pullDecay[i]);
            gainDb                = detail::clampOrdered(gainDb > threshold[i] ? attacked : decayed, minGain, maxGain);
            exponent[i]           = static_cast<float>(gainDb * detail::kLog2PerDecibel);
        }
        return gainDb;
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());

        if (!settings.enabled) {
            const float manualGain = _manualGain;
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                output[i] = input[i] * manualGain;
            }
            _appliedGain = manualGain;
            return work::Status::OK;
        }

        double gainDb = _gainDb;
        if (settings.update_decimation == 1U) {
            float level[kSubBlock];
            float exponent[kSubBlock];
            for (std::size_t base = 0UZ; base < nSamples; base += kSubBlock) {
                const std::size_t count = std::min(kSubBlock, nSamples - base);
                const T*          x     = input.data() + base;
                T*                y     = output.data() + base;
                for (std::size_t i = 0UZ; i < count; ++i) {
                    level[i] = detail::fastLog2(std::max(magnitudeSquared(x[i]), detail::kMagnitudeFloorSquared));
                }
                gainDb = runLoop(gainDb, level, exponent, count);
                for (std::size_t i = 0UZ; i < count; ++i) {
                    y[i] = x[i] * detail::fastExp2(exponent[i]);
                }
            }
        } else {
            const std::size_t stride = settings.update_decimation;
            // the first sample of this call that starts a hold interval, then every stride-th after it
            const std::size_t first   = _updatePhase == 0UZ ? 0UZ : std::min(stride - _updatePhase, nSamples);
            const std::size_t updates = first < nSamples ? (nSamples - 1UZ - first) / stride + 1UZ : 0UZ;

            for (std::size_t i = 0UZ; i < first; ++i) {
                output[i] = input[i] * _appliedGain;
            }
            float level[kSubBlock];
            float exponent[kSubBlock];
            float gain[kSubBlock];
            for (std::size_t firstUpdate = 0UZ; firstUpdate < updates; firstUpdate += kSubBlock) {
                const std::size_t count = std::min(kSubBlock, updates - firstUpdate);
                const std::size_t start = first + firstUpdate * stride;
                for (std::size_t j = 0UZ; j < count; ++j) {
                    level[j] = detail::fastLog2(std::max(magnitudeSquared(input[start + j * stride]), detail::kMagnitudeFloorSquared));
                }
                gainDb = runLoop(gainDb, level, exponent, count);
                for (std::size_t j = 0UZ; j < count; ++j) {
                    gain[j] = detail::fastExp2(exponent[j]);
                }
                for (std::size_t j = 0UZ; j < count; ++j) {
                    const std::size_t from = start + j * stride;
                    const std::size_t to   = std::min(from + stride, nSamples);
                    for (std::size_t i = from; i < to; ++i) {
                        output[i] = input[i] * gain[j];
                    }
                }
                _appliedGain = gain[count - 1UZ];
            }
            _updatePhase = (_updatePhase + nSamples) % stride;
        }

        _gainDb                = gainDb;
        status.current_gain_db = gainDb;
        return work::Status::OK;
    }

    [[nodiscard]] static constexpr float magnitudeSquared(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return sample * sample;
        } else {
            return sample.real() * sample.real() + sample.imag() * sample.imag();
        }
    }

    [[nodiscard]] double loopCoefficient(double tau) const noexcept {
        const double samples = tau * static_cast<double>(settings.sample_rate);
        return samples < 1.0 ? 1.0 : std::clamp(1.0 - std::exp(-1.0 / samples), std::numeric_limits<double>::denorm_min(), 1.0);
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_AGC_HPP
