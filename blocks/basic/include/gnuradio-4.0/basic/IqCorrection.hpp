#ifndef GNURADIO_IQ_CORRECTION_HPP
#define GNURADIO_IQ_CORRECTION_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/channel/IqImbalanceCorrector.hpp>
#include <gnuradio-4.0/algorithm/filter/SinglePoleAverage.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

namespace detail {

/// @brief Maps the estimate under @p key to the swapped signal `j*conj(x)`: a phase `p` to `pi/2 - p`, reduced to
/// `[-pi, pi]`, and a frequency `f` to `-f`. Removes an estimate held as neither `float` nor `double`.
inline void swapEstimate(property_map& map, std::string_view key, bool isPhase) {
    const auto found = map.find(property_map::key_type{key});
    if (found == map.end()) {
        return;
    }
    if (const double* value = found->second.get_if<double>(); value != nullptr) {
        found->second = isPhase ? std::remainder(std::numbers::pi / 2.0 - *value, 2.0 * std::numbers::pi) : -*value;
    } else if (const float* single = found->second.get_if<float>(); single != nullptr) {
        found->second = isPhase ? std::remainder(std::numbers::pi_v<float> / 2.f - *single, 2.f * std::numbers::pi_v<float>) : -*single;
    } else {
        map.erase(found);
    }
}

/// @brief Maps `phase_est` and `freq_est` to the swapped signal, at the top level and inside `gr:trigger_meta_info`.
inline void swapCarrierEstimates(property_map& map) {
    swapEstimate(map, "phase_est", true);
    swapEstimate(map, "freq_est", false);
    for (auto& [key, value] : map) {
        if (const property_map* meta = value.get_if<property_map>(); meta != nullptr && key == gr::tag::TRIGGER_META_INFO) {
            property_map swapped = *meta;
            swapEstimate(swapped, "phase_est", true);
            swapEstimate(swapped, "freq_est", false);
            value = pmt::Value(std::move(swapped));
        }
    }
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::IqSwap, [T], [std::complex<float>])

template<typename T>
requires(gr::meta::complex_like<T>)
struct IqSwap : Block<IqSwap<T>> {
    using Description = Doc<R""(
@brief Exchanges the real and imaginary components of every sample, mirroring the spectrum about DC.

The block follows a receiver whose hardware or driver exchanges I and Q. Such a receiver shows a signal from `+f` at
`-f`, with its sidebands inverted. The block puts the signal back. It takes complex samples and gives one out for each
one in. Every tag passes through on its own sample.

`enabled` turns the exchange on, from the next sample the block processes. A change of `enabled` that a tag past the
first sample of a chunk brings applies from the next chunk on. The input's `min_samples` can keep a chunk from ending at
such a tag. Disabled, the default, the block passes every sample through unchanged. The exchange equals conjugation
followed by a quarter turn, `swap(x) = j * conj(x)`.

The carrier estimates `phase_est` and `freq_est` describe the signal they arrive with. While the exchange is on, the
block maps both to the exchanged signal, at a tag's top level and inside its `gr:trigger_meta_info`. A phase `p` leaves
as `pi/2 - p`, reduced to `[-pi, pi]`, and a frequency `f` as `-f`, of the same type. An estimate that is neither a
`float` nor a `double` is dropped.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<bool, Doc<"I/Q swap">, Visible> enabled = false;

        GR_MAKE_REFLECTABLE(Settings, enabled);
    } settings;

    GR_MAKE_REFLECTABLE(IqSwap, in, out, settings);

    [[nodiscard]] constexpr T processOne(T sample) const noexcept { return settings.enabled ? T(sample.imag(), sample.real()) : sample; }

    /// Forwards every key of every tag, with the carrier estimates mapped while `enabled` is on. A call over samples
    /// forwards each tag of its chunk at the sample it arrived on and skips the tags behind the read position, which
    /// an earlier call forwarded. The call at the end of the stream forwards the tags at or past the read position at
    /// the end-of-stream index, less their `gr:end_of_stream` key.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const bool atEnd   = processedIn == 0UZ;
        const auto forward = [this, atEnd, &outputSpans](const property_map& map, std::size_t offset) {
            property_map forwarded = map;
            if (atEnd) {
                forwarded.erase(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM));
            }
            if (settings.enabled) {
                detail::swapCarrierEstimates(forwarded);
            }
            if (forwarded.empty()) {
                return;
            }
            gr::for_each_writer_span([&forwarded, offset](auto& outSpan) { outSpan.publishTag(forwarded, offset); }, outputSpans);
        };
        gr::for_each_reader_span(
            [atEnd, &forward](auto& inSpan) {
                if (!inSpan.isSync || !inSpan.isConnected) {
                    return;
                }
                if (atEnd) {
                    for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
                        if (relIndex >= 0) {
                            forward(tagMapRef.get(), 0UZ);
                        }
                    }
                    return;
                }
                for (const auto& [relIndex, tagMapRef] : inSpan.tags(inSpan.size())) {
                    if (relIndex >= 0) {
                        forward(tagMapRef.get(), static_cast<std::size_t>(relIndex));
                    }
                }
            },
            inputSpans);
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::FrontEndCorrection, [T], [ std::complex<float>, float ])

template<typename T>
requires(gr::meta::complex_like<T> || std::floating_point<T>)
struct FrontEndCorrection : Block<FrontEndCorrection<T>> {
    using Description = Doc<R""(
@brief Corrects a receiver's stream: exchanges I and Q, removes DC and corrects the IQ gain and phase imbalance, each
part switched on its own.

The block sits directly after a receiver, in place of a chain of one block for each part. On a complex stream it puts
back the I and Q a receiver exchanged, removes a converter's bias and an LO leaking into its own mixer, and corrects the
gain and phase imbalance between the I and Q arms. On a real stream it removes the carrier level from an AM detector's
envelope, and the IQ part has no effect. It gives one sample out for each one in, of the input's type. The output has no
delay. Every tag passes through on its own sample with the values it arrived with. The block does not map the carrier
estimates `phase_est` and `freq_est` of a tag to the exchanged signal; `IqSwap` does.

`swap_iq` exchanges I and Q of every sample, `swap(x) = j * conj(x)`, before the DC and IQ parts see it. Their estimates
and their output are of the exchanged stream. A change of `swap_iq` exchanges the two components of the DC estimate and
swaps the I and Q arms of the imbalance estimate: the gain of Q against I becomes its reciprocal and the quadrature
error stays. The estimates then follow the stream the block now outputs, and the imbalance run in progress keeps the
samples it has. On a real stream there is nothing to exchange, and the block refuses `swap_iq` set to true.

`enable_dc` turns the DC removal on. The block subtracts a single-pole average, `e = alpha * x + (1 - alpha) * e`, with
`alpha = 1/(1 + tau*sample_rate)`. It runs in `double`, since in `float` the factor `1 - alpha` rounds to 1 once
`tau*sample_rate` reaches `2^25`. An output sample can differ from the sample-by-sample recursion by about one float
rounding of the signal, and where the scheduler splits the stream can change which samples differ. A sample with a NaN
or infinite component leaves the estimate unchanged, and the block outputs that sample less the held estimate.
`dcEstimate()` returns the estimate, with the imaginary part zero on a real stream. `tau` sets the time constant in
seconds, and `sample_rate` the stream rate it counts against. A change of either moves the pole and keeps the estimate.
The block refuses a `tau` and `sample_rate` whose `alpha` rounds to 0. The block reports the derived `alpha` and
`corner_hz`, the highpass -3 dB point.

`enable_iq` turns the IQ correction on. Over runs of 1024 samples the block averages `P_I = E[I^2]`, `P_Q = E[Q^2]` and
`C = E[I*Q]` of the first difference of the stream, `x[n] - x[n-1]`, with the time constant `iq_tau` in seconds. The
difference carries the same imbalance and no DC. It weighs content at `f` by `4*sin^2(pi*f/sample_rate)`: a signal
within a small fraction of `sample_rate` of the tuned frequency, such as a carrier or an AM station tuned to the center,
barely enters the estimate. The block outputs `I' = I` and `Q' = k*(Q - c*I)`, with `c = C/P_I` and
`k = sqrt(P_I/(P_Q - C^2/P_I))`, after the DC removal when both parts are on. Each run is corrected with the
coefficients the runs before it gave. The correction reaches at most 3 dB of gain and 10 degrees of phase. Only an
imbalance that does not depend on frequency is corrected. The estimate assumes circular content, as noise and most band
content are. A strong real-valued signal at the tuned frequency whose content spreads across the band, such as a fast
BPSK or one with unfiltered symbol steps, biases it. The block then adds an image of its own, at most -14.3 dBc on a
receiver with no imbalance of its own. The block reports the estimated imbalance as `iq_gain_db`, the gain of Q against
I, and `iq_phase_deg`, the quadrature error, as status values after each run of 1024 samples.

Each estimate is held while its part is off. With all three off, the default, the block passes every sample through
unchanged. `reset()` sets both estimates to zero and the correction to none.

The swap runs before the other corrections. The DC estimate's corner is 1/(2*pi*tau). `alpha` reports the derived
coefficient 1/(1 + tau*sample_rate) and `corner_hz` the highpass -3 dB point, alpha*sample_rate/(2*pi).
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<bool, Doc<"I/Q swap">, Visible>                                       swap_iq     = false;
        Annotated<bool, Doc<"DC removal">, Visible>                                     enable_dc   = false;
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>             sample_rate = 96000.f;
        Annotated<double, Unit<"s">, Doc<"DC time constant">, Limits<Over<0.0>>>        tau         = 1.0;
        Annotated<bool, Doc<"I/Q imbalance correction">, Visible>                       enable_iq   = false;
        Annotated<double, Unit<"s">, Doc<"imbalance time constant">, Limits<Over<0.0>>> iq_tau      = 0.1;

        GR_MAKE_REFLECTABLE(Settings, swap_iq, enable_dc, sample_rate, tau, enable_iq, iq_tau);
    } settings;

    struct Status {
        Annotated<double, Doc<"DC smoothing coefficient">>            alpha        = alphaOf(Settings{}.tau, Settings{}.sample_rate);
        Annotated<double, Unit<"Hz">, Doc<"DC highpass corner">>      corner_hz    = cornerOf(alpha.value, Settings{}.sample_rate);
        Annotated<double, Unit<"dB">, Doc<"Q-to-I gain">>             iq_gain_db   = 0.0;
        Annotated<double, Unit<"deg">, Doc<"quadrature phase error">> iq_phase_deg = 0.0;

        GR_MAKE_REFLECTABLE(Status, alpha, corner_hz, iq_gain_db, iq_phase_deg);
    } status;

    GR_MAKE_REFLECTABLE(FrontEndCorrection, in, out, settings, status);

    // the private members below make the block no aggregate, so it takes the base's constructors
    using Block<FrontEndCorrection<T>>::Block;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if constexpr (!meta::complex_like<T>) {
            if (s.swap_iq) {
                return gr::Refused(s.swap_iq, "must be false on a real stream");
            }
        }
        // each average refuses a coefficient outside its range, and a `tau * sample_rate` past `double` gives one
        if (!gr::filter::SinglePoleAverage<T>{}.setAlpha(alphaOf(s.tau, s.sample_rate))) {
            return gr::Refused(s.tau, "must give an alpha > 0 at this sample_rate");
        }
        if (!gr::channel::IqImbalanceCorrector<Real>{}.setTimeConstant(s.iq_tau * static_cast<double>(s.sample_rate))) {
            return gr::Refused(s.iq_tau, "must give the imbalance runs a weight at this sample_rate");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        if (!_iq.setTimeConstant(s.iq_tau * static_cast<double>(s.sample_rate))) {
            this->fail(s.iq_tau, "gives the imbalance runs no weight");
        }
        const double nextAlpha = alphaOf(s.tau, s.sample_rate);
        if (_dc.setAlpha(nextAlpha)) {
            status.alpha     = nextAlpha;
            status.corner_hz = cornerOf(nextAlpha, s.sample_rate);
        } else {
            this->fail(s.tau, "gives an alpha outside (0, 1]");
        }
        if (!s.enable_iq) {
            _iq.markDiscontinuity();
        }
        // the estimates then describe the exchanged stream
        if constexpr (meta::complex_like<T>) {
            if (s.swap_iq.changed()) {
                _dc.exchange();
                _iq.exchange();
            }
        }
        publishIq();
    }

    void reset() {
        _dc.reset();
        _iq.reset();
        publishIq();
    }

    /// The tracked DC offset. Call it on the thread that runs the block, or while the block is stopped.
    [[nodiscard]] std::complex<double> dcEstimate() const noexcept { return std::complex<double>(_dc.estimate()); }

    /// Writes each sample with the parts that are on applied: exchanged, then less the DC estimate updated with it,
    /// then corrected for the imbalance. Each of the eight states has its own path, and the exchange reads the samples
    /// where the next part loads them.
    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        if constexpr (meta::complex_like<T>) {
            if (settings.enable_iq) {
                std::size_t updates = 0UZ;
                if (settings.swap_iq) {
                    updates = settings.enable_dc ? _iq.processExchanged(input, output, [this](std::span<const T> raw, std::span<T> base) { _dc.subtractExchanged(raw, base); }) : _iq.processExchanged(input, output);
                } else {
                    updates = settings.enable_dc ? _iq.process(input, output, [this](std::span<const T> raw, std::span<T> base) { _dc.subtract(raw, base); }) : _iq.process(input, output);
                }
                if (updates > 0UZ) {
                    publishIq();
                }
                return work::Status::OK;
            }
            if (settings.swap_iq) {
                if (settings.enable_dc) {
                    _dc.subtractExchanged(input, output);
                } else {
                    std::ranges::transform(input, output.begin(), [](T sample) { return T(sample.imag(), sample.real()); });
                }
                return work::Status::OK;
            }
        }
        if (settings.enable_dc) {
            _dc.subtract(input, output);
        } else {
            std::ranges::copy(input, output.begin());
        }
        return work::Status::OK;
    }

private:
    using Real = decltype(std::real(std::declval<T>()));

    gr::filter::SinglePoleAverage<T>        _dc;
    gr::channel::IqImbalanceCorrector<Real> _iq;

    /// Reports the estimates, from the work function and the hooks.
    void publishIq() {
        status.iq_gain_db   = _iq.gainDb();
        status.iq_phase_deg = _iq.phaseDeg();
    }

    [[nodiscard]] static constexpr double alphaOf(double seconds, float rate) noexcept { return 1.0 / (1.0 + seconds * static_cast<double>(rate)); }
    [[nodiscard]] static constexpr double cornerOf(double nextAlpha, float rate) noexcept { return nextAlpha * static_cast<double>(rate) / (2.0 * std::numbers::pi); }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_IQ_CORRECTION_HPP
