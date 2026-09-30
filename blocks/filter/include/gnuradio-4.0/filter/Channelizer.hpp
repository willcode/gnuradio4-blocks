#ifndef GNURADIO_CHANNELIZER_HPP
#define GNURADIO_CHANNELIZER_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <format>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/filter/PolyphaseChannelizer.hpp>

namespace gr::blocks::filter {

namespace detail {

/**
 * @brief The designed prototype and what it was measured to deliver.
 *
 * The design lives in the library beside the bank, where the resampler designs also sit. It searches a length and
 * does not trust an estimate. The result carries the stopband and ripple it was measured to achieve. A caller can
 * see whether the request was met.
 */
[[nodiscard]] inline gr::filter::ChannelizerDesign channelizerPrototype(std::size_t channels, std::string_view family, double attenuationDb, double transition, std::size_t span, std::size_t oversample) {
    if (!(transition > 0.) || !(transition < 1.)) {
        throw gr::exception(std::format("transition is a fraction of the channel spacing and must lie in (0, 1), got {}", transition));
    }
    if (!(attenuationDb > 0.)) {
        throw gr::exception(std::format("attenuation_db must be positive, got {}", attenuationDb));
    }
    if (family != "root_nyquist" && family != "lowpass" && family != "boxcar") {
        throw gr::exception(std::format("prototype must be 'root_nyquist', 'lowpass' or 'boxcar', got '{}'", family));
    }
    if (family == "root_nyquist" && span < 2UZ) {
        throw gr::exception(std::format("span is the channel spacings the prototype covers and must be at least 2, got {}", span));
    }
    return gr::filter::designChannelizerPrototype(channels, family, attenuationDb, transition, std::max(span, 2UZ), oversample);
}

/// The documentation the two banks share, so their settings read the same way.
using ChannelizerSettingsDoc = Doc<R""(
`n_channels` is M. `taps` supplies an explicit prototype. When `taps` is empty, a prototype is designed in the family
`prototype` names. `root_nyquist`, the default, is a root-raised cosine of excess bandwidth `transition` covering
`span` channel spacings. Its squared responses sum flat across the bank. An analysis and a synthesis bank then cancel
each other. It still rejects 61 to 72 dB across the channel counts this bank accepts. `lowpass` is a Kaiser at half a
channel spacing with `attenuation_db` of stopband rejection and a transition of `transition` channel spacings. It
rejects some 30 dB more and costs more taps. It does not reconstruct at all. `boxcar` is one channel long. The bank is
then a plain block transform whose inverse is its own synthesis. The lowpass search designs to a target. The other
families have no target and are measured.

Critical sampling forces a choice between isolation and reconstruction. The three families are the corners of that
choice. The table gives, at sixteen channels, the worst leakage into a channel two or more away and the
reconstruction after the cascade.

    prototype       taps   isolation   critically sampled   oversampled by two
    boxcar            16    -18.9 dB           -141.5 dB            -143.3 dB
    root_nyquist     255    -67.1 dB              -9.1 dB             -59.7 dB
    lowpass          161   -107.0 dB             -12.9 dB             -15.0 dB

A critically sampled bank isolates or reconstructs, but not both. An `oversample` of 2 gives both at once. Designing
any of them costs well under a millisecond at the widest bank, once per settings change. `oversample` is 1 for a
commutator that advances a whole M per step, or 2 for one that advances M/2. Reconstruction needs the 2. `taps` stays
as supplied, empty or not. The observable `designed_taps` holds the prototype that runs. A design setting can
therefore be changed live and read back. `designed_taps` is zero-padded to a whole number of branches. A response
scan reads that padding as a shifted center. A measurement uses the prototype as designed and not `designed_taps`.
)"">;

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::filter::PolyphaseChannelizer, [T], [float])

template<std::floating_point F>
struct PolyphaseChannelizer : Block<PolyphaseChannelizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(
@brief A polyphase analysis filter bank, with one stream in and `n_channels` equally spaced channels out.

Channel `k` carries the input's band around `k * fs / M`, shifted to baseband and decimated. Indices above `M/2` are
the negative frequencies in the usual transform order. Each output runs at `fs/M`, or twice that when the bank is
oversampled.

One prototype serves every channel. The bank filters through its polyphase branches and takes an inverse discrete
Fourier transform across them. That transform gives its advantage over `M` separate filters. Adjacent-channel
rejection equals the prototype's stopband attenuation. The design settings therefore state the isolation directly.

An input tag lands on channel zero at its own step index and is not copied to the other channels. A tag on every
channel would multiply the stream's tag traffic by M. A consumer that needs a tag on another channel taps the stream
ahead of the bank. The block divides a `sample_rate` on that tag by the commutator stride. The tag then states the
channel rate and not the input rate.
)"">;

    PortIn<std::complex<F>>               in;
    std::vector<PortOut<std::complex<F>>> out;

    Annotated<gr::Size_t, "n_channels", Visible, Doc<"M, 2 to 256, a non-live setting that re-plumbs the ports">>                                    n_channels = 4U;
    Annotated<std::vector<float>, "taps", detail::ChannelizerSettingsDoc>                                                                            taps{};
    Annotated<std::vector<float>, "designed_taps", Doc<"observable running prototype, zero-padded to whole branches">>                               designed_taps{};
    Annotated<double, "stopband_db", Unit<"dB">, Doc<"observable worst level, alias edge to Nyquist, relative to unit gain">>                        stopband_db{};
    Annotated<double, "ripple_db", Unit<"dB">, Doc<"observable passband ripple from DC to the pass edge">>                                           ripple_db{};
    Annotated<bool, "design_ok", Doc<"observable, whether the lowpass design met its target">>                                                       design_ok{};
    Annotated<double, "attenuation_db", Unit<"dB">, Doc<"stopband target of the lowpass search">>                                                    attenuation_db = 60.0;
    Annotated<double, "transition", Doc<"excess bandwidth in channel spacings, lowpass transition or root-Nyquist rolloff">>                         transition     = 0.5;
    Annotated<std::string, "prototype", Visible, Doc<"'root_nyquist', 'lowpass' to reject the most, or 'boxcar' to reconstruct critically sampled">> prototype      = std::string("root_nyquist");
    Annotated<gr::Size_t, "span", Doc<"channel spacings the root_nyquist prototype covers, setting its rejection, at least 2">>                      span           = 16U;
    Annotated<gr::Size_t, "oversample", Visible, Doc<"1 for critical sampling, 2 for a commutator stride of M/2">>                                   oversample     = 1U;

    GR_MAKE_REFLECTABLE(PolyphaseChannelizer, in, out, n_channels, taps, designed_taps, stopband_db, ripple_db, design_ok, attenuation_db, transition, prototype, span, oversample);

    gr::filter::PolyphaseChannelizer<F> _bank{};
    std::vector<std::complex<F>>        _step{};
    std::size_t                         _channels = 4UZ;
    std::size_t                         _stride   = 4UZ;

    /// The bank is built from the members. A settings batch that changes no value makes no call. A block constructed
    /// at its declared defaults must still start with its ports plumbed. The graph reads the port collection once, at
    /// the first connection.
    explicit PolyphaseChannelizer(property_map init = {}) : Block<PolyphaseChannelizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>>(std::move(init)) { rebuild(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        const auto         channels = static_cast<std::size_t>(n_channels.value);
        const auto         factor   = static_cast<std::size_t>(oversample.value);
        std::vector<float> designed = taps.value;
        if (designed.empty()) {
            const auto measured = detail::channelizerPrototype(channels, prototype, attenuation_db, transition, static_cast<std::size_t>(span.value), factor);
            designed            = measured.taps;
            stopband_db         = measured.stopbandDb;
            ripple_db           = measured.rippleDb;
            design_ok           = measured.ok;
        } else { // a supplied prototype has no target either, and it is measured on the same terms
            const auto measured = gr::filter::measureChannelizerPrototype(designed, channels, transition, factor);
            stopband_db         = measured.stopbandDb;
            ripple_db           = measured.rippleDb;
            design_ok           = measured.ok;
        }

        _bank.configure(channels, designed, factor);
        _channels = _bank.channels();
        _stride   = _bank.stride();
        _step.assign(_channels, std::complex<F>{});
        out.resize(_channels);
        designed_taps.value = std::vector<float>(_bank.prototype().begin(), _bank.prototype().end());

        this->input_chunk_size  = static_cast<gr::Size_t>(_stride);
        this->output_chunk_size = 1U;
    }

    template<typename TOutputSpanType>
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, std::span<TOutputSpanType>& outputs) {
        const std::size_t stride   = _stride;
        const std::size_t channels = _channels;
        std::size_t       nSteps   = inSpan.size() / stride;
        for (const auto& port : outputs) {
            nSteps = std::min(nSteps, port.size());
        }

        for (std::size_t s = 0UZ; s < nSteps; ++s) {
            _bank.step(std::span<const std::complex<F>>(inSpan).subspan(s * stride, stride), _step);
            for (std::size_t c = 0UZ; c < channels; ++c) {
                outputs[c][s] = _step[c];
            }
        }

        // A channel runs at the input rate over the commutator stride. A sample_rate crossing this block would state a
        // rate that is not the stream's. Every key is carried through, per the block's tag rule. Only the rate is
        // rescaled, by the framework's chunk ratio, which is 1/stride here. The channel rate is then derived from the
        // declared resampling and not computed separately.
        if (!outputs.empty()) {
            for (const auto& [relIndex, tagMap] : inSpan.tags()) {
                if (relIndex < 0) {
                    continue;
                }
                const std::size_t at = static_cast<std::size_t>(relIndex) / stride;
                if (at < nSteps) {
                    property_map forwarded = tagMap.get();
                    this->scaleSampleRateByChunkRatio(forwarded);
                    outputs[0].publishTag(std::move(forwarded), at);
                }
            }
        }

        std::ignore = inSpan.consume(nSteps * stride);
        for (auto& port : outputs) {
            port.publish(nSteps);
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::filter::PolyphaseSynthesizer, [T], [float])

template<std::floating_point F>
struct PolyphaseSynthesizer : Block<PolyphaseSynthesizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(
@brief A polyphase synthesis filter bank, with `n_channels` channels in and one stream out.

It is the transpose of the analysis bank, with the same prototype and channel numbering. Channel `k` is placed at
`k * fs / M` of the output rate. A synthesizer fed an analyzer's channels returns the analyzer's input. The output is
delayed by the prototype's span and scaled by the cascade's gain.

Reconstruction depends on the prototype and the commutator stride, not on the bank. Two conditions must hold
together. The prototype is the `root_nyquist` family, whose squared responses sum flat across the bank. `oversample`
is 2. Meeting both recovers the analyzer's input to about -60 dB at sixteen channels. A `lowpass` prototype caps it
near -19 dB whatever else is done. A critically sampled bank does not reconstruct at all. The aliasing each channel
folds in has no partner to cancel it.

Tags on the channel inputs are dropped and not merged onto the output. The M channels carry independent tag streams,
and one output would need them reconciled. A `sample_rate` among them states a channel rate and not the
reconstructed rate. Forwarding any one of them unchanged would be wrong. A consumer that needs a tag across this
block carries it around the bank.
)"">;

    std::vector<PortIn<std::complex<F>>> in;
    PortOut<std::complex<F>>             out;

    Annotated<gr::Size_t, "n_channels", Visible, Doc<"M, 2 to 256, a non-live setting that re-plumbs the ports">>                                    n_channels = 4U;
    Annotated<std::vector<float>, "taps", detail::ChannelizerSettingsDoc>                                                                            taps{};
    Annotated<std::vector<float>, "designed_taps", Doc<"observable running prototype, zero-padded to whole branches">>                               designed_taps{};
    Annotated<double, "stopband_db", Unit<"dB">, Doc<"observable worst level, alias edge to Nyquist, relative to unit gain">>                        stopband_db{};
    Annotated<double, "ripple_db", Unit<"dB">, Doc<"observable passband ripple from DC to the pass edge">>                                           ripple_db{};
    Annotated<bool, "design_ok", Doc<"observable, whether the lowpass design met its target">>                                                       design_ok{};
    Annotated<double, "attenuation_db", Unit<"dB">, Doc<"stopband target of the lowpass search">>                                                    attenuation_db = 60.0;
    Annotated<double, "transition", Doc<"excess bandwidth in channel spacings, lowpass transition or root-Nyquist rolloff">>                         transition     = 0.5;
    Annotated<std::string, "prototype", Visible, Doc<"'root_nyquist', 'lowpass' to reject the most, or 'boxcar' to reconstruct critically sampled">> prototype      = std::string("root_nyquist");
    Annotated<gr::Size_t, "span", Doc<"channel spacings the root_nyquist prototype covers, setting its rejection, at least 2">>                      span           = 16U;
    Annotated<gr::Size_t, "oversample", Visible, Doc<"1 for critical sampling, 2 for a commutator stride of M/2">>                                   oversample     = 1U;

    GR_MAKE_REFLECTABLE(PolyphaseSynthesizer, in, out, n_channels, taps, designed_taps, stopband_db, ripple_db, design_ok, attenuation_db, transition, prototype, span, oversample);

    gr::filter::PolyphaseSynthesizer<F> _bank{};
    std::vector<std::complex<F>>        _step{};
    std::size_t                         _channels = 4UZ;
    std::size_t                         _stride   = 4UZ;

    /// As in the analysis bank, the ports must exist before the graph reads the collection.
    explicit PolyphaseSynthesizer(property_map init = {}) : Block<PolyphaseSynthesizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>>(std::move(init)) { rebuild(); }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        const auto         channels = static_cast<std::size_t>(n_channels.value);
        const auto         factor   = static_cast<std::size_t>(oversample.value);
        std::vector<float> designed = taps.value;
        if (designed.empty()) {
            const auto measured = detail::channelizerPrototype(channels, prototype, attenuation_db, transition, static_cast<std::size_t>(span.value), factor);
            designed            = measured.taps;
            stopband_db         = measured.stopbandDb;
            ripple_db           = measured.rippleDb;
            design_ok           = measured.ok;
        } else { // a supplied prototype has no target either, and it is measured on the same terms
            const auto measured = gr::filter::measureChannelizerPrototype(designed, channels, transition, factor);
            stopband_db         = measured.stopbandDb;
            ripple_db           = measured.rippleDb;
            design_ok           = measured.ok;
        }

        _bank.configure(channels, designed, factor);
        _channels = _bank.channels();
        _stride   = _bank.stride();
        _step.assign(_channels, std::complex<F>{});
        in.resize(_channels);
        designed_taps.value = std::vector<float>(_bank.prototype().begin(), _bank.prototype().end());

        this->input_chunk_size  = 1U;
        this->output_chunk_size = static_cast<gr::Size_t>(_stride);
    }

    template<typename TInputSpanType>
    [[nodiscard]] work::Status processBulk(std::span<TInputSpanType>& inputs, std::span<std::complex<F>> output) {
        const std::size_t stride   = _stride;
        const std::size_t channels = _channels;
        std::size_t       nSteps   = output.size() / stride;
        for (const auto& port : inputs) {
            nSteps = std::min(nSteps, port.size());
        }

        for (std::size_t s = 0UZ; s < nSteps; ++s) {
            for (std::size_t c = 0UZ; c < channels; ++c) {
                _step[c] = inputs[c][s];
            }
            _bank.step(_step, output.subspan(s * stride, stride));
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_CHANNELIZER_HPP
