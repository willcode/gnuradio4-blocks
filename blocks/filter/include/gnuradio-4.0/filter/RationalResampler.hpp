#ifndef GNURADIO_RATIONAL_RESAMPLER_HPP
#define GNURADIO_RATIONAL_RESAMPLER_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <numeric>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/PolyphaseResampler.hpp>

#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

GR_REGISTER_BLOCK(gr::blocks::filter::RationalResampler, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct RationalResampler : Block<RationalResampler<T>, Resampling<1UZ, 1UZ, false>> {
    using TParent     = Block<RationalResampler<T>, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(
@brief Changes the sample rate by the exact ratio `interpolation / decimation`, images and aliases suppressed.

`L` outputs for every `M` inputs. One filter does both jobs: its stopband starts at the Nyquist frequency of whichever
rate is lower. With `taps` empty the prototype is designed at the interpolated rate `L*fs_in` from `rolloff`,
`attenuation_db` and `max_ripple_db`, `L/M` reduced by its gcd first; supplied taps are used as given, at the
unreduced ratio and with no gain applied.

Single-stage only: 48 kHz to 44.1 kHz is `147/160` after reduction and costs 5880 taps. Changing `interpolation`,
`decimation` or `taps` rebuilds the filter and resets the phase and the history, with a discontinuity at the seam; a
ratio change also moves the tag map's origin, everything else leaving the alignment alone. A forwarded `sample_rate`
tag is multiplied by `L/M`, so downstream reads the rate of the stream this block hands it.

Each tag moves with all its keys by the filter's delay `d`. `d` is in samples of the interpolated rate `L*fs_in`. A
tag on input `i` leaves on output `round((i*L + d) / M)`. A designed or other symmetric prototype of `N` taps delays by
`(N-1)/2`. An asymmetric supplied prototype moves its tags by the centroid of its energy, rounded to the whole
interpolated sample. A tag keeps the output it was given when it crossed, through any later rebuild. A tag whose output
lies past the stream's last output leaves at the end-of-stream index, one past that output.

The block produces `interpolation` outputs per `decimation` inputs. Supplied `taps` are the prototype at the
interpolated rate L*fs_in and carry their own gain, L included. `rolloff` is the fraction of the surviving band spent on
the transition, for designed taps only.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<gr::Size_t, Doc<"interpolation">, Visible, Limits<Min<1U>>>   interpolation = 1U;
        Annotated<gr::Size_t, Doc<"decimation">, Visible, Limits<Min<1U>>>      decimation    = 1U;
        Annotated<std::vector<float>, Doc<"prototype taps (empty = designed)">> taps{};
        Annotated<float, Doc<"transition share">>                               rolloff        = 0.2f;
        Annotated<float, Unit<"dB">, Doc<"stopband target">>                    attenuation_db = 60.f;
        Annotated<float, Unit<"dB">, Doc<"passband ripple target">>             max_ripple_db  = 0.1f;

        GR_MAKE_REFLECTABLE(Settings, interpolation, decimation, taps, rolloff, attenuation_db, max_ripple_db);
    } settings;

    GR_MAKE_REFLECTABLE(RationalResampler, in, out, settings);

    std::optional<gr::filter::PolyphaseResampler<T>> _resampler;
    std::uint64_t                                    _interpolation = 1ULL; /// after reduction, where the taps were designed
    std::uint64_t                                    _decimation    = 1ULL;
    std::uint64_t                                    _twiceDelay    = 0ULL; /// the prototype's delay in half interpolated samples, `twiceTapDelay`
    std::uint64_t                                    _inOrigin      = 0ULL;
    std::uint64_t                                    _outOrigin     = 0ULL;
    bool                                             _reorigin      = false;
    detail::TagDelayLine                             _tags;

    // a design can take seconds, so the apply hook designs once and fails a ratio no filter meets
    void applySettings(const Settings& s) {
        const bool live        = _resampler.has_value();
        const bool rateChanged = s.interpolation.changed() || s.decimation.changed();
        const bool redesign    = rateChanged || s.taps.changed() || s.rolloff.changed() || s.attenuation_db.changed() || s.max_ripple_db.changed();
        if (redesign) {
            if (const std::optional<std::string> failure = rebuild()) { // only the design shows it
                this->fail(s.attenuation_db, *failure);
                return;
            }
        }
        _reorigin = _reorigin || (live && rateChanged);
    }

    void start() {
        if (const std::optional<std::string> failure = rebuild()) {
            throw gr::exception(*failure);
        }
        _tags.reset();
        _inOrigin  = 0ULL;
        _outOrigin = 0ULL;
        _reorigin  = false;
    }

    /// @brief The delay every forwarded tag moves by, in input samples. Generally not an integer.
    [[nodiscard]] double groupDelaySamples() const noexcept { return static_cast<double>(_twiceDelay) / (2.0 * static_cast<double>(_interpolation)); }

    /// Builds the filter the settings name, or returns why no design meets them and changes nothing.
    [[nodiscard]] std::optional<std::string> rebuild() {
        std::vector<float> prototype = settings.taps;
        std::uint64_t      l         = settings.interpolation;
        std::uint64_t      m         = settings.decimation;

        // Supplied taps were designed against L*fs_in, and a reduced L is a different interpolated rate. Only a
        // designed prototype reduces the ratio. Supplied taps at a reducible ratio cost gcd(L, M) times the branches.
        if (prototype.empty()) {
            const std::uint64_t g = std::gcd(l, m);
            l /= g;
            m /= g;
            const gr::filter::ResamplerDesign design = gr::filter::designResampler(static_cast<std::size_t>(l), static_cast<std::size_t>(m), static_cast<double>(settings.rolloff.value), static_cast<double>(settings.attenuation_db.value), static_cast<double>(settings.max_ripple_db.value));
            if (!design.ok) {
                return std::format("must be lower, or max_ripple_db higher, for a filter of at most {} taps at {}/{}", gr::filter::kResamplerMaxTaps, l, m);
            }
            prototype = design.taps;
        }

        _resampler.emplace(static_cast<std::size_t>(l), static_cast<std::size_t>(m), std::span<const float>(prototype));
        _interpolation = l;
        _decimation    = m;
        _twiceDelay    = detail::twiceTapDelay(std::span<const float>(prototype));

        this->input_chunk_size  = static_cast<gr::Size_t>(m);
        this->output_chunk_size = static_cast<gr::Size_t>(l);
        return std::nullopt;
    }

    /**
     * @brief Place each input tag with all its keys on the output that carries its input sample's energy, from the
     * current phase origin.
     *
     * Input `i` maps to output `round((i*L + d) / M)`, where `d` is the prototype's delay at the interpolated rate. A
     * half rounds up. A tag whose output a later call makes is held for that call. A tag is placed once, when it
     * crosses, under the delay and the ratio in force then. A later rebuild leaves every held tag where it is. Tags
     * leave in the order they arrived. A tag held past the stream's last output leaves at the end-of-stream index.
     *
     * This replaces the framework's forwarding rather than adjusting it: the default publishes a tag at the output
     * index matching its input index, which is only right at a ratio of one. It is also where a ratio change takes its
     * new origin: the change is applied on the settings path, between calls, where neither absolute offset is knowable.
     * A tag already held keeps its absolute output offset, which the new origin leaves alone, and the `sample_rate`
     * value it was scaled by on arrival, which is the ratio that was in force when it crossed.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (_reorigin) {
            gr::for_each_reader_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _inOrigin = static_cast<std::uint64_t>(span.streamIndex);
                    }
                },
                inputSpans);
            gr::for_each_writer_span(
                [this](auto& span) {
                    if (span.isSync && span.isConnected) {
                        _outOrigin = static_cast<std::uint64_t>(span.streamIndex);
                    }
                },
                outputSpans);
            _reorigin = false;
        }

        gr::for_each_reader_span(
            [processedIn, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                _tags.take(span, processedIn, [this](std::uint64_t at, property_map& tag) {
                    this->scaleSampleRateByChunkRatio(tag); // the ratio in force where the tag crossed, not where it is published
                    return _outOrigin + detail::mapDelayedOffset(at - _inOrigin, _interpolation, _decimation, _twiceDelay);
                });
            },
            inputSpans);

        gr::for_each_writer_span(
            [processedIn, this](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                const std::size_t made = detail::TagDelayLine::outputsToMake(processedIn / static_cast<std::size_t>(_decimation), span.size(), static_cast<std::size_t>(_interpolation));
                if (made > 0UZ) { // a call that makes nothing publishes nothing
                    _tags.release(span, made, false);
                }
            },
            outputSpans);
    }

    /// @brief Make as many of the outputs of the stream's last input as the output span holds, and publish every held
    /// tag, at the end-of-stream index for a tag past the last output.
    template<InputSpanLike TInput, OutputSpanLike TOutput>
    [[nodiscard]] work::Status processEpilogue(TInput& input, TOutput& output) {
        if (detail::dropAtStop(*this, _tags, output)) {
            return work::Status::OK;
        }
        const std::size_t chunks = std::min(input.size() / static_cast<std::size_t>(_decimation), output.size() / static_cast<std::size_t>(_interpolation));
        if (chunks > 0UZ) {
            std::ignore = processBulk(std::span<const T>(input.data(), chunks * static_cast<std::size_t>(_decimation)), std::span<T>(output.data(), chunks * static_cast<std::size_t>(_interpolation)));
        }
        _tags.takeRemainder(this->in, static_cast<std::uint64_t>(input.streamIndex) + static_cast<std::uint64_t>(input.size()), [this](property_map& tag) { this->scaleSampleRateByChunkRatio(tag); });
        _tags.release(output, chunks * static_cast<std::size_t>(_interpolation), true);
        output.publish(chunks * static_cast<std::size_t>(_interpolation));
        return work::Status::OK;
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        // The framework hands whole chunks — M in, L out — so the phase is zero at every call
        // boundary and the two counts agree exactly. Anything else is a contract violation.
        const std::size_t made = _resampler->process(input, output);
        if (made != output.size()) {
            throw gr::exception(std::format("{} input samples yield {} outputs, not the {} reserved", input.size(), made, output.size()));
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_RATIONAL_RESAMPLER_HPP
