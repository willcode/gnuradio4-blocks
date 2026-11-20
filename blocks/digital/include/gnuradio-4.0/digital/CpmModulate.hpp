#ifndef GNURADIO_CPM_MODULATE_HPP
#define GNURADIO_CPM_MODULATE_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/CpmPulse.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::CpmModulate, [T], [float])

template<std::floating_point T>
struct CpmModulate : Block<CpmModulate<T>, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(
@brief Continuous-phase modulation: real symbols on the M-PAM grid in, constant-envelope complex baseband out.

One symbol of amplitude `a` turns the carrier by exactly `pi * modulation_index * a` radians, spread over the
`pulse_length` symbols the frequency pulse spans. The phase is the running sum of the per-sample increments and is
never reset, so the output is continuous across every record and every call, and the envelope is constant by
construction.

`rect` at `pulse_length` 1 is CPFSK, which on two levels is classic FSK at deviation
`modulation_index / (2 * T_sym)`. `gaussian` is GFSK, and GMSK is that with two levels and `modulation_index` 0.5.
Longer pulses spread each symbol over its neighbors, narrowing the spectrum at the price of intersymbol
interference the receiver has to live with.

Symbols arrive as floats on the odd grid `±1, ±3, …, ±(M-1)`, the grid `PamSlicer` and `LevelTracker` decode.
Turning bits into that grid is an affine map on two levels and a Gray map above it, both of which belong upstream.

A symbol of amplitude a turns the carrier by pi * h * a radians, h the modulation index. A pulse length of 1 is full
response. Only the gaussian pulse reads `bt`; 0.3 and 0.5 are usual.
)"">;

    PortIn<T>                in;
    PortOut<std::complex<T>> out;

    struct Settings {
        Annotated<double, Visible, Doc<"modulation index">, Limits<Over<0.0>, Max<4.0>>>                          modulation_index   = 0.5;
        Annotated<gr::Size_t, Visible, Doc<"samples per symbol">, Limits<Min<2U>>>                                samples_per_symbol = 8U;
        Annotated<gr::digital::CpmPulseShape, Visible, Doc<"frequency pulse">>                                    pulse              = gr::digital::CpmPulseShape::Rect;
        Annotated<gr::Size_t, Unit<"symbols">, Doc<"pulse length (1 = full response)">, Limits<Min<1U>, Max<8U>>> pulse_length       = 1U;
        Annotated<double, Doc<"bandwidth-time product">>                                                          bt                 = 0.3;
        Annotated<T, Doc<"envelope amplitude">>                                                                   amplitude          = T(1);

        GR_MAKE_REFLECTABLE(Settings, modulation_index, samples_per_symbol, pulse, pulse_length, bt, amplitude);
    } settings;

    GR_MAKE_REFLECTABLE(CpmModulate, in, out, settings);

    gr::digital::CpmPulse<T> _pulse{};
    gr::signal::Phasor<T>    _phasor{};
    std::vector<double>      _increments{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.pulse == gr::digital::CpmPulseShape::Gaussian && s.bt <= 0.) {
            return gr::Refused(s.bt, "must be > 0 when pulse is gaussian");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        const bool redesign = s.modulation_index.changed() || s.samples_per_symbol.changed() || s.pulse.changed() || s.pulse_length.changed() || s.bt.changed();
        if (!redesign) {
            return;
        }
        _pulse.configure(s.pulse, s.pulse_length, s.samples_per_symbol, s.modulation_index, s.bt);
        this->input_chunk_size  = 1U;
        this->output_chunk_size = s.samples_per_symbol;
    }

    void start() {
        _pulse.reset();
        _phasor.setPhase(0.);
    }

    /// @brief One symbol becomes `samples_per_symbol` outputs, so a tag at input `t` belongs at output `sps*t`.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        const std::size_t                                 sps = settings.samples_per_symbol;
        std::vector<std::pair<std::size_t, property_map>> arriving;
        gr::for_each_reader_span(
            [&](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // already forwarded when it first arrived
                        continue;
                    }
                    auto forwarded = this->filterAndSubstituteTag(tagMap.get());
                    if (!forwarded.empty()) {
                        arriving.emplace_back(sps * static_cast<std::size_t>(relIndex), std::move(forwarded));
                    }
                }
            },
            inputSpans);
        if (arriving.empty()) {
            return;
        }
        gr::for_each_writer_span(
            [&arriving](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [offset, tagMap] : arriving) {
                    if (offset < span.size()) {
                        span.publishTag(tagMap, offset);
                    }
                }
            },
            outputSpans);
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<std::complex<T>> output) {
        const std::size_t sps      = settings.samples_per_symbol;
        const std::size_t nSymbols = std::min(input.size(), output.size() / sps);
        const std::size_t nSamples = nSymbols * sps;

        if (_increments.size() < nSamples) { // grows to the largest chunk seen and stays there
            _increments.resize(nSamples);
        }
        const std::span<double> increments(_increments.data(), nSamples);

        _pulse.incrementsFor(input.first(nSymbols), increments);
        _phasor.fillModulated(std::span<const double>(increments), output.first(nSamples));

        if (settings.amplitude != T(1)) { // scaling by exactly one is the identity, so the common case skips the pass
            const T gain = settings.amplitude;
            std::ranges::transform(output.first(nSamples), output.begin(), [gain](std::complex<T> v) { return gain * v; });
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_CPM_MODULATE_HPP
