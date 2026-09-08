#ifndef GNURADIO_ROTATOR_HPP
#define GNURADIO_ROTATOR_HPP

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/math/NamespaceCompatibility.hpp>
#include <gnuradio-4.0/meta/utils.hpp>
#include <numbers>
#include <optional>
#include <tuple>

namespace gr::blocks::math {

GR_REGISTER_BLOCK(gr::blocks::math::Rotator, [T], [std::complex<float>])

template<gr::meta::complex_like T>
struct Rotator : gr::Block<Rotator<T>> {
    using value_type  = typename T::value_type;
    using Description = Doc<R""(
@brief Rotator block shifts complex input samples by a given incremental phase every sample,
       thus effectively performing a frequency translation.

This block supports either `phase_increment` in radians per sample (x) or relative `frequency_shift` in Hz for a
given 'sample_rate' in Hz (N.B sample_rate is normalised to '1' by default).
A `frequency` key on a passing tag is retuned by the same shift, so it keeps describing the center of the stream it
is attached to.
 )"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<float, "sample rate", Doc<"signal sample rate">, Unit<"Hz">>                           sample_rate     = 1.f;
    Annotated<float, "frequency shift", Doc<"rel. frequency shift">, Unit<"Hz">>                     frequency_shift = 0.0f;
    Annotated<value_type, "phase_increment", Unit<"rad">, Doc<"how many radians to add per sample">> phase_increment{0};
    Annotated<value_type, "initial_phase", Unit<"rad">, Doc<"starting offset for each new chunk">>   initial_phase{0};

    // the phase state and the generation recurrence both live in the kernel, which keeps them in double
    // regardless of T for the reason its Doc records
    gr::signal::Phasor<value_type> _phasor{};

    GR_MAKE_REFLECTABLE(Rotator, in, out, sample_rate, frequency_shift, initial_phase, phase_increment);

    // 'frequency_shift' is the commanded quantity and survives a 'sample_rate' change;
    // setting 'phase_increment' instead commands the increment and re-derives the shift.
    // 'initial_phase' applied at a phase the block does not hold restarts the accumulator there; re-applying the
    // phase it already holds moves no value and does not restart it.
    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const bool haveShift     = newSettings.contains("frequency_shift");
        const bool haveIncrement = newSettings.contains("phase_increment");
        if (haveShift && haveIncrement) {
            throw gr::exception(std::format("cannot set both 'frequency_shift' and 'phase_increment' in new setting (XOR): {}", newSettings));
        }

        // The two settings name one rotation: the key that moved is the master and the other follows from the
        // members, so a batch that names neither of them still leaves the increment stating what the members say.
        const double previousIncrement = _phasor.increment();

        if (haveIncrement) {
            frequency_shift = static_cast<float>(phase_increment / (value_type(2) * std::numbers::pi_v<value_type>)) * sample_rate;
        } else {
            phase_increment = value_type(2) * static_cast<value_type>(std::numbers::pi_v<float> * frequency_shift / sample_rate);
        }
        const double newIncrement = static_cast<double>(phase_increment);

        // a settings change is not a phase discontinuity: the running phase moves only when `initial_phase` does

        // This block advances then rotates — sample k carries the accumulated phase plus (k+1) increments —
        // while the kernel rotates then advances, so the phasor is held one increment ahead throughout. Keeping
        // the offset in the phase rather than stepping the phasor per call is what lets the kernel's lane state
        // run unbroken across calls, which is where its bit-identical chunk independence comes from.
        // a new 'initial_phase' re-anchors the accumulation there; re-writing the one in force moves nothing
        if (newSettings.contains("initial_phase")) {
            seedPhase();
        } else if (newIncrement != previousIncrement) {
            // the pending advance takes the new increment, so a frequency change carries the phase forward
            const double advanced = _phasor.phase() - previousIncrement + newIncrement;
            _phasor.setIncrement(newIncrement);
            _phasor.setPhase(advanced);
        }
    }

    /// @brief The stream begins at 'initial_phase', so a run starts there whether or not the value moved.
    void start() { seedPhase(); }

    /// @brief Anchor the phasor at 'initial_phase'; it is held one increment ahead throughout.
    void seedPhase() {
        const double increment = static_cast<double>(phase_increment);
        _phasor.setIncrement(increment);
        _phasor.setPhase(static_cast<double>(initial_phase) + increment);
    }

    // rotating by 'frequency_shift' moves the spectrum, so the center frequency a passing tag announces moves by the
    // same amount in the opposite direction: what sits at the output's zero frequency arrived at -'frequency_shift'.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t /*processedIn*/) {
        std::optional<property_map> cachedSettings;
        gr::for_each_reader_span(
            [&](auto& inSpan) {
                if (!inSpan.isSync || !inSpan.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMapRef] : inSpan.tags(1UZ)) {
                    property_map forwarded = this->filterAndSubstituteTag(tagMapRef.get(), cachedSettings);
                    if (forwarded.empty()) {
                        continue;
                    }
                    retuneCenterFrequency(forwarded);
                    const std::size_t offset = static_cast<std::size_t>(std::max(std::ptrdiff_t(0), relIndex));
                    gr::for_each_writer_span([&forwarded, offset](auto& outSpan) { outSpan.publishTag(forwarded, offset); }, outputSpans);
                }
            },
            inputSpans);
    }

    void retuneCenterFrequency(property_map& tagMap) const noexcept {
        auto it = tagMap.find(gr::tag::FREQUENCY.shortKey());
        if (it == tagMap.end()) {
            return;
        }
        if (const double* asDouble = it->second.template get_if<double>(); asDouble != nullptr) {
            it->second = *asDouble - static_cast<double>(frequency_shift.value);
        } else if (const float* asFloat = it->second.template get_if<float>(); asFloat != nullptr) {
            it->second = *asFloat - frequency_shift.value;
        }
    }

    [[nodiscard]] constexpr work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        assert(output.size() >= input.size());

        _phasor.mix(input, output);

        return work::Status::OK;
    }

    // the entry point for callers that rotate one sample at a time. A block offers the framework
    // exactly one process function, so this cannot be called 'processOne' beside 'processBulk'; it
    // runs the same recurrence, and one call advances the phase as one element of a bulk call does.
    [[nodiscard]] constexpr T rotateOne(const T& inSample) noexcept {
        T outSample{};
        std::ignore = processBulk(std::span<const T>(&inSample, 1UZ), std::span<T>(&outSample, 1UZ));
        return outSample;
    }
};

} // namespace gr::blocks::math

#endif // GNURADIO_ROTATOR_HPP
