#ifndef GNURADIO_NOISE_BLANKER_HPP
#define GNURADIO_NOISE_BLANKER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/analog/NamespaceCompatibility.hpp>
#include <gnuradio-4.0/analog/detail/Averager.hpp>

namespace gr::blocks::analog {

namespace detail {

inline constexpr const char* kNoiseBlanked = "noise_blanked";

enum class Replacement : std::uint8_t { Interpolate, Hold, Zero };

[[nodiscard]] inline Replacement parseReplacement(std::string_view name) {
    if (name == "interpolate") {
        return Replacement::Interpolate;
    }
    if (name == "hold") {
        return Replacement::Hold;
    }
    if (name == "zero") {
        return Replacement::Zero;
    }
    throw gr::exception(std::format("replacement must be 'interpolate', 'hold' or 'zero', got '{}'", name));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::NoiseBlanker, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct NoiseBlanker : Block<NoiseBlanker<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Removes impulse noise - ignition, arcing, radar, switching supplies - by replacing the offending samples.

A sample whose power exceeds the tracked mean power by `threshold_db` opens a window of `blank_samples`, and the
`lookback_samples` before it are replaced too; `replacement` chooses how. The output is the input delayed by
`delay_samples` in every case, including while disabled. The default `threshold_db = 9.32` blanks 0.17 % of a clean
circular-complex-Gaussian stream.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<bool, "enabled", Doc<"false is a pass-through through the same delay; true restarts the warm-up">, Visible>       enabled            = false;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate; a change retunes the tracker and restarts warm-up">>          sample_rate        = 96000.f;
    Annotated<double, "threshold_db", Unit<"dB">, Doc<"how far above the tracked mean power a sample must be">, Visible>        threshold_db       = 9.32;
    Annotated<double, "averaging_time", Unit<"s">, Doc<"tracker time constant; 0 uses alpha directly">>                         averaging_time     = 0.01;
    Annotated<double, "alpha", Doc<"the pole, used when averaging_time is 0; reads back as the resolved value">>                alpha              = 1e-3;
    Annotated<gr::Size_t, "blank_samples", Doc<"length of the replaced run once a detection fires; changes the delay">>         blank_samples      = 7U;
    Annotated<gr::Size_t, "lookback_samples", Doc<"samples before the detected one also replaced; changes the delay">>          lookback_samples   = 2U;
    Annotated<std::string, "replacement", Doc<"'interpolate' (default), 'hold' or 'zero'">, Visible>                            replacement        = std::string("interpolate");
    Annotated<bool, "retrigger", Doc<"restart the window on a detection inside one, up to max_window_samples">>                 retrigger          = false;
    Annotated<gr::Size_t, "max_window_samples", Doc<"cap on a retriggering window; 0 selects 4 * blank_samples">>               max_window_samples = 0U;
    Annotated<bool, "emit_tags", Doc<"publish a private noise_blanked tag at each replaced run, with its length">>              emit_tags          = false;
    Annotated<gr::Size_t, "delay_samples", Doc<"observable: lookback_samples + blank_samples, present when disabled">>          delay_samples      = 9U;
    Annotated<double, "tracked_power", Doc<"observable: the censored mean power the threshold is taken against">>               tracked_power      = 0.0;
    Annotated<double, "blanked_fraction", Doc<"observable: a slow average of the replaced duty, so a threshold can be judged">> blanked_fraction   = 0.0;

    GR_MAKE_REFLECTABLE(NoiseBlanker, in, out, enabled, sample_rate, threshold_db, averaging_time, alpha, blank_samples, lookback_samples, replacement, retrigger, max_window_samples, emit_tags, delay_samples, tracked_power, blanked_fraction);

    std::vector<T>            _line{};
    std::vector<std::uint8_t> _marked{}; // a byte per slot rather than a bitset: read and written once per sample
    std::size_t               _cursor       = 0UZ;
    std::size_t               _oldest       = 1UZ;
    std::size_t               _ring         = 10UZ;
    std::size_t               _delay        = 9UZ;
    std::size_t               _lookback     = 2UZ;
    std::size_t               _blank        = 7UZ;
    std::size_t               _maxWindow    = 28UZ;
    detail::Replacement       _rule         = detail::Replacement::Interpolate;
    double                    _alpha        = 1e-3;
    double                    _threshold    = 8.5507;
    double                    _power        = 0.0;
    double                    _duty         = 0.0;
    std::size_t               _warmup       = 960UZ;
    std::size_t               _warmupLength = 960UZ;
    std::uint32_t             _remaining    = 0U;
    std::uint32_t             _windowLength = 0U;
    std::uint32_t             _runPosition  = 0U;
    T                         _anchor{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (blank_samples < 1U) {
            throw gr::exception(std::format("blank_samples must be at least one, got {}", blank_samples.value));
        }
        if (averaging_time < 0.0 || !std::isfinite(averaging_time)) {
            throw gr::exception(std::format("averaging_time must be zero or a positive number of seconds, got {}", averaging_time.value));
        }
        if (!(sample_rate > 0.f) || !std::isfinite(sample_rate)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        _rule      = detail::parseReplacement(replacement);
        _alpha     = detail::averagerCoefficient(alpha, averaging_time, static_cast<double>(sample_rate));
        alpha      = _alpha;
        _threshold = std::pow(10.0, threshold_db / 10.0);

        _lookback     = static_cast<std::size_t>(lookback_samples.value);
        _blank        = static_cast<std::size_t>(blank_samples.value);
        _maxWindow    = max_window_samples == 0U ? 4UZ * _blank : static_cast<std::size_t>(max_window_samples.value);
        _warmupLength = averaging_time > 0.0 ? static_cast<std::size_t>(std::ceil(static_cast<double>(sample_rate) * averaging_time)) : static_cast<std::size_t>(std::ceil(1.0 / _alpha));

        const std::size_t delay = _lookback + _blank;
        if (delay != _delay || _line.size() != delay + 1UZ) {
            _delay        = delay;
            delay_samples = static_cast<gr::Size_t>(delay);
            _ring         = delay + 1UZ;
            _line.assign(_ring, T{});
            _marked.assign(_ring, std::uint8_t{0});
            _cursor       = 0UZ;
            _oldest       = 1UZ;
            _remaining    = 0U;
            _windowLength = 0U;
            _runPosition  = 0U;
            _anchor       = T{};
        }
        if (newSettings.contains("sample_rate") || newSettings.contains("averaging_time") || newSettings.contains("alpha") || (newSettings.contains("enabled") && enabled)) {
            _warmup = _warmupLength; // the tracked power is stale after a pole move or after the tracker was disabled
        }
    }

    void reset() {
        std::ranges::fill(_line, T{});
        _marked.assign(_ring, std::uint8_t{0});
        _cursor       = 0UZ;
        _oldest       = 1UZ;
        _power        = 0.0;
        _duty         = 0.0;
        _warmup       = _warmupLength;
        _remaining    = 0U;
        _windowLength = 0U;
        _runPosition  = 0U;
        _anchor       = T{};
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t           nSamples = std::min(inSpan.size(), outSpan.size());
        const std::size_t           nTags    = inSpan.rawTags.size();
        std::size_t                 tag      = 0UZ;
        std::optional<property_map> cachedSettings;

        // Forwarding is manual because this block publishes its own tags into the same span, and the framework's
        // forwarding runs before processBulk and would leave them out of order. The filter itself is the framework's.
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            while (tag < nTags && inSpan.rawTags[tag].index <= inSpan.streamIndex + i) {
                if (inSpan.rawTags[tag].index >= inSpan.streamIndex) {
                    const property_map forwarded = this->filterAndSubstituteTag(inSpan.rawTags[tag].map, cachedSettings);
                    if (!forwarded.empty()) { // 1:1 and no offset arithmetic: a tag stays where it arrived
                        outSpan.publishTag(forwarded, i);
                    }
                }
                ++tag;
            }
            accept(inSpan[i]);
            outSpan[i] = emit(outSpan, i);
        }

        tracked_power    = _power;
        blanked_fraction = _duty;
        std::ignore      = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

private:
    [[nodiscard]] static constexpr double squaredMagnitude(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return static_cast<double>(sample) * static_cast<double>(sample);
        } else {
            return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
    }

    /// The oldest entry plus @p ahead, wrapped by a compare, which is cheaper than a runtime integer modulo.
    [[nodiscard]] std::size_t slot(std::size_t ahead) const noexcept {
        const std::size_t at = _oldest + ahead;
        return at >= _ring ? at - _ring : at;
    }

    /// @brief Take one input: decide, then track the power from the samples the decision left in place.
    void accept(T sample) {
        const double power      = squaredMagnitude(sample);
        const bool   windowOpen = _remaining > 0U;

        // Compare first and update after: updating from every sample, impulse
        // included, and then testing against the value the impulse just raised is what desensitizes a blanker.
        bool detected = false;
        if (_warmup > 0UZ) {
            --_warmup;
        } else if (enabled) {
            detected = power > _threshold * _power;
        }
        if (!windowOpen && !detected && enabled) {
            _power += _alpha * (power - _power);
        }

        _cursor        = _cursor + 1UZ == _ring ? 0UZ : _cursor + 1UZ;
        _oldest        = _cursor + 1UZ == _ring ? 0UZ : _cursor + 1UZ;
        _line[_cursor] = sample;
        bool marked    = windowOpen;

        if (detected && !windowOpen) {
            for (std::size_t back = 1UZ; back <= _lookback; ++back) { // the samples before the detected one, still unemitted
                _marked[_cursor >= back ? _cursor - back : _cursor + _ring - back] = 1U;
            }
            _windowLength = static_cast<std::uint32_t>(_lookback);
            _remaining    = static_cast<std::uint32_t>(_blank);
            marked        = true;
        }

        _marked[_cursor] = marked ? std::uint8_t{1} : std::uint8_t{0};
        _windowLength    = marked ? _windowLength + 1U : 0U;
        _duty += 0.1 * _alpha * ((marked ? 1.0 : 0.0) - _duty); // ten tracker time constants: a 0.17 % duty needs a longer average than the power does

        if (detected && windowOpen && retrigger) {
            const std::uint32_t room = _windowLength > static_cast<std::uint32_t>(_maxWindow) ? 0U : static_cast<std::uint32_t>(_maxWindow) + 1U - _windowLength;
            _remaining               = std::min(static_cast<std::uint32_t>(_blank), room);
        }
        if (_remaining > 0U) {
            --_remaining;
        }
    }

    /// @brief Produce the output for the sample that entered `delay_samples` ago, replaced if it was marked.
    template<typename TOutSpan>
    [[nodiscard]] T emit(TOutSpan& outSpan, std::size_t at) {
        const std::size_t oldest = slot(0UZ);
        if (_marked[oldest] == 0U) {
            _anchor      = _line[oldest];
            _runPosition = 0U;
            return _anchor;
        }

        std::size_t ahead = 1UZ;
        while (ahead <= _delay && _marked[slot(ahead)] != 0U) {
            ++ahead;
        }
        const std::uint32_t length = _runPosition + static_cast<std::uint32_t>(ahead);

        if (_runPosition == 0U && emit_tags) {
            outSpan.publishTag(property_map{{detail::kNoiseBlanked, static_cast<gr::Size_t>(length)}}, at);
        }
        ++_runPosition;

        switch (_rule) {
        case detail::Replacement::Zero: return T{};
        case detail::Replacement::Hold: return _anchor;
        default: break;
        }
        const T      after  = _line[slot(std::min(ahead, _delay))];
        const double weight = static_cast<double>(_runPosition) / static_cast<double>(length + 1U);
        if constexpr (std::same_as<T, float>) {
            return static_cast<float>(static_cast<double>(_anchor) + (static_cast<double>(after) - static_cast<double>(_anchor)) * weight);
        } else {
            return T(static_cast<float>(static_cast<double>(_anchor.real()) + (static_cast<double>(after.real()) - static_cast<double>(_anchor.real())) * weight), //
                static_cast<float>(static_cast<double>(_anchor.imag()) + (static_cast<double>(after.imag()) - static_cast<double>(_anchor.imag())) * weight));
        }
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_NOISE_BLANKER_HPP
