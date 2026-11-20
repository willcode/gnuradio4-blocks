#ifndef GNURADIO_FUNCTION_GENERATOR_HPP
#define GNURADIO_FUNCTION_GENERATOR_HPP

#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/BlockingSync.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/algorithm/signal/NoiseGenerator.hpp>
#include <gnuradio-4.0/algorithm/signal/SignalGeneratorCore.hpp>
#include <gnuradio-4.0/algorithm/signal/ToneGenerator.hpp>

#include <magic_enum.hpp>
#include <magic_enum_utility.hpp>

namespace gr::blocks::basic {

using namespace gr;

namespace function_generator {
enum class SignalType : int { constant, linear_ramp, parabolic_ramp, cubic_spline, impulse_response, uniform_noise, triangular_noise, gaussian_noise, sin, cos, fast_sin, fast_cos };
enum class ParameterType : int { signal_type, start_value, final_value, duration, round_off_time, impulse_time0, impulse_time1, tone_frequency, phase };

using enum SignalType;
using enum ParameterType;
constexpr auto SignalTypeList = magic_enum::enum_values<SignalType>();

constexpr bool isToneType(SignalType type) noexcept { return type == sin || type == cos || type == fast_sin || type == fast_cos; }

constexpr gr::signal::ToneType toToneType(SignalType type) noexcept {
    switch (type) {
    case sin: return gr::signal::ToneType::Sin;
    case cos: return gr::signal::ToneType::Cos;
    case fast_sin: return gr::signal::ToneType::FastSin;
    case fast_cos: return gr::signal::ToneType::FastCos;
    default: std::unreachable();
    }
}

template<typename T>
requires std::is_same_v<T, SignalType> || std::is_same_v<T, ParameterType>
constexpr std::pmr::string toString(T type) {
    return std::pmr::string(magic_enum::enum_name(type));
}

template<typename EnumType, typename T>
requires std::is_same_v<EnumType, SignalType> || std::is_same_v<EnumType, ParameterType>
[[nodiscard]] std::pair<std::pmr::string, pmt::Value> createPropertyMapEntry(EnumType enumType, T value) {
    if constexpr (std::is_same_v<T, SignalType>) {
        return {toString(enumType), static_cast<pmt::Value>(toString(value))};
    } else {
        return {toString(enumType), static_cast<pmt::Value>(value)};
    }
}

// each create*PropertyMap returns one segment: the settings a FunctionGenerator puts in force when its `segment` names it
template<typename T>
[[nodiscard]] property_map createConstPropertyMap(T startValue) {
    return {createPropertyMapEntry(signal_type, constant), createPropertyMapEntry(start_value, startValue)};
}

template<typename T>
[[nodiscard]] property_map createLinearRampPropertyMap(T startValue, T finalValue, T durationValue) {
    return {createPropertyMapEntry(signal_type, linear_ramp), createPropertyMapEntry(start_value, startValue), createPropertyMapEntry(final_value, finalValue), createPropertyMapEntry(duration, durationValue)};
}

template<typename T>
[[nodiscard]] property_map createParabolicRampPropertyMap(T startValue, T finalValue, T durationValue, T roundOffTime) {
    return {createPropertyMapEntry(signal_type, parabolic_ramp), createPropertyMapEntry(start_value, startValue), createPropertyMapEntry(final_value, finalValue), createPropertyMapEntry(duration, durationValue), createPropertyMapEntry(round_off_time, roundOffTime)};
}

template<typename T>
[[nodiscard]] property_map createCubicSplinePropertyMap(T startValue, T finalValue, T durationValue) {
    return {createPropertyMapEntry(signal_type, cubic_spline), createPropertyMapEntry(start_value, startValue), createPropertyMapEntry(final_value, finalValue), createPropertyMapEntry(duration, durationValue)};
}

template<typename T>
[[nodiscard]] property_map createImpulseResponsePropertyMap(T startValue, T finalValue, T time0, T time1) {
    return {createPropertyMapEntry(signal_type, impulse_response), createPropertyMapEntry(start_value, startValue), createPropertyMapEntry(final_value, finalValue), createPropertyMapEntry(impulse_time0, time0), createPropertyMapEntry(impulse_time1, time1)};
}

template<typename T>
[[nodiscard]] property_map createUniformNoisePropertyMap(T amplitude) {
    return {createPropertyMapEntry(signal_type, uniform_noise), createPropertyMapEntry(start_value, amplitude)};
}

template<typename T>
[[nodiscard]] property_map createTriangularNoisePropertyMap(T amplitude) {
    return {createPropertyMapEntry(signal_type, triangular_noise), createPropertyMapEntry(start_value, amplitude)};
}

template<typename T>
[[nodiscard]] property_map createGaussianNoisePropertyMap(T amplitude) {
    return {createPropertyMapEntry(signal_type, gaussian_noise), createPropertyMapEntry(start_value, amplitude)};
}

template<typename T>
[[nodiscard]] property_map createTonePropertyMap(SignalType toneType, T freq, T amplitude, T phase, T offset, T durationValue) {
    return {createPropertyMapEntry(signal_type, toneType), createPropertyMapEntry(tone_frequency, freq), createPropertyMapEntry(final_value, amplitude), createPropertyMapEntry(ParameterType::phase, phase), createPropertyMapEntry(start_value, offset), createPropertyMapEntry(duration, durationValue)};
}

template<typename T>
[[nodiscard]] property_map createSinPropertyMap(T freq, T amplitude, T phase = T(0), T offset = T(0), T durationValue = T(0)) {
    return createTonePropertyMap(sin, freq, amplitude, phase, offset, durationValue);
}

template<typename T>
[[nodiscard]] property_map createCosPropertyMap(T freq, T amplitude, T phase = T(0), T offset = T(0), T durationValue = T(0)) {
    return createTonePropertyMap(cos, freq, amplitude, phase, offset, durationValue);
}

template<typename T>
[[nodiscard]] property_map createFastSinPropertyMap(T freq, T amplitude, T phase = T(0), T offset = T(0), T durationValue = T(0)) {
    return createTonePropertyMap(fast_sin, freq, amplitude, phase, offset, durationValue);
}

template<typename T>
[[nodiscard]] property_map createFastCosPropertyMap(T freq, T amplitude, T phase = T(0), T offset = T(0), T durationValue = T(0)) {
    return createTonePropertyMap(fast_cos, freq, amplitude, phase, offset, durationValue);
}

/**
 * The values that putting a segment in force writes: the entries of the segment `name` in `segments`, less each key
 * for which `namedByCall(key)` holds, since a key the call names itself keeps the call's value. An empty name, a name
 * with no segment or a segment that is no map writes nothing.
 *
 * The expansion takes no type of the block: a table of named maps of settings, the name a selector holds, and the keys
 * a call names.
 */
template<typename TNamedByCall>
[[nodiscard]] property_map segmentValues(const property_map& segments, std::string_view name, TNamedByCall&& namedByCall) {
    property_map values;
    if (name.empty()) {
        return values;
    }
    const auto found = segments.find(name);
    if (found == segments.end()) {
        return values;
    }
    const property_map* entries = found->second.template get_if<property_map>();
    if (entries == nullptr) {
        return values;
    }
    for (const auto& [key, value] : *entries) {
        if (!namedByCall(std::string_view(key))) {
            values.insert_or_assign(key, value);
        }
    }
    return values;
}

// a number held in `value` as a TNumber: a whole number for an integral TNumber, within the range of TNumber
template<typename TNumber>
[[nodiscard]] std::optional<TNumber> numberAs(const pmt::Value& value) {
    std::optional<TNumber> result;
    const auto             takeIfHeld = [&value, &result]<typename TStored>(std::type_identity<TStored>) {
        const TStored* stored = value.get_if<TStored>();
        if (stored == nullptr) {
            return;
        }
        if constexpr (std::floating_point<TNumber>) {
            if (!std::isfinite(static_cast<double>(*stored)) || std::fabs(static_cast<double>(*stored)) <= static_cast<double>(std::numeric_limits<TNumber>::max())) {
                result = static_cast<TNumber>(*stored);
            }
        } else if constexpr (std::integral<TStored>) {
            if (std::in_range<TNumber>(*stored)) {
                result = static_cast<TNumber>(*stored);
            }
        } else {
            // 2^digits is a power of two, so the comparison with it is exact in a double
            const double upper = std::ldexp(1., std::numeric_limits<TNumber>::digits);
            const double lower = std::is_signed_v<TNumber> ? -upper : 0.;
            const double whole = static_cast<double>(*stored);
            if (std::isfinite(whole) && std::trunc(whole) == whole && whole >= lower && whole < upper) {
                result = static_cast<TNumber>(whole);
            }
        }
    };
    takeIfHeld(std::type_identity<float>{});
    takeIfHeld(std::type_identity<double>{});
    takeIfHeld(std::type_identity<std::int8_t>{});
    takeIfHeld(std::type_identity<std::int16_t>{});
    takeIfHeld(std::type_identity<std::int32_t>{});
    takeIfHeld(std::type_identity<std::int64_t>{});
    takeIfHeld(std::type_identity<std::uint8_t>{});
    takeIfHeld(std::type_identity<std::uint16_t>{});
    takeIfHeld(std::type_identity<std::uint32_t>{});
    takeIfHeld(std::type_identity<std::uint64_t>{});
    return result;
}

// a segment's value as the type of the setting it names, or nothing when it does not convert; an enumerator's name is
// read as the setting reads it, exactly as spelled
template<typename TField>
[[nodiscard]] std::optional<TField> segmentValueAs(const pmt::Value& value) {
    if constexpr (std::is_enum_v<TField>) {
        if (!value.is_string()) {
            return std::nullopt;
        }
        const std::string_view name = value.value_or(std::string_view{});
        for (const TField enumerator : magic_enum::enum_values<TField>()) {
            if (magic_enum::enum_name(enumerator) == name) {
                return enumerator;
            }
        }
        return std::nullopt;
    } else if constexpr (std::same_as<TField, std::string>) {
        return value.is_string() ? std::optional<TField>(value.value_or(std::string{})) : std::nullopt;
    } else {
        return numberAs<TField>(value);
    }
}

} // namespace function_generator

GR_REGISTER_BLOCK(gr::blocks::basic::FunctionGenerator, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct FunctionGenerator : Block<FunctionGenerator<T>>, BlockingSync<FunctionGenerator<T>> {
    using Description = Doc<R""(@brief generates function waveforms and their combinations, one named segment after another.

Supported functions: constant, linear_ramp, parabolic_ramp, cubic_spline, impulse_response,
                     uniform_noise, triangular_noise, gaussian_noise,
                     sin, cos, fast_sin, fast_cos.

Segments: `segments` maps a segment's name to the signal settings it holds (signal_type, the values, the duration).
Setting `segment` to a name runs that segment's values and restarts the waveform; a signal setting the same call
changes keeps the call's value. The host's signal settings keep the values the host wrote; `current_segment` reports
the segment's name and the values the waveform runs. A graph couples `segment` to a tag key, e.g. gr:context, so that
a tag in the clock's stream switches the segment from its sample on. Setting `trigger` true restarts the segment in
force, and `trigger` reads false again, so the next true restarts again; naming the segment in force again restarts
nothing.

Operating modes:
  clk_in connected: generates one sample per clock input sample
  clk_in disconnected: free-running mode synchronised to wall-clock time

`chunk_size` is the samples per update in free-running mode. `round_off_time` serves parabolic_ramp; `impulse_time0`
and `impulse_time1` serve impulse_response.
`tone_frequency` is the tone's own frequency, apart from the stream's gr:frequency.
)"">;

    PortIn<std::uint8_t, Optional> clk_in;
    PortOut<T>                     out;

    struct Settings {
        Annotated<float, Unit<"Hz">, Visible, Doc<"sample rate">, Limits<Over<0.f>>> sample_rate = 1000.f;
        Annotated<gr::Size_t, Visible, Doc<"samples per update">>                    chunk_size  = 100;

        Annotated<property_map, Doc<"signal settings by segment name">> segments{};
        Annotated<std::string, Visible, Doc<"segment in force">>        segment;
        // a command, not a state: the apply hook restarts the waveform and assigns false, so each true restarts again
        Annotated<bool, Doc<"segment restart">> trigger = false;

        Annotated<function_generator::SignalType, Visible, Doc<"signal type">>   signal_type    = function_generator::constant;
        Annotated<float>                                                         start_value    = 0.f;
        Annotated<float>                                                         final_value    = 0.f;
        Annotated<float, Unit<"s">, Doc<"duration">>                             duration       = 0.f;
        Annotated<float, Unit<"s">, Doc<"round-off time">>                       round_off_time = 0.f;
        Annotated<float, Unit<"s">, Doc<"first impulse time">>                   impulse_time0  = 0.f;
        Annotated<float, Unit<"s">, Doc<"second impulse time">>                  impulse_time1  = 0.f;
        Annotated<float, Unit<"Hz">, Visible, Doc<"tone frequency">>             tone_frequency = 0.f;
        Annotated<float, Unit<"rad">, Visible, Doc<"phase">>                     phase          = 0.f;
        Annotated<std::uint64_t, Visible, Doc<"noise seed (0 = fixed default)">> seed           = 0ULL;

        GR_MAKE_REFLECTABLE(Settings, sample_rate, chunk_size, segments, segment, trigger, signal_type, start_value, final_value, duration, round_off_time, impulse_time0, impulse_time1, tone_frequency, phase, seed);
    } settings;

    struct Status {
        Annotated<property_map, Doc<"segment in force and its values">> current_segment{};

        GR_MAKE_REFLECTABLE(Status, current_segment);
    } status;

    GR_MAKE_REFLECTABLE(FunctionGenerator, clk_in, out, settings, status);

    // the values the waveform runs: the host's signal settings, overlaid by the values of the segment in force
    struct Signal {
        function_generator::SignalType signal_type    = function_generator::constant;
        float                          start_value    = 0.f;
        float                          final_value    = 0.f;
        float                          duration       = 0.f;
        float                          round_off_time = 0.f;
        float                          impulse_time0  = 0.f;
        float                          impulse_time1  = 0.f;
        float                          tone_frequency = 0.f;
        float                          phase          = 0.f;
        std::uint64_t                  seed           = 0ULL;
    };

    Signal _signal{};
    double _currentTime   = 0.;
    int    _sampleCounter = 0;
    double _timeTick      = 1. / static_cast<double>(settings.sample_rate);
    bool   _applied       = false; // the setup's apply has run

    gr::signal::NoiseGenerator<double> _noise;
    gr::signal::ToneGenerator<double>  _tone;

    // ~Block() invokes stop() for a block that is still active, and by then the timer state the mixin's stop()
    // waits on has been released: the active states have to be left here, while the derived object still stands.
    ~FunctionGenerator() {
        if (lifecycle::isActive(this->state())) {
            std::ignore = this->changeStateTo(lifecycle::State::REQUESTED_STOP);
        }
        this->stopTimerAndJoin();
    }

    void start() {
        _currentTime = 0.;
        _timeTick    = 1. / static_cast<double>(settings.sample_rate);
        configureGeneratorsFromSettings();
        this->blockingSyncStart();
    }

    void stop() { this->blockingSyncStop(); }

    work::Status processBulk(InputSpanLike auto& input, OutputSpanLike auto& output) {
        const auto nSamples = this->syncSamples(input, output);
        if (nSamples == 0UZ) {
            std::ignore = input.consume(0UZ);
            output.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        for (std::size_t i = 0; i < nSamples; ++i) {
            output[i] = generateSample();
        }

        std::ignore = input.consume(this->isFreeRunning() ? 0UZ : nSamples);
        output.publish(nSamples);
        return work::Status::OK;
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const property_map& table = s.segments.value;
        for (const auto& [name, entry] : table) {
            if (name.empty()) {
                return gr::Refused(s.segments, "must give each segment a name");
            }
            const std::optional<std::string> reason = segmentRefusal(entry);
            if (reason) {
                return gr::Refused(s.segments, std::format("segment '{}' {}", std::string_view(name), *reason));
            }
        }
        const std::string& selected = s.segment.value;
        if (!selected.empty() && !table.contains(std::pmr::string(selected))) {
            return gr::Refused(s.segment, "must be empty or name a segment in segments");
        }
        return {};
    }

    // A signal setting that changed goes in force; a segment that is selected, or whose table changed, puts its values
    // in force over the others. At the setup a value given at construction counts as changed and a default does not: a
    // key the call names keeps the call's value over the segment's, as the selection names no key.
    // non-const: clears trigger after acting on it, so the next true triggers again
    void applySettings(Settings& s) {
        const bool                           setup      = !_applied;
        const function_generator::SignalType kindBefore = _signal.signal_type;
        _applied                                        = true;

        std::vector<std::string_view> taken;
        std::apply(
            [&](const auto&... entry) {
                const auto takeSetting = [&](const auto& candidate) {
                    const auto& [key, settingMember, signalMember] = candidate;
                    const bool named                               = setup ? this->isGiven(s.*settingMember) : (s.*settingMember).changed();
                    if (setup || named) {
                        _signal.*signalMember = (s.*settingMember).value;
                    }
                    if (named) {
                        taken.push_back(key);
                    }
                };
                (takeSetting(entry), ...);
            },
            segmentSettings());

        const bool selected = s.segment.changed();
        if (selected || s.segments.changed()) {
            putInForce(function_generator::segmentValues(s.segments, s.segment.value, [&taken](std::string_view key) { return std::ranges::find(taken, key) != taken.end(); }));
        }
        // a segment restarts on the selection that names it, and a change of the waveform's kind restarts it too; an
        // empty name, as a timing event without a context carries, selects nothing and leaves the waveform running
        if ((selected && !s.segment.value.empty()) || _signal.signal_type != kindBefore) {
            _currentTime = 0.;
        }
        // naming the segment in force again changes nothing; a trigger restarts it, and reads false again so that
        // the next true restarts it again
        if (s.trigger) {
            _currentTime = 0.;
            s.trigger    = false;
        }
        reportSegment();
        _timeTick = 1. / static_cast<double>(s.sample_rate);
        configureGeneratorsFromSettings();
    }

    [[nodiscard]] T generateSample() noexcept {
        _sampleCounter++;
        using enum function_generator::SignalType;

        if (_signal.signal_type == uniform_noise || _signal.signal_type == triangular_noise || _signal.signal_type == gaussian_noise) {
            _currentTime += _timeTick;
            if constexpr (gr::signal::detail::is_complex_v<T>) {
                using F           = typename T::value_type;
                const auto sample = _noise.generateComplexSample();
                return T(static_cast<F>(sample.real()), static_cast<F>(sample.imag()));
            } else {
                return toOutputType(_noise.generateSample());
            }
        }

        if (function_generator::isToneType(_signal.signal_type)) {
            const double sv      = static_cast<double>(_signal.start_value); // tone offset (start_value = offset for tone types)
            const double dur     = static_cast<double>(_signal.duration);
            const bool   expired = dur > 0. && _currentTime > dur;
            _currentTime += _timeTick;
            if (expired) {
                if constexpr (gr::signal::detail::is_complex_v<T>) {
                    using F = typename T::value_type;
                    return T(static_cast<F>(sv), F(0));
                } else {
                    return toOutputType(sv);
                }
            }
            if constexpr (gr::signal::detail::is_complex_v<T>) {
                using F           = typename T::value_type;
                const auto sample = _tone.generateComplexSample();
                return T(static_cast<F>(sample.real()), static_cast<F>(sample.imag()));
            } else {
                return toOutputType(_tone.generateSample());
            }
        }

        const double sv  = static_cast<double>(_signal.start_value);
        const double fv  = static_cast<double>(_signal.final_value);
        const double dur = static_cast<double>(_signal.duration);

        double value{};
        switch (_signal.signal_type) {
        case constant: value = sv; break;
        case linear_ramp: value = _currentTime > dur ? fv : sv + (fv - sv) * (_currentTime / dur); break;
        case parabolic_ramp: value = calculateParabolicRamp(); break;
        case cubic_spline: {
            const double t  = _currentTime / dur;
            const double t2 = 3. * t * t;
            const double t3 = 2. * t * t * t;
            value           = _currentTime > dur ? fv : (t3 - t2 + 1.) * sv + (-t3 + t2) * fv;
            break;
        }
        case impulse_response: {
            const double it0 = static_cast<double>(_signal.impulse_time0);
            const double it1 = static_cast<double>(_signal.impulse_time1);
            value            = (_currentTime < it0 || _currentTime > it0 + it1) ? sv : fv;
            break;
        }
        default: value = 0.;
        }
        _currentTime += _timeTick;

        if constexpr (gr::signal::detail::is_complex_v<T>) {
            using F = typename T::value_type;
            return T(static_cast<F>(value), F(0));
        } else {
            return toOutputType(value);
        }
    }

private:
    [[nodiscard]] double calculateParabolicRamp() const noexcept {
        const double sv  = static_cast<double>(_signal.start_value);
        const double fv  = static_cast<double>(_signal.final_value);
        const double dur = static_cast<double>(_signal.duration);
        const double rot = static_cast<double>(_signal.round_off_time);

        const double linearLength = dur - 2. * rot;
        const double a            = (fv - sv) / (2. * rot * (linearLength + rot));
        const double ar2          = a * rot * rot;
        const double slope        = (fv - sv - 2. * ar2) / linearLength;

        if (_currentTime > dur) {
            return fv;
        }
        if (_currentTime < rot) {
            return sv + a * _currentTime * _currentTime;
        }
        if (_currentTime < dur - rot) {
            return sv + ar2 + slope * (_currentTime - rot);
        }
        const double shiftedTime = _currentTime - (dur - rot);
        return fv - ar2 + slope * shiftedTime - a * shiftedTime * shiftedTime;
    }

    template<typename TKey, typename TSettingMember, typename TSignalMember>
    static constexpr auto segmentSetting(TKey key, TSettingMember settingMember, TSignalMember signalMember) noexcept {
        using SettingValue = typename std::remove_cvref_t<decltype(std::declval<Settings&>().*settingMember)>::value_type;
        using SignalValue  = std::remove_cvref_t<decltype(std::declval<Signal&>().*signalMember)>;
        static_assert(std::same_as<SettingValue, SignalValue>, "a signal value has the type of its setting");
        return std::tuple{std::string_view(key), settingMember, signalMember};
    }

    // the settings a segment holds, each with its key and the value the waveform runs
    static constexpr auto segmentSettings() noexcept {
        return std::tuple{segmentSetting("signal_type", &Settings::signal_type, &Signal::signal_type), segmentSetting("start_value", &Settings::start_value, &Signal::start_value),  //
            segmentSetting("final_value", &Settings::final_value, &Signal::final_value), segmentSetting("duration", &Settings::duration, &Signal::duration),                         //
            segmentSetting("round_off_time", &Settings::round_off_time, &Signal::round_off_time), segmentSetting("impulse_time0", &Settings::impulse_time0, &Signal::impulse_time0), //
            segmentSetting("impulse_time1", &Settings::impulse_time1, &Signal::impulse_time1), segmentSetting("tone_frequency", &Settings::tone_frequency, &Signal::tone_frequency), //
            segmentSetting("phase", &Settings::phase, &Signal::phase), segmentSetting("seed", &Settings::seed, &Signal::seed)};
    }

    template<typename TSignalMember>
    using FieldOf = std::remove_cvref_t<decltype(std::declval<Signal&>().*std::declval<TSignalMember>())>;

    // why a segment cannot be put in force, or nothing when each key it holds is a setting a segment holds, at that setting's type
    [[nodiscard]] static std::optional<std::string> segmentRefusal(const pmt::Value& entry) {
        const property_map* values = entry.get_if<property_map>();
        if (values == nullptr) {
            return std::string("must be a map from signal setting to value");
        }
        for (const auto& [key, value] : *values) {
            if (key == "segments" || key == "segment") {
                return std::format("must drop '{}', a block setting", std::string_view(key));
            }
            bool known    = false;
            bool converts = false;
            std::apply(
                [&](const auto&... setting) {
                    const auto matchSetting = [&](const auto& candidate) {
                        const auto& [candidateKey, settingMember, signalMember] = candidate;
                        if (candidateKey == std::string_view(key)) {
                            known    = true;
                            converts = function_generator::segmentValueAs<FieldOf<std::remove_cvref_t<decltype(signalMember)>>>(value).has_value();
                        }
                    };
                    (matchSetting(setting), ...);
                },
                segmentSettings());
            if (!known) {
                return std::format("must drop '{}', which names no signal setting", std::string_view(key));
            }
            if (!converts) {
                return std::format("must give '{}' a value of that setting's type", std::string_view(key));
            }
        }
        return std::nullopt;
    }

    void putInForce(const property_map& values) {
        std::apply(
            [&](const auto&... setting) {
                const auto takeValue = [&](const auto& candidate) {
                    const auto& [key, settingMember, signalMember] = candidate;
                    const auto found                               = values.find(key);
                    if (found == values.end()) {
                        return;
                    }
                    using Field = FieldOf<std::remove_cvref_t<decltype(signalMember)>>;
                    if (std::optional<Field> value = function_generator::segmentValueAs<Field>(found->second); value) {
                        _signal.*signalMember = std::move(*value);
                    }
                };
                (takeValue(setting), ...);
            },
            segmentSettings());
    }

    // the name of the segment in force and the ten values the waveform runs, in the form a segment holds them
    void reportSegment() {
        using namespace function_generator;
        status.current_segment = property_map{{"segment", settings.segment.value}, createPropertyMapEntry(ParameterType::signal_type, _signal.signal_type),           //
            createPropertyMapEntry(ParameterType::start_value, _signal.start_value), createPropertyMapEntry(ParameterType::final_value, _signal.final_value),         //
            createPropertyMapEntry(ParameterType::duration, _signal.duration), createPropertyMapEntry(ParameterType::round_off_time, _signal.round_off_time),         //
            createPropertyMapEntry(ParameterType::impulse_time0, _signal.impulse_time0), createPropertyMapEntry(ParameterType::impulse_time1, _signal.impulse_time1), //
            createPropertyMapEntry(ParameterType::tone_frequency, _signal.tone_frequency), createPropertyMapEntry(ParameterType::phase, _signal.phase), {"seed", _signal.seed}};
    }

    void configureGeneratorsFromSettings() noexcept {
        using enum function_generator::SignalType;
        if (_signal.signal_type == uniform_noise || _signal.signal_type == triangular_noise || _signal.signal_type == gaussian_noise) {
            const auto noiseType = static_cast<gr::signal::NoiseType>(static_cast<int>(_signal.signal_type) - static_cast<int>(uniform_noise));
            _noise.configure(noiseType, static_cast<double>(_signal.start_value), 0., _signal.seed); // start_value = noise amplitude
        } else if (function_generator::isToneType(_signal.signal_type)) {
            // for tone types: final_value = amplitude, start_value = offset (reused from ramp semantics)
            _tone.configure(function_generator::toToneType(_signal.signal_type), static_cast<double>(_signal.tone_frequency), static_cast<double>(settings.sample_rate), static_cast<double>(_signal.phase), static_cast<double>(_signal.final_value), static_cast<double>(_signal.start_value));
            _tone.reset();
        }
    }

    static constexpr T toOutputType(double value) noexcept {
        if constexpr (std::same_as<T, double>) {
            return value;
        } else if constexpr (std::floating_point<T>) {
            return static_cast<T>(value);
        } else if constexpr (std::integral<T>) {
            return gr::signal::detail::clampToInt<T>(value);
        } else {
            return T(0); // unreachable for supported types
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_FUNCTION_GENERATOR_HPP
