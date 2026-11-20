#ifndef GNURADIO_SQUELCH_HPP
#define GNURADIO_SQUELCH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <expected>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/analog/detail/Averager.hpp>

namespace gr::blocks::analog {

namespace detail {

inline constexpr const char* kSquelchStartOfBurst = "squelch_sob";
inline constexpr const char* kSquelchEndOfBurst   = "squelch_eob";

enum class GateState : std::uint8_t { Muted, Attack, Unmuted, Decay };

/// @brief A burst marker: the key carries the whole meaning, so the value is none.
[[nodiscard]] inline const property_map& burstTag(bool startOfBurst) {
    static const property_map start{{kSquelchStartOfBurst, gr::pmt::Value(std::monostate{})}};
    static const property_map end{{kSquelchEndOfBurst, gr::pmt::Value(std::monostate{})}};
    return startOfBurst ? start : end;
}

/**
 * @brief The four-state ramp machine shared by the ramping squelches.
 *
 * Turns a per-sample "signal present" boolean into a raised-cosine envelope and a burst whose first and last produced
 * samples are the two tag positions. The attack runs env(1)..env(ramp) and the decay env(ramp)..env(1), so a burst is
 * symmetric and never truncated; neither ramp re-consults the detector, which costs a minimum burst of 2*ramp+1
 * samples and buys immunity to chatter at the threshold. A ramp length arriving mid-ramp rescales the counter.
 */
struct GateMachine {
    std::vector<float> _envelope{1.f};
    std::uint32_t      _ramp     = 0U;
    std::uint32_t      _counter  = 0U;
    GateState          _state    = GateState::Muted;
    bool               _starting = false;
    bool               _closing  = false;

    struct Step {
        float envelope;
        bool  produce;
        bool  endOfBurst;
    };

    void reset() noexcept {
        _state    = GateState::Muted;
        _counter  = 0U;
        _starting = false;
        _closing  = false;
    }

    [[nodiscard]] bool isOpen() const noexcept { return _state == GateState::Attack || _state == GateState::Unmuted; }

    [[nodiscard]] bool takeStart() noexcept { return std::exchange(_starting, false); }

    /// @brief Install a new ramp length, keeping a ramp in progress continuous and, at length zero, terminable.
    void setRamp(std::uint32_t length) {
        const std::uint32_t previous = _ramp;
        _ramp                        = length;
        _envelope.resize(static_cast<std::size_t>(length) + 1UZ);
        for (std::uint32_t r = 0U; r <= length; ++r) {
            _envelope[r] = length == 0U ? 1.f : static_cast<float>(0.5 * (1.0 - std::cos(std::numbers::pi * static_cast<double>(r) / static_cast<double>(length))));
        }
        if (_state != GateState::Attack && _state != GateState::Decay) {
            return;
        }
        if (length == 0U) {
            _closing = _state == GateState::Decay; // a canceled decay still owes its end-of-burst tag a sample
            _state   = _state == GateState::Attack ? GateState::Unmuted : GateState::Muted;
            _counter = 0U;
            return;
        }
        const double rescaled = std::round(static_cast<double>(_counter) * static_cast<double>(length) / static_cast<double>(previous));
        _counter              = static_cast<std::uint32_t>(std::clamp(rescaled, 1.0, static_cast<double>(length)));
    }

    [[nodiscard]] Step step(bool unmute) noexcept {
        if (_closing) {
            _closing = false;
            return {0.f, true, true};
        }
        if (_state == GateState::Muted) {
            if (!unmute) {
                return {0.f, false, false};
            }
            _state    = _ramp > 0U ? GateState::Attack : GateState::Unmuted;
            _counter  = 0U;
            _starting = _ramp == 0U;
        }
        return _state == GateState::Attack ? attack() : (_state == GateState::Unmuted ? open(unmute) : decay());
    }

private:
    [[nodiscard]] Step attack() noexcept {
        ++_counter;
        _starting            = _starting || _counter == 1U;
        const float envelope = _envelope[_counter];
        if (_counter == _ramp) {
            _state = GateState::Unmuted;
        }
        return {envelope, true, false};
    }

    [[nodiscard]] Step open(bool unmute) noexcept {
        if (unmute) {
            return {1.f, true, false};
        }
        if (_ramp == 0U) {
            _state = GateState::Muted;
            return {1.f, true, true};
        }
        _state   = GateState::Decay; // the sample the detector closed on still goes out at full amplitude
        _counter = _ramp;
        return {1.f, true, false};
    }

    [[nodiscard]] Step decay() noexcept {
        const float envelope = _envelope[_counter];
        --_counter;
        if (_counter == 0U) {
            _state = GateState::Muted;
            return {envelope, true, true};
        }
        return {envelope, true, false};
    }
};

template<typename T>
[[nodiscard]] constexpr double magnitudeSquared(T sample) noexcept {
    if constexpr (std::same_as<T, float>) {
        return static_cast<double>(sample) * static_cast<double>(sample);
    } else {
        return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
    }
}

/**
 * @brief Single-pole power average against a linear threshold; the dB conversion happens on set, not per sample.
 *
 * `begin` copies the average into a `Pass`, which a call holds in a local and updates per sample. `end` stores it back
 * and sets an average below the smallest normal `double` to zero. A silent input decays the average into the
 * subnormals, where each operation on it costs a microcode assist.
 */
struct PowerDetector {
    double _power     = 0.0;
    double _alpha     = 1e-4;
    double _threshold = 1e-15;

    struct Pass {
        double _power;
        double _alpha;
        double _threshold;

        [[nodiscard]] bool unmuted(double squared) noexcept {
            _power = (1.0 - _alpha) * _power + _alpha * squared;
            return _power >= _threshold;
        }

        template<typename T>
        [[nodiscard]] bool detectorUnmuted(T sample) noexcept {
            return unmuted(detail::magnitudeSquared(sample));
        }
    };

    void reset() noexcept { _power = 0.0; }

    /// @brief Sets the threshold from a level in dB. A positive threshold below the smallest normal `double` becomes
    /// that value: an average is either zero or normal once flushed, and the unflushed average of a call must decide
    /// as the flushed one does.
    void setThresholdDb(double thresholdDb) noexcept {
        const double threshold = std::pow(10.0, thresholdDb / 10.0);
        _threshold             = threshold > 0.0 ? std::max(threshold, std::numeric_limits<double>::min()) : threshold;
    }

    [[nodiscard]] Pass begin() const noexcept { return {_power, _alpha, _threshold}; }

    void end(const Pass& pass) noexcept { _power = pass._power < std::numeric_limits<double>::min() ? 0.0 : pass._power; }
};

/// @brief One single-bin DFT of length N by the three-term recursion, normalized so |X| does not depend on N.
struct Goertzel {
    double _cosine = 0.0;
    double _sine   = 0.0;
    double _s1     = 0.0;
    double _s2     = 0.0;

    void configure(double frequency, double sampleRate) {
        const double omega = 2.0 * std::numbers::pi * frequency / sampleRate;
        _cosine            = 2.0 * std::cos(omega);
        _sine              = std::sin(omega);
        reset();
    }

    void reset() noexcept { _s1 = _s2 = 0.0; }

    void push(double sample) noexcept {
        const double next = sample + _cosine * _s1 - _s2;
        _s2               = _s1;
        _s1               = next;
    }

    [[nodiscard]] double magnitude(std::uint32_t length) const noexcept {
        const double real = 0.5 * _cosine * _s1 - _s2;
        const double imag = _sine * _s1;
        return std::sqrt(real * real + imag * imag) / static_cast<double>(length);
    }
};

/**
 * @brief Ports, tags and gating for a squelch whose detector the owning block supplies.
 *
 * The detector handed to `run` provides `detectorUnmuted(sample)`; everything here is the part the ramping squelches
 * would otherwise each get subtly differently. Tag forwarding is manual because the produced-sample map is
 * data-dependent while gating: an input tag on a suppressed sample moves forward to the next produced one, and one with
 * no produced sample left in the call is held rather than dropped. The key filter is the framework's, so an owned key
 * such as `sample_rate` leaves carrying the owner's value.
 */
template<typename TOwner>
struct SquelchGate {
    GateMachine               _gate{};
    std::vector<property_map> _carry{};

    void reset() {
        _gate.reset();
        _carry.clear();
    }

    template<typename TInSpan, typename TOutSpan>
    [[nodiscard]] std::size_t run(TOwner& owner, auto& detector, TInSpan& inSpan, TOutSpan& outSpan, std::size_t nSamples, bool gating) {
        const std::size_t nTags  = inSpan.rawTags.size();
        std::size_t       cursor = 0UZ;
        std::size_t       made   = 0UZ;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            if (cursor < nTags) {
                collect(owner, inSpan, i, cursor);
            }
            const GateMachine::Step step = _gate.step(detector.detectorUnmuted(inSpan[i]));
            if (!step.produce && gating) {
                continue;
            }
            outSpan[made] = inSpan[i] * step.envelope;
            emit(outSpan, made, step.endOfBurst);
            ++made;
        }
        return made;
    }

private:
    template<typename TInSpan>
    void collect(TOwner& owner, TInSpan& inSpan, std::size_t index, std::size_t& cursor) {
        const auto&       raw   = inSpan.rawTags;
        const std::size_t until = inSpan.streamIndex + index;
        while (cursor < raw.size() && raw[cursor].index <= until) {
            if (raw[cursor].index >= inSpan.streamIndex) {
                _carry.push_back(owner.filterAndSubstituteTag(raw[cursor].map));
            }
            ++cursor;
        }
    }

    template<typename TOutSpan>
    void emit(TOutSpan& outSpan, std::size_t at, bool endOfBurst) {
        for (const property_map& forwarded : _carry) {
            outSpan.publishTag(forwarded, at);
        }
        _carry.clear();
        if (_gate.takeStart()) {
            outSpan.publishTag(burstTag(true), at);
        }
        if (endOfBurst) {
            outSpan.publishTag(burstTag(false), at);
        }
    }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::PowerSquelch, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct PowerSquelch : Block<PowerSquelch<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Silences a stream while its average power sits below a threshold, with click-free ramps and burst tags.

The detector is a single-pole average of `|x[n]|^2` compared against `threshold_db`, which is a power level: `-6.02 dB`
sits at magnitude `0.5`, not `0.25`. The average starts at zero, so the block starts muted and needs roughly `1/alpha`
samples to settle - about 0.2 s at 48 kHz at the default - which is why a squelch appears to take a moment to open.

`alpha` is the averager coefficient; `averaging_time_s`, in seconds and when positive, overrides it with
`alpha = 1 - exp(-1/(sample_rate * averaging_time_s))`, and `current_alpha` reports the coefficient in force. `threshold_db` reads back exactly as
set, never round-tripped through the linear form, so a slider does not drift. `threshold_min_db`, `threshold_max_db`
and `threshold_step_db` are advisory metadata for a user interface and take no part in processing.

`ramp` is the length in samples of a raised-cosine attack and decay; `0` makes transitions instantaneous. A burst is
the closed interval between the `squelch_sob` and `squelch_eob` tags and every sample in it carries signal, both ramps
included, so a consumer slicing on the tag pair gets exactly the audio the squelch passed. With `gate = false` the
block is exactly 1:1 and emits zeros while muted; with `gate = true` it emits nothing while muted and the output rate
becomes data-dependent. Input tags are re-published by hand: while gating, a tag on a suppressed sample moves forward
to the next produced sample rather than to an offset the rate change has invalidated. Every key survives that
re-publication with the value it arrived with.

Average power below `threshold_db` mutes the output. `alpha` is the single-pole power averager's coefficient; a positive
`averaging_time_s` sets it from that time constant instead. `sample_rate` can follow gr:sample_rate. `ramp` is the
attack and decay length. With `gate`, muted samples are dropped instead of zeroed. `threshold_min_db`,
`threshold_max_db` and `threshold_step_db` are an advisory range for a user interface. `unmuted` is true while the gate
is open or attacking.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<double, Unit<"dB">, Doc<"squelch threshold">>                     threshold_db      = -150.0;
        Annotated<double, Doc<"averager coefficient">, Limits<Over<0.0>, Max<1.0>>> alpha             = 1e-4;
        Annotated<double, Unit<"s">, Doc<"averaging time constant (0 = alpha)">>    averaging_time_s  = 0.0;
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>         sample_rate       = 48000.f;
        Annotated<gr::Size_t, Unit<"samples">, Doc<"ramp length (0 = instant)">>    ramp              = 0U;
        Annotated<bool, Doc<"muted sample dropping">>                               gate              = false;
        Annotated<double, Unit<"dB">, Doc<"threshold UI minimum">>                  threshold_min_db  = -150.0;
        Annotated<double, Unit<"dB">, Doc<"threshold UI maximum">>                  threshold_max_db  = 10.0;
        Annotated<double, Unit<"dB">, Doc<"threshold UI step">>                     threshold_step_db = 1.0;

        GR_MAKE_REFLECTABLE(Settings, threshold_db, alpha, averaging_time_s, sample_rate, ramp, gate, threshold_min_db, threshold_max_db, threshold_step_db);
    } settings;

    struct Status {
        Annotated<bool, Doc<"unmuted state">>                   unmuted       = false;
        Annotated<double, Doc<"averager coefficient in force">> current_alpha = 1e-4;

        GR_MAKE_REFLECTABLE(Status, unmuted, current_alpha);
    } status;

    GR_MAKE_REFLECTABLE(PowerSquelch, in, out, settings, status);

    detail::PowerDetector                _detector{};
    detail::SquelchGate<PowerSquelch<T>> _machine{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (!(detail::averagerCoefficient(s.alpha, s.averaging_time_s, static_cast<double>(s.sample_rate)) > 0.0)) {
            return gr::Refused(s.averaging_time_s, "must give a pole above 0 at this sample_rate");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _detector._alpha = detail::averagerCoefficient(s.alpha, s.averaging_time_s, static_cast<double>(s.sample_rate));
        _detector.setThresholdDb(s.threshold_db);
        status.current_alpha = _detector._alpha;
        _machine._gate.setRamp(s.ramp);
    }

    void reset() {
        _detector.reset();
        _machine.reset();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        auto              pass     = _detector.begin();
        const std::size_t made     = _machine.run(*this, pass, inSpan, outSpan, nSamples, settings.gate);
        _detector.end(pass);

        status.unmuted = _machine._gate.isOpen();
        std::ignore    = inSpan.consume(nSamples);
        outSpan.publish(made);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::analog::CtcssSquelch)

struct CtcssSquelch : Block<CtcssSquelch, NoTagPropagation> {
    using Description = Doc<R""(
@brief Opens only while a specific sub-audible continuous tone is present, using three single-bin DFTs.

Three Goertzel evaluators of length `length` run at `tone_freq` and at `tone_freq * (1 -/+ epsilon)`. Every `length`
samples the block unmutes if the center magnitude reaches `level` and exceeds both neighbors; between decisions the
previous one holds, and the decision before the first block completes is mute. The `1/N` normalization makes the
magnitude independent of the integration length: a unit-amplitude sinusoid exactly on the analysis frequency gives
`0.5`.

What the neighbors can do is bounded by the integration time, a Goertzel of length `N` having a bin width of
`sample_rate/N = 1/T`. The rule reliably rejects broadband noise and tones displaced by more than about `1/T` Hz, but
it does not separate adjacent standard CTCSS tones, which are 3 to 8 Hz apart, because no 100 ms measurement can. A
caller who needs that lengthens `length`, widens `epsilon` to match, and accepts the slower open.

`sample_rate` is a live setting, so a rate change retunes the evaluators instead of needing a new graph; that,
`tone_freq`, `length` or `epsilon` discards the partial integration and restarts, and the standing decision holds until
the first new block completes. Ramping, gating, the burst tags and the tag re-publication are the shared squelch
machine documented on PowerSquelch.

`sample_rate` can follow gr:sample_rate. The normalized magnitude of the center bin must reach `level` to unmute; the
neighbor bins sit at tone_freq * (1 -/+ epsilon). `ramp` is the attack and decay length. With `gate`, muted samples are
dropped instead of zeroed. `level_min`, `level_max` and `level_step` are an advisory range for a user interface.
`unmuted` is true while the gate is open or attacking.
)"">;

    PortIn<float>  in;
    PortOut<float> out;

    struct Settings {
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>                      sample_rate = 8000.f;
        Annotated<double, Unit<"Hz">, Doc<"tone frequency">, Limits<Over<0.0>>>                  tone_freq   = 100.0;
        Annotated<double, Doc<"unmute level">>                                                   level       = 0.01;
        Annotated<gr::Size_t, Unit<"samples">, Doc<"integration length (0 = sample_rate / 10)">> length      = 0U;
        Annotated<double, Doc<"neighbor bin spacing">, Limits<Over<0.0>>>                        epsilon     = 0.02;
        Annotated<gr::Size_t, Unit<"samples">, Doc<"ramp length (0 = instant)">>                 ramp        = 0U;
        Annotated<bool, Doc<"muted sample dropping">>                                            gate        = false;
        Annotated<double, Doc<"level UI minimum">>                                               level_min   = 0.0;
        Annotated<double, Doc<"level UI maximum">>                                               level_max   = 1.0;
        Annotated<double, Doc<"level UI step">>                                                  level_step  = 0.01;

        GR_MAKE_REFLECTABLE(Settings, sample_rate, tone_freq, level, length, epsilon, ramp, gate, level_min, level_max, level_step);
    } settings;

    struct Status {
        Annotated<bool, Doc<"unmuted state">> unmuted = false;

        GR_MAKE_REFLECTABLE(Status, unmuted);
    } status;

    GR_MAKE_REFLECTABLE(CtcssSquelch, in, out, settings, status);

    std::array<detail::Goertzel, 3UZ> _bins{};
    std::array<double, 3UZ>           _magnitude{};
    std::uint32_t                     _length = 800U;
    std::uint32_t                     _filled = 0U;
    bool                              _open   = false;
    detail::SquelchGate<CtcssSquelch> _machine{};

    // the private members make the block no aggregate, so it takes the constructors of its base
    using Block<CtcssSquelch, NoTagPropagation>::Block;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.tone_freq * (1.0 + s.epsilon) >= 0.5 * static_cast<double>(s.sample_rate)) {
            return gr::Refused("tone_freq * (1 + epsilon) must be < sample_rate / 2");
        }
        if (s.length == 0U && s.sample_rate < 10.f) {
            return gr::Refused(s.sample_rate, "must be >= 10 when length is 0");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _length = s.length == 0U ? static_cast<std::uint32_t>(s.sample_rate / 10.f) : static_cast<std::uint32_t>(s.length);
        _bins[0].configure(s.tone_freq * (1.0 - s.epsilon), static_cast<double>(s.sample_rate));
        _bins[1].configure(s.tone_freq, static_cast<double>(s.sample_rate));
        _bins[2].configure(s.tone_freq * (1.0 + s.epsilon), static_cast<double>(s.sample_rate));
        _filled = 0U;
        _machine._gate.setRamp(s.ramp);
    }

    void reset() {
        for (detail::Goertzel& bin : _bins) {
            bin.reset();
        }
        _filled = 0U;
        _open   = false;
        _machine.reset();
    }

    [[nodiscard]] bool detectorUnmuted(float sample) noexcept {
        for (detail::Goertzel& bin : _bins) {
            bin.push(static_cast<double>(sample));
        }
        ++_filled;
        return _filled < _length ? _open : decide();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        const std::size_t made     = _machine.run(*this, *this, inSpan, outSpan, nSamples, settings.gate);

        status.unmuted = _machine._gate.isOpen();
        std::ignore    = inSpan.consume(nSamples);
        outSpan.publish(made);
        return work::Status::OK;
    }

    /// @brief The magnitudes the most recent decision was taken on, in the order lower, center, upper.
    [[nodiscard]] double lastMagnitude(std::size_t which) const noexcept { return _magnitude[which]; }

    [[nodiscard]] std::uint32_t integrationLength() const noexcept { return _length; }

private:
    [[nodiscard]] bool decide() noexcept {
        for (std::size_t which = 0UZ; which < _bins.size(); ++which) {
            _magnitude[which] = _bins[which].magnitude(_length);
            _bins[which].reset();
        }
        _filled = 0U;
        _open   = _magnitude[1] >= settings.level && _magnitude[1] > _magnitude[0] && _magnitude[1] > _magnitude[2];
        return _open;
    }
};

GR_REGISTER_BLOCK(gr::blocks::analog::SimpleSquelch, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct SimpleSquelch : Block<SimpleSquelch<T>> {
    using Description = Doc<R""(
@brief A hard power gate: no ramp, no burst tags, no rate change.

The same single-pole power detector PowerSquelch uses, wired straight to a multiply by one or zero, for a chain that
wants exactly that and no ramp state to reason about. For the same `alpha` and `threshold_db` it agrees with
PowerSquelch at `ramp = 0`, `gate = false` on every sample; the two detectors are one piece of code.

The average starts at zero, so the block starts muted, and `threshold_db` is a power level: the conversion uses `/10`
and happens once per settings change. Always exactly 1:1 and synchronous. Every input tag leaves at its own offset
with the values it arrived with.

Average power below `threshold_db` mutes the output. `alpha` is the single-pole power averager's coefficient; a positive
`averaging_time_s` sets it from that time constant instead. `sample_rate` can follow gr:sample_rate. `unmuted` reports
the most recent sample's decision.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<double, Unit<"dB">, Doc<"squelch threshold">>                     threshold_db     = -150.0;
        Annotated<double, Doc<"averager coefficient">, Limits<Over<0.0>, Max<1.0>>> alpha            = 1e-4;
        Annotated<double, Unit<"s">, Doc<"averaging time constant (0 = alpha)">>    averaging_time_s = 0.0;
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>         sample_rate      = 48000.f;

        GR_MAKE_REFLECTABLE(Settings, threshold_db, alpha, averaging_time_s, sample_rate);
    } settings;

    struct Status {
        Annotated<bool, Doc<"unmuted state">>                   unmuted       = false;
        Annotated<double, Doc<"averager coefficient in force">> current_alpha = 1e-4;

        GR_MAKE_REFLECTABLE(Status, unmuted, current_alpha);
    } status;

    GR_MAKE_REFLECTABLE(SimpleSquelch, in, out, settings, status);

    detail::PowerDetector _detector{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (!(detail::averagerCoefficient(s.alpha, s.averaging_time_s, static_cast<double>(s.sample_rate)) > 0.0)) {
            return gr::Refused(s.averaging_time_s, "must give a pole above 0 at this sample_rate");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _detector._alpha = detail::averagerCoefficient(s.alpha, s.averaging_time_s, static_cast<double>(s.sample_rate));
        _detector.setThresholdDb(s.threshold_db);
        status.current_alpha = _detector._alpha;
    }

    void reset() { _detector.reset(); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        bool              open     = status.unmuted.value;
        auto              pass     = _detector.begin();
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            open      = pass.detectorUnmuted(input[i]);
            output[i] = open ? input[i] : T{};
        }
        _detector.end(pass);
        status.unmuted = open;
        return work::Status::OK;
    }
};

} // namespace gr::blocks::analog

#endif // GNURADIO_SQUELCH_HPP
