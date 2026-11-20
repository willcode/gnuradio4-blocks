#ifndef GNURADIO_NOISE_BLANKER_HPP
#define GNURADIO_NOISE_BLANKER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <expected>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/analog/detail/Averager.hpp>

namespace gr::blocks::analog {

namespace detail {

inline constexpr const char* kNoiseBlanked = "noise_blanked";

enum class Replacement : std::uint8_t { interpolate, hold, zero };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::analog::NoiseBlanker, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct NoiseBlanker : Block<NoiseBlanker<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Removes impulse noise from ignition, arcing, radar or switching supplies by replacing the offending samples.

The block works on a float or complex stream ahead of the filters that would spread an impulse. It gives one sample out
for each one in, the input delayed by `delay_samples`, the sum of `lookback_samples` and `blank_samples`. The delay
holds while the block is disabled. Every tag moves by `delay_samples` with its sample. With `emit_tags` set, the block
publishes a `noise_blanked` tag on the last replaced sample of each run, stating the run's length. At the end of the
stream the tags still held leave at the end-of-stream index.

A sample whose power `|x|^2` exceeds the tracked mean power by `threshold_db` opens a window. The window replaces the
`lookback_samples` before that sample and `blank_samples` from it on. A sample whose power is not finite counts as a
detection. The tracker leaves out each detected sample and the rest of its window. A detected sample never raises the
threshold it is compared against. At the default settings the block blanks about 0.18 % of a clean
circular-complex-Gaussian stream. With `retrigger` set, a detection inside a window restarts it, up to
`max_window_samples`.

`replacement` sets what replaces a run. `interpolate`, the default, draws a line from the last sample before the run to
the first after it. `hold` repeats the sample before the run, and `zero` writes zeros. Under `interpolate`, a run longer
than the delay line holds the sample before it until the run's end enters the line, then interpolates over the samples
left.

`averaging_time` sets the tracker's time constant in seconds at `sample_rate`. At 0 the block takes `alpha` as the pole,
and otherwise it derives the pole; `current_alpha` reports the pole in force. The block refuses a pole outside (0, 1] and a `replacement`
other than the three names. The tracker learns the level
during a warm-up of one averaging time, in which only a sample whose power is not finite counts as a detection. Enabling
the block and a change of the pole restart the warm-up. A count rises with each input from a detection to the end of its
window and falls with each passed one. When it reaches one averaging time of samples, the warm-up restarts too. A rise
of the level above the threshold is therefore replaced for about one averaging time and then passes.

A change of `lookback_samples` or `blank_samples` resizes the line at once and keeps the newest inputs. A shorter line
drops its oldest inputs. Their tags, and the marker of a run that ends among them, move to the next sample out. A longer
line puts out as many zeros as it grew by ahead of the kept inputs, or extends the run that is leaving. `reset()` drops
every input in the line as a shorter line does, fills the line with zeros and restarts the tracker.

`tracked_power` reports the mean power the threshold is taken against. `blanked_fraction` reports a slow average, over
ten tracker time constants, of the fraction of inputs from a detection to the end of its window. It leaves out the
`lookback_samples` replaced ahead of each detection. At the default settings it reads about 0.137 % on a clean
circular-complex-Gaussian stream, against about 0.18 % replaced. The block writes both to its members and to its
settings at the end of each call that changes them.

A sample is detected when its power exceeds the tracked mean power by `threshold_db`. With `averaging_time` 0, `alpha`
is the tracker's pole. A detection replaces `blank_samples` from the detected sample and `lookback_samples` before it.
With `retrigger`, a detection inside a window restarts it, up to `max_window_samples`, which 0 sets to 4 *
blank_samples. With `emit_tags`, a noise_blanked tag holding the run's length marks each run's last replaced sample.
`delay_samples` is lookback_samples + blank_samples and holds also while blanking is disabled. `tracked_power` is the
censored mean power the threshold is taken against; `blanked_fraction` is a slow average of the window duty.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<bool, Doc<"blanking">, Visible>                                                enabled            = false;
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>                      sample_rate        = 96000.f;
        Annotated<double, Unit<"dB">, Doc<"detection threshold">, Visible>                       threshold_db       = 9.32;
        Annotated<double, Unit<"s">, Doc<"tracker time constant (0 = alpha)">, Limits<Min<0.0>>> averaging_time     = 0.01;
        Annotated<double, Doc<"tracker pole">, Limits<Over<0.0>, Max<1.0>>>                      alpha              = 1e-3;
        Annotated<gr::Size_t, Unit<"samples">, Doc<"blanked run length">, Limits<Min<1U>>>       blank_samples      = 7U;
        Annotated<gr::Size_t, Unit<"samples">, Doc<"lookback length">>                           lookback_samples   = 2U;
        Annotated<detail::Replacement, Doc<"replacement">, Visible>                              replacement        = detail::Replacement::interpolate;
        Annotated<bool, Doc<"retriggering">>                                                     retrigger          = false;
        Annotated<gr::Size_t, Unit<"samples">, Doc<"retriggered window limit (0 = derived)">>    max_window_samples = 0U;
        Annotated<bool, Doc<"blanking tags">>                                                    emit_tags          = false;

        GR_MAKE_REFLECTABLE(Settings, enabled, sample_rate, threshold_db, averaging_time, alpha, blank_samples, lookback_samples, replacement, retrigger, max_window_samples, emit_tags);
    } settings;

    struct Status {
        Annotated<gr::Size_t, Unit<"samples">, Doc<"delay">> delay_samples    = 9U;
        Annotated<double, Doc<"tracked mean power">>         tracked_power    = 0.0;
        Annotated<double, Doc<"blanked fraction">>           blanked_fraction = 0.0;

        Annotated<double, Doc<"tracker pole in force">> current_alpha = 1e-3;

        GR_MAKE_REFLECTABLE(Status, delay_samples, tracked_power, blanked_fraction, current_alpha);
    } status;

    GR_MAKE_REFLECTABLE(NoiseBlanker, in, out, settings, status);

    std::vector<T>                                      _line{};
    std::vector<std::uint8_t>                           _marked{};      // a byte per slot and not a bitset, read and written once per sample
    std::vector<std::pair<std::uint64_t, property_map>> _pendingTags{}; ///< forwarded tags, at the output index their sample leaves at
    std::size_t                                         _cursor       = 0UZ;
    std::size_t                                         _oldest       = 1UZ;
    std::size_t                                         _ring         = 10UZ;
    std::size_t                                         _delay        = 9UZ;
    std::size_t                                         _lookback     = 2UZ;
    std::size_t                                         _blank        = 7UZ;
    std::size_t                                         _maxWindow    = 28UZ;
    double                                              _alpha        = 1e-3;
    double                                              _threshold    = 8.5507;
    std::size_t                                         _warmup       = 960UZ;
    std::size_t                                         _warmupLength = 960UZ;
    std::uint32_t                                       _remaining    = 0U;
    std::uint32_t                                       _windowLength = 0U;
    std::uint32_t                                       _excess       = 0U;   ///< replaced inputs less passed ones, never below zero
    std::uint32_t                                       _runPosition  = 0U;   ///< replaced outputs of the current run so far
    std::uint32_t                                       _held         = 0U;   ///< of those, the outputs that held the sample before the run
    std::uint64_t                                       _emitted      = 0ULL; ///< outputs since start(), the index of the next one
    T                                                   _anchor{};

    // the block has private members, so it takes its settings through the base's constructor
    using Block<NoiseBlanker<T>, NoTagPropagation>::Block;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (!(detail::averagerCoefficient(s.alpha, s.averaging_time, static_cast<double>(s.sample_rate)) > 0.0)) {
            return gr::Refused(s.averaging_time, "must give a pole above 0 at this sample_rate");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        const double pole = detail::averagerCoefficient(s.alpha, s.averaging_time, static_cast<double>(s.sample_rate));
        // the tracked power is stale after a pole move or after the tracker was disabled
        const bool restart = pole != _alpha || (s.enabled && s.enabled.changed());
        derive(pole);
        if (restart) {
            _warmup = _warmupLength;
        }
        if (s.lookback_samples.changed() || s.blank_samples.changed()) {
            relay(s.lookback_samples, s.blank_samples);
        }
    }

    /// @brief The tracker's pole, the threshold and the window cap the settings give; `current_alpha` reports the pole.
    void derive(double pole) {
        _alpha               = pole;
        status.current_alpha = pole;
        _threshold           = std::pow(10.0, settings.threshold_db / 10.0);
        _maxWindow           = settings.max_window_samples == 0U ? 4UZ * settings.blank_samples : static_cast<std::size_t>(settings.max_window_samples);
        _warmupLength        = settings.averaging_time > 0.0 ? static_cast<std::size_t>(std::ceil(static_cast<double>(settings.sample_rate) * settings.averaging_time)) : static_cast<std::size_t>(std::ceil(1.0 / pole));
    }

    /// @brief The tracker's state is its two status members.
    void reportTracker(double power, double duty) {
        status.tracked_power    = power;
        status.blanked_fraction = duty;
    }

    void start() {
        _pendingTags.clear();
        _emitted = 0ULL;
        restart();
    }

    /// @brief Restart the line and the tracker. The line's tags move to the next sample out. `start()` discards them.
    void reset() {
        drop(_delay);
        restart();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        const std::size_t nTags    = inSpan.rawTags.size();
        std::size_t       tag      = 0UZ;
        std::size_t       leaving  = 0UZ;

        if (nSamples > 0UZ && passesUnchanged()) {
            leaving = passLine(inSpan, outSpan, nSamples);
        } else {
            // The block forwards the tags itself. Each tag waits `delay_samples` for its sample and leaves in index
            // order with the block's own markers. The filter is the framework's.
            double meanPower = status.tracked_power.value;
            double meanDuty  = status.blanked_fraction.value;
            for (std::size_t i = 0UZ; i < nSamples; ++i) {
                const std::uint64_t at = _emitted + i;
                while (tag < nTags && inSpan.rawTags[tag].index <= inSpan.streamIndex + i) {
                    if (inSpan.rawTags[tag].index >= inSpan.streamIndex) {
                        property_map forwarded = this->filterAndSubstituteTag(inSpan.rawTags[tag].map);
                        if (!forwarded.empty()) {
                            _pendingTags.emplace_back(at + _delay, std::move(forwarded));
                        }
                    }
                    ++tag;
                }
                accept(inSpan[i], meanPower, meanDuty);
                while (leaving < _pendingTags.size() && _pendingTags[leaving].first <= at) {
                    outSpan.publishTag(_pendingTags[leaving].second, i);
                    ++leaving;
                }
                outSpan[i] = emit(outSpan, i);
            }
            reportTracker(flushed(meanPower), flushed(meanDuty));
        }
        _pendingTags.erase(_pendingTags.begin(), _pendingTags.begin() + static_cast<std::ptrdiff_t>(leaving));
        _emitted += nSamples;

        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

    /**
     * @brief At the end of the stream, publish every tag the block holds at the end-of-stream index.
     *
     * `processBulk` places every tag of an ordinary call. The framework's end-of-stream call passes `processedIn` 0 and
     * output spans of no samples, where offset 0 is the end-of-stream index. The samples in the line never leave. Their
     * forwarded tags, the markers of their runs and the tags left on the input go out there, in index order. The
     * `gr:end_of_stream` key stays behind, since the framework publishes its own.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (processedIn > 0UZ) {
            return;
        }
        std::vector<std::pair<std::uint64_t, property_map>> leaving = std::move(_pendingTags);
        _pendingTags.clear();
        if (settings.emit_tags) {
            std::uint32_t length = _runPosition;
            for (std::size_t ahead = 1UZ; ahead <= _delay; ++ahead) {
                if (_marked[slot(ahead)] == 0U) {
                    continue;
                }
                ++length;
                if (ahead == _delay || _marked[slot(ahead + 1UZ)] == 0U) {
                    leaving.emplace_back(_emitted + ahead - 1UZ, marker(length));
                    length = 0U;
                }
            }
        }
        std::ranges::stable_sort(leaving, std::ranges::less{}, &std::pair<std::uint64_t, property_map>::first);

        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        gr::for_each_reader_span(
            [this, &leaving, &endOfStream](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const gr::Tag& tag : span.rawTags) {
                    if (tag.index < span.streamIndex) { // placed by an earlier call
                        continue;
                    }
                    property_map forwarded = this->filterAndSubstituteTag(tag.map);
                    forwarded.erase(endOfStream);
                    if (!forwarded.empty()) {
                        leaving.emplace_back(std::numeric_limits<std::uint64_t>::max(), std::move(forwarded));
                    }
                }
            },
            inputSpans);

        for (const auto& held : leaving) {
            gr::for_each_writer_span(
                [&held](auto& span) {
                    if (span.isSync && span.isConnected) {
                        span.publishTag(held.second, 0UZ);
                    }
                },
                outputSpans);
        }
    }

private:
    [[nodiscard]] static constexpr double squaredMagnitude(T sample) noexcept {
        if constexpr (std::same_as<T, float>) {
            return static_cast<double>(sample) * static_cast<double>(sample);
        } else {
            return static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
        }
    }

    [[nodiscard]] static property_map marker(std::uint32_t length) { return property_map{{detail::kNoiseBlanked, static_cast<gr::Size_t>(length)}}; }

    /// The oldest entry plus @p ahead, wrapped by a compare. A compare costs less than a runtime integer modulo.
    [[nodiscard]] std::size_t slot(std::size_t ahead) const noexcept {
        const std::size_t at = _oldest + ahead;
        return at >= _ring ? at - _ring : at;
    }

    /// True when the block is disabled, no window is open, no input in the line is marked, and the excess count is
    /// below the warm-up length. Each input then leaves `delay_samples` later unchanged.
    [[nodiscard]] bool passesUnchanged() const noexcept {
        if (settings.enabled || _remaining > 0U || _excess >= _warmupLength) {
            return false;
        }
        const std::span<const std::uint8_t> marked(_marked); // every slot but `_oldest`, which holds the sample emitted last
        return !std::ranges::contains(marked.first(_oldest), std::uint8_t{1}) && !std::ranges::contains(marked.subspan(_oldest + 1UZ), std::uint8_t{1});
    }

    /**
     * @brief Move @p nSamples inputs through the line as copies, with the state the per-sample loop leaves, and
     * publish the tags due among them. Returns the number of held tags published.
     *
     * Item 0 is the sample emitted last and items 1 to `delay_samples` the inputs still in the line, item `j` in slot
     * `(_oldest + j) % _ring`. The call's inputs follow. Output `i` is item `i + 1`.
     */
    template<typename TInSpan, typename TOutSpan>
    [[nodiscard]] std::size_t passLine(TInSpan& inSpan, TOutSpan& outSpan, std::size_t nSamples) {
        std::size_t collected = 0UZ; // the input the per-sample loop takes the tag at
        for (std::size_t tag = 0UZ; tag < inSpan.rawTags.size() && inSpan.rawTags[tag].index < inSpan.streamIndex + nSamples; ++tag) {
            const std::size_t index = inSpan.rawTags[tag].index;
            if (index < inSpan.streamIndex) {
                continue;
            }
            collected              = std::max(collected, index - inSpan.streamIndex);
            property_map forwarded = this->filterAndSubstituteTag(inSpan.rawTags[tag].map);
            if (!forwarded.empty()) {
                _pendingTags.emplace_back(_emitted + collected + _delay, std::move(forwarded));
            }
        }
        std::size_t leaving = 0UZ;
        for (std::size_t at = 0UZ; leaving < _pendingTags.size(); ++leaving) { // each held tag leaves at its due output, and never ahead of the tag before it
            const std::uint64_t due   = _pendingTags[leaving].first;
            const std::uint64_t ahead = due > _emitted ? due - _emitted : 0ULL;
            if (ahead >= nSamples) {
                break;
            }
            at = std::max(at, static_cast<std::size_t>(ahead));
            outSpan.publishTag(_pendingTags[leaving].second, at);
        }

        const auto        input    = inSpan.begin();
        const auto        output   = outSpan.begin();
        const std::size_t fromLine = std::min(nSamples, _delay);
        const std::size_t start    = _oldest + 1UZ == _ring ? 0UZ : _oldest + 1UZ;
        const std::size_t head     = std::min(fromLine, _ring - start);
        std::copy_n(_line.begin() + static_cast<std::ptrdiff_t>(start), head, output);
        std::copy_n(_line.begin(), fromLine - head, output + static_cast<std::ptrdiff_t>(head));
        if (nSamples > _delay) {
            std::copy_n(input, nSamples - _delay, output + static_cast<std::ptrdiff_t>(_delay));
        }
        const std::size_t kept  = std::min(nSamples, _ring); // the newest inputs, the ones the line keeps
        const std::size_t first = (_oldest + nSamples - kept) % _ring;
        const std::size_t wrap  = std::min(kept, _ring - first);
        std::copy_n(input + static_cast<std::ptrdiff_t>(nSamples - kept), wrap, _line.begin() + static_cast<std::ptrdiff_t>(first));
        std::copy_n(input + static_cast<std::ptrdiff_t>(nSamples - kept + wrap), kept - wrap, _line.begin());

        _oldest = (_oldest + nSamples) % _ring;
        _cursor = _oldest == 0UZ ? _ring - 1UZ : _oldest - 1UZ;
        std::ranges::fill(_marked, std::uint8_t{0});
        _anchor       = output[static_cast<std::ptrdiff_t>(nSamples - 1UZ)];
        _runPosition  = 0U;
        _held         = 0U;
        _windowLength = 0U;
        _warmup       = _warmup > nSamples ? _warmup - nSamples : 0UZ;
        _excess       = _excess > nSamples ? _excess - static_cast<std::uint32_t>(nSamples) : 0U;
        double duty   = status.blanked_fraction.value;
        for (std::size_t i = 0UZ; i < nSamples && duty != 0.0; ++i) { // the duty average decays toward zero, as in accept()
            duty += 0.1 * _alpha * (0.0 - duty);
            if (duty < std::numeric_limits<double>::min()) {
                duty = 0.0;
            }
        }
        reportTracker(status.tracked_power.value, duty);
        return leaving;
    }

    /**
     * @brief Size the line for @p lookback and @p blank, keeping the newest inputs and their marks.
     *
     * Slot 0 holds the sample emitted last, and slots 1 to the delay the inputs still to leave. A longer line takes
     * zeros ahead of the kept inputs. While a run is leaving, the zeros count as part of it. The run stays whole.
     */
    void relay(std::size_t lookback, std::size_t blank) {
        const std::size_t         delay = lookback + blank;
        std::vector<T>            line(delay + 1UZ, T{});
        std::vector<std::uint8_t> marked(delay + 1UZ, std::uint8_t{0});
        if (!_line.empty()) {
            const std::size_t kept = std::min(delay, _delay);
            line[0UZ]              = _line[slot(0UZ)];
            marked[0UZ]            = _marked[slot(0UZ)];
            for (std::size_t k = 1UZ; k <= kept; ++k) {
                line[delay - kept + k]   = _line[slot(_delay - kept + k)];
                marked[delay - kept + k] = _marked[slot(_delay - kept + k)];
            }
            if (_runPosition > 0U) {
                std::fill(marked.begin() + 1, marked.begin() + static_cast<std::ptrdiff_t>(delay - kept + 1UZ), std::uint8_t{1});
            }
            if (delay >= _delay) {
                for (auto& held : _pendingTags) {
                    held.first += delay - _delay;
                }
            } else {
                drop(_delay - delay);
            }
        }
        if (marked[1UZ] == 0U) {
            _runPosition = 0U;
            _held        = 0U;
        }
        _line                = std::move(line);
        _marked              = std::move(marked);
        _ring                = delay + 1UZ;
        _cursor              = delay;
        _oldest              = 0UZ;
        _delay               = delay;
        _lookback            = lookback;
        _blank               = blank;
        _remaining           = std::min(_remaining, static_cast<std::uint32_t>(blank));
        status.delay_samples = static_cast<gr::Size_t>(delay);
    }

    /// @brief Fill the line with zeros, clear its marks and restart the tracker.
    void restart() {
        std::ranges::fill(_line, T{});
        _marked.assign(_ring, std::uint8_t{0});
        _cursor = 0UZ;
        _oldest = 1UZ;
        reportTracker(0.0, 0.0);
        _warmup       = _warmupLength;
        _remaining    = 0U;
        _windowLength = 0U;
        _excess       = 0U;
        _runPosition  = 0U;
        _held         = 0U;
        _anchor       = T{};
    }

    /**
     * @brief Drop the @p dropped oldest inputs still to leave, as a shorter line or a reset does.
     *
     * Their tags, and the marker of each run that ends among them, move to the next sample out, in index order. A run
     * on the newest input of the line ends there. The last dropped input that was not replaced becomes the sample
     * before a later run. A run that goes on past the dropped inputs counts them in its length and in its
     * interpolation.
     */
    void drop(std::size_t dropped) {
        const bool                                          holding = _runPosition > 0U && _held == _runPosition;
        std::uint32_t                                       length  = _runPosition;
        std::size_t                                         tag     = 0UZ;
        std::vector<std::pair<std::uint64_t, property_map>> pending;
        pending.reserve(_pendingTags.size() + dropped);
        for (std::size_t ahead = 1UZ; ahead <= dropped; ++ahead) {
            for (; tag < _pendingTags.size() && _pendingTags[tag].first < _emitted + ahead; ++tag) {
                pending.emplace_back(_emitted, std::move(_pendingTags[tag].second));
            }
            if (_marked[slot(ahead)] == 0U) {
                _anchor = _line[slot(ahead)];
                continue;
            }
            ++length;
            if (ahead == _delay || _marked[slot(ahead + 1UZ)] == 0U) {
                if (settings.emit_tags) {
                    pending.emplace_back(_emitted, marker(length));
                }
                length = 0U;
            }
        }
        for (; tag < _pendingTags.size(); ++tag) {
            pending.emplace_back(_pendingTags[tag].first - dropped, std::move(_pendingTags[tag].second));
        }
        const bool goesOn = _runPosition > 0U && length == _runPosition + dropped;
        _held             = goesOn ? (holding ? length : _held) : 0U;
        _runPosition      = length;
        _pendingTags      = std::move(pending);
    }

    /// @brief A running average below the smallest normal `double` is zero.
    [[nodiscard]] static constexpr double flushed(double average) noexcept { return average < std::numeric_limits<double>::min() ? 0.0 : average; }

    /// @brief Take one input. Decide first, then track the power from the samples the decision left in place.
    void accept(T sample, double& meanPower, double& meanDuty) {
        const double power      = squaredMagnitude(sample);
        const bool   windowOpen = _remaining > 0U;

        // Compare first and update after. A blanker that updates from every sample, impulse included, and
        // then tests against the value the impulse just raised desensitizes itself. A power that is not finite fails
        // every comparison and counts as a detection.
        bool detected = false;
        if (_warmup > 0UZ) {
            --_warmup;
            detected = settings.enabled && !(power < std::numeric_limits<double>::infinity());
        } else if (settings.enabled) {
            detected = !(power <= _threshold * meanPower);
        }
        if (!windowOpen && !detected && settings.enabled) {
            meanPower += _alpha * (power - meanPower);
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
        _excess          = marked ? _excess + 1U : (_excess > 0U ? _excess - 1U : 0U);
        if (_excess >= _warmupLength) { // a level rise keeps most samples above the threshold: learn the new level
            _warmup = _warmupLength;
            _excess = 0U;
        }
        meanDuty += 0.1 * _alpha * ((marked ? 1.0 : 0.0) - meanDuty); // ten tracker time constants, since a 0.14 % duty needs a longer average than the power

        if (detected && windowOpen && settings.retrigger) {
            const std::uint32_t room = _windowLength > static_cast<std::uint32_t>(_maxWindow) ? 0U : static_cast<std::uint32_t>(_maxWindow) + 1U - _windowLength;
            _remaining               = std::min(static_cast<std::uint32_t>(_blank), room);
        }
        if (_remaining > 0U) {
            --_remaining;
        }
    }

    /**
     * @brief Produce the output for the sample that entered `delay_samples` ago, replaced if it was marked.
     *
     * A run whose end is in the line is interpolated from the sample before it to the sample after it. A run that
     * reaches past the line holds the sample before it until its end enters the line. The interpolation then spans
     * the samples left. The marker goes on the run's last replaced sample.
     */
    template<typename TOutSpan>
    [[nodiscard]] T emit(TOutSpan& outSpan, std::size_t at) {
        const std::size_t oldest = slot(0UZ);
        if (_marked[oldest] == 0U) {
            _anchor      = _line[oldest];
            _runPosition = 0U;
            _held        = 0U;
            return _anchor;
        }

        std::size_t ahead = 1UZ;
        while (ahead <= _delay && _marked[slot(ahead)] != 0U) {
            ++ahead;
        }
        ++_runPosition;
        if (ahead > _delay) {
            _held = _runPosition;
            return settings.replacement.value == detail::Replacement::zero ? T{} : _anchor;
        }

        const std::uint32_t position = _runPosition - _held;
        const std::uint32_t span     = position + static_cast<std::uint32_t>(ahead) - 1U;
        if (ahead == 1UZ) {
            if (settings.emit_tags) {
                outSpan.publishTag(marker(_runPosition), at);
            }
            _runPosition = 0U;
            _held        = 0U;
        }

        switch (settings.replacement.value) {
        case detail::Replacement::zero: return T{};
        case detail::Replacement::hold: return _anchor;
        default: break;
        }
        const T      after  = _line[slot(ahead)];
        const double weight = static_cast<double>(position) / static_cast<double>(span + 1U);
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
