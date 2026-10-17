#ifndef GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP
#define GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <initializer_list>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::SchmidlCoxSync)

/**
 * @brief Estimates coarse frame timing and fractional carrier frequency offset from a repeated-half preamble.
 *
 * With `L = fft_len/2`, the block runs two sums over the stream.
 *
 *     P(d) = sum_{m=0..L-1} conj(r[d+m]) * r[d+m+L]      R(d) = sum_{m=0..L-1} |r[d+m+L]|^2
 *
 * Both are sliding updates. `P(d+1) = P(d) + conj(r[d+L])*r[d+2L] - conj(r[d])*r[d+L]`, and `R` has the matching
 * update. Each sample costs one complex multiply and one squared magnitude, whatever `fft_len` is. The stream before
 * its first sample counts as silence. The recurrence is then exact from the first sample and needs no warm-up.
 *
 * The block tests `M(d) = |P(d)|^2 / R(d)^2` as `|P(d)|^2 > threshold * R(d)^2`, the same comparison without the
 * division. `M` forms a plateau at a preamble. The correlation is exact for every `d` from the preamble's first prefix
 * sample to `cp_len` samples later, and the plateau is `cp_len + 1` samples wide. The block takes the midpoint of the
 * threshold crossings as the timing instant.
 *
 * The trigger goes on the frame's first sample, the first sample of the preamble's own cyclic prefix. It lies
 * `cp_len/2` before the plateau midpoint. A window `d` holds a pair of repeated samples only for `d` from `L - 1`
 * samples before the frame's first sample to `cp_len + L - 1` samples after it. A crossing wider than those
 * `cp_len + fft_len - 1` positions is refused and counted in `nWidePlateaus()`. The sample that ends an accepted
 * crossing arrives at most `3*fft_len/2 + cp_len - 1` samples after the trigger's sample. `out` lags `in` by that many
 * samples, and every accepted trigger lands on a sample not yet output, at any chunk size. A detection whose sample
 * has already been published is dropped and counted in `nLate()`. Placing it later would misalign the frame.
 *
 * The output carries each input sample `delaySamples()` samples later. The last `delaySamples()` inputs of a stream
 * never reach the output, and a tag on one of them leaves at the end-of-stream index. The block drops the trigger keys
 * of an input tag whose `trigger_name` equals `trigger_label`. Its own detections carry that label. It forwards the
 * other keys of that tag.
 *
 * The tag's `trigger_meta_info` carries `cfo_fractional`, in subcarrier spacings.
 *
 *     eps_frac = angle(P(d_opt)) / pi
 *
 * `P` correlates samples `L = fft_len/2` apart. A carrier offset of `eps` spacings turns them by
 * `2*pi*eps*L/fft_len = pi*eps`. The estimate covers `|eps| < 1`. A larger offset aliases into that range.
 *
 * `correct_cfo` derotates the output from the trigger onward. At each trigger the phasor takes the new increment and
 * keeps its phase. The correction is therefore phase-continuous across a re-estimate. A constant phase remains after
 * the correction.
 *
 * `r_floor` is the received energy a trigger needs. `M` is a ratio and reads high where both sums are near zero.
 * `min_gap` symbols of dead time after a trigger keep a frame from being detected twice. A NaN or infinite sample adds
 * nothing to either sum. The windows that hold it lose its terms, and later windows are exact.
 *
 * Settings may change while the block runs. A change of `fft_len` or `cp_len` sets a new lag and restarts both sums.
 * The delay line keeps its newest inputs. A longer lag puts zeros ahead of them, and a shorter lag drops the oldest.
 * Each held tag stays on its sample. A held input tag whose sample is dropped leaves on the next output, and a held
 * detection whose sample is dropped counts in `nLate()`. A change of `threshold`, `min_gap` or `r_floor` keeps the
 * delay line, the sums and the held tags. The counters run from `start()` across every change.
 *
 * A refused setting throws and changes nothing the block computes. The members keep the values of the refused
 * application, and a later change the block accepts can take them. Any change takes `correct_cfo` and
 * `trigger_label`. A change of `fft_len`, `cp_len`, `threshold`, `min_gap` or `r_floor` checks those five again and
 * takes them when they pass.
 */
struct SchmidlCoxSync : Block<SchmidlCoxSync> {
    using Description = Doc<"Estimates Schmidl-Cox coarse timing and fractional carrier frequency offset. The block passes the complex stream through one to one, delayed by 3*fft_len/2 + cp_len - 1 samples. The output ends that many samples before the input does, and a tag on one of the last delaySamples() inputs leaves at the end-of-stream index. It tags each frame's first sample with trigger_name and with cfo_fractional in subcarrier spacings, and it can derotate the stream by that estimate. It drops the trigger keys of an input tag whose trigger_name equals its trigger_label. Both sliding sums cost O(1) per sample. The timing instant is the midpoint of the threshold crossings">;

    PortIn<Complex>  in;
    PortOut<Complex> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length, a power of two">>                                               fft_len       = 64U;
    Annotated<gr::Size_t, "cp_len", Visible, Doc<"prefix length of the preamble symbol">>                                            cp_len        = 16U;
    Annotated<float, "threshold", Visible, Doc<"metric level that opens a plateau, in (0, 1)">>                                      threshold     = 0.6f;
    Annotated<gr::Size_t, "min_gap", Visible, Unit<"symbols">, Doc<"dead time after a trigger, in cp_len + fft_len sample symbols">> min_gap       = 1U;
    Annotated<bool, "correct_cfo", Visible, Doc<"derotate the output by the estimate from the trigger onward">>                      correct_cfo   = true;
    Annotated<float, "r_floor", Doc<"least received energy a trigger needs">>                                                        r_floor       = 1e-9f;
    Annotated<std::string, "trigger_label", Doc<"the label written under the trigger_name key of the emitted tags">>                 trigger_label = std::string("ofdm_frame");

    GR_MAKE_REFLECTABLE(SchmidlCoxSync, in, out, fft_len, cp_len, threshold, min_gap, correct_cfo, r_floor, trigger_label);

    /// A tag held until the output reaches its sample. On the block's own detections, `increment` re-tunes the
    /// derotation at the trigger's sample.
    struct PendingTag {
        std::size_t           at{0UZ}; ///< absolute output index
        property_map          map{};
        std::optional<double> increment{};
        bool                  ownDetection{false};
    };

    gr::signal::Phasor<float> _phasor{};

    std::vector<Complex>    _history{}; ///< the last `delaySamples()` inputs, which form the delay line
    std::vector<Complex>    _window{};  ///< `_history` followed by this call's input
    std::vector<Complex>    _pRing{};   ///< P(d) for recent d; the falling edge reads the midpoint's value here
    std::vector<PendingTag> _pending{};

    /// The last `L` per-sample terms of each sum. `conj(r[n-L]) * r[n]` is the term P(d) gains at sample `n` and loses
    /// `L` samples later. `|r[n]|^2` has the same relation to R(d). Keeping the terms halves the multiplies of the
    /// sliding update. `L` is a power of two, and the index is a mask.
    std::vector<double> _termRe{};
    std::vector<double> _termIm{};
    std::vector<double> _termNorm{};

    double        _pRe      = 0.; ///< the running correlation, held as two scalars
    double        _pIm      = 0.;
    double        _r        = 0.;  ///< the running energy of the later half
    std::size_t   _half     = 0UZ; ///< L = fft_len/2
    std::size_t   _delay    = 0UZ; ///< 3*fft_len/2 + cp_len - 1
    std::size_t   _ring     = 0UZ; ///< retained P history, a power of two
    std::size_t   _mask     = 0UZ;
    std::size_t   _deadTime = 0UZ; ///< min_gap symbols, in samples
    std::int64_t  _midShift = 0;   ///< cp_len/2, from the plateau midpoint back to the frame's first sample
    bool          _above    = false;
    std::size_t   _riseAt   = 0UZ;
    std::size_t   _resumeAt = 0UZ; ///< absolute sample index where the dead time ends
    std::uint64_t _triggers = 0ULL;
    std::uint64_t _late     = 0ULL;
    std::uint64_t _short    = 0ULL; ///< crossings too narrow to be a preamble's plateau
    std::uint64_t _wide     = 0ULL; ///< crossings wider than any preamble's plateau
    std::size_t   _minWidth = 1UZ;
    std::size_t   _maxWidth = 1UZ; ///< cp_len + fft_len - 1
    double        _lastCfo  = 0.;
    double        _level    = 0.;    ///< `threshold` as last accepted
    double        _floor    = 0.;    ///< `r_floor` as last accepted
    bool          _correct  = false; ///< `correct_cfo` as last accepted
    std::string   _label{};          ///< `trigger_label` as last accepted

    /// @brief The number of samples the output lags the input.
    [[nodiscard]] std::size_t delaySamples() const noexcept { return _delay; }
    /// @brief Frames detected over the run.
    [[nodiscard]] std::uint64_t nTriggers() const noexcept { return _triggers; }
    /// @brief Detections not placed: the frame began before the stream, or its sample left the delay line first.
    [[nodiscard]] std::uint64_t nLate() const noexcept { return _late; }
    /// @brief Threshold crossings too narrow to be a preamble's plateau.
    [[nodiscard]] std::uint64_t nShortPlateaus() const noexcept { return _short; }
    /// @brief Threshold crossings wider than any preamble's plateau.
    [[nodiscard]] std::uint64_t nWidePlateaus() const noexcept { return _wide; }
    /// @brief The fractional offset of the most recent trigger, in subcarrier spacings.
    [[nodiscard]] double lastCfoFractional() const noexcept { return _lastCfo; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kKeys{"fft_len", "cp_len", "threshold", "min_gap", "r_floor"};
        const bool                  estimator = _history.empty() || std::ranges::any_of(kKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); });
        if (estimator) {
            validate();
        }
        accept(estimator);
    }

    /// @brief Resets the estimator. `settingsChanged()` builds it when a setting takes a value other than its default.
    /// For a block left at its defaults, `start()` builds it.
    void start() {
        if (_history.empty()) {
            accept(true);
        }
        reset();
    }

    void reset() {
        std::ranges::fill(_history, Complex{});
        std::ranges::fill(_termRe, 0.);
        std::ranges::fill(_termIm, 0.);
        std::ranges::fill(_termNorm, 0.);
        _pRe      = 0.;
        _pIm      = 0.;
        _r        = 0.;
        _above    = false;
        _riseAt   = 0UZ;
        _resumeAt = 0UZ;
        _triggers = 0ULL;
        _late     = 0ULL;
        _short    = 0ULL;
        _wide     = 0ULL;
        _lastCfo  = 0.;
        _pending.clear();
        _phasor.configure(0., 0.);
    }

    /// @brief Holds each arriving input tag for the output index of its sample, `delaySamples()` later.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        if (processedIn == 0UZ) {
            releaseAtEndOfStream(inputSpans, outputSpans);
            return;
        }
        gr::for_each_reader_span(
            [this, processedIn](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // a tag from before this window is one this block has already placed
                        continue;
                    }
                    property_map map = tagMap.get();
                    dropOwnTrigger(map);
                    if (!map.empty()) {
                        _pending.emplace_back(span.streamIndex + static_cast<std::size_t>(relIndex) + _delay, std::move(map), std::nullopt, false);
                    }
                }
            },
            inputSpans);
    }

    /**
     * @brief Publishes every held input tag, and every tag left on the input, at the end-of-stream index.
     *
     * The framework's end-of-stream call passes `processedIn` 0 and empty output spans, where offset 0 is the
     * end-of-stream index. The tags leave in the order of their input samples. The block drops the `end_of_stream` key,
     * which the framework publishes itself. It drops a held detection, whose frame start is never output.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void releaseAtEndOfStream(TInputSpans& inputSpans, TOutputSpans& outputSpans) {
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        gr::for_each_reader_span(
            [this, &endOfStream](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const gr::Tag& tag : span.rawTags) {
                    if (tag.index < span.streamIndex) { // placed by an earlier call
                        continue;
                    }
                    property_map map = tag.map;
                    map.erase(endOfStream);
                    dropOwnTrigger(map);
                    if (!map.empty()) {
                        _pending.emplace_back(tag.index + _delay, std::move(map), std::nullopt, false);
                    }
                }
            },
            inputSpans);
        std::ranges::stable_sort(_pending, std::ranges::less{}, &PendingTag::at);
        for (const PendingTag& tag : _pending) {
            if (!tag.ownDetection) {
                gr::for_each_writer_span([&tag](auto& span) { span.publishTag(tag.map, 0UZ); }, outputSpans);
            }
        }
        _pending.clear();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        if (nSamples == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const std::size_t base = inSpan.streamIndex;

        _window.assign(_history.begin(), _history.end());
        _window.insert(_window.end(), inSpan.begin(), std::next(inSpan.begin(), static_cast<std::ptrdiff_t>(nSamples)));

        detect(base, nSamples);
        if (const auto emitted = emit(base, nSamples, outSpan); !emitted) {
            this->emitErrorMessage("processBulk", emitted.error());
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        _history.assign(_window.end() - static_cast<std::ptrdiff_t>(_delay), _window.end());
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

private:
    /// @brief Removes the trigger keys of an input tag whose `trigger_name` equals `trigger_label`.
    void dropOwnTrigger(property_map& map) const {
        const auto  entry = map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
        const auto* label = entry == map.end() ? nullptr : entry->second.get_if<std::pmr::string>();
        if (label == nullptr || std::string_view(label->data(), label->size()) != std::string_view(_label)) {
            return;
        }
        for (const char* key : {gr::tag::TRIGGER_NAME.shortKey(), gr::tag::TRIGGER_TIME.shortKey(), gr::tag::TRIGGER_OFFSET.shortKey(), gr::tag::TRIGGER_META_INFO.shortKey()}) {
            map.erase(std::pmr::string(key));
        }
    }

    /// @brief Throws `gr::exception` on a setting the estimator cannot work under.
    void validate() const {
        detail::requireFftLength(fft_len);
        if (cp_len.value > fft_len.value) {
            throw gr::exception(std::format("cp_len is {} and fft_len is {}: a prefix is a copy of the symbol's own tail and cannot be longer than the symbol", cp_len.value, fft_len.value));
        }
        if (!(threshold.value > 0.f) || !(threshold.value < 1.f)) {
            throw gr::exception(std::format("threshold is M at the plateau's edge and must lie in (0, 1), got {}", threshold.value));
        }
        if (!(r_floor.value > 0.f)) {
            throw gr::exception(std::format("r_floor must be strictly positive, got {}", r_floor.value));
        }
    }

    /// @brief Takes the settings the block computes on. With @p estimator set, it derives the estimator's values first
    /// and resizes the delay line when the lag changes.
    void accept(bool estimator) {
        if (estimator) {
            const std::size_t previousDelay = _delay;
            derive();
            if (_history.size() != _delay || _termRe.size() != _half) {
                resize(previousDelay);
            }
        }
        _correct = correct_cfo.value;
        _label   = trigger_label.value;
    }

    /// @brief Derives the lag, the dead time, the ring, the width bounds and the levels from the settings.
    void derive() noexcept {
        const auto        fft      = static_cast<std::size_t>(fft_len.value);
        const auto        cp       = static_cast<std::size_t>(cp_len.value);
        const std::size_t maxWidth = cp + fft - 1UZ;
        _half                      = fft / 2UZ;
        _delay                     = 3UZ * _half + cp - 1UZ;
        _deadTime                  = std::max(std::size_t{1}, static_cast<std::size_t>(min_gap.value) * (cp + fft));
        _midShift                  = static_cast<std::int64_t>(cp / 2UZ);
        _level                     = static_cast<double>(threshold.value);
        _floor                     = static_cast<double>(r_floor.value);
        // The falling edge reads P at the crossing's midpoint, at most half the widest accepted crossing back. The ring
        // holds that many values. A power-of-two size turns the per-sample index into a mask.
        _ring = std::bit_ceil(maxWidth / 2UZ + 1UZ);
        _mask = _ring - 1UZ;
        // A repeated-half preamble holds M above any threshold below 1 across its whole prefix. A much narrower
        // crossing comes from the ratio at the end of a burst, where one sample divided by itself reads as a perfect
        // correlation. A crossing shorter than half the prefix is refused and counted in nShortPlateaus().
        _minWidth = cp / 2UZ + 1UZ;
        _maxWidth = maxWidth;
    }

    /**
     * @brief Sizes the delay line and the rings for the derived lag and restarts both sums.
     *
     * The delay line keeps its newest inputs, and each held tag moves with its sample. A tag whose sample is dropped
     * keeps an index below the next output, and `emit()` publishes it on that output. The counters, the dead time and
     * the derotation continue.
     */
    void resize(std::size_t previousDelay) {
        std::vector<Complex> history(_delay, Complex{});
        const std::size_t    kept = std::min(previousDelay, _delay);
        std::copy(_history.end() - static_cast<std::ptrdiff_t>(kept), _history.end(), history.end() - static_cast<std::ptrdiff_t>(kept));
        _history = std::move(history);
        for (PendingTag& tag : _pending) {
            tag.at = tag.at - previousDelay + _delay; // `at` is the sample's index plus the lag, never below the lag
        }
        _pRing.assign(_ring, Complex{});
        _termRe.assign(_half, 0.);
        _termIm.assign(_half, 0.);
        _termNorm.assign(_half, 0.);
        _pRe    = 0.;
        _pIm    = 0.;
        _r      = 0.;
        _above  = false;
        _riseAt = 0UZ;
    }

    /// @brief Slides `P` and `R` over the call's samples and turns each plateau into one trigger.
    void detect(std::size_t base, std::size_t nSamples) {
        const std::size_t half    = _half;
        const std::size_t offset  = _delay; // `_window[offset + j]` is input `base + j`
        const double      floorSq = _floor;
        const double      level   = _level;
        const Complex*    window  = _window.data();

        double            pRe      = _pRe;
        double            pIm      = _pIm;
        double            r        = _r;
        const std::size_t termMask = half - 1UZ;
        double*           termRe   = _termRe.data();
        double*           termIm   = _termIm.data();
        double*           termNorm = _termNorm.data();

        for (std::size_t j = 0UZ; j < nSamples; ++j) {
            // The arriving sample completes the window that starts at `d`. The zeroed history and term rings hold
            // silence before the stream. The arithmetic is written out in scalars: a `std::complex` multiply calls a
            // library helper that handles infinities.
            const double newRe = static_cast<double>(window[offset + j].real());
            const double newIm = static_cast<double>(window[offset + j].imag());
            const double midRe = static_cast<double>(window[offset + j - half].real());
            const double midIm = static_cast<double>(window[offset + j - half].imag());

            double gainRe   = midRe * newRe + midIm * newIm;
            double gainIm   = midRe * newIm - midIm * newRe;
            double gainNorm = newRe * newRe + newIm * newIm;
            if (!std::isfinite(gainRe + gainIm + gainNorm)) {
                gainRe   = 0.;
                gainIm   = 0.;
                gainNorm = 0.;
            }

            const std::size_t slot = (base + j) & termMask;
            pRe += gainRe - termRe[slot];
            pIm += gainIm - termIm[slot];
            r += gainNorm - termNorm[slot];
            termRe[slot]   = gainRe;
            termIm[slot]   = gainIm;
            termNorm[slot] = gainNorm;

            const std::int64_t d = static_cast<std::int64_t>(base + j) - static_cast<std::int64_t>(2UZ * half) + 1;
            if (d < 0) {
                continue;
            }
            const auto at    = static_cast<std::size_t>(d);
            const bool above = r > floorSq && (pRe * pRe + pIm * pIm) > level * r * r;
            if (above) {
                if (!_above) {
                    _above  = true;
                    _riseAt = at;
                }
                // P is stored only inside a plateau, the one place it is read back. Off a plateau the loop makes no
                // store that could alias its loads.
                _pRing[at & _mask] = Complex(static_cast<float>(pRe), static_cast<float>(pIm));
                continue;
            }
            if (!_above) {
                continue;
            }
            _pRe   = pRe;
            _pIm   = pIm;
            _r     = r;
            _above = false;
            close(at, _riseAt);
        }

        _pRe = pRe;
        _pIm = pIm;
        _r   = r;
    }

    /// @brief Places the trigger of the ended plateau `[riseAt, fallAt)` at the frame's first sample.
    void close(std::size_t fallAt, std::size_t riseAt) {
        const std::size_t width = fallAt - riseAt;
        if (width < _minWidth) {
            ++_short;
            return;
        }
        if (width > _maxWidth) {
            ++_wide;
            return;
        }
        const std::size_t middle = riseAt + (width - 1UZ) / 2UZ;
        const Complex     peak   = _pRing[middle & _mask];

        const std::int64_t start = static_cast<std::int64_t>(middle) - _midShift;
        if (start < 0) {
            ++_late; // the frame began before the stream
            return;
        }
        const auto frameAt = static_cast<std::size_t>(start);
        if (_triggers > 0ULL && frameAt < _resumeAt) {
            return; // inside the dead time a frame's own structure can reopen the plateau
        }

        const double eps = std::atan2(static_cast<double>(peak.imag()), static_cast<double>(peak.real())) / std::numbers::pi;
        _lastCfo         = eps;
        ++_triggers;
        _resumeAt = frameAt + _deadTime;

        _pending.emplace_back(frameAt + _delay,
            property_map{{gr::tag::TRIGGER_NAME.shortKey(), _label}, //
                {gr::tag::TRIGGER_OFFSET.shortKey(), 0.f},           //
                {gr::tag::TRIGGER_META_INFO.shortKey(), property_map{{"cfo_fractional", static_cast<float>(eps)}, {"plateau_samples", static_cast<gr::Size_t>(width)}}}},
            _correct ? std::optional<double>(-std::numbers::pi * eps / static_cast<double>(_half)) : std::nullopt, true);
    }

    /**
     * @brief Writes the delayed stream, split at every tag this call reaches.
     *
     * The forwarded input tags and the block's own detections go out as one sequence sorted by sample. The derotation
     * takes its new increment at the sample its trigger marks. A correction that is not finite returns an error.
     */
    [[nodiscard]] std::expected<void, gr::Error> emit(std::size_t base, std::size_t nSamples, OutputSpanLike auto& outSpan) {
        std::ranges::stable_sort(_pending, std::ranges::less{}, &PendingTag::at);

        const Complex* source   = _window.data();
        std::size_t    produced = 0UZ;
        std::size_t    k        = 0UZ;

        while (produced < nSamples) {
            while (k < _pending.size() && _pending[k].at < base + produced) {
                if (_pending[k].ownDetection) {
                    ++_late; // its sample has left the delay line, and placing it here would misalign the frame
                } else {
                    outSpan.publishTag(_pending[k].map, produced);
                }
                ++k;
            }
            std::size_t next = nSamples;
            if (k < _pending.size() && _pending[k].at < base + nSamples) {
                next = _pending[k].at - base;
            }
            if (next > produced) {
                write(source + produced, next - produced, produced, outSpan);
                produced = next;
                continue;
            }
            outSpan.publishTag(_pending[k].map, produced);
            if (_pending[k].increment.has_value()) {
                const double increment = *_pending[k].increment;
                if (!std::isfinite(increment)) {
                    return std::unexpected(gr::Error(std::format("gr::blocks::ofdm::SchmidlCoxSync: the carrier-offset correction {} rad per sample is not finite", increment)));
                }
                _phasor.setIncrement(increment);
            }
            ++k;
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(k));
        return {};
    }

    void write(const Complex* source, std::size_t count, std::size_t at, OutputSpanLike auto& outSpan) {
        const std::span<Complex> destination(outSpan.begin() + static_cast<std::ptrdiff_t>(at), count);
        if (_correct) {
            _phasor.mix(std::span<const Complex>(source, count), destination);
        } else {
            std::copy_n(source, count, destination.begin());
        }
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP
