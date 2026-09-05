#ifndef GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP
#define GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <span>
#include <string>
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
 * @brief Coarse frame timing and fractional carrier frequency offset from a repeated-half preamble.
 *
 * With `L = fft_len/2`, the block runs two sums over the stream:
 *
 *     P(d) = sum_{m=0..L-1} conj(r[d+m]) * r[d+m+L]      R(d) = sum_{m=0..L-1} |r[d+m+L]|^2
 *
 * both as sliding updates — `P(d+1) = P(d) + conj(r[d+L])*r[d+2L] - conj(r[d])*r[d+L]` and the matching one for `R` —
 * so the cost is two complex multiplies and two magnitudes a sample and does not grow with `fft_len`. The stream
 * before its first sample counts as silence, which makes the recurrence exact from the first sample rather than
 * needing a warm-up the chunking could disturb.
 *
 * `M(d) = |P(d)|^2 / R(d)^2` is tested as `|P(d)|^2 > threshold * R(d)^2`, which is the same comparison without the
 * division. `M` does not peak at a preamble, it plateaus: the correlation is exact for every `d` from the preamble's
 * first prefix sample to `cp_len` samples later, so the plateau is `cp_len + 1` samples wide and its midpoint, not
 * its maximum, is the timing instant. The block takes the midpoint of the threshold crossings, which is the same
 * point and is what a noisy maximum is not.
 *
 * The trigger is written at the frame's first sample — the first sample of the preamble's own cyclic prefix, the
 * convention `CpRemove` cuts on — which is `cp_len/2` before the plateau midpoint. `out` therefore lags `in` by
 * `fft_len + 2*cp_len` samples, which is what leaves room to place a tag at a sample the falling crossing only
 * confirmed later; a detection whose sample has nonetheless already been published is dropped and counted in
 * `nLate()` rather than placed where it would misalign a frame.
 *
 * The tag's `trigger_meta_info` carries `cfo_fractional`, in subcarrier spacings:
 *
 *     eps_frac = angle(P(d_opt)) / pi
 *
 * since `P` correlates samples `L = fft_len/2` apart and a carrier offset of `eps` spacings turns them by
 * `2*pi*eps*L/fft_len = pi*eps`. The estimate is unambiguous for `|eps| <= 1`. Integer offsets — whole bins — are not
 * estimated here; the classical answer is a differentially encoded second preamble symbol, and it waits for a
 * consumer whose oscillators need more than the equalizer's own sync-word estimate can absorb.
 *
 * `correct_cfo` derotates the output from the trigger onward. The phasor's increment changes at each trigger and its
 * phase runs on, so the correction is phase-continuous across a re-estimate; the constant phase this leaves is one
 * the equalizer's channel estimate absorbs.
 *
 * `r_floor` is the received-energy floor below which no trigger is emitted: `M` is a ratio, and silence divided by
 * silence is not a detection. `min_gap` symbols of dead time after a trigger keep a frame from being detected twice.
 */
struct SchmidlCoxSync : Block<SchmidlCoxSync> {
    using Description = Doc<"Schmidl-Cox coarse timing and fractional CFO: a complex passthrough, 1:1, lagging its input by fft_len + 2*cp_len samples, emitting a trigger_name tag at each frame's first sample with cfo_fractional in subcarrier spacings and optionally derotating the stream by it. Both sliding sums are O(1) per sample and the timing instant is the plateau midpoint, not its maximum">;

    PortIn<Complex>  in;
    PortOut<Complex> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length; the repeated halves are fft_len/2 samples each">>                      fft_len       = 64U;
    Annotated<gr::Size_t, "cp_len", Visible, Doc<"the preamble symbol's own prefix, which sets the plateau width and the midpoint offset">> cp_len        = 16U;
    Annotated<float, "threshold", Visible, Doc<"M above which the plateau is held to have started, in (0, 1)">>                             threshold     = 0.6f;
    Annotated<gr::Size_t, "min_gap", Visible, Unit<"symbols">, Doc<"dead time after a trigger, in cp_len + fft_len sample symbols">>        min_gap       = 1U;
    Annotated<bool, "correct_cfo", Visible, Doc<"derotate the output by the estimate from the trigger onward">>                             correct_cfo   = true;
    Annotated<float, "r_floor", Doc<"received energy below which no trigger is emitted; silence over silence is not a detection">>          r_floor       = 1e-9f;
    Annotated<std::string, "trigger_label", Doc<"the label written under the trigger_name key of the emitted tags">>                        trigger_label = std::string("ofdm_frame");

    GR_MAKE_REFLECTABLE(SchmidlCoxSync, in, out, fft_len, cp_len, threshold, min_gap, correct_cfo, r_floor, trigger_label);

    /// A tag waiting for the output sample it belongs to. `increment` is set on the block's own detections and is
    /// what re-tunes the derotation exactly where the trigger lands.
    struct PendingTag {
        std::uint64_t         at{0ULL}; ///< absolute output index
        property_map          map{};
        std::optional<double> increment{};
        bool                  ownDetection{false};
    };

    gr::signal::Phasor<float> _phasor{};

    std::vector<Complex>    _history{}; ///< the last `delay()` inputs, which are also the delay line
    std::vector<Complex>    _window{};  ///< `_history` followed by this call's input
    std::vector<Complex>    _pRing{};   ///< P(d) for the recent d, so the falling edge can read the midpoint's
    std::vector<PendingTag> _pending{};

    /// The last `L` per-sample terms of each sum, so neither is computed twice. `conj(r[n-L]) * r[n]` is the term
    /// P(d) gains at sample `n` and the one it loses `L` samples later, and `|r[n]|^2` stands in the same relation to
    /// R(d): keeping them halves the multiplies the sliding update would otherwise do. `L` is a power of two, so the
    /// index is a mask.
    std::vector<double> _termRe{};
    std::vector<double> _termIm{};
    std::vector<double> _termNorm{};

    double        _pRe      = 0.; ///< the running correlation, held as two scalars
    double        _pIm      = 0.;
    double        _r        = 0.;  ///< the running energy of the later half
    std::size_t   _half     = 0UZ; ///< L = fft_len/2
    std::size_t   _delay    = 0UZ; ///< fft_len + 2*cp_len
    std::size_t   _ring     = 0UZ; ///< retained P history, a power of two so the index is a mask
    std::size_t   _mask     = 0UZ;
    std::size_t   _deadTime = 0UZ; ///< min_gap symbols, in samples
    bool          _above    = false;
    std::uint64_t _riseAt   = 0ULL;
    std::uint64_t _resumeAt = 0ULL; ///< absolute frame index the dead time ends at
    std::uint64_t _triggers = 0ULL;
    std::uint64_t _late     = 0ULL;
    std::uint64_t _clipped  = 0ULL; ///< plateaus longer than the ring, whose midpoint P was overwritten
    std::uint64_t _short    = 0ULL; ///< crossings too narrow to be a preamble's plateau
    std::uint64_t _minWidth = 1ULL;
    double        _lastCfo  = 0.;

    /// @brief Samples the output lags the input by, which is where a trigger's room to be placed comes from.
    [[nodiscard]] std::size_t delaySamples() const noexcept { return _delay; }
    /// @brief Triggers emitted over the run.
    [[nodiscard]] std::uint64_t nTriggers() const noexcept { return _triggers; }
    /// @brief Detections whose output sample had already been published, and which were therefore not placed.
    [[nodiscard]] std::uint64_t nLate() const noexcept { return _late; }
    /// @brief Plateaus wider than the retained history of `P`, whose midpoint estimate fell back to the oldest kept.
    [[nodiscard]] std::uint64_t nClipped() const noexcept { return _clipped; }
    /// @brief Threshold crossings narrower than a preamble's plateau can be, and so not taken for one.
    [[nodiscard]] std::uint64_t nShortPlateaus() const noexcept { return _short; }
    /// @brief The fractional offset of the most recent trigger, in subcarrier spacings.
    [[nodiscard]] double lastCfoFractional() const noexcept { return _lastCfo; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_len", "cp_len", "threshold", "min_gap", "r_floor"};
        if (!_history.empty() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        rebuild();
    }

    void start() { rebuild(); }

    void rebuild() {
        detail::requireFftLength(fft_len);
        if (cp_len.value > fft_len.value) {
            throw gr::exception(std::format("cp_len is {} and fft_len is {}: a prefix is a copy of the symbol's own tail and cannot be longer than the symbol", cp_len.value, fft_len.value));
        }
        if (!(threshold.value > 0.f) || !(threshold.value < 1.f)) {
            throw gr::exception(std::format("threshold is M at the plateau's edge and must lie in (0, 1), got {}", threshold.value));
        }
        if (!(r_floor.value > 0.f)) {
            throw gr::exception(std::format("r_floor guards the division and must be strictly positive, got {}", r_floor.value));
        }

        _half     = static_cast<std::size_t>(fft_len.value) / 2UZ;
        _delay    = static_cast<std::size_t>(fft_len.value) + 2UZ * static_cast<std::size_t>(cp_len.value);
        _deadTime = std::max(std::size_t{1}, static_cast<std::size_t>(min_gap.value) * (static_cast<std::size_t>(cp_len.value) + static_cast<std::size_t>(fft_len.value)));
        // Wide enough for any plateau the threshold can hold open, so the midpoint's P is still there when the
        // falling edge asks for it, and rounded up to a power of two: the ring is indexed once a sample by a
        // stream-absolute count, and a mask is what keeps that from being an integer division on the hot path.
        _ring = std::bit_ceil(_delay + 2UZ);
        _mask = _ring - 1UZ;
        // A repeated-half preamble holds M above any threshold under 1 for the whole width of its prefix, so a
        // crossing much narrower than that is the metric's own ratio running away where the received energy is
        // collapsing -- the last samples of a burst, where one signal sample divided by itself reads as a perfect
        // correlation. Half the prefix is the widest such crossing worth refusing and the narrowest real plateau
        // worth keeping; what it refuses is counted in nShortPlateaus().
        _minWidth = static_cast<std::uint64_t>(cp_len.value) / 2ULL + 1ULL;

        _history.assign(_delay, Complex{});
        _pRing.assign(_ring, Complex{});
        _termRe.assign(_half, 0.);
        _termIm.assign(_half, 0.);
        _termNorm.assign(_half, 0.);
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
        _riseAt   = 0ULL;
        _resumeAt = 0ULL;
        _triggers = 0ULL;
        _late     = 0ULL;
        _clipped  = 0ULL;
        _short    = 0ULL;
        _lastCfo  = 0.;
        _pending.clear();
        _phasor.configure(0., 0.);
    }

    /// @brief Take in the arriving input tags at the output index their sample reaches, which is `delaySamples()` later.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& /*outputSpans*/, std::size_t processedIn) {
        const auto delay = static_cast<std::uint64_t>(_delay);
        gr::for_each_reader_span(
            [this, processedIn, delay](auto& span) {
                if (!span.isSync || !span.isConnected) {
                    return;
                }
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    if (relIndex < 0) { // a tag from before this window is one this block has already placed
                        continue;
                    }
                    _pending.emplace_back(static_cast<std::uint64_t>(span.streamIndex) + static_cast<std::uint64_t>(relIndex) + delay, tagMap.get(), std::nullopt, false);
                }
            },
            inputSpans);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t nSamples = std::min(inSpan.size(), outSpan.size());
        if (nSamples == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        const auto base = static_cast<std::uint64_t>(inSpan.streamIndex);

        _window.assign(_history.begin(), _history.end());
        _window.insert(_window.end(), inSpan.begin(), std::next(inSpan.begin(), static_cast<std::ptrdiff_t>(nSamples)));

        detect(base, nSamples);
        emit(base, nSamples, outSpan);

        _history.assign(_window.end() - static_cast<std::ptrdiff_t>(_delay), _window.end());
        std::ignore = inSpan.consume(nSamples);
        outSpan.publish(nSamples);
        return work::Status::OK;
    }

private:
    /// @brief Slide `P` and `R` over the call's samples and turn each plateau into one trigger.
    void detect(std::uint64_t base, std::size_t nSamples) {
        const std::size_t  half     = _half;
        const std::size_t  offset   = _delay; // `_window[offset + j]` is input `base + j`
        const double       floorSq  = static_cast<double>(r_floor.value);
        const double       level    = static_cast<double>(threshold.value);
        const Complex*     window   = _window.data();
        const std::int64_t midShift = static_cast<std::int64_t>(cp_len.value) / 2;

        double            pRe      = _pRe;
        double            pIm      = _pIm;
        double            r        = _r;
        const std::size_t termMask = half - 1UZ;
        double*           termRe   = _termRe.data();
        double*           termIm   = _termIm.data();
        double*           termNorm = _termNorm.data();

        for (std::size_t j = 0UZ; j < nSamples; ++j) {
            // The sample arriving completes the window that starts at `d`; everything before the stream is silence,
            // which is what the zero-initialized history and term rings make true. The complex arithmetic is written
            // out in scalars rather than in std::complex: a complex multiply in the language calls the library's
            // infinity-preserving helper, which is a function call in the middle of what has to be a handful of
            // arithmetic instructions.
            const double newRe = static_cast<double>(window[offset + j].real());
            const double newIm = static_cast<double>(window[offset + j].imag());
            const double midRe = static_cast<double>(window[offset + j - half].real());
            const double midIm = static_cast<double>(window[offset + j - half].imag());

            // conj(r[n-L]) * r[n] and |r[n]|^2: the terms P and R gain now and lose L samples from now, so each is
            // computed once and read back out of the ring rather than formed a second time from the older samples.
            const double gainRe   = midRe * newRe + midIm * newIm;
            const double gainIm   = midRe * newIm - midIm * newRe;
            const double gainNorm = newRe * newRe + newIm * newIm;

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
            const auto at    = static_cast<std::uint64_t>(d);
            const bool above = r > floorSq && (pRe * pRe + pIm * pIm) > level * r * r;
            if (above) {
                if (!_above) {
                    _above  = true;
                    _riseAt = at;
                }
                // P is only ever read back from inside a plateau, so it is only kept there. Off the plateau -- which
                // is almost every sample of a stream -- the loop makes no store at all, and the loads it does make
                // cannot be aliased by one.
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
            close(at, _riseAt, midShift);
        }

        _pRe = pRe;
        _pIm = pIm;
        _r   = r;
    }

    /// @brief The plateau `[riseAt, fallAt)` has ended: place its trigger at the frame's first sample.
    void close(std::uint64_t fallAt, std::uint64_t riseAt, std::int64_t midShift) {
        if (fallAt - riseAt < _minWidth) {
            ++_short;
            return;
        }
        std::uint64_t middle = riseAt + (fallAt - 1ULL - riseAt) / 2ULL;
        if (fallAt - middle >= static_cast<std::uint64_t>(_ring)) {
            ++_clipped;
            middle = fallAt - static_cast<std::uint64_t>(_ring) + 1ULL;
        }
        const Complex peak = _pRing[middle & _mask];

        const std::int64_t start = static_cast<std::int64_t>(middle) - midShift;
        if (start < 0) {
            ++_late; // the frame began before the stream did, so there is no sample to mark
            return;
        }
        const auto frameAt = static_cast<std::uint64_t>(start);
        if (_triggers > 0ULL && frameAt < _resumeAt) {
            return; // inside the dead time a frame's own structure can reopen the plateau
        }

        const double eps = std::atan2(static_cast<double>(peak.imag()), static_cast<double>(peak.real())) / std::numbers::pi;
        _lastCfo         = eps;
        ++_triggers;
        _resumeAt = frameAt + static_cast<std::uint64_t>(_deadTime);

        _pending.emplace_back(frameAt + static_cast<std::uint64_t>(_delay),
            property_map{{gr::tag::TRIGGER_NAME.shortKey(), trigger_label.value}, //
                {gr::tag::TRIGGER_OFFSET.shortKey(), 0.f},                        //
                {gr::tag::TRIGGER_META_INFO.shortKey(), property_map{{"cfo_fractional", static_cast<float>(eps)}, {"plateau_samples", static_cast<gr::Size_t>(fallAt - riseAt)}}}},
            correct_cfo.value ? std::optional<double>(-2. * std::numbers::pi * eps / static_cast<double>(fft_len.value)) : std::nullopt, true);
    }

    /**
     * @brief Write the delayed stream out, splitting it at every tag this call reaches.
     *
     * The forwarded input tags and the block's own detections are one ordered sequence on the port, so they are
     * merged here rather than published as each arises, and the derotation is re-tuned at exactly the sample its
     * trigger marks.
     */
    void emit(std::uint64_t base, std::size_t nSamples, OutputSpanLike auto& outSpan) {
        std::ranges::stable_sort(_pending, std::ranges::less{}, &PendingTag::at);

        const Complex* source   = _window.data();
        std::size_t    produced = 0UZ;
        std::size_t    k        = 0UZ;

        while (produced < nSamples) {
            while (k < _pending.size() && _pending[k].at < base + static_cast<std::uint64_t>(produced)) {
                if (_pending[k].ownDetection) {
                    ++_late; // its sample is already downstream; placing it here would misalign the frame
                } else {
                    outSpan.publishTag(_pending[k].map, produced);
                }
                ++k;
            }
            std::size_t next = nSamples;
            if (k < _pending.size() && _pending[k].at < base + static_cast<std::uint64_t>(nSamples)) {
                next = _pending[k].at - base;
            }
            if (next > produced) {
                write(source + produced, next - produced, produced, outSpan);
                produced = next;
                continue;
            }
            outSpan.publishTag(_pending[k].map, produced);
            if (_pending[k].increment.has_value()) {
                _phasor.setIncrement(*_pending[k].increment);
            }
            ++k;
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(k));
    }

    void write(const Complex* source, std::size_t count, std::size_t at, OutputSpanLike auto& outSpan) {
        const std::span<Complex> destination(outSpan.begin() + static_cast<std::ptrdiff_t>(at), count);
        if (correct_cfo.value) {
            _phasor.mix(std::span<const Complex>(source, count), destination);
        } else {
            std::copy_n(source, count, destination.begin());
        }
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_SCHMIDL_COX_SYNC_HPP
