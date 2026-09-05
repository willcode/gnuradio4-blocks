#ifndef GNURADIO_MEASUREMENT_POLARIZATION_COMBINER_HPP
#define GNURADIO_MEASUREMENT_POLARIZATION_COMBINER_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/measurement/PolarizationCombiner.hpp>

namespace gr::blocks::measurement {

namespace detail {
inline constexpr gr::Size_t kMinPolarizationWindow = 64U;
inline constexpr gr::Size_t kMaxPolarizationWindow = 1U << 24U;
} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::PolarizationCombiner)

/**
 * @brief Two orthogonal antenna branches combined at the maximal-ratio optimum, from the 2x2 covariance in closed form.
 *
 * Over window `w` the block accumulates the branch covariance in `double` while emitting output computed with the
 * weights `BranchCovariance::solve` derived from window `w-1` — causal, with no lookahead. That makes the weights one
 * window old, which is the block's stated adaptation latency. Before the first window closes, `out` carries branch 0
 * unchanged and `ortho` carries branch 1 unchanged (`polarizationPassthrough(0)`): a valid signal, with no phase
 * discontinuity relative to what follows once branch 0 is the gauge reference, and no invented relative phase.
 *
 * `emit_orthogonal` is the switch `polarizationCombine`'s own optional span is for: when clear, or when nothing is
 * connected to `ortho`, the orthogonal arithmetic is skipped rather than computed and discarded, and the port
 * publishes nothing.
 *
 * A degenerate window — no noise in either branch, or nothing correlated between them — makes an SNR figure or the
 * amplitude ratio unbounded; both are reported saturated at `1e6` linear (`60 dB`) rather than as an infinity, and
 * every saturation is counted in `nSaturatedFigures()` so the record's finite number is never mistaken for a
 * measurement. At the other end, a branch whose estimated signal power is exactly zero has a signal-to-noise ratio
 * of zero and so a decibel figure of `-inf`; those channels read `0` dB and the record's `valid` is written false,
 * the same convention a degenerate window uses, so nothing non-finite reaches a record or a reader.
 */
struct PolarizationCombiner : Block<PolarizationCombiner> {
    using Description = Doc<R""(
@brief Two orthogonal antenna branches into one at the maximal-ratio optimum, with the estimator's own figures readable.

Accumulates the branch covariance over a stream-absolute window and combines with the weights the *previous* window's
covariance implies (causal, one window of adaptation latency). `mode = mrc` maximizes the combined signal-to-noise
ratio; `selection` picks the stronger branch from the same covariance. Before the first window, `out` passes branch 0
through unchanged and `ortho` carries branch 1 unchanged.
)"">;

    PortIn<std::complex<float>>              in0;
    PortIn<std::complex<float>>              in1;
    PortOut<std::complex<float>>             out;
    PortOut<std::complex<float>, Optional>   ortho;
    PortOut<DataSet<float>, Async, Optional> measurements;

    Annotated<std::string, "mode", Visible, Doc<"'mrc' or 'selection'">>                                                          mode             = std::string("mrc");
    Annotated<gr::Size_t, "window", Visible, Doc<"samples per estimate; below 64 or above 2^24 is refused">>                      window           = 4096U;
    Annotated<float, "sample_rate", Unit<"Hz">, Doc<"stream rate stated in every record; must be positive and finite">>           sample_rate      = 96000.f;
    Annotated<std::vector<double>, "noise_powers", Visible, Doc<"empty for equal branch noise, or exactly two positive entries">> noise_powers     = std::vector<double>{};
    Annotated<std::string, "normalize", Visible, Doc<"'unit_noise' (||v||=1) or 'unit_signal' (signal gain 1)">>                  normalize        = std::string("unit_noise");
    Annotated<double, "weight_smoothing", Visible, Doc<"one-pole blend of the weight vector across windows; must lie in [0, 1)">> weight_smoothing = 0.0;
    Annotated<bool, "emit_orthogonal", Visible, Doc<"compute and publish the orthogonal (interference-only) output">>             emit_orthogonal  = true;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per completed window">>                       emit_records     = true;

    GR_MAKE_REFLECTABLE(PolarizationCombiner, in0, in1, out, ortho, measurements, mode, window, sample_rate, noise_powers, normalize, weight_smoothing, emit_orthogonal, emit_records);

    gr::measurement::PolarizationMode          _mode{gr::measurement::PolarizationMode::mrc};
    gr::measurement::PolarizationNormalization _normalize{gr::measurement::PolarizationNormalization::unit_noise};
    gr::measurement::BranchCovariance          _covariance{};
    gr::measurement::PolarizationEstimate      _estimate{gr::measurement::polarizationPassthrough(0UZ)}; ///< weights in force
    bool                                       _haveEstimate{false};
    bool                                       _configured{false};

    std::size_t   _covCount{0UZ};
    std::uint64_t _streamAt{0ULL};
    std::uint64_t _windowStartAt{0ULL};
    bool          _flushed{false};

    int  _lastSelected{-1};
    bool _haveSelected{false};

    gr::measurement::MeasurementSlot<7UZ> _slot{}; ///< relativePhase(rad), amplitudeRatio, branch0Db, branch1Db, combinedDb, selectedBranch, combiningGainDb

    std::atomic<std::uint64_t> _nWindows{0ULL};
    std::atomic<std::uint64_t> _nWindowResets{0ULL};
    std::atomic<std::uint64_t> _nBranchSwitches{0ULL};
    std::atomic<std::uint64_t> _nSaturatedFigures{0ULL};
    std::atomic<std::uint64_t> _nDroppedSampleTags{0ULL};

    std::vector<DataSet<float>> _pending{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const auto parsedMode = gr::measurement::polarizationModeFrom(mode.value);
        if (!parsedMode.has_value()) {
            throw gr::exception(std::format("mode must be 'mrc' or 'selection', got '{}'", mode.value));
        }
        const auto parsedNorm = gr::measurement::polarizationNormalizationFrom(normalize.value);
        if (!parsedNorm.has_value()) {
            throw gr::exception(std::format("normalize must be 'unit_noise' or 'unit_signal', got '{}'", normalize.value));
        }
        if (window.value < detail::kMinPolarizationWindow || window.value > detail::kMaxPolarizationWindow) {
            throw gr::exception(std::format("window must lie in [{}, {}], got {}", detail::kMinPolarizationWindow, detail::kMaxPolarizationWindow, window.value));
        }
        if (!(weight_smoothing.value >= 0.) || !(weight_smoothing.value < 1.)) {
            throw gr::exception(std::format("weight_smoothing must lie in [0, 1), got {}", weight_smoothing.value));
        }
        if (!std::isfinite(sample_rate.value) || !(sample_rate.value > 0.f)) {
            throw gr::exception(std::format("sample_rate must be positive and finite, got {}", sample_rate.value));
        }
        if (!noise_powers.value.empty()) {
            if (noise_powers.value.size() != 2UZ) {
                throw gr::exception(std::format("noise_powers takes exactly two branches, got {}", noise_powers.value.size()));
            }
            for (const double power : noise_powers.value) {
                if (!std::isfinite(power) || !(power > 0.)) {
                    throw gr::exception(std::format("noise_powers entries must be finite and positive, got {}", power));
                }
            }
        }

        _mode      = *parsedMode;
        _normalize = *parsedNorm;

        static constexpr std::array kResetKeys{"window", "noise_powers", "mode", "normalize"};
        if (_configured && std::ranges::any_of(kResetKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            if (_covCount > 0UZ) {
                _nWindowResets.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _covariance.reset();
            _covCount      = 0UZ;
            _windowStartAt = _streamAt;
        }
        _configured = true;
    }

    void start() {
        _covariance.reset();
        _covCount      = 0UZ;
        _streamAt      = 0ULL;
        _windowStartAt = 0ULL;
        _flushed       = false;
        _estimate      = gr::measurement::polarizationPassthrough(0UZ);
        _haveEstimate  = false;
        _lastSelected  = -1;
        _haveSelected  = false;
        _nWindows.store(0ULL, std::memory_order_relaxed);
        _nWindowResets.store(0ULL, std::memory_order_relaxed);
        _nBranchSwitches.store(0ULL, std::memory_order_relaxed);
        _nSaturatedFigures.store(0ULL, std::memory_order_relaxed);
        _nDroppedSampleTags.store(0ULL, std::memory_order_relaxed);
        _pending.clear();
        _slot.publish({0., 0., 0., 0., 0., 0., 0.}, 0ULL);
        // A block built entirely from defaults stages nothing, so `settingsChanged` has not necessarily run by now.
        // From here on every settings change is a change to a running block, and the window reset rule applies to it.
        _configured = true;
        // The last sample of a call is held back so the end-of-stream epilogue always has a span to run on.
        in0.min_samples = 2UZ;
        in1.min_samples = 2UZ;
    }

    [[nodiscard]] double relativePhase() const noexcept { return _slot.read().first[0]; }
    [[nodiscard]] double amplitudeRatio() const noexcept { return _slot.read().first[1]; }
    [[nodiscard]] double branchSnrDb(std::size_t branch) const noexcept { return _slot.read().first[branch == 0UZ ? 2UZ : 3UZ]; }
    [[nodiscard]] double combinedSnrDb() const noexcept { return _slot.read().first[4]; }
    /// @brief What the combination bought over the better branch, the same finite figure the record's channel carries.
    [[nodiscard]] double combiningGainDb() const noexcept { return _slot.read().first[6]; }
    [[nodiscard]] int    selectedBranch() const noexcept { return static_cast<int>(_slot.read().first[5]); }
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_slot.read().second) / static_cast<double>(std::max(window.value, 1U))); }

    [[nodiscard]] std::uint64_t nWindows() const noexcept { return _nWindows.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nWindowResets() const noexcept { return _nWindowResets.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nBranchSwitches() const noexcept { return _nBranchSwitches.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nSaturatedFigures() const noexcept { return _nSaturatedFigures.load(std::memory_order_relaxed); }
    /// @brief `n_dropped_samples` tags seen on either branch. Callable from any thread.
    [[nodiscard]] std::uint64_t nDroppedSampleTags() const noexcept { return _nDroppedSampleTags.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& in0Span, InputSpanLike auto& in1Span, OutputSpanLike auto& outSpan, OutputSpanLike auto& orthoSpan, OutputSpanLike auto& measurementsSpan) {
        std::size_t made = drain(measurementsSpan, 0UZ);

        const bool        wantOrtho = emit_orthogonal.value && orthoSpan.isConnected;
        const std::size_t rawOffer  = std::min(in0Span.size(), in1Span.size());
        const std::size_t offer     = rawOffer >= 2UZ ? rawOffer - 1UZ : rawOffer;
        const std::size_t orthoRoom = wantOrtho ? orthoSpan.size() : std::numeric_limits<std::size_t>::max();
        const std::size_t take      = std::min({offer, outSpan.size(), orthoRoom, roomFor(measurementsSpan, made)});

        countDroppedTags(in0Span, in1Span, take);
        combine(std::span<const std::complex<float>>(in0Span).first(take), std::span<const std::complex<float>>(in1Span).first(take), std::span<std::complex<float>>(outSpan).first(take), std::span<std::complex<float>>(orthoSpan.data(), wantOrtho ? take : 0UZ), measurementsSpan.isConnected);
        made += drain(measurementsSpan, made);

        outSpan.publish(take);
        orthoSpan.publish(wantOrtho ? take : 0UZ);
        measurementsSpan.publish(made);
        std::ignore = in0Span.consume(take);
        std::ignore = in1Span.consume(take);
        return take == 0UZ && made == 0UZ && rawOffer > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief End of stream: fold the trailing samples, then flush the partial window with the count it actually covers.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& in0Span, InputSpanLike auto& in1Span, OutputSpanLike auto& outSpan, OutputSpanLike auto& orthoSpan, OutputSpanLike auto& measurementsSpan) {
        const bool        wantOrtho = emit_orthogonal.value && orthoSpan.isConnected;
        const std::size_t take      = std::min({in0Span.size(), in1Span.size(), outSpan.size(), wantOrtho ? orthoSpan.size() : std::numeric_limits<std::size_t>::max()});
        countDroppedTags(in0Span, in1Span, take);
        combine(std::span<const std::complex<float>>(in0Span).first(take), std::span<const std::complex<float>>(in1Span).first(take), std::span<std::complex<float>>(outSpan).first(take), std::span<std::complex<float>>(orthoSpan.data(), wantOrtho ? take : 0UZ), measurementsSpan.isConnected);

        std::size_t made = drain(measurementsSpan, 0UZ);
        if (!_flushed && _covCount > 0UZ) {
            closeWindow(measurementsSpan.isConnected, false);
            made += drain(measurementsSpan, made);
            _flushed = true;
        }
        outSpan.publish(take);
        orthoSpan.publish(wantOrtho ? take : 0UZ);
        measurementsSpan.publish(made);
        return work::Status::OK;
    }

private:
    /// @brief Count the `n_dropped_samples` tags sitting on the @p count samples this call takes, on either branch,
    /// bounded so a tag that belongs to a later call is not counted twice.
    void countDroppedTags(InputSpanLike auto& in0Span, InputSpanLike auto& in1Span, std::size_t count) {
        const std::uint64_t base   = _streamAt;
        const auto          onSpan = [this, base, count](const auto& span) {
            for (const gr::Tag& tag : span.rawTags) {
                const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
                if (at < base || at >= base + count) {
                    continue;
                }
                if (tag.map.find(gr::tag::N_DROPPED_SAMPLES.shortKey()) == tag.map.end()) {
                    continue;
                }
                _nDroppedSampleTags.fetch_add(1ULL, std::memory_order_relaxed);
            }
        };
        onSpan(in0Span);
        onSpan(in1Span);
    }

    void combine(std::span<const std::complex<float>> branch0, std::span<const std::complex<float>> branch1, std::span<std::complex<float>> combined, std::span<std::complex<float>> orthogonal, bool wantRecords) {
        std::size_t at = 0UZ;
        while (at < branch0.size()) {
            const std::size_t remaining = static_cast<std::size_t>(window.value) - _covCount;
            const std::size_t chunk     = std::min(remaining, branch0.size() - at);

            gr::measurement::polarizationCombine(branch0.subspan(at, chunk), branch1.subspan(at, chunk), _estimate, combined.subspan(at, chunk), orthogonal.empty() ? std::span<std::complex<float>>{} : orthogonal.subspan(at, chunk));
            _covariance.add(branch0.subspan(at, chunk), branch1.subspan(at, chunk));

            _covCount += chunk;
            _streamAt += chunk;
            at += chunk;
            if (_covCount == static_cast<std::size_t>(window.value)) {
                closeWindow(wantRecords, true);
            }
        }
    }

    void closeWindow(bool wantRecords, bool full) {
        const std::span<const double>         noisePowers(noise_powers.value);
        gr::measurement::PolarizationEstimate solved = _covariance.solve(_mode, _normalize, noisePowers);

        if (solved.saturatedSnr) {
            _nSaturatedFigures.fetch_add(1ULL, std::memory_order_relaxed);
        }
        if (solved.saturatedRatio) {
            _nSaturatedFigures.fetch_add(1ULL, std::memory_order_relaxed);
        }

        if (_mode == gr::measurement::PolarizationMode::selection) {
            if (_haveSelected && solved.selectedBranch != _lastSelected) {
                _nBranchSwitches.fetch_add(1ULL, std::memory_order_relaxed);
            }
            _lastSelected = solved.selectedBranch;
            _haveSelected = true;
        }

        gr::measurement::PolarizationEstimate applied = solved;
        if (weight_smoothing.value > 0. && _haveEstimate) {
            applied = smoothed(solved, _estimate, weight_smoothing.value, noisePowers);
        }

        const std::uint64_t filled  = full ? static_cast<std::uint64_t>(window.value) : static_cast<std::uint64_t>(_covCount);
        const DbFigures     figures = dbFigures(solved);
        publishSlot(solved, figures, filled);

        if (wantRecords && emit_records.value) {
            _pending.push_back(makeRecord(solved, figures, _windowStartAt, filled));
        }
        if (full) {
            _nWindows.fetch_add(1ULL, std::memory_order_relaxed);
        }

        _estimate     = applied;
        _haveEstimate = true;
        _covariance.reset();
        _covCount      = 0UZ;
        _windowStartAt = _streamAt;
    }

    /**
     * @brief `v_new = (1-g) v_est + g v_prev` on the *whitened* weight vectors, renormalized, gauge re-fixed, then
     * unwhitened again with the fresh estimate's own normalization scalar and the orthogonal complement rebuilt.
     *
     * Blending the unwhitened weights instead would mix two vectors living in different noise bases and would leave
     * an orthogonal channel that no longer nulls the signal once the branch noises differ, which is exactly the case
     * `noise_powers` exists for.
     */
    [[nodiscard]] static gr::measurement::PolarizationEstimate smoothed(const gr::measurement::PolarizationEstimate& fresh, const gr::measurement::PolarizationEstimate& prior, double g, std::span<const double> noisePowers) {
        const double sqrtN0 = noisePowers.empty() ? 1. : std::sqrt(noisePowers[0]);
        const double sqrtN1 = noisePowers.empty() ? 1. : std::sqrt(noisePowers[1]);

        // The whitened vector is the weight times sqrt(N_i), normalized: the estimate's own scale divides out.
        const auto whitened = [sqrtN0, sqrtN1](const gr::measurement::PolarizationEstimate& estimate) {
            std::complex<double> v0    = estimate.weight0 * sqrtN0;
            std::complex<double> v1    = estimate.weight1 * sqrtN1;
            const double         norm2 = std::sqrt(std::norm(v0) + std::norm(v1));
            if (norm2 > 0.) {
                v0 /= norm2;
                v1 /= norm2;
            }
            return std::pair<std::complex<double>, std::complex<double>>{v0, v1};
        };

        const auto [freshV0, freshV1] = whitened(fresh);
        const auto [priorV0, priorV1] = whitened(prior);

        std::complex<double> v0    = (1. - g) * freshV0 + g * priorV0;
        std::complex<double> v1    = (1. - g) * freshV1 + g * priorV1;
        const double         norm2 = std::sqrt(std::norm(v0) + std::norm(v1));
        if (norm2 > 0.) {
            v0 /= norm2;
            v1 /= norm2;
        }
        if (const double magnitude = std::abs(v0); magnitude > 0.) { // gauge: arg(v_0) = 0
            const std::complex<double> rotation = std::conj(v0) / magnitude;
            v0 *= rotation;
            v1 *= rotation;
        }

        gr::measurement::PolarizationEstimate out = fresh;
        out.weight0                               = fresh.scale * v0 / sqrtN0;
        out.weight1                               = fresh.scale * v1 / sqrtN1;
        out.ortho0                                = -std::conj(v1) / sqrtN0;
        out.ortho1                                = std::conj(v0) / sqrtN1;
        return out;
    }

    /**
     * @brief The five decibel figures a window reports, with the infinity an unbounded ratio produces replaced by
     * zero and the substitution flagged.
     *
     * A branch whose estimated signal power is exactly zero has a signal-to-noise ratio of zero, whose logarithm is
     * `-inf`, and the difference of two such is a NaN. Zero is the same placeholder a degenerate window uses, and
     * `finite` is what marks the record invalid, so no reader and no record channel ever carries a non-finite number.
     */
    struct DbFigures {
        double branch0{0.};
        double branch1{0.};
        double combined{0.};
        double gain{0.};
        bool   finite{true};
    };

    [[nodiscard]] static DbFigures dbFigures(const gr::measurement::PolarizationEstimate& e) noexcept {
        const double branch0  = e.branchSnrDb(0UZ);
        const double branch1  = e.branchSnrDb(1UZ);
        const double combined = e.combinedSnrDb();
        const double gain     = combined - std::max(branch0, branch1);

        DbFigures figures;
        figures.finite   = std::isfinite(branch0) && std::isfinite(branch1) && std::isfinite(combined) && std::isfinite(gain);
        figures.branch0  = std::isfinite(branch0) ? branch0 : 0.;
        figures.branch1  = std::isfinite(branch1) ? branch1 : 0.;
        figures.combined = std::isfinite(combined) ? combined : 0.;
        figures.gain     = std::isfinite(gain) ? gain : 0.;
        return figures;
    }

    void publishSlot(const gr::measurement::PolarizationEstimate& e, const DbFigures& figures, std::uint64_t filled) { _slot.publish({e.relativePhase, e.amplitudeRatio, figures.branch0, figures.branch1, figures.combined, static_cast<double>(e.selectedBranch), figures.gain}, filled); }

    [[nodiscard]] DataSet<float> makeRecord(const gr::measurement::PolarizationEstimate& e, const DbFigures& figures, std::uint64_t sampleStart, std::uint64_t filled) const {
        const std::array<gr::measurement::ScalarChannel, 6UZ> channels{{
            {"relative_phase", "Phase", "rad", static_cast<float>(e.relativePhase)},
            {"amplitude_ratio", "Ratio", "", static_cast<float>(e.amplitudeRatio)},
            {"branch0_snr_db", "SNR", "dB", static_cast<float>(figures.branch0)},
            {"branch1_snr_db", "SNR", "dB", static_cast<float>(figures.branch1)},
            {"combined_snr_db", "SNR", "dB", static_cast<float>(figures.combined)},
            {"combining_gain_db", "CombiningGain", "dB", static_cast<float>(figures.gain)},
        }};
        property_map                                          extra{
                                                     {std::pmr::string("window"), pmt::Value(static_cast<std::uint64_t>(window.value))},
                                                     {std::pmr::string("mode"), pmt::Value(mode.value)},
                                                     {std::pmr::string("normalize"), pmt::Value(normalize.value)},
                                                     {std::pmr::string("selected_branch"), pmt::Value(static_cast<std::int32_t>(e.selectedBranch))},
                                                     {std::pmr::string("valid"), pmt::Value(filled > 0ULL && figures.finite)},
        };
        return gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), sample_rate.value, sampleStart, std::move(extra));
    }

    [[nodiscard]] std::size_t roomFor(OutputSpanLike auto& measurementsSpan, std::size_t made) const {
        if (!measurementsSpan.isConnected) {
            return std::numeric_limits<std::size_t>::max();
        }
        const std::size_t free       = measurementsSpan.size() > made + _pending.size() ? measurementsSpan.size() - made - _pending.size() : 0UZ;
        const std::size_t windowSize = static_cast<std::size_t>(window.value);
        const std::size_t untilClose = windowSize > _covCount ? windowSize - _covCount : 0UZ;
        if (free == 0UZ) {
            return untilClose > 0UZ ? untilClose - 1UZ : 0UZ;
        }
        return untilClose + (free - 1UZ) * windowSize;
    }

    [[nodiscard]] std::size_t drain(OutputSpanLike auto& measurementsSpan, std::size_t at) {
        if (_pending.empty() || !measurementsSpan.isConnected || at >= measurementsSpan.size()) {
            return 0UZ;
        }
        const std::size_t made = std::min(_pending.size(), measurementsSpan.size() - at);
        for (std::size_t k = 0UZ; k < made; ++k) {
            measurementsSpan[at + k] = std::move(_pending[k]);
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        return made;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_POLARIZATION_COMBINER_HPP
