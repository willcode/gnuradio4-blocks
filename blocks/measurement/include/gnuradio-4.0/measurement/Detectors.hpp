#ifndef GNURADIO_MEASUREMENT_DETECTORS_HPP
#define GNURADIO_MEASUREMENT_DETECTORS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/fourier/SpectralCalibration.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::measurement {

namespace detail {

/// @brief One detection, in the units a consumer reads rather than in bins.
struct Detection {
    float frequencyHz{};
    float levelDb{};
    float widthHz{};
};

/// @brief The three columns a detection record carries, one record per input record that found something.
[[nodiscard]] inline DataSet<float> makeDetectionRecord(const std::vector<Detection>& detections, const DataSet<float>& source, std::string_view producer) {
    DataSet<float>    ds;
    const std::size_t n = detections.size();

    ds.extents = {static_cast<std::int32_t>(n)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Detection"};
    ds.axis_units = {"index"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k);
    }

    ds.signal_names      = {"frequency", "level", "width"};
    ds.signal_quantities = {"Frequency", "PowerSpectralDensity", "Bandwidth"};
    // A detection is read by eye and by threshold, so its level is in decibels — the logarithm of the source
    // record's linear density, against the same full-scale-sine reference the source names.
    ds.signal_units = {"Hz", "dBFS/Hz", "Hz"};
    ds.signal_values.resize(3UZ * n);
    ds.signal_ranges.resize(3UZ);
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.signal_values[k]           = detections[k].frequencyHz;
        ds.signal_values[n + k]       = detections[k].levelDb;
        ds.signal_values[2UZ * n + k] = detections[k].widthHz;
    }

    // the source record's facts carry through, so a detection stays timestamped by provenance rather than invention
    ds.meta_information.resize(3UZ);
    property_map carried;
    if (!source.meta_information.empty()) {
        carried = source.meta_information[0UZ];
    }
    carried.insert_or_assign(property_map::key_type("n_detections"), pmt::Value(static_cast<std::uint64_t>(n)));
    carried.insert_or_assign(property_map::key_type("detector"), pmt::Value(std::string(producer)));
    for (std::size_t s = 0UZ; s < 3UZ; ++s) {
        ds.meta_information[s] = carried;
    }
    ds.timing_events.resize(3UZ);
    ds.timestamp = source.timestamp;
    return ds;
}

/// @brief The frequency step of a record's axis, or NaN when the axis cannot state one.
[[nodiscard]] inline float axisStep(const DataSet<float>& record) {
    if (record.axis_values.empty() || record.axis_values[0UZ].size() < 2UZ) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return record.axis_values[0UZ][1UZ] - record.axis_values[0UZ][0UZ];
}

/**
 * @brief Sub-bin peak position by three-point parabolic interpolation, in bins relative to `k`.
 *
 * Fits a parabola through the decibel values at `k-1`, `k`, `k+1` and returns its vertex. The refinement is exact
 * for a parabola and biased for a windowed sinc, by an amount that depends on the window and that the qa records
 * rather than assumes. The decibel domain is used because a windowed main lobe is far closer to a parabola in
 * decibels than in linear power, which is what makes the correction worth applying at all.
 */
[[nodiscard]] inline float parabolicOffset(float leftDb, float centerDb, float rightDb) noexcept {
    const float denominator = leftDb - 2.f * centerDb + rightDb;
    if (!(std::abs(denominator) > 0.f)) {
        return 0.f;
    }
    const float offset = 0.5f * (leftDb - rightDb) / denominator;
    return std::abs(offset) <= 1.f ? offset : 0.f; // a vertex outside the neighboring bins is not a refinement
}

/**
 * @brief Half-power width around `k`, in bins, by linear interpolation on the record's own linear power.
 *
 * Both detectors state a width the same way. The walk runs outward from `k` to the first bin at or below half the
 * peak's power on each side and interpolates between it and its inward neighbor. Half power is a linear notion, so
 * the arithmetic stays in the record's stored form and no detector has to build a decibel curve to state a width.
 * A side that never falls to half power before the record ends contributes zero, which is what makes a width of
 * zero mean "the record does not contain this signal's skirt" rather than "the signal is one bin wide".
 */
[[nodiscard]] inline float halfPowerWidthBins(std::span<const float> power, std::size_t k) noexcept {
    const float target = 0.5f * power[k];
    float       left   = 0.f;
    float       right  = 0.f;

    std::size_t i = k;
    while (i > 0UZ && power[i] > target) {
        --i;
    }
    if (power[i] <= target && i < k) {
        const float span = power[i + 1UZ] - power[i];
        left             = static_cast<float>(k - i) - (span > 0.f ? (target - power[i]) / span : 0.f);
    }

    std::size_t j = k;
    while (j + 1UZ < power.size() && power[j] > target) {
        ++j;
    }
    if (power[j] <= target && j > k) {
        const float span = power[j - 1UZ] - power[j];
        right            = static_cast<float>(j - k) - (span > 0.f ? (target - power[j]) / span : 0.f);
    }
    return left + right;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::PeakDetect)

/**
 * @brief Local maxima of a density record, above a stated threshold, with sub-bin frequency.
 *
 * `reference` says what `threshold_db` is measured against: `absolute` reads it as a level in dBFS per hertz, and
 * `above_median` reads it as a margin over the record's own median, which is the reading that survives a changing
 * noise floor without being re-tuned. Peaks closer together than `min_distance_hz` collapse to the strongest, taken
 * in order of level, so a single emitter's shoulder does not arrive as a second signal.
 *
 * A record that yields no detection emits no record: an empty `DataSet` fails the tier's admission predicates, whose
 * first question of a record is whether its extent is positive. The absence of a record for a PSD input is therefore
 * what says nothing was found, and `nEmptyResults()` counts how often that has happened beside `nRecords()`, so a
 * graph can still tell "nothing found" from "nothing ran".
 */
struct PeakDetect : Block<PeakDetect, NoTagPropagation> {
    using Description = Doc<"Peak detection on a spectral density record: local maxima above a threshold, with three-point parabolic sub-bin frequency and half-power width. A record that finds nothing emits no record and is counted by nEmptyResults()">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<double, "threshold_db", Visible, Unit<"dB">, Doc<"the level a maximum must reach, read against `reference`">> threshold_db    = 10.0;
    Annotated<std::string, "reference", Visible, Doc<"absolute (dBFS/Hz) or above_median (dB over the record's median)">>   reference       = std::string("above_median");
    Annotated<double, "min_distance_hz", Visible, Unit<"Hz">, Doc<"peaks nearer than this collapse to the strongest">>      min_distance_hz = 0.0;
    Annotated<gr::Size_t, "max_peaks", Visible, Doc<"0 keeps every peak that passes; otherwise the strongest this many">>   max_peaks       = 0U;

    GR_MAKE_REFLECTABLE(PeakDetect, in, out, threshold_db, reference, min_distance_hz, max_peaks);

    std::vector<float>             _db{};
    std::vector<float>             _sorted{};
    std::vector<detail::Detection> _found{};
    std::atomic<std::uint64_t>     _records{0ULL};
    std::atomic<std::uint64_t>     _emptyResults{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (reference.value != "absolute" && reference.value != "above_median") {
            throw gr::exception(std::format("reference must be 'absolute' or 'above_median', got '{}'", reference.value));
        }
        if (!(min_distance_hz >= 0.0)) {
            throw gr::exception(std::format("min_distance_hz is a separation and cannot be negative, got {}", min_distance_hz.value));
        }
    }

    void start() {
        _records.store(0ULL, std::memory_order_relaxed);
        _emptyResults.store(0ULL, std::memory_order_relaxed);
    }

    /// @brief Detection records emitted so far. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _records.load(std::memory_order_relaxed); }

    /// @brief Input records that found nothing and therefore emitted no record. Callable from any thread.
    [[nodiscard]] std::uint64_t nEmptyResults() const noexcept { return _emptyResults.load(std::memory_order_relaxed); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            detect(inSpan[consumed]);
            if (_found.empty()) {
                _emptyResults.fetch_add(1ULL, std::memory_order_relaxed);
                continue;
            }
            outSpan[made] = detail::makeDetectionRecord(_found, inSpan[consumed], "peak");
            ++made;
            _records.fetch_add(1ULL, std::memory_order_relaxed);
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(consumed);
        return consumed == 0UZ && inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

private:
    void detect(const DataSet<float>& record) {
        const std::span<const float> values(record.signal_values);
        const float                  step = detail::axisStep(record);
        _found.clear();

        if (values.size() < 3UZ || !std::isfinite(step)) {
            return;
        }
        _db.assign(values.size(), 0.f);
        for (std::size_t k = 0UZ; k < values.size(); ++k) {
            _db[k] = gr::algorithm::fft::powerToDb(values[k]);
        }

        float floorDb = 0.f;
        if (reference.value == "above_median" && !_db.empty()) {
            _sorted.assign(_db.begin(), _db.end()); // a member, so a record costs no allocation once the sizes settle
            const std::size_t middle = _sorted.size() / 2UZ;
            std::nth_element(_sorted.begin(), _sorted.begin() + static_cast<std::ptrdiff_t>(middle), _sorted.end());
            floorDb = _sorted[middle];
        }
        const float cutoff = floorDb + static_cast<float>(threshold_db.value);

        // A maximum is `>=` on the left and `>` on the right, so a plateau — two bins of exactly equal level,
        // which is what a lobe centered half a bin away produces — reports once rather than twice.
        for (std::size_t k = 1UZ; k + 1UZ < _db.size(); ++k) {
            if (_db[k] <= cutoff || _db[k] < _db[k - 1UZ] || _db[k] <= _db[k + 1UZ]) {
                continue;
            }
            const float offset = detail::parabolicOffset(_db[k - 1UZ], _db[k], _db[k + 1UZ]);
            _found.push_back(detail::Detection{
                .frequencyHz = record.axis_values[0UZ][k] + offset * step,
                .levelDb     = _db[k],
                .widthHz     = detail::halfPowerWidthBins(values, k) * std::abs(step),
            });
        }

        std::ranges::stable_sort(_found, [](const detail::Detection& a, const detail::Detection& b) { return a.levelDb > b.levelDb; });
        if (min_distance_hz > 0.0) {
            std::vector<detail::Detection> kept;
            for (const detail::Detection& candidate : _found) {
                const bool crowded = std::ranges::any_of(kept, [&](const detail::Detection& other) { return std::abs(other.frequencyHz - candidate.frequencyHz) < static_cast<float>(min_distance_hz.value); });
                if (!crowded) {
                    kept.push_back(candidate);
                }
            }
            _found = std::move(kept);
        }
        if (max_peaks.value > 0U && _found.size() > static_cast<std::size_t>(max_peaks.value)) {
            _found.resize(static_cast<std::size_t>(max_peaks.value));
        }
        std::ranges::stable_sort(_found, [](const detail::Detection& a, const detail::Detection& b) { return a.frequencyHz < b.frequencyHz; });
    }
};

GR_REGISTER_BLOCK(gr::blocks::measurement::CfarDetect)

/**
 * @brief Cell-averaging constant-false-alarm-rate detection on a density record.
 *
 * For each cell under test the noise is estimated as the mean of `n_train` cells each side, with `n_guard` cells
 * either side excluded so a wide signal does not raise the estimate of its own noise. A cell is a detection when it
 * exceeds `alpha` times that estimate, with
 *
 *     alpha = N * (pfa^(-1/N) - 1),    N = 2 * n_train
 *
 * the closed cell-averaging form: for exponentially distributed power, that threshold holds the false-alarm rate at
 * `pfa` whatever the noise level is, which is the whole point of the method and is what the qa measures rather than
 * assumes.
 *
 * The first and last `n_train + n_guard` cells have no full training window and are never tested. That is stated
 * because it is also the denominator of any false-alarm rate computed from these records.
 *
 * A detection's `width` is the half-power width of the peak it sits on, measured the way `PeakDetect` measures one;
 * a cell whose skirt does not fall to half power inside the record reports zero. Its `frequency` is the cell's own,
 * unrefined: a CFAR cell is a decision about one bin, and this detector states no sub-bin position.
 *
 * A record that yields no detection emits no record — an empty `DataSet` fails the tier's admission predicates —
 * and `nEmptyResults()` counts how often that happened beside `nRecords()`.
 */
struct CfarDetect : Block<CfarDetect, NoTagPropagation> {
    using Description = Doc<"Cell-averaging CFAR detection on a spectral density record, at a stated design false-alarm rate. A record that finds nothing emits no record and is counted by nEmptyResults()">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<gr::Size_t, "n_train", Visible, Doc<"training cells each side of the guard band">>        n_train = 16U;
    Annotated<gr::Size_t, "n_guard", Visible, Doc<"cells each side excluded from the noise estimate">>  n_guard = 2U;
    Annotated<double, "pfa", Visible, Doc<"design false-alarm probability per tested cell, in (0, 1)">> pfa     = 1e-3;

    GR_MAKE_REFLECTABLE(CfarDetect, in, out, n_train, n_guard, pfa);

    double                         _alpha = 0.;
    std::vector<detail::Detection> _found{};
    std::atomic<std::uint64_t>     _records{0ULL};
    std::atomic<std::uint64_t>     _emptyResults{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n_train.value == 0U) {
            throw gr::exception("n_train counts the cells the noise is estimated from and must be at least 1");
        }
        if (!(pfa > 0.0) || !(pfa < 1.0)) {
            throw gr::exception(std::format("pfa is a probability and must lie in (0, 1), got {}", pfa.value));
        }
        const double n = 2. * static_cast<double>(n_train.value);
        _alpha         = n * (std::pow(pfa.value, -1. / n) - 1.);
    }

    void start() {
        _records.store(0ULL, std::memory_order_relaxed);
        _emptyResults.store(0ULL, std::memory_order_relaxed);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            detect(inSpan[consumed]);
            if (_found.empty()) {
                _emptyResults.fetch_add(1ULL, std::memory_order_relaxed);
                continue;
            }
            outSpan[made] = detail::makeDetectionRecord(_found, inSpan[consumed], "cfar");
            ++made;
            _records.fetch_add(1ULL, std::memory_order_relaxed);
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(consumed);
        return consumed == 0UZ && inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief The multiplier the design false-alarm rate implies, exposed so a graph can state what it is running at.
    [[nodiscard]] double alpha() const noexcept { return _alpha; }

    /// @brief The cells a record of `bins` bins actually tests — the denominator of a measured false-alarm rate.
    [[nodiscard]] std::size_t testableCells(std::size_t bins) const noexcept {
        const std::size_t margin = static_cast<std::size_t>(n_train.value) + static_cast<std::size_t>(n_guard.value);
        return bins > 2UZ * margin ? bins - 2UZ * margin : 0UZ;
    }

    /// @brief Detection records emitted so far. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _records.load(std::memory_order_relaxed); }

    /// @brief Input records that found nothing and therefore emitted no record. Callable from any thread.
    [[nodiscard]] std::uint64_t nEmptyResults() const noexcept { return _emptyResults.load(std::memory_order_relaxed); }

private:
    void detect(const DataSet<float>& record) {
        const std::span<const float> values(record.signal_values);
        const float                  step = detail::axisStep(record);
        _found.clear();

        const std::size_t train  = static_cast<std::size_t>(n_train.value);
        const std::size_t guard  = static_cast<std::size_t>(n_guard.value);
        const std::size_t margin = train + guard;

        if (values.size() <= 2UZ * margin || !std::isfinite(step)) {
            return;
        }
        for (std::size_t k = margin; k + margin < values.size(); ++k) {
            double noise = 0.;
            for (std::size_t t = 0UZ; t < train; ++t) {
                noise += static_cast<double>(values[k - guard - 1UZ - t]);
                noise += static_cast<double>(values[k + guard + 1UZ + t]);
            }
            noise /= 2. * static_cast<double>(train);
            if (static_cast<double>(values[k]) > _alpha * noise) {
                _found.push_back(detail::Detection{
                    .frequencyHz = record.axis_values[0UZ][k],
                    .levelDb     = gr::algorithm::fft::powerToDb(values[k]),
                    .widthHz     = detail::halfPowerWidthBins(values, k) * std::abs(step),
                });
            }
        }
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_DETECTORS_HPP
