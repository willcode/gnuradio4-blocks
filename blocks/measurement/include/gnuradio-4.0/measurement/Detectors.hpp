#ifndef GNURADIO_MEASUREMENT_DETECTORS_HPP
#define GNURADIO_MEASUREMENT_DETECTORS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/fourier/SpectralCalibration.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::measurement {

namespace detail {

/// @brief One detection: its frequency and width in hertz, and its level in decibels.
struct Detection {
    float frequencyHz{};
    float levelDb{};
    float widthHz{};
};

/// @brief Builds a detection record with three signals, `frequency`, `level` and `width`, and one value per detection.
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
    // The level is the source record's density in decibels, on the source record's level reference.
    ds.signal_units = {"Hz", "dBFS/Hz", "Hz"};
    ds.signal_values.resize(3UZ * n);
    ds.signal_ranges.resize(3UZ);
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.signal_values[k]           = detections[k].frequencyHz;
        ds.signal_values[n + k]       = detections[k].levelDb;
        ds.signal_values[2UZ * n + k] = detections[k].widthHz;
    }

    // Each signal carries the source record's metadata, the detection count and the detector's name. The record keeps
    // the source's timestamp.
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

/**
 * @brief The first signal of a one-dimensional record, or an empty span for a record the detectors cannot read.
 *
 * A readable record has one positive extent, at least one signal name, `extent` values per signal and an axis of
 * `extent` values. The span holds the first `extent` values, and the detectors read them against the axis.
 */
[[nodiscard]] inline std::span<const float> firstSignal(const DataSet<float>& record) {
    if (record.extents.size() != 1UZ || record.extents[0UZ] <= 0 || record.signal_names.empty() || record.axis_values.empty()) {
        return {};
    }
    const auto extent = static_cast<std::size_t>(record.extents[0UZ]);
    if (record.signal_values.size() != record.signal_names.size() * extent || record.axis_values[0UZ].size() != extent) {
        return {};
    }
    return std::span<const float>(record.signal_values).first(extent);
}

/// @brief The frequency step of a record's axis, or NaN when the axis has fewer than two values.
[[nodiscard]] inline float axisStep(const DataSet<float>& record) {
    if (record.axis_values.empty() || record.axis_values[0UZ].size() < 2UZ) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return record.axis_values[0UZ][1UZ] - record.axis_values[0UZ][0UZ];
}

/**
 * @brief The offset of a peak from its center bin by three-point parabolic interpolation, in bins.
 *
 * The parabola runs through the levels of the center bin and its two neighbors, in decibels, and the offset is the
 * position of its vertex. The offset is exact for a parabola. For a windowed tone it has a bias that depends on the
 * window. A windowed main lobe is closer to a parabola in decibels than in linear power. Three levels on a straight
 * line give zero, and so does a vertex more than one bin from the center.
 */
[[nodiscard]] inline float parabolicOffset(float leftDb, float centerDb, float rightDb) noexcept {
    const float denominator = leftDb - 2.f * centerDb + rightDb;
    if (!(std::abs(denominator) > 0.f)) {
        return 0.f;
    }
    const float offset = 0.5f * (leftDb - rightDb) / denominator;
    return std::abs(offset) <= 1.f ? offset : 0.f;
}

/**
 * @brief The half-power width of the peak at bin `k`, in bins.
 *
 * On each side, the search runs outward from `k` to the first bin at or below half the peak's power. The edge lies
 * between that bin and its inward neighbor, by linear interpolation on the record's linear power. A side that stays
 * above half power to the end of the record adds zero to the width.
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
 * @brief Reports the local maxima of a density record above a threshold, with a sub-bin frequency and a half-power
 * width.
 *
 * The block reads the first signal of each record against the record's axis. A record whose axis or values do not
 * match its extent is dropped, and `nMalformed()` counts it.
 *
 * `reference` selects the level `threshold_db` is measured from. With `absolute`, the threshold is a level in dBFS per
 * hertz. With `above_median`, it is a margin in decibels over the median bin of each record. The block takes the
 * maxima in order of level and drops each one within `min_distance_hz` of a stronger one it kept. `max_peaks` then
 * keeps the strongest. The record lists the detections in order of frequency.
 *
 * An input record with no detection produces no output record. `nEmptyResults()` counts those input records, and
 * `nRecords()` counts the records emitted.
 *
 * The block detects on the settings it last accepted, starting at the defaults. A refused application changes nothing
 * the block computes. The refused value stays in its member, and an application that checks that member again is
 * refused until the member holds a value the block accepts. The block stops every input tag.
 */
struct PeakDetect : Block<PeakDetect, NoTagPropagation> {
    using Description = Doc<"Detects peaks in the first signal of a spectral density record. Each peak is a local maximum above a threshold. The block reports its frequency, refined by a three-point parabolic fit, its level in dBFS/Hz and its half-power width. An input record with no peak produces no output record, and nEmptyResults() counts it. The block stops every input tag.">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<double, "threshold_db", Visible, Unit<"dB">, Doc<"level a maximum must exceed, measured from reference">>                   threshold_db    = 10.0;
    Annotated<std::string, "reference", Visible, Doc<"threshold reference, 'absolute' (dBFS/Hz) or 'above_median' (dB over the median)">> reference       = std::string("above_median");
    Annotated<double, "min_distance_hz", Visible, Unit<"Hz">, Doc<"least separation between reported peaks">>                             min_distance_hz = 0.0;
    Annotated<gr::Size_t, "max_peaks", Visible, Doc<"largest number of peaks kept, strongest first; 0 keeps all">>                        max_peaks       = 0U;

    GR_MAKE_REFLECTABLE(PeakDetect, in, out, threshold_db, reference, min_distance_hz, max_peaks);

    double      _thresholdDb   = threshold_db.value;                ///< `threshold_db` as last accepted
    bool        _aboveMedian   = reference.value == "above_median"; ///< `reference` as last accepted
    double      _minDistanceHz = min_distance_hz.value;             ///< `min_distance_hz` as last accepted
    std::size_t _maxPeaks      = static_cast<std::size_t>(max_peaks.value);

    std::vector<float>             _db{};
    std::vector<float>             _sorted{};
    std::vector<detail::Detection> _found{};
    std::atomic<std::uint64_t>     _records{0ULL};
    std::atomic<std::uint64_t>     _emptyResults{0ULL};
    std::atomic<std::uint64_t>     _malformed{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (reference.value != "absolute" && reference.value != "above_median") {
            throw gr::exception(std::format("reference must be 'absolute' or 'above_median', got '{}'", reference.value));
        }
        if (!(min_distance_hz >= 0.0)) {
            throw gr::exception(std::format("min_distance_hz must be at least 0, got {}", min_distance_hz.value));
        }
        _thresholdDb   = threshold_db.value;
        _aboveMedian   = reference.value == "above_median";
        _minDistanceHz = min_distance_hz.value;
        _maxPeaks      = static_cast<std::size_t>(max_peaks.value);
    }

    void start() {
        _records.store(0ULL, std::memory_order_relaxed);
        _emptyResults.store(0ULL, std::memory_order_relaxed);
        _malformed.store(0ULL, std::memory_order_relaxed);
    }

    /// @brief Detection records emitted in this run. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _records.load(std::memory_order_relaxed); }

    /// @brief Input records in this run with no detection. Callable from any thread.
    [[nodiscard]] std::uint64_t nEmptyResults() const noexcept { return _emptyResults.load(std::memory_order_relaxed); }

    /// @brief Input records in this run the block dropped because their axis or values do not match their extent.
    /// Callable from any thread.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed.load(std::memory_order_relaxed); }

    /// @brief Takes input records while the output has room. A call that moves nothing returns
    /// `INSUFFICIENT_INPUT_ITEMS` for an empty input and `INSUFFICIENT_OUTPUT_ITEMS` for a full output.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const std::span<const float> values = detail::firstSignal(inSpan[consumed]);
            if (values.empty()) {
                _malformed.fetch_add(1ULL, std::memory_order_relaxed);
                continue;
            }
            detect(inSpan[consumed], values);
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
        if (consumed > 0UZ) {
            return work::Status::OK;
        }
        return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
    }

private:
    void detect(const DataSet<float>& record, std::span<const float> values) {
        const float step = detail::axisStep(record);
        _found.clear();

        if (values.size() < 3UZ || !std::isfinite(step)) {
            return;
        }
        _db.assign(values.size(), 0.f);
        for (std::size_t k = 0UZ; k < values.size(); ++k) {
            _db[k] = gr::algorithm::fft::powerToDb(values[k]);
        }

        float floorDb = 0.f;
        if (_aboveMedian && !_db.empty()) {
            _sorted.assign(_db.begin(), _db.end());
            const std::size_t middle = _sorted.size() / 2UZ;
            std::nth_element(_sorted.begin(), _sorted.begin() + static_cast<std::ptrdiff_t>(middle), _sorted.end());
            floorDb = _sorted[middle];
        }
        const float cutoff = floorDb + static_cast<float>(_thresholdDb);

        // A maximum is at least its left neighbor and above its right neighbor. A plateau of two equal bins gives one
        // detection, at its right bin. A tone midway between two bins gives such a plateau.
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
        if (_minDistanceHz > 0.0) {
            std::vector<detail::Detection> kept;
            for (const detail::Detection& candidate : _found) {
                const bool crowded = std::ranges::any_of(kept, [&](const detail::Detection& other) { return std::abs(other.frequencyHz - candidate.frequencyHz) < static_cast<float>(_minDistanceHz); });
                if (!crowded) {
                    kept.push_back(candidate);
                }
            }
            _found = std::move(kept);
        }
        if (_maxPeaks > 0UZ && _found.size() > _maxPeaks) {
            _found.resize(_maxPeaks);
        }
        std::ranges::stable_sort(_found, [](const detail::Detection& a, const detail::Detection& b) { return a.frequencyHz < b.frequencyHz; });
    }
};

GR_REGISTER_BLOCK(gr::blocks::measurement::CfarDetect)

/**
 * @brief Cell-averaging constant false-alarm rate (CFAR) detection on a density record.
 *
 * For each cell under test, the noise estimate is the mean of `n_train` training cells on each side. `n_guard` guard
 * cells on each side separate the cell under test from its training cells and stay out of the estimate. A cell is a
 * detection when its power exceeds `alpha` times the estimate:
 *
 *     alpha = N * (pfa^(-1/N) - 1),    N = 2 * n_train
 *
 * For exponentially distributed noise power, this threshold gives a false-alarm probability of `pfa` per cell at any
 * noise level. The bins of a single periodogram have that distribution, as do the records of `WelchPsd` at an
 * `n_averages` of 1. An average of several segments narrows the spread of each bin, and on such records the
 * false-alarm rate falls below `pfa`.
 *
 * The block tests every cell with a full training window on both sides. The first and last `n_train + n_guard` cells
 * have none. `testableCells()` gives the number of cells tested.
 *
 * A detection reports the cell's own frequency, with no sub-bin refinement. Its width is the half-power width of the
 * peak at that cell, measured as in `PeakDetect`. The block reads the first signal of each record, and it counts empty
 * results and malformed records as `PeakDetect` does. It computes on the settings it last accepted, as `PeakDetect`
 * does, and it stops every input tag.
 */
struct CfarDetect : Block<CfarDetect, NoTagPropagation> {
    using Description = Doc<"Detects the cells of a spectral density record that exceed a cell-averaging CFAR threshold. The threshold gives a false-alarm probability of pfa per cell in exponentially distributed noise, the distribution of a single periodogram's bins. Each detection reports the cell's frequency, its level in dBFS/Hz and its half-power width. An input record with no detection produces no output record, and nEmptyResults() counts it. The block stops every input tag.">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<gr::Size_t, "n_train", Visible, Doc<"training cells on each side, at least 1">>                     n_train = 16U;
    Annotated<gr::Size_t, "n_guard", Visible, Doc<"guard cells on each side of the cell under test">>             n_guard = 2U;
    Annotated<double, "pfa", Visible, Doc<"false-alarm probability per cell of a single periodogram, in (0, 1)">> pfa     = 1e-3;

    GR_MAKE_REFLECTABLE(CfarDetect, in, out, n_train, n_guard, pfa);

    std::size_t _train = static_cast<std::size_t>(n_train.value); ///< `n_train` as last accepted
    std::size_t _guard = static_cast<std::size_t>(n_guard.value); ///< `n_guard` as last accepted
    double      _alpha = alphaFor(_train, pfa.value);             ///< the multiplier of the accepted settings

    std::vector<detail::Detection> _found{};
    std::atomic<std::uint64_t>     _records{0ULL};
    std::atomic<std::uint64_t>     _emptyResults{0ULL};
    std::atomic<std::uint64_t>     _malformed{0ULL};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n_train.value == 0U) {
            throw gr::exception("n_train must be at least 1");
        }
        if (!(pfa > 0.0) || !(pfa < 1.0)) {
            throw gr::exception(std::format("pfa is a probability and must lie in (0, 1), got {}", pfa.value));
        }
        _train = static_cast<std::size_t>(n_train.value);
        _guard = static_cast<std::size_t>(n_guard.value);
        _alpha = alphaFor(_train, pfa.value);
    }

    void start() {
        _records.store(0ULL, std::memory_order_relaxed);
        _emptyResults.store(0ULL, std::memory_order_relaxed);
        _malformed.store(0ULL, std::memory_order_relaxed);
    }

    /// @brief Takes input records while the output has room, and reports a call that moves nothing as `PeakDetect`
    /// does.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const std::span<const float> values = detail::firstSignal(inSpan[consumed]);
            if (values.empty()) {
                _malformed.fetch_add(1ULL, std::memory_order_relaxed);
                continue;
            }
            detect(inSpan[consumed], values);
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
        if (consumed > 0UZ) {
            return work::Status::OK;
        }
        return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
    }

    /// @brief `N * (pfa^(-1/N) - 1)` at `N = 2 * train`.
    [[nodiscard]] static double alphaFor(std::size_t train, double falseAlarm) noexcept {
        const double n = 2. * static_cast<double>(train);
        return n * (std::pow(falseAlarm, -1. / n) - 1.);
    }

    /// @brief The threshold multiplier of the accepted `n_train` and `pfa`.
    [[nodiscard]] double alpha() const noexcept { return _alpha; }

    /// @brief The number of cells tested in a record of `bins` bins, at the accepted `n_train` and `n_guard`.
    [[nodiscard]] std::size_t testableCells(std::size_t bins) const noexcept {
        const std::size_t margin = _train + _guard;
        return bins > 2UZ * margin ? bins - 2UZ * margin : 0UZ;
    }

    /// @brief Detection records emitted in this run. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _records.load(std::memory_order_relaxed); }

    /// @brief Input records in this run with no detection. Callable from any thread.
    [[nodiscard]] std::uint64_t nEmptyResults() const noexcept { return _emptyResults.load(std::memory_order_relaxed); }

    /// @brief Input records in this run the block dropped, as `PeakDetect::nMalformed()` counts them.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed.load(std::memory_order_relaxed); }

private:
    void detect(const DataSet<float>& record, std::span<const float> values) {
        const float step = detail::axisStep(record);
        _found.clear();

        const std::size_t train  = _train;
        const std::size_t guard  = _guard;
        const std::size_t margin = train + guard;

        if (values.size() <= 2UZ * margin || !std::isfinite(step)) {
            return;
        }
        const double threshold = _alpha;
        for (std::size_t k = margin; k + margin < values.size(); ++k) {
            double noise = 0.;
            for (std::size_t t = 0UZ; t < train; ++t) {
                noise += static_cast<double>(values[k - guard - 1UZ - t]);
                noise += static_cast<double>(values[k + guard + 1UZ + t]);
            }
            noise /= 2. * static_cast<double>(train);
            if (static_cast<double>(values[k]) > threshold * noise) {
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
