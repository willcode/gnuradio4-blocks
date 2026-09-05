#ifndef GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP
#define GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>

namespace gr::blocks::measurement {

namespace detail {

inline constexpr gr::Size_t kMinSpectralKurtosisSpectra = 8U;
inline constexpr gr::Size_t kMaxSpectralKurtosisSpectra = 1U << 20U;

/// @brief A `double` meta value read whichever numeric alternative the map holds; `std::nullopt` when absent or the
/// wrong kind entirely. `WelchPsd` writes `n_averaged` as `std::uint64_t` and `overlap` as `double`, and this reads
/// either without the caller having to know which.
[[nodiscard]] inline std::optional<double> metaNumber(const property_map& map, std::string_view key) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const auto* asU64 = it->second.get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asDouble = it->second.get_if<double>()) {
        return *asDouble;
    }
    if (const auto* asFloat = it->second.get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    return std::nullopt;
}

[[nodiscard]] inline std::optional<std::string> metaString(const property_map& map, std::string_view key) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const auto* asString = it->second.get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return std::nullopt;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::measurement::SpectralKurtosis)

/**
 * @brief Spectral kurtosis per bin over `n_spectra` independent density records, homogeneous in the record's own scale.
 *
 * `SpectralKurtosisAccumulator` folds one `S1`/`S2` pair per bin; at `n_spectra` inputs the per-bin statistic is
 * evaluated and one record emitted. Independence is the estimator's precondition and this cannot see it in the
 * numbers, so it is checked in the one place it is stated: a producer's `overlap` metadata. A record is folded only
 * when it carries `overlap` and that value is exactly zero — overlapping Welch segments share samples and bias
 * `E[SK]` away from the one exact value (1) the statistic is defined to hold on noise, and a producer that states no
 * overlap at all has not said its spectra are independent.
 *
 * The shape parameter `d` is taken from the producer's own `n_averaged` metadata by default (`shape == 0`); a
 * nonzero `shape` is instead checked for agreement with the record and refused on a disagreement, which is what
 * catches a caller's setting drifting out of step with the producer's own configuration. A bin whose accumulated
 * `S1` is exactly zero has no defined ratio and is written `0.f`, counted in `nDegenerateBins()`.
 */
struct SpectralKurtosis : Block<SpectralKurtosis, NoTagPropagation> {
    using Description = Doc<R""(
@brief Spectral kurtosis per frequency bin over a run of independent density records.

Consumes `DataSet<float>` density records (as `WelchPsd` or `Spectrogram` emit) and emits one `spectral_kurtosis`
record per `n_spectra` inputs, one value per bin. `SK` reads exactly 1 on noise, 0 on a noiseless tone, above 1 on an
intermittent interferer and below 1 on steady structure. A record is refused unless its `overlap` metadata is present
and exactly zero: the statistic's derivation assumes independent spectra, which overlapping Welch segments are not.
)"">;

    PortIn<DataSet<float>, Async>  spectra;
    PortOut<DataSet<float>, Async> measurements;

    Annotated<gr::Size_t, "n_spectra", Visible, Doc<"independent spectra per reading; below 8 or above 2^20 is refused">>              n_spectra    = 32U;
    Annotated<gr::Size_t, "shape", Visible, Doc<"Gamma shape d; 0 takes it from the record's n_averaged, nonzero must agree with it">> shape        = 0U;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per completed accumulation">>                      emit_records = true;

    GR_MAKE_REFLECTABLE(SpectralKurtosis, spectra, measurements, n_spectra, shape, emit_records);

    gr::measurement::SpectralKurtosisAccumulator _acc{};
    std::vector<double>                          _skValues{};

    std::atomic<std::uint64_t> _nRecords{0ULL};
    std::atomic<std::uint64_t> _nRefusedRecords{0ULL};
    std::atomic<std::uint64_t> _nDegenerateBins{0ULL};
    std::atomic<double>        _worstBinSk{0.};
    std::atomic<std::uint64_t> _worstBin{0ULL};
    std::atomic<std::uint64_t> _filled{0ULL}; ///< spectra folded into the accumulation now in progress

    bool               _haveCycle{false}; ///< the accumulation in progress has at least one accepted spectrum
    double             _cycleShape{0.};   ///< the shape governing the accumulation in progress
    std::uint64_t      _cycleSampleStart{0ULL};
    float              _cycleSampleRate{std::numeric_limits<float>::quiet_NaN()};
    std::string        _cycleWindow{};
    std::vector<float> _cycleAxis{};
    std::string        _cycleAxisName{"Frequency"};
    std::string        _cycleAxisUnit{"Hz"};

    std::vector<DataSet<float>> _pending{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n_spectra.value < detail::kMinSpectralKurtosisSpectra || n_spectra.value > detail::kMaxSpectralKurtosisSpectra) {
            throw gr::exception(std::format("n_spectra must lie in [{}, {}], got {}", detail::kMinSpectralKurtosisSpectra, detail::kMaxSpectralKurtosisSpectra, n_spectra.value));
        }
    }

    void start() {
        _acc.resize(0UZ);
        _skValues.clear();
        _nRecords.store(0ULL, std::memory_order_relaxed);
        _nRefusedRecords.store(0ULL, std::memory_order_relaxed);
        _nDegenerateBins.store(0ULL, std::memory_order_relaxed);
        _worstBinSk.store(0., std::memory_order_relaxed);
        _worstBin.store(0ULL, std::memory_order_relaxed);
        _filled.store(0ULL, std::memory_order_relaxed);
        _haveCycle  = false;
        _cycleShape = 0.;
        _pending.clear();
    }

    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _nRecords.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedRecords() const noexcept { return _nRefusedRecords.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDegenerateBins() const noexcept { return _nDegenerateBins.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t worstBin() const noexcept { return _worstBin.load(std::memory_order_relaxed); }
    [[nodiscard]] double        worstBinSk() const noexcept { return _worstBinSk.load(std::memory_order_relaxed); }
    /// @brief Spectra folded into the accumulation now in progress, as a fraction of `n_spectra`. Any thread.
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_filled.load(std::memory_order_relaxed)) / static_cast<double>(std::max(n_spectra.value, 1U))); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            std::optional<DataSet<float>> record = fold(inSpan[consumed]);
            if (record.has_value()) {
                outSpan[made] = std::move(*record);
                ++made;
            }
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(consumed);
        return consumed == 0UZ && inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

private:
    /// @brief Fold one input record in, refusing it and naming why when it cannot be used. Returns the emitted
    /// record when this input completed the accumulation.
    [[nodiscard]] std::optional<DataSet<float>> fold(const DataSet<float>& record) {
        if (record.signal_values.empty() || record.axis_values.empty()) {
            refuse();
            return std::nullopt;
        }
        const property_map& meta = record.meta_information.empty() ? property_map{} : record.meta_information.front();

        const std::optional<double> overlap = detail::metaNumber(meta, "overlap");
        if (!overlap.has_value() || *overlap != 0.) {
            refuse();
            return std::nullopt;
        }

        const std::optional<double> averaged = detail::metaNumber(meta, "n_averaged");
        double                      effectiveShape;
        if (shape.value != 0U) {
            effectiveShape = static_cast<double>(shape.value);
            if (averaged.has_value() && *averaged != effectiveShape) {
                refuse();
                return std::nullopt;
            }
        } else {
            if (!averaged.has_value()) {
                refuse();
                return std::nullopt;
            }
            effectiveShape = *averaged;
        }

        const std::size_t bins = record.signal_values.size();
        if (_acc.bins() == 0UZ) {
            _acc.resize(bins); // the accumulation has not been configured yet: this record sets its shape
        } else if (_acc.bins() != bins) {
            // A bin count that does not match the accumulation in progress cannot be folded into it: the
            // accumulation is discarded and resized so the next matching record starts a fresh one.
            _acc.resize(bins);
            _haveCycle = false;
            _filled.store(0ULL, std::memory_order_relaxed);
            refuse();
            return std::nullopt;
        }

        if (_haveCycle && effectiveShape != _cycleShape) {
            // The producer's shape moved while an accumulation was in progress. Gamma draws of two different shapes
            // cannot share one `S1`/`S2` pair, so what has been folded is discarded and only this record is refused;
            // the next one starts a fresh cycle under whichever shape it carries.
            _acc.resize(bins);
            _haveCycle = false;
            _filled.store(0ULL, std::memory_order_relaxed);
            refuse();
            return std::nullopt;
        }

        const gr::measurement::SpectrumResponse response = _acc.accumulate(std::span<const float>(record.signal_values));
        if (response != gr::measurement::SpectrumResponse::accepted) {
            refuse();
            return std::nullopt;
        }

        if (!_haveCycle) {
            const auto sampleStart = detail::metaNumber(meta, "sample_start");
            _cycleShape            = effectiveShape;
            _cycleSampleStart      = sampleStart.has_value() ? static_cast<std::uint64_t>(*sampleStart) : 0ULL;
            _cycleSampleRate       = static_cast<float>(detail::metaNumber(meta, "sample_rate").value_or(std::numeric_limits<double>::quiet_NaN()));
            _cycleWindow           = detail::metaString(meta, "window").value_or(std::string());
            _cycleAxis             = record.axis_values.front();
            _cycleAxisName         = record.axis_names.empty() ? std::string("Frequency") : record.axis_names.front();
            _cycleAxisUnit         = record.axis_units.empty() ? std::string("Hz") : record.axis_units.front();
            _haveCycle             = true;
        }
        const std::uint64_t filled = _filled.fetch_add(1ULL, std::memory_order_relaxed) + 1ULL;

        if (filled < static_cast<std::uint64_t>(n_spectra.value)) {
            return std::nullopt;
        }
        DataSet<float> record_ = evaluate();
        if (!emit_records.value) {
            return std::nullopt;
        }
        return record_;
    }

    void refuse() { _nRefusedRecords.fetch_add(1ULL, std::memory_order_relaxed); }

    [[nodiscard]] DataSet<float> evaluate() {
        const std::size_t bins = _acc.bins();
        _skValues.assign(bins, 0.);
        const std::size_t degenerate = _acc.evaluate(_cycleShape, std::span<double>(_skValues));
        _nDegenerateBins.fetch_add(static_cast<std::uint64_t>(degenerate), std::memory_order_relaxed);

        std::size_t worst      = 0UZ;
        double      worstDelta = -1.;
        for (std::size_t k = 0UZ; k < bins; ++k) {
            const double delta = std::abs(_skValues[k] - gr::measurement::spectralKurtosisExpectation());
            if (delta > worstDelta) {
                worstDelta = delta;
                worst      = k;
            }
        }
        _worstBin.store(static_cast<std::uint64_t>(worst), std::memory_order_relaxed);
        _worstBinSk.store(bins == 0UZ ? 0. : _skValues[worst], std::memory_order_relaxed);

        const std::size_t m = _acc.count();
        DataSet<float>    ds;
        ds.extents = {static_cast<std::int32_t>(bins)};
        ds.layout  = gr::LayoutRight{};

        ds.axis_names = {_cycleAxisName};
        ds.axis_units = {_cycleAxisUnit};
        ds.axis_values.resize(1UZ);
        ds.axis_values[0UZ] = _cycleAxis;

        ds.signal_names      = {"spectral_kurtosis"};
        ds.signal_quantities = {"SpectralKurtosis"};
        ds.signal_units      = {""};
        ds.signal_values.resize(bins);
        for (std::size_t k = 0UZ; k < bins; ++k) {
            ds.signal_values[k] = static_cast<float>(_skValues[k]);
        }
        ds.signal_ranges.resize(1UZ);

        property_map extra{
            {std::pmr::string("sample_start"), pmt::Value(_cycleSampleStart)},
            {std::pmr::string("window"), pmt::Value(_cycleWindow)},
            {std::pmr::string("n_spectra"), pmt::Value(static_cast<std::uint64_t>(m))},
            {std::pmr::string("shape"), pmt::Value(_cycleShape)},
            {std::pmr::string("sk_expectation"), pmt::Value(gr::measurement::spectralKurtosisExpectation())},
            {std::pmr::string("sk_std"), pmt::Value(gr::measurement::spectralKurtosisSpread(m, _cycleShape))},
            {std::pmr::string("valid"), pmt::Value(degenerate < bins)},
        };
        if (std::isfinite(_cycleSampleRate)) {
            extra.insert_or_assign(std::pmr::string("sample_rate"), pmt::Value(_cycleSampleRate));
        }
        ds.meta_information.resize(1UZ);
        ds.meta_information[0UZ] = std::move(extra);
        ds.timing_events.resize(1UZ);
        ds.timestamp = 0;

        _acc.resize(bins); // fresh accumulation, same bin count, ready for the next cycle
        _haveCycle = false;
        _filled.store(0ULL, std::memory_order_relaxed);
        _nRecords.fetch_add(1ULL, std::memory_order_relaxed);
        return ds;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP
