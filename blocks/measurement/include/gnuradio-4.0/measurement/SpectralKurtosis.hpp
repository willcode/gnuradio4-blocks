#ifndef GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP
#define GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <expected>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
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

/// @brief The value under @p key as a `double`, read from `std::uint64_t`, `double` or `float`. `std::nullopt` when
/// the key is absent or holds another type.
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

/// @brief The flag under @p key, or `std::nullopt` when the key is absent or holds another type.
[[nodiscard]] inline std::optional<bool> metaFlag(const property_map& map, std::string_view key) {
    const auto it = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const auto* asBool = it->second.get_if<bool>()) {
        return *asBool;
    }
    return std::nullopt;
}

/// @brief The string under @p key, or `std::nullopt` when the key is absent or holds another type.
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
 * @brief Measures spectral kurtosis per bin over `n_spectra` independent density records.
 *
 * `gr::measurement::SpectralKurtosisAccumulator` sums `S1` and `S2` per bin. After `n_spectra` accepted records the
 * block evaluates the per-bin statistic and emits one record. The statistic does not depend on the scale of the
 * input records.
 *
 * The statistic requires independent spectra. The block accepts a record only when its `overlap` metadata is present
 * and exactly zero. It also requires the mean of the periodograms. A record whose `mode` metadata names another
 * accumulation, such as `max_hold`, is refused. A record without `mode` is taken as a mean.
 *
 * With `shape` at zero the shape parameter `d` is the record's `n_averaged`. A nonzero `shape` must equal the
 * record's `n_averaged` when the record states one, and the block refuses a record that differs. A record with a
 * different bin count is refused, and the block discards the accumulation and resizes it to the new count. A record
 * whose shape or `one_sided` flag differs from the accumulation in progress is refused, and the block discards the
 * accumulation. Every refused record is counted in `nRefusedRecords()`.
 *
 * A record with `one_sided` true is the spectrum of real input. Its DC bin is real, and so is its last bin when the
 * record's `fft_size` equals `2 * (bins - 1)`. The power of a real bin has shape `d/2`. The block evaluates
 * those bins at `d/2` and states their spread in `sk_std_edge`. `worstBin()` ranks each bin by its distance from 1
 * in units of its own spread.
 *
 * A bin whose `S1` is exactly zero has no defined ratio. It is written as `0.f` and counted in `nDegenerateBins()`.
 * The record's `valid` key is false when every bin is degenerate. An evaluation error goes out through
 * `emitErrorMessage`, and the work call returns `ERROR`.
 *
 * At the end of the stream the block evaluates the partial accumulation and emits its record. That record states
 * the spectra it covers in `n_spectra`, and its `valid` key is false below 8 spectra, the floor of `n_spectra`.
 * `nRecords()` does not count it.
 *
 * The block computes on the settings it last accepted. When it refuses an application, the members keep the refused
 * values, and the block keeps the `n_spectra`, `shape` and `emit_records` it accepted before.
 */
struct SpectralKurtosis : Block<SpectralKurtosis, NoTagPropagation> {
    using Description = Doc<R""(
@brief Measures spectral kurtosis per frequency bin over a run of independent density records.

The block takes `DataSet<float>` density records and emits one `spectral_kurtosis` record per `n_spectra` accepted
inputs, with one value per bin. `SK` reads 1 on noise, 0 on a noiseless tone, above 1 on a signal present in fewer
than half the spectra and below 1 on a steady one. The statistic assumes independent spectra. The block refuses a
record unless its `overlap` metadata is present and exactly zero.
)"">;

    PortIn<DataSet<float>, Async>            spectra;
    PortOut<DataSet<float>, Async, Optional> measurements;

    Annotated<gr::Size_t, "n_spectra", Visible, Doc<"spectra per reading, 8 to 2^20">>                  n_spectra    = 32U;
    Annotated<gr::Size_t, "shape", Visible, Doc<"Gamma shape d; 0 reads the record's n_averaged">>      shape        = 0U;
    Annotated<bool, "emit_records", Visible, Doc<"publish one DataSet<float> record per accumulation">> emit_records = true;

    GR_MAKE_REFLECTABLE(SpectralKurtosis, spectra, measurements, n_spectra, shape, emit_records);

    gr::measurement::SpectralKurtosisAccumulator _acc{};
    std::vector<double>                          _skValues{};

    std::atomic<std::uint64_t> _nRecords{0ULL};
    std::atomic<std::uint64_t> _nRefusedRecords{0ULL};
    std::atomic<std::uint64_t> _nDegenerateBins{0ULL};
    std::atomic<double>        _worstBinSk{0.};
    std::atomic<std::uint64_t> _worstBin{0ULL};
    std::atomic<std::uint64_t> _filled{0ULL}; ///< spectra folded into the accumulation in progress

    // The settings the block last accepted. The work thread reads these copies.
    std::atomic<gr::Size_t> _nSpectra{0U};
    gr::Size_t              _shape{0U};
    bool                    _emitRecords{false};
    bool                    _configured{false};

    bool                  _haveCycle{false};        ///< the accumulation in progress holds at least one accepted spectrum
    double                _cycleShape{0.};          ///< the shape of the accumulation in progress
    bool                  _cycleOneSided{false};    ///< the accumulation holds spectra of real input
    bool                  _cycleRealLastBin{false}; ///< the last bin of those spectra is the Nyquist bin
    std::uint64_t         _cycleSampleStart{0ULL};
    float                 _cycleSampleRate{std::numeric_limits<float>::quiet_NaN()};
    std::optional<double> _cycleCenterFrequency{};
    std::string           _cycleWindow{};
    std::vector<float>    _cycleAxis{};
    std::string           _cycleAxisName{"Frequency"};
    std::string           _cycleAxisUnit{"Hz"};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n_spectra.value < detail::kMinSpectralKurtosisSpectra || n_spectra.value > detail::kMaxSpectralKurtosisSpectra) {
            throw gr::exception(std::format("n_spectra must lie in [{}, {}], got {}", detail::kMinSpectralKurtosisSpectra, detail::kMaxSpectralKurtosisSpectra, n_spectra.value));
        }
        accept();
    }

    /// @brief Clears the accumulation and the counters. A block that has applied no settings accepts its defaults here.
    void start() {
        if (!_configured) {
            accept();
        }
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
    }

    /// @brief Accumulations of `n_spectra` spectra evaluated. Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _nRecords.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nRefusedRecords() const noexcept { return _nRefusedRecords.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint64_t nDegenerateBins() const noexcept { return _nDegenerateBins.load(std::memory_order_relaxed); }
    /// @brief The bin of the last evaluation that lies furthest from 1 in units of its own spread. Callable from any
    /// thread.
    [[nodiscard]] std::uint64_t worstBin() const noexcept { return _worstBin.load(std::memory_order_relaxed); }
    /// @brief The spectral kurtosis of that bin. Callable from any thread.
    [[nodiscard]] double worstBinSk() const noexcept { return _worstBinSk.load(std::memory_order_relaxed); }
    /// @brief Spectra folded into the accumulation in progress, as a fraction of the accepted `n_spectra`. Callable
    /// from any thread.
    [[nodiscard]] double coverage() const noexcept { return std::min(1., static_cast<double>(_filled.load(std::memory_order_relaxed)) / static_cast<double>(std::max(_nSpectra.load(std::memory_order_relaxed), gr::Size_t{1U}))); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool   wantRecords = _emitRecords && outSpan.isConnected;
        const Folded folded      = foldAll(inSpan, outSpan, wantRecords, "processBulk");
        outSpan.publish(folded.made);
        std::ignore = inSpan.consume(folded.consumed);
        if (folded.failed) {
            return work::Status::ERROR;
        }
        return folded.consumed == 0UZ && inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief Folds in the trailing records at the end of the stream and evaluates the partial accumulation.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool wantRecords = _emitRecords && outSpan.isConnected;
        Folded     folded      = foldAll(inSpan, outSpan, wantRecords, "processEpilogue");
        if (!folded.failed && _haveCycle) {
            auto record = evaluate(wantRecords);
            if (!record) {
                this->emitErrorMessage("processEpilogue", record.error());
                folded.failed = true;
            } else if (record->has_value() && folded.made < outSpan.size()) {
                outSpan[folded.made] = std::move(**record);
                ++folded.made;
            }
        }
        outSpan.publish(folded.made);
        std::ignore = inSpan.consume(folded.consumed);
        return folded.failed ? work::Status::ERROR : work::Status::OK;
    }

private:
    struct Folded {
        std::size_t made{0UZ};
        std::size_t consumed{0UZ};
        bool        failed{false};
    };

    void accept() {
        _nSpectra.store(n_spectra.value, std::memory_order_relaxed);
        _shape       = shape.value;
        _emitRecords = emit_records.value;
        _configured  = true;
    }

    /// @brief Folds the input records in while the output has room for a record. Stops at an evaluation error, which
    /// goes out through `emitErrorMessage`.
    [[nodiscard]] Folded foldAll(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, bool wantRecords, std::string_view site) {
        Folded folded;
        for (; folded.consumed < inSpan.size() && (!wantRecords || folded.made < outSpan.size()); ++folded.consumed) {
            auto record = fold(inSpan[folded.consumed], wantRecords);
            if (!record) {
                this->emitErrorMessage(site, record.error());
                folded.failed = true;
                break;
            }
            if (record->has_value()) {
                outSpan[folded.made] = std::move(**record);
                ++folded.made;
            }
        }
        return folded;
    }

    /// @brief Folds one input record in or refuses it. Returns the output record when this input completes the
    /// accumulation and @p wantRecords is set, or the evaluation's error.
    [[nodiscard]] std::expected<std::optional<DataSet<float>>, gr::Error> fold(const DataSet<float>& record, bool wantRecords) {
        if (record.signal_values.empty() || record.axis_values.empty()) {
            refuse();
            return std::nullopt;
        }
        static const property_map kNoMetadata{};
        const property_map&       meta = record.meta_information.empty() ? kNoMetadata : record.meta_information.front();

        const std::optional<double> overlap = detail::metaNumber(meta, "overlap");
        if (!overlap.has_value() || *overlap != 0.) {
            refuse();
            return std::nullopt;
        }
        if (const std::optional<std::string> mode = detail::metaString(meta, "mode"); mode.has_value() && *mode != "mean") {
            refuse();
            return std::nullopt;
        }

        const std::optional<double> averaged = detail::metaNumber(meta, "n_averaged");
        double                      effectiveShape;
        if (_shape != 0U) {
            effectiveShape = static_cast<double>(_shape);
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

        const std::size_t           bins     = record.signal_values.size();
        const bool                  oneSided = detail::metaFlag(meta, "one_sided").value_or(false);
        const std::optional<double> fftSize  = detail::metaNumber(meta, "fft_size");
        const bool                  realLast = oneSided && bins >= 2UZ && fftSize.has_value() && *fftSize == 2. * static_cast<double>(bins - 1UZ);

        if (_acc.bins() == 0UZ) {
            _acc.resize(bins); // the first record sets the bin count
        } else if (_acc.bins() != bins) {
            // A different bin count discards the accumulation and refuses this record. The next record of the new
            // count starts a new accumulation.
            _acc.resize(bins);
            _haveCycle = false;
            _filled.store(0ULL, std::memory_order_relaxed);
            refuse();
            return std::nullopt;
        }

        if (_haveCycle && (effectiveShape != _cycleShape || oneSided != _cycleOneSided || realLast != _cycleRealLastBin)) {
            // One `S1`/`S2` pair holds spectra of one shape and one sidedness. A change discards the accumulation and
            // refuses this record. The next record starts a new accumulation under its own shape.
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
            _cycleOneSided         = oneSided;
            _cycleRealLastBin      = realLast;
            _cycleSampleStart      = sampleStart.has_value() ? static_cast<std::uint64_t>(*sampleStart) : 0ULL;
            _cycleSampleRate       = static_cast<float>(detail::metaNumber(meta, "sample_rate").value_or(std::numeric_limits<double>::quiet_NaN()));
            _cycleCenterFrequency  = detail::metaNumber(meta, "center_frequency");
            _cycleWindow           = detail::metaString(meta, "window").value_or(std::string());
            _cycleAxis             = record.axis_values.front();
            _cycleAxisName         = record.axis_names.empty() ? std::string("Frequency") : record.axis_names.front();
            _cycleAxisUnit         = record.axis_units.empty() ? std::string("Hz") : record.axis_units.front();
            _haveCycle             = true;
        }
        const std::uint64_t filled = _filled.fetch_add(1ULL, std::memory_order_relaxed) + 1ULL;

        if (filled < static_cast<std::uint64_t>(_nSpectra.load(std::memory_order_relaxed))) {
            return std::nullopt;
        }
        auto evaluated = evaluate(wantRecords);
        if (evaluated) {
            _nRecords.fetch_add(1ULL, std::memory_order_relaxed);
        }
        return evaluated;
    }

    void refuse() { _nRefusedRecords.fetch_add(1ULL, std::memory_order_relaxed); }

    /// @brief True for a real bin of a one-sided spectrum: bin 0, and the last bin when it is the Nyquist bin.
    [[nodiscard]] bool realBin(std::size_t k, std::size_t bins) const noexcept { return _cycleOneSided && (k == 0UZ || (_cycleRealLastBin && k + 1UZ == bins)); }

    /// @brief Evaluates the accumulation in progress and clears it. Returns the record when @p wantRecords is set.
    [[nodiscard]] std::expected<std::optional<DataSet<float>>, gr::Error> evaluate(bool wantRecords) {
        const std::size_t bins = _acc.bins();
        const std::size_t m    = _acc.count();
        _skValues.assign(bins, 0.);
        const auto degenerate = _acc.evaluate(_cycleShape, std::span<double>(_skValues));
        if (!degenerate) {
            return std::unexpected(degenerate.error());
        }
        _nDegenerateBins.fetch_add(static_cast<std::uint64_t>(*degenerate), std::memory_order_relaxed);

        const double edgeShape = _cycleShape / 2.;
        if (_cycleOneSided && bins > 0UZ) {
            _skValues[0UZ] = gr::measurement::spectralKurtosis(_acc.sums()[0UZ], _acc.sumSquares()[0UZ], m, edgeShape);
        }
        if (_cycleRealLastBin) {
            _skValues[bins - 1UZ] = gr::measurement::spectralKurtosis(_acc.sums()[bins - 1UZ], _acc.sumSquares()[bins - 1UZ], m, edgeShape);
        }

        const double spread     = gr::measurement::spectralKurtosisSpread(m, _cycleShape);
        const double edgeSpread = gr::measurement::spectralKurtosisSpread(m, edgeShape);
        std::size_t  worst      = 0UZ;
        double       worstScore = -1.;
        for (std::size_t k = 0UZ; k < bins; ++k) {
            const double binSpread = realBin(k, bins) ? edgeSpread : spread;
            const double score     = std::abs(_skValues[k] - gr::measurement::spectralKurtosisExpectation()) / (binSpread > 0. ? binSpread : 1.);
            if (score > worstScore) {
                worstScore = score;
                worst      = k;
            }
        }
        _worstBin.store(static_cast<std::uint64_t>(worst), std::memory_order_relaxed);
        _worstBinSk.store(bins == 0UZ ? 0. : _skValues[worst], std::memory_order_relaxed);

        std::optional<DataSet<float>> record;
        if (wantRecords) {
            record = makeRecord(m, spread, edgeSpread, *degenerate < bins && m >= static_cast<std::size_t>(detail::kMinSpectralKurtosisSpectra));
        }
        _acc.resize(bins); // clears the sums for the next accumulation
        _haveCycle = false;
        _filled.store(0ULL, std::memory_order_relaxed);
        return record;
    }

    [[nodiscard]] DataSet<float> makeRecord(std::size_t m, double spread, double edgeSpread, bool valid) const {
        const std::size_t bins = _skValues.size();
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
            {std::pmr::string("one_sided"), pmt::Value(_cycleOneSided)},
            {std::pmr::string("sk_expectation"), pmt::Value(gr::measurement::spectralKurtosisExpectation())},
            {std::pmr::string("sk_std"), pmt::Value(spread)},
            {std::pmr::string("valid"), pmt::Value(valid)},
        };
        if (_cycleOneSided) {
            extra.insert_or_assign(std::pmr::string("sk_std_edge"), pmt::Value(edgeSpread));
        }
        if (std::isfinite(_cycleSampleRate)) {
            extra.insert_or_assign(std::pmr::string("sample_rate"), pmt::Value(_cycleSampleRate));
        }
        if (_cycleCenterFrequency.has_value()) {
            extra.insert_or_assign(std::pmr::string("center_frequency"), pmt::Value(*_cycleCenterFrequency));
        }
        ds.meta_information.resize(1UZ);
        ds.meta_information[0UZ] = std::move(extra);
        ds.timing_events.resize(1UZ);
        ds.timestamp = 0;
        return ds;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_SPECTRAL_KURTOSIS_HPP
