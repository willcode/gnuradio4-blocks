#ifndef GNURADIO_LTE_CELL_SCANNER_HPP
#define GNURADIO_LTE_CELL_SCANNER_HPP

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/ArbitraryResampler.hpp>
#include <gnuradio-4.0/algorithm/lte/SyncSignals.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>

#include <gnuradio-4.0/lte/CellSearch.hpp>

namespace gr::blocks::lte {

/// Which confirmed detections of a sweep become records.
enum class ScanReport : std::uint8_t { Unique = 0, All };

namespace detail {

[[nodiscard]] inline ScanReport parseScanReport(std::string_view name) {
    if (name == "unique") {
        return ScanReport::Unique;
    }
    if (name == "all") {
        return ScanReport::All;
    }
    throw gr::exception(std::format("LteCellScanner: 'report' must be 'unique' or 'all', got '{}'", name));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::lte::LteCellScanner)

/**
 * @brief Sweeps a wideband stream for E-UTRA cells and keeps a table of the distinct stations it found.
 *
 * The sweep and the table are the two things a graph dialect has no words for — a graph is topology with
 * derived numbers, with no loop to step a tuner along a raster and nowhere to grow a set — so both live here
 * and a scanning application is three blocks with nothing left over.
 *
 * Per dwell position the block translates the input down by the position, resamples it to the 1.92 MS/s the
 * identifier works at, and runs the same two kernels `CellSearch` runs, half-frame by half-frame, with the
 * integer frequency search wide enough that every raster point inside the position's own step is reachable. A
 * station is the tuple (center frequency on the raster, cell identity, duplex mode, cyclic-prefix type), and a
 * detection that names one already in the table is another sighting of it rather than a new one.
 *
 * Cells sharing a carrier arrive across dwells rather than within one. One correlation peak per root per
 * half-frame means the strongest cell on a root wins that half-frame; fading changes which one wins from dwell
 * to dwell, so a table built over many passes collects more than three identities on one center, while a cell
 * that never wins its root is never seen. That is the identifier's limit, and it is why a scan reports what it
 * saw rather than what is there.
 *
 * A station enters the table only once one dwell has named it in `min_confirmations` of its half-frames. A
 * carrier is in the band for the whole dwell and is confirmed in half-frame after half-frame of it; a false
 * confirmation is one draw of a search that draws hundreds of millions of correlation powers per pass, and it
 * does not come back. Persistence within a dwell therefore separates the two where no single threshold can,
 * and it costs nothing a scan was not already paying.
 *
 * The absolute center of a station is the input's own center plus the dwell position plus the residual offset
 * the correlator measured, rounded to the channel raster of TS 36.101 section 5.7.3. With no center known —
 * neither the setting nor a `frequency` tag — centers are reported relative to the input and every record says
 * so, because a relative number presented as an absolute one is worse than no number.
 *
 * A record's `pss_position` and `frame_start` are samples of the stream that entered this block, and its
 * `sample_rate` is that stream's: a dwell's own 1.92 MS/s stream begins afresh at every position, so an index
 * into it would name nothing anyone outside the block can see. The mapping applies both halves of the
 * resampler's geometry — the rate its fixed-point step realizes, and the prototype's own group delay.
 */
struct LteCellScanner : Block<LteCellScanner, NoTagPropagation> {
    using Description = Doc<R""(
@brief Sweeps a wideband stream over a channel raster and reports the distinct E-UTRA cells it finds.

Per dwell position: translate, resample to 1.92 MS/s, and run the identifier's own two kernels with the
frequency search on. Emits the identifier's records with the station's absolute center added, keeps a unique
station table across passes, and prints it at stop().
)"">;

    PortIn<std::complex<float>> in;
    /// One record per reported detection, the identifier's own with the scan's own keys added and its two sample
    /// indices mapped back onto the input stream, which is the rate the record states.
    PortOut<DataSet<float>, Async> out;

    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"the input rate; at least 1920000">>                                             sample_rate         = 0.f;
    Annotated<float, "center_frequency", Visible, Unit<"Hz">, Doc<"the input's absolute center; a frequency tag overrides it">>               center_frequency    = 0.f;
    Annotated<float, "span_hz", Visible, Unit<"Hz">, Doc<"band swept, centered on the input; 0 is the passband less a guard at each edge">>   span_hz             = 0.f;
    Annotated<float, "raster_hz", Visible, Unit<"Hz">, Doc<"channel raster carriers sit on">>                                                 raster_hz           = 100'000.f;
    Annotated<float, "step_hz", Visible, Unit<"Hz">, Doc<"spacing of dwell positions">>                                                       step_hz             = 400'000.f;
    Annotated<float, "frequency_search_hz", Visible, Unit<"Hz">, Doc<"half-width of the integer frequency search, per dwell">>                frequency_search_hz = 200'000.f;
    Annotated<gr::Size_t, "dwell_half_frames", Visible, Doc<"half-frames examined per position; 4 is 20 ms, both forms twice">>               dwell_half_frames   = 4U;
    Annotated<gr::Size_t, "passes", Visible, Doc<"sweeps over the input; 0 runs until the input ends">>                                       passes              = 0U;
    Annotated<float, "pss_threshold", Visible, Doc<"peak-over-mean correlation power below which no primary signal is declared">>             pss_threshold       = 20.f;
    Annotated<float, "sss_threshold", Visible, Doc<"secondary correlation quality below which a detection is dropped as unconfirmed">>        sss_threshold       = 0.5f;
    Annotated<gr::Size_t, "min_confirmations", Visible, Doc<"half-frames of one dwell a station must be confirmed in before it is reported">> min_confirmations   = 2U;
    Annotated<std::string, "report", Visible, Doc<"'unique' reports one record per new station, 'all' every confirmed detection">>            report              = std::string("unique");

    GR_MAKE_REFLECTABLE(LteCellScanner, in, out, sample_rate, center_frequency, span_hz, raster_hz, step_hz, frequency_search_hz, dwell_half_frames, passes, pss_threshold, sss_threshold, min_confirmations, report);

    std::uint64_t nSamples             = 0ULL; ///< input samples read
    std::uint64_t nPositions           = 0ULL; ///< dwell positions one pass visits
    std::uint64_t nDwells              = 0ULL; ///< dwells examined
    std::uint64_t nPrimary             = 0ULL; ///< primary detections that cleared their threshold, confirmed or not
    std::uint64_t nSssRejected         = 0ULL; ///< of those, the ones no secondary reading confirmed
    std::uint64_t nDetections          = 0ULL; ///< confirmed identifications a dwell saw often enough to report, one station counted as many times as it was seen
    std::uint64_t nBelowConfirmations  = 0ULL; ///< confirmed identifications their own dwell saw fewer than `min_confirmations` times
    std::uint64_t nStations            = 0ULL; ///< distinct stations in the table
    std::uint64_t nPasses              = 0ULL; ///< sweeps completed
    std::uint64_t nPublished           = 0ULL; ///< records emitted
    std::uint64_t nTagsDropped         = 0ULL; ///< input tags other than the rate and the center, which a record does not carry
    std::uint64_t nFrameStartOffStream = 0ULL; ///< records whose radio frame began before the input stream, published with `frame_start` at zero

    /// One entry of the table the sweep builds: what was seen at one center on one identity.
    struct Station {
        std::int64_t  centerHz{0};
        std::uint32_t cellId{0U};
        std::uint32_t nId1{0U};
        std::uint32_t nId2{0U};
        bool          tdd{false};
        bool          extended{false};
        float         bestPss{0.f};
        float         bestSss{0.f};
        std::uint64_t detections{0ULL};
        std::uint64_t firstPass{0ULL};
        std::uint64_t firstPosition{0ULL};
    };

    /// One confirmed identification of a dwell, with the station it names, held until the dwell's persistence rule
    /// has seen every half-frame of it.
    struct Sighting {
        gr::lte::CellDetection detection{};
        std::size_t            resampledIndex{0UZ};
        std::int64_t           centerHz{0};
        std::uint32_t          cellId{0U};
        bool                   tdd{false};
        bool                   extended{false};

        [[nodiscard]] bool sameStation(const Sighting& other) const noexcept { return centerHz == other.centerHz && cellId == other.cellId && tdd == other.tdd && extended == other.extended; }
    };

    /// @brief The distinct stations found so far, in the order they were first seen. Owning thread only.
    [[nodiscard]] const std::vector<Station>& stations() const noexcept { return _stations; }

    /// The sweep's own state. Public, as every block's is, so that the type stays an aggregate and the
    /// framework can initialize it from a settings map.
    ScanReport                                          _report{ScanReport::Unique};
    gr::signal::Phasor<float>                           _phasor{};
    gr::filter::ArbitraryResampler<std::complex<float>> _resampler{1., 1UZ, 1, std::array<float, 1UZ>{1.f}};
    gr::lte::CellDetector                               _detector{0.f};

    /// The settings the sweep's geometry is derived from. A change to one of them rebuilds the raster, the
    /// resampler and the dwell; a change to anything else is read where it is used and rebuilds nothing.
    struct Geometry {
        float      rate{0.f};
        float      span{0.f};
        float      step{0.f};
        float      search{0.f};
        gr::Size_t halfFrames{0U};

        [[nodiscard]] bool operator==(const Geometry&) const = default;
    };

    std::vector<double> _scanPositions{}; ///< dwell positions, relative to the input center
    std::size_t         _position{0UZ};   ///< which of them the next dwell uses
    std::uint64_t       _pass{0ULL};
    Geometry            _built{};
    double              _rate{1.};       ///< output samples per input sample, as asked for
    double              _realized{1.};   ///< and as the fixed-point step delivers it
    double              _groupDelay{0.}; ///< the resampler prototype's own delay, in input samples
    std::size_t         _warmup{0UZ};    ///< resampled samples discarded while the bank fills
    std::size_t         _dwellInput{1UZ};
    std::size_t         _dwellOutput{1UZ};
    double              _center{0.};
    bool                _centerKnown{false};
    bool                _centerFromTag{false}; ///< a tag outranks the setting and outlives a restage
    bool                _finished{false};      ///< the pass count is spent; the sweep is over whatever else arrives
    std::uint64_t       _dwellStart{0ULL};     ///< input samples consumed before the current dwell

    std::vector<std::complex<float>> _buffer{};
    std::vector<std::complex<float>> _mixed{};
    std::vector<std::complex<float>> _resampled{};
    std::vector<Sighting>            _sightings{}; ///< the current dwell's confirmations, before its persistence rule
    std::vector<Station>             _stations{};
    std::vector<DataSet<float>>      _pending{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!(raster_hz > 0.f) || !std::isfinite(raster_hz)) {
            throw gr::exception(std::format("LteCellScanner: 'raster_hz' must be a positive finite spacing, got {}", raster_hz.value));
        }
        if (!(step_hz > 0.f) || !std::isfinite(step_hz)) {
            throw gr::exception(std::format("LteCellScanner: 'step_hz' must be a positive finite spacing, got {}", step_hz.value));
        }
        if (!std::isfinite(span_hz) || span_hz < 0.f) {
            throw gr::exception(std::format("LteCellScanner: 'span_hz' must be zero or a positive finite width, got {}", span_hz.value));
        }
        if (!std::isfinite(frequency_search_hz) || frequency_search_hz < 0.f || frequency_search_hz > kMaxSearchHz) {
            throw gr::exception(std::format("LteCellScanner: 'frequency_search_hz' must lie between 0 and {} Hz, got {}", kMaxSearchHz, frequency_search_hz.value));
        }
        if (dwell_half_frames < 1U) {
            throw gr::exception("LteCellScanner: 'dwell_half_frames' must be at least one");
        }
        if (min_confirmations < 1U) {
            throw gr::exception("LteCellScanner: 'min_confirmations' must be at least one");
        }
        if (min_confirmations > dwell_half_frames) {
            throw gr::exception(std::format("LteCellScanner: 'min_confirmations' of {} cannot be met in a dwell of {} half-frames", min_confirmations.value, dwell_half_frames.value));
        }
        _report = detail::parseScanReport(report);

        // The setting is the center until a tag carries one, and no center at all is a fact the records state
        // rather than a zero a consumer could mistake for a carrier at DC.
        if (!_centerFromTag) {
            _center      = static_cast<double>(center_frequency.value);
            _centerKnown = center_frequency.value != 0.f;
        }

        const Geometry wanted{sample_rate.value, span_hz.value, step_hz.value, frequency_search_hz.value, dwell_half_frames.value};
        if (wanted != _built) {
            _built = wanted;
            rebuild();
        }
    }

    void start() {
        if (!(sample_rate >= gr::lte::kSampleRate)) {
            throw gr::exception(std::format("LteCellScanner: 'sample_rate' must be at least {} Hz, got {}", gr::lte::kSampleRate, sample_rate.value));
        }
        _built = Geometry{sample_rate.value, span_hz.value, step_hz.value, frequency_search_hz.value, dwell_half_frames.value};
        rebuild();
        reset();
    }

    void stop() {
        if (_stations.empty()) {
            std::println(stderr, "gr::blocks::lte::LteCellScanner '{}': {} dwells over {} positions, no station identified", this->name, nDwells, nPositions);
            return;
        }
        std::vector<Station> ordered = _stations;
        std::ranges::sort(ordered, [](const Station& a, const Station& b) { return a.centerHz != b.centerHz ? a.centerHz < b.centerHz : a.cellId < b.cellId; });
        std::println(stderr, "gr::blocks::lte::LteCellScanner '{}': {} stations over {} passes, {} dwells, {} positions, {} primary detections of which {} unconfirmed, {} admitted and {} short of {} confirmations{}", this->name, _stations.size(), nPasses, nDwells, nPositions, nPrimary, nSssRejected, nDetections, nBelowConfirmations, min_confirmations.value, _centerKnown ? "" : " (centers relative to the input)");
        std::println(stderr, "  {:>12}  {:>7}  {:>6} {:>4}  {:>6}  {:>9}  {:>8}  {:>8}  {:>10}  {:>5} {:>4}", "center/MHz", "cell_id", "n_id_1", "n_2", "duplex", "prefix", "pss", "sss", "detections", "pass", "pos");
        for (const Station& station : ordered) {
            std::println(stderr, "  {:12.4f}  {:7}  {:6} {:4}  {:>6}  {:>9}  {:8.1f}  {:8.4f}  {:10}  {:5} {:4}", static_cast<double>(station.centerHz) * 1e-6, station.cellId, station.nId1, station.nId2, station.tdd ? "tdd" : "fdd", station.extended ? "extended" : "normal", static_cast<double>(station.bestPss), static_cast<double>(station.bestSss), station.detections, station.firstPass, station.firstPosition);
        }
        if (nFrameStartOffStream > 0ULL) {
            std::println(stderr, "  records whose frame start precedes the input, published at zero: {}", nFrameStartOffStream);
        }
        if (nTagsDropped > 0ULL) {
            std::println(stderr, "  tags dropped: {}", nTagsDropped);
        }
    }

    /// @brief Forget every buffered sample, every counter and the whole station table.
    void reset() {
        _buffer.clear();
        _stations.clear();
        _sightings.clear();
        _position            = 0UZ;
        _pass                = 0ULL;
        _finished            = false;
        _dwellStart          = 0ULL;
        nSamples             = 0ULL;
        nDwells              = 0ULL;
        nPrimary             = 0ULL;
        nSssRejected         = 0ULL;
        nDetections          = 0ULL;
        nBelowConfirmations  = 0ULL;
        nStations            = 0ULL;
        nPasses              = 0ULL;
        nPublished           = 0ULL;
        nTagsDropped         = 0ULL;
        nFrameStartOffStream = 0ULL;
        _pending.clear();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        countTags(inSpan);

        const std::span<const std::complex<float>> input = std::span<const std::complex<float>>(inSpan);
        _buffer.insert(_buffer.end(), input.begin(), input.end());
        nSamples += input.size();

        while (_buffer.size() >= _dwellInput && !_finished) {
            dwell();
            _buffer.erase(_buffer.begin(), _buffer.begin() + static_cast<std::ptrdiff_t>(_dwellInput));
            _dwellStart += _dwellInput;
            ++_position;
            if (_position >= _scanPositions.size()) {
                _position = 0UZ;
                ++_pass;
                ++nPasses;
                _finished = passes != 0U && _pass >= static_cast<std::uint64_t>(passes.value);
            }
        }

        std::size_t made = 0UZ;
        if (outSpan.isConnected) {
            const std::size_t take = std::min(_pending.size(), outSpan.size());
            for (; made < take; ++made) {
                outSpan[made] = std::move(_pending[made]);
            }
            _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        } else {
            _pending.clear(); // an unconnected port is not a backlog
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(input.size());
        return _finished && _pending.empty() ? work::Status::DONE : work::Status::OK;
    }

private:
    /// The identifier's own bound on a frequency search: beyond it the cost is a receiver's, not a scan's.
    static constexpr float kMaxSearchHz = 200'000.f;
    /// Guard at each edge of the passband: a primary signal is 0.93 MHz wide and must fit inside the swept band.
    static constexpr float kEdgeGuardHz = 960'000.f;

    void rebuild() {
        _scanPositions.clear();
        if (!(sample_rate >= gr::lte::kSampleRate)) {
            return; // an unconfigured rate has no geometry; start() is where that is refused
        }

        const double span = span_hz > 0.f ? static_cast<double>(span_hz.value) : std::max(0., static_cast<double>(sample_rate.value) - 2. * static_cast<double>(kEdgeGuardHz));
        const auto   half = static_cast<std::ptrdiff_t>(std::floor(span / 2. / static_cast<double>(step_hz.value)));
        for (std::ptrdiff_t i = -half; i <= half; ++i) {
            _scanPositions.push_back(static_cast<double>(i) * static_cast<double>(step_hz.value));
        }
        nPositions = _scanPositions.size();

        // The resampler's own prototype is the anti-alias low-pass: a second filter ahead of it would band-limit
        // what the bank already band-limits and cost a multiply per input sample to do it.
        _rate                                    = static_cast<double>(gr::lte::kSampleRate) / static_cast<double>(sample_rate.value);
        const std::size_t                 bank   = gr::filter::arbitraryBankSize(kStopbandDb, kRolloff, kInterpolationOrder);
        const gr::filter::ResamplerDesign design = gr::filter::designArbitraryResampler(bank, _rate, kRolloff, kStopbandDb);
        _resampler                               = gr::filter::ArbitraryResampler<std::complex<float>>(_rate, bank, kInterpolationOrder, design.taps);
        _realized                                = _resampler.realizedRate();
        _groupDelay                              = _resampler.groupDelaySamples();

        // Each dwell resamples from silence, so the outputs whose filter window still reaches back into that silence
        // are transient and discarded. The window is `tapsPerArm` input samples long, which is that many times the
        // rate in outputs — an input count and an output count are not interchangeable, and the dwell needs both.
        _warmup      = static_cast<std::size_t>(std::ceil(static_cast<double>(_resampler.tapsPerArm()) * _realized)) + 8UZ;
        _dwellOutput = _warmup + gr::lte::CellDetector::windowFor(static_cast<std::size_t>(dwell_half_frames.value) * gr::lte::kHalfFrameSamples);
        _resampler.reset();
        _dwellInput = _resampler.inputsFor(_dwellOutput);

        _detector = gr::lte::CellDetector(frequency_search_hz);
        _mixed.assign(_dwellInput, std::complex<float>(0.f, 0.f));
        _resampled.clear();
    }

    void countTags(const auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            bool other = false;
            for (const auto& [key, value] : tag.map) {
                const std::string_view tagKey(key.data(), key.size());
                if (tagKey == gr::tag::FREQUENCY.shortKey()) {
                    // The capture's own center, which is what turns a baseband offset into an absolute carrier
                    _center        = value.value_or(static_cast<double>(center_frequency.value));
                    _centerKnown   = true;
                    _centerFromTag = true;
                } else if (tagKey != gr::tag::SAMPLE_RATE.shortKey()) {
                    other = true;
                }
            }
            if (other) {
                ++nTagsDropped;
            }
        }
    }

    void dwell() {
        const double position = _scanPositions[_position];

        _phasor.configure(-2. * std::numbers::pi * position / static_cast<double>(sample_rate.value), 0.);
        const std::span<const std::complex<float>> raw(_buffer.data(), _dwellInput);
        _phasor.mix(raw, std::span<std::complex<float>>(_mixed));

        _resampler.reset();
        _resampled.assign(_resampler.outputsFor(_dwellInput), std::complex<float>(0.f, 0.f));
        std::ignore = _resampler.process(std::span<const std::complex<float>>(_mixed), std::span<std::complex<float>>(_resampled));
        ++nDwells;
        if (_resampled.size() < _dwellOutput) {
            return; // a dwell the resampler could not fill is not a dwell that saw anything
        }

        _sightings.clear();
        const std::size_t halfFrames = static_cast<std::size_t>(dwell_half_frames.value);
        for (std::size_t h = 0UZ; h < halfFrames; ++h) {
            const std::size_t                          at = _warmup + h * gr::lte::kHalfFrameSamples;
            const std::span<const std::complex<float>> window(_resampled.data() + at, gr::lte::CellDetector::windowFor(gr::lte::kHalfFrameSamples));

            gr::lte::ExamineCounts                        counts;
            const std::span<const gr::lte::CellDetection> found = _detector.examine(window, gr::lte::kHalfFrameSamples, pss_threshold, sss_threshold, counts);
            nPrimary += counts.primaryFound;
            nSssRejected += counts.secondaryRejected;
            for (const gr::lte::CellDetection& detection : found) {
                _sightings.push_back(sightingOf(detection, position, at + gr::lte::kMaxSecondaryLookBehind + detection.primary.position));
            }
        }
        admit(position);
    }

    /// @brief What one confirmed identification says about the station it names, before the dwell has decided.
    [[nodiscard]] Sighting sightingOf(const gr::lte::CellDetection& detection, double position, std::size_t resampledIndex) const {
        const double absolute = _center + position + static_cast<double>(detection.primary.frequencyHz);
        const auto   centerHz = static_cast<std::int64_t>(std::llround(absolute / static_cast<double>(raster_hz.value))) * static_cast<std::int64_t>(std::llround(static_cast<double>(raster_hz.value)));
        return Sighting{detection, resampledIndex, centerHz, gr::lte::cellIdentity(detection.secondary.nId1, detection.primary.nId2), detection.secondary.geometry.duplex == gr::lte::DuplexMode::Tdd, detection.secondary.geometry.cyclicPrefix == gr::lte::CyclicPrefix::Extended};
    }

    /// The dwell's own persistence rule. A carrier is in the band for the whole dwell and is confirmed in half-frame
    /// after half-frame of it; a false confirmation is one draw of a very large search and does not come back, so
    /// how many of a dwell's half-frames named a station separates the two where no single threshold can.
    void admit(double position) {
        const auto minimum = static_cast<std::size_t>(min_confirmations.value);
        for (const Sighting& sighting : _sightings) {
            const auto seen = static_cast<std::size_t>(std::ranges::count_if(_sightings, [&sighting](const Sighting& other) { return other.sameStation(sighting); }));
            if (seen < minimum) {
                ++nBelowConfirmations;
                continue;
            }
            ++nDetections;
            record(sighting, position);
        }
    }

    /// Turn one admitted identification into a station sighting and, where the reporting rule wants it, a record.
    void record(const Sighting& sighting, double position) {
        const gr::lte::CellDetection& detection      = sighting.detection;
        const std::uint32_t           nId1           = detection.secondary.nId1;
        const std::uint32_t           nId2           = detection.primary.nId2;
        const std::uint32_t           identity       = sighting.cellId;
        const bool                    tdd            = sighting.tdd;
        const bool                    extended       = sighting.extended;
        const std::int64_t            centerHz       = sighting.centerHz;
        const std::size_t             resampledIndex = sighting.resampledIndex;

        const auto known = std::ranges::find_if(_stations, [centerHz, identity, tdd, extended](const Station& entry) { return entry.centerHz == centerHz && entry.cellId == identity && entry.tdd == tdd && entry.extended == extended; });

        const bool first = known == _stations.end();
        if (first) {
            _stations.push_back(Station{centerHz, identity, nId1, nId2, tdd, extended, detection.primary.metric, detection.secondary.metric, 1ULL, _pass, static_cast<std::uint64_t>(_position)});
            nStations = _stations.size();
        } else {
            known->bestPss = std::max(known->bestPss, detection.primary.metric);
            known->bestSss = std::max(known->bestSss, detection.secondary.metric);
            ++known->detections;
        }
        if (_report == ScanReport::Unique && !first) {
            return;
        }

        // The resampled index names an input sample through the resampler's own geometry, so a consumer can place
        // the detection on the capture it came from rather than inside a dwell it cannot see: output `k` is anchored
        // at input `k / rate`, and the prototype delays what it carries by its own group delay, so the sample the
        // detection sits on is that many earlier. Both indices are input samples, which is the rate the record
        // states; a position at 1.92 MS/s would name a dwell that nothing outside this block can see.
        const double       atDwell = static_cast<double>(resampledIndex) / _realized - _groupDelay;
        const auto         onInput = static_cast<std::int64_t>(_dwellStart) + std::llround(atDwell);
        const std::int64_t frameAt = onInput + std::llround(static_cast<double>(detection.secondary.geometry.frameStartOffset(detection.secondary.halfFrame)) / _realized);
        if (frameAt < 0 || onInput < 0) {
            // The radio frame began before the input stream did, which the first dwell of a run sees whenever a
            // detection came from the second half-frame: there is no non-negative index to name that sample by, so
            // the record carries zero and this counter is what says the number is a floor rather than a position.
            // The station itself is kept, because a station is what a scan is for and a later dwell places it.
            ++nFrameStartOffStream;
        }

        DataSet<float> ds;
        ds.signal_values.assign(detection.secondary.soft.begin(), detection.secondary.soft.end());
        ds.extents.push_back(static_cast<std::int32_t>(detection.secondary.soft.size()));
        ds.signal_names.emplace_back("sss");
        ds.signal_quantities.emplace_back("");
        ds.signal_units.emplace_back("");
        ds.meta_information.emplace_back();
        ds.timing_events.emplace_back();
        ds.timestamp = 0;

        property_map& meta = ds.meta_information[0UZ];
        meta.insert_or_assign(property_map::key_type("cell_id"), pmt::Value(static_cast<gr::Size_t>(identity)));
        meta.insert_or_assign(property_map::key_type("n_id_1"), pmt::Value(static_cast<gr::Size_t>(nId1)));
        meta.insert_or_assign(property_map::key_type("n_id_2"), pmt::Value(static_cast<gr::Size_t>(nId2)));
        meta.insert_or_assign(property_map::key_type("duplex"), pmt::Value(std::string(tdd ? "tdd" : "fdd")));
        meta.insert_or_assign(property_map::key_type("cyclic_prefix"), pmt::Value(std::string(extended ? "extended" : "normal")));
        meta.insert_or_assign(property_map::key_type("half_frame"), pmt::Value(static_cast<gr::Size_t>(detection.secondary.halfFrame)));
        meta.insert_or_assign(property_map::key_type("frame_start"), pmt::Value(static_cast<std::uint64_t>(std::max<std::int64_t>(0, frameAt))));
        meta.insert_or_assign(property_map::key_type("pss_position"), pmt::Value(static_cast<std::uint64_t>(std::max<std::int64_t>(0, onInput))));
        meta.insert_or_assign(property_map::key_type("frequency_offset_hz"), pmt::Value(detection.primary.frequencyHz));
        meta.insert_or_assign(property_map::key_type("pss_metric"), pmt::Value(detection.primary.metric));
        meta.insert_or_assign(property_map::key_type("sss_metric"), pmt::Value(detection.secondary.metric));
        meta.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(sample_rate.value));
        meta.insert_or_assign(property_map::key_type("sequence"), pmt::Value(nPublished));
        meta.insert_or_assign(property_map::key_type("center_frequency_hz"), pmt::Value(static_cast<double>(centerHz)));
        meta.insert_or_assign(property_map::key_type("scan_position_hz"), pmt::Value(position));
        meta.insert_or_assign(property_map::key_type("first_seen"), pmt::Value(first));
        meta.insert_or_assign(property_map::key_type("center_is_relative"), pmt::Value(!_centerKnown));
        meta.insert_or_assign(property_map::key_type("pass"), pmt::Value(_pass));

        _pending.push_back(std::move(ds));
        ++nPublished;
    }

    /// Stopband the resampler's prototype is designed for, and the fraction of the output band it keeps flat.
    static constexpr double kStopbandDb = 60.;
    static constexpr double kRolloff    = 0.2;
    /// Linear interpolation between the bank's arms: the even orders put their nodes off-center and the higher
    /// ones cost more than the arms they save.
    static constexpr int kInterpolationOrder = 1;
};

} // namespace gr::blocks::lte

#endif // GNURADIO_LTE_CELL_SCANNER_HPP
