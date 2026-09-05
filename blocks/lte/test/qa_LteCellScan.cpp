#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/filter/ArbitraryResampler.hpp>
#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>
#include <gnuradio-4.0/algorithm/signal/Phasor.hpp>
#include <gnuradio-4.0/fileio/SigMfIo.hpp>
#include <gnuradio-4.0/lte/CellScanner.hpp>

#include "LteDownlinkScene.hpp"

using namespace boost::ut;
using gr::blocks::lte::LteCellScanner;
using gr::test::lte::makeDownlink;
using gr::test::lte::Scene;
using gr::test::lte::SceneConfig;

namespace {

using Complex = std::complex<float>;

/// The wideband rate the scan is written against, and the capture's own.
constexpr double      kWideRate    = 25'000'000.;
constexpr double      kWideCenter  = 757'000'000.;
constexpr const char* kCaptureName = "20260730_182327_757000000_25000000_fc.sigmf-meta";
/// A wideband rate that is a power-of-two multiple of the identifier's own, so that a primary symbol at it is an
/// exact 1024-point transform of the same 62 subcarriers and a test can say where one is in a span without asking
/// the resampler under test where it put it.
constexpr std::size_t kExactMultiple = 8UZ;
constexpr double      kExactRate     = static_cast<double>(kExactMultiple) * 1'920'000.;
constexpr std::size_t kExactSymbol   = kExactMultiple * gr::lte::kSymbolSamples;
/// Sweeps of the capture the gate takes. The whole file is about 27 of them at half a minute a sweep; the gate
/// takes one, which already visits every position, and the full run is a measurement rather than a gate.
constexpr std::uint32_t kCapturePasses = 1U;
/// Radio frames per cell of the synthetic span. A dwell consumes about 258 000 input samples, so the span has to be
/// long enough for the sweep to reach the highest carrier on it and come round again.
constexpr std::size_t kSpanFrames = 40UZ;

/// A source that hands the graph a fixed stream and finishes.
struct VectorSource : gr::Block<VectorSource> {
    gr::PortOut<Complex> out;
    GR_MAKE_REFLECTABLE(VectorSource, out);
    std::vector<Complex> _data{};
    std::size_t          _at{0UZ};

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _at);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_at), n, outSpan.begin());
        outSpan.publish(n);
        _at += n;
        return _at == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<float>> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

[[nodiscard]] double metaDouble(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = record.meta_information.empty() ? gr::property_map{} : record.meta_information.front();
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<double>::quiet_NaN() : entry->second.value_or(std::numeric_limits<double>::quiet_NaN());
}

[[nodiscard]] std::uint64_t metaU64(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = record.meta_information.empty() ? gr::property_map{} : record.meta_information.front();
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

[[nodiscard]] float metaFloat(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = record.meta_information.empty() ? gr::property_map{} : record.meta_information.front();
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<float>::quiet_NaN() : entry->second.value_or(std::numeric_limits<float>::quiet_NaN());
}

[[nodiscard]] bool metaBool(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = record.meta_information.empty() ? gr::property_map{} : record.meta_information.front();
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

/**
 * @brief One cell's 1.92 MS/s scene lifted to the wideband rate and placed at @p offsetHz from center.
 *
 * The same arbitrary resampler the scanner brings the band back down with, run the other way: a rate above one
 * interpolates, and its own prototype is the reconstruction filter. The result is added into @p span, which is what
 * one antenna sees when two carriers share a band.
 */
void placeCell(std::vector<Complex>& span, const SceneConfig& config, double offsetHz, float amplitude, double wideRate = kWideRate, std::size_t limit = ~0UZ) {
    const Scene scene = makeDownlink(config);

    const double                            rate   = wideRate / static_cast<double>(gr::lte::kSampleRate);
    const std::size_t                       bank   = gr::filter::arbitraryBankSize(60., 0.2, 1);
    const gr::filter::ResamplerDesign       design = gr::filter::designArbitraryResampler(bank, 1., 0.2, 60.);
    gr::filter::ArbitraryResampler<Complex> up(rate, bank, 1, design.taps);

    std::vector<Complex> wide(up.outputsFor(scene.samples.size()));
    std::ignore = up.process(std::span<const Complex>(scene.samples), std::span<Complex>(wide));
    if (limit < wide.size()) {
        wide.resize(limit); // a cell that is only on the air for part of the span
    }

    gr::signal::Phasor<float> phasor;
    phasor.configure(2. * std::numbers::pi * offsetHz / wideRate, 0.);
    std::vector<Complex> shifted(wide.size());
    phasor.mix(std::span<const Complex>(wide), std::span<Complex>(shifted));

    if (span.size() < shifted.size()) {
        span.resize(shifted.size(), Complex(0.f, 0.f));
    }
    for (std::size_t n = 0UZ; n < shifted.size(); ++n) {
        span[n] += shifted[n] * amplitude;
    }
}

/**
 * @brief Where the primary symbol of @p nId2 sits in @p span, in samples of @p span, found rather than derived.
 *
 * At a rate that is a whole multiple of 1.92 MS/s the primary symbol is the same 62 subcarriers through a
 * proportionally longer transform, so the reference is built from the standard's own sequence and owes nothing to
 * the resampler that put the cell on the span. The correlation peak over @p search positions is then the symbol's
 * first useful sample, which is what a record's `pss_position` names.
 */
[[nodiscard]] std::size_t findPrimary(const std::vector<Complex>& span, std::uint32_t nId2, std::size_t search) {
    const std::array<std::complex<double>, gr::lte::kSignalLength> sequence = gr::lte::pssSequence(nId2);

    std::vector<Complex> bins(kExactSymbol, Complex(0.f, 0.f));
    for (std::size_t n = 0UZ; n < gr::lte::kSignalLength; ++n) {
        const std::ptrdiff_t subcarrier = static_cast<std::ptrdiff_t>(n) - (n < 31UZ ? 31 : 30);
        const auto           bin        = static_cast<std::size_t>((subcarrier + static_cast<std::ptrdiff_t>(kExactSymbol)) % static_cast<std::ptrdiff_t>(kExactSymbol));
        bins[bin]                       = Complex(static_cast<float>(sequence[n].real()), static_cast<float>(sequence[n].imag()));
    }
    std::vector<Complex>                                                                             reference(kExactSymbol);
    gr::algorithm::FFT<std::complex<float>, std::complex<float>, gr::algorithm::Direction::Backward> inverse;
    inverse.compute(bins, reference);

    double      best  = -1.;
    std::size_t where = 0UZ;
    for (std::size_t at = 0UZ; at + kExactSymbol <= std::min(span.size(), search + kExactSymbol); ++at) {
        std::complex<double> sum{};
        for (std::size_t n = 0UZ; n < kExactSymbol; ++n) {
            sum += std::complex<double>(span[at + n]) * std::conj(std::complex<double>(reference[n]));
        }
        const double power = std::norm(sum);
        if (power > best) {
            best  = power;
            where = at;
        }
    }
    return where;
}

/// Add complex Gaussian noise of the stated power to @p span.
void addNoise(std::vector<Complex>& span, double power, std::uint64_t seed) {
    gr::rng::Xoshiro256pp         rng(seed);
    gr::rng::GaussianNoise<float> draw(rng);
    const auto                    amplitude = static_cast<float>(std::sqrt(power));
    for (Complex& sample : span) {
        sample += draw.complexSample() * amplitude;
    }
}

/// Everything one scanner run reported.
struct ScanRun {
    std::vector<LteCellScanner::Station> stations{};
    std::vector<gr::DataSet<float>>      records{};
    std::uint64_t                        positions{0ULL};
    std::uint64_t                        dwells{0ULL};
    std::uint64_t                        primary{0ULL};
    std::uint64_t                        rejected{0ULL};
    std::uint64_t                        detections{0ULL};
    std::uint64_t                        belowConfirmations{0ULL};
    std::uint64_t                        passes{0ULL};
    double                               seconds{0.};
};

[[nodiscard]] ScanRun scan(std::vector<Complex> samples, gr::property_map settings) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<VectorSource>();
    source._data     = std::move(samples);
    auto& scanner    = flow.emplaceBlock<LteCellScanner>(std::move(settings));
    auto& sink       = flow.emplaceBlock<RecordSink>();
    expect(flow.connect<"out", "in">(source, scanner).has_value());
    expect(flow.connect<"out", "in">(scanner, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    expect(scheduler.exchange(std::move(flow)).has_value());
    const auto started = std::chrono::steady_clock::now();
    expect(scheduler.runAndWait().has_value());
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

    return ScanRun{scanner.stations(), sink._records, scanner.nPositions, scanner.nDwells, scanner.nPrimary, scanner.nSssRejected, scanner.nDetections, scanner.nBelowConfirmations, scanner.nPasses, seconds};
}

/// The directory the recording legs read, or empty when the build was configured without one.
[[nodiscard]] std::string recordingsDirectory() {
    const char* fromEnvironment = std::getenv("GR4_RECORDINGS_DIR");
    return fromEnvironment == nullptr ? std::string{} : std::string(fromEnvironment);
}

[[nodiscard]] std::string readGraph() {
    std::ifstream      file(std::format("{}/lte_scan.yaml", EXAMPLE_GRAPHS_PATH), std::ios::binary);
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

/// Rewrite the graph's one editable line, which is how the same file serves a capture and a synthetic stand-in.
[[nodiscard]] std::string withCapture(std::string graph, const std::string& path, std::uint32_t passes) {
    const std::size_t at = graph.find("file_name:");
    boost::ut::expect(at != std::string::npos) << "the graph names a capture";
    const std::size_t end = graph.find('\n', at);
    graph.replace(at, end - at, std::format("file_name: {}", path));

    const std::size_t bounded = graph.find("passes: !!uint32 ");
    if (bounded != std::string::npos) {
        const std::size_t lineEnd = graph.find('\n', bounded);
        graph.replace(bounded, lineEnd - bounded, std::format("passes: !!uint32 {}", passes));
    }
    return graph;
}

} // namespace

const boost::ut::suite<"LteCellScan"> _lteCellScan = [] {
    "a record names the input sample the detection sits on"_test = [] {
        // One cell at the span's own center, at a rate the scan resamples by exactly eight, so that where the
        // primary symbol is can be found in the span itself and compared with what the record says about it.
        std::vector<Complex> span;
        SceneConfig          config;
        config.nId1   = 45U;
        config.nId2   = 0U;
        config.frames = 4UZ;
        config.seed   = 0x91ULL;
        placeCell(span, config, 0., 1.f, kExactRate);
        addNoise(span, 0.01, 0x92ULL);

        const std::size_t truth = findPrimary(span, config.nId2, 16'000UZ);

        const ScanRun run = scan(span, {{"sample_rate", static_cast<float>(kExactRate)}, {"center_frequency", static_cast<float>(kWideCenter)}, {"span_hz", 100'000.f}, {"frequency_search_hz", 0.f}, {"dwell_half_frames", gr::Size_t(4)}, {"passes", gr::Size_t(1)}, {"report", std::string("all")}});
        expect(eq(run.positions, 1ULL)) << "a span narrower than one step is a single dwell position";
        expect(eq(run.dwells, 1ULL));
        expect(!run.records.empty()) << "the cell at the center is identified";
        if (run.records.empty()) {
            return;
        }

        const auto         reported = static_cast<std::int64_t>(metaU64(run.records.front(), "pss_position"));
        const std::int64_t error    = reported - static_cast<std::int64_t>(truth);
        std::println("criterion 16: the first primary symbol sits at input sample {}, the record names {}, error {} samples of {:.0f} S/s", truth, reported, error, kExactRate);
        // One resampled sample is eight input samples, which is the mapping's own resolution; the tolerance is two
        // of them. A warm-up counted as an output count and a dropped group delay together cost about 2200 here.
        expect(std::abs(error) <= 16) << std::format("the record's primary position is the input sample it sits on, off by {}", error);

        const double       rate   = static_cast<double>(kExactRate) / static_cast<double>(gr::lte::kSampleRate);
        const auto         frame  = static_cast<std::int64_t>(metaU64(run.records.front(), "frame_start"));
        const std::int64_t offset = static_cast<std::int64_t>(std::llround(static_cast<double>(gr::lte::kStructures[0UZ].frameStartOffset(0U)) * rate));
        expect(std::abs(frame - (reported + offset)) <= 1) << "and the frame start is the structure's own distance from it, at the input's rate";
        expect(eq(metaFloat(run.records.front(), "sample_rate"), static_cast<float>(kExactRate))) << "both indices are input samples, and the record says which rate that is";
    };

    "a dwell admits the station it saw again and refuses the one it saw once"_test = [] {
        // Two cells inside one dwell's own passband, 200 kHz apart so neither is the other's interference: one on
        // the air for the whole dwell, one for its first half-frame only. Nothing separates them but how many of
        // the dwell's half-frames named them.
        std::vector<Complex> span;
        SceneConfig          persistent;
        persistent.nId1   = 12U;
        persistent.nId2   = 0U;
        persistent.frames = 4UZ;
        persistent.seed   = 0x93ULL;
        placeCell(span, persistent, 0., 1.f, kExactRate);

        SceneConfig fleeting;
        fleeting.nId1   = 77U;
        fleeting.nId2   = 1U;
        fleeting.frames = 4UZ;
        fleeting.seed   = 0x94ULL;
        // Five milliseconds of it, which reaches the first of the dwell's four half-frames and none of the rest.
        placeCell(span, fleeting, 200'000., 1.f, kExactRate, static_cast<std::size_t>(kExactRate * 0.005));
        addNoise(span, 0.01, 0x95ULL);

        const gr::property_map common{{"sample_rate", static_cast<float>(kExactRate)}, {"center_frequency", static_cast<float>(kWideCenter)}, {"span_hz", 100'000.f}, {"frequency_search_hz", 200'000.f}, {"dwell_half_frames", gr::Size_t(4)}, {"passes", gr::Size_t(1)}, {"report", std::string("all")}};
        const auto             named = [](const ScanRun& run, std::uint32_t nId1) { return std::ranges::any_of(run.stations, [nId1](const LteCellScanner::Station& s) { return s.nId1 == nId1; }); };

        gr::property_map once = common;
        once.insert_or_assign(gr::property_map::key_type("min_confirmations"), gr::pmt::Value(gr::Size_t(1)));
        const ScanRun single = scan(span, once);
        std::println("criterion 16: min_confirmations 1 admits {} detections over {} stations, {} short of the gate", single.detections, single.stations.size(), single.belowConfirmations);
        expect(named(single, 12U)) << "a single sighting is enough for the persistent cell";
        expect(named(single, 77U)) << "and for the fleeting one";
        expect(eq(single.belowConfirmations, 0ULL)) << "nothing is short of one confirmation";

        gr::property_map twice = common;
        twice.insert_or_assign(gr::property_map::key_type("min_confirmations"), gr::pmt::Value(gr::Size_t(2)));
        const ScanRun paired = scan(span, twice);
        std::println("criterion 16: min_confirmations 2 admits {} detections over {} stations, {} short of the gate", paired.detections, paired.stations.size(), paired.belowConfirmations);
        expect(named(paired, 12U)) << "the cell the dwell saw in half-frame after half-frame is kept";
        expect(!named(paired, 77U)) << "the one it saw once is refused";
        expect(eq(paired.belowConfirmations, single.detections - paired.detections)) << "and every refusal is counted";
        expect(paired.belowConfirmations > 0ULL) << "the gate acted";

        expect(throws([&common] {
            gr::property_map beyond = common;
            beyond.insert_or_assign(gr::property_map::key_type("min_confirmations"), gr::pmt::Value(gr::Size_t(5)));
            LteCellScanner block(std::move(beyond));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        })) << "a gate a dwell cannot meet is refused at staging, by name";
    };

    "a synthetic span is swept and every cell on it named"_test = [] {
        // Three carriers across the span, two of them sharing the +2 MHz center with different groups, and a fourth
        // 6 dB below one of those two on the same root: the pair that shares a root is what F4's limit acts on.
        std::vector<Complex> span;
        SceneConfig          low;
        low.nId1   = 30U;
        low.nId2   = 0U;
        low.frames = kSpanFrames;
        low.seed   = 0x11ULL;
        placeCell(span, low, -8'000'000., 1.f);

        SceneConfig middle;
        middle.nId1   = 100U;
        middle.nId2   = 1U;
        middle.duplex = gr::lte::DuplexMode::Tdd;
        middle.frames = kSpanFrames;
        middle.seed   = 0x22ULL;
        placeCell(span, middle, 2'000'000., 1.f);

        SceneConfig beside;
        beside.nId1         = 7U;
        beside.nId2         = 2U;
        beside.cyclicPrefix = gr::lte::CyclicPrefix::Extended;
        beside.frames       = kSpanFrames;
        beside.seed         = 0x33ULL;
        placeCell(span, beside, 2'000'000., 1.f);

        SceneConfig shadowed;
        shadowed.nId1         = 160U;
        shadowed.nId2         = 2U; // the same root as `beside`, and 6 dB below it
        shadowed.duplex       = gr::lte::DuplexMode::Tdd;
        shadowed.cyclicPrefix = gr::lte::CyclicPrefix::Extended;
        shadowed.frames       = kSpanFrames;
        shadowed.seed         = 0x44ULL;
        placeCell(span, shadowed, 2'000'000., std::pow(10.f, -6.f / 20.f));

        SceneConfig high;
        high.nId1         = 167U;
        high.nId2         = 0U;
        high.cyclicPrefix = gr::lte::CyclicPrefix::Extended;
        high.frames       = kSpanFrames;
        high.seed         = 0x55ULL;
        placeCell(span, high, 9'000'000., 1.f);

        addNoise(span, 0.1, 0x66ULL);

        // A megahertz step over a 19 MHz span puts every carrier on a position of its own in 19 dwells, and the
        // scene is long enough for the sweep to come round again; the frequency search, the thresholds and the
        // reporting rule are the application's own defaults. The persistence gate is set to one sighting, because
        // what this case measures is the sweep, the raster and the first-sighting mark rather than that gate, which
        // has an arm of its own; a dwell of two half-frames would otherwise decide the answer.
        const ScanRun run = scan(span, {{"sample_rate", static_cast<float>(kWideRate)}, {"center_frequency", static_cast<float>(kWideCenter)}, {"span_hz", 19'000'000.f}, {"step_hz", 1'000'000.f}, {"dwell_half_frames", gr::Size_t(2)}, {"min_confirmations", gr::Size_t(1)}, {"report", std::string("all")}});

        std::println("criterion 17: {} positions, {} dwells, {} passes, {} primary detections of which {} unconfirmed, {} confirmed, {} stations", run.positions, run.dwells, run.passes, run.primary, run.rejected, run.detections, run.stations.size());
        for (const LteCellScanner::Station& station : run.stations) {
            std::println("  {:.4f} MHz cell {:3} ({:3},{}) {} {} pss {:.1f} sss {:.3f} seen {}", static_cast<double>(station.centerHz) * 1e-6, station.cellId, station.nId1, station.nId2, station.tdd ? "tdd" : "fdd", station.extended ? "extended" : "normal", static_cast<double>(station.bestPss), static_cast<double>(station.bestSss), station.detections);
        }

        const auto found = [&run](double centerMHz, std::uint32_t nId1, std::uint32_t nId2, bool tdd, bool extended) {
            const auto at = static_cast<std::int64_t>(std::llround(centerMHz * 1e6));
            return std::ranges::any_of(run.stations, [at, nId1, nId2, tdd, extended](const LteCellScanner::Station& s) { return s.centerHz == at && s.nId1 == nId1 && s.nId2 == nId2 && s.tdd == tdd && s.extended == extended; });
        };
        expect(found(749.0, 30U, 0U, false, false)) << "the cell 8 MHz below center";
        expect(found(759.0, 100U, 1U, true, false)) << "the unpaired cell 2 MHz above center";
        // The two carriers the criterion puts on one center are joined there by its own fourth cell, so that
        // center carries three at once and what a root's single peak resolves to is measured, not assumed.
        std::println("criterion 17: the extended-prefix cell sharing the +2 MHz center was {}", found(759.0, 7U, 2U, false, true) ? "identified" : "never identified");
        expect(found(766.0, 167U, 0U, false, true)) << "the cell 9 MHz above center";

        for (const LteCellScanner::Station& station : run.stations) {
            expect(eq(station.centerHz % 100'000, std::int64_t{0})) << std::format("station at {} sits on the 100 kHz raster", station.centerHz);
        }

        // `first_seen` marks exactly one record per station, which is what makes the unique rule a filter over the
        // same stream rather than a different one.
        std::size_t firsts = 0UZ;
        for (const gr::DataSet<float>& record : run.records) {
            if (metaBool(record, "first_seen")) {
                ++firsts;
            }
            expect(!metaBool(record, "center_is_relative")) << "the center is absolute where one is known";
            expect(eq(static_cast<std::int64_t>(metaDouble(record, "center_frequency_hz")) % 100'000, std::int64_t{0}));
        }
        expect(eq(firsts, run.stations.size())) << "one first sighting per station";

        const bool sawShadowed = std::ranges::any_of(run.stations, [](const LteCellScanner::Station& s) { return s.nId1 == 160U; });
        std::println("criterion 17: the cell 6 dB below another on the same root was {} across {} dwells", sawShadowed ? "seen" : "never seen", run.dwells);
    };

    "the scan graph loads and runs"_test = [] {
        const std::string directory = recordingsDirectory();
        const std::string capture   = directory.empty() ? std::string{} : std::format("{}/{}", directory, kCaptureName);

        std::string path = capture;
        if (path.empty() || !std::filesystem::exists(path)) {
            // No capture: the graph still has to load and run, so it is pointed at a stand-in written here. The
            // recording leg below is what skips; this one never does.
            std::vector<Complex> span;
            SceneConfig          one;
            one.nId1   = 42U;
            one.nId2   = 1U;
            one.frames = 4UZ;
            one.seed   = 0x77ULL;
            placeCell(span, one, 1'000'000., 1.f);
            addNoise(span, 0.1, 0x88ULL);

            const std::filesystem::path base = std::filesystem::temp_directory_path() / "gr4_lte_scan_stand_in";
            std::ofstream               data(std::format("{}.sigmf-data", base.string()), std::ios::binary);
            for (const Complex& sample : span) {
                const std::array<std::int16_t, 2UZ> pair{static_cast<std::int16_t>(std::lround(std::clamp(sample.real(), -1.f, 1.f) * 32000.f)), static_cast<std::int16_t>(std::lround(std::clamp(sample.imag(), -1.f, 1.f) * 32000.f))};
                data.write(reinterpret_cast<const char*>(pair.data()), static_cast<std::streamsize>(sizeof(pair)));
            }
            data.close();
            std::ofstream meta(std::format("{}.sigmf-meta", base.string()));
            meta << std::format(R"({{"global":{{"core:datatype":"ci16_le","core:version":"1.2.0","core:num_channels":1,"core:sample_rate":{}}},"captures":[{{"core:sample_start":0,"core:frequency":{}}}],"annotations":[]}})", kWideRate, kWideCenter);
            meta.close();
            path = std::format("{}.sigmf-meta", base.string());
        }

        const std::string text = withCapture(readGraph(), path, 1U);
        expect(nothrow([&text] {
            auto                    graph = gr::loadGrc(gr::globalPluginLoader(), text);
            gr::scheduler::Simple<> scheduler;
            expect(scheduler.exchange(std::move(*graph)).has_value());
            expect(scheduler.runAndWait().has_value());
        })) << "lte_scan.yaml loads and runs to the end of its input";
    };

    "the capture is scanned and its stations named"_test = [] {
        const std::string directory = recordingsDirectory();
        const std::string capture   = directory.empty() ? std::string{} : std::format("{}/{}", directory, kCaptureName);
        if (capture.empty() || !std::filesystem::exists(capture)) {
            std::println("SKIP: no capture at '{}'; configure with -DGR4_RECORDINGS_DIR=<dir> holding {}", capture.empty() ? std::string("<GR4_RECORDINGS_DIR unset>") : capture, kCaptureName);
            std::fflush(nullptr);
            std::_Exit(77);
        }

        gr::Graph flow;
        auto&     source  = flow.emplaceBlock<gr::blocks::fileio::SigMfSource<Complex>>({{"file_name", capture}});
        auto&     scanner = flow.emplaceBlock<LteCellScanner>({{"sample_rate", static_cast<float>(kWideRate)}, {"center_frequency", static_cast<float>(kWideCenter)}, {"passes", gr::Size_t(kCapturePasses)}, {"report", std::string("all")}});
        auto&     sink    = flow.emplaceBlock<RecordSink>();
        expect(flow.connect<"out", "in">(source, scanner).has_value());
        expect(flow.connect<"out", "in">(scanner, sink).has_value());

        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(flow)).has_value());
        const auto started = std::chrono::steady_clock::now();
        expect(scheduler.runAndWait().has_value());
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

        const std::vector<LteCellScanner::Station> stations = scanner.stations();
        std::println("criterion 16: {} passes over {} positions, {} dwells, {} detections, {} stations, {:.1f} s ({:.1f} s per pass)", scanner.nPasses, scanner.nPositions, scanner.nDwells, scanner.nDetections, stations.size(), seconds, scanner.nPasses == 0ULL ? seconds : seconds / static_cast<double>(scanner.nPasses));
        std::vector<LteCellScanner::Station> ordered = stations;
        std::ranges::sort(ordered, [](const auto& a, const auto& b) { return a.centerHz != b.centerHz ? a.centerHz < b.centerHz : a.cellId < b.cellId; });
        for (const LteCellScanner::Station& station : ordered) {
            std::println("  {:.4f} MHz cell {:3} ({:3},{}) {} {} pss {:.1f} sss {:.3f} seen {:5} first pass {} position {}", static_cast<double>(station.centerHz) * 1e-6, station.cellId, station.nId1, station.nId2, station.tdd ? "tdd" : "fdd", station.extended ? "extended" : "normal", static_cast<double>(station.bestPss), static_cast<double>(station.bestSss), station.detections, station.firstPass, station.firstPosition);
        }

        std::vector<std::int64_t> centers;
        for (const LteCellScanner::Station& station : ordered) {
            expect(eq(station.centerHz % 100'000, std::int64_t{0})) << "every center sits on the 100 kHz raster";
            if (centers.empty() || centers.back() != station.centerHz) {
                centers.push_back(station.centerHz);
            }
        }
        expect(centers.size() >= 2UZ) << std::format("at least two distinct centers, got {}", centers.size());

        // The band plan's two carriers are what one pass at the default gate must name, and it must name them with
        // metrics two orders of magnitude clear of the thresholds rather than at them; a station admitted by a
        // single sighting is a draw of a very large search and is what the persistence gate exists to refuse.
        const auto carrier = [&ordered](std::int64_t centerHz, std::uint32_t cellId) {
            const auto at = std::ranges::find_if(ordered, [centerHz, cellId](const LteCellScanner::Station& s) { return s.centerHz == centerHz && s.cellId == cellId; });
            if (at == ordered.end()) {
                return false;
            }
            expect(at->bestPss > 100.f) << std::format("cell {} at {} MHz: primary metric {:.1f}", cellId, static_cast<double>(centerHz) * 1e-6, static_cast<double>(at->bestPss));
            expect(at->bestSss > 0.8f) << std::format("cell {} at {} MHz: secondary quality {:.3f}", cellId, static_cast<double>(centerHz) * 1e-6, static_cast<double>(at->bestSss));
            expect(at->detections >= 2ULL) << std::format("cell {} at {} MHz: seen {} times", cellId, static_cast<double>(centerHz) * 1e-6, at->detections);
            return true;
        };
        expect(carrier(751'000'000, 19U)) << "band 13's downlink, cell 19, at 751.0 MHz";
        expect(carrier(763'000'000, 52U)) << "band 14's downlink, cell 52, at 763.0 MHz";
    };
};

int main() { /* not needed for UT */ }
