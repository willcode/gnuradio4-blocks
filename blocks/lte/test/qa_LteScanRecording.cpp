#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <print>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/fileio/SigMfIo.hpp>
#include <gnuradio-4.0/lte/CellScanner.hpp>

using namespace boost::ut;
using gr::blocks::lte::LteCellScanner;

namespace qa_lte_scan_recording {

using Complex = std::complex<float>;

/// The recording this unit scans, and what its metadata says it is.
constexpr double      kWideRate    = 25'000'000.;
constexpr double      kWideCenter  = 757'000'000.;
constexpr const char* kCaptureName = "20260730_182327_757000000_25000000_fc.sigmf-meta";
/// Sweeps of the recording taken. The whole file is about 27 of them, and one already visits every position of the
/// band swept; more of them is a measurement rather than a gate.
constexpr std::uint32_t kCapturePasses = 1U;
/// The thresholds the scan runs at, which are the block's own defaults; the assertions below are multiples of them
/// rather than levels of their own.
constexpr float kPssThreshold = 20.f;
constexpr float kSssThreshold = 0.5f;

/// Whether the long arm runs. The short arm sweeps the band that holds the two carriers the case names; the long
/// arm sweeps the whole passband, which is the same relationship over every position of it.
[[nodiscard]] bool longRun() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

/// The directory the recording is read from, or empty when the build was configured without one.
[[nodiscard]] std::string recordingsDirectory() {
    const char* fromEnvironment = std::getenv("GR4_RECORDINGS_DIR");
    return fromEnvironment == nullptr ? std::string{} : std::string(fromEnvironment);
}

/// Keeps every record the sweep published, which is what a host reads a scan through.
struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<float>> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        // Waiting for the next record is not work: OK with nothing consumed is what the runtime reads as a block
        // the scheduler cannot make progress on, and a sink between two dwells is exactly short of input.
        const std::size_t taken = inSpan.size();
        std::ignore             = inSpan.consume(taken);
        return taken == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

} // namespace qa_lte_scan_recording

using namespace qa_lte_scan_recording;

const boost::ut::suite<"LteScanRecording"> _lteScanRecording = [] {
    "the capture is scanned and its stations named"_test = [] {
        const std::string directory = recordingsDirectory();
        const std::string capture   = directory.empty() ? std::string{} : std::format("{}/{}", directory, kCaptureName);
        if (capture.empty() || !std::filesystem::exists(capture)) {
            std::println("SKIP: no capture at '{}'; configure with -DGR4_RECORDINGS_DIR=<dir> holding {}", capture.empty() ? std::string("<GR4_RECORDINGS_DIR unset>") : capture, kCaptureName);
            std::fflush(nullptr);
            std::_Exit(77);
        }

        // The two carriers the band plan puts in this recording sit 6 MHz either side of its center. The short arm
        // sweeps the 12.5 MHz that holds them both, at a step that puts each of them on a dwell position of its
        // own, and keeps every other setting the application's: the search width, the dwell, the thresholds and
        // the persistence gate are what this leg is for. The long arm sweeps the whole passband at the application's
        // own step, which visits every position between them as well.
        gr::property_map settings{{"sample_rate", static_cast<float>(kWideRate)}, {"center_frequency", static_cast<float>(kWideCenter)}, {"passes", gr::Size_t(kCapturePasses)}, {"report", std::string("all")}};
        if (!longRun()) {
            settings.insert_or_assign(gr::property_map::key_type("span_hz"), gr::pmt::Value(12'500'000.f));
            settings.insert_or_assign(gr::property_map::key_type("step_hz"), gr::pmt::Value(3'000'000.f));
        }

        gr::Graph flow;
        auto&     source  = flow.emplaceBlock<gr::blocks::fileio::SigMfSource<Complex>>({{"file_name", capture}});
        auto&     scanner = flow.emplaceBlock<LteCellScanner>(std::move(settings));
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

        // The band plan's two carriers are what one pass at the default gate must name, and it must name them
        // several times over with metrics a multiple of the thresholds they are admitted on rather than at them; a
        // station admitted by a single sighting at a metric near its threshold is a draw of a very large search,
        // which is what the persistence gate exists to refuse.
        const auto carrier = [&ordered](std::int64_t centerHz, std::uint32_t cellId) {
            const auto at = std::ranges::find_if(ordered, [centerHz, cellId](const LteCellScanner::Station& s) { return s.centerHz == centerHz && s.cellId == cellId; });
            if (at == ordered.end()) {
                return false;
            }
            expect(at->bestPss > 3.f * kPssThreshold) << std::format("cell {} at {} MHz: primary metric {:.1f} against a threshold of {:.0f}", cellId, static_cast<double>(centerHz) * 1e-6, static_cast<double>(at->bestPss), static_cast<double>(kPssThreshold));
            expect(at->bestSss > 1.5f * kSssThreshold) << std::format("cell {} at {} MHz: secondary quality {:.3f} against a threshold of {:.2f}", cellId, static_cast<double>(centerHz) * 1e-6, static_cast<double>(at->bestSss), static_cast<double>(kSssThreshold));
            expect(at->detections >= 2ULL) << std::format("cell {} at {} MHz: seen {} times", cellId, static_cast<double>(centerHz) * 1e-6, at->detections);
            return true;
        };
        expect(carrier(751'000'000, 19U)) << "band 13's downlink, cell 19, at 751.0 MHz";
        expect(carrier(763'000'000, 52U)) << "band 14's downlink, cell 52, at 763.0 MHz";
    };
};

int main() { /* not needed for UT */ }
