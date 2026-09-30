#include <boost/ut.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <fstream>
#include <future>
#include <iterator>
#include <map>
#include <memory>
#include <print>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/YamlPmt.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>

#include <gnuradio-4.0/GrAnalogBlocks.hpp>
#include <gnuradio-4.0/GrBasicBlocks.hpp>
#include <gnuradio-4.0/GrChannelBlocks.hpp>
#include <gnuradio-4.0/GrFilterBlocks.hpp>
#include <gnuradio-4.0/GrMathBlocks.hpp>
#include <gnuradio-4.0/GrMeasurementBlocks.hpp>

#include "ExampleGraphs.hpp"

// The example graphs, loaded through the YAML importer of the core, which graph editors and host applications also use.
// Each graph is read from the text embedded when the build is configured and, where the host file system is readable,
// from the path the install rule gives it. Every block entry of the file has to be constructed under the id the entry
// names.

namespace {

using namespace std::chrono_literals;

constexpr std::string_view kFmRadioMono               = "fm_radio_mono.yaml";
constexpr std::string_view kChannelImpairmentSpectrum = "channel_impairment_spectrum.yaml";
constexpr auto             kDeliveryTimeout           = 10s;

/// @brief One block entry of a graph file: the registry id and the block's name.
struct FileBlock {
    std::string id;
    std::string name;
};

/// @brief Enters the blocks of the analog, basic, channel, filter, math and measurement families into the global
/// registry.
///
/// A graph names its blocks by id, and the loader finds them in the registry. Each block library registers its blocks
/// when it is loaded. An as-needed link leaves out a shared library that supplies no symbol the binary uses. A static
/// link leaves out an archive member that supplies none. The blocks of a library left out are never registered. The
/// call to each library's init function takes a symbol from it and registers its blocks. A block registered twice keeps
/// one entry.
void registerGraphFamilies() {
    gr::BlockRegistry& registry = gr::globalBlockRegistry();
    gr::blocklib::initGrAnalogBlocks(registry);
    gr::blocklib::initGrBasicBlocks(registry);
    gr::blocklib::initGrChannelBlocks(registry);
    gr::blocklib::initGrFilterBlocks(registry);
    gr::blocklib::initGrMathBlocks(registry);
    gr::blocklib::initGrMeasurementBlocks(registry);
}

/// @brief The embedded text of the example graph `fileName`.
[[nodiscard]] std::string_view graphText(std::string_view fileName) {
    const auto found = std::ranges::find(gr::examples::kExampleGraphs, fileName, &gr::examples::ExampleGraph::fileName);
    boost::ut::expect(found != gr::examples::kExampleGraphs.end()) << std::format("{} is not embedded", fileName) << boost::ut::fatal;
    return found->text;
}

#ifdef INSTALLED_EXAMPLE_GRAPHS_PATH
[[nodiscard]] std::string readFile(std::string_view directory, std::string_view fileName) {
    const std::string path = std::format("{}/{}", directory, fileName);
    std::ifstream     file(path, std::ios::binary);
    boost::ut::expect(file.is_open()) << std::format("cannot open {}", path) << boost::ut::fatal;
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}
#endif

/// @brief The id and name of every block entry of a graph file, read with the parser the importer uses.
[[nodiscard]] std::vector<FileBlock> fileBlocks(std::string_view text) {
    using namespace boost::ut;
    const auto yaml = gr::pmt::yaml::deserialize(text);
    expect(yaml.has_value()) << "the graph file parses as YAML" << fatal;
    const auto blocksIt = yaml->find("blocks");
    expect(blocksIt != yaml->end()) << "the graph file has a blocks list" << fatal;
    const auto* entries = blocksIt->second.get_if<gr::Tensor<gr::pmt::Value>>();
    expect(entries != nullptr) << "the blocks entry is a list" << fatal;

    std::vector<FileBlock> result;
    for (const gr::pmt::Value& entry : *entries) {
        const auto* block = entry.get_if<gr::property_map>();
        expect(block != nullptr) << "a block entry is a map" << fatal;
        const auto idIt         = block->find("id");
        const auto parametersIt = block->find("parameters");
        expect(idIt != block->end() && parametersIt != block->end()) << "a block entry has an id and parameters" << fatal;
        const auto* parameters = parametersIt->second.get_if<gr::property_map>();
        expect(parameters != nullptr) << "a block's parameters are a map" << fatal;
        const auto nameIt = parameters->find("name");
        expect(nameIt != parameters->end()) << "a block entry has a name" << fatal;
        result.push_back({.id = std::string(idIt->second.value_or(std::string_view{})), .name = std::string(nameIt->second.value_or(std::string_view{}))});
    }
    return result;
}

/// @brief Loads a graph that is checked and never run.
///
/// A DataSink or DataSetSink adds its signal_name to the global sink registry when its settings are applied, and removes
/// it when it stops. A sink that is loaded and destroyed without running stays registered under its name, and the next
/// sink that takes the name is refused. Every such sink of a graph loaded here takes an empty name, which the registry
/// skips.
[[nodiscard]] gr::meta::indirect<gr::Graph> loadForInspection(std::string_view text) {
    gr::BlockSettings overrides;
    for (const FileBlock& entry : fileBlocks(text)) {
        if (entry.id.starts_with("gr::blocks::basic::DataSink<") || entry.id.starts_with("gr::blocks::basic::DataSetSink<")) {
            overrides.emplace(entry.name, gr::property_map{{"signal_name", std::string{}}});
        }
    }
    return gr::loadGrc(gr::globalPluginLoader(), text, overrides);
}

/// @brief Expects the loaded graph to hold one block per entry of the file, each under the name and id of its entry.
void expectBlocksUnderTheirIds(const gr::Graph& graph, std::string_view text) {
    using namespace boost::ut;
    const std::vector<FileBlock> entries  = fileBlocks(text);
    gr::BlockRegistry&           registry = gr::globalPluginLoader().registry();

    expect(eq(graph.blocks().size(), entries.size())) << "the graph holds one block per entry of the file";
    for (const FileBlock& entry : entries) {
        const auto found = std::ranges::find_if(graph.blocks(), [&entry](const auto& block) { return block->name() == entry.name; });
        expect(found != graph.blocks().end()) << std::format("no block is named '{}'", entry.name);
        if (found != graph.blocks().end()) {
            expect(eq(registry.typeName(*found), entry.id)) << std::format("block '{}'", entry.name);
        }
    }
}

/// @brief Runs the scheduler on its own thread until `collect` reports enough or the timeout passes, then stops it.
[[nodiscard]] bool runUntil(gr::scheduler::Simple<>& scheduler, auto collect) {
    auto       runner   = std::async(std::launch::async, [&scheduler] { return scheduler.runAndWait(); });
    const auto deadline = std::chrono::steady_clock::now() + kDeliveryTimeout;

    bool enough = false;
    while (!enough && std::chrono::steady_clock::now() < deadline) {
        enough = collect();
        if (!enough) {
            std::this_thread::sleep_for(2ms);
        }
    }

    scheduler.requestStop();
    const auto finished = runner.get();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);
    return enough;
}

/// one numeric record-metadata entry, whichever numeric alternative it was written as
[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return fallback;
    }
    if (const auto* asU64 = it->second.template get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asFloat = it->second.template get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    if (const auto* asDouble = it->second.template get_if<double>()) {
        return *asDouble;
    }
    return fallback;
}

/// One scalar parameter as the graph file states it, found by key in the loaded text.
/// A calibration reads the graph's own numbers and keeps no copy of them. A copy stops testing
/// the file as soon as the file changes. Comment lines are not parameters. A YAML type tag is
/// ignored, and `!!float32 1.0` is the same number as `1.0`.
[[nodiscard]] double graphParameter(std::string_view graph, std::string_view key) {
    for (std::size_t at = 0UZ; at < graph.size();) {
        const std::size_t lineEnd = std::min(graph.find('\n', at), graph.size());
        std::string_view  line    = graph.substr(at, lineEnd - at);
        at                        = lineEnd + 1UZ;

        const std::size_t start = line.find_first_not_of(" -");
        if (start == std::string_view::npos || line[start] == '#') {
            continue;
        }
        line.remove_prefix(start);
        if (!line.starts_with(key) || line.size() <= key.size() || line[key.size()] != ':') {
            continue;
        }
        line.remove_prefix(key.size() + 1UZ);
        const std::size_t value = line.find_first_not_of(' ');
        if (value == std::string_view::npos) {
            continue;
        }
        line.remove_prefix(value);
        if (line.starts_with("!!")) {
            const std::size_t tagEnd = line.find(' ');
            const std::size_t number = tagEnd == std::string_view::npos ? std::string_view::npos : line.find_first_not_of(' ', tagEnd);
            if (number == std::string_view::npos) {
                continue;
            }
            line.remove_prefix(number);
        }
        double parsed{};
        if (std::from_chars(line.data(), line.data() + line.size(), parsed).ec == std::errc{}) {
            return parsed;
        }
    }
    throw std::runtime_error(std::format("the graph file states no numeric '{}'", key));
}

/// The circular distance between two bins of a two-sided spectrum, which wraps at the band edge.
[[nodiscard]] std::size_t binDistance(std::size_t lhs, std::size_t rhs, std::size_t bins) noexcept {
    const std::size_t direct = lhs < rhs ? rhs - lhs : lhs - rhs;
    return std::min(direct, bins - direct);
}

} // namespace

const boost::ut::suite<"example graphs"> exampleGraphTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;

    registerGraphFamilies();

    "every example graph loads with every block under its id"_test = [] {
        expect(std::ranges::contains(gr::examples::kExampleGraphs, kFmRadioMono, &gr::examples::ExampleGraph::fileName)) << "the graphs are embedded";
        for (const auto& [fileName, text] : gr::examples::kExampleGraphs) {
            std::println("loading {}", fileName);
            auto graph = loadForInspection(text);
            expectBlocksUnderTheirIds(*graph, text);
        }
    };

#ifdef INSTALLED_EXAMPLE_GRAPHS_PATH
    // ctest installs the files into a prefix inside the build tree before this binary runs
    "every example graph in the install layout is the source file and loads with every block under its id"_test = [] {
        for (const auto& [fileName, text] : gr::examples::kExampleGraphs) {
            const std::string installed = readFile(INSTALLED_EXAMPLE_GRAPHS_PATH, fileName);
            expect(eq(std::string_view{installed}, text)) << std::format("the installed {} is the source file", fileName);
            auto graph = loadForInspection(installed);
            expectBlocksUnderTheirIds(*graph, installed);
        }
    };
#endif

    "a copy naming a block type the registry does not hold fails to load"_test = [] {
        constexpr std::string_view kRegistered   = "gr::blocks::analog::QuadratureDemod<float32>";
        constexpr std::string_view kUnregistered = "gr::blocks::analog::NoSuchDemod<float32>";
        expect(!gr::globalPluginLoader().isBlockAvailable(kUnregistered)) << fatal;

        std::string text{graphText(kFmRadioMono)};
        const auto  at = text.find(kRegistered);
        expect(at != std::string::npos) << "the file names the discriminator's type" << fatal;
        text.replace(at, kRegistered.size(), kUnregistered);

        std::string refusal;
        try {
            auto graph = loadForInspection(text);
        } catch (const std::exception& error) {
            refusal = error.what();
        }
        expect(!refusal.empty()) << "the copy loaded" << fatal;
        expect(refusal.contains(kUnregistered)) << std::format("the refusal names the type: {}", refusal);
    };

    "fm_radio_mono.yaml runs headless and hands back demodulated audio"_test = [] {
        // The generator's tone sits 15 kHz above center, a fifth of the 75 kHz deviation the discriminator gain is scaled
        // to, so the audio settles at 0.2.
        constexpr std::size_t kWanted   = 8192UZ;
        constexpr std::size_t kSettled  = 4096UZ; // the tail, past the filters' start-up transients
        constexpr double      kExpected = 0.2;

        auto                    graph = gr::loadGrc(gr::globalPluginLoader(), graphText(kFmRadioMono));
        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(*graph)).has_value()) << fatal;

        std::shared_ptr<StreamingPoller<float>> poller;
        std::vector<float>                      audio;
        const bool                              delivered = runUntil(scheduler, [&poller, &audio] {
            if (poller == nullptr) {
                poller = globalDataSinkRegistry().getStreamingPoller<float>(DataSinkQuery::signalName("fm_audio"), {.overflowPolicy = OverflowPolicy::Drop});
                return false;
            }
            while (poller->process([&audio](const auto& data) { audio.insert(audio.end(), data.begin(), data.end()); })) {
            }
            return audio.size() >= kWanted;
        });
        expect(delivered) << std::format("audio samples delivered within the timeout: {}", audio.size()) << fatal;

        double sum = 0.0;
        for (const float sample : std::span<const float>(audio).last(kSettled)) {
            sum += static_cast<double>(sample);
        }
        const double level = sum / static_cast<double>(kSettled);
        expect(lt(std::abs(level - kExpected), 0.02 * kExpected)) << std::format("audio settled at {:.5f}, expected {:.5f}", level, kExpected);
    };

    "channel_impairment_spectrum.yaml runs headless and hands back calibrated records"_test = [] {
        constexpr std::size_t kWanted     = 4UZ;
        constexpr double      kSampleRate = 1000000.0;

        const std::string       file{graphText(kChannelImpairmentSpectrum)};
        auto                    graph = gr::loadGrc(gr::globalPluginLoader(), file);
        gr::scheduler::Simple<> scheduler;
        expect(scheduler.exchange(std::move(*graph)).has_value()) << fatal;

        std::shared_ptr<DataSetPoller<float>> poller;
        std::vector<gr::DataSet<float>>       records;
        const bool                            delivered = runUntil(scheduler, [&poller, &records] {
            if (poller == nullptr) {
                poller = globalDataSinkRegistry().getDataSetPoller<float>(DataSinkQuery::signalName("psd"), {.overflowPolicy = OverflowPolicy::Drop});
                return false;
            }
            while (poller->process([&records](const auto& incoming) { records.insert(records.end(), incoming.begin(), incoming.end()); })) {
            }
            return records.size() >= kWanted;
        });

        expect(delivered) << std::format("records produced within the timeout: {}", records.size()) << fatal;

        // The four keys are the spectral estimator's record contract. Without them a density cannot be
        // turned back into a power. The graph is an example only if the keys arrive.
        const auto& record = records.front();
        expect(eq(record.meta_information.size(), 1UZ)) << fatal;
        expect(eq(metaNumber(record, "sample_rate"), kSampleRate)) << "the record states the rate the graph set";
        expect(ge(metaNumber(record, "sample_start"), 0.)) << "the record states where in the stream it started";
        expect(eq(metaNumber(record, "n_averaged"), 8.)) << "the record states the segments that went into it";
        expect(gt(metaNumber(record, "enbw_bins"), 1.)) << "a Hann window's noise bandwidth exceeds a bin";
        expect(eq(record.signal_values.size(), 1024UZ)) << "one two-sided density per transform bin";

        // The graph is a calibration as well as a picture. Each level it is checked against is read
        // back from the file that was just run. The chain's effect on each level is closed form.
        //
        //   tone   SignalGenerator's complex Sin is the analytic signal -j*A*exp(j*theta), a constant
        //          envelope. The tone's power is A^2 and not the A^2/2 of a real sine of amplitude A.
        //   noise  AwgnChannel adds circular noise of mean power `noise_power`, flat across the band.
        //   phase  FrequencyOffset and PhaseNoise multiply by a unit modulus. They move power in
        //          frequency and spread it without changing its total.
        //   image  IqImbalance is `out = alpha*x + beta*conj(x)` with `alpha = (1 + g*exp(j*phi))/2` and
        //          `beta = (1 - g*exp(j*phi))/2`. It leaves |alpha|^2 of the tone on its own frequency
        //          and puts |beta|^2 into the image at -100 kHz. It passes circular noise at
        //          |alpha|^2 + |beta|^2. At 0.5 dB and 0.02 rad that is +0.25 dB on the tone. Without
        //          the chain's own gain the reading would be off by that quarter of a decibel.
        const double               amplitude       = graphParameter(file, "amplitude");
        const double               noise           = graphParameter(file, "noise_power");
        const std::complex<double> rotated         = std::polar(std::pow(10., graphParameter(file, "amplitude_imbalance_db") / 20.), graphParameter(file, "phase_imbalance"));
        const double               direct          = std::norm((1. + rotated) * 0.5);
        const double               image           = std::norm((1. - rotated) * 0.5);
        const double               expectedToneDb  = 10. * std::log10(direct * amplitude * amplitude);
        const double               expectedFloorDb = 10. * std::log10((direct + image) * noise);

        for (std::size_t index = 0UZ; index < records.size(); ++index) {
            const auto&       spectrum = records[index];
            const auto&       density  = spectrum.signal_values;
            const std::size_t bins     = density.size();
            const double      rate     = metaNumber(spectrum, "sample_rate");
            const double      binWidth = rate / static_cast<double>(bins);
            const double      enbwBins = metaNumber(spectrum, "enbw_bins");
            const std::size_t peak     = static_cast<std::size_t>(std::ranges::distance(density.begin(), std::ranges::max_element(density)));

            // The tone. The density is linear power per hertz referred to a full-scale sine, as
            // `level_reference` states. `sum(psd) * bin_width` over a band is therefore that band's power,
            // and a full-scale complex tone integrates to one. The lobe is the peak bin and every bin within
            // two ENBW of it, to the nearest bin. At Hann's 1.5015 that is three bins each side, 2.9 kHz of a
            // 1 MHz band. One ENBW holds the power of the peak density alone. The second holds what the chain
            // spreads. The Hann kernel carries real power two bins out when the tone falls between bins. The
            // drift moves the line 92 Hz within a record, at 20 kHz/s across the 4608 samples that eight
            // half-overlapped segments span. The 2 Hz Lorentzian leaves a skirt. The AWGN inside the lobe is
            // seven bins of a floor 30 dB down. That is 7e-6 of the tone and 0.00003 dB of the reading, and
            // it is not subtracted.
            const std::size_t lobeHalf = static_cast<std::size_t>(std::llround(2. * enbwBins));
            double            lobe     = 0.;
            for (std::size_t offset = 0UZ; offset <= 2UZ * lobeHalf; ++offset) {
                lobe += static_cast<double>(density[(peak + bins + offset - lobeHalf) % bins]);
            }
            lobe *= binWidth;

            // The floor. Away from the line the density is the AWGN's alone. Density times the sample rate
            // is then the configured noise power. The far bins start 20 ENBW out, at 31 bins. Past that point
            // the 2 Hz skirt is under the floor. The median over the far bins ignores the image and the inner
            // bins the skirt still reaches. Half a decibel covers the estimate's own scatter with room to
            // spare. One bin at eight averages has a relative standard deviation near 1/sqrt(8), 35% or
            // 1.5 dB. The median over the ~960 far bins tightens that by about 1.25/sqrt(960) to 1.4%, or
            // 0.06 dB. The factor 1.25 is the median's own factor over the mean. A chi-square's median also
            // sits about 0.1 dB below its mean. The tolerance is several times the sum, and still small
            // against a real fault.
            const std::size_t   away = static_cast<std::size_t>(std::ceil(20. * enbwBins));
            std::vector<double> far;
            far.reserve(bins);
            for (std::size_t k = 0UZ; k < bins; ++k) {
                if (binDistance(k, peak, bins) >= away) {
                    far.push_back(static_cast<double>(density[k]));
                }
            }
            expect(gt(far.size(), bins / 2UZ)) << "the far half of the band is what the floor is read from" << fatal;
            std::ranges::nth_element(far, far.begin() + static_cast<std::ptrdiff_t>(far.size() / 2UZ));

            const double toneDb  = 10. * std::log10(lobe);
            const double floorDb = 10. * std::log10(far[far.size() / 2UZ] * rate);
            std::println("record {}: tone {:+.3f} dB against {:+.3f} expected, floor {:+.2f} dB against {:+.2f} expected, lobe +-{} bins, floor from {} bins", index, toneDb, expectedToneDb, floorDb, expectedFloorDb, lobeHalf, far.size());

            expect(lt(std::abs(toneDb - expectedToneDb), 0.1)) << std::format("record {}: the tone's lobe integrates to {:+.3f} dB, the graph's own amplitude and imbalance say {:+.3f} dB", index, toneDb, expectedToneDb);
            expect(lt(std::abs(floorDb - expectedFloorDb), 0.5)) << std::format("record {}: the floor away from the tone reads {:+.2f} dB, the graph's own noise_power says {:+.2f} dB", index, floorDb, expectedFloorDb);
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
