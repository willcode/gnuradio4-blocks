/* The example gate: every graph in blocks/examples/graphs loads through the tree's own YAML
 * importer, runs headless, and delivers what the file says it delivers. An example that no
 * longer loads is worse than no example, because it is the first thing a new user runs. Each
 * case runs its graph for a bounded stretch and reads the result through the sink registry,
 * which is the route a host application takes. */
#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <fstream>
#include <future>
#include <iterator>
#include <print>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/DataSink.hpp>

namespace {

using namespace std::chrono_literals;

/// how long a case waits for its graph to deliver before it calls the graph broken
constexpr auto kDeliveryTimeout = 4s;

[[nodiscard]] std::string readGraph(std::string_view fileName) {
    std::ifstream      file(std::format("{}/{}", EXAMPLE_GRAPHS_PATH, fileName), std::ios::binary);
    std::ostringstream content;
    content << file.rdbuf();
    return content.str();
}

/// Polls `collect` until it reports it has enough or the timeout runs out, then stops the graph.
/// The scheduler runs on its own thread throughout, because the sinks hand data to a consumer
/// thread and a stopped graph hands out nothing.
[[nodiscard]] bool runUntil(auto& scheduler, auto collect) {
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

/// One scalar parameter as the graph file itself states it, found by key in the text that was loaded.
/// A calibration reads the graph's own numbers rather than carrying a copy of them: a copy stops
/// testing the file the moment the file moves. Comment lines are not parameters, and a YAML type tag
/// is the writer's business - `!!float32 1.0` is the same number as `1.0`.
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

const boost::ut::suite<"example graphs"> ExampleGraphTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;

    "fm_radio_mono.yaml loads, runs headless and hands back demodulated audio"_test = [] {
        // The generator's carrier sits 15 kHz above center, a fifth of the 75 kHz the discriminator
        // gain is scaled against, so the audio settles at 0.2 and stays there.
        constexpr std::size_t kWanted   = 8192UZ;
        constexpr std::size_t kSettled  = 4096UZ; // the tail, past the resampler's ramp-up
        constexpr double      kExpected = 0.2;

        auto                  graph = gr::loadGrc(gr::globalPluginLoader(), readGraph("fm_radio_mono.yaml"));
        gr::scheduler::Simple scheduler;
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

        expect(delivered) << std::format("audio samples produced within the timeout: {}", audio.size()) << fatal;

        double sum = 0.0;
        for (const float sample : std::span<const float>(audio).last(kSettled)) {
            sum += static_cast<double>(sample);
        }
        const double level = sum / static_cast<double>(kSettled);
        expect(lt(std::abs(level - kExpected), 0.02 * kExpected)) << std::format("audio settled at {:.5f}, expected {:.5f}", level, kExpected);
    };

    "channel_impairment_spectrum.yaml loads, runs headless and hands back calibrated records"_test = [] {
        constexpr std::size_t kWanted     = 4UZ;
        constexpr double      kSampleRate = 1000000.0;

        const std::string     file  = readGraph("channel_impairment_spectrum.yaml");
        auto                  graph = gr::loadGrc(gr::globalPluginLoader(), file);
        gr::scheduler::Simple scheduler;
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

        // The four keys are the spectral tier's own record contract: a consumer that cannot read
        // them cannot turn a density back into a power, so the graph is only an example if they arrive.
        const auto& record = records.front();
        expect(eq(record.meta_information.size(), 1UZ)) << fatal;
        expect(eq(metaNumber(record, "sample_rate"), kSampleRate)) << "the record states the rate the graph set";
        expect(ge(metaNumber(record, "sample_start"), 0.)) << "the record states where in the stream it started";
        expect(eq(metaNumber(record, "n_averaged"), 8.)) << "the record states the segments that went into it";
        expect(gt(metaNumber(record, "enbw_bins"), 1.)) << "a Hann window's noise bandwidth exceeds a bin";
        expect(eq(record.signal_values.size(), 1024UZ)) << "one two-sided density per transform bin";

        // The graph is a calibration as well as a picture, and every level it is held to is read back out
        // of the file that was just run rather than restated here: a copy of the numbers would stop
        // testing the file the moment the file moved. What the chain does to a level is closed form.
        //
        //   tone   SignalGenerator's complex Sin is the analytic signal -j*A*exp(j*theta), a constant
        //          envelope, so the tone's power is A^2 and not the A^2/2 a real sine of amplitude A holds
        //   noise  AwgnChannel adds circular noise of mean power `noise_power`, flat across the band
        //   phase  FrequencyOffset and PhaseNoise multiply by a unit modulus: they move power in frequency,
        //          and spread it, without changing how much of it there is
        //   image  IqImbalance is `out = alpha*x + beta*conj(x)` with `alpha = (1 + g*exp(j*phi))/2` and
        //          `beta = (1 - g*exp(j*phi))/2`, so it leaves |alpha|^2 of the tone on its own frequency,
        //          puts |beta|^2 into the image at -100 kHz, and passes circular noise at
        //          |alpha|^2 + |beta|^2. At 0.5 dB and 0.02 rad that is +0.25 dB on the tone - a quarter of
        //          a decibel the reading would be out by if the chain's own gain were left out of it.
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

            // The tone. The density is linear power per hertz referred to a full-scale sine, which is what
            // `level_reference` states, so `sum(psd) * bin_width` over a band is that band's power and a
            // full-scale complex tone integrates to one. The lobe is the peak bin and every bin within TWO
            // ENBW of it, to the nearest bin - three each side at Hann's 1.5015, 2.9 kHz of a 1 MHz band. One
            // ENBW is what the peak density alone is worth; the second covers what the chain spreads: the
            // Hann kernel carries real power two bins out when the tone falls between bins, the drift walks
            // the line 92 Hz within a record (20 kHz/s across the 4608 samples eight half-overlapped
            // segments span), and the 2 Hz Lorentzian leaves a skirt. The AWGN caught inside the lobe is
            // seven bins of a floor 30 dB down, 7e-6 of the tone and 0.00003 dB of the reading, so it is
            // not subtracted.
            const std::size_t lobeHalf = static_cast<std::size_t>(std::llround(2. * enbwBins));
            double            lobe     = 0.;
            for (std::size_t offset = 0UZ; offset <= 2UZ * lobeHalf; ++offset) {
                lobe += static_cast<double>(density[(peak + bins + offset - lobeHalf) % bins]);
            }
            lobe *= binWidth;

            // The floor. Away from the line the density is the AWGN's alone, so density times the sample
            // rate is the power the graph asked for. The far bins start 20 ENBW out, 31 bins, past which the
            // 2 Hz skirt is under the floor; the median over them ignores the image and the inner bins the
            // skirt still reaches. Half a decibel is the estimate's own scatter with room to spare: one bin
            // at eight averages has a relative standard deviation near 1/sqrt(8), 35% or 1.5 dB, and the
            // median over the ~960 far bins tightens that by about 1.25/sqrt(960) - the median's own factor
            // over the mean - to 1.4%, 0.06 dB, on top of the ~0.1 dB by which a chi-square's median sits
            // below its mean. The tolerance is several times the sum, and still small against a real fault.
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

int main() { /* not needed for ut */ }
