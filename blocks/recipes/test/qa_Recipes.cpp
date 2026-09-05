/* The recipes gate: every recipe listed in blocks/recipes/index.yaml loads through the
 * standard YAML-definitions machinery and instantiates as a composite block with its
 * exported ports in place. A recipe is data; this is the test that keeps it honest. A
 * general recipe's required parameters are part of the contract: instantiating it bare
 * must refuse by name, and instantiating it with the parameters must derive the interior
 * settings and keep deriving them when a parameter changes live. */
#include <cmath>
#include <complex>
#include <format>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include <fstream>
#include <sstream>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/recipes/NbfmDemod.hpp>
#include <gnuradio-4.0/recipes/WbfmMonoDemod.hpp>

#include "RecipeHeaderEmitter.hpp"

namespace {

gr::PluginLoader makeRecipeLoader() {
    static gr::SchedulerRegistry          schedulerRegistry;
    static const std::vector<std::string> paths{std::string(RECIPES_SOURCE_PATH)};
    return gr::PluginLoader(gr::globalBlockRegistry(), schedulerRegistry, paths);
}

// exportedInputPorts()/exportedOutputPorts() return a nested map:
//   { blockUniqueName -> { internalPortName -> { "exportedName" -> name } } }
std::vector<std::string> exportedNames(const gr::property_map& portsMap) {
    std::vector<std::string> names;
    for (const auto& [blockName, portInfoValue] : portsMap) {
        const auto* portMap = portInfoValue.get_if<gr::property_map>();
        if (portMap == nullptr) {
            continue;
        }
        for (const auto& [internalName, exportInfoValue] : *portMap) {
            const auto* exportMap = exportInfoValue.get_if<gr::property_map>();
            if (exportMap == nullptr) {
                continue;
            }
            if (const auto it = exportMap->find("exportedName"); it != exportMap->end()) {
                names.emplace_back(it->second.value_or(std::string_view{}));
            }
        }
    }
    return names;
}

[[nodiscard]] std::shared_ptr<gr::BlockModel> interiorByName(const std::shared_ptr<gr::BlockModel>& composite, std::string_view name) {
    if (composite == nullptr || composite->graph() == nullptr) {
        return nullptr;
    }
    for (const auto& candidate : composite->graph()->blocks()) {
        if (candidate->name() == name) {
            return candidate;
        }
    }
    return nullptr;
}

[[nodiscard]] double numericOf(const gr::pmt::Value& value) { return gr::recipe::detail::doubleOf(value).value_or(0.0); }

[[nodiscard]] std::string stringOf(const gr::pmt::Value& value) { return std::string(value.value_or(std::string_view{})); }

/// what one frequency accounts for in a real stream: its amplitude, and its share of the stream's mean square
struct ToneReading {
    double amplitude{};
    double powerShare{};
};

/// A single-frequency DFT bin evaluated at an arbitrary frequency, so the reading does not depend on the
/// window length landing on a transform grid.
[[nodiscard]] ToneReading readTone(std::span<const float> samples, double frequency, double sampleRate) {
    double real  = 0.0;
    double imag  = 0.0;
    double power = 0.0;
    for (std::size_t i = 0UZ; i < samples.size(); ++i) {
        const double angle = 2.0 * std::numbers::pi * frequency * static_cast<double>(i) / sampleRate;
        const double value = static_cast<double>(samples[i]);
        real += value * std::cos(angle);
        imag -= value * std::sin(angle);
        power += value * value;
    }
    const double count      = static_cast<double>(samples.size());
    const double meanSquare = power / count;
    const double amplitude  = 2.0 * std::hypot(real, imag) / count;
    return {amplitude, meanSquare > 0.0 ? 0.5 * amplitude * amplitude / meanSquare : 0.0};
}

} // namespace

const boost::ut::suite<"recipes"> RecipeTests = [] {
    using namespace boost::ut;

    "every indexed recipe loads; bare instantiation succeeds or refuses only for required parameters"_test = [] {
        auto        loader = makeRecipeLoader();
        const auto& defs   = loader.definitionForBlockName();
        expect(!defs.empty()) << "the index listed no recipes, or was not read";
        for (const auto& [name, definition] : defs) {
            const auto bare = gr::detail::instantiateBlockFromYamlDefinition(loader, definition, {});
            if (!bare.has_value()) {
                expect(bare.error().message.contains("recipe_parameter_required")) << name << ": " << bare.error().message;
            }
        }
    };

    "NbfmDemod demands its rate and deviation, derives, and re-derives live"_test = [] {
        auto loader = makeRecipeLoader();

        auto bare = loader.instantiate("gr::recipes::NbfmDemod");
        expect(bare == nullptr) << "the general recipe must not instantiate with hidden defaults";

        gr::property_map parameters;
        parameters["sample_rate"] = 96000.0f;
        parameters["deviation"]   = 5000.0f;
        auto composite            = loader.instantiate("gr::recipes::NbfmDemod", parameters);
        expect(composite != nullptr) << "instantiate with the required parameters";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr);
        if (discriminator == nullptr) {
            return;
        }
        const auto gain = discriminator->settings().get("gain");
        expect(gain.has_value());
        if (gain.has_value()) {
            expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(96000.0 / (2.0 * std::numbers::pi * 5000.0)))) << "the derivation reached the discriminator";
        }
        const auto deemphasis = interiorByName(composite, "deemphasis");
        expect(deemphasis != nullptr);
        if (deemphasis != nullptr) {
            const auto tau = deemphasis->settings().get("tau");
            expect(tau.has_value());
            if (tau.has_value()) {
                expect(eq(numericOf(*tau), 7.5e-05)) << "the defaulted parameter forwarded";
            }
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr) << "the composite carries the live binding machinery";
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["deviation"] = 2500.0f;
        const auto applied  = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        const auto staged = discriminator->settings().stagedParameters();
        const auto gainIt = staged.find("gain");
        expect(gainIt != staged.end()) << "the re-derived gain is staged";
        if (gainIt != staged.end()) {
            expect(eq(static_cast<float>(numericOf(gainIt->second)), static_cast<float>(96000.0 / (2.0 * std::numbers::pi * 2500.0)))) << "halving the deviation doubles the gain, live";
        }
    };

    "SampleClockOffset turns parts per million into a resampling rate, and re-derives live"_test = [] {
        auto loader = makeRecipeLoader();

        // a clock error has a meaningful zero, so unlike the general demod this one defaults and instantiates
        auto nominal = loader.instantiate("gr::recipes::SampleClockOffset");
        expect(nominal != nullptr) << "a defaulted clock offset is a valid, and inert, channel";

        gr::property_map parameters;
        parameters["ppm"] = 20.0;
        auto composite    = loader.instantiate("gr::recipes::SampleClockOffset", parameters);
        expect(composite != nullptr);
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto clock = interiorByName(composite, "clock");
        expect(clock != nullptr);
        if (clock == nullptr) {
            return;
        }
        const auto rate = clock->settings().get("rate");
        expect(rate.has_value());
        if (rate.has_value()) {
            expect(eq(numericOf(*rate), 1.0 + 20.0e-6)) << "20 ppm fast is 1 + 20e-6 output samples per input";
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr);
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["ppm"]      = -50.0;
        const auto applied = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        const auto staged = clock->settings().stagedParameters();
        const auto rateIt = staged.find("rate");
        expect(rateIt != staged.end()) << "the re-derived rate is staged";
        if (rateIt != staged.end()) {
            expect(eq(numericOf(rateIt->second), 1.0 - 50.0e-6)) << "a slow clock resamples below unity";
        }
    };

    "FskDemod demands the link's own numbers, derives the discriminator gain, and re-derives live"_test = [] {
        auto loader = makeRecipeLoader();

        auto bare = loader.instantiate("gr::recipes::FskDemod");
        expect(bare == nullptr) << "the general recipe must not instantiate with hidden defaults";

        constexpr double kSampleRate      = 48000.0;
        constexpr double kSymbolRate      = 4800.0;
        constexpr double kModulationIndex = 0.5;

        gr::property_map parameters;
        parameters["sample_rate"]      = static_cast<float>(kSampleRate);
        parameters["symbol_rate"]      = static_cast<float>(kSymbolRate);
        parameters["modulation_index"] = kModulationIndex;
        auto composite                 = loader.instantiate("gr::recipes::FskDemod", parameters);
        expect(composite != nullptr) << "instantiate with the required parameters";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr);
        if (discriminator != nullptr) {
            const auto gain = discriminator->settings().get("gain");
            expect(gain.has_value());
            if (gain.has_value()) {
                expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(kSampleRate / (kSymbolRate * std::numbers::pi * kModulationIndex)))) << "the gain hands the transmitted PAM grid back";
            }
        }

        const auto timing = interiorByName(composite, "timing");
        expect(timing != nullptr);
        if (timing != nullptr) {
            const auto sps = timing->settings().get("samples_per_symbol");
            expect(sps.has_value());
            if (sps.has_value()) {
                expect(eq(numericOf(*sps), kSampleRate / kSymbolRate)) << "timing recovery runs at the rate the link states";
            }
            const auto detector = timing->settings().get("detector");
            expect(detector.has_value());
            if (detector.has_value()) {
                expect(eq(stringOf(*detector), std::string("mueller_muller"))) << "a string parameter left at its default is substituted like any other";
            }
        }

        const auto channel = interiorByName(composite, "channel");
        expect(channel != nullptr);
        if (channel != nullptr) {
            const auto cutoff = channel->settings().get("cutoff");
            expect(cutoff.has_value());
            if (cutoff.has_value()) {
                expect(eq(numericOf(*cutoff), 0.6 * kSymbolRate)) << "the filter ahead of the discriminator is what keeps it above the click threshold";
            }
        }

        const auto lowpass = interiorByName(composite, "lowpass");
        expect(lowpass != nullptr);
        if (lowpass != nullptr) {
            const auto cutoff = lowpass->settings().get("cutoff");
            expect(cutoff.has_value());
            if (cutoff.has_value()) {
                expect(eq(numericOf(*cutoff), 0.5 * kSymbolRate)) << "the post-detection lowpass passes half a symbol rate";
            }
        }

        const auto slicer = interiorByName(composite, "slicer");
        expect(slicer != nullptr);
        if (slicer != nullptr) {
            const auto levels = slicer->settings().get("n_levels");
            expect(levels.has_value());
            if (levels.has_value()) {
                expect(eq(numericOf(*levels), 2.0)) << "two levels is the defaulted grid";
            }
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr);
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["modulation_index"] = 1.0;
        change["detector"]         = std::pmr::string("gardner");
        const auto applied         = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        if (discriminator != nullptr) {
            const auto staged = discriminator->settings().stagedParameters();
            const auto gainIt = staged.find("gain");
            expect(gainIt != staged.end()) << "the re-derived gain is staged";
            if (gainIt != staged.end()) {
                expect(eq(static_cast<float>(numericOf(gainIt->second)), static_cast<float>(kSampleRate / (kSymbolRate * std::numbers::pi * 1.0)))) << "doubling the index halves the gain, live";
            }
        }
        if (timing != nullptr) {
            const auto staged     = timing->settings().stagedParameters();
            const auto detectorIt = staged.find("detector");
            expect(detectorIt != staged.end()) << "the re-substituted detector is staged";
            if (detectorIt != staged.end()) {
                expect(eq(stringOf(detectorIt->second), std::string("gardner"))) << "a string parameter re-substitutes live, like a derived number";
            }
        }
    };

    "FskDemod takes the timing detector it is given, a string carried through by substitution"_test = [] {
        auto loader = makeRecipeLoader();

        gr::property_map parameters;
        parameters["sample_rate"]      = 48000.f;
        parameters["symbol_rate"]      = 4800.f;
        parameters["modulation_index"] = 0.5;
        parameters["detector"]         = std::pmr::string("zero_crossing");
        auto composite                 = loader.instantiate("gr::recipes::FskDemod", parameters);
        expect(composite != nullptr) << "a named detector must instantiate";
        if (composite == nullptr) {
            return;
        }

        const auto timing = interiorByName(composite, "timing");
        expect(timing != nullptr);
        if (timing != nullptr) {
            const auto detector = timing->settings().get("detector");
            expect(detector.has_value());
            if (detector.has_value()) {
                expect(eq(stringOf(*detector), std::string("zero_crossing"))) << "the interior block takes the spelling the caller gave";
            }
        }
    };

    "WbfmMonoDemod defaults the service's numbers and demands only the front end's rate"_test = [] {
        auto loader = makeRecipeLoader();

        auto bare = loader.instantiate("gr::recipes::WbfmMonoDemod");
        expect(bare == nullptr) << "the front end's rate is the one number a service recipe still cannot default";

        constexpr double kSampleRate  = 1920000.0;
        constexpr double kDecimation  = 8.0;
        constexpr double kChannelRate = kSampleRate / kDecimation;
        constexpr double kAudioRate   = 48000.0;

        gr::property_map parameters;
        parameters["sample_rate"]        = static_cast<float>(kSampleRate);
        parameters["channel_decimation"] = static_cast<std::uint32_t>(kDecimation);
        auto composite                   = loader.instantiate("gr::recipes::WbfmMonoDemod", parameters);
        expect(composite != nullptr) << "the rate and the decimation are all it needs";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto tuner = interiorByName(composite, "tuner");
        expect(tuner != nullptr);
        if (tuner != nullptr) {
            const auto shift = tuner->settings().get("frequency_shift");
            expect(shift.has_value());
            if (shift.has_value()) {
                expect(eq(numericOf(*shift), 0.0)) << "the tuner is inert at the default offset";
            }
        }

        const auto channel = interiorByName(composite, "channel");
        expect(channel != nullptr);
        if (channel != nullptr) {
            const auto decimation = channel->settings().get("decimation");
            expect(decimation.has_value());
            if (decimation.has_value()) {
                expect(eq(numericOf(*decimation), kDecimation)) << "an integral expression reaches an integral setting whole";
            }
        }

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr);
        if (discriminator != nullptr) {
            const auto gain = discriminator->settings().get("gain");
            expect(gain.has_value());
            if (gain.has_value()) {
                expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(kChannelRate / (2.0 * std::numbers::pi * 75000.0)))) << "the gain is derived at the channel rate, not the front end's";
            }
        }

        const auto audio = interiorByName(composite, "audio");
        expect(audio != nullptr);
        if (audio != nullptr) {
            const auto rate = audio->settings().get("rate");
            expect(rate.has_value());
            if (rate.has_value()) {
                expect(eq(numericOf(*rate), kAudioRate * kDecimation / kSampleRate)) << "the audio ratio follows all three rates";
            }
        }

        const auto deemphasis = interiorByName(composite, "deemphasis");
        expect(deemphasis != nullptr);
        if (deemphasis != nullptr) {
            const auto rate = deemphasis->settings().get("sample_rate");
            const auto tau  = deemphasis->settings().get("tau");
            expect(rate.has_value() && tau.has_value());
            if (rate.has_value() && tau.has_value()) {
                expect(eq(numericOf(*rate), kAudioRate)) << "de-emphasis runs at the audio rate, last in the chain";
                expect(eq(numericOf(*tau), 7.5e-05)) << "75 us is the service's own figure and is defaulted here";
            }
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr);
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["offset_hz"]  = 250000.0;
        change["audio_rate"] = 32000.0f;
        const auto applied   = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        if (tuner != nullptr) {
            expect(shiftIt != staged.end()) << "the re-derived shift is staged";
            if (shiftIt != staged.end()) {
                expect(eq(numericOf(shiftIt->second), -250000.0)) << "a station above the center is brought down to it";
            }
        }
        if (audio != nullptr && deemphasis != nullptr) {
            const auto stagedAudio = audio->settings().stagedParameters();
            const auto rateIt      = stagedAudio.find("rate");
            expect(rateIt != stagedAudio.end());
            if (rateIt != stagedAudio.end()) {
                expect(eq(numericOf(rateIt->second), 32000.0 * kDecimation / kSampleRate)) << "a new audio rate moves the resampler, live";
            }
            const auto stagedDeemphasis = deemphasis->settings().stagedParameters();
            const auto deemphasisIt     = stagedDeemphasis.find("sample_rate");
            expect(deemphasisIt != stagedDeemphasis.end());
            if (deemphasisIt != stagedDeemphasis.end()) {
                expect(eq(numericOf(deemphasisIt->second), 32000.0)) << "and retunes the de-emphasis with it";
            }
        }
    };

    "WbfmMonoDemod hands back the modulating tone at the deviation ratio"_test = [] {
        // A recipe's numbers only mean something if the chain they configure demodulates. The signal is one
        // audio tone at a stated peak deviation; the recipe's `deviation` is the reference full deviation, so
        // the audio the chain hands back must be that tone at exactly the ratio of the two, and nothing else.
        using CF = std::complex<float>;
        using gr::blocks::testing::ProcessFunction;

        constexpr double      kSampleRate = 240000.0; // already a channel, so channel_decimation stays at 1
        constexpr double      kAudioRate  = 48000.0;
        constexpr double      kTone       = 1000.0;
        constexpr double      kDeviation  = 15000.0; // what the signal actually swings
        constexpr double      kReference  = 75000.0; // what the recipe scales against
        constexpr std::size_t kInput      = 60000UZ; // 250 ms at the channel rate

        gr::Tensor<CF> values;
        values.reserve(kInput);
        double phase = 0.0;
        for (std::size_t n = 0UZ; n < kInput; ++n) {
            values.push_back(CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))));
            phase += 2.0 * std::numbers::pi * (kDeviation / kSampleRate) * std::sin(2.0 * std::numbers::pi * kTone * static_cast<double>(n) / kSampleRate);
        }

        gr::Graph  graph;
        auto&      source = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kInput)}, {"values", values}, {"mark_tag", false}});
        const auto demod  = gr::recipes::WbfmMonoDemod::emplace(graph, [] {
            gr::recipes::WbfmMonoDemod::Parameters parameters(static_cast<float>(kSampleRate));
            parameters.tau = 0.0; // bypassed, so the measured amplitude is the discriminator's alone
            return parameters;
        }());
        expect(demod != nullptr) << boost::ut::fatal;
        auto& sink = graph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"name", "audio"}});

        const auto sourceModel = gr::graph::findBlock(graph, source);
        const auto sinkModel   = gr::graph::findBlock(graph, sink);
        expect(sourceModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
        expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, demod, gr::PortDefinition{"in"}).has_value());
        expect(graph.connect(demod, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
        const auto finished = scheduler.runAndWait();
        expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);

        // the resampler's filter is not compensated, so the first samples are its ramp-up rather than the tone
        constexpr std::size_t kSkip   = 2000UZ;
        constexpr std::size_t kWindow = 9600UZ; // 200 cycles of the tone at the audio rate
        expect(ge(sink._samples.size(), kSkip + kWindow)) << std::format("audio samples produced: {}", sink._samples.size()) << boost::ut::fatal;

        const auto reading = readTone(std::span<const float>(sink._samples.data() + kSkip, kWindow), kTone, kAudioRate);
        const auto ratio   = kDeviation / kReference;
        expect(lt(std::abs(reading.amplitude - ratio), 0.001 * ratio)) << std::format("tone amplitude {:.5f}, expected {:.5f}", reading.amplitude, ratio);
        expect(gt(reading.powerShare, 0.9999)) << std::format("the tone accounts for {:.4f} of the audio's power", reading.powerShare);
    };

    "the committed typed headers match a fresh emission byte for byte"_test = [] {
        auto        loader = makeRecipeLoader();
        const auto& defs   = loader.definitionForBlockName();
        expect(!defs.empty());
        for (const auto& [name, definition] : defs) {
            const auto lastColon  = name.rfind("::");
            const auto structName = lastColon == std::string::npos ? name : name.substr(lastColon + 2);
            const auto emitted    = gr::recipe::emitter::emitRecipeHeader(definition, structName + ".yaml");
            expect(emitted.has_value()) << name << (emitted.has_value() ? "" : emitted.error().message);
            if (!emitted.has_value()) {
                continue;
            }
            std::ifstream committed(std::string(RECIPES_SOURCE_PATH) + "/include/gnuradio-4.0/recipes/" + structName + ".hpp", std::ios::binary);
            expect(bool(committed)) << structName << ".hpp is not committed — run gr4-recipe-gen";
            if (!committed) {
                continue;
            }
            std::ostringstream content;
            content << committed.rdbuf();
            expect(content.str() == *emitted) << structName << ".hpp drifted from its recipe — regenerate and commit";
        }
    };

    "the generated header and the loader produce equivalent composites"_test = [] {
        auto       loader = makeRecipeLoader();
        gr::Graph  viaHeaderGraph;
        const auto viaHeader = gr::recipes::NbfmDemod::emplace(viaHeaderGraph, gr::recipes::NbfmDemod::Parameters(96000.0f, 5000.0f));
        expect(viaHeader != nullptr) << "the generated emplace must build the composite";

        gr::property_map parameters;
        parameters["sample_rate"] = 96000.0f;
        parameters["deviation"]   = 5000.0f;
        const auto viaLoader      = loader.instantiate("gr::recipes::NbfmDemod", parameters);
        expect(viaLoader != nullptr);
        if (viaHeader == nullptr || viaLoader == nullptr) {
            return;
        }
        for (const char* interiorName : {"discriminator", "deemphasis"}) {
            const auto a = interiorByName(viaHeader, interiorName);
            const auto b = interiorByName(viaLoader, interiorName);
            expect(a != nullptr && b != nullptr) << interiorName;
            if (a == nullptr || b == nullptr) {
                continue;
            }
            for (const char* key : {"gain", "sample_rate", "tau"}) {
                const auto left  = a->settings().get(key);
                const auto right = b->settings().get(key);
                expect(left.has_value() == right.has_value()) << interiorName << "." << key;
                if (left.has_value() && right.has_value()) {
                    expect(eq(numericOf(*left), numericOf(*right))) << interiorName << "." << key << " must agree across front ends";
                }
            }
        }
        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(viaHeader.get());
        expect(wrapper != nullptr) << "the generated composite carries the live machinery too";
        if (wrapper != nullptr) {
            gr::property_map change;
            change["deviation"] = 2500.0f;
            expect(wrapper->applyRecipeParameters(change).has_value()) << "live re-evaluation works through the generated front end";
        }
    };
};

int main() { /* not needed for ut */ }
