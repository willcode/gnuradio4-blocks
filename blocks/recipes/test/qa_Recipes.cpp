/* The recipes gate: every recipe listed in blocks/recipes/index.yaml loads through the
 * standard YAML-definitions machinery and instantiates as a composite block with its
 * exported ports in place. A recipe is data; this is the test that keeps it honest. A
 * general recipe's required parameters are part of the contract: instantiating it bare
 * must refuse by name, and instantiating it with the parameters must derive the interior
 * settings and keep deriving them when a parameter changes live. */
#include <numbers>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include <fstream>
#include <sstream>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/recipes/NbfmDemod.hpp>

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
