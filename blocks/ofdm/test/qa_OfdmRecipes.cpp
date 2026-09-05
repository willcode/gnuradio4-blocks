/* The numerology-as-a-recipe gate. Four OFDM blocks read the same transform length, the same
 * carrier sets, the same pilot cycle and the same sync words, and nothing in a graph makes
 * three settings agree. The two recipes state the numerology once each; what is tested here is
 * that the vector-typed exported parameters -- which the recipe engine carries by substitution
 * rather than through an expression -- reach the interior blocks whole. */
#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

#include <gnuradio-4.0/algorithm/ofdm/CarrierMap.hpp>

namespace qa_ofdm_recipes {

using CF = std::complex<float>;

[[nodiscard]] gr::PluginLoader makeRecipeLoader() {
    static gr::SchedulerRegistry          schedulerRegistry;
    static const std::vector<std::string> paths{std::string(RECIPES_SOURCE_PATH)};
    return gr::PluginLoader(gr::globalBlockRegistry(), schedulerRegistry, paths);
}

[[nodiscard]] std::vector<std::string> exportedNames(const gr::property_map& portsMap) {
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

constexpr std::uint32_t kFft = 64U;

[[nodiscard]] std::vector<std::int32_t> pilotCarriers() { return {-21, -7, 7, 21}; }

[[nodiscard]] std::vector<std::int32_t> dataCarriers() {
    std::vector<std::int32_t> carriers;
    const auto                pilots = pilotCarriers();
    for (std::int32_t c = -26; c <= 26; ++c) {
        if (c == 0 || std::ranges::find(pilots, c) != pilots.end()) {
            continue;
        }
        carriers.push_back(c);
    }
    return carriers;
}

[[nodiscard]] std::vector<float> interleave(std::span<const CF> values) {
    std::vector<float> flat(2UZ * values.size());
    for (std::size_t k = 0UZ; k < values.size(); ++k) {
        flat[2UZ * k]       = values[k].real();
        flat[2UZ * k + 1UZ] = values[k].imag();
    }
    return flat;
}

[[nodiscard]] std::vector<CF> sounding() {
    std::vector<CF> word(kFft, CF{});
    std::uint32_t   state = 987654321U;
    const auto      place = [&word, &state](std::int32_t carrier) {
        state                                            = state * 1664525U + 1013904223U;
        word[gr::ofdm::CarrierMap::binOf(kFft, carrier)] = CF(((state >> 16U) & 1U) != 0U ? 0.7071f : -0.7071f, ((state >> 17U) & 1U) != 0U ? 0.7071f : -0.7071f);
    };
    for (const std::int32_t carrier : dataCarriers()) {
        place(carrier);
    }
    for (const std::int32_t carrier : pilotCarriers()) {
        place(carrier);
    }
    return word;
}

[[nodiscard]] std::vector<float> pilotSymbols() { return interleave(std::vector<CF>{CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)}); }

/// @brief The vector a block's setting holds, or an empty vector when the key is absent or of another type.
template<typename T>
[[nodiscard]] std::vector<T> settingVector(const std::shared_ptr<gr::BlockModel>& block, const std::string& key) {
    const auto value = block->settings().get(key);
    if (!value.has_value()) {
        return {};
    }
    const auto* tensor = value->template get_if<gr::Tensor<T>>();
    if (tensor == nullptr) {
        return {};
    }
    return std::vector<T>(tensor->begin(), tensor->end());
}

const boost::ut::suite<"OFDM recipes"> _recipes = [] {
    using namespace boost::ut;

    "OfdmModulator demands the numerology and hands every vector through whole"_test = [] {
        auto loader = makeRecipeLoader();
        expect(loader.instantiate("gr::recipes::OfdmModulator") == nullptr) << "a numerology has no default that means anything";

        const auto      preamble = gr::ofdm::schmidlCoxPreamble(kFft, 52UZ, 0xC0FFEEULL);
        const auto      known    = sounding();
        std::vector<CF> words(preamble.begin(), preamble.end());
        words.insert(words.end(), known.begin(), known.end());

        gr::property_map parameters;
        parameters["fft_len"]        = kFft;
        parameters["data_carriers"]  = dataCarriers();
        parameters["pilot_carriers"] = pilotCarriers();
        parameters["pilot_symbols"]  = pilotSymbols();
        parameters["sync_words"]     = interleave(std::span<const CF>(words));
        parameters["frame_len"]      = std::uint32_t{20};
        parameters["cp_len"]         = std::vector<std::uint32_t>{16U, 8U};

        auto composite = loader.instantiate("gr::recipes::OfdmModulator", parameters);
        expect(composite != nullptr) << "instantiate with the numerology";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(std::ranges::find(inputs, "in") != inputs.end());
        expect(std::ranges::find(outputs, "out") != outputs.end());

        const auto allocator = interiorByName(composite, "allocator");
        const auto prefix    = interiorByName(composite, "prefix");
        expect(allocator != nullptr && prefix != nullptr);
        if (allocator == nullptr || prefix == nullptr) {
            return;
        }
        expect(std::ranges::equal(settingVector<std::int32_t>(allocator, "data_carriers"), dataCarriers())) << "48 signed carrier indices, substituted whole";
        expect(std::ranges::equal(settingVector<std::int32_t>(allocator, "pilot_carriers"), pilotCarriers()));
        expect(std::ranges::equal(settingVector<float>(allocator, "sync_words"), interleave(std::span<const CF>(words)))) << "two whole symbols of interleaved re,im";
        expect(std::ranges::equal(settingVector<std::uint32_t>(prefix, "cp_len"), std::vector<std::uint32_t>{16U, 8U})) << "the prefix cycle reaches the block that walks it";
    };

    "OfdmDemodulator shares that numerology across its three blocks"_test = [] {
        auto loader = makeRecipeLoader();
        expect(loader.instantiate("gr::recipes::OfdmDemodulator") == nullptr);

        gr::property_map parameters;
        parameters["fft_len"]         = kFft;
        parameters["data_carriers"]   = dataCarriers();
        parameters["pilot_carriers"]  = pilotCarriers();
        parameters["pilot_symbols"]   = pilotSymbols();
        parameters["sync_word"]       = interleave(std::span<const CF>(sounding()));
        parameters["n_sync"]          = std::uint32_t{2};
        parameters["frame_len"]       = std::uint32_t{20};
        parameters["cp_len"]          = std::vector<std::uint32_t>{16U};
        parameters["preamble_cp_len"] = std::uint32_t{16};
        parameters["timing_offset"]   = std::int32_t{-6};
        parameters["tracking"]        = std::string("cpe_interp");

        auto composite = loader.instantiate("gr::recipes::OfdmDemodulator", parameters);
        expect(composite != nullptr) << "instantiate with the numerology";
        if (composite == nullptr) {
            return;
        }

        const auto outputs = exportedNames(composite->exportedOutputPorts());
        const auto inputs  = exportedNames(composite->exportedInputPorts());
        expect(std::ranges::find(inputs, "in") != inputs.end());
        expect(std::ranges::find(outputs, "out") != outputs.end());
        expect(std::ranges::find(outputs, "channel") != outputs.end()) << "the channel estimate is reachable from outside the composite";

        const auto sync      = interiorByName(composite, "sync");
        const auto prefix    = interiorByName(composite, "prefix");
        const auto equalizer = interiorByName(composite, "equalizer");
        expect(sync != nullptr && prefix != nullptr && equalizer != nullptr);
        if (sync == nullptr || prefix == nullptr || equalizer == nullptr) {
            return;
        }

        const auto fftOf = [](const std::shared_ptr<gr::BlockModel>& block) {
            const auto  value = block->settings().get("fft_len");
            const auto* held  = value.has_value() ? value->get_if<std::uint32_t>() : nullptr;
            return held == nullptr ? 0U : *held;
        };
        expect(eq(fftOf(sync), kFft) && eq(fftOf(prefix), kFft) && eq(fftOf(equalizer), kFft)) << "one transform length, three blocks";

        expect(std::ranges::equal(settingVector<std::int32_t>(equalizer, "data_carriers"), dataCarriers()));
        expect(std::ranges::equal(settingVector<float>(equalizer, "sync_word"), interleave(std::span<const CF>(sounding()))));
        expect(std::ranges::equal(settingVector<std::uint32_t>(prefix, "cp_len"), std::vector<std::uint32_t>{16U}));

        const auto offset = prefix->settings().get("timing_offset");
        expect(offset.has_value() && offset->get_if<std::int32_t>() != nullptr && *offset->get_if<std::int32_t>() == -6) << "a signed bias reaches the block that applies it";
        const auto mode = equalizer->settings().get("tracking");
        expect(mode.has_value() && mode->value_or(std::string_view{}) == "cpe_interp") << "a string parameter is substituted, not evaluated";
    };
};

} // namespace qa_ofdm_recipes

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
