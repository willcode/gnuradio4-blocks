// GENERATED FILE — do not edit. Source of truth: blocks/recipes/FskDemod.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_FSKDEMOD_HPP
#define GNURADIO_RECIPES_FSKDEMOD_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct FskDemod {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(float sample_rate_, float symbol_rate_, double modulation_index_) : sample_rate(std::move(sample_rate_)), symbol_rate(std::move(symbol_rate_)), modulation_index(std::move(modulation_index_)) {}
        float sample_rate; // input sample rate in hertz; required
        float symbol_rate; // symbol rate in hertz; required
        double modulation_index; // h, the phase in units of pi one unit of symbol amplitude turns the carrier by; required, and signed: a negative value means the higher tone is the zero
        double channel_bandwidth = 0.6; // cutoff of the filter ahead of the discriminator, as a multiple of the symbol rate
        double lowpass_bandwidth = 0.5; // cutoff of the post-discriminator lowpass, as a multiple of the symbol rate
        double noise_bandwidth = 0.002; // closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate
        std::string detector = std::string("mueller_muller"); // timing error detector: mueller_muller, modified_mueller_muller, zero_crossing, gardner, early_late, signal_slope_ml or signum_slope_ml
        std::uint32_t preamble_symbols = std::uint32_t{0}; // alternating training symbols a burst opens with, which the timing preset is measured over; 0 disables the preset and the stream passes the stage through
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::FskDemod";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("input sample rate in hertz; required"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sample_rate"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("symbol rate in hertz; required"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("symbol_rate"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("h, the phase in units of pi one unit of symbol amplitude turns the carrier by; required, and signed: a negative value means the higher tone is the zero"));
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("modulation_index"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("cutoff of the filter ahead of the discriminator, as a multiple of the symbol rate"));
            m11[std::pmr::string("default")] = gr::pmt::Value(0.6);
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel_bandwidth"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("cutoff of the post-discriminator lowpass, as a multiple of the symbol rate"));
            m13[std::pmr::string("default")] = gr::pmt::Value(0.5);
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("lowpass_bandwidth"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate"));
            m15[std::pmr::string("default")] = gr::pmt::Value(0.002);
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("noise_bandwidth"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            gr::pmt::Value e16;
            gr::property_map m17;
            m17[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("timing error detector: mueller_muller, modified_mueller_muller, zero_crossing, gardner, early_late, signal_slope_ml or signum_slope_ml"));
            m17[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("mueller_muller"));
            m17[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m17[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("detector"));
            e16 = gr::pmt::Value(std::move(m17));
            t3.push_back(std::move(e16));
            gr::pmt::Value e18;
            gr::property_map m19;
            m19[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("alternating training symbols a burst opens with, which the timing preset is measured over; 0 disables the preset and the stream passes the stage through"));
            m19[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{0});
            m19[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m19[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("preamble_symbols"));
            e18 = gr::pmt::Value(std::move(m19));
            t3.push_back(std::move(e18));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m20;
            m20[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("fsk_demod"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m20));
            gr::property_map m21;
            gr::Tensor<gr::pmt::Value> t22;
            gr::pmt::Value e23;
            gr::Tensor<gr::pmt::Value> t24;
            gr::pmt::Value e25;
            e25 = gr::pmt::Value(std::pmr::string("channel"));
            t24.push_back(std::move(e25));
            gr::pmt::Value e26;
            e26 = gr::pmt::Value(std::pmr::string("INPUT"));
            t24.push_back(std::move(e26));
            gr::pmt::Value e27;
            e27 = gr::pmt::Value(std::pmr::string("in"));
            t24.push_back(std::move(e27));
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("in"));
            t24.push_back(std::move(e28));
            e23 = gr::pmt::Value(std::move(t24));
            t22.push_back(std::move(e23));
            gr::pmt::Value e29;
            gr::Tensor<gr::pmt::Value> t30;
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::pmr::string("slicer"));
            t30.push_back(std::move(e31));
            gr::pmt::Value e32;
            e32 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t30.push_back(std::move(e32));
            gr::pmt::Value e33;
            e33 = gr::pmt::Value(std::pmr::string("out"));
            t30.push_back(std::move(e33));
            gr::pmt::Value e34;
            e34 = gr::pmt::Value(std::pmr::string("out"));
            t30.push_back(std::move(e34));
            e29 = gr::pmt::Value(std::move(t30));
            t22.push_back(std::move(e29));
            m21[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t22));
            gr::Tensor<gr::pmt::Value> t35;
            gr::pmt::Value e36;
            gr::Tensor<gr::pmt::Value> t37;
            gr::pmt::Value e38;
            e38 = gr::pmt::Value(std::pmr::string("channel"));
            t37.push_back(std::move(e38));
            gr::pmt::Value e39;
            e39 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e39));
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("discriminator"));
            t37.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e41));
            e36 = gr::pmt::Value(std::move(t37));
            t35.push_back(std::move(e36));
            gr::pmt::Value e42;
            gr::Tensor<gr::pmt::Value> t43;
            gr::pmt::Value e44;
            e44 = gr::pmt::Value(std::pmr::string("discriminator"));
            t43.push_back(std::move(e44));
            gr::pmt::Value e45;
            e45 = gr::pmt::Value(std::int64_t{0});
            t43.push_back(std::move(e45));
            gr::pmt::Value e46;
            e46 = gr::pmt::Value(std::pmr::string("lowpass"));
            t43.push_back(std::move(e46));
            gr::pmt::Value e47;
            e47 = gr::pmt::Value(std::int64_t{0});
            t43.push_back(std::move(e47));
            e42 = gr::pmt::Value(std::move(t43));
            t35.push_back(std::move(e42));
            gr::pmt::Value e48;
            gr::Tensor<gr::pmt::Value> t49;
            gr::pmt::Value e50;
            e50 = gr::pmt::Value(std::pmr::string("lowpass"));
            t49.push_back(std::move(e50));
            gr::pmt::Value e51;
            e51 = gr::pmt::Value(std::int64_t{0});
            t49.push_back(std::move(e51));
            gr::pmt::Value e52;
            e52 = gr::pmt::Value(std::pmr::string("preamble"));
            t49.push_back(std::move(e52));
            gr::pmt::Value e53;
            e53 = gr::pmt::Value(std::int64_t{0});
            t49.push_back(std::move(e53));
            e48 = gr::pmt::Value(std::move(t49));
            t35.push_back(std::move(e48));
            gr::pmt::Value e54;
            gr::Tensor<gr::pmt::Value> t55;
            gr::pmt::Value e56;
            e56 = gr::pmt::Value(std::pmr::string("preamble"));
            t55.push_back(std::move(e56));
            gr::pmt::Value e57;
            e57 = gr::pmt::Value(std::int64_t{0});
            t55.push_back(std::move(e57));
            gr::pmt::Value e58;
            e58 = gr::pmt::Value(std::pmr::string("timing"));
            t55.push_back(std::move(e58));
            gr::pmt::Value e59;
            e59 = gr::pmt::Value(std::int64_t{0});
            t55.push_back(std::move(e59));
            e54 = gr::pmt::Value(std::move(t55));
            t35.push_back(std::move(e54));
            gr::pmt::Value e60;
            gr::Tensor<gr::pmt::Value> t61;
            gr::pmt::Value e62;
            e62 = gr::pmt::Value(std::pmr::string("timing"));
            t61.push_back(std::move(e62));
            gr::pmt::Value e63;
            e63 = gr::pmt::Value(std::int64_t{0});
            t61.push_back(std::move(e63));
            gr::pmt::Value e64;
            e64 = gr::pmt::Value(std::pmr::string("slicer"));
            t61.push_back(std::move(e64));
            gr::pmt::Value e65;
            e65 = gr::pmt::Value(std::int64_t{0});
            t61.push_back(std::move(e65));
            e60 = gr::pmt::Value(std::move(t61));
            t35.push_back(std::move(e60));
            m21[std::pmr::string("connections")] = gr::pmt::Value(std::move(t35));
            gr::Tensor<gr::pmt::Value> t66;
            gr::pmt::Value e67;
            gr::property_map m68;
            gr::property_map m69;
            m69[std::pmr::string("transition_width")] = gr::pmt::Value(std::pmr::string("=0.5 * channel_bandwidth * symbol_rate"));
            m69[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m69[std::pmr::string("cutoff")] = gr::pmt::Value(std::pmr::string("=channel_bandwidth * symbol_rate"));
            m69[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m69[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel"));
            m68[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m69));
            m68[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<complex<float32>, float32>"));
            e67 = gr::pmt::Value(std::move(m68));
            t66.push_back(std::move(e67));
            gr::pmt::Value e70;
            gr::property_map m71;
            gr::property_map m72;
            m72[std::pmr::string("gain")] = gr::pmt::Value(std::pmr::string("=sample_rate / (symbol_rate * pi * modulation_index)"));
            m72[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("discriminator"));
            m71[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m72));
            m71[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::QuadratureDemod<float32>"));
            e70 = gr::pmt::Value(std::move(m71));
            t66.push_back(std::move(e70));
            gr::pmt::Value e73;
            gr::property_map m74;
            gr::property_map m75;
            m75[std::pmr::string("transition_width")] = gr::pmt::Value(std::pmr::string("=0.5 * lowpass_bandwidth * symbol_rate"));
            m75[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m75[std::pmr::string("cutoff")] = gr::pmt::Value(std::pmr::string("=lowpass_bandwidth * symbol_rate"));
            m75[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m75[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m74[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m75));
            m74[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<float32, float32>"));
            e73 = gr::pmt::Value(std::move(m74));
            t66.push_back(std::move(e73));
            gr::pmt::Value e76;
            gr::property_map m77;
            gr::property_map m78;
            m78[std::pmr::string("preamble_symbols")] = gr::pmt::Value(std::pmr::string("=preamble_symbols"));
            m78[std::pmr::string("symbol_rate")] = gr::pmt::Value(std::pmr::string("=symbol_rate"));
            m78[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m78[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("preamble"));
            m77[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m78));
            m77[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::PreambleTiming<float32>"));
            e76 = gr::pmt::Value(std::move(m77));
            t66.push_back(std::move(e76));
            gr::pmt::Value e79;
            gr::property_map m80;
            gr::property_map m81;
            m81[std::pmr::string("noise_bandwidth")] = gr::pmt::Value(std::pmr::string("=noise_bandwidth"));
            m81[std::pmr::string("samples_per_symbol")] = gr::pmt::Value(std::pmr::string("=sample_rate / symbol_rate"));
            m81[std::pmr::string("detector")] = gr::pmt::Value(std::pmr::string("=detector"));
            m81[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing"));
            m80[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m81));
            m80[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::SymbolSync<float32>"));
            e79 = gr::pmt::Value(std::move(m80));
            t66.push_back(std::move(e79));
            gr::pmt::Value e82;
            gr::property_map m83;
            gr::property_map m84;
            m84[std::pmr::string("n_levels")] = gr::pmt::Value(std::uint32_t{2});
            m84[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("slicer"));
            m83[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m84));
            m83[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::PamSlicer<float32>"));
            e82 = gr::pmt::Value(std::move(m83));
            t66.push_back(std::move(e82));
            m21[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t66));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m21));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m85;
            m85[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m85[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m85[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m85[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m85[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::FskDemod"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m85));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("sample_rate")] = parameters.sample_rate;
        values[std::pmr::string("symbol_rate")] = parameters.symbol_rate;
        values[std::pmr::string("modulation_index")] = parameters.modulation_index;
        values[std::pmr::string("channel_bandwidth")] = parameters.channel_bandwidth;
        values[std::pmr::string("lowpass_bandwidth")] = parameters.lowpass_bandwidth;
        values[std::pmr::string("noise_bandwidth")] = parameters.noise_bandwidth;
        values[std::pmr::string("detector")] = std::pmr::string(parameters.detector);
        values[std::pmr::string("preamble_symbols")] = parameters.preamble_symbols;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_FSKDEMOD_HPP
