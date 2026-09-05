// GENERATED FILE — do not edit. Source of truth: blocks/recipes/FskDemodAudio.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_FSKDEMODAUDIO_HPP
#define GNURADIO_RECIPES_FSKDEMODAUDIO_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct FskDemodAudio {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(float sample_rate_, float symbol_rate_) : sample_rate(std::move(sample_rate_)), symbol_rate(std::move(symbol_rate_)) {}
        float sample_rate; // input sample rate in hertz; required
        float symbol_rate; // symbol rate in hertz; required
        double lowpass_bandwidth = 0.5; // cutoff of the post-detection lowpass, as a multiple of the symbol rate
        double noise_bandwidth = 0.002; // closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate
        std::string detector = std::string("mueller_muller"); // timing error detector: mueller_muller, modified_mueller_muller, zero_crossing, gardner, early_late, signal_slope_ml or signum_slope_ml
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::FskDemodAudio";
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
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("cutoff of the post-detection lowpass, as a multiple of the symbol rate"));
            m9[std::pmr::string("default")] = gr::pmt::Value(0.5);
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("lowpass_bandwidth"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate"));
            m11[std::pmr::string("default")] = gr::pmt::Value(0.002);
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("noise_bandwidth"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("timing error detector: mueller_muller, modified_mueller_muller, zero_crossing, gardner, early_late, signal_slope_ml or signum_slope_ml"));
            m13[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("mueller_muller"));
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("detector"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m14;
            m14[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("fsk_demod_audio"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m14));
            gr::property_map m15;
            gr::Tensor<gr::pmt::Value> t16;
            gr::pmt::Value e17;
            gr::Tensor<gr::pmt::Value> t18;
            gr::pmt::Value e19;
            e19 = gr::pmt::Value(std::pmr::string("lowpass"));
            t18.push_back(std::move(e19));
            gr::pmt::Value e20;
            e20 = gr::pmt::Value(std::pmr::string("INPUT"));
            t18.push_back(std::move(e20));
            gr::pmt::Value e21;
            e21 = gr::pmt::Value(std::pmr::string("in"));
            t18.push_back(std::move(e21));
            gr::pmt::Value e22;
            e22 = gr::pmt::Value(std::pmr::string("in"));
            t18.push_back(std::move(e22));
            e17 = gr::pmt::Value(std::move(t18));
            t16.push_back(std::move(e17));
            gr::pmt::Value e23;
            gr::Tensor<gr::pmt::Value> t24;
            gr::pmt::Value e25;
            e25 = gr::pmt::Value(std::pmr::string("slicer"));
            t24.push_back(std::move(e25));
            gr::pmt::Value e26;
            e26 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t24.push_back(std::move(e26));
            gr::pmt::Value e27;
            e27 = gr::pmt::Value(std::pmr::string("out"));
            t24.push_back(std::move(e27));
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("out"));
            t24.push_back(std::move(e28));
            e23 = gr::pmt::Value(std::move(t24));
            t16.push_back(std::move(e23));
            m15[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t16));
            gr::Tensor<gr::pmt::Value> t29;
            gr::pmt::Value e30;
            gr::Tensor<gr::pmt::Value> t31;
            gr::pmt::Value e32;
            e32 = gr::pmt::Value(std::pmr::string("lowpass"));
            t31.push_back(std::move(e32));
            gr::pmt::Value e33;
            e33 = gr::pmt::Value(std::int64_t{0});
            t31.push_back(std::move(e33));
            gr::pmt::Value e34;
            e34 = gr::pmt::Value(std::pmr::string("timing"));
            t31.push_back(std::move(e34));
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::int64_t{0});
            t31.push_back(std::move(e35));
            e30 = gr::pmt::Value(std::move(t31));
            t29.push_back(std::move(e30));
            gr::pmt::Value e36;
            gr::Tensor<gr::pmt::Value> t37;
            gr::pmt::Value e38;
            e38 = gr::pmt::Value(std::pmr::string("timing"));
            t37.push_back(std::move(e38));
            gr::pmt::Value e39;
            e39 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e39));
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("slicer"));
            t37.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e41));
            e36 = gr::pmt::Value(std::move(t37));
            t29.push_back(std::move(e36));
            m15[std::pmr::string("connections")] = gr::pmt::Value(std::move(t29));
            gr::Tensor<gr::pmt::Value> t42;
            gr::pmt::Value e43;
            gr::property_map m44;
            gr::property_map m45;
            m45[std::pmr::string("transition_width")] = gr::pmt::Value(std::pmr::string("=0.5 * lowpass_bandwidth * symbol_rate"));
            m45[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m45[std::pmr::string("cutoff")] = gr::pmt::Value(std::pmr::string("=lowpass_bandwidth * symbol_rate"));
            m45[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m45[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m44[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m45));
            m44[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<float32, float32>"));
            e43 = gr::pmt::Value(std::move(m44));
            t42.push_back(std::move(e43));
            gr::pmt::Value e46;
            gr::property_map m47;
            gr::property_map m48;
            m48[std::pmr::string("noise_bandwidth")] = gr::pmt::Value(std::pmr::string("=noise_bandwidth"));
            m48[std::pmr::string("samples_per_symbol")] = gr::pmt::Value(std::pmr::string("=sample_rate / symbol_rate"));
            m48[std::pmr::string("detector")] = gr::pmt::Value(std::pmr::string("=detector"));
            m48[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing"));
            m47[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m48));
            m47[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::SymbolSync<float32>"));
            e46 = gr::pmt::Value(std::move(m47));
            t42.push_back(std::move(e46));
            gr::pmt::Value e49;
            gr::property_map m50;
            gr::property_map m51;
            m51[std::pmr::string("n_levels")] = gr::pmt::Value(std::uint32_t{2});
            m51[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("slicer"));
            m50[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m51));
            m50[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::PamSlicer<float32>"));
            e49 = gr::pmt::Value(std::move(m50));
            t42.push_back(std::move(e49));
            m15[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t42));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m15));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m52;
            m52[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m52[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m52[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m52[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m52[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::FskDemodAudio"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m52));
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
        values[std::pmr::string("lowpass_bandwidth")] = parameters.lowpass_bandwidth;
        values[std::pmr::string("noise_bandwidth")] = parameters.noise_bandwidth;
        values[std::pmr::string("detector")] = std::pmr::string(parameters.detector);
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_FSKDEMODAUDIO_HPP
