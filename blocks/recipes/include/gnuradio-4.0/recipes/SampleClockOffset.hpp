// GENERATED FILE — do not edit. Source of truth: blocks/recipes/SampleClockOffset.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_SAMPLECLOCKOFFSET_HPP
#define GNURADIO_RECIPES_SAMPLECLOCKOFFSET_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct SampleClockOffset {
    struct Parameters {
        double ppm = 0.0; // clock error in parts per million; positive means this clock runs fast and emits more samples than it takes
        float attenuation_db = 60.0f; // stopband target of the resampler's designed prototype, and what its polyphase bank is sized against
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::SampleClockOffset";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("clock error in parts per million; positive means this clock runs fast and emits more samples than it takes"));
            m5[std::pmr::string("default")] = gr::pmt::Value(0.0);
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("ppm"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("stopband target of the resampler's designed prototype, and what its polyphase bank is sized against"));
            m7[std::pmr::string("default")] = gr::pmt::Value(60.0f);
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("attenuation_db"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m8;
            m8[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sample_clock_offset"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m8));
            gr::property_map m9;
            gr::Tensor<gr::pmt::Value> t10;
            gr::pmt::Value e11;
            gr::Tensor<gr::pmt::Value> t12;
            gr::pmt::Value e13;
            e13 = gr::pmt::Value(std::pmr::string("clock"));
            t12.push_back(std::move(e13));
            gr::pmt::Value e14;
            e14 = gr::pmt::Value(std::pmr::string("INPUT"));
            t12.push_back(std::move(e14));
            gr::pmt::Value e15;
            e15 = gr::pmt::Value(std::pmr::string("in"));
            t12.push_back(std::move(e15));
            gr::pmt::Value e16;
            e16 = gr::pmt::Value(std::pmr::string("in"));
            t12.push_back(std::move(e16));
            e11 = gr::pmt::Value(std::move(t12));
            t10.push_back(std::move(e11));
            gr::pmt::Value e17;
            gr::Tensor<gr::pmt::Value> t18;
            gr::pmt::Value e19;
            e19 = gr::pmt::Value(std::pmr::string("clock"));
            t18.push_back(std::move(e19));
            gr::pmt::Value e20;
            e20 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t18.push_back(std::move(e20));
            gr::pmt::Value e21;
            e21 = gr::pmt::Value(std::pmr::string("out"));
            t18.push_back(std::move(e21));
            gr::pmt::Value e22;
            e22 = gr::pmt::Value(std::pmr::string("out"));
            t18.push_back(std::move(e22));
            e17 = gr::pmt::Value(std::move(t18));
            t10.push_back(std::move(e17));
            m9[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t10));
            gr::Tensor<gr::pmt::Value> t23;
            m9[std::pmr::string("connections")] = gr::pmt::Value(std::move(t23));
            gr::Tensor<gr::pmt::Value> t24;
            gr::pmt::Value e25;
            gr::property_map m26;
            gr::property_map m27;
            m27[std::pmr::string("attenuation_db")] = gr::pmt::Value(std::pmr::string("=attenuation_db"));
            m27[std::pmr::string("rate")] = gr::pmt::Value(std::pmr::string("=1 + ppm * 1e-6"));
            m27[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("clock"));
            m26[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m27));
            m26[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::ArbitraryResampler<complex<float32>>"));
            e25 = gr::pmt::Value(std::move(m26));
            t24.push_back(std::move(e25));
            m9[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t24));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m9));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m28;
            m28[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-08-31"));
            m28[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m28[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m28[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m28[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::SampleClockOffset"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m28));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("ppm")] = parameters.ppm;
        values[std::pmr::string("attenuation_db")] = parameters.attenuation_db;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_SAMPLECLOCKOFFSET_HPP
