// GENERATED FILE — do not edit. Source of truth: blocks/recipes/NbfmDemod.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_NBFMDEMOD_HPP
#define GNURADIO_RECIPES_NBFMDEMOD_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct NbfmDemod {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(float sample_rate_, float deviation_) : sample_rate(std::move(sample_rate_)), deviation(std::move(deviation_)) {}
        float sample_rate; // channel sample rate in hertz; required
        float deviation; // nominal FM deviation in hertz; required
        double tau = 7.5e-05; // de-emphasis time constant in seconds; 75 us is the common narrow-band choice
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::NbfmDemod";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("channel sample rate in hertz; required"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sample_rate"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("nominal FM deviation in hertz; required"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("deviation"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("de-emphasis time constant in seconds; 75 us is the common narrow-band choice"));
            m9[std::pmr::string("default")] = gr::pmt::Value(7.5e-05);
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("tau"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m10;
            m10[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("nbfm_demod"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m10));
            gr::property_map m11;
            gr::Tensor<gr::pmt::Value> t12;
            gr::pmt::Value e13;
            gr::Tensor<gr::pmt::Value> t14;
            gr::pmt::Value e15;
            e15 = gr::pmt::Value(std::pmr::string("discriminator"));
            t14.push_back(std::move(e15));
            gr::pmt::Value e16;
            e16 = gr::pmt::Value(std::pmr::string("INPUT"));
            t14.push_back(std::move(e16));
            gr::pmt::Value e17;
            e17 = gr::pmt::Value(std::pmr::string("in"));
            t14.push_back(std::move(e17));
            gr::pmt::Value e18;
            e18 = gr::pmt::Value(std::pmr::string("in"));
            t14.push_back(std::move(e18));
            e13 = gr::pmt::Value(std::move(t14));
            t12.push_back(std::move(e13));
            gr::pmt::Value e19;
            gr::Tensor<gr::pmt::Value> t20;
            gr::pmt::Value e21;
            e21 = gr::pmt::Value(std::pmr::string("deemphasis"));
            t20.push_back(std::move(e21));
            gr::pmt::Value e22;
            e22 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t20.push_back(std::move(e22));
            gr::pmt::Value e23;
            e23 = gr::pmt::Value(std::pmr::string("out"));
            t20.push_back(std::move(e23));
            gr::pmt::Value e24;
            e24 = gr::pmt::Value(std::pmr::string("out"));
            t20.push_back(std::move(e24));
            e19 = gr::pmt::Value(std::move(t20));
            t12.push_back(std::move(e19));
            m11[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t12));
            gr::Tensor<gr::pmt::Value> t25;
            gr::pmt::Value e26;
            gr::Tensor<gr::pmt::Value> t27;
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("discriminator"));
            t27.push_back(std::move(e28));
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("deemphasis"));
            t27.push_back(std::move(e30));
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e31));
            e26 = gr::pmt::Value(std::move(t27));
            t25.push_back(std::move(e26));
            m11[std::pmr::string("connections")] = gr::pmt::Value(std::move(t25));
            gr::Tensor<gr::pmt::Value> t32;
            gr::pmt::Value e33;
            gr::property_map m34;
            gr::property_map m35;
            m35[std::pmr::string("gain")] = gr::pmt::Value(std::pmr::string("=sample_rate / (2 * pi * deviation)"));
            m35[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("discriminator"));
            m34[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m35));
            m34[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::QuadratureDemod<float32>"));
            e33 = gr::pmt::Value(std::move(m34));
            t32.push_back(std::move(e33));
            gr::pmt::Value e36;
            gr::property_map m37;
            gr::property_map m38;
            m38[std::pmr::string("tau")] = gr::pmt::Value(std::pmr::string("=tau"));
            m38[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m38[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("deemphasis"));
            m37[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m38));
            m37[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::FmDeemphasis<float32>"));
            e36 = gr::pmt::Value(std::move(m37));
            t32.push_back(std::move(e36));
            m11[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t32));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m11));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m39;
            m39[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-08-28"));
            m39[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m39[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m39[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m39[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::NbfmDemod"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m39));
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
        values[std::pmr::string("deviation")] = parameters.deviation;
        values[std::pmr::string("tau")] = parameters.tau;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_NBFMDEMOD_HPP
