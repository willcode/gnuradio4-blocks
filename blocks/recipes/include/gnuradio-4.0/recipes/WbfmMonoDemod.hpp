// GENERATED FILE — do not edit. Source of truth: blocks/recipes/WbfmMonoDemod.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_WBFMMONODEMOD_HPP
#define GNURADIO_RECIPES_WBFMMONODEMOD_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct WbfmMonoDemod {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(float sample_rate_) : sample_rate(std::move(sample_rate_)) {}
        float sample_rate; // front-end sample rate in hertz; required
        std::uint32_t channel_decimation = std::uint32_t{1}; // input samples per channel sample; pick it so sample_rate / channel_decimation is 200 to 400 kHz. 1 is a pass-through
        double offset_hz = 0.0; // where the station sits relative to the front end's center, in hertz; 0 leaves the tuner inert
        float deviation = 75000.0f; // peak FM deviation in hertz; 75 kHz is the broadcast figure
        double tau = 7.5e-05; // de-emphasis time constant in seconds; 75 us in the Americas and South Korea, 50 us elsewhere, 0 bypasses
        float audio_rate = 48000.0f; // output audio sample rate in hertz
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::WbfmMonoDemod";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("front-end sample rate in hertz; required"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sample_rate"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("input samples per channel sample; pick it so sample_rate / channel_decimation is 200 to 400 kHz. 1 is a pass-through"));
            m7[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel_decimation"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("where the station sits relative to the front end's center, in hertz; 0 leaves the tuner inert"));
            m9[std::pmr::string("default")] = gr::pmt::Value(0.0);
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("offset_hz"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("peak FM deviation in hertz; 75 kHz is the broadcast figure"));
            m11[std::pmr::string("default")] = gr::pmt::Value(75000.0f);
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("deviation"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("de-emphasis time constant in seconds; 75 us in the Americas and South Korea, 50 us elsewhere, 0 bypasses"));
            m13[std::pmr::string("default")] = gr::pmt::Value(7.5e-05);
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("tau"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("output audio sample rate in hertz"));
            m15[std::pmr::string("default")] = gr::pmt::Value(48000.0f);
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("audio_rate"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m16;
            m16[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("wbfm_mono_demod"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m16));
            gr::property_map m17;
            gr::Tensor<gr::pmt::Value> t18;
            gr::pmt::Value e19;
            gr::Tensor<gr::pmt::Value> t20;
            gr::pmt::Value e21;
            e21 = gr::pmt::Value(std::pmr::string("tuner"));
            t20.push_back(std::move(e21));
            gr::pmt::Value e22;
            e22 = gr::pmt::Value(std::pmr::string("INPUT"));
            t20.push_back(std::move(e22));
            gr::pmt::Value e23;
            e23 = gr::pmt::Value(std::pmr::string("in"));
            t20.push_back(std::move(e23));
            gr::pmt::Value e24;
            e24 = gr::pmt::Value(std::pmr::string("in"));
            t20.push_back(std::move(e24));
            e19 = gr::pmt::Value(std::move(t20));
            t18.push_back(std::move(e19));
            gr::pmt::Value e25;
            gr::Tensor<gr::pmt::Value> t26;
            gr::pmt::Value e27;
            e27 = gr::pmt::Value(std::pmr::string("deemphasis"));
            t26.push_back(std::move(e27));
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t26.push_back(std::move(e28));
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::pmr::string("out"));
            t26.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("out"));
            t26.push_back(std::move(e30));
            e25 = gr::pmt::Value(std::move(t26));
            t18.push_back(std::move(e25));
            m17[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t18));
            gr::Tensor<gr::pmt::Value> t31;
            gr::pmt::Value e32;
            gr::Tensor<gr::pmt::Value> t33;
            gr::pmt::Value e34;
            e34 = gr::pmt::Value(std::pmr::string("tuner"));
            t33.push_back(std::move(e34));
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e35));
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("channel"));
            t33.push_back(std::move(e36));
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e37));
            e32 = gr::pmt::Value(std::move(t33));
            t31.push_back(std::move(e32));
            gr::pmt::Value e38;
            gr::Tensor<gr::pmt::Value> t39;
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("channel"));
            t39.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e41));
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("discriminator"));
            t39.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e43));
            e38 = gr::pmt::Value(std::move(t39));
            t31.push_back(std::move(e38));
            gr::pmt::Value e44;
            gr::Tensor<gr::pmt::Value> t45;
            gr::pmt::Value e46;
            e46 = gr::pmt::Value(std::pmr::string("discriminator"));
            t45.push_back(std::move(e46));
            gr::pmt::Value e47;
            e47 = gr::pmt::Value(std::int64_t{0});
            t45.push_back(std::move(e47));
            gr::pmt::Value e48;
            e48 = gr::pmt::Value(std::pmr::string("audio"));
            t45.push_back(std::move(e48));
            gr::pmt::Value e49;
            e49 = gr::pmt::Value(std::int64_t{0});
            t45.push_back(std::move(e49));
            e44 = gr::pmt::Value(std::move(t45));
            t31.push_back(std::move(e44));
            gr::pmt::Value e50;
            gr::Tensor<gr::pmt::Value> t51;
            gr::pmt::Value e52;
            e52 = gr::pmt::Value(std::pmr::string("audio"));
            t51.push_back(std::move(e52));
            gr::pmt::Value e53;
            e53 = gr::pmt::Value(std::int64_t{0});
            t51.push_back(std::move(e53));
            gr::pmt::Value e54;
            e54 = gr::pmt::Value(std::pmr::string("deemphasis"));
            t51.push_back(std::move(e54));
            gr::pmt::Value e55;
            e55 = gr::pmt::Value(std::int64_t{0});
            t51.push_back(std::move(e55));
            e50 = gr::pmt::Value(std::move(t51));
            t31.push_back(std::move(e50));
            m17[std::pmr::string("connections")] = gr::pmt::Value(std::move(t31));
            gr::Tensor<gr::pmt::Value> t56;
            gr::pmt::Value e57;
            gr::property_map m58;
            gr::property_map m59;
            m59[std::pmr::string("frequency_shift")] = gr::pmt::Value(std::pmr::string("=-offset_hz"));
            m59[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m59[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("tuner"));
            m58[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m59));
            m58[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::math::Rotator<complex<float32>>"));
            e57 = gr::pmt::Value(std::move(m58));
            t56.push_back(std::move(e57));
            gr::pmt::Value e60;
            gr::property_map m61;
            gr::property_map m62;
            m62[std::pmr::string("decimation")] = gr::pmt::Value(std::pmr::string("=channel_decimation"));
            m62[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel"));
            m61[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m62));
            m61[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::StagedDecimator<complex<float32>>"));
            e60 = gr::pmt::Value(std::move(m61));
            t56.push_back(std::move(e60));
            gr::pmt::Value e63;
            gr::property_map m64;
            gr::property_map m65;
            m65[std::pmr::string("gain")] = gr::pmt::Value(std::pmr::string("=sample_rate / (channel_decimation * 2 * pi * deviation)"));
            m65[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("discriminator"));
            m64[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m65));
            m64[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::QuadratureDemod<float32>"));
            e63 = gr::pmt::Value(std::move(m64));
            t56.push_back(std::move(e63));
            gr::pmt::Value e66;
            gr::property_map m67;
            gr::property_map m68;
            m68[std::pmr::string("rate")] = gr::pmt::Value(std::pmr::string("=audio_rate * channel_decimation / sample_rate"));
            m68[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("audio"));
            m67[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m68));
            m67[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::ArbitraryResampler<float32>"));
            e66 = gr::pmt::Value(std::move(m67));
            t56.push_back(std::move(e66));
            gr::pmt::Value e69;
            gr::property_map m70;
            gr::property_map m71;
            m71[std::pmr::string("tau")] = gr::pmt::Value(std::pmr::string("=tau"));
            m71[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=audio_rate"));
            m71[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("deemphasis"));
            m70[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m71));
            m70[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::FmDeemphasis<float32>"));
            e69 = gr::pmt::Value(std::move(m70));
            t56.push_back(std::move(e69));
            m17[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t56));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m17));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m72;
            m72[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m72[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m72[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m72[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m72[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::WbfmMonoDemod"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m72));
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
        values[std::pmr::string("channel_decimation")] = parameters.channel_decimation;
        values[std::pmr::string("offset_hz")] = parameters.offset_hz;
        values[std::pmr::string("deviation")] = parameters.deviation;
        values[std::pmr::string("tau")] = parameters.tau;
        values[std::pmr::string("audio_rate")] = parameters.audio_rate;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_WBFMMONODEMOD_HPP
