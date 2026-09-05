// GENERATED FILE — do not edit. Source of truth: blocks/recipes/OfdmModulator.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_OFDMMODULATOR_HPP
#define GNURADIO_RECIPES_OFDMMODULATOR_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct OfdmModulator {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::uint32_t fft_len_, std::vector<std::int32_t> data_carriers_, std::vector<std::int32_t> pilot_carriers_, std::vector<float> pilot_symbols_, std::vector<float> sync_words_, std::uint32_t frame_len_, std::vector<std::uint32_t> cp_len_) : fft_len(std::move(fft_len_)), data_carriers(std::move(data_carriers_)), pilot_carriers(std::move(pilot_carriers_)), pilot_symbols(std::move(pilot_symbols_)), sync_words(std::move(sync_words_)), frame_len(std::move(frame_len_)), cp_len(std::move(cp_len_)) {}
        std::uint32_t fft_len; // transform length, a power of two; one symbol is this many carriers; required
        std::vector<std::int32_t> data_carriers; // signed logical carrier indices the data stream fills, in order; required
        std::vector<std::int32_t> pilot_carriers; // signed logical carrier indices the pilot cycle fills; empty for a numerology without pilots; required
        std::vector<float> pilot_symbols; // interleaved re,im read by (s * n_pilots + p) % len with s the symbol's data index in its frame; required
        std::vector<float> sync_words; // interleaved re,im, a whole number of fft_len-carrier symbols, emitted verbatim at each frame's head; required
        std::uint32_t frame_len; // data symbols per frame; required
        std::vector<std::uint32_t> cp_len; // prefix samples: one entry is a constant, several are a per-symbol cycle restarting at each frame; required
        std::uint32_t window_len = std::uint32_t{0}; // raised-cosine edge samples overlapped with the next symbol; 0 is off
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::OfdmModulator";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("transform length, a power of two; one symbol is this many carriers; required"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("fft_len"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("signed logical carrier indices the data stream fills, in order; required"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("int32[]"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("data_carriers"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("signed logical carrier indices the pilot cycle fills; empty for a numerology without pilots; required"));
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("int32[]"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("pilot_carriers"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("interleaved re,im read by (s * n_pilots + p) % len with s the symbol's data index in its frame; required"));
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32[]"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("pilot_symbols"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("interleaved re,im, a whole number of fft_len-carrier symbols, emitted verbatim at each frame's head; required"));
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32[]"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sync_words"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("data symbols per frame; required"));
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("frame_len"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            gr::pmt::Value e16;
            gr::property_map m17;
            m17[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("prefix samples: one entry is a constant, several are a per-symbol cycle restarting at each frame; required"));
            m17[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32[]"));
            m17[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("cp_len"));
            e16 = gr::pmt::Value(std::move(m17));
            t3.push_back(std::move(e16));
            gr::pmt::Value e18;
            gr::property_map m19;
            m19[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("raised-cosine edge samples overlapped with the next symbol; 0 is off"));
            m19[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{0});
            m19[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m19[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("window_len"));
            e18 = gr::pmt::Value(std::move(m19));
            t3.push_back(std::move(e18));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m20;
            m20[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("ofdm_modulator"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m20));
            gr::property_map m21;
            gr::Tensor<gr::pmt::Value> t22;
            gr::pmt::Value e23;
            gr::Tensor<gr::pmt::Value> t24;
            gr::pmt::Value e25;
            e25 = gr::pmt::Value(std::pmr::string("allocator"));
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
            e31 = gr::pmt::Value(std::pmr::string("prefix"));
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
            e38 = gr::pmt::Value(std::pmr::string("allocator"));
            t37.push_back(std::move(e38));
            gr::pmt::Value e39;
            e39 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e39));
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("prefix"));
            t37.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e41));
            e36 = gr::pmt::Value(std::move(t37));
            t35.push_back(std::move(e36));
            m21[std::pmr::string("connections")] = gr::pmt::Value(std::move(t35));
            gr::Tensor<gr::pmt::Value> t42;
            gr::pmt::Value e43;
            gr::property_map m44;
            gr::property_map m45;
            m45[std::pmr::string("frame_len")] = gr::pmt::Value(std::pmr::string("=frame_len"));
            m45[std::pmr::string("sync_words")] = gr::pmt::Value(std::pmr::string("=sync_words"));
            m45[std::pmr::string("pilot_symbols")] = gr::pmt::Value(std::pmr::string("=pilot_symbols"));
            m45[std::pmr::string("pilot_carriers")] = gr::pmt::Value(std::pmr::string("=pilot_carriers"));
            m45[std::pmr::string("data_carriers")] = gr::pmt::Value(std::pmr::string("=data_carriers"));
            m45[std::pmr::string("fft_len")] = gr::pmt::Value(std::pmr::string("=fft_len"));
            m45[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("allocator"));
            m44[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m45));
            m44[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ofdm::CarrierAllocator"));
            e43 = gr::pmt::Value(std::move(m44));
            t42.push_back(std::move(e43));
            gr::pmt::Value e46;
            gr::property_map m47;
            gr::property_map m48;
            m48[std::pmr::string("emit_trigger")] = gr::pmt::Value(false);
            m48[std::pmr::string("window_len")] = gr::pmt::Value(std::pmr::string("=window_len"));
            m48[std::pmr::string("cp_len")] = gr::pmt::Value(std::pmr::string("=cp_len"));
            m48[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("prefix"));
            m47[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m48));
            m47[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ofdm::CpInsert"));
            e46 = gr::pmt::Value(std::move(m47));
            t42.push_back(std::move(e46));
            m21[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t42));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m21));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m49;
            m49[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m49[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m49[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m49[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m49[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::OfdmModulator"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m49));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("fft_len")] = parameters.fft_len;
        values[std::pmr::string("data_carriers")] = parameters.data_carriers;
        values[std::pmr::string("pilot_carriers")] = parameters.pilot_carriers;
        values[std::pmr::string("pilot_symbols")] = parameters.pilot_symbols;
        values[std::pmr::string("sync_words")] = parameters.sync_words;
        values[std::pmr::string("frame_len")] = parameters.frame_len;
        values[std::pmr::string("cp_len")] = parameters.cp_len;
        values[std::pmr::string("window_len")] = parameters.window_len;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_OFDMMODULATOR_HPP
