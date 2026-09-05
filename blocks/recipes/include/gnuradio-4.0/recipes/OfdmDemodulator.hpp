// GENERATED FILE — do not edit. Source of truth: blocks/recipes/OfdmDemodulator.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_OFDMDEMODULATOR_HPP
#define GNURADIO_RECIPES_OFDMDEMODULATOR_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct OfdmDemodulator {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::uint32_t fft_len_, std::vector<std::int32_t> data_carriers_, std::vector<std::int32_t> pilot_carriers_, std::vector<float> pilot_symbols_, std::vector<float> sync_word_, std::uint32_t n_sync_, std::uint32_t frame_len_, std::vector<std::uint32_t> cp_len_, std::uint32_t preamble_cp_len_) : fft_len(std::move(fft_len_)), data_carriers(std::move(data_carriers_)), pilot_carriers(std::move(pilot_carriers_)), pilot_symbols(std::move(pilot_symbols_)), sync_word(std::move(sync_word_)), n_sync(std::move(n_sync_)), frame_len(std::move(frame_len_)), cp_len(std::move(cp_len_)), preamble_cp_len(std::move(preamble_cp_len_)) {}
        std::uint32_t fft_len; // transform length, a power of two; one symbol is this many carriers; required
        std::vector<std::int32_t> data_carriers; // signed logical carrier indices the output holds, in order; required
        std::vector<std::int32_t> pilot_carriers; // signed logical carrier indices the tracking reads; required
        std::vector<float> pilot_symbols; // interleaved re,im read by (s * n_pilots + p) % len with s the symbol's data index in its frame; required
        std::vector<float> sync_word; // interleaved re,im, one whole fft_len symbol: the known symbol least squares divides by; required
        std::uint32_t n_sync; // sync symbols at a frame's head, the preamble among them; required
        std::uint32_t sync_index = std::uint32_t{1}; // which of those sync symbols is the known one; a Schmidl-Cox preamble occupies even carriers only and cannot be it
        std::uint32_t frame_len; // data symbols per frame; required
        std::vector<std::uint32_t> cp_len; // prefix samples per symbol, the transmitter's own cycle; required
        std::uint32_t preamble_cp_len; // the preamble symbol's prefix, which sets the sync's plateau width; cp_len's first entry; required
        std::int32_t timing_offset = std::int32_t{0}; // signed bias of the cut, in [-min(cp_len), 0]; negative starts the transform window inside the prefix
        float threshold = 0.6f; // the timing metric above which the sync holds a preamble's plateau to have started, in (0, 1)
        float r_floor = 1e-09f; // received energy below which no trigger is emitted; scale dependent, set it from the link's own level
        std::uint32_t min_gap = std::uint32_t{1}; // dead time after a trigger, in symbols; a frame's own length keeps it from being detected twice
        std::string tracking = std::string("cpe"); // 'cpe', 'cpe_interp' or 'none'
        float alpha = 0.1f; // single-pole coefficient of the per-carrier update, in (0, 1]; read by 'cpe_interp'
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::OfdmDemodulator";
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
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("signed logical carrier indices the output holds, in order; required"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("int32[]"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("data_carriers"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("signed logical carrier indices the tracking reads; required"));
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
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("interleaved re,im, one whole fft_len symbol: the known symbol least squares divides by; required"));
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32[]"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sync_word"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("sync symbols at a frame's head, the preamble among them; required"));
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("n_sync"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            gr::pmt::Value e16;
            gr::property_map m17;
            m17[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("which of those sync symbols is the known one; a Schmidl-Cox preamble occupies even carriers only and cannot be it"));
            m17[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m17[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m17[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sync_index"));
            e16 = gr::pmt::Value(std::move(m17));
            t3.push_back(std::move(e16));
            gr::pmt::Value e18;
            gr::property_map m19;
            m19[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("data symbols per frame; required"));
            m19[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m19[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("frame_len"));
            e18 = gr::pmt::Value(std::move(m19));
            t3.push_back(std::move(e18));
            gr::pmt::Value e20;
            gr::property_map m21;
            m21[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("prefix samples per symbol, the transmitter's own cycle; required"));
            m21[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32[]"));
            m21[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("cp_len"));
            e20 = gr::pmt::Value(std::move(m21));
            t3.push_back(std::move(e20));
            gr::pmt::Value e22;
            gr::property_map m23;
            m23[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the preamble symbol's prefix, which sets the sync's plateau width; cp_len's first entry; required"));
            m23[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m23[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("preamble_cp_len"));
            e22 = gr::pmt::Value(std::move(m23));
            t3.push_back(std::move(e22));
            gr::pmt::Value e24;
            gr::property_map m25;
            m25[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("signed bias of the cut, in [-min(cp_len), 0]; negative starts the transform window inside the prefix"));
            m25[std::pmr::string("default")] = gr::pmt::Value(std::int32_t{0});
            m25[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("int32"));
            m25[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing_offset"));
            e24 = gr::pmt::Value(std::move(m25));
            t3.push_back(std::move(e24));
            gr::pmt::Value e26;
            gr::property_map m27;
            m27[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the timing metric above which the sync holds a preamble's plateau to have started, in (0, 1)"));
            m27[std::pmr::string("default")] = gr::pmt::Value(0.6f);
            m27[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m27[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("threshold"));
            e26 = gr::pmt::Value(std::move(m27));
            t3.push_back(std::move(e26));
            gr::pmt::Value e28;
            gr::property_map m29;
            m29[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("received energy below which no trigger is emitted; scale dependent, set it from the link's own level"));
            m29[std::pmr::string("default")] = gr::pmt::Value(1e-09f);
            m29[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m29[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("r_floor"));
            e28 = gr::pmt::Value(std::move(m29));
            t3.push_back(std::move(e28));
            gr::pmt::Value e30;
            gr::property_map m31;
            m31[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("dead time after a trigger, in symbols; a frame's own length keeps it from being detected twice"));
            m31[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m31[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m31[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("min_gap"));
            e30 = gr::pmt::Value(std::move(m31));
            t3.push_back(std::move(e30));
            gr::pmt::Value e32;
            gr::property_map m33;
            m33[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("'cpe', 'cpe_interp' or 'none'"));
            m33[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("cpe"));
            m33[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m33[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("tracking"));
            e32 = gr::pmt::Value(std::move(m33));
            t3.push_back(std::move(e32));
            gr::pmt::Value e34;
            gr::property_map m35;
            m35[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("single-pole coefficient of the per-carrier update, in (0, 1]; read by 'cpe_interp'"));
            m35[std::pmr::string("default")] = gr::pmt::Value(0.1f);
            m35[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m35[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("alpha"));
            e34 = gr::pmt::Value(std::move(m35));
            t3.push_back(std::move(e34));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m36;
            m36[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("ofdm_demodulator"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m36));
            gr::property_map m37;
            gr::Tensor<gr::pmt::Value> t38;
            gr::pmt::Value e39;
            gr::Tensor<gr::pmt::Value> t40;
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::pmr::string("sync"));
            t40.push_back(std::move(e41));
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("INPUT"));
            t40.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::pmr::string("in"));
            t40.push_back(std::move(e43));
            gr::pmt::Value e44;
            e44 = gr::pmt::Value(std::pmr::string("in"));
            t40.push_back(std::move(e44));
            e39 = gr::pmt::Value(std::move(t40));
            t38.push_back(std::move(e39));
            gr::pmt::Value e45;
            gr::Tensor<gr::pmt::Value> t46;
            gr::pmt::Value e47;
            e47 = gr::pmt::Value(std::pmr::string("equalizer"));
            t46.push_back(std::move(e47));
            gr::pmt::Value e48;
            e48 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t46.push_back(std::move(e48));
            gr::pmt::Value e49;
            e49 = gr::pmt::Value(std::pmr::string("out"));
            t46.push_back(std::move(e49));
            gr::pmt::Value e50;
            e50 = gr::pmt::Value(std::pmr::string("out"));
            t46.push_back(std::move(e50));
            e45 = gr::pmt::Value(std::move(t46));
            t38.push_back(std::move(e45));
            gr::pmt::Value e51;
            gr::Tensor<gr::pmt::Value> t52;
            gr::pmt::Value e53;
            e53 = gr::pmt::Value(std::pmr::string("equalizer"));
            t52.push_back(std::move(e53));
            gr::pmt::Value e54;
            e54 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t52.push_back(std::move(e54));
            gr::pmt::Value e55;
            e55 = gr::pmt::Value(std::pmr::string("channel"));
            t52.push_back(std::move(e55));
            gr::pmt::Value e56;
            e56 = gr::pmt::Value(std::pmr::string("channel"));
            t52.push_back(std::move(e56));
            e51 = gr::pmt::Value(std::move(t52));
            t38.push_back(std::move(e51));
            m37[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t38));
            gr::Tensor<gr::pmt::Value> t57;
            gr::pmt::Value e58;
            gr::Tensor<gr::pmt::Value> t59;
            gr::pmt::Value e60;
            e60 = gr::pmt::Value(std::pmr::string("sync"));
            t59.push_back(std::move(e60));
            gr::pmt::Value e61;
            e61 = gr::pmt::Value(std::int64_t{0});
            t59.push_back(std::move(e61));
            gr::pmt::Value e62;
            e62 = gr::pmt::Value(std::pmr::string("prefix"));
            t59.push_back(std::move(e62));
            gr::pmt::Value e63;
            e63 = gr::pmt::Value(std::int64_t{0});
            t59.push_back(std::move(e63));
            e58 = gr::pmt::Value(std::move(t59));
            t57.push_back(std::move(e58));
            gr::pmt::Value e64;
            gr::Tensor<gr::pmt::Value> t65;
            gr::pmt::Value e66;
            e66 = gr::pmt::Value(std::pmr::string("prefix"));
            t65.push_back(std::move(e66));
            gr::pmt::Value e67;
            e67 = gr::pmt::Value(std::int64_t{0});
            t65.push_back(std::move(e67));
            gr::pmt::Value e68;
            e68 = gr::pmt::Value(std::pmr::string("equalizer"));
            t65.push_back(std::move(e68));
            gr::pmt::Value e69;
            e69 = gr::pmt::Value(std::int64_t{0});
            t65.push_back(std::move(e69));
            e64 = gr::pmt::Value(std::move(t65));
            t57.push_back(std::move(e64));
            m37[std::pmr::string("connections")] = gr::pmt::Value(std::move(t57));
            gr::Tensor<gr::pmt::Value> t70;
            gr::pmt::Value e71;
            gr::property_map m72;
            gr::property_map m73;
            m73[std::pmr::string("min_gap")] = gr::pmt::Value(std::pmr::string("=min_gap"));
            m73[std::pmr::string("r_floor")] = gr::pmt::Value(std::pmr::string("=r_floor"));
            m73[std::pmr::string("threshold")] = gr::pmt::Value(std::pmr::string("=threshold"));
            m73[std::pmr::string("cp_len")] = gr::pmt::Value(std::pmr::string("=preamble_cp_len"));
            m73[std::pmr::string("fft_len")] = gr::pmt::Value(std::pmr::string("=fft_len"));
            m73[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sync"));
            m72[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m73));
            m72[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ofdm::SchmidlCoxSync"));
            e71 = gr::pmt::Value(std::move(m72));
            t70.push_back(std::move(e71));
            gr::pmt::Value e74;
            gr::property_map m75;
            gr::property_map m76;
            m76[std::pmr::string("timing_offset")] = gr::pmt::Value(std::pmr::string("=timing_offset"));
            m76[std::pmr::string("frame_len")] = gr::pmt::Value(std::pmr::string("=frame_len"));
            m76[std::pmr::string("n_sync")] = gr::pmt::Value(std::pmr::string("=n_sync"));
            m76[std::pmr::string("cp_len")] = gr::pmt::Value(std::pmr::string("=cp_len"));
            m76[std::pmr::string("fft_len")] = gr::pmt::Value(std::pmr::string("=fft_len"));
            m76[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("prefix"));
            m75[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m76));
            m75[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ofdm::CpRemove"));
            e74 = gr::pmt::Value(std::move(m75));
            t70.push_back(std::move(e74));
            gr::pmt::Value e77;
            gr::property_map m78;
            gr::property_map m79;
            m79[std::pmr::string("tracking")] = gr::pmt::Value(std::pmr::string("=tracking"));
            m79[std::pmr::string("n_sync")] = gr::pmt::Value(std::pmr::string("=n_sync"));
            m79[std::pmr::string("sync_word")] = gr::pmt::Value(std::pmr::string("=sync_word"));
            m79[std::pmr::string("pilot_symbols")] = gr::pmt::Value(std::pmr::string("=pilot_symbols"));
            m79[std::pmr::string("alpha")] = gr::pmt::Value(std::pmr::string("=alpha"));
            m79[std::pmr::string("pilot_carriers")] = gr::pmt::Value(std::pmr::string("=pilot_carriers"));
            m79[std::pmr::string("data_carriers")] = gr::pmt::Value(std::pmr::string("=data_carriers"));
            m79[std::pmr::string("fft_len")] = gr::pmt::Value(std::pmr::string("=fft_len"));
            m79[std::pmr::string("sync_index")] = gr::pmt::Value(std::pmr::string("=sync_index"));
            m79[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("equalizer"));
            m78[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m79));
            m78[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ofdm::OfdmChannelEqualizer"));
            e77 = gr::pmt::Value(std::move(m78));
            t70.push_back(std::move(e77));
            m37[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t70));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m37));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m80;
            m80[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m80[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m80[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m80[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m80[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::OfdmDemodulator"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m80));
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
        values[std::pmr::string("sync_word")] = parameters.sync_word;
        values[std::pmr::string("n_sync")] = parameters.n_sync;
        values[std::pmr::string("sync_index")] = parameters.sync_index;
        values[std::pmr::string("frame_len")] = parameters.frame_len;
        values[std::pmr::string("cp_len")] = parameters.cp_len;
        values[std::pmr::string("preamble_cp_len")] = parameters.preamble_cp_len;
        values[std::pmr::string("timing_offset")] = parameters.timing_offset;
        values[std::pmr::string("threshold")] = parameters.threshold;
        values[std::pmr::string("r_floor")] = parameters.r_floor;
        values[std::pmr::string("min_gap")] = parameters.min_gap;
        values[std::pmr::string("tracking")] = std::pmr::string(parameters.tracking);
        values[std::pmr::string("alpha")] = parameters.alpha;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_OFDMDEMODULATOR_HPP
