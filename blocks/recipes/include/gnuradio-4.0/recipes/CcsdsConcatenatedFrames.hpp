// GENERATED FILE — do not edit. Source of truth: blocks/recipes/CcsdsConcatenatedFrames.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_CCSDSCONCATENATEDFRAMES_HPP
#define GNURADIO_RECIPES_CCSDSCONCATENATEDFRAMES_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct CcsdsConcatenatedFrames {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::uint32_t frame_length_, std::uint32_t error_capability_, std::string code_, std::string basis_, std::string convolutional_, std::string encoded_marker_, std::uint32_t sync_errors_) : frame_length(std::move(frame_length_)), error_capability(std::move(error_capability_)), code(std::move(code_)), basis(std::move(basis_)), convolutional(std::move(convolutional_)), encoded_marker(std::move(encoded_marker_)), sync_errors(std::move(sync_errors_)) {}
        std::uint32_t frame_length; // transfer frame octets, the standard's kI - Q; required, and a multiple of interleave
        std::uint32_t error_capability; // E, the symbol-correction capability per codeword: 16 or 8; must agree with code
        std::string code; // the Reed-Solomon profile: ccsds_255_223 for E = 16, ccsds_255_239 for E = 8
        std::string basis; // conventional or dual; there is no default, because a wrong choice decodes nothing
        std::string convolutional; // the inner convention: ccsds, ccsds_uninverted, nasa_dsn or nasa_dsn_uninverted; four live conventions, no default
        std::string encoded_marker; // the marker's 52 state-independent encoded symbols, gr::blocks::digital::syncword::ccsdsEncodedAsm(convolutional); a definition cannot call the derivation, so its result is required here
        std::uint32_t sync_errors; // the correlator's max_errors, in raw channel symbols of the 52-symbol word; an operating point, so required
        std::uint32_t interleave = std::uint32_t{1}; // I, the codewords of one codeblock: 1, 2, 3, 4, 5 or 8
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::CcsdsConcatenatedFrames";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("transfer frame octets, the standard's kI - Q; required, and a multiple of interleave"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("frame_length"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("E, the symbol-correction capability per codeword: 16 or 8; must agree with code"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("error_capability"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the Reed-Solomon profile: ccsds_255_223 for E = 16, ccsds_255_239 for E = 8"));
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("code"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("conventional or dual; there is no default, because a wrong choice decodes nothing"));
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("basis"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the inner convention: ccsds, ccsds_uninverted, nasa_dsn or nasa_dsn_uninverted; four live conventions, no default"));
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("convolutional"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the marker's 52 state-independent encoded symbols, gr::blocks::digital::syncword::ccsdsEncodedAsm(convolutional); a definition cannot call the derivation, so its result is required here"));
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("encoded_marker"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            gr::pmt::Value e16;
            gr::property_map m17;
            m17[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the correlator's max_errors, in raw channel symbols of the 52-symbol word; an operating point, so required"));
            m17[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m17[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sync_errors"));
            e16 = gr::pmt::Value(std::move(m17));
            t3.push_back(std::move(e16));
            gr::pmt::Value e18;
            gr::property_map m19;
            m19[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("I, the codewords of one codeblock: 1, 2, 3, 4, 5 or 8"));
            m19[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m19[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m19[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("interleave"));
            e18 = gr::pmt::Value(std::move(m19));
            t3.push_back(std::move(e18));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m20;
            m20[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("ccsds_concatenated_frames"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m20));
            gr::property_map m21;
            gr::Tensor<gr::pmt::Value> t22;
            gr::pmt::Value e23;
            gr::Tensor<gr::pmt::Value> t24;
            gr::pmt::Value e25;
            e25 = gr::pmt::Value(std::pmr::string("marker"));
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
            e31 = gr::pmt::Value(std::pmr::string("rs"));
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
            e38 = gr::pmt::Value(std::pmr::string("marker"));
            t37.push_back(std::move(e38));
            gr::pmt::Value e39;
            e39 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e39));
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("symbol_record"));
            t37.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t37.push_back(std::move(e41));
            e36 = gr::pmt::Value(std::move(t37));
            t35.push_back(std::move(e36));
            gr::pmt::Value e42;
            gr::Tensor<gr::pmt::Value> t43;
            gr::pmt::Value e44;
            e44 = gr::pmt::Value(std::pmr::string("symbol_record"));
            t43.push_back(std::move(e44));
            gr::pmt::Value e45;
            e45 = gr::pmt::Value(std::int64_t{0});
            t43.push_back(std::move(e45));
            gr::pmt::Value e46;
            e46 = gr::pmt::Value(std::pmr::string("inner"));
            t43.push_back(std::move(e46));
            gr::pmt::Value e47;
            e47 = gr::pmt::Value(std::int64_t{0});
            t43.push_back(std::move(e47));
            e42 = gr::pmt::Value(std::move(t43));
            t35.push_back(std::move(e42));
            gr::pmt::Value e48;
            gr::Tensor<gr::pmt::Value> t49;
            gr::pmt::Value e50;
            e50 = gr::pmt::Value(std::pmr::string("inner"));
            t49.push_back(std::move(e50));
            gr::pmt::Value e51;
            e51 = gr::pmt::Value(std::int64_t{0});
            t49.push_back(std::move(e51));
            gr::pmt::Value e52;
            e52 = gr::pmt::Value(std::pmr::string("trim"));
            t49.push_back(std::move(e52));
            gr::pmt::Value e53;
            e53 = gr::pmt::Value(std::int64_t{0});
            t49.push_back(std::move(e53));
            e48 = gr::pmt::Value(std::move(t49));
            t35.push_back(std::move(e48));
            gr::pmt::Value e54;
            gr::Tensor<gr::pmt::Value> t55;
            gr::pmt::Value e56;
            e56 = gr::pmt::Value(std::pmr::string("trim"));
            t55.push_back(std::move(e56));
            gr::pmt::Value e57;
            e57 = gr::pmt::Value(std::int64_t{0});
            t55.push_back(std::move(e57));
            gr::pmt::Value e58;
            e58 = gr::pmt::Value(std::pmr::string("derandomizer"));
            t55.push_back(std::move(e58));
            gr::pmt::Value e59;
            e59 = gr::pmt::Value(std::int64_t{0});
            t55.push_back(std::move(e59));
            e54 = gr::pmt::Value(std::move(t55));
            t35.push_back(std::move(e54));
            gr::pmt::Value e60;
            gr::Tensor<gr::pmt::Value> t61;
            gr::pmt::Value e62;
            e62 = gr::pmt::Value(std::pmr::string("derandomizer"));
            t61.push_back(std::move(e62));
            gr::pmt::Value e63;
            e63 = gr::pmt::Value(std::int64_t{0});
            t61.push_back(std::move(e63));
            gr::pmt::Value e64;
            e64 = gr::pmt::Value(std::pmr::string("packer"));
            t61.push_back(std::move(e64));
            gr::pmt::Value e65;
            e65 = gr::pmt::Value(std::int64_t{0});
            t61.push_back(std::move(e65));
            e60 = gr::pmt::Value(std::move(t61));
            t35.push_back(std::move(e60));
            gr::pmt::Value e66;
            gr::Tensor<gr::pmt::Value> t67;
            gr::pmt::Value e68;
            e68 = gr::pmt::Value(std::pmr::string("packer"));
            t67.push_back(std::move(e68));
            gr::pmt::Value e69;
            e69 = gr::pmt::Value(std::int64_t{0});
            t67.push_back(std::move(e69));
            gr::pmt::Value e70;
            e70 = gr::pmt::Value(std::pmr::string("rs"));
            t67.push_back(std::move(e70));
            gr::pmt::Value e71;
            e71 = gr::pmt::Value(std::int64_t{0});
            t67.push_back(std::move(e71));
            e66 = gr::pmt::Value(std::move(t67));
            t35.push_back(std::move(e66));
            m21[std::pmr::string("connections")] = gr::pmt::Value(std::move(t35));
            gr::Tensor<gr::pmt::Value> t72;
            gr::pmt::Value e73;
            gr::property_map m74;
            gr::property_map m75;
            m75[std::pmr::string("tag_at")] = gr::pmt::Value(std::pmr::string("code_start"));
            m75[std::pmr::string("trigger_label")] = gr::pmt::Value(std::pmr::string("ccsds_asm_symbols"));
            m75[std::pmr::string("max_errors")] = gr::pmt::Value(std::pmr::string("=sync_errors"));
            m75[std::pmr::string("access_code")] = gr::pmt::Value(std::pmr::string("=encoded_marker"));
            m75[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("marker"));
            m74[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m75));
            m74[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::AccessCodeCorrelator<float32>"));
            e73 = gr::pmt::Value(std::move(m74));
            t72.push_back(std::move(e73));
            gr::pmt::Value e76;
            gr::property_map m77;
            gr::property_map m78;
            m78[std::pmr::string("trigger")] = gr::pmt::Value(std::pmr::string("ccsds_asm_symbols"));
            m78[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::pmr::string("=52 + (frame_length + 2 * error_capability * interleave) * 16 + 12"));
            m78[std::pmr::string("fixed_payload_items")] = gr::pmt::Value(std::pmr::string("=52 + (frame_length + 2 * error_capability * interleave) * 16 + 12"));
            m78[std::pmr::string("header_format")] = gr::pmt::Value(std::pmr::string("fixed_length"));
            m78[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("symbol_record"));
            m77[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m78));
            m77[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::PacketFramer<float32>"));
            e76 = gr::pmt::Value(std::move(m77));
            t72.push_back(std::move(e76));
            gr::pmt::Value e79;
            gr::property_map m80;
            gr::property_map m81;
            m81[std::pmr::string("termination")] = gr::pmt::Value(std::pmr::string("open"));
            m81[std::pmr::string("code")] = gr::pmt::Value(std::pmr::string("=convolutional"));
            m81[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("inner"));
            m80[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m81));
            m80[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::fec::ViterbiDecodeSoft"));
            e79 = gr::pmt::Value(std::move(m80));
            t72.push_back(std::move(e79));
            gr::pmt::Value e82;
            gr::property_map m83;
            gr::property_map m84;
            m84[std::pmr::string("drop_tail")] = gr::pmt::Value(std::uint32_t{6});
            m84[std::pmr::string("drop_head")] = gr::pmt::Value(std::uint32_t{26});
            m84[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("trim"));
            m83[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m84));
            m83[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::RecordTrim"));
            e82 = gr::pmt::Value(std::move(m83));
            t72.push_back(std::move(e82));
            gr::pmt::Value e85;
            gr::property_map m86;
            gr::property_map m87;
            m87[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{1});
            m87[std::pmr::string("seed")] = gr::pmt::Value(std::pmr::string("11111111"));
            m87[std::pmr::string("taps")] = gr::pmt::Value(std::pmr::string("1,3,5,8"));
            m87[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("derandomizer"));
            m86[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m87));
            m86[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::AdditiveScrambler<gr::DataSet<uint8>>"));
            e85 = gr::pmt::Value(std::move(m86));
            t72.push_back(std::move(e85));
            gr::pmt::Value e88;
            gr::property_map m89;
            gr::property_map m90;
            m90[std::pmr::string("bits_out")] = gr::pmt::Value(std::uint32_t{8});
            m90[std::pmr::string("bits_in")] = gr::pmt::Value(std::uint32_t{1});
            m90[std::pmr::string("output_bit_order")] = gr::pmt::Value(std::pmr::string("msb_first"));
            m90[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("packer"));
            m89[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m90));
            m89[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::RecordRepackBits"));
            e88 = gr::pmt::Value(std::move(m89));
            t72.push_back(std::move(e88));
            gr::pmt::Value e91;
            gr::property_map m92;
            gr::property_map m93;
            m93[std::pmr::string("pad")] = gr::pmt::Value(std::pmr::string("=(255 - 2 * error_capability) - frame_length / interleave"));
            m93[std::pmr::string("interleave")] = gr::pmt::Value(std::pmr::string("=interleave"));
            m93[std::pmr::string("roots")] = gr::pmt::Value(std::pmr::string("=2 * error_capability"));
            m93[std::pmr::string("basis")] = gr::pmt::Value(std::pmr::string("=basis"));
            m93[std::pmr::string("code")] = gr::pmt::Value(std::pmr::string("=code"));
            m93[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("rs"));
            m92[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m93));
            m92[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::fec::RsDecode"));
            e91 = gr::pmt::Value(std::move(m92));
            t72.push_back(std::move(e91));
            m21[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t72));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m21));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m94;
            m94[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m94[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m94[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m94[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m94[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::CcsdsConcatenatedFrames"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m94));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("frame_length")] = parameters.frame_length;
        values[std::pmr::string("error_capability")] = parameters.error_capability;
        values[std::pmr::string("code")] = std::pmr::string(parameters.code);
        values[std::pmr::string("basis")] = std::pmr::string(parameters.basis);
        values[std::pmr::string("convolutional")] = std::pmr::string(parameters.convolutional);
        values[std::pmr::string("encoded_marker")] = std::pmr::string(parameters.encoded_marker);
        values[std::pmr::string("sync_errors")] = parameters.sync_errors;
        values[std::pmr::string("interleave")] = parameters.interleave;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_CCSDSCONCATENATEDFRAMES_HPP
