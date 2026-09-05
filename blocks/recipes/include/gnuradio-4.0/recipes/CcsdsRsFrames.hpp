// GENERATED FILE — do not edit. Source of truth: blocks/recipes/CcsdsRsFrames.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_CCSDSRSFRAMES_HPP
#define GNURADIO_RECIPES_CCSDSRSFRAMES_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct CcsdsRsFrames {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::uint32_t frame_length_, std::uint32_t error_capability_, std::string code_, std::string basis_, std::uint32_t sync_errors_) : frame_length(std::move(frame_length_)), error_capability(std::move(error_capability_)), code(std::move(code_)), basis(std::move(basis_)), sync_errors(std::move(sync_errors_)) {}
        std::uint32_t frame_length; // transfer frame octets, the standard's kI - Q; required, and a multiple of interleave
        std::uint32_t error_capability; // E, the symbol-correction capability per codeword: 16 or 8; must agree with code
        std::string code; // the Reed-Solomon profile: ccsds_255_223 for E = 16, ccsds_255_239 for E = 8
        std::string basis; // conventional or dual; 4.3.9.1 mandates dual and part of the flown population uses conventional, so there is no default
        std::uint32_t sync_errors; // the marker correlator's max_errors, in bits of the 32-bit ASM; an operating point, so required
        std::uint32_t interleave = std::uint32_t{1}; // I, the codewords of one codeblock: 1, 2, 3, 4, 5 or 8
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::CcsdsRsFrames";
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
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("conventional or dual; 4.3.9.1 mandates dual and part of the flown population uses conventional, so there is no default"));
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("basis"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the marker correlator's max_errors, in bits of the 32-bit ASM; an operating point, so required"));
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sync_errors"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("I, the codewords of one codeblock: 1, 2, 3, 4, 5 or 8"));
            m15[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("interleave"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m16;
            m16[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("ccsds_rs_frames"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m16));
            gr::property_map m17;
            gr::Tensor<gr::pmt::Value> t18;
            gr::pmt::Value e19;
            gr::Tensor<gr::pmt::Value> t20;
            gr::pmt::Value e21;
            e21 = gr::pmt::Value(std::pmr::string("slicer"));
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
            e27 = gr::pmt::Value(std::pmr::string("rs"));
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
            e34 = gr::pmt::Value(std::pmr::string("slicer"));
            t33.push_back(std::move(e34));
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e35));
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("asm_correlator"));
            t33.push_back(std::move(e36));
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e37));
            e32 = gr::pmt::Value(std::move(t33));
            t31.push_back(std::move(e32));
            gr::pmt::Value e38;
            gr::Tensor<gr::pmt::Value> t39;
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("asm_correlator"));
            t39.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e41));
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("codeblock"));
            t39.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e43));
            e38 = gr::pmt::Value(std::move(t39));
            t31.push_back(std::move(e38));
            gr::pmt::Value e44;
            gr::Tensor<gr::pmt::Value> t45;
            gr::pmt::Value e46;
            e46 = gr::pmt::Value(std::pmr::string("codeblock"));
            t45.push_back(std::move(e46));
            gr::pmt::Value e47;
            e47 = gr::pmt::Value(std::int64_t{0});
            t45.push_back(std::move(e47));
            gr::pmt::Value e48;
            e48 = gr::pmt::Value(std::pmr::string("derandomizer"));
            t45.push_back(std::move(e48));
            gr::pmt::Value e49;
            e49 = gr::pmt::Value(std::int64_t{0});
            t45.push_back(std::move(e49));
            e44 = gr::pmt::Value(std::move(t45));
            t31.push_back(std::move(e44));
            gr::pmt::Value e50;
            gr::Tensor<gr::pmt::Value> t51;
            gr::pmt::Value e52;
            e52 = gr::pmt::Value(std::pmr::string("derandomizer"));
            t51.push_back(std::move(e52));
            gr::pmt::Value e53;
            e53 = gr::pmt::Value(std::int64_t{0});
            t51.push_back(std::move(e53));
            gr::pmt::Value e54;
            e54 = gr::pmt::Value(std::pmr::string("packer"));
            t51.push_back(std::move(e54));
            gr::pmt::Value e55;
            e55 = gr::pmt::Value(std::int64_t{0});
            t51.push_back(std::move(e55));
            e50 = gr::pmt::Value(std::move(t51));
            t31.push_back(std::move(e50));
            gr::pmt::Value e56;
            gr::Tensor<gr::pmt::Value> t57;
            gr::pmt::Value e58;
            e58 = gr::pmt::Value(std::pmr::string("packer"));
            t57.push_back(std::move(e58));
            gr::pmt::Value e59;
            e59 = gr::pmt::Value(std::int64_t{0});
            t57.push_back(std::move(e59));
            gr::pmt::Value e60;
            e60 = gr::pmt::Value(std::pmr::string("rs"));
            t57.push_back(std::move(e60));
            gr::pmt::Value e61;
            e61 = gr::pmt::Value(std::int64_t{0});
            t57.push_back(std::move(e61));
            e56 = gr::pmt::Value(std::move(t57));
            t31.push_back(std::move(e56));
            m17[std::pmr::string("connections")] = gr::pmt::Value(std::move(t31));
            gr::Tensor<gr::pmt::Value> t62;
            gr::pmt::Value e63;
            gr::property_map m64;
            gr::property_map m65;
            m65[std::pmr::string("n_levels")] = gr::pmt::Value(std::uint32_t{2});
            m65[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("slicer"));
            m64[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m65));
            m64[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::PamSlicer<float32>"));
            e63 = gr::pmt::Value(std::move(m64));
            t62.push_back(std::move(e63));
            gr::pmt::Value e66;
            gr::property_map m67;
            gr::property_map m68;
            m68[std::pmr::string("trigger_label")] = gr::pmt::Value(std::pmr::string("ccsds_asm"));
            m68[std::pmr::string("max_errors")] = gr::pmt::Value(std::pmr::string("=sync_errors"));
            m68[std::pmr::string("access_code")] = gr::pmt::Value(std::pmr::string("00011010110011111111110000011101"));
            m68[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("asm_correlator"));
            m67[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m68));
            m67[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::AccessCodeCorrelator<uint8>"));
            e66 = gr::pmt::Value(std::move(m67));
            t62.push_back(std::move(e66));
            gr::pmt::Value e69;
            gr::property_map m70;
            gr::property_map m71;
            m71[std::pmr::string("trigger")] = gr::pmt::Value(std::pmr::string("ccsds_asm"));
            m71[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::pmr::string("=(frame_length + 2 * error_capability * interleave) * 8"));
            m71[std::pmr::string("fixed_payload_items")] = gr::pmt::Value(std::pmr::string("=(frame_length + 2 * error_capability * interleave) * 8"));
            m71[std::pmr::string("header_format")] = gr::pmt::Value(std::pmr::string("fixed_length"));
            m71[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("codeblock"));
            m70[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m71));
            m70[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::PacketFramer<uint8>"));
            e69 = gr::pmt::Value(std::move(m70));
            t62.push_back(std::move(e69));
            gr::pmt::Value e72;
            gr::property_map m73;
            gr::property_map m74;
            m74[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{1});
            m74[std::pmr::string("seed")] = gr::pmt::Value(std::pmr::string("11111111"));
            m74[std::pmr::string("taps")] = gr::pmt::Value(std::pmr::string("1,3,5,8"));
            m74[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("derandomizer"));
            m73[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m74));
            m73[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::AdditiveScrambler<gr::DataSet<uint8>>"));
            e72 = gr::pmt::Value(std::move(m73));
            t62.push_back(std::move(e72));
            gr::pmt::Value e75;
            gr::property_map m76;
            gr::property_map m77;
            m77[std::pmr::string("bits_out")] = gr::pmt::Value(std::uint32_t{8});
            m77[std::pmr::string("bits_in")] = gr::pmt::Value(std::uint32_t{1});
            m77[std::pmr::string("output_bit_order")] = gr::pmt::Value(std::pmr::string("msb_first"));
            m77[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("packer"));
            m76[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m77));
            m76[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::RecordRepackBits"));
            e75 = gr::pmt::Value(std::move(m76));
            t62.push_back(std::move(e75));
            gr::pmt::Value e78;
            gr::property_map m79;
            gr::property_map m80;
            m80[std::pmr::string("pad")] = gr::pmt::Value(std::pmr::string("=(255 - 2 * error_capability) - frame_length / interleave"));
            m80[std::pmr::string("interleave")] = gr::pmt::Value(std::pmr::string("=interleave"));
            m80[std::pmr::string("roots")] = gr::pmt::Value(std::pmr::string("=2 * error_capability"));
            m80[std::pmr::string("basis")] = gr::pmt::Value(std::pmr::string("=basis"));
            m80[std::pmr::string("code")] = gr::pmt::Value(std::pmr::string("=code"));
            m80[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("rs"));
            m79[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m80));
            m79[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::fec::RsDecode"));
            e78 = gr::pmt::Value(std::move(m79));
            t62.push_back(std::move(e78));
            m17[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t62));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m17));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m81;
            m81[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m81[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m81[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m81[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m81[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::CcsdsRsFrames"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m81));
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

#endif // GNURADIO_RECIPES_CCSDSRSFRAMES_HPP
