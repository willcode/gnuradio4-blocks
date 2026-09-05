// GENERATED FILE — do not edit. Source of truth: blocks/recipes/HdlcDeframe.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_HDLCDEFRAME_HPP
#define GNURADIO_RECIPES_HDLCDEFRAME_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct HdlcDeframe {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::uint32_t max_payload_items_) : max_payload_items(std::move(max_payload_items_)) {}
        std::uint32_t max_payload_items; // largest payload a frame may carry, in octets; required, there is no default — the extractor contract admits none
        std::string coding = std::string("nrzi"); // line code ahead of the framing: 'nrzi', or 'differential' for a link coded the other way round
        std::string payload_bit_order = std::string("lsb_first"); // order the pack stage assembles a de-stuffed octet in; HDLC transmits least significant bit first
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::HdlcDeframe";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("largest payload a frame may carry, in octets; required, there is no default — the extractor contract admits none"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("max_payload_items"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("line code ahead of the framing: 'nrzi', or 'differential' for a link coded the other way round"));
            m7[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("nrzi"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("coding"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("order the pack stage assembles a de-stuffed octet in; HDLC transmits least significant bit first"));
            m9[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("lsb_first"));
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("payload_bit_order"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m10;
            m10[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("hdlc_deframe"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m10));
            gr::property_map m11;
            gr::Tensor<gr::pmt::Value> t12;
            gr::pmt::Value e13;
            gr::Tensor<gr::pmt::Value> t14;
            gr::pmt::Value e15;
            e15 = gr::pmt::Value(std::pmr::string("line_code"));
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
            e21 = gr::pmt::Value(std::pmr::string("check"));
            t20.push_back(std::move(e21));
            gr::pmt::Value e22;
            e22 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t20.push_back(std::move(e22));
            gr::pmt::Value e23;
            e23 = gr::pmt::Value(std::pmr::string("ok"));
            t20.push_back(std::move(e23));
            gr::pmt::Value e24;
            e24 = gr::pmt::Value(std::pmr::string("out"));
            t20.push_back(std::move(e24));
            e19 = gr::pmt::Value(std::move(t20));
            t12.push_back(std::move(e19));
            gr::pmt::Value e25;
            gr::Tensor<gr::pmt::Value> t26;
            gr::pmt::Value e27;
            e27 = gr::pmt::Value(std::pmr::string("check"));
            t26.push_back(std::move(e27));
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t26.push_back(std::move(e28));
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::pmr::string("fail"));
            t26.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("fail"));
            t26.push_back(std::move(e30));
            e25 = gr::pmt::Value(std::move(t26));
            t12.push_back(std::move(e25));
            gr::pmt::Value e31;
            gr::Tensor<gr::pmt::Value> t32;
            gr::pmt::Value e33;
            e33 = gr::pmt::Value(std::pmr::string("extractor"));
            t32.push_back(std::move(e33));
            gr::pmt::Value e34;
            e34 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t32.push_back(std::move(e34));
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::pmr::string("reject"));
            t32.push_back(std::move(e35));
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("reject"));
            t32.push_back(std::move(e36));
            e31 = gr::pmt::Value(std::move(t32));
            t12.push_back(std::move(e31));
            m11[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t12));
            gr::Tensor<gr::pmt::Value> t37;
            gr::pmt::Value e38;
            gr::Tensor<gr::pmt::Value> t39;
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("line_code"));
            t39.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e41));
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("extractor"));
            t39.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e43));
            e38 = gr::pmt::Value(std::move(t39));
            t37.push_back(std::move(e38));
            gr::pmt::Value e44;
            gr::Tensor<gr::pmt::Value> t45;
            gr::pmt::Value e46;
            e46 = gr::pmt::Value(std::pmr::string("extractor"));
            t45.push_back(std::move(e46));
            gr::pmt::Value e47;
            e47 = gr::pmt::Value(std::int64_t{0});
            t45.push_back(std::move(e47));
            gr::pmt::Value e48;
            e48 = gr::pmt::Value(std::pmr::string("check"));
            t45.push_back(std::move(e48));
            gr::pmt::Value e49;
            e49 = gr::pmt::Value(std::int64_t{0});
            t45.push_back(std::move(e49));
            e44 = gr::pmt::Value(std::move(t45));
            t37.push_back(std::move(e44));
            m11[std::pmr::string("connections")] = gr::pmt::Value(std::move(t37));
            gr::Tensor<gr::pmt::Value> t50;
            gr::pmt::Value e51;
            gr::property_map m52;
            gr::property_map m53;
            m53[std::pmr::string("coding")] = gr::pmt::Value(std::pmr::string("=coding"));
            m53[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("line_code"));
            m52[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m53));
            m52[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::DifferentialDecoder<uint8>"));
            e51 = gr::pmt::Value(std::move(m52));
            t50.push_back(std::move(e51));
            gr::pmt::Value e54;
            gr::property_map m55;
            gr::property_map m56;
            m56[std::pmr::string("payload_pack_bits")] = gr::pmt::Value(std::uint32_t{8});
            m56[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{1});
            m56[std::pmr::string("payload_bit_order")] = gr::pmt::Value(std::pmr::string("=payload_bit_order"));
            m56[std::pmr::string("abort_ones")] = gr::pmt::Value(std::uint32_t{7});
            m56[std::pmr::string("stuff_after_ones")] = gr::pmt::Value(std::uint32_t{5});
            m56[std::pmr::string("frame_label")] = gr::pmt::Value(std::pmr::string("hdlc"));
            m56[std::pmr::string("min_payload_items")] = gr::pmt::Value(std::uint32_t{24});
            m56[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::pmr::string("=max_payload_items * 8"));
            m56[std::pmr::string("transparency")] = gr::pmt::Value(std::pmr::string("bit_stuffing"));
            m56[std::pmr::string("end_delimiter")] = gr::pmt::Value(std::pmr::string("01111110"));
            m56[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("extractor"));
            m55[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m56));
            m55[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::DelimiterExtractor<uint8>"));
            e54 = gr::pmt::Value(std::move(m55));
            t50.push_back(std::move(e54));
            gr::pmt::Value e57;
            gr::property_map m58;
            gr::property_map m59;
            m59[std::pmr::string("discard_crc")] = gr::pmt::Value(true);
            m59[std::pmr::string("result_reflected")] = gr::pmt::Value(true);
            m59[std::pmr::string("input_reflected")] = gr::pmt::Value(true);
            m59[std::pmr::string("crc_byte_order")] = gr::pmt::Value(std::pmr::string("little"));
            m59[std::pmr::string("final_xor")] = gr::pmt::Value(std::uint64_t{65535});
            m59[std::pmr::string("poly")] = gr::pmt::Value(std::uint64_t{4129});
            m59[std::pmr::string("width")] = gr::pmt::Value(std::uint32_t{16});
            m59[std::pmr::string("initial_value")] = gr::pmt::Value(std::uint64_t{65535});
            m59[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("check"));
            m58[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m59));
            m58[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::CrcCheck"));
            e57 = gr::pmt::Value(std::move(m58));
            t50.push_back(std::move(e57));
            m11[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t50));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m11));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m60;
            m60[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m60[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m60[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m60[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m60[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::HdlcDeframe"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m60));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("max_payload_items")] = parameters.max_payload_items;
        values[std::pmr::string("coding")] = std::pmr::string(parameters.coding);
        values[std::pmr::string("payload_bit_order")] = std::pmr::string(parameters.payload_bit_order);
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_HDLCDEFRAME_HPP
