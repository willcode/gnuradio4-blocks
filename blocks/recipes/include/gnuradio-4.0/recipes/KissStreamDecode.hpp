// GENERATED FILE — do not edit. Source of truth: blocks/recipes/KissStreamDecode.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_KISSSTREAMDECODE_HPP
#define GNURADIO_RECIPES_KISSSTREAMDECODE_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct KissStreamDecode {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::uint32_t max_payload_items_) : max_payload_items(std::move(max_payload_items_)) {}
        std::uint32_t drop_head = std::uint32_t{0}; // items removed from the start of every input record before the KISS stream begins; RecordTrim's own default
        std::uint32_t max_payload_items; // DelimiterExtractor's own bound on a decoded frame's items; required, there is no default
        bool read_timestamp = false; // interpret a command-9 control frame as a timestamp for the next data frame; KissDecode's own default
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::KissStreamDecode";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("items removed from the start of every input record before the KISS stream begins; RecordTrim's own default"));
            m5[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{0});
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("drop_head"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("DelimiterExtractor's own bound on a decoded frame's items; required, there is no default"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("max_payload_items"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("interpret a command-9 control frame as a timestamp for the next data frame; KissDecode's own default"));
            m9[std::pmr::string("default")] = gr::pmt::Value(false);
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("bool"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("read_timestamp"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m10;
            m10[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss_stream_decode"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m10));
            gr::property_map m11;
            gr::Tensor<gr::pmt::Value> t12;
            gr::pmt::Value e13;
            gr::Tensor<gr::pmt::Value> t14;
            gr::pmt::Value e15;
            e15 = gr::pmt::Value(std::pmr::string("trim"));
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
            e21 = gr::pmt::Value(std::pmr::string("kiss"));
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
            e28 = gr::pmt::Value(std::pmr::string("trim"));
            t27.push_back(std::move(e28));
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("flatten"));
            t27.push_back(std::move(e30));
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e31));
            e26 = gr::pmt::Value(std::move(t27));
            t25.push_back(std::move(e26));
            gr::pmt::Value e32;
            gr::Tensor<gr::pmt::Value> t33;
            gr::pmt::Value e34;
            e34 = gr::pmt::Value(std::pmr::string("flatten"));
            t33.push_back(std::move(e34));
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e35));
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("extractor"));
            t33.push_back(std::move(e36));
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e37));
            e32 = gr::pmt::Value(std::move(t33));
            t25.push_back(std::move(e32));
            gr::pmt::Value e38;
            gr::Tensor<gr::pmt::Value> t39;
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("extractor"));
            t39.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e41));
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("kiss"));
            t39.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e43));
            e38 = gr::pmt::Value(std::move(t39));
            t25.push_back(std::move(e38));
            m11[std::pmr::string("connections")] = gr::pmt::Value(std::move(t25));
            gr::Tensor<gr::pmt::Value> t44;
            gr::pmt::Value e45;
            gr::property_map m46;
            gr::property_map m47;
            m47[std::pmr::string("drop_head")] = gr::pmt::Value(std::pmr::string("=drop_head"));
            m47[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("trim"));
            m46[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m47));
            m46[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::RecordTrim"));
            e45 = gr::pmt::Value(std::move(m46));
            t44.push_back(std::move(e45));
            gr::pmt::Value e48;
            gr::property_map m49;
            gr::property_map m50;
            m50[std::pmr::string("boundary_label")] = gr::pmt::Value(std::pmr::string(""));
            m50[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("flatten"));
            m49[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m50));
            m49[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::DataSetToStream<uint8>"));
            e48 = gr::pmt::Value(std::move(m49));
            t44.push_back(std::move(e48));
            gr::pmt::Value e51;
            gr::property_map m52;
            gr::property_map m53;
            m53[std::pmr::string("trigger_resets")] = gr::pmt::Value(false);
            m53[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{8});
            m53[std::pmr::string("escape_map")] = gr::pmt::Value(gr::Tensor<std::uint32_t>(gr::data_from, {std::uint32_t{220}, std::uint32_t{192}, std::uint32_t{221}, std::uint32_t{219}}));
            m53[std::pmr::string("escape_item")] = gr::pmt::Value(std::uint32_t{219});
            m53[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::pmr::string("=max_payload_items"));
            m53[std::pmr::string("transparency")] = gr::pmt::Value(std::pmr::string("byte_escape"));
            m53[std::pmr::string("end_delimiter")] = gr::pmt::Value(std::pmr::string("11000000"));
            m53[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("extractor"));
            m52[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m53));
            m52[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::DelimiterExtractor<uint8>"));
            e51 = gr::pmt::Value(std::move(m52));
            t44.push_back(std::move(e51));
            gr::pmt::Value e54;
            gr::property_map m55;
            gr::property_map m56;
            m56[std::pmr::string("read_timestamp")] = gr::pmt::Value(std::pmr::string("=read_timestamp"));
            m56[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss"));
            m55[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m56));
            m55[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ax25::KissDecode"));
            e54 = gr::pmt::Value(std::move(m55));
            t44.push_back(std::move(e54));
            m11[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t44));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m11));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m57;
            m57[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m57[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m57[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m57[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m57[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::KissStreamDecode"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m57));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("drop_head")] = parameters.drop_head;
        values[std::pmr::string("max_payload_items")] = parameters.max_payload_items;
        values[std::pmr::string("read_timestamp")] = parameters.read_timestamp;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_KISSSTREAMDECODE_HPP
