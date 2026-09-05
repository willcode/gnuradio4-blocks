// GENERATED FILE — do not edit. Source of truth: blocks/recipes/KissFileRead.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_KISSFILEREAD_HPP
#define GNURADIO_RECIPES_KISSFILEREAD_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct KissFileRead {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::string file_name_, std::uint32_t max_payload_items_) : file_name(std::move(file_name_)), max_payload_items(std::move(max_payload_items_)) {}
        std::string file_name; // the KISS file's path; required, there is no default
        std::uint32_t max_payload_items; // DelimiterExtractor's own bound on a decoded frame's items; required, there is no default
        bool read_timestamp = false; // interpret a command-9 control frame as a timestamp for the next data frame; KissDecode's own default
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::KissFileRead";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the KISS file's path; required, there is no default"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("file_name"));
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
            m10[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss_file_read"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m10));
            gr::property_map m11;
            gr::Tensor<gr::pmt::Value> t12;
            gr::pmt::Value e13;
            gr::Tensor<gr::pmt::Value> t14;
            gr::pmt::Value e15;
            e15 = gr::pmt::Value(std::pmr::string("kiss"));
            t14.push_back(std::move(e15));
            gr::pmt::Value e16;
            e16 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t14.push_back(std::move(e16));
            gr::pmt::Value e17;
            e17 = gr::pmt::Value(std::pmr::string("out"));
            t14.push_back(std::move(e17));
            gr::pmt::Value e18;
            e18 = gr::pmt::Value(std::pmr::string("out"));
            t14.push_back(std::move(e18));
            e13 = gr::pmt::Value(std::move(t14));
            t12.push_back(std::move(e13));
            m11[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t12));
            gr::Tensor<gr::pmt::Value> t19;
            gr::pmt::Value e20;
            gr::Tensor<gr::pmt::Value> t21;
            gr::pmt::Value e22;
            e22 = gr::pmt::Value(std::pmr::string("file"));
            t21.push_back(std::move(e22));
            gr::pmt::Value e23;
            e23 = gr::pmt::Value(std::int64_t{0});
            t21.push_back(std::move(e23));
            gr::pmt::Value e24;
            e24 = gr::pmt::Value(std::pmr::string("extractor"));
            t21.push_back(std::move(e24));
            gr::pmt::Value e25;
            e25 = gr::pmt::Value(std::int64_t{0});
            t21.push_back(std::move(e25));
            e20 = gr::pmt::Value(std::move(t21));
            t19.push_back(std::move(e20));
            gr::pmt::Value e26;
            gr::Tensor<gr::pmt::Value> t27;
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("extractor"));
            t27.push_back(std::move(e28));
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("kiss"));
            t27.push_back(std::move(e30));
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e31));
            e26 = gr::pmt::Value(std::move(t27));
            t19.push_back(std::move(e26));
            m11[std::pmr::string("connections")] = gr::pmt::Value(std::move(t19));
            gr::Tensor<gr::pmt::Value> t32;
            gr::pmt::Value e33;
            gr::property_map m34;
            gr::property_map m35;
            m35[std::pmr::string("file_name")] = gr::pmt::Value(std::pmr::string("=file_name"));
            m35[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("file"));
            m34[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m35));
            m34[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::fileio::BasicFileSource<uint8>"));
            e33 = gr::pmt::Value(std::move(m34));
            t32.push_back(std::move(e33));
            gr::pmt::Value e36;
            gr::property_map m37;
            gr::property_map m38;
            m38[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{8});
            m38[std::pmr::string("escape_map")] = gr::pmt::Value(gr::Tensor<std::uint32_t>(gr::data_from, {std::uint32_t{220}, std::uint32_t{192}, std::uint32_t{221}, std::uint32_t{219}}));
            m38[std::pmr::string("escape_item")] = gr::pmt::Value(std::uint32_t{219});
            m38[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::pmr::string("=max_payload_items"));
            m38[std::pmr::string("transparency")] = gr::pmt::Value(std::pmr::string("byte_escape"));
            m38[std::pmr::string("end_delimiter")] = gr::pmt::Value(std::pmr::string("11000000"));
            m38[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("extractor"));
            m37[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m38));
            m37[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::DelimiterExtractor<uint8>"));
            e36 = gr::pmt::Value(std::move(m37));
            t32.push_back(std::move(e36));
            gr::pmt::Value e39;
            gr::property_map m40;
            gr::property_map m41;
            m41[std::pmr::string("read_timestamp")] = gr::pmt::Value(std::pmr::string("=read_timestamp"));
            m41[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss"));
            m40[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m41));
            m40[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ax25::KissDecode"));
            e39 = gr::pmt::Value(std::move(m40));
            t32.push_back(std::move(e39));
            m11[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t32));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m11));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m42;
            m42[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m42[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m42[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m42[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m42[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::KissFileRead"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m42));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("file_name")] = std::pmr::string(parameters.file_name);
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

#endif // GNURADIO_RECIPES_KISSFILEREAD_HPP
