// GENERATED FILE — do not edit. Source of truth: blocks/recipes/KissFileWrite.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_KISSFILEWRITE_HPP
#define GNURADIO_RECIPES_KISSFILEWRITE_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct KissFileWrite {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::string file_name_) : file_name(std::move(file_name_)) {}
        std::string file_name; // the KISS file's path; required, there is no default
        std::string mode = std::string("overwrite"); // BasicFileSink's own: 'overwrite', 'append' or 'multi'
        std::uint32_t kiss_port = std::uint32_t{0}; // the terminal node controller port a frame is for, 0 to 15; KissEncode's own default
        bool emit_timestamp = false; // publish a command-9 timestamp frame ahead of each data frame whose record's DataSet::timestamp is non-zero
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::KissFileWrite";
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
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("BasicFileSink's own: 'overwrite', 'append' or 'multi'"));
            m7[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("overwrite"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("mode"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the terminal node controller port a frame is for, 0 to 15; KissEncode's own default"));
            m9[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{0});
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss_port"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("publish a command-9 timestamp frame ahead of each data frame whose record's DataSet::timestamp is non-zero"));
            m11[std::pmr::string("default")] = gr::pmt::Value(false);
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("bool"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("emit_timestamp"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m12;
            m12[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss_file_write"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m12));
            gr::property_map m13;
            gr::Tensor<gr::pmt::Value> t14;
            gr::pmt::Value e15;
            gr::Tensor<gr::pmt::Value> t16;
            gr::pmt::Value e17;
            e17 = gr::pmt::Value(std::pmr::string("kiss"));
            t16.push_back(std::move(e17));
            gr::pmt::Value e18;
            e18 = gr::pmt::Value(std::pmr::string("INPUT"));
            t16.push_back(std::move(e18));
            gr::pmt::Value e19;
            e19 = gr::pmt::Value(std::pmr::string("in"));
            t16.push_back(std::move(e19));
            gr::pmt::Value e20;
            e20 = gr::pmt::Value(std::pmr::string("in"));
            t16.push_back(std::move(e20));
            e15 = gr::pmt::Value(std::move(t16));
            t14.push_back(std::move(e15));
            m13[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t14));
            gr::Tensor<gr::pmt::Value> t21;
            gr::pmt::Value e22;
            gr::Tensor<gr::pmt::Value> t23;
            gr::pmt::Value e24;
            e24 = gr::pmt::Value(std::pmr::string("kiss"));
            t23.push_back(std::move(e24));
            gr::pmt::Value e25;
            e25 = gr::pmt::Value(std::int64_t{0});
            t23.push_back(std::move(e25));
            gr::pmt::Value e26;
            e26 = gr::pmt::Value(std::pmr::string("framer"));
            t23.push_back(std::move(e26));
            gr::pmt::Value e27;
            e27 = gr::pmt::Value(std::int64_t{0});
            t23.push_back(std::move(e27));
            e22 = gr::pmt::Value(std::move(t23));
            t21.push_back(std::move(e22));
            gr::pmt::Value e28;
            gr::Tensor<gr::pmt::Value> t29;
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("framer"));
            t29.push_back(std::move(e30));
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::int64_t{0});
            t29.push_back(std::move(e31));
            gr::pmt::Value e32;
            e32 = gr::pmt::Value(std::pmr::string("flatten"));
            t29.push_back(std::move(e32));
            gr::pmt::Value e33;
            e33 = gr::pmt::Value(std::int64_t{0});
            t29.push_back(std::move(e33));
            e28 = gr::pmt::Value(std::move(t29));
            t21.push_back(std::move(e28));
            gr::pmt::Value e34;
            gr::Tensor<gr::pmt::Value> t35;
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("flatten"));
            t35.push_back(std::move(e36));
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::int64_t{0});
            t35.push_back(std::move(e37));
            gr::pmt::Value e38;
            e38 = gr::pmt::Value(std::pmr::string("file"));
            t35.push_back(std::move(e38));
            gr::pmt::Value e39;
            e39 = gr::pmt::Value(std::int64_t{0});
            t35.push_back(std::move(e39));
            e34 = gr::pmt::Value(std::move(t35));
            t21.push_back(std::move(e34));
            m13[std::pmr::string("connections")] = gr::pmt::Value(std::move(t21));
            gr::Tensor<gr::pmt::Value> t40;
            gr::pmt::Value e41;
            gr::property_map m42;
            gr::property_map m43;
            m43[std::pmr::string("emit_timestamp")] = gr::pmt::Value(std::pmr::string("=emit_timestamp"));
            m43[std::pmr::string("kiss_port")] = gr::pmt::Value(std::pmr::string("=kiss_port"));
            m43[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss"));
            m42[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m43));
            m42[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ax25::KissEncode"));
            e41 = gr::pmt::Value(std::move(m42));
            t40.push_back(std::move(e41));
            gr::pmt::Value e44;
            gr::property_map m45;
            gr::property_map m46;
            m46[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{8});
            m46[std::pmr::string("escape_map")] = gr::pmt::Value(gr::Tensor<std::uint32_t>(gr::data_from, {std::uint32_t{220}, std::uint32_t{192}, std::uint32_t{221}, std::uint32_t{219}}));
            m46[std::pmr::string("escape_item")] = gr::pmt::Value(std::uint32_t{219});
            m46[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::uint32_t{4096});
            m46[std::pmr::string("transparency")] = gr::pmt::Value(std::pmr::string("byte_escape"));
            m46[std::pmr::string("end_delimiter")] = gr::pmt::Value(std::pmr::string("11000000"));
            m46[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("framer"));
            m45[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m46));
            m45[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::DelimiterFramer"));
            e44 = gr::pmt::Value(std::move(m45));
            t40.push_back(std::move(e44));
            gr::pmt::Value e47;
            gr::property_map m48;
            gr::property_map m49;
            m49[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("flatten"));
            m48[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m49));
            m48[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::DataSetToStream<uint8>"));
            e47 = gr::pmt::Value(std::move(m48));
            t40.push_back(std::move(e47));
            gr::pmt::Value e50;
            gr::property_map m51;
            gr::property_map m52;
            m52[std::pmr::string("mode")] = gr::pmt::Value(std::pmr::string("=mode"));
            m52[std::pmr::string("file_name")] = gr::pmt::Value(std::pmr::string("=file_name"));
            m52[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("file"));
            m51[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m52));
            m51[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::fileio::BasicFileSink<uint8>"));
            e50 = gr::pmt::Value(std::move(m51));
            t40.push_back(std::move(e50));
            m13[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t40));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m13));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m53;
            m53[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m53[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m53[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m53[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m53[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::KissFileWrite"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m53));
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
        values[std::pmr::string("mode")] = std::pmr::string(parameters.mode);
        values[std::pmr::string("kiss_port")] = parameters.kiss_port;
        values[std::pmr::string("emit_timestamp")] = parameters.emit_timestamp;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_KISSFILEWRITE_HPP
