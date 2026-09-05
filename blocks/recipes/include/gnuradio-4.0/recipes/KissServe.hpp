// GENERATED FILE — do not edit. Source of truth: blocks/recipes/KissServe.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_KISSSERVE_HPP
#define GNURADIO_RECIPES_KISSSERVE_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct KissServe {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(std::string endpoint_, std::uint32_t queue_bytes_) : endpoint(std::move(endpoint_)), queue_bytes(std::move(queue_bytes_)) {}
        std::string endpoint; // 'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default
        bool bind = true; // listen on the endpoint rather than connect to it; TcpByteSink's own default
        std::string overflow = std::string("drop_oldest"); // 'drop_oldest' or 'backpressure', applied when the in-process byte queue is full; TcpByteSink's own default
        std::uint32_t queue_bytes; // the in-process byte queue's bound; required, there is no default
        std::uint32_t kiss_port = std::uint32_t{0}; // the terminal node controller port a frame is for, 0 to 15; KissEncode's own default
        bool emit_timestamp = false; // publish a command-9 timestamp frame ahead of each data frame whose record's DataSet::timestamp is non-zero
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::KissServe";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("'host:port', e.g. 127.0.0.1:5555 or [::1]:5555; required, there is no default"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("endpoint"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("listen on the endpoint rather than connect to it; TcpByteSink's own default"));
            m7[std::pmr::string("default")] = gr::pmt::Value(true);
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("bool"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("bind"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("'drop_oldest' or 'backpressure', applied when the in-process byte queue is full; TcpByteSink's own default"));
            m9[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("drop_oldest"));
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("overflow"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the in-process byte queue's bound; required, there is no default"));
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("queue_bytes"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the terminal node controller port a frame is for, 0 to 15; KissEncode's own default"));
            m13[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{0});
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss_port"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("publish a command-9 timestamp frame ahead of each data frame whose record's DataSet::timestamp is non-zero"));
            m15[std::pmr::string("default")] = gr::pmt::Value(false);
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("bool"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("emit_timestamp"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m16;
            m16[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss_serve"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m16));
            gr::property_map m17;
            gr::Tensor<gr::pmt::Value> t18;
            gr::pmt::Value e19;
            gr::Tensor<gr::pmt::Value> t20;
            gr::pmt::Value e21;
            e21 = gr::pmt::Value(std::pmr::string("kiss"));
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
            m17[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t18));
            gr::Tensor<gr::pmt::Value> t25;
            gr::pmt::Value e26;
            gr::Tensor<gr::pmt::Value> t27;
            gr::pmt::Value e28;
            e28 = gr::pmt::Value(std::pmr::string("kiss"));
            t27.push_back(std::move(e28));
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("framer"));
            t27.push_back(std::move(e30));
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::int64_t{0});
            t27.push_back(std::move(e31));
            e26 = gr::pmt::Value(std::move(t27));
            t25.push_back(std::move(e26));
            gr::pmt::Value e32;
            gr::Tensor<gr::pmt::Value> t33;
            gr::pmt::Value e34;
            e34 = gr::pmt::Value(std::pmr::string("framer"));
            t33.push_back(std::move(e34));
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e35));
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("flatten"));
            t33.push_back(std::move(e36));
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::int64_t{0});
            t33.push_back(std::move(e37));
            e32 = gr::pmt::Value(std::move(t33));
            t25.push_back(std::move(e32));
            gr::pmt::Value e38;
            gr::Tensor<gr::pmt::Value> t39;
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("flatten"));
            t39.push_back(std::move(e40));
            gr::pmt::Value e41;
            e41 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e41));
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("tcp"));
            t39.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::int64_t{0});
            t39.push_back(std::move(e43));
            e38 = gr::pmt::Value(std::move(t39));
            t25.push_back(std::move(e38));
            m17[std::pmr::string("connections")] = gr::pmt::Value(std::move(t25));
            gr::Tensor<gr::pmt::Value> t44;
            gr::pmt::Value e45;
            gr::property_map m46;
            gr::property_map m47;
            m47[std::pmr::string("emit_timestamp")] = gr::pmt::Value(std::pmr::string("=emit_timestamp"));
            m47[std::pmr::string("kiss_port")] = gr::pmt::Value(std::pmr::string("=kiss_port"));
            m47[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("kiss"));
            m46[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m47));
            m46[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::ax25::KissEncode"));
            e45 = gr::pmt::Value(std::move(m46));
            t44.push_back(std::move(e45));
            gr::pmt::Value e48;
            gr::property_map m49;
            gr::property_map m50;
            m50[std::pmr::string("bits_per_item")] = gr::pmt::Value(std::uint32_t{8});
            m50[std::pmr::string("escape_map")] = gr::pmt::Value(gr::Tensor<std::uint32_t>(gr::data_from, {std::uint32_t{220}, std::uint32_t{192}, std::uint32_t{221}, std::uint32_t{219}}));
            m50[std::pmr::string("escape_item")] = gr::pmt::Value(std::uint32_t{219});
            m50[std::pmr::string("max_payload_items")] = gr::pmt::Value(std::uint32_t{4096});
            m50[std::pmr::string("transparency")] = gr::pmt::Value(std::pmr::string("byte_escape"));
            m50[std::pmr::string("end_delimiter")] = gr::pmt::Value(std::pmr::string("11000000"));
            m50[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("framer"));
            m49[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m50));
            m49[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::digital::DelimiterFramer"));
            e48 = gr::pmt::Value(std::move(m49));
            t44.push_back(std::move(e48));
            gr::pmt::Value e51;
            gr::property_map m52;
            gr::property_map m53;
            m53[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("flatten"));
            m52[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m53));
            m52[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::DataSetToStream<uint8>"));
            e51 = gr::pmt::Value(std::move(m52));
            t44.push_back(std::move(e51));
            gr::pmt::Value e54;
            gr::property_map m55;
            gr::property_map m56;
            m56[std::pmr::string("queue_bytes")] = gr::pmt::Value(std::pmr::string("=queue_bytes"));
            m56[std::pmr::string("overflow")] = gr::pmt::Value(std::pmr::string("=overflow"));
            m56[std::pmr::string("bind")] = gr::pmt::Value(std::pmr::string("=bind"));
            m56[std::pmr::string("endpoint")] = gr::pmt::Value(std::pmr::string("=endpoint"));
            m56[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("tcp"));
            m55[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m56));
            m55[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::network::TcpByteSink"));
            e54 = gr::pmt::Value(std::move(m55));
            t44.push_back(std::move(e54));
            m17[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t44));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m17));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m57;
            m57[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m57[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m57[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m57[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m57[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::KissServe"));
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
        values[std::pmr::string("endpoint")] = std::pmr::string(parameters.endpoint);
        values[std::pmr::string("bind")] = parameters.bind;
        values[std::pmr::string("overflow")] = std::pmr::string(parameters.overflow);
        values[std::pmr::string("queue_bytes")] = parameters.queue_bytes;
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

#endif // GNURADIO_RECIPES_KISSSERVE_HPP
