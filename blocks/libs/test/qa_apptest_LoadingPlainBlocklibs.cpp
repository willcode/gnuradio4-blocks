#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <format>
#include <iterator>
#include <print>
#include <string>

using namespace std::literals;

// This tests automatic loading of .so files that are not
// plugins, but ordinary dynamic block libraries -- they
// just need to be in a path that the PluginLoader searches in.
//
// This is intentionally not a ut test as it tests
// how a normal application would use the global block
// registry in the lifetime of main

int main() {
    gr::globalPluginLoader();

    auto known = gr::globalBlockRegistry().keys();
    std::ranges::sort(known);
    std::vector<std::string> desired{
        //
        "gr::blocks::basic::DataSink<float32>"s,                //
        "gr::blocks::basic::DataSetSink<float32>"s,             //
        "gr::blocks::basic::FunctionGenerator<int16>"s,         //
        "gr::blocks::basic::FunctionGenerator<float32>"s,       //
        "gr::blocks::basic::Selector<int32>"s,                  //
        "gr::blocks::basic::Selector<float32>"s,                //
        "gr::blocks::basic::SignalGenerator<float32>"s,         //
        "gr::blocks::basic::SignalGenerator<complex<float32>>"s //
    };
    std::ranges::sort(desired);

    std::vector<std::string> missing;
    std::ranges::set_difference(desired, known, std::back_inserter(missing));
    if (!missing.empty()) {
        for (const auto& name : missing) {
            std::println(stderr, "missing block registration: {}", name);
        }
        return EXIT_FAILURE;
    }

    // A marker that fixes a template parameter -- Add's std::plus<T> -- registers the block under
    // the whole parameter list and under the short alias, its name over the type alone. Both are
    // keys, and the short one is what a flowgraph file writes, so the importer must build a graph
    // from it: this is the one place these libraries are exercised through a factory rather than
    // read off the key list.
    const std::string shortAlias = "gr::blocks::math::Add<float32>"s;
    for (const auto& name : {shortAlias, "gr::blocks::math::Add<float32, std::plus<float32>>"s}) {
        if (!gr::globalBlockRegistry().contains(name)) {
            std::println(stderr, "missing block registration: {}", name);
            return EXIT_FAILURE;
        }
    }

    try {
        const auto graph = gr::loadGrc(gr::globalPluginLoader(), std::format("blocks:\n  - id: {}\n    parameters:\n      name: sum\n", shortAlias));
        if (graph->blocks().size() != 1UZ) {
            std::println(stderr, "the importer built {} block(s) from a one-block graph", graph->blocks().size());
            return EXIT_FAILURE;
        }
        std::println("the importer built {} from {}", graph->blocks().front()->typeName(), shortAlias);
    } catch (const std::exception& error) {
        std::println(stderr, "the importer rejected {}: {}", shortAlias, error.what());
        return EXIT_FAILURE;
    }

    std::println("All ok");
}
