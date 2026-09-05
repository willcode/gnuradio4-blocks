#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <print>
#include <sstream>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

/*
 * Load one graph file and run it until its sources are finished.
 *
 * Nothing here knows anything about any block family: a graph file names its blocks by their registered
 * identifiers and the plugin loader finds them, so this program is the shell entry point every graph file in the
 * tree has been missing. It is deliberately the whole of what it does - there is no argument for a duration, a
 * sample count or a sink to poll, because a graph that should stop says so through the blocks it is built from,
 * and one that should be watched wants the control plane rather than a process that exits.
 *
 * It exits through _Exit after flushing, because the runtime otherwise unloads block plugins while the scheduler's
 * own threads are still winding down.
 */
namespace {

[[noreturn]] void leave(int status) {
    std::fflush(nullptr);
    std::_Exit(status);
}

} // namespace

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc != 2) {
        std::println(stderr, "usage: {} <graph-file>", argc > 0 ? argv[0] : "gr4-run-graph");
        leave(64);
    }

    std::ifstream file(argv[1], std::ios::binary);
    if (!file) {
        std::println(stderr, "gr4-run-graph: cannot read '{}'", argv[1]);
        leave(66);
    }
    std::ostringstream text;
    text << file.rdbuf();

    try {
        auto graph = gr::loadGrc(gr::globalPluginLoader(), text.str());
        if (graph.valueless_after_move()) {
            std::println(stderr, "gr4-run-graph: '{}' produced no graph", argv[1]);
            leave(65);
        }

        gr::scheduler::Simple<> scheduler;
        if (const auto exchanged = scheduler.exchange(std::move(*graph)); !exchanged) {
            std::println(stderr, "gr4-run-graph: {}", exchanged.error().message);
            leave(70);
        }
        if (const auto ran = scheduler.runAndWait(); !ran) {
            std::println(stderr, "gr4-run-graph: {}", ran.error().message);
            leave(70);
        }
    } catch (const std::exception& error) {
        std::println(stderr, "gr4-run-graph: {}", error.what());
        leave(65);
    }
    leave(0);
}
