#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <print>
#include <string>
#include <vector>

using namespace std::literals;

// The SDR family's block library is an ordinary shared library rather than a plugin: the loader opens every
// library in the directories it scans, and the library's registration units enter its blocks into the global
// registry as they are mapped. That is the only way a radio block reaches a graph built from a saved
// description, which names a block by its registry key.
//
// This is intentionally not a ut test, for the same reason as qa_apptest_LoadingPlainBlocklibs: it asks what an
// ordinary application sees of the global registry over the lifetime of main. It opens no device -- the registry
// is read, and no block is created.

int main() {
    gr::globalPluginLoader();

    auto known = gr::globalBlockRegistry().keys();
    std::ranges::sort(known);

    for (const auto& name : known) {
        if (name.contains("::sdr::")) {
            std::println("registered: {}", name);
        }
    }

    std::vector<std::string> desired{
        "gr::blocks::sdr::SoapySink<complex<float32>>"s,   //
        "gr::blocks::sdr::SoapySink<int16>"s,              //
        "gr::blocks::sdr::SoapySink<uint8>"s,              //
        "gr::blocks::sdr::SoapySource<complex<float32>>"s, //
        "gr::blocks::sdr::SoapySource<int16>"s,            //
        "gr::blocks::sdr::SoapySource<uint8>"s             //
    };
    std::ranges::sort(desired);

    std::vector<std::string> missing;
    std::ranges::set_difference(desired, known, std::back_inserter(missing));
    if (!missing.empty()) {
        if (const char* directories = std::getenv("GNURADIO4_PLUGIN_DIRECTORIES"); directories == nullptr) {
            std::println(stderr, "GNURADIO4_PLUGIN_DIRECTORIES is not set: the loader scanned only {}. ctest sets the variable to the directory holding the SDR block library.", gr::installedPluginDirectory());
        } else if (std::ranges::none_of(known, [](const std::string& name) { return name.contains("::sdr::"); })) {
            std::println(stderr, "no SDR block library was mapped from GNURADIO4_PLUGIN_DIRECTORIES={}", directories);
        }
        for (const auto& name : missing) {
            std::println(stderr, "missing block registration: {}", name);
        }
        return EXIT_FAILURE;
    }
    std::println("All ok");
}
