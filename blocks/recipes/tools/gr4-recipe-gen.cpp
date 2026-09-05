/* gr4-recipe-gen <recipes-dir> <out-include-dir>
 *
 * Reads the recipe catalog (index.yaml + definitions) and writes one generated typed
 * header per recipe into <out-include-dir>/gnuradio-4.0/recipes/. The YAML is the source
 * of truth; the emitted headers are committed and qa_Recipes diffs them against a fresh
 * emission, so running this tool is how a recipe change reaches the C++ surface. */
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "RecipeHeaderEmitter.hpp"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <recipes-dir> <out-include-dir>\n", argv[0]);
        return 2;
    }
    const std::vector<std::string>    paths{std::string(argv[1])};
    gr::detail::YamlDefinitionsLoader catalog{std::span<const std::string>(paths)};
    const std::filesystem::path       outDir = std::filesystem::path(argv[2]) / "gnuradio-4.0" / "recipes";
    std::error_code                   ec;
    std::filesystem::create_directories(outDir, ec);
    if (ec) {
        std::fprintf(stderr, "cannot create %s: %s\n", outDir.string().c_str(), ec.message().c_str());
        return 1;
    }

    int emitted = 0;
    for (const auto& [name, definition] : catalog._definitionForBlockName) {
        const auto lastColon  = name.rfind("::");
        const auto structName = lastColon == std::string::npos ? name : name.substr(lastColon + 2);
        const auto header     = gr::recipe::emitter::emitRecipeHeader(definition, structName + ".yaml");
        if (!header.has_value()) {
            std::fprintf(stderr, "%s: %s\n", name.c_str(), header.error().message.c_str());
            return 1;
        }
        const auto    outPath = outDir / (structName + ".hpp");
        std::ofstream out(outPath, std::ios::binary | std::ios::trunc);
        if (!out) {
            std::fprintf(stderr, "cannot write %s\n", outPath.string().c_str());
            return 1;
        }
        out << *header;
        std::printf("%s -> %s\n", name.c_str(), outPath.string().c_str());
        ++emitted;
    }
    if (emitted == 0) {
        std::fprintf(stderr, "no recipes found under %s\n", argv[1]);
        return 1;
    }
    return 0;
}
