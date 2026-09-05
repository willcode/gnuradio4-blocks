#ifndef GNURADIO_RECIPEHEADEREMITTER_HPP
#define GNURADIO_RECIPEHEADEREMITTER_HPP

// Emits the typed C++ header for one recipe definition. The YAML recipe is the single
// source of truth; the emitted header is a committed artifact a qa regenerates and diffs,
// so drift between the two is a red test rather than a mystery. The generated emplace()
// builds the definition as a property-map literal — no YAML text, no file I/O, no parser
// at run time — and hands it to the same instantiation path the loader uses, so the two
// front ends cannot disagree about semantics.

#include <algorithm>
#include <cctype>
#include <expected>
#include <format>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Graph_yaml_importer.hpp>

namespace gr::recipe::emitter {

namespace detail {

[[nodiscard]] inline std::string cppTypeFor(std::string_view typeWord);

/// @brief Whether a type word names a vector, which the dialect writes as an element word and a `[]`.
[[nodiscard]] inline bool vectorTypeWord(std::string_view typeWord) noexcept { return typeWord.size() > 2UZ && typeWord.ends_with("[]"); }

[[nodiscard]] inline std::string cppTypeFor(std::string_view typeWord) {
    if (vectorTypeWord(typeWord)) {
        return std::format("std::vector<{}>", cppTypeFor(typeWord.substr(0UZ, typeWord.size() - 2UZ)));
    }
    if (typeWord == "float32") {
        return "float";
    }
    if (typeWord == "float64") {
        return "double";
    }
    if (typeWord == "bool") {
        return "bool";
    }
    if (typeWord == "string") {
        return "std::string";
    }
    return std::format("std::{}_t", typeWord); // int8..uint64
}

[[nodiscard]] inline std::string escapeStringLiteral(std::string_view text) {
    std::string out;
    out.reserve(text.size() + 2);
    for (const char c : text) {
        if (c == '\\' || c == '"') {
            out.push_back('\\');
        }
        out.push_back(c);
    }
    return out;
}

/// one scalar Value as the exact C++ expression that reconstructs it, type and all
[[nodiscard]] inline std::expected<std::string, gr::Error> scalarExpression(const gr::pmt::Value& value) {
    if (const auto* v = value.get_if<bool>()) {
        return std::string(*v ? "true" : "false");
    }
    if (const auto* v = value.get_if<float>()) {
        // std::format is shortest-round-trip, so the text re-parses exactly, but it drops the
        // point on an integral value and `60f` is not a literal: put the point back before the suffix
        const std::string text = std::format("{}", *v);
        return text.find_first_of(".eE") == std::string::npos ? text + ".0f" : text + "f";
    }
    if (const auto* v = value.get_if<double>()) {
        const std::string text = std::format("{}", *v);
        return text.find_first_of(".eE") == std::string::npos ? text + ".0" : text;
    }
    if (const auto* v = value.get_if<std::int8_t>()) {
        return std::format("std::int8_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::int16_t>()) {
        return std::format("std::int16_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::int32_t>()) {
        return std::format("std::int32_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::int64_t>()) {
        return std::format("std::int64_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::uint8_t>()) {
        return std::format("std::uint8_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::uint16_t>()) {
        return std::format("std::uint16_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::uint32_t>()) {
        return std::format("std::uint32_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::uint64_t>()) {
        return std::format("std::uint64_t{{{}}}", *v);
    }
    if (const auto* v = value.get_if<std::pmr::string>()) {
        return std::format("std::pmr::string(\"{}\")", escapeStringLiteral(std::string_view(*v)));
    }
    return std::unexpected(gr::Error("recipe emitter: a definition value has a type the emitter does not carry"));
}

/// @brief A vector default as a braced initializer of the element literals `scalarExpression` writes.
///
/// The element types are the numeric ones the dialect names. `string[]` is refused rather than emitted, because a
/// string element's literal is a `std::pmr::string` and the member it would initialize is a `std::vector<std::string>`.
template<typename... TElements>
[[nodiscard]] inline std::expected<std::string, gr::Error> vectorExpressionAs(const gr::pmt::Value& value, const std::string& cppType) {
    std::string joined;
    bool        matched = false;
    auto        collect = [&](auto tag) -> std::expected<void, gr::Error> {
        using T = decltype(tag);
        if (matched) {
            return {};
        }
        const auto* tensor = value.get_if<gr::Tensor<T>>();
        if (tensor == nullptr) {
            return {};
        }
        matched = true;
        for (const auto& element : *tensor) {
            auto literal = scalarExpression(gr::pmt::Value(static_cast<T>(element)));
            if (!literal.has_value()) {
                return std::unexpected(literal.error());
            }
            joined += std::format("{}{}", joined.empty() ? "" : ", ", *literal);
        }
        return {};
    };
    std::expected<void, gr::Error> outcome{};
    ((outcome = outcome.has_value() ? collect(TElements{}) : outcome), ...);
    if (!outcome.has_value()) {
        return std::unexpected(outcome.error());
    }
    if (!matched) {
        return std::unexpected(gr::Error("recipe emitter: a vector default has an element type the emitter does not carry"));
    }
    return std::format("{}{{{}}}", cppType, joined);
}

[[nodiscard]] inline std::expected<std::string, gr::Error> vectorExpression(const gr::pmt::Value& value, const std::string& cppType) { //
    return vectorExpressionAs<bool, float, double, std::int8_t, std::int16_t, std::int32_t, std::int64_t, std::uint8_t, std::uint16_t, std::uint32_t, std::uint64_t>(value, cppType);
}

/// @brief One homogeneous rank-1 `gr::Tensor<T>`, as the C++ expression that reconstructs it exactly.
///
/// A YAML sequence with a whole-list type tag (`escape_map: !!uint32 [220, 192, 221, 219]`) parses to this shape
/// rather than to the heterogeneous `gr::Tensor<gr::pmt::Value>` the untagged-list branch above handles, and it is
/// what a `std::vector<T>`-typed interior block setting's conversion requires exactly: `Settings.hpp`'s
/// `convertParameter` looks for `Tensor<TTensorElem>`, not `Tensor<Value>`. Rank 1 is the only shape with a literal
/// spelling here — `gr::data_from` builds rank 1 and throws on anything else — so a tensor of higher rank is
/// declined rather than reshaped behind the reader's back. An empty tensor takes the default constructor, because an
/// empty braced list deduces no element type and `gr::Tensor<T>(gr::data_from, {})` does not compile.
///
/// `nullopt` means this function has no expression for @p value: another element type may still have one, and when
/// none does the caller refuses the value rather than emitting something that does not reconstruct it.
template<typename T>
[[nodiscard]] inline std::optional<std::string> numericTensorExpressionOf(const gr::pmt::Value& value, std::string_view typeName) {
    const auto* tensor = value.get_if<gr::Tensor<T>>();
    if (tensor == nullptr || tensor->rank() > 1UZ) {
        return std::nullopt;
    }
    if (tensor->size() == 0UZ) {
        return std::format("gr::Tensor<{}>()", typeName);
    }
    std::string joined;
    for (const auto& element : *tensor) {
        const auto literal = scalarExpression(gr::pmt::Value(static_cast<T>(element)));
        if (!literal.has_value()) {
            return std::nullopt;
        }
        joined += std::format("{}{}", joined.empty() ? "" : ", ", *literal);
    }
    return std::format("gr::Tensor<{}>(gr::data_from, {{{}}})", typeName, joined);
}

[[nodiscard]] inline std::optional<std::string> numericTensorExpression(const gr::pmt::Value& value) {
    if (auto r = numericTensorExpressionOf<bool>(value, "bool")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<float>(value, "float")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<double>(value, "double")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::int8_t>(value, "std::int8_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::int16_t>(value, "std::int16_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::int32_t>(value, "std::int32_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::int64_t>(value, "std::int64_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::uint8_t>(value, "std::uint8_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::uint16_t>(value, "std::uint16_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::uint32_t>(value, "std::uint32_t")) {
        return r;
    }
    if (auto r = numericTensorExpressionOf<std::uint64_t>(value, "std::uint64_t")) {
        return r;
    }
    return std::nullopt;
}

/// emits statements reconstructing `value` into the named local; containers recurse with
/// uniquely-numbered locals
inline std::expected<void, gr::Error> emitValueInto(std::string& out, const gr::pmt::Value& value, const std::string& target, int& counter, const std::string& indent) {
    if (const auto* map = value.get_if<gr::property_map>()) {
        const std::string local = std::format("m{}", counter++);
        out += std::format("{}gr::property_map {};\n", indent, local);
        for (const auto& [key, entry] : *map) {
            if (auto emitted = emitValueInto(out, entry, std::format("{}[std::pmr::string(\"{}\")]", local, escapeStringLiteral(std::string_view(key))), counter, indent); !emitted.has_value()) {
                return emitted;
            }
        }
        out += std::format("{}{} = gr::pmt::Value(std::move({}));\n", indent, target, local);
        return {};
    }
    if (const auto* tensor = value.get_if<gr::Tensor<gr::pmt::Value>>()) {
        const std::string local = std::format("t{}", counter++);
        out += std::format("{}gr::Tensor<gr::pmt::Value> {};\n", indent, local);
        for (const auto& element : *tensor) {
            const std::string elementLocal = std::format("e{}", counter++);
            out += std::format("{}gr::pmt::Value {};\n", indent, elementLocal);
            if (auto emitted = emitValueInto(out, element, elementLocal, counter, indent); !emitted.has_value()) {
                return emitted;
            }
            out += std::format("{}{}.push_back(std::move({}));\n", indent, local, elementLocal);
        }
        out += std::format("{}{} = gr::pmt::Value(std::move({}));\n", indent, target, local);
        return {};
    }
    if (const auto numeric = numericTensorExpression(value); numeric.has_value()) {
        out += std::format("{}{} = gr::pmt::Value({});\n", indent, target, *numeric);
        return {};
    }
    auto scalar = scalarExpression(value);
    if (!scalar.has_value()) {
        return std::unexpected(scalar.error());
    }
    out += std::format("{}{} = gr::pmt::Value({});\n", indent, target, *scalar);
    return {};
}

} // namespace detail

/// The whole generated header for one definition, or the reason it cannot be emitted.
[[nodiscard]] inline std::expected<std::string, gr::Error> emitRecipeHeader(const gr::detail::YamlDefinitionsLoader::Definition& definition, std::string_view sourceFileName) {
    const std::string blockType  = definition.metadata.block_type;
    const auto        lastColon  = blockType.rfind("::");
    const std::string structName = lastColon == std::string::npos ? blockType : blockType.substr(lastColon + 2);
    if (structName.empty()) {
        return std::unexpected(gr::Error("recipe emitter: the definition names no block_type"));
    }

    std::vector<gr::recipe::ParameterDeclaration> declarations;
    if (const auto blocksIt = definition.definition.find("blocks"); blocksIt != definition.definition.end()) {
        if (const auto* blocks = blocksIt->second.get_if<gr::Tensor<gr::pmt::Value>>()) {
            for (const auto& entry : *blocks) {
                const auto* blockEntry = entry.get_if<gr::property_map>();
                if (blockEntry != nullptr && blockEntry->contains("graph")) {
                    auto read = gr::detail::readRecipeDeclarations(*blockEntry);
                    if (!read.has_value()) {
                        return std::unexpected(read.error());
                    }
                    declarations = std::move(*read);
                    break;
                }
            }
        }
    }

    std::string guard = "GNURADIO_RECIPES_";
    for (const char c : structName) {
        guard.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    guard += "_HPP";

    std::string out;
    out += std::format("// GENERATED FILE — do not edit. Source of truth: blocks/recipes/{}.\n", sourceFileName);
    out += "// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.\n";
    out += std::format("#ifndef {0}\n#define {0}\n\n", guard);
    const bool carriesVector = std::ranges::any_of(declarations, [](const auto& declaration) { return detail::vectorTypeWord(declaration.type); });
    out += "#include <memory>\n#include <string>\n#include <utility>\n";
    if (carriesVector) {
        out += "#include <vector>\n";
    }
    out += "\n#include <gnuradio-4.0/Graph.hpp>\n#include <gnuradio-4.0/Graph_yaml_importer.hpp>\n#include <gnuradio-4.0/PluginLoader.hpp>\n\n";
    out += "namespace gr::recipes {\n\n";
    out += std::format("struct {} {{\n", structName);
    out += "    struct Parameters {\n";

    std::string ctorArguments;
    std::string ctorInits;
    std::string members;
    for (const auto& declaration : declarations) {
        const std::string cppType = detail::cppTypeFor(declaration.type);
        if (declaration.required()) {
            ctorArguments += std::format("{}{} {}_", ctorArguments.empty() ? "" : ", ", cppType, declaration.name);
            ctorInits += std::format("{}{}(std::move({}_))", ctorInits.empty() ? "" : ", ", declaration.name, declaration.name);
            members += std::format("        {} {};", cppType, declaration.name);
        } else {
            auto literal = detail::vectorTypeWord(declaration.type) ? detail::vectorExpression(*declaration.defaultValue, cppType) : detail::scalarExpression(*declaration.defaultValue);
            if (!literal.has_value()) {
                return std::unexpected(literal.error());
            }
            std::string initial = *literal;
            if (declaration.type == "string") {
                initial = std::format("std::string(\"{}\")", detail::escapeStringLiteral(declaration.defaultValue->value_or(std::string_view{})));
            }
            members += std::format("        {} {} = {};", cppType, declaration.name, initial);
        }
        if (!declaration.doc.empty()) {
            members += std::format(" // {}", declaration.doc);
        }
        members += "\n";
    }
    if (!ctorArguments.empty()) {
        out += "        // required parameters are constructor arguments: omitting one is a compile error,\n";
        out += "        // the same requirement the loader enforces at run time\n";
        out += std::format("        Parameters({}) : {} {{}}\n", ctorArguments, ctorInits);
    }
    out += members;
    out += "    };\n\n";

    out += "    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {\n";
    out += "        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {\n";
    out += "            gr::detail::YamlDefinitionsLoader::Definition def;\n";
    out += std::format("            def.metadata.block_type = \"{}\";\n", detail::escapeStringLiteral(blockType));
    {
        int         counter = 0;
        std::string body;
        for (const auto& [key, value] : definition.definition) {
            if (auto emitted = detail::emitValueInto(body, value, std::format("def.definition[std::pmr::string(\"{}\")]", detail::escapeStringLiteral(std::string_view(key))), counter, "            "); !emitted.has_value()) {
                return std::unexpected(emitted.error());
            }
        }
        out += body;
    }
    out += "            return def;\n        }();\n        return kDefinition;\n    }\n\n";

    out += "    // Builds the composite through the same instantiation path the loader uses — the\n";
    out += "    // bindings attach identically, so live parameter changes behave identically — and\n";
    out += "    // adds it to `graph`. No YAML is parsed and no file is read.\n";
    out += "    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {\n";
    out += "        gr::property_map values;\n";
    for (const auto& declaration : declarations) {
        if (declaration.type == "string") {
            out += std::format("        values[std::pmr::string(\"{0}\")] = std::pmr::string(parameters.{0});\n", declaration.name);
        } else {
            out += std::format("        values[std::pmr::string(\"{0}\")] = parameters.{0};\n", declaration.name);
        }
    }
    out += "        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);\n";
    out += "        if (!composite.has_value()) {\n";
    out += "            return nullptr;\n";
    out += "        }\n";
    out += "        return graph.addBlock(*composite);\n";
    out += "    }\n";
    out += "};\n\n} // namespace gr::recipes\n\n";
    out += std::format("#endif // {}\n", guard);
    return out;
}

} // namespace gr::recipe::emitter

#endif // GNURADIO_RECIPEHEADEREMITTER_HPP
