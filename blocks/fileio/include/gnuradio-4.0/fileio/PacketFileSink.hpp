#ifndef GNURADIO_FILEIO_PACKET_FILE_SINK_HPP
#define GNURADIO_FILEIO_PACKET_FILE_SINK_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fileio/FileIo.hpp>

#include <gnuradio-4.0/fileio/NamespaceCompatibility.hpp>

namespace gr::blocks::fileio {

namespace detail::packet_file_sink {

/// @brief `[A-Za-z0-9._-]{1,64}`, not `.` or `..`, not beginning with `.` or `-`. Applied to `prefix + identifier +
/// suffix` after concatenation, so a `prefix` of `"../"` is refused exactly as an identifier carrying a separator
/// would be: the settings are as attacker-shaped as the metadata when a graph is built from a description file, and
/// both spellings draw from the same closed character set or they are rejected.
[[nodiscard]] inline bool isValidFileName(std::string_view name) noexcept {
    if (name.empty() || name.size() > 64UZ) {
        return false;
    }
    if (name == "." || name == "..") {
        return false;
    }
    if (name.front() == '.' || name.front() == '-') {
        return false;
    }
    for (const char character : name) {
        const bool ok = (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z') || (character >= '0' && character <= '9') || character == '.' || character == '_' || character == '-';
        if (!ok) {
            return false;
        }
    }
    return true;
}

/// @brief A name rendered for a log line: printable ASCII passes through, every other byte becomes `\xNN`, and a long
/// name is cut short. The identifier arrives as metadata and may carry an escape sequence, a NUL or a kilobyte of
/// text, and a refusal message is not a place to hand a terminal control bytes.
[[nodiscard]] inline std::string escaped(std::string_view name) {
    constexpr std::size_t kMaxShown = 96UZ;
    std::string           out;
    out.reserve(std::min(name.size(), kMaxShown) + 8UZ);
    for (const char character : name.substr(0UZ, std::min(name.size(), kMaxShown))) {
        const std::uint8_t byte = static_cast<std::uint8_t>(character);
        if (byte >= 0x20U && byte < 0x7FU && character != '\\') {
            out.push_back(character);
        } else {
            std::format_to(std::back_inserter(out), "\\x{:02x}", byte);
        }
    }
    if (name.size() > kMaxShown) {
        out += "...";
    }
    return out;
}

enum class OnExists { refuse, overwrite, unique };

[[nodiscard]] inline OnExists onExistsFromName(std::string_view name) {
    if (name == "refuse") {
        return OnExists::refuse;
    }
    if (name == "overwrite") {
        return OnExists::overwrite;
    }
    if (name == "unique") {
        return OnExists::unique;
    }
    throw gr::exception(std::format("on_exists must be 'refuse', 'overwrite' or 'unique', got '{}'", name));
}

} // namespace detail::packet_file_sink

GR_REGISTER_BLOCK(gr::blocks::fileio::PacketFileSink)

struct PacketFileSink : Block<PacketFileSink> {
    using Description = Doc<R""(
@brief Writes each completed gr::Packet<uint8_t> to its own whole file, named from its metadata.

Off the hot path, and the reason is its rate: one open, one write and one close per *completed file* — an image every
tens of seconds on a good pass, not a sample rate. What the arrangement buys is safety rather than speed: **the sink
never seeks.** A packet's payload is written once, contiguous, in one write call,
which never issues a seek — so a corrupted offset has no code path into the filesystem, unlike a receiver that seeks
to a header-supplied position and can produce a sparse file of arbitrary length from one bad header.

The file name is `prefix + <identifier> + suffix`, the identifier read from the packet's `id_key` metadata key.
`suffix` is a setting, never bytes off the wire, because a file's extension chosen by a received value is the
survey's own defect: an operator cannot correlate an invented extension with the pass that produced it. The
assembled name must match `[A-Za-z0-9._-]{1,64}`, must not be `.` or `..`, and must not begin with `.` or `-`, checked
*after* `prefix`, the identifier and `suffix` are concatenated, so a `prefix` of `"../"` is refused exactly as an
identifier carrying a separator would be. `on_exists` decides what happens when the name is already taken: `"refuse"`
(default, counted) leaves the existing file untouched, `"overwrite"` replaces it, and `"unique"` appends `-1`, `-2`,
… at the first free index — after the whole name, so a `suffix` spelling an extension lands as `name.bin-1`, and the
64-character cap is the checked name's, not the uniqued one's.

`directory` is checked once, at `start()`, and a missing one refuses to start; a directory removed while the graph
runs is re-created by the write rather than refused. Every refusal is a count and a `stop()` line, and so is a
filesystem that refuses the write itself: that packet's file is lost, counted in `nWriteFailures` and named on
stderr, and the graph keeps running, because the packets still arriving are the rest of the pass.
)"">;

    PortIn<Packet<std::uint8_t>> in;

    Annotated<std::string, "directory", Doc<"must exist at start(), where a missing one refuses to start; one removed later is re-created by the write">, Visible> directory{};
    Annotated<std::string, "prefix", Doc<"prepended to the identifier">>                                                                                           prefix{""};
    Annotated<std::string, "suffix", Doc<"appended after the identifier; the extension is a setting, never received bytes">>                                       suffix{""};
    Annotated<std::string, "on_exists", Doc<"'refuse' (default), 'overwrite' or 'unique'">, Visible>                                                               on_exists{"refuse"};
    Annotated<std::string, "id_key", Doc<"the metadata key naming the file">>                                                                                      id_key{"file_id"};

    GR_MAKE_REFLECTABLE(PacketFileSink, in, directory, prefix, suffix, on_exists, id_key);

    std::uint64_t nFilesWritten  = 0ULL;
    std::uint64_t nBytesWritten  = 0ULL;
    std::uint64_t nRefusedName   = 0ULL; ///< the assembled name fails the character class or the traversal guard
    std::uint64_t nRefusedNoId   = 0ULL; ///< no metadata map, or id_key absent, or not a string
    std::uint64_t nRefusedEmpty  = 0ULL; ///< a packet with no payload
    std::uint64_t nRefusedExists = 0ULL; ///< on_exists = "refuse" and the file is already there
    std::uint64_t nWriteFailures = 0ULL; ///< the filesystem refused the open, the write or the final flush

    detail::packet_file_sink::OnExists _onExists = detail::packet_file_sink::OnExists::refuse;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { _onExists = detail::packet_file_sink::onExistsFromName(on_exists.value); }

    void start() {
        _onExists = detail::packet_file_sink::onExistsFromName(on_exists.value);
        std::error_code ec;
        if (!std::filesystem::exists(directory.value, ec) || !std::filesystem::is_directory(directory.value, ec)) {
            throw gr::exception(std::format("directory '{}' does not exist", directory.value));
        }
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("files written", nFilesWritten);
        append("bytes written", nBytesWritten);
        append("refused name", nRefusedName);
        append("refused no id", nRefusedNoId);
        append("refused empty", nRefusedEmpty);
        append("refused exists", nRefusedExists);
        append("write failures", nWriteFailures);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::fileio::PacketFileSink '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan) {
        if (inSpan.empty()) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            writeOne(inSpan[i]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return work::Status::OK;
    }

private:
    /// @brief The identifier at `id_key`, or nothing where the map is absent, the key is missing, or it is not a
    /// string — a value of the wrong type reads as absent rather than as an error, on this whole document's rule.
    [[nodiscard]] std::optional<std::string> readIdentifier(const Packet<std::uint8_t>& packet) const {
        if (packet.meta_information.empty()) {
            return std::nullopt;
        }
        const property_map& map = packet.meta_information[0UZ];
        const auto          it  = map.find(property_map::key_type(id_key.value));
        if (it == map.end()) {
            return std::nullopt;
        }
        const std::pmr::string* value = it->second.get_if<std::pmr::string>();
        if (value == nullptr) {
            return std::nullopt;
        }
        return std::string(std::string_view(*value));
    }

    /// @brief The first free path under `_onExists`'s rule, or nothing where "refuse" finds the name already taken.
    [[nodiscard]] std::optional<std::filesystem::path> resolvePath(const std::string& fileName) {
        const std::filesystem::path base = std::filesystem::path(directory.value) / fileName;
        if (_onExists == detail::packet_file_sink::OnExists::overwrite) {
            return base;
        }
        std::error_code ec;
        if (!std::filesystem::exists(base, ec)) {
            return base;
        }
        if (_onExists == detail::packet_file_sink::OnExists::refuse) {
            return std::nullopt;
        }
        // "unique": append -1, -2, ... at the first free index
        for (std::uint64_t index = 1ULL;; ++index) {
            const std::filesystem::path candidate = std::filesystem::path(directory.value) / std::format("{}-{}", fileName, index);
            if (!std::filesystem::exists(candidate, ec)) {
                return candidate;
            }
        }
    }

    void writeOne(const Packet<std::uint8_t>& packet) {
        const std::optional<std::string> identifier = readIdentifier(packet);
        if (!identifier.has_value()) {
            ++nRefusedNoId;
            return;
        }
        if (packet.signal_values.empty()) {
            ++nRefusedEmpty;
            return;
        }

        const std::string fileName = prefix.value + *identifier + suffix.value;
        if (!detail::packet_file_sink::isValidFileName(fileName)) {
            ++nRefusedName;
            // The rejected name is what lets an operator correlate the refusal with the pass that produced it, and
            // it arrived as data: printed through the escaping rendering, so a control byte in it cannot reach a
            // terminal as a control byte.
            std::println(stderr, "gr::blocks::fileio::PacketFileSink '{}': refusing name '{}' (identifier '{}')", this->name, detail::packet_file_sink::escaped(fileName), detail::packet_file_sink::escaped(*identifier));
            return;
        }

        const std::optional<std::filesystem::path> path = resolvePath(fileName);
        if (!path.has_value()) {
            ++nRefusedExists;
            return;
        }

        const std::span<const std::uint8_t> bytes(packet.signal_values);
        auto                                result = gr::algorithm::fileio::write(path->string(), bytes, gr::algorithm::fileio::WriterConfig{.mode = gr::algorithm::fileio::WriteMode::overwrite});
        if (!result.has_value()) {
            // A full disk or a directory that went away between start() and now costs this one file. It is counted
            // and named and the graph keeps running: the packets still arriving are the rest of the pass, and
            // stopping the graph would lose those too.
            ++nWriteFailures;
            std::println(stderr, "gr::blocks::fileio::PacketFileSink '{}': {}", this->name, result.error().message);
            return;
        }
        ++nFilesWritten;
        nBytesWritten += bytes.size();
    }
};

} // namespace gr::blocks::fileio

#endif // GNURADIO_FILEIO_PACKET_FILE_SINK_HPP
