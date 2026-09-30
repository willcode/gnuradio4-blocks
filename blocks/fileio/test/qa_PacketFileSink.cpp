#include <boost/ut.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>

#include <gnuradio-4.0/basic/ChunkReassembler.hpp>
#include <gnuradio-4.0/fileio/PacketFileSink.hpp>

// PacketFileSink checks its directory at start(). These scenes therefore run through a real gr::test::RuntimeTest
// graph, as blocks/fileio/test/qa_BasicFileIo.cpp does for BasicFileSink. A bare span harness never invokes the
// block's lifecycle.

namespace {

using gr::blocks::fileio::PacketFileSink;
using PacketU = gr::Packet<std::uint8_t>;

/// @brief A temp directory this qa scene owns, removed unconditionally on destruction.
struct TempDir {
    std::filesystem::path path;

    TempDir() {
        const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
        path             = std::filesystem::temp_directory_path() / std::format("qa_PacketFileSink_{}", stamp);
        std::filesystem::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
    TempDir(const TempDir&)            = delete;
    TempDir& operator=(const TempDir&) = delete;
};

template<typename TItem>
struct ItemSource : gr::Block<ItemSource<TItem>> {
    gr::PortOut<TItem> out;
    GR_MAKE_REFLECTABLE(ItemSource, out);

    std::vector<TItem> _items{};
    std::size_t        _emitted = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t nItems = std::min(outSpan.size(), _items.size() - _emitted);
        for (std::size_t k = 0UZ; k < nItems; ++k) {
            outSpan[k] = _items[_emitted + k];
        }
        _emitted += nItems;
        outSpan.publish(nItems);
        return _emitted >= _items.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename TItem>
struct Collector : gr::Block<Collector<TItem>> {
    gr::PortIn<TItem> in;
    GR_MAKE_REFLECTABLE(Collector, in);

    std::vector<TItem> _items{};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _items.push_back(inSpan[k]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

[[nodiscard]] PacketU makePacket(std::optional<std::string> idValue, std::vector<std::uint8_t> payload, std::string_view idKey = "file_id") {
    PacketU packet;
    packet.signal_values = std::move(payload);
    if (idValue.has_value()) {
        packet.meta_information.resize(1UZ);
        packet.meta_information[0UZ].insert_or_assign(gr::property_map::key_type(idKey), gr::pmt::Value(*idValue));
    }
    return packet;
}

[[nodiscard]] std::vector<std::uint8_t> readFile(const std::filesystem::path& path) {
    std::ifstream             in(path, std::ios::binary);
    std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return data;
}

/// @brief The counters `runSink` reports, as a plain copyable summary.
///
/// Block types are move-only, and the RuntimeTest that owns the live PacketFileSink ends with the function that ran it.
struct SinkResult {
    std::uint64_t nFilesWritten  = 0ULL;
    std::uint64_t nBytesWritten  = 0ULL;
    std::uint64_t nRefusedName   = 0ULL;
    std::uint64_t nRefusedNoId   = 0ULL;
    std::uint64_t nRefusedEmpty  = 0ULL;
    std::uint64_t nRefusedExists = 0ULL;
    std::uint64_t nWriteFailures = 0ULL;
};

/// @brief Runs one PacketFileSink over @p packets under @p settings plus 'directory' and reports its counters.
///
/// The caller inspects the filesystem separately, in the temp directory it owns.
[[nodiscard]] SinkResult runSink(const std::filesystem::path& directory, std::vector<PacketU> packets, gr::property_map settings = {}) {
    settings["directory"] = directory.string();
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<ItemSource<PacketU>>();
    source._items                = std::move(packets);
    auto& sink                   = test.emplace<PacketFileSink>(settings);
    std::ignore                  = test.connect(source, "out", sink, "in");
    std::ignore                  = test.run();
    return SinkResult{.nFilesWritten = sink.nFilesWritten, .nBytesWritten = sink.nBytesWritten, .nRefusedName = sink.nRefusedName, .nRefusedNoId = sink.nRefusedNoId, .nRefusedEmpty = sink.nRefusedEmpty, .nRefusedExists = sink.nRefusedExists, .nWriteFailures = sink.nWriteFailures};
}

/// @brief A sink with @p settings staged and applied as a graph's lifecycle would.
///
/// A refused setting is refused here and not inside a running graph.
[[nodiscard]] PacketFileSink makeSink(gr::property_map settings) {
    PacketFileSink sink(std::move(settings));
    sink.settings().init();
    std::ignore = sink.settings().applyStagedParameters();
    return sink;
}

} // namespace

const boost::ut::suite<"PacketFileSink"> packetFileSinkTests = [] {
    using namespace boost::ut;

    // The sink writes once and does not seek
    "the sink writes once and never seeks"_test = [] {
        // The sink reaches the filesystem through one call. The call opens the file truncating and writes the
        // payload in one go. No code path in the sink issues a seek. The test asserts what that rules out. A writer
        // that seeked to a position and wrote would leave the file's old bytes beyond its payload. A payload
        // beginning with zero bytes would look like a sparse hole. Both cases are checked below. The file's size
        // equals the payload's after a shorter payload replaces a longer file. A payload that starts with zeros has
        // those zeros on disk.
        TempDir                         dir;
        const std::vector<std::uint8_t> payload{0U, 0U, 0U, 1U, 2U, 3U, 4U, 5U}; // begins with zero bytes: a seek-based
                                                                                 // writer and a whole-write are otherwise indistinguishable
        SinkResult sink = runSink(dir.path, {makePacket("alpha", payload)});
        expect(eq(sink.nFilesWritten, 1ULL));
        expect(eq(sink.nBytesWritten, payload.size()));

        const std::filesystem::path written = dir.path / "alpha";
        expect(std::filesystem::exists(written));
        if (std::filesystem::exists(written)) {
            expect(eq(std::filesystem::file_size(written), payload.size())) << "the written file's size equals the payload's length";
            expect(std::ranges::equal(readFile(written), payload)) << "the file's bytes equal the packet's exactly";
        }

        // on_exists = "refuse" (default): the existing file is left untouched, counted
        {
            const std::vector<std::uint8_t> other{9U, 9U, 9U};
            SinkResult                      refused = runSink(dir.path, {makePacket("alpha", other)}, {{"on_exists", std::string("refuse")}});
            expect(eq(refused.nRefusedExists, 1ULL));
            expect(eq(refused.nFilesWritten, 0ULL));
            expect(std::ranges::equal(readFile(written), payload)) << "the original file is untouched";
        }
        // on_exists = "overwrite": replaced, and the shorter payload leaves a file of exactly its own length
        {
            const std::vector<std::uint8_t> replacement{7U, 7U, 7U, 7U};
            expect(lt(replacement.size(), payload.size())) << "so a leftover tail would be visible";
            SinkResult overwritten = runSink(dir.path, {makePacket("alpha", replacement)}, {{"on_exists", std::string("overwrite")}});
            expect(eq(overwritten.nFilesWritten, 1ULL));
            expect(eq(std::filesystem::file_size(written), replacement.size())) << "nothing of the longer file it replaced survives, which a seeking writer would have left";
            expect(std::ranges::equal(readFile(written), replacement));

            // put the original back for the scenes that follow
            SinkResult restored = runSink(dir.path, {makePacket("alpha", payload)}, {{"on_exists", std::string("overwrite")}});
            expect(eq(restored.nFilesWritten, 1ULL));
        }
        // on_exists = "unique": appends -1 at the first free index
        {
            const std::vector<std::uint8_t> extra{3U, 1U, 4U, 1U, 5U};
            SinkResult                      unique = runSink(dir.path, {makePacket("alpha", extra)}, {{"on_exists", std::string("unique")}});
            expect(eq(unique.nFilesWritten, 1ULL));
            const std::filesystem::path uniquePath = dir.path / "alpha-1";
            expect(std::filesystem::exists(uniquePath));
            if (std::filesystem::exists(uniquePath)) {
                expect(std::ranges::equal(readFile(uniquePath), extra));
            }
            expect(std::filesystem::exists(written)) << "the original 'alpha' is left as it was";
        }
    };

    // Traversal and naming
    "traversal and naming"_test = [] {
        TempDir                                                dir;
        const std::vector<std::pair<std::string, std::string>> badIdentifiers{
            {"../etc/passwd", "traversal"},
            {"..", "parent directory"},
            {".", "current directory"},
            {".hidden", "leading dot"},
            {"-rf", "leading dash"},
            {std::string(65UZ, 'a'), "65 characters, one over the cap"},
        };
        for (const auto& [identifier, label] : badIdentifiers) {
            SinkResult sink = runSink(dir.path, {makePacket(identifier, {1U, 2U, 3U})});
            expect(eq(sink.nRefusedName, 1ULL)) << label;
            expect(eq(sink.nFilesWritten, 0ULL)) << label;
        }
        { // an identifier containing a NUL byte is outside [A-Za-z0-9._-] and is not truncated as a C string
            const std::string nulIdentifier{'a', 'b', '\0', 'c', 'd'};
            expect(eq(nulIdentifier.size(), 5UZ));
            SinkResult sink = runSink(dir.path, {makePacket(nulIdentifier, {1U, 2U, 3U})});
            expect(eq(sink.nRefusedName, 1ULL));
        }
        { // a prefix of "../" with an otherwise valid identifier is refused after concatenation
            SinkResult sink = runSink(dir.path, {makePacket("valid123", {1U, 2U, 3U})}, {{"prefix", std::string("../")}});
            expect(eq(sink.nRefusedName, 1ULL));
            expect(eq(sink.nFilesWritten, 0ULL));
        }
        { // a suffix that carries the assembled name past the 64-character cap, with a valid identifier
            SinkResult sink = runSink(dir.path, {makePacket(std::string(60UZ, 'a'), {1U, 2U, 3U})}, {{"suffix", std::string(".binary")}});
            expect(eq(sink.nRefusedName, 1ULL)) << "the check is on the whole name, after concatenation";
            expect(eq(sink.nFilesWritten, 0ULL));
        }
        { // no id_key at all, and a packet with no payload, each a separately counted refusal
            SinkResult noId = runSink(dir.path, {makePacket(std::nullopt, {1U, 2U, 3U})});
            expect(eq(noId.nRefusedNoId, 1ULL));
            expect(eq(noId.nFilesWritten, 0ULL));

            SinkResult wrongKey = runSink(dir.path, {makePacket("valid123", {1U, 2U, 3U}, "other_key")});
            expect(eq(wrongKey.nRefusedNoId, 1ULL)) << "the key the setting names, and no other";
            expect(eq(wrongKey.nFilesWritten, 0ULL));

            SinkResult empty = runSink(dir.path, {makePacket("valid123", {})});
            expect(eq(empty.nRefusedEmpty, 1ULL));
            expect(eq(empty.nFilesWritten, 0ULL));
        }
        expect(std::ranges::distance(std::filesystem::directory_iterator(dir.path)) == 0) << "nothing from the refused names reached the filesystem";

        { // a non-default id_key, which does write
            SinkResult named = runSink(dir.path, {makePacket("byotherkey", {1U, 2U, 3U}, "packet_name")}, {{"id_key", std::string("packet_name")}});
            expect(eq(named.nFilesWritten, 1ULL));
            expect(std::filesystem::exists(dir.path / "byotherkey"));
            std::error_code ec;
            std::filesystem::remove(dir.path / "byotherkey", ec);
        }

        // A packet produced by ChunkReassembler passes at every id_bytes in [1,8]. The traversal class is
        // unreachable for the reassembler's own output. A small scheduler graph produces the packet, since the
        // reassembler's lifecycle must run to configure it. The packet feeds a second sink directly.
        for (gr::Size_t idBytes = 1U; idBytes <= 8U; ++idBytes) {
            std::vector<std::uint8_t> record(static_cast<std::size_t>(idBytes) + 8UZ + 16UZ, 0U);
            for (std::size_t i = 0UZ; i < idBytes; ++i) { // an all-0xFF identifier, the widest lowercase-hex spelling
                record[i] = 0xFFU;
            }
            // index stays 0 in the zero-filled buffer. total_size = 16, big-endian in the last byte of its 4-byte
            // field. This single 16-byte chunk (chunk_size = 16) completes the file on arrival.
            record[idBytes + 7U] = 16U;
            gr::DataSet<std::uint8_t> input;
            input.signal_values = record;

            gr::test::RuntimeTest runtimeTest;
            auto&                 source = runtimeTest.emplace<ItemSource<gr::DataSet<std::uint8_t>>>();
            source._items                = {input};
            auto& reassembler            = runtimeTest.emplace<gr::blocks::basic::ChunkReassembler>(gr::property_map{
                           {"chunk_format", std::string("indexed")},                      //
                           {"id_offset", gr::Size_t{0U}}, {"id_bytes", idBytes},          //
                           {"index_offset", idBytes}, {"index_bytes", gr::Size_t{4U}},    //
                           {"size_offset", idBytes + 4U}, {"size_bytes", gr::Size_t{4U}}, //
                           {"payload_offset", idBytes + 8U},                              //
                           {"chunk_size", gr::Size_t{16U}},                               //
                           {"max_open_files", gr::Size_t{4U}},                            //
                           {"max_file_bytes", std::uint64_t{1UZ << 16UZ}},                //
            });
            auto& collected              = runtimeTest.emplace<Collector<PacketU>>();
            std::ignore                  = runtimeTest.connect(source, "out", reassembler, "in");
            std::ignore                  = runtimeTest.connect(reassembler, "out", collected, "in");
            std::ignore                  = runtimeTest.run();

            expect(eq(collected._items.size(), 1UZ)) << std::format("id_bytes {}", idBytes);
            if (collected._items.empty()) {
                continue;
            }

            SinkResult sink = runSink(dir.path, {collected._items.front()});
            expect(eq(sink.nFilesWritten, 1ULL)) << std::format("id_bytes {}: the reassembler's own hex file_id is always an admissible name", idBytes);
            expect(eq(sink.nRefusedName, 0ULL)) << std::format("id_bytes {}", idBytes);
        }
    };

    "the name check is the checked name's, and the unique suffix is added after it"_test = [] {
        TempDir           dir;
        const std::string longest(64UZ, 'n'); // exactly the cap, and admissible
        expect(eq(longest.size(), 64UZ));

        SinkResult first = runSink(dir.path, {makePacket(longest, {1U, 2U})});
        expect(eq(first.nFilesWritten, 1ULL)) << "sixty-four characters is the cap, not one over it";

        SinkResult second = runSink(dir.path, {makePacket(longest, {3U, 4U})}, {{"on_exists", std::string("unique")}});
        expect(eq(second.nFilesWritten, 1ULL));
        const std::filesystem::path uniquePath = dir.path / (longest + "-1");
        expect(std::filesystem::exists(uniquePath)) << "the free index is appended after the whole checked name";
        expect(eq(uniquePath.filename().string().size(), 66UZ)) << "so the name on disk may exceed the cap the check applied";
    };

    "the settings the sink refuses, and the directory it refuses to start without"_test = [] {
        TempDir dir;
        expect(throws([] { std::ignore = makeSink({{"on_exists", std::string("clobber")}}); })) << "on_exists is one of three spellings or it is refused";
        expect(nothrow([] { std::ignore = makeSink({{"on_exists", std::string("unique")}}); }));

        expect(throws([&dir] {
            PacketFileSink sink = makeSink({{"directory", (dir.path / "not-there").string()}});
            sink.start();
        })) << "a missing directory refuses to start rather than creating one";
        expect(nothrow([&dir] {
            PacketFileSink sink = makeSink({{"directory", dir.path.string()}});
            sink.start();
        }));
    };

    "a write the filesystem refuses costs that one file and not the graph"_test = [] {
        TempDir dir;
        // A directory stands where the file would go. The open fails, whatever the payload is.
        std::filesystem::create_directories(dir.path / "blocked");

        SinkResult sink = runSink(dir.path, {makePacket("blocked", {1U, 2U, 3U}), makePacket("after", {4U, 5U})}, {{"on_exists", std::string("overwrite")}});
        expect(eq(sink.nWriteFailures, 1ULL));
        expect(eq(sink.nFilesWritten, 1ULL)) << "the packet after the failure is still written";
        expect(eq(sink.nBytesWritten, 2ULL));
        expect(std::filesystem::exists(dir.path / "after"));
        expect(std::filesystem::is_directory(dir.path / "blocked")) << "and what was in the way is untouched";
    };
};

int main() { /* not needed for UT */ }
