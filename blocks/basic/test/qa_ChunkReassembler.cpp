#include <boost/ut.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numeric>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/algorithm/packet/ChunkReassembly.hpp>
#include <gnuradio-4.0/basic/ChunkReassembler.hpp>
#include <gnuradio-4.0/basic/PacketToDataSet.hpp>

// Criteria 1-14 of spec-chunk-reassembly.md §10. Criterion 13 (the degenerate descriptor) is exercised
// against the landed kernel, gr::packet::ChunkReassembler, directly: the block only ever configures the two shipped
// formats, and both require their defining field (index for "indexed", offset for "offset"), so a descriptor with
// neither is not reachable through the block's own settings — it is reachable only through pushDescriptor(), the
// hook shape a protocol-specific format would use. Everything else is driven through the block, span to span, on
// the pattern blocks/basic/test/qa_DataSetToPacket.cpp uses, because the graph a scheduler would add contributes
// nothing here: a chunk's fate is decided by processBulk alone.

namespace {

using gr::blocks::basic::ChunkReassembler;
using Record  = gr::DataSet<std::uint8_t>;
using PacketU = gr::Packet<std::uint8_t>;

// ─── a minimal four-port span harness, in TestSpans.hpp's own shape ─────────────────────────────────────────────────

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr TagReaderSpan(std::span<const gr::Tag> tags) : std::span<const gr::Tag>(tags) {}
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
    constexpr TagWriterSpan(std::span<gr::Tag> tags) : std::span<gr::Tag>(tags) {}
    constexpr void publish(std::size_t) const noexcept {}
};

template<typename T>
struct InSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    bool          isConnected = true;
    bool          isSync      = true;

    InSpan(std::span<const T> items, std::size_t at = 0UZ) : std::span<const T>(items), streamIndex(at) {}

    constexpr bool consume(std::size_t nItems) noexcept {
        consumed = nItems;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::span<const gr::Tag> tags() const noexcept { return {}; }
    [[nodiscard]] std::span<const gr::Tag> tags(std::size_t) const noexcept { return {}; }
};

template<typename T>
struct OutSpan : std::span<T> {
    using value_type = T;

    std::vector<gr::Tag>* sink = nullptr;
    TagWriterSpan         tags{};
    std::size_t           streamIndex = 0UZ;
    std::size_t           count       = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = false;

    OutSpan(std::span<T> items, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr, bool connected = true) : std::span<T>(items), sink(published), streamIndex(at), isConnected(connected) {}

    constexpr void publish(std::size_t nItems) noexcept { count = nItems; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (isConnected && sink != nullptr) {
            sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
        }
    }
};

struct Capture {
    std::vector<PacketU> out{};
    std::vector<PacketU> incomplete{};
    std::vector<Record>  rejected{};
    std::vector<gr::Tag> rejectTags{};
    std::size_t          consumed = 0UZ;
    std::size_t          calls    = 0UZ;
    bool                 stalled  = false;
};

// ─── graph-side blocks for criterion 12, which needs a real port and a real downstream block's own tag handling ────

template<typename TItem>
struct ItemSource : gr::Block<ItemSource<TItem>> {
    gr::PortOut<TItem> out;
    GR_MAKE_REFLECTABLE(ItemSource, out);

    std::vector<TItem> _items{};
    std::size_t        _emitted    = 0UZ;
    std::size_t        _maxPerCall = 0UZ; ///< 0: as many as the span holds

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t room   = _maxPerCall == 0UZ ? outSpan.size() : std::min(outSpan.size(), _maxPerCall);
        const std::size_t nItems = std::min(room, _items.size() - _emitted);
        for (std::size_t k = 0UZ; k < nItems; ++k) {
            outSpan[k] = _items[_emitted + k];
        }
        _emitted += nItems;
        outSpan.publish(nItems);
        return _emitted >= _items.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename TItem>
struct GraphCollector : gr::Block<GraphCollector<TItem>> {
    gr::PortIn<TItem> in;
    GR_MAKE_REFLECTABLE(GraphCollector, in);

    std::vector<TItem> _items{};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _items.push_back(inSpan[k]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief Drive @p block over @p records. @p room caps every output span alike; 0 means "one slot per record".
/// @p inputChunk caps how many records one processBulk call is offered; 0 means the whole remainder.
[[nodiscard]] Capture run(ChunkReassembler& block, std::span<const Record> records, bool outConnected = true, bool incompleteConnected = true, bool rejectConnected = true, std::size_t room = 0UZ, std::size_t inputChunk = 0UZ) {
    Capture           capture;
    const std::size_t theRoom = room == 0UZ ? std::max(records.size(), 1UZ) : room;

    std::vector<PacketU> outScratch(theRoom);
    std::vector<PacketU> incScratch(theRoom);
    std::vector<Record>  rejScratch(records.size() + 1UZ);

    std::size_t consumed = 0UZ;
    while (consumed < records.size()) {
        const std::size_t offered = inputChunk == 0UZ ? records.size() - consumed : std::min(inputChunk, records.size() - consumed);
        InSpan<Record>    inSpan(records.subspan(consumed, offered), consumed);
        OutSpan<PacketU>  outSpan(outConnected ? std::span<PacketU>(outScratch) : std::span<PacketU>{}, capture.out.size(), nullptr, outConnected);
        OutSpan<PacketU>  incSpan(incompleteConnected ? std::span<PacketU>(incScratch) : std::span<PacketU>{}, capture.incomplete.size(), nullptr, incompleteConnected);
        OutSpan<Record>   rejSpan(rejectConnected ? std::span<Record>(rejScratch) : std::span<Record>{}, capture.rejected.size(), &capture.rejectTags, rejectConnected);

        std::ignore = block.processBulk(inSpan, outSpan, incSpan, rejSpan);
        ++capture.calls;

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            capture.out.push_back(std::move(outScratch[k]));
        }
        for (std::size_t k = 0UZ; k < incSpan.count; ++k) {
            capture.incomplete.push_back(std::move(incScratch[k]));
        }
        for (std::size_t k = 0UZ; k < rejSpan.count; ++k) {
            capture.rejected.push_back(std::move(rejScratch[k]));
        }
        consumed += inSpan.consumed;
        capture.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && incSpan.count == 0UZ && rejSpan.count == 0UZ) {
            capture.stalled = true;
            break;
        }
    }
    return capture;
}

[[nodiscard]] ChunkReassembler make(gr::property_map settings) {
    ChunkReassembler block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

// ─── header field layout and record builders ─────────────────────────────────────────────────────────────────────

struct Layout {
    std::size_t  idOffset = 0UZ, idBytes = 0UZ;
    std::size_t  indexOffset = 0UZ, indexBytes = 0UZ;
    std::size_t  offsetOffset = 0UZ, offsetBytes = 0UZ;
    std::size_t  payloadOffset = 0UZ;
    std::size_t  countOffset = 0UZ, countBytes = 0UZ;
    std::size_t  sizeOffset = 0UZ, sizeBytes = 0UZ;
    std::size_t  flagOffset = 0UZ;
    std::uint8_t flagBit    = 8U;
};

void putBigEndian(std::vector<std::uint8_t>& buf, std::size_t offset, std::size_t width, std::uint64_t value) {
    for (std::size_t i = 0UZ; i < width; ++i) {
        buf[offset + i] = static_cast<std::uint8_t>((value >> (8UZ * (width - 1UZ - i))) & 0xFFULL);
    }
}

struct Fields {
    std::optional<std::uint64_t> id{};
    std::optional<std::uint64_t> index{};
    std::optional<std::uint64_t> offset{};
    std::optional<std::uint64_t> count{};
    std::optional<std::uint64_t> size{};
    bool                         lastChunk = false;
};

[[nodiscard]] std::vector<std::uint8_t> buildBytes(const Layout& layout, const Fields& fields, std::span<const std::uint8_t> payload) {
    std::vector<std::uint8_t> buf(layout.payloadOffset + payload.size(), 0U);
    if (fields.id.has_value() && layout.idBytes != 0UZ) {
        putBigEndian(buf, layout.idOffset, layout.idBytes, *fields.id);
    }
    if (fields.index.has_value() && layout.indexBytes != 0UZ) {
        putBigEndian(buf, layout.indexOffset, layout.indexBytes, *fields.index);
    }
    if (fields.offset.has_value() && layout.offsetBytes != 0UZ) {
        putBigEndian(buf, layout.offsetOffset, layout.offsetBytes, *fields.offset);
    }
    if (fields.count.has_value() && layout.countBytes != 0UZ) {
        putBigEndian(buf, layout.countOffset, layout.countBytes, *fields.count);
    }
    if (fields.size.has_value() && layout.sizeBytes != 0UZ) {
        putBigEndian(buf, layout.sizeOffset, layout.sizeBytes, *fields.size);
    }
    if (fields.lastChunk && layout.flagBit < 8U) {
        buf[layout.flagOffset] = static_cast<std::uint8_t>(buf[layout.flagOffset] | (1U << layout.flagBit));
    }
    std::ranges::copy(payload, buf.begin() + static_cast<std::ptrdiff_t>(layout.payloadOffset));
    return buf;
}

[[nodiscard]] Record makeRecord(std::vector<std::uint8_t> bytes, std::optional<bool> crcOk = std::nullopt, gr::Size_t correctedErrors = 0U, gr::Size_t uncorrectableErrors = 0U) {
    Record record;
    record.signal_values = std::move(bytes);
    if (crcOk.has_value() || correctedErrors != 0U || uncorrectableErrors != 0U) {
        record.meta_information.resize(1UZ);
        if (crcOk.has_value()) {
            record.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("crc_ok"), gr::pmt::Value(*crcOk));
        }
        if (correctedErrors != 0U) {
            record.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("corrected_errors"), gr::pmt::Value(correctedErrors));
        }
        if (uncorrectableErrors != 0U) {
            record.meta_information[0UZ].insert_or_assign(gr::property_map::key_type("uncorrectable_errors"), gr::pmt::Value(uncorrectableErrors));
        }
    }
    return record;
}

/// @brief A deterministic, non-repeating byte pattern: byte `i` of the reference file is `(i * 131 + 7) mod 256`.
[[nodiscard]] std::vector<std::uint8_t> referenceFile(std::size_t length) {
    std::vector<std::uint8_t> bytes(length);
    for (std::size_t i = 0UZ; i < length; ++i) {
        bytes[i] = static_cast<std::uint8_t>((i * 131UZ + 7UZ) & 0xFFUZ);
    }
    return bytes;
}

constexpr std::size_t kChunkSize   = 224UZ;
constexpr std::size_t kFileBytes   = 8192UZ;
constexpr std::size_t kFullChunks  = 36UZ; // 8192 = 36 * 224 + 128
constexpr std::size_t kLastChunk   = 128UZ;
constexpr std::size_t kTotalChunks = 37UZ;

/// @brief The "basic" indexed layout most criteria share: id(4)@0, index(4)@4, size(4)@8, payload@12.
constexpr Layout kBasicLayout{.idOffset = 0UZ, .idBytes = 4UZ, .indexOffset = 4UZ, .indexBytes = 4UZ, .payloadOffset = 12UZ, .sizeOffset = 8UZ, .sizeBytes = 4UZ};

[[nodiscard]] gr::property_map basicSettings(gr::Size_t maxOpenFiles = 8U, std::uint64_t maxFileBytes = 1UZ << 20UZ, std::uint64_t evictAfterRecords = 0ULL, bool requireCrcOk = false, bool publishIncomplete = false, bool newFileOnRegression = true) {
    return gr::property_map{
        {"chunk_format", std::string("indexed")},                          //
        {"id_offset", gr::Size_t{0U}}, {"id_bytes", gr::Size_t{4U}},       //
        {"index_offset", gr::Size_t{4U}}, {"index_bytes", gr::Size_t{4U}}, //
        {"size_offset", gr::Size_t{8U}}, {"size_bytes", gr::Size_t{4U}},   //
        {"payload_offset", gr::Size_t{12U}},                               //
        {"chunk_size", gr::Size_t{static_cast<gr::Size_t>(kChunkSize)}},   //
        {"max_open_files", maxOpenFiles},                                  //
        {"max_file_bytes", maxFileBytes},                                  //
        {"evict_after_records", evictAfterRecords},                        //
        {"require_crc_ok", requireCrcOk},                                  //
        {"publish_incomplete", publishIncomplete},                         //
        {"new_file_on_index_regression", newFileOnRegression},             //
    };
}

/// @brief Every chunk of a reference file of `fileBytes` at `chunkSize`, as (index, payload) pairs.
[[nodiscard]] std::vector<std::pair<std::uint64_t, std::vector<std::uint8_t>>> splitIntoChunks(std::span<const std::uint8_t> file, std::size_t chunkSize) {
    std::vector<std::pair<std::uint64_t, std::vector<std::uint8_t>>> chunks;
    std::size_t                                                      offset = 0UZ;
    std::uint64_t                                                    index  = 0ULL;
    while (offset < file.size()) {
        const std::size_t take = std::min(chunkSize, file.size() - offset);
        chunks.emplace_back(index, std::vector<std::uint8_t>(file.begin() + static_cast<std::ptrdiff_t>(offset), file.begin() + static_cast<std::ptrdiff_t>(offset + take)));
        offset += take;
        ++index;
    }
    return chunks;
}

[[nodiscard]] Record encodeChunk(std::uint64_t id, std::uint64_t index, std::optional<std::uint64_t> size, std::span<const std::uint8_t> payload) { return makeRecord(buildBytes(kBasicLayout, Fields{.id = id, .index = index, .size = size}, payload)); }

[[nodiscard]] std::optional<std::string> metaString(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(gr::property_map::key_type(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const std::pmr::string* value = it->second.get_if<std::pmr::string>(); value != nullptr) {
        return std::string(std::string_view(*value));
    }
    return std::nullopt;
}

template<typename T>
[[nodiscard]] std::optional<T> metaValue(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(gr::property_map::key_type(key));
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const T* value = it->second.get_if<T>(); value != nullptr) {
        return *value;
    }
    return std::nullopt;
}

[[nodiscard]] gr::property_map metaOf(const PacketU& packet) { return packet.meta_information.size() == 1UZ ? packet.meta_information.front() : gr::property_map{}; }

} // namespace

const boost::ut::suite<"ChunkReassembler"> chunkReassemblerTests = [] {
    using namespace boost::ut;

    // criterion 1 — out-of-order reassembles identically to in-order
    "out-of-order reassembles identically to in-order"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);
        expect(eq(chunks.size(), kTotalChunks));
        expect(eq(chunks.front().second.size(), kChunkSize));
        expect(eq(chunks.back().second.size(), kLastChunk));
        expect(eq(kFullChunks * kChunkSize + kLastChunk, kFileBytes));

        const auto runOrder = [&](std::span<const std::size_t> order) -> PacketU {
            auto                block = make(basicSettings());
            std::vector<Record> records;
            for (const std::size_t k : order) {
                records.push_back(encodeChunk(1ULL, chunks[k].first, kFileBytes, chunks[k].second));
            }
            const Capture capture = run(block, std::span<const Record>(records));
            expect(!capture.stalled);
            expect(eq(capture.out.size(), 1UZ));
            if (capture.out.empty()) {
                return {};
            }
            expect(eq(metaValue<gr::Size_t>(metaOf(capture.out.front()), "file_chunks").value_or(0U), static_cast<gr::Size_t>(kTotalChunks)));
            return capture.out.front();
        };

        std::vector<std::size_t> identity(kTotalChunks);
        std::iota(identity.begin(), identity.end(), 0UZ);
        const PacketU reference = runOrder(identity);
        expect(eq(reference.signal_values.size(), kFileBytes));
        expect(std::ranges::equal(reference.signal_values, file));

        std::vector<std::size_t> reversed(identity.rbegin(), identity.rend());
        const PacketU            reversedPacket = runOrder(reversed);
        expect(std::ranges::equal(reversedPacket.signal_values, file)) << "reverse order";

        std::mt19937 rng(1234U);
        for (int trial = 0; trial < 10; ++trial) {
            std::vector<std::size_t> shuffled = identity;
            std::ranges::shuffle(shuffled, rng);
            const PacketU shuffledPacket = runOrder(shuffled);
            expect(std::ranges::equal(shuffledPacket.signal_values, file)) << std::format("permutation {}", trial);
        }
    };

    // criterion 2 — duplicates with equal content are counted and change nothing
    "duplicates with equal content are counted and change nothing"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        // Every chunk but the last is pushed immediately followed by its own duplicate; the last chunk (whichever
        // index happens to complete the file) is pushed once only. A file is retired the instant it completes
        // (F15: "a retransmission arriving after a file completed and was released opens a new file"), so there is
        // no push after the completing one that could still land on this same file — 36 of the 37 chunks are the
        // most this scene can duplicate against one open file, and that bound holds for any push order, not just
        // this one.
        auto                block = make(basicSettings());
        std::vector<Record> records;
        for (std::size_t k = 0UZ; k < chunks.size(); ++k) {
            const auto& [index, payload] = chunks[k];
            records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
            if (k + 1UZ < chunks.size()) {
                records.push_back(encodeChunk(1ULL, index, kFileBytes, payload)); // the duplicate
            }
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 1UZ));
        if (!capture.out.empty()) {
            expect(std::ranges::equal(capture.out.front().signal_values, file));
        }
        expect(eq(block._engine->counters().duplicate_chunks, kTotalChunks - 1UZ));
        expect(eq(block._engine->counters().conflicting_chunks, 0ULL));
    };

    // criterion 3 — a conflicting duplicate does not overwrite
    "a conflicting duplicate does not overwrite"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        auto                block = make(basicSettings());
        std::vector<Record> records;
        for (const auto& [index, payload] : chunks) {
            records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
            if (index == 5ULL) { // one byte altered, re-pushed right after the good copy
                std::vector<std::uint8_t> damaged = payload;
                damaged.front()                   = static_cast<std::uint8_t>(damaged.front() ^ 0xFFU);
                records.push_back(encodeChunk(1ULL, index, kFileBytes, damaged));
            }
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 1UZ));
        if (!capture.out.empty()) {
            expect(std::ranges::equal(capture.out.front().signal_values, file)) << "the first write stands";
            expect(eq(metaValue<gr::Size_t>(metaOf(capture.out.front()), "file_conflicts").value_or(0U), gr::Size_t{1U}));
        }
        expect(eq(block._engine->counters().conflicting_chunks, 1ULL));
        expect(eq(capture.rejected.size(), 1UZ));
        expect(eq(metaString(capture.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("content_conflict")));
    };

    // criterion 4 — a missing chunk never completes and is evicted with a count
    "a missing chunk never completes and is evicted with a count"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        auto                block = make(basicSettings(8U, 1UZ << 20UZ, 50ULL, false, true));
        std::vector<Record> records;
        for (const auto& [index, payload] : chunks) {
            if (index == 17ULL) {
                continue; // withheld
            }
            records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
        }
        for (std::uint64_t k = 0ULL; k < 60ULL; ++k) { // another file, well past the 50-record staleness window; its
            // declared size is far larger than what is ever pushed, so it stays incomplete and never itself
            // completes on 'out' (which would otherwise defeat this scene's "nothing on out" assertion)
            const std::vector<std::uint8_t> filler(kChunkSize, static_cast<std::uint8_t>(k));
            records.push_back(encodeChunk(2ULL, k, 1'000'000ULL, filler));
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 0UZ)) << "the first file never completes";
        expect(eq(block._engine->counters().evicted_stale, 1ULL));
        if (capture.incomplete.size() == 1UZ) {
            const gr::property_map meta = metaOf(capture.incomplete.front());
            expect(eq(metaValue<std::uint64_t>(meta, "file_covered_bytes").value_or(0ULL), kFileBytes - kChunkSize)) << "8192 - 224 = 7968";
            expect(eq(metaValue<std::uint64_t>(meta, "file_covered_bytes").value_or(0ULL), 7968ULL));
            expect(eq(metaString(meta, "discard_reason").value_or(""), std::string("evicted_stale")));
        } else {
            expect(false) << "expected exactly one incomplete packet";
        }
    };

    // criterion 5 — two interleaved files complete independently
    "two interleaved files complete independently"_test = [] {
        const std::vector<std::uint8_t> fileA = referenceFile(kFileBytes);
        std::vector<std::uint8_t>       fileB(kFileBytes);
        for (std::size_t i = 0UZ; i < kFileBytes; ++i) {
            fileB[i] = static_cast<std::uint8_t>(0xFFU - fileA[i]);
        }
        const auto chunksA = splitIntoChunks(fileA, kChunkSize);
        const auto chunksB = splitIntoChunks(fileB, kChunkSize);

        auto                block = make(basicSettings());
        std::vector<Record> records;
        for (std::size_t k = 0UZ; k < kTotalChunks; ++k) {
            records.push_back(encodeChunk(1ULL, chunksA[k].first, kFileBytes, chunksA[k].second));
            records.push_back(encodeChunk(2ULL, chunksB[k].first, kFileBytes, chunksB[k].second));
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 2UZ));
        expect(eq(block._engine->counters().files_opened, 2ULL));
        expect(eq(block._engine->counters().evicted_for_cap, 0ULL));
        expect(eq(block._engine->counters().evicted_stale, 0ULL));
        if (capture.out.size() == 2UZ) {
            const std::string idA = metaString(metaOf(capture.out[0UZ]), "file_id").value_or("");
            const std::string idB = metaString(metaOf(capture.out[1UZ]), "file_id").value_or("");
            expect(eq(idA, std::string("00000001")));
            expect(eq(idB, std::string("00000002")));
            expect(std::ranges::equal(capture.out[0UZ].signal_values, fileA));
            expect(std::ranges::equal(capture.out[1UZ].signal_values, fileB));
        }
    };

    // criterion 6 — max_open_files evicts the least recently touched and admits the new file
    "max_open_files evicts the least recently touched and admits the new file"_test = [] {
        auto block = make(basicSettings(2U, 1UZ << 16UZ, 0ULL, false, true));

        // each file declares 128 bytes and receives one 64-byte chunk at index 0 — the file's last index, since 128
        // bytes at a chunk size of 224 is one chunk — so it stays open (incomplete) at half coverage rather than
        // completing and retiring itself before cap pressure ever gets to act on it
        const std::vector<std::uint8_t> small(64UZ, 0xAAU);
        std::vector<Record>             records;
        records.push_back(encodeChunk(1ULL, 0ULL, 128ULL, small)); // file 1 opens, touched
        records.push_back(encodeChunk(2ULL, 0ULL, 128ULL, small)); // file 2 opens, touched; cap now at 2
        records.push_back(encodeChunk(3ULL, 0ULL, 128ULL, small)); // opening file 3 evicts the least-recently-touched (file 1)

        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(block._engine->counters().evicted_for_cap, 1ULL));
        expect(eq(capture.out.size(), 0UZ)) << "none of the two-chunk files ever receives its second chunk";
        expect(eq(capture.incomplete.size(), 1UZ)) << "the evicted file, since publish_incomplete is set";
        if (capture.incomplete.size() == 1UZ) {
            expect(eq(metaString(metaOf(capture.incomplete.front()), "file_id").value_or(""), std::string("00000001"))) << "file 1 is the least recently touched";
            expect(eq(metaString(metaOf(capture.incomplete.front()), "discard_reason").value_or(""), std::string("evicted_for_cap")));
        }
    };

    // criterion 7 — max_file_bytes refuses by range, not by allocation, and the guard is checked before the multiply
    "max_file_bytes refuses by range and the guard precedes the multiply"_test = [] {
        constexpr std::uint64_t kBigChunkSize = 1'000'000ULL;
        constexpr std::uint64_t kMaxFileBytes = 100'000'000ULL;
        auto                    block         = make(gr::property_map{
                                       {"chunk_format", std::string("indexed")},
                                       {"id_offset", gr::Size_t{0U}},
                                       {"id_bytes", gr::Size_t{4U}},
                                       {"index_offset", gr::Size_t{4U}},
                                       {"index_bytes", gr::Size_t{8U}},
                                       {"size_offset", gr::Size_t{12U}},
                                       {"size_bytes", gr::Size_t{4U}},
                                       {"payload_offset", gr::Size_t{16U}},
                                       {"chunk_size", gr::Size_t{static_cast<gr::Size_t>(kBigChunkSize)}},
                                       {"max_open_files", gr::Size_t{4U}},
                                       {"max_file_bytes", kMaxFileBytes},
        });

        const Layout                    bigLayout{.idOffset = 0UZ, .idBytes = 4UZ, .indexOffset = 4UZ, .indexBytes = 8UZ, .payloadOffset = 16UZ, .sizeOffset = 12UZ, .sizeBytes = 4UZ};
        const std::vector<std::uint8_t> payload(16UZ, 0x55U);
        // A one-chunk file: chunk 0 is the last chunk, so a payload short of the chunk size is what this file's
        // last chunk is, and the scene is about the index guard rather than about coverage geometry.
        constexpr std::uint64_t kDeclaredSize = 16ULL;

        // An index whose product with the chunk size wraps a 64-bit register: 1 000 000 * ((2^64 - 1)/1e6 + 1) is
        // 448 384 modulo 2^64, which is inside both the declared size and the cap, so an engine that multiplied
        // first would place this chunk at byte 448 384 and accept it. The guard divides instead.
        constexpr std::uint64_t hugeIndex = (std::numeric_limits<std::uint64_t>::max() / kBigChunkSize) + 1ULL;
        static_assert(hugeIndex * kBigChunkSize == 448'384ULL, "the scene only tests the guard if the product wraps");
        static_assert(448'384ULL < kMaxFileBytes, "and if the wrapped product would otherwise be admitted");
        std::vector<Record> records;
        records.push_back(makeRecord(buildBytes(bigLayout, Fields{.id = 9ULL, .index = hugeIndex, .size = kDeclaredSize}, payload)));
        records.push_back(makeRecord(buildBytes(bigLayout, Fields{.id = 9ULL, .index = 0ULL, .size = kDeclaredSize}, payload))); // in range, same file

        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(block._engine->counters().refused_over_max, 1ULL));
        expect(eq(capture.rejected.size(), 1UZ));
        expect(eq(metaString(capture.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("over_max_bytes")));
        expect(eq(block._engine->counters().chunks_written, 1ULL)) << "the in-range chunk for the same file is written; the file is not marked broken";
        expect(eq(capture.out.size(), 1UZ)) << "and it completes the file, so nothing landed at the wrapped product's byte";
        if (!capture.out.empty()) {
            expect(eq(capture.out.front().signal_values.size(), payload.size()));
        }

        // Q3's own cap, refused where the configuration is offered rather than by building a two-gigabyte file.
        expect(throws([] { std::ignore = make(basicSettings(8U, 2'147'483'648ULL)); })) << "max_file_bytes above 2^31 - 1";
        expect(nothrow([] { std::ignore = make(basicSettings(8U, 2'147'483'647ULL)); })) << "and the cap itself is admissible";
    };

    // criterion 8 — max_gaps refuses only chunks that would add a gap (the interval path, via 'offset')
    "max_gaps refuses only chunks that would add a gap"_test = [] {
        const Layout offsetLayout{.idOffset = 0UZ, .idBytes = 4UZ, .offsetOffset = 4UZ, .offsetBytes = 4UZ, .payloadOffset = 12UZ, .sizeOffset = 8UZ, .sizeBytes = 4UZ};
        auto         block = make(gr::property_map{
                    {"chunk_format", std::string("offset")},
                    {"id_offset", gr::Size_t{0U}},
                    {"id_bytes", gr::Size_t{4U}},
                    {"offset_offset", gr::Size_t{4U}},
                    {"offset_bytes", gr::Size_t{4U}},
                    {"size_offset", gr::Size_t{8U}},
                    {"size_bytes", gr::Size_t{4U}},
                    {"payload_offset", gr::Size_t{12U}},
                    {"max_open_files", gr::Size_t{4U}},
                    {"max_file_bytes", std::uint64_t{1UZ << 20UZ}},
                    {"max_gaps", gr::Size_t{4U}},
        });

        constexpr std::size_t           kCellBytes = 8UZ;
        constexpr std::uint64_t         kTotal     = 24ULL * kCellBytes; // 12 covered cells, 12 gaps if every other one lands
        const std::vector<std::uint8_t> cell(kCellBytes, 0x11U);
        std::vector<Record>             records;
        // alternating chunks (even cells only): each is disjoint from every other, so each opens its own interval.
        // max_gaps = 4 caps the interval count at max_gaps + 1 = 5, so five disjoint cells (0,2,4,6,8) fill the cap
        // exactly and are all still accepted.
        for (std::uint64_t cell_i = 0ULL; cell_i < 10ULL; cell_i += 2ULL) {
            records.push_back(makeRecord(buildBytes(offsetLayout, Fields{.id = 1ULL, .offset = cell_i * kCellBytes, .size = kTotal}, cell)));
        }
        const Capture partial = run(block, std::span<const Record>(records));
        expect(eq(block._engine->counters().refused_too_fragmented, 0ULL)) << "five disjoint intervals is exactly the cap, still accepted";

        // a sixth disjoint cell would need a sixth interval: refused
        std::vector<Record> overflow{makeRecord(buildBytes(offsetLayout, Fields{.id = 1ULL, .offset = 10ULL * kCellBytes, .size = kTotal}, cell))};
        const Capture       overflowCapture = run(block, std::span<const Record>(overflow));
        expect(eq(block._engine->counters().refused_too_fragmented, 1ULL));
        expect(eq(overflowCapture.rejected.size(), 1UZ));
        expect(eq(metaString(overflowCapture.rejectTags.at(0UZ).map, "discard_reason").value_or(""), std::string("too_fragmented")));

        // a chunk that fills an existing gap in the same state is accepted (merging is attempted before the cap test)
        std::vector<Record> fillGap{makeRecord(buildBytes(offsetLayout, Fields{.id = 1ULL, .offset = 1ULL * kCellBytes, .size = kTotal}, cell))};
        const Capture       fillCapture = run(block, std::span<const Record>(fillGap));
        expect(eq(fillCapture.rejected.size(), 0UZ)) << "filling a hole is never refused for making one";
        expect(eq(block._engine->counters().refused_too_fragmented, 1ULL)) << "unchanged by the fill";
    };

    // criterion 9 — completion is coverage, not a write pointer
    "completion is coverage, not a write pointer"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        auto                block = make(basicSettings());
        std::vector<Record> records;
        for (const auto& [index, payload] : chunks) {
            if (index == 3ULL) {
                continue; // withheld: the last chunk (which reaches the declared size) still arrives
            }
            records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 0UZ)) << "a write pointer reaching the declared size is not completion";
    };

    // criterion 10 — the two declarations, and the conflict rule
    "the two declarations, and the conflict rule"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        // (a) total_size alone completes — basicSettings() already declares size only
        {
            auto                block = make(basicSettings());
            std::vector<Record> records;
            for (const auto& [index, payload] : chunks) {
                records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
            }
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 1UZ)) << "(a) total_size alone";
        }
        // (b) total_chunks alone completes
        {
            const Layout        countLayout{.idOffset = 0UZ, .idBytes = 4UZ, .indexOffset = 4UZ, .indexBytes = 4UZ, .payloadOffset = 12UZ, .countOffset = 8UZ, .countBytes = 4UZ};
            auto                block = make(gr::property_map{{"chunk_format", std::string("indexed")}, {"id_offset", gr::Size_t{0U}}, {"id_bytes", gr::Size_t{4U}}, {"index_offset", gr::Size_t{4U}}, {"index_bytes", gr::Size_t{4U}}, {"count_offset", gr::Size_t{8U}}, {"count_bytes", gr::Size_t{4U}}, {"payload_offset", gr::Size_t{12U}}, {"chunk_size", gr::Size_t{static_cast<gr::Size_t>(kChunkSize)}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 20UZ}}});
            std::vector<Record> records;
            for (const auto& [index, payload] : chunks) {
                records.push_back(makeRecord(buildBytes(countLayout, Fields{.id = 1ULL, .index = index, .count = kTotalChunks}, payload)));
            }
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 1UZ)) << "(b) total_chunks alone";
            if (!capture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, file));
            }
        }
        // (c) a last-chunk flag alone completes, equivalent to declaring total_chunks = index + 1
        {
            const Layout        flagLayout{.idOffset = 0UZ, .idBytes = 4UZ, .indexOffset = 4UZ, .indexBytes = 4UZ, .payloadOffset = 9UZ, .flagOffset = 8UZ, .flagBit = 0U};
            auto                block = make(gr::property_map{{"chunk_format", std::string("indexed")}, {"id_offset", gr::Size_t{0U}}, {"id_bytes", gr::Size_t{4U}}, {"index_offset", gr::Size_t{4U}}, {"index_bytes", gr::Size_t{4U}}, {"last_flag_offset", gr::Size_t{8U}}, {"last_flag_bit", gr::Size_t{0U}}, {"payload_offset", gr::Size_t{9U}}, {"chunk_size", gr::Size_t{static_cast<gr::Size_t>(kChunkSize)}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 20UZ}}});
            std::vector<Record> records;
            for (const auto& [index, payload] : chunks) {
                records.push_back(makeRecord(buildBytes(flagLayout, Fields{.id = 1ULL, .index = index, .lastChunk = index == kTotalChunks - 1UZ}, payload)));
            }
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 1UZ)) << "(c) a last-chunk flag alone";
            if (!capture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, file));
                expect(eq(metaValue<gr::Size_t>(metaOf(capture.out.front()), "file_chunks").value_or(0U), static_cast<gr::Size_t>(kTotalChunks)));
            }
        }
        // (d) a second, differing declaration on the same chunk index increments nDeclarationConflicts and the
        // first stands; asserted with the conflicting re-declaration inserted early and late in the stream, both
        // producing the same completed file.
        const auto conflictScene = [&](std::size_t insertAfter) {
            auto                block = make(basicSettings());
            std::vector<Record> records;
            for (std::size_t k = 0UZ; k < chunks.size(); ++k) {
                records.push_back(encodeChunk(1ULL, chunks[k].first, kFileBytes, chunks[k].second));
                if (k == insertAfter) { // chunk 0's bytes again, but declaring a different (wrong) total_size
                    records.push_back(encodeChunk(1ULL, 0ULL, kFileBytes + 1ULL, chunks[0UZ].second));
                }
            }
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 1UZ)) << std::format("insertAfter {}", insertAfter);
            expect(eq(block._engine->counters().declaration_conflicts, 1ULL)) << std::format("insertAfter {}", insertAfter);
            if (!capture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, file)) << std::format("insertAfter {}", insertAfter);
            }
        };
        conflictScene(0UZ);                 // right after the first declaration
        conflictScene(chunks.size() - 2UZ); // late, but still before the file's completing chunk — completion
                                            // retires the file at once, so nothing pushed after it could land on
                                            // this same open file (F15's post-completion-reopens rule)
    };

    // criterion 11 — new file on index regression
    "new file on index regression"_test = [] {
        constexpr std::size_t     kCellBytes = 16UZ;
        const Layout              noIdLayout{.indexOffset = 0UZ, .indexBytes = 4UZ, .payloadOffset = 12UZ, .sizeOffset = 4UZ, .sizeBytes = 4UZ};
        std::vector<std::uint8_t> payloadA(16UZ, 0x22U);
        std::vector<std::uint8_t> payloadB(16UZ, 0x33U);
        std::vector<std::uint8_t> both = payloadA;
        both.insert(both.end(), payloadB.begin(), payloadB.end());

        const auto settings = [](bool newFileOnRegression) { return gr::property_map{{"chunk_format", std::string("indexed")}, {"index_offset", gr::Size_t{0U}}, {"index_bytes", gr::Size_t{4U}}, {"size_offset", gr::Size_t{4U}}, {"size_bytes", gr::Size_t{4U}}, {"payload_offset", gr::Size_t{12U}}, {"chunk_size", gr::Size_t{16U}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 16UZ}}, {"publish_incomplete", true}, {"new_file_on_index_regression", newFileOnRegression}}; };

        {
            auto                block = make(settings(true));
            std::vector<Record> records;
            records.push_back(makeRecord(buildBytes(noIdLayout, Fields{.index = 0ULL, .size = 3ULL * 16ULL}, payloadA)));
            records.push_back(makeRecord(buildBytes(noIdLayout, Fields{.index = 1ULL, .size = 3ULL * 16ULL}, payloadB)));
            records.push_back(makeRecord(buildBytes(noIdLayout, Fields{.index = 0ULL, .size = 1ULL * 16ULL}, payloadB))); // regresses: a second file opens
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(block._engine->counters().files_opened, 2ULL)) << "true: a second file opens on regression";
            expect(eq(block.nEvictedSuperseded, 1ULL)) << "the file the transmitter moved on from is a counted, named departure";
            expect(eq(capture.out.size(), 1UZ)) << "the second (1-chunk) file completes";
            if (!capture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, payloadB)) << "and carries the bytes of the chunk that opened it";
                expect(eq(metaString(metaOf(capture.out.front()), "file_id").value_or(""), std::string("anon-1"))) << "the second anonymous file to leave the engine";
            }
            expect(eq(capture.incomplete.size(), 1UZ));
            if (capture.incomplete.size() == 1UZ) {
                const gr::property_map meta = metaOf(capture.incomplete.front());
                expect(eq(metaString(meta, "discard_reason").value_or(""), std::string("evicted_superseded")));
                expect(eq(metaString(meta, "file_id").value_or(""), std::string("anon-0"))) << "the anonymous sequence counts departures, in the order they left";
                expect(eq(metaValue<std::uint64_t>(meta, "file_covered_bytes").value_or(0ULL), 32ULL));
                expect(eq(capture.incomplete.front().signal_values.size(), 3UZ * kCellBytes)) << "storage is the declared size, whatever arrived";
                expect(std::ranges::equal(std::span<const std::uint8_t>(capture.incomplete.front().signal_values).first(both.size()), std::span<const std::uint8_t>(both))) << "both chunks landed where their indices said";
            }
        }
        {
            auto                block = make(settings(false));
            std::vector<Record> records;
            records.push_back(makeRecord(buildBytes(noIdLayout, Fields{.index = 1ULL, .size = 2ULL * 16ULL}, payloadB)));
            records.push_back(makeRecord(buildBytes(noIdLayout, Fields{.index = 0ULL, .size = 2ULL * 16ULL}, payloadA))); // false: placed by index in the current file
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(block._engine->counters().files_opened, 1ULL)) << "false: the regressing chunk is placed in the current file";
            expect(eq(block.nEvictedSuperseded, 0ULL));
            expect(eq(capture.out.size(), 1UZ)) << "and completes it";
            if (!capture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, both)) << "index 0 ahead of index 1, whichever arrived first";
                expect(eq(metaString(metaOf(capture.out.front()), "file_id").value_or(""), std::string("anon-0")));
            }
        }
    };

    // criterion 12 — every emitted packet is admissible by the Tier-1 predicates
    "every emitted packet is admissible by the Tier-1 predicates"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        auto                block = make(basicSettings(8U, 1UZ << 20UZ));
        std::vector<Record> records;
        for (std::uint64_t id = 1ULL; id <= 3ULL; ++id) {
            for (const auto& [index, payload] : chunks) {
                records.push_back(encodeChunk(id, index, kFileBytes, payload));
            }
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 3UZ));

        for (const PacketU& packet : capture.out) {
            expect(eq(packet.meta_information.size(), 1UZ)) << "Q1";
            expect(ge(packet.signal_values.size(), 1UZ)) << "Q2";
            expect(le(packet.signal_values.size(), 2'147'483'647UZ)) << "Q3";
        }

        // fed through the landed PacketToDataSet in one scheduler graph: a real port is what that block's own tag
        // handling needs, which a bare span mock does not supply.
        gr::test::RuntimeTest test;
        auto&                 source = test.emplace<ItemSource<PacketU>>();
        source._items                = capture.out;
        auto& convert                = test.emplace<gr::blocks::basic::PacketToDataSet<std::uint8_t>>(gr::property_map{});
        auto& admitted               = test.emplace<GraphCollector<gr::DataSet<std::uint8_t>>>();
        auto& refused                = test.emplace<GraphCollector<PacketU>>();
        std::ignore                  = test.connect(source, "out", convert, "in");
        std::ignore                  = test.connect(convert, "out", admitted, "in");
        std::ignore                  = test.connect(convert, "reject", refused, "in");
        std::ignore                  = test.run();

        expect(eq(admitted._items.size(), capture.out.size())) << "every packet admitted";
        expect(eq(refused._items.size(), 0UZ)) << "none on reject";
    };

    // criterion 13 — the degenerate descriptor's continuity rule (kernel-direct: unreachable through the block, §6.3,
    // because both shipped formats always populate their own defining field, index or offset)
    "the degenerate descriptor's continuity rule"_test = [] {
        using gr::packet::ChunkDescriptor;
        constexpr std::size_t                   kStep = 32UZ;
        constexpr gr::packet::OffsetChunkFormat kHookFormat{.offset = gr::packet::FieldSpec{0UZ, 4UZ}, .total_size = gr::packet::FieldSpec{4UZ, 4UZ}, .payload_offset = 8UZ};

        // (a) no index, no offset: contiguous chunks assemble exactly.
        {
            // The format is never read here — pushDescriptor is the route a protocol's own parser takes — but the
            // engine still validates the one it was configured with, so it names a field layout that could be read.
            gr::packet::ChunkReassembler::Config config{.format = kHookFormat, .max_open_files = 4UZ, .max_file_bytes = 1UZ << 16UZ};
            gr::packet::ChunkReassembler         engine(config);
            const std::vector<std::uint8_t>      file = referenceFile(160UZ);

            ChunkDescriptor base;
            base.total_size = file.size();
            std::optional<std::uint64_t> completedId;
            for (std::size_t offset = 0UZ; offset < file.size(); offset += kStep) {
                ChunkDescriptor descriptor = base;
                descriptor.payload_begin   = 0UZ;
                descriptor.payload_end     = kStep;
                const std::vector<std::uint8_t> chunk(file.begin() + static_cast<std::ptrdiff_t>(offset), file.begin() + static_cast<std::ptrdiff_t>(offset + kStep));
                const auto                      result = engine.pushDescriptor(descriptor, std::span<const std::uint8_t>(chunk));
                expect(result.status == gr::packet::ChunkReassembler::Status::accepted || result.status == gr::packet::ChunkReassembler::Status::completed) << std::format("offset {}", offset);
                if (result.status == gr::packet::ChunkReassembler::Status::completed) {
                    completedId = result.completed_id;
                }
            }
            expect(eq(engine.counters().refused_no_position, 0ULL));
            expect(completedId.has_value());
            if (completedId.has_value()) {
                expect(std::ranges::equal(engine.file(*completedId), file)) << "contiguous chunks assemble exactly";
            }
        }

        // (b) a hole in coverage (opened here through an explicit offset, since a purely degenerate sequence can
        // never itself create one — every accepted degenerate write extends the write pointer by construction) and
        // the next degenerate push against it is 'no_position', counted, and the file is evicted in that same call
        // with its coverage reported.
        {
            gr::packet::ChunkReassembler::Config config{.format = kHookFormat, .max_open_files = 4UZ, .max_file_bytes = 1UZ << 16UZ};
            gr::packet::ChunkReassembler         engine(config);
            const std::vector<std::uint8_t>      file = referenceFile(160UZ);

            ChunkDescriptor first;
            first.total_size    = file.size();
            first.offset        = 0ULL;
            first.payload_begin = 0UZ;
            first.payload_end   = kStep;
            const std::vector<std::uint8_t> firstChunk(file.begin(), file.begin() + static_cast<std::ptrdiff_t>(kStep));
            expect(engine.pushDescriptor(first, std::span<const std::uint8_t>(firstChunk)).status == gr::packet::ChunkReassembler::Status::accepted);

            ChunkDescriptor holeAhead;
            holeAhead.total_size    = file.size();
            holeAhead.offset        = 2ULL * kStep; // skips [kStep, 2*kStep): a hole
            holeAhead.payload_begin = 0UZ;
            holeAhead.payload_end   = kStep;
            const std::vector<std::uint8_t> aheadChunk(file.begin() + 2 * static_cast<std::ptrdiff_t>(kStep), file.begin() + 3 * static_cast<std::ptrdiff_t>(kStep));
            expect(engine.pushDescriptor(holeAhead, std::span<const std::uint8_t>(aheadChunk)).status == gr::packet::ChunkReassembler::Status::accepted);

            ChunkDescriptor degenerate; // neither offset nor index
            degenerate.total_size    = file.size();
            degenerate.payload_begin = 0UZ;
            degenerate.payload_end   = kStep;
            const std::vector<std::uint8_t> anyChunk(kStep, 0x00U);
            const auto                      result = engine.pushDescriptor(degenerate, std::span<const std::uint8_t>(anyChunk));
            expect(result.status == gr::packet::ChunkReassembler::Status::refused);
            expect(result.reason == gr::packet::ChunkReassembler::RefusalReason::no_position);
            expect(eq(engine.counters().refused_no_position, 1ULL));
            expect(eq(engine.departed().size(), 1UZ)) << "the file is evicted in the same call";
            if (!engine.departed().empty()) {
                const gr::packet::FileFacts facts = engine.facts(engine.departed().front());
                expect(facts.reason == gr::packet::EvictReason::no_position);
                expect(eq(facts.covered_bytes, 2ULL * kStep)) << "the two written chunks, coverage reported at eviction";
            }
        }
    };

    // criterion 14 — chunk independence and CRC accounting
    "chunk independence and CRC accounting"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        std::vector<Record> records;
        for (const auto& [index, payload] : chunks) {
            records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
        }
        auto          reference        = make(basicSettings());
        const Capture referenceCapture = run(reference, std::span<const Record>(records));
        expect(eq(referenceCapture.out.size(), 1UZ));

        // the same records handed to processBulk in spans of 1, 7 and 4096: the file is byte-identical, which is
        // what chunk independence means for a block whose input span is its whole unit of work
        for (const std::size_t inputChunk : {1UZ, 7UZ, 4096UZ}) {
            auto          block   = make(basicSettings());
            const Capture capture = run(block, std::span<const Record>(records), true, true, true, 0UZ, inputChunk);
            expect(eq(capture.out.size(), 1UZ)) << std::format("input chunk {}", inputChunk);
            expect(eq(capture.calls, (records.size() + inputChunk - 1UZ) / inputChunk)) << std::format("input chunk {}: the records really arrived in spans of that size", inputChunk);
            if (!capture.out.empty() && !referenceCapture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, referenceCapture.out.front().signal_values)) << std::format("input chunk {}", inputChunk);
            }
        }

        // and once through a scheduler graph, where the ports and their buffers are real and the source hands the
        // graph seven records at a time; 'incomplete' and 'reject' are left unconnected, which is a graph a
        // recipe would build
        {
            gr::test::RuntimeTest test;
            auto&                 source = test.emplace<ItemSource<Record>>();
            source._items                = records;
            source._maxPerCall           = 7UZ;
            auto& reassembler            = test.emplace<ChunkReassembler>(basicSettings());
            auto& collected              = test.emplace<GraphCollector<PacketU>>();
            std::ignore                  = test.connect(source, "out", reassembler, "in");
            std::ignore                  = test.connect(reassembler, "out", collected, "in");
            std::ignore                  = test.run();

            expect(eq(collected._items.size(), 1UZ)) << "under the scheduler";
            if (!collected._items.empty() && !referenceCapture.out.empty()) {
                expect(std::ranges::equal(collected._items.front().signal_values, referenceCapture.out.front().signal_values)) << "under the scheduler, byte for byte";
            }
        }

        // three contributing records carry crc_ok = false: admitted, and the file says so
        {
            auto                block = make(basicSettings());
            std::vector<Record> crcRecords;
            for (std::size_t k = 0UZ; k < chunks.size(); ++k) {
                const std::optional<bool> crcOk = k < 3UZ ? std::optional<bool>(false) : std::nullopt;
                crcRecords.push_back(makeRecord(buildBytes(kBasicLayout, Fields{.id = 1ULL, .index = chunks[k].first, .size = kFileBytes}, chunks[k].second), crcOk));
            }
            const Capture capture = run(block, std::span<const Record>(crcRecords));
            expect(eq(capture.out.size(), 1UZ));
            expect(eq(block._engine->counters().chunks_crc_failed, 3ULL));
            if (!capture.out.empty()) {
                expect(eq(metaValue<bool>(metaOf(capture.out.front()), "file_all_chunks_crc_ok").value_or(true), false));
            }
        }
        // the same scene at require_crc_ok = true: no completed file, three records on reject
        {
            auto                block = make(basicSettings(8U, 1UZ << 20UZ, 0ULL, true));
            std::vector<Record> crcRecords;
            for (std::size_t k = 0UZ; k < chunks.size(); ++k) {
                const std::optional<bool> crcOk = k < 3UZ ? std::optional<bool>(false) : std::nullopt;
                crcRecords.push_back(makeRecord(buildBytes(kBasicLayout, Fields{.id = 1ULL, .index = chunks[k].first, .size = kFileBytes}, chunks[k].second), crcOk));
            }
            const Capture capture = run(block, std::span<const Record>(crcRecords));
            expect(eq(capture.out.size(), 0UZ));
            expect(eq(block._engine->counters().refused_crc_failed, 3ULL));
            expect(eq(capture.rejected.size(), 3UZ));
        }
    };

    // §7.2's refusals, each one offered to the block and refused where it is offered
    "every configuration the block refuses, refused where it is offered"_test = [] {
        const auto staged = [](gr::property_map settings) { std::ignore = make(std::move(settings)); };
        const auto with   = [](std::string_view key, gr::pmt::Value value) {
            gr::property_map settings = basicSettings();
            settings.insert_or_assign(gr::property_map::key_type(key), std::move(value));
            return settings;
        };

        expect(nothrow([&staged] { staged(basicSettings()); })) << "the scene's own settings are admissible";

        expect(throws([] {
            ChunkReassembler block{};
            block.start();
        })) << "chunk_format is required and has no default, so an unconfigured block refuses to start";
        expect(throws([&staged, &with] { staged(with("chunk_format", gr::pmt::Value(std::string("index")))); })) << "neither of the two spellings";

        expect(throws([&staged, &with] { staged(with("max_open_files", gr::pmt::Value(gr::Size_t{0U}))); })) << "max_open_files 0 is the unset state";
        expect(throws([&staged, &with] { staged(with("max_file_bytes", gr::pmt::Value(std::uint64_t{0ULL}))); })) << "max_file_bytes 0 is the unset state";
        expect(throws([&staged, &with] { staged(with("max_file_bytes", gr::pmt::Value(std::uint64_t{2'147'483'648ULL}))); })) << "and above the record carrier's extent";
        expect(throws([&staged, &with] { staged(with("max_gaps", gr::pmt::Value(gr::Size_t{0U}))); })) << "max_gaps 0 leaves the interval path no room";

        expect(throws([&staged, &with] { staged(with("offset_bytes", gr::pmt::Value(gr::Size_t{4U}))); })) << "an offset field under 'indexed' is refused, not ignored";
        expect(throws([&staged] { staged(gr::property_map{{"chunk_format", std::string("offset")}, {"offset_offset", gr::Size_t{0U}}, {"offset_bytes", gr::Size_t{4U}}, {"index_bytes", gr::Size_t{4U}}, {"size_offset", gr::Size_t{4U}}, {"size_bytes", gr::Size_t{4U}}, {"payload_offset", gr::Size_t{8U}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 16UZ}}}); })) << "an index field under 'offset' likewise";
        expect(throws([&staged] { staged(gr::property_map{{"chunk_format", std::string("offset")}, {"offset_offset", gr::Size_t{0U}}, {"offset_bytes", gr::Size_t{0U}}, {"payload_offset", gr::Size_t{8U}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 16UZ}}}); })) << "the offset format's defining field is required";
        expect(throws([&staged, &with] { staged(with("index_bytes", gr::pmt::Value(gr::Size_t{0U}))); })) << "the indexed format's defining field is required";
        expect(throws([&staged, &with] { staged(with("chunk_size", gr::pmt::Value(gr::Size_t{0U}))); })) << "an index means nothing without a chunk size";

        expect(throws([&staged, &with] { staged(with("id_bytes", gr::pmt::Value(gr::Size_t{9U}))); })) << "a field wider than eight bytes does not fit a 64-bit value";
        expect(throws([&staged, &with] { staged(with("payload_offset", gr::pmt::Value(gr::Size_t{4U}))); })) << "a header field reaching into the payload is a configuration error";
        expect(throws([&staged, &with] { staged(with("size_offset", gr::pmt::Value(gr::Size_t{6U}))); })) << "the size field would overlap the index field";
        expect(throws([&staged, &with] { staged(with("size_bytes", gr::pmt::Value(gr::Size_t{0U}))); })) << "no size, no count and no flag: this configuration can never complete a file";
        expect(throws([&staged] { staged(gr::property_map{{"chunk_format", std::string("offset")}, {"offset_offset", gr::Size_t{0U}}, {"offset_bytes", gr::Size_t{4U}}, {"count_offset", gr::Size_t{4U}}, {"count_bytes", gr::Size_t{4U}}, {"payload_offset", gr::Size_t{8U}}, {"chunk_size", gr::Size_t{16U}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 16UZ}}}); })) << "a chunk count names cells an offset-addressed file does not have, so it cannot complete one";

        expect(throws([&staged, &with] { staged(with("field_byte_order", gr::pmt::Value(std::string("middle")))); })) << "byte order is stated or refused, never guessed";
        expect(throws([&staged, &with] { staged(with("last_flag_bit", gr::pmt::Value(gr::Size_t{9U}))); })) << "a bit above 8 would silently disable the flag";
        expect(nothrow([&staged, &with] { staged(with("last_flag_bit", gr::pmt::Value(gr::Size_t{8U}))); })) << "8 is the documented spelling of 'no flag'";

        // a refused configuration leaves the block inert rather than half-configured
        ChunkReassembler          inert{};
        const std::vector<Record> records{encodeChunk(1ULL, 0ULL, kFileBytes, std::vector<std::uint8_t>(kChunkSize, 0x01U))};
        std::vector<PacketU>      outScratch(1UZ);
        std::vector<PacketU>      incScratch(1UZ);
        std::vector<Record>       rejScratch(1UZ);
        InSpan<Record>            inSpan(std::span<const Record>(records), 0UZ);
        OutSpan<PacketU>          outSpan(std::span<PacketU>(outScratch), 0UZ, nullptr, true);
        OutSpan<PacketU>          incSpan(std::span<PacketU>(incScratch), 0UZ, nullptr, true);
        OutSpan<Record>           rejSpan(std::span<Record>(rejScratch), 0UZ, nullptr, true);
        expect(inert.processBulk(inSpan, outSpan, incSpan, rejSpan) == gr::work::Status::ERROR);
        expect(eq(inSpan.consumed, 0UZ));
        expect(eq(outSpan.count, 0UZ));
    };

    // §7.4's table, on the four reasons no numbered criterion reaches
    "unparsable, zero size, beyond the declared size and a bad payload span each name themselves"_test = [] {
        auto                            block = make(basicSettings());
        const std::vector<std::uint8_t> full(kChunkSize, 0x77U);
        const std::vector<std::uint8_t> half(kChunkSize / 2UZ, 0x77U);

        std::vector<Record> records;
        records.push_back(makeRecord(std::vector<std::uint8_t>(5UZ, 0U))); // shorter than the header: unparsable
        records.push_back(encodeChunk(1ULL, 0ULL, 0ULL, full));            // a declared size of zero
        records.push_back(encodeChunk(1ULL, 40ULL, kFileBytes, full));     // 40 * 224 is past the declared 8192
        records.push_back(encodeChunk(1ULL, 0ULL, kFileBytes, half));      // half a cell at an index that is not the last
        records.push_back(encodeChunk(1ULL, 1ULL, kFileBytes, full));      // and the block carries on

        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.rejected.size(), 4UZ));
        const std::vector<std::string> expected{"unparsable", "zero_size", "offset_beyond_size", "bad_payload_span"};
        for (std::size_t k = 0UZ; k < std::min(expected.size(), capture.rejectTags.size()); ++k) {
            expect(eq(metaString(capture.rejectTags[k].map, "discard_reason").value_or(""), expected[k])) << expected[k];
        }
        const gr::packet::ChunkReassembler::Counters& counters = block._engine->counters();
        expect(eq(counters.refused_unparsable, 1ULL));
        expect(eq(counters.refused_zero_size, 1ULL));
        expect(eq(counters.refused_beyond_size, 1ULL));
        expect(eq(counters.refused_payload_span, 1ULL));
        expect(eq(counters.chunks_written, 1ULL)) << "the record after four refusals is written";
        if (capture.rejected.size() == 4UZ) {
            expect(std::ranges::equal(capture.rejected[1UZ].signal_values, records[1UZ].signal_values)) << "the refused record is republished unchanged";
        }
    };

    "a little-endian field is read the other way, and a trailer is dropped before the payload is placed"_test = [] {
        constexpr std::size_t kCell = 16UZ;
        const auto            file  = referenceFile(2UZ * kCell);

        // id (2) | index (2) | total size (4) | payload (16) | trailer (4), every field little-endian
        const auto record = [&file](std::uint64_t index) {
            std::vector<std::uint8_t> bytes(8UZ + kCell + 4UZ, 0xEEU);
            const auto                putLittleEndian = [&bytes](std::size_t at, std::size_t width, std::uint64_t value) {
                for (std::size_t i = 0UZ; i < width; ++i) {
                    bytes[at + i] = static_cast<std::uint8_t>((value >> (8UZ * i)) & 0xFFULL);
                }
            };
            putLittleEndian(0UZ, 2UZ, 1ULL);
            putLittleEndian(2UZ, 2UZ, index);
            putLittleEndian(4UZ, 4UZ, 2ULL * kCell);
            std::ranges::copy(std::span{file}.subspan(static_cast<std::size_t>(index) * kCell, kCell), bytes.begin() + 8);
            return makeRecord(bytes);
        };
        const std::vector<Record> records{record(0ULL), record(1ULL)};

        const auto settings = [](std::string byteOrder, gr::Size_t trim) { return gr::property_map{{"chunk_format", std::string("indexed")}, {"field_byte_order", std::move(byteOrder)}, {"id_offset", gr::Size_t{0U}}, {"id_bytes", gr::Size_t{2U}}, {"index_offset", gr::Size_t{2U}}, {"index_bytes", gr::Size_t{2U}}, {"size_offset", gr::Size_t{4U}}, {"size_bytes", gr::Size_t{4U}}, {"payload_offset", gr::Size_t{8U}}, {"payload_trim", trim}, {"chunk_size", gr::Size_t{static_cast<gr::Size_t>(kCell)}}, {"max_open_files", gr::Size_t{4U}}, {"max_file_bytes", std::uint64_t{1UZ << 20UZ}}}; };

        {
            auto          block   = make(settings("little", 4U));
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 1UZ)) << "little: the fields read as they were written";
            if (!capture.out.empty()) {
                expect(std::ranges::equal(capture.out.front().signal_values, file)) << "and the 0xEE trailer is not in the file";
                expect(eq(metaString(metaOf(capture.out.front()), "file_id").value_or(""), std::string("0001")));
            }
        }
        {
            auto          block   = make(settings("big", 4U));
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 0UZ)) << "big: an index of 1 written little-endian reads as 256, and nothing completes";
        }
        {
            auto          block   = make(settings("little", 0U));
            const Capture capture = run(block, std::span<const Record>(records));
            expect(eq(capture.out.size(), 0UZ));
            expect(eq(capture.rejected.size(), 2UZ));
            expect(eq(block._engine->counters().refused_payload_span, 1ULL)) << "untrimmed, chunk 0's payload is 20 bytes against a 16-byte cell";
            expect(eq(block._engine->counters().refused_beyond_size, 1ULL)) << "and chunk 1's twenty bytes reach past the declared 32";
        }
    };

    "a settings change evicts what is open, and the next call through processBulk hands it to incomplete"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        auto                block = make(basicSettings(8U, 1UZ << 20UZ, 0ULL, false, true));
        std::vector<Record> opening;
        for (std::size_t k = 0UZ; k < 3UZ; ++k) {
            opening.push_back(encodeChunk(1ULL, chunks[k].first, kFileBytes, chunks[k].second));
        }
        const Capture first = run(block, std::span<const Record>(opening));
        expect(eq(first.out.size(), 0UZ));
        expect(eq(block._engine->openFiles(), 1UZ));

        std::ignore = block.settings().setStaged({{"max_open_files", gr::Size_t{4U}}});
        std::ignore = block.settings().applyStagedParameters();
        expect(eq(block.nEvictedOnReconfigure, 1ULL)) << "no setting is live: the open file belonged to the previous configuration";
        expect(eq(block._engine->openFiles(), 0UZ));
        expect(eq(block._engine->counters().files_opened, 1ULL)) << "the counters carry across a reconfiguration, so one stop() line covers the whole run";

        const std::vector<Record> more{encodeChunk(2ULL, 0ULL, kFileBytes, chunks[0UZ].second)};
        const Capture             second = run(block, std::span<const Record>(more));
        expect(eq(second.incomplete.size(), 1UZ)) << "the eviction reaches the port on the first call that owns its span";
        if (second.incomplete.size() == 1UZ) {
            const gr::property_map meta = metaOf(second.incomplete.front());
            expect(eq(metaString(meta, "discard_reason").value_or(""), std::string("evicted_on_reconfigure")));
            expect(eq(metaString(meta, "file_id").value_or(""), std::string("00000001"))) << "rendered under the identifier width that produced it";
            expect(eq(metaValue<std::uint64_t>(meta, "file_covered_bytes").value_or(0ULL), 3ULL * kChunkSize));
        }
    };

    "end of stream retires what is still open and counts it"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);

        auto                block = make(basicSettings(8U, 1UZ << 20UZ, 0ULL, false, true));
        std::vector<Record> records;
        for (std::size_t k = 0UZ; k < 10UZ; ++k) {
            records.push_back(encodeChunk(1ULL, chunks[k].first, kFileBytes, chunks[k].second));
        }
        const Capture capture = run(block, std::span<const Record>(records));
        expect(eq(capture.out.size(), 0UZ));

        block.stop();
        expect(eq(block._engine->counters().incomplete_at_stop, 1ULL)) << "a partial file is never published on out, and the operator learns what the pass lost";
        expect(eq(block.nIncompleteDroppedNoRoom, 0ULL)) << "an unconnected 'incomplete' is the departure's own counted drop, not a room failure";
    };

    "each output unconnected is a counted drop and never a stall"_test = [] {
        const std::vector<std::uint8_t> file   = referenceFile(kFileBytes);
        const auto                      chunks = splitIntoChunks(file, kChunkSize);
        std::vector<Record>             records;
        for (const auto& [index, payload] : chunks) {
            records.push_back(encodeChunk(1ULL, index, kFileBytes, payload));
        }

        { // 'out' unconnected: the file still completes and the records are still consumed
            auto          block   = make(basicSettings());
            const Capture capture = run(block, std::span<const Record>(records), false, true, true);
            expect(!capture.stalled);
            expect(eq(capture.consumed, records.size()));
            expect(eq(capture.out.size(), 0UZ));
            expect(eq(block._engine->counters().files_completed, 1ULL));
        }
        { // 'incomplete' unconnected under publish_incomplete: the eviction is counted and named, and nothing is held
            auto                            block = make(basicSettings(2U, 1UZ << 16UZ, 0ULL, false, true));
            const std::vector<std::uint8_t> small(64UZ, 0xAAU);
            const std::vector<Record>       three{encodeChunk(1ULL, 0ULL, 128ULL, small), encodeChunk(2ULL, 0ULL, 128ULL, small), encodeChunk(3ULL, 0ULL, 128ULL, small)};
            const Capture                   capture = run(block, std::span<const Record>(three), true, false, true);
            expect(!capture.stalled);
            expect(eq(capture.consumed, three.size()));
            expect(eq(capture.incomplete.size(), 0UZ));
            expect(eq(block._engine->counters().evicted_for_cap, 1ULL));
            expect(eq(block.nIncompleteDroppedNoRoom, 0ULL)) << "an unconnected port is not a port that ran out of room";
        }
        { // 'reject' unconnected: the refusal is the kernel's own count and the block does not stall on it
            auto                      block = make(basicSettings());
            const std::vector<Record> refused{encodeChunk(1ULL, 0ULL, 0ULL, std::vector<std::uint8_t>(kChunkSize, 0x01U))};
            const Capture             capture = run(block, std::span<const Record>(refused), true, true, false);
            expect(!capture.stalled);
            expect(eq(capture.consumed, 1UZ));
            expect(eq(capture.rejected.size(), 0UZ));
            expect(eq(block._engine->counters().refused_zero_size, 1ULL));
        }
    };
};

int main() { /* not needed for UT */ }
