#ifndef GNURADIO_BASIC_CHUNK_REASSEMBLER_HPP
#define GNURADIO_BASIC_CHUNK_REASSEMBLER_HPP

#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/packet/ChunkReassembly.hpp>

namespace gr::blocks::basic {

namespace detail::chunk_reassembler {

[[nodiscard]] inline gr::packet::ByteOrder byteOrderFromName(std::string_view name) {
    if (name == "big") {
        return gr::packet::ByteOrder::big;
    }
    if (name == "little") {
        return gr::packet::ByteOrder::little;
    }
    throw gr::exception(std::format("field_byte_order must be 'big' or 'little', got '{}'", name));
}

[[nodiscard]] inline std::string_view refusalReasonName(gr::packet::ChunkReassembler::RefusalReason reason) noexcept {
    using enum gr::packet::ChunkReassembler::RefusalReason;
    switch (reason) {
    case unparsable: return "unparsable";
    case bad_payload_span: return "bad_payload_span";
    case zero_size: return "zero_size";
    case offset_beyond_size: return "offset_beyond_size";
    case over_max_bytes: return "over_max_bytes";
    case too_fragmented: return "too_fragmented";
    case no_position: return "no_position";
    case content_conflict: return "content_conflict";
    case crc_failed: return "crc_failed";
    default: return "none";
    }
}

/// @brief The `discard_reason` of an evicted, incomplete file on `incomplete`. `evicted_superseded` names the
/// index-regression case, `EvictReason::superseded`. That case is named and counted like every other eviction.
[[nodiscard]] inline std::string_view evictReasonName(gr::packet::EvictReason reason) noexcept {
    using enum gr::packet::EvictReason;
    switch (reason) {
    case cap: return "evicted_for_cap";
    case stale: return "evicted_stale";
    case no_position: return "evicted_no_position";
    case reconfigure: return "evicted_on_reconfigure";
    case at_stop: return "incomplete_at_stop";
    case superseded: return "evicted_superseded";
    default: return "none";
    }
}

/// @brief `value` rendered as lowercase hexadecimal, most significant byte first, exactly `2 * idBytes` characters.
[[nodiscard]] inline std::string toLowerHex(std::uint64_t value, std::size_t idBytes) { return std::format("{:0{}x}", value, 2UZ * idBytes); }

/// @brief Narrows a wider count to `gr::Size_t`, saturating at the largest `gr::Size_t`.
[[nodiscard]] inline gr::Size_t saturateToSize(std::uint64_t value) noexcept { return static_cast<gr::Size_t>(std::min(value, static_cast<std::uint64_t>(std::numeric_limits<gr::Size_t>::max()))); }

} // namespace detail::chunk_reassembler

GR_REGISTER_BLOCK(gr::blocks::basic::ChunkReassembler)

struct ChunkReassembler : Block<ChunkReassembler, NoTagPropagation> {
    using Description = Doc<R""(
@brief Reassembles a file delivered as chunks in ordinary frames into one gr::Packet<uint8_t>, in bounded memory.

The block wraps `gr::packet::ChunkReassembler` from `algorithm/packet/ChunkReassembly.hpp`. The engine reads three
facts from each chunk: the file identifier, the chunk position and the declared file extent. `chunk_format` selects
one of two header formats. `"indexed"` carries a 0-based chunk index over a configured `chunk_size`. `"offset"`
carries the byte position of the chunk in the file. `chunk_format` has no default, because the two formats read
different headers. A setting that belongs to the other format is refused with an error that names both.

`max_open_files` and `max_file_bytes` are required, and zero is refused for both. Together they bound the memory held
by all files in flight. The offsets, declared sizes and chunk counts that size a file come from received data.
`max_file_bytes` above `2^31 - 1` is refused. Every emitted packet then fits the `Packet<T>` extent bound. A file is
complete when its chunks cover every byte. A file with a hole stays incomplete, however far its last chunk reached.
Each open file holds one bitmap or one interval set to track its coverage.

A refused chunk is counted. It is republished unchanged on `reject` when that port is connected, with a
`discard_reason` tag that names the refusal. With `reject` unconnected the chunk is counted and dropped, and the block
does not stall. A completed file is published on `out` as a `Packet<uint8_t>` that holds the whole file. An evicted,
incomplete file is published on `incomplete` when `publish_incomplete` is set and the port is connected. Otherwise it
is counted under its reason and dropped. `in` is synchronous. Each call consumes the records it has read. The block
keeps no cursor and holds no record between calls.

No setting is live. A settings change resets the engine and evicts every open file under the `publish_incomplete`
rule. A chunk format and a memory bound belong to one link. The reset keeps bytes read under the previous
configuration out of a file completed under the new one. The evicted files are named under the configuration that
produced them. The next `processBulk` call publishes them on `incomplete`, because that call holds the port's output
span. At end of stream every open file is incomplete. It is published on `incomplete` or counted in the engine's
`incomplete_at_stop` counter, as a live eviction is. It is not published on `out`.
)"">;

    PortIn<DataSet<std::uint8_t>>                   in;
    PortOut<Packet<std::uint8_t>, Async>            out;
    PortOut<Packet<std::uint8_t>, Async, Optional>  incomplete;
    PortOut<DataSet<std::uint8_t>, Async, Optional> reject;

    Annotated<std::string, "chunk_format", Doc<"'indexed' or 'offset', required">, Visible>                                          chunk_format{};
    Annotated<std::string, "field_byte_order", Doc<"'big' (default) or 'little': the byte order of every multi-byte header field">>  field_byte_order{"big"};
    Annotated<gr::Size_t, "id_offset", Unit<"byte">, Doc<"the identifier field's byte position">>                                    id_offset{0U};
    Annotated<gr::Size_t, "id_bytes", Unit<"byte">, Doc<"the identifier field's width in [0,8], 0 when absent">>                     id_bytes{0U};
    Annotated<gr::Size_t, "index_offset", Unit<"byte">, Doc<"the chunk-index field's byte position in the 'indexed' format">>        index_offset{0U};
    Annotated<gr::Size_t, "index_bytes", Unit<"byte">, Doc<"the chunk-index field's width in [1,8], required for 'indexed'">>        index_bytes{0U};
    Annotated<gr::Size_t, "offset_offset", Unit<"byte">, Doc<"the byte-offset field's byte position in the 'offset' format">>        offset_offset{0U};
    Annotated<gr::Size_t, "offset_bytes", Unit<"byte">, Doc<"the byte-offset field's width in [1,8], required for 'offset'">>        offset_bytes{0U};
    Annotated<gr::Size_t, "count_offset", Unit<"byte">, Doc<"the declared chunk-count field's byte position">>                       count_offset{0U};
    Annotated<gr::Size_t, "count_bytes", Unit<"byte">, Doc<"the declared chunk-count field's width in [0,8], 0 when absent">>        count_bytes{0U};
    Annotated<gr::Size_t, "size_offset", Unit<"byte">, Doc<"the declared byte-size field's byte position">>                          size_offset{0U};
    Annotated<gr::Size_t, "size_bytes", Unit<"byte">, Doc<"the declared byte-size field's width in [0,8], 0 when absent">>           size_bytes{0U};
    Annotated<gr::Size_t, "last_flag_offset", Unit<"byte">, Doc<"the last-chunk flag's byte position">>                              last_flag_offset{0U};
    Annotated<gr::Size_t, "last_flag_bit", Doc<"the last-chunk flag's bit in [0,7], 8 (default) to disable">>                        last_flag_bit{8U};
    Annotated<gr::Size_t, "payload_offset", Unit<"byte">, Doc<"header bytes skipped at the head of every record">>                   payload_offset{0U};
    Annotated<gr::Size_t, "payload_trim", Unit<"byte">, Doc<"trailer bytes dropped from the tail of every record">>                  payload_trim{0U};
    Annotated<gr::Size_t, "chunk_size", Unit<"byte">, Doc<"chunk size, required for 'indexed', a consistency check for 'offset'">>   chunk_size{0U};
    Annotated<gr::Size_t, "max_open_files", Doc<"incomplete files held at once, required, 0 refused">, Visible>                      max_open_files{0U};
    Annotated<std::uint64_t, "max_file_bytes", Unit<"byte">, Doc<"largest file assembled, required, from 1 to 2147483647">, Visible> max_file_bytes{0ULL};
    Annotated<gr::Size_t, "max_gaps", Doc<"gap cap of the interval path, 0 refused">>                                                max_gaps{1024U};
    Annotated<std::uint64_t, "evict_after_records", Doc<"records a file may sit idle before stale eviction, 0 disables">>            evict_after_records{0ULL};
    Annotated<bool, "require_crc_ok", Doc<"refuse a chunk whose record states crc_ok = false">>                                      require_crc_ok{false};
    Annotated<bool, "publish_incomplete", Doc<"also publish an evicted, incomplete file on 'incomplete'">>                           publish_incomplete{false};
    Annotated<bool, "new_file_on_index_regression", Doc<"start a new file on index regression in an identifier-free format">>        new_file_on_index_regression{true};

    GR_MAKE_REFLECTABLE(ChunkReassembler, in, out, incomplete, reject, //
        chunk_format, field_byte_order, id_offset, id_bytes, index_offset, index_bytes, offset_offset, offset_bytes, count_offset, count_bytes, size_offset, size_bytes, last_flag_offset, last_flag_bit, payload_offset, payload_trim, chunk_size, max_open_files, max_file_bytes, max_gaps, evict_after_records, require_crc_ok, publish_incomplete, new_file_on_index_regression);

    // Drop counts the engine does not keep. The block's own thread reads them, and stop() reports them once with the
    // engine's counters.
    std::uint64_t nEvictedOnReconfigure    = 0ULL; ///< files evicted because a settings change reset the engine
    std::uint64_t nEvictedSuperseded       = 0ULL; ///< files superseded by an index regression (EvictReason::superseded)
    std::uint64_t nIncompleteDroppedNoRoom = 0ULL; ///< an incomplete file the 'incomplete' port had no room for

    std::optional<gr::packet::ChunkReassembler> _engine{};
    gr::Size_t                                  _idBytes            = 0U;
    std::uint64_t                               _nextAnonDisplay    = 0ULL;  ///< the next "anon-N" number, see renderFileId()
    bool                                        _degenerateIdentity = false; ///< the format has no identifier and no index, reported once at stop()
    bool                                        _degenerateNoted    = false;
    /// Files a settings change evicted, held for the next processBulk() to publish on 'incomplete'. The scheduler
    /// applies a settings change while it holds the output ports reserved. The port refuses a publication made then.
    /// processBulk() holds the output span and can publish them.
    std::vector<Packet<std::uint8_t>> _pendingIncomplete{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() {
        _engine.reset();
        _pendingIncomplete.clear();
        _nextAnonDisplay         = 0ULL;
        _degenerateNoted         = false;
        nEvictedOnReconfigure    = 0ULL;
        nEvictedSuperseded       = 0ULL;
        nIncompleteDroppedNoRoom = 0ULL;
    }

    /// @brief Validates every setting and commits them only when every check passes. A refused configuration leaves
    /// the previous one in force. On a live engine the new configuration evicts every open file.
    void rebuild() {
        const gr::packet::ChunkReassembler::Config config = buildAndValidateConfig();

        if (!_engine.has_value()) {
            _engine.emplace(config);
            _idBytes            = id_bytes.value;
            _nextAnonDisplay    = 0ULL;
            _degenerateIdentity = degenerateIdentity(config);
            return;
        }

        const std::span<const std::uint64_t> departedIds = _engine->reconfigure(config);
        for (const std::uint64_t id : departedIds) {
            // The evicted files belong to the previous configuration. They are named under its identifier width,
            // before the new width is adopted.
            const gr::packet::FileFacts facts  = _engine->facts(id);
            const std::string           fileId = renderFileId(facts);
            countEviction(facts.reason);
            if (publish_incomplete.value) {
                _pendingIncomplete.push_back(buildPacket(id, facts, fileId, /*asIncomplete=*/true));
            }
            _engine->release(id);
        }
        _idBytes            = id_bytes.value;
        _nextAnonDisplay    = 0ULL;
        _degenerateIdentity = degenerateIdentity(config);
    }

    void stop() {
        for (Packet<std::uint8_t>& pending : _pendingIncomplete) {
            publishIncompleteDirect(std::move(pending));
        }
        _pendingIncomplete.clear();
        if (_engine.has_value()) {
            const std::span<const std::uint64_t> departedIds = _engine->finish();
            for (const std::uint64_t id : departedIds) {
                const gr::packet::FileFacts facts  = _engine->facts(id);
                const std::string           fileId = renderFileId(facts);
                // finish() retires every open file with EvictReason::at_stop. The engine's incomplete_at_stop
                // counter counts it, and the block counts nothing more.
                if (publish_incomplete.value) {
                    publishIncompleteDirect(buildPacket(id, facts, fileId, /*asIncomplete=*/true));
                }
                _engine->release(id);
            }
        }
        report();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& incompleteSpan, OutputSpanLike auto& rejectSpan) {
        if (!_engine.has_value()) { // without an engine the block consumes nothing and returns ERROR
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            incompleteSpan.publish(0UZ);
            rejectSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const bool outConnected        = outSpan.isConnected;
        const bool incompleteConnected = incompleteSpan.isConnected;
        const bool rejectConnected     = rejectSpan.isConnected;

        std::size_t consumed     = 0UZ;
        std::size_t onOut        = 0UZ;
        std::size_t onIncomplete = 0UZ;
        std::size_t onReject     = 0UZ;

        // Files a settings change evicted go first, at the head of this call's span. Files that do not fit wait for
        // the next call and are not dropped.
        if (!_pendingIncomplete.empty()) {
            if (incompleteConnected) {
                while (onIncomplete < _pendingIncomplete.size() && onIncomplete < incompleteSpan.size()) {
                    incompleteSpan[onIncomplete] = std::move(_pendingIncomplete[onIncomplete]);
                    ++onIncomplete;
                }
                _pendingIncomplete.erase(_pendingIncomplete.begin(), _pendingIncomplete.begin() + static_cast<std::ptrdiff_t>(onIncomplete));
            } else {
                _pendingIncomplete.clear(); // each file was counted under its reason when it left the engine
            }
        }

        // One push() completes at most one file, the file it wrote into. The same record can also retire several
        // incomplete files, for example in a stale sweep ahead of its own chunk. The engine cannot take a retired file
        // back. The loop keeps room on 'out' for everything one record can publish. It stops before it would drop a
        // completed file, and it does the same for 'reject'. When the port offers less room, the loop runs one record
        // at a time.
        const std::size_t outHeadroom = std::max(1UZ, std::min(static_cast<std::size_t>(max_open_files.value) + 1UZ, outSpan.size()));

        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            if (rejectConnected && onReject >= rejectSpan.size()) {
                break;
            }
            if (outConnected && outSpan.size() - onOut < outHeadroom) {
                break;
            }

            const DataSet<std::uint8_t>&        record = inSpan[i];
            const std::span<const std::uint8_t> bytes(record.signal_values);
            const auto [crcOk, crcStated, correctedErrors, uncorrectableErrors] = readRecordMeta(record);

            const gr::packet::ChunkReassembler::ChunkResult result = _engine->push(bytes, crcOk, crcStated, correctedErrors, uncorrectableErrors);

            for (const std::uint64_t id : _engine->departed()) {
                const gr::packet::FileFacts facts = _engine->facts(id);
                // Every file that leaves the engine is named, published or not. The anonymous sequence counts these
                // departures, and no two files leave under one name.
                const std::string fileId = renderFileId(facts);
                if (facts.reason == gr::packet::EvictReason::completed) {
                    if (outConnected) {
                        outSpan[onOut] = buildPacket(id, facts, fileId, /*asIncomplete=*/false);
                        ++onOut;
                    }
                } else {
                    countEviction(facts.reason);
                    if (publish_incomplete.value) {
                        if (incompleteConnected && onIncomplete < incompleteSpan.size()) {
                            incompleteSpan[onIncomplete] = buildPacket(id, facts, fileId, /*asIncomplete=*/true);
                            ++onIncomplete;
                        } else if (incompleteConnected) {
                            ++nIncompleteDroppedNoRoom;
                        }
                        // With the port unconnected, countEviction() or the engine's counters have counted the file.
                    }
                }
                _engine->release(id);
            }

            if (result.status == gr::packet::ChunkReassembler::Status::refused) {
                if (rejectConnected && onReject < rejectSpan.size()) {
                    rejectSpan[onReject] = record;
                    rejectSpan.publishTag(property_map{{property_map::key_type("discard_reason"), pmt::Value(std::string(detail::chunk_reassembler::refusalReasonName(result.reason)))}}, onReject);
                    ++onReject;
                }
                // the kernel's own refusal counters already count this whether or not reject is connected
            }
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(outConnected ? onOut : 0UZ);
        incompleteSpan.publish(incompleteConnected ? onIncomplete : 0UZ);
        rejectSpan.publish(rejectConnected ? onReject : 0UZ);
        if (consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    struct RecordMeta {
        bool          crcOk;
        bool          crcStated;
        std::uint64_t correctedErrors;
        std::uint64_t uncorrectableErrors;
    };

    /// @brief Reads crc_ok, corrected_errors and uncorrectable_errors from the record. These three keys have a
    /// defined rule for combining the records of one file. A value of the wrong type reads as absent and raises no
    /// error. A record with no metadata map reads the same way. The records may come off a network, and a received
    /// record must not stop a graph.
    [[nodiscard]] static RecordMeta readRecordMeta(const DataSet<std::uint8_t>& record) noexcept {
        RecordMeta meta{.crcOk = true, .crcStated = false, .correctedErrors = 0ULL, .uncorrectableErrors = 0ULL};
        if (record.meta_information.empty()) {
            return meta;
        }
        const property_map& map = record.meta_information[0UZ];
        if (const auto it = map.find(property_map::key_type("crc_ok")); it != map.end()) {
            if (const bool* value = it->second.get_if<bool>(); value != nullptr) {
                meta.crcOk     = *value;
                meta.crcStated = true;
            }
        }
        if (const auto it = map.find(property_map::key_type("corrected_errors")); it != map.end()) {
            if (const gr::Size_t* value = it->second.get_if<gr::Size_t>(); value != nullptr) {
                meta.correctedErrors = static_cast<std::uint64_t>(*value);
            }
        }
        if (const auto it = map.find(property_map::key_type("uncorrectable_errors")); it != map.end()) {
            if (const gr::Size_t* value = it->second.get_if<gr::Size_t>(); value != nullptr) {
                meta.uncorrectableErrors = static_cast<std::uint64_t>(*value);
            }
        }
        return meta;
    }

    void countEviction(gr::packet::EvictReason reason) {
        switch (reason) {
        case gr::packet::EvictReason::reconfigure: ++nEvictedOnReconfigure; break;
        case gr::packet::EvictReason::superseded: ++nEvictedSuperseded; break;
        default: break; // the engine counts cap, stale and at_stop itself, and no_position as a refusal
        }
    }

    /// @brief Names a file by its identifier field in lowercase hexadecimal, most significant byte first. The name
    /// has exactly `2 * id_bytes` characters from `[0-9a-f]`. It cannot hold a path separator or a leading dot. An
    /// anonymous file is named "anon-N" from this block's own sequence. The engine's internal counter is base-offset
    /// and private. The sequence advances once for each anonymous file that leaves the engine, published or not. It
    /// numbers the files in the order they left.
    [[nodiscard]] std::string renderFileId(const gr::packet::FileFacts& facts) {
        if (facts.anonymous) {
            return std::format("anon-{}", _nextAnonDisplay++);
        }
        return detail::chunk_reassembler::toLowerHex(facts.id, _idBytes);
    }

    /// @brief Builds the emitted packet from the assembled bytes and the keys every packet carries. A packet for
    /// `incomplete` also carries `discard_reason`, `file_covered_bytes` and `file_declared_size`. The metadata of the
    /// contributing records is not merged. A file is built from many records, and no rule picks the record whose
    /// `frequency` or `sample_start` describes the file. The packet carries the three keys with a defined rule.
    [[nodiscard]] Packet<std::uint8_t> buildPacket(std::uint64_t id, const gr::packet::FileFacts& facts, const std::string& fileId, bool asIncomplete) {
        Packet<std::uint8_t>                packet;
        const std::span<const std::uint8_t> bytes = _engine->file(id);
        packet.signal_values.assign(bytes.begin(), bytes.end());
        packet.meta_information.resize(1UZ);
        property_map& map = packet.meta_information[0UZ];

        map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("file/chunked")));
        map.insert_or_assign(property_map::key_type("corrected_errors"), pmt::Value(detail::chunk_reassembler::saturateToSize(facts.corrected_errors)));
        map.insert_or_assign(property_map::key_type("uncorrectable_errors"), pmt::Value(detail::chunk_reassembler::saturateToSize(facts.uncorrectable_errors)));
        map.insert_or_assign(property_map::key_type("file_id"), pmt::Value(fileId));
        map.insert_or_assign(property_map::key_type("file_size"), pmt::Value(static_cast<std::uint64_t>(facts.size)));
        map.insert_or_assign(property_map::key_type("file_chunks"), pmt::Value(detail::chunk_reassembler::saturateToSize(facts.chunks)));
        if (facts.crc_stated) {
            map.insert_or_assign(property_map::key_type("file_all_chunks_crc_ok"), pmt::Value(facts.all_chunks_crc_ok));
        }
        if (facts.conflicts != 0ULL) {
            map.insert_or_assign(property_map::key_type("file_conflicts"), pmt::Value(detail::chunk_reassembler::saturateToSize(facts.conflicts)));
        }
        if (asIncomplete) {
            map.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(std::string(detail::chunk_reassembler::evictReasonName(facts.reason))));
            map.insert_or_assign(property_map::key_type("file_covered_bytes"), pmt::Value(facts.covered_bytes));
            if (facts.declared_size.has_value()) {
                map.insert_or_assign(property_map::key_type("file_declared_size"), pmt::Value(*facts.declared_size));
            }
        }
        return packet;
    }

    /// @brief Publishes one incomplete file from stop(). No output span is open there. The block reserves the slot
    /// with `tryReserve` and calls `publish` itself, as a source block does from its own thread. With the port
    /// unconnected the file is dropped. It was counted and named when it left the engine. A port without a free slot
    /// drops the file, and this function counts it.
    void publishIncompleteDirect(Packet<std::uint8_t> packet) {
        if (!incomplete.isConnected()) {
            return;
        }
        auto span = incomplete.template tryReserve<gr::SpanReleasePolicy::ProcessNone>(1UZ);
        if (span.empty()) {
            ++nIncompleteDroppedNoRoom;
            return;
        }
        span[0UZ] = std::move(packet);
        span.publish(1UZ);
    }

    [[nodiscard]] static bool degenerateIdentity(const gr::packet::ChunkReassembler::Config& config) {
        // A format with neither an identifier nor an index is 'offset' with id_bytes = 0. 'indexed' carries an index
        // in every chunk. The offset format leaves a descriptor's index unset. Without either field a new file cannot
        // be told from a continuation. The engine keeps one anonymous file until it completes or is evicted. The
        // index-regression rule has no effect.
        if (const gr::packet::OffsetChunkFormat* offsetFormat = std::get_if<gr::packet::OffsetChunkFormat>(&config.format); offsetFormat != nullptr) {
            return !offsetFormat->identifier.present();
        }
        return false;
    }

    void report() {
        std::string line;
        const auto  append = [&line](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(line), "{}{}: {}", line.empty() ? "" : ", ", label, count);
            }
        };
        if (_engine.has_value()) {
            const gr::packet::ChunkReassembler::Counters& counters = _engine->counters();
            append("records", counters.records);
            append("chunks written", counters.chunks_written);
            append("payload bytes", counters.payload_bytes);
            append("files opened", counters.files_opened);
            append("files completed", counters.files_completed);
            append("duplicate chunks", counters.duplicate_chunks);
            append("conflicting chunks", counters.conflicting_chunks);
            append("declaration conflicts", counters.declaration_conflicts);
            append("evicted for cap", counters.evicted_for_cap);
            append("evicted stale", counters.evicted_stale);
            append("evicted on reconfigure", nEvictedOnReconfigure);
            append("evicted superseded", nEvictedSuperseded);
            append("incomplete at stop", counters.incomplete_at_stop);
            append("chunks crc failed", counters.chunks_crc_failed);
            append("refused unparsable", counters.refused_unparsable);
            append("refused bad payload span", counters.refused_payload_span);
            append("refused zero size", counters.refused_zero_size);
            append("refused beyond declared size", counters.refused_beyond_size);
            append("refused over max bytes", counters.refused_over_max);
            append("refused too fragmented", counters.refused_too_fragmented);
            append("refused no position", counters.refused_no_position);
            append("refused crc failed", counters.refused_crc_failed);
            append("incomplete files dropped, no room", nIncompleteDroppedNoRoom);
        }
        if (!line.empty()) {
            std::println(stderr, "gr::blocks::basic::ChunkReassembler '{}': {}", this->name, line);
        }
        if (_degenerateIdentity && !_degenerateNoted) {
            std::println(stderr,
                "gr::blocks::basic::ChunkReassembler '{}': the configured format carries neither an identifier nor an index; "
                "new_file_on_index_regression has no effect and one anonymous file is kept until it completes or is evicted",
                this->name);
            _degenerateNoted = true;
        }
    }

    /// @brief Every field setting, named, for a validation message that cannot pin down a single offending one.
    [[nodiscard]] std::string describeFields() const {
        return std::format("id_offset={} id_bytes={} index_offset={} index_bytes={} offset_offset={} offset_bytes={} count_offset={} count_bytes={} size_offset={} size_bytes={} "
                           "last_flag_offset={} last_flag_bit={} payload_offset={} payload_trim={} chunk_size={}",
            id_offset.value, id_bytes.value, index_offset.value, index_bytes.value, offset_offset.value, offset_bytes.value, count_offset.value, count_bytes.value, size_offset.value, size_bytes.value, last_flag_offset.value, last_flag_bit.value, payload_offset.value, payload_trim.value, chunk_size.value);
    }

    /// @brief Validates every setting into a local `Config` and returns it. The first failing check throws and names
    /// the setting and its value. No member changes before this returns. A refused configuration leaves the previous
    /// one in force. A block without an accepted configuration has no engine and stays inert.
    [[nodiscard]] gr::packet::ChunkReassembler::Config buildAndValidateConfig() const {
        if (chunk_format.value != "indexed" && chunk_format.value != "offset") {
            throw gr::exception(std::format("chunk_format is '{}'; it is required and must be 'indexed' or 'offset'", chunk_format.value));
        }
        const gr::packet::ByteOrder order = detail::chunk_reassembler::byteOrderFromName(field_byte_order.value);

        const auto checkWidth = [](std::string_view settingName, gr::Size_t width) {
            if (width > 8U) {
                throw gr::exception(std::format("{} is {}; a field wider than 8 bytes does not fit a 64-bit value", settingName, width));
            }
        };
        checkWidth("id_bytes", id_bytes.value);
        checkWidth("index_bytes", index_bytes.value);
        checkWidth("offset_bytes", offset_bytes.value);
        checkWidth("count_bytes", count_bytes.value);
        checkWidth("size_bytes", size_bytes.value);

        const bool indexed = chunk_format.value == "indexed";
        if (indexed && offset_bytes.value != 0U) {
            throw gr::exception(std::format("offset_bytes is {} but chunk_format is 'indexed': the two formats read different headers, and a cross-format setting is refused rather than ignored", offset_bytes.value));
        }
        if (!indexed && index_bytes.value != 0U) {
            throw gr::exception(std::format("index_bytes is {} but chunk_format is 'offset': the two formats read different headers, and a cross-format setting is refused rather than ignored", index_bytes.value));
        }
        if (indexed && index_bytes.value == 0U) {
            throw gr::exception("index_bytes is 0 but chunk_format is 'indexed': the index is that format's defining field and is required");
        }
        if (indexed && chunk_size.value == 0U) {
            throw gr::exception("chunk_size is 0 but chunk_format is 'indexed': an index means nothing without a chunk size");
        }
        if (!indexed && offset_bytes.value == 0U) {
            throw gr::exception("offset_bytes is 0 but chunk_format is 'offset': the offset is that format's defining field and is required");
        }

        if (max_open_files.value == 0U) {
            throw gr::exception("max_open_files is 0; it is required and has no default: without it the sum of every file in flight is unbounded");
        }
        if (max_file_bytes.value == 0ULL) {
            throw gr::exception("max_file_bytes is 0; it is required and has no default: without it a single file's own storage is unbounded");
        }
        if (max_file_bytes.value > gr::packet::kMaxFileBytes) {
            throw gr::exception(std::format("max_file_bytes is {}; it must not exceed {} so that every packet this block emits fits a DataSet<T>::extents entry", max_file_bytes.value, gr::packet::kMaxFileBytes));
        }
        if (max_gaps.value == 0U) {
            throw gr::exception("max_gaps is 0; the interval path needs room for at least one gap");
        }
        if (last_flag_bit.value > 8U) {
            throw gr::exception(std::format("last_flag_bit is {}; a bit in [0,7] enables the last-chunk flag and 8 disables it, so a larger value would silently turn the flag off", last_flag_bit.value));
        }

        const gr::packet::FieldSpec identifier{static_cast<std::size_t>(id_offset.value), static_cast<std::size_t>(id_bytes.value)};
        const gr::packet::FieldSpec count{static_cast<std::size_t>(count_offset.value), static_cast<std::size_t>(count_bytes.value)};
        const gr::packet::FieldSpec size{static_cast<std::size_t>(size_offset.value), static_cast<std::size_t>(size_bytes.value)};
        const gr::packet::FlagSpec  lastFlag{static_cast<std::size_t>(last_flag_offset.value), static_cast<std::uint8_t>(last_flag_bit.value)};

        // No setting names a whole-file check field. Neither format reads one, and no emitted packet carries a
        // declared check value. A protocol that sends one gives it to the engine through its own descriptor hook.
        gr::packet::ChunkFormat format;
        if (indexed) {
            format = gr::packet::IndexedChunkFormat{.identifier = identifier, .index = gr::packet::FieldSpec{static_cast<std::size_t>(index_offset.value), static_cast<std::size_t>(index_bytes.value)}, .chunk_count = count, .total_size = size, .check_value = {}, .last_flag = lastFlag, .payload_offset = static_cast<std::size_t>(payload_offset.value), .payload_trim = static_cast<std::size_t>(payload_trim.value), .chunk_size = static_cast<std::size_t>(chunk_size.value), .byte_order = order};
        } else {
            format = gr::packet::OffsetChunkFormat{.identifier = identifier, .offset = gr::packet::FieldSpec{static_cast<std::size_t>(offset_offset.value), static_cast<std::size_t>(offset_bytes.value)}, .chunk_count = count, .total_size = size, .check_value = {}, .last_flag = lastFlag, .payload_offset = static_cast<std::size_t>(payload_offset.value), .payload_trim = static_cast<std::size_t>(payload_trim.value), .chunk_size = static_cast<std::size_t>(chunk_size.value), .byte_order = order};
        }

        if (const gr::packet::FormatError error = gr::packet::validateChunkFormat(format); error != gr::packet::FormatError::ok) {
            throw gr::exception(std::format("chunk_format '{}' is refused: {} ({})", chunk_format.value, gr::packet::formatErrorName(error), describeFields()));
        }

        return gr::packet::ChunkReassembler::Config{.format = format, .max_open_files = static_cast<std::size_t>(max_open_files.value), .max_file_bytes = max_file_bytes.value, .max_gaps = static_cast<std::size_t>(max_gaps.value), .evict_after_records = evict_after_records.value, .require_crc_ok = require_crc_ok.value, .new_file_on_index_regression = new_file_on_index_regression.value};
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_CHUNK_REASSEMBLER_HPP
