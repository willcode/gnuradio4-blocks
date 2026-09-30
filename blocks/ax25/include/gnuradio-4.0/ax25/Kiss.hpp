#ifndef GNURADIO_AX25_KISS_HPP
#define GNURADIO_AX25_KISS_HPP

#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <optional>
#include <print>
#include <string>
#include <string_view>
#include <utility>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

/**
 * KISS puts one command byte in front of each frame passed between a host and a terminal node
 * controller.
 *
 * A KISS frame is that byte followed by the payload. SLIP framing delimits it on the wire.
 * `DelimiterFramer` and `DelimiterExtractor` apply `gr::digital::slip()` in both directions. These
 * two blocks handle the command byte alone. Its high nibble is the terminal node controller's port,
 * 0 to 15. Its low nibble is the command. Zero means data, and the values above zero configure the
 * controller.
 *
 * These blocks carry data frames only. The parameter commands set transmit delay, persistence, slot
 * time and the rest of a controller's radio timing. These blocks do not use them. `KissDecode`
 * counts and skips a parameter frame. No setting sends one.
 *
 * The one exception is the gr-satellites timestamp record. It is a frame with command byte 0x09 and eight bytes of
 * big-endian unsigned milliseconds since the Unix epoch. It precedes the data frame it stamps. Chepponis and Karn did
 * not define `0x09`. The extension defines it. Each block enables it through a setting, `emit_timestamp` or
 * `read_timestamp`. With neither set, the blocks handle plain KISS.
 */
namespace gr::blocks::ax25 {

GR_REGISTER_BLOCK(gr::blocks::ax25::KissDecode)

/*!
@brief Strips the KISS command byte from each de-SLIPped frame and publishes the payload with its port in metadata.

The first byte is the command byte. A command nibble of zero marks a data frame. The block strips the byte and
publishes the rest of the record. `kiss_port` carries the high nibble and tells the ports of a multi-port controller
apart. The record's own metadata passes through unchanged beneath that key.

Any other command marks a parameter frame. The block counts it in `nControlFrames` and drops it. The count is per
frame, not per byte. A controller that sends its timing parameters once at startup shows as a few frames. An empty
record carries no command byte and is counted in `nRefusedEmpty`. In both cases the next record decodes.

With `read_timestamp` set, the block reads a command nibble of 9 as a timestamp. A nine-byte frame sets a pending
stamp and counts in `nTimestampsRead`. The output record of the **next** data frame carries
`timestamp = milliseconds * 1'000'000`. A second timestamp frame before the next data frame replaces the first. A
change to `read_timestamp` discards a pending stamp. `nTimestampsUnused` counts both cases. It also counts a stamp
still held when the stream ends, since no data frame will carry it. A change to any other setting keeps a pending
stamp, because its data frame is still the next one.

A command-9 frame is malformed when it is not exactly nine bytes long. It is also malformed when its milliseconds
exceed `2^63 / 10^6`, about 292 million years past the epoch. The wire field is unsigned and 64 bits wide.
`DataSet::timestamp` is signed nanoseconds and cannot express a later time. The block counts a malformed frame in
`nTimestampsMalformed` and never holds it as pending. The check keeps a corrupt frame from overflowing the
multiplication. With `read_timestamp` false, a command-9 frame is a parameter frame and counts in `nControlFrames`.
*/
struct KissDecode : Block<KissDecode> {
    using Description = Doc<"Strips the KISS command byte from a de-SLIPped record and publishes the payload with 'kiss_port'. It counts the parameter frames it skips. With read_timestamp set, a command-9 frame stamps the next data record">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<bool, "read_timestamp", Doc<"read a command-9 frame as the next data frame's timestamp">, Visible> read_timestamp = false;

    GR_MAKE_REFLECTABLE(KissDecode, in, out, read_timestamp);

    static constexpr unsigned      kTimestampNibble = 9U;
    static constexpr std::size_t   kTimestampBytes  = 9UZ; // the command byte plus eight bytes of milliseconds
    static constexpr std::uint64_t kMaxMilliseconds = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1'000'000LL);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords             = 0ULL; ///< data frames published on `out`
    std::uint64_t nPayloadBytes        = 0ULL; ///< payload bytes those frames carry
    std::uint64_t nRefusedEmpty        = 0ULL; ///< records with no command byte to read
    std::uint64_t nControlFrames       = 0ULL; ///< parameter frames, counted once each and not published
    std::uint64_t nTimestampsRead      = 0ULL; ///< well-formed command-9 frames read into the pending stamp
    std::uint64_t nTimestampsUnused    = 0ULL; ///< a pending stamp superseded or dropped by a change to read_timestamp, including one still held
    std::uint64_t nTimestampsMalformed = 0ULL; ///< a command-9 frame that was not nine bytes, or whose milliseconds exceed what nanoseconds hold

    std::optional<std::int64_t> _pendingTimestamp{}; ///< nanoseconds, ready for the next data frame's DataSet::timestamp
    bool                        _stamping = false;   ///< the read_timestamp in force. Only a change to it discards a pending stamp.
    /// @brief The unused stamps the worker thread has retired. `nTimestampsUnused` adds the one still held.
    std::uint64_t _timestampsUnusedBanked = 0ULL;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    /// @brief Discards a pending stamp when `read_timestamp` changes and counts it as a superseded one.
    /// Any other setting keeps the stamp. Its data frame is still the next one to arrive.
    void rebuild() {
        if (read_timestamp.value == _stamping) {
            return;
        }
        _stamping = read_timestamp.value;
        if (_pendingTimestamp.has_value()) {
            ++_timestampsUnusedBanked;
            _pendingTimestamp.reset();
        }
        countUnusedTimestamps();
    }

    /// @brief Reports the counters and changes no state. It does not run on the worker thread.
    void stop() {
        // SchedulerBase::stop() calls changeStateTo(REQUESTED_STOP) on the thread that requested the stop. That call
        // reaches here while this block's worker may still be inside processBulk. The pending stamp belongs to the
        // worker. Every call therefore leaves the counters complete, and this function reports them.
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records", nRecords);
        append("payload bytes", nPayloadBytes);
        append("empty records refused", nRefusedEmpty);
        append("control frames", nControlFrames);
        append("timestamps read", nTimestampsRead);
        append("timestamps unused", nTimestampsUnused);
        append("timestamps malformed", nTimestampsMalformed);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::ax25::KissDecode '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            if (record.signal_values.empty()) {
                ++nRefusedEmpty;
                continue;
            }
            const unsigned command = record.signal_values[0UZ];
            const unsigned nibble  = command & 0x0FU;
            if (nibble != 0U) {
                if (read_timestamp.value && nibble == kTimestampNibble) {
                    readTimestampFrame(record);
                } else {
                    ++nControlFrames;
                }
                continue;
            }

            DataSet<std::uint8_t> payload;
            payload.signal_values.assign(record.signal_values.begin() + 1, record.signal_values.end());
            payload.extents.push_back(static_cast<std::int32_t>(payload.signal_values.size()));
            payload.signal_names.emplace_back(record.signal_names.empty() ? std::string("kiss") : record.signal_names[0UZ]);
            payload.timing_events.resize(1UZ);
            payload.meta_information.resize(1UZ);
            property_map& map = payload.meta_information[0UZ];
            if (!record.meta_information.empty()) {
                map = record.meta_information[0UZ]; // the record's metadata passes through, the port key written over it
            }
            map.insert_or_assign(property_map::key_type("kiss_port"), pmt::Value(gr::Size_t{command >> 4U}));
            if (read_timestamp.value && _pendingTimestamp.has_value()) {
                payload.timestamp = *_pendingTimestamp;
                _pendingTimestamp.reset();
            }

            ++nRecords;
            nPayloadBytes += payload.signal_values.size();
            outSpan[made] = std::move(payload);
            ++made;
        }

        countUnusedTimestamps(); // a stamp still held has no data frame yet, and only the worker thread reads it
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Sets `nTimestampsUnused` to the retired stamps plus the stamp still waiting for a data frame.
    void countUnusedTimestamps() noexcept { nTimestampsUnused = _timestampsUnusedBanked + (_pendingTimestamp.has_value() ? 1ULL : 0ULL); }

    /// @brief Reads a command-9 frame into the pending stamp or counts it malformed. Does not throw.
    void readTimestampFrame(const DataSet<std::uint8_t>& record) {
        if (record.signal_values.size() != kTimestampBytes) {
            ++nTimestampsMalformed;
            return;
        }
        std::uint64_t milliseconds = 0ULL;
        for (std::size_t i = 1UZ; i < kTimestampBytes; ++i) {
            milliseconds = (milliseconds << 8U) | static_cast<std::uint64_t>(record.signal_values[i]);
        }
        if (milliseconds > kMaxMilliseconds) { // beyond signed nanoseconds, and the multiplication below would wrap
            ++nTimestampsMalformed;
            return;
        }
        if (_pendingTimestamp.has_value()) { // superseded before any data frame consumed it
            ++_timestampsUnusedBanked;
        }
        _pendingTimestamp = static_cast<std::int64_t>(milliseconds) * 1'000'000LL;
        ++nTimestampsRead;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ax25::KissEncode)

/*!
@brief Puts a KISS data command byte in front of each record.

The output still needs SLIP framing, as `DelimiterFramer` applies it. `kiss_port` names the terminal node controller
port of the frame. It goes into the byte's high nibble. The low nibble is zero, the data command. The block sends no
parameter frames. A record may name its own port under a `kiss_port` metadata key. One chain can then feed several
ports. The block drops a record whose port is outside 0 to 15 and counts it in `nRefusedOverride`. A truncated
nibble would send the frame to the wrong port.

Metadata passes through unchanged. The block adds no keys.

With `emit_timestamp` set, a record with a non-zero `DataSet::timestamp` gets a command-9 frame ahead of it. That
frame holds the command byte and eight bytes of `timestamp / 1'000'000`, big-endian unsigned. The value comes from
the record's own field, truncated toward zero to whole milliseconds. The block does not read the host clock. A zero
`timestamp` means the field is unset. Such a record gets no stamp frame and is counted in `nTimestampsUnavailable`.
*/
struct KissEncode : Block<KissEncode> {
    using Description = Doc<"Prepends the KISS data command byte, its high nibble the 'kiss_port' setting or the record's own override. With emit_timestamp set, a command-9 frame precedes each record whose DataSet::timestamp is non-zero">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "kiss_port", Doc<"terminal node controller port, 0 to 15, overridable by a 'kiss_port' key">, Visible> kiss_port      = 0U;
    Annotated<bool, "emit_timestamp", Doc<"precede each record with a non-zero timestamp by a command-9 frame">, Visible>        emit_timestamp = false;

    GR_MAKE_REFLECTABLE(KissEncode, in, out, kiss_port, emit_timestamp);

    static constexpr gr::Size_t  kMaxPort        = 15U;
    static constexpr std::size_t kTimestampBytes = 9UZ; // the command byte plus eight bytes of milliseconds

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords               = 0ULL; ///< frames published on `out`
    std::uint64_t nRefusedOverride       = 0ULL; ///< records whose `kiss_port` key named no port
    std::uint64_t nTimestampsUnavailable = 0ULL; ///< records with emit_timestamp set whose timestamp was 0

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (kiss_port.value > kMaxPort) {
            throw gr::exception(std::format("kiss_port is the command byte's high nibble and must be in [0, 15], got {}", kiss_port.value));
        }
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records", nRecords);
        append("overrides refused", nRefusedOverride);
        append("timestamps unavailable", nTimestampsUnavailable);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::ax25::KissEncode '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        bool        roomHold = false; ///< a record was left in the buffer for want of output slots, not for want of input
        for (; consumed < inSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];

            // A stamped record needs two slots and an unstamped one needs one. The room check is therefore per record,
            // not a loop condition. A stamp frame published without the frame it stamps would be stranded.
            const bool        stamping = emit_timestamp.value && record.timestamp != 0;
            const std::size_t needed   = stamping ? 2UZ : 1UZ;
            if (made + needed > outSpan.size()) {
                roomHold = true;
                break;
            }

            const property_map* meta = record.meta_information.empty() ? nullptr : &record.meta_information[0UZ];
            gr::Size_t          port = kiss_port.value;
            if (const std::optional<gr::Size_t> named = number(meta, "kiss_port"); named.has_value()) {
                if (*named > kMaxPort) {
                    std::println(stderr, "gr::blocks::ax25::KissEncode '{}': dropping a record whose kiss_port override is {}, which is no port", this->name, *named);
                    ++nRefusedOverride;
                    continue;
                }
                port = *named;
            }

            if (emit_timestamp.value && record.timestamp == 0) {
                ++nTimestampsUnavailable;
            }
            if (stamping) {
                outSpan[made] = timestampFrame(record.timestamp);
                ++made;
            }

            DataSet<std::uint8_t> framed;
            framed.signal_values.reserve(record.signal_values.size() + 1UZ);
            framed.signal_values.push_back(static_cast<std::uint8_t>(port << 4U));
            framed.signal_values.insert(framed.signal_values.end(), record.signal_values.begin(), record.signal_values.end());
            framed.extents.push_back(static_cast<std::int32_t>(framed.signal_values.size()));
            framed.signal_names.emplace_back(record.signal_names.empty() ? std::string("kiss") : record.signal_names[0UZ]);
            framed.timing_events.resize(1UZ);
            framed.meta_information.resize(1UZ);
            if (meta != nullptr) {
                framed.meta_information[0UZ] = *meta; // the record's metadata passes through, and the block adds no keys
            }

            ++nRecords;
            outSpan[made] = std::move(framed);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            // A record held for lack of slots is short of output, whatever the input port holds. One free slot is not
            // room for a stamped record. Reporting a shortage of input would name the wrong port.
            return (roomHold || outSpan.size() == 0UZ) ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief The port a record names. A key holding something that is not a number reads back out of range.
    [[nodiscard]] static std::optional<gr::Size_t> number(const property_map* map, const char* key) {
        if (map == nullptr) {
            return std::nullopt;
        }
        const auto entry = map->find(property_map::key_type(key));
        return entry == map->end() ? std::nullopt : std::optional<gr::Size_t>(entry->second.value_or(gr::Size_t{0x100U}));
    }

    /// @brief The command-9 frame for @p timestampNs, the command byte followed by its milliseconds, big-endian.
    [[nodiscard]] static DataSet<std::uint8_t> timestampFrame(std::int64_t timestampNs) {
        const std::uint64_t milliseconds = static_cast<std::uint64_t>(timestampNs / 1'000'000LL); // truncating toward zero

        DataSet<std::uint8_t> stamp;
        stamp.signal_values.reserve(kTimestampBytes);
        stamp.signal_values.push_back(0x09U);
        for (int shift = 56; shift >= 0; shift -= 8) {
            stamp.signal_values.push_back(static_cast<std::uint8_t>((milliseconds >> static_cast<unsigned>(shift)) & 0xFFU));
        }
        stamp.extents.push_back(static_cast<std::int32_t>(stamp.signal_values.size()));
        stamp.signal_names.emplace_back("kiss");
        stamp.timing_events.resize(1UZ);
        stamp.meta_information.resize(1UZ);
        return stamp;
    }
};

} // namespace gr::blocks::ax25

#endif // GNURADIO_AX25_KISS_HPP
