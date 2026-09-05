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
 * KISS, the one command byte a host and a terminal node controller put in front of a frame.
 *
 * A KISS frame is that byte followed by the payload, delimited on the wire by SLIP framing, which is
 * `gr::digital::slip()` in both directions and belongs to `DelimiterFramer` and `DelimiterExtractor`.
 * What is left for these two blocks is the byte itself: its high nibble the terminal node
 * controller's port, 0 to 15, and its low nibble the command, where zero means data and the values
 * above it configure the controller.
 *
 * These blocks carry data frames only. The parameter commands set transmit delay, persistence, slot
 * time and the rest of a controller's radio timing, which a receive or transmit chain here has no
 * consumer for; a parameter frame arriving on the wire is therefore counted and passed over rather
 * than decoded, and no setting sends one.
 *
 * The one exception is the gr-satellites timestamp record: a frame whose command byte is 0x09, carrying eight
 * bytes, big-endian unsigned milliseconds since the Unix epoch, ahead of the data frame it stamps. `0x09` is not a
 * command Chepponis and Karn defined; it is this extension's own, opt in on both blocks through `emit_timestamp` and
 * `read_timestamp` so a chain that sets neither is unchanged.
 */
namespace gr::blocks::ax25 {

GR_REGISTER_BLOCK(gr::blocks::ax25::KissDecode)

/*!
@brief One de-SLIPped KISS frame per record in, its payload out with the port it arrived on in metadata.

The first byte is the command byte. A command nibble of zero is a data frame: the byte is stripped, the rest of the
record is published, and `kiss_port` carries the high nibble so a multi-port controller's frames can be told apart
downstream. The record's own metadata crosses verbatim underneath that key.

Anything else is a parameter frame, counted in `nControlFrames` and dropped. The count is per frame rather than per
byte, which is what makes it readable: a controller sending its timing parameters once at startup shows as a handful
rather than as a byte total nobody can interpret. An empty record carries no command byte at all and is counted
separately. Either way the next record decodes.

With `read_timestamp` set, a command nibble of 9 is read instead of counted: a nine-byte frame sets a pending stamp,
counted `nTimestampsRead`, and the **next** data frame's output record carries `timestamp = milliseconds * 1'000'000`.
A second timestamp frame arriving before any data frame consumed the first supersedes it, and a pending stamp still
held at `stop()` or at a change to `read_timestamp` is discarded; both are counted `nTimestampsUnused`. A change to
any other setting leaves a pending stamp where it is, because the frame it belongs to is still the next one.

A command-9 frame is malformed, counted `nTimestampsMalformed` and never held as pending, when it is not exactly nine
bytes or when its eight bytes hold more milliseconds than a nanosecond count can carry: the wire field is unsigned
and 64 bits wide where `DataSet::timestamp` is signed nanoseconds, so everything above `2^63 / 10^6` milliseconds —
some 292 million years past the epoch — names a time this carrier cannot express, and refusing it is what keeps a
hostile or corrupt frame from wrapping the multiplication. With `read_timestamp` false a command-9 frame is a
parameter frame like any other, counted in `nControlFrames`.
*/
struct KissDecode : Block<KissDecode> {
    using Description = Doc<"KISS decode: strips the command byte from a de-SLIPped record, publishing the payload with 'kiss_port' and counting the parameter frames it passes over; with read_timestamp, a command-9 frame stamps the next data record instead">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<bool, "read_timestamp", Doc<"interpret a command-9 control frame as a timestamp for the next data frame">, Visible> read_timestamp = false;

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
    std::uint64_t nTimestampsUnused    = 0ULL; ///< a pending stamp superseded, discarded at stop(), or dropped by a change to read_timestamp
    std::uint64_t nTimestampsMalformed = 0ULL; ///< a command-9 frame that was not nine bytes, or whose milliseconds exceed what nanoseconds hold

    std::optional<std::int64_t> _pendingTimestamp{}; ///< nanoseconds, ready for the next data frame's DataSet::timestamp
    bool                        _stamping = false;   ///< the read_timestamp in force, so that only a change to it discards a pending stamp

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    /// @brief Discards a pending stamp when `read_timestamp` itself changes, counted the same way a superseded one is.
    /// Any other setting leaves it alone: the data frame it belongs to is still the next one to arrive.
    void rebuild() {
        if (read_timestamp.value == _stamping) {
            return;
        }
        _stamping = read_timestamp.value;
        if (_pendingTimestamp.has_value()) {
            ++nTimestampsUnused;
            _pendingTimestamp.reset();
        }
    }

    void stop() {
        if (_pendingTimestamp.has_value()) { // held at stop(): no data frame arrived to carry it
            ++nTimestampsUnused;
            _pendingTimestamp.reset();
        }
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
                map = record.meta_information[0UZ]; // the record's facts carry through, the port key over them
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

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Reads a command-9 frame into the pending stamp, or counts it malformed. Never throws.
    void readTimestampFrame(const DataSet<std::uint8_t>& record) {
        if (record.signal_values.size() != kTimestampBytes) {
            ++nTimestampsMalformed;
            return;
        }
        std::uint64_t milliseconds = 0ULL;
        for (std::size_t i = 1UZ; i < kTimestampBytes; ++i) {
            milliseconds = (milliseconds << 8U) | static_cast<std::uint64_t>(record.signal_values[i]);
        }
        if (milliseconds > kMaxMilliseconds) { // a time no signed nanosecond count reaches; the multiplication below would wrap
            ++nTimestampsMalformed;
            return;
        }
        if (_pendingTimestamp.has_value()) { // superseded before any data frame consumed it
            ++nTimestampsUnused;
        }
        _pendingTimestamp = static_cast<std::int64_t>(milliseconds) * 1'000'000LL;
        ++nTimestampsRead;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ax25::KissEncode)

/*!
@brief Puts a KISS data command byte in front of each record, for `DelimiterFramer` under SLIP framing.

`kiss_port` is the terminal node controller port the frame is for and goes into the byte's high nibble; the low
nibble is zero, the data command, because a chain that carries frames has no parameter frame to send. A record may
name its own port under a `kiss_port` metadata key, which is what lets one chain feed several ports, and a port
outside 0 to 15 there is a counted drop rather than a truncated nibble sending the frame somewhere else.

Metadata crosses verbatim; prefixing a byte has nothing to report about the frame.

With `emit_timestamp` set, a record whose `DataSet::timestamp` is non-zero gets its own command-9 frame ahead of it:
the command byte followed by eight bytes, big-endian unsigned, of `timestamp / 1'000'000` — the record's own field,
truncated to the millisecond it divides evenly into, never the host clock. A record whose `timestamp` is zero, the
field's unstated value, gets no stamp frame and is counted `nTimestampsUnavailable`; nothing is invented for it.
*/
struct KissEncode : Block<KissEncode> {
    using Description = Doc<"KISS encode: prepends the data command byte, its high nibble the 'kiss_port' setting or the record's own override; with emit_timestamp, a command-9 frame precedes any record whose DataSet::timestamp is non-zero">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "kiss_port", Doc<"the terminal node controller port a frame is for, 0 to 15; a record's own 'kiss_port' key overrides it">, Visible>    kiss_port      = 0U;
    Annotated<bool, "emit_timestamp", Doc<"publish a command-9 timestamp frame ahead of each data frame whose record's DataSet::timestamp is non-zero">, Visible> emit_timestamp = false;

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

            // a stamped record needs two slots and an unstamped one needs one, so the room check is per record rather
            // than a loop condition: publishing the stamp frame without the frame it stamps would strand it
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
                framed.meta_information[0UZ] = *meta; // the record's facts carry through; a prefixed byte adds none
            }

            ++nRecords;
            outSpan[made] = std::move(framed);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            // a record held for want of slots is short of output, whatever the input port has waiting: one free slot
            // is not room for a stamped record, and calling that a shortage of input names the wrong port
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

    /// @brief The command-9 frame for @p timestampNs: the command byte, then its milliseconds, big-endian.
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
