#ifndef GNURADIO_BASIC_RECORD_UTILS_HPP
#define GNURADIO_BASIC_RECORD_UTILS_HPP

#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <print>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

/**
 * Two record utilities with no arithmetic in them: a trim that removes a stated head and tail from
 * every record, and a length filter that routes records by their item count. Both copy every fact a
 * record carries and transform none of them; what they own is the item span and the routing, and a
 * refusal is a counted, stated drop on a `reject` port rather than a silent absence.
 *
 * `sample_start` is deliberately not rewritten by the trim: the key counts samples of the stream the
 * record was cut from, the trim moves items of the record, and the two units agree only when one
 * item is one sample — a rewrite would be right exactly there and silently wrong everywhere else.
 */
namespace gr::blocks::basic {

namespace recorddetail {

//! Copy every fact of @p record onto @p out, whose values the caller has already set.
inline void carryFacts(DataSet<std::uint8_t>& out, const DataSet<std::uint8_t>& record) {
    out.extents.push_back(static_cast<std::int32_t>(out.signal_values.size()));
    out.signal_names.emplace_back(record.signal_names.empty() ? std::string("basic") : record.signal_names[0UZ]);
    out.timing_events.resize(1UZ);
    out.meta_information.resize(1UZ);
    if (!record.meta_information.empty()) {
        out.meta_information[0UZ] = record.meta_information[0UZ];
    }
}

//! @p record with @p reason written beside its facts, which is what a reject port publishes.
[[nodiscard]] inline DataSet<std::uint8_t> rejected(const DataSet<std::uint8_t>& record, const char* reason) {
    DataSet<std::uint8_t> out = record;
    if (out.meta_information.empty()) {
        out.meta_information.resize(1UZ);
    }
    out.meta_information[0UZ]["discard_reason"] = std::string(reason);
    return out;
}

} // namespace recorddetail

GR_REGISTER_BLOCK(gr::blocks::basic::RecordTrim)

/*!
@brief Removes a stated number of items from the front and the back of every record.

What a chain uses to discard the positions whose purpose is spent: a sync word a detector has
already consumed, or a decode margin extracted only so the bits inside it kept their future. The
output is items `[drop_head, n - drop_tail)` of the input, exactly `n - drop_head - drop_tail` of
them; a record exactly as long as the two trims is published empty, and a shorter one is a counted,
stated drop — `reject` carries it with `discard_reason = "shorter_than_trim"` and the record after
it trims normally. Every fact crosses verbatim; only the item span moves.

A connected `reject` bounds a call as `out` does: a refusal that has no room on it leaves its record
in the input buffer for the next call rather than being counted and written nowhere. An unconnected
`reject` is the stated drop, and there the count is all that is left to say so.

Only drop-head and drop-tail ship. A keep-head or keep-tail spelling would make two settings answer
one question, and the chains that want a kept window address it from the ends they know.
*/
struct RecordTrim : Block<RecordTrim> {
    using Description = Doc<"record trim: items [drop_head, n - drop_tail) of every record, the rest discarded; too short is a counted drop">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           out;
    PortOut<DataSet<std::uint8_t>, Async, Optional> reject;

    Annotated<gr::Size_t, "drop_head", Unit<"items">, Doc<"items removed from the start of every record">, Visible> drop_head = 0U;
    Annotated<gr::Size_t, "drop_tail", Unit<"items">, Doc<"items removed from the end of every record">, Visible>   drop_tail = 0U;

    GR_MAKE_REFLECTABLE(RecordTrim, in, out, reject, drop_head, drop_tail);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords      = 0ULL; ///< records published on `out`
    std::uint64_t nItemsDropped = 0ULL; ///< items the trims removed, totaled
    std::uint64_t nRefusedShort = 0ULL; ///< records shorter than the two trims together

    void stop() {
        if (nRefusedShort > 0ULL) {
            std::println(stderr, "gr::blocks::basic::RecordTrim '{}': {} record(s) shorter than the {} + {} items the trims remove", this->name, nRefusedShort, drop_head.value, drop_tail.value);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        const std::size_t head = static_cast<std::size_t>(drop_head.value);
        const std::size_t tail = static_cast<std::size_t>(drop_tail.value);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        std::size_t refused  = 0UZ;
        // a connected `reject` bounds the loop as `out` does: a refused record with nowhere to go waits in the input
        // buffer for the next call, where counting it and writing it nowhere would lose it
        const std::size_t rejectRoom = rejectSpan.isConnected ? rejectSpan.size() : std::numeric_limits<std::size_t>::max();
        for (; consumed < inSpan.size() && made < outSpan.size() && refused < rejectRoom; ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            items  = record.signal_values.size();
            if (items < head + tail) {
                ++nRefusedShort;
                if (rejectSpan.isConnected) {
                    rejectSpan[refused] = recorddetail::rejected(record, "shorter_than_trim");
                    ++refused;
                }
                continue;
            }

            DataSet<std::uint8_t> trimmed;
            trimmed.signal_values.assign(record.signal_values.begin() + static_cast<std::ptrdiff_t>(head), record.signal_values.end() - static_cast<std::ptrdiff_t>(tail));
            recorddetail::carryFacts(trimmed, record);

            ++nRecords;
            nItemsDropped += head + tail;
            outSpan[made] = std::move(trimmed);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        rejectSpan.publish(refused);
        if (made == 0UZ && refused == 0UZ && consumed == 0UZ) {
            const bool noRoom = outSpan.size() == 0UZ || (rejectSpan.isConnected && rejectSpan.size() == 0UZ);
            return noRoom ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::RecordLengthFilter)

/*!
@brief Routes records by their item count: inside `[min_items, max_items]` to `out`, the rest to `reject`.

The guard a chain puts in front of a consumer whose contract is a length: both bounds are inclusive,
`max_items` is required — a filter with no ceiling filters nothing anyone asked for — and a bound
pair that admits nothing refuses at staging naming both. A rejected record carries which side it
failed, `discard_reason` of `"length_below_min"` or `"length_above_max"`, and an unconnected
`reject` keeps the counts and drops the record with the total stated at `stop()`. Connected, the
port bounds a call as `out` does: a rejection with no room waits in the input buffer for the next
call rather than being counted and written nowhere.
*/
struct RecordLengthFilter : Block<RecordLengthFilter> {
    using Description = Doc<"record length filter: [min_items, max_items] passes, everything else is a counted, stated reject">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           out;
    PortOut<DataSet<std::uint8_t>, Async, Optional> reject;

    Annotated<gr::Size_t, "min_items", Unit<"items">, Doc<"inclusive lower bound">, Visible>                                     min_items = 0U;
    Annotated<gr::Size_t, "max_items", Unit<"items">, Doc<"inclusive upper bound; required, because 0 admits nothing">, Visible> max_items = 0U;

    GR_MAKE_REFLECTABLE(RecordLengthFilter, in, out, reject, min_items, max_items);

    bool _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords      = 0ULL; ///< records published on `out`
    std::uint64_t nRefusedShort = 0ULL; ///< records below min_items
    std::uint64_t nRefusedLong  = 0ULL; ///< records above max_items

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (max_items.value == 0U) {
            throw gr::exception("max_items is the inclusive ceiling and has no default: a filter that admits nothing filters nothing anyone asked for");
        }
        if (min_items.value > max_items.value) {
            throw gr::exception(std::format("min_items {} is above max_items {}: the pair admits nothing", min_items.value, max_items.value));
        }
        _configured = true;
    }

    void stop() {
        if (nRefusedShort + nRefusedLong > 0ULL) {
            std::println(stderr, "gr::blocks::basic::RecordLengthFilter '{}': {} record(s) below {} items, {} above {}", this->name, nRefusedShort, min_items.value, nRefusedLong, max_items.value);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        if (!_configured) { // inert rather than filtering by a bound nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            rejectSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        std::size_t refused  = 0UZ;
        // a connected `reject` bounds the loop as `out` does: a refused record with nowhere to go waits in the input
        // buffer for the next call, where counting it and writing it nowhere would lose it
        const std::size_t rejectRoom = rejectSpan.isConnected ? rejectSpan.size() : std::numeric_limits<std::size_t>::max();
        for (; consumed < inSpan.size() && made < outSpan.size() && refused < rejectRoom; ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            items  = record.signal_values.size();
            if (items >= static_cast<std::size_t>(min_items.value) && items <= static_cast<std::size_t>(max_items.value)) {
                outSpan[made] = record;
                ++made;
                ++nRecords;
                continue;
            }
            const bool below = items < static_cast<std::size_t>(min_items.value);
            (below ? nRefusedShort : nRefusedLong) += 1ULL;
            if (rejectSpan.isConnected) {
                rejectSpan[refused] = recorddetail::rejected(record, below ? "length_below_min" : "length_above_max");
                ++refused;
            }
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        rejectSpan.publish(refused);
        if (made == 0UZ && refused == 0UZ && consumed == 0UZ) {
            const bool noRoom = outSpan.size() == 0UZ || (rejectSpan.isConnected && rejectSpan.size() == 0UZ);
            return noRoom ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_RECORD_UTILS_HPP
