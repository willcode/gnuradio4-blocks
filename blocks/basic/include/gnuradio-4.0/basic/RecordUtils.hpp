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
 * Two record utilities that do no arithmetic. RecordTrim removes a stated head and tail from every
 * record. RecordLengthFilter routes records by their item count. Both copy every fact a record
 * carries and change none. They act on the item span and the routing alone. A refused record is
 * counted, and a `reject` port carries it with its reason.
 *
 * The trim does not rewrite `sample_start`. The key counts samples of the stream the record was cut
 * from. The trim removes items of the record. The two units agree only when one item is one sample.
 * A rewrite would be right only for one sample per item.
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

//! A copy of @p record with `discard_reason` set to @p reason, as a reject port publishes it.
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

The block discards positions a chain has finished with. Examples are a sync word a detector has
already consumed and a decode margin extracted only to give the bits inside the record a
look-ahead. The output is items `[drop_head, n - drop_tail)` of the input, exactly
`n - drop_head - drop_tail` items. A record exactly as long as the two trims is published empty.
A shorter record is counted and refused. `reject` carries it with
`discard_reason = "shorter_than_trim"`, and the next record trims normally. Every fact crosses
unchanged. Only the item span changes.

A connected `reject` bounds a call as `out` does. A refused record without room on `reject` stays
in the input buffer for the next call. With `reject` unconnected the record is dropped, and the
count alone records the drop.

The block has drop-head and drop-tail settings only. Keep-head or keep-tail settings would give two
settings for one quantity. A kept window is stated by its distance from the two ends.
*/
struct RecordTrim : Block<RecordTrim> {
    using Description = Doc<"Keeps items [drop_head, n - drop_tail) of every record. A record shorter than the two trims is counted and refused.">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           out;
    PortOut<DataSet<std::uint8_t>, Async, Optional> reject;

    Annotated<gr::Size_t, "drop_head", Unit<"items">, Doc<"items removed from the start of every record">, Visible> drop_head = 0U;
    Annotated<gr::Size_t, "drop_tail", Unit<"items">, Doc<"items removed from the end of every record">, Visible>   drop_tail = 0U;

    GR_MAKE_REFLECTABLE(RecordTrim, in, out, reject, drop_head, drop_tail);

    // Plain members, written by the owning thread and reported once at stop().
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
        // a connected `reject` bounds the loop as `out` does, and a refused record without room waits for the next call
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
@brief Routes records by item count, those within `[min_items, max_items]` to `out` and the rest to `reject`.

The block guards a consumer whose contract is a length. Both bounds are inclusive. `max_items` is
required and has no default. A bound pair that admits nothing is refused at staging, with both
values named. A rejected record carries the side it failed in `discard_reason`, either
`"length_below_min"` or `"length_above_max"`. With `reject` unconnected the block counts and drops
the record and states the totals at `stop()`. A connected `reject` bounds a call as `out` does. A
rejected record without room waits in the input buffer for the next call.
*/
struct RecordLengthFilter : Block<RecordLengthFilter> {
    using Description = Doc<"Passes records of min_items to max_items items. Every other record is counted and rejected.">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           out;
    PortOut<DataSet<std::uint8_t>, Async, Optional> reject;

    Annotated<gr::Size_t, "min_items", Unit<"items">, Doc<"inclusive lower bound">, Visible>                      min_items = 0U;
    Annotated<gr::Size_t, "max_items", Unit<"items">, Doc<"inclusive upper bound, required, 0 refused">, Visible> max_items = 0U;

    GR_MAKE_REFLECTABLE(RecordLengthFilter, in, out, reject, min_items, max_items);

    bool _configured = false;

    // Plain members, written by the owning thread and reported once at stop().
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
        if (!_configured) { // without an accepted bound pair the block is inert and returns ERROR
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            rejectSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        std::size_t refused  = 0UZ;
        // a connected `reject` bounds the loop as `out` does
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
