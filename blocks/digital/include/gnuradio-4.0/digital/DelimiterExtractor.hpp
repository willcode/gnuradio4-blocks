#ifndef GNURADIO_DIGITAL_DELIMITER_EXTRACTOR_HPP
#define GNURADIO_DIGITAL_DELIMITER_EXTRACTOR_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Delimiter.hpp>
#include <gnuradio-4.0/algorithm/math/BitPacking.hpp>

#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::DelimiterExtractor, [T], [ std::uint8_t, float ])

/**
 * @brief The third extractor: frames delimited by a reserved item pattern the block matches itself.
 *
 * `StreamToDataSet` cuts a fixed window from a trigger and `PacketFramer` reads a length out of a header. A
 * delimiter-framed protocol has neither, because the length is not known until the frame ends, so this block
 * matches the delimiter and cuts on it. The scanning, the transparency decoding and the length classification are
 * `gr::digital::DelimiterScanner`'s; what is added here is the ports, the settings, the records and the tags.
 */
template<BitLike T>
struct DelimiterExtractor : Block<DelimiterExtractor<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Cuts frames out of a stream on a reserved delimiter that may not occur in the payload, one `DataSet<T>` per frame.

The extractor for protocols that mark the end of a frame rather than declaring its length: HDLC, AX.25 and
synchronous PPP with a bit-stuffed `01111110` flag, SLIP and KISS with a byte-escaped `0xC0`, NMEA 0183 with `$` and
`CR LF` over a printable-ASCII payload. `gr::digital::hdlc()`, `slip()`, `pppAsync()` and `nmea0183()` are those four
framings as library constants; none of them is a default here, because there is no universal delimiter and a silent
one would be a silent interoperability assumption.

The delimiter is spelled as a bit string, most significant bit first in wire order, and `bits_per_item` says how many
bits an input item carries, so a bit-level and a byte-level delimiter are the same mechanism at 1 and at 8. A soft
stream carries one bit per item.
`transparency` names how the protocol keeps the delimiter out of the payload, and the settings are checked against
that claim when they are staged: under bit stuffing a delimiter a conforming encoder could produce is refused
outright, and under byte escaping, where the guarantee depends on a peer this block cannot inspect, it is warned
about.

`end_delimiter` and `max_payload_items` are both required and neither has a default; without either the block is not
ready. The bound is what keeps a frame that never ends — a corrupted closing delimiter,
or a stream that is not the protocol configured — from accumulating with the stream.

Every record on `out` was closed by a delimiter the block matched. A frame that reached the bound, one shorter than
`min_payload_items`, and one whose decoded length is not a multiple of `payload_pack_bits` leave by `reject` instead,
carrying `discard_reason`. A region between two delimiters carrying no items is idle rather than a short frame and is
counted without being published, because an idle link is back-to-back delimiters. `sequence` counts records across
both ports, so a gap in `out`'s own sequence is exactly a record that went to `reject`.

The delimiters are written as '0'/'1' characters, MSB first in wire order; `start_delimiter` is read under frame_open
start, and under frame_open delimiter the closing delimiter opens the next frame. Under bit stuffing, a 0 arriving after
exactly `stuff_after_ones` consecutive 1s is removed, and `abort_ones` consecutive 1s abandon the frame in progress.
Under byte escaping, `escape_map` holds flat (escaped, original) pairs, at most 256. `max_payload_items` and
`min_payload_items` count decoded items, and a closed region shorter than `min_payload_items` is refused as undersize;
an HDLC link at bits_per_item 1 sets 24 for a three-octet minimum. A `payload_pack_bits` of 1 to 8 repacks the decoded
bits into items of that width, with bits_per_item 1 and byte items. `trigger` names the gr:trigger_name that
resynchronizes the machine, and `frame_label` is written under the record's trigger_name key when no detector supplied
one.
)"">;

    PortIn<T>                            in;
    PortOut<DataSet<T>, Async>           out;
    PortOut<DataSet<T>, Async, Optional> reject;

    struct Settings {
        Annotated<std::string, Doc<"closing delimiter bits">, Visible>              end_delimiter{};
        Annotated<gr::digital::FrameOpen, Doc<"frame opening">, Visible>            frame_open = gr::digital::FrameOpen::delimiter;
        Annotated<std::string, Doc<"opening delimiter bits">>                       start_delimiter{};
        Annotated<gr::Size_t, Doc<"bits per input item">, Limits<Min<1U>, Max<8U>>> bits_per_item    = 1U;
        Annotated<gr::math::BitOrder, Doc<"bit order">>                             bit_order        = gr::math::BitOrder::msb_first;
        Annotated<gr::digital::Transparency, Doc<"payload transparency">, Visible>  transparency     = gr::digital::Transparency::none;
        Annotated<gr::Size_t, Doc<"ones before a stuffed bit">>                     stuff_after_ones = 5U;
        Annotated<gr::Size_t, Doc<"abort run length (0 = off)">>                    abort_ones       = 7U;
        Annotated<gr::Size_t, Doc<"escape value">, Limits<Max<255U>>>               escape_item      = 0U;
        Annotated<std::vector<gr::Size_t>, Doc<"escape pairs">>                     escape_map{};
        Annotated<gr::Size_t, Unit<"items">, Doc<"payload length limit">, Visible>  max_payload_items = 0U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"minimum payload length">>         min_payload_items = 1U;
        Annotated<gr::Size_t, Doc<"payload pack width (0 = unpacked)">>             payload_pack_bits = 0U;
        Annotated<gr::math::BitOrder, Doc<"payload bit order">>                     payload_bit_order = gr::math::BitOrder::msb_first;
        Annotated<std::string, Doc<"resynchronizing trigger name (empty = any)">>   trigger{};
        Annotated<bool, Doc<"trigger reset">>                                       trigger_resets = true;
        Annotated<std::string, Doc<"default frame trigger name">>                   frame_label    = std::string("delimiter");

        GR_MAKE_REFLECTABLE(Settings, end_delimiter, frame_open, start_delimiter, bits_per_item, bit_order, transparency, stuff_after_ones, abort_ones, escape_item, escape_map, max_payload_items, min_payload_items, payload_pack_bits, payload_bit_order, trigger, trigger_resets, frame_label);
    } settings;

    GR_MAKE_REFLECTABLE(DelimiterExtractor, in, out, reject, settings);

    // Counted, stated drops. Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nFramesEmitted       = 0ULL; ///< records published on `out`; the denominator the others are read against
    std::uint64_t nOverrunFrames       = 0ULL; ///< frames that reached `max_payload_items` with no closing delimiter
    std::uint64_t nUndersizeFrames     = 0ULL; ///< closed frames shorter than `min_payload_items`
    std::uint64_t nUnalignedFrames     = 0ULL; ///< closed frames whose decoded count is not a multiple of `payload_pack_bits`
    std::uint64_t nIdleDelimiters      = 0ULL; ///< delimiters closing a zero-item region
    std::uint64_t nAborts              = 0ULL; ///< abort runs under bit stuffing
    std::uint64_t nRestarts            = 0ULL; ///< start delimiters arriving while a frame was open
    std::uint64_t nTriggerResets       = 0ULL; ///< trigger tags that resynchronized the machine
    std::uint64_t nEscapeViolations    = 0ULL; ///< escaped items with no entry in the escape map
    std::uint64_t nItemsMasked         = 0ULL; ///< items whose bits above `bits_per_item` were not zero
    std::uint64_t nItemsDiscarded      = 0ULL; ///< items consumed outside a frame, or belonging to a frame that was abandoned
    std::uint64_t nTagsOutsideFrames   = 0ULL; ///< input tags outside any emitted frame's payload region
    std::uint64_t nTagsOnRemovedItems  = 0ULL; ///< input tags on a removed item, carried to the next surviving item
    std::uint64_t nDroppedTimingEvents = 0ULL; ///< events whose computed index fell outside the record

    /// @brief One input tag waiting for its frame to close, at the record index the machine computed when it arrived.
    struct Pending {
        std::size_t  index   = 0UZ;
        bool         removed = false; ///< whether the tag's own item was removed, which displaces it by one item
        property_map map{};
    };

    /// @brief One arriving tag, as a position in the current span and a borrowed map.
    struct Arriving {
        std::size_t         at  = 0UZ;
        const property_map* map = nullptr;
    };

    /// @brief The event list is bounded by `max_payload_items`; this is how much of that bound is reserved up front.
    static constexpr std::size_t kPendingReserve = 4096UZ;

    gr::digital::DelimiterScanner<T> _scanner{};
    std::vector<Pending>             _pending{};
    std::vector<Arriving>            _tags{};
    property_map                     _triggerMeta{};
    std::optional<float>             _rate{};
    std::uint64_t                    _sequence = 0ULL;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        using gr::digital::Transparency;
        const std::vector<gr::Size_t> map = s.escape_map;
        if (s.end_delimiter.value.empty()) {
            return gr::Refused(s.end_delimiter, "required");
        }
        if (s.max_payload_items == 0U) {
            return gr::Refused(s.max_payload_items, "required");
        }
        if (s.transparency == Transparency::byte_escape && map.size() % 2UZ != 0UZ) {
            return gr::Refused(s.escape_map, "must have an even length");
        }
        if (s.transparency == Transparency::byte_escape && std::ranges::any_of(map, [](gr::Size_t value) { return value > 255U; })) {
            return gr::Refused(s.escape_map, "each value must be <= 255");
        }
        if (std::floating_point<T> && s.bits_per_item != 1U) {
            return gr::Refused(s.bits_per_item, "must be 1 on a floating-point stream");
        }
        if (std::floating_point<T> && s.payload_pack_bits != 0U) {
            return gr::Refused(s.payload_pack_bits, "must be 0 on a floating-point stream");
        }
        std::ignore = framing(s);
        return {};
    }

    void applySettings(const Settings& s) {
        gr::digital::DelimiterConfig cfg     = framing(s);
        const std::string            warning = gr::digital::unforgeabilityWarning(cfg);
        if (!warning.empty()) {
            std::println(stderr, "gr::blocks::digital::DelimiterExtractor '{}': {}", this->name, warning);
        }

        _scanner.prepare(std::move(cfg));
        _pending.reserve(std::min<std::size_t>(s.max_payload_items, kPendingReserve));
        clearRun();
    }

    void start() { clearRun(); }

    /// @brief An idle machine with every count at zero: a new framing starts there, and so does a restart.
    void clearRun() {
        reset();
        _scanner.counters    = {};
        nFramesEmitted       = 0ULL;
        nTriggerResets       = 0ULL;
        nTagsOutsideFrames   = 0ULL;
        nTagsOnRemovedItems  = 0ULL;
        nDroppedTimingEvents = 0ULL;
        refreshCounters();
    }

    void reset() {
        _scanner.reset();
        _pending.clear();
        _triggerMeta.clear();
        _rate.reset();
        _sequence = 0ULL;
    }

    /// @brief The framing the settings in @p n name, configured by the library, which throws for one it refuses.
    [[nodiscard]] gr::digital::DelimiterConfig framing(const Settings& s) const {
        gr::digital::DelimiterConfig cfg;
        cfg.endDelimiter    = s.end_delimiter.value;
        cfg.frameOpen       = s.frame_open.value;
        cfg.transparency    = s.transparency.value;
        cfg.bitOrder        = s.bit_order.value;
        cfg.packOrder       = s.payload_bit_order.value;
        cfg.bitsPerItem     = s.bits_per_item.value;
        cfg.maxPayloadItems = s.max_payload_items.value;
        cfg.minPayloadItems = s.min_payload_items.value;
        cfg.packBits        = s.payload_pack_bits.value;
        // a setting of a mode not configured keeps the library's default: ignored, never refused
        if (cfg.frameOpen == gr::digital::FrameOpen::start) {
            cfg.startDelimiter = s.start_delimiter.value;
        }
        if (cfg.transparency == gr::digital::Transparency::bit_stuffing) {
            cfg.stuffAfterOnes = s.stuff_after_ones.value;
            cfg.abortOnes      = s.abort_ones.value;
        }
        if (cfg.transparency == gr::digital::Transparency::byte_escape) {
            cfg.escapeItem                    = static_cast<std::uint8_t>(s.escape_item.value);
            const std::vector<gr::Size_t> map = s.escape_map.value;
            for (std::size_t i = 0UZ; i + 1UZ < map.size(); i += 2UZ) {
                cfg.escapeMap.emplace_back(static_cast<std::uint8_t>(map[i]), static_cast<std::uint8_t>(map[i + 1UZ]));
            }
        }
        gr::digital::configureDelimiter(cfg);
        return cfg;
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("frames emitted", nFramesEmitted);
        append("overrun frames", nOverrunFrames);
        append("undersize frames", nUndersizeFrames);
        append("unaligned frames", nUnalignedFrames);
        append("idle delimiters", nIdleDelimiters);
        append("aborts", nAborts);
        append("restarts", nRestarts);
        append("trigger resets", nTriggerResets);
        append("escape violations", nEscapeViolations);
        append("items masked", nItemsMasked);
        append("items discarded", nItemsDiscarded);
        append("tags outside frames", nTagsOutsideFrames);
        append("tags on removed items", nTagsOnRemovedItems);
        append("dropped timing events", nDroppedTimingEvents);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::DelimiterExtractor '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& rejectSpan) {
        _tags.clear();
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            // a block holding a partial frame across calls is shown an earlier call's tag again at a negative index
            if (relIndex >= 0 && static_cast<std::size_t>(relIndex) < inSpan.size()) {
                _tags.push_back({static_cast<std::size_t>(relIndex), &tagMap.get()});
            }
        }
        std::ranges::stable_sort(_tags, std::ranges::less{}, &Arriving::at);

        const bool        outConnected = outSpan.isConnected;
        const bool        rejConnected = rejectSpan.isConnected;
        const std::size_t outRoom      = outConnected ? outSpan.size() : std::numeric_limits<std::size_t>::max();
        const std::size_t rejRoom      = rejConnected ? rejectSpan.size() : std::numeric_limits<std::size_t>::max();

        std::size_t onOut     = 0UZ;
        std::size_t onReject  = 0UZ;
        std::size_t consumed  = 0UZ;
        std::size_t tagCursor = 0UZ;
        bool        starved   = false;

        _scanner.seek(inSpan.streamIndex);
        for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
            if (onOut >= outRoom || onReject >= rejRoom) { // any item may close a frame, so both ports need room for one
                starved = true;
                break;
            }

            const std::size_t tagBegin = tagCursor;
            while (tagCursor < _tags.size() && _tags[tagCursor].at == i) {
                ++tagCursor;
            }
            const std::size_t tagEnd = tagCursor;

            bool resetHere = false;
            for (std::size_t k = tagBegin; k < tagEnd; ++k) {
                readRate(*_tags[k].map);
                if (!resetHere && settings.trigger_resets && matches(*_tags[k].map)) {
                    _triggerMeta = *_tags[k].map;
                    _scanner.resynchronize();
                    dropPending();
                    ++nTriggerResets;
                    resetHere = true;
                }
            }

            const gr::digital::ScanStep step = _scanner.push(inSpan[i]);

            for (std::size_t k = tagBegin; k < tagEnd; ++k) {
                if (_tags[k].map->empty() || (resetHere && matches(*_tags[k].map))) {
                    continue; // the trigger tag is consumed into the record's metadata rather than becoming an event
                }
                if (!step.inFrame) {
                    ++nTagsOutsideFrames;
                    continue;
                }
                if (_pending.size() >= static_cast<std::size_t>(settings.max_payload_items.value)) {
                    ++nDroppedTimingEvents; // the framework permits several tags at one index, so the list is bounded by fiat
                    continue;
                }
                _pending.push_back({step.index, step.removed, *_tags[k].map});
            }

            switch (step.event) {
            case gr::digital::ScanEvent::Record: {
                DataSet<T> record = buildRecord();
                if (_scanner.frameOutcome() == gr::digital::FrameOutcome::Emitted) {
                    if (outConnected && onOut < outSpan.size()) {
                        outSpan[onOut] = std::move(record);
                    }
                    ++onOut;
                    ++nFramesEmitted;
                } else {
                    if (rejConnected && onReject < rejectSpan.size()) {
                        rejectSpan[onReject] = std::move(record);
                    }
                    ++onReject;
                }
                break;
            }
            case gr::digital::ScanEvent::Idle:
            case gr::digital::ScanEvent::Abort:
            case gr::digital::ScanEvent::Restart: dropPending(); break;
            default: break;
            }

            consumed = i + 1UZ;
        }

        refreshCounters();
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(outConnected ? onOut : 0UZ);
        rejectSpan.publish(rejConnected ? onReject : 0UZ);
        if (consumed == 0UZ && onOut == 0UZ && onReject == 0UZ) {
            return starved ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief The record the closed frame becomes, with its metadata, its timing events and its sequence number.
    [[nodiscard]] DataSet<T> buildRecord() {
        const std::span<const T> payload = _scanner.frame();
        const std::size_t        decoded = _scanner.frameDecodedItems();
        // a packed item holds `payload_pack_bits` decoded items, so a tag on any of them lands on that one item
        const std::size_t width = _scanner.framePacked() ? static_cast<std::size_t>(settings.payload_pack_bits.value) : 1UZ;

        DataSet<T> record;
        record.signal_values.assign(payload.begin(), payload.end());
        record.extents.push_back(static_cast<std::int32_t>(payload.size()));
        record.signal_names.emplace_back("payload");
        record.signal_quantities.emplace_back("");
        record.signal_units.emplace_back("");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        record.timestamp = triggerTime();

        const std::uint64_t start    = _scanner.frameStart();
        const std::uint64_t sequence = _sequence;
        property_map&       meta     = record.meta_information[0UZ];
        meta.insert_or_assign(property_map::key_type(gr::tag::TRIGGER_NAME.name()), triggerName());
        meta.insert_or_assign(property_map::key_type("sample_start"), pmt::Value(start));
        meta.insert_or_assign(property_map::key_type("sequence"), pmt::Value(sequence));
        meta.insert_or_assign(property_map::key_type("stuffing_removed"), pmt::Value(static_cast<gr::Size_t>(_scanner.frameRemoved())));
        if (_rate.has_value()) {
            meta.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.name()), pmt::Value(*_rate));
        }
        if (const std::optional<pmt::Value> errors = syncErrors(); errors.has_value()) {
            meta.insert_or_assign(property_map::key_type("sync_errors"), *errors);
        }
        if (const std::string_view reason = gr::digital::discardReasonName(_scanner.frameOutcome()); !reason.empty()) {
            meta.insert_or_assign(property_map::key_type("discard_reason"), pmt::Value(reason));
        }

        auto& events = record.timing_events[0UZ];
        for (Pending& event : _pending) {
            if (event.index >= decoded) {
                // a tag on the frame's last raw item when that item was removed lands one past the end, and a tag
                // inside the closing delimiter's own items lands further out still; neither is clamped
                ++(event.removed ? nDroppedTimingEvents : nTagsOutsideFrames);
                continue;
            }
            if (event.removed) {
                ++nTagsOnRemovedItems;
            }
            events.emplace_back(static_cast<std::ptrdiff_t>(event.index / width), std::move(event.map));
        }
        _pending.clear();

        ++_sequence; // one counter across both ports, so a gap in `out`'s own sequence is exactly a rejected record
        _triggerMeta.clear();
        return record;
    }

    /// @brief Discards the events of a frame that never became a record, counting them as tags outside a frame.
    void dropPending() {
        nTagsOutsideFrames += _pending.size();
        _pending.clear();
    }

    /// @brief Whether @p tagMap is a trigger tag this block accepts; an empty `trigger` setting accepts any.
    [[nodiscard]] bool matches(const property_map& tagMap) const {
        const auto entry = tagMap.find(property_map::key_type(gr::tag::TRIGGER_NAME.key()));
        if (entry == tagMap.end()) {
            return false;
        }
        if (settings.trigger.value.empty()) {
            return true;
        }
        return entry->second.value_or(std::string_view{}) == std::string_view(settings.trigger.value);
    }

    /// @brief Holds the most recent input `sample_rate` as provenance about the stream the frames were taken from.
    void readRate(const property_map& tagMap) {
        const auto entry = tagMap.find(property_map::key_type(gr::tag::SAMPLE_RATE.key()));
        if (entry == tagMap.end()) {
            return;
        }
        if (const float* value = entry->second.template get_if<float>(); value != nullptr) {
            _rate = *value;
        }
    }

    /// @brief The trigger's own label, or `frame_label` when no detector supplied one.
    [[nodiscard]] pmt::Value triggerName() const {
        const auto entry = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_NAME.key()));
        return entry == _triggerMeta.end() ? pmt::Value(std::string_view(settings.frame_label.value)) : entry->second;
    }

    /// @brief The detector's Hamming distance, which travels inside `gr:trigger_meta_info` rather than as a tag key.
    [[nodiscard]] std::optional<pmt::Value> syncErrors() const {
        const auto outer = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_META_INFO.key()));
        if (outer == _triggerMeta.end()) {
            return std::nullopt;
        }
        const auto* nested = outer->second.template get_if<property_map>();
        if (nested == nullptr) {
            return std::nullopt;
        }
        const auto inner = nested->find(property_map::key_type("sync_errors"));
        return inner == nested->end() ? std::nullopt : std::optional<pmt::Value>(inner->second);
    }

    /// @brief The trigger's `gr:trigger_time`, or zero when none was supplied; an absent time is a stated absence.
    [[nodiscard]] std::int64_t triggerTime() const {
        const auto entry = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_TIME.key()));
        if (entry != _triggerMeta.end()) {
            if (const auto* stamp = entry->second.template get_if<std::uint64_t>(); stamp != nullptr) {
                return static_cast<std::int64_t>(*stamp);
            }
        }
        return 0;
    }

    /// @brief Mirrors the scanner's counters onto this block's own, once per call rather than once per item.
    void refreshCounters() noexcept {
        const gr::digital::DelimiterCounters& counted = _scanner.counters;
        nOverrunFrames                                = counted.nOverrunFrames;
        nUndersizeFrames                              = counted.nUndersizeFrames;
        nUnalignedFrames                              = counted.nUnalignedFrames;
        nIdleDelimiters                               = counted.nIdleDelimiters;
        nAborts                                       = counted.nAborts;
        nRestarts                                     = counted.nRestarts;
        nEscapeViolations                             = counted.nEscapeViolations;
        nItemsMasked                                  = counted.nItemsMasked;
        nItemsDiscarded                               = counted.nItemsDiscarded;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_DELIMITER_EXTRACTOR_HPP
