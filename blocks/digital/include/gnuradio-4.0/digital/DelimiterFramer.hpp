#ifndef GNURADIO_DIGITAL_DELIMITER_FRAMER_HPP
#define GNURADIO_DIGITAL_DELIMITER_FRAMER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
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
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Delimiter.hpp>
#include <gnuradio-4.0/algorithm/math/BitPacking.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::DelimiterFramer)

/**
 * @brief `DelimiterExtractor`'s inverse: one record becomes one delimited, transparency-coded frame.
 *
 * The assembly, the transparency coding and the transmit-side validation are `gr::digital::DelimiterEncoder`'s over
 * the same `DelimiterConfig` the extractor scans with; what is added here is the ports, the settings, the record and
 * the counters.
 */
struct DelimiterFramer : Block<DelimiterFramer> {
    using Description = Doc<R""(
@brief Wraps each record in a delimiter a receiver can find, coding the payload so the delimiter cannot occur inside it.

The transmit side of `DelimiterExtractor`, over the same framing: HDLC, AX.25 and synchronous PPP with a bit-stuffed
`01111110` flag, SLIP and KISS with a byte-escaped `0xC0`, NMEA 0183 with `$` and `CR LF` over a printable-ASCII
payload. `gr::digital::hdlc()`, `slip()`, `pppAsync()` and `nmea0183()` are those four framings as library constants,
and none of them is a default here either, because a silent delimiter is a silent interoperability assumption.

One record in, one framed record out. The output record's items are wire items at the configured `bits_per_item` —
bit items for HDLC, byte items for SLIP, KISS and PPP — carrying the opening delimiter, the coded payload and the
closing delimiter in wire order. A framed record is a burst a transmitter can key on; turning a run of them into a
continuous stream is `DataSetToStream`'s job, and an idle link's inter-frame fill is the transmitter's policy.

`end_delimiter` and `max_payload_items` are both required and neither has a default; without either the block is not
ready. The bound is the number the receiving end refuses above, which makes it the number
this end must not send: a record whose payload exceeds it, and an empty record, are counted drops that publish nothing,
because framing either would put something on the air the peer is specified to reject.

Under byte escaping the settings are held to more than a receiver can hold them to. Every item value of a delimiter,
and the escape introducer itself, must appear as an original in the escape map, because here the encoder is the peer
whose conformance a receiver could only warn about. Under `none` nothing is checkable at all, so the block counts
instead: every payload item at which the receiver's match register would hold the delimiter increments
`nForgedDelimiters`, each one a frame the receiver will cut in half.

Metadata crosses verbatim and the record gains `stuffing_inserted`, the mirror of the extractor's `stuffing_removed`,
so a loopback chain can be read frame by frame.

The delimiters are written as '0'/'1' characters, MSB first; `start_delimiter` is read under frame_open start.
`stuff_after_ones` consecutive payload 1s are followed by a stuffed 0 under bit stuffing, and `abort_ones` is the
receiver's abort run length. Under byte escaping, `escape_map` holds flat (escaped, original) pairs, and the last pair
for a value wins. Records longer than `max_payload_items` wire items are dropped. A nonzero `payload_pack_bits` unpacks
that many bits per record item and needs bits_per_item 1.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<std::string, Doc<"closing delimiter bits">, Visible>               end_delimiter{};
        Annotated<gr::digital::FrameOpen, Doc<"frame opening">, Visible>             frame_open = gr::digital::FrameOpen::delimiter;
        Annotated<std::string, Doc<"opening delimiter bits">>                        start_delimiter{};
        Annotated<gr::Size_t, Doc<"bits per output item">, Limits<Min<1U>, Max<8U>>> bits_per_item    = 1U;
        Annotated<gr::math::BitOrder, Doc<"bit order">>                              bit_order        = gr::math::BitOrder::msb_first;
        Annotated<gr::digital::Transparency, Doc<"payload transparency">, Visible>   transparency     = gr::digital::Transparency::none;
        Annotated<gr::Size_t, Doc<"ones before a stuffed bit">>                      stuff_after_ones = 5U;
        Annotated<gr::Size_t, Doc<"receiver abort run length">>                      abort_ones       = 7U;
        Annotated<gr::Size_t, Doc<"escape value">, Limits<Max<255U>>>                escape_item      = 0U;
        Annotated<std::vector<gr::Size_t>, Doc<"escape pairs">>                      escape_map{};
        Annotated<gr::Size_t, Unit<"items">, Doc<"payload length limit">, Visible>   max_payload_items = 0U;
        Annotated<gr::Size_t, Doc<"payload pack width (0 = unpacked)">>              payload_pack_bits = 0U;
        Annotated<gr::math::BitOrder, Doc<"payload bit order">>                      payload_bit_order = gr::math::BitOrder::msb_first;

        GR_MAKE_REFLECTABLE(Settings, end_delimiter, frame_open, start_delimiter, bits_per_item, bit_order, transparency, stuff_after_ones, abort_ones, escape_item, escape_map, max_payload_items, payload_pack_bits, payload_bit_order);
    } settings;

    GR_MAKE_REFLECTABLE(DelimiterFramer, in, out, settings);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords          = 0ULL; ///< records framed and published; the denominator the others are read against
    std::uint64_t nItemsEmitted     = 0ULL; ///< wire items published, both delimiters included
    std::uint64_t nStuffedBits      = 0ULL; ///< zeros inserted under bit stuffing
    std::uint64_t nEscapedItems     = 0ULL; ///< introducers inserted under byte escaping
    std::uint64_t nForgedDelimiters = 0ULL; ///< payload items carrying the delimiter under 'none', each a frame the receiver cuts in half
    std::uint64_t nRecordsRefused   = 0ULL; ///< records dropped as empty or longer than the bound

    gr::digital::DelimiterEncoder _encoder{};
    std::vector<std::uint8_t>     _framed{};

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
        std::ignore = framing(s);
        return {};
    }

    void applySettings(const Settings& s) {
        _encoder.prepare(framing(s));
        _framed.clear();
        _framed.reserve(gr::digital::framedItemsBound(_encoder.config, s.max_payload_items));
        clearCounts();
    }

    void start() { clearCounts(); }

    /// @brief Every count at zero: a new framing counts from there, and so does a restart.
    void clearCounts() {
        nRecords          = 0ULL;
        nItemsEmitted     = 0ULL;
        nStuffedBits      = 0ULL;
        nEscapedItems     = 0ULL;
        nForgedDelimiters = 0ULL;
        nRecordsRefused   = 0ULL;
    }

    /// @brief The framing the settings in @p n name, configured for the transmit side by the library, which throws for one it refuses.
    [[nodiscard]] gr::digital::DelimiterConfig framing(const Settings& s) const {
        gr::digital::DelimiterConfig cfg;
        cfg.endDelimiter    = s.end_delimiter.value;
        cfg.frameOpen       = s.frame_open.value;
        cfg.transparency    = s.transparency.value;
        cfg.bitOrder        = s.bit_order.value;
        cfg.packOrder       = s.payload_bit_order.value;
        cfg.bitsPerItem     = s.bits_per_item.value;
        cfg.maxPayloadItems = s.max_payload_items.value;
        cfg.packBits        = s.payload_pack_bits.value;
        // the settings of the other modes stay at the library's defaults
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
        gr::digital::framerConfigure(cfg);
        return cfg;
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records framed", nRecords);
        append("items emitted", nItemsEmitted);
        append("stuffed bits", nStuffedBits);
        append("escaped items", nEscapedItems);
        append("forged delimiters", nForgedDelimiters);
        append("records refused", nRecordsRefused);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::DelimiterFramer '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool        connected = outSpan.isConnected; // read once: the record count below is derived from it
        const std::size_t available = inSpan.size();
        const std::size_t nInput    = connected ? std::min(available, outSpan.size()) : available;
        std::size_t       published = 0UZ;

        for (std::size_t i = 0UZ; i < nInput; ++i) {
            const std::span<const std::uint8_t> payload = payloadOf(inSpan[i]);
            const std::size_t                   items   = _encoder.unpackedItems(payload.size());
            if (items == 0UZ || items > static_cast<std::size_t>(settings.max_payload_items.value)) {
                // a frame with no items is the receiver's idle rather than a frame, and one above the bound is a frame
                // the peer is specified to reject; both are counted here and stated at stop()
                ++nRecordsRefused;
                continue;
            }

            const gr::digital::EncodeResult framed = _encoder.encode(payload, _framed);
            nItemsEmitted += static_cast<std::uint64_t>(framed.emitted);
            nStuffedBits += _encoder.config.transparency == gr::digital::Transparency::bit_stuffing ? static_cast<std::uint64_t>(framed.inserted) : 0ULL;
            nEscapedItems += _encoder.config.transparency == gr::digital::Transparency::byte_escape ? static_cast<std::uint64_t>(framed.inserted) : 0ULL;
            nForgedDelimiters += static_cast<std::uint64_t>(framed.forged);
            ++nRecords;

            DataSet<std::uint8_t> record = inSpan[i];
            record.signal_values.assign(_framed.begin(), _framed.end());
            if (record.extents.empty()) {
                record.extents.push_back(static_cast<std::int32_t>(_framed.size()));
            } else {
                record.extents[0UZ] = static_cast<std::int32_t>(_framed.size());
            }
            if (record.signal_names.empty()) {
                record.signal_names.emplace_back("frame");
                record.signal_quantities.emplace_back("");
                record.signal_units.emplace_back("");
            }
            // the record's items are wire items now, and an index measured in payload items names none of them
            record.timing_events.clear();
            record.timing_events.emplace_back();
            if (record.meta_information.empty()) {
                record.meta_information.emplace_back();
            }
            record.meta_information[0UZ].insert_or_assign(property_map::key_type("stuffing_inserted"), pmt::Value(static_cast<gr::Size_t>(framed.inserted)));

            if (connected && published < outSpan.size()) {
                outSpan[published] = std::move(record);
            }
            ++published;
        }

        std::ignore = inSpan.consume(nInput);
        outSpan.publish(connected ? published : 0UZ);
        // No room on the output is backpressure, not a stall: the framework counts a bare OK with nothing consumed and
        // nothing published as no progress and reports the block.
        // An empty input returns INSUFFICIENT_INPUT_ITEMS. The framework calls on an empty input too, because the
        // async output counts as ready whenever it has room.
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return nInput == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }

    /// @brief One record's item array, with the single-signal restriction stated at the port rather than assumed.
    [[nodiscard]] static std::span<const std::uint8_t> payloadOf(const DataSet<std::uint8_t>& record) {
        if (record.signal_names.size() > 1UZ) {
            throw gr::exception(std::format("a frame carries one flat item array; this DataSet carries {} signals", record.signal_names.size()));
        }
        return std::span<const std::uint8_t>(record.signal_values);
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_DELIMITER_FRAMER_HPP
