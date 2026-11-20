#ifndef GNURADIO_DIGITAL_LENGTH_HEADER_FRAMER_HPP
#define GNURADIO_DIGITAL_LENGTH_HEADER_FRAMER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <optional>
#include <print>
#include <span>
#include <stdexcept>
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

#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>
#include <gnuradio-4.0/algorithm/math/BitPacking.hpp>

#include <gnuradio-4.0/digital/HeaderLayout.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::LengthHeaderFramer)

struct LengthHeaderFramer : Block<LengthHeaderFramer> {
    using Description = Doc<R""(
@brief Prefixes each record with a sync word and a length header a `PacketFramer` can frame from.

The transmit side of `PacketFramer`, over the same header layouts and the same settings in the same spellings, so the
two ends of a chain are one configuration written twice rather than two that have to be kept in step. One record in,
one framed record out:

    | sync_word (optional) | header items | payload items |

The emitted items are bit items, because a length header is a bit field: `payload_pack_bits` of 8 unpacks a
byte-carrying record into them first, and 0 takes the record's items as wire items already. The length the header
carries is the wire payload count, after any unpacking, inverted through the same `count_covers` arithmetic the
receiving end reads it with -- a 34-byte record at `payload_pack_bits` 8, `count_covers` 'payload', `count_unit_items`
8 and `check_items` 16 writes the field value 32 and puts 272 items on the wire.

`sync_word` has no default and empty means none is emitted, which is the right shape for a chain whose preamble is
generated upstream; there is no universal sync word and a silent one would be a silent interoperability assumption.
`max_payload_items` is required too, and it is the number the receiving `PacketFramer` refuses above, which is what
makes it the number this end must not send.

Three counted drops, each publishing nothing and each named at `stop()`: a record that is empty or above the bound, a
payload whose item count does not invert to a whole field value the layout can carry, and a metadata value the layout
read and rejected -- a `packet_number` or `header_flags` too wide for its field. Metadata crosses verbatim and the
record gains `header_items`, the count the header occupied, so a loopback chain can be read frame by frame.

`sync_word` is written as '0'/'1' characters ahead of the header. `fixed_payload_items` applies under fixed_length,
`byte_order` under length_plain and `flags_position` under length_golay24; the length formats need `length_bits`.
`check_items` counts uncounted trailing check items when count_covers is payload, and `frame_prefix_items` the counted
prefix items when it is whole_frame. Records longer than `max_payload_items` are dropped. A nonzero `payload_pack_bits`
unpacks that many bits per record item.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<std::string, Doc<"sync word bits (empty = omitted)">, Visible>                      sync_word{};
        Annotated<HeaderLayout, Doc<"header layout">, Visible>                                        header_format       = HeaderLayout::length_crc;
        Annotated<gr::Size_t, Unit<"items">, Doc<"fixed payload length">>                             fixed_payload_items = 0U;
        Annotated<gr::Size_t, Unit<"bit">, Doc<"length field width">>                                 length_bits         = 0U;
        Annotated<gr::digital::HeaderByteOrder, Doc<"length field byte order">>                       byte_order          = gr::digital::HeaderByteOrder::Big;
        Annotated<gr::digital::FlagsPosition, Doc<"Golay flags position">>                            flags_position      = gr::digital::FlagsPosition::High;
        Annotated<gr::digital::CountCovers, Doc<"length field coverage">, Visible>                    count_covers        = gr::digital::CountCovers::Unset;
        Annotated<gr::Size_t, Unit<"items">, Doc<"items per length unit">, Limits<Min<1U>, Max<64U>>> count_unit_items    = 8U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"trailing check length">>                            check_items         = 0U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"counted prefix length (0 = header)">>               frame_prefix_items  = 0U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"payload length limit">, Visible>                    max_payload_items   = 0U;
        Annotated<gr::Size_t, Doc<"payload pack width (0 = unpacked)">, Limits<Max<8U>>>              payload_pack_bits   = 0U;
        Annotated<gr::math::BitOrder, Doc<"payload bit order">>                                       payload_bit_order   = gr::math::BitOrder::MsbFirst;

        GR_MAKE_REFLECTABLE(Settings, sync_word, header_format, fixed_payload_items, length_bits, byte_order, flags_position, count_covers, count_unit_items, check_items, frame_prefix_items, max_payload_items, payload_pack_bits, payload_bit_order);
    } settings;

    GR_MAKE_REFLECTABLE(LengthHeaderFramer, in, out, settings);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords               = 0ULL; ///< records framed and published; the denominator the others are read against
    std::uint64_t nItemsEmitted          = 0ULL; ///< wire items published, sync word and header included
    std::uint64_t nRecordsRefused        = 0ULL; ///< records dropped as empty or longer than the bound
    std::uint64_t nLengthUnrepresentable = 0ULL; ///< payloads whose item count does not invert to a whole field value in range
    std::uint64_t nMetaRefused           = 0ULL; ///< records carrying a metadata value the layout read and rejected

    gr::digital::HeaderFormat _format{gr::digital::LengthCrcHeader{}};
    std::size_t               _headerItems = 0UZ;
    std::vector<std::uint8_t> _sync{};
    gr::math::BitRepack       _unpack{};
    std::vector<std::uint8_t> _frame{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::string word = s.sync_word;
        if (word.size() > 64UZ) {
            return gr::Refused(s.sync_word, "must be <= 64 bits");
        }
        if (std::ranges::any_of(word, [](char c) { return c != '0' && c != '1'; })) {
            return gr::Refused(s.sync_word, "must contain only 0s and 1s");
        }
        if (s.max_payload_items == 0U) {
            return gr::Refused(s.max_payload_items, "required");
        }
        return detail::checkHeaderLayout(s);
    }

    void applySettings(const Settings& s) {
        _sync.clear();
        for (const char character : s.sync_word.value) {
            _sync.push_back(static_cast<std::uint8_t>(character == '1' ? 1U : 0U));
        }
        if (s.payload_pack_bits != 0U) {
            gr::math::configureRepack(_unpack, s.payload_pack_bits, 1U, s.payload_bit_order, gr::math::BitOrder::MsbFirst);
        }
        _format      = detail::headerLayoutOf(s);
        _headerItems = gr::digital::headerItemsOf(_format);

        _frame.clear();
        _frame.reserve(_sync.size() + _headerItems + s.max_payload_items);
        clearCounts();
    }

    void start() { clearCounts(); }

    /// @brief Zeroes the counts stop() reports, for a new layout and for a restart.
    void clearCounts() {
        nRecords               = 0ULL;
        nItemsEmitted          = 0ULL;
        nRecordsRefused        = 0ULL;
        nLengthUnrepresentable = 0ULL;
        nMetaRefused           = 0ULL;
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
        append("records refused", nRecordsRefused);
        append("lengths unrepresentable", nLengthUnrepresentable);
        append("metadata refused", nMetaRefused);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::LengthHeaderFramer '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool        connected = outSpan.isConnected; // read once: the record count below is derived from it
        const std::size_t available = inSpan.size();
        const std::size_t nInput    = connected ? std::min(available, outSpan.size()) : available;
        const std::size_t packBits  = settings.payload_pack_bits;
        std::size_t       published = 0UZ;

        for (std::size_t i = 0UZ; i < nInput; ++i) {
            const DataSet<std::uint8_t>&        record  = inSpan[i];
            const std::span<const std::uint8_t> payload = payloadOf(record);

            // the wire count is computed before anything is resized, so nothing an upstream record supplied reaches a
            // resize unchecked
            const std::size_t wireItems = packBits == 0UZ ? payload.size() : payload.size() * packBits;
            if (wireItems == 0UZ || wireItems > static_cast<std::size_t>(settings.max_payload_items.value)) {
                ++nRecordsRefused;
                continue;
            }

            _frame.assign(_sync.begin(), _sync.end());
            _frame.resize(_sync.size() + _headerItems + wireItems, std::uint8_t{0});
            const std::span<std::uint8_t> header = std::span<std::uint8_t>(_frame).subspan(_sync.size(), _headerItems);

            const property_map extra = headerExtraOf(record);
            if (!gr::digital::formatHeader<std::uint8_t>(_format, wireItems, extra, header)) {
                // `formatHeader` reports one failure for two causes, so the cause is attributed by asking it again
                // with nothing staged: what fails without the metadata is the length, and what only fails with it is
                // the metadata. This runs on the refusal path alone.
                ++(gr::digital::formatHeader<std::uint8_t>(_format, wireItems, property_map{}, header) ? nMetaRefused : nLengthUnrepresentable);
                continue;
            }

            const std::span<std::uint8_t> wire = std::span<std::uint8_t>(_frame).subspan(_sync.size() + _headerItems, wireItems);
            if (packBits == 0UZ) {
                std::ranges::copy(payload, wire.begin());
            } else {
                gr::math::repack(_unpack, payload, wire);
            }

            nItemsEmitted += _frame.size();
            ++nRecords;

            DataSet<std::uint8_t> framed = record;
            framed.signal_values.assign(_frame.begin(), _frame.end());
            if (framed.extents.empty()) {
                framed.extents.push_back(static_cast<std::int32_t>(_frame.size()));
            } else {
                framed.extents[0UZ] = static_cast<std::int32_t>(_frame.size());
            }
            if (framed.signal_names.empty()) {
                framed.signal_names.emplace_back("frame");
                framed.signal_quantities.emplace_back("");
                framed.signal_units.emplace_back("");
            }
            // the record's items are wire items now, and an index measured in payload items names none of them
            framed.timing_events.clear();
            framed.timing_events.emplace_back();
            if (framed.meta_information.empty()) {
                framed.meta_information.emplace_back();
            }
            framed.meta_information[0UZ].insert_or_assign(property_map::key_type("header_items"), pmt::Value(static_cast<gr::Size_t>(_headerItems)));

            if (connected && published < outSpan.size()) {
                outSpan[published] = std::move(framed);
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

    /// @brief The keys a header layout reads out of a record's own metadata. A key the selected layout does not read
    /// is ignored by it; one it reads and rejects is the counted refusal above.
    [[nodiscard]] static property_map headerExtraOf(const DataSet<std::uint8_t>& record) {
        property_map extra;
        if (record.meta_information.empty()) {
            return extra;
        }
        const property_map& meta = record.meta_information[0UZ];
        for (const std::string_view name : {"packet_number", "header_flags"}) {
            if (const auto entry = meta.find(property_map::key_type(name)); entry != meta.end()) {
                extra.insert_or_assign(property_map::key_type(name), entry->second);
            }
        }
        return extra;
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

#endif // GNURADIO_DIGITAL_LENGTH_HEADER_FRAMER_HPP
