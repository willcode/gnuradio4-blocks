#ifndef GNURADIO_DIGITAL_PACKET_FRAMER_HPP
#define GNURADIO_DIGITAL_PACKET_FRAMER_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>

#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/HeaderLayout.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::PacketFramer, [T], [ std::uint8_t, float ])

template<BitLike T>
struct PacketFramer : Block<PacketFramer<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Cuts packets out of a triggered stream using a header that carries the length, one `DataSet<T>` per packet.

Three states: Search discards items while watching for a trigger tag whose `gr:trigger_name` matches, Armed collects
`trigger_to_header + headerItems()` items and parses them, Payload accumulates the parsed length and publishes. The
header is parsed in line, so it must be in the same domain as the payload.

`max_payload_items` is required and has no default, and a header claiming zero or more than it is rejected: a payload
length demodulated from a noisy header is attacker-controlled, and unbounded it wedges the flowgraph.

A packet is one `DataSet<T>`: `signal_values` are its items, `meta_information[0]` carries the per-packet scalars and
`timing_events[0]` every input tag that fell inside the payload, at its index relative to the first payload item.
Tags in the header and search regions are dropped. `sample_start` is the absolute input offset of the first payload
item, spelled the way a record consumer reads it, so that a gap between two packets is detectable downstream.

`length_plain` is a bare field of `length_bits` bits in `byte_order`, and `length_golay24` is a length and some flags
under the extended Golay(24,12,8) code, so a header that took three bit errors still frames its packet and one that
took four is refused rather than framing garbage; a corrected header carries `header_corrected_errors` and, where the
width leaves any flag bits, `header_flags`. Both need `count_covers`, which starts at `unset` because `payload`,
`payload_and_check` and `whole_frame` give different answers for the same wire bits: reading a `payload` frame as
`payload_and_check` hands the CRC checker a record two bytes short, and reading a `whole_frame` frame as `payload`
over-runs into the next one. A setting belonging to one layout and staged away from its default while another is
selected is a settings error rather than a harmless extra, and says so.

`max_payload_items` is the largest payload a header may claim. `fixed_payload_items` applies under fixed_length,
`byte_order` under length_plain and `flags_position` under length_golay24; the length formats need `length_bits`.
`check_items` counts uncounted trailing check items when count_covers is payload, and `frame_prefix_items` the counted
prefix items when it is whole_frame. `trigger` names the gr:trigger_name that starts a packet.
)"">;

    PortIn<T>                  in;
    PortOut<DataSet<T>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"items">, Doc<"payload length limit">, Visible>                    max_payload_items   = 0U;
        Annotated<HeaderLayout, Doc<"header layout">, Visible>                                        header_format       = HeaderLayout::length_crc;
        Annotated<gr::Size_t, Unit<"items">, Doc<"fixed payload length">>                             fixed_payload_items = 0U;
        Annotated<gr::Size_t, Unit<"bit">, Doc<"length field width">>                                 length_bits         = 0U;
        Annotated<gr::digital::HeaderByteOrder, Doc<"length field byte order">>                       byte_order          = gr::digital::HeaderByteOrder::big;
        Annotated<gr::digital::FlagsPosition, Doc<"Golay flags position">>                            flags_position      = gr::digital::FlagsPosition::high;
        Annotated<gr::digital::CountCovers, Doc<"length field coverage">, Visible>                    count_covers        = gr::digital::CountCovers::unset;
        Annotated<gr::Size_t, Unit<"items">, Doc<"items per length unit">, Limits<Min<1U>, Max<64U>>> count_unit_items    = 8U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"trailing check length">>                            check_items         = 0U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"counted prefix length (0 = header)">>               frame_prefix_items  = 0U;
        Annotated<gr::Size_t, Unit<"items">, Doc<"trigger to header offset">>                         trigger_to_header   = 0U;
        Annotated<std::string, Doc<"packet trigger name (empty = any)">>                              trigger{};

        GR_MAKE_REFLECTABLE(Settings, max_payload_items, header_format, fixed_payload_items, length_bits, byte_order, flags_position, count_covers, count_unit_items, check_items, frame_prefix_items, trigger_to_header, trigger);
    } settings;

    GR_MAKE_REFLECTABLE(PacketFramer, in, out, settings);

    enum class State : std::uint8_t { Search, Armed, Payload };

    gr::digital::HeaderFormat                   _format{gr::digital::LengthCrcHeader{}};
    std::size_t                                 _headerItems = 0UZ;
    State                                       _state       = State::Search;
    DataSet<T>                                  _packet{};
    std::size_t                                 _payloadWanted = 0UZ;
    std::size_t                                 _payloadFilled = 0UZ;
    property_map                                _triggerMeta{};
    std::array<T, gr::digital::kMaxHeaderItems> _header{};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.max_payload_items == 0U) {
            return gr::Refused(s.max_payload_items, "required");
        }
        return detail::checkHeaderLayout(s);
    }

    void applySettings(const Settings& s) {
        _format      = detail::headerLayoutOf(s);
        _headerItems = gr::digital::headerItemsOf(_format);
        reset();
    }

    void reset() {
        _state         = State::Search;
        _payloadWanted = 0UZ;
        _payloadFilled = 0UZ;
        _packet        = DataSet<T>{};
        _triggerMeta.clear();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        struct Arriving {
            std::size_t         at;
            const property_map* map;
        };
        std::vector<Arriving> tags;
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex >= 0) { // a negative index is a tag from an earlier call, presented again
                tags.push_back({static_cast<std::size_t>(relIndex), &tagMap.get()});
            }
        }

        const std::size_t room      = outSpan.isConnected ? outSpan.size() : 0UZ;
        const std::size_t need      = static_cast<std::size_t>(settings.trigger_to_header) + _headerItems;
        std::size_t       consumed  = 0UZ;
        std::size_t       published = 0UZ;
        std::size_t       i         = 0UZ;
        bool              starved   = false;

        while (!starved) {
            if (_state == State::Payload) {
                if (_payloadFilled == _payloadWanted) {
                    if (published >= room) {
                        starved = true;
                        break;
                    }
                    finish();
                    outSpan[published] = std::move(_packet);
                    ++published;
                    reset();
                    continue;
                }
                const std::size_t take = std::min(_payloadWanted - _payloadFilled, inSpan.size() - i);
                if (take == 0UZ) {
                    starved = true;
                    break;
                }
                _packet.signal_values.insert(_packet.signal_values.end(), std::next(inSpan.begin(), static_cast<std::ptrdiff_t>(i)), std::next(inSpan.begin(), static_cast<std::ptrdiff_t>(i + take)));
                for (const Arriving& tag : tags) {
                    if (tag.at >= i && tag.at < i + take && !tag.map->empty()) {
                        _packet.timing_events[0UZ].emplace_back(static_cast<std::ptrdiff_t>(_payloadFilled + tag.at - i), *tag.map);
                    }
                }
                _payloadFilled += take;
                i += take;
                consumed = i;
                continue;
            }

            if (_state == State::Search) {
                const auto found = std::ranges::find_if(tags, [this, i](const Arriving& tag) { return tag.at >= i && matches(*tag.map); });
                if (found == tags.end()) {
                    i        = inSpan.size();
                    consumed = i;
                    starved  = true;
                    break;
                }
                i            = found->at;
                consumed     = i;
                _triggerMeta = *found->map;
                _state       = State::Armed;
                continue;
            }

            // Armed: the trigger item is at `i` and nothing from it on has been consumed, so a rejected header rewinds
            if (inSpan.size() - i < need) {
                starved = true;
                break;
            }
            for (std::size_t k = 0UZ; k < _headerItems; ++k) {
                _header[k] = inSpan[i + static_cast<std::size_t>(settings.trigger_to_header) + k];
            }
            const std::optional<gr::digital::ParsedHeader> parsed = gr::digital::parseHeader<T>(_format, std::span<const T>(_header.data(), _headerItems));
            if (!parsed.has_value() || parsed->payloadItems == 0UZ || parsed->payloadItems > static_cast<std::size_t>(settings.max_payload_items)) {
                ++i; // one item, so a genuine trigger inside a false header is still reachable
                consumed = i;
                _state   = State::Search;
                continue;
            }

            begin(*parsed, inSpan.streamIndex + i + need);
            i += need;
            consumed = i;
            _state   = State::Payload;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(outSpan.isConnected ? published : 0UZ);
        return consumed == 0UZ && published == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::OK;
    }

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

    void begin(const gr::digital::ParsedHeader& parsed, std::size_t payloadStart) {
        _packet = DataSet<T>{};
        _packet.signal_values.reserve(parsed.payloadItems);
        _packet.signal_names.emplace_back("payload");
        _packet.meta_information.emplace_back();
        _packet.timing_events.emplace_back();
        _payloadWanted = parsed.payloadItems;
        _payloadFilled = 0UZ;

        property_map& meta = _packet.meta_information[0UZ];
        for (const auto& [key, value] : parsed.meta) {
            meta.insert_or_assign(key, value);
        }
        meta.insert_or_assign(property_map::key_type(gr::tag::TRIGGER_NAME.name()), triggerName());
        meta.insert_or_assign(property_map::key_type("sync_errors"), syncErrors());
        meta.insert_or_assign(property_map::key_type("header_ok"), pmt::Value(true));
        meta.insert_or_assign(property_map::key_type("sample_start"), pmt::Value(static_cast<std::uint64_t>(payloadStart)));

        if (const auto entry = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_TIME.key())); entry != _triggerMeta.end()) {
            if (const auto* stamp = entry->second.template get_if<std::uint64_t>(); stamp != nullptr) {
                _packet.timestamp = static_cast<std::int64_t>(*stamp);
            }
        }
    }

    void finish() {
        _packet.extents.push_back(static_cast<std::int32_t>(_packet.signal_values.size()));
        _packet.signal_quantities.emplace_back("");
        _packet.signal_units.emplace_back("");
    }

    [[nodiscard]] pmt::Value triggerName() const {
        const auto entry = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_NAME.key()));
        return entry == _triggerMeta.end() ? pmt::Value(std::string_view{}) : entry->second;
    }

    /// @brief The Hamming distance the detector reported, which travels inside `gr:trigger_meta_info`.
    [[nodiscard]] pmt::Value syncErrors() const {
        const auto outer = _triggerMeta.find(property_map::key_type(gr::tag::TRIGGER_META_INFO.key()));
        if (outer != _triggerMeta.end()) {
            if (const auto* nested = outer->second.template get_if<property_map>(); nested != nullptr) {
                if (const auto inner = nested->find(property_map::key_type("sync_errors")); inner != nested->end()) {
                    return inner->second;
                }
            }
        }
        return pmt::Value(gr::Size_t{0});
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_PACKET_FRAMER_HPP
