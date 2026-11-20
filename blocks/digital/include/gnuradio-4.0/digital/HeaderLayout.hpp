#ifndef GNURADIO_DIGITAL_HEADER_LAYOUT_HPP
#define GNURADIO_DIGITAL_HEADER_LAYOUT_HPP

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>

#include <gnuradio-4.0/algorithm/digital/HeaderFormat.hpp>

namespace gr::blocks::digital {

/// @brief The header layout a framer reads or writes, one per `gr::digital::HeaderFormat` alternative.
enum class HeaderLayout : std::uint8_t { length_crc, length_repeated, fixed_length, length_plain, length_golay24 };

} // namespace gr::blocks::digital

namespace gr::blocks::digital::detail {

// The settings a header layout is built from, and the one place that turns them into a `gr::digital::HeaderFormat`.
// A receiving `PacketFramer` and a transmitting `LengthHeaderFramer` must agree on every one of these, or the two ends
// of a chain frame different numbers of items; one builder means they cannot drift. Both blocks declare the same member
// names in their `Settings`, so both functions take that struct.

/**
 * @brief The fault in the header layout settings @p s, under the setting at fault, or nothing when the layout can be
 * built.
 *
 * Every refusal names a setting. A setting that belongs to another layout than the selected one, or to another
 * `count_covers`, and is away from its default is refused: a chain configured for one layout and switched to another
 * says so rather than ignoring half of what it was given. The field widths, the little-endian whole-byte rule and the
 * count rules are the kernel constructors' as well; they are stated here so that each refusal names its setting.
 */
template<typename TSettings>
[[nodiscard]] std::optional<gr::Refused> checkHeaderLayout(const TSettings& s) {
    const HeaderLayout format      = s.header_format.value;
    const bool         lengthField = format == HeaderLayout::length_plain || format == HeaderLayout::length_golay24;

    if (!lengthField) {
        constexpr std::string_view kLengthOnly = "must keep its default unless header_format is length_plain or length_golay24";
        if (s.length_bits.value != 0U) {
            return gr::Refused(s.length_bits, kLengthOnly);
        }
        if (s.count_covers.value != gr::digital::CountCovers::unset) {
            return gr::Refused(s.count_covers, kLengthOnly);
        }
        if (s.count_unit_items.value != 8U) {
            return gr::Refused(s.count_unit_items, kLengthOnly);
        }
        if (s.check_items.value != 0U) {
            return gr::Refused(s.check_items, kLengthOnly);
        }
        if (s.frame_prefix_items.value != 0U) {
            return gr::Refused(s.frame_prefix_items, kLengthOnly);
        }
    }
    if (format != HeaderLayout::length_plain && s.byte_order.value != gr::digital::HeaderByteOrder::big) {
        return gr::Refused(s.byte_order, "must keep its default unless header_format is length_plain");
    }
    if (format != HeaderLayout::length_golay24 && s.flags_position.value != gr::digital::FlagsPosition::high) {
        return gr::Refused(s.flags_position, "must keep its default unless header_format is length_golay24");
    }
    if (format != HeaderLayout::fixed_length && s.fixed_payload_items.value != 0U) {
        return gr::Refused(s.fixed_payload_items, "must keep its default unless header_format is fixed_length");
    }

    if (format == HeaderLayout::fixed_length && s.fixed_payload_items.value == 0U) {
        return gr::Refused(s.fixed_payload_items, "required");
    }
    if (format == HeaderLayout::fixed_length && s.fixed_payload_items.value > s.max_payload_items.value) {
        return gr::Refused(s.fixed_payload_items, "must be <= max_payload_items");
    }
    if (!lengthField) {
        return {};
    }

    if (s.length_bits.value == 0U) {
        return gr::Refused(s.length_bits, "required");
    }
    if (format == HeaderLayout::length_plain && s.length_bits.value > 32U) {
        return gr::Refused(s.length_bits, "must be <= 32 when header_format is length_plain");
    }
    if (format == HeaderLayout::length_golay24 && s.length_bits.value > 12U) {
        return gr::Refused(s.length_bits, "must be <= 12 when header_format is length_golay24");
    }
    if (format == HeaderLayout::length_plain && s.byte_order.value == gr::digital::HeaderByteOrder::little && s.length_bits.value % 8U != 0U) {
        return gr::Refused(s.byte_order, "little requires a length_bits multiple of 8");
    }
    if (s.count_covers.value == gr::digital::CountCovers::unset) {
        return gr::Refused(s.count_covers, "required");
    }
    if (s.count_covers.value != gr::digital::CountCovers::payload && s.check_items.value != 0U) {
        return gr::Refused(s.check_items, "must be 0 unless count_covers is payload");
    }
    if (s.count_covers.value != gr::digital::CountCovers::whole_frame && s.frame_prefix_items.value != 0U) {
        return gr::Refused(s.frame_prefix_items, "must be 0 unless count_covers is whole_frame");
    }
    return {};
}

/// @brief The layout the settings in @p s name, for settings `checkHeaderLayout` accepted.
template<typename TSettings>
[[nodiscard]] gr::digital::HeaderFormat headerLayoutOf(const TSettings& s) {
    switch (s.header_format.value) {
    case HeaderLayout::length_crc: return gr::digital::LengthCrcHeader{};
    case HeaderLayout::length_repeated: return gr::digital::LengthRepeatedHeader{};
    case HeaderLayout::fixed_length: return gr::digital::FixedLengthHeader{s.fixed_payload_items.value};
    case HeaderLayout::length_plain:
    case HeaderLayout::length_golay24: break;
    }

    gr::digital::LengthCount count;
    count.covers           = s.count_covers.value;
    count.unitItems        = s.count_unit_items.value;
    count.checkItems       = s.check_items.value;
    count.framePrefixItems = s.frame_prefix_items.value;
    if (s.header_format.value == HeaderLayout::length_plain) {
        return gr::digital::LengthPlainHeader{s.length_bits.value, s.byte_order.value, count};
    }
    return gr::digital::LengthGolay24Header{s.length_bits.value, s.flags_position.value, count};
}

} // namespace gr::blocks::digital::detail

#endif // GNURADIO_DIGITAL_HEADER_LAYOUT_HPP
