#ifndef GNURADIO_CCSDS_BLOCKS_TIME_CODES_HPP
#define GNURADIO_CCSDS_BLOCKS_TIME_CODES_HPP

#include <algorithm>
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
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/ccsds/TimeCodes.hpp>
#include <gnuradio-4.0/ccsds/RecordHelpers.hpp>

/**
 * @brief `TimeCodeDecode` and `TimeCodeEncode`, thin adapters over `gr::ccsds`'s time-code kernel, CCSDS 301.0-B-4.
 *
 * Each is a 1:1 record adapter that calls one kernel function and writes one carrier field. The decoded instant
 * goes to `DataSet<T>::timestamp`, the carrier's own `std::int64_t` field, and nowhere else: a `timestamp`
 * metadata key would be a second spelling for a fact the carrier already holds, typed and reflected, with no
 * rule for which wins. Four producer-private keys carry provenance — which code, which time scale, and the
 * resolution and leap-second facts that did not fit in the field — and no vocabulary key is written by either
 * block.
 */
namespace gr::blocks::ccsds {

namespace time_detail {

/// @brief "No epoch was given", chosen outside the axis so that `0` stays a legal custom epoch: it is
/// the Unix epoch itself, and a setting that reads it as absence is a setting one instant cannot say.
inline constexpr std::int64_t kNoEpoch = std::numeric_limits<std::int64_t>::min();

/// @brief The name a synthesized signal takes where the input names none.
inline constexpr std::string_view kSignalName = "time_code";

/// @brief The picoseconds below a nanosecond a producer recorded beside an instant, 0 to 999.
[[nodiscard]] inline std::uint32_t readSubNs(const property_map* meta) noexcept {
    if (meta == nullptr) {
        return 0U;
    }
    const auto entry = meta->find(property_map::key_type("ccsds_time_sub_ns"));
    if (entry == meta->end()) {
        return 0U;
    }
    const gr::Size_t* value = entry->second.get_if<gr::Size_t>();
    // A value of the wrong type, or one outside what a picosecond residue can be, reads as absent.
    return value != nullptr && *value < 1000U ? static_cast<std::uint32_t>(*value) : 0U;
}

[[nodiscard]] inline gr::ccsds::TimeCodeKind kindFromName(std::string_view code) {
    if (code == "cuc") {
        return gr::ccsds::TimeCodeKind::cuc;
    }
    if (code == "cds") {
        return gr::ccsds::TimeCodeKind::cds;
    }
    if (code == "ccs") {
        return gr::ccsds::TimeCodeKind::ccs;
    }
    if (code == "ascii_a") {
        return gr::ccsds::TimeCodeKind::ascii_a;
    }
    return gr::ccsds::TimeCodeKind::ascii_b;
}

[[nodiscard]] inline bool validCode(std::string_view code) { return code == "cuc" || code == "cds" || code == "ccs" || code == "ascii_a" || code == "ascii_b"; }

/// @brief The layout implicit settings build, validated for the code they apply to.
struct LayoutSettings {
    gr::Size_t coarse_octets    = detail::kUnset;
    gr::Size_t fine_octets      = detail::kUnset;
    gr::Size_t day_octets       = detail::kUnset;
    gr::Size_t submilli_octets  = detail::kUnset;
    bool       day_of_year      = false;
    gr::Size_t subsecond_octets = detail::kUnset;
};

[[nodiscard]] inline gr::ccsds::Layout buildLayout(gr::ccsds::TimeCodeKind kind, bool customEpoch, const LayoutSettings& settings) {
    gr::ccsds::Layout layout{};
    layout.kind         = kind;
    layout.custom_epoch = customEpoch;
    switch (kind) {
    case gr::ccsds::TimeCodeKind::cuc:
        layout.coarse_octets  = static_cast<std::uint8_t>(settings.coarse_octets);
        layout.fine_octets    = static_cast<std::uint8_t>(settings.fine_octets);
        layout.t_field_octets = std::size_t{layout.coarse_octets} + std::size_t{layout.fine_octets};
        break;
    case gr::ccsds::TimeCodeKind::cds:
        layout.day_octets      = static_cast<std::uint8_t>(settings.day_octets);
        layout.submilli_octets = static_cast<std::uint8_t>(settings.submilli_octets);
        layout.t_field_octets  = std::size_t{layout.day_octets} + 4UZ + std::size_t{layout.submilli_octets};
        break;
    case gr::ccsds::TimeCodeKind::ccs:
        layout.day_of_year      = settings.day_of_year;
        layout.subsecond_octets = static_cast<std::uint8_t>(settings.subsecond_octets);
        layout.t_field_octets   = 7UZ + std::size_t{layout.subsecond_octets};
        break;
    default: break;
    }
    return layout;
}

} // namespace time_detail

GR_REGISTER_BLOCK(gr::blocks::ccsds::TimeCodeDecode)

/*!
@brief Reads a CCSDS time code out of the record's payload at a stated offset and writes it to `timestamp`.

Payload, not metadata: 133.0-B-2 4.1.4.2.1.5 puts the Time Code Field first in a packet secondary header, so it
sits at a fixed offset in the record `SpacePacketDecode` publishes, and a metadata map is not a carrier for a
variable-length octet string. `require_time = false` (the default) publishes every record regardless of whether
its code decoded, leaving `timestamp` untouched on a failure and counting which named fault it was — one counter
for each refusal the kernel can state, plus a record too short to hold the code and a wire P-field that
contradicts the settings — the rule `P25PayloadDecode` follows for the same shape: data rides, status judges.

Every length is known before the kernel is called, so a record short of `offset + p_field + t_field` is
`nShortRecord` and never reaches a conversion; `nShortField` remains for the kernel's own `short_field`, which
that check leaves unreachable from here.
*/
struct TimeCodeDecode : Block<TimeCodeDecode> {
    using Description = Doc<"Reads a CCSDS time code from the payload at a stated offset into DataSet::timestamp, never into metadata (CCSDS 301.0-B-4)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "code", Doc<"'cuc', 'cds', 'ccs', 'ascii_a' or 'ascii_b'; required">, Visible>                                                                             code{};
    Annotated<std::string, "p_field", Doc<"'explicit' (read from the wire, the default) or 'implicit' (from the layout settings); naming either one is refused for the ASCII codes">> p_field{std::string()};
    Annotated<gr::Size_t, "offset", Doc<"octets into the payload at which the code begins">>                                                                                          offset = 0U;
    Annotated<std::string, "epoch", Doc<"'tai1958' or 'custom'; CUC and CDS only, refused for CCS and ASCII">>                                                                        epoch{std::string("tai1958")};
    Annotated<std::int64_t, "epoch_ns", Doc<"the custom epoch's position on the Unix nanosecond axis, 0 included; required when epoch == 'custom', refused otherwise">>               epoch_ns{time_detail::kNoEpoch};
    Annotated<std::int32_t, "tai_utc_offset_s", Doc<"TAI minus UTC at the instant in question; CUC only, refused for CDS, CCS and ASCII">>                                            tai_utc_offset_s = 0;
    Annotated<bool, "strip", Doc<"remove the code's octets from the published payload">>                                                                                              strip            = false;
    Annotated<bool, "require_time", Doc<"a record whose code fails to decode is a counted drop rather than a counted pass-through">>                                                  require_time     = false;

    Annotated<gr::Size_t, "coarse_octets", Doc<"CUC, implicit p_field only: 1 to 7">>            coarse_octets{detail::kUnset};
    Annotated<gr::Size_t, "fine_octets", Doc<"CUC, implicit p_field only: 0 to 10">>             fine_octets{detail::kUnset};
    Annotated<gr::Size_t, "day_octets", Doc<"CDS, implicit p_field only: 2 or 3">>               day_octets{detail::kUnset};
    Annotated<gr::Size_t, "submillisecond_octets", Doc<"CDS, implicit p_field only: 0, 2 or 4">> submillisecond_octets{detail::kUnset};
    Annotated<bool, "day_of_year", Doc<"CCS, implicit p_field only">>                            day_of_year = false;
    Annotated<gr::Size_t, "subsecond_octets", Doc<"CCS, implicit p_field only: 0 to 6">>         subsecond_octets{detail::kUnset};

    GR_MAKE_REFLECTABLE(TimeCodeDecode, in, out, code, p_field, offset, epoch, epoch_ns, tai_utc_offset_s, strip, require_time, coarse_octets, fine_octets, day_octets, submillisecond_octets, day_of_year, subsecond_octets);

    std::uint64_t nDecoded                = 0ULL;
    std::uint64_t nShortRecord            = 0ULL;
    std::uint64_t nShortField             = 0ULL;
    std::uint64_t nBadPField              = 0ULL;
    std::uint64_t nReservedCode           = 0ULL;
    std::uint64_t nNoEpochDefined         = 0ULL;
    std::uint64_t nReservedResolution     = 0ULL;
    std::uint64_t nBadBcd                 = 0ULL;
    std::uint64_t nBadCharacter           = 0ULL;
    std::uint64_t nIncompleteCalendar     = 0ULL;
    std::uint64_t nSegmentOutOfRange      = 0ULL;
    std::uint64_t nAxisOverflow           = 0ULL;
    std::uint64_t nPFieldMismatch         = 0ULL;
    std::uint64_t nLeapSecondValues       = 0ULL;
    std::uint64_t nSubNanosecondTruncated = 0ULL;
    std::uint64_t nPrecisionDiscarded     = 0ULL;

    bool                    _configured = false;
    gr::ccsds::TimeCodeKind _kind       = gr::ccsds::TimeCodeKind::cuc;
    bool                    _isAscii    = false;
    bool                    _implicit   = false;
    gr::ccsds::Layout       _layout{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (!time_detail::validCode(code.value)) {
            throw gr::exception(std::format("code must be 'cuc', 'cds', 'ccs', 'ascii_a' or 'ascii_b' and has no default, got '{}'", code.value));
        }
        if (!p_field.value.empty() && p_field.value != "explicit" && p_field.value != "implicit") {
            throw gr::exception(std::format("p_field must be 'explicit' or 'implicit', got '{}'", p_field.value));
        }
        if (epoch.value != "tai1958" && epoch.value != "custom") {
            throw gr::exception(std::format("epoch must be 'tai1958' or 'custom', got '{}'", epoch.value));
        }

        _kind     = time_detail::kindFromName(code.value);
        _isAscii  = _kind == gr::ccsds::TimeCodeKind::ascii_a || _kind == gr::ccsds::TimeCodeKind::ascii_b;
        _implicit = p_field.value == "implicit";

        // The empty string is the setting's own sentinel for "never staged", distinct from both legal
        // names, the same device `epoch_ns` uses against the nanosecond axis: 3.5.2 gives the ASCII
        // codes no P-field at all, so naming one -- even the harmless default's name -- is refused,
        // while a decode that never touched the setting reads unset and proceeds.
        if (_isAscii && !p_field.value.empty()) {
            throw gr::exception("p_field is refused for the ASCII codes: 3.5.2 gives them no P-field to name, implicit or explicit");
        }
        // CUC and CDS count from an epoch; CCS and ASCII carry a calendar date and have none to name.
        const bool hasEpoch = _kind == gr::ccsds::TimeCodeKind::cuc || _kind == gr::ccsds::TimeCodeKind::cds;
        if (!hasEpoch && epoch.value != "tai1958") {
            throw gr::exception(std::format("epoch is refused for code = '{}': a calendar code carries its own date and counts from no epoch", code.value));
        }
        if (!hasEpoch && epoch_ns.value != time_detail::kNoEpoch) {
            throw gr::exception(std::format("epoch_ns is refused for code = '{}': a calendar code counts from no epoch", code.value));
        }
        if (hasEpoch && epoch.value == "custom" && epoch_ns.value == time_detail::kNoEpoch) {
            throw gr::exception("epoch_ns is required when epoch == 'custom' and has no default");
        }
        if (epoch.value != "custom" && epoch_ns.value != time_detail::kNoEpoch) {
            throw gr::exception("epoch_ns is refused when epoch != 'custom' rather than ignored");
        }
        if (_kind != gr::ccsds::TimeCodeKind::cuc && tai_utc_offset_s.value != 0) {
            throw gr::exception(std::format("tai_utc_offset_s is refused for code = '{}': only CUC is TAI-based", code.value));
        }

        const bool wantsCuc = _implicit && _kind == gr::ccsds::TimeCodeKind::cuc;
        const bool wantsCds = _implicit && _kind == gr::ccsds::TimeCodeKind::cds;
        const bool wantsCcs = _implicit && _kind == gr::ccsds::TimeCodeKind::ccs;
        if (!wantsCuc && (coarse_octets.value != detail::kUnset || fine_octets.value != detail::kUnset)) {
            throw gr::exception("coarse_octets and fine_octets apply only when p_field == 'implicit' and code == 'cuc'");
        }
        if (!wantsCds && (day_octets.value != detail::kUnset || submillisecond_octets.value != detail::kUnset)) {
            throw gr::exception("day_octets and submillisecond_octets apply only when p_field == 'implicit' and code == 'cds'");
        }
        if (!wantsCcs && subsecond_octets.value != detail::kUnset) {
            throw gr::exception("subsecond_octets applies only when p_field == 'implicit' and code == 'ccs'");
        }
        if (!wantsCcs && day_of_year.value) {
            throw gr::exception("day_of_year applies only when p_field == 'implicit' and code == 'ccs'; in explicit mode the P-field's own bit names the variation");
        }

        time_detail::LayoutSettings settings{};
        if (wantsCuc) {
            if (coarse_octets.value == detail::kUnset || coarse_octets.value < 1U || coarse_octets.value > 7U) {
                throw gr::exception("coarse_octets is required and must be 1 to 7 when p_field == 'implicit' and code == 'cuc'");
            }
            if (fine_octets.value == detail::kUnset || fine_octets.value > 10U) {
                throw gr::exception("fine_octets is required and must be 0 to 10 when p_field == 'implicit' and code == 'cuc'");
            }
            settings.coarse_octets = coarse_octets.value;
            settings.fine_octets   = fine_octets.value;
        }
        if (wantsCds) {
            if (day_octets.value == detail::kUnset || (day_octets.value != 2U && day_octets.value != 3U)) {
                throw gr::exception("day_octets is required and must be 2 or 3 when p_field == 'implicit' and code == 'cds'");
            }
            if (submillisecond_octets.value == detail::kUnset || (submillisecond_octets.value != 0U && submillisecond_octets.value != 2U && submillisecond_octets.value != 4U)) {
                throw gr::exception("submillisecond_octets is required and must be 0, 2 or 4 when p_field == 'implicit' and code == 'cds'");
            }
            settings.day_octets      = day_octets.value;
            settings.submilli_octets = submillisecond_octets.value;
        }
        if (wantsCcs) {
            if (subsecond_octets.value == detail::kUnset || subsecond_octets.value > 6U) {
                throw gr::exception("subsecond_octets is required and must be 0 to 6 when p_field == 'implicit' and code == 'ccs'");
            }
            settings.subsecond_octets = subsecond_octets.value;
            settings.day_of_year      = day_of_year.value;
        }
        if (_implicit && !_isAscii) {
            _layout = time_detail::buildLayout(_kind, epoch.value == "custom", settings);
        }
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "TimeCodeDecode", {{"decoded", nDecoded}, {"short record", nShortRecord}, {"short field", nShortField}, {"bad p-field", nBadPField}, {"reserved code", nReservedCode}, {"no epoch defined", nNoEpochDefined}, {"reserved resolution", nReservedResolution}, {"bad bcd", nBadBcd}, {"bad character", nBadCharacter}, {"incomplete calendar", nIncompleteCalendar}, {"segment out of range", nSegmentOutOfRange}, {"axis overflow", nAxisOverflow}, {"p-field mismatch", nPFieldMismatch}, {"leap second values", nLeapSecondValues}, {"sub-nanosecond truncated", nSubNanosecondTruncated}, {"precision discarded", nPrecisionDiscarded}}); }

    void countStatus(gr::ccsds::TimeStatus status) {
        switch (status) {
        case gr::ccsds::TimeStatus::short_field: ++nShortField; break;
        case gr::ccsds::TimeStatus::bad_p_field: ++nBadPField; break;
        case gr::ccsds::TimeStatus::reserved_code: ++nReservedCode; break;
        case gr::ccsds::TimeStatus::no_epoch_defined: ++nNoEpochDefined; break;
        case gr::ccsds::TimeStatus::reserved_resolution: ++nReservedResolution; break;
        case gr::ccsds::TimeStatus::bad_bcd: ++nBadBcd; break;
        case gr::ccsds::TimeStatus::bad_character: ++nBadCharacter; break;
        case gr::ccsds::TimeStatus::incomplete_calendar: ++nIncompleteCalendar; break;
        case gr::ccsds::TimeStatus::segment_out_of_range: ++nSegmentOutOfRange; break;
        case gr::ccsds::TimeStatus::axis_overflow: ++nAxisOverflow; break;
        default: break;
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>&        record = inSpan[consumed];
            const std::span<const std::uint8_t> bytes(record.signal_values);

            if (bytes.size() < std::size_t{offset.value}) {
                ++nShortRecord;
                if (require_time.value) {
                    continue;
                }
                DataSet<std::uint8_t> passthrough = record;
                outSpan[made]                     = std::move(passthrough);
                ++made;
                continue;
            }

            const std::span<const std::uint8_t> fromOffset     = bytes.subspan(offset.value);
            gr::ccsds::Layout                   layout         = _layout;
            std::size_t                         consumedOctets = 0UZ;
            gr::ccsds::TimeStatus               status         = gr::ccsds::TimeStatus::ok;
            gr::ccsds::Instant                  instant{};

            if (_isAscii) {
                std::string_view  text(reinterpret_cast<const char*>(fromOffset.data()), fromOffset.size());
                const std::size_t nul = text.find('\0');
                if (nul != std::string_view::npos) {
                    text = text.substr(0UZ, nul);
                }
                status = gr::ccsds::decodeAscii(text, _kind, instant);
                // The code ends at the NUL that closes it, or at the end of the payload where there is
                // none; a NUL belongs to the code it terminates, so `strip` removes it with the code.
                consumedOctets = nul != std::string_view::npos ? nul + 1UZ : text.size();
            } else {
                if (!_implicit) {
                    status = gr::ccsds::parsePField(fromOffset, layout);
                    if (status == gr::ccsds::TimeStatus::short_field) {
                        // The payload does not reach the end of the P-field, so the record is short of
                        // the code and nothing has been parsed out of it yet.
                        ++nShortRecord;
                        if (require_time.value) {
                            continue;
                        }
                        outSpan[made] = record;
                        ++made;
                        continue;
                    }
                    if (status != gr::ccsds::TimeStatus::ok) {
                        countStatus(status);
                        if (require_time.value) {
                            continue;
                        }
                        outSpan[made] = record;
                        ++made;
                        continue;
                    }
                    // The wire declares which code it is and which epoch it counts from, and the
                    // settings declare the same two facts. Where they disagree the record is refused
                    // rather than decoded under one of them: decoding under the wire's would make
                    // `ccsds_time_code` name a code the instant did not come out of, and decoding under
                    // the setting's would read a field of one width as a field of another.
                    if (layout.kind != _kind || layout.custom_epoch != (epoch.value == "custom")) {
                        ++nPFieldMismatch;
                        if (require_time.value) {
                            continue;
                        }
                        outSpan[made] = record;
                        ++made;
                        continue;
                    }
                }
                const std::size_t needed = layout.p_field_octets + gr::ccsds::timeFieldOctets(layout);
                if (fromOffset.size() < needed) {
                    ++nShortRecord;
                    if (require_time.value) {
                        continue;
                    }
                    outSpan[made] = record;
                    ++made;
                    continue;
                }
                const std::int64_t epochNs = epoch.value == "custom" ? epoch_ns.value : 0;
                status                     = gr::ccsds::decode(fromOffset.subspan(layout.p_field_octets), layout, epochNs, tai_utc_offset_s.value, instant);
                consumedOctets             = needed;
            }

            if (status != gr::ccsds::TimeStatus::ok) {
                countStatus(status);
                if (require_time.value) {
                    continue;
                }
                DataSet<std::uint8_t> passthrough = record;
                outSpan[made]                     = std::move(passthrough);
                ++made;
                continue;
            }

            DataSet<std::uint8_t> decoded;
            if (strip.value) {
                decoded.signal_values.assign(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(offset.value));
                decoded.signal_values.insert(decoded.signal_values.end(), bytes.begin() + static_cast<std::ptrdiff_t>(offset.value + consumedOctets), bytes.end());
            } else {
                decoded.signal_values.assign(bytes.begin(), bytes.end());
            }
            detail::startRecord(record, decoded, time_detail::kSignalName);
            decoded.timestamp = instant.ns;
            property_map& map = decoded.meta_information[0UZ];
            // The configured code, which the P-field refusal above makes truthful in explicit mode too.
            map.insert_or_assign(property_map::key_type("ccsds_time_code"), pmt::Value(std::string(code.value)));
            map.insert_or_assign(property_map::key_type("ccsds_time_scale"), pmt::Value(std::string(instant.tai ? "tai" : "utc")));
            if (instant.sub_ns_ps != 0U) {
                map.insert_or_assign(property_map::key_type("ccsds_time_sub_ns"), pmt::Value(gr::Size_t{instant.sub_ns_ps}));
                ++nSubNanosecondTruncated;
            }
            if (instant.leap) {
                map.insert_or_assign(property_map::key_type("ccsds_time_leap_second"), pmt::Value(true));
                ++nLeapSecondValues;
            }
            if (instant.precision_discarded_digits != 0U) {
                // A producer emitting more digits than the axis and its picosecond residue together
                // hold is telling a consumer something neither can carry, and the count says so.
                ++nPrecisionDiscarded;
            }

            ++nDecoded;
            outSpan[made] = std::move(decoded);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::TimeCodeEncode)

/*!
@brief Reads `in.timestamp` and writes a CCSDS time code into the payload at a stated offset.

A record whose `timestamp` is `0` is not special-cased: zero is the Unix epoch, a legal instant, and treating it
as "unset" would silently drop legitimate records at exactly that value. A graph that wants to encode only where
a time is known filters on something that says so, not on a sentinel that a legitimate instant can equal.
*/
struct TimeCodeEncode : Block<TimeCodeEncode> {
    using Description = Doc<"Writes DataSet::timestamp into the payload as a CCSDS time code at a stated offset (CCSDS 301.0-B-4)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "code", Doc<"'cuc', 'cds', 'ccs', 'ascii_a' or 'ascii_b'; required">, Visible>                                                               code{};
    Annotated<std::string, "p_field", Doc<"'explicit' emits the preamble, 'implicit' does not; both need the layout settings below">>                                   p_field{std::string("explicit")};
    Annotated<gr::Size_t, "offset", Doc<"octets into the payload at which the code is written">>                                                                        offset = 0U;
    Annotated<bool, "insert", Doc<"insert the code's octets at offset, growing the record; false overwrites in place">>                                                 insert = true;
    Annotated<std::string, "epoch", Doc<"'tai1958' or 'custom'; CUC and CDS only, refused for CCS and ASCII">>                                                          epoch{std::string("tai1958")};
    Annotated<std::int64_t, "epoch_ns", Doc<"the custom epoch's position on the Unix nanosecond axis, 0 included; required when epoch == 'custom', refused otherwise">> epoch_ns{time_detail::kNoEpoch};
    Annotated<std::int32_t, "tai_utc_offset_s", Doc<"TAI minus UTC at the instant in question; CUC only, refused for CDS, CCS and ASCII">>                              tai_utc_offset_s = 0;
    Annotated<gr::Size_t, "fraction_digits", Doc<"ASCII only: 0 to 9 digits of fractional seconds; refused for the binary codes">>                                      fraction_digits  = 6U;
    Annotated<bool, "terminator", Doc<"ASCII only: emit the optional trailing 'Z'; refused for the binary codes">>                                                      terminator       = true;

    Annotated<gr::Size_t, "coarse_octets", Doc<"CUC: 1 to 7">>            coarse_octets{detail::kUnset};
    Annotated<gr::Size_t, "fine_octets", Doc<"CUC: 0 to 10">>             fine_octets{detail::kUnset};
    Annotated<gr::Size_t, "day_octets", Doc<"CDS: 2 or 3">>               day_octets{detail::kUnset};
    Annotated<gr::Size_t, "submillisecond_octets", Doc<"CDS: 0, 2 or 4">> submillisecond_octets{detail::kUnset};
    Annotated<bool, "day_of_year", Doc<"CCS">>                            day_of_year = false;
    Annotated<gr::Size_t, "subsecond_octets", Doc<"CCS: 0 to 6">>         subsecond_octets{detail::kUnset};

    GR_MAKE_REFLECTABLE(TimeCodeEncode, in, out, code, p_field, offset, insert, epoch, epoch_ns, tai_utc_offset_s, fraction_digits, terminator, coarse_octets, fine_octets, day_octets, submillisecond_octets, day_of_year, subsecond_octets);

    std::uint64_t nEncoded      = 0ULL;
    std::uint64_t nAxisOverflow = 0ULL;
    std::uint64_t nShortRecord  = 0ULL;

    bool                    _configured = false;
    gr::ccsds::TimeCodeKind _kind       = gr::ccsds::TimeCodeKind::cuc;
    bool                    _isAscii    = false;
    bool                    _explicit   = true;
    gr::ccsds::Layout       _layout{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (!time_detail::validCode(code.value)) {
            throw gr::exception(std::format("code must be 'cuc', 'cds', 'ccs', 'ascii_a' or 'ascii_b' and has no default, got '{}'", code.value));
        }
        if (p_field.value != "explicit" && p_field.value != "implicit") {
            throw gr::exception(std::format("p_field must be 'explicit' or 'implicit', got '{}'", p_field.value));
        }
        if (epoch.value != "tai1958" && epoch.value != "custom") {
            throw gr::exception(std::format("epoch must be 'tai1958' or 'custom', got '{}'", epoch.value));
        }
        _kind     = time_detail::kindFromName(code.value);
        _isAscii  = _kind == gr::ccsds::TimeCodeKind::ascii_a || _kind == gr::ccsds::TimeCodeKind::ascii_b;
        _explicit = p_field.value == "explicit";

        if (_isAscii && _explicit == false) {
            throw gr::exception("p_field must be 'explicit' for the ASCII codes: 3.5.2 gives them no P-field");
        }
        // CUC and CDS count from an epoch; CCS and ASCII carry a calendar date and have none to name.
        const bool hasEpoch = _kind == gr::ccsds::TimeCodeKind::cuc || _kind == gr::ccsds::TimeCodeKind::cds;
        if (!hasEpoch && epoch.value != "tai1958") {
            throw gr::exception(std::format("epoch is refused for code = '{}': a calendar code carries its own date and counts from no epoch", code.value));
        }
        if (!hasEpoch && epoch_ns.value != time_detail::kNoEpoch) {
            throw gr::exception(std::format("epoch_ns is refused for code = '{}': a calendar code counts from no epoch", code.value));
        }
        if (hasEpoch && epoch.value == "custom" && epoch_ns.value == time_detail::kNoEpoch) {
            throw gr::exception("epoch_ns is required when epoch == 'custom' and has no default");
        }
        if (epoch.value != "custom" && epoch_ns.value != time_detail::kNoEpoch) {
            throw gr::exception("epoch_ns is refused when epoch != 'custom' rather than ignored");
        }
        if (_kind != gr::ccsds::TimeCodeKind::cuc && tai_utc_offset_s.value != 0) {
            throw gr::exception(std::format("tai_utc_offset_s is refused for code = '{}': only CUC is TAI-based", code.value));
        }
        if (_isAscii && fraction_digits.value > 9U) {
            throw gr::exception("fraction_digits must be 0 to 9");
        }
        if (!_isAscii && fraction_digits.value != 6U) {
            throw gr::exception(std::format("fraction_digits is refused for code = '{}': only the ASCII codes write a decimal fraction", code.value));
        }
        if (!_isAscii && !terminator.value) {
            throw gr::exception(std::format("terminator is refused for code = '{}': only the ASCII codes have the optional trailing 'Z'", code.value));
        }

        time_detail::LayoutSettings settings{};
        if (_kind == gr::ccsds::TimeCodeKind::cuc) {
            if (coarse_octets.value == detail::kUnset || coarse_octets.value < 1U || coarse_octets.value > 7U) {
                throw gr::exception("coarse_octets is required and must be 1 to 7 for code == 'cuc'");
            }
            if (fine_octets.value == detail::kUnset || fine_octets.value > 10U) {
                throw gr::exception("fine_octets is required and must be 0 to 10 for code == 'cuc'");
            }
            settings.coarse_octets = coarse_octets.value;
            settings.fine_octets   = fine_octets.value;
        } else if (coarse_octets.value != detail::kUnset || fine_octets.value != detail::kUnset) {
            throw gr::exception("coarse_octets and fine_octets apply only to code == 'cuc'");
        }
        if (_kind == gr::ccsds::TimeCodeKind::cds) {
            if (day_octets.value == detail::kUnset || (day_octets.value != 2U && day_octets.value != 3U)) {
                throw gr::exception("day_octets is required and must be 2 or 3 for code == 'cds'");
            }
            if (submillisecond_octets.value == detail::kUnset || (submillisecond_octets.value != 0U && submillisecond_octets.value != 2U && submillisecond_octets.value != 4U)) {
                throw gr::exception("submillisecond_octets is required and must be 0, 2 or 4 for code == 'cds'");
            }
            settings.day_octets      = day_octets.value;
            settings.submilli_octets = submillisecond_octets.value;
        } else if (day_octets.value != detail::kUnset || submillisecond_octets.value != detail::kUnset) {
            throw gr::exception("day_octets and submillisecond_octets apply only to code == 'cds'");
        }
        if (_kind == gr::ccsds::TimeCodeKind::ccs) {
            if (subsecond_octets.value == detail::kUnset || subsecond_octets.value > 6U) {
                throw gr::exception("subsecond_octets is required and must be 0 to 6 for code == 'ccs'");
            }
            settings.subsecond_octets = subsecond_octets.value;
            settings.day_of_year      = day_of_year.value;
        } else if (subsecond_octets.value != detail::kUnset) {
            throw gr::exception("subsecond_octets applies only to code == 'ccs'");
        } else if (day_of_year.value) {
            throw gr::exception("day_of_year applies only to code == 'ccs': it names the 3.4.1.2 variation of the calendar code");
        }

        if (!_isAscii) {
            _layout = time_detail::buildLayout(_kind, epoch.value == "custom", settings);
        }
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "TimeCodeEncode", {{"encoded", nEncoded}, {"axis overflow", nAxisOverflow}, {"short record", nShortRecord}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            // The record's own field carries the nanoseconds; the picoseconds under one nanosecond, if
            // a decoder recorded any, are in the metadata beside it, and the layouts that can hold them
            // -- CDS with four submillisecond octets, CCS with five or six -- write them back out.
            const gr::ccsds::Instant instant{.ns = record.timestamp, .sub_ns_ps = time_detail::readSubNs(detail::metaOf(record)), .leap = false, .tai = tai_utc_offset_s.value == 0, .precision_discarded_digits = 0U};
            const std::int64_t       epochNs = epoch.value == "custom" ? epoch_ns.value : 0;

            std::vector<std::uint8_t> code_bytes;
            bool                      ok = false;
            if (_isAscii) {
                const std::size_t           base   = _kind == gr::ccsds::TimeCodeKind::ascii_a ? 19UZ : 17UZ;
                const std::size_t           needed = base + (fraction_digits.value != 0U ? 1UZ + fraction_digits.value : 0UZ) + (terminator.value ? 1UZ : 0UZ);
                std::vector<char>           text(needed, '\0');
                std::size_t                 written = 0UZ;
                const gr::ccsds::TimeStatus status  = gr::ccsds::encodeAscii(instant, _kind, static_cast<std::uint8_t>(fraction_digits.value), terminator.value, std::span<char>(text), written);
                if (status == gr::ccsds::TimeStatus::ok) {
                    code_bytes.assign(text.begin(), text.begin() + static_cast<std::ptrdiff_t>(written));
                    ok = true;
                } else {
                    ++nAxisOverflow;
                }
            } else {
                std::size_t               pFieldOctets = 0UZ;
                std::vector<std::uint8_t> pField(2UZ, std::uint8_t{0U});
                if (_explicit) {
                    static_cast<void>(gr::ccsds::writePField(_layout, std::span<std::uint8_t>(pField), pFieldOctets));
                }
                std::vector<std::uint8_t>   tField(gr::ccsds::timeFieldOctets(_layout), std::uint8_t{0U});
                const gr::ccsds::TimeStatus status = gr::ccsds::encode(instant, _layout, epochNs, tai_utc_offset_s.value, std::span<std::uint8_t>(tField));
                if (status == gr::ccsds::TimeStatus::ok) {
                    code_bytes.assign(pField.begin(), pField.begin() + static_cast<std::ptrdiff_t>(pFieldOctets));
                    code_bytes.insert(code_bytes.end(), tField.begin(), tField.end());
                    ok = true;
                } else {
                    ++nAxisOverflow;
                }
            }
            if (!ok) {
                continue;
            }

            DataSet<std::uint8_t>               encoded;
            const std::span<const std::uint8_t> bytes(record.signal_values);
            if (insert.value) {
                const std::size_t at = std::min(std::size_t{offset.value}, bytes.size());
                encoded.signal_values.assign(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(at));
                encoded.signal_values.insert(encoded.signal_values.end(), code_bytes.begin(), code_bytes.end());
                encoded.signal_values.insert(encoded.signal_values.end(), bytes.begin() + static_cast<std::ptrdiff_t>(at), bytes.end());
            } else {
                if (bytes.size() < offset.value + code_bytes.size()) {
                    ++nShortRecord;
                    continue;
                }
                encoded.signal_values.assign(bytes.begin(), bytes.end());
                std::ranges::copy(code_bytes, encoded.signal_values.begin() + static_cast<std::ptrdiff_t>(offset.value));
            }
            detail::startRecord(record, encoded, time_detail::kSignalName);
            encoded.timestamp = record.timestamp;

            ++nEncoded;
            outSpan[made] = std::move(encoded);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::ccsds

#endif // GNURADIO_CCSDS_BLOCKS_TIME_CODES_HPP
