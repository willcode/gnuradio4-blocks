#ifndef GNURADIO_AX25_AX25_HPP
#define GNURADIO_AX25_AX25_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <limits>
#include <optional>
#include <print>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

/**
 * The address, control and protocol-identifier layer of the AX.25 version 2.2 frame, as a pair of
 * record adapters over `DataSet<std::uint8_t>`.
 *
 * A frame reaches `Ax25Decode` after the flags and the bit stuffing have been taken off by
 * `DelimiterExtractor` and the frame check sequence has been validated and stripped by `CrcCheck`,
 * so what these blocks see is an address field of two to ten seven-byte subfields, one control
 * byte, a protocol identifier on the two frame types that carry one, and the information field.
 * `Ax25Encode` builds exactly that and hands it to `CrcAppend` and `DelimiterFramer`, which put the
 * check sequence and the framing back.
 *
 * The address arithmetic is shifts and masks over a fixed layout, so it lives in `detail` here
 * rather than in the algorithm library: both blocks that need it are in this module and neither
 * kernel would have a second caller.
 *
 * Decode refuses on structure alone and never on content. The low bits of the callsign bytes are
 * not policed and the characters are not held to an alphabet, because the frame's integrity check
 * has already passed upstream and because real traffic puts data there: APRS Mic-E encodes a
 * position in the destination callsign's characters, and a charset rule would drop those frames.
 */
namespace gr::blocks::ax25 {

namespace detail {

constexpr std::size_t kSubfieldBytes = 7UZ;  ///< bytes in one address subfield
constexpr std::size_t kCallsignChars = 6UZ;  ///< callsign characters in a subfield, padded with spaces on the right
constexpr std::size_t kMaxRepeaters  = 8UZ;  ///< repeater subfields a path may name
constexpr std::size_t kMaxSubfields  = 10UZ; ///< destination, source and the repeaters together

constexpr unsigned kHighBit      = 0x80U; ///< the C bit on a destination or source subfield, the H bit on a repeater
constexpr unsigned kReservedBits = 0x60U; ///< the reserved pair, transmitted as 11
constexpr unsigned kSsidMask     = 0x1EU; ///< the SSID's four bits inside the SSID byte
constexpr unsigned kExtensionBit = 0x01U; ///< clear while further subfields follow, set on the last one
constexpr unsigned kPollFinalBit = 0x10U; ///< the poll/final bit of every control byte
constexpr unsigned kMaxSsid      = 15U;
constexpr unsigned kMaxByte      = 255U;

/// @brief The nine named unnumbered modifiers, with the poll/final bit clear.
constexpr std::array<std::pair<unsigned, std::string_view>, 9UZ> kUnnumbered{{{0x03U, "UI"}, {0x0FU, "DM"}, {0x2FU, "SABM"}, {0x43U, "DISC"}, {0x63U, "UA"}, {0x6FU, "SABME"}, {0x87U, "FRMR"}, {0xAFU, "XID"}, {0xE3U, "TEST"}}};

/// @brief One address subfield's text form: the callsign, the SSID and the repeated marker a '*' spells.
struct Address {
    std::string call{};
    unsigned    ssid     = 0U;
    bool        repeated = false;
};

/// @brief What a control byte says about the frame it opens.
struct Control {
    std::string_view type{};         ///< the type name the decoded record carries
    gr::Size_t       masked    = 0U; ///< the control byte with the poll/final bit cleared
    gr::Size_t       nr        = 0U; ///< the receive sequence number, on I and supervisory frames
    gr::Size_t       ns        = 0U; ///< the send sequence number, on I frames
    bool             pollFinal = false;
    bool             named     = true;  ///< false for an unlisted unnumbered modifier, which decodes as "U"
    bool             hasPid    = false; ///< true on the I and UI frames, the two types that carry a protocol identifier
    bool             hasNr     = false;
    bool             hasNs     = false;
};

/// @brief Why a frame could not be parsed, or that it was.
enum class Outcome : std::uint8_t { Ok, ShortFrame, AddressOverrun };

/// @brief One parsed frame, down to the offset its information field starts at.
struct Frame {
    Outcome     outcome = Outcome::ShortFrame;
    std::string destination{};
    std::string source{};
    std::string via{};
    Control     control{};
    gr::Size_t  pid          = 0U;
    std::size_t infoAt       = 0UZ;
    bool        command      = false;
    bool        commandKnown = false; ///< false when both C bits agree, the pre-2.2 convention that says nothing
};

/// @brief Render @p subfield as `CALL`, `CALL-N`, or a repeater's `CALL-N*` when its H bit is set.
[[nodiscard]] inline std::string subfieldText(std::span<const std::uint8_t> subfield, bool repeater) {
    std::string text;
    text.reserve(kCallsignChars + 4UZ);
    for (std::size_t i = 0UZ; i < kCallsignChars; ++i) {
        text.push_back(static_cast<char>(subfield[i] >> 1U));
    }
    while (!text.empty() && text.back() == ' ') {
        text.pop_back();
    }
    if (const unsigned ssid = (subfield[kCallsignChars] & kSsidMask) >> 1U; ssid != 0U) {
        std::format_to(std::back_inserter(text), "-{}", ssid);
    }
    if (repeater && (subfield[kCallsignChars] & kHighBit) != 0U) {
        text.push_back('*');
    }
    return text;
}

/// @brief Read @p text as one address, reporting what the grammar refuses through `std::invalid_argument`.
///
/// A trailing '*' is accepted only on a repeater, where it sets the H bit, so that a chain regenerating frames it
/// heard can reproduce them; a transmitter of new frames writes no '*' and leaves the bit clear.
[[nodiscard]] inline Address addressFromText(std::string_view text, bool repeater) {
    Address          address;
    std::string_view rest = text;
    if (repeater && rest.ends_with('*')) {
        address.repeated = true;
        rest.remove_suffix(1UZ);
    }

    const std::size_t      dash = rest.find('-');
    const std::string_view call = dash == std::string_view::npos ? rest : rest.substr(0UZ, dash);
    if (call.empty() || call.size() > kCallsignChars) {
        throw std::invalid_argument(std::format("a callsign is one to six characters of 'A'-'Z' and '0'-'9', got '{}'", text));
    }
    for (const char character : call) {
        const bool letter = character >= 'A' && character <= 'Z';
        const bool digit  = character >= '0' && character <= '9';
        if (!letter && !digit) {
            throw std::invalid_argument(std::format("a callsign is one to six characters of 'A'-'Z' and '0'-'9', got '{}'", text));
        }
    }

    if (dash != std::string_view::npos) {
        const std::string_view digits = rest.substr(dash + 1UZ);
        if (digits.empty() || digits.size() > 2UZ) {
            throw std::invalid_argument(std::format("an SSID is a one or two digit number in [0, 15], got '{}'", text));
        }
        unsigned ssid = 0U;
        for (const char digit : digits) {
            if (digit < '0' || digit > '9') {
                throw std::invalid_argument(std::format("an SSID is a one or two digit number in [0, 15], got '{}'", text));
            }
            ssid = ssid * 10U + static_cast<unsigned>(digit - '0');
        }
        if (ssid > kMaxSsid) {
            throw std::invalid_argument(std::format("an SSID is a one or two digit number in [0, 15], got '{}'", text));
        }
        address.ssid = ssid;
    }

    address.call.assign(call);
    return address;
}

/// @brief Read @p text as a comma-separated repeater path, refusing a ninth hop.
[[nodiscard]] inline std::vector<Address> viaFromText(std::string_view text) {
    std::vector<Address> hops;
    if (text.empty()) {
        return hops;
    }
    for (std::size_t at = 0UZ;;) {
        const std::size_t      comma = text.find(',', at);
        const std::string_view hop   = comma == std::string_view::npos ? text.substr(at) : text.substr(at, comma - at);
        if (hops.size() == kMaxRepeaters) {
            throw std::invalid_argument(std::format("a path names at most eight repeaters, got '{}'", text));
        }
        hops.push_back(addressFromText(hop, true));
        if (comma == std::string_view::npos) {
            return hops;
        }
        at = comma + 1UZ;
    }
}

/// @brief Append @p address as a subfield, @p highBit carrying the C or H bit and @p last the extension bit.
inline void packAddress(const Address& address, bool highBit, bool last, std::vector<std::uint8_t>& frame) {
    for (std::size_t i = 0UZ; i < kCallsignChars; ++i) {
        const char character = i < address.call.size() ? address.call[i] : ' ';
        frame.push_back(static_cast<std::uint8_t>(static_cast<unsigned>(static_cast<unsigned char>(character)) << 1U));
    }
    frame.push_back(static_cast<std::uint8_t>((highBit ? kHighBit : 0U) | kReservedBits | (address.ssid << 1U) | (last ? kExtensionBit : 0U)));
}

/// @brief Classify @p control by its low bits, modulo 8.
///
/// An unlisted unnumbered modifier is reported as type "U" with the masked byte kept, because an unknown unnumbered
/// frame is still a frame and refusing it would lose traffic the parser has no quarrel with.
[[nodiscard]] inline Control classify(std::uint8_t control) noexcept {
    Control        result;
    const unsigned value = control;
    result.pollFinal     = (value & kPollFinalBit) != 0U;
    result.masked        = value & ~kPollFinalBit;

    if ((value & 0x01U) == 0U) {
        result.type   = "I";
        result.hasPid = true;
        result.hasNr  = true;
        result.hasNs  = true;
        result.ns     = (value >> 1U) & 0x07U;
        result.nr     = (value >> 5U) & 0x07U;
        return result;
    }
    if ((value & 0x03U) == 0x01U) {
        constexpr std::array<std::string_view, 4UZ> kSupervisory{"RR", "RNR", "REJ", "SREJ"};
        result.type  = kSupervisory[(value >> 2U) & 0x03U];
        result.hasNr = true;
        result.nr    = (value >> 5U) & 0x07U;
        return result;
    }
    for (const auto& [modifier, name] : kUnnumbered) {
        if (modifier == result.masked) {
            result.type   = name;
            result.hasPid = name == "UI";
            return result;
        }
    }
    result.type  = "U";
    result.named = false;
    return result;
}

/// @brief The control byte @p name spells with its poll/final bit clear, refusing the types a setting cannot number.
[[nodiscard]] inline std::uint8_t controlFromName(std::string_view name) {
    for (const auto& [modifier, label] : kUnnumbered) {
        if (label == name) {
            return static_cast<std::uint8_t>(modifier);
        }
    }
    throw std::invalid_argument(std::format("must be one of 'UI', 'DM', 'SABM', 'DISC', 'UA', 'SABME', 'FRMR', 'XID' or 'TEST'; the I and supervisory types carry sequence numbers no setting supplies, got '{}'", name));
}

/// @brief Walk @p frame by position: seven bytes a subfield, then the control byte and the identifier behind it.
[[nodiscard]] inline Frame parseFrame(std::span<const std::uint8_t> frame) {
    Frame       parsed;
    std::size_t at         = 0UZ;
    std::size_t subfields  = 0UZ;
    bool        closed     = false;
    bool        destCbit   = false;
    bool        sourceCbit = false;

    while (subfields < kMaxSubfields) {
        if (at + kSubfieldBytes > frame.size()) {
            parsed.outcome = Outcome::ShortFrame;
            return parsed;
        }
        const std::span<const std::uint8_t> subfield = frame.subspan(at, kSubfieldBytes);
        std::string                         text     = subfieldText(subfield, subfields >= 2UZ);
        if (subfields == 0UZ) {
            parsed.destination = std::move(text);
            destCbit           = (subfield[kCallsignChars] & kHighBit) != 0U;
        } else if (subfields == 1UZ) {
            parsed.source = std::move(text);
            sourceCbit    = (subfield[kCallsignChars] & kHighBit) != 0U;
        } else {
            if (!parsed.via.empty()) {
                parsed.via.push_back(',');
            }
            parsed.via += text;
        }
        at += kSubfieldBytes;
        ++subfields;
        if ((subfield[kCallsignChars] & kExtensionBit) != 0U) {
            closed = true;
            break;
        }
    }
    if (!closed) {
        parsed.outcome = Outcome::AddressOverrun;
        return parsed;
    }
    if (subfields < 2UZ || at >= frame.size()) {
        // an address field closing after the destination has no source subfield, and one closing at the record's end
        // has no control byte; both are fewer bytes than the structure requires at the point reached
        parsed.outcome = Outcome::ShortFrame;
        return parsed;
    }

    parsed.control      = classify(frame[at]);
    parsed.command      = destCbit;
    parsed.commandKnown = destCbit != sourceCbit;
    ++at;
    if (parsed.control.hasPid) {
        if (at >= frame.size()) {
            parsed.outcome = Outcome::ShortFrame;
            return parsed;
        }
        parsed.pid = frame[at];
        ++at;
    }
    parsed.infoAt  = at;
    parsed.outcome = Outcome::Ok;
    return parsed;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::ax25::Ax25Decode)

/*!
@brief One validated AX.25 frame in, its information field out with the address and control layer in metadata.

The record's items are the information field, which is often empty: a UA or a DISC frame carries none, and its record
has zero items and its answer entirely in the keys. The input record's metadata crosses verbatim and the block's own
keys are written over it, so what `CrcCheck` and `DelimiterExtractor` reported about the frame survives beside what
the frame says about itself.

`ax25_destination`, `ax25_source` and `ax25_via` are the text forms: `CALL` or `CALL-N`, a repeater that has been
repeated gaining a trailing `*`, and the path comma-joined and empty when there is none. `ax25_type` names the frame,
`ax25_poll_final` carries the poll/final bit, and the sequence numbers, the protocol identifier and the masked control
byte appear on the frame types that have them. `ax25_command` appears only when the two C bits disagree, which is the
version 2.2 encoding; two equal C bits are the older convention that carries no information and the key is omitted
rather than guessed at.

Two structural refusals publish nothing and are counted: a record too short for the structure reached, and an address
field whose tenth subfield still has its extension bit clear. Both are stated at `stop()`, and the record after either
one decodes normally.
*/
struct Ax25Decode : Block<Ax25Decode> {
    using Description = Doc<"AX.25 decode: one FCS-stripped frame per record becomes its information field, with the address, control and PID layer written to metadata">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    GR_MAKE_REFLECTABLE(Ax25Decode, in, out);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nInfoBytes      = 0ULL; ///< information bytes those records carry
    std::uint64_t nRefusedShort   = 0ULL; ///< records with fewer bytes than the structure requires
    std::uint64_t nRefusedAddress = 0ULL; ///< records whose address field had no extension bit within ten subfields

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records", nRecords);
        append("info bytes", nInfoBytes);
        append("short frames", nRefusedShort);
        append("address overruns", nRefusedAddress);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::ax25::Ax25Decode '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>&        record = inSpan[consumed];
            const std::span<const std::uint8_t> bytes(record.signal_values);
            const detail::Frame                 parsed = detail::parseFrame(bytes);
            if (parsed.outcome != detail::Outcome::Ok) {
                ++(parsed.outcome == detail::Outcome::ShortFrame ? nRefusedShort : nRefusedAddress);
                continue;
            }

            DataSet<std::uint8_t> info;
            const auto            payload = bytes.subspan(parsed.infoAt);
            info.signal_values.assign(payload.begin(), payload.end());
            info.extents.push_back(static_cast<std::int32_t>(info.signal_values.size()));
            info.signal_names.emplace_back(record.signal_names.empty() ? std::string("ax25") : record.signal_names[0UZ]);
            info.timing_events.resize(1UZ);
            info.meta_information.resize(1UZ);
            property_map& map = info.meta_information[0UZ];
            if (!record.meta_information.empty()) {
                map = record.meta_information[0UZ]; // the record's facts carry through, this block's keys over them
            }
            map.insert_or_assign(property_map::key_type("ax25_destination"), pmt::Value(parsed.destination));
            map.insert_or_assign(property_map::key_type("ax25_source"), pmt::Value(parsed.source));
            map.insert_or_assign(property_map::key_type("ax25_via"), pmt::Value(parsed.via));
            map.insert_or_assign(property_map::key_type("ax25_type"), pmt::Value(std::string(parsed.control.type)));
            map.insert_or_assign(property_map::key_type("ax25_poll_final"), pmt::Value(parsed.control.pollFinal));
            if (!parsed.control.named) {
                map.insert_or_assign(property_map::key_type("ax25_control"), pmt::Value(parsed.control.masked));
            }
            if (parsed.control.hasPid) {
                map.insert_or_assign(property_map::key_type("ax25_pid"), pmt::Value(parsed.pid));
            }
            if (parsed.control.hasNr) {
                map.insert_or_assign(property_map::key_type("ax25_nr"), pmt::Value(parsed.control.nr));
            }
            if (parsed.control.hasNs) {
                map.insert_or_assign(property_map::key_type("ax25_ns"), pmt::Value(parsed.control.ns));
            }
            if (parsed.commandKnown) {
                map.insert_or_assign(property_map::key_type("ax25_command"), pmt::Value(parsed.command));
            }

            ++nRecords;
            nInfoBytes += info.signal_values.size();
            outSpan[made] = std::move(info);
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

GR_REGISTER_BLOCK(gr::blocks::ax25::Ax25Encode)

/*!
@brief One information field per record in, the assembled AX.25 frame out, ready for `CrcAppend`.

The addresses are settings because a transmitter's own call and its path are properties of the station rather than of
a frame, and they are held to the text grammar `Ax25Decode` renders: one to six characters of `A`-`Z` and `0`-`9`, an
optional `-N` with N in 0 to 15, and on a repeater an optional trailing `*` that sets the H bit. A setting outside
that grammar is refused when it is staged, naming the setting, because a chain built on a callsign nobody can read is
better stopped than started.

A record may override `ax25_destination`, `ax25_source`, `ax25_via`, `ax25_command`, `ax25_poll_final` or `ax25_pid`
for its own frame, which is how a digipeater or a test regenerates frames it read. An override arrives mid-stream
rather than at staging, so one that fails the same grammar is a counted drop naming the key rather than an error that
takes the chain down, and the record after it is built from the settings again.

`control_type` names the frame. The I and supervisory types are refused: they carry sequence numbers that belong to a
connected-mode state machine, and nothing here numbers a frame. The reserved SSID bits go out as `11`, the C bits
follow `command`, and the protocol identifier is emitted for the UI frame alone, the only accepted type that carries
one.
*/
struct Ax25Encode : Block<Ax25Encode> {
    using Description = Doc<"AX.25 encode: a record of information bytes becomes an addressed frame - address field, control byte and PID - for CrcAppend and DelimiterFramer">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "destination", Doc<"the destination address as 'CALL' or 'CALL-N'; required, and empty leaves the block inert">, Visible>    destination{};
    Annotated<std::string, "source", Doc<"the source address as 'CALL' or 'CALL-N'; required, and empty leaves the block inert">, Visible>              source{};
    Annotated<std::string, "via", Doc<"the repeater path, comma separated and at most eight hops, a trailing '*' on a hop setting its H bit">, Visible> via{};
    Annotated<bool, "command", Doc<"true sends the version 2.2 command pair of C bits, destination 1 and source 0; false sends the response pair">>     command      = true;
    Annotated<bool, "poll_final", Doc<"the poll/final bit of the control byte">>                                                                        poll_final   = false;
    Annotated<gr::Size_t, "pid", Doc<"the protocol identifier of a UI frame, 0 to 255; 0xF0 is the no-layer-3 value APRS and plain text use">>          pid          = 0xF0U;
    Annotated<std::string, "control_type", Doc<"'UI', 'DM', 'SABM', 'DISC', 'UA', 'SABME', 'FRMR', 'XID' or 'TEST'">, Visible>                          control_type = std::string("UI");

    GR_MAKE_REFLECTABLE(Ax25Encode, in, out, destination, source, via, command, poll_final, pid, control_type);

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords         = 0ULL; ///< frames published on `out`
    std::uint64_t nFrameBytes      = 0ULL; ///< bytes those frames carry, the check sequence not yet appended
    std::uint64_t nRefusedOverride = 0ULL; ///< records whose metadata carried a value the grammar refuses

    detail::Address              _destination{};
    detail::Address              _source{};
    std::vector<detail::Address> _via{};
    std::uint8_t                 _control    = 0x03U;
    bool                         _hasPid     = true;
    bool                         _configured = false;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        if (destination.value.empty() || source.value.empty()) {
            throw gr::exception("destination and source are required and have no default: a frame with no addresses names neither end of the link");
        }
    }

    /// @brief Rebuilds the frame's fixed parts from the settings, refusing by name what the grammar does not accept.
    void rebuild() {
        try {
            _control = detail::controlFromName(control_type.value);
        } catch (const std::invalid_argument& error) {
            throw gr::exception(std::format("control_type {}", error.what()));
        }
        _hasPid = control_type.value == "UI";
        if (pid.value > detail::kMaxByte) {
            throw gr::exception(std::format("pid is one byte and must be in [0, 255], got {}", pid.value));
        }
        if (!destination.value.empty()) {
            _destination = address(destination.value, false, "destination");
        }
        if (!source.value.empty()) {
            _source = address(source.value, false, "source");
        }
        try {
            _via = detail::viaFromText(via.value);
        } catch (const std::invalid_argument& error) {
            throw gr::exception(std::format("via: {}", error.what()));
        }
        _configured = !destination.value.empty() && !source.value.empty();
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records", nRecords);
        append("frame bytes", nFrameBytes);
        append("overrides refused", nRefusedOverride);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::ax25::Ax25Encode '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than addressing frames to nobody
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const property_map*          meta   = record.meta_information.empty() ? nullptr : &record.meta_information[0UZ];

            detail::Address              theirs    = _destination;
            detail::Address              ours      = _source;
            std::vector<detail::Address> path      = _via;
            bool                         asCommand = command.value;
            bool                         asPoll    = poll_final.value;
            gr::Size_t                   asPid     = pid.value;
            bool                         refused   = false;

            if (const std::optional<std::string> text = string(meta, "ax25_destination"); text.has_value()) {
                refused = !overrideAddress(*text, false, "ax25_destination", theirs);
            }
            if (!refused) {
                if (const std::optional<std::string> text = string(meta, "ax25_source"); text.has_value()) {
                    refused = !overrideAddress(*text, false, "ax25_source", ours);
                }
            }
            if (!refused) {
                if (const std::optional<std::string> text = string(meta, "ax25_via"); text.has_value()) {
                    try {
                        path = detail::viaFromText(*text);
                    } catch (const std::invalid_argument& error) {
                        std::println(stderr, "gr::blocks::ax25::Ax25Encode '{}': dropping a record whose ax25_via override {}", this->name, error.what());
                        refused = true;
                    }
                }
            }
            if (!refused) {
                if (const std::optional<bool> flag = boolean(meta, "ax25_command"); flag.has_value()) {
                    asCommand = *flag;
                }
                if (const std::optional<bool> flag = boolean(meta, "ax25_poll_final"); flag.has_value()) {
                    asPoll = *flag;
                }
                if (const std::optional<gr::Size_t> value = number(meta, "ax25_pid"); value.has_value()) {
                    if (*value > detail::kMaxByte) {
                        std::println(stderr, "gr::blocks::ax25::Ax25Encode '{}': dropping a record whose ax25_pid override is {}, which is not a byte", this->name, *value);
                        refused = true;
                    } else {
                        asPid = *value;
                    }
                }
            }
            if (refused) {
                ++nRefusedOverride;
                continue;
            }

            DataSet<std::uint8_t> frame;
            frame.signal_values.reserve((2UZ + path.size()) * detail::kSubfieldBytes + 2UZ + record.signal_values.size());
            detail::packAddress(theirs, asCommand, false, frame.signal_values);
            detail::packAddress(ours, !asCommand, path.empty(), frame.signal_values);
            for (std::size_t hop = 0UZ; hop < path.size(); ++hop) {
                detail::packAddress(path[hop], path[hop].repeated, hop + 1UZ == path.size(), frame.signal_values);
            }
            frame.signal_values.push_back(static_cast<std::uint8_t>(_control | (asPoll ? detail::kPollFinalBit : 0U)));
            if (_hasPid) {
                frame.signal_values.push_back(static_cast<std::uint8_t>(asPid));
            }
            frame.signal_values.insert(frame.signal_values.end(), record.signal_values.begin(), record.signal_values.end());

            frame.extents.push_back(static_cast<std::int32_t>(frame.signal_values.size()));
            frame.signal_names.emplace_back(record.signal_names.empty() ? std::string("ax25") : record.signal_names[0UZ]);
            frame.timing_events.resize(1UZ);
            frame.meta_information.resize(1UZ);
            if (meta != nullptr) {
                frame.meta_information[0UZ] = *meta; // the record's facts carry through; assembly has nothing to add
            }

            ++nRecords;
            nFrameBytes += frame.signal_values.size();
            outSpan[made] = std::move(frame);
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
    /// @brief One setting's address, with the refusal reported under the setting's own name.
    [[nodiscard]] static detail::Address address(const std::string& text, bool repeater, std::string_view setting) {
        try {
            return detail::addressFromText(text, repeater);
        } catch (const std::invalid_argument& error) {
            throw gr::exception(std::format("{}: {}", setting, error.what()));
        }
    }

    /// @brief One record's address override, stating the key it failed under and leaving @p target alone when it did.
    [[nodiscard]] bool overrideAddress(const std::string& text, bool repeater, std::string_view key, detail::Address& target) {
        try {
            target = detail::addressFromText(text, repeater);
            return true;
        } catch (const std::invalid_argument& error) {
            std::println(stderr, "gr::blocks::ax25::Ax25Encode '{}': dropping a record whose {} override {}", this->name, key, error.what());
            return false;
        }
    }

    [[nodiscard]] static std::optional<std::string> string(const property_map* map, const char* key) {
        if (map == nullptr) {
            return std::nullopt;
        }
        const auto entry = map->find(property_map::key_type(key));
        return entry == map->end() ? std::nullopt : std::optional<std::string>(entry->second.value_or(std::string{}));
    }

    [[nodiscard]] static std::optional<bool> boolean(const property_map* map, const char* key) {
        if (map == nullptr) {
            return std::nullopt;
        }
        const auto entry = map->find(property_map::key_type(key));
        return entry == map->end() ? std::nullopt : std::optional<bool>(entry->second.value_or(false));
    }

    /// @brief A numeric override. A key holding something that is not a number reads back out of range and is refused.
    [[nodiscard]] static std::optional<gr::Size_t> number(const property_map* map, const char* key) {
        if (map == nullptr) {
            return std::nullopt;
        }
        const auto entry = map->find(property_map::key_type(key));
        return entry == map->end() ? std::nullopt : std::optional<gr::Size_t>(entry->second.value_or(gr::Size_t{0x100U}));
    }
};

GR_REGISTER_BLOCK(gr::blocks::ax25::Ax25AddressFilter)

/*!
@brief Routes a decoded frame to `ok` or `fail` by address, over the keys `Ax25Decode` already wrote.

One predicate, and it parses nothing: `ax25_destination`, `ax25_source` and `ax25_via` are read as `Ax25Decode` wrote
them and nothing about the frame's bytes is touched again. `direction` says which of the first two keys the `address`
setting is read against — `"destination"`, `"source"` or `"either"`, required with no default because addressed-to
and heard-from are different streams and a default would silently pick one. `address` is `CALL` or `CALL-N`: without
an SSID it matches any, with one it matches that SSID exactly. `digipeater`, when non-empty, is read under that same
grammar and gives a frame a second, independent way in — checked only when the address and direction did not already
match — if it names a hop of `ax25_via` carrying the trailing `*` that `Ax25Decode` writes for a hop whose H bit was
set. Both settings are refused when staged if the grammar does not accept them, so a callsign nobody can spell is a
message at configuration time rather than a filter that silently never matches; the `*` belongs to the frame and not
to the setting, and writing one into `digipeater` is one of the spellings refused.

A record none of whose direction's keys is present goes to `fail` and is counted `nMissingKey` before the digipeater
path is considered: a missing key is not a value to fall back past. Under `"either"` that means both keys absent —
one key present and not matching is an ordinary failure, because the frame did say who it was. A key of the wrong
type reads as absent, the same convention every metadata-reading block in this tree follows. Nothing is written to
metadata on either port — the frame's facts were `Ax25Decode`'s to write and are identical on both outputs, so a
`matched` flag here would only be a second spelling of which port the record left by.

`fail` is `gr::Optional`. Connected, it bounds the loop exactly as `ok` does: a refused record whose port has no room
stays in the input buffer for the next call rather than being counted and dropped on the floor. Unconnected, a
refused record is a counted stated drop and only `nFailed` is left to say it happened.
*/
struct Ax25AddressFilter : Block<Ax25AddressFilter> {
    using Description = Doc<"AX.25 address filter: routes a decoded frame to 'ok' or 'fail' by 'address', 'direction' and an optional 'digipeater' hop, reading only ax25_destination, ax25_source and ax25_via">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           ok;
    PortOut<DataSet<std::uint8_t>, Async, Optional> fail;

    Annotated<std::string, "address", Doc<"'CALL' or 'CALL-N'; required. Without '-N' any SSID matches; with '-N' the SSID must match exactly">, Visible> address{};
    Annotated<std::string, "direction", Doc<"'destination', 'source' or 'either'; required, no default">, Visible>                                        direction{};
    Annotated<std::string, "digipeater", Doc<"when non-empty, a frame also matches if this callsign appears in ax25_via with its '*' marker">, Visible>   digipeater{};

    GR_MAKE_REFLECTABLE(Ax25AddressFilter, in, ok, fail, address, direction, digipeater);

    detail::Address _address{};
    bool            _hasSsid = false;
    detail::Address _digipeater{};
    bool            _digipeaterHasSsid = false;
    bool            _configured        = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords    = 0ULL; ///< records published on `ok`
    std::uint64_t nFailed     = 0ULL; ///< records routed to `fail`, for any reason, whether or not the port is connected
    std::uint64_t nMissingKey = 0ULL; ///< records carrying none of the keys `direction` names, a subset of nFailed

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() {
        rebuild();
        if (address.value.empty()) {
            throw gr::exception("address is required and has no default: an empty value cannot be told from a setting nobody staged");
        }
        if (direction.value.empty()) {
            throw gr::exception("direction is required and has no default: an empty value cannot be told from a setting nobody staged");
        }
    }

    /// @brief Rebuilds the parsed address from the settings, refusing by name what the grammar or the direction value does not accept.
    void rebuild() {
        if (!direction.value.empty() && direction.value != "destination" && direction.value != "source" && direction.value != "either") {
            throw gr::exception(std::format("direction must be 'destination', 'source' or 'either', got '{}'", direction.value));
        }
        if (!address.value.empty()) {
            _hasSsid = address.value.find('-') != std::string::npos;
            try {
                _address = detail::addressFromText(address.value, false);
            } catch (const std::invalid_argument& error) {
                throw gr::exception(std::format("address: {}", error.what()));
            }
        }
        if (!digipeater.value.empty()) {
            _digipeaterHasSsid = digipeater.value.find('-') != std::string::npos;
            try { // the repeated marker is the frame's; a hop is named here the way `address` names one
                _digipeater = detail::addressFromText(digipeater.value, false);
            } catch (const std::invalid_argument& error) {
                throw gr::exception(std::format("digipeater: {}", error.what()));
            }
        }
        _configured = !address.value.empty() && !direction.value.empty();
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records", nRecords);
        append("failed", nFailed);
        append("missing key", nMissingKey);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::ax25::Ax25AddressFilter '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& okSpan, OutputSpanLike auto& failSpan) {
        if (!_configured) { // inert rather than routing frames nobody named
            std::ignore = inSpan.consume(0UZ);
            okSpan.publish(0UZ);
            failSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        std::size_t refused  = 0UZ;
        // a connected `fail` bounds the loop as `ok` does: a refused record with nowhere to go waits in the input
        // buffer for the next call, where counting it and writing it nowhere would lose it
        const std::size_t failRoom = failSpan.isConnected ? failSpan.size() : std::numeric_limits<std::size_t>::max();
        for (; consumed < inSpan.size() && made < okSpan.size() && refused < failRoom; ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const property_map*          meta   = record.meta_information.empty() ? nullptr : &record.meta_information[0UZ];

            const Verdict verdict = classify(meta);
            if (verdict == Verdict::Matched) {
                ++nRecords;
                okSpan[made] = record;
                ++made;
                continue;
            }
            ++nFailed;
            if (verdict == Verdict::MissingKey) {
                ++nMissingKey;
            }
            if (failSpan.isConnected) {
                failSpan[refused] = record;
                ++refused;
            }
        }

        std::ignore = inSpan.consume(consumed);
        okSpan.publish(made);
        failSpan.publish(refused);
        if (made == 0UZ && refused == 0UZ && consumed == 0UZ) {
            const bool noRoom = okSpan.size() == 0UZ || (failSpan.isConnected && failSpan.size() == 0UZ);
            return noRoom ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    enum class Verdict : std::uint8_t { Matched, Failed, MissingKey };

    /// @brief @p key read as a string, an absent or wrongly typed key reading back as no value at all.
    [[nodiscard]] static std::optional<std::string> stringKey(const property_map* map, const char* key) {
        if (map == nullptr) {
            return std::nullopt;
        }
        const auto entry = map->find(property_map::key_type(key));
        if (entry == map->end()) {
            return std::nullopt;
        }
        const std::pmr::string* value = entry->second.get_if<std::pmr::string>();
        return value == nullptr ? std::nullopt : std::optional<std::string>(std::string(value->begin(), value->end()));
    }

    /// @brief Whether @p text, read as one address, is the address `address` names, its SSID rule applied.
    [[nodiscard]] bool addressMatches(const std::string& text) const {
        try {
            const detail::Address parsed = detail::addressFromText(text, false);
            if (parsed.call != _address.call) {
                return false;
            }
            return !_hasSsid || parsed.ssid == _address.ssid;
        } catch (const std::invalid_argument&) {
            return false; // a value the grammar does not accept matches nothing rather than throwing mid-stream
        }
    }

    /// @brief Whether `digipeater` names a hop present in @p via with its repeated ('*') marker.
    [[nodiscard]] bool digipeaterMatches(const std::string& via) const {
        if (digipeater.value.empty()) {
            return false;
        }
        for (std::size_t at = 0UZ; at <= via.size();) {
            const std::size_t      comma = via.find(',', at);
            const std::string_view hop   = comma == std::string::npos ? std::string_view(via).substr(at) : std::string_view(via).substr(at, comma - at);
            try {
                const detail::Address parsed = detail::addressFromText(hop, true);
                if (parsed.repeated && parsed.call == _digipeater.call && (!_digipeaterHasSsid || parsed.ssid == _digipeater.ssid)) {
                    return true;
                }
            } catch (const std::invalid_argument&) {
                // a hop the grammar does not accept names no repeater, and the hops beside it still do
            }
            if (comma == std::string::npos) {
                break;
            }
            at = comma + 1UZ;
        }
        return false;
    }

    /// @brief One record's verdict: matched, failed, or missing the key `direction` names.
    [[nodiscard]] Verdict classify(const property_map* meta) const {
        const bool checkDestination = direction.value == "destination" || direction.value == "either";
        const bool checkSource      = direction.value == "source" || direction.value == "either";

        const std::optional<std::string> destination = checkDestination ? stringKey(meta, "ax25_destination") : std::nullopt;
        const std::optional<std::string> source      = checkSource ? stringKey(meta, "ax25_source") : std::nullopt;

        if (!destination.has_value() && !source.has_value()) {
            return Verdict::MissingKey;
        }
        if ((destination.has_value() && addressMatches(*destination)) || (source.has_value() && addressMatches(*source))) {
            return Verdict::Matched;
        }
        if (digipeaterMatches(stringKey(meta, "ax25_via").value_or(std::string{}))) {
            return Verdict::Matched;
        }
        return Verdict::Failed;
    }
};

} // namespace gr::blocks::ax25

#endif // GNURADIO_AX25_AX25_HPP
