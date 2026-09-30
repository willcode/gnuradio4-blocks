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
 * `Ax25Decode` and `Ax25Encode` handle the address, control and protocol-identifier layer of the
 * AX.25 version 2.2 frame. Both are record blocks over `DataSet<std::uint8_t>`.
 *
 * `Ax25Decode` expects a frame without flags, bit stuffing or frame check sequence, as
 * `DelimiterExtractor` and `CrcCheck` leave it. Such a frame holds an address field of two to ten
 * seven-byte subfields, one control byte, a protocol identifier on the two frame types that carry
 * one, and the information field. `Ax25Encode` builds the same layout. `CrcAppend` and
 * `DelimiterFramer` add the check sequence and the framing.
 *
 * The address arithmetic is shifts and masks over a fixed layout. It lives in `detail` in this
 * header, not in the algorithm library. The blocks that use it are in this header.
 *
 * `Ax25Decode` refuses a frame for its structure alone. It does not check the low bits of the
 * callsign bytes or hold the characters to an alphabet. The frame has already passed its integrity
 * check. Real traffic also carries data in those characters. APRS Mic-E encodes a position in the
 * destination callsign, and a character-set rule would drop those frames.
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

/// @brief The callsign, the SSID and the repeated ('*') marker of one address subfield.
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
    bool        commandKnown = false; ///< false when both C bits agree, the pre-2.2 convention with no command flag
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

/// @brief Parses @p text as one address and throws `std::invalid_argument` when the grammar refuses it.
///
/// A trailing '*' is accepted on a repeater alone and sets its H bit. A chain that regenerates received frames uses it
/// to reproduce them. A new frame carries no '*' and leaves the bit clear.
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
/// An unlisted unnumbered modifier is reported as type "U" with the masked byte kept. The frame is still valid, and
/// refusing it would lose valid traffic.
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

/// @brief Parses @p frame by position, seven bytes per subfield, then the control byte and the protocol identifier.
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
        // An address field that ends after the destination has no source subfield. One that ends at the record's end
        // has no control byte. Both frames are shorter than the structure requires.
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
@brief Decodes one validated AX.25 frame per record into its information field and writes its address layer to metadata.

The output record's items are the information field. The field is often empty. A UA or DISC frame carries none, and
its record has zero items and carries everything in the keys. The block copies the input record's metadata and writes
its own keys over it. Keys that earlier blocks such as `CrcCheck` and `DelimiterExtractor` wrote stay beside the
frame's own keys.

`ax25_destination`, `ax25_source` and `ax25_via` hold the text forms. An address reads `CALL` or `CALL-N`. A repeater
whose H bit is set gains a trailing `*`. `ax25_via` joins the path with commas and is empty when the frame has none.
`ax25_type` names the frame type. `ax25_poll_final` carries the poll/final bit. The sequence numbers, the protocol
identifier and the masked control byte appear on the frame types that have them. `ax25_command` appears only when the
two C bits differ, as version 2.2 encodes them. Two equal C bits follow the older convention and carry no information.
The block then omits the key.

The block refuses two structures, publishes nothing for them and counts them. One is a record too short for the
structure parsed so far. The other is an address field whose tenth subfield still has its extension bit clear. Both
counts are reported at `stop()`. The next record decodes normally.
*/
struct Ax25Decode : Block<Ax25Decode> {
    using Description = Doc<"Decodes one FCS-stripped AX.25 frame per record into its information field and writes the address, control and PID fields to metadata">;

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
                map = record.meta_information[0UZ]; // the input metadata passes through, and the block's keys overwrite it
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
@brief Builds one AX.25 frame per record around its information field, without the frame check sequence.

The addresses are settings. A transmitter's own call and its path belong to the station, not to one frame. They follow
the text grammar that `Ax25Decode` writes. A callsign is one to six characters of `A`-`Z` and `0`-`9`, with an
optional `-N` for N from 0 to 15. A repeater may carry a trailing `*`, which sets its H bit. A setting outside the
grammar is refused when it is staged, with an error that names the setting. A chain with an unreadable callsign stops
before it starts. An empty `destination` or `source` leaves the block inert.

A record may override `ax25_destination`, `ax25_source`, `ax25_via`, `ax25_command`, `ax25_poll_final` or `ax25_pid`
for its own frame. A digipeater or a test uses these overrides to regenerate frames it read. An override arrives
mid-stream. The block drops a record whose override fails the grammar, names the key in a message and counts the
record in `nRefusedOverride`. The chain keeps running. The next record is built from the settings again.

`control_type` names the frame type. The block refuses the I and supervisory types. Their sequence numbers belong to a
connected-mode state machine, and this block numbers no frames. The reserved SSID bits go out as `11`. `command` true
sends the command pair of C bits, destination 1 and source 0. `command` false sends the response pair. The block
emits a protocol identifier on UI frames alone. UI is the only accepted type that carries one. `pid` defaults to
0xF0, the no-layer-3 value that APRS and plain text use.
*/
struct Ax25Encode : Block<Ax25Encode> {
    using Description = Doc<"Builds an AX.25 frame from each record of information bytes by adding the address field, control byte and PID. The frame carries no check sequence or flags">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "destination", Doc<"destination address, 'CALL' or 'CALL-N', required">, Visible>                   destination{};
    Annotated<std::string, "source", Doc<"source address, 'CALL' or 'CALL-N', required">, Visible>                             source{};
    Annotated<std::string, "via", Doc<"up to eight comma-separated repeater hops, '*' setting the H bit">, Visible>            via{};
    Annotated<bool, "command", Doc<"send the command pair of C bits, else the response pair">>                                 command      = true;
    Annotated<bool, "poll_final", Doc<"the poll/final bit of the control byte">>                                               poll_final   = false;
    Annotated<gr::Size_t, "pid", Doc<"protocol identifier of a UI frame, 0 to 255">>                                           pid          = 0xF0U;
    Annotated<std::string, "control_type", Doc<"'UI', 'DM', 'SABM', 'DISC', 'UA', 'SABME', 'FRMR', 'XID' or 'TEST'">, Visible> control_type = std::string("UI");

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
        if (!_configured) { // inert until destination and source are set
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
                frame.meta_information[0UZ] = *meta; // the record's metadata passes through, and the block adds no keys
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
@brief Routes a decoded AX.25 frame to `ok` or `fail` by the address keys in its metadata.

The block reads the `ax25_destination`, `ax25_source` and `ax25_via` keys that `Ax25Decode` writes. It does not parse
the frame's bytes. `direction` selects the key that `address` is compared against. It takes `"destination"`,
`"source"` or `"either"`. It is required and has no default, because addressed-to and heard-from are different
streams. `address` is `CALL` or `CALL-N`. Without an SSID it matches any SSID. With one it matches that SSID exactly.

A non-empty `digipeater` gives a frame a second way to match. The block checks it only when `address` did not match.
It matches when `ax25_via` names that hop with the trailing `*`. `Ax25Decode` writes the `*` for a hop whose H bit is
set. `digipeater` follows the `address` grammar. The `*` belongs to the frame, and a `*` in the setting is refused.
Both settings are refused at staging when the grammar does not accept them. A misspelled callsign is then an error at
configuration time. It does not become a filter that never matches.

A record that lacks every key `direction` names goes to `fail` and is counted in `nMissingKey`. The block does not
try the digipeater path for it. Under `"either"` both keys must be absent. A record with one key present that does
not match is an ordinary failure. A key of the wrong type reads as absent. The block writes no metadata on either
port. The frame's keys are identical on both outputs, and the output port already shows whether the record matched.

`fail` is `gr::Optional`. When `fail` is connected, it bounds the loop as `ok` does. A refused record that finds no
room on `fail` stays in the input buffer for the next call. When `fail` is unconnected, the block drops a refused
record and counts it in `nFailed`.
*/
struct Ax25AddressFilter : Block<Ax25AddressFilter> {
    using Description = Doc<"Routes a decoded AX.25 frame to 'ok' or 'fail' by 'address', 'direction' and an optional 'digipeater' hop. It reads only ax25_destination, ax25_source and ax25_via">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           ok;
    PortOut<DataSet<std::uint8_t>, Async, Optional> fail;

    Annotated<std::string, "address", Doc<"required callsign, 'CALL' matching any SSID or 'CALL-N' matching one">, Visible> address{};
    Annotated<std::string, "direction", Doc<"required address key, 'destination', 'source' or 'either'">, Visible>          direction{};
    Annotated<std::string, "digipeater", Doc<"repeater callsign that also matches when marked '*' in ax25_via">, Visible>   digipeater{};

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
            try { // the setting names a hop without the '*' marker, as `address` names an address
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
        if (!_configured) { // inert until address and direction are set
            std::ignore = inSpan.consume(0UZ);
            okSpan.publish(0UZ);
            failSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        std::size_t refused  = 0UZ;
        // A connected `fail` bounds the loop as `ok` does. A refused record with no room on `fail` waits in the input
        // buffer for the next call. Counting it and writing it nowhere would lose it.
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

    /// @brief Whether @p text parses as the address that `address` names, under its SSID rule.
    [[nodiscard]] bool addressMatches(const std::string& text) const {
        try {
            const detail::Address parsed = detail::addressFromText(text, false);
            if (parsed.call != _address.call) {
                return false;
            }
            return !_hasSsid || parsed.ssid == _address.ssid;
        } catch (const std::invalid_argument&) {
            return false; // a value the grammar refuses matches nothing and throws nothing mid-stream
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

    /// @brief Classifies one record as matched, failed, or missing the keys `direction` names.
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
