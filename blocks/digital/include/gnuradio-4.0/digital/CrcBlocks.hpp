#ifndef GNURADIO_DIGITAL_CRC_BLOCKS_HPP
#define GNURADIO_DIGITAL_CRC_BLOCKS_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Crc.hpp>

namespace gr::blocks::digital {

namespace detail {

/// @brief Which end of the appended field the most significant CRC byte sits at.
enum class ByteOrder : std::uint8_t { big, little };

[[nodiscard]] inline std::uint64_t readTrailingCrc(std::span<const std::uint8_t> bytes, std::size_t crcBytes, ByteOrder order) noexcept {
    const std::span<const std::uint8_t> field = bytes.last(crcBytes);
    std::uint64_t                       value = 0ULL;
    for (std::size_t i = 0UZ; i < crcBytes; ++i) {
        const std::size_t which = order == ByteOrder::big ? i : crcBytes - 1UZ - i;
        value                   = (value << 8U) | static_cast<std::uint64_t>(field[which]);
    }
    return value;
}

inline void appendCrc(std::vector<std::uint8_t>& bytes, std::uint64_t value, std::size_t crcBytes, ByteOrder order) {
    for (std::size_t i = 0UZ; i < crcBytes; ++i) {
        const std::size_t shift = order == ByteOrder::big ? 8UZ * (crcBytes - 1UZ - i) : 8UZ * i;
        bytes.push_back(static_cast<std::uint8_t>((value >> shift) & 0xFFULL));
    }
}

/// @brief One record's byte array, with the single-signal restriction enforced at the port rather than assumed.
[[nodiscard]] inline std::span<const std::uint8_t> payloadOf(const DataSet<std::uint8_t>& record) {
    if (record.signal_names.size() > 1UZ) {
        throw gr::exception(std::format("a CRC covers one flat byte array; this DataSet carries {} signals", record.signal_names.size()));
    }
    return std::span<const std::uint8_t>(record.signal_values);
}

inline void putRecordMeta(DataSet<std::uint8_t>& record, std::string_view key, pmt::Value value) {
    if (record.meta_information.empty()) {
        record.meta_information.emplace_back();
    }
    record.meta_information[0UZ].insert_or_assign(property_map::key_type(key), std::move(value));
}

inline void resizeRecord(DataSet<std::uint8_t>& record, std::size_t items) {
    record.signal_values.resize(items);
    if (record.extents.empty()) {
        record.extents.push_back(static_cast<std::int32_t>(items));
    } else {
        record.extents[0UZ] = static_cast<std::int32_t>(items);
    }
}

/// @brief The six-tuple's documentation, quoted by both blocks so the `initial_value` domain is stated at each of them.
using CrcSettingsDoc = Doc<R""(
The Rocksoft model's six-tuple. `poly` is MSB-first with the implicit `x^w` term omitted; `initial_value` is the seed
in the unreflected domain, which is the domain every published catalog's INIT column is in, so a catalog value is
used as printed. A seed taken from an implementation that loads it into a mirrored register is the bit-reversal of
the same number and has to be reversed first, unless it is a palindrome.
)"">;

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::CrcAppend)

struct CrcAppend : Block<CrcAppend> {
    using Description = Doc<R""(
@brief Appends the CRC of each packet to it, as whole bytes, and records what it computed.

A packet is one `DataSet<std::uint8_t>` whose `signal_values` are its bytes, carried one per item on an `Async` port;
GNU Radio 4 has no PDU type and no length-tag convention, so the native variable-length record is the container.
Per-record scalars live in `meta_information[0]`, and the block writes `crc_value` and `crc_width` there, which is what
lets a receiver's `CrcCheck` be told what to expect by the stream rather than by a duplicated setting.

`width` must be a multiple of 8 and at least 8, because the block appends whole bytes; the kernel itself goes down to 3
and is reached that way by the header formats. `skip_header_bytes` excludes leading bytes from the computation and
keeps them in the output, and a record no longer than `skip_header_bytes` has nothing to protect and is dropped with a
warning naming both numbers rather than being published unprotected. `crc_byte_order` says which end of the appended
field the most significant byte sits at and defaults to `big`.

`poly` is written MSB first with x^w omitted, and `initial_value` in the unreflected domain; `final_xor` is applied
last. With `input_reflected`, each message byte enters LSB first; with `result_reflected`, the register is bit-reversed
before final_xor. The leading `skip_header_bytes` are excluded from the computation and kept in the output.
)"">;

    PortIn<DataSet<std::uint8_t>>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"bit">, detail::CrcSettingsDoc, Visible, Limits<Min<8U>, Max<64U>>> width             = 32U;
        Annotated<std::uint64_t, Doc<"generator polynomial">>                                          poly              = 0x04C11DB7ULL;
        Annotated<std::uint64_t, Doc<"initial register value">>                                        initial_value     = 0xFFFFFFFFULL;
        Annotated<std::uint64_t, Doc<"final XOR value">>                                               final_xor         = 0xFFFFFFFFULL;
        Annotated<bool, Doc<"input reflection">>                                                       input_reflected   = true;
        Annotated<bool, Doc<"result reflection">>                                                      result_reflected  = true;
        Annotated<detail::ByteOrder, Doc<"CRC byte order">>                                            crc_byte_order    = detail::ByteOrder::big;
        Annotated<gr::Size_t, Unit<"byte">, Doc<"skipped header length">>                              skip_header_bytes = 0U;

        GR_MAKE_REFLECTABLE(Settings, width, poly, initial_value, final_xor, input_reflected, result_reflected, crc_byte_order, skip_header_bytes);
    } settings;

    GR_MAKE_REFLECTABLE(CrcAppend, in, out, settings);

    gr::digital::Crc _crc{32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true};
    std::size_t      _crcBytes = 4UZ;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.width % 8U != 0U) {
            return gr::Refused(s.width, "must be a multiple of 8");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _crc      = gr::digital::Crc(static_cast<std::uint8_t>(s.width), s.poly, s.initial_value, s.final_xor, s.input_reflected, s.result_reflected);
        _crcBytes = s.width / 8U;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool        connected = outSpan.isConnected; // read once: the count below is derived from it
        const std::size_t available = inSpan.size();
        const std::size_t nRecords  = connected ? std::min(available, outSpan.size()) : available;
        std::size_t       published = 0UZ;

        for (std::size_t i = 0UZ; i < nRecords; ++i) {
            const std::span<const std::uint8_t> bytes = detail::payloadOf(inSpan[i]);
            if (bytes.size() <= settings.skip_header_bytes) {
                std::println(stderr, "gr::blocks::digital::CrcAppend: dropping a record of {} bytes, which is no longer than skip_header_bytes = {} and has nothing to protect", bytes.size(), settings.skip_header_bytes.value);
                continue;
            }
            const std::uint64_t value = _crc.compute(bytes.subspan(settings.skip_header_bytes));

            DataSet<std::uint8_t> record = inSpan[i];
            record.signal_values.reserve(bytes.size() + _crcBytes);
            detail::appendCrc(record.signal_values, value, _crcBytes, settings.crc_byte_order);
            detail::resizeRecord(record, bytes.size() + _crcBytes);
            detail::putRecordMeta(record, "crc_value", pmt::Value(value));
            detail::putRecordMeta(record, "crc_width", pmt::Value(std::uint64_t{settings.width}));

            if (connected && published < outSpan.size()) {
                outSpan[published] = std::move(record);
            }
            ++published;
        }

        std::ignore = inSpan.consume(nRecords);
        outSpan.publish(connected ? published : 0UZ);
        // No room on the output is backpressure, not a stall: the framework counts a bare OK with nothing consumed and
        // nothing published as no progress and reports the block.
        // An empty input returns INSUFFICIENT_INPUT_ITEMS. The framework calls on an empty input too, because the
        // async output counts as ready whenever it has room.
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return nRecords == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::CrcCheck)

struct CrcCheck : Block<CrcCheck> {
    using Description = Doc<R""(
@brief Recomputes each packet's CRC, compares it against the trailing bytes, and routes the record by the answer.

A failing packet leaves by the `fail` port rather than being dropped, because a receive chain that cannot see its
failures cannot be debugged. A passing record is never dropped: with no room on `ok` it waits in the input buffer
and the block asks for output room. A failing record is never held: with `fail` unconnected, or connected and
full, it is consumed and dropped, and the block reports the first such drop. A block that returns without
consuming or publishing is the scheduler's definition of no progress, and a chain whose records are nearly all
damaged would stop at this one.

The length guard is mandatory: a record must hold more than `skip_header_bytes + width/8` bytes or it is dropped with
a warning, a packet length being attacker-controlled in any receive chain. The check compares values rather than
testing the residue, so it can report what it received: `meta_information[0]` carries `crc_ok` and the received
`crc_value`.

`poly` is written MSB first with x^w omitted, and `initial_value` in the unreflected domain; `final_xor` is applied
last. With `input_reflected`, each message byte enters LSB first; with `result_reflected`, the register is bit-reversed
before final_xor. The leading `skip_header_bytes` are excluded from the computation and always kept. `discard_crc`
strips the CRC bytes on both ports alike.
)"">;

    PortIn<DataSet<std::uint8_t>>         in;
    PortOut<DataSet<std::uint8_t>, Async> ok;
    PortOut<DataSet<std::uint8_t>, Async> fail;

    struct Settings {
        Annotated<gr::Size_t, Unit<"bit">, detail::CrcSettingsDoc, Visible, Limits<Min<8U>, Max<64U>>> width             = 32U;
        Annotated<std::uint64_t, Doc<"generator polynomial">>                                          poly              = 0x04C11DB7ULL;
        Annotated<std::uint64_t, Doc<"initial register value">>                                        initial_value     = 0xFFFFFFFFULL;
        Annotated<std::uint64_t, Doc<"final XOR value">>                                               final_xor         = 0xFFFFFFFFULL;
        Annotated<bool, Doc<"input reflection">>                                                       input_reflected   = true;
        Annotated<bool, Doc<"result reflection">>                                                      result_reflected  = true;
        Annotated<detail::ByteOrder, Doc<"CRC byte order">>                                            crc_byte_order    = detail::ByteOrder::big;
        Annotated<gr::Size_t, Unit<"byte">, Doc<"skipped header length">>                              skip_header_bytes = 0U;
        Annotated<bool, Doc<"CRC removal">>                                                            discard_crc       = false;

        GR_MAKE_REFLECTABLE(Settings, width, poly, initial_value, final_xor, input_reflected, result_reflected, crc_byte_order, skip_header_bytes, discard_crc);
    } settings;

    GR_MAKE_REFLECTABLE(CrcCheck, in, ok, fail, settings);

    gr::digital::Crc _crc{32U, 0x04C11DB7ULL, 0xFFFFFFFFULL, 0xFFFFFFFFULL, true, true};
    std::size_t      _crcBytes        = 4UZ;
    bool             _reportedRefused = false;

    /// @brief Records that failed and were dropped because the `fail` port had no room for them.
    std::uint64_t nRefusedDropped = 0ULL;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.width % 8U != 0U) {
            return gr::Refused(s.width, "must be a multiple of 8");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _crc      = gr::digital::Crc(static_cast<std::uint8_t>(s.width), s.poly, s.initial_value, s.final_xor, s.input_reflected, s.result_reflected);
        _crcBytes = s.width / 8U;
    }

    void start() {
        nRefusedDropped  = 0ULL;
        _reportedRefused = false;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& okSpan, OutputSpanLike auto& failSpan) {
        // the guard subtracts rather than adds: skip_header_bytes is 32 bits wide, so on a target whose size_t is also
        // 32 bits a skip within width/8 bytes of the maximum would carry the sum past the length it is compared with,
        // and every record would pass a guard it cannot satisfy. shortest is formed in 64 bits, for the message alone.
        const std::size_t   skip          = settings.skip_header_bytes;
        const std::uint64_t shortest      = std::uint64_t{settings.skip_header_bytes} + _crcBytes;
        const std::size_t   available     = inSpan.size();
        const bool          okConnected   = okSpan.isConnected; // read once, so the room test and the store cannot disagree
        const bool          failConnected = failSpan.isConnected;
        std::size_t         consumed      = 0UZ;
        std::size_t         onOk          = 0UZ;
        std::size_t         onFail        = 0UZ;

        for (std::size_t i = 0UZ; i < available; ++i) {
            const std::span<const std::uint8_t> bytes = detail::payloadOf(inSpan[i]);
            if (bytes.size() <= skip || bytes.size() - skip <= _crcBytes) {
                std::println(stderr, "gr::blocks::digital::CrcCheck: dropping a record of {} bytes, which is no longer than skip_header_bytes + width/8 = {}", bytes.size(), shortest);
                ++consumed;
                continue;
            }

            const std::uint64_t received   = detail::readTrailingCrc(bytes, _crcBytes, settings.crc_byte_order);
            const std::uint64_t recomputed = _crc.compute(bytes.subspan(skip, bytes.size() - skip - _crcBytes));
            const bool          passed     = received == recomputed;

            if (passed) {
                if (okConnected && onOk >= okSpan.size()) {
                    break; // a record that passed is not dropped; it waits in the buffer until there is room
                }
            } else if (failConnected && onFail >= failSpan.size()) {
                // A refused record is dropped rather than held: holding it consumes nothing, and a chain whose
                // records are nearly all damaged would stop there for as long as the fail port stays full.
                if (!_reportedRefused) {
                    _reportedRefused = true;
                    std::println(stderr, "gr::blocks::digital::CrcCheck: the fail port has no room; refused records are dropped from here on rather than held, which would stop the graph");
                }
                ++nRefusedDropped;
                ++consumed;
                continue;
            }

            DataSet<std::uint8_t> record = inSpan[i];
            if (settings.discard_crc) {
                detail::resizeRecord(record, bytes.size() - _crcBytes);
            }
            detail::putRecordMeta(record, "crc_ok", pmt::Value(passed));
            detail::putRecordMeta(record, "crc_value", pmt::Value(received));

            if (passed) {
                if (okConnected) {
                    okSpan[onOk] = std::move(record);
                }
                ++onOk;
            } else {
                if (failConnected) {
                    failSpan[onFail] = std::move(record);
                }
                ++onFail;
            }
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        okSpan.publish(okConnected ? onOk : 0UZ);
        failSpan.publish(failConnected ? onFail : 0UZ);
        // No room on `ok` is backpressure, not a stall: the framework counts a bare OK with nothing consumed and
        // nothing published as no progress and reports the block.
        // An empty input returns INSUFFICIENT_INPUT_ITEMS, as in CrcAppend.
        if (available == 0UZ) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return consumed == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_CRC_BLOCKS_HPP
