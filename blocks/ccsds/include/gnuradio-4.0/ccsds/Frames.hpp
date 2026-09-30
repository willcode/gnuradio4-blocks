#ifndef GNURADIO_CCSDS_FRAMES_HPP
#define GNURADIO_CCSDS_FRAMES_HPP

#include <algorithm>
#include <array>
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

#include <gnuradio-4.0/algorithm/ccsds/TransferFrame.hpp>
#include <gnuradio-4.0/ccsds/RecordHelpers.hpp>

/**
 * @brief The TM, AOS and TC transfer-frame decoders and encoders, CCSDS 132.0-B-3 / 732.0-B-4 / 232.0-B-4.
 *
 * Each decoder takes one frame per `DataSet<std::uint8_t>` record and publishes its data field. It writes the
 * primary header to metadata. The TM secondary header and the operational control field of either standard go to
 * metadata or to their own optional ports. Every setting is immutable configuration. `rebuild()` validates it and
 * runs from both `settingsChanged` and `start()`. A refused configuration leaves the block inert. The block then
 * publishes and consumes nothing and returns `work::Status::ERROR`. Each refused record is dropped and counted in a
 * named counter. The counters are reported once at `stop()`.
 *
 * Each decoder drops a frame whose spacecraft identifier differs from `spacecraft_id` and counts it in
 * `nFilteredScid`. An unset `spacecraft_id` accepts any. With `require_crc_ok` set, a decoder drops a record whose
 * `crc_ok` key is false and counts it in `nCrcFailed`. A record without the key passes, because no check was
 * claimed for it.
 *
 * The carrier is `DataSet<std::uint8_t>` throughout. Input metadata passes through, and a block writes its own keys
 * over it. These blocks never compute the frame error control field. `digital::CrcCheck` and `digital::CrcAppend`
 * compute it with the parameter set of 132.0-B-3 4.1.6.2.2 / 232.0-B-4 4.1.4.2 (CRC-16/IBM-3740).
 */
namespace gr::blocks::ccsds {

namespace frames_detail {

inline constexpr gr::Size_t kUnsetScid = 0xFFFFU; // wider than TM's ten bits and AOS/TC's eight

inline constexpr gr::Size_t kTmMaxScid  = 1023U; // 132.0-B-3 4.1.2.2.3, ten bits
inline constexpr gr::Size_t kAosMaxScid = 255U;  // 732.0-B-4 4.1.2.2.3, eight bits

/// A frame count register per virtual channel. The count of 4.1.2.6 belongs to its own channel alone.
inline constexpr std::size_t kTmVirtualChannels  = 8UZ;  // 132.0-B-3 4.1.2.3, three bits
inline constexpr std::size_t kAosVirtualChannels = 64UZ; // 732.0-B-4 4.1.2.3, six bits

/**
 * @brief The operational control field's metadata, 132.0-B-3 4.1.5.
 *
 * Bit 0 selects the report type. A Type-1 report is the CLCW of 232.0-B-4 4.2.1.1.2, and its eleven fields are
 * parsed. A Type-2 report is four opaque octets of the SDLS protocol. Only its bit 1 (4.1.5.5) is read. The two
 * reports carry different `protocol` labels because they are different objects. Only the Type-1 record holds a
 * control link word.
 */
inline void writeOcfMetadata(property_map& map, std::span<const std::uint8_t> ocfBytes) {
    const gr::ccsds::OcfReportType type   = gr::ccsds::ocfReportType(ocfBytes);
    const bool                     isClcw = type == gr::ccsds::OcfReportType::type_1_clcw;
    map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string(isClcw ? "ccsds/clcw" : "ccsds/ocf")));
    map.insert_or_assign(property_map::key_type("ccsds_ocf_type"), pmt::Value(gr::Size_t{isClcw ? 0U : 1U}));
    if (!isClcw) {
        map.insert_or_assign(property_map::key_type("ccsds_ocf_sdls"), pmt::Value(type == gr::ccsds::OcfReportType::type_2_sdls));
        return;
    }
    gr::ccsds::Clcw clcw{};
    static_cast<void>(gr::ccsds::parseClcw(ocfBytes, clcw));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_version"), pmt::Value(gr::Size_t{clcw.version}));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_status"), pmt::Value(gr::Size_t{clcw.status}));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_cop_in_effect"), pmt::Value(gr::Size_t{clcw.cop_in_effect}));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_vcid"), pmt::Value(gr::Size_t{clcw.virtual_channel}));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_no_rf"), pmt::Value(clcw.no_rf_available));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_no_bit_lock"), pmt::Value(clcw.no_bit_lock));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_lockout"), pmt::Value(clcw.lockout));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_wait"), pmt::Value(clcw.wait));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_retransmit"), pmt::Value(clcw.retransmit));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_farm_b"), pmt::Value(gr::Size_t{clcw.farm_b_counter}));
    map.insert_or_assign(property_map::key_type("ccsds_clcw_report_value"), pmt::Value(gr::Size_t{clcw.report_value}));
}

} // namespace frames_detail

GR_REGISTER_BLOCK(gr::blocks::ccsds::TmFrameDecode)

/*!
@brief Decodes one TM transfer frame per record into its data field, 132.0-B-3 4.1.

`frame_length` is required. 4.1.1.2 makes the frame length constant for a mission phase, and no bit states it. The
block reads the presence of the operational control field from the primary header (4.1.2.4). No bit states the
presence of the frame error control field (4.1.6.1.2), and `has_fecf` sets it. `secondary_header` and `ocf` are
optional ports. With neither connected, the block does no work for them beyond the counters. The counters count
what was not published in either case. A gap larger than a non-zero `max_frames_lost_report` is reported as a
discontinuity and left out of the frames-lost total.
*/
struct TmFrameDecode : Block<TmFrameDecode> {
    using Description = Doc<"Decodes one TM transfer frame per record into its data field. It writes the primary header, secondary header and OCF to metadata and publishes the last two on their own ports (132.0-B-3 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           out;
    PortOut<DataSet<std::uint8_t>, Async, Optional> secondary_header;
    PortOut<DataSet<std::uint8_t>, Async, Optional> ocf;

    Annotated<gr::Size_t, "frame_length", Doc<"required total octets of each arriving frame, 4.1.1.2">, Visible>        frame_length = 0U;
    Annotated<bool, "has_fecf", Doc<"two trailing FECF octets, excluded from the data field, 4.1.6.1.2">>               has_fecf     = false;
    Annotated<gr::Size_t, "spacecraft_id", Doc<"spacecraft identifier to accept, unset for any">>                       spacecraft_id{frames_detail::kUnsetScid};
    Annotated<bool, "require_crc_ok", Doc<"drop records whose crc_ok key is false">>                                    require_crc_ok         = false;
    Annotated<gr::Size_t, "max_frames_lost_report", Doc<"largest gap added to the frames-lost total, 0 for unlimited">> max_frames_lost_report = 0U;

    GR_MAKE_REFLECTABLE(TmFrameDecode, in, out, secondary_header, ocf, frame_length, has_fecf, spacecraft_id, require_crc_ok, max_frames_lost_report);

    std::uint64_t nFrames             = 0ULL;
    std::uint64_t nDataFieldOctets    = 0ULL;
    std::uint64_t nFramesLost         = 0ULL;
    std::uint64_t nMcFramesLost       = 0ULL;
    std::uint64_t nDuplicateFrames    = 0ULL;
    std::uint64_t nReservedViolations = 0ULL;
    std::uint64_t nSecondaryHeaders   = 0ULL;
    std::uint64_t nOcfRecords         = 0ULL;
    std::uint64_t nRefusedShort       = 0ULL;
    std::uint64_t nWrongVersion       = 0ULL;
    std::uint64_t nFilteredScid       = 0ULL;
    std::uint64_t nBadSecondaryHeader = 0ULL;
    std::uint64_t nBadGeometry        = 0ULL;
    std::uint64_t nCrcFailed          = 0ULL;

    bool          _configured    = false;
    std::size_t   _fixedOverhead = 0UZ; // 6 + (has_fecf ? 2 : 0): the part known without reading the frame
    std::uint32_t _lastMc        = 0U;
    bool          _haveMc        = false;

    std::array<std::uint32_t, frames_detail::kTmVirtualChannels> _lastVc{};
    std::array<bool, frames_detail::kTmVirtualChannels>          _haveVc{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (frame_length.value == 0U) {
            throw gr::exception("frame_length is required and has no default: 4.1.1.2 makes it a mission property and a wrong guess mis-slices every frame");
        }
        const std::size_t fecfOctets = has_fecf.value ? gr::ccsds::kFecfSize : 0UZ;
        _fixedOverhead               = gr::ccsds::kTmPrimaryHeaderSize + fecfOctets;
        if (frame_length.value <= _fixedOverhead) {
            throw gr::exception(std::format("frame_length {} leaves no room for a data field once the {}-octet primary header and{} are removed", frame_length.value, gr::ccsds::kTmPrimaryHeaderSize, has_fecf.value ? " the 2-octet FECF" : " nothing else"));
        }
        _haveMc = false;
        _haveVc.fill(false);
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "TmFrameDecode", {{"frames", nFrames}, {"data field octets", nDataFieldOctets}, {"frames lost", nFramesLost}, {"mc frames lost", nMcFramesLost}, {"duplicate frames", nDuplicateFrames}, {"reserved violations", nReservedViolations}, {"secondary headers", nSecondaryHeaders}, {"ocf records", nOcfRecords}, {"short frames", nRefusedShort}, {"wrong version", nWrongVersion}, {"wrong spacecraft", nFilteredScid}, {"bad secondary header", nBadSecondaryHeader}, {"bad geometry", nBadGeometry}, {"crc failed", nCrcFailed}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& shSpan, OutputSpanLike auto& ocfSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            if (shSpan.isConnected) {
                shSpan.publish(0UZ);
            }
            if (ocfSpan.isConnected) {
                ocfSpan.publish(0UZ);
            }
            return work::Status::ERROR;
        }

        const bool  shConnected  = shSpan.isConnected;
        const bool  ocfConnected = ocfSpan.isConnected;
        std::size_t consumed     = 0UZ;
        std::size_t made         = 0UZ;
        std::size_t madeSh       = 0UZ;
        std::size_t madeOcf      = 0UZ;

        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>&        record = inSpan[consumed];
            const std::span<const std::uint8_t> bytes(record.signal_values);

            if (bytes.size() < frame_length.value) {
                ++nRefusedShort;
                continue;
            }

            gr::ccsds::TmPrimaryHeader   header{};
            const gr::ccsds::ParseStatus status = gr::ccsds::parseTmPrimaryHeader(bytes, header);
            if (status == gr::ccsds::ParseStatus::bad_version) {
                ++nWrongVersion;
                continue;
            }
            if (spacecraft_id.value != frames_detail::kUnsetScid && header.spacecraft_id != spacecraft_id.value) {
                ++nFilteredScid;
                continue;
            }
            if (require_crc_ok.value) {
                if (const std::optional<bool> ok = detail::readBool(detail::metaOf(record), "crc_ok"); ok.has_value() && !*ok) {
                    ++nCrcFailed;
                    continue;
                }
            }

            std::size_t                    tfshOctets = 0UZ;
            gr::ccsds::TmSecondaryHeaderId shId{};
            if (header.secondary_header) {
                if (bytes.size() < gr::ccsds::kTmPrimaryHeaderSize + gr::ccsds::kTmSecondaryHeaderIdSize) {
                    ++nBadSecondaryHeader;
                    continue;
                }
                static_cast<void>(gr::ccsds::parseTmSecondaryHeaderId(bytes.subspan(gr::ccsds::kTmPrimaryHeaderSize), shId));
                if (!gr::ccsds::tmSecondaryHeaderUsable(shId)) {
                    ++nBadSecondaryHeader;
                    continue;
                }
                tfshOctets = gr::ccsds::tmSecondaryHeaderOctets(shId);
                if (gr::ccsds::kTmPrimaryHeaderSize + tfshOctets > frame_length.value) {
                    ++nBadSecondaryHeader;
                    continue;
                }
            }

            const std::size_t ocfOctets = header.ocf_present ? gr::ccsds::kOcfSize : 0UZ;
            const std::size_t overhead  = _fixedOverhead + tfshOctets + ocfOctets;
            if (overhead >= frame_length.value) {
                ++nBadGeometry;
                continue;
            }
            const std::size_t dataFieldStart  = gr::ccsds::kTmPrimaryHeaderSize + tfshOctets;
            const std::size_t dataFieldLength = frame_length.value - overhead;
            const std::size_t ocfStart        = dataFieldStart + dataFieldLength;

            // Every port this record needs must have room before any of it is counted, published or folded into the
            // gap state. A full optional port then leaves the record whole for the next call.
            if ((header.secondary_header && shConnected && madeSh >= shSpan.size()) || (header.ocf_present && ocfConnected && madeOcf >= ocfSpan.size())) {
                break;
            }
            if (status == gr::ccsds::ParseStatus::reserved_violation) {
                ++nReservedViolations; // the frame is still valid, reported and not refused
            }

            bool       gapDetected  = false;
            gr::Size_t framesLost   = 0U;
            gr::Size_t mcFramesLost = 0U;
            if (_haveMc) {
                const gr::ccsds::SequenceGap mcGap = gr::ccsds::frameGap(header.master_frame_count, _lastMc, gr::ccsds::kTmCountModulus);
                if (!mcGap.duplicate && !mcGap.continuous) {
                    mcFramesLost = mcGap.lost;
                    gapDetected  = true;
                }
            }
            _lastMc = header.master_frame_count;
            _haveMc = true;

            const std::size_t channel = std::size_t{header.virtual_channel};
            if (_haveVc[channel]) {
                const gr::ccsds::SequenceGap vcGap = gr::ccsds::frameGap(header.vc_frame_count, _lastVc[channel], gr::ccsds::kTmCountModulus);
                if (vcGap.duplicate) {
                    ++nDuplicateFrames;
                } else if (!vcGap.continuous) {
                    framesLost  = vcGap.lost;
                    gapDetected = true;
                }
            }
            _lastVc[channel] = header.vc_frame_count;
            _haveVc[channel] = true;

            if (max_frames_lost_report.value == 0U || framesLost <= max_frames_lost_report.value) {
                nFramesLost += framesLost;
            }
            if (max_frames_lost_report.value == 0U || mcFramesLost <= max_frames_lost_report.value) {
                nMcFramesLost += mcFramesLost;
            }

            DataSet<std::uint8_t> dataField;
            dataField.signal_values.assign(bytes.begin() + static_cast<std::ptrdiff_t>(dataFieldStart), bytes.begin() + static_cast<std::ptrdiff_t>(dataFieldStart + dataFieldLength));
            detail::startRecord(record, dataField, "ccsds");
            property_map& map = dataField.meta_information[0UZ];
            map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("ccsds/tm_frame")));
            map.insert_or_assign(property_map::key_type("ccsds_frame_version"), pmt::Value(gr::Size_t{header.version}));
            map.insert_or_assign(property_map::key_type("ccsds_scid"), pmt::Value(gr::Size_t{header.spacecraft_id}));
            map.insert_or_assign(property_map::key_type("ccsds_vcid"), pmt::Value(gr::Size_t{header.virtual_channel}));
            map.insert_or_assign(property_map::key_type("ccsds_ocf_present"), pmt::Value(header.ocf_present));
            map.insert_or_assign(property_map::key_type("ccsds_mc_frame_count"), pmt::Value(gr::Size_t{header.master_frame_count}));
            map.insert_or_assign(property_map::key_type("ccsds_vc_frame_count"), pmt::Value(gr::Size_t{header.vc_frame_count}));
            map.insert_or_assign(property_map::key_type("ccsds_sec_hdr_present"), pmt::Value(header.secondary_header));
            map.insert_or_assign(property_map::key_type("ccsds_sync_flag"), pmt::Value(header.sync_flag));
            map.insert_or_assign(property_map::key_type("ccsds_packet_order_flag"), pmt::Value(header.packet_order));
            map.insert_or_assign(property_map::key_type("ccsds_segment_length_id"), pmt::Value(gr::Size_t{header.segment_length_id}));
            map.insert_or_assign(property_map::key_type("ccsds_first_header_pointer"), pmt::Value(gr::Size_t{header.first_header_pointer}));
            if (gapDetected) {
                map.insert_or_assign(property_map::key_type("ccsds_frames_lost"), pmt::Value(framesLost));
                map.insert_or_assign(property_map::key_type("ccsds_mc_frames_lost"), pmt::Value(mcFramesLost));
                detail::appendDiscontinuity(map, "frame_gap");
            }

            ++nFrames;
            nDataFieldOctets += dataField.signal_values.size();
            outSpan[made] = std::move(dataField);
            ++made;

            if (header.secondary_header) {
                ++nSecondaryHeaders;
                if (shConnected) {
                    const std::size_t     shDataStart  = gr::ccsds::kTmPrimaryHeaderSize + gr::ccsds::kTmSecondaryHeaderIdSize;
                    const std::size_t     shDataOctets = tfshOctets - gr::ccsds::kTmSecondaryHeaderIdSize;
                    DataSet<std::uint8_t> shRecord;
                    shRecord.signal_values.assign(bytes.begin() + static_cast<std::ptrdiff_t>(shDataStart), bytes.begin() + static_cast<std::ptrdiff_t>(shDataStart + shDataOctets));
                    detail::startRecord(record, shRecord, "ccsds_tfsh");
                    property_map& shMap = shRecord.meta_information[0UZ];
                    shMap.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("ccsds/tfsh")));
                    shMap.insert_or_assign(property_map::key_type("ccsds_vcid"), pmt::Value(gr::Size_t{header.virtual_channel}));
                    shSpan[madeSh] = std::move(shRecord);
                    ++madeSh;
                }
            }

            if (header.ocf_present) {
                ++nOcfRecords;
                if (ocfConnected) {
                    const std::span<const std::uint8_t> ocfBytes = bytes.subspan(ocfStart, gr::ccsds::kOcfSize);
                    DataSet<std::uint8_t>               ocfRecord;
                    ocfRecord.signal_values.assign(ocfBytes.begin(), ocfBytes.end());
                    detail::startRecord(record, ocfRecord, "ccsds_ocf");
                    frames_detail::writeOcfMetadata(ocfRecord.meta_information[0UZ], ocfBytes);
                    ocfSpan[madeOcf] = std::move(ocfRecord);
                    ++madeOcf;
                }
            }
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (shConnected) {
            shSpan.publish(madeSh);
        }
        if (ocfConnected) {
            ocfSpan.publish(madeOcf);
        }
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::AosFrameDecode)

/*!
@brief Decodes one AOS transfer frame per record into its data field or its M_PDU packet zone, 732.0-B-4 4.1.

The AOS header has no presence bit for the operational control field (732.0-B-4 4.1.1.1). `has_ocf` is therefore a
setting here. `TmFrameDecode` reads the same presence from its header. `data_unit` is required. 4.1.4.1.4 makes it a
static property of the virtual channel, and no bit states it. For `"m_pdu"` the output is the packet zone. The block
removes the two-octet M_PDU header and takes `ccsds_first_header_pointer` from it. The key has the same meaning as
the one `TmFrameDecode` takes from its primary header. The block skips the fixed insert zone of `insert_zone_length`
octets, counts them in `nInsertOctets` and never publishes them. There is no `secondary_header` port, because AOS has
no transfer frame secondary header.
*/
struct AosFrameDecode : Block<AosFrameDecode> {
    using Description = Doc<"Decodes one AOS transfer frame per record into its data field or M_PDU packet zone and writes the primary header and OCF to metadata (732.0-B-4 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>            in;
    PortOut<DataSet<std::uint8_t>, Async>           out;
    PortOut<DataSet<std::uint8_t>, Async, Optional> ocf;

    Annotated<gr::Size_t, "frame_length", Doc<"required total octets of each arriving frame">, Visible>                       frame_length{0U};
    Annotated<bool, "has_fhec", Doc<"2-octet frame header error control present, 8-octet primary header, 4.1.2.6.2">>         has_fhec           = false;
    Annotated<gr::Size_t, "insert_zone_length", Doc<"octets of the fixed insert zone, 4.1.3.4.1">>                            insert_zone_length = 0U;
    Annotated<bool, "has_ocf", Doc<"four-octet operational control field present">>                                           has_ocf            = false;
    Annotated<bool, "has_fecf", Doc<"two trailing FECF octets, excluded from the data field">>                                has_fecf           = false;
    Annotated<std::string, "data_unit", Doc<"required data unit, 'm_pdu', 'b_pdu', 'vca_sdu' or 'idle', 4.1.4.1.4">, Visible> data_unit{};
    Annotated<gr::Size_t, "spacecraft_id", Doc<"spacecraft identifier to accept, unset for any">>                             spacecraft_id{frames_detail::kUnsetScid};
    Annotated<bool, "require_crc_ok", Doc<"drop records whose crc_ok key is false">>                                          require_crc_ok = false;

    GR_MAKE_REFLECTABLE(AosFrameDecode, in, out, ocf, frame_length, has_fhec, insert_zone_length, has_ocf, has_fecf, data_unit, spacecraft_id, require_crc_ok);

    std::uint64_t nFrames             = 0ULL;
    std::uint64_t nDataFieldOctets    = 0ULL;
    std::uint64_t nInsertOctets       = 0ULL;
    std::uint64_t nFramesLost         = 0ULL;
    std::uint64_t nDuplicateFrames    = 0ULL;
    std::uint64_t nReservedViolations = 0ULL;
    std::uint64_t nOcfRecords         = 0ULL;
    std::uint64_t nRefusedShort       = 0ULL;
    std::uint64_t nWrongVersion       = 0ULL;
    std::uint64_t nFilteredScid       = 0ULL;
    std::uint64_t nBadGeometry        = 0ULL;
    std::uint64_t nCrcFailed          = 0ULL;

    bool        _configured          = false;
    bool        _isMpdu              = false;
    std::size_t _primaryHeaderOctets = 0UZ;
    std::size_t _dataFieldStart      = 0UZ;
    std::size_t _dataFieldLength     = 0UZ;
    std::size_t _ocfStart            = 0UZ;

    std::array<std::uint32_t, frames_detail::kAosVirtualChannels> _lastCount{};
    std::array<bool, frames_detail::kAosVirtualChannels>          _haveCount{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (frame_length.value == 0U) {
            throw gr::exception("frame_length is required and has no default");
        }
        if (data_unit.value != "m_pdu" && data_unit.value != "b_pdu" && data_unit.value != "vca_sdu" && data_unit.value != "idle") {
            throw gr::exception(std::format("data_unit must be 'm_pdu', 'b_pdu', 'vca_sdu' or 'idle' and has no default, got '{}'", data_unit.value));
        }
        _isMpdu                 = data_unit.value == "m_pdu";
        _primaryHeaderOctets    = gr::ccsds::aosPrimaryHeaderOctets(has_fhec.value);
        const std::size_t fixed = _primaryHeaderOctets + std::size_t{insert_zone_length.value} + (has_ocf.value ? gr::ccsds::kOcfSize : 0UZ) + (has_fecf.value ? gr::ccsds::kFecfSize : 0UZ);
        if (frame_length.value <= fixed || (_isMpdu && frame_length.value < fixed + gr::ccsds::kMpduHeaderSize)) {
            throw gr::exception(std::format("frame_length {} leaves no room for a data field once the fixed overhead of {} octets is removed", frame_length.value, fixed));
        }
        _dataFieldStart  = _primaryHeaderOctets + std::size_t{insert_zone_length.value};
        _dataFieldLength = frame_length.value - fixed;
        _ocfStart        = _dataFieldStart + _dataFieldLength;
        _haveCount.fill(false);
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "AosFrameDecode", {{"frames", nFrames}, {"data field octets", nDataFieldOctets}, {"insert zone octets", nInsertOctets}, {"frames lost", nFramesLost}, {"duplicate frames", nDuplicateFrames}, {"reserved violations", nReservedViolations}, {"ocf records", nOcfRecords}, {"short frames", nRefusedShort}, {"wrong version", nWrongVersion}, {"wrong spacecraft", nFilteredScid}, {"bad geometry", nBadGeometry}, {"crc failed", nCrcFailed}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& ocfSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            if (ocfSpan.isConnected) {
                ocfSpan.publish(0UZ);
            }
            return work::Status::ERROR;
        }

        const bool  ocfConnected = ocfSpan.isConnected;
        std::size_t consumed     = 0UZ;
        std::size_t made         = 0UZ;
        std::size_t madeOcf      = 0UZ;

        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>&        record = inSpan[consumed];
            const std::span<const std::uint8_t> bytes(record.signal_values);

            if (bytes.size() < frame_length.value) {
                ++nRefusedShort;
                continue;
            }

            gr::ccsds::AosPrimaryHeader  header{};
            const gr::ccsds::ParseStatus status = gr::ccsds::parseAosPrimaryHeader(bytes, has_fhec.value, header);
            if (status == gr::ccsds::ParseStatus::bad_version) {
                ++nWrongVersion;
                continue;
            }
            if (spacecraft_id.value != frames_detail::kUnsetScid && header.spacecraft_id != spacecraft_id.value) {
                ++nFilteredScid;
                continue;
            }
            if (require_crc_ok.value) {
                if (const std::optional<bool> ok = detail::readBool(detail::metaOf(record), "crc_ok"); ok.has_value() && !*ok) {
                    ++nCrcFailed;
                    continue;
                }
            }

            // The OCF port must have room before any of this record is counted, published or folded into the gap
            // state. A full optional port then leaves the record whole for the next call.
            if (has_ocf.value && ocfConnected && madeOcf >= ocfSpan.size()) {
                break;
            }
            if (status == gr::ccsds::ParseStatus::reserved_violation) {
                ++nReservedViolations; // the frame is still valid, reported and not refused
            }

            const std::uint32_t widened     = gr::ccsds::aosWidenedFrameCount(header);
            const std::uint32_t modulus     = gr::ccsds::aosCountModulus(header);
            const std::size_t   channel     = std::size_t{header.virtual_channel};
            bool                gapDetected = false;
            gr::Size_t          framesLost  = 0U;
            if (_haveCount[channel]) {
                const gr::ccsds::SequenceGap gap = gr::ccsds::frameGap(widened, _lastCount[channel], modulus);
                if (gap.duplicate) {
                    ++nDuplicateFrames;
                } else if (!gap.continuous) {
                    framesLost  = gap.lost;
                    gapDetected = true;
                    nFramesLost += framesLost;
                }
            }
            _lastCount[channel] = widened;
            _haveCount[channel] = true;

            nInsertOctets += insert_zone_length.value;

            std::size_t zoneStart   = _dataFieldStart;
            std::size_t zoneLength  = _dataFieldLength;
            gr::Size_t  pointer     = 0U;
            bool        havePointer = false;
            if (_isMpdu) {
                gr::ccsds::MpduHeader        mpdu{};
                const gr::ccsds::ParseStatus mpduStatus = gr::ccsds::parseMpduHeader(bytes.subspan(_dataFieldStart, gr::ccsds::kMpduHeaderSize), mpdu);
                if (mpduStatus == gr::ccsds::ParseStatus::reserved_violation) {
                    ++nReservedViolations;
                }
                zoneStart   = _dataFieldStart + gr::ccsds::kMpduHeaderSize;
                zoneLength  = _dataFieldLength - gr::ccsds::kMpduHeaderSize;
                pointer     = mpdu.first_header_pointer;
                havePointer = true;
            }

            DataSet<std::uint8_t> zone;
            zone.signal_values.assign(bytes.begin() + static_cast<std::ptrdiff_t>(zoneStart), bytes.begin() + static_cast<std::ptrdiff_t>(zoneStart + zoneLength));
            detail::startRecord(record, zone, "ccsds");
            property_map& map = zone.meta_information[0UZ];
            map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("ccsds/aos_frame")));
            map.insert_or_assign(property_map::key_type("ccsds_frame_version"), pmt::Value(gr::Size_t{header.version}));
            map.insert_or_assign(property_map::key_type("ccsds_scid"), pmt::Value(gr::Size_t{header.spacecraft_id}));
            map.insert_or_assign(property_map::key_type("ccsds_vcid"), pmt::Value(gr::Size_t{header.virtual_channel}));
            map.insert_or_assign(property_map::key_type("ccsds_vc_frame_count"), pmt::Value(gr::Size_t{widened}));
            map.insert_or_assign(property_map::key_type("ccsds_replay_flag"), pmt::Value(header.replay));
            map.insert_or_assign(property_map::key_type("ccsds_vcfc_cycle_use"), pmt::Value(header.vc_count_cycle_used));
            map.insert_or_assign(property_map::key_type("ccsds_vcfc_cycle"), pmt::Value(gr::Size_t{header.vc_count_cycle}));
            if (havePointer) {
                map.insert_or_assign(property_map::key_type("ccsds_first_header_pointer"), pmt::Value(pointer));
            }
            if (gapDetected) {
                map.insert_or_assign(property_map::key_type("ccsds_frames_lost"), pmt::Value(framesLost));
                detail::appendDiscontinuity(map, "frame_gap");
            }

            ++nFrames;
            nDataFieldOctets += zone.signal_values.size();
            outSpan[made] = std::move(zone);
            ++made;

            if (has_ocf.value) {
                ++nOcfRecords;
                if (ocfConnected) {
                    const std::span<const std::uint8_t> ocfBytes = bytes.subspan(_ocfStart, gr::ccsds::kOcfSize);
                    DataSet<std::uint8_t>               ocfRecord;
                    ocfRecord.signal_values.assign(ocfBytes.begin(), ocfBytes.end());
                    detail::startRecord(record, ocfRecord, "ccsds_ocf");
                    frames_detail::writeOcfMetadata(ocfRecord.meta_information[0UZ], ocfBytes);
                    ocfSpan[madeOcf] = std::move(ocfRecord);
                    ++madeOcf;
                }
            }
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (ocfConnected) {
            ocfSpan.publish(madeOcf);
        }
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::TcFrameDecode)

/*!
@brief Decodes one TC transfer frame per record into its data field, 232.0-B-4 4.1.

Of the three frame types, only TC describes its own length. The frame length field (4.1.2.7.2) carries the total,
and there is no `frame_length` setting. The block accepts a record longer than the declared total. The surplus is
not part of the frame. The block counts it in `nTrailingOctets` and does not treat it as an error. A fixed-size
channel can carry a variable-length frame.
*/
struct TcFrameDecode : Block<TcFrameDecode> {
    using Description = Doc<"Decodes one self-describing TC transfer frame per record into its data field and writes the primary header to metadata (232.0-B-4 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<bool, "has_fecf", Doc<"two trailing FECF octets, excluded from the data field">>    has_fecf = false;
    Annotated<gr::Size_t, "spacecraft_id", Doc<"spacecraft identifier to accept, unset for any">> spacecraft_id{frames_detail::kUnsetScid};
    Annotated<bool, "require_crc_ok", Doc<"drop records whose crc_ok key is false">>              require_crc_ok = false;

    GR_MAKE_REFLECTABLE(TcFrameDecode, in, out, has_fecf, spacecraft_id, require_crc_ok);

    std::uint64_t nFrames             = 0ULL;
    std::uint64_t nDataFieldOctets    = 0ULL;
    std::uint64_t nTrailingOctets     = 0ULL;
    std::uint64_t nReservedViolations = 0ULL;
    std::uint64_t nRefusedShort       = 0ULL;
    std::uint64_t nWrongVersion       = 0ULL;
    std::uint64_t nFilteredScid       = 0ULL;
    std::uint64_t nBadGeometry        = 0ULL;
    std::uint64_t nCrcFailed          = 0ULL;

    void settingsChanged(const property_map&, const property_map&) {}
    void start() {}

    void stop() { detail::reportCounters(*this, "TcFrameDecode", {{"frames", nFrames}, {"data field octets", nDataFieldOctets}, {"trailing octets", nTrailingOctets}, {"reserved violations", nReservedViolations}, {"short frames", nRefusedShort}, {"wrong version", nWrongVersion}, {"wrong spacecraft", nFilteredScid}, {"bad geometry", nBadGeometry}, {"crc failed", nCrcFailed}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>&        record = inSpan[consumed];
            const std::span<const std::uint8_t> bytes(record.signal_values);

            if (bytes.size() < gr::ccsds::kTcPrimaryHeaderSize) {
                ++nRefusedShort;
                continue;
            }
            gr::ccsds::TcPrimaryHeader   header{};
            const gr::ccsds::ParseStatus status = gr::ccsds::parseTcPrimaryHeader(bytes, header);
            if (status == gr::ccsds::ParseStatus::bad_version) {
                ++nWrongVersion;
                continue;
            }
            if (status == gr::ccsds::ParseStatus::reserved_violation) {
                ++nReservedViolations;
            }
            const std::size_t totalOctets = gr::ccsds::totalTcFrameOctets(header);
            if (bytes.size() < totalOctets) {
                ++nRefusedShort;
                continue;
            }
            if (spacecraft_id.value != frames_detail::kUnsetScid && header.spacecraft_id != spacecraft_id.value) {
                ++nFilteredScid;
                continue;
            }
            if (require_crc_ok.value) {
                if (const std::optional<bool> ok = detail::readBool(detail::metaOf(record), "crc_ok"); ok.has_value() && !*ok) {
                    ++nCrcFailed;
                    continue;
                }
            }

            const std::size_t fecfOctets = has_fecf.value ? gr::ccsds::kFecfSize : 0UZ;
            if (totalOctets <= gr::ccsds::kTcPrimaryHeaderSize + fecfOctets) {
                ++nBadGeometry;
                continue;
            }
            const std::size_t dataFieldLength = totalOctets - gr::ccsds::kTcPrimaryHeaderSize - fecfOctets;
            nTrailingOctets += bytes.size() - totalOctets;

            DataSet<std::uint8_t> dataField;
            dataField.signal_values.assign(bytes.begin() + static_cast<std::ptrdiff_t>(gr::ccsds::kTcPrimaryHeaderSize), bytes.begin() + static_cast<std::ptrdiff_t>(gr::ccsds::kTcPrimaryHeaderSize + dataFieldLength));
            detail::startRecord(record, dataField, "ccsds");
            property_map& map = dataField.meta_information[0UZ];
            map.insert_or_assign(property_map::key_type("protocol"), pmt::Value(std::string("ccsds/tc_frame")));
            map.insert_or_assign(property_map::key_type("ccsds_frame_version"), pmt::Value(gr::Size_t{header.version}));
            map.insert_or_assign(property_map::key_type("ccsds_bypass_flag"), pmt::Value(header.bypass));
            map.insert_or_assign(property_map::key_type("ccsds_control_command_flag"), pmt::Value(header.control_command));
            map.insert_or_assign(property_map::key_type("ccsds_scid"), pmt::Value(gr::Size_t{header.spacecraft_id}));
            map.insert_or_assign(property_map::key_type("ccsds_vcid"), pmt::Value(gr::Size_t{header.virtual_channel}));
            map.insert_or_assign(property_map::key_type("ccsds_frame_length"), pmt::Value(gr::Size_t{header.frame_length}));
            map.insert_or_assign(property_map::key_type("ccsds_frame_sequence_number"), pmt::Value(gr::Size_t{header.sequence_number}));

            ++nFrames;
            nDataFieldOctets += dataField.signal_values.size();
            outSpan[made] = std::move(dataField);
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

GR_REGISTER_BLOCK(gr::blocks::ccsds::TmFrameEncode)

/*!
@brief Encodes one data field per record as one whole TM transfer frame.

The block owns `master_frame_count` and `vc_frame_count` and increments each modulo 256 per frame (132.0-B-3
4.1.2.5.2, 4.1.2.6.2). Unlike a COP-1 sequence number, a frame count follows one rule of the standard and depends on
nothing outside the block. The block reads the first header pointer from the input record's
`ccsds_first_header_pointer` key. `SpacePacketSegment` writes that key for every zone it emits. A record without the
key is treated as a pure continuation (`kFhpNoPacketStart`), which claims nothing about the zone. A key that does not
fit the field's eleven bits is treated the same way and counted in `nBadPointerKey`. The block pads a data field
shorter than the frame's with the pseudo-noise fill of 4.1.4.6.2. The fill generator runs across frames and does
not restart per frame (4.1.4.6.2.1). The pointer is written as the metadata gave it.
*/
struct TmFrameEncode : Block<TmFrameEncode> {
    using Description = Doc<"Encodes one data field per record as a whole TM transfer frame with its own master and virtual channel frame counts (132.0-B-3 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "frame_length", Doc<"required total octets of each produced frame">, Visible>                       frame_length{0U};
    Annotated<gr::Size_t, "spacecraft_id", Doc<"required ten-bit spacecraft identifier">, Visible>                            spacecraft_id{detail::kUnset};
    Annotated<gr::Size_t, "virtual_channel", Doc<"required three-bit virtual channel identifier">, Visible>                   virtual_channel{detail::kUnset};
    Annotated<bool, "has_fecf", Doc<"reserve two unwritten trailing octets for the FECF">>                                    has_fecf = false;
    Annotated<bool, "has_ocf", Doc<"reserve four OCF octets, written as a zeroed CLCW">>                                      has_ocf  = false;
    Annotated<std::vector<std::uint8_t>, "secondary_header", Doc<"1 to 63 secondary header octets, empty clearing the flag">> secondary_header{};

    GR_MAKE_REFLECTABLE(TmFrameEncode, in, out, frame_length, spacecraft_id, virtual_channel, has_fecf, has_ocf, secondary_header);

    std::uint64_t nFrames          = 0ULL;
    std::uint64_t nRefusedOversize = 0ULL;
    std::uint64_t nRefusedHeader   = 0ULL;
    std::uint64_t nBadPointerKey   = 0ULL;

    bool               _configured = false;
    std::uint8_t       _mcCount    = 0U;
    std::uint8_t       _vcCount    = 0U;
    gr::ccsds::OidFill _fill{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (frame_length.value == 0U) {
            throw gr::exception("frame_length is required and has no default");
        }
        if (spacecraft_id.value == detail::kUnset || spacecraft_id.value > frames_detail::kTmMaxScid) {
            throw gr::exception(std::format("spacecraft_id is required and must be 0 to {}: a transmitter must know whose frame it is sending, and 4.1.2.2.3's field is ten bits wide", frames_detail::kTmMaxScid));
        }
        if (virtual_channel.value == detail::kUnset || virtual_channel.value > 7U) {
            throw gr::exception(std::format("virtual_channel is required and must be 0 to 7, got {}", virtual_channel.value == detail::kUnset ? -1 : static_cast<long long>(virtual_channel.value)));
        }
        if (secondary_header.value.size() > 63UZ) {
            throw gr::exception(std::format("secondary_header is at most 63 octets, got {}", secondary_header.value.size()));
        }
        const std::size_t tfshOctets = secondary_header.value.empty() ? 0UZ : gr::ccsds::kTmSecondaryHeaderIdSize + secondary_header.value.size();
        const std::size_t overhead   = gr::ccsds::kTmPrimaryHeaderSize + tfshOctets + (has_ocf.value ? gr::ccsds::kOcfSize : 0UZ) + (has_fecf.value ? gr::ccsds::kFecfSize : 0UZ);
        if (frame_length.value <= overhead) {
            throw gr::exception(std::format("frame_length {} leaves no room for a data field once the fixed overhead of {} octets is removed", frame_length.value, overhead));
        }
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "TmFrameEncode", {{"frames", nFrames}, {"oversize payloads refused", nRefusedOversize}, {"headers refused", nRefusedHeader}, {"out-of-range pointer keys", nBadPointerKey}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record          = inSpan[consumed];
            const std::size_t            tfshOctets      = secondary_header.value.empty() ? 0UZ : gr::ccsds::kTmSecondaryHeaderIdSize + secondary_header.value.size();
            const std::size_t            overhead        = gr::ccsds::kTmPrimaryHeaderSize + tfshOctets + (has_ocf.value ? gr::ccsds::kOcfSize : 0UZ) + (has_fecf.value ? gr::ccsds::kFecfSize : 0UZ);
            const std::size_t            dataFieldLength = frame_length.value - overhead;
            if (record.signal_values.size() > dataFieldLength) {
                ++nRefusedOversize;
                continue;
            }

            gr::ccsds::TmPrimaryHeader header{};
            header.version                          = 0U;
            header.spacecraft_id                    = static_cast<std::uint16_t>(spacecraft_id.value);
            header.virtual_channel                  = static_cast<std::uint8_t>(virtual_channel.value);
            header.ocf_present                      = has_ocf.value;
            header.master_frame_count               = _mcCount;
            header.vc_frame_count                   = _vcCount;
            header.secondary_header                 = !secondary_header.value.empty();
            header.sync_flag                        = false;
            header.packet_order                     = false;
            header.segment_length_id                = 3U;
            const std::optional<gr::Size_t> pointer = detail::readSize(detail::metaOf(record), "ccsds_first_header_pointer");
            if (pointer.has_value() && *pointer > gr::Size_t{gr::ccsds::kFhpNoPacketStart}) {
                ++nBadPointerKey; // a value the eleven-bit field cannot hold says nothing about the zone and reads as absent
            }
            const bool havePointer      = pointer.has_value() && *pointer <= gr::Size_t{gr::ccsds::kFhpNoPacketStart};
            header.first_header_pointer = havePointer ? static_cast<std::uint16_t>(*pointer) : gr::ccsds::kFhpNoPacketStart;

            DataSet<std::uint8_t> frame;
            frame.signal_values.resize(frame_length.value, std::uint8_t{0U});
            if (gr::ccsds::writeTmPrimaryHeader(header, std::span<std::uint8_t>(frame.signal_values)) != gr::ccsds::WriteStatus::ok) {
                ++nRefusedHeader; // the block drops and counts a record whose header cannot be written, and no frame goes out with a zeroed header
                continue;
            }
            std::size_t at = gr::ccsds::kTmPrimaryHeaderSize;
            if (header.secondary_header) {
                gr::ccsds::TmSecondaryHeaderId shId{.version = 0U, .length = static_cast<std::uint8_t>(secondary_header.value.size() - 1UZ)};
                if (gr::ccsds::writeTmSecondaryHeaderId(shId, std::span<std::uint8_t>(frame.signal_values).subspan(at)) != gr::ccsds::WriteStatus::ok) {
                    ++nRefusedHeader;
                    continue;
                }
                std::ranges::copy(secondary_header.value, frame.signal_values.begin() + static_cast<std::ptrdiff_t>(at + gr::ccsds::kTmSecondaryHeaderIdSize));
                at += tfshOctets;
            }
            std::ranges::copy(record.signal_values, frame.signal_values.begin() + static_cast<std::ptrdiff_t>(at));
            if (record.signal_values.size() < dataFieldLength) {
                _fill.next(std::span<std::uint8_t>(frame.signal_values).subspan(at + record.signal_values.size(), dataFieldLength - record.signal_values.size()));
            }
            at += dataFieldLength;
            if (has_ocf.value) {
                at += gr::ccsds::kOcfSize; // left zeroed, since the block synthesizes no CLCW content
            }
            static_cast<void>(at);

            detail::startRecord(record, frame, "ccsds");

            ++nFrames;
            outSpan[made] = std::move(frame);
            ++made;

            _vcCount = static_cast<std::uint8_t>((_vcCount + 1U) % gr::ccsds::kTmCountModulus);
            _mcCount = static_cast<std::uint8_t>((_mcCount + 1U) % gr::ccsds::kTmCountModulus);
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::AosFrameEncode)

/*!
@brief Encodes one data field per record as one whole AOS transfer frame.

The block owns `vc_frame_count`, a 24-bit counter. With `vcfc_cycle_use` set, the four-bit cycle field increments
when the counter wraps (732.0-B-4 4.1.2.5.5.2). The count follows the same reasoning as those of `TmFrameEncode`.
The block pads a packet zone shorter than the frame's with the pseudo-noise fill of 4.1.4.1.5.2. The fill generator
runs across frames and does not restart per frame. The M_PDU pointer is written as the input record's metadata gave
it. A pointer key too wide for the field's eleven bits reads as absent and is counted in `nBadPointerKey`.
*/
struct AosFrameEncode : Block<AosFrameEncode> {
    using Description = Doc<"Encodes one data field per record as a whole AOS transfer frame with its own virtual channel frame count (732.0-B-4 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "frame_length", Doc<"required total octets of each produced frame">, Visible>                           frame_length{0U};
    Annotated<gr::Size_t, "spacecraft_id", Doc<"required eight-bit spacecraft identifier">, Visible>                              spacecraft_id{detail::kUnset};
    Annotated<gr::Size_t, "virtual_channel", Doc<"required six-bit virtual channel identifier">, Visible>                         virtual_channel{detail::kUnset};
    Annotated<bool, "has_fhec", Doc<"write the optional 2-octet frame header error control, making the primary header 8 octets">> has_fhec           = false;
    Annotated<gr::Size_t, "insert_zone_length", Doc<"octets of the fixed insert zone, written as zero">>                          insert_zone_length = 0U;
    Annotated<bool, "has_ocf", Doc<"reserve four OCF octets, written as a zeroed CLCW">>                                          has_ocf            = false;
    Annotated<bool, "has_fecf", Doc<"reserve two unwritten trailing octets for the FECF">>                                        has_fecf           = false;
    Annotated<std::string, "data_unit", Doc<"required data unit, 'm_pdu', 'b_pdu', 'vca_sdu' or 'idle'">, Visible>                data_unit{};
    Annotated<bool, "replay", Doc<"the replay flag, 4.1.2.5.2">>                                                                  replay         = false;
    Annotated<bool, "vcfc_cycle_use", Doc<"whether the four-bit cycle field extends the count to 28 bits">>                       vcfc_cycle_use = false;

    GR_MAKE_REFLECTABLE(AosFrameEncode, in, out, frame_length, spacecraft_id, virtual_channel, has_fhec, insert_zone_length, has_ocf, has_fecf, data_unit, replay, vcfc_cycle_use);

    std::uint64_t nFrames          = 0ULL;
    std::uint64_t nRefusedOversize = 0ULL;
    std::uint64_t nRefusedHeader   = 0ULL;
    std::uint64_t nBadPointerKey   = 0ULL;

    bool               _configured          = false;
    bool               _isMpdu              = false;
    std::size_t        _primaryHeaderOctets = 0UZ;
    std::size_t        _dataFieldLength     = 0UZ;
    std::uint32_t      _vcCount             = 0U;
    std::uint8_t       _cycle               = 0U;
    gr::ccsds::OidFill _fill{};

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (frame_length.value == 0U) {
            throw gr::exception("frame_length is required and has no default");
        }
        if (spacecraft_id.value == detail::kUnset || spacecraft_id.value > frames_detail::kAosMaxScid) {
            throw gr::exception(std::format("spacecraft_id is required and must be 0 to {}: a transmitter must know whose frame it is sending, and 4.1.2.2.3's field is eight bits wide", frames_detail::kAosMaxScid));
        }
        if (virtual_channel.value == detail::kUnset || virtual_channel.value > 63U) {
            throw gr::exception("virtual_channel is required and must be 0 to 63");
        }
        if (data_unit.value != "m_pdu" && data_unit.value != "b_pdu" && data_unit.value != "vca_sdu" && data_unit.value != "idle") {
            throw gr::exception(std::format("data_unit must be 'm_pdu', 'b_pdu', 'vca_sdu' or 'idle', got '{}'", data_unit.value));
        }
        _isMpdu                 = data_unit.value == "m_pdu";
        _primaryHeaderOctets    = gr::ccsds::aosPrimaryHeaderOctets(has_fhec.value);
        const std::size_t fixed = _primaryHeaderOctets + std::size_t{insert_zone_length.value} + (has_ocf.value ? gr::ccsds::kOcfSize : 0UZ) + (has_fecf.value ? gr::ccsds::kFecfSize : 0UZ);
        if (frame_length.value <= fixed || (_isMpdu && frame_length.value < fixed + gr::ccsds::kMpduHeaderSize)) {
            throw gr::exception(std::format("frame_length {} leaves no room for a data field once the fixed overhead of {} octets is removed", frame_length.value, fixed));
        }
        _dataFieldLength = frame_length.value - fixed;
        _configured      = true;
    }

    void stop() { detail::reportCounters(*this, "AosFrameEncode", {{"frames", nFrames}, {"oversize payloads refused", nRefusedOversize}, {"headers refused", nRefusedHeader}, {"out-of-range pointer keys", nBadPointerKey}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record    = inSpan[consumed];
            const std::size_t            zoneLimit = _isMpdu ? _dataFieldLength - gr::ccsds::kMpduHeaderSize : _dataFieldLength;
            if (record.signal_values.size() > zoneLimit) {
                ++nRefusedOversize;
                continue;
            }

            gr::ccsds::AosPrimaryHeader header{};
            header.version                    = 1U;
            header.spacecraft_id              = static_cast<std::uint8_t>(spacecraft_id.value);
            header.virtual_channel            = static_cast<std::uint8_t>(virtual_channel.value);
            header.vc_frame_count             = _vcCount;
            header.replay                     = replay.value;
            header.vc_count_cycle_used        = vcfc_cycle_use.value;
            header.reserved                   = 0U;
            header.vc_count_cycle             = vcfc_cycle_use.value ? _cycle : 0U;
            header.has_fhec                   = has_fhec.value;
            header.frame_header_error_control = 0U;

            DataSet<std::uint8_t> frame;
            frame.signal_values.resize(frame_length.value, std::uint8_t{0U});
            if (gr::ccsds::writeAosPrimaryHeader(header, std::span<std::uint8_t>(frame.signal_values)) != gr::ccsds::WriteStatus::ok) {
                ++nRefusedHeader; // the block drops and counts a record whose header cannot be written, and no frame goes out with a zeroed header
                continue;
            }
            std::size_t at = _primaryHeaderOctets + std::size_t{insert_zone_length.value};
            if (_isMpdu) {
                const std::optional<gr::Size_t> pointer = detail::readSize(detail::metaOf(record), "ccsds_first_header_pointer");
                if (pointer.has_value() && *pointer > gr::Size_t{gr::ccsds::kFhpNoPacketStart}) {
                    ++nBadPointerKey; // a value the eleven-bit field cannot hold says nothing about the zone and reads as absent
                }
                const bool            havePointer = pointer.has_value() && *pointer <= gr::Size_t{gr::ccsds::kFhpNoPacketStart};
                gr::ccsds::MpduHeader mpdu{.reserved = 0U, .first_header_pointer = havePointer ? static_cast<std::uint16_t>(*pointer) : gr::ccsds::kFhpNoPacketStart};
                if (gr::ccsds::writeMpduHeader(mpdu, std::span<std::uint8_t>(frame.signal_values).subspan(at)) != gr::ccsds::WriteStatus::ok) {
                    ++nRefusedHeader;
                    continue;
                }
                at += gr::ccsds::kMpduHeaderSize;
            }
            std::ranges::copy(record.signal_values, frame.signal_values.begin() + static_cast<std::ptrdiff_t>(at));
            if (record.signal_values.size() < zoneLimit) {
                _fill.next(std::span<std::uint8_t>(frame.signal_values).subspan(at + record.signal_values.size(), zoneLimit - record.signal_values.size()));
            }

            detail::startRecord(record, frame, "ccsds");

            ++nFrames;
            outSpan[made] = std::move(frame);
            ++made;

            const std::uint32_t next = (_vcCount + 1U) % gr::ccsds::kAosCountModulus;
            if (next < _vcCount && vcfc_cycle_use.value) {
                _cycle = static_cast<std::uint8_t>((_cycle + 1U) % 16U);
            }
            _vcCount = next;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ccsds::TcFrameEncode)

/*!
@brief Encodes one data field per record as one whole TC transfer frame.

`frame_sequence_number` is a setting. The block does not increment it. 232.0-B-4 4.1.2.8 NOTE 1 places its
assignment in the COP-1 procedures, a link-layer engine this module does not build. The TM and AOS encoders own
their frame counts. 4.1.1.1 b) makes the data field mandatory. An empty payload therefore has no encoding. The block
refuses it and counts it in `nRefusedEmpty`. It emits no five-octet frame for a decoder to drop.
*/
struct TcFrameEncode : Block<TcFrameEncode> {
    using Description = Doc<"Encodes one data field per record as a whole, self-describing TC transfer frame (232.0-B-4 4.1)">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "spacecraft_id", Doc<"required ten-bit spacecraft identifier">, Visible>        spacecraft_id{detail::kUnset};
    Annotated<gr::Size_t, "virtual_channel", Doc<"required six-bit virtual channel identifier">, Visible> virtual_channel{detail::kUnset};
    Annotated<bool, "has_fecf", Doc<"reserve two unwritten trailing octets for the FECF">>                has_fecf              = false;
    Annotated<bool, "bypass_flag", Doc<"'0' Type-A, '1' Type-B, 4.1.2.3.1.2">>                            bypass_flag           = false;
    Annotated<bool, "control_command_flag", Doc<"'0' data, '1' control commands, 4.1.2.3.2.2">>           control_command_flag  = false;
    Annotated<gr::Size_t, "frame_sequence_number", Doc<"fixed frame sequence number N(S), 4.1.2.8">>      frame_sequence_number = 0U;

    GR_MAKE_REFLECTABLE(TcFrameEncode, in, out, spacecraft_id, virtual_channel, has_fecf, bypass_flag, control_command_flag, frame_sequence_number);

    std::uint64_t nFrames          = 0ULL;
    std::uint64_t nRefusedOversize = 0ULL;
    std::uint64_t nRefusedEmpty    = 0ULL;
    std::uint64_t nRefusedHeader   = 0ULL;

    bool _configured = false;

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (spacecraft_id.value == detail::kUnset || spacecraft_id.value > 1023U) {
            throw gr::exception("spacecraft_id is required and must be 0 to 1023");
        }
        if (virtual_channel.value == detail::kUnset || virtual_channel.value > 63U) {
            throw gr::exception("virtual_channel is required and must be 0 to 63");
        }
        _configured = true;
    }

    void stop() { detail::reportCounters(*this, "TcFrameEncode", {{"frames", nFrames}, {"oversize payloads refused", nRefusedOversize}, {"empty payloads refused", nRefusedEmpty}, {"headers refused", nRefusedHeader}}); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record      = inSpan[consumed];
            const std::size_t            fecfOctets  = has_fecf.value ? gr::ccsds::kFecfSize : 0UZ;
            const std::size_t            totalOctets = gr::ccsds::kTcPrimaryHeaderSize + record.signal_values.size() + fecfOctets;
            if (record.signal_values.empty()) {
                ++nRefusedEmpty;
                continue;
            }
            if (totalOctets > 1024UZ) {
                ++nRefusedOversize;
                continue;
            }

            gr::ccsds::TcPrimaryHeader header{};
            header.version         = 0U;
            header.bypass          = bypass_flag.value;
            header.control_command = control_command_flag.value;
            header.reserved        = 0U;
            header.spacecraft_id   = static_cast<std::uint16_t>(spacecraft_id.value);
            header.virtual_channel = static_cast<std::uint8_t>(virtual_channel.value);
            header.frame_length    = static_cast<std::uint16_t>(totalOctets - 1UZ);
            header.sequence_number = static_cast<std::uint8_t>(frame_sequence_number.value);

            DataSet<std::uint8_t> frame;
            frame.signal_values.resize(totalOctets, std::uint8_t{0U});
            if (gr::ccsds::writeTcPrimaryHeader(header, std::span<std::uint8_t>(frame.signal_values)) != gr::ccsds::WriteStatus::ok) {
                ++nRefusedHeader; // the block drops and counts a record whose header cannot be written, and no frame goes out with a zeroed header
                continue;
            }
            std::ranges::copy(record.signal_values, frame.signal_values.begin() + static_cast<std::ptrdiff_t>(gr::ccsds::kTcPrimaryHeaderSize));

            detail::startRecord(record, frame, "ccsds");

            ++nFrames;
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
};

} // namespace gr::blocks::ccsds

#endif // GNURADIO_CCSDS_FRAMES_HPP
