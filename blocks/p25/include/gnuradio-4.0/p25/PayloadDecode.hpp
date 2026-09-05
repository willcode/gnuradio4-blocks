#ifndef GNURADIO_P25_PAYLOAD_DECODE_HPP
#define GNURADIO_P25_PAYLOAD_DECODE_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/p25/ImbeFrame.hpp>
#include <gnuradio-4.0/p25/PayloadLayer.hpp>

namespace gr::blocks::p25 {

GR_REGISTER_BLOCK(gr::blocks::p25::P25PayloadDecode)

/*!
@brief P25 Phase 1 payload decode: frame records in, decoded-payload records out, one for one.

Record-domain and stateless in stream time — every input record resolves on its own, in order,
against the frame dibits it carries. For a header or voice frame the field-protecting
Reed-Solomon block decodes and its contents land in metadata: link control's talkgroup and
source unit, the encryption sync's algorithm and key identifiers, the header's talkgroup and
manufacturer id. `crc_ok` mirrors the outer code's verdict; `corrected_errors` and
`uncorrectable_errors` carry the inner codes' work and refusals; `rs_errors` the outer's.

A voice frame's record carries its speech parameters as data — the nine codewords' parameter
words u0..u7 in transmitted order, 72 values — whether or not the outer code succeeded. That
code protects only the identity fields; the speech has its own Golay and Hamming codes inside
each codeword (their per-codeword u0 repair counts ride `u0_errors`, the total `voice_errors`).
A frame with no voice, or one that arrived truncated, passes through with empty data and its
state intact.

Each voice record also carries `voice_sample_start`, its position on the 8000 Hz voice
clock. The interleave spreads every codeword across the whole frame, so a frame's speech
cannot begin before its last transmitted symbol has passed; the position is therefore
`ceil(frame_end_dibits * 5/3)`, the exact 8000/4800 rate ratio applied to the frame's end.
A vocoder can place its output from this value alone, without knowing the framing that
produced it.

The clear-state window is the one piece of cross-record state, because it follows the
standard's frame order rather than application policy: the encryption state rides in the
header that opens a transmission and in the second of each pair of voice frames, while the
first of each pair carries link control instead — so a receiver always vocodes some frames
using a state decoded up to two frames earlier. Four voice frames on the dibit clock is the
window: longer than the two-frame gap the frame order produces, longer than the three a lost
second frame produces, far shorter than the hang time between transmissions. Outside it the
state is unknown, and unknown traffic is not treated as clear. Each voice record carries the
verdict as `clear`, beside the `algid` and `keyid` it was judged by.
*/
struct P25PayloadDecode : Block<P25PayloadDecode> {
    using Description = Doc<"P25 Phase 1 payload decode: frame records to records carrying identity metadata and voice parameter words; the clear-state window rides the standard's frame order">;

    PortIn<DataSet<std::uint8_t>, Async>   in;
    PortOut<DataSet<std::uint16_t>, Async> out;

    GR_MAKE_REFLECTABLE(P25PayloadDecode, in, out);

    //! How long a decoded encryption state authorizes vocoding, in transmitted symbols.
    static constexpr std::uint64_t kClearStateLifeDibits = 4ULL * 864ULL;

    gr::p25::P25PayloadLayer _layer{}; //!< decodePayload and the pad-corruption counter; its stream half is unused

    bool          _haveEncryption  = false;
    std::uint8_t  _algid           = gr::p25::kAlgidClear;
    std::uint16_t _keyid           = 0U;
    std::uint64_t _encryptionDibit = 0ULL;

    std::uint64_t _messages  = 0ULL;
    std::uint64_t _decoded   = 0ULL;
    std::uint64_t _fecFailed = 0ULL;

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t done = 0UZ;
        for (; done < inSpan.size() && done < outSpan.size(); ++done) {
            outSpan[done] = resolve(inSpan[done]);
        }
        std::ignore = inSpan.consume(done);
        outSpan.publish(done);
        if (done == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    template<typename V>
    [[nodiscard]] static V metaOr(const property_map& map, const char* key, V fallback) {
        if (const auto entry = map.find(property_map::key_type(key)); entry != map.end()) {
            return entry->second.value_or(V(fallback));
        }
        return fallback;
    }

    [[nodiscard]] DataSet<std::uint16_t> resolve(const DataSet<std::uint8_t>& frameRecord) {
        DataSet<std::uint16_t> record;
        record.signal_names.emplace_back("p25");
        record.timing_events.resize(1UZ);
        record.meta_information.resize(1UZ);
        property_map& map = record.meta_information[0UZ];
        if (!frameRecord.meta_information.empty()) {
            map = frameRecord.meta_information[0UZ]; // the frame's facts carry through
        }
        ++_messages;

        gr::p25::P25Frame frame;
        frame.dibit_index = metaOr<std::uint64_t>(map, "sample_start", 0ULL);
        frame.nac         = static_cast<std::uint16_t>(metaOr<gr::Size_t>(map, "nac", 0U));
        frame.duid        = static_cast<std::uint8_t>(metaOr<gr::Size_t>(map, "duid", 0U));
        frame.kind        = gr::p25::duidKind(frame.duid);
        frame.sync_errors = metaOr<gr::Size_t>(map, "sync_errors", 0U);
        frame.bch_errors  = metaOr<gr::Size_t>(map, "corrected_errors", 0U);
        frame.parity_ok   = metaOr<bool>(map, "parity_ok", false);

        const std::size_t length = gr::p25::duidTransmittedDibits(frame.duid);
        if (frameRecord.signal_values.size() < length || length == 0UZ) {
            record.extents.push_back(0);
            return record; // truncated, length-unknown or no-payload: metadata rides on
        }

        const gr::p25::P25Message message = _layer.decodePayload(frame, frameRecord.signal_values.data());

        const char* state = "no_payload";
        switch (message.state) {
        case gr::p25::P25PayloadState::Decoded:
            state = "decoded";
            ++_decoded;
            break;
        case gr::p25::P25PayloadState::FecFailed:
            state = "fec_failed";
            ++_fecFailed;
            break;
        default: break;
        }
        map["state"]                = std::string(state);
        map["crc_ok"]               = message.state == gr::p25::P25PayloadState::Decoded;
        map["corrected_errors"]     = gr::Size_t{frame.bch_errors + message.inner_corrected};
        map["uncorrectable_errors"] = gr::Size_t{message.inner_refused};
        map["rs_errors"]            = gr::Size_t{message.rs_errors};

        if (message.has_link_control && message.link_control.has_addresses) {
            map["talkgroup"]   = static_cast<gr::Size_t>(message.link_control.talkgroup);
            map["source_unit"] = gr::Size_t{message.link_control.source_unit};
        }
        if (message.has_link_control) {
            map["lco"]  = static_cast<gr::Size_t>(message.link_control.lco);
            map["mfid"] = static_cast<gr::Size_t>(message.link_control.mfid);
        }
        if (message.has_header_fields) {
            map["talkgroup"] = static_cast<gr::Size_t>(message.header_talkgroup);
            map["mfid"]      = static_cast<gr::Size_t>(message.header_mfid);
        }
        if (message.has_encryption_sync && message.decoded()) {
            _haveEncryption  = true;
            _algid           = message.encryption_sync.algid;
            _keyid           = message.encryption_sync.keyid;
            _encryptionDibit = frame.dibit_index;
        }
        if (_haveEncryption) {
            map["algid"] = static_cast<gr::Size_t>(_algid);
            map["keyid"] = static_cast<gr::Size_t>(_keyid);
        }

        const bool voice = frame.duid == static_cast<std::uint8_t>(gr::p25::P25Duid::Ldu1) //
                           || frame.duid == static_cast<std::uint8_t>(gr::p25::P25Duid::Ldu2);
        if (!voice) {
            record.extents.push_back(0);
            return record;
        }

        // The outer code's verdict does not gate the speech: its own codes sit inside each
        // codeword, decoded here so the record carries what the vocoder reads.
        std::vector<gr::Size_t> u0Errors;
        u0Errors.reserve(gr::p25::kImbeCodewordsPerFrame);
        gr::Size_t voiceErrors = 0U;
        record.signal_values.reserve(gr::p25::kImbeCodewordsPerFrame * gr::p25::kImbeParameterWords);
        for (std::size_t n = 0UZ; n < gr::p25::kImbeCodewordsPerFrame; ++n) {
            const gr::p25::ImbeParameters parameters = gr::p25::imbeDecodeVoiceCodeword(frameRecord.signal_values.data(), n);
            for (const std::uint16_t word : parameters.u) {
                record.signal_values.push_back(word);
            }
            u0Errors.push_back(gr::Size_t{parameters.u0_errors});
            voiceErrors += gr::Size_t{parameters.errors};
        }
        record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));

        const bool clear = _haveEncryption && _algid == gr::p25::kAlgidClear //
                           && frame.dibit_index >= _encryptionDibit          //
                           && frame.dibit_index - _encryptionDibit <= kClearStateLifeDibits;
        map["clear"]        = clear;
        map["codewords"]    = static_cast<gr::Size_t>(gr::p25::kImbeCodewordsPerFrame);
        map["u0_errors"]    = u0Errors;
        map["voice_errors"] = voiceErrors;
        // ceil(frame_end * 5/3): the frame's position translated onto the 8000 Hz voice clock.
        const std::uint64_t voiceStart = ((frame.dibit_index + length) * 5U + 2U) / 3U;
        map["voice_sample_start"]      = voiceStart;
        return record;
    }
};

} // namespace gr::blocks::p25

#endif // GNURADIO_P25_PAYLOAD_DECODE_HPP
