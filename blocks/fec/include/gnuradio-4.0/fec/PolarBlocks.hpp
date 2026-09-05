#ifndef GNURADIO_FEC_POLAR_BLOCKS_HPP
#define GNURADIO_FEC_POLAR_BLOCKS_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/fec/Aff3ctWall.hpp>

/**
 * The record-native adapters over AFF3CT's Polar family.
 *
 * A polar code is `n` a power of two, `k` bits carried, and a rule that says which of the `n`
 * synthetic channels are frozen. Two rules are offered: `ga`, the Gaussian approximation
 * evaluated at a stated design `Eb/N0`, and `5g`, the reliability sequence the release ships.
 * Three decoders are offered: `sc`, the successive cancellation the family is defined by; `scl`,
 * its list form; and `ca_scl`, the list form that uses a CRC to choose among the survivors.
 *
 * **The CRC is this tree's.** `ca_scl` asks of each surviving path whether its information bits
 * check, and it is `gr::digital::Crc` under the `crc_*` settings that answers, not AFF3CT's own
 * table — so one polynomial vocabulary serves the whole tree and a chain that computes a CRC
 * before this block and one that computes it inside agree by construction. The signature is
 * appended by `PolarEncode` and checked by `PolarDecode`, so a record carries `k - crc_width`
 * payload bits, and that payload must be a whole number of bytes because the kernel reads bytes.
 *
 * The carriers are the family's usual split: `PolarEncode` takes and `PolarDecode` publishes
 * `DataSet<std::uint8_t>` bit records, and `PolarDecode` takes a `DataSet<float>` of
 * log-likelihood ratios in this tree's sense — positive carries a one. AFF3CT's convention is the
 * opposite sign sense and the wall negates on the way in; the inversion is stated once and pinned
 * by a QA anchor.
 *
 * A record holds a whole number of frames, a misaligned record is a counted stated drop, and the
 * decode's account rides the record: `corrected_errors` gains the coded bits by which the frame
 * received and the frame re-encoded from the decode disagree, and `uncorrectable_errors` counts
 * the frames whose delivered answer fails the CRC — which under `ca_scl` is exactly the case where
 * no path survived and the decoder fell back to its best one. Under `sc` and `scl` there is no
 * CRC and therefore no refusal to report, which is `ViterbiDecode`'s position for the same reason.
 */
namespace gr::blocks::fec {

GR_REGISTER_BLOCK(gr::blocks::fec::PolarEncode)

/*!
@brief Polar encode: payload records in, codeword records out, the CRC appended where one is set.

Each record carries a whole number of payload frames of `k - crc_width` bits. Where `crc_width` is
nonzero the block appends the signature, making the `k` bits the encoder takes; where it is zero
the payload is the whole of `k`.

An encoder has no status to report, so the record's metadata crosses unchanged; its signal name
and its single-map shape follow it, and the output record's extent names its own length.
*/
struct PolarEncode : Block<PolarEncode> {
    using Description = Doc<"Polar encode: payload records to codeword records under the construction the 'n', 'k' and 'frozen_construction' settings name, with this tree's CRC appended where one is set">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "n", Doc<"codeword bits, a power of two">, Visible>                                                                                             n                    = 0U;
    Annotated<gr::Size_t, "k", Doc<"bits the encoder takes, the CRC bits included">, Visible>                                                                             k                    = 0U;
    Annotated<std::string, "frozen_construction", Doc<"'ga' for the Gaussian approximation at 'design_snr_db', or '5g' for the release's reliability sequence">, Visible> frozen_construction  = std::string("ga");
    Annotated<double, "design_snr_db", Unit<"dB">, Doc<"the Eb/N0 the Gaussian approximation freezes the channels at">>                                                   design_snr_db        = 2.5;
    Annotated<gr::Size_t, "crc_width", Unit<"bit">, Doc<"signature bits appended to the payload; 0 for no CRC, and only decoder 'ca_scl' uses one">, Visible>             crc_width            = 0U;
    Annotated<std::uint64_t, "crc_poly", Doc<"generator polynomial, MSB-first, x^w omitted">>                                                                             crc_poly             = 0ULL;
    Annotated<std::uint64_t, "crc_initial_value", Doc<"seed, in the unreflected domain">>                                                                                 crc_initial_value    = 0ULL;
    Annotated<std::uint64_t, "crc_final_xor", Doc<"XORed into the result last">>                                                                                          crc_final_xor        = 0ULL;
    Annotated<bool, "crc_input_reflected", Doc<"each message byte enters LSB first">>                                                                                     crc_input_reflected  = false;
    Annotated<bool, "crc_result_reflected", Doc<"the register is bit-reversed before crc_final_xor">>                                                                     crc_result_reflected = false;

    GR_MAKE_REFLECTABLE(PolarEncode, in, out, n, k, frozen_construction, design_snr_db, crc_width, crc_poly, crc_initial_value, crc_final_xor, crc_input_reflected, crc_result_reflected);

    std::optional<wall::PolarCodec> _codec{};
    bool                            _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nFrames         = 0ULL; ///< codewords those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a whole number of frames

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        wall::PolarSettings settings;
        settings.n                  = static_cast<std::size_t>(n.value);
        settings.k                  = static_cast<std::size_t>(k.value);
        settings.frozenConstruction = frozen_construction.value;
        settings.designSnrDb        = design_snr_db.value;
        // The encode side runs no decoder; it is told the aided form only when a signature is set,
        // because that is what makes the wall append one.
        settings.decoder            = (crc_width.value == 0U) ? std::string("sc") : std::string("ca_scl");
        settings.listSize           = 1UZ;
        settings.crcWidth           = static_cast<std::size_t>(crc_width.value);
        settings.crcPolynomial      = crc_poly.value;
        settings.crcInitialValue    = crc_initial_value.value;
        settings.crcFinalXor        = crc_final_xor.value;
        settings.crcInputReflected  = crc_input_reflected.value;
        settings.crcResultReflected = crc_result_reflected.value;
        _codec.emplace(settings);
        _configured = true; // only reached when the settings named a construction the wall accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"frames", nFrames}, {"records refused", nRecordsRefused}}};
        detail::reportFrames("gr::blocks::fec::PolarEncode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than coding under a construction nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const std::size_t payload = _codec->payloadBits();
        const std::size_t coded   = _codec->codedBits();

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            bits   = record.signal_values.size();
            if (bits == 0UZ || bits % payload != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t frames = bits / payload;

            DataSet<std::uint8_t> word;
            word.signal_values.resize(frames * coded);
            for (std::size_t f = 0UZ; f < frames; ++f) {
                _codec->encode(std::span<const std::uint8_t>(record.signal_values).subspan(f * payload, payload), std::span<std::uint8_t>(word.signal_values).subspan(f * coded, coded));
            }
            detail::carryFrame(word, record);

            ++nRecords;
            nFrames += frames;
            outSpan[made] = std::move(word);
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

GR_REGISTER_BLOCK(gr::blocks::fec::PolarDecode)

/*!
@brief Polar decode: soft codeword records in, payload records out, with the channel's account and
the CRC's verdict in metadata.

Each record carries a whole number of `n`-value frames and each yields `k - crc_width` payload
bits. A record whose length fails that test is dropped and counted exactly as PolarEncode drops
one.

`decoder` selects successive cancellation, its list form, or the CRC-aided list form. The aided
form is the one the family's published performance belongs to, and it is the one that can refuse:
where no path in the list checks, the decoder falls back to its best path and the block counts the
frame in `uncorrectable_errors`, the payload still delivered. The list forms this release ships in
a fast variant are broken at the pinned tag, so the wrap takes the naive ones, which are the
reference implementations.
*/
struct PolarDecode : Block<PolarDecode> {
    using Description = Doc<"Polar decode: soft codeword records to payload records, the channel's account and this tree's CRC verdict accumulating in metadata">;

    PortIn<DataSet<float>, Async>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "n", Doc<"codeword bits, a power of two">, Visible>                                                                                             n                    = 0U;
    Annotated<gr::Size_t, "k", Doc<"bits the encoder took, the CRC bits included">, Visible>                                                                              k                    = 0U;
    Annotated<std::string, "frozen_construction", Doc<"'ga' for the Gaussian approximation at 'design_snr_db', or '5g' for the release's reliability sequence">, Visible> frozen_construction  = std::string("ga");
    Annotated<double, "design_snr_db", Unit<"dB">, Doc<"the Eb/N0 the Gaussian approximation freezes the channels at">>                                                   design_snr_db        = 2.5;
    Annotated<std::string, "decoder", Doc<"'sc', 'scl' or 'ca_scl'">, Visible>                                                                                            decoder              = std::string("sc");
    Annotated<gr::Size_t, "list_size", Doc<"paths a list decoder carries">, Visible>                                                                                      list_size            = 8U;
    Annotated<gr::Size_t, "crc_width", Unit<"bit">, Doc<"signature bits the encoder appended; required by 'ca_scl' and refused by the others">, Visible>                  crc_width            = 0U;
    Annotated<std::uint64_t, "crc_poly", Doc<"generator polynomial, MSB-first, x^w omitted">>                                                                             crc_poly             = 0ULL;
    Annotated<std::uint64_t, "crc_initial_value", Doc<"seed, in the unreflected domain">>                                                                                 crc_initial_value    = 0ULL;
    Annotated<std::uint64_t, "crc_final_xor", Doc<"XORed into the result last">>                                                                                          crc_final_xor        = 0ULL;
    Annotated<bool, "crc_input_reflected", Doc<"each message byte enters LSB first">>                                                                                     crc_input_reflected  = false;
    Annotated<bool, "crc_result_reflected", Doc<"the register is bit-reversed before crc_final_xor">>                                                                     crc_result_reflected = false;

    GR_MAKE_REFLECTABLE(PolarDecode, in, out, n, k, frozen_construction, design_snr_db, decoder, list_size, crc_width, crc_poly, crc_initial_value, crc_final_xor, crc_input_reflected, crc_result_reflected);

    std::optional<wall::PolarCodec> _codec{};
    bool                            _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords             = 0ULL; ///< records published on `out`
    std::uint64_t nFrames              = 0ULL; ///< codewords those records carried
    std::uint64_t nRecordsRefused      = 0ULL; ///< records whose length was not a whole number of frames
    std::uint64_t nCorrectedErrors     = 0ULL; ///< coded bits between the frames received and the frames decoded
    std::uint64_t nUncorrectableFrames = 0ULL; ///< frames whose delivered payload failed the CRC

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        wall::PolarSettings settings;
        settings.n                  = static_cast<std::size_t>(n.value);
        settings.k                  = static_cast<std::size_t>(k.value);
        settings.frozenConstruction = frozen_construction.value;
        settings.designSnrDb        = design_snr_db.value;
        settings.decoder            = decoder.value;
        settings.listSize           = static_cast<std::size_t>(list_size.value);
        settings.crcWidth           = static_cast<std::size_t>(crc_width.value);
        settings.crcPolynomial      = crc_poly.value;
        settings.crcInitialValue    = crc_initial_value.value;
        settings.crcFinalXor        = crc_final_xor.value;
        settings.crcInputReflected  = crc_input_reflected.value;
        settings.crcResultReflected = crc_result_reflected.value;
        _codec.emplace(settings);
        _configured = true; // only reached when the settings named a construction the wall accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 5UZ> counters{{{"records", nRecords}, {"frames", nFrames}, {"records refused", nRecordsRefused}, //
            {"corrected errors", nCorrectedErrors}, {"uncorrectable frames", nUncorrectableFrames}}};
        detail::reportFrames("gr::blocks::fec::PolarDecode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than decoding under a construction nobody chose
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const std::size_t payload = _codec->payloadBits();
        const std::size_t coded   = _codec->codedBits();
        const bool        aided   = crc_width.value != 0U;

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<float>& record = inSpan[consumed];
            const std::size_t     values = record.signal_values.size();
            if (values == 0UZ || values % coded != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t frames = values / coded;

            DataSet<std::uint8_t> info;
            info.signal_values.resize(frames * payload);
            gr::Size_t corrected     = 0U;
            gr::Size_t uncorrectable = 0U;
            for (std::size_t f = 0UZ; f < frames; ++f) {
                const wall::DecodeReport report = _codec->decode(std::span<const float>(record.signal_values).subspan(f * coded, coded), std::span<std::uint8_t>(info.signal_values).subspan(f * payload, payload));
                corrected += static_cast<gr::Size_t>(report.correctedErrors);
                uncorrectable += report.refused ? 1U : 0U;
            }
            detail::carryFrame(info, record);

            property_map& map       = info.meta_information[0UZ];
            map["corrected_errors"] = gr::Size_t{detail::metaOrZero<gr::Size_t>(map, "corrected_errors", 0U) + corrected};
            if (aided) { // without a CRC there is no refusal, and a key that could only ever be zero would misstate that
                map["uncorrectable_errors"] = gr::Size_t{detail::metaOrZero<gr::Size_t>(map, "uncorrectable_errors", 0U) + uncorrectable};
            }

            ++nRecords;
            nFrames += frames;
            nCorrectedErrors += corrected;
            nUncorrectableFrames += uncorrectable;
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

} // namespace gr::blocks::fec

#endif // GNURADIO_FEC_POLAR_BLOCKS_HPP
