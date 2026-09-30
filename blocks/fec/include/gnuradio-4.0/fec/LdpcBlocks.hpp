#ifndef GNURADIO_FEC_LDPC_BLOCKS_HPP
#define GNURADIO_FEC_LDPC_BLOCKS_HPP

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
 * @brief Record blocks over AFF3CT's LDPC family.
 *
 * The code is a parity-check matrix and a belief-propagation schedule, both fixed at
 * configuration. `standard` names one of the matrices the pinned release ships, or `alist_path`
 * names one on disk. The other setting stays empty. Neither has a default. A matrix is the code,
 * and a default matrix would assume an interoperability nobody chose. The decoder objects are
 * built once and reused for every record. A settings change builds new ones in `rebuild()`.
 *
 * The encoder carries bits and the decoder carries soft values, as in every soft-decision family in
 * this module. `LdpcEncode` takes a `DataSet<std::uint8_t>` of information bits and publishes a
 * `DataSet<std::uint8_t>` of coded bits, one item per bit. `LdpcDecode` takes a `DataSet<float>` of
 * log-likelihood ratios in this module's sense and publishes the information bits. A positive value
 * carries a one, the magnitude is confidence, and zero is an erasure. A hard-decision receiver
 * presents its bits to the decoder as saturated values of that sign. No second port is needed.
 *
 * A record holds a whole number of frames. Its length must be a nonzero multiple of the code's
 * `k` on the way in and of its `n` on the way back. A record that fails that test is dropped,
 * counted in `nRecordsRefused` and reported at `stop()`. The next record is coded normally.
 *
 * This family can refuse a frame, and the error counts report it. A decode whose syndrome still
 * fails after the last iteration is counted in `uncorrectable_errors`. `corrected_errors` gains
 * the coded bits where the received word and the decoded word differ. Every other decoder in this
 * module gives the same account of the channel. The block emits the information estimate in both
 * cases. The counts say what it is worth, and no bit is zeroed or fabricated.
 */
namespace gr::blocks::fec {

GR_REGISTER_BLOCK(gr::blocks::fec::LdpcEncode)

/*!
@brief Encodes information-bit records into LDPC codeword records.

Each record carries a whole number of information frames of `k` bits, and each frame becomes one
`n`-bit codeword. An encoder has no status to report, and the record's metadata passes through
unchanged. Its signal name and its single-map shape carry over, and the output record's extent
names its own length.

The block builds the code's generator matrix from its parity-check matrix once, at configuration.
That step is the costly part of an LDPC encoder. See LdpcDecode for the counterpart.
*/
struct LdpcEncode : Block<LdpcEncode> {
    using Description = Doc<"Encodes information-bit records into LDPC codeword records under the matrix the 'standard' or 'alist_path' setting names">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "standard", Doc<"name of a parity-check matrix the pinned AFF3CT release ships">, Visible> standard{};
    Annotated<std::string, "alist_path", Doc<"path of a parity-check matrix in alist form">, Visible>                 alist_path{};

    GR_MAKE_REFLECTABLE(LdpcEncode, in, out, standard, alist_path);

    std::optional<wall::LdpcCodec> _codec{};
    bool                           _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nFrames         = 0ULL; ///< codewords those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a whole number of frames

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        wall::LdpcSettings settings;
        settings.standard  = standard.value;
        settings.alistPath = alist_path.value;
        // The encode side runs no decoder. The adapter holds its cheapest schedule beside the
        // encoder.
        settings.decoder    = std::string("min_sum");
        settings.iterations = 1UZ;
        _codec.emplace(settings);
        _configured = true; // only reached when the settings named a matrix the release or the disk holds
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"frames", nFrames}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::LdpcEncode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert until the settings name a matrix
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const std::size_t k = _codec->payloadBits();
        const std::size_t n = _codec->codedBits();

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            bits   = record.signal_values.size();
            if (bits == 0UZ || bits % k != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t frames = bits / k;

            DataSet<std::uint8_t> coded;
            coded.signal_values.resize(frames * n);
            for (std::size_t f = 0UZ; f < frames; ++f) {
                _codec->encode(std::span<const std::uint8_t>(record.signal_values).subspan(f * k, k), std::span<std::uint8_t>(coded.signal_values).subspan(f * n, n));
            }
            detail::carry(coded, record);

            ++nRecords;
            nFrames += frames;
            outSpan[made] = std::move(coded);
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

GR_REGISTER_BLOCK(gr::blocks::fec::LdpcDecode)

/*!
@brief Decodes soft LDPC codeword records into information-bit records, with the account and verdict in metadata.

Each record carries a whole number of `n`-value frames, and each frame yields `k` information bits.
A record whose length fails that test is dropped and counted as LdpcEncode drops one.

The values are log-likelihood ratios in the convention these blocks use. A positive value carries a
one, the magnitude is confidence, and zero is a pure erasure. AFF3CT's own convention has the
opposite sign, and the adapter negates each value on the way in. That inversion is stated in one
place and pinned by a QA anchor. A decode under the wrong sign returns the bitwise complement with
no other symptom.

The verdict travels with the record. `corrected_errors` gains the coded bits where the received
frame and the frame re-encoded from the decode differ. `uncorrectable_errors` gains the count of
frames whose syndrome still failed after the last iteration. Each count is added to the value the key
already held. Every other key passes through unchanged, and a record without a metadata map gains
one.
*/
struct LdpcDecode : Block<LdpcDecode> {
    using Description = Doc<"Decodes soft LDPC codeword records into information-bit records and accumulates the channel's account and the syndrome's verdict in metadata">;

    PortIn<DataSet<float>, Async>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "standard", Doc<"name of a parity-check matrix the pinned AFF3CT release ships">, Visible>            standard{};
    Annotated<std::string, "alist_path", Doc<"path of a parity-check matrix in alist form">, Visible>                            alist_path{};
    Annotated<std::string, "decoder", Doc<"'bp_flooding', 'bp_horizontal_layered', 'min_sum' or 'normalized_min_sum'">, Visible> decoder       = std::string("normalized_min_sum");
    Annotated<float, "normalization", Doc<"the factor the normalized min-sum rule scales a check message by">>                   normalization = 0.75F;
    Annotated<gr::Size_t, "n_iterations", Doc<"belief propagation iterations before the decode gives up">, Visible>              n_iterations  = 50U;
    Annotated<bool, "early_exit", Doc<"stop at the first iteration whose syndrome checks">>                                      early_exit    = true;

    GR_MAKE_REFLECTABLE(LdpcDecode, in, out, standard, alist_path, decoder, normalization, n_iterations, early_exit);

    std::optional<wall::LdpcCodec> _codec{};
    bool                           _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords             = 0ULL; ///< records published on `out`
    std::uint64_t nFrames              = 0ULL; ///< codewords those records carried
    std::uint64_t nRecordsRefused      = 0ULL; ///< records whose length was not a whole number of frames
    std::uint64_t nCorrectedErrors     = 0ULL; ///< coded bits between the frames received and the frames decoded
    std::uint64_t nUncorrectableFrames = 0ULL; ///< frames whose syndrome still failed after the last iteration

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        wall::LdpcSettings settings;
        settings.standard      = standard.value;
        settings.alistPath     = alist_path.value;
        settings.decoder       = decoder.value;
        settings.normalization = normalization.value;
        settings.iterations    = static_cast<std::size_t>(n_iterations.value);
        settings.earlyExit     = early_exit.value;
        _codec.emplace(settings);
        _configured = true; // only reached when the settings named a code the adapter accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 5UZ> counters{{{"records", nRecords}, {"frames", nFrames}, {"records refused", nRecordsRefused}, //
            {"corrected errors", nCorrectedErrors}, {"uncorrectable frames", nUncorrectableFrames}}};
        detail::report("gr::blocks::fec::LdpcDecode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert until the settings name a matrix
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const std::size_t k = _codec->payloadBits();
        const std::size_t n = _codec->codedBits();

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<float>& record = inSpan[consumed];
            const std::size_t     values = record.signal_values.size();
            if (values == 0UZ || values % n != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t frames = values / n;

            DataSet<std::uint8_t> info;
            info.signal_values.resize(frames * k);
            gr::Size_t corrected     = 0U;
            gr::Size_t uncorrectable = 0U;
            for (std::size_t f = 0UZ; f < frames; ++f) {
                const wall::DecodeReport report = _codec->decode(std::span<const float>(record.signal_values).subspan(f * n, n), std::span<std::uint8_t>(info.signal_values).subspan(f * k, k));
                corrected += static_cast<gr::Size_t>(report.correctedErrors);
                uncorrectable += report.refused ? 1U : 0U;
            }
            detail::carry(info, record);

            property_map& map           = info.meta_information[0UZ];
            map["corrected_errors"]     = gr::Size_t{detail::metaOr<gr::Size_t>(map, "corrected_errors", 0U) + corrected};
            map["uncorrectable_errors"] = gr::Size_t{detail::metaOr<gr::Size_t>(map, "uncorrectable_errors", 0U) + uncorrectable};

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

#endif // GNURADIO_FEC_LDPC_BLOCKS_HPP
