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
 * The record-native adapters over AFF3CT's LDPC family.
 *
 * The code is a parity-check matrix and a belief-propagation schedule, both fixed at
 * configuration: `standard` names one of the matrices the pinned release ships, or `alist_path`
 * names one on disk, and neither has a default because a matrix is the code and a default matrix
 * would be an interoperability assumption nobody made. The decoder objects are built once and
 * reused for every record; a settings change is a new object, which is what a graph rebuild is
 * for.
 *
 * The encoder carries bits and the decoder carries soft values, which is the split every
 * soft-decision family in this module has. `LdpcEncode` takes a `DataSet<std::uint8_t>` of
 * information bits and publishes a `DataSet<std::uint8_t>` of coded bits, one item per bit.
 * `LdpcDecode` takes a `DataSet<float>` of log-likelihood ratios in this tree's sense — positive
 * carries a one, the magnitude is confidence, zero is an erasure — and publishes the information
 * bits. A hard-decision receiver enters the decoder by presenting its bits as saturated values of
 * that sign, which is the same bridge `ViterbiDecodeSoft` takes and needs no second port.
 *
 * A record holds a whole number of frames. Its length must be a nonzero multiple of the code's
 * `k` on the way in and of its `n` on the way back; a record that fails that test is dropped,
 * counted in `nRecordsRefused` and stated at `stop()`, and the record after it is coded normally.
 *
 * This family can refuse, and the honesty channel uses it. A decode whose syndrome still fails
 * when the iterations are spent is counted in `uncorrectable_errors`; `corrected_errors` gains
 * the coded bits by which the received word and the word decoded disagree, which is the same
 * account of the channel every other decoder in this module gives. The information estimate is
 * emitted either way — the counts say what it is worth and nothing is zeroed or invented.
 */
namespace gr::blocks::fec {

GR_REGISTER_BLOCK(gr::blocks::fec::LdpcEncode)

/*!
@brief LDPC encode: information-bit records in, codeword records out.

Each record carries a whole number of information frames of `k` bits, and each becomes one `n`-bit
codeword. An encoder has no status to report, so the record's metadata crosses unchanged; its
signal name and its single-map shape follow it, and the output record's extent names its own
length.

The block builds the code's generator matrix from its parity-check matrix at configuration, which
is the expensive part of an LDPC encoder and is done once. See LdpcDecode for the counterpart.
*/
struct LdpcEncode : Block<LdpcEncode> {
    using Description = Doc<"LDPC encode: information-bit records to codeword records, under the matrix the 'standard' or 'alist_path' setting names">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "standard", Doc<"a parity-check matrix the pinned AFF3CT release ships, by name; empty when 'alist_path' names one instead">, Visible> standard{};
    Annotated<std::string, "alist_path", Doc<"a parity-check matrix on disk, in alist form; empty when 'standard' names one instead">, Visible>                   alist_path{};

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
        // The encode side never runs a decoder, so the cheapest schedule the wall carries is what it
        // is told to hold beside the encoder.
        settings.decoder    = std::string("min_sum");
        settings.iterations = 1UZ;
        _codec.emplace(settings);
        _configured = true; // only reached when the settings named a matrix the release or the disk holds
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"frames", nFrames}, {"records refused", nRecordsRefused}}};
        detail::reportFrames("gr::blocks::fec::LdpcEncode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than coding under a matrix nobody chose
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
            detail::carryFrame(coded, record);

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
@brief LDPC decode: soft codeword records in, information-bit records out, with the channel's
account and the code's verdict in metadata.

Each record carries a whole number of `n`-value frames and each yields `k` information bits. A
record whose length fails that test is dropped and counted exactly as LdpcEncode drops one.

The values are log-likelihood ratios in this tree's sense: a positive value carries a one and the
magnitude is confidence, with zero a pure erasure. AFF3CT's own convention is the opposite sign
sense, and the wall negates on the way in; that inversion is stated in one place and is pinned by
a QA anchor, because a decode under the wrong sign returns the bitwise complement and nothing else
would show.

The verdict rides the record. `corrected_errors` gains the coded bits by which the received frame
and the frame re-encoded from the decode disagree, and `uncorrectable_errors` the count of frames
whose syndrome still failed when the iterations were spent, each added to whatever the key already
carried. Every other key crosses verbatim, and a record arriving without a metadata map gains one.
*/
struct LdpcDecode : Block<LdpcDecode> {
    using Description = Doc<"LDPC decode: soft codeword records to information-bit records, the channel's account and the syndrome's verdict accumulating in metadata">;

    PortIn<DataSet<float>, Async>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<std::string, "standard", Doc<"a parity-check matrix the pinned AFF3CT release ships, by name; empty when 'alist_path' names one instead">, Visible> standard{};
    Annotated<std::string, "alist_path", Doc<"a parity-check matrix on disk, in alist form; empty when 'standard' names one instead">, Visible>                   alist_path{};
    Annotated<std::string, "decoder", Doc<"'bp_flooding', 'bp_horizontal_layered', 'min_sum' or 'normalized_min_sum'">, Visible>                                  decoder       = std::string("normalized_min_sum");
    Annotated<float, "normalization", Doc<"the factor the normalized min-sum rule scales a check message by">>                                                    normalization = 0.75F;
    Annotated<gr::Size_t, "n_iterations", Doc<"belief propagation iterations before the decode gives up">, Visible>                                               n_iterations  = 50U;
    Annotated<bool, "early_exit", Doc<"stop as soon as the syndrome checks, rather than running every iteration">>                                                early_exit    = true;

    GR_MAKE_REFLECTABLE(LdpcDecode, in, out, standard, alist_path, decoder, normalization, n_iterations, early_exit);

    std::optional<wall::LdpcCodec> _codec{};
    bool                           _configured = false;

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecords             = 0ULL; ///< records published on `out`
    std::uint64_t nFrames              = 0ULL; ///< codewords those records carried
    std::uint64_t nRecordsRefused      = 0ULL; ///< records whose length was not a whole number of frames
    std::uint64_t nCorrectedErrors     = 0ULL; ///< coded bits between the frames received and the frames decoded
    std::uint64_t nUncorrectableFrames = 0ULL; ///< frames whose syndrome failed when the iterations were spent

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
        _configured = true; // only reached when the settings named a code the wall accepts
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 5UZ> counters{{{"records", nRecords}, {"frames", nFrames}, {"records refused", nRecordsRefused}, //
            {"corrected errors", nCorrectedErrors}, {"uncorrectable frames", nUncorrectableFrames}}};
        detail::reportFrames("gr::blocks::fec::LdpcDecode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (!_configured) { // inert rather than decoding under a matrix nobody chose
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
            detail::carryFrame(info, record);

            property_map& map           = info.meta_information[0UZ];
            map["corrected_errors"]     = gr::Size_t{detail::metaOrZero<gr::Size_t>(map, "corrected_errors", 0U) + corrected};
            map["uncorrectable_errors"] = gr::Size_t{detail::metaOrZero<gr::Size_t>(map, "uncorrectable_errors", 0U) + uncorrectable};

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
