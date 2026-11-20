#ifndef GNURADIO_FEC_CONV_BLOCKS_HPP
#define GNURADIO_FEC_CONV_BLOCKS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fec/Convolutional.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the convolutional code and the Viterbi decoder in
 * `gnuradio-4.0/algorithm/fec`.
 *
 * The hard-decision carrier is the same record of bits the block-code adapters take: a
 * `DataSet<std::uint8_t>` with one bit per item, the low bit holding the value and the bits above
 * it ignored on read and emitted zero. Item `i` of a coded record is coded bit `i`, and the coded
 * bits of one input step follow one another in the order the polynomials are given, so a puncturer
 * or an interleaver composes downstream without either end restating the layout.
 *
 * The soft-decision carrier is a `DataSet<float>` of one value per coded bit, the sign carrying
 * the bit and the magnitude the confidence, with zero a pure erasure. The branch metric is a
 * correlation, so a demodulator hands over whatever amplitude it has: scaling every value scales
 * the metric and moves no decision.
 *
 * One record is one frame: the encoder terminates it and the decoder holds the whole trellis and
 * tracks back from that termination, so a decode is maximum likelihood over the record rather than
 * the survivor of a truncated window.
 *
 * The decoders also take `termination = "open"` for a record cut out of a continuous convolutional
 * stream, as a CCSDS channel's frames are: every step carries an information bit, neither end
 * state is known, the trellis converges within about 5K steps of the record's start, and the last
 * K-1 information bits lack the future that would resolve them, so a caller extracts that margin
 * beyond its payload and discards it. The encoder always terminates.
 */
namespace gr::blocks::fec {

/// @brief The convolutional code a block runs: `custom` takes `constraint_length`, `polynomials` and `invert_outputs` as
/// given, and every other enumerator is the library's published convention of that name.
enum class ConvCode : std::uint8_t { custom, ccsds, ccsds_uninverted, nasa_dsn, nasa_dsn_uninverted };

namespace detail {

// each enumerator after `custom` names the row of the library's table at its own position less one, so a lookup is an
// index and a reordered table fails here rather than selecting another code
static_assert(
    [] {
        constexpr auto codes = gr::meta::enumValues<ConvCode>();
        if (codes.size() != gr::fec::kConvConventions.size() + 1UZ) {
            return false;
        }
        for (std::size_t i = 1UZ; i < codes.size(); ++i) {
            if (gr::meta::enumName(codes[i]) != gr::fec::kConvConventions[i - 1UZ].name) {
                return false;
            }
        }
        return true;
    }(),
    "ConvCode's named enumerators must follow gr::fec::kConvConventions");

//! The published convention @p code names; `code` is not `custom`.
[[nodiscard]] inline const gr::fec::ConvolutionalConvention& conventionOf(ConvCode code) noexcept { return gr::fec::kConvConventions[static_cast<std::size_t>(code) - 1UZ]; }

//! The fault in the code the settings name — a convention by `code`, or under `custom` the explicit `constraint_length`
//! and `polynomials` pair — under the setting at fault, or nothing when they name one. An explicit setting staged beside
//! a named convention must agree with it: two answers to one question is a refusal, never a silent preference. Zero, or
//! an empty list, is a setting not given.
template<typename TSettings>
[[nodiscard]] std::optional<gr::Refused> checkConvolutionalCode(const TSettings& s) {
    const std::vector<gr::Size_t> generators = s.polynomials.value;
    if (s.code.value != ConvCode::custom) {
        const gr::fec::ConvolutionalConvention* convention = &conventionOf(s.code.value);
        if (s.constraint_length.value != 0U && s.constraint_length.value != convention->constraintLength) {
            return gr::Refused(s.constraint_length, std::format("must be 0 or {}", convention->constraintLength));
        }
        const std::span<const std::uint32_t> supplied(convention->polynomials.data(), convention->polynomialCount);
        if (!generators.empty() && !std::ranges::equal(generators, supplied)) {
            return gr::Refused(s.polynomials, "must be empty or match the polynomials of code");
        }
        if (s.invert_outputs.value != 0U && s.invert_outputs.value != convention->outputInversion) {
            return gr::Refused(s.invert_outputs, std::format("must be 0 or {:#b}", convention->outputInversion));
        }
        return {};
    }

    if (s.constraint_length.value == 0U && generators.empty()) {
        return gr::Refused(s.code, "required");
    }
    if (s.constraint_length.value < gr::fec::kConvMinConstraintLength || s.constraint_length.value > gr::fec::kConvMaxConstraintLength) {
        return gr::Refused(s.constraint_length, std::format("must be >= {} and <= {}", gr::fec::kConvMinConstraintLength, gr::fec::kConvMaxConstraintLength));
    }
    if (generators.size() < gr::fec::kConvMinPolynomials || generators.size() > gr::fec::kConvMaxPolynomials) {
        return gr::Refused(s.polynomials, std::format("must have >= {} and <= {} entries", gr::fec::kConvMinPolynomials, gr::fec::kConvMaxPolynomials));
    }
    if ((s.invert_outputs.value >> generators.size()) != 0U) {
        return gr::Refused(s.invert_outputs, std::format("must be < {}", 1U << generators.size()));
    }

    // the length, the count and the inversion passed above, so a configure that fails has the generators at fault
    gr::fec::ConvolutionalCode trial;
    const bool                 configured = trial.configure(s.constraint_length.value, generators, s.invert_outputs.value);
    if (!configured) {
        return gr::Refused(s.polynomials, "each polynomial must be > 0 and < 2^constraint_length and differ from the others");
    }
    return {};
}

//! The code the settings name, for settings `checkConvolutionalCode` accepts.
[[nodiscard]] inline gr::fec::ConvolutionalCode makeConvolutionalCode(gr::Size_t constraintLength, const std::vector<gr::Size_t>& polynomials, ConvCode named, gr::Size_t invertOutputs) {
    gr::fec::ConvolutionalCode code;
    if (named == ConvCode::custom) {
        std::ignore = code.configure(constraintLength, polynomials, invertOutputs);
    } else {
        std::ignore = gr::fec::configureConvention(code, conventionOf(named).name);
    }
    return code;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::ConvEncode)

struct ConvEncode : Block<ConvEncode> {
    using Description = Doc<R""(
@brief Convolutional encode: one information-bit record in, one terminated coded frame out, under the code
`constraint_length` and `polynomials` name.

A record is a frame. Its `k` information bits become `(k + K - 1) * n` coded bits, the tail being the K-1 zero bits
that leave the register in the zero state, so the decoder's traceback is exact. Any nonzero length encodes; an empty
record is refused, counted in `nRecordsRefused` and stated at `stop()`, and the record after it is encoded normally.
Every frame is terminated, so this block has no `termination` setting.

An encoder has no status to report, so the record's metadata crosses unchanged; its signal name and its single-map
shape follow it, and the output record's extent names its own length. ViterbiDecode and ViterbiDecodeSoft are the
decoding counterparts.

`polynomials` are octal, bit 0 the current input, and bit j of `invert_outputs` complements output j. code custom takes
the explicit settings.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"bits">, Doc<"constraint length">, Visible>        constraint_length{};
        Annotated<std::vector<gr::Size_t>, Doc<"generator polynomials">, Visible>     polynomials{};
        Annotated<ConvCode, Doc<"published convention (custom = explicit)">, Visible> code           = ConvCode::custom;
        Annotated<gr::Size_t, Doc<"output inversion mask">>                           invert_outputs = 0U;

        GR_MAKE_REFLECTABLE(Settings, constraint_length, polynomials, code, invert_outputs);
    } settings;

    GR_MAKE_REFLECTABLE(ConvEncode, in, out, settings);

    gr::fec::ConvolutionalCode _code{};

    // ConvEncode's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< frames published on `out`
    std::uint64_t nInfoBits       = 0ULL; ///< information bits those frames carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records that were not a frame this code can encode

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkConvolutionalCode(s); }

    void applySettings(const Settings& s) { _code = detail::makeConvolutionalCode(s.constraint_length, s.polynomials, s.code, s.invert_outputs); }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"frames", nRecords}, {"information bits", nInfoBits}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::ConvEncode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record = inSpan[consumed];
            const std::size_t            bits   = record.signal_values.size();
            if (bits == 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> coded;
            coded.signal_values.resize(gr::fec::convolutionalEncodedBits(_code, bits));
            std::ignore = gr::fec::convolutionalEncode(_code, record.signal_values, coded.signal_values);
            detail::carry(coded, record);

            ++nRecords;
            nInfoBits += bits;
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

GR_REGISTER_BLOCK(gr::blocks::fec::ViterbiDecode)

struct ViterbiDecode : Block<ViterbiDecode> {
    using Description = Doc<R""(
@brief Viterbi decode of hard decisions: one terminated coded frame in, its information bits out, with the channel's
account in metadata.

A record's length must be a nonzero multiple of `n` and hold at least `K * n` bits, a frame shorter than the tail
carrying no information; the record then yields `len / n - (K - 1)` information bits. A length failing that test is a
counted, stated drop, as an empty record is at ConvEncode.

The account is carried in metadata: `corrected_errors` gains the decode's distance (the bits by which the received frame and
the winning path disagree) added to whatever the key already carried, so a chain of correcting stages reports one
total rather than its last stage's share. Every other key crosses verbatim, and a record arriving without a metadata
map gains one to carry the key.

There is no `uncorrectable_errors`: the trellis has a best path through any received word, so the key would always be
zero. The distance in `corrected_errors` is how far the received frame stood from the path chosen.

`polynomials` are octal, bit 0 the current input, and bit j of `invert_outputs` complements output j. code custom takes
the explicit settings. termination terminated adds constraint_length - 1 tail bits.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"bits">, Doc<"constraint length">, Visible>        constraint_length{};
        Annotated<std::vector<gr::Size_t>, Doc<"generator polynomials">, Visible>     polynomials{};
        Annotated<ConvCode, Doc<"published convention (custom = explicit)">, Visible> code           = ConvCode::custom;
        Annotated<gr::Size_t, Doc<"output inversion mask">>                           invert_outputs = 0U;
        Annotated<gr::fec::ConvTermination, Doc<"frame termination">>                 termination    = gr::fec::ConvTermination::Terminated;

        GR_MAKE_REFLECTABLE(Settings, constraint_length, polynomials, code, invert_outputs, termination);
    } settings;

    GR_MAKE_REFLECTABLE(ViterbiDecode, in, out, settings);

    gr::fec::ConvolutionalCode _code{};
    gr::fec::ViterbiDecoder    _decoder{};

    // ViterbiDecode's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords         = 0ULL; ///< frames published on `out`
    std::uint64_t nInfoBits        = 0ULL; ///< information bits those frames carry
    std::uint64_t nRecordsRefused  = 0ULL; ///< records whose length was not a frame of this code
    std::uint64_t nCorrectedErrors = 0ULL; ///< bits between the received frames and the paths decoded, totaled

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkConvolutionalCode(s); }

    void applySettings(const Settings& s) {
        _code       = detail::makeConvolutionalCode(s.constraint_length, s.polynomials, s.code, s.invert_outputs);
        std::ignore = _decoder.configure(_code, s.termination);
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 4UZ> counters{{{"frames", nRecords}, {"information bits", nInfoBits}, {"records refused", nRecordsRefused}, {"corrected errors", nCorrectedErrors}}};
        detail::report("gr::blocks::fec::ViterbiDecode", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record  = inSpan[consumed];
            const std::size_t            carried = gr::fec::convolutionalInfoBits(_code, record.signal_values.size(), settings.termination);
            if (carried == 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> info;
            info.signal_values.resize(carried);
            const gr::fec::ViterbiResult result = _decoder.decodeHard(record.signal_values, info.signal_values);
            detail::carry(info, record);

            const gr::Size_t corrected = static_cast<gr::Size_t>(result.distance);
            property_map&    map       = info.meta_information[0UZ];
            map["corrected_errors"]    = gr::Size_t{detail::metaOr<gr::Size_t>(map, "corrected_errors", 0U) + corrected};

            ++nRecords;
            nInfoBits += carried;
            nCorrectedErrors += result.distance;
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

GR_REGISTER_BLOCK(gr::blocks::fec::ViterbiDecodeSoft)

struct ViterbiDecodeSoft : Block<ViterbiDecodeSoft> {
    using Description = Doc<R""(
@brief Viterbi decode of soft decisions: one record of soft coded values in, its information bits out, with the
channel's account in metadata.

Identical to ViterbiDecode but for the carrier: one `float` per coded bit, the sign carrying the bit (positive is a
one) and the magnitude the confidence, with zero a pure erasure. The branch metric is the correlation between a
branch's output and the received values, so any consistent scaling of the input scales the metric and changes no
decision; a demodulator needs neither a normalization step nor a quantization table before this block.

`corrected_errors` gains the distance between the sign-sliced input and the winning path, the same account of the
channel the hard decoder gives, and there is no `uncorrectable_errors` here either. The length contract, the counted
drops and the metadata carry are ViterbiDecode's.

`polynomials` are octal, bit 0 the current input, and bit j of `invert_outputs` complements output j. code custom takes
the explicit settings. termination terminated adds constraint_length - 1 tail bits.
)"">;

    PortIn<DataSet<float>, Async>         in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"bits">, Doc<"constraint length">, Visible>        constraint_length{};
        Annotated<std::vector<gr::Size_t>, Doc<"generator polynomials">, Visible>     polynomials{};
        Annotated<ConvCode, Doc<"published convention (custom = explicit)">, Visible> code           = ConvCode::custom;
        Annotated<gr::Size_t, Doc<"output inversion mask">>                           invert_outputs = 0U;
        Annotated<gr::fec::ConvTermination, Doc<"frame termination">>                 termination    = gr::fec::ConvTermination::Terminated;

        GR_MAKE_REFLECTABLE(Settings, constraint_length, polynomials, code, invert_outputs, termination);
    } settings;

    GR_MAKE_REFLECTABLE(ViterbiDecodeSoft, in, out, settings);

    gr::fec::ConvolutionalCode _code{};
    gr::fec::ViterbiDecoder    _decoder{};

    // ViterbiDecodeSoft's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords         = 0ULL; ///< frames published on `out`
    std::uint64_t nInfoBits        = 0ULL; ///< information bits those frames carry
    std::uint64_t nRecordsRefused  = 0ULL; ///< records whose length was not a frame of this code
    std::uint64_t nCorrectedErrors = 0ULL; ///< bits between the sign-sliced frames and the paths decoded, totaled

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkConvolutionalCode(s); }

    void applySettings(const Settings& s) {
        _code       = detail::makeConvolutionalCode(s.constraint_length, s.polynomials, s.code, s.invert_outputs);
        std::ignore = _decoder.configure(_code, s.termination);
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 4UZ> counters{{{"frames", nRecords}, {"information bits", nInfoBits}, {"records refused", nRecordsRefused}, {"corrected errors", nCorrectedErrors}}};
        detail::report("gr::blocks::fec::ViterbiDecodeSoft", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<float>& record  = inSpan[consumed];
            const std::size_t     carried = gr::fec::convolutionalInfoBits(_code, record.signal_values.size(), settings.termination);
            if (carried == 0UZ) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<std::uint8_t> info;
            info.signal_values.resize(carried);
            const gr::fec::ViterbiResult result = _decoder.decodeSoft(record.signal_values, info.signal_values);
            detail::carry(info, record);

            const gr::Size_t corrected = static_cast<gr::Size_t>(result.distance);
            property_map&    map       = info.meta_information[0UZ];
            map["corrected_errors"]    = gr::Size_t{detail::metaOr<gr::Size_t>(map, "corrected_errors", 0U) + corrected};

            ++nRecords;
            nInfoBits += carried;
            nCorrectedErrors += result.distance;
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

#endif // GNURADIO_FEC_CONV_BLOCKS_HPP
