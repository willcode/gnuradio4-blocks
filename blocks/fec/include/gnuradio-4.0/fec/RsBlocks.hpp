#ifndef GNURADIO_FEC_RS_BLOCKS_HPP
#define GNURADIO_FEC_RS_BLOCKS_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <format>
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

#include <gnuradio-4.0/algorithm/fec/ReedSolomon.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the Reed-Solomon kernel in `gnuradio-4.0/algorithm/fec`.
 *
 * The carrier is a record of symbols: a `DataSet<std::uint8_t>` with one symbol per item, each
 * item holding a field value in its low bits — six of them for GF(64), the whole item for the
 * CCSDS profiles' GF(256). A symbol is the code's natural unit, and packing symbols into a bit
 * stream is composition work for the bit-packing blocks rather than something to fold in here.
 *
 * Two families share the pair. Under `code` `gf64` the validated GF(64) instantiations are selected
 * by `roots` at 8, 12 or 16 parity symbols over the 63-symbol block. The `code` setting names a CCSDS
 * profile instead — `ccsds_255_223` or `ccsds_255_239`, 131.0-B-5 section 4's codes over
 * GF(256) with their first consecutive root and primitive step — and with it come the two
 * conventions the standard attaches: `basis` says whether the wire's symbols are dual-basis
 * strings (4.3.9's recoding, one table lookup per symbol on the way in and on the information
 * on the way out), and `interleave` is the codeword count `I` of one codeblock (4.3.5.1's 1 to 5 or 8), the symbols of
 * the `I` codewords alternating on the wire so a channel burst lands spread across them. In
 * every family both ends know that the leading `pad` symbols of each codeword are zero and
 * neither sends them. The adapters stage each codeword into the kernel's block with the
 * padding restored, call it with `pad`, and copy the wire region back out; the kernel owns the
 * arithmetic and the checks.
 */
namespace gr::blocks::fec {

//! The Reed-Solomon code a block runs: the GF(64) family `roots` selects in, or a CCSDS profile over GF(256).
enum class RsCodeFamily : std::uint8_t { gf64, ccsds_255_223, ccsds_255_239 };

//! The basis of a CCSDS profile's wire symbols, conventional or dual. A profile starts at `unset`: neither end
//! can guess it.
enum class RsBasis : std::uint8_t { unset, conventional, dual };

namespace detail {

//! The longest unshortened block any family here works on, which sizes the staging buffers.
inline constexpr std::size_t kRsMaxBlockSymbols = gr::fec::ReedSolomonCcsds255_223::kBlock;

//! Stage @p word into the kernel's own block type, run it, and copy the result back out. The
//! copies keep one type-erased surface over the GF(64) and GF(256) families.
template<typename TKernel>
inline void rsEncodeThrough(std::span<std::uint8_t> word, std::size_t pad) {
    typename TKernel::Block block{};
    std::copy_n(word.begin(), TKernel::kBlock, block.begin());
    // RsEncode masks each information item with the code's mask. A masked item is an element of
    // the field, and encode accepts every block of them.
    [[maybe_unused]] const bool encoded = TKernel::encode(block, pad);
    assert(encoded);
    std::copy_n(block.begin(), TKernel::kBlock, word.begin());
}

template<typename TKernel>
[[nodiscard]] inline gr::fec::RsResult rsDecodeThrough(std::span<std::uint8_t> word, std::size_t pad) {
    typename TKernel::Block block{};
    std::copy_n(word.begin(), TKernel::kBlock, block.begin());
    const gr::fec::RsResult result = TKernel::decode(block, pad);
    std::copy_n(block.begin(), TKernel::kBlock, word.begin());
    return result;
}

//! One Reed-Solomon code: its geometry and the kernel entry points the settings select.
struct RsCode {
    std::size_t  roots = 0UZ;   //!< parity symbols per codeword
    std::size_t  block = 0UZ;   //!< symbols of the unshortened block, 63 or 255
    std::uint8_t mask  = 0xFFU; //!< the bits of an item a symbol occupies; the ones above are ignored on read, emitted zero
    bool         ccsds = false; //!< a CCSDS profile, whose basis and interleave rules apply

    void (*encode)(std::span<std::uint8_t>, std::size_t)              = nullptr;
    gr::fec::RsResult (*decode)(std::span<std::uint8_t>, std::size_t) = nullptr;
};

template<typename TKernel>
[[nodiscard]] inline RsCode rsCodeOver(std::uint8_t mask, bool ccsds) {
    return {TKernel::kRoots, TKernel::kBlock, mask, ccsds, &rsEncodeThrough<TKernel>, &rsDecodeThrough<TKernel>};
}

//! The code the `code` and `roots` settings name: a CCSDS profile, or the validated GF(64) family
//! by parity count, for settings `checkReedSolomon` accepts.
[[nodiscard]] inline RsCode reedSolomonCode(RsCodeFamily family, gr::Size_t roots) {
    switch (family) {
    case RsCodeFamily::ccsds_255_223: return rsCodeOver<gr::fec::ReedSolomonCcsds255_223>(0xFFU, true);
    case RsCodeFamily::ccsds_255_239: return rsCodeOver<gr::fec::ReedSolomonCcsds255_239>(0xFFU, true);
    case RsCodeFamily::gf64: break;
    }
    if (roots == 8U) {
        return rsCodeOver<gr::fec::ReedSolomon6<8UZ>>(0x3FU, false);
    }
    if (roots == 12U) {
        return rsCodeOver<gr::fec::ReedSolomon6<12UZ>>(0x3FU, false);
    }
    return rsCodeOver<gr::fec::ReedSolomon6<16UZ>>(0x3FU, false);
}

/**
 * The fault in the Reed-Solomon settings, under the setting at fault, or nothing when they name a code.
 *
 * A `roots` staged beside a profile must equal the profile's own parity count — one question, one answer. A CCSDS
 * profile requires the `basis` choice — 131.0-B-5 4.3.9.1 mandates `dual` while a substantial flown population runs
 * `conventional`, a wrong choice decodes nothing, and neither end can guess. The GF(64) family has no dual-basis
 * convention, and the block refuses a `basis` under `gf64`.
 */
template<typename TSettings>
[[nodiscard]] std::optional<gr::Refused> checkReedSolomon(const TSettings& s) {
    const RsCodeFamily family = s.code.value;
    if (family == RsCodeFamily::gf64) {
        if (s.roots.value == 0U) {
            return gr::Refused(s.roots, "required");
        }
        if (s.roots.value != 8U && s.roots.value != 12U && s.roots.value != 16U) {
            return gr::Refused(s.roots, "must be one of: 8, 12, 16");
        }
        if (s.basis.value != RsBasis::unset) {
            return gr::Refused(s.basis, "must be unset under code gf64: basis selects a CCSDS profile's symbol convention, and the GF(64) family has none");
        }
    } else {
        const gr::Size_t profileRoots = family == RsCodeFamily::ccsds_255_223 ? 32U : 16U;
        if (s.roots.value != 0U && s.roots.value != profileRoots) {
            return gr::Refused(s.roots, std::format("must be 0 or {}", profileRoots));
        }
        if (s.basis.value == RsBasis::unset) {
            return gr::Refused(s.basis, "required");
        }
        if (!gr::fec::ccsdsInterleaveDepthAllowed(s.interleave.value)) {
            return gr::Refused(s.interleave, "must be one of: 1, 2, 3, 4, 5, 8");
        }
    }
    const RsCode named = reedSolomonCode(family, s.roots.value);
    if (s.pad.value + named.roots >= named.block) {
        return gr::Refused(s.pad, std::format("must be < {}", named.block - named.roots));
    }
    return {};
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::RsEncode)

struct RsEncode : Block<RsEncode> {
    using Description = Doc<R""(
@brief Reed-Solomon encode over the GF(64) family or a CCSDS GF(256) profile: information-symbol records in, codeword
records out, one for one.

Each input record carries a whole number of codeblocks, so its length must be a nonzero multiple of
`(block - roots - pad) * interleave` symbols, `block` being 63 in the GF(64) family and 255 under a CCSDS profile.
Each group of `block - roots - pad` symbols becomes one codeword of `block - pad` symbols on the wire, and the
`interleave` codewords of a codeblock alternate symbol by symbol there. A GF(64) item holds its symbol in the low six
bits, the bits above ignored on read and emitted zero; a CCSDS symbol is the whole byte, dual-basis where `basis`
says so.

An encoder has no status to report, so the record's metadata crosses unchanged; its signal name and its single-map
shape follow it, and the output record's extent names its own length. RsDecode is the counterpart.

A record whose length is not such a multiple is dropped and counted in `nRecordsRefused`, `stop()` states the total,
and the record that follows is processed normally, so a misaligned record costs one record rather than the stream.

`roots` is 8, 12 or 16 in the GF(64) family; under a CCSDS profile it is 0 or the profile's count. `pad` shortens the
code by untransmitted leading zero symbols.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Doc<"parity symbols per codeword">, Visible>              roots{};
        Annotated<gr::Size_t, Unit<"symbols">, Doc<"shortening">>                       pad        = 0U;
        Annotated<RsCodeFamily, Doc<"code family">, Visible>                            code       = RsCodeFamily::gf64;
        Annotated<RsBasis, Doc<"CCSDS symbol basis">>                                   basis      = RsBasis::unset;
        Annotated<gr::Size_t, Doc<"codewords per codeblock">, Limits<Min<1U>, Max<8U>>> interleave = 1U;

        GR_MAKE_REFLECTABLE(Settings, roots, pad, code, basis, interleave);
    } settings;

    GR_MAKE_REFLECTABLE(RsEncode, in, out, settings);

    detail::RsCode _code{};
    std::size_t    _pad   = 0UZ;
    std::size_t    _depth = 1UZ;
    bool           _dual  = false;

    std::vector<std::uint8_t> _infoBlock{}; //!< one codeblock's information symbols, in wire order
    std::vector<std::uint8_t> _infoWords{}; //!< the same symbols gathered codeword by codeword
    std::vector<std::uint8_t> _wireWords{}; //!< the encoded codewords, before interleaving back out

    // RsEncode's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nCodewords      = 0ULL; ///< codewords those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a multiple of the information length

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkReedSolomon(s); }

    void applySettings(const Settings& s) {
        _code  = detail::reedSolomonCode(s.code, s.roots);
        _dual  = _code.ccsds && s.basis == RsBasis::dual; // the GF(64) family reads no basis
        _depth = s.interleave;
        _pad   = s.pad;
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"codewords", nCodewords}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::RsEncode", this->name, counters);
    }

    //! Information symbols in one codeword, which is also the input length one codeword consumes.
    [[nodiscard]] std::size_t infoSymbols() const noexcept { return _code.block - _code.roots - _pad; }

    //! Symbols one codeword occupies on the wire, the unshortened block minus the shortening.
    [[nodiscard]] std::size_t wireSymbols() const noexcept { return _code.block - _pad; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t info = infoSymbols();
        const std::size_t wire = wireSymbols();
        _infoBlock.resize(info * _depth);
        _infoWords.resize(info * _depth);
        _wireWords.resize(wire * _depth);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record  = inSpan[consumed];
            const std::size_t            symbols = record.signal_values.size();
            if (symbols == 0UZ || symbols % (info * _depth) != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t codeblocks = symbols / (info * _depth);

            DataSet<std::uint8_t> coded;
            coded.signal_values.resize(codeblocks * wire * _depth);
            for (std::size_t cb = 0UZ; cb < codeblocks; ++cb) {
                // the information arrives in wire order — dual-basis strings where the profile says so,
                // the codewords interleaved — and the recoding is per symbol, so it comes off first
                for (std::size_t i = 0UZ; i < info * _depth; ++i) {
                    const std::uint8_t symbol = static_cast<std::uint8_t>(record.signal_values[cb * info * _depth + i] & _code.mask);
                    _infoBlock[i]             = _dual ? gr::fec::CcsdsDualBasis::fromDual(symbol) : symbol;
                }
                gr::fec::deinterleaveCodewords(_infoBlock, _infoWords, info, _depth);

                for (std::size_t w = 0UZ; w < _depth; ++w) {
                    std::array<std::uint8_t, detail::kRsMaxBlockSymbols> buffer{};
                    std::copy_n(_infoWords.begin() + static_cast<std::ptrdiff_t>(w * info), info, buffer.begin() + static_cast<std::ptrdiff_t>(_pad));
                    _code.encode(std::span(buffer.data(), _code.block), _pad);
                    std::copy_n(buffer.begin() + static_cast<std::ptrdiff_t>(_pad), wire, _wireWords.begin() + static_cast<std::ptrdiff_t>(w * wire));
                }

                const std::span<std::uint8_t> outBlock(coded.signal_values.data() + cb * wire * _depth, wire * _depth);
                gr::fec::interleaveCodewords(_wireWords, outBlock, wire, _depth);
                if (_dual) {
                    for (std::uint8_t& symbol : outBlock) {
                        symbol = gr::fec::CcsdsDualBasis::toDual(symbol);
                    }
                }
            }
            const std::size_t words = codeblocks * _depth;
            coded.extents.push_back(static_cast<std::int32_t>(coded.signal_values.size()));
            coded.signal_names.emplace_back(record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ]);
            coded.timing_events.resize(1UZ);
            coded.meta_information.resize(1UZ);
            if (!record.meta_information.empty()) {
                coded.meta_information[0UZ] = record.meta_information[0UZ]; // the record's facts carry through
            }

            ++nRecords;
            nCodewords += words;
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

GR_REGISTER_BLOCK(gr::blocks::fec::RsDecode)

struct RsDecode : Block<RsDecode> {
    using Description = Doc<R""(
@brief Reed-Solomon decode over the GF(64) family or a CCSDS GF(256) profile: codeword records in, information-symbol
records out, one for one, with the code's verdict in metadata.

Each input record carries a whole number of codeblocks, so its length must be a nonzero multiple of
`(block - pad) * interleave` symbols, `block` being 63 in the GF(64) family and 255 under a CCSDS profile, and each
codeword becomes `block - roots - pad` information symbols. A record whose length fails that test is dropped and
counted as RsEncode drops one.

The verdict is carried in metadata: `corrected_errors` gains this record's per-codeword error sum and
`uncorrectable_errors` the count of codewords the kernel reported invalid, each added to whatever the key already
carried, so a chain of correcting stages reports one total rather than its last stage's share. Every other key crosses
verbatim, and a record arriving without a metadata map gains one to carry the two status keys.

A codeword whose correction lands inside the padding is counted in `nPadCorrupted` and in `uncorrectable_errors` as
well: such a word violates the shortening both ends agreed on. The metadata carries the two status keys and the
separate counter keeps the distinction visible. Information symbols are emitted for such a codeword as for any other.

`roots` is 8, 12 or 16 in the GF(64) family; under a CCSDS profile it is 0 or the profile's count. `pad` shortens the
code by untransmitted leading zero symbols.
)"">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortOut<DataSet<std::uint8_t>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Doc<"parity symbols per codeword">, Visible>              roots{};
        Annotated<gr::Size_t, Unit<"symbols">, Doc<"shortening">>                       pad        = 0U;
        Annotated<RsCodeFamily, Doc<"code family">, Visible>                            code       = RsCodeFamily::gf64;
        Annotated<RsBasis, Doc<"CCSDS symbol basis">>                                   basis      = RsBasis::unset;
        Annotated<gr::Size_t, Doc<"codewords per codeblock">, Limits<Min<1U>, Max<8U>>> interleave = 1U;

        GR_MAKE_REFLECTABLE(Settings, roots, pad, code, basis, interleave);
    } settings;

    GR_MAKE_REFLECTABLE(RsDecode, in, out, settings);

    detail::RsCode _code{};
    std::size_t    _pad   = 0UZ;
    std::size_t    _depth = 1UZ;
    bool           _dual  = false;

    std::vector<std::uint8_t> _codeblock{}; //!< one codeblock's wire symbols, the recoding taken off
    std::vector<std::uint8_t> _codewords{}; //!< the same symbols gathered codeword by codeword
    std::vector<std::uint8_t> _infoWords{}; //!< the corrected information regions, codeword by codeword

    // RsDecode's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords                = 0ULL; ///< records published on `out`
    std::uint64_t nCodewords              = 0ULL; ///< codewords those records carried
    std::uint64_t nRecordsRefused         = 0ULL; ///< records whose length was not a multiple of the wire length
    std::uint64_t nCorrectedErrors        = 0ULL; ///< symbols the kernel corrected, totaled over every codeword
    std::uint64_t nUncorrectableCodewords = 0ULL; ///< codewords the kernel reported invalid
    std::uint64_t nPadCorrupted           = 0ULL; ///< codewords whose correction fell in the shortening's padding

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkReedSolomon(s); }

    void applySettings(const Settings& s) {
        _code  = detail::reedSolomonCode(s.code, s.roots);
        _dual  = _code.ccsds && s.basis == RsBasis::dual; // the GF(64) family reads no basis
        _depth = s.interleave;
        _pad   = s.pad;
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 6UZ> counters{{{"records", nRecords}, {"codewords", nCodewords}, {"records refused", nRecordsRefused}, //
            {"corrected errors", nCorrectedErrors}, {"uncorrectable codewords", nUncorrectableCodewords}, {"pad corrupted", nPadCorrupted}}};
        detail::report("gr::blocks::fec::RsDecode", this->name, counters);
    }

    //! Information symbols one codeword yields.
    [[nodiscard]] std::size_t infoSymbols() const noexcept { return _code.block - _code.roots - _pad; }

    //! Symbols one codeword occupies on the wire, which is also the input length one codeword consumes.
    [[nodiscard]] std::size_t wireSymbols() const noexcept { return _code.block - _pad; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t info = infoSymbols();
        const std::size_t wire = wireSymbols();
        _codeblock.resize(wire * _depth);
        _codewords.resize(wire * _depth);
        _infoWords.resize(info * _depth);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint8_t>& record  = inSpan[consumed];
            const std::size_t            symbols = record.signal_values.size();
            if (symbols == 0UZ || symbols % (wire * _depth) != 0UZ) {
                ++nRecordsRefused;
                continue;
            }
            const std::size_t codeblocks = symbols / (wire * _depth);
            const std::size_t words      = codeblocks * _depth;

            DataSet<std::uint8_t> recovered;
            recovered.signal_values.resize(words * info);
            gr::Size_t corrected     = 0U;
            gr::Size_t uncorrectable = 0U;
            gr::Size_t padCorrupted  = 0U;
            for (std::size_t cb = 0UZ; cb < codeblocks; ++cb) {
                // the recoding is per symbol and the interleaving per index, so the basis comes off
                // the whole codeblock first and the deinterleave sees plain field elements
                for (std::size_t i = 0UZ; i < wire * _depth; ++i) {
                    const std::uint8_t symbol = static_cast<std::uint8_t>(record.signal_values[cb * wire * _depth + i] & _code.mask);
                    _codeblock[i]             = _dual ? gr::fec::CcsdsDualBasis::fromDual(symbol) : symbol;
                }
                gr::fec::deinterleaveCodewords(_codeblock, _codewords, wire, _depth);

                for (std::size_t w = 0UZ; w < _depth; ++w) {
                    std::array<std::uint8_t, detail::kRsMaxBlockSymbols> buffer{};
                    std::copy_n(_codewords.begin() + static_cast<std::ptrdiff_t>(w * wire), wire, buffer.begin() + static_cast<std::ptrdiff_t>(_pad));
                    const gr::fec::RsResult result = _code.decode(std::span(buffer.data(), _code.block), _pad);
                    std::copy_n(buffer.begin() + static_cast<std::ptrdiff_t>(_pad), info, _infoWords.begin() + static_cast<std::ptrdiff_t>(w * info));
                    corrected += result.errors;
                    if (result.pad_corrupted) {
                        ++padCorrupted;
                    }
                    if (!result.valid) { // a corrupted padding already reads as invalid, and is counted here once
                        ++uncorrectable;
                    }
                }

                // the transfer frame's octets are themselves dual-basis strings, so the information
                // leaves in the interleaved wire order with the recoding put back on
                const std::span<std::uint8_t> outBlock(recovered.signal_values.data() + cb * info * _depth, info * _depth);
                gr::fec::interleaveCodewords(_infoWords, outBlock, info, _depth);
                if (_dual) {
                    for (std::uint8_t& symbol : outBlock) {
                        symbol = gr::fec::CcsdsDualBasis::toDual(symbol);
                    }
                }
            }
            recovered.extents.push_back(static_cast<std::int32_t>(recovered.signal_values.size()));
            recovered.signal_names.emplace_back(record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ]);
            recovered.timing_events.resize(1UZ);
            recovered.meta_information.resize(1UZ);
            property_map& map = recovered.meta_information[0UZ];
            if (!record.meta_information.empty()) {
                map = record.meta_information[0UZ]; // the record's facts carry through, the status keys over them
            }
            const gr::Size_t priorCorrected     = metaOr<gr::Size_t>(map, "corrected_errors", 0U);
            const gr::Size_t priorUncorrectable = metaOr<gr::Size_t>(map, "uncorrectable_errors", 0U);
            map["corrected_errors"]             = gr::Size_t{priorCorrected + corrected};
            map["uncorrectable_errors"]         = gr::Size_t{priorUncorrectable + uncorrectable};

            ++nRecords;
            nCodewords += words;
            nCorrectedErrors += corrected;
            nUncorrectableCodewords += uncorrectable;
            nPadCorrupted += padCorrupted;
            outSpan[made] = std::move(recovered);
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
    template<typename V>
    [[nodiscard]] static V metaOr(const property_map& map, const char* key, V fallback) {
        if (const auto entry = map.find(property_map::key_type(key)); entry != map.end()) {
            return entry->second.value_or(V(fallback));
        }
        return fallback;
    }
};

} // namespace gr::blocks::fec

#endif // GNURADIO_FEC_RS_BLOCKS_HPP
