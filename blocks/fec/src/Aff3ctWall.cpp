#include <gnuradio-4.0/fec/Aff3ctWall.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/algorithm/digital/Crc.hpp>

#include <Module/CRC/CRC.hpp>
#include <Module/Decoder/LDPC/BP/Flooding/Decoder_LDPC_BP_flooding.hpp>
#include <Module/Decoder/LDPC/BP/Horizontal_layered/Decoder_LDPC_BP_horizontal_layered.hpp>
#include <Module/Decoder/Polar/SC/Decoder_polar_SC_naive_sys.hpp>
#include <Module/Decoder/Polar/SCL/CRC/Decoder_polar_SCL_naive_CA_sys.hpp>
#include <Module/Decoder/Polar/SCL/Decoder_polar_SCL_naive_sys.hpp>
#include <Module/Encoder/LDPC/From_H/Encoder_LDPC_from_H.hpp>
#include <Module/Encoder/Polar/Encoder_polar_sys.hpp>
#include <Tools/Algo/Matrix/Sparse_matrix/Sparse_matrix.hpp>
#include <Tools/Code/LDPC/AList/AList.hpp>
#include <Tools/Code/LDPC/Update_rule/MS/Update_rule_MS.hpp>
#include <Tools/Code/LDPC/Update_rule/NMS/Update_rule_NMS.hpp>
#include <Tools/Code/LDPC/Update_rule/SPA/Update_rule_SPA.hpp>
#include <Tools/Code/Polar/Frozenbits_generator/Frozenbits_generator_GA_Arikan.hpp>
#include <Tools/Code/Polar/Frozenbits_generator/Frozenbits_generator_file.hpp>
#include <Tools/Noise/Sigma.hpp>

namespace gr::blocks::fec::wall {

namespace {

//! Where the pinned release keeps the parity-check matrices it ships. Supplied by the build, which
//! is the only place that knows where the package was found.
#ifndef GR4_AFF3CT_LDPC_CONF_DIR
#define GR4_AFF3CT_LDPC_CONF_DIR ""
#endif

//! Where it keeps the polar reliability sequences, for the same reason.
#ifndef GR4_AFF3CT_POLAR_CONF_DIR
#define GR4_AFF3CT_POLAR_CONF_DIR ""
#endif

//! The one codeword length the release ships a 5G reliability sequence for.
constexpr std::size_t kPolar5gLength = 1024UZ;

/*!
 * @brief The constructions the release ships that this wrap names, and the file each resolves to.
 *
 * The set is what the pinned release actually carries at the shapes the tiers ask for, not what a
 * specification wished for: the release ships no 802.11n `(648, 324)` matrix, and its Wi-Fi matrix
 * is the rate 5/6 `(648, 540)` one. A name here is a promise that the file is present and that the
 * dimensions below are its own; anything else is `alist_path`.
 */
const std::map<std::string, std::string>& ldpcCatalog() {
    static const std::map<std::string, std::string> known{
        {"wimax_576_288", "WIMAX_288_576.alist"},    //
        {"wimax_576_480", "WIMAX_480_576.alist"},    //
        {"wifi_648_540", "WIFI_540_648.alist"},      //
        {"wran_480_360", "WRAN_360_480.alist"},      //
        {"ccsds_128_64", "CCSDS_64_128.alist"},      //
        {"mackay_1008_504", "MACKAY_504_1008.alist"} //
    };
    return known;
}

//! The parity-check matrix @p settings names, refusing through the graph's exception type.
[[nodiscard]] std::string ldpcMatrixPath(const LdpcSettings& settings) {
    if (!settings.standard.empty() && !settings.alistPath.empty()) {
        throw gr::exception(std::format("LDPC: 'standard' names {} and 'alist_path' names {} — a code is one matrix, not two", settings.standard, settings.alistPath));
    }
    if (!settings.alistPath.empty()) {
        if (!std::filesystem::exists(settings.alistPath)) {
            throw gr::exception(std::format("LDPC: alist_path '{}' does not exist", settings.alistPath));
        }
        return settings.alistPath;
    }
    if (settings.standard.empty()) {
        throw gr::exception("LDPC: one of 'standard' and 'alist_path' is required and neither has a default — a matrix is the code, and a default matrix would be an interoperability assumption nobody made");
    }
    const auto entry = ldpcCatalog().find(settings.standard);
    if (entry == ldpcCatalog().end()) {
        std::string names;
        for (const auto& [name, file] : ldpcCatalog()) {
            std::format_to(std::back_inserter(names), "{}{}", names.empty() ? "" : ", ", name);
        }
        throw gr::exception(std::format("LDPC: standard '{}' is not one this release ships; it carries {}", settings.standard, names));
    }
    const std::string directory(GR4_AFF3CT_LDPC_CONF_DIR);
    if (directory.empty()) {
        throw gr::exception(std::format("LDPC: standard '{}' names a matrix the release ships, but this build was configured without the release's matrix directory; use alist_path", settings.standard));
    }
    const std::string path = (std::filesystem::path(directory) / entry->second).string();
    if (!std::filesystem::exists(path)) {
        throw gr::exception(std::format("LDPC: standard '{}' should be at '{}' and is not there", settings.standard, path));
    }
    return path;
}

//! Read @p bits, one bit per item, as the integer frame AFF3CT takes.
void toFrame(std::span<const std::uint8_t> bits, std::vector<int>& frame) {
    frame.resize(bits.size());
    for (std::size_t i = 0UZ; i < bits.size(); ++i) {
        frame[i] = static_cast<int>(bits[i] & 1U);
    }
}

//! Write @p frame back out as one bit per item.
void fromFrame(const std::vector<int>& frame, std::span<std::uint8_t> bits) {
    for (std::size_t i = 0UZ; i < bits.size() && i < frame.size(); ++i) {
        bits[i] = static_cast<std::uint8_t>(frame[i] & 1);
    }
}

/*!
 * @brief The LLR bridge: this tree's sense in, AFF3CT's out.
 *
 * Positive carries a one here and favors a zero there, so the crossing is a negation and nothing
 * else. No scaling: every decoder behind this wall is either a min-sum family, whose decisions are
 * invariant under a positive scale, or a sum-product one, whose LLRs the caller is expected to have
 * scaled by the channel it measured.
 */
void toAff3ctLlr(std::span<const float> llr, std::vector<float>& frame) {
    frame.resize(llr.size());
    for (std::size_t i = 0UZ; i < llr.size(); ++i) {
        frame[i] = -llr[i];
    }
}

//! The hard decision this tree takes from a soft value: the sign, with zero reading as a one.
[[nodiscard]] int slice(float value) noexcept { return (value >= 0.0F) ? 1 : 0; }

/*!
 * @brief The tree's CRC in the shape AFF3CT's list decoders ask for.
 *
 * The list decoder needs to ask, of each surviving path, whether its information bits check. That
 * question is answered by `gr::digital::Crc` so the tree keeps one polynomial vocabulary, and this
 * class is the adapter and nothing more: `K` is the payload, `size` the signature, and the bits are
 * packed most significant first, which is the bit order every other adapter in this module uses.
 */
class TreeCrc : public aff3ct::module::CRC<int> {
public:
    TreeCrc(int payloadBits, int signatureBits, const gr::digital::Crc& crc) : aff3ct::module::CRC<int>(payloadBits, signatureBits), _payload(static_cast<std::size_t>(payloadBits)), _signature(static_cast<std::size_t>(signatureBits)), _crc(crc) {}

    //! Write the payload's signature into @p bits at the payload's end, which is what the encoder does.
    void sign(int* bits) const { appendSignature(bits); }

    /*!
     * @brief Whether the payload and signature bits at @p bits carry their own signature.
     *
     * Every bit is read for truth rather than for the value 1. The list decoder hands over the bits
     * its trees hold, and those are truthy rather than normalized — AFF3CT's own store writes
     * `s[i] ? 1 : 0` on the way out, which is the same statement made at the other end of the same
     * data. Comparing the raw values instead rejects every correct path and is exactly the fault
     * this comment exists to keep from coming back.
     */
    [[nodiscard]] bool signed_correctly(const int* bits) const {
        const std::uint64_t value = signatureOf(bits);
        for (std::size_t i = 0UZ; i < _signature; ++i) {
            const int carried = (bits[_payload + i] != 0) ? 1 : 0;
            if (carried != static_cast<int>((value >> (_signature - 1UZ - i)) & 1ULL)) {
                return false;
            }
        }
        return true;
    }

    TreeCrc* clone() const override {
        auto* copy = new TreeCrc(*this);
        copy->deep_copy(*this);
        return copy;
    }

protected:
    void _build(const int* U_K1, int* U_K2, const size_t /*frame*/) override {
        std::copy_n(U_K1, _payload, U_K2);
        appendSignature(U_K2);
    }

    void _extract(const int* V_K1, int* V_K2, const size_t /*frame*/) override { std::copy_n(V_K1, _payload, V_K2); }

    bool _check(const int* V_K, const size_t /*frame*/) override { return signed_correctly(V_K); }

    bool _check_packed(const int* V_K, const size_t frame) override { return _check(V_K, frame); }

private:
    //! The signature of the payload bits at @p bits, packed most significant bit first.
    [[nodiscard]] std::uint64_t signatureOf(const int* bits) const {
        _bytes.assign(_payload / 8UZ, 0U);
        for (std::size_t i = 0UZ; i < _payload; ++i) {
            _bytes[i / 8UZ] = static_cast<std::uint8_t>(_bytes[i / 8UZ] | (((bits[i] != 0) ? 1 : 0) << (7UZ - (i % 8UZ))));
        }
        return _crc.compute(std::span<const std::uint8_t>(_bytes));
    }

    void appendSignature(int* bits) const {
        const std::uint64_t value = signatureOf(bits);
        for (std::size_t i = 0UZ; i < _signature; ++i) {
            bits[_payload + i] = static_cast<int>((value >> (_signature - 1UZ - i)) & 1ULL);
        }
    }

    std::size_t                       _payload;
    std::size_t                       _signature;
    gr::digital::Crc                  _crc;
    mutable std::vector<std::uint8_t> _bytes{};
};

//! Re-raise whatever AFF3CT threw as the graph's own exception type, naming @p family and @p what.
[[noreturn]] void reraise(std::string_view family, const std::string& configuration, const std::exception& refusal) { throw gr::exception(std::format("{} ({}): {}", family, configuration, refusal.what())); }

} // namespace

std::vector<std::string> ldpcStandards() {
    std::vector<std::string> names;
    for (const auto& [name, file] : ldpcCatalog()) {
        names.push_back(name);
    }
    return names;
}

// --- LDPC ------------------------------------------------------------------------------------

struct LdpcCodec::Impl {
    std::size_t                                               k = 0UZ;
    std::size_t                                               n = 0UZ;
    std::unique_ptr<aff3ct::module::Encoder<int>>             encoder;
    std::unique_ptr<aff3ct::module::Decoder_SIHO<int, float>> decoder;

    std::vector<int>             frame{};
    std::vector<int>             coded{};
    std::vector<float>           llr{};
    std::array<std::int8_t, 1UZ> status{}; //!< the codeword status AFF3CT writes, one entry per frame
};

LdpcCodec::LdpcCodec(const LdpcSettings& settings) : _impl(std::make_unique<Impl>()) {
    const std::string path          = ldpcMatrixPath(settings);
    const std::string configuration = std::format("{} decoder {} at {} iterations", path, settings.decoder, settings.iterations);
    if (settings.iterations == 0UZ) {
        throw gr::exception(std::format("LDPC ({}): n_iterations is 0 — a belief propagation that runs no iteration is not a decode", configuration));
    }

    try {
        std::ifstream matrixFile(path);
        if (!matrixFile) {
            throw std::runtime_error(std::format("cannot read '{}'", path));
        }
        const aff3ct::tools::Sparse_matrix h = aff3ct::tools::AList::read(matrixFile);

        _impl->n = h.get_n_rows();
        _impl->k = _impl->n - h.get_n_cols();

        auto encoder = std::make_unique<aff3ct::module::Encoder_LDPC_from_H<int>>(static_cast<int>(_impl->k), static_cast<int>(_impl->n), h);
        // The decoder reads its information bits out of the posterior at the positions the encoder
        // put them, so the two must be told the same set; the encoder is what computed it.
        const std::vector<uint32_t> infoBitsPos = encoder->get_info_bits_pos();

        const int  iterations = static_cast<int>(settings.iterations);
        const auto ki         = static_cast<int>(_impl->k);
        const auto ni         = static_cast<int>(_impl->n);
        if (settings.decoder == "bp_flooding") {
            const aff3ct::tools::Update_rule_SPA<float> rule(static_cast<unsigned>(h.get_cols_max_degree()));
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_LDPC_BP_flooding<int, float, aff3ct::tools::Update_rule_SPA<float>>>(ki, ni, iterations, h, infoBitsPos, rule, settings.earlyExit);
        } else if (settings.decoder == "bp_horizontal_layered") {
            const aff3ct::tools::Update_rule_SPA<float> rule(static_cast<unsigned>(h.get_cols_max_degree()));
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_LDPC_BP_horizontal_layered<int, float, aff3ct::tools::Update_rule_SPA<float>>>(ki, ni, iterations, h, infoBitsPos, rule, settings.earlyExit);
        } else if (settings.decoder == "min_sum") {
            const aff3ct::tools::Update_rule_MS<float> rule;
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_LDPC_BP_flooding<int, float, aff3ct::tools::Update_rule_MS<float>>>(ki, ni, iterations, h, infoBitsPos, rule, settings.earlyExit);
        } else if (settings.decoder == "normalized_min_sum") {
            const aff3ct::tools::Update_rule_NMS<float> rule(settings.normalization);
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_LDPC_BP_flooding<int, float, aff3ct::tools::Update_rule_NMS<float>>>(ki, ni, iterations, h, infoBitsPos, rule, settings.earlyExit);
        } else {
            throw gr::exception(std::format("LDPC: decoder must be 'bp_flooding', 'bp_horizontal_layered', 'min_sum' or 'normalized_min_sum', got '{}'", settings.decoder));
        }
        _impl->encoder = std::move(encoder);
    } catch (const gr::exception&) {
        throw;
    } catch (const std::exception& refusal) {
        reraise("LDPC", configuration, refusal);
    }

    _impl->frame.resize(_impl->k);
    _impl->coded.resize(_impl->n);
    _impl->llr.resize(_impl->n);
}

LdpcCodec::~LdpcCodec()                               = default;
LdpcCodec::LdpcCodec(LdpcCodec&&) noexcept            = default;
LdpcCodec& LdpcCodec::operator=(LdpcCodec&&) noexcept = default;

std::size_t LdpcCodec::payloadBits() const noexcept { return _impl->k; }
std::size_t LdpcCodec::codedBits() const noexcept { return _impl->n; }

void LdpcCodec::encode(std::span<const std::uint8_t> payload, std::span<std::uint8_t> coded) {
    toFrame(payload, _impl->frame);
    try {
        _impl->encoder->encode(_impl->frame, _impl->coded);
    } catch (const std::exception& refusal) {
        reraise("LDPC", "encode", refusal);
    }
    fromFrame(_impl->coded, coded);
}

DecodeReport LdpcCodec::decode(std::span<const float> llr, std::span<std::uint8_t> payload) {
    toAff3ctLlr(llr, _impl->llr);
    try {
        // The three-argument vector overload cannot deduce its allocator, since the status frame is int8_t
        // and the information frame is int and the signature shares one allocator parameter between them;
        // the buffers are exactly N and K long, so the pointer form is the same call without the check.
        std::ignore = _impl->decoder->decode_siho(_impl->llr.data(), _impl->status.data(), _impl->frame.data());
    } catch (const std::exception& refusal) {
        reraise("LDPC", "decode", refusal);
    }
    fromFrame(_impl->frame, payload);

    // The account of the channel is the same one every other decoder in this module gives: the coded
    // bits by which the word received and the word decoded disagree. Re-encoding the estimate is what
    // makes that comparable across families, since only the encoder knows where the parity went.
    // AFF3CT's status socket is a codeword-detected flag rather than a failure flag: the belief
    // propagation writes 1 where the syndrome checked and 0 where the iterations ran out with it
    // still failing, which is the refusal this family can make.
    DecodeReport report;
    report.refused = _impl->status[0UZ] == 0;
    try {
        _impl->encoder->encode(_impl->frame, _impl->coded);
    } catch (const std::exception& refusal) {
        reraise("LDPC", "decode", refusal);
    }
    for (std::size_t i = 0UZ; i < _impl->n && i < llr.size(); ++i) {
        report.correctedErrors += (slice(llr[i]) != (_impl->coded[i] & 1)) ? 1UZ : 0UZ;
    }
    return report;
}

// --- Polar -----------------------------------------------------------------------------------

struct PolarCodec::Impl {
    std::size_t k       = 0UZ; //!< bits the encoder takes, the signature included
    std::size_t n       = 0UZ;
    std::size_t payload = 0UZ; //!< bits one record carries

    std::unique_ptr<aff3ct::module::Encoder<int>>             encoder;
    std::unique_ptr<aff3ct::module::Decoder_SIHO<int, float>> decoder;
    std::unique_ptr<TreeCrc>                                  crc;

    std::vector<int>             frame{};
    std::vector<int>             coded{};
    std::vector<float>           llr{};
    std::array<std::int8_t, 1UZ> status{}; //!< the codeword status AFF3CT writes, one entry per frame
};

PolarCodec::PolarCodec(const PolarSettings& settings) : _impl(std::make_unique<Impl>()) {
    const std::string configuration = std::format("({}, {}) {} decoder {}", settings.n, settings.k, settings.frozenConstruction, settings.decoder);
    if (settings.n == 0UZ || (settings.n & (settings.n - 1UZ)) != 0UZ) {
        throw gr::exception(std::format("Polar ({}): n must be a power of two, got {}", configuration, settings.n));
    }
    if (settings.k == 0UZ || settings.k >= settings.n) {
        throw gr::exception(std::format("Polar ({}): k must lie between 1 and n - 1, got {}", configuration, settings.k));
    }
    const bool aided = settings.decoder == "ca_scl";
    if (aided && settings.crcWidth == 0UZ) {
        throw gr::exception(std::format("Polar ({}): decoder 'ca_scl' needs a CRC to choose a surviving path and 'crc_width' is 0", configuration));
    }
    if (!aided && settings.crcWidth != 0UZ) {
        throw gr::exception(std::format("Polar ({}): only decoder 'ca_scl' uses a CRC, and 'crc_width' is {}", configuration, settings.crcWidth));
    }
    if (settings.crcWidth >= settings.k) {
        throw gr::exception(std::format("Polar ({}): crc_width {} leaves no payload inside k {}", configuration, settings.crcWidth, settings.k));
    }
    _impl->k       = settings.k;
    _impl->n       = settings.n;
    _impl->payload = settings.k - settings.crcWidth;
    if (aided && _impl->payload % 8UZ != 0UZ) {
        throw gr::exception(std::format("Polar ({}): the tree's CRC kernel reads whole bytes, so k less crc_width must be a multiple of 8, and it is {}", configuration, _impl->payload));
    }
    if ((settings.decoder == "scl" || aided) && settings.listSize == 0UZ) {
        throw gr::exception(std::format("Polar ({}): list_size is 0 — a list decoder with no list is not one", configuration));
    }

    try {
        std::vector<bool>                                    frozen(settings.n, false);
        std::unique_ptr<aff3ct::tools::Frozenbits_generator> generator;
        if (settings.frozenConstruction == "5g") {
            // The release ships the 5G reliability sequence as a data file and its installed package
            // does not tell the library where that file went, so the wall names it: the sequence is
            // read from the release's own conf directory, which the build supplies. Only the 1024-bit
            // sequence is shipped, and subsampling it to a shorter code is the standard's own rule
            // rather than a file, so a shorter code is refused here rather than approximated.
            const std::string directory(GR4_AFF3CT_POLAR_CONF_DIR);
            if (directory.empty()) {
                throw gr::exception(std::format("Polar ({}): frozen_construction '5g' reads the release's reliability sequence, and this build was configured without the release's sequence directory; use 'ga'", configuration));
            }
            if (settings.n != kPolar5gLength) {
                throw gr::exception(std::format("Polar ({}): the release ships the 5G reliability sequence at n = {} only, and n is {}; use 'ga'", configuration, kPolar5gLength, settings.n));
            }
            const std::string path = (std::filesystem::path(directory) / "5G" / "N_1024.pc").string();
            if (!std::filesystem::exists(path)) {
                throw gr::exception(std::format("Polar ({}): the 5G reliability sequence should be at '{}' and is not there", configuration, path));
            }
            generator = std::make_unique<aff3ct::tools::Frozenbits_generator_file>(static_cast<int>(settings.k), static_cast<int>(settings.n), path);
        } else if (settings.frozenConstruction == "ga") {
            generator = std::make_unique<aff3ct::tools::Frozenbits_generator_GA_Arikan>(static_cast<int>(settings.k), static_cast<int>(settings.n));
        } else {
            throw gr::exception(std::format("Polar ({}): frozen_construction must be 'ga' or '5g', got '{}'", configuration, settings.frozenConstruction));
        }
        // Every construction is handed the design point whether or not it reads it: the Gaussian
        // approximation is evaluated at one noise level and that level is what "design SNR" means,
        // while a tabulated sequence ignores it. The number is the design Eb/N0 turned into the
        // standard deviation of antipodal signaling at this code's own rate.
        const double rate  = static_cast<double>(settings.k) / static_cast<double>(settings.n);
        const double esn0  = settings.designSnrDb + 10.0 * std::log10(rate);
        const double sigma = std::sqrt(1.0 / (2.0 * std::pow(10.0, esn0 / 10.0)));
        // The generator keeps a pointer to the noise rather than a copy, so the design point has to
        // outlive the call that reads it; a temporary here is a dangling read inside generate().
        const aff3ct::tools::Sigma<float> designPoint(static_cast<float>(sigma));
        generator->set_noise(designPoint);
        generator->generate(frozen);

        _impl->encoder = std::make_unique<aff3ct::module::Encoder_polar_sys<int>>(static_cast<int>(settings.k), static_cast<int>(settings.n), frozen);

        const int ki = static_cast<int>(settings.k);
        const int ni = static_cast<int>(settings.n);
        if (settings.decoder == "sc") {
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_polar_SC_naive_sys<int, float>>(ki, ni, frozen);
        } else if (settings.decoder == "scl") {
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_polar_SCL_naive_sys<int, float>>(ki, ni, static_cast<int>(settings.listSize), frozen);
        } else if (aided) {
            const gr::digital::Crc kernel(static_cast<std::uint8_t>(settings.crcWidth), settings.crcPolynomial, settings.crcInitialValue, settings.crcFinalXor, settings.crcInputReflected, settings.crcResultReflected);
            _impl->crc     = std::make_unique<TreeCrc>(static_cast<int>(_impl->payload), static_cast<int>(settings.crcWidth), kernel);
            _impl->decoder = std::make_unique<aff3ct::module::Decoder_polar_SCL_naive_CA_sys<int, float>>(ki, ni, static_cast<int>(settings.listSize), frozen, *_impl->crc);
        } else {
            throw gr::exception(std::format("Polar ({}): decoder must be 'sc', 'scl' or 'ca_scl', got '{}'", configuration, settings.decoder));
        }
        // The list decoders this release ships in a *_fast_* form are broken at this tag, so the wrap
        // takes the naive forms, which are the reference implementations and are correct.
    } catch (const gr::exception&) {
        throw;
    } catch (const std::exception& refusal) {
        reraise("Polar", configuration, refusal);
    }

    _impl->frame.resize(_impl->k);
    _impl->coded.resize(_impl->n);
    _impl->llr.resize(_impl->n);
}

PolarCodec::~PolarCodec()                                = default;
PolarCodec::PolarCodec(PolarCodec&&) noexcept            = default;
PolarCodec& PolarCodec::operator=(PolarCodec&&) noexcept = default;

std::size_t PolarCodec::payloadBits() const noexcept { return _impl->payload; }
std::size_t PolarCodec::codedBits() const noexcept { return _impl->n; }

void PolarCodec::encode(std::span<const std::uint8_t> payload, std::span<std::uint8_t> coded) {
    _impl->frame.assign(_impl->k, 0);
    for (std::size_t i = 0UZ; i < _impl->payload && i < payload.size(); ++i) {
        _impl->frame[i] = static_cast<int>(payload[i] & 1U);
    }
    try {
        if (_impl->crc) {
            // The signature is written straight into the k-bit frame rather than through AFF3CT's own
            // build task: the wall owns this CRC, and one code path that both the encoder and the list
            // decoder's path check go through is one fewer place for the two to disagree.
            _impl->crc->sign(_impl->frame.data());
        }
        _impl->encoder->encode(_impl->frame, _impl->coded);
    } catch (const std::exception& refusal) {
        reraise("Polar", "encode", refusal);
    }
    fromFrame(_impl->coded, coded);
}

DecodeReport PolarCodec::decode(std::span<const float> llr, std::span<std::uint8_t> payload) {
    toAff3ctLlr(llr, _impl->llr);
    try {
        // The three-argument vector overload cannot deduce its allocator, since the status frame is int8_t
        // and the information frame is int and the signature shares one allocator parameter between them;
        // the buffers are exactly N and K long, so the pointer form is the same call without the check.
        std::ignore = _impl->decoder->decode_siho(_impl->llr.data(), _impl->status.data(), _impl->frame.data());
    } catch (const std::exception& refusal) {
        reraise("Polar", "decode", refusal);
    }
    for (std::size_t i = 0UZ; i < payload.size() && i < _impl->payload; ++i) {
        payload[i] = static_cast<std::uint8_t>(_impl->frame[i] & 1);
    }

    DecodeReport report;
    if (_impl->crc) {
        // The list decoder falls back to its best path when no survivor checks, so the refusal is
        // the tree's own CRC asked once more of the answer actually delivered.
        report.refused = !_impl->crc->signed_correctly(_impl->frame.data());
    }
    try {
        _impl->encoder->encode(_impl->frame, _impl->coded);
    } catch (const std::exception& refusal) {
        reraise("Polar", "decode", refusal);
    }
    for (std::size_t i = 0UZ; i < _impl->n && i < llr.size(); ++i) {
        report.correctedErrors += (slice(llr[i]) != (_impl->coded[i] & 1)) ? 1UZ : 0UZ;
    }
    return report;
}

} // namespace gr::blocks::fec::wall
