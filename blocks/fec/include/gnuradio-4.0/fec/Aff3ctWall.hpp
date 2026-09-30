#ifndef GNURADIO_FEC_AFF3CT_WALL_HPP
#define GNURADIO_FEC_AFF3CT_WALL_HPP

#include <cstddef>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * @brief Adapters between the FEC blocks and AFF3CT's LDPC and Polar codes.
 *
 * AFF3CT provides the LDPC and Polar families that the algorithm layer does not implement. It
 * enters as a build option, not as a dependency. The module builds whole without it, and the four
 * blocks that need it are registered only when `GR4_ENABLE_AFF3CT` is on. The blocks see only
 * this header. It names only this module's own types. It carries no AFF3CT include, exception
 * type or enumeration. The AFF3CT objects live behind a pointer to an
 * implementation type defined in `src/Aff3ctWall.cpp`. The network module's libzmq split set
 * that pattern. A version bump is then a prefix change and a rebuild of one translation unit.
 *
 * The adapter does three things itself instead of passing them through.
 *
 * **The LLR sign.** In these blocks a positive soft value carries a one, and the magnitude is
 * confidence. AFF3CT's BPSK modem maps bit 0 to +1 and bit 1 to -1. A positive AFF3CT LLR
 * therefore favors zero, the opposite sense. The adapter negates each value on the way in. The
 * rule is stated here and asserted by a sign anchor in the QA. It is never taken implicitly from
 * AFF3CT. A decode under the wrong sign returns the bitwise complement with no other symptom.
 *
 * **The CRC.** The CRC-aided list decoder needs a CRC to choose its surviving path.
 * `gr::digital::Crc` computes it, not AFF3CT's own table. One polynomial vocabulary then serves
 * every block here. The adapter wraps the kernel in the shape AFF3CT expects and passes it on.
 *
 * **The refusal.** An LDPC decode can exhaust its iterations with the syndrome still failing. A
 * list decode can end with no survivor passing the CRC. The families make these refusals, and the
 * blocks report them. The block emits the information estimate in both cases. Every other decoder
 * in this module also emits its best answer, with the counts saying what it is worth.
 *
 * The adapter catches every AFF3CT exception and re-raises it as `gr::exception`, naming the
 * family and the configuration that caused it. No foreign exception type reaches a graph.
 */
namespace gr::blocks::fec::wall {

//! What one soft decode reports back through the adapter.
struct DecodeReport {
    std::size_t correctedErrors = 0UZ;   //!< coded bits between the sliced input and the codeword decoded
    bool        refused         = false; //!< the family's own refusal, a failed syndrome or no list survivor
};

//! The LDPC configuration, in our own spelling of AFF3CT's taxonomy.
struct LdpcSettings {
    std::string standard{};                    //!< a construction the pinned release ships, or empty
    std::string alistPath{};                   //!< an explicit parity-check matrix, or empty
    std::string decoder{"normalized_min_sum"}; //!< bp_flooding, bp_horizontal_layered, min_sum, normalized_min_sum
    float       normalization = 0.75F;         //!< the normalized min-sum factor
    std::size_t iterations    = 50UZ;
    bool        earlyExit     = true; //!< stop as soon as the syndrome checks
};

//! The Polar configuration. The CRC fields are our own vocabulary and are used only by ca_scl.
struct PolarSettings {
    std::size_t n = 0UZ; //!< codeword bits, a power of two
    std::size_t k = 0UZ; //!< bits the encoder takes, the CRC bits included

    std::string frozenConstruction{"ga"}; //!< ga or 5g
    double      designSnrDb = 2.5;        //!< the Eb/N0 the Gaussian approximation is evaluated at

    std::string decoder{"sc"}; //!< sc, scl, ca_scl
    std::size_t listSize = 8UZ;

    std::size_t   crcWidth           = 0UZ;
    std::uint64_t crcPolynomial      = 0ULL;
    std::uint64_t crcInitialValue    = 0ULL;
    std::uint64_t crcFinalXor        = 0ULL;
    bool          crcInputReflected  = false;
    bool          crcResultReflected = false;
};

//! The constructions this build of the adapter can name, for a refusal that lists what it does carry.
[[nodiscard]] std::vector<std::string> ldpcStandards();

/*!
 * @brief One LDPC code and its decoder, constructed once and reused for every record.
 *
 * The dimensions are fixed at construction. An LDPC decoder holds a graph and a message store
 * sized from the parity-check matrix. Rebuilding either per record would be quietly quadratic. A
 * settings change needs a new object, which a graph rebuild provides.
 */
class LdpcCodec {
public:
    explicit LdpcCodec(const LdpcSettings& settings);
    ~LdpcCodec();
    LdpcCodec(LdpcCodec&&) noexcept;
    LdpcCodec& operator=(LdpcCodec&&) noexcept;
    LdpcCodec(const LdpcCodec&)            = delete;
    LdpcCodec& operator=(const LdpcCodec&) = delete;

    [[nodiscard]] std::size_t payloadBits() const noexcept; //!< bits one input record carries
    [[nodiscard]] std::size_t codedBits() const noexcept;   //!< bits one coded record carries

    void encode(std::span<const std::uint8_t> payload, std::span<std::uint8_t> coded);

    //! @p llr is in this module's sense, where positive is a one. The adapter negates it for AFF3CT.
    [[nodiscard]] DecodeReport decode(std::span<const float> llr, std::span<std::uint8_t> payload);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

//! One Polar code and its decoder, on LdpcCodec's contract exactly.
class PolarCodec {
public:
    explicit PolarCodec(const PolarSettings& settings);
    ~PolarCodec();
    PolarCodec(PolarCodec&&) noexcept;
    PolarCodec& operator=(PolarCodec&&) noexcept;
    PolarCodec(const PolarCodec&)            = delete;
    PolarCodec& operator=(const PolarCodec&) = delete;

    [[nodiscard]] std::size_t payloadBits() const noexcept; //!< `k` less the CRC bits the adapter appends
    [[nodiscard]] std::size_t codedBits() const noexcept;

    void encode(std::span<const std::uint8_t> payload, std::span<std::uint8_t> coded);

    [[nodiscard]] DecodeReport decode(std::span<const float> llr, std::span<std::uint8_t> payload);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace gr::blocks::fec::wall

#endif // GNURADIO_FEC_AFF3CT_WALL_HPP
