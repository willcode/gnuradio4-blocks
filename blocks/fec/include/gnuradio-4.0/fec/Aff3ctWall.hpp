#ifndef GNURADIO_FEC_AFF3CT_WALL_HPP
#define GNURADIO_FEC_AFF3CT_WALL_HPP

#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <memory>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>

/**
 * The wall between this tree and AFF3CT.
 *
 * AFF3CT carries the LDPC and Polar families this tree does not implement, and it enters as a
 * build option rather than as a dependency: the module builds whole without it and the four
 * blocks that need it are registered only when `GR4_ENABLE_AFF3CT` is on. What crosses the wall
 * is this header, which names the tree's own types and nothing of AFF3CT's — no include, no
 * exception type, no enumeration. The AFF3CT objects live behind a pointer to an implementation
 * type defined in `src/Aff3ctWall.cpp`, on the pattern the network module's libzmq split
 * established, so a version bump is a prefix change and a rebuild of one translation unit.
 *
 * Three things the wall does rather than passes through.
 *
 * **The LLR sign.** This tree's soft convention is `spec-conv-viterbi.md`'s: a positive value
 * carries a one and the magnitude is confidence. AFF3CT's BPSK modem maps bit 0 to +1 and bit 1
 * to -1, so a positive AFF3CT LLR favors zero — the opposite sense. The wall negates on the way
 * in. That is stated here, asserted by a sign anchor in the QA, and never inherited: a decode
 * under the wrong sign returns the bitwise complement with nothing else to see.
 *
 * **The CRC.** The CRC-aided list decoder needs a CRC to choose its surviving path, and it is
 * this tree's `gr::digital::Crc` that computes it rather than AFF3CT's own table, so that one
 * polynomial vocabulary serves the whole tree. The wall wraps the kernel in the shape AFF3CT
 * expects and hands that over.
 *
 * **The refusal.** An LDPC decode that exhausts its iterations with the syndrome still failing,
 * and a list decode with no survivor passing the CRC, are refusals the families can make and the
 * blocks report. The information estimate is emitted either way, as every other decoder in this
 * module emits its best answer with the counts saying what it is worth.
 *
 * Every AFF3CT exception is caught here and re-raised as `gr::exception` naming the family and
 * the configuration that caused it, so no foreign exception type reaches a graph.
 */
namespace gr::blocks::fec::wall {

//! What one soft decode reports back across the wall.
struct DecodeReport {
    std::size_t correctedErrors = 0UZ;   //!< coded bits between the sliced input and the codeword decoded
    bool        refused         = false; //!< the family's own refusal: a failed syndrome, or no list survivor
};

//! The LDPC configuration, in the tree's spelling of AFF3CT's taxonomy.
struct LdpcSettings {
    std::string standard{};                    //!< a construction the pinned release ships, or empty
    std::string alistPath{};                   //!< an explicit parity-check matrix, or empty
    std::string decoder{"normalized_min_sum"}; //!< bp_flooding, bp_horizontal_layered, min_sum, normalized_min_sum
    float       normalization = 0.75F;         //!< the normalized min-sum factor
    std::size_t iterations    = 50UZ;
    bool        earlyExit     = true; //!< stop as soon as the syndrome checks
};

//! The Polar configuration. The CRC fields are this tree's own vocabulary and are used only by ca_scl.
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

//! The constructions this build of the wall can name, for a refusal that lists what it does carry.
[[nodiscard]] std::vector<std::string> ldpcStandards();

/*!
 * @brief One LDPC code and its decoder, constructed once and reused for every record.
 *
 * The dimensions are fixed at construction: an LDPC decoder holds a graph and a message store
 * sized from the parity-check matrix, and rebuilding either per record would be quietly
 * quadratic. A settings change is a new object, which is what a graph rebuild is for.
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

    //! @p llr is in this tree's sense: positive is a one. The wall negates it for AFF3CT.
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

    [[nodiscard]] std::size_t payloadBits() const noexcept; //!< `k` less the CRC bits the wall appends
    [[nodiscard]] std::size_t codedBits() const noexcept;

    void encode(std::span<const std::uint8_t> payload, std::span<std::uint8_t> coded);

    [[nodiscard]] DecodeReport decode(std::span<const float> llr, std::span<std::uint8_t> payload);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace gr::blocks::fec::wall

namespace gr::blocks::fec::detail {

/*
 * The record shaping the two gated families share. It lives here rather than in one of the two
 * block headers because only those two include this file, and a block header that had to include
 * another block header to borrow three functions would be a worse arrangement than one copy in the
 * place they both already depend on.
 */

//! A record's value under @p key, with the key's absence answered by @p fallback.
template<typename V>
[[nodiscard]] inline V metaOrZero(const property_map& map, const char* key, V fallback) {
    if (const auto entry = map.find(property_map::key_type(key)); entry != map.end()) {
        return entry->second.value_or(V(fallback));
    }
    return fallback;
}

//! Shape @p out as this module shapes an output record, carrying @p record's facts onto it.
template<typename TOut, typename TIn>
inline void carryFrame(DataSet<TOut>& out, const DataSet<TIn>& record) {
    out.extents.push_back(static_cast<std::int32_t>(out.signal_values.size()));
    out.signal_names.emplace_back(record.signal_names.empty() ? std::string("fec") : record.signal_names[0UZ]);
    out.timing_events.resize(1UZ);
    out.meta_information.resize(1UZ);
    if (!record.meta_information.empty()) {
        out.meta_information[0UZ] = record.meta_information[0UZ]; // the record's facts carry through
    }
}

//! One line of counters at `stop()`, in the module's landed shape.
inline void reportFrames(std::string_view block, std::string_view name, std::span<const std::pair<std::string_view, std::uint64_t>> counters) {
    std::string line;
    for (const auto& [label, count] : counters) {
        if (count > 0ULL) {
            std::format_to(std::back_inserter(line), "{}{}: {}", line.empty() ? "" : ", ", label, count);
        }
    }
    if (!line.empty()) {
        std::println(stderr, "{} '{}': {}", block, name, line);
    }
}

} // namespace gr::blocks::fec::detail

#endif // GNURADIO_FEC_AFF3CT_WALL_HPP
