#ifndef GNURADIO_OFDM_NUMEROLOGY_HPP
#define GNURADIO_OFDM_NUMEROLOGY_HPP

#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/ofdm/CarrierMap.hpp>

namespace gr::blocks::ofdm {

using Complex = std::complex<float>;

/// @brief The record every block in this module emits or consumes, and the settings vocabulary they share.
///
/// One OFDM symbol is one `DataSet<std::complex<float>>` of `fft_len` values in FFT bin order. DC is at index 0, the
/// positive carriers ascend, and the negative ones fill the upper half. Carrier indices in settings are signed logical
/// indices. `gr::ofdm::CarrierMap` alone converts them to bins, and nothing here repeats that arithmetic. The record's
/// axis carries the signed index of each value. A consumer reads the mapping off the record and does not re-derive it.
namespace detail {

/// The transform lengths the family accepts. The lower bound is the shortest transform a numerology with guards,
/// pilots and data can populate. The upper bound is the kernel's own ceiling.
inline constexpr std::size_t kMinFftLength = 8UZ;
inline constexpr std::size_t kMaxFftLength = gr::ofdm::CarrierMap::kMaxFftLength;

/// @brief The meta keys a symbol record carries, named once so the four blocks cannot spell them differently.
inline constexpr std::string_view kFftLenKey       = "fft_len";
inline constexpr std::string_view kSymbolIndexKey  = "symbol_index";
inline constexpr std::string_view kFrameIndexKey   = "frame_index";
inline constexpr std::string_view kSymbolInFrame   = "symbol_in_frame";
inline constexpr std::string_view kSymbolKindKey   = "symbol_kind";
inline constexpr std::string_view kSampleStartKey  = "sample_start";
inline constexpr std::string_view kPadCarriersKey  = "pad_carriers";
inline constexpr std::string_view kDiscardedKey    = "discarded_samples";
inline constexpr std::string_view kCpeRadKey       = "cpe_rad";
inline constexpr std::string_view kEvmDbKey        = "evm_db";
inline constexpr std::string_view kFrameEvmDbKey   = "frame_evm_db";
inline constexpr std::string_view kTimingOffsetKey = "timing_offset";

/// What a record holds, written under `symbol_kind`.
inline constexpr std::string_view kKindSync      = "sync";
inline constexpr std::string_view kKindData      = "data";
inline constexpr std::string_view kKindEqualized = "equalized";

/**
 * @brief Complex multiply, divide and reciprocal written out.
 *
 * The language's own operators call library helpers that preserve infinities and guard the exponent range of the
 * division. Each helper is a function call inside a loop that runs once per carrier per symbol. The numerology bounds
 * the values these functions see. A carrier's channel estimate comes from a sync word that every occupied carrier
 * must occupy. A zero divisor is therefore refused at configure time and never reaches these functions. No quantity
 * in an OFDM symbol approaches the exponent range the guarded form exists for.
 */
[[nodiscard]] inline constexpr Complex multiply(Complex a, Complex b) noexcept { return Complex(a.real() * b.real() - a.imag() * b.imag(), a.real() * b.imag() + a.imag() * b.real()); }

[[nodiscard]] inline constexpr Complex reciprocal(Complex a) noexcept {
    const float scale = 1.f / (a.real() * a.real() + a.imag() * a.imag());
    return Complex(a.real() * scale, -a.imag() * scale);
}

[[nodiscard]] inline constexpr Complex divide(Complex a, Complex b) noexcept { return multiply(a, reciprocal(b)); }

/// @brief Reject a transform length the family cannot address.
inline void requireFftLength(gr::Size_t fftLength) {
    const auto value = static_cast<std::size_t>(fftLength);
    if (value < kMinFftLength || value > kMaxFftLength || (value & (value - 1UZ)) != 0UZ) {
        throw gr::exception(std::format("fft_len must be a power of two in [{}, {}], got {}", kMinFftLength, kMaxFftLength, fftLength));
    }
}

/// @brief Build the numerology, restating the kernel's refusal as the failure a graph reports.
[[nodiscard]] inline gr::ofdm::CarrierMap buildMap(gr::Size_t fftLength, std::span<const std::int32_t> dataCarriers, std::span<const std::int32_t> pilotCarriers) {
    requireFftLength(fftLength);
    // The kernel takes `int`, the same width as the settings type on every platform this builds for. The copy lets
    // the two types differ without this file noticing.
    const std::vector<int> data(dataCarriers.begin(), dataCarriers.end());
    const std::vector<int> pilots(pilotCarriers.begin(), pilotCarriers.end());
    try {
        return gr::ofdm::CarrierMap(static_cast<std::size_t>(fftLength), std::span<const int>(data), std::span<const int>(pilots));
    } catch (const std::invalid_argument& error) {
        throw gr::exception(error.what());
    }
}

/// @brief An interleaved re,im setting as complex values, refusing an odd length by name.
[[nodiscard]] inline std::vector<Complex> complexFrom(std::span<const float> interleaved, std::string_view setting) {
    if (interleaved.size() % 2UZ != 0UZ) {
        throw gr::exception(std::format("{} is a list of interleaved re,im pairs and must have an even length, got {}", setting, interleaved.size()));
    }
    std::vector<Complex> values(interleaved.size() / 2UZ);
    for (std::size_t k = 0UZ; k < values.size(); ++k) {
        values[k] = Complex(interleaved[2UZ * k], interleaved[2UZ * k + 1UZ]);
    }
    return values;
}

/**
 * @brief The sync words a setting holds, as whole symbols.
 *
 * A setting carries no vector of vectors. The words are one concatenation of interleaved re,im pairs, and the length
 * gives their count. A concatenation that is not a whole number of symbols is refused, not truncated. A short final
 * word would be emitted as a symbol with silent carriers. A receiver cannot tell that failure from a real symbol.
 */
[[nodiscard]] inline std::vector<std::vector<Complex>> syncWordsFrom(std::span<const float> interleaved, std::size_t fftLength) {
    const std::vector<Complex> flat = complexFrom(interleaved, "sync_words");
    if (flat.size() % fftLength != 0UZ) {
        throw gr::exception(std::format("sync_words holds {} carriers, which is not a whole number of {}-carrier symbols", flat.size(), fftLength));
    }
    std::vector<std::vector<Complex>> words(flat.size() / fftLength);
    for (std::size_t w = 0UZ; w < words.size(); ++w) {
        words[w].assign(flat.begin() + static_cast<std::ptrdiff_t>(w * fftLength), flat.begin() + static_cast<std::ptrdiff_t>((w + 1UZ) * fftLength));
    }
    return words;
}

/// @brief The cyclic-prefix length of symbol @p symbolInFrame, from a setting that is a scalar or a cycle.
///
/// One setting expresses both shapes. A single entry is a constant prefix, and several entries are read in turn. The
/// cycle expresses the long-then-short opening of 802.11. The cycle position is the symbol's index within its frame.
/// The cycle therefore restarts with every frame and does not drift against it.
[[nodiscard]] inline std::size_t cyclicPrefixLength(std::span<const gr::Size_t> cycle, std::size_t symbolInFrame) noexcept { return static_cast<std::size_t>(cycle[symbolInFrame % cycle.size()]); }

/// @brief Reject a prefix cycle that says nothing, or one longer than the symbol it precedes.
inline void requireCyclicPrefix(std::span<const gr::Size_t> cycle, gr::Size_t fftLength) {
    if (cycle.empty()) {
        throw gr::exception("cp_len is the prefix length, a scalar or a per-symbol cycle, and must hold at least one entry");
    }
    for (const gr::Size_t length : cycle) {
        if (length > fftLength) {
            throw gr::exception(std::format("cp_len entry {} exceeds fft_len {}: a prefix is a copy of the symbol's own tail and cannot be longer than the symbol", length, fftLength));
        }
    }
}

/**
 * @brief One OFDM symbol as a record in the family's layout.
 *
 * @param values     the symbol, `fft_len` values in bin order, or the data carriers alone for an equalizer output
 * @param carriers   the signed logical index each value belongs to, which becomes the record's axis
 *
 * The axis makes the record self-describing. A consumer reads the carrier each value came from and does not
 * re-derive the signed-to-bin map. An equalizer output holds a subset of the symbol's values and states that subset
 * in the same place a whole symbol does.
 */
[[nodiscard]] inline DataSet<Complex> makeSymbolRecord(std::vector<Complex> values, std::span<const int> carriers, std::string_view kind, std::string_view signalName, property_map extra) {
    DataSet<Complex>  ds;
    const std::size_t count = values.size();

    ds.extents = {static_cast<std::int32_t>(count)};
    ds.layout  = gr::LayoutRight{};

    ds.axis_names = {"Carrier"};
    ds.axis_units = {"1"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(count);
    for (std::size_t k = 0UZ; k < count; ++k) {
        // The axis of a complex record is complex. A carrier index is real, and the imaginary part stays zero.
        ds.axis_values[0UZ][k] = Complex(static_cast<float>(carriers[k]), 0.f);
    }

    ds.signal_names      = {std::string(signalName)};
    ds.signal_quantities = {"OfdmSymbol"};
    ds.signal_units      = {"a.u."};
    ds.signal_values     = std::move(values);
    // A complex range would have to order two samples, and the type has no such comparison. The vector is sized for
    // a consumer that indexes it and left at the default, which states no limit.
    ds.signal_ranges.resize(1UZ);

    ds.meta_information.resize(1UZ);
    property_map& meta                     = ds.meta_information[0UZ];
    meta[std::pmr::string(kSymbolKindKey)] = pmt::Value(std::string(kind));
    for (auto& [key, value] : extra) {
        meta[key] = value;
    }

    ds.timing_events.resize(1UZ);
    ds.timestamp = 0;
    return ds;
}

/**
 * @brief The invariant part of a symbol record, built once and copy-assigned into the slot the port returns.
 *
 * A record is a dozen containers and a map. Building one from nothing per symbol costs more than the transform that
 * produced it. An output port returns a slot it has already used, and the assignment reuses that slot's storage.
 * Vectors keep their capacity, strings their buffers, and the map its nodes. Per record, only the values and the few
 * meta entries that change remain to write. The FFT block of the fourier module keeps its record template the same
 * way for the same reason.
 */
class SymbolRecordShape {
public:
    /// @brief Build the shape for a record of @p carriers values. @p constantMeta is what every record repeats.
    void build(std::span<const int> carriers, std::string_view signalName, property_map constantMeta) { _skeleton = makeSymbolRecord(std::vector<Complex>(carriers.size(), Complex{}), carriers, kKindData, signalName, std::move(constantMeta)); }

    [[nodiscard]] bool built() const noexcept { return !_skeleton.signal_values.empty(); }

    /// @brief Writes @p values into @p target on this shape and returns the meta map for the caller to stamp.
    [[nodiscard]] property_map& emitInto(DataSet<Complex>& target, std::span<const Complex> values, std::string_view kind) const {
        target = _skeleton;
        std::ranges::copy(values, target.signal_values.begin());
        property_map& meta                     = target.meta_information[0UZ];
        meta[std::pmr::string(kSymbolKindKey)] = pmt::Value(std::string(kind));
        return meta;
    }

private:
    DataSet<Complex> _skeleton{};
};

/// @brief The unsigned meta value under @p key, or @p fallback when the record does not carry one.
[[nodiscard]] inline std::uint64_t metaCount(const DataSet<Complex>& record, std::string_view key, std::uint64_t fallback) noexcept {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto entry = record.meta_information[0UZ].find(std::pmr::string(key));
    if (entry == record.meta_information[0UZ].end()) {
        return fallback;
    }
    if (const auto* value = entry->second.get_if<std::uint64_t>(); value != nullptr) {
        return *value;
    }
    return fallback;
}

} // namespace detail

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_NUMEROLOGY_HPP
