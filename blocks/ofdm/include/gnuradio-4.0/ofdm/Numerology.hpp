#ifndef GNURADIO_OFDM_NUMEROLOGY_HPP
#define GNURADIO_OFDM_NUMEROLOGY_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/ofdm/CarrierMap.hpp>

// The symbol record the blocks of this family exchange, and the settings vocabulary they share.
//
// One OFDM symbol is one `DataSet<std::complex<float>>` of `fft_len` values in FFT bin order. DC is at index 0, the
// positive carriers ascend, and the negative ones fill the upper half. Settings name carriers by signed logical index,
// and `gr::ofdm::CarrierMap` converts them to bins. The record's axis holds the signed index of each value.

namespace gr::blocks::ofdm {

using Complex = std::complex<float>;

namespace detail {

/// The transform lengths the blocks accept. Eight bins is the family's minimum.
/// The upper bound is the carrier map's own.
inline constexpr std::size_t kMinFftLength = 8UZ;
inline constexpr std::size_t kMaxFftLength = gr::ofdm::CarrierMap::kMaxFftLength;

/// @brief The meta keys a symbol record carries.
inline constexpr std::string_view kFftLenKey       = "fft_len";
inline constexpr std::string_view kSymbolIndexKey  = "symbol_index";
inline constexpr std::string_view kFrameIndexKey   = "frame_index";
inline constexpr std::string_view kSymbolInFrame   = "symbol_in_frame";
inline constexpr std::string_view kSymbolKindKey   = "symbol_kind";
inline constexpr std::string_view kSampleStartKey  = "sample_start";
inline constexpr std::string_view kPadCarriersKey  = "pad_carriers";
inline constexpr std::string_view kDiscardedKey    = "discarded_samples";
inline constexpr std::string_view kTimingOffsetKey = "timing_offset";

/// The record kinds written under `symbol_kind`.
inline constexpr std::string_view kKindSync = "sync";
inline constexpr std::string_view kKindData = "data";

/// @brief Refuses a transform length that is not a power of two between the two bounds.
inline void requireFftLength(gr::Size_t fftLength) {
    const auto value = static_cast<std::size_t>(fftLength);
    if (value < kMinFftLength || value > kMaxFftLength || (value & (value - 1UZ)) != 0UZ) {
        throw gr::exception(std::format("fft_len must be a power of two in [{}, {}], got {}", kMinFftLength, kMaxFftLength, fftLength));
    }
}

/// @brief Builds the numerology. A numerology the carrier map refuses throws the map's gr::exception.
[[nodiscard]] inline gr::ofdm::CarrierMap buildMap(gr::Size_t fftLength, std::span<const std::int32_t> dataCarriers, std::span<const std::int32_t> pilotCarriers) {
    requireFftLength(fftLength);
    // The carrier map takes `int`, and the settings hold `std::int32_t`. The copy converts between the two.
    const std::vector<int> data(dataCarriers.begin(), dataCarriers.end());
    const std::vector<int> pilots(pilotCarriers.begin(), pilotCarriers.end());
    return gr::ofdm::CarrierMap(static_cast<std::size_t>(fftLength), std::span<const int>(data), std::span<const int>(pilots));
}

/// @brief Throws a failed numerology check as `gr::exception`.
inline void requireValid(const std::expected<void, gr::Error>& checked) {
    if (!checked) {
        throw gr::exception(checked.error().message, checked.error().sourceLocation);
    }
}

/// @brief The signed carrier of FFT bin @p bin. A bin outside the transform throws the carrier map's message.
[[nodiscard]] inline int carrierAt(std::size_t fftLength, std::size_t bin) {
    const auto carrier = gr::ofdm::CarrierMap::carrierOf(fftLength, bin);
    if (!carrier) {
        throw gr::exception(carrier.error().message, carrier.error().sourceLocation);
    }
    return *carrier;
}

/// @brief Converts an interleaved re,im setting to complex values. An odd length throws and names the setting.
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
 * @brief The sync words a setting holds, one whole symbol each.
 *
 * The setting holds the words as one concatenation of interleaved re,im pairs, and its length gives the word count. A
 * length that is not a whole number of symbols throws. A short final word would otherwise go out as a symbol with
 * silent carriers.
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

/// @brief The cyclic-prefix length of symbol @p symbolInFrame.
///
/// One entry is a constant prefix. Several entries form a cycle indexed by the symbol's position in its frame. The
/// cycle restarts with every frame.
[[nodiscard]] inline std::size_t cyclicPrefixLength(std::span<const gr::Size_t> cycle, std::size_t symbolInFrame) noexcept { return static_cast<std::size_t>(cycle[symbolInFrame % cycle.size()]); }

/// @brief Refuses an empty prefix cycle and any entry longer than `fft_len`.
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
 * @brief One OFDM symbol as a record in the layout above.
 *
 * @param values     the symbol's values: `fft_len` values in bin order, or a subset such as the data carriers
 * @param carriers   the signed carrier index of each value, written as the record's axis
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
    // Complex values have no ordering. The range vector holds one default entry, `{0, 0}`.
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
 * @brief The invariant part of a symbol record, built once and copy-assigned into each output slot.
 *
 * Copy assignment reuses the storage of a slot the port has used before. Vectors keep their capacity, strings their
 * buffers and the map its nodes. Each record then writes only its values and the meta entries that change.
 */
class SymbolRecordShape {
public:
    /// @brief Builds the shape for a record with one value per entry of @p carriers. Every record carries
    /// @p constantMeta.
    void build(std::span<const int> carriers, std::string_view signalName, property_map constantMeta) { _skeleton = makeSymbolRecord(std::vector<Complex>(carriers.size(), Complex{}), carriers, kKindData, signalName, std::move(constantMeta)); }

    [[nodiscard]] bool built() const noexcept { return !_skeleton.signal_values.empty(); }

    /// @brief Writes @p values into @p target on this shape and returns the record's meta map.
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
