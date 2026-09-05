#ifndef GNURADIO_CCSDS_RECORD_HELPERS_HPP
#define GNURADIO_CCSDS_RECORD_HELPERS_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <initializer_list>
#include <iterator>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

/**
 * @brief The record-shaping helpers the CCSDS blocks share: metadata reads, the `discontinuity` cause list, the
 * fields every published `DataSet<std::uint8_t>` needs before a block writes its own keys, and the counter report.
 *
 * One definition each, because two blocks that shape a record differently produce records a downstream block has to
 * tell apart, and a metadata read that answers "absent" in one block and throws in another turns a wire value into
 * a block-dependent outcome. Every read here treats a value of the wrong type as absent: a record that has crossed
 * a network can carry anything under a key, and a guess at what a mistyped value meant is a guess a decoder would
 * then act on.
 */
namespace gr::blocks::ccsds::detail {

/// @brief "No value was given" for a `gr::Size_t` setting, outside every field width this module validates.
inline constexpr gr::Size_t kUnset = 0xFFFFFFFFU;

/// @brief The record's metadata map, or `nullptr` where it carries none.
[[nodiscard]] inline const property_map* metaOf(const DataSet<std::uint8_t>& record) noexcept { return record.meta_information.empty() ? nullptr : &record.meta_information[0UZ]; }

/// @brief @p key read as a `bool`, or `std::nullopt` where it is absent, absent from an absent map, or another type.
[[nodiscard]] inline std::optional<bool> readBool(const property_map* map, const char* key) {
    if (map == nullptr) {
        return std::nullopt;
    }
    const auto it = map->find(property_map::key_type(key));
    if (it == map->end()) {
        return std::nullopt;
    }
    if (const bool* value = it->second.get_if<bool>(); value != nullptr) {
        return *value;
    }
    return std::nullopt;
}

/// @brief @p key read as a `gr::Size_t`, or `std::nullopt` where it is absent, absent from an absent map, or another type.
[[nodiscard]] inline std::optional<gr::Size_t> readSize(const property_map* map, const char* key) {
    if (map == nullptr) {
        return std::nullopt;
    }
    const auto it = map->find(property_map::key_type(key));
    if (it == map->end()) {
        return std::nullopt;
    }
    if (const gr::Size_t* value = it->second.get_if<gr::Size_t>(); value != nullptr) {
        return *value;
    }
    return std::nullopt;
}

/// @brief Whether the comma-separated cause list @p causes already names @p cause.
[[nodiscard]] inline bool hasCause(std::string_view causes, std::string_view cause) noexcept {
    for (std::size_t at = 0UZ; at <= causes.size();) {
        const std::size_t end = std::min(causes.find(',', at), causes.size());
        if (causes.substr(at, end - at) == cause) {
            return true;
        }
        at = end + 1UZ;
    }
    return false;
}

/**
 * @brief `discontinuity`'s transform rule: append the cause, never replace what is already there, and never twice.
 *
 * The key is a set of causes rather than a single label because two blocks in one chain can each break continuity
 * for their own reason, and a second cause that overwrote the first would hide the earlier break entirely. A cause
 * the list already names is left as it stands, so a record that crosses a block which re-detects the same break
 * carries one mention of it and not a run of them.
 */
inline void appendDiscontinuity(property_map& map, std::string_view cause) {
    const auto  it       = map.find(property_map::key_type("discontinuity"));
    std::string combined = it == map.end() ? std::string{} : it->second.value_or(std::string{});
    if (hasCause(combined, cause)) {
        return;
    }
    if (!combined.empty()) {
        combined += ',';
    }
    combined += cause;
    map.insert_or_assign(property_map::key_type("discontinuity"), pmt::Value(combined));
}

/// @brief Take one cause out of the `discontinuity` key, dropping the key when it names nothing else.
inline void removeDiscontinuity(property_map& map, std::string_view cause) {
    const auto it = map.find(property_map::key_type("discontinuity"));
    if (it == map.end()) {
        return;
    }
    const std::string causes = it->second.value_or(std::string{});
    std::string       kept;
    for (std::size_t at = 0UZ; at < causes.size();) {
        const std::size_t      end = std::min(causes.find(',', at), causes.size());
        const std::string_view part(causes.data() + at, end - at);
        if (!part.empty() && part != cause) {
            if (!kept.empty()) {
                kept += ',';
            }
            kept += part;
        }
        at = end + 1UZ;
    }
    if (kept.empty()) {
        map.erase(it);
    } else {
        map.insert_or_assign(property_map::key_type("discontinuity"), pmt::Value(kept));
    }
}

/// @brief The extent, signal name and metadata map every output record derived from @p in needs before its own keys
/// are written: the input's metadata crosses verbatim, and @p fallbackName names the signal where the input does not.
inline void startRecord(const DataSet<std::uint8_t>& in, DataSet<std::uint8_t>& out, std::string_view fallbackName) {
    out.extents.push_back(static_cast<std::int32_t>(out.signal_values.size()));
    out.signal_names.emplace_back(in.signal_names.empty() ? std::string(fallbackName) : in.signal_names[0UZ]);
    out.timing_events.resize(1UZ);
    out.meta_information.resize(1UZ);
    if (!in.meta_information.empty()) {
        out.meta_information[0UZ] = in.meta_information[0UZ];
    }
}

/// @brief The same fields for a record synthesized from many inputs or from none, whose metadata is the block's own.
inline void freshRecord(DataSet<std::uint8_t>& out, std::string_view name) {
    out.extents.push_back(static_cast<std::int32_t>(out.signal_values.size()));
    out.signal_names.emplace_back(name);
    out.timing_events.resize(1UZ);
    out.meta_information.resize(1UZ);
}

/// @brief One line on stderr naming @p block and every counter of @p counters that is not zero, or nothing at all
/// where none of them moved: a run in which nothing was refused says so by staying silent.
template<typename TBlock>
inline void reportCounters(const TBlock& block, std::string_view label, std::initializer_list<std::pair<std::string_view, std::uint64_t>> counters) {
    std::string report;
    for (const auto& [name, count] : counters) {
        if (count > 0ULL) {
            std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", name, count);
        }
    }
    if (!report.empty()) {
        std::println(stderr, "gr::blocks::ccsds::{} '{}': {}", label, block.name, report);
    }
}

} // namespace gr::blocks::ccsds::detail

#endif // GNURADIO_CCSDS_RECORD_HELPERS_HPP
