#ifndef GNURADIO_TESTING_INIT_CHECKS_HPP
#define GNURADIO_TESTING_INIT_CHECKS_HPP

#include <algorithm>
#include <cstddef>
#include <expected>
#include <format>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Settings.hpp>
#include <gnuradio-4.0/Tag.hpp>

namespace gr::testing {

/// A test's stand-in for the top of a block's work call: applies the changes a running block holds and returns the
/// keys that failed.
inline settings::Rejected applyPending(SettingsBase& blockSettings) { return settings::detail::FrameworkAccess::applyStagedParameters(blockSettings); }

/// The value a read returns for each key that holds one.
[[nodiscard]] inline property_map inForce(const SettingsBase& blockSettings) { return settings::heldValues(blockSettings.get()); }

/// Sets `changes` without waiting, then applies what the block holds, as its next work call would. Returns the call's
/// refusals, or each key that failed at the call or at the apply; empty when every key applied.
inline settings::Rejected setAndApply(SettingsBase& blockSettings, const property_map& changes) {
    const std::expected<settings::Values, settings::Rejected> result = blockSettings.set(changes, settings::noWait);
    if (!result.has_value()) {
        return result.error();
    }
    settings::Rejected failed;
    for (const auto& [key, entry] : *result) {
        if (entry.isFailed()) {
            failed.push_back(settings::Rejection{.key = key, .reason = entry.reason});
        }
    }
    std::ranges::move(applyPending(blockSettings), std::back_inserter(failed));
    return failed;
}

/// One setting at a value other than its default, inside its limits.
struct OffDefault {
    std::string key;
    pmt::Value  value;
};

/// Initializes a `TBlock` built at its defaults, then one `TBlock` per entry of `offDefault` built with that setting
/// alone, and returns one line per initialization that the block refused, naming the setting and the reasons. The
/// first `check` of a block's settings initializes them, and the settings hook runs then with every setting changed.
template<typename TBlock>
[[nodiscard]] std::vector<std::string> initFailures(const std::vector<OffDefault>& offDefault) {
    std::vector<std::string> failures;
    const auto               record = [&failures](std::string_view built, const settings::Rejected& refused) {
        std::string line = std::format("built {}:", built);
        for (const settings::Rejection& entry : refused) {
            line += std::format(" '{}' {}", entry.key, entry.reason);
        }
        failures.push_back(std::move(line));
    };
    {
        TBlock block;
        if (auto result = block.check({}); !result.has_value()) {
            record("at its defaults", result.error());
        }
    }
    for (const OffDefault& entry : offDefault) {
        TBlock block(property_map{{std::pmr::string(entry.key), entry.value}});
        if (auto result = block.check({}); !result.has_value()) {
            record(std::format("with {}", entry.key), result.error());
        }
    }
    return failures;
}

/// True when a block built from `constructed` refuses its initialization.
template<typename TBlock>
[[nodiscard]] bool initRefused(property_map constructed) {
    TBlock block(std::move(constructed));
    return !block.check({}).has_value();
}

/// For a block whose declared defaults its check refuses: not ready at its defaults, ready with `required`, and still
/// ready with each entry of `offDefault` added to `required`. Returns one line per expectation that failed.
template<typename TBlock>
[[nodiscard]] std::vector<std::string> requiredInitFailures(const property_map& required, const std::vector<OffDefault>& offDefault) {
    std::vector<std::string> failures;
    if (!initRefused<TBlock>({})) {
        failures.emplace_back("ready at its defaults");
    }
    if (initRefused<TBlock>(required)) {
        failures.emplace_back("refused with its required settings");
    }
    for (const OffDefault& entry : offDefault) {
        property_map settings = required;
        settings.insert_or_assign(std::pmr::string(entry.key), entry.value);
        if (initRefused<TBlock>(std::move(settings))) {
            failures.push_back(std::format("refused with {}", entry.key));
        }
    }
    return failures;
}

/// Sets `changes` on `block`, which no graph holds, so they apply at the call. True when the block refuses every key,
/// in its check or in its apply.
template<typename TBlock>
[[nodiscard]] bool refusesAll(TBlock& block, property_map changes) {
    const std::size_t keys   = changes.size();
    const auto        result = block.set(std::move(changes));
    if (!result.has_value()) {
        return true; // the check refused the whole call
    }
    return static_cast<std::size_t>(std::ranges::count_if(*result, [](const auto& entry) { return entry.second.isFailed(); })) == keys;
}

} // namespace gr::testing

#endif // GNURADIO_TESTING_INIT_CHECKS_HPP
