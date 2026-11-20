#ifndef GNURADIO_TESTING_SETTINGS_LIMITS_HPP
#define GNURADIO_TESTING_SETTINGS_LIMITS_HPP

#include <string>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

namespace gr::blocks::testing {

/// Returns the names of the settings whose values lie outside the `Limits<>` the block declares for them.
/// A setting without `Limits<>` is never listed. A block that holds its settings in a `settings` member is read
/// through that member.
template<typename TBlock>
[[nodiscard]] std::vector<std::string> settingsOutsideLimits(const TBlock& block) {
    if constexpr (requires { requires gr::refl::reflectable<std::remove_cvref_t<decltype(block.settings)>>; }) {
        return settingsOutsideLimits(block.settings);
    } else {
        std::vector<std::string> outside;
        gr::refl::for_each_data_member_index<TBlock>([&](auto kIdx) {
            using RawType = std::remove_cvref_t<gr::refl::data_member_type<TBlock, kIdx>>;
            if constexpr (gr::AnnotatedType<RawType>) {
                using Limit = typename RawType::LimitType;
                if constexpr (!std::is_same_v<Limit, gr::EmptyLimit>) {
                    if (!Limit::validate(gr::refl::data_member<kIdx>(block).value)) {
                        outside.emplace_back(gr::refl::data_member_name<TBlock, kIdx>.view());
                    }
                }
            }
        });
        return outside;
    }
}

} // namespace gr::blocks::testing

#endif // GNURADIO_TESTING_SETTINGS_LIMITS_HPP
