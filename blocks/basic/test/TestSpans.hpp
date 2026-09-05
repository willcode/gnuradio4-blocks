#ifndef GNURADIO_BASIC_TEST_SPANS_HPP
#define GNURADIO_BASIC_TEST_SPANS_HPP

#include <algorithm>
#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

// Minimal InputSpanLike/OutputSpanLike pair, so a processBulk that publishes its own tags can be driven at an exact
// chunk size and from an exact absolute stream position without standing up a graph and a scheduler.
namespace gr::blocks::basic::test {

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr TagReaderSpan(std::span<const gr::Tag> tags) : std::span<const gr::Tag>(tags) {}
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
    constexpr TagWriterSpan(std::span<gr::Tag> tags) : std::span<gr::Tag>(tags) {}
    constexpr void publish(std::size_t) const noexcept {}
};

template<typename T>
struct InputSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    bool          isConnected = true;
    bool          isSync      = true;

    InputSpan(std::span<const T> samples, std::size_t at = 0UZ, std::span<const gr::Tag> tags = {}) : std::span<const T>(samples), rawTags(tags), streamIndex(at) {}

    constexpr bool consume(std::size_t nSamples) noexcept {
        consumed = nSamples;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::span<const gr::Tag> tags() const noexcept { return rawTags; }
    [[nodiscard]] std::span<const gr::Tag> tags(std::size_t) const noexcept { return rawTags; }
};

template<typename T>
struct OutputSpan : std::span<T> {
    using value_type = T;

    std::vector<gr::Tag>* sink = nullptr;
    TagWriterSpan         tags{};
    std::size_t           streamIndex = 0UZ;
    std::size_t           count       = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = true;

    OutputSpan(std::span<T> samples, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr) : std::span<T>(samples), sink(published), streamIndex(at) {}

    constexpr void publish(std::size_t nSamples) noexcept { count = nSamples; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (sink != nullptr) {
            sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
        }
    }
};

template<typename T>
struct Result {
    std::vector<T>       samples{};
    std::vector<gr::Tag> tags{};
    std::size_t          consumed = 0UZ;

    [[nodiscard]] std::vector<std::size_t> offsetsOf(std::string_view key) const {
        const typename gr::property_map::key_type wanted{key};
        std::vector<std::size_t>                  offsets;
        for (const gr::Tag& tag : tags) {
            if (tag.map.contains(wanted)) {
                offsets.push_back(tag.index);
            }
        }
        return offsets;
    }
};

/// @brief Drive @p block over @p input in chunks of @p chunkSize, injecting @p tags at their absolute input offsets.
template<typename TBlock, typename T>
[[nodiscard]] Result<T> run(TBlock& block, std::span<const T> input, std::size_t chunkSize = 0UZ, std::span<const gr::Tag> tags = {}) {
    Result<T>         result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<T>    scratch(stride);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);
        const auto        first = std::ranges::lower_bound(tags, base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<T>  inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutputSpan<T> outSpan(std::span<T>(scratch.data(), count), result.samples.size(), &result.tags);
        std::ignore = block.processBulk(inSpan, outSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ) { // no progress: the chunk is shorter than the group the block requires
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

} // namespace gr::blocks::basic::test

#endif // GNURADIO_BASIC_TEST_SPANS_HPP
