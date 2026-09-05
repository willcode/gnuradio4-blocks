#ifndef GNURADIO_TIMING_TEST_SPANS_HPP
#define GNURADIO_TIMING_TEST_SPANS_HPP

#include <algorithm>
#include <cstddef>
#include <functional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

// Minimal ReaderSpanLike/WriterSpanLike pair, so a processBulk can be driven at an exact chunk size and from an exact
// absolute stream position without standing up a graph and a scheduler. Tag *forwarding* is the framework's and is not
// modeled here: what these spans reach is the block's own reading of the tags and its own publishing.
namespace gr::blocks::timing::test {

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

using TagView = std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>;

template<typename T>
struct InputSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex  = 0UZ;
    std::size_t   consumed     = 0UZ;
    std::size_t   tagsConsumed = 0UZ;
    bool          isConnected  = true;
    bool          isSync       = true;

    InputSpan(std::span<const T> samples, std::size_t at = 0UZ, std::span<const gr::Tag> incoming = {}, bool sync = true) : std::span<const T>(samples), rawTags(incoming), streamIndex(at), isSync(sync) {}

    constexpr bool consume(std::size_t nSamples) noexcept {
        consumed = nSamples;
        return true;
    }
    constexpr void consumeTags(std::size_t untilLocalIndex) noexcept { tagsConsumed = untilLocalIndex; }

    [[nodiscard]] std::vector<TagView> tags() const { return tags(this->size()); }

    [[nodiscard]] std::vector<TagView> tags(std::size_t window) const {
        std::vector<TagView> view;
        for (const gr::Tag& tag : rawTags) {
            const std::ptrdiff_t relIndex = static_cast<std::ptrdiff_t>(tag.index) - static_cast<std::ptrdiff_t>(streamIndex);
            if (relIndex < static_cast<std::ptrdiff_t>(window)) {
                view.emplace_back(relIndex, std::cref(tag.map));
            }
        }
        return view;
    }
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

    OutputSpan(std::span<T> samples, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr, bool sync = true) : std::span<T>(samples), sink(published), streamIndex(at), isSync(sync) {}

    constexpr void publish(std::size_t nSamples) noexcept { count = nSamples; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (!isConnected || sink == nullptr) {
            return;
        }
        sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
    }
};

/// @brief One run of a block: everything it published, and everything it consumed.
template<typename TOut>
struct Capture {
    std::vector<TOut>    samples{};
    std::vector<gr::Tag> tags{};
    std::size_t          consumed = 0UZ;
};

/**
 * @brief Drive a one-in one-out block over @p input in chunks of @p chunkSize, with @p tags at their absolute offsets.
 *
 * @p outRoom is the output slots offered per call, which for a block whose output is a record port rather than a
 * sample port is what limits how many records one call may publish; zero offers one slot per input sample.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> run(TBlock& block, std::span<const TIn> input, std::size_t chunkSize = 0UZ, std::span<const gr::Tag> tags = {}, std::size_t outRoom = 0UZ, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut> scratch(outRoom == 0UZ ? stride : outRoom);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);
        const std::size_t room  = outRoom == 0UZ ? count : outRoom;
        const auto        first = std::ranges::lower_bound(tags, startOffset + base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, startOffset + base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(base, count), startOffset + base, std::span<const gr::Tag>(first, last));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags);
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

/**
 * @brief Drive a variable-rate block over `Async` ports: @p feed samples arrive per call and everything not yet
 * consumed is presented again, with @p outRoom output slots offered.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> runVariable(TBlock& block, std::span<const TIn> input, std::size_t feed, std::size_t outRoom, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t room     = std::max(outRoom, 1UZ);
    std::vector<TOut> scratch(room);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed              = std::min(input.size(), fed + arriving);
        const auto first = std::ranges::lower_bound(tags, startOffset + consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(tags, startOffset + fed, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(consumed, fed - consumed), startOffset + consumed, std::span<const gr::Tag>(first, last), false);
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags, false);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, fed - consumed); }) {
            block.forwardTags(inputs, outputs, fed - consumed);
        }
        std::ignore = block.processBulk(inSpan, outSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

} // namespace gr::blocks::timing::test

#endif // GNURADIO_TIMING_TEST_SPANS_HPP
