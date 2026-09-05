#ifndef GNURADIO_DIGITAL_TEST_SPANS_HPP
#define GNURADIO_DIGITAL_TEST_SPANS_HPP

#include <algorithm>
#include <cstddef>
#include <functional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

// Minimal ReaderSpanLike/WriterSpanLike pair, so a processBulk — or a forwardTags override — can be driven at an exact
// chunk size and from an exact absolute stream position without standing up a graph and a scheduler. `rawTags` plus
// `streamIndex` is the idiom SchmittTrigger reads; `tags(window)` is the idiom the framework's own forwarding reads,
// and both are offered here because the blocks under test use one or the other.
namespace gr::blocks::digital::test {

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
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    bool          isConnected = true;
    bool          isSync      = true;

    InputSpan(std::span<const T> samples, std::size_t at = 0UZ, std::span<const gr::Tag> incoming = {}) : std::span<const T>(samples), rawTags(incoming), streamIndex(at) {}

    constexpr bool consume(std::size_t nSamples) noexcept {
        consumed = nSamples;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

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

    OutputSpan(std::span<T> samples, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr, bool connected = true) : std::span<T>(samples), sink(published), streamIndex(at), isConnected(connected) {}

    constexpr void publish(std::size_t nSamples) noexcept { count = nSamples; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (!isConnected || sink == nullptr) {
            return;
        }
        sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
    }
};

/// @brief One run of a one-in one-out block: everything it published, and everything it consumed.
template<typename TOut>
struct Capture {
    std::vector<TOut>    samples{};
    std::vector<gr::Tag> tags{};
    std::size_t          consumed = 0UZ;
};

/**
 * @brief Drive a one-in one-out block over @p input in chunks of @p chunkSize, at a rate of @p outPerIn.
 *
 * @p outPerIn is the output items reserved per input item — 1 for a passthrough, `bitsPerSymbol()` for the soft
 * decoder. @p startOffset puts the whole run at an absolute stream position, which is what the tag-offset tests
 * need and what no scheduler-driven test can reach.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> run(TBlock& block, std::span<const TIn> input, std::size_t chunkSize = 0UZ, std::size_t outPerIn = 1UZ, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut> scratch(stride * outPerIn);

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);
        const auto        first = std::ranges::lower_bound(tags, startOffset + base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, startOffset + base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(base, count), startOffset + base, std::span<const gr::Tag>(first, last));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), count * outPerIn), outPerIn * (startOffset + base), &result.tags);

        std::ignore  = block.processBulk(inSpan, outSpan);
        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, count); }) {
            block.forwardTags(inputs, outputs, count);
        }

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(count * outPerIn));
        result.consumed += count;
        base += count;
    }
    return result;
}

/**
 * @brief `run`, with a second output port beside the main one — the correlation-magnitude port of a detector.
 *
 * @p auxTags collects what the second port published, which is what a test asking whether a tag reached every port
 * needs. The call order here is the framework's — `forwardTags` first, then `processBulk` — because a block that
 * publishes on both paths has to keep one port's indices in order across the two.
 */
template<typename TOut, typename TAux, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> run3(TBlock& block, std::span<const TIn> input, std::size_t chunkSize, std::vector<TAux>& aux, bool auxConnected = true, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ, std::vector<gr::Tag>* auxTags = nullptr) {
    Capture<TOut>     result;
    const std::size_t stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut> scratch(stride);
    std::vector<TAux> auxScratch(stride);
    aux.clear();

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);
        const auto        first = std::ranges::lower_bound(tags, startOffset + base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, startOffset + base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(base, count), startOffset + base, std::span<const gr::Tag>(first, last));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), count), startOffset + base, &result.tags);
        OutputSpan<TAux> auxSpan(auxConnected ? std::span<TAux>(auxScratch.data(), count) : std::span<TAux>{}, startOffset + base, auxTags, auxConnected);

        auto inputs  = std::tie(inSpan);
        auto outputs = std::tie(outSpan, auxSpan);
        if constexpr (requires { block.forwardTags(inputs, outputs, count); }) {
            block.forwardTags(inputs, outputs, count);
        }
        std::ignore = block.processBulk(inSpan, outSpan, auxSpan);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        aux.insert(aux.end(), auxScratch.begin(), auxScratch.begin() + static_cast<std::ptrdiff_t>(auxSpan.count));
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/**
 * @brief Drive a variable-rate block: @p feed items arrive per call and everything not yet consumed is presented again.
 *
 * A block that needs a whole header before it can decide consumes nothing until it has one, and a scheduler answers
 * that by growing the window rather than by repeating a call it cannot serve. So a feed of one is one item arriving
 * per call, not a one-item window.
 */
template<typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut> runVariable(TBlock& block, std::span<const TIn> input, std::size_t feed, std::size_t outRoom, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut>     result;
    const std::size_t arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t room     = outRoom == 0UZ ? std::max(input.size(), 1UZ) : outRoom;
    std::vector<TOut> scratch(room);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed              = std::min(input.size(), fed + arriving);
        const auto first = std::ranges::lower_bound(tags, startOffset + consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(tags, startOffset + fed, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(consumed, fed - consumed), startOffset + consumed, std::span<const gr::Tag>(first, last));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags);

        std::ignore = block.processBulk(inSpan, outSpan);

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            result.samples.push_back(std::move(scratch[k]));
        }
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

} // namespace gr::blocks::digital::test

#endif // GNURADIO_DIGITAL_TEST_SPANS_HPP
