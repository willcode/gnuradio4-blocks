#ifndef GNURADIO_SYNC_TEST_SPANS_HPP
#define GNURADIO_SYNC_TEST_SPANS_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

// Minimal InputSpanLike/OutputSpanLike pair, so a processBulk with optional output ports can be driven at an exact
// chunk size, from an exact absolute stream position, and with any subset of those ports left unwired, without
// standing up a graph and a scheduler. An unconnected port is presented the way the framework presents one: a
// zero-length span whose isConnected is false.
namespace gr::blocks::sync::test {

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

    OutputSpan(std::span<T> samples, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr, bool connected = true) : std::span<T>(samples), sink(published), streamIndex(at), isConnected(connected) {}

    constexpr void publish(std::size_t nSamples) noexcept { count = nSamples; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (!isConnected || sink == nullptr) {
            return;
        }
        sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
    }
};

/// @brief One run's output: the main stream, up to three `float` side streams, and the tags each port published.
template<typename TOut, std::size_t NAux>
struct Capture {
    std::vector<TOut>                      samples{};
    std::array<std::vector<float>, NAux>   aux{};
    std::vector<gr::Tag>                   tags{};
    std::array<std::vector<gr::Tag>, NAux> auxTags{};
    std::size_t                            consumed = 0UZ;

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

/// @brief Drive @p block over @p input in chunks of @p chunkSize, with @p connected selecting which side ports exist.
template<std::size_t NAux, typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut, NAux> run(TBlock& block, std::span<const TIn> input, std::size_t chunkSize, std::array<bool, NAux> connected, std::span<const gr::Tag> tags = {}) {
    Capture<TOut, NAux>                  result;
    const std::size_t                    stride = chunkSize == 0UZ ? std::max(input.size(), 1UZ) : chunkSize;
    std::vector<TOut>                    scratch(stride);
    std::array<std::vector<float>, NAux> auxScratch;
    for (std::vector<float>& buffer : auxScratch) {
        buffer.resize(stride);
    }

    for (std::size_t base = 0UZ; base < input.size();) {
        const std::size_t count = std::min(stride, input.size() - base);
        const auto        first = std::ranges::lower_bound(tags, base, std::ranges::less{}, &gr::Tag::index);
        const auto        last  = std::ranges::lower_bound(tags, base + count, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(base, count), base, std::span<const gr::Tag>(first, last));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), count), result.samples.size(), &result.tags);

        std::array<OutputSpan<float>, NAux> auxSpans = [&]<std::size_t... I>(std::index_sequence<I...>) { return std::array<OutputSpan<float>, NAux>{OutputSpan<float>(connected[I] ? std::span<float>(auxScratch[I].data(), count) : std::span<float>{}, result.aux[I].size(), &result.auxTags[I], connected[I])...}; }(std::make_index_sequence<NAux>{});

        if constexpr (NAux == 2UZ) {
            std::ignore = block.processBulk(inSpan, outSpan, auxSpans[0], auxSpans[1]);
        } else {
            static_assert(NAux == 3UZ, "the sync blocks carry two or three optional side ports");
            std::ignore = block.processBulk(inSpan, outSpan, auxSpans[0], auxSpans[1], auxSpans[2]);
        }

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        for (std::size_t which = 0UZ; which < NAux; ++which) {
            result.aux[which].insert(result.aux[which].end(), auxScratch[which].begin(), auxScratch[which].begin() + static_cast<std::ptrdiff_t>(auxSpans[which].count));
        }
        result.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ) {
            break;
        }
        base += inSpan.consumed;
    }
    return result;
}

/**
 * @brief Drive a variable-rate block: @p feed samples arrive per call and everything not yet consumed is presented.
 *
 * `run` above hands over a fixed window and moves past it, which is the right model for a block that consumes what it
 * is given. A block that needs an interpolation window consumes nothing until it has one, and a scheduler answers that
 * by growing the window rather than by repeating a call it cannot serve — so here a chunk size of one is one sample
 * arriving per call, not a one-sample window. @p outRoom caps the outputs a call may produce, which is the other half
 * of a variable-rate block's chunking, and @p startOffset puts the whole stream at an absolute input position.
 */
template<std::size_t NAux, typename TOut, typename TBlock, typename TIn>
[[nodiscard]] Capture<TOut, NAux> runVariable(TBlock& block, std::span<const TIn> input, std::size_t feed, std::size_t outRoom, std::array<bool, NAux> connected, std::span<const gr::Tag> tags = {}, std::size_t startOffset = 0UZ) {
    Capture<TOut, NAux> result;
    const std::size_t   arriving = feed == 0UZ ? std::max(input.size(), 1UZ) : feed;
    const std::size_t   room     = outRoom == 0UZ ? std::max(input.size(), 1UZ) : outRoom;

    std::vector<TOut>                    scratch(room);
    std::array<std::vector<float>, NAux> auxScratch;
    for (std::vector<float>& buffer : auxScratch) {
        buffer.resize(room);
    }

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed              = std::min(input.size(), fed + arriving);
        const auto first = std::ranges::lower_bound(tags, startOffset + consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(tags, startOffset + fed, std::ranges::less{}, &gr::Tag::index);

        InputSpan<TIn>   inSpan(input.subspan(consumed, fed - consumed), startOffset + consumed, std::span<const gr::Tag>(first, last));
        OutputSpan<TOut> outSpan(std::span<TOut>(scratch.data(), room), result.samples.size(), &result.tags);

        std::array<OutputSpan<float>, NAux> auxSpans = [&]<std::size_t... I>(std::index_sequence<I...>) { return std::array<OutputSpan<float>, NAux>{OutputSpan<float>(connected[I] ? std::span<float>(auxScratch[I].data(), room) : std::span<float>{}, result.aux[I].size(), &result.auxTags[I], connected[I])...}; }(std::make_index_sequence<NAux>{});

        static_assert(NAux == 3UZ, "the variable-rate shim is written for the three optional ports SymbolSync carries");
        std::ignore = block.processBulk(inSpan, outSpan, auxSpans[0], auxSpans[1], auxSpans[2]);

        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        for (std::size_t which = 0UZ; which < NAux; ++which) {
            result.aux[which].insert(result.aux[which].end(), auxScratch[which].begin(), auxScratch[which].begin() + static_cast<std::ptrdiff_t>(auxSpans[which].count));
        }
        result.consumed += inSpan.consumed;
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.size()) {
            break;
        }
    }
    return result;
}

} // namespace gr::blocks::sync::test

#endif // GNURADIO_SYNC_TEST_SPANS_HPP
