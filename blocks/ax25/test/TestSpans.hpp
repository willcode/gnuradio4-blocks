#ifndef GNURADIO_AX25_TEST_SPANS_HPP
#define GNURADIO_AX25_TEST_SPANS_HPP

#include <cstddef>
#include <functional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Tag.hpp>

// Minimal ReaderSpanLike/WriterSpanLike pair, so a processBulk can be driven at an exact record count and with an exact
// amount of output room without standing up a graph and a scheduler. The record adapters here carry no tags, so the tag
// members are present for the concepts' sake and do nothing.
namespace gr::blocks::ax25::test {

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
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

    explicit InputSpan(std::span<const T> records, std::size_t at = 0UZ) : std::span<const T>(records), streamIndex(at) {}

    constexpr bool consume(std::size_t nRecords) noexcept {
        consumed = nRecords;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::vector<TagView> tags() const { return {}; }
    [[nodiscard]] std::vector<TagView> tags(std::size_t) const { return {}; }
};

template<typename T>
struct OutputSpan : std::span<T> {
    using value_type = T;

    TagWriterSpan tags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   count       = 0UZ;
    bool          isConnected = true;
    bool          isSync      = true;

    explicit OutputSpan(std::span<T> room, bool connected = true) : std::span<T>(room), isConnected(connected) {}

    constexpr void publish(std::size_t nRecords) noexcept { count = nRecords; }
    constexpr void publishTag(const gr::property_map&, std::size_t = 0UZ) const noexcept {}
};

} // namespace gr::blocks::ax25::test

#endif // GNURADIO_AX25_TEST_SPANS_HPP
