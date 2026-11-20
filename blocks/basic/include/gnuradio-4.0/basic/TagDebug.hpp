#ifndef GNURADIO_BASIC_TAG_DEBUG_HPP
#define GNURADIO_BASIC_TAG_DEBUG_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <format>
#include <iostream>
#include <string>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/formatter/ValueFormatter.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::TagDebug, [T], [ std::uint8_t, std::int16_t, std::int32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct TagDebug : Block<TagDebug<T>> {
    using Description = Doc<R""(
@brief Passes samples and tags through unchanged and prints one line per tag to standard output.

Each line holds the block's `name`, the tag's sample index counted from the start of the stream, and the tag's map.
With `key_filter` set, the block prints and counts only the tags that hold that key. With `display` false, the block
counts the matching tags and prints nothing. `n_tags` holds the count, and a get reads it.

The framework forwards a tag at the index where the stream ends without a work call. The block prints and counts only
the tags that arrive with samples.

The block keeps no record of samples or tags. Each tag leaves with the keys and values it arrived with.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<std::string, Doc<"printed tag key (empty = all)">, Visible> key_filter{};
        Annotated<bool, Doc<"tag printing">, Visible>                         display = true;

        GR_MAKE_REFLECTABLE(Settings, key_filter, display);
    } settings;

    struct Status {
        Annotated<gr::Size_t, Doc<"matching tag count">> n_tags = 0U;

        GR_MAKE_REFLECTABLE(Status, n_tags);
    } status;

    GR_MAKE_REFLECTABLE(TagDebug, in, out, settings, status);

    /**
     * @brief Copies the samples and prints the tags that reach the output with them.
     *
     * The index of each tag in the input span is its position in the stream. The framework forwards every tag to the
     * output at the offset it arrived at. The block reads the tags and copies none.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input, OutputSpanLike auto& output) {
        const std::size_t nSamples = std::min(input.size(), output.size());
        std::copy_n(input.begin(), nSamples, output.begin());

        gr::Size_t nMatched = 0U;
        for (const Tag& tag : input.rawTags) {
            if (!settings.key_filter.value.empty() && !tag.map.contains(std::string_view(settings.key_filter.value))) {
                continue;
            }
            ++nMatched;
            if (settings.display.value) {
                const std::string line = std::format("{} tag at {}: {}\n", this->name, tag.index, tag.map);
                std::cout.write(line.data(), static_cast<std::streamsize>(line.size())); // one write keeps the line whole
            }
        }

        if (nMatched > 0U) {
            status.n_tags.value += nMatched;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_TAG_DEBUG_HPP
