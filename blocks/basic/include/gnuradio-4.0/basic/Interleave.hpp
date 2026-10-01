#ifndef GNURADIO_BASIC_INTERLEAVE_HPP
#define GNURADIO_BASIC_INTERLEAVE_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <format>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>

namespace gr::blocks::basic {

namespace interleave_detail {

/// The tag key of the channel count, in its short and its `gr:`-prefixed spelling.
[[nodiscard]] inline bool isChannelCountKey(std::string_view key) noexcept { return key == gr::tag::NUM_CHANNELS.shortKey() || key == std::string_view("gr:num_channels"); }

/// The channel count a tag value states, for any integer type a tag may carry.
[[nodiscard]] inline std::optional<std::size_t> channelCount(const pmt::Value& value) noexcept {
    if (const auto* v = value.get_if<std::uint32_t>(); v != nullptr) {
        return static_cast<std::size_t>(*v);
    }
    if (const auto* v = value.get_if<std::uint64_t>(); v != nullptr) {
        return static_cast<std::size_t>(*v);
    }
    if (const auto* v = value.get_if<std::int32_t>(); v != nullptr && *v >= 0) {
        return static_cast<std::size_t>(*v);
    }
    if (const auto* v = value.get_if<std::int64_t>(); v != nullptr && *v >= 0) {
        return static_cast<std::size_t>(*v);
    }
    return std::nullopt;
}

/// The tag's map without the channel count.
[[nodiscard]] inline property_map withoutChannelCount(const property_map& map) {
    property_map kept;
    for (const auto& [key, value] : map) {
        if (!isChannelCountKey(std::string_view(key))) {
            kept.insert_or_assign(key, value);
        }
    }
    return kept;
}

} // namespace interleave_detail

GR_REGISTER_BLOCK(gr::blocks::basic::Interleave, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::complex<double> ])

template<typename T>
struct Interleave : Block<Interleave<T>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(@brief Interleaves N input streams into one stream of frames of N items.

Output item `j*N + k` is item `j` of input `k`. N is `num_channels`, the number of inputs `in#0` to
`in#N-1`. A tag at index `j` of any input lands at output index `j*N`. Tags of several inputs at the same
index merge into one tag, in input order, and a later input's value wins for a shared key. The block
writes a `num_channels` tag of N at output index 0, merged into the tag there, so a file sink records the
channel count. A `num_channels` key arriving on an input is dropped. A `sample_rate` tag passes unchanged,
as the rate of one channel.
)"">;

    std::vector<PortIn<T>> in = std::vector<PortIn<T>>(2UZ);
    PortOut<T>             out;

    Annotated<gr::Size_t, "num_channels", Visible, Doc<"number of inputs and items per frame">, Limits<1U, 1024U>> num_channels = 2U;

    GR_MAKE_REFLECTABLE(Interleave, in, out, num_channels);

    bool _running      = false;
    bool _countWritten = false;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!newSettings.contains("num_channels")) {
            return;
        }
        if (_running) { // the inputs are connected and keep their count
            num_channels = static_cast<gr::Size_t>(in.size());
            return;
        }
        in.resize(static_cast<std::size_t>(num_channels.value));
        this->output_chunk_size = num_channels.value;
    }

    void start() {
        _running                = true;
        _countWritten           = false;
        this->input_chunk_size  = 1U;
        this->output_chunk_size = static_cast<gr::Size_t>(in.size());
    }

    void stop() { _running = false; }

    template<InputSpanLike TInSpan>
    [[nodiscard]] work::Status processBulk(std::span<TInSpan>& inputs, OutputSpanLike auto& outSpan) {
        const std::size_t nChannels = inputs.size();
        std::size_t       nFrames   = outSpan.size() / nChannels;
        for (const auto& input : inputs) {
            nFrames = std::min(nFrames, input.size());
        }

        for (std::size_t k = 0UZ; k < nChannels; ++k) {
            const auto& input = inputs[k];
            for (std::size_t j = 0UZ; j < nFrames; ++j) {
                outSpan[j * nChannels + k] = input[j];
            }
        }

        std::map<std::size_t, property_map> merged;
        for (auto& input : inputs) {
            for (const auto& [relIndex, tagMap] : input.tags()) {
                if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= nFrames) {
                    continue;
                }
                property_map& frameTag = merged[static_cast<std::size_t>(relIndex)];
                for (auto& [key, value] : interleave_detail::withoutChannelCount(tagMap.get())) {
                    frameTag.insert_or_assign(key, std::move(value));
                }
            }
        }
        if (!_countWritten && nFrames > 0UZ) {
            merged[0UZ].insert_or_assign(std::pmr::string(gr::tag::NUM_CHANNELS.shortKey()), pmt::Value(static_cast<gr::Size_t>(nChannels)));
            _countWritten = true;
        }
        for (auto& [frame, map] : merged) {
            if (!map.empty()) {
                outSpan.publishTag(std::move(map), frame * nChannels);
            }
        }

        for (auto& input : inputs) {
            std::ignore = input.consume(nFrames);
        }
        outSpan.publish(nFrames * nChannels);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::Deinterleave, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, std::complex<double> ])

template<typename T>
struct Deinterleave : Block<Deinterleave<T>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<R""(@brief Splits one stream of frames of N interleaved items into N output streams.

Item `j*N + k` of the input goes to output `k` at index `j`. The outputs are `out#0` to `out#N-1`, and N
is `num_channels`. The first `num_channels` tag on the stream overrides the setting. The tag must state
the number of outputs: a different count ends the run with an error that names the tag's count and the
number of outputs. A later `num_channels` tag with a count other than the first ends the run with an
error that names both counts. A tag at input index `i` reaches every output at index `i / N`, without
its `num_channels` key. A `sample_rate` tag passes unchanged, as the rate of one channel.
)"">;

    PortIn<T>               in;
    std::vector<PortOut<T>> out = std::vector<PortOut<T>>(2UZ);

    Annotated<gr::Size_t, "num_channels", Visible, Doc<"number of outputs and items per frame">, Limits<1U, 1024U>> num_channels = 2U;

    GR_MAKE_REFLECTABLE(Deinterleave, in, out, num_channels);

    bool                       _running = false;
    std::optional<std::size_t> _streamCount{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!newSettings.contains("num_channels") || _running) { // a running block keeps its connected outputs
            return;
        }
        out.resize(static_cast<std::size_t>(num_channels.value));
        this->input_chunk_size = num_channels.value;
    }

    void start() {
        _running = true;
        _streamCount.reset();
        this->input_chunk_size  = static_cast<gr::Size_t>(out.size());
        this->output_chunk_size = 1U;
    }

    void stop() { _running = false; }

    template<OutputSpanLike TOutSpan>
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, std::span<TOutSpan>& outputs) {
        const std::size_t nChannels = outputs.size();
        if (const auto refusal = checkChannelCount(inSpan, nChannels); refusal.has_value()) {
            this->emitErrorMessage("processBulk", *refusal);
            return work::Status::ERROR;
        }

        std::size_t nFrames = inSpan.size() / nChannels;
        for (const auto& output : outputs) {
            nFrames = std::min(nFrames, output.size());
        }
        for (std::size_t k = 0UZ; k < nChannels; ++k) {
            auto& output = outputs[k];
            for (std::size_t j = 0UZ; j < nFrames; ++j) {
                output[j] = inSpan[j * nChannels + k];
            }
        }

        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= nFrames * nChannels) {
                continue;
            }
            const property_map forwarded = interleave_detail::withoutChannelCount(tagMap.get());
            if (forwarded.empty()) {
                continue;
            }
            for (auto& output : outputs) {
                output.publishTag(forwarded, static_cast<std::size_t>(relIndex) / nChannels);
            }
        }

        std::ignore = inSpan.consume(nFrames * nChannels);
        for (auto& output : outputs) {
            output.publish(nFrames);
        }
        return work::Status::OK;
    }

    /// The error a `num_channels` tag in the span raises, if any. The first such tag sets the stream's count.
    [[nodiscard]] std::optional<std::string> checkChannelCount(InputSpanLike auto& inSpan, std::size_t nOutputs) {
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex < 0) {
                continue;
            }
            for (const auto& [key, value] : tagMap.get()) {
                if (!interleave_detail::isChannelCountKey(std::string_view(key))) {
                    continue;
                }
                const std::optional<std::size_t> count = interleave_detail::channelCount(value);
                if (!count.has_value()) {
                    return std::format("a num_channels tag at index {} holds no integer count", inSpan.streamIndex + static_cast<std::size_t>(relIndex));
                }
                if (!_streamCount.has_value()) {
                    if (*count != nOutputs) {
                        return std::format("the stream carries {} channels and the block has {} outputs", *count, nOutputs);
                    }
                    _streamCount = *count;
                    num_channels = static_cast<gr::Size_t>(*count);
                } else if (*count != *_streamCount) {
                    return std::format("a num_channels tag of {} follows the stream's count of {}", *count, *_streamCount);
                }
            }
        }
        return std::nullopt;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_INTERLEAVE_HPP
