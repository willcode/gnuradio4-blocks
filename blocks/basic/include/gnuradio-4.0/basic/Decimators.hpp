#ifndef GNURADIO_DECIMATORS_HPP
#define GNURADIO_DECIMATORS_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <format>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {

namespace detail {

/**
 * @brief Republishes input tags at the output offsets where a decimator placed their samples.
 *
 * Each tag is carried to the next item the block keeps, so the map is the block's own phase by construction
 * rather than an offset scaled by a rate, which would assume a phase aligned to stream offset zero and would lose
 * exactness through a `float`. A tag with no kept item left in the call is held for the next one rather than dropped,
 * and order and multiplicity are preserved.
 */
struct TagRelay {
    std::vector<property_map> _carry{};
    std::size_t               _cursor = 0UZ;
    std::size_t               _count  = 0UZ;

    void reset() {
        _carry.clear();
        _cursor = 0UZ;
        _count  = 0UZ;
    }

    template<typename TInSpan>
    void begin(const TInSpan& inSpan) noexcept {
        _cursor = 0UZ;
        _count  = inSpan.rawTags.size();
    }

    template<typename TInSpan>
    void collect(const TInSpan& inSpan, std::size_t index) {
        const auto&       raw   = inSpan.rawTags;
        const std::size_t until = inSpan.streamIndex + index;
        while (_cursor < _count && raw[_cursor].index <= until) {
            if (raw[_cursor].index >= inSpan.streamIndex) {
                _carry.push_back(raw[_cursor].map);
            }
            ++_cursor;
        }
    }

    template<typename TOutSpan>
    void release(TOutSpan& outSpan, std::size_t at) {
        for (const property_map& forwarded : _carry) {
            outSpan.publishTag(forwarded, at);
        }
        _carry.clear();
    }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::KeepOneInN, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

template<typename T>
struct KeepOneInN : Block<KeepOneInN<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Keeps the last item of every group of `n` and drops the rest.

Display downsampling and slow status streams, where no anti-alias filtering is wanted or needed. After a reset the
first item out is input index `n-1`, then `2n-1`, and so on. Setting `n` restarts the phase: the counter reloads
immediately and the next item out is `n_new` inputs after the change. `n` must be at least one or the settings change throws.

Tag offsets are computed with integer arithmetic against the block's own phase, and a tag on a dropped item moves
forward to the next kept item. The output rate is `1/n` and the block is variable-rate rather than a declared
decimator, so a very large `n` does not force the scheduler into large buffer demands.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "n", Doc<"group size; the last item of each group is kept">> n = 1U;

    GR_MAKE_REFLECTABLE(KeepOneInN, in, out, n);

    gr::Size_t       _counter = 1U;
    detail::TagRelay _relay{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (n < 1U) {
            throw gr::exception(std::format("n must be at least one, got {}", n.value));
        }
        if (newSettings.contains("n") || _counter > n) {
            _counter = n; // a change to n restarts the phase
        }
    }

    void reset() {
        _counter = n;
        _relay.reset();
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made   = 0UZ;
        std::size_t walked = 0UZ;
        _relay.begin(inSpan);

        for (; walked < inSpan.size() && made < outSpan.size(); ++walked) {
            _relay.collect(inSpan, walked);
            if (--_counter > 0U) {
                continue;
            }
            _counter      = n;
            outSpan[made] = inSpan[walked];
            _relay.release(outSpan, made);
            ++made;
        }

        std::ignore = inSpan.consume(walked);
        outSpan.publish(made);
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::KeepMInN, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

template<typename T>
struct KeepMInN : Block<KeepMInN<T>, Resampling<1UZ, 1UZ, false>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Keeps `m` consecutive items out of every `n`, starting `offset` items into each group.

Extracts a fixed slice from a repeating frame. The block consumes in whole groups and produces in whole slices, so a
change to `m`, `n` or `offset` takes effect only on a group boundary. `m` and `n` must be at least one, and
`offset + m > n` is rejected at settings time rather than wrapped; a wrapped read would be a separate block.

Tag offsets are computed with integer arithmetic against the block's own group phase, and a tag on a dropped item
moves forward to the next kept item - which, for an item past the end of a slice, is the first item of the next group.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "m", Doc<"items kept per group">>                           m      = 1U;
    Annotated<gr::Size_t, "n", Doc<"group size">>                                     n      = 1U;
    Annotated<gr::Size_t, "offset", Doc<"index in the group of the first kept item">> offset = 0U;

    GR_MAKE_REFLECTABLE(KeepMInN, in, out, m, n, offset);

    detail::TagRelay _relay{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (m < 1U || n < 1U) {
            throw gr::exception(std::format("m ({}) and n ({}) must both be at least one", m.value, n.value));
        }
        if (offset + m > n) {
            throw gr::exception(std::format("offset ({}) + m ({}) must not exceed n ({}); a wrapped read is a different block", offset.value, m.value, n.value));
        }
        this->input_chunk_size  = n;
        this->output_chunk_size = m;
    }

    void reset() { _relay.reset(); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t group  = n;
        const std::size_t groups = std::min(inSpan.size() / group, outSpan.size() / static_cast<std::size_t>(m.value));
        if (groups == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        std::size_t made = 0UZ;
        _relay.begin(inSpan);
        for (std::size_t i = 0UZ; i < groups * group; ++i) {
            _relay.collect(inSpan, i);
            const std::size_t phase = i % group;
            if (phase < offset || phase >= offset + m) {
                continue;
            }
            outSpan[made] = inSpan[i];
            _relay.release(outSpan, made);
            ++made;
        }

        std::ignore = inSpan.consume(groups * group);
        outSpan.publish(made);
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_DECIMATORS_HPP
