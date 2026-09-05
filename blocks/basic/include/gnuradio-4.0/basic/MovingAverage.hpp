#ifndef GNURADIO_MOVING_AVERAGE_HPP
#define GNURADIO_MOVING_AVERAGE_HPP

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

/// @brief Integer sample types accumulate in a wider type, so a long window cannot overflow.
template<typename T>
struct WindowAccumulator {
    using type = T;
};
template<>
struct WindowAccumulator<std::int16_t> {
    using type = std::int64_t;
};
template<>
struct WindowAccumulator<std::int32_t> {
    using type = std::int64_t;
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::MovingAverage, [T], [ float, std::complex<float>, std::int16_t, std::int32_t ])

template<typename T>
struct MovingAverage : Block<MovingAverage<T>> {
    using TAccumulator = typename detail::WindowAccumulator<T>::type;
    using Description  = Doc<R""(
@brief The sum of the last `length` samples, scaled - a running sum, so its cost does not depend on the window.

The accumulator is reseeded by exact direct summation whenever the absolute output offset is a multiple of
`reseed_interval`, so the output does not depend on how the scheduler split the stream. The window is causal and
uncompensated: the first `length - 1` outputs are partial-window sums over a zeroed history.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "length", Doc<"window length N in samples">>                                             length          = 4U;
    Annotated<T, "scale", Doc<"multiplies the window sum; 1/length makes it a mean">>                              scale           = T(1);
    Annotated<gr::Size_t, "reseed_interval", Doc<"reseed at absolute output offsets that are a multiple of this">> reseed_interval = 4096U;
    Annotated<gr::Size_t, "vlen", Doc<"element count of a vector sample; one average per element">>                vlen            = 1U;

    GR_MAKE_REFLECTABLE(MovingAverage, in, out, length, scale, reseed_interval, vlen);

    std::vector<T>            _history{};
    std::vector<TAccumulator> _sum{};
    std::size_t               _capacity = 0UZ;
    std::size_t               _cursor   = 0UZ;
    std::uint64_t             _offset   = 0ULL;
    bool                      _reseed   = true;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (length < 1U || reseed_interval < 1U || vlen < 1U) {
            throw gr::exception(std::format("length ({}), reseed_interval ({}) and vlen ({}) must all be at least one", length.value, reseed_interval.value, vlen.value));
        }
        grow(static_cast<std::size_t>(length.value) + 1UZ);
        _sum.assign(vlen, TAccumulator{});
        _reseed = true; // a settings change takes effect at the next output sample, and reseeds it
    }

    void reset() {
        std::ranges::fill(_history, T{});
        std::ranges::fill(_sum, TAccumulator{});
        _cursor = 0UZ;
        _offset = 0ULL;
        _reseed = true;
    }

    /// @brief The running sum of element @p element, in the accumulator's own width; inspected by the tests.
    [[nodiscard]] TAccumulator windowSum(std::size_t element) const noexcept { return _sum[element]; }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        const std::size_t width    = vlen;
        const std::size_t nVectors = std::min(input.size(), output.size()) / width;
        const std::size_t window   = length;
        const std::size_t interval = reseed_interval;

        for (std::size_t v = 0UZ; v < nVectors; ++v) {
            const bool reseed = _reseed || _offset % interval == 0ULL;
            _reseed           = false;
            step(input.subspan(v * width, width), output.subspan(v * width, width), window, reseed);
        }
        return work::Status::OK;
    }

private:
    void grow(std::size_t capacity) {
        if (capacity <= _capacity) {
            return;
        }
        std::vector<T> grown(capacity * static_cast<std::size_t>(vlen.value), T{});
        for (std::size_t k = 0UZ; k < _capacity; ++k) { // keep the real past where there is any, oldest first
            const std::size_t from = (_cursor + k) % _capacity;
            std::copy_n(_history.begin() + static_cast<std::ptrdiff_t>(from * vlen), vlen.value, grown.begin() + static_cast<std::ptrdiff_t>((capacity - _capacity + k) * vlen));
        }
        _history  = std::move(grown);
        _cursor   = 0UZ;
        _capacity = capacity;
    }

    void step(std::span<const T> sample, std::span<T> result, std::size_t window, bool reseed) {
        const std::size_t width = sample.size();
        std::copy_n(sample.begin(), width, _history.begin() + static_cast<std::ptrdiff_t>(_cursor * width));
        _cursor = _cursor + 1UZ == _capacity ? 0UZ : _cursor + 1UZ;

        if (reseed) {
            std::ranges::fill(_sum, TAccumulator{});
            for (std::size_t k = _capacity - window; k < _capacity; ++k) { // ascending, oldest first
                const std::size_t at = (_cursor + k) % _capacity;
                for (std::size_t e = 0UZ; e < width; ++e) {
                    _sum[e] += widen(_history[at * width + e]);
                }
            }
        } else {
            const std::size_t dropped = (_cursor + _capacity - 1UZ - window) % _capacity;
            for (std::size_t e = 0UZ; e < width; ++e) {
                _sum[e] += widen(sample[e]) - widen(_history[dropped * width + e]);
            }
        }

        for (std::size_t e = 0UZ; e < width; ++e) {
            result[e] = scaled(_sum[e]);
        }
        ++_offset;
    }

    [[nodiscard]] static constexpr TAccumulator widen(T value) noexcept {
        if constexpr (std::same_as<T, TAccumulator>) {
            return value;
        } else {
            return static_cast<TAccumulator>(value);
        }
    }

    [[nodiscard]] constexpr T scaled(TAccumulator sum) const noexcept {
        if constexpr (std::same_as<T, TAccumulator>) {
            return scale.value * sum;
        } else {
            return static_cast<T>(static_cast<TAccumulator>(scale.value) * sum);
        }
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_MOVING_AVERAGE_HPP
