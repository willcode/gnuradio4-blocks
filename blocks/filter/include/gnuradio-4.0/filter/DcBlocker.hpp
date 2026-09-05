#ifndef GNURADIO_DC_BLOCKER_HPP
#define GNURADIO_DC_BLOCKER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/filter/NamespaceCompatibility.hpp>

namespace gr::blocks::filter {

namespace detail {

template<typename T>
struct RealOf {
    using type = T;
};
template<typename TReal>
struct RealOf<std::complex<TReal>> {
    using type = TReal;
};

/// @brief A length-D moving average by running sum, reseeded on a fixed absolute-offset schedule.
template<typename T>
struct Boxcar {
    std::vector<T> _history{};
    std::size_t    _cursor = 0UZ;
    T              _sum{};

    void configure(std::size_t length) {
        _history.assign(length, T{});
        _cursor = 0UZ;
        _sum    = T{};
    }

    [[nodiscard]] T step(T sample, bool reseed) noexcept {
        const T dropped   = _history[_cursor];
        _history[_cursor] = sample;
        _cursor           = _cursor + 1UZ == _history.size() ? 0UZ : _cursor + 1UZ;

        if (reseed) {
            T sum{};
            for (std::size_t k = 0UZ; k < _history.size(); ++k) { // ascending, oldest first
                sum += _history[(_cursor + k) % _history.size()];
            }
            _sum = sum;
        } else {
            _sum += sample - dropped;
        }
        return _sum / static_cast<typename RealOf<T>::type>(_history.size());
    }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::filter::DcBlocker, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct DcBlocker : Block<DcBlocker<T>, UnfilteredTagPropagation> {
    using Description = Doc<R""(
@brief Removes DC with a narrower notch and less delay than a single-pole highpass of the same cost.

DC is estimated with a cascade of length-`length` boxcar averages and subtracted from a copy of the input delayed to
match the cascade's group delay, which is an exact integer - `D-1` for two boxcars, `2D-2` for four - so the
subtraction is aligned to the sample and the response is exactly `|1 - A(f)^stages|` with
`A(f) = sin(pi*D*f/fs) / (D*sin(pi*f/fs))`, which is zero at DC and returns to 1 at every multiple of `fs/D`.

The long form is the default and is both the flatter and the narrower of the two: at `D = 32` its -3 dB corner sits at
`0.419 * fs/D` against the short form's `0.573 * fs/D`, for twice the delay and twice the boxcars. `length` and
`long_form` are construction-time, since changing either would invalidate the pipeline state and the group delay at
the same moment. `length` must be at least two and `reseed_interval` at least one, or the settings change throws.

Each boxcar's running sum is recomputed exactly whenever the absolute sample offset is a multiple of
`reseed_interval`, oldest sample first, and updated incrementally otherwise, which is what stops a long float run
drifting without bound.

The block is 1:1, so every input tag key passes through at its own offset; a tag is not shifted by the group delay,
which the samples carry and the annotation does not.

The two forms, their group delays and the cascade construction are from R. Yates, "DC Blocker Algorithms", IEEE Signal
Processing Magazine, Mar. 2008, pp. 132-134.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "length", Doc<"boxcar length D; longer narrows the notch. Construction-time">>          length          = 32U;
    Annotated<bool, "long_form", Doc<"true: four boxcars and 2D-2 delay; false: two and D-1. Construction-time">> long_form       = true;
    Annotated<gr::Size_t, "reseed_interval", Doc<"absolute sample offsets that are a multiple reseed the sums">>  reseed_interval = 4096U;
    Annotated<gr::Size_t, "group_delay", Doc<"read-only: D-1 for the short form, 2D-2 for the long one">>         group_delay     = 31U;

    GR_MAKE_REFLECTABLE(DcBlocker, in, out, length, long_form, reseed_interval, group_delay);

    std::vector<detail::Boxcar<T>> _stages{};
    std::vector<T>                 _delay{};
    std::size_t                    _cursor = 0UZ;
    std::uint64_t                  _offset = 0ULL;

    bool       _running = false;
    gr::Size_t _builtLength{};   /// the length the current pipeline was built for
    bool       _builtLongForm{}; /// and the form

    void start() { _running = true; }

    void stop() { _running = false; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        // The documented contract, now enforced. Rebuilding the pipeline mid-stream is not unsafe — `build()`
        // reassigns every buffer — but it discards the history and moves `group_delay`, so the block's latency
        // changes underneath whatever downstream was aligned against it, and there is no tag that says so. The
        // test is against the values in force rather than which keys the transaction carried, because a
        // transaction can restage a value that did not move.
        if (_running && (length.value != _builtLength || long_form.value != _builtLongForm)) {
            // A refused change stays staged and is written again on the next apply, so this block keeps refusing
            // until it is stopped and the value put back. That is the framework's behavior for any throwing
            // settings change, not this block's, and it is why a caller catches this once rather than retrying.
            throw gr::exception(std::format("length and long_form are construction-time: changing either would rebuild the pipeline and move the group delay ({} samples) underneath a running graph, with nothing to announce it — stop the graph to change them", group_delay.value));
        }
        if (length < 2U) {
            throw gr::exception(std::format("length must be at least two, got {}", length.value));
        }
        if (reseed_interval < 1U) {
            throw gr::exception(std::format("reseed_interval must be at least one, got {}", reseed_interval.value));
        }
        group_delay = static_cast<gr::Size_t>(stageCount() / 2UZ * (static_cast<std::size_t>(length.value) - 1UZ));
        if (!_stages.empty() && !newSettings.contains("length") && !newSettings.contains("long_form")) {
            return; // only reseed_interval moved, and that does not touch the pipeline
        }
        build();
    }

    void reset() { build(); }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        const std::size_t interval = reseed_interval;

        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            const bool reseed = _offset % interval == 0ULL;
            T          value  = input[i];
            for (detail::Boxcar<T>& stage : _stages) {
                value = stage.step(value, reseed);
            }
            const T aligned = _delay[_cursor];
            _delay[_cursor] = input[i];
            _cursor         = _cursor + 1UZ == _delay.size() ? 0UZ : _cursor + 1UZ;
            output[i]       = aligned - value;
            ++_offset;
        }
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::size_t stageCount() const noexcept { return long_form ? 4UZ : 2UZ; }

    void build() {
        _stages.assign(stageCount(), detail::Boxcar<T>{});
        for (detail::Boxcar<T>& stage : _stages) {
            stage.configure(length);
        }
        _delay.assign(group_delay, T{}); // exactly group_delay slots, so _delay[_cursor] is the sample due now
        _cursor        = 0UZ;
        _offset        = 0ULL;
        _builtLength   = length;
        _builtLongForm = long_form;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_DC_BLOCKER_HPP
