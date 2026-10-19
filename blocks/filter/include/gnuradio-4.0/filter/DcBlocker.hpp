#ifndef GNURADIO_DC_BLOCKER_HPP
#define GNURADIO_DC_BLOCKER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/filter/TagDelay.hpp>

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

/// A length-D moving average by running sum. A step with `reseed` set recomputes the sum exactly from the history.
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

/**
 * `length` takes values from two up and `reseed_interval` from one up. The framework refuses any other value before the
 * block sees it. It refuses that key alone and applies the rest of the application. The member and `settings().get()`
 * keep the accepted value. A refused value from a tag is reported in a plain message, and the run goes on. A refused
 * staged value or Set message is reported in an error message, which ends a run whose messages nothing reads.
 * `settings().set()` throws at the call and stores nothing.
 *
 * The block computes on the `length`, `long_form` and `reseed_interval` it last accepted, starting at the defaults, and
 * keeps them apart from its members. It refuses a change of `length` or `long_form` while it runs. A reset to defaults
 * while it runs is such a change when it moves either one back from a value accepted while the block was stopped. This
 * refusal covers the whole application and changes nothing the block computes. The refused values stay in their
 * members, and `settings().get()` returns them. The block ignores each one until an application changes that setting
 * again. The framework drops a staged or tagged value equal to its member before the block sees it. A refused value
 * sent again alone therefore passes with no refusal and changes nothing.
 *
 * `group_delay` follows from the accepted `length` and `long_form`. The block ignores a `group_delay` written in the
 * same application as `length` or `long_form`. It compares a `group_delay` written without either one with the delay
 * of the running pipeline. It accepts an equal value and refuses the application on any other, as it refuses a change
 * while it runs. It writes `group_delay` again at each application it accepts, from the pipeline that application
 * leaves. Between a refused write and the next accepted application, `settings().get()` returns the written value.
 *
 * `reset()` rebuilds the accepted pipeline with an empty history, as at the start of a stream. The framework calls it
 * at the lifecycle's reset and at a reset to defaults the block accepts, after the defaults are applied. A refused
 * reset to defaults skips it, and the block keeps its history. After a reset the next `group_delay` outputs come from
 * the empty delay line. Every tag still leaves `group_delay` outputs after its input.
 */
template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct DcBlocker : Block<DcBlocker<T>>, detail::DelayedTagFilter<DcBlocker<T>, T, T> {
    using Description = Doc<R""(
@brief Removes DC with a cascade of boxcar averages, a linear-phase notch with an exact integer delay.

A cascade of length-`length` boxcar averages estimates DC. The block subtracts it from a copy of the input delayed to
match the cascade's group delay. That delay is an exact integer, `D-1` for two boxcars and `2D-2` for four. The
subtraction is therefore aligned to the sample, and the phase is linear. The response is exactly `|1 - A(f)^stages|`
with `A(f) = sin(pi*D*f/fs) / (D*sin(pi*f/fs))`. It is zero at DC and returns to 1 at every multiple of `fs/D`.

The long form is the default. It is both the flatter and the narrower of the two. At `D = 32` its -3 dB corner sits at
`0.419 * fs/D`, against `0.573 * fs/D` for the short form. It costs twice the delay and twice the boxcars. A longer
`length` narrows the notch. `length` and `long_form` are fixed while the block runs. A change rebuilds the pipeline,
discards its history and moves the delay. `length` must be at least two and `reseed_interval` at least one.

Each boxcar's running sum is recomputed exactly, oldest sample first, whenever the count of samples since the pipeline
was built is a multiple of `reseed_interval`. Otherwise the sum is updated incrementally. The recomputation stops a
long float run from drifting without bound. A NaN or infinite input sample makes the output non-finite until every
boxcar has recomputed its sum after the value left its history. The output is finite again fewer than
`stages * (reseed_interval + length)` samples after that input.

Every tag moves with all its keys by the group delay. A tag on input `i` leaves on output `i + group_delay`. A tag
whose output lies past the stream's last output leaves at the end-of-stream index, one past that output.

The two forms, their group delays and the cascade construction are from R. Yates, "DC Blocker Algorithms", IEEE Signal
Processing Magazine, Mar. 2008, pp. 132-134.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "length", Doc<"boxcar length D">, Limits<2U, std::numeric_limits<gr::Size_t>::max()>>                                           length          = 32U;
    Annotated<bool, "long_form", Doc<"true for four boxcars and 2D-2 delay, false for two and D-1">>                                                      long_form       = true;
    Annotated<gr::Size_t, "reseed_interval", Doc<"samples between exact recomputations of the sums">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>> reseed_interval = 4096U;
    Annotated<gr::Size_t, "group_delay", Doc<"derived pipeline delay in samples, D-1 or 2D-2">>                                                           group_delay     = 62U;

    GR_MAKE_REFLECTABLE(DcBlocker, in, out, length, long_form, reseed_interval, group_delay);

    std::vector<detail::Boxcar<T>> _stages{};
    std::vector<T>                 _delay{};
    std::size_t                    _cursor = 0UZ;
    std::uint64_t                  _offset = 0ULL;

    bool       _running       = false;
    gr::Size_t _builtLength   = length.value;          ///< the accepted `length`, which the pipeline is built for
    bool       _builtLongForm = long_form.value;       ///< the accepted `long_form`, which the pipeline is built for
    gr::Size_t _interval      = reseed_interval.value; ///< the accepted `reseed_interval`

    /// The framework calls settingsChanged() only for a batch that changes a value. The constructor builds the pipeline
    /// from the defaults. A block constructed at its declared defaults therefore starts with its boxcars and delay line
    /// in place.
    explicit DcBlocker(property_map init = {}) : Block<DcBlocker<T>>(std::move(init)) { build(_builtLength, _builtLongForm); }

    void start() {
        _running = true;
        this->tagsStart();
    }

    void stop() { _running = false; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const bool       shapeWritten = newSettings.contains("length") || newSettings.contains("long_form");
        const gr::Size_t builtDelay   = static_cast<gr::Size_t>(_delay.size());
        if (!shapeWritten && newSettings.contains("group_delay") && group_delay.value != builtDelay) {
            throw gr::exception(std::format("group_delay follows from length and long_form, which give {}", builtDelay));
        }
        const gr::Size_t nextLength   = newSettings.contains("length") ? length.value : _builtLength;
        const bool       nextLongForm = newSettings.contains("long_form") ? long_form.value : _builtLongForm;
        const gr::Size_t nextInterval = newSettings.contains("reseed_interval") ? reseed_interval.value : _interval;
        const bool       rebuild      = nextLength != _builtLength || nextLongForm != _builtLongForm;
        if (rebuild && _running) {
            throw gr::exception(std::format("length and long_form are fixed while the block runs; stop it to change them from {} and {}", _builtLength, _builtLongForm));
        }
        _interval = nextInterval;
        if (rebuild) {
            build(nextLength, nextLongForm);
        }
        group_delay = static_cast<gr::Size_t>(_delay.size());
    }

    void reset() { build(_builtLength, _builtLongForm); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// The delay every tag moves by, in half samples.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept { return 2ULL * static_cast<std::uint64_t>(_delay.size()); }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t nSamples = std::min(input.size(), output.size());
        const std::size_t interval = _interval;

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
    }

private:
    [[nodiscard]] static constexpr std::size_t delayOf(gr::Size_t newLength, bool newLongForm) noexcept { return (newLongForm ? 2UZ : 1UZ) * (static_cast<std::size_t>(newLength) - 1UZ); }

    void build(gr::Size_t newLength, bool newLongForm) {
        _stages.assign(newLongForm ? 4UZ : 2UZ, detail::Boxcar<T>{});
        for (detail::Boxcar<T>& stage : _stages) {
            stage.configure(newLength);
        }
        // The line holds exactly the group delay in slots. `_delay[_cursor]` holds the sample due now.
        _delay.assign(delayOf(newLength, newLongForm), T{});
        _cursor        = 0UZ;
        _offset        = 0ULL;
        _builtLength   = newLength;
        _builtLongForm = newLongForm;
        group_delay    = static_cast<gr::Size_t>(_delay.size());
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_DC_BLOCKER_HPP
