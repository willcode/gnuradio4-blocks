#ifndef GNURADIO_PAM_SYMBOLS_HPP
#define GNURADIO_PAM_SYMBOLS_HPP

#include <cmath>
#include <concepts>
#include <cstdint>
#include <format>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::digital {

namespace detail {

/// The uniform M-PAM grid both blocks share: levels at `(2r - (M-1)) * spread / 2` for rank
/// `r = 0 .. M-1` — `±1, ±3, …` at the nominal spacing 2 — with decision thresholds at the
/// midpoints and the two outer regions unbounded.
[[nodiscard]] inline std::size_t pamRank(double value, double spread, std::size_t levels) noexcept {
    const double scaled = value / spread + 0.5 * static_cast<double>(levels - 1UZ);
    if (!(scaled > 0.0)) {
        return 0UZ;
    }
    const auto rank = static_cast<std::size_t>(std::lround(scaled));
    return rank >= levels ? levels - 1UZ : rank;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::digital::LevelTracker, [T], [float])

template<std::floating_point T>
struct LevelTracker : Block<LevelTracker<T>> {
    using Description = Doc<R""(
@brief Decision-directed gain and DC normalization for real M-PAM symbols.

One sample per symbol in, the same sample normalized out so the M levels land on the `±1, ±3, …` grid whatever
spacing and common shift the stream carries. Two loops ride the decision error - the distance from the nearest level
of the tracker's current belief: the OFFSET integrates it (a common shift of every level is exactly the running mean
of the error; for a discriminator-fed chain it is the carrier frequency offset), and the SPREAD - the spacing between
adjacent levels, nominally 2 - grows or shrinks as symbols fall outside or inside their decided places. An outer
level's distance from center carries proportionally more leverage on the spacing, so its update weight is halved
against the innermost pair's - the proven compromise carried from the C4FM tracker this block generalizes, whose
deviation-tracking accuracy (±15 % measured to ±0.05 of truth) is the behavior it must reproduce. The spread is
clamped to a band about nominal so a noise burst cannot walk it somewhere it cannot return from; the outer decision
regions are unbounded, an overshoot being still the symbol it overshot.

Even M only; a grid with a level at zero changes the decision and both loops, and no consumer exists.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "n_levels", Visible, Doc<"M, even and at least 2">>                                 n_levels     = 4U;
    Annotated<double, "spread_rate", Doc<"level-spacing loop gain">>                                          spread_rate  = 0.0100;
    Annotated<double, "offset_rate", Doc<"common-shift loop gain">>                                           offset_rate  = 0.1250;
    Annotated<double, "spread_limit", Doc<"fractional band the spacing may move in, about its nominal 2">>    spread_limit = 0.20;
    Annotated<double, "spread", Doc<"observable: the tracked spacing between adjacent levels; 2 at nominal">> spread       = 2.0;
    Annotated<double, "offset", Doc<"observable: the tracked common shift of every level, in input units">>   offset       = 0.0;

    GR_MAKE_REFLECTABLE(LevelTracker, in, out, n_levels, spread_rate, offset_rate, spread_limit, spread, offset);

    double _spread = 2.0;
    double _offset = 0.0;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (n_levels < 2U || (n_levels % 2U) != 0U) {
            throw gr::exception(std::format("n_levels must be even and at least 2, got {}", n_levels.value));
        }
        if (newSettings.contains("n_levels")) { // a new grid invalidates every tracked quantity
            _spread = 2.0;
            _offset = 0.0;
        }
    }

    void start() {
        _spread = 2.0;
        _offset = 0.0;
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) noexcept {
        const auto        levels   = static_cast<std::size_t>(n_levels.value);
        const double      spreadLo = 2.0 * (1.0 - spread_limit);
        const double      spreadHi = 2.0 * (1.0 + spread_limit);
        const std::size_t innerLo  = levels / 2UZ - 1UZ;

        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            const double value = static_cast<double>(input[i]) - _offset;
            output[i]          = static_cast<T>(2.0 * value / _spread);

            const std::size_t rank  = detail::pamRank(value, _spread, levels);
            const double      level = (2.0 * static_cast<double>(rank) - static_cast<double>(levels - 1UZ)) * 0.5 * _spread;
            const double      error = value - level;

            const double weight = (rank == innerLo || rank == innerLo + 1UZ) ? 1.0 : 0.5;
            _spread += (level > 0.0 ? error : -error) * weight * spread_rate;
            _spread = std::clamp(_spread, spreadLo, spreadHi);
            _offset += error * offset_rate;
        }
        spread = _spread; // observables refresh once per call, never per sample
        offset = _offset;
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::PamSlicer, [T], [float])

template<std::floating_point T>
struct PamSlicer : Block<PamSlicer<T>> {
    using Description = Doc<R""(
@brief The M-PAM hard decision: one symbol on the `±1, ±3, …` grid in, one label out.

Thresholds sit at the midpoints of the adjacent-level gaps and the two outer regions are unbounded: a symbol that
overshoots is still the symbol it overshot, and bounding the top region would turn a one-level overshoot into the
most distant wrong answer available. The label table maps level rank, lowest to highest, to the emitted value,
because real protocols' maps are non-monotonic - C4FM's is {3, 2, 0, 1} for {-3, -1, +1, +3}. An empty table emits the
rank itself, the identity map for any M. The input's spacing is the nominal 2; a stream at any other spacing is
normalized upstream.
)"">;

    PortIn<T>             in;
    PortOut<std::uint8_t> out;

    Annotated<gr::Size_t, "n_levels", Visible, Doc<"M, even and at least 2">>                                                                      n_levels = 4U;
    Annotated<std::vector<gr::Size_t>, "labels", Doc<"level rank (lowest to highest) to emitted value; size M, or empty to emit the rank itself">> labels{};

    GR_MAKE_REFLECTABLE(PamSlicer, in, out, n_levels, labels);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n_levels < 2U || (n_levels % 2U) != 0U) {
            throw gr::exception(std::format("n_levels must be even and at least 2, got {}", n_levels.value));
        }
        if (!labels.value.empty() && labels.value.size() != static_cast<std::size_t>(n_levels.value)) {
            throw gr::exception(std::format("labels holds {} entries for {} levels; one per level rank", labels.value.size(), n_levels.value));
        }
    }

    [[nodiscard]] std::uint8_t processOne(T sample) const noexcept {
        const std::size_t rank = detail::pamRank(static_cast<double>(sample), 2.0, static_cast<std::size_t>(n_levels.value));
        return static_cast<std::uint8_t>(labels.value.empty() ? rank : labels.value[rank]);
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_PAM_SYMBOLS_HPP
