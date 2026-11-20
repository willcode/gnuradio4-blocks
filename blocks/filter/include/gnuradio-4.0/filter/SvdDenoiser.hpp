#ifndef GNURADIO_SVD_DENOISER_HPP
#define GNURADIO_SVD_DENOISER_HPP

#include <algorithm>
#include <cstdint>
#include <optional>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/filter/SvdFilter.hpp>

#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

using namespace gr;

GR_REGISTER_BLOCK(gr::blocks::filter::SvdDenoiser, [T], [ float, std::complex<float> ])

template<typename T>
struct SvdDenoiser : Block<SvdDenoiser<T>>, detail::DelayedTagFilter<SvdDenoiser<T>, T, T> {
    using Block<SvdDenoiser<T>>::Block;
    static_assert(std::floating_point<T> || gr::meta::complex_like<T>, "T must be floating_point or complex_like");

    using Description = Doc<R""(@brief SVD-based signal denoiser

Denoises signals using Singular Value Decomposition applied to a Hankel matrix
representation. Effective for removing broadband noise from periodic or
quasi-periodic signals while preserving signal structure.

Singular values are kept if ALL criteria are satisfied:
- count ≤ max_rank
- σ_i / σ_0 ≥ relative_threshold
- σ_i ≥ absolute_threshold
- cumulative energy ≤ energy_fraction × total energy

The output lags the input by `max((window_size-1)/2, hop-1)` samples, where `hop` is the SVD recomputation interval in
samples. Each tag moves with all its keys by that lag. A tag on input `i` leaves on output `i + lag`. A tag whose
output lies past the stream's last output leaves at the end-of-stream index, one past that output.

`max rank` set to its maximum keeps every singular value. `hop fraction` is the SVD recomputation interval as a fraction
of the window size.
)"">; // clang-format off

    using RealT = gr::meta::fundamental_base_value_type_t<T>;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"samples">, Doc<"window size">>                   window_size = 64U;
        Annotated<gr::Size_t, Doc<"Hankel row count (0 = window_size / 2)">> hankel_rows = 0U;
        Annotated<gr::Size_t, Doc<"rank limit">>    max_rank    = std::numeric_limits<gr::Size_t>::max();
        Annotated<RealT, Doc<"relative singular value threshold">, Unit<"ratio">> relative_threshold = std::numeric_limits<RealT>::epsilon();
        Annotated<RealT, Doc<"absolute singular value threshold">> absolute_threshold = std::numeric_limits<RealT>::epsilon();
        Annotated<RealT, Doc<"retained energy fraction">, Unit<"ratio">> energy_fraction = RealT(0.95);
        Annotated<RealT, Doc<"recomputation interval">, Unit<"ratio">> hop_fraction = RealT{0.25};

        GR_MAKE_REFLECTABLE(Settings, window_size, hankel_rows, max_rank, relative_threshold, absolute_threshold, energy_fraction, hop_fraction);
    } settings;

    GR_MAKE_REFLECTABLE(SvdDenoiser, in, out, settings);

private:
    algorithm::svd_filter::SvdDenoiser<T> _state;

    [[nodiscard]] algorithm::svd_filter::Config<RealT> buildConfig() const {
        return {
            .maxRank           = static_cast<std::size_t>(settings.max_rank),
            .relativeThreshold = settings.relative_threshold,
            .absoluteThreshold = settings.absolute_threshold,
            .energyFraction    = settings.energy_fraction,
            .hopFraction       = settings.hop_fraction
        };
    }

public:
    void start() {
        _state = algorithm::svd_filter::SvdDenoiser<T>(
            static_cast<std::size_t>(settings.window_size),
            static_cast<std::size_t>(settings.hankel_rows),
            buildConfig());
        this->tagsStart();
    }

    void applySettings(const Settings& s) {
        _state.setParameters(
            static_cast<std::size_t>(s.window_size),
            static_cast<std::size_t>(s.hankel_rows),
            buildConfig());
    }

    void reset() { _state.reset(); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// @brief The lag in half samples. Each SVD computation serves the next `hop` outputs, starting at index `W-1-lag`
    /// of its window counted from the oldest sample. That index is `W-1-(W-1)/2`, or `W-hop` when a hop reaches past
    /// the window's middle.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept {
        const std::uint64_t window = _state.windowSize();
        const std::uint64_t hop    = _state.hopSize();
        const std::uint64_t first  = std::min(window - 1ULL - (window - 1ULL) / 2ULL, window > hop ? window - hop : 0ULL);
        return 2ULL * (window - 1ULL - first);
    }

    void filterSamples(std::span<const T> input, std::span<T> output) {
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            output[i] = _state.processOne(input[i]);
        }
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_SVD_DENOISER_HPP
