#ifndef GNURADIO_SAVITZKY_GOLAY_FILTER_HPP
#define GNURADIO_SAVITZKY_GOLAY_FILTER_HPP

#include <cstdint>
#include <optional>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/algorithm/dataset/DataSetHelper.hpp>
#include <gnuradio-4.0/algorithm/filter/SavitzkyGolay.hpp>

#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

using namespace gr;

// ============================================================================
// Streaming Savitzky-Golay Filter (scalar T samples)
// ============================================================================

GR_REGISTER_BLOCK(gr::blocks::filter::SavitzkyGolayFilter, [T], [float])

template<typename T>
struct SavitzkyGolayFilter : Block<SavitzkyGolayFilter<T>>, detail::DelayedTagFilter<SavitzkyGolayFilter<T>, T, T> {
    using Block<SavitzkyGolayFilter<T>>::Block;
    static_assert(std::floating_point<T>, "T must be floating_point");

    using Description = Doc<R""(@brief Savitzky-Golay streaming filter

Applies local polynomial smoothing/differentiation to streaming scalar data.
Filter coefficients are computed using SVD-based least-squares fitting.

Alignment modes, by the value of `alignment`:
- every value other than `Causal`, the default included: the fit is at the window's middle sample, the older one for
  an even window. The output lags by ceil((window_size-1)/2) samples. An odd window is linear-phase.
- `Causal`: past-only window, minimal latency, non-linear phase

Each tag moves with all its keys by that lag, which is 0 for `Causal`. A tag on input `i` leaves on output `i + lag`.
A tag whose output lies past the stream's last output leaves at the end-of-stream index, one past that output.

`sample rate` scales the derivative.
)"">; // clang-format off

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"samples">, Doc<"window size">>  window_size  = 11U;
        Annotated<gr::Size_t, Doc<"polynomial order">>                       poly_order   = 4U;
        Annotated<gr::Size_t, Doc<"derivative order (0 = smoothing)">>     deriv_order  = 0U;
        Annotated<float, Doc<"sample rate">, Unit<"Hz">>        sample_rate  = 1.0f;
        Annotated<algorithm::savitzky_golay::Alignment, Doc<"alignment">> alignment    = algorithm::savitzky_golay::Alignment::Centred;

        GR_MAKE_REFLECTABLE(Settings, window_size, poly_order, deriv_order, sample_rate, alignment);
    } settings;

    GR_MAKE_REFLECTABLE(SavitzkyGolayFilter, in, out, settings);

private:
    algorithm::savitzky_golay::SavitzkyGolayFilter<T> _state;

    [[nodiscard]] algorithm::savitzky_golay::Config<T> buildConfig() const {
        const T sampleRateT = static_cast<T>(settings.sample_rate.value);
        return {
            .derivOrder = static_cast<std::size_t>(settings.deriv_order),
            .delta      = (sampleRateT > T{0}) ? T{1} / sampleRateT : T{1},
            .alignment  = settings.alignment.value
        };
    }

public:
    void start() {
        _state = algorithm::savitzky_golay::SavitzkyGolayFilter<T>(
            static_cast<std::size_t>(settings.window_size),
            static_cast<std::size_t>(settings.poly_order),
            buildConfig());
        this->tagsStart();
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.poly_order >= s.window_size) {
            return gr::Refused(s.poly_order, "must be < window_size");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        _state.setParameters(
            static_cast<std::size_t>(s.window_size),
            static_cast<std::size_t>(s.poly_order),
            buildConfig());
    }

    void reset() { _state.reset(); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// @brief The lag in half samples. The centered fit is evaluated at index `(W-1)/2` of the window, counted from the
    /// oldest sample. That index is `W-1-(W-1)/2` samples behind the newest.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept {
        if (_state.alignment() == algorithm::savitzky_golay::Alignment::Causal) {
            return 0ULL;
        }
        const std::uint64_t window = _state.windowSize();
        return 2ULL * (window - 1ULL - (window - 1ULL) / 2ULL);
    }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            output[i] = _state.processOne(input[i]);
        }
    }
};

// ============================================================================
// DataSet Savitzky-Golay Filter (zero-phase batch processing)
// ============================================================================

GR_REGISTER_BLOCK(gr::blocks::filter::SavitzkyGolayDataSetFilter, [T], [ float ])

template<typename T>
struct SavitzkyGolayDataSetFilter : Block<SavitzkyGolayDataSetFilter<T>> {
    using Block<SavitzkyGolayDataSetFilter<T>>::Block;
    static_assert(std::floating_point<T>, "T must be floating_point");

    using Description = Doc<R""(@brief Savitzky-Golay filter for DataSet (zero-phase)

Applies zero-phase Savitzky-Golay filtering to DataSet signal_values using
forward-backward filtering. This eliminates phase distortion and preserves
peak positions - essential for spectroscopy applications.

Note: zero-phase filtering squares the magnitude response, providing
additional smoothing compared to single-pass filtering.

Boundary policy:
- Reflect: mirror indices at boundaries (default, continuous derivatives)
- Replicate: extend edge values
)"">; // clang-format off

    PortIn<DataSet<T>>  in;
    PortOut<DataSet<T>> out;

    struct Settings {
        Annotated<gr::Size_t, Unit<"samples">, Doc<"window size">>  window_size     = 11U;
        Annotated<gr::Size_t, Doc<"polynomial order">>                       poly_order      = 4U;
        Annotated<gr::Size_t, Doc<"derivative order (0 = smoothing)">>    deriv_order     = 0U;
        Annotated<algorithm::savitzky_golay::BoundaryPolicy, Doc<"boundary policy">, Limits<OneOf<algorithm::savitzky_golay::BoundaryPolicy::Reflect, algorithm::savitzky_golay::BoundaryPolicy::Replicate>>> boundary_policy = algorithm::savitzky_golay::BoundaryPolicy::Reflect;

        GR_MAKE_REFLECTABLE(Settings, window_size, poly_order, deriv_order, boundary_policy);
    } settings;

    GR_MAKE_REFLECTABLE(SavitzkyGolayDataSetFilter, in, out, settings);

private:
    std::vector<T> _coeffs;

    [[nodiscard]] static algorithm::savitzky_golay::Config<T> buildConfig(const Settings& s) {
        return {
            .derivOrder     = static_cast<std::size_t>(s.deriv_order),
            .delta          = T{1},
            .alignment      = algorithm::savitzky_golay::Alignment::Centred,
            .boundaryPolicy = s.boundary_policy.value
        };
    }

    void updateCoefficients(const Settings& s) {
        _coeffs = algorithm::savitzky_golay::computeCoefficients<T>(
            static_cast<std::size_t>(s.window_size),
            static_cast<std::size_t>(s.poly_order),
            buildConfig(s));
    }

public:
    void start() { updateCoefficients(settings); }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.poly_order >= s.window_size) {
            return gr::Refused(s.poly_order, "must be < window_size");
        }
        return {};
    }

    void applySettings(const Settings& s) { updateCoefficients(s); }

    [[nodiscard]] DataSet<T> processOne(DataSet<T> input) {
        if (input.signal_values.empty() || _coeffs.empty()) {
            return input;
        }

        std::vector<T> filtered(input.signal_values.size());
        if (input.size() <= 1UZ) {
            algorithm::savitzky_golay::applyZeroPhase<T>(std::span<const T>(input.signal_values), std::span<T>(filtered), std::span<const T>(_coeffs), buildConfig(settings));
        } else {
            // each signal filters on its own: one flat pass would slide the window across the
            // join between two signals
            std::size_t offset = 0UZ;
            for (std::size_t i = 0UZ; i < input.size(); ++i) {
                const std::span<const T> signal = input.signalValues(i);
                algorithm::savitzky_golay::applyZeroPhase<T>(signal, std::span<T>(filtered).subspan(offset, signal.size()), std::span<const T>(_coeffs), buildConfig(settings));
                offset += signal.size();
            }
        }
        input.signal_values = std::move(filtered);
        if (!input.signal_ranges.empty()) {
            gr::dataset::updateMinMax(input); // the filter moved every value, so a carried range is stale
        }
        return input;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_SAVITZKY_GOLAY_FILTER_HPP
