#ifndef GNURADIO_TIME_DOMAIN_FILTER_HPP
#define GNURADIO_TIME_DOMAIN_FILTER_HPP
#include <algorithm>
#include <cmath>
#include <concepts>
#include <execution>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <span>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/HistoryBuffer.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterTool.hpp>
#include <gnuradio-4.0/meta/UncertainValue.hpp>

#include <magic_enum.hpp>

#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

using namespace gr;

/// @brief Zeroes a filter state that has decayed below the smallest normal value.
///
/// A recursion driven by exact zeros decays into the subnormals and stays there, and on x86 every operation on a
/// subnormal costs a microcode assist of order a hundred cycles. The test costs a compare and a conditional move.
template<std::floating_point T>
[[nodiscard]] T flushSubnormal(T value) noexcept {
    return std::abs(value) < std::numeric_limits<T>::min() ? T{} : value;
}

GR_REGISTER_BLOCK(gr::blocks::filter::fir_filter, [T], [float])

template<typename T>
requires std::floating_point<T>
struct fir_filter : Block<fir_filter<T>>, detail::DelayedTagFilter<fir_filter<T>, T, T> {
    using Description = Doc<R""(@brief Finite Impulse Response (FIR) filter class

The transfer function of an FIR filter is given by:
H(z) = b[0] + b[1]*z^-1 + b[2]*z^-2 + ... + b[N]*z^-N

Each tag moves with all its keys by the filter's delay `d`. A tag on input `i` leaves on output `i + d`, with a half
rounded up. A symmetric or antisymmetric set of `N+1` coefficients delays by `N/2`. An asymmetric set moves its tags by
the centroid of its energy, rounded to the whole sample. A tag whose output lies past the stream's last output leaves at
the end-of-stream index, one past that output.
)"">;
    PortIn<T>  in;
    PortOut<T> out;
    struct Settings {
        Annotated<Tensor<T>> b = Tensor<T>{T{1}}; // feedforward coefficients

        GR_MAKE_REFLECTABLE(Settings, b);
    } settings;

    GR_MAKE_REFLECTABLE(fir_filter, in, out, settings);

    HistoryBuffer<T> inputHistory{32};

    void applySettings(const Settings& s) noexcept {
        if (s.b.changed() && s.b->size() > inputHistory.capacity()) {
            inputHistory = HistoryBuffer<T>(std::bit_ceil(s.b->size()));
        }
    }

    void start() { this->tagsStart(); }

    [[nodiscard]] std::size_t tagDecimation() const noexcept { return 1UZ; }

    /// @brief The delay every forwarded tag moves by, in half samples. The value is `N` for a symmetric set of `N+1`
    /// coefficients.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept { return detail::twiceTapDelay(std::span<const T>(settings.b->data(), settings.b->size())); }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            inputHistory.push_front(input[i]);
            output[i] = std::transform_reduce(std::execution::unseq, settings.b->cbegin(), settings.b->cend(), inputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{});
        }
    }
};

enum class IIRForm {
    DF_I,  /// direct form I: preferred for fixed-point arithmetics (e.g. no overflow)
    DF_II, /// direct form II: preferred for floating-point arithmetics (less operations)
    DF_I_TRANSPOSED,
    DF_II_TRANSPOSED,
};

GR_REGISTER_BLOCK("gr::blocks::filter::IirFilterDirectForm1", gr::blocks::filter::IirFilterDirectForm1, [T], [float])
GR_REGISTER_BLOCK("gr::blocks::filter::IirFilterDirectForm2", gr::blocks::filter::IirFilterDirectForm2, [T], [float])
GR_REGISTER_BLOCK("gr::blocks::filter::IirFilterDirectForm1Transposed", gr::blocks::filter::IirFilterDirectForm1Transposed, [T], [float])
GR_REGISTER_BLOCK("gr::blocks::filter::IirFilterDirectForm2Transposed", gr::blocks::filter::IirFilterDirectForm2Transposed, [T], [float])

template<typename T, IIRForm form = std::is_floating_point_v<T> ? IIRForm::DF_II : IIRForm::DF_I>
requires std::floating_point<T>
struct iir_filter : Block<iir_filter<T, form>> {
    using Description = Doc<R""(
@brief Infinite Impulse Response (IIR) filter class

b are the feed-forward coefficients (N.B. b[0] denoting the newest and b[-1] the previous sample)
a are the feedback coefficients
)"">;
    PortIn<T>  in;
    PortOut<T> out;
    struct Settings {
        Annotated<Tensor<T>> b = Tensor<T>{1}; // feed-forward coefficients
        Annotated<Tensor<T>> a = Tensor<T>{1}; // feedback coefficients

        GR_MAKE_REFLECTABLE(Settings, b, a);
    } settings;

    GR_MAKE_REFLECTABLE(iir_filter, in, out, settings);

    HistoryBuffer<T> inputHistory{32};
    HistoryBuffer<T> outputHistory{32};

    void applySettings(const Settings& s) noexcept {
        const auto new_size = std::max(s.a->size(), s.b->size());
        if ((s.b.changed() || s.a.changed()) && new_size >= inputHistory.capacity()) {
            inputHistory  = HistoryBuffer<T>(std::bit_ceil(new_size));
            outputHistory = HistoryBuffer<T>(std::bit_ceil(new_size));
        }
    }

    // Every form flushes the value it feeds back, so a silent input leaves the state at zero rather than subnormal.
    [[nodiscard]] T processOne(T input) noexcept {
        if constexpr (form == IIRForm::DF_I) {
            // y[n] = b[0] * x[n]   + b[1] * x[n-1] + ... + b[N] * x[n-N]
            //      - a[1] * y[n-1] - a[2] * y[n-2] - ... - a[M] * y[n-M]
            inputHistory.push_front(input);
            const T feedforward = std::transform_reduce(std::execution::unseq, settings.b->cbegin(), settings.b->cend(), inputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{});
            const T feedback    = std::transform_reduce(std::execution::unseq, settings.a->cbegin() + 1, settings.a->cend(), outputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{});
            const T output      = flushSubnormal(feedforward - feedback);
            outputHistory.push_front(output);
            return output;
        } else if constexpr (form == IIRForm::DF_II) {
            // w[n] = x[n] - a[1] * w[n-1] - a[2] * w[n-2] - ... - a[M] * w[n-M]
            // y[n] =        b[0] * w[n]   + b[1] * w[n-1] + ... + b[N] * w[n-N]
            const T w = flushSubnormal(input - std::transform_reduce(std::execution::unseq, settings.a->cbegin() + 1, settings.a->cend(), inputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{}));
            inputHistory.push_front(w);

            return std::transform_reduce(std::execution::unseq, settings.b->cbegin(), settings.b->cend(), inputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{});
        } else if constexpr (form == IIRForm::DF_I_TRANSPOSED) {
            // w_1[n] = x[n] - a[1] * w_2[n-1] - a[2] * w_2[n-2] - ... - a[M] * w_2[n-M]
            // y[n]   = b[0] * w_2[n] + b[1] * w_2[n-1] + ... + b[N] * w_2[n-N]
            const T v0 = flushSubnormal(input - std::transform_reduce(std::execution::unseq, settings.a->cbegin() + 1, settings.a->cend(), outputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{}));
            outputHistory.push_front(v0);

            return std::transform_reduce(std::execution::unseq, settings.b->cbegin(), settings.b->cend(), outputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{});
        } else if constexpr (form == IIRForm::DF_II_TRANSPOSED) {
            // y[n] = b_0 * f[n] + Σ (b_k * f[n−k] − a_k * y[n−k]) for k = 1 to N
            const T output = flushSubnormal(settings.b[0] * input + std::transform_reduce(std::execution::unseq, settings.b->cbegin() + 1, settings.b->cend(), inputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{}) - std::transform_reduce(std::execution::unseq, settings.a->cbegin() + 1, settings.a->cend(), outputHistory.cbegin(), T{0}, std::plus<>{}, std::multiplies<>{}));

            inputHistory.push_front(input);
            outputHistory.push_front(output);
            return output;
        }
    }
};

// one name per form: the four differ only in the difference equation processOne evaluates, which the enumerator names
// and the type carries, so each is registered under a name that says which form it is
template<typename T>
using IirFilterDirectForm1 = iir_filter<T, IIRForm::DF_I>;
template<typename T>
using IirFilterDirectForm2 = iir_filter<T, IIRForm::DF_II>;
template<typename T>
using IirFilterDirectForm1Transposed = iir_filter<T, IIRForm::DF_I_TRANSPOSED>;
template<typename T>
using IirFilterDirectForm2Transposed = iir_filter<T, IIRForm::DF_II_TRANSPOSED>;

// both aliases need the explicit registry name: derived from the type, an alias carries the name of the template it
// expands to together with the whole argument list — for the decimating one the tag policy and the resampling
// annotation included — and a saved graph naming the alias would not resolve
GR_REGISTER_BLOCK("gr::blocks::filter::BasicFilter", gr::blocks::filter::BasicFilter, ([T]), [ float, gr::UncertainValue<float> ])
GR_REGISTER_BLOCK("gr::blocks::filter::BasicDecimatingFilter", gr::blocks::filter::BasicDecimatingFilter, ([T]), [ float, gr::UncertainValue<float> ])

enum class FilterType { FIR, IIR };

template<typename T, typename... Args>
requires(std::floating_point<T> or std::is_arithmetic_v<meta::fundamental_base_value_type_t<T>>)
struct BasicFilterProto : Block<BasicFilterProto<T, Args...>, Args...>, detail::DelayedTagFilter<BasicFilterProto<T, Args...>, T, T> {
    using TParent     = Block<BasicFilterProto<T, Args...>, Args...>;
    using Description = Doc<R""(@brief Basic Digital Filter class supporting FIR and IIR filters

This block implements a digital filter which can be configured as either FIR or IIR,
with selectable filter type (low-pass, high-pass, band-pass, band-stop), and supports resampling.

In FIR mode each tag moves with all its keys by the designed filter's delay `d`, which is `(N-1)/2` for its `N`
coefficients. A tag on input `i` leaves on output `round((i + d) / M)`. `M` is the decimation of `BasicDecimatingFilter`
and 1 for `BasicFilter`. A tag whose output lies past the stream's last output leaves at the end-of-stream index, one
past that output. In IIR mode the framework places the tags and forwards every key. An IIR response has
no single delay.
)"">;
    using ValueType   = meta::fundamental_base_value_type_t<T>;

    PortIn<T>  in;
    PortOut<T> out;

    using FilterImpl = std::conditional_t<UncertainValueLike<T>, gr::filter::ErrorPropagatingFilter<T>, gr::filter::Filter<T>>;

    FilterImpl    _filter;
    std::uint64_t _twiceFirDelay = 0ULL; /// the FIR design's delay in half input samples, `twiceTapDelay`

    // Public settings
    struct Settings {
        Annotated<FilterType, Doc<"filter type">, Visible>           filter_type     = FilterType::IIR;
        Annotated<gr::filter::Type, Doc<"filter response">, Visible> filter_response = gr::filter::Type::LOWPASS;
        Annotated<gr::Size_t, Doc<"filter order">>                   filter_order{3};
        Annotated<float, Unit<"Hz">, Doc<"low cutoff">, Visible>     f_low{0.1f};
        Annotated<float, Unit<"Hz">, Doc<"high cutoff">, Visible>    f_high{0.2f};
        Annotated<float, Unit<"Hz">, Doc<"sample rate">, Visible>    sample_rate{1.0f};
        Annotated<gr::Size_t, Doc<"decimation">, Limits<Min<1U>>>    decimate{1U};
        Annotated<gr::filter::iir::Design, Doc<"IIR design method">> iir_design_method = gr::filter::iir::Design::BUTTERWORTH;
        Annotated<algorithm::window::Type, Doc<"FIR design window">> fir_design_method = algorithm::window::Type::Kaiser;

        GR_MAKE_REFLECTABLE(Settings, filter_type, filter_response, filter_order, f_low, f_high, sample_rate, decimate, iir_design_method, fir_design_method);
    } settings;

    GR_MAKE_REFLECTABLE(BasicFilterProto, in, out, settings);

    // a restart designs the filter again, which clears its history
    void start() {
        designFilter();
        this->tagsStart();
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const gr::filter::FilterParameters params{.order = s.filter_order.value, .fLow = static_cast<double>(s.f_low.value), .fHigh = static_cast<double>(s.f_high.value), .fs = static_cast<double>(s.sample_rate.value)};
        if (s.filter_type.value == FilterType::FIR) {
            std::ignore = gr::filter::fir::designFilter<ValueType>(s.filter_response.value, params, s.fir_design_method.value);
        } else {
            std::ignore = gr::filter::iir::designFilter<ValueType>(s.filter_response.value, params, s.iir_design_method.value);
        }
        return {};
    }

    // re-designing resets the filter history, so only the settings the design depends on may trigger it
    void applySettings(const Settings& s) {
        const bool redesign = s.filter_type.changed() || s.filter_response.changed() || s.filter_order.changed() || s.f_low.changed() || s.f_high.changed() || s.sample_rate.changed() || s.decimate.changed() || s.iir_design_method.changed() || s.fir_design_method.changed();
        if (redesign) {
            designFilter();
        }
        if (s.decimate.changed()) {
            this->tagsMarkReorigin();
        }
    }

    void designFilter() {
        using namespace gr::filter;

        if constexpr (not TParent::ResamplingControl::kIsConst) {
            this->input_chunk_size = settings.decimate;
        }

        // Set up filter parameters
        FilterParameters params;
        params.order = settings.filter_order;
        params.fLow  = static_cast<double>(settings.f_low);
        params.fHigh = static_cast<double>(settings.f_high);
        params.fs    = static_cast<double>(settings.sample_rate);

        if (settings.filter_type == FilterType::FIR) { // design FIR filter
            const FilterCoefficients<ValueType> coefficients = fir::designFilter<ValueType>(settings.filter_response, params, settings.fir_design_method);
            _twiceFirDelay                                   = detail::twiceTapDelay(std::span<const ValueType>(coefficients.b));
            _filter                                          = FilterImpl(coefficients);
        } else if (settings.filter_type == FilterType::IIR) { // design IIR filter
            _filter = FilterImpl(iir::designFilter<ValueType>(settings.filter_response, params, settings.iir_design_method));
        }
    }

    /// @brief Input samples per output. It is `decimate` where the block resamples and 1 otherwise.
    [[nodiscard]] std::size_t tagDecimation() const noexcept {
        if constexpr (TParent::ResamplingControl::kIsConst) {
            return 1UZ;
        } else {
            return static_cast<std::size_t>(settings.decimate);
        }
    }

    /// @brief The FIR design's delay in half input samples, or no value for an IIR design.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept {
        if (settings.filter_type == FilterType::FIR) {
            return _twiceFirDelay;
        }
        return std::nullopt;
    }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        const std::size_t decimation = tagDecimation();
        assert(output.size() >= input.size() / decimation);

        std::size_t out_sample_idx = 0;
        for (std::size_t i = 0; i < input.size(); ++i) {
            T output_sample = _filter.processOne(input[i]);

            if (i % decimation == 0) {
                output[out_sample_idx++] = output_sample;
            }
        }
    }
};

template<typename T>
using BasicFilter = BasicFilterProto<T>;

template<typename T>
using BasicDecimatingFilter = BasicFilterProto<T, BackwardTagPropagation, Resampling<1UZ, 1UZ, false>>;

GR_REGISTER_BLOCK(gr::blocks::filter::Decimator, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::UncertainValue<float> ])

template<typename T>
struct Decimator : Block<Decimator<T>, BackwardTagPropagation, Resampling<1UZ, 1UZ, false>>, detail::DelayedTagFilter<Decimator<T>, T, T> {
    using TParent     = Block<Decimator<T>, BackwardTagPropagation, Resampling<1UZ, 1UZ, false>>;
    using Description = Doc<R""(@brief Basic Decimator Block

This block implements a decimator for downsampling (dropping) input data by a
configurable factor. Filtering is not included in this implementation so expect
aliasing and sub-sampling related effects.

The framework places the tags and forwards every key. A stream whose length is not a multiple of the
factor ends in a partial chunk, which makes no output. The tags on that chunk leave at the end-of-stream index, one past
the last output. So do the tags past the last input sample.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<gr::Size_t, Doc<"decimation">, Visible, Limits<Min<1U>>> decim{1};

        GR_MAKE_REFLECTABLE(Settings, decim);
    } settings;

    GR_MAKE_REFLECTABLE(Decimator, in, out, settings);

    void start() { this->tagsStart(); }

    void applySettings(const Settings& s) { this->input_chunk_size = s.decim; }

    /// @brief Input samples per output, `decim`.
    [[nodiscard]] std::size_t tagDecimation() const noexcept { return static_cast<std::size_t>(settings.decim); }

    /// @brief No delay. The framework places the tags.
    [[nodiscard]] std::optional<std::uint64_t> twiceTagDelay() const noexcept { return std::nullopt; }

    void filterSamples(std::span<const T> input, std::span<T> output) noexcept {
        assert(output.size() >= input.size() / settings.decim);

        std::size_t out_sample_idx = 0;
        for (std::size_t i = 0; i < input.size(); ++i) {
            if (i % settings.decim == 0) {
                output[out_sample_idx++] = input[i];
            }
        }
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_TIME_DOMAIN_FILTER_HPP
