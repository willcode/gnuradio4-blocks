#ifndef GNURADIO_ARRAY_COVARIANCE_ESTIMATE_HPP
#define GNURADIO_ARRAY_COVARIANCE_ESTIMATE_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/algorithm/math/Covariance.hpp>

namespace gr::blocks::array {

GR_REGISTER_BLOCK(gr::blocks::array::CovarianceEstimate, [T], [ std::complex<float>, std::complex<double> ])

template<typename T>
struct CovarianceEstimate : Block<CovarianceEstimate<T>> {
    using Description = Doc<R""(@brief Estimates the N x N sample covariance of N inputs and publishes one DataSet per window.

Entry `[i][j]` of the estimate is the average of `in[i] * conj(in[j])`, so the matrix is Hermitian. The inputs
are `in#0` to `in#N-1`, sized by `n_inputs`. Every `window` samples the block publishes the estimate as a
DataSet of extents N x N, row by row. With `mode` 'block' each estimate averages the window's samples with
equal weight. With `mode` 'exponential' the estimate runs across windows: a sample k samples old has the
weight `forgetting_factor^k`, and the sum is divided by the sum of the weights. The two axes of the DataSet
are the row element and the column element, with values 0 to N-1. Its signal is named `covariance`, and the
signal's meta information holds `window_start`, the input index of the window's first sample. A setting
outside its range ends the run with an error that names it.
)"">;

    std::vector<PortIn<T>>     in = std::vector<PortIn<T>>(2UZ);
    PortOut<DataSet<T>, Async> out;

    Annotated<gr::Size_t, "n_inputs", Visible, Doc<"number of inputs N">, Limits<1U, 1024U>>               n_inputs          = 2U;
    Annotated<gr::Size_t, "window", Visible, Doc<"samples per estimate">>                                  window            = 1024U;
    Annotated<std::string, "mode", Visible, Doc<"'block' or 'exponential'">>                               mode              = std::string("block");
    Annotated<double, "forgetting_factor", Doc<"weight per sample of age in exponential mode, in (0, 1]">> forgetting_factor = 0.99;

    GR_MAKE_REFLECTABLE(CovarianceEstimate, in, out, n_inputs, window, mode, forgetting_factor);

    using real_type = gr::meta::fundamental_base_value_type_t<T>;

    bool                               _running = false;
    gr::math::CovarianceAccumulator<T> _accumulator{};
    std::vector<T>                     _snapshot{};
    gr::Tensor<T>                      _matrix{};
    std::size_t                        _filled      = 0UZ;
    std::uint64_t                      _windowStart = 0U;
    std::optional<std::string>         _refusal{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!_running && newSettings.contains("n_inputs")) {
            in.resize(static_cast<std::size_t>(n_inputs.value));
        }
        _refusal = configure();
    }

    void start() {
        _running     = true;
        _filled      = 0UZ;
        _windowStart = 0U;
        _refusal     = configure();
    }

    void stop() { _running = false; }

    /// Rebuilds the accumulator from the settings and restarts the window, or returns the error that refuses them.
    [[nodiscard]] std::optional<std::string> configure() {
        if (window.value == 0U) {
            return std::string("window must be at least 1 sample");
        }
        _accumulator = gr::math::CovarianceAccumulator<T>(in.size(), static_cast<std::size_t>(window.value));
        if (mode.value == "block") {
            _accumulator.setBlockAveraging(static_cast<std::size_t>(window.value));
        } else if (mode.value == "exponential") {
            if (!_accumulator.setExponentialAveraging(static_cast<real_type>(forgetting_factor.value))) {
                return std::format("forgetting_factor must lie in (0, 1], got {}", forgetting_factor.value);
            }
        } else {
            return std::format("mode must be 'block' or 'exponential', got '{}'", mode.value);
        }
        _snapshot.assign(in.size(), T{});
        _windowStart += _filled;
        _filled = 0UZ;
        return std::nullopt;
    }

    template<InputSpanLike TInSpan>
    [[nodiscard]] work::Status processBulk(std::span<TInSpan>& inputs, OutputSpanLike auto& outSpan) {
        if (_refusal.has_value()) {
            this->emitErrorMessage("processBulk", *_refusal);
            return work::Status::ERROR;
        }
        std::size_t available = std::numeric_limits<std::size_t>::max();
        for (const auto& input : inputs) {
            available = std::min(available, input.size());
        }
        const std::size_t windowSize = static_cast<std::size_t>(window.value);
        const std::size_t untilClose = windowSize - _filled;
        const std::size_t room       = outSpan.size();
        const std::size_t limit      = room == 0UZ ? untilClose - 1UZ : untilClose + (room - 1UZ) * windowSize;
        const std::size_t take       = std::min(available, limit);

        std::size_t made = 0UZ;
        for (std::size_t s = 0UZ; s < take; ++s) {
            for (std::size_t k = 0UZ; k < inputs.size(); ++k) {
                _snapshot[k] = inputs[k][s];
            }
            std::ignore = _accumulator.add(std::span<const T>(_snapshot));
            if (++_filled == windowSize) {
                outSpan[made++] = estimate();
                _windowStart += windowSize;
                _filled = 0UZ;
            }
        }

        for (auto& input : inputs) {
            std::ignore = input.consume(take);
        }
        outSpan.publish(made);
        return work::Status::OK;
    }

    [[nodiscard]] DataSet<T> estimate() {
        _accumulator.matrix(_matrix);
        const std::size_t n = in.size();

        DataSet<T> dataSet;
        dataSet.axis_names = {"row element", "column element"};
        dataSet.axis_units = {"", ""};
        std::vector<T> elements(n);
        for (std::size_t i = 0UZ; i < n; ++i) {
            elements[i] = T{static_cast<real_type>(i)};
        }
        dataSet.axis_values       = {elements, elements};
        dataSet.extents           = {static_cast<std::int32_t>(n), static_cast<std::int32_t>(n)};
        dataSet.layout            = LayoutRight{};
        dataSet.signal_names      = {"covariance"};
        dataSet.signal_quantities = {"covariance"};
        dataSet.signal_units      = {""};
        dataSet.signal_values.assign(_matrix.data(), _matrix.data() + n * n);
        dataSet.meta_information = {property_map{{std::pmr::string("window_start"), pmt::Value(_windowStart)}}};
        return dataSet;
    }
};

} // namespace gr::blocks::array

#endif // GNURADIO_ARRAY_COVARIANCE_ESTIMATE_HPP
