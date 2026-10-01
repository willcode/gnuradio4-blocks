#ifndef GNURADIO_ARRAY_LINEAR_COMBINE_HPP
#define GNURADIO_ARRAY_LINEAR_COMBINE_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/algorithm/math/TensorMath.hpp>

namespace gr::blocks::array {

namespace detail {

/// The extents of a tensor as text, for an error message.
template<typename T>
[[nodiscard]] std::string extentsText(const gr::Tensor<T>& tensor) {
    std::string text;
    for (const std::size_t extent : tensor.extents()) {
        text += text.empty() ? std::format("{}", extent) : std::format(" x {}", extent);
    }
    return text.empty() ? std::string("none") : text;
}

/// The rows x columns matrix a tensor holds: an M x N tensor, or a flat list of M*N entries read row by row.
/// The error names the tensor's extents and the expected ones.
template<typename T>
[[nodiscard]] std::optional<std::string> toMatrix(const gr::Tensor<T>& tensor, std::size_t rows, std::size_t columns, std::string_view name, gr::Tensor<T>& matrix) {
    const bool square = tensor.rank() == 2UZ && tensor.extent(0) == rows && tensor.extent(1) == columns;
    const bool flat   = tensor.rank() == 1UZ && tensor.size() == rows * columns;
    if (rows == 0UZ || columns == 0UZ || !(square || flat)) {
        return std::format("{} of extents {} disagree with {} outputs by {} inputs, which need {} x {}", name, extentsText(tensor), rows, columns, rows, columns);
    }
    matrix.resize({rows, columns});
    std::copy_n(tensor.data(), rows * columns, matrix.data());
    return std::nullopt;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::array::LinearCombine, [T], [ std::complex<float>, std::complex<double> ])

template<typename T>
struct LinearCombine : Block<LinearCombine<T>> {
    using Description = Doc<R""(@brief Combines N inputs into M outputs through an M x N complex weight matrix.

Output `m` carries `sum over n of weights[m][n] * in[n]` at each sample. The inputs are `in#0` to `in#N-1`
and the outputs `out#0` to `out#M-1`, sized by `n_inputs` and `n_outputs`. `weights` holds the matrix as
an M x N tensor or as a flat list of M*N entries, row by row. Weights of other extents end the run with
an error that names their extents and the expected ones. A settings message replaces the weights from
the next sample the block processes. A `weights` tag on any input replaces them from the tag's index.
The block forms the outputs of a span with one matrix product, the span's samples times the transposed weight matrix.
)"">;

    std::vector<PortIn<T>>  in  = std::vector<PortIn<T>>(2UZ);
    std::vector<PortOut<T>> out = std::vector<PortOut<T>>(1UZ);

    Annotated<gr::Size_t, "n_inputs", Visible, Doc<"number of inputs N">, Limits<1U, 1024U>>                n_inputs  = 2U;
    Annotated<gr::Size_t, "n_outputs", Visible, Doc<"number of outputs M">, Limits<1U, 1024U>>              n_outputs = 1U;
    Annotated<gr::Tensor<T>, "weights", Visible, Doc<"M x N weight matrix, or its M*N entries row by row">> weights{};

    GR_MAKE_REFLECTABLE(LinearCombine, in, out, n_inputs, n_outputs, weights);

    bool                       _running = false;
    gr::Tensor<T>              _transposed{}; ///< N x M, the transpose of the weights in force
    gr::Tensor<T>              _samples{};    ///< L x N, one row per sample index of a span
    gr::Tensor<T>              _combined{};   ///< L x M, one row per sample index of a span
    std::optional<std::string> _refusal{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!_running && (newSettings.contains("n_inputs") || newSettings.contains("n_outputs"))) {
            in.resize(static_cast<std::size_t>(n_inputs.value));
            out.resize(static_cast<std::size_t>(n_outputs.value));
        }
        if (newSettings.contains("weights") || newSettings.contains("n_inputs") || newSettings.contains("n_outputs")) {
            _refusal = install(weights.value);
        }
    }

    void start() {
        _running = true;
        _refusal = install(weights.value);
    }

    void stop() { _running = false; }

    template<InputSpanLike TInSpan, OutputSpanLike TOutSpan>
    [[nodiscard]] work::Status processBulk(std::span<TInSpan>& inputs, std::span<TOutSpan>& outputs) {
        if (_refusal.has_value()) {
            this->emitErrorMessage("processBulk", *_refusal);
            return work::Status::ERROR;
        }
        std::size_t n = std::numeric_limits<std::size_t>::max();
        for (const auto& input : inputs) {
            n = std::min(n, input.size());
        }
        for (const auto& output : outputs) {
            n = std::min(n, output.size());
        }

        std::vector<std::pair<std::size_t, const gr::Tensor<T>*>> changes;
        for (auto& input : inputs) {
            for (const auto& [relIndex, tagMap] : input.tags()) {
                if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= n) {
                    continue;
                }
                if (const auto it = tagMap.get().find("weights"); it != tagMap.get().end()) {
                    changes.emplace_back(static_cast<std::size_t>(relIndex), it->second.template get_if<gr::Tensor<T>>());
                }
            }
        }
        std::ranges::stable_sort(changes, {}, &std::pair<std::size_t, const gr::Tensor<T>*>::first);

        std::size_t done = 0UZ;
        for (const auto& [at, tensor] : changes) {
            combine(inputs, outputs, done, at);
            done     = at;
            _refusal = tensor == nullptr ? std::optional<std::string>(std::format("the weights tag at index {} holds no tensor of the block's sample type", inputs[0].streamIndex + at)) : install(*tensor);
            if (_refusal.has_value()) {
                break;
            }
        }
        if (!_refusal.has_value()) {
            combine(inputs, outputs, done, n);
            done = n;
        }

        for (auto& input : inputs) {
            std::ignore = input.consume(done);
        }
        for (auto& output : outputs) {
            output.publish(done);
        }
        if (_refusal.has_value()) {
            this->emitErrorMessage("processBulk", *_refusal);
            return work::Status::ERROR;
        }
        return work::Status::OK;
    }

    /// Takes `tensor` as the weights in force, or returns the error that refuses it and keeps the weights.
    [[nodiscard]] std::optional<std::string> install(const gr::Tensor<T>& tensor) {
        const std::size_t nOutputs = out.size();
        const std::size_t nInputs  = in.size();
        gr::Tensor<T>     matrix;
        if (auto refusal = detail::toMatrix(tensor, nOutputs, nInputs, "weights", matrix); refusal.has_value()) {
            return refusal;
        }
        _transposed.resize({nInputs, nOutputs});
        for (std::size_t m = 0UZ; m < nOutputs; ++m) {
            for (std::size_t n = 0UZ; n < nInputs; ++n) {
                _transposed[n, m] = matrix[m, n];
            }
        }
        return std::nullopt;
    }

    /// Writes outputs [from, to) of every output port from the same input samples.
    template<typename TInSpan, typename TOutSpan>
    void combine(std::span<TInSpan>& inputs, std::span<TOutSpan>& outputs, std::size_t from, std::size_t to) {
        if (to <= from) {
            return;
        }
        const std::size_t length  = to - from;
        const std::size_t nInputs = inputs.size();
        _samples.resize({length, nInputs});
        for (std::size_t k = 0UZ; k < nInputs; ++k) {
            const auto& input = inputs[k];
            for (std::size_t l = 0UZ; l < length; ++l) {
                _samples[l, k] = input[from + l];
            }
        }
        const std::size_t nOutputs = outputs.size();
        _combined.resize({length, nOutputs});
        gr::math::gemm(_combined, _samples, _transposed);
        for (std::size_t m = 0UZ; m < nOutputs; ++m) {
            auto& output = outputs[m];
            for (std::size_t l = 0UZ; l < length; ++l) {
                output[from + l] = _combined[l, m];
            }
        }
    }
};

} // namespace gr::blocks::array

#endif // GNURADIO_ARRAY_LINEAR_COMBINE_HPP
