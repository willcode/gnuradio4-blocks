#ifndef GNURADIO_MATH_STREAM_HPP
#define GNURADIO_MATH_STREAM_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::math {

namespace stream {

/// `a + b`; an integer sum wraps modulo the range of `T`
template<typename T>
[[nodiscard]] constexpr T wrappingAdd(T a, T b) noexcept {
    if constexpr (std::integral<T>) {
        using U = std::make_unsigned_t<T>;
        return static_cast<T>(static_cast<U>(static_cast<U>(a) + static_cast<U>(b)));
    } else {
        return a + b;
    }
}

/// `a - b`; an integer difference wraps modulo the range of `T`
template<typename T>
[[nodiscard]] constexpr T wrappingSubtract(T a, T b) noexcept {
    if constexpr (std::integral<T>) {
        using U = std::make_unsigned_t<T>;
        return static_cast<T>(static_cast<U>(static_cast<U>(a) - static_cast<U>(b)));
    } else {
        return a - b;
    }
}

/// `acc + w * x`, a complex product formed from its components
template<typename T>
[[nodiscard]] constexpr T multiplyAdd(T acc, T w, T x) noexcept {
    if constexpr (meta::complex_like<T>) {
        return T(acc.real() + w.real() * x.real() - w.imag() * x.imag(), acc.imag() + w.real() * x.imag() + w.imag() * x.real());
    } else {
        return acc + w * x;
    }
}

/// The extents of a tensor as text, for an error message.
template<typename T>
[[nodiscard]] std::string extentsText(const gr::Tensor<T>& tensor) {
    std::string text;
    for (const std::size_t extent : tensor.extents()) {
        text += text.empty() ? std::format("{}", extent) : std::format(" x {}", extent);
    }
    return text.empty() ? std::string("none") : text;
}

/// A `rows` x `columns` matrix that maps each group of `columns` input samples to `rows` output samples. The entries
/// are held column by column. The inner loop runs over the rows, one independent sum per output sample.
template<typename T>
struct Weights {
    std::size_t    rows    = 1UZ;
    std::size_t    columns = 1UZ;
    std::vector<T> entries = std::vector<T>{T{1}}; ///< entry (r, c) at `c * rows + r`

    /// true when `nIn` input and `nOut` output samples are the same number of whole groups
    [[nodiscard]] constexpr bool fits(std::size_t nIn, std::size_t nOut) const noexcept { return nIn % columns == 0UZ && nOut % rows == 0UZ && nIn / columns == nOut / rows; }

    /// `output[g * rows + r] = sum over c of (r, c) * input[g * columns + c]` for each group `g`, summed in order of c.
    /// One row sums in a register; more rows sum in tiles of `kTile` local accumulators.
    void apply(std::span<const T> input, std::span<T> output) const noexcept {
        constexpr std::size_t kTile  = 16UZ;
        const std::size_t     groups = input.size() / columns;
        for (std::size_t g = 0UZ; g < groups; ++g) {
            const T* x = input.data() + g * columns;
            if (rows == 1UZ) {
                T acc{};
                for (std::size_t c = 0UZ; c < columns; ++c) {
                    acc = multiplyAdd(acc, entries[c], x[c]);
                }
                output[g] = acc;
                continue;
            }
            for (std::size_t r0 = 0UZ; r0 < rows; r0 += kTile) {
                const std::size_t    n = std::min(kTile, rows - r0);
                std::array<T, kTile> acc{};
                for (std::size_t c = 0UZ; c < columns; ++c) {
                    const T* w = entries.data() + c * rows + r0;
                    for (std::size_t r = 0UZ; r < n; ++r) {
                        acc[r] = multiplyAdd(acc[r], w[r], x[c]);
                    }
                }
                std::copy_n(acc.begin(), n, output.begin() + static_cast<std::ptrdiff_t>(g * rows + r0));
            }
        }
    }
};

/// The weights of a matrix: a 2-D matrix by its extents, where `rows` is 0 or its first extent; a 1-D matrix read row
/// by row into `rows` rows. The error names the matrix's extents.
template<typename T>
[[nodiscard]] std::expected<Weights<T>, std::string> matrixWeights(const gr::Tensor<T>& matrix, std::size_t rows) {
    const std::string extents = extentsText(matrix);
    Weights<T>        weights;
    if (matrix.size() == 0UZ) {
        return std::unexpected(std::format("matrix of extents {} is empty", extents));
    }
    if (matrix.rank() == 2UZ) {
        if (rows != 0UZ && rows != matrix.extent(0)) {
            return std::unexpected(std::format("matrix of extents {} has {} rows, and rows is {}", extents, matrix.extent(0), rows));
        }
        weights.rows    = matrix.extent(0);
        weights.columns = matrix.extent(1);
    } else if (matrix.rank() == 1UZ) {
        if (rows == 0UZ || matrix.size() % rows != 0UZ) {
            return std::unexpected(std::format("matrix of extents {} does not divide into rows {}", extents, rows));
        }
        weights.rows    = rows;
        weights.columns = matrix.size() / rows;
    } else {
        return std::unexpected(std::format("matrix of extents {} is not 2-D", extents));
    }
    weights.entries.resize(matrix.size());
    for (std::size_t r = 0UZ; r < weights.rows; ++r) {
        for (std::size_t c = 0UZ; c < weights.columns; ++c) {
            weights.entries[c * weights.rows + r] = matrix.data()[r * weights.columns + c];
        }
    }
    return weights;
}

/// The weights of a vector of `n` entries: one row of `n` columns. The error names the vector's extents.
template<typename T>
[[nodiscard]] std::expected<Weights<T>, std::string> vectorWeights(const gr::Tensor<T>& vector) {
    if (vector.rank() != 1UZ || vector.size() == 0UZ) {
        return std::unexpected(std::format("vector of extents {} is not 1-D with at least one entry", extentsText(vector)));
    }
    Weights<T> weights;
    weights.rows    = 1UZ;
    weights.columns = vector.size();
    weights.entries.assign(vector.data(), vector.data() + vector.size());
    return weights;
}

/// The weights a block computes with, the ones a settings change staged, and the refusal of the last change.
template<typename T>
struct StagedWeights {
    Weights<T>                 inForce{};
    std::optional<Weights<T>>  pending{};
    std::optional<std::string> refusal{};

    /// stages `next` and sets the block's chunk sizes to its group, or keeps the refusal
    template<typename TBlock>
    void stage(TBlock& block, std::expected<Weights<T>, std::string> next) {
        if (!next.has_value()) {
            refusal = std::move(next.error());
            return;
        }
        refusal.reset();
        block.input_chunk_size  = static_cast<gr::Size_t>(next->columns);
        block.output_chunk_size = static_cast<gr::Size_t>(next->rows);
        pending                 = std::move(*next);
    }

    /// takes the staged weights when the call's sample counts fit them; returns the error when the weights in force
    /// do not fit either. A second call for the same counts selects the same weights.
    [[nodiscard]] std::optional<std::string> select(std::size_t nIn, std::size_t nOut) {
        if (refusal.has_value()) {
            return refusal;
        }
        if (pending.has_value() && pending->fits(nIn, nOut)) {
            inForce = std::move(*pending);
            pending.reset();
        }
        if (!inForce.fits(nIn, nOut)) {
            return std::format("{} input and {} output samples are not whole groups of a {} x {} matrix", nIn, nOut, inForce.rows, inForce.columns);
        }
        return std::nullopt;
    }
};

/// Places a tag at input index `i` on output index `(i / columns) * rows`, the first output of its group, keeping the
/// keys and the rate scaling of `Block::filterAndSubstituteTag`. The block forwards a call's tags before it processes
/// the call, and this function selects the call's weights first. A tag at a negative index was placed by an earlier
/// call. A call with no input is the end of the stream, where every tag goes to output offset 0.
template<typename TBlock, typename T, typename TInputSpans, typename TOutputSpans>
void forwardGroupTags(TBlock& block, StagedWeights<T>& staged, TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
    std::size_t outputCount = 0UZ;
    gr::for_each_writer_span([&outputCount](auto& span) { outputCount = span.size(); }, outputSpans);
    if (processedIn > 0UZ) {
        std::ignore = staged.select(processedIn, outputCount);
    }
    const Weights<T>&           weights     = staged.inForce;
    const std::pmr::string      endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
    std::optional<property_map> cachedSettings;
    const auto                  place = [&](std::ptrdiff_t relIndex, const property_map& tagMap) {
        if (relIndex < 0) {
            return;
        }
        property_map forwarded = block.filterAndSubstituteTag(tagMap, cachedSettings);
        forwarded.erase(endOfStream);
        if (forwarded.empty()) {
            return;
        }
        const std::size_t at = processedIn == 0UZ ? 0UZ : static_cast<std::size_t>(relIndex) / weights.columns * weights.rows;
        gr::for_each_writer_span([&forwarded, at](auto& outSpan) { outSpan.publishTag(forwarded, at); }, outputSpans);
    };
    gr::for_each_reader_span(
        [&](auto& span) {
            if (!span.isSync || !span.isConnected) {
                return;
            }
            if (processedIn == 0UZ) {
                for (const auto& [relIndex, tagMap] : span.tags()) {
                    place(relIndex, tagMap.get());
                }
            } else {
                for (const auto& [relIndex, tagMap] : span.tags(processedIn)) {
                    place(relIndex, tagMap.get());
                }
            }
        },
        inputSpans);
}

/// a 1 x 1 matrix holding 1
template<typename T>
[[nodiscard]] gr::Tensor<T> unitMatrix() {
    gr::Tensor<T> matrix(gr::extents_from, {1UZ, 1UZ});
    matrix.data()[0] = T{1};
    return matrix;
}

} // namespace stream

GR_REGISTER_BLOCK(gr::blocks::math::CumulativeSum, [T], [ int16_t, int32_t, float, double, std::complex<float> ])

/**
 * @brief The running sum of the input, `y[i] = y[i-1] + x[i]`, zeroed by a tag.
 *
 * The sum starts at zero when the block starts. A tag on the input that carries the key `reset_key` zeroes the sum
 * before the tag's sample. The output at the tag's index equals the input there. An integer sum wraps modulo the
 * range of `T`. Every tag passes at its own index with all its keys, the reset tag included.
 */
template<typename T>
struct CumulativeSum : Block<CumulativeSum<T>> {
    using Description = Doc<"@brief Outputs the running sum of the input, zeroed at each tag that carries the key reset_key, an integer sum wrapping in the sample type.">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<std::string, "reset_key", Doc<"tag key that zeroes the sum">> reset_key = std::string("reset");

    GR_MAKE_REFLECTABLE(CumulativeSum, in, out, reset_key);

    T                _sum{};
    std::pmr::string _resetKey{"reset"};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { _resetKey = std::pmr::string(reset_key.value); }

    void start() { _sum = T{}; }

    /// writes the running sum of `input` to `output`, continuing from the sum of the previous samples
    void accumulate(std::span<const T> input, std::span<T> output) noexcept {
        T sum = _sum;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            sum       = stream::wrappingAdd(sum, input[i]);
            output[i] = sum;
        }
        _sum = sum;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const T> input(inSpan.data(), inSpan.size());
        const std::span<T>       output(outSpan.data(), outSpan.size());
        std::size_t              done = 0UZ;
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) >= input.size() || !tagMap.get().contains(_resetKey)) {
                continue;
            }
            const auto at = static_cast<std::size_t>(relIndex);
            accumulate(input.subspan(done, at - done), output.subspan(done, at - done));
            _sum = T{};
            done = at;
        }
        accumulate(input.subspan(done), output.subspan(done));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::math::Differentiate, [T], [ int16_t, int32_t, float, double, std::complex<float> ])

/**
 * @brief The first difference of the input, `y[i] = x[i] - x[i-1]`.
 *
 * The sample before the first is taken as zero. The first output equals the first input. An integer difference
 * wraps modulo the range of `T`. Every tag passes at its own index with all its keys.
 */
template<typename T>
struct Differentiate : Block<Differentiate<T>> {
    using Description = Doc<"@brief Outputs the difference of each input sample and the one before it, the first sample passing unchanged.">;

    PortIn<T>  in;
    PortOut<T> out;

    GR_MAKE_REFLECTABLE(Differentiate, in, out);

    T _previous{};

    void start() { _previous = T{}; }

    /// writes the first difference of `input` to `output`, continuing from the last sample of the previous call
    void differentiate(std::span<const T> input, std::span<T> output) noexcept {
        T previous = _previous;
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            output[i] = stream::wrappingSubtract(input[i], previous);
            previous  = input[i];
        }
        _previous = previous;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        differentiate(std::span<const T>(inSpan.data(), inSpan.size()), std::span<T>(outSpan.data(), outSpan.size()));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::math::MultiplyByMatrix, [T], [ float, double, std::complex<float> ])

/**
 * @brief `rows` output samples per group of `columns` input samples, the group multiplied by a matrix.
 *
 * `out[r] = sum over c of matrix[r][c] * in[c]` for each group, the groups following one another without overlap.
 * `matrix` is a `rows` x `columns` tensor; a graph file gives its entries as a list, row by row, and sets `rows`.
 * An empty matrix, or one whose extents give no rows and columns, ends the run with an error that names its
 * extents. The chunk sizes are `columns` in and `rows` out. A new matrix applies from the first call whose sample
 * counts are whole groups of it. The samples of a last group shorter than `columns` stay unconsumed at the end of
 * the stream. A tag at input index `i` lands at output index `(i / columns) * rows` with its auto-forward keys, and
 * its `sample_rate` is scaled by `rows / columns`.
 */
template<typename T>
struct MultiplyByMatrix : Block<MultiplyByMatrix<T>, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<"@brief Outputs rows samples for each group of columns input samples, the group multiplied by the matrix.">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Tensor<T>, "matrix", Doc<"rows x columns weights, or their entries row by row">> matrix = stream::unitMatrix<T>();
    Annotated<gr::Size_t, "rows", Doc<"rows of a 1-D matrix, or 0 for a 2-D matrix">>              rows   = 0U;

    GR_MAKE_REFLECTABLE(MultiplyByMatrix, in, out, matrix, rows);

    stream::StagedWeights<T> _weights{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (newSettings.contains("matrix") || newSettings.contains("rows")) {
            _weights.stage(*this, stream::matrixWeights(matrix.value, static_cast<std::size_t>(rows.value)));
        }
    }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        stream::forwardGroupTags(*this, _weights, inputSpans, outputSpans, processedIn);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (const auto refusal = _weights.select(inSpan.size(), outSpan.size()); refusal.has_value()) {
            this->emitErrorMessage("processBulk", *refusal);
            return work::Status::ERROR;
        }
        _weights.inForce.apply(std::span<const T>(inSpan.data(), inSpan.size()), std::span<T>(outSpan.data(), outSpan.size()));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::math::DotProduct, [T], [ float, double, std::complex<float> ])

/**
 * @brief One output sample per group of `n` input samples, the dot product of the group with a vector of `n` entries.
 *
 * `out = sum over c of vector[c] * in[c]`, with no conjugate. An empty vector, or one that is not 1-D, ends the run
 * with an error that names its extents. Groups, chunk sizes, the remainder and tags follow `MultiplyByMatrix` with
 * one row.
 */
template<typename T>
struct DotProduct : Block<DotProduct<T>, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<"@brief Outputs the dot product of each group of n input samples with the vector of n entries.">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Tensor<T>, "vector", Doc<"n weights, one per sample of a group">> vector = gr::Tensor<T>(gr::data_from, {T{1}});

    GR_MAKE_REFLECTABLE(DotProduct, in, out, vector);

    stream::StagedWeights<T> _weights{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (newSettings.contains("vector")) {
            _weights.stage(*this, stream::vectorWeights(vector.value));
        }
    }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        stream::forwardGroupTags(*this, _weights, inputSpans, outputSpans, processedIn);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (const auto refusal = _weights.select(inSpan.size(), outSpan.size()); refusal.has_value()) {
            this->emitErrorMessage("processBulk", *refusal);
            return work::Status::ERROR;
        }
        _weights.inForce.apply(std::span<const T>(inSpan.data(), inSpan.size()), std::span<T>(outSpan.data(), outSpan.size()));
        return work::Status::OK;
    }
};

} // namespace gr::blocks::math

#endif // GNURADIO_MATH_STREAM_HPP
