#ifndef GNURADIO_MATH_REDUCE_HPP
#define GNURADIO_MATH_REDUCE_HPP

#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/math/Binary.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::math {

namespace reduce {

/*
 * An operation is a type with:
 * - `template<typename T> using result`, the output type for the input type `T`;
 * - `kSimd`, true when `apply` folds a group of an arithmetic type in `vir::stdx::simd` vectors wherever it
 *   accumulates in the sample type;
 * - `static apply(group, block)`, the function of one group of samples, a `std::span` of at least one sample;
 * - `Description`, the documentation of the blocks it makes.
 */

/// folds `group` from its first sample with `combine`, a function of two samples or of two simd vectors. Where
/// `kVector` holds for an arithmetic `T` and the group fills two vectors, the whole vectors are folded lane by lane
/// first, then the lanes in order, then the samples past the last whole vector.
template<bool kVector, typename T, typename F>
[[nodiscard]] constexpr T fold(std::span<const T> group, F combine) noexcept {
    std::size_t i   = 1UZ;
    T           acc = group.front();
    if constexpr (kVector && std::is_arithmetic_v<T>) {
        using V = vir::stdx::native_simd<T>;
        if (group.size() >= 2UZ * V::size()) {
            V lanes(group.data(), vir::stdx::element_aligned);
            for (i = V::size(); i + V::size() <= group.size(); i += V::size()) {
                lanes = combine(lanes, V(group.data() + i, vir::stdx::element_aligned));
            }
            acc = lanes[0];
            for (std::size_t lane = 1UZ; lane < V::size(); ++lane) {
                acc = combine(acc, static_cast<T>(lanes[lane]));
            }
        }
    }
    for (; i < group.size(); ++i) {
        acc = combine(acc, group[i]);
    }
    return acc;
}

/// the sum of a group: an integer group summed in `std::int64_t`, a real or complex group in its own type
template<typename T>
[[nodiscard]] constexpr auto wideSum(std::span<const T> group) noexcept {
    if constexpr (std::integral<T>) {
        std::int64_t sum = 0;
        for (const T x : group) {
            sum += x;
        }
        return sum;
    } else {
        return fold<true>(group, [](const auto& a, const auto& b) { return a + b; });
    }
}

/// the sum of the group; an integer sum is computed in 64 bits and narrowed to the sample type, modulo its range
struct Sum {
    using Description = Doc<"@brief Outputs the sum of each group of n input samples, an integer sum computed in 64 bits and narrowed to the sample type.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename T, typename B>
    [[nodiscard]] static constexpr T apply(std::span<const T> group, const B& /*block*/) noexcept {
        return static_cast<T>(wideSum(group));
    }
};

/// the mean of the group; an integer mean is the 64-bit sum divided by n, truncated toward zero
struct Mean {
    using Description = Doc<"@brief Outputs the mean of each group of n input samples, an integer mean truncated toward zero.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename T, typename B>
    [[nodiscard]] static constexpr T apply(std::span<const T> group, const B& /*block*/) noexcept {
        if constexpr (std::integral<T>) {
            return static_cast<T>(wideSum(group) / static_cast<std::int64_t>(group.size()));
        } else {
            return wideSum(group) / static_cast<meta::fundamental_base_value_type_t<T>>(group.size());
        }
    }
};

/// the largest sample of the group; a NaN is kept or passed over as `binary::Max` keeps or passes it, lane by lane
struct Max {
    using Description = Doc<"@brief Outputs the largest sample of each group of n input samples.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename T, typename B>
    [[nodiscard]] static constexpr T apply(std::span<const T> group, const B& block) noexcept {
        return fold<kSimd>(group, [&block](const auto& a, const auto& b) { return binary::Max::apply(a, b, block); });
    }
};

/// the smallest sample of the group; a NaN is kept or passed over as `binary::Min` keeps or passes it, lane by lane
struct Min {
    using Description = Doc<"@brief Outputs the smallest sample of each group of n input samples.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename T, typename B>
    [[nodiscard]] static constexpr T apply(std::span<const T> group, const B& block) noexcept {
        return fold<kSimd>(group, [&block](const auto& a, const auto& b) { return binary::Min::apply(a, b, block); });
    }
};

/// the index within the group of its largest sample, the first of equal ones; a NaN after the first sample is passed over
struct Argmax {
    using Description = Doc<"@brief Outputs the index within each group of n input samples of its largest sample, the first of equal ones.">;
    template<typename T>
    using result                = std::uint32_t;
    static constexpr bool kSimd = false;
    template<typename T, typename B>
    [[nodiscard]] static constexpr std::uint32_t apply(std::span<const T> group, const B& /*block*/) noexcept {
        std::size_t best = 0UZ;
        for (std::size_t i = 1UZ; i < group.size(); ++i) {
            if (group[best] < group[i]) {
                best = i;
            }
        }
        return static_cast<std::uint32_t>(best);
    }
};

/// the index within the group of its smallest sample, the first of equal ones; a NaN after the first sample is passed over
struct Argmin {
    using Description = Doc<"@brief Outputs the index within each group of n input samples of its smallest sample, the first of equal ones.">;
    template<typename T>
    using result                = std::uint32_t;
    static constexpr bool kSimd = false;
    template<typename T, typename B>
    [[nodiscard]] static constexpr std::uint32_t apply(std::span<const T> group, const B& /*block*/) noexcept {
        std::size_t best = 0UZ;
        for (std::size_t i = 1UZ; i < group.size(); ++i) {
            if (group[i] < group[best]) {
                best = i;
            }
        }
        return static_cast<std::uint32_t>(best);
    }
};

/// the mean of the squared magnitudes of the group, accumulated in the real type of the sample
template<typename T>
[[nodiscard]] constexpr auto meanSquare(std::span<const T> group) noexcept {
    using R = meta::fundamental_base_value_type_t<T>;
    R sum{};
    for (const T& x : group) {
        if constexpr (meta::complex_like<T>) {
            sum += x.real() * x.real() + x.imag() * x.imag();
        } else {
            sum += x * x;
        }
    }
    return sum / static_cast<R>(group.size());
}

struct MeanSquare {
    using Description = Doc<"@brief Outputs the mean of the squared magnitudes of each group of n input samples.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = false;
    template<typename T, typename B>
    [[nodiscard]] static constexpr auto apply(std::span<const T> group, const B& /*block*/) noexcept {
        return meanSquare(group);
    }
};

struct Rms {
    using Description = Doc<"@brief Outputs the root mean square magnitude of each group of n input samples.">;
    template<typename T>
    using result                = meta::fundamental_base_value_type_t<T>;
    static constexpr bool kSimd = false;
    template<typename T, typename B>
    [[nodiscard]] static auto apply(std::span<const T> group, const B& /*block*/) noexcept {
        return std::sqrt(meanSquare(group));
    }
};

} // namespace reduce

/**
 * @brief One output sample per group of `n` input samples: `out = op(group)`.
 *
 * The groups follow one another without overlap. The operation `op` states the output type and the documentation.
 * The chunk sizes are `n` in and 1 out, so `processBulk` sees whole groups. A change to `n` sets the chunk sizes, and
 * the new size applies from the next chunk: the group size of a chunk is its input count over its output count. The
 * samples of a last group shorter than `n` stay unconsumed at the end of the stream.
 *
 * A tag at input index `i` lands at output index `i / n`, and a tag on the unconsumed samples at the end-of-stream
 * index. The tag keeps the auto-forward keys, and its `sample_rate` is divided by `n`.
 */
template<typename op, typename T>
struct Reduce : Block<Reduce<op, T>, Resampling<1UZ, 1UZ, false>> {
    using R           = typename op::template result<T>;
    using Description = typename op::Description;

    PortIn<T>  in;
    PortOut<R> out;

    Annotated<gr::Size_t, "n", Doc<"input samples per output sample, at least 1">> n = 1U;

    GR_MAKE_REFLECTABLE(Reduce, in, out, n);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (n < 1U) {
            throw gr::exception(std::format("n must be at least 1, got {}", n.value));
        }
        this->input_chunk_size  = n.value;
        this->output_chunk_size = 1U;
    }

    /// reduces `input` to `output`, each output sample from a group of `input.size() / output.size()` samples
    void reduceGroups(std::span<const T> input, std::span<R> output) const {
        if (output.empty()) {
            return;
        }
        const std::size_t group = input.size() / output.size();
        if (group == 0UZ || group * output.size() != input.size()) {
            throw gr::exception(std::format("{} input samples are not whole groups of the {} outputs", input.size(), output.size()));
        }
        for (std::size_t k = 0UZ; k < output.size(); ++k) {
            output[k] = op::apply(input.subspan(k * group, group), *this);
        }
    }

    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        std::size_t outputCount = 0UZ;
        gr::for_each_writer_span([&outputCount](auto& span) { outputCount = span.size(); }, outputSpans);
        // a call with no input is the end of the stream, and its output offset 0 is the end-of-stream index
        const std::size_t group = processedIn == 0UZ || outputCount == 0UZ ? 0UZ : processedIn / outputCount;

        const std::pmr::string      endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        std::optional<property_map> cachedSettings;
        const auto                  place = [&](std::ptrdiff_t relIndex, const property_map& tagMap) {
            if (relIndex < 0) { // a tag this block placed in an earlier call
                return;
            }
            property_map forwarded = this->filterAndSubstituteTag(tagMap, cachedSettings);
            forwarded.erase(endOfStream);
            if (forwarded.empty()) {
                return;
            }
            const std::size_t at = group == 0UZ ? 0UZ : static_cast<std::size_t>(relIndex) / group;
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

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        reduceGroups(std::span<const T>(inSpan.data(), inSpan.size()), std::span<R>(outSpan.data(), outSpan.size()));
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK("gr::blocks::math::Sum", gr::blocks::math::Sum, [T], [ int16_t, int32_t, float, double, std::complex<float> ])
template<typename T>
using Sum = Reduce<reduce::Sum, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Mean", gr::blocks::math::Mean, [T], [ int16_t, int32_t, float, double, std::complex<float> ])
template<typename T>
using Mean = Reduce<reduce::Mean, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Max", gr::blocks::math::Max, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Max = Reduce<reduce::Max, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Min", gr::blocks::math::Min, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Min = Reduce<reduce::Min, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Argmax", gr::blocks::math::Argmax, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Argmax = Reduce<reduce::Argmax, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Argmin", gr::blocks::math::Argmin, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Argmin = Reduce<reduce::Argmin, T>;

GR_REGISTER_BLOCK("gr::blocks::math::MeanSquare", gr::blocks::math::MeanSquare, [T], [ float, double, std::complex<float> ])
template<typename T>
using MeanSquare = Reduce<reduce::MeanSquare, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Rms", gr::blocks::math::Rms, [T], [ float, double, std::complex<float> ])
template<typename T>
using Rms = Reduce<reduce::Rms, T>;

} // namespace gr::blocks::math

#endif // GNURADIO_MATH_REDUCE_HPP
