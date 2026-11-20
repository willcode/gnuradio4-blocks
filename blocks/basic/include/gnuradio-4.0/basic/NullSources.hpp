#ifndef GNURADIO_BASIC_NULL_SOURCES_HPP
#define GNURADIO_BASIC_NULL_SOURCES_HPP

#include <algorithm>
#include <cstdint>
#include <limits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::NullSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct NullSource : Block<NullSource<T>> {
    using Description = Doc<R""(@brief Emits the value-initialized `T{}` on every output sample.

The block emits samples until the graph stops it.)"">;

    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(NullSource, out);

    [[nodiscard]] constexpr T processOne() const noexcept { return T{}; }

    [[nodiscard]] constexpr auto processOne(meta::constexpr_value auto N) const noexcept
    requires std::is_arithmetic_v<T>
    {
        return meta::simdize<T, N>{};
    }
};

static_assert(gr::BlockLike<NullSource<float>>);

namespace detail {
// the samples a block that stops at `limit` may still handle after `handled`; past the limit, one more before the stop
[[nodiscard]] constexpr std::size_t samplesBeforeLimit(std::uint64_t limit, std::uint64_t handled) noexcept {
    if (limit == 0U) {
        return std::numeric_limits<std::size_t>::max();
    }
    const std::uint64_t remaining = handled < limit ? limit - handled : 1U;
    return static_cast<std::size_t>(std::min<std::uint64_t>(remaining, std::numeric_limits<std::size_t>::max()));
}
} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::ConstantSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct ConstantSource : Block<ConstantSource<T>> {
    using value_t     = std::conditional_t<std::is_same_v<T, std::string>, std::string, meta::fundamental_base_value_type_t<T>>; // use base-type for types not supported by settings
    using Description = Doc<R""(@brief Emits `default_value` on every output sample.

The block counts the samples it emits in `count`. When `count` reaches a non-zero `n_samples_max`, the block emits that
sample and stops. With `n_samples_max` at zero, the block runs until the graph stops. The block reports `count` as a status
value once per work call that emits samples, and a get reads it.)"">;

    gr::PortOut<T> out;

    struct Settings {
        Annotated<value_t, Visible, Doc<"sample value">>              default_value{};
        Annotated<std::uint64_t, Doc<"sample limit (0 = unlimited)">> n_samples_max = 0UZ;

        GR_MAKE_REFLECTABLE(Settings, default_value, n_samples_max);
    } settings;

    struct Status {
        Annotated<std::uint64_t, Doc<"sample count">> count = 0UZ;

        GR_MAKE_REFLECTABLE(Status, count);
    } status;

    GR_MAKE_REFLECTABLE(ConstantSource, out, settings, status);

    void reset() { status.count = 0U; }

    // counts the call's samples and reports `count` once
    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& output) {
        const std::size_t nSamples = std::min(output.size(), detail::samplesBeforeLimit(settings.n_samples_max.value, status.count.value));
        std::fill_n(output.begin(), nSamples, T(settings.default_value.value));
        output.publish(nSamples);
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }
        status.count.value += nSamples;
        return settings.n_samples_max.value > 0U && status.count.value >= settings.n_samples_max.value ? work::Status::DONE : work::Status::OK;
    }
};

static_assert(gr::BlockLike<ConstantSource<float>>);

GR_REGISTER_BLOCK(gr::blocks::basic::HeadBlock, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct HeadBlock : Block<HeadBlock<T>> {
    using Description = Doc<R""(@brief Copies input samples to the output and stops after `n_samples_max` samples.

`count` holds the samples copied, and the block reports it as a status value once per work call that copies samples. With
`n_samples_max` at zero, the block copies every sample.)"">;

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    struct Settings {
        Annotated<std::uint64_t, Doc<"sample limit (0 = unlimited)">> n_samples_max = 0UZ;

        GR_MAKE_REFLECTABLE(Settings, n_samples_max);
    } settings;

    struct Status {
        Annotated<std::uint64_t, Doc<"sample count">> count = 0UZ;

        GR_MAKE_REFLECTABLE(Status, count);
    } status;

    GR_MAKE_REFLECTABLE(HeadBlock, in, out, settings, status);

    void reset() { status.count = 0U; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& input, OutputSpanLike auto& output) {
        const std::size_t nSamples = std::min({input.size(), output.size(), detail::samplesBeforeLimit(settings.n_samples_max.value, status.count.value)});
        std::copy_n(input.begin(), nSamples, output.begin());
        std::ignore = input.consume(nSamples);
        output.publish(nSamples);
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }
        status.count.value += nSamples;
        return settings.n_samples_max.value > 0U && status.count.value >= settings.n_samples_max.value ? work::Status::DONE : work::Status::OK;
    }
};

static_assert(gr::BlockLike<HeadBlock<float>>);

GR_REGISTER_BLOCK(gr::blocks::basic::NullSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct NullSink : Block<NullSink<T>> {
    using Description = Doc<R""(@brief Consumes every input sample and discards it.)"">;

    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(NullSink, in);

    template<gr::meta::t_or_simd<T> V>
    void processOne(V) const noexcept { /* do nothing */ }
};

static_assert(gr::BlockLike<NullSink<float>>);

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_NULL_SOURCES_HPP
