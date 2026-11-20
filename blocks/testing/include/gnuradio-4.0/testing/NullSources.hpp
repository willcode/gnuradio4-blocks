#ifndef GNURADIO_NULLSOURCES_HPP
#define GNURADIO_NULLSOURCES_HPP

#include <algorithm>
#include <cstdint>
#include <limits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK(gr::blocks::testing::SlowSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct SlowSource : Block<SlowSource<T>> {
    using value_t     = std::conditional_t<std::is_same_v<T, std::string>, std::string, meta::fundamental_base_value_type_t<T>>; // use base-type for types not supported by settings
    using Description = Doc<R""(A source block that emits a constant default value every n miliseconds)"">;

    gr::PortOut<T> out;

    struct Settings {
        Annotated<value_t, Visible, Doc<"sample value">>               default_value{};
        Annotated<gr::Size_t, Unit<"ms">, Doc<"delay between values">> delay = 100U;

        GR_MAKE_REFLECTABLE(Settings, default_value, delay);
    } settings;

    GR_MAKE_REFLECTABLE(SlowSource, out, settings);

    std::optional<std::chrono::time_point<std::chrono::system_clock>> lastEventAt;

    [[nodiscard]] gr::work::Status processBulk(OutputSpanLike auto& output) {
        if (!output.empty() && (!lastEventAt || std::chrono::system_clock::now() - *lastEventAt > std::chrono::milliseconds(settings.delay))) {
            lastEventAt = std::chrono::system_clock::now();

            output[0] = T(settings.default_value.value);
            output.publish(1UZ);
        } else {
            output.publish(0UZ);
        }

        return gr::work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::CountingSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct CountingSource : Block<CountingSource<T>> {
    using Description = Doc<R""(A source block that emits an increasing sequence starting from a specified default value.
This block counts the number of samples emitted and optionally halts after reaching a specified maximum.
Commonly used for testing and simulations where consistent output and finite execution are required.
The block reports `count` once per work call that emits samples.)"">;

    gr::PortOut<T> out;

    struct Settings {
        Annotated<T, Visible, Doc<"sample value">>                    default_value{};
        Annotated<std::uint64_t, Doc<"sample limit (0 = unlimited)">> n_samples_max = 0UZ;

        GR_MAKE_REFLECTABLE(Settings, default_value, n_samples_max);
    } settings;

    struct Status {
        Annotated<std::uint64_t, Doc<"sample count">> count = 0UZ;

        GR_MAKE_REFLECTABLE(Status, count);
    } status;

    GR_MAKE_REFLECTABLE(CountingSource, out, settings, status);

    void reset() { status.count = 0U; }

    // `count` is assigned once per work call
    [[nodiscard]] gr::work::Status processBulk(OutputSpanLike auto& output) {
        const std::uint64_t nMax      = settings.n_samples_max.value;
        const std::uint64_t count     = status.count.value;
        const bool          limited   = nMax > 0U;
        const std::uint64_t remaining = !limited ? std::numeric_limits<std::uint64_t>::max() : (count < nMax ? nMax - count : 1U); // past the limit, one more sample before the stop
        const std::size_t   nSamples  = static_cast<std::size_t>(std::min<std::uint64_t>(output.size(), remaining));
        const T             base      = static_cast<T>(settings.default_value.value);
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
            output[i] = base + static_cast<T>(count + 1U + i);
#pragma GCC diagnostic pop
        }
        output.publish(nSamples);
        if (nSamples == 0UZ) {
            return gr::work::Status::OK;
        }
        status.count = count + nSamples;
        return limited && status.count.value >= nMax ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::Copy, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct Copy : Block<Copy<T>> {
    using Description = Doc<R""(A block that passes/copies input samples directly to its output without modification.
Commonly used used to isolate parts of a flowgraph, manage buffer sizes, or simply duplicate the signal path.)"">;

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(Copy, in, out);

    template<gr::meta::t_or_simd<T> V>
    [[nodiscard]] constexpr auto processOne(V input) const noexcept {
        return input;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::CountingSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct CountingSink : Block<CountingSink<T>> {
    using Description = Doc<R""(A sink block that consumes and discards a fixed number of input samples, after which it signals the flow graph to halt.
This block is used to control execution in systems requiring precise input processing without data output, similar to how a 'HeadBlock' manages output samples.
Commonly used for testing scenarios and signal termination where output is unnecessary but precise input count control is needed.)"">;

    gr::PortIn<T> in;

    struct Settings {
        Annotated<std::uint64_t, Doc<"sample limit (0 = unlimited)">> n_samples_max = 0UZ;

        GR_MAKE_REFLECTABLE(Settings, n_samples_max);
    } settings;

    struct Status {
        Annotated<std::uint64_t, Doc<"sample count">> count = 0UZ;

        GR_MAKE_REFLECTABLE(Status, count);
    } status;

    GR_MAKE_REFLECTABLE(CountingSink, in, settings, status);

    void reset() { status.count = 0U; }

    [[nodiscard]] gr::work::Status processBulk(InputSpanLike auto& input) {
        const std::uint64_t nAvailable = input.size();
        const std::uint64_t nMax       = settings.n_samples_max.value;
        if (nMax > 0 && status.count.value + nAvailable >= nMax) {
            const std::uint64_t nRemaining = nMax - status.count.value; // at most nAvailable
            std::ignore                    = input.consume(static_cast<std::size_t>(nRemaining));
            status.count                   = nMax;
            return gr::work::Status::DONE;
        }
        if (nAvailable > 0U) {
            status.count += nAvailable;
        }
        return gr::work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::testing::SimCompute, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float>, gr::Packet<float>, gr::Tensor<float>, gr::DataSet<float> ])

template<typename T>
struct SimCompute : Block<SimCompute<T>> {
    using Description = Doc<R""(Block to simulate compute delay and model latency effects for a given complexity order

Delay depends on work size: delay_seconds = (N / reference_work_size)^complexity_order * (reference_work_size / target_throughput)

Default: linear O(N) delay that limits the block processing to 100 MS/s)"">;

    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    struct Settings {
        Annotated<float, Unit<"S/s">, Doc<"target throughput">>        target_throughput   = 100e6f;
        Annotated<gr::Size_t, Unit<"[N]">, Doc<"reference work size">> reference_work_size = 1024U;
        Annotated<float, Doc<"complexity order (1 = linear)">>         complexity_order    = 1.0f;
        Annotated<bool, Doc<"busy wait">>                              busy_wait           = true;

        GR_MAKE_REFLECTABLE(Settings, target_throughput, reference_work_size, complexity_order, busy_wait);
    } settings;

    GR_MAKE_REFLECTABLE(SimCompute, in, out, settings);

    [[nodiscard]] gr::work::Status processBulk(InputSpanLike auto& input, OutputSpanLike auto& output) {
        using Clock_t    = std::chrono::high_resolution_clock;
        const auto start = Clock_t::now();

        std::ranges::copy(input, output.begin());

        const std::chrono::duration<double> delay_seconds = compute_delay_seconds(input.size());
        const std::chrono::nanoseconds      delay_ns      = std::chrono::duration_cast<std::chrono::nanoseconds>(delay_seconds);
        while (Clock_t::now() - start < delay_ns) {
            if (!settings.busy_wait) {
                std::this_thread::sleep_for(std::chrono::microseconds(10)); // avoid hard spin
            }
        }
        return gr::work::Status::OK;
    }

    [[nodiscard]] constexpr std::chrono::duration<double> compute_delay_seconds(std::size_t N) const noexcept {
        const auto N_r        = static_cast<double>(settings.reference_work_size);
        const auto throughput = static_cast<double>(settings.target_throughput);
        const auto k          = static_cast<double>(settings.complexity_order);

        const double delay_s = std::pow(static_cast<double>(N) / N_r, k) * (N_r / throughput);

        return std::chrono::duration<double>(delay_s);
    }
};

} // namespace gr::blocks::testing

#endif // GNURADIO_NULLSOURCES_HPP
