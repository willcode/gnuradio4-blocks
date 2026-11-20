#ifndef GNURADIO_MATH_HPP
#define GNURADIO_MATH_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/meta/UncertainValue.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::math {

namespace detail {
template<typename T>
T defaultValue() noexcept {
    if constexpr (gr::arithmetic_or_complex_like<T> || gr::UncertainValueLike<T>) {
        return static_cast<T>(1);
    } else if constexpr (std::is_arithmetic_v<T>) {
        return static_cast<T>(1);
    } else {
        return T{};
    }
}

// integral division by zero is undefined (SIGFPE on x86); Divide/DivideConst define it as 0
template<typename T, typename op>
struct SafeOp {
    [[nodiscard]] constexpr T operator()(const T& a, const T& b) const noexcept { return op{}(a, b); }
};

template<std::integral T>
struct SafeOp<T, std::divides<T>> {
    [[nodiscard]] constexpr T operator()(const T& a, const T& b) const noexcept { return b == T{} ? T{} : static_cast<T>(a / b); }
};

// the complex product by components, without the library's recovery of an infinite product from NaN parts
template<gr::meta::complex_like T>
struct SafeOp<T, std::multiplies<T>> {
    [[nodiscard]] constexpr T operator()(const T& a, const T& b) const noexcept { return T(a.real() * b.real() - a.imag() * b.imag(), a.real() * b.imag() + a.imag() * b.real()); }
};

template<typename T, typename op>
inline constexpr bool kMultiplicative = std::same_as<op, std::multiplies<T>> || std::same_as<op, std::divides<T>>;

/// A complex constant factor rotates the carrier by its argument. Its block drops `phase_est`.
template<typename T, typename op>
using ConstOpDroppedKeys = std::conditional_t<kMultiplicative<T, op> && gr::meta::complex_like<T>, DroppedTagKeys<"phase_est">, DroppedTagKeys<>>;

/// A product or a quotient of two streams mixes their carriers. Its block drops `freq_est` and `phase_est`.
template<typename T, typename op>
using MultiPortOpDroppedKeys = std::conditional_t<kMultiplicative<T, op>, DroppedTagKeys<"freq_est", "phase_est">, DroppedTagKeys<>>;
} // namespace detail

GR_REGISTER_BLOCK("gr::blocks::math::AddConst", gr::blocks::math::AddConst, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::math::SubtractConst", gr::blocks::math::SubtractConst, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::math::MultiplyConst", gr::blocks::math::MultiplyConst, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::math::DivideConst", gr::blocks::math::DivideConst, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T, typename op>
struct MathOpImpl : Block<MathOpImpl<T, op>, detail::ConstOpDroppedKeys<T, op>> {
    PortIn<T>  in{};
    PortOut<T> out{};

    struct Settings {
        T value = detail::defaultValue<T>();

        GR_MAKE_REFLECTABLE(Settings, value);
    } settings;

    GR_MAKE_REFLECTABLE(MathOpImpl, in, out, settings);

    template<gr::meta::t_or_simd<T> V>
    [[nodiscard]] constexpr V processOne(const V& a) const noexcept {
        if constexpr (std::same_as<op, std::divides<T>> && std::integral<T>) {
            if (settings.value == T{}) {
                return V{};
            }
        }
        if constexpr (gr::meta::any_simd<V, T>) {
            if constexpr (std::same_as<op, std::multiplies<T>>) {
                return a * settings.value;
            } else if constexpr (std::same_as<op, std::divides<T>>) {
                return a / settings.value;
            } else if constexpr (std::same_as<op, std::plus<T>>) {
                return a + settings.value;
            } else if constexpr (std::same_as<op, std::minus<T>>) {
                return a - settings.value;
            } else {
                static_assert(gr::meta::always_false<T>, "unknown op");
                return V{};
            }
        } else { // non-simd branch
            return detail::SafeOp<T, op>{}(a, settings.value);
        }
    }
};

template<typename T>
using AddConst = MathOpImpl<T, std::plus<T>>;
template<typename T>
using SubtractConst = MathOpImpl<T, std::minus<T>>;
template<typename T>
using MultiplyConst = MathOpImpl<T, std::multiplies<T>>;
template<typename T>
using DivideConst = MathOpImpl<T, std::divides<T>>;

GR_REGISTER_BLOCK("gr::blocks::math::Add", gr::blocks::math::Add, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::math::Subtract", gr::blocks::math::Subtract, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::math::Multiply", gr::blocks::math::Multiply, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::math::Divide", gr::blocks::math::Divide, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T, typename op>
requires gr::arithmetic_or_complex_like<gr::meta::fundamental_base_value_type_t<T>>
struct MathOpMultiPortImpl : Block<MathOpMultiPortImpl<T, op>, detail::MultiPortOpDroppedKeys<T, op>> {
    using Description = Doc<R""(@brief Math block combining multiple inputs into a single output with a given operation

    Depending on the operator op this block computes:
    - Multiply: out = in_1 * in_2 * in_3 * ...
    - Divide: out = in_1 / in_2 / in_3 / ...
    - Add: out = in_1 + in_2 + in_3 + ...
    - Subtract: out = in_1 - in_2 - in_3 - ...

    For integral types, division by a zero divisor yields 0. A complex product is taken by components, without C's
    recovery of an infinite product from NaN parts.
    )"">;

    // ports
    // The input vector starts at the default `n_inputs`. `applySettings` resizes it only when the value changes.
    std::vector<PortIn<T>> in = std::vector<PortIn<T>>(1UZ);
    PortOut<T>             out;

    // settings
    struct Settings {
        Annotated<gr::Size_t, Visible, Doc<"input count">, Limits<Min<1U>, Max<32U>>, PortCountFor<"in">> n_inputs = 1U;

        GR_MAKE_REFLECTABLE(Settings, n_inputs);
    } settings;

    GR_MAKE_REFLECTABLE(MathOpMultiPortImpl, in, out, settings);

    void applySettings(const Settings& s) { in.resize(s.n_inputs); }

    template<gr::InputSpanLike TInSpan>
    gr::work::Status processBulk(const std::span<TInSpan>& ins, gr::OutputSpanLike auto& sout) const {
        if (ins.size() == 1UZ) {
            std::copy(ins[0].begin(), ins[0].end(), sout.begin());
            return gr::work::Status::OK;
        }
        std::transform(ins[0].begin(), ins[0].end(), ins[1].begin(), sout.begin(), detail::SafeOp<T, op>{});
        for (std::size_t n = 2; n < ins.size(); n++) {
            std::transform(sout.begin(), sout.end(), ins[n].begin(), sout.begin(), detail::SafeOp<T, op>{});
        }
        return gr::work::Status::OK;
    }
};

template<typename T>
using Add = MathOpMultiPortImpl<T, std::plus<T>>;
template<typename T>
using Subtract = MathOpMultiPortImpl<T, std::minus<T>>;
template<typename T>
using Multiply = MathOpMultiPortImpl<T, std::multiplies<T>>;
template<typename T>
using Divide = MathOpMultiPortImpl<T, std::divides<T>>;

} // namespace gr::blocks::math

#endif // GNURADIO_MATH_HPP
