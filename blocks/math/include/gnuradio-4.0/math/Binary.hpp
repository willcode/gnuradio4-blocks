#ifndef GNURADIO_MATH_BINARY_HPP
#define GNURADIO_MATH_BINARY_HPP

#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::math {

namespace binary {

/*
 * An operation is a type with:
 * - `template<typename T> using result`, the output type for the input type `T`;
 * - `kSimd`, true when `apply` takes two `vir::stdx::simd` vectors of an arithmetic input type;
 * - `static apply(a, b, block)`, the function of one pair of samples or of one pair of simd vectors;
 * - `Description`, the documentation of the blocks it makes.
 */

/// a times the complex conjugate of b, computed by components
struct MultiplyConjugate {
    using Description = Doc<"@brief Multiplies each sample of a by the complex conjugate of the sample of b.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        return V(a.real() * b.real() + a.imag() * b.imag(), a.imag() * b.real() - a.real() * b.imag());
    }
};

/// the larger of a and b; a when either is NaN
struct Max {
    using Description = Doc<"@brief Outputs the larger of the two input samples. A NaN in either input gives the sample of a.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        if constexpr (meta::any_simd<V>) {
            V larger             = a;
            where(a < b, larger) = b;
            return larger;
        } else {
            return a < b ? b : a;
        }
    }
};

/// the smaller of a and b; a when either is NaN
struct Min {
    using Description = Doc<"@brief Outputs the smaller of the two input samples. A NaN in either input gives the sample of a.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        if constexpr (meta::any_simd<V>) {
            V smaller             = a;
            where(b < a, smaller) = b;
            return smaller;
        } else {
            return b < a ? b : a;
        }
    }
};

/// the remainder of a / b with the sign of a: `%` for an integer type, `fmod` for a real one; 0 when b is 0. A signed
/// b of -1 gives its exact remainder 0 without the division, which overflows for the lowest value of the type.
struct Modulo {
    using Description = Doc<"@brief Outputs the remainder of a divided by b with the sign of a, and 0 where b is 0.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        if constexpr (std::integral<V>) {
            return b == V{} || (std::is_signed_v<V> && b == V(-1)) ? V{} : static_cast<V>(a % b);
        } else {
            return b == V{} ? V{} : std::fmod(a, b);
        }
    }
};

/// a to the power b, by the C library's `pow`; a negative a with a fractional b gives NaN
struct Pow {
    using Description = Doc<"@brief Raises each sample of a to the power of the sample of b.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        return std::pow(a, b);
    }
};

/// the angle of the point (b, a), by the C library's `atan2`: a is the ordinate and b the abscissa
struct Atan2 {
    using Description = Doc<"@brief Outputs the angle of the point (b, a) in radians, in [-pi, pi].">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        return std::atan2(a, b);
    }
};

/// sqrt(a * a + b * b), by the C library's `hypot`, without overflow in the squares
struct Hypot {
    using Description = Doc<"@brief Outputs the square root of a squared plus b squared.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = false;
    template<typename V, typename B>
    [[nodiscard]] static V apply(const V& a, const V& b, const B& /*block*/) noexcept {
        return std::hypot(a, b);
    }
};

} // namespace binary

/**
 * @brief Two inputs, one output: `out = op(a, b)`.
 *
 * The block is synchronous: each output sample takes one sample of `a` and one of `b`. The operation `op` states
 * the output type and the documentation. An operation with a simd form over an arithmetic input of the output type
 * makes `processOne` take `vir::stdx::simd` vectors; the others take one pair of samples.
 */
template<typename op, typename T>
struct Binary : Block<Binary<op, T>> {
    using R           = typename op::template result<T>;
    using Description = typename op::Description;

    PortIn<T>  a;
    PortIn<T>  b;
    PortOut<R> out;

    GR_MAKE_REFLECTABLE(Binary, a, b, out);

    static constexpr bool kSimdPath = op::kSimd && std::is_arithmetic_v<T> && std::same_as<T, R>;

    template<gr::meta::t_or_simd<T> V>
    requires(kSimdPath)
    [[nodiscard]] constexpr V processOne(const V& x, const V& y) const noexcept {
        return op::apply(x, y, *this);
    }

    [[nodiscard]] constexpr R processOne(const T& x, const T& y) const noexcept
    requires(!kSimdPath)
    {
        return op::apply(x, y, *this);
    }
};

GR_REGISTER_BLOCK("gr::blocks::math::MultiplyConjugate", gr::blocks::math::MultiplyConjugate, [T], [std::complex<float>])
template<typename T>
using MultiplyConjugate = Binary<binary::MultiplyConjugate, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Max", gr::blocks::math::Max, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Max = Binary<binary::Max, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Min", gr::blocks::math::Min, [T], [ uint8_t, int16_t, int32_t, float, double ])
template<typename T>
using Min = Binary<binary::Min, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Pow", gr::blocks::math::Pow, [T], [ float, double ])
template<typename T>
using Pow = Binary<binary::Pow, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Atan2", gr::blocks::math::Atan2, [T], [ float, double ])
template<typename T>
using Atan2 = Binary<binary::Atan2, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Hypot", gr::blocks::math::Hypot, [T], [ float, double ])
template<typename T>
using Hypot = Binary<binary::Hypot, T>;

GR_REGISTER_BLOCK("gr::blocks::math::Modulo", gr::blocks::math::Modulo, [T], [ int16_t, int32_t, float, double ])
template<typename T>
using Modulo = Binary<binary::Modulo, T>;

} // namespace gr::blocks::math

#endif // GNURADIO_MATH_BINARY_HPP
