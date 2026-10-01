#ifndef GNURADIO_MATH_BITS_HPP
#define GNURADIO_MATH_BITS_HPP

#include <cstdint>
#include <functional>
#include <limits>
#include <type_traits>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/math/Math.hpp>
#include <gnuradio-4.0/math/Unary.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::math {

namespace detail {
template<typename T>
struct MultiPortDescription<std::bit_and<T>> {
    using type = Doc<"@brief Outputs the bitwise AND of the samples of all inputs.">;
};

template<typename T>
struct MultiPortDescription<std::bit_or<T>> {
    using type = Doc<"@brief Outputs the bitwise OR of the samples of all inputs.">;
};

template<typename T>
struct MultiPortDescription<std::bit_xor<T>> {
    using type = Doc<"@brief Outputs the bitwise exclusive OR of the samples of all inputs.">;
};
} // namespace detail

namespace bits {

/// the ports of a `Unary` bit operation and its operand `constant`
template<typename Derived, typename T, typename R>
struct ConstantPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>                                                       in;
    PortOut<R>                                                      out;
    Annotated<T, "constant", Doc<"operand applied to each sample">> constant = T{};

    GR_MAKE_REFLECTABLE(ConstantPorts, in, out, constant);
};

/// the ports of a `Unary` shift and its bit count `count`
template<typename Derived, typename T, typename R>
struct ShiftPorts : Block<Derived> {
    using Block<Derived>::Block;

    PortIn<T>                                                          in;
    PortOut<R>                                                         out;
    Annotated<std::uint8_t, "count", Doc<"bit positions to shift by">> count = 1U;

    GR_MAKE_REFLECTABLE(ShiftPorts, in, out, count);
};

/// the number of bits of the integer type of `V`, a sample or a simd vector
template<typename V>
inline constexpr int kWidth = std::numeric_limits<std::make_unsigned_t<meta::fundamental_base_value_type_t<V>>>::digits;

// The operations follow the contract of `unary`: each keeps the input type, and `apply` takes one sample or one simd
// vector. The integer promotion of `~`, `&`, `|`, `^`, `<<` and `>>` is narrowed back to the input type.

struct Not {
    using Description = Doc<"@brief Outputs the bitwise complement of each sample.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& /*block*/) noexcept {
        return unary::narrow<V>(~x);
    }
};

struct AndConst {
    using Description = Doc<"@brief Outputs the bitwise AND of each sample and constant.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename Derived, typename T, typename R>
    using ports = ConstantPorts<Derived, T, R>;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& block) noexcept {
        return unary::narrow<V>(x & V(block.constant.value));
    }
};

struct OrConst {
    using Description = Doc<"@brief Outputs the bitwise OR of each sample and constant.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename Derived, typename T, typename R>
    using ports = ConstantPorts<Derived, T, R>;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& block) noexcept {
        return unary::narrow<V>(x | V(block.constant.value));
    }
};

struct XorConst {
    using Description = Doc<"@brief Outputs the bitwise exclusive OR of each sample and constant.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename Derived, typename T, typename R>
    using ports = ConstantPorts<Derived, T, R>;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& block) noexcept {
        return unary::narrow<V>(x ^ V(block.constant.value));
    }
};

/// x shifted left by `count` bits, the bits past the top of the type dropped; 0 for a count at or past the width
struct ShiftLeft {
    using Description = Doc<"@brief Shifts each sample left by count bits and outputs 0 for a count at or past the bit width of the type.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename Derived, typename T, typename R>
    using ports = ShiftPorts<Derived, T, R>;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& block) noexcept {
        const int count = block.count.value;
        return count >= kWidth<V> ? V(0) : unary::narrow<V>(x << count);
    }
};

/// x shifted right by `count` bits, the sign bit copied in for a signed type; 0 for a count at or past the width
struct ShiftRight {
    using Description = Doc<"@brief Shifts each sample right by count bits, copying the sign bit of a signed type, and outputs 0 for a count at or past the bit width of the type.">;
    template<typename T>
    using result                = T;
    static constexpr bool kSimd = true;
    template<typename Derived, typename T, typename R>
    using ports = ShiftPorts<Derived, T, R>;
    template<typename V, typename B>
    [[nodiscard]] static constexpr V apply(const V& x, const B& block) noexcept {
        const int count = block.count.value;
        return count >= kWidth<V> ? V(0) : unary::narrow<V>(x >> count);
    }
};

} // namespace bits

GR_REGISTER_BLOCK("gr::blocks::math::And", gr::blocks::math::And, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using And = MathOpMultiPortImpl<T, std::bit_and<T>>;

GR_REGISTER_BLOCK("gr::blocks::math::Or", gr::blocks::math::Or, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using Or = MathOpMultiPortImpl<T, std::bit_or<T>>;

GR_REGISTER_BLOCK("gr::blocks::math::Xor", gr::blocks::math::Xor, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using Xor = MathOpMultiPortImpl<T, std::bit_xor<T>>;

GR_REGISTER_BLOCK("gr::blocks::math::Not", gr::blocks::math::Not, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using Not = Unary<bits::Not, T>;

GR_REGISTER_BLOCK("gr::blocks::math::AndConst", gr::blocks::math::AndConst, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using AndConst = Unary<bits::AndConst, T>;

GR_REGISTER_BLOCK("gr::blocks::math::OrConst", gr::blocks::math::OrConst, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using OrConst = Unary<bits::OrConst, T>;

GR_REGISTER_BLOCK("gr::blocks::math::XorConst", gr::blocks::math::XorConst, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using XorConst = Unary<bits::XorConst, T>;

GR_REGISTER_BLOCK("gr::blocks::math::ShiftLeft", gr::blocks::math::ShiftLeft, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using ShiftLeft = Unary<bits::ShiftLeft, T>;

GR_REGISTER_BLOCK("gr::blocks::math::ShiftRight", gr::blocks::math::ShiftRight, [T], [ uint8_t, int16_t, int32_t ])
template<typename T>
using ShiftRight = Unary<bits::ShiftRight, T>;

} // namespace gr::blocks::math

#endif // GNURADIO_MATH_BITS_HPP
