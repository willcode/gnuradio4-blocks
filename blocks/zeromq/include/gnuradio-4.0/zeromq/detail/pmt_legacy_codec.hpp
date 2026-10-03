#pragma once

#include "ZmqDecodeError.hpp"

#include <gnuradio-4.0/Message.hpp>
#include <gnuradio-4.0/Value.hpp>

#include <algorithm>
#include <bit>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <limits>
#include <memory_resource>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace legacy_pmt {

using gr::Tensor;
using gr::pmt::Value;

using Encoded = std::expected<std::vector<uint8_t>, gr::Error>;

inline Encoded serialize_to_legacy(const Value& obj);

enum class legacy_tag : uint8_t { LEGACY_PMT_TRUE = 0x00, LEGACY_PMT_FALSE = 0x01, LEGACY_PMT_SYMBOL = 0x02, LEGACY_PMT_INT32 = 0x03, LEGACY_PMT_DOUBLE = 0x04, LEGACY_PMT_COMPLEX = 0x05, LEGACY_PMT_NULL = 0x06, LEGACY_PMT_PAIR = 0x07, LEGACY_PMT_VECTOR = 0x08, LEGACY_PMT_DICT = 0x09, LEGACY_PMT_UNIFORM_VECTOR = 0x0A, LEGACY_PMT_UINT64 = 0x0B, LEGACY_PMT_TUPLE = 0x0C, LEGACY_PMT_INT64 = 0x0D };

enum class legacy_uniform_type : uint8_t { U8 = 0x00, S8 = 0x01, U16 = 0x02, S16 = 0x03, U32 = 0x04, S32 = 0x05, U64 = 0x06, S64 = 0x07, F32 = 0x08, F64 = 0x09, C32 = 0x0A, C64 = 0x0B, UNKNOWN = 0xFF };

static void write_u8(std::vector<uint8_t>& out, uint8_t v) { out.push_back(v); }

static void write_u16(std::vector<uint8_t>& out, uint32_t v) {
    for (int i = 1; i >= 0; --i) {
        out.push_back((v >> (i * 8)) & 0xFF);
    }
}

static void write_u32(std::vector<uint8_t>& out, uint32_t v) {
    for (int i = 3; i >= 0; --i) {
        out.push_back((v >> (i * 8)) & 0xFF);
    }
}

static void write_u64(std::vector<uint8_t>& out, uint64_t v) {
    for (int i = 7; i >= 0; --i) {
        out.push_back((v >> (i * 8)) & 0xFF);
    }
}

static void write_double(std::vector<uint8_t>& out, double d) {
    uint64_t bits;
    std::memcpy(&bits, &d, sizeof(double));
    write_u64(out, bits);
}

using gr::blocks::zeromq::detail::DecodeError;

template<typename T>
using Decoded = std::expected<T, DecodeError>;

static Decoded<void> require_remaining(const uint8_t* data, const uint8_t* end, std::size_t needed) {
    if (data > end || static_cast<std::size_t>(end - data) < needed) {
        return std::unexpected(DecodeError::truncated);
    }
    return {};
}

static Decoded<uint16_t> read_u16(const uint8_t*& data, const uint8_t* end) {
    return require_remaining(data, end, 2).transform([&] {
        const auto result = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
        data += 2;
        return result;
    });
}

static Decoded<uint64_t> read_u32(const uint8_t*& data, const uint8_t* end) {
    return require_remaining(data, end, 4).transform([&] {
        uint64_t result = 0;
        for (int i = 0; i < 4; ++i) {
            result = (result << 8) | data[i];
        }
        data += 4;
        return result;
    });
}

static Decoded<uint64_t> read_u64(const uint8_t*& data, const uint8_t* end) {
    return require_remaining(data, end, 8).transform([&] {
        uint64_t result = 0;
        for (int i = 0; i < 8; ++i) {
            result = (result << 8) | data[i];
        }
        data += 8;
        return result;
    });
}

static Decoded<double> read_double(const uint8_t*& data, const uint8_t* end) {
    return read_u64(data, end).transform([](uint64_t bits) { return std::bit_cast<double>(bits); });
}

template<typename T>
constexpr legacy_uniform_type legacy_uniform_type_for() {
    if constexpr (std::is_same_v<T, uint8_t>) {
        return legacy_uniform_type::U8;
    } else if constexpr (std::is_same_v<T, int8_t>) {
        return legacy_uniform_type::S8;
    } else if constexpr (std::is_same_v<T, uint16_t>) {
        return legacy_uniform_type::U16;
    } else if constexpr (std::is_same_v<T, int16_t>) {
        return legacy_uniform_type::S16;
    } else if constexpr (std::is_same_v<T, uint32_t>) {
        return legacy_uniform_type::U32;
    } else if constexpr (std::is_same_v<T, int32_t>) {
        return legacy_uniform_type::S32;
    } else if constexpr (std::is_same_v<T, uint64_t>) {
        return legacy_uniform_type::U64;
    } else if constexpr (std::is_same_v<T, int64_t>) {
        return legacy_uniform_type::S64;
    } else if constexpr (std::is_same_v<T, float>) {
        return legacy_uniform_type::F32;
    } else if constexpr (std::is_same_v<T, double>) {
        return legacy_uniform_type::F64;
    } else if constexpr (std::is_same_v<T, std::complex<float>>) {
        return legacy_uniform_type::C32;
    } else if constexpr (std::is_same_v<T, std::complex<double>>) {
        return legacy_uniform_type::C64;
    } else {
        return legacy_uniform_type::UNKNOWN;
    }
}

template<typename T>
requires std::is_integral_v<T> && (!std::is_same_v<T, bool>)
T to_big_endian_integral(T value) {
    if constexpr (std::endian::native == std::endian::little) {
        if constexpr (sizeof(T) == 1) {
            return value;
        } else if constexpr (sizeof(T) == 2) {
            return static_cast<T>(((static_cast<uint16_t>(value) & 0xFF00) >> 8) | ((static_cast<uint16_t>(value) & 0x00FF) << 8));
        } else if constexpr (sizeof(T) == 4) {
            return static_cast<T>(((static_cast<uint32_t>(value) & 0xFF000000) >> 24) | ((static_cast<uint32_t>(value) & 0x00FF0000) >> 8) | ((static_cast<uint32_t>(value) & 0x0000FF00) << 8) | ((static_cast<uint32_t>(value) & 0x000000FF) << 24));
        } else if constexpr (sizeof(T) == 8) {
            uint64_t u64_val = static_cast<uint64_t>(value);
            return static_cast<T>(((u64_val & 0xFF00000000000000ULL) >> 56) | ((u64_val & 0x00FF000000000000ULL) >> 40) | ((u64_val & 0x0000FF0000000000ULL) >> 24) | ((u64_val & 0x000000FF00000000ULL) >> 8) | ((u64_val & 0x00000000FF000000ULL) << 8) | ((u64_val & 0x0000000000FF0000ULL) << 24) | ((u64_val & 0x000000000000FF00ULL) << 40) | ((u64_val & 0x00000000000000FFULL) << 56));
        } else {
            std::vector<uint8_t> bytes(sizeof(T));
            std::memcpy(bytes.data(), &value, sizeof(T));
            std::reverse(bytes.begin(), bytes.end());
            T result;
            std::memcpy(&result, bytes.data(), sizeof(T));
            return result;
        }
    }
    return value;
}

template<typename T>
requires std::is_trivially_copyable_v<T>
void serialize_to_big_endian(const T& value, std::vector<uint8_t>& out) {
    if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
        T              big_endian_val = to_big_endian_integral(value);
        const uint8_t* bytes          = reinterpret_cast<const uint8_t*>(&big_endian_val);
        out.insert(out.end(), bytes, bytes + sizeof(T));
    } else if constexpr (std::is_floating_point_v<T>) {
        if constexpr (sizeof(T) == sizeof(uint32_t)) {
            uint32_t       int_repr       = std::bit_cast<uint32_t>(value);
            uint32_t       big_endian_int = to_big_endian_integral(int_repr);
            const uint8_t* bytes          = reinterpret_cast<const uint8_t*>(&big_endian_int);
            out.insert(out.end(), bytes, bytes + sizeof(uint32_t));
        } else if constexpr (sizeof(T) == sizeof(uint64_t)) {
            uint64_t       int_repr       = std::bit_cast<uint64_t>(value);
            uint64_t       big_endian_int = to_big_endian_integral(int_repr);
            const uint8_t* bytes          = reinterpret_cast<const uint8_t*>(&big_endian_int);
            out.insert(out.end(), bytes, bytes + sizeof(uint64_t));
        } else {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
            out.insert(out.end(), bytes, bytes + sizeof(T));
        }
    } else if constexpr (std::is_same_v<T, std::complex<float>>) {
        serialize_to_big_endian(value.real(), out);
        serialize_to_big_endian(value.imag(), out);
    } else if constexpr (std::is_same_v<T, std::complex<double>>) {
        serialize_to_big_endian(value.real(), out);
        serialize_to_big_endian(value.imag(), out);
    } else {
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
        out.insert(out.end(), bytes, bytes + sizeof(T));
    }
}

template<typename T>
std::vector<uint8_t> serialize_uniform_vector(const std::vector<T>& vec) {
    std::vector<uint8_t> out;

    write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_UNIFORM_VECTOR));
    write_u8(out, static_cast<uint8_t>(legacy_uniform_type_for<T>()));
    write_u32(out, static_cast<uint32_t>(vec.size()));
    write_u8(out, static_cast<uint8_t>(1));
    write_u8(out, static_cast<uint8_t>(0));

    for (const auto& val : vec) {
        serialize_to_big_endian(val, out);
    }

    return out;
}

static void append_encoded(std::vector<uint8_t>& out, const std::vector<uint8_t>& bytes) { out.insert(out.end(), bytes.begin(), bytes.end()); }

static Encoded serialize_string(std::string_view str) {
    if (str.size() > std::numeric_limits<uint16_t>::max()) {
        return std::unexpected(gr::Error("Legacy PMT symbol is too long"));
    }
    std::vector<uint8_t> out;
    write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_SYMBOL));
    write_u16(out, static_cast<uint32_t>(str.size()));
    out.insert(out.end(), str.begin(), str.end());
    return out;
}

static Encoded serialize_pair(const Tensor<Value>& tensor);
static Encoded serialize_tuple(const Tensor<Value>& tensor);

static Encoded serialize_dict(const Value::Map& map) {
    std::vector<std::string> keys;
    keys.reserve(map.size());
    for (const auto& [k, _] : map) {
        keys.emplace_back(k);
    }
    std::sort(keys.begin(), keys.end());

    std::vector<uint8_t> out;
    for (const auto& key : keys) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_DICT));
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_PAIR));
        const auto key_bytes = serialize_string(key);
        if (!key_bytes) {
            return key_bytes;
        }
        append_encoded(out, *key_bytes);
        std::pmr::string key_pmr(key, std::pmr::get_default_resource());
        auto             it = map.find(key_pmr);
        if (it == map.end()) {
            return std::unexpected(gr::Error("Legacy PMT dict missing key"));
        }
        const auto val_bytes = serialize_to_legacy(it->second);
        if (!val_bytes) {
            return val_bytes;
        }
        append_encoded(out, *val_bytes);
    }
    write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_NULL));
    return out;
}

static Encoded serialize_tuple(const Tensor<Value>& tensor) {
    std::vector<uint8_t> out;
    write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_TUPLE));
    write_u32(out, static_cast<uint32_t>(tensor.size()));
    for (const auto& v : tensor) {
        const auto bytes = serialize_to_legacy(v);
        if (!bytes) {
            return bytes;
        }
        append_encoded(out, *bytes);
    }
    return out;
}

static Encoded serialize_pair(const Tensor<Value>& tensor) {
    if (tensor.size() != 2) {
        return std::unexpected(gr::Error("Legacy PMT pair serialization requires a 2-element tensor"));
    }

    std::vector<uint8_t> out;
    write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_PAIR));
    for (const auto& element : tensor) {
        const auto bytes = serialize_to_legacy(element);
        if (!bytes) {
            return bytes;
        }
        append_encoded(out, *bytes);
    }
    return out;
}

static std::vector<uint8_t> serialize_complex(std::complex<double> value) {
    std::vector<uint8_t> out;
    write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_COMPLEX));
    write_double(out, value.real());
    write_double(out, value.imag());
    return out;
}

template<typename T>
static std::vector<uint8_t> serialize_tensor_elements(const Tensor<T>& tensor) {
    return serialize_uniform_vector(std::vector<T>(tensor.begin(), tensor.end()));
}

static Encoded serialize_tensor(const Value& obj) {
    if (auto t = obj.get_if<Tensor<uint8_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<int8_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<uint16_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<int16_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<uint32_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<int32_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<uint64_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<int64_t>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<float>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<double>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<std::complex<float>>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<std::complex<double>>>()) {
        return serialize_tensor_elements(*t);
    }
    if (auto t = obj.get_if<Tensor<Value>>()) {
        return t->size() == 2 ? serialize_pair(*t) : serialize_tuple(*t);
    }
    return std::unexpected(gr::Error("Unsupported tensor type for legacy PMT serialization"));
}

inline Encoded serialize_to_legacy(const Value& obj) {
    std::vector<uint8_t> out;

    if (obj.is_monostate()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_NULL));
        return out;
    }

    if (obj.holds<bool>()) {
        write_u8(out, static_cast<uint8_t>(obj.value_or<bool>(false) ? legacy_tag::LEGACY_PMT_TRUE : legacy_tag::LEGACY_PMT_FALSE));
        return out;
    }

    if (obj.holds<int8_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_INT32));
        write_u32(out, static_cast<uint32_t>(static_cast<int32_t>(obj.value_or<int8_t>(0))));
        return out;
    }
    if (obj.holds<int16_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_INT32));
        write_u32(out, static_cast<uint32_t>(static_cast<int32_t>(obj.value_or<int16_t>(0))));
        return out;
    }
    if (obj.holds<int32_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_INT32));
        write_u32(out, static_cast<uint32_t>(obj.value_or<int32_t>(0)));
        return out;
    }
    if (obj.holds<int64_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_INT64));
        write_u64(out, static_cast<uint64_t>(obj.value_or<int64_t>(0)));
        return out;
    }
    if (obj.holds<uint8_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_UINT64));
        write_u64(out, static_cast<uint64_t>(obj.value_or<uint8_t>(0)));
        return out;
    }
    if (obj.holds<uint16_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_UINT64));
        write_u64(out, static_cast<uint64_t>(obj.value_or<uint16_t>(0)));
        return out;
    }
    if (obj.holds<uint32_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_UINT64));
        write_u64(out, static_cast<uint64_t>(obj.value_or<uint32_t>(0)));
        return out;
    }
    if (obj.holds<uint64_t>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_UINT64));
        write_u64(out, obj.value_or<uint64_t>(0));
        return out;
    }

    if (obj.holds<float>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_DOUBLE));
        write_double(out, static_cast<double>(obj.value_or<float>(0.0f)));
        return out;
    }
    if (obj.holds<double>()) {
        write_u8(out, static_cast<uint8_t>(legacy_tag::LEGACY_PMT_DOUBLE));
        write_double(out, obj.value_or<double>(0.0));
        return out;
    }

    if (obj.holds<std::complex<float>>()) {
        return serialize_complex(static_cast<std::complex<double>>(obj.value_or<std::complex<float>>({})));
    }
    if (obj.holds<std::complex<double>>()) {
        return serialize_complex(obj.value_or<std::complex<double>>({}));
    }

    if (obj.is_string()) {
        return serialize_string(obj.value_or(std::string_view{}));
    }

    if (obj.is_tensor()) {
        return serialize_tensor(obj);
    }

    if (obj.is_map()) {
        if (auto map = obj.get_if<Value::Map>()) {
            return serialize_dict(*map);
        }
        return std::unexpected(gr::Error("Unsupported map type for legacy PMT serialization"));
    }

    return std::unexpected(gr::Error("Unsupported PMT type for legacy serialization"));
}

template<typename T>
requires std::is_integral_v<T> && (!std::is_same_v<T, bool>)
T from_big_endian_integral_to_native(T value) {
    if constexpr (std::endian::native == std::endian::little) {
        if constexpr (sizeof(T) == 1) {
            return value;
        } else if constexpr (sizeof(T) == 2) {
            return static_cast<T>(((static_cast<uint16_t>(value) & 0xFF00) >> 8) | ((static_cast<uint16_t>(value) & 0x00FF) << 8));
        } else if constexpr (sizeof(T) == 4) {
            return static_cast<T>(((static_cast<uint32_t>(value) & 0xFF000000) >> 24) | ((static_cast<uint32_t>(value) & 0x00FF0000) >> 8) | ((static_cast<uint32_t>(value) & 0x0000FF00) << 8) | ((static_cast<uint32_t>(value) & 0x000000FF) << 24));
        } else if constexpr (sizeof(T) == 8) {
            uint64_t u64_val = static_cast<uint64_t>(value);
            return static_cast<T>(((u64_val & 0xFF00000000000000ULL) >> 56) | ((u64_val & 0x00FF000000000000ULL) >> 40) | ((u64_val & 0x0000FF0000000000ULL) >> 24) | ((u64_val & 0x000000FF00000000ULL) >> 8) | ((u64_val & 0x00000000FF000000ULL) << 8) | ((u64_val & 0x0000000000FF0000ULL) << 24) | ((u64_val & 0x000000000000FF00ULL) << 40) | ((u64_val & 0x00000000000000FFULL) << 56));
        } else {
            std::vector<uint8_t> bytes(sizeof(T));
            std::memcpy(bytes.data(), &value, sizeof(T));
            std::reverse(bytes.begin(), bytes.end());
            T result;
            std::memcpy(&result, bytes.data(), sizeof(T));
            return result;
        }
    }
    return value;
}

// The caller checks that sizeof(T) bytes remain.
template<typename T>
requires std::is_trivially_copyable_v<T>
T read_big_endian(const uint8_t*& ptr) {
    T result;

    if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
        T temp_val;
        std::memcpy(&temp_val, ptr, sizeof(T));
        result = from_big_endian_integral_to_native(temp_val);
        ptr += sizeof(T);
    } else if constexpr (std::is_floating_point_v<T>) {
        if constexpr (sizeof(T) == sizeof(uint32_t)) {
            uint32_t int_repr;
            std::memcpy(&int_repr, ptr, sizeof(uint32_t));
            uint32_t native_endian_int = from_big_endian_integral_to_native(int_repr);
            result                     = std::bit_cast<T>(native_endian_int);
            ptr += sizeof(uint32_t);
        } else if constexpr (sizeof(T) == sizeof(uint64_t)) {
            uint64_t int_repr;
            std::memcpy(&int_repr, ptr, sizeof(uint64_t));
            uint64_t native_endian_int = from_big_endian_integral_to_native(int_repr);
            result                     = std::bit_cast<T>(native_endian_int);
            ptr += sizeof(uint64_t);
        } else {
            std::memcpy(&result, ptr, sizeof(T));
            ptr += sizeof(T);
        }
    } else if constexpr (std::is_same_v<T, std::complex<float>>) {
        float real_part = read_big_endian<float>(ptr);
        float imag_part = read_big_endian<float>(ptr);
        result          = std::complex<float>(real_part, imag_part);
    } else if constexpr (std::is_same_v<T, std::complex<double>>) {
        double real_part = read_big_endian<double>(ptr);
        double imag_part = read_big_endian<double>(ptr);
        result           = std::complex<double>(real_part, imag_part);
    } else {
        std::memcpy(&result, ptr, sizeof(T));
        ptr += sizeof(T);
    }
    return result;
}

template<typename VTYPE>
Decoded<std::vector<VTYPE>> create_vector_from_big_endian(const uint8_t*& ptr, const uint8_t* end, std::size_t num_elements) {
    if (ptr > end || num_elements > static_cast<std::size_t>(end - ptr) / sizeof(VTYPE)) {
        return std::unexpected(DecodeError::truncated);
    }
    std::vector<VTYPE> vec;
    vec.reserve(num_elements);

    for (std::size_t i = 0; i < num_elements; ++i) {
        vec.push_back(read_big_endian<VTYPE>(ptr));
    }
    return vec;
}

template<typename VTYPE>
Decoded<Value> uniform_vector_value(const uint8_t*& ptr, const uint8_t* end, std::size_t num_elements) {
    return create_vector_from_big_endian<VTYPE>(ptr, end, num_elements).transform([](std::vector<VTYPE> vec) { return Value(Tensor<VTYPE>(gr::data_from, vec)); });
}

static Decoded<Value> deserialize_value(const uint8_t*& ptr, const uint8_t* end);

static Decoded<Value> deserialize_pair(const uint8_t*& ptr, const uint8_t* end) {
    auto car = deserialize_value(ptr, end);
    if (!car) {
        return car;
    }
    auto cdr = deserialize_value(ptr, end);
    if (!cdr) {
        return cdr;
    }
    std::vector<Value> values;
    values.reserve(2);
    values.push_back(std::move(*car));
    values.push_back(std::move(*cdr));
    return Value(Tensor<Value>(std::move(values)));
}

static Decoded<Value> deserialize_dict(const uint8_t*& ptr, const uint8_t* end) {
    if (ptr >= end) {
        return std::unexpected(DecodeError::truncated);
    }

    Value::Map map{std::pmr::get_default_resource()};

    auto pair_tag = static_cast<legacy_tag>(*ptr++);
    if (pair_tag == legacy_tag::LEGACY_PMT_NULL) {
        return Value(std::move(map));
    }
    if (pair_tag != legacy_tag::LEGACY_PMT_PAIR) {
        return std::unexpected(DecodeError::malformed);
    }

    auto key = deserialize_value(ptr, end);
    if (!key) {
        return key;
    }
    auto val = deserialize_value(ptr, end);
    if (!val) {
        return val;
    }

    if (!key->is_string()) {
        return std::unexpected(DecodeError::malformed);
    }
    std::string key_str = std::string(key->value_or(std::string_view{}));
    map.emplace(std::pmr::string(key_str, std::pmr::get_default_resource()), std::move(*val));

    auto tail = deserialize_value(ptr, end);
    if (!tail) {
        return tail;
    }
    if (tail->is_map()) {
        if (auto tail_map = tail->get_if<Value::Map>()) {
            for (auto& [k, v] : *tail_map) {
                map.emplace(std::pmr::string(k, std::pmr::get_default_resource()), std::move(v));
            }
        }
    } else if (!tail->is_monostate()) {
        return std::unexpected(DecodeError::malformed);
    }

    return Value(std::move(map));
}

static Decoded<Value> deserialize_value(const uint8_t*& ptr, const uint8_t* end) {
    if (ptr >= end) {
        return std::unexpected(DecodeError::truncated);
    }

    auto tag = static_cast<legacy_tag>(*ptr++);
    switch (tag) {
    case legacy_tag::LEGACY_PMT_NULL: return Value{};
    case legacy_tag::LEGACY_PMT_TRUE: return Value(true);
    case legacy_tag::LEGACY_PMT_FALSE: return Value(false);
    case legacy_tag::LEGACY_PMT_INT32: return read_u32(ptr, end).transform([](uint64_t v) { return Value(static_cast<int32_t>(v)); });
    case legacy_tag::LEGACY_PMT_INT64: return read_u64(ptr, end).transform([](uint64_t v) { return Value(static_cast<int64_t>(v)); });
    case legacy_tag::LEGACY_PMT_UINT64: return read_u64(ptr, end).transform([](uint64_t v) { return Value(v); });
    case legacy_tag::LEGACY_PMT_DOUBLE: return read_double(ptr, end).transform([](double v) { return Value(v); });
    case legacy_tag::LEGACY_PMT_COMPLEX: {
        const auto real = read_double(ptr, end);
        if (!real) {
            return std::unexpected(real.error());
        }
        return read_double(ptr, end).transform([&real](double imag) { return Value(std::complex<double>(*real, imag)); });
    }
    case legacy_tag::LEGACY_PMT_SYMBOL: {
        const auto len = read_u16(ptr, end);
        if (!len) {
            return std::unexpected(len.error());
        }
        return require_remaining(ptr, end, *len).transform([&] {
            std::string sym(reinterpret_cast<const char*>(ptr), *len);
            ptr += *len;
            return Value(std::move(sym));
        });
    }
    case legacy_tag::LEGACY_PMT_UNIFORM_VECTOR: {
        if (ptr >= end) {
            return std::unexpected(DecodeError::truncated);
        }
        legacy_uniform_type dtype = static_cast<legacy_uniform_type>(*ptr++);
        const auto          len   = read_u32(ptr, end);
        if (!len) {
            return std::unexpected(len.error());
        }
        if (ptr >= end) {
            return std::unexpected(DecodeError::truncated);
        }
        uint8_t npad = *ptr++;
        if (auto padding = require_remaining(ptr, end, npad); !padding) {
            return std::unexpected(padding.error());
        }
        ptr += npad;

        const auto count = static_cast<std::size_t>(static_cast<uint32_t>(*len));
        switch (dtype) {
        case legacy_uniform_type::U8: return uniform_vector_value<uint8_t>(ptr, end, count);
        case legacy_uniform_type::S8: return uniform_vector_value<int8_t>(ptr, end, count);
        case legacy_uniform_type::U16: return uniform_vector_value<uint16_t>(ptr, end, count);
        case legacy_uniform_type::S16: return uniform_vector_value<int16_t>(ptr, end, count);
        case legacy_uniform_type::U32: return uniform_vector_value<uint32_t>(ptr, end, count);
        case legacy_uniform_type::S32: return uniform_vector_value<int32_t>(ptr, end, count);
        case legacy_uniform_type::U64: return uniform_vector_value<uint64_t>(ptr, end, count);
        case legacy_uniform_type::S64: return uniform_vector_value<int64_t>(ptr, end, count);
        case legacy_uniform_type::F32: return uniform_vector_value<float>(ptr, end, count);
        case legacy_uniform_type::F64: return uniform_vector_value<double>(ptr, end, count);
        case legacy_uniform_type::C32: return uniform_vector_value<std::complex<float>>(ptr, end, count);
        case legacy_uniform_type::C64: return uniform_vector_value<std::complex<double>>(ptr, end, count);
        default: return std::unexpected(DecodeError::unsupported_type);
        }
    }
    case legacy_tag::LEGACY_PMT_TUPLE: {
        const auto len = read_u32(ptr, end);
        if (!len) {
            return std::unexpected(len.error());
        }
        const auto count = static_cast<uint32_t>(*len);
        if (ptr > end || static_cast<std::size_t>(count) > static_cast<std::size_t>(end - ptr)) {
            return std::unexpected(DecodeError::truncated);
        }
        std::vector<Value> values;
        values.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            auto value = deserialize_value(ptr, end);
            if (!value) {
                return value;
            }
            values.push_back(std::move(*value));
        }
        return Value(Tensor<Value>(values));
    }
    case legacy_tag::LEGACY_PMT_PAIR: {
        return deserialize_pair(ptr, end);
    }
    case legacy_tag::LEGACY_PMT_DICT: return deserialize_dict(ptr, end);
    case legacy_tag::LEGACY_PMT_VECTOR:
    default: return std::unexpected(DecodeError::unsupported_type);
    }
}

inline Decoded<gr::pmt::Value> deserialize_from_legacy(const uint8_t* data, std::size_t size) {
    if (!data || size == 0) {
        return std::unexpected(DecodeError::truncated);
    }
    const uint8_t* ptr    = data;
    const uint8_t* end    = data + size;
    auto           result = deserialize_value(ptr, end);
    if (result && ptr != end) {
        return std::unexpected(DecodeError::trailing_bytes);
    }
    return result;
}

inline Decoded<gr::pmt::Value> deserialize_from_legacy(const uint8_t*& data, const uint8_t* end) { return deserialize_value(data, end); }

} // namespace legacy_pmt
