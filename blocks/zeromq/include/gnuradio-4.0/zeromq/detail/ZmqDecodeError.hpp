#pragma once
#include <cstdint>

namespace gr::blocks::zeromq::detail {
// The reason a peer's message does not decode. A receiving block counts such a
// message as refused. Allocation, transport, configuration and publication
// failures are not decode errors and propagate.
enum class DecodeError : std::uint8_t {
    truncated,          // the message ends inside a field
    bad_magic,          // the tag header does not start with its magic number
    bad_version,        // the tag header carries an unknown version
    malformed,          // a field holds a value its format does not allow
    unsupported_type,   // a PMT type tag this codec does not decode
    trailing_bytes,     // bytes follow a complete value
    item_size_mismatch, // the payload is not a whole number of items
};
} // namespace gr::blocks::zeromq::detail
