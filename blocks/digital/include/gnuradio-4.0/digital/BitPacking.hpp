#ifndef GNURADIO_DIGITAL_BIT_PACKING_HPP
#define GNURADIO_DIGITAL_BIT_PACKING_HPP

// DEPRECATED COMPATIBILITY HEADER: Do not use in new code. Include <gnuradio-4.0/math/BitPacking.hpp> and name
// gr::blocks::math instead. PackBits, UnpackBits, RepackBits and RecordRepackBits are defined in gr::blocks::math.
// This header names them in gr::blocks::digital and registers the math blocks under their gr::blocks::digital keys.
// Release 4.0 is the last release that ships these names and keys.

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/math/BitPacking.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK("gr::blocks::digital::PackBits", gr::blocks::math::PackBits)
using gr::blocks::math::PackBits;

GR_REGISTER_BLOCK("gr::blocks::digital::UnpackBits", gr::blocks::math::UnpackBits)
using gr::blocks::math::UnpackBits;

GR_REGISTER_BLOCK("gr::blocks::digital::RepackBits", gr::blocks::math::RepackBits)
using gr::blocks::math::RepackBits;

GR_REGISTER_BLOCK("gr::blocks::digital::RecordRepackBits", gr::blocks::math::RecordRepackBits)
using gr::blocks::math::RecordRepackBits;

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_BIT_PACKING_HPP
