#ifndef GR4_ANALOG_NAMESPACE_COMPATIBILITY_HPP
#define GR4_ANALOG_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::analog {}

namespace gr::analog {

// Deprecated compatibility import: do not use in new code, use gr::blocks::analog instead.
// The forwarding will be removed in a future release.
using namespace ::gr::blocks::analog;

} // namespace gr::analog

#endif // GR4_ANALOG_NAMESPACE_COMPATIBILITY_HPP
