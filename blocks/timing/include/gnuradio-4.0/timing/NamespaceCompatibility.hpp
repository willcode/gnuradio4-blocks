#ifndef GR4_TIMING_NAMESPACE_COMPATIBILITY_HPP
#define GR4_TIMING_NAMESPACE_COMPATIBILITY_HPP

namespace gr::blocks::timing {}

namespace gr::timing {

// Deprecated compatibility import: do not use in new code, use gr::blocks::timing instead.
// The forwarding will be removed in a future release. It is an import rather than a namespace alias because
// gr::timing is a real namespace — the library's SampleClock and FrequencySchedule live in it — and an alias of
// that name in gr would be a redeclaration the moment a header included both.
using namespace ::gr::blocks::timing;

} // namespace gr::timing

#endif // GR4_TIMING_NAMESPACE_COMPATIBILITY_HPP
