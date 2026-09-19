#ifndef GNURADIO_TESTING_INSTRUMENTATION_HPP
#define GNURADIO_TESTING_INSTRUMENTATION_HPP

// GCC states a sanitizer in a macro, clang answers __has_feature; neither spelling covers both compilers, so a
// translation unit that asks has to ask twice.
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define GR_TESTING_SANITIZER_PRESENT 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(memory_sanitizer)
#define GR_TESTING_SANITIZER_PRESENT 1
#endif
#endif

namespace gr::blocks::testing {

/// @brief Whether a sanitizer rewrites this translation unit's memory accesses.
///
/// The instrumentation costs several times what an access costs, and a loop's share of it is its share of memory
/// traffic, so two loops are not slowed by the same factor.
inline constexpr bool kInstrumented =
#ifdef GR_TESTING_SANITIZER_PRESENT
    true;
#else
    false;
#endif

/// @brief Whether the translation unit is compiled with optimization; both compilers define `__OPTIMIZE__` from -O1.
inline constexpr bool kOptimized =
#ifdef __OPTIMIZE__
    true;
#else
    false;
#endif

/// @brief Whether a time measured here is the time the same source costs in the build a receiver runs.
///
/// A cost stated as a ratio of two measured loops is a statement about generated code: which work the compiler
/// hoisted, vectorized or elided. Unoptimized code is not that code, and instrumented code is that code plus a
/// per-access charge neither loop pays in proportion, so a ratio taken in either build says nothing about a release.
/// A test that asserts such a ratio measures it only where this holds.
inline constexpr bool kCostMeasurable = kOptimized && !kInstrumented;

} // namespace gr::blocks::testing

#undef GR_TESTING_SANITIZER_PRESENT

#endif // GNURADIO_TESTING_INSTRUMENTATION_HPP
