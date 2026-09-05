#include <algorithm>
#include <complex>
#include <cstdint>
#include <span>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/timing/DiscontinuityMonitor.hpp>

#include "TestSpans.hpp"

namespace {

using gr::blocks::timing::DiscontinuityMonitor;
namespace test = gr::blocks::timing::test;
using CF       = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 65536UZ;
constexpr std::size_t kRepeats        = 9UZ;

/// One discontinuity per call, which is already far denser than a real stream drops: the tag path is what the block
/// adds beyond the copy, so measuring it at zero tags would flatter the block rather than bound it.
constexpr std::size_t kTagsPerCall = 1UZ;

/// The account's seqlock holds atomics, so the block is built where it stands and staged in place.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

[[nodiscard]] std::vector<CF> noise() {
    std::vector<CF> data(kSamplesPerCall);
    std::uint64_t   state = 0x243f6a8885a308d3ULL;
    for (CF& sample : data) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        sample = CF(2.f * static_cast<float>(state % 1024ULL) / 1024.f - 1.f, 2.f * static_cast<float>((state >> 20U) % 1024ULL) / 1024.f - 1.f);
    }
    return data;
}

[[nodiscard]] std::vector<gr::Tag> tags() {
    std::vector<gr::Tag> out;
    for (std::size_t k = 0UZ; k < kTagsPerCall; ++k) {
        gr::property_map map;
        map.insert_or_assign(gr::property_map::key_type{gr::tag::N_DROPPED_SAMPLES.shortKey()}, gr::pmt::Value(gr::Size_t{17U}));
        map.insert_or_assign(gr::property_map::key_type{"discontinuity"}, gr::pmt::Value(std::string("gap")));
        out.emplace_back(k * 4096UZ + 11UZ, std::move(map));
    }
    return out;
}

} // namespace

int main() {
    const std::vector<CF>      input = noise();
    const std::vector<gr::Tag> marks = tags();
    std::vector<CF>            output(kSamplesPerCall);

    DiscontinuityMonitor<CF> monitor({{"nominal_rate", 0.0}});
    init(monitor);

    std::vector<gr::Tag> shifted = marks;
    std::size_t          at      = 0UZ;

    std::vector<gr::blocks::testing::bench::Arm> arms{
        {"std::ranges::copy of the same stream",
            [&] {
                std::ranges::copy(input, output.begin());
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"DiscontinuityMonitor, no tags",
            [&] {
                test::InputSpan<CF>  inSpan(std::span<const CF>(input), at);
                test::OutputSpan<CF> outSpan(std::span<CF>(output), at);
                std::ignore = monitor.processBulk(inSpan, outSpan);
                at += kSamplesPerCall;
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
        {"DiscontinuityMonitor, one tag per call",
            [&] {
                for (std::size_t k = 0UZ; k < shifted.size(); ++k) { // moved rather than rebuilt: the arm measures the block, not a vector
                    shifted[k].index = marks[k].index + at;
                }
                test::InputSpan<CF>  inSpan(std::span<const CF>(input), at, std::span<const gr::Tag>(shifted));
                test::OutputSpan<CF> outSpan(std::span<CF>(output), at);
                std::ignore = monitor.processBulk(inSpan, outSpan);
                at += kSamplesPerCall;
                return static_cast<double>(output[kSamplesPerCall / 2UZ].real());
            }},
    };

    gr::blocks::testing::bench::report(std::span<gr::blocks::testing::bench::Arm>(arms), kSamplesPerCall, kRepeats);
}
