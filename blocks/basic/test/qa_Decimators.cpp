#include <boost/ut.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <cstdlib>
#include <numeric>
#include <print>
#include <span>
#include <vector>

#include <gnuradio-4.0/basic/Decimators.hpp>
#include <gnuradio-4.0/meta/UnitTestHelper.hpp>

#include "TestSpans.hpp"

namespace {

using gr::blocks::basic::KeepMInN;
using gr::blocks::basic::KeepOneInN;
namespace spans = gr::blocks::basic::test;

template<typename TBlock>
[[nodiscard]] TBlock makeBlock(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] std::vector<float> counted(std::size_t nSamples, float first = 0.f) {
    std::vector<float> input(nSamples);
    std::iota(input.begin(), input.end(), first);
    return input;
}

[[nodiscard]] gr::property_map probe(std::size_t which) { return {{"probe", static_cast<gr::Size_t>(which)}}; }

/// @brief One call at an exact absolute input and output position, which is how the large-offset anchors are reached.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<gr::Tag> callAt(TBlock& block, std::size_t nSamples, std::size_t inputAt, std::size_t outputAt, std::span<const gr::Tag> tags) {
    const std::vector<T> input(nSamples, T{});
    std::vector<T>       output(nSamples);
    std::vector<gr::Tag> published;
    spans::InputSpan<T>  inSpan{std::span<const T>(input), inputAt, tags};
    spans::OutputSpan<T> outSpan{std::span<T>(output), outputAt, &published};
    std::ignore = block.processBulk(inSpan, outSpan);
    return published;
}

} // namespace

const boost::ut::suite<"KeepOneInN"> keepOneInNTests = [] {
    using namespace boost::ut;

    "it keeps the last item of every group"_test = [] {
        KeepOneInN<float> block  = makeBlock<KeepOneInN<float>>({{"n", 5U}});
        const auto        input  = counted(10UZ, 1.f);
        const auto        result = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input));
        expect(that % (result.samples == std::vector<float>{5.f, 10.f})) << "the last of each group, not the first";
        expect(eq(result.consumed, input.size()));
    };

    "n = 1 is a pass-through, tags included"_test = [] {
        KeepOneInN<float>          block = makeBlock<KeepOneInN<float>>({{"n", 1U}});
        const auto                 input = counted(64UZ);
        const std::vector<gr::Tag> tags{gr::Tag{0UZ, probe(0)}, gr::Tag{17UZ, probe(1)}, gr::Tag{63UZ, probe(2)}};

        const auto result = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input), 7UZ, std::span<const gr::Tag>(tags));
        expect(std::ranges::equal(result.samples, input));
        expect(that % (result.offsetsOf("probe") == std::vector<std::size_t>{0UZ, 17UZ, 63UZ}));
    };

    "the output count is floor(L/n)"_test = [] {
        for (const gr::Size_t n : {1U, 2U, 3U, 7U, 64U}) {
            KeepOneInN<float> block  = makeBlock<KeepOneInN<float>>({{"n", n}});
            const auto        input  = counted(1000UZ);
            const auto        result = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input), 137UZ);
            expect(eq(result.samples.size(), 1000UZ / static_cast<std::size_t>(n))) << "n = " << n;
        }
    };

    "a tag offset above 2^24 survives exactly"_test = [] {
        // Scaling the offset by a float reciprocal loses it: a float holds integers exactly only to 2^24, so offset
        // 16777217 becomes 16777216, and 50331651 * (1.0f/3.0f) evaluates to 16777218 where the answer is 16777217.
        KeepOneInN<float>          identity = makeBlock<KeepOneInN<float>>({{"n", 1U}});
        const std::vector<gr::Tag> one{gr::Tag{16777217UZ, probe(0)}};
        const auto                 kept = callAt<KeepOneInN<float>, float>(identity, 4UZ, 16777215UZ, 16777215UZ, std::span<const gr::Tag>(one));
        expect(eq(kept.size(), 1UZ));
        expect(eq(kept.front().index, 16777217UZ)) << "n = 1 leaves the tag offset unchanged";

        KeepOneInN<float>          third = makeBlock<KeepOneInN<float>>({{"n", 3U}});
        const std::vector<gr::Tag> three{gr::Tag{50331651UZ, probe(0)}};
        const auto                 moved = callAt<KeepOneInN<float>, float>(third, 30UZ, 50331630UZ, 16777210UZ, std::span<const gr::Tag>(three));
        expect(eq(moved.size(), 1UZ));
        expect(eq(moved.front().index, 16777217UZ)) << "the next kept item, in integer arithmetic";
    };

    "a tag on a dropped item moves forward to the next kept one"_test = [] {
        KeepOneInN<float>          block = makeBlock<KeepOneInN<float>>({{"n", 4U}});
        const auto                 input = counted(16UZ);
        const std::vector<gr::Tag> tags{gr::Tag{0UZ, probe(0)}, gr::Tag{1UZ, probe(1)}, gr::Tag{2UZ, probe(2)}, gr::Tag{3UZ, probe(3)}, gr::Tag{4UZ, probe(4)}};

        const auto result = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input), 0UZ, std::span<const gr::Tag>(tags));
        expect(that % (result.offsetsOf("probe") == std::vector<std::size_t>{0UZ, 0UZ, 0UZ, 0UZ, 1UZ})) << "four onto the first kept item, one onto the second";

        std::vector<gr::Size_t>          order;
        const gr::property_map::key_type key{"probe"};
        for (const gr::Tag& tag : result.tags) {
            order.push_back(gr::test::get_value_or_fail<gr::Size_t>(tag.map.at(key)));
        }
        expect(that % (order == std::vector<gr::Size_t>{0U, 1U, 2U, 3U, 4U})) << "and in input order";
    };

    "setting n restarts the phase"_test = [] {
        KeepOneInN<float> block = makeBlock<KeepOneInN<float>>({{"n", 5U}});
        const auto        input = counted(40UZ, 1.f);

        const auto before = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input).first(13UZ));
        expect(that % (before.samples == std::vector<float>{5.f, 10.f})) << "two groups of five out of thirteen";

        std::ignore = block.settings().setStaged({{"n", 4U}});
        std::ignore = block.settings().applyStagedParameters();

        const auto after = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input).subspan(13UZ));
        expect(eq(after.samples.front(), 17.f)) << "the next item out is n_new inputs after the change, not three";
    };

    "n below one is rejected at settings time"_test = [] { expect(throws([] { std::ignore = makeBlock<KeepOneInN<float>>({{"n", 0U}}); })); };

    "output and tag offsets do not depend on chunking"_test = [] {
        constexpr gr::Size_t n     = 7U;
        const auto           input = counted(1000UZ);
        std::vector<gr::Tag> tags;
        for (const std::size_t at : {0UZ, 6UZ, 7UZ, 13UZ, 500UZ, 999UZ}) {
            tags.push_back(gr::Tag{at, probe(at)});
        }

        KeepOneInN<float> reference = makeBlock<KeepOneInN<float>>({{"n", n}});
        const auto        want      = spans::run<KeepOneInN<float>, float>(reference, std::span<const float>(input), 0UZ, std::span<const gr::Tag>(tags));

        for (const std::size_t chunkSize : {1UZ, 6UZ, 7UZ, 8UZ, 4096UZ}) {
            KeepOneInN<float> block = makeBlock<KeepOneInN<float>>({{"n", n}});
            const auto        got   = spans::run<KeepOneInN<float>, float>(block, std::span<const float>(input), chunkSize, std::span<const gr::Tag>(tags));
            expect(std::ranges::equal(got.samples, want.samples)) << "chunk " << chunkSize;
            expect(that % (got.offsetsOf("probe") == want.offsetsOf("probe"))) << "chunk " << chunkSize;
        }
    };
};

const boost::ut::suite<"KeepMInN"> keepMInNTests = [] {
    using namespace boost::ut;

    "it keeps the slice the parameters name, in order"_test = [] {
        KeepMInN<float> everyOther = makeBlock<KeepMInN<float>>({{"m", 1U}, {"n", 2U}, {"offset", 0U}});
        const auto      hundred    = counted(100UZ);
        const auto      even       = spans::run<KeepMInN<float>, float>(everyOther, std::span<const float>(hundred));
        expect(eq(even.samples.size(), 50UZ));
        expect(eq(even.samples.front(), 0.f));
        expect(eq(even.samples.back(), 98.f));

        KeepMInN<float> everyThird = makeBlock<KeepMInN<float>>({{"m", 1U}, {"n", 3U}, {"offset", 1U}});
        const auto      thirds     = spans::run<KeepMInN<float>, float>(everyThird, std::span<const float>(hundred));
        expect(that % (std::vector<float>(thirds.samples.begin(), thirds.samples.begin() + 3) == std::vector<float>{1.f, 4.f, 7.f}));

        KeepMInN<float> slice  = makeBlock<KeepMInN<float>>({{"m", 2U}, {"n", 5U}, {"offset", 1U}});
        const auto      ten    = counted(10UZ);
        const auto      sliced = spans::run<KeepMInN<float>, float>(slice, std::span<const float>(ten));
        expect(that % (sliced.samples == std::vector<float>{1.f, 2.f, 6.f, 7.f})) << "in that order, without sorting";
    };

    "the output count is m * floor(L/n)"_test = [] {
        KeepMInN<float> block  = makeBlock<KeepMInN<float>>({{"m", 3U}, {"n", 8U}, {"offset", 2U}});
        const auto      input  = counted(1000UZ);
        const auto      result = spans::run<KeepMInN<float>, float>(block, std::span<const float>(input), 137UZ);
        expect(eq(result.samples.size(), 3UZ * (1000UZ / 8UZ)));
        expect(eq(result.consumed, 8UZ * (1000UZ / 8UZ))) << "whole groups only";
    };

    "the item size is carried, not assumed"_test = [] {
        using CF             = std::complex<float>;
        KeepMInN<CF>    wide = makeBlock<KeepMInN<CF>>({{"m", 2U}, {"n", 5U}, {"offset", 1U}});
        std::vector<CF> input(20UZ);
        for (std::size_t i = 0UZ; i < input.size(); ++i) {
            input[i] = CF{static_cast<float>(i), static_cast<float>(-static_cast<double>(i))};
        }
        const auto result = spans::run<KeepMInN<CF>, CF>(wide, std::span<const CF>(input));
        expect(that % (result.samples == std::vector<CF>{input[1], input[2], input[6], input[7], input[11], input[12], input[16], input[17]}));

        KeepMInN<std::uint8_t>          narrow = makeBlock<KeepMInN<std::uint8_t>>({{"m", 2U}, {"n", 5U}, {"offset", 1U}});
        const std::vector<std::uint8_t> bytes{0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U};
        const auto                      kept = spans::run<KeepMInN<std::uint8_t>, std::uint8_t>(narrow, std::span<const std::uint8_t>(bytes));
        expect(that % (kept.samples == std::vector<std::uint8_t>{1U, 2U, 6U, 7U}));
    };

    "a wrapped or empty slice is rejected at settings time"_test = [] {
        expect(throws([] { std::ignore = makeBlock<KeepMInN<float>>({{"m", 3U}, {"n", 5U}, {"offset", 3U}}); })) << "offset + m > n would wrap the slice, and is refused";
        expect(throws([] { std::ignore = makeBlock<KeepMInN<float>>({{"m", 0U}, {"n", 5U}}); })) << "m = 0";
        expect(throws([] { std::ignore = makeBlock<KeepMInN<float>>({{"m", 1U}, {"n", 0U}}); })) << "n = 0";
        expect(throws([] { std::ignore = makeBlock<KeepMInN<float>>({{"m", 6U}, {"n", 5U}}); })) << "m > n";
        expect(throws([] { std::ignore = makeBlock<KeepMInN<float>>({{"m", 1U}, {"n", 5U}, {"offset", 5U}}); })) << "offset >= n";
    };

    "tags map through the group phase, above 2^24 too"_test = [] {
        KeepMInN<float>            block = makeBlock<KeepMInN<float>>({{"m", 2U}, {"n", 5U}, {"offset", 1U}});
        const std::vector<gr::Tag> tags{gr::Tag{16777216UZ, probe(0)}, gr::Tag{16777219UZ, probe(1)}};
        const auto                 published = callAt<KeepMInN<float>, float>(block, 20UZ, 16777215UZ, 6710886UZ, std::span<const gr::Tag>(tags));

        expect(eq(published.size(), 2UZ));
        expect(eq(published[0].index, 6710886UZ)) << "a kept item lands on its own output offset";
        expect(eq(published[1].index, 6710888UZ)) << "an item past the slice lands on the next group's first kept item";
    };

    "output and tag offsets do not depend on chunking"_test = [] {
        const auto           input = counted(1000UZ);
        std::vector<gr::Tag> tags;
        for (const std::size_t at : {0UZ, 3UZ, 4UZ, 5UZ, 501UZ, 999UZ}) {
            tags.push_back(gr::Tag{at, probe(at)});
        }
        const gr::property_map settings = {{"m", 2U}, {"n", 5U}, {"offset", 1U}};

        KeepMInN<float> reference = makeBlock<KeepMInN<float>>(settings);
        const auto      want      = spans::run<KeepMInN<float>, float>(reference, std::span<const float>(input), 0UZ, std::span<const gr::Tag>(tags));

        for (const std::size_t chunkSize : {5UZ, 10UZ, 4096UZ, 137UZ}) {
            KeepMInN<float> block = makeBlock<KeepMInN<float>>(settings);
            const auto      got   = spans::run<KeepMInN<float>, float>(block, std::span<const float>(input), chunkSize, std::span<const gr::Tag>(tags));
            expect(std::ranges::equal(got.samples, want.samples)) << "chunk " << chunkSize;
            expect(that % (got.offsetsOf("probe") == want.offsetsOf("probe"))) << "chunk " << chunkSize;
        }
    };

    "a live change lands on a group boundary"_test = [] {
        KeepMInN<float> block = makeBlock<KeepMInN<float>>({{"m", 2U}, {"n", 5U}, {"offset", 1U}});
        const auto      input = counted(100UZ);

        const auto before = spans::run<KeepMInN<float>, float>(block, std::span<const float>(input).first(20UZ));
        expect(eq(before.consumed, 20UZ)) << "four whole groups";

        std::ignore = block.settings().setStaged({{"m", 1U}, {"n", 4U}, {"offset", 3U}});
        std::ignore = block.settings().applyStagedParameters();

        const auto after = spans::run<KeepMInN<float>, float>(block, std::span<const float>(input).subspan(20UZ));
        expect(eq(after.samples.front(), 23.f)) << "the first group after the change starts where the last one ended";
    };

    "nanoseconds per sample"_test = [] {
        if (std::getenv("ENABLE_BENCHMARK_TESTS") == nullptr) {
            return; // opt-in: a throughput figure belongs to a controlled run, not to every ctest invocation
        }
        using Clock = std::chrono::steady_clock;
        using CF    = std::complex<float>;

        std::vector<CF> x(1UZ << 16);
        for (std::size_t i = 0UZ; i < x.size(); ++i) {
            x[i] = CF{static_cast<float>(i), 0.f};
        }
        std::vector<CF> y(x.size());

        KeepOneInN<CF> oneInN   = makeBlock<KeepOneInN<CF>>({{"n", 8U}});
        KeepMInN<CF>   mInN     = makeBlock<KeepMInN<CF>>({{"m", 3U}, {"n", 8U}, {"offset", 2U}});
        constexpr int  kRepeats = 7;

        double bestOne = 1e30, worstOne = 0.0, bestM = 1e30, worstM = 0.0;
        for (int repeat = 0; repeat < kRepeats; ++repeat) { // arms interleaved, so a thermal drift moves both of them
            spans::InputSpan<CF>  inOne{std::span<const CF>(x)};
            spans::OutputSpan<CF> outOne{std::span<CF>(y)};
            auto                  start = Clock::now();
            std::ignore                 = oneInN.processBulk(inOne, outOne);
            double ns                   = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
            bestOne                     = std::min(bestOne, ns);
            worstOne                    = std::max(worstOne, ns);

            spans::InputSpan<CF>  inM{std::span<const CF>(x)};
            spans::OutputSpan<CF> outM{std::span<CF>(y)};
            start       = Clock::now();
            std::ignore = mInN.processBulk(inM, outM);
            ns          = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()) / static_cast<double>(x.size());
            bestM       = std::min(bestM, ns);
            worstM      = std::max(worstM, ns);
        }
        std::println("KeepOneInN<complex<float>> n=8: best {:.3f} ns/input sample, spread {:.3f} ns", bestOne, worstOne - bestOne);
        std::println("KeepMInN<complex<float>> 3/8: best {:.3f} ns/input sample, spread {:.3f} ns", bestM, worstM - bestM);
    };
};

int main() { /* tests are automatically registered and run */ }
