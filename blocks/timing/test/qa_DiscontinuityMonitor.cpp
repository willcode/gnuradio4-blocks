#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/timing/DiscontinuityMonitor.hpp>

#include "TestSpans.hpp"

namespace {

using gr::blocks::timing::DiscontinuityCause;
using gr::blocks::timing::DiscontinuityMonitor;
namespace test = gr::blocks::timing::test;

/// The slot the account is published through holds atomics, so the block is neither copyable nor movable: it is
/// built where it stands and staged in place, as `PowerMeter`'s own suite does.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

[[nodiscard]] gr::Tag dropped(std::size_t at, gr::Size_t count, std::string causes = "gap") {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{gr::tag::N_DROPPED_SAMPLES.shortKey()}, gr::pmt::Value(count));
    map.insert_or_assign(gr::property_map::key_type{"discontinuity"}, gr::pmt::Value(causes));
    return gr::Tag{at, std::move(map)};
}

/// A count with no cause list: the vocabulary allows it, and the total must still carry it.
[[nodiscard]] gr::Tag countOnly(std::size_t at, gr::Size_t count) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{gr::tag::N_DROPPED_SAMPLES.shortKey()}, gr::pmt::Value(count));
    return gr::Tag{at, std::move(map)};
}

[[nodiscard]] gr::Tag causeOnly(std::size_t at, std::string causes) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{"discontinuity"}, gr::pmt::Value(causes));
    return gr::Tag{at, std::move(map)};
}

[[nodiscard]] gr::Tag rateTag(std::size_t at, float rate) {
    gr::property_map map;
    map.insert_or_assign(gr::property_map::key_type{gr::tag::SAMPLE_RATE.shortKey()}, gr::pmt::Value(rate));
    return gr::Tag{at, std::move(map)};
}

[[nodiscard]] std::vector<float> ramp(std::size_t n) {
    std::vector<float> out(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        out[k] = static_cast<float>(k) * 0.25f;
    }
    return out;
}

/// The stream criterion 3 is stated over: four events naming eight causes between them, and 1150 dropped samples.
[[nodiscard]] std::vector<gr::Tag> ledger() {
    return {
        dropped(10UZ, 100U),                                       //
        causeOnly(200UZ, "sample_rate,signal_unit"),               //
        dropped(400UZ, 1000U, "gap,signal_range"),                 //
        causeOnly(700UZ, "signal_name,signal_quantity,who_knows"), //
        countOnly(900UZ, 50U),                                     //
    };
}

/// @brief The offsets at which a key outside `gr::tag::kDefaultTags` reaches a sink through the monitor.
[[nodiscard]] std::vector<std::size_t> forwardedOffsets(std::string_view key) {
    using gr::testing::ProcessFunction;
    using gr::testing::TagSink;
    using gr::testing::TagSource;

    const gr::property_map::key_type wanted{key};

    gr::Graph graph;
    auto&     source = graph.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", gr::Size_t(2048)}, {"mark_tag", false}});
    for (const std::size_t at : {7UZ, 300UZ, 1000UZ}) {
        gr::property_map map;
        map.insert_or_assign(wanted, gr::pmt::Value(std::string("gap")));
        source._tags.emplace_back(at, std::move(map));
    }
    auto& block = graph.emplaceBlock<DiscontinuityMonitor<float>>({{"nominal_rate", 0.0}});
    auto& sink  = graph.emplaceBlock<TagSink<float, ProcessFunction::USE_PROCESS_ONE>>({{"name", "TagSink"}});

    boost::ut::expect(graph.connect<"out", "in">(source, block).has_value());
    boost::ut::expect(graph.connect<"out", "in">(block, sink).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());

    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : sink._tags) {
        if (tag.map.contains(wanted)) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

} // namespace

const boost::ut::suite<"discontinuity monitor"> discontinuityMonitorTests = [] {
    using namespace boost::ut;

    // criterion 3 — the totals and the per-cause counts are exact
    "the ledger totals dropped samples and counts every cause"_test = [] {
        const std::vector<float>   input = ramp(1200UZ);
        const std::vector<gr::Tag> tags  = ledger();

        DiscontinuityMonitor<float> block({{"nominal_rate", 0.0}});
        init(block);
        const auto result = test::run<float>(block, std::span<const float>(input), 0UZ, std::span<const gr::Tag>(tags));

        expect(eq(result.consumed, 1200UZ));
        expect(eq(block.nDroppedSamples(), 1150ULL)) << "100 + 1000 + 50, exactly";
        expect(eq(block.nEvents(), 4ULL)) << "four tags carried a discontinuity list; the fifth carried only a count";
        expect(eq(block.nEvents(DiscontinuityCause::Gap), 2ULL));
        expect(eq(block.nEvents(DiscontinuityCause::SampleRate), 1ULL));
        expect(eq(block.nEvents(DiscontinuityCause::SignalUnit), 1ULL));
        expect(eq(block.nEvents(DiscontinuityCause::SignalRange), 1ULL));
        expect(eq(block.nEvents(DiscontinuityCause::SignalName), 1ULL));
        expect(eq(block.nEvents(DiscontinuityCause::SignalQuantity), 1ULL));
        expect(eq(block.nEvents(DiscontinuityCause::Other), 1ULL)) << "a cause name this vocabulary does not know is counted, not dropped";
        expect(eq(block.nRateChanges(), 0ULL));
        expect(eq(block.nRateMismatches(), 0ULL));
    };

    // criterion 3 — a rate change re-anchors, and each epoch converts exactly on its own clock
    "a rate change re-anchors the clock and both epochs convert exactly"_test = [] {
        constexpr std::int64_t     kEpochNs = 1'756'000'000'000'000'000LL; // a real wall-clock nanosecond, far past 2^53
        const std::vector<float>   input    = ramp(2000UZ);
        const std::vector<gr::Tag> tags{rateTag(1000UZ, 200'000.f)};

        DiscontinuityMonitor<float> block({{"nominal_rate", 0.0}, {"anchor_index", std::uint64_t{0}}, {"anchor_ns", kEpochNs}});
        init(block);

        // the first epoch: 250 kS/s stated as the block's own starting rate, taken from the first tag the stream sends
        const std::vector<gr::Tag> first{rateTag(0UZ, 250'000.f)};
        std::ignore = test::run<float>(block, std::span<const float>(input).first(1UZ), 0UZ, std::span<const gr::Tag>(first));
        expect(block.hasClock());
        const gr::timing::SampleClock before = block.clock();
        expect(eq(before.timeOf(0ULL), kEpochNs));
        expect(eq(before.timeOf(1000ULL), kEpochNs + 4'000'000LL)) << "1000 samples at 250 kS/s is exactly 4 ms";
        expect(eq(before.timeOf(999ULL), kEpochNs + 3'996'000LL));

        const auto second = test::run<float>(block, std::span<const float>(input).subspan(1UZ), 0UZ, std::span<const gr::Tag>(tags), 0UZ, 1UZ);
        expect(eq(second.consumed, 1999UZ));
        expect(eq(block.nRateChanges(), 2ULL)) << "the stream's first stated rate and the change at 1000";

        const gr::timing::SampleClock after = block.clock();
        expect(eq(after.timeOf(1000ULL), before.timeOf(1000ULL))) << "the change is continuous: the anchor sample reads the same time on either clock";
        expect(eq(after.timeOf(1500ULL), kEpochNs + 4'000'000LL + 2'500'000LL)) << "500 samples at 200 kS/s is exactly 2.5 ms";
        expect(eq(after.timeOf(2000ULL), kEpochNs + 4'000'000LL + 5'000'000LL));
        expect(eq(after.indexOf(kEpochNs + 4'000'000LL + 2'500'000LL).index, 1500ULL));
        expect(eq(after.indexOf(kEpochNs + 4'000'000LL + 2'500'000LL).remainder_num, 0ULL)) << "the conversion back is exact, not near";
    };

    "a stated rate outside the tolerance is refused, counted and named"_test = [] {
        const std::vector<float>   input = ramp(400UZ);
        const std::vector<gr::Tag> tags{rateTag(100UZ, 96'000.f), rateTag(200UZ, 48'000.5f), rateTag(300UZ, 48'000.f)};

        DiscontinuityMonitor<float> block({{"nominal_rate", 48'000.0}, {"rate_tolerance", 1e-5}});
        init(block);
        expect(block.hasClock()) << "a positive nominal_rate is itself a rate";
        std::ignore = test::run<float>(block, std::span<const float>(input), 64UZ, std::span<const gr::Tag>(tags));

        expect(eq(block.nRateMismatches(), 2ULL)) << "96 kHz and 48000.5 Hz both disagree by more than 1e-5 relative";
        expect(eq(block.lastRefusedRate(), 48'000.5)) << "the refused rate is named, not merely counted";
        expect(eq(block.nRateChanges(), 0ULL)) << "the clock keeps the rate it had; 48 kHz restated is not a change";
        expect(eq(block.clock().rate_num, 48'000'000'000ULL));
        expect(eq(block.clock().rate_den, 1'000'000ULL));
    };

    // criterion 6 — the account and the stream are the same however the stream is cut
    "the account and the samples are independent of the chunking"_test = [] {
        const std::vector<float>   input = ramp(1200UZ);
        const std::vector<gr::Tag> tags  = ledger();

        DiscontinuityMonitor<float> reference({{"nominal_rate", 0.0}});
        init(reference);
        const auto whole  = test::run<float>(reference, std::span<const float>(input), 0UZ, std::span<const gr::Tag>(tags));
        const auto wanted = reference.account();

        expect(eq(whole.samples.size(), input.size()));
        expect(std::ranges::equal(whole.samples, input)) << "a passthrough passes the samples through bit for bit";

        for (const std::size_t chunk : {1UZ, 7UZ, 128UZ, 1199UZ}) {
            DiscontinuityMonitor<float> block({{"nominal_rate", 0.0}});
            init(block);
            const auto result = test::run<float>(block, std::span<const float>(input), chunk, std::span<const gr::Tag>(tags));
            expect(eq(result.samples.size(), input.size())) << std::format("chunk {}", chunk);
            expect(std::ranges::equal(result.samples, input)) << std::format("chunk {}", chunk);
            expect(eq(block.nDroppedSamples(), 1150ULL)) << std::format("chunk {}", chunk);
            expect(block.account() == wanted) << std::format("chunk {}: the whole account, not just the total", chunk);
        }
    };

    "every type the block carries passes its samples through"_test = [] {
        const std::vector<gr::Tag> tags = ledger();

        std::vector<std::uint8_t> bytes(1200UZ);
        for (std::size_t k = 0UZ; k < bytes.size(); ++k) {
            bytes[k] = static_cast<std::uint8_t>(k & 0xFFUZ);
        }
        DiscontinuityMonitor<std::uint8_t> byteBlock({{"nominal_rate", 0.0}});
        init(byteBlock);
        const auto byteRun = test::run<std::uint8_t>(byteBlock, std::span<const std::uint8_t>(bytes), 97UZ, std::span<const gr::Tag>(tags));
        expect(std::ranges::equal(byteRun.samples, bytes)) << "std::uint8_t is what PpsSource emits";
        expect(eq(byteBlock.nDroppedSamples(), 1150ULL));

        std::vector<std::complex<float>> iq(1200UZ);
        for (std::size_t k = 0UZ; k < iq.size(); ++k) {
            iq[k] = std::complex<float>(static_cast<float>(k), -static_cast<float>(k));
        }
        DiscontinuityMonitor<std::complex<float>> iqBlock({{"nominal_rate", 0.0}});
        init(iqBlock);
        const auto iqRun = test::run<std::complex<float>>(iqBlock, std::span<const std::complex<float>>(iq), 97UZ, std::span<const gr::Tag>(tags));
        expect(std::ranges::equal(iqRun.samples, iq));
        expect(eq(iqBlock.nDroppedSamples(), 1150ULL));
    };

    // the passthrough must not eat the evidence: `discontinuity` is no reserved tag, and the framework's default
    // filter drops exactly such a key
    "a key the default filter would drop reaches the sink at its own offset"_test = [] {
        static_assert(DiscontinuityMonitor<float>::unfilteredTagPropagation, "the block forwards every key, which is what makes the account reproducible downstream");
        expect(!std::ranges::contains(gr::tag::kDefaultTags, std::string_view("discontinuity"))) << "the key is not a reserved tag, so filtered propagation would drop it";

        const std::vector<std::size_t> offsets = forwardedOffsets("discontinuity");
        expect(eq(offsets.size(), 3UZ)) << "all three tags reach the sink";
        expect(offsets == std::vector<std::size_t>{7UZ, 300UZ, 1000UZ}) << "each at the offset it arrived at";
    };
};

int main() { /* not needed for UT */ }
