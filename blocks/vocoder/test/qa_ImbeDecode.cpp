#include <boost/ut.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/vocoder/Imbe.hpp>
#include <gnuradio-4.0/vocoder/ImbeDecode.hpp>

namespace {

using gr::blocks::vocoder::ImbeDecode;

//! Four consecutive parameter-word sets a real IMBE transmitter sent, in order. The codec
//! decodes each set against the one before it, so order is significant here.
constexpr std::array<std::array<std::uint16_t, 8>, 4> kAirParameters{{
    {{402, 301, 1264, 3494, 50, 58, 508, 26}},
    {{400, 762, 3246, 1855, 58, 1816, 1065, 91}},
    {{410, 698, 1037, 1925, 33, 822, 2044, 114}},
    {{408, 639, 3900, 3520, 27, 2031, 41, 115}},
}};

//! A parameter-word record of two codewords, placed and marked as the caller states.
[[nodiscard]] gr::DataSet<std::uint16_t> record(std::size_t firstRow, std::uint64_t voiceSampleStart, bool clear) {
    gr::DataSet<std::uint16_t> r;
    for (std::size_t n = 0UZ; n < 2UZ; ++n) {
        for (const std::uint16_t word : kAirParameters[firstRow + n]) {
            r.signal_values.push_back(word);
        }
    }
    r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
    r.signal_names.emplace_back("voice");
    r.timing_events.resize(1UZ);
    r.meta_information.resize(1UZ);
    r.meta_information[0UZ]["voice_sample_start"] = voiceSampleStart;
    if (clear) {
        r.meta_information[0UZ]["clear"] = true;
    }
    return r;
}

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<gr::DataSet<std::uint16_t>, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<gr::DataSet<std::uint16_t>> _records;
    std::size_t                             _pos{0UZ};
    [[nodiscard]] gr::work::Status          processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<float>> _records;
    [[nodiscard]] gr::work::Status  processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

//! Run parameter-word records through the block to completion.
[[nodiscard]] std::vector<gr::DataSet<float>> runBlock(std::vector<gr::DataSet<std::uint16_t>> records) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource>();
    src._records  = std::move(records);
    auto& voice   = flow.emplaceBlock<ImbeDecode>();
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, voice).has_value());
    boost::ut::expect(flow.connect<"out", "in">(voice, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!done.load()) {
        scheduler.requestStop();
    }
    runner.join();
    boost::ut::expect(done.load());
    return std::move(sink._records);
}

[[nodiscard]] std::uint64_t placedAt(const gr::DataSet<float>& r) {
    const auto& map   = r.meta_information[0UZ];
    const auto  entry = map.find(gr::property_map::key_type("sample_start"));
    boost::ut::expect(entry != map.end());
    return entry == map.end() ? 0ULL : entry->second.value_or(std::uint64_t{0ULL});
}

//! The kernel's output for rows `first..first+count` decoded in order from a fresh decoder.
[[nodiscard]] std::vector<float> freshDecode(std::size_t first, std::size_t count) {
    gr::vocoder::ImbeDecoder decoder;
    std::vector<float>       samples(count * 160UZ);
    for (std::size_t n = 0UZ; n < count; ++n) {
        decoder.decode(std::span<const std::uint16_t, 8UZ>(kAirParameters[first + n]), std::span<float, 160UZ>(samples.data() + n * 160UZ, 160UZ));
    }
    return samples;
}

} // namespace

int main() {
    using namespace boost::ut;

    "records place at their stamped positions, clamped forward past overlap"_test = [] {
        const auto pcm = runBlock({record(0UZ, 2400ULL, true), record(2UZ, 2500ULL, true)});
        expect(eq(pcm.size(), 2UZ));
        if (pcm.size() != 2UZ) {
            return;
        }
        expect(eq(placedAt(pcm[0]), 2400ULL));
        expect(eq(placedAt(pcm[1]), 2720ULL)) << "a stamp inside the previous voice moves directly after it";

        // Decoder state carries across records inside one utterance: the two records together
        // are byte-identical to the kernel fed the same four codewords in order.
        const std::vector<float> expected = freshDecode(0UZ, 4UZ);
        expect(eq(pcm[0].signal_values.size(), 320UZ));
        expect(eq(pcm[1].signal_values.size(), 320UZ));
        for (std::size_t i = 0UZ; i < 320UZ; ++i) {
            expect(pcm[0].signal_values[i] == expected[i]) << "sample" << i;
            expect(pcm[1].signal_values[i] == expected[320UZ + i]) << "sample" << (320UZ + i);
        }
    };

    "a record not marked clear is muted"_test = [] {
        const auto pcm = runBlock({record(0UZ, 100ULL, false), record(2UZ, 1000ULL, true)});
        expect(eq(pcm.size(), 1UZ)) << "only the clear record is vocoded";
        if (pcm.size() == 1UZ) {
            expect(eq(placedAt(pcm[0]), 1000ULL));
        }
    };

    "a record past the speech gap opens a new utterance"_test = [] {
        const std::uint64_t far = 320ULL + 320ULL + ImbeDecode::kSpeechGapSamples + 1ULL;
        const auto          pcm = runBlock({record(0UZ, 320ULL, true), record(2UZ, far, true)});
        expect(eq(pcm.size(), 2UZ));
        if (pcm.size() != 2UZ) {
            return;
        }
        expect(eq(placedAt(pcm[1]), far));
        // The carried state is dropped: the distant record decodes as a fresh decoder would.
        const std::vector<float> expected = freshDecode(2UZ, 2UZ);
        for (std::size_t i = 0UZ; i < 320UZ; ++i) {
            expect(pcm[1].signal_values[i] == expected[i]) << "sample" << i;
        }
    };

    return 0;
}
