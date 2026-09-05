#include <boost/ut.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/ClockedDataSetToStream.hpp>

namespace {

using gr::blocks::basic::ClockedDataSetToStream;

//! A record of the given values, stamped at the given output-stream position.
[[nodiscard]] gr::DataSet<float> record(std::uint64_t sampleStart, std::vector<float> values) {
    gr::DataSet<float> r;
    r.signal_values = std::move(values);
    r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
    r.signal_names.emplace_back("test");
    r.timing_events.resize(1UZ);
    r.meta_information.resize(1UZ);
    r.meta_information[0UZ]["sample_start"] = sampleStart;
    return r;
}

struct ClockSource : gr::Block<ClockSource> {
    gr::PortOut<std::uint8_t> out;
    GR_MAKE_REFLECTABLE(ClockSource, out);
    std::size_t                    _remaining{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _remaining);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = 0U;
        }
        outSpan.publish(n);
        _remaining -= n;
        return _remaining == 0UZ ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<gr::DataSet<float>, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<gr::DataSet<float>> _records;
    std::size_t                     _pos{0UZ};
    [[nodiscard]] gr::work::Status  processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct StreamSink : gr::Block<StreamSink> {
    gr::PortIn<float> in;
    GR_MAKE_REFLECTABLE(StreamSink, in);
    std::vector<float>             _samples;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const float v : inSpan) {
            _samples.push_back(v);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

//! Run records against a clock of the given length; returns the emitted stream.
[[nodiscard]] std::vector<float> run(std::size_t clockItems, std::vector<gr::DataSet<float>> records, gr::property_map settings = {}) {
    gr::Graph flow;
    auto&     clk  = flow.emplaceBlock<ClockSource>();
    clk._remaining = clockItems;
    auto& src      = flow.emplaceBlock<RecordSource>();
    src._records   = std::move(records);
    auto& paced    = flow.emplaceBlock<ClockedDataSetToStream<float>>(std::move(settings));
    auto& sink     = flow.emplaceBlock<StreamSink>();
    boost::ut::expect(flow.connect<"out", "clock">(clk, paced).has_value());
    boost::ut::expect(flow.connect<"out", "in">(src, paced).has_value());
    boost::ut::expect(flow.connect<"out", "in">(paced, sink).has_value());

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
    return std::move(sink._samples);
}

} // namespace

int main() {
    using namespace boost::ut;

    "a record plays at its stated position, the idle value elsewhere, at the declared ratio"_test = [] {
        const std::vector<float> out = run(60UZ, {record(25ULL, {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f})});
        expect(eq(out.size(), 100UZ)) << "sixty clock items at 3:5 make exactly one hundred samples";
        for (std::size_t i = 0UZ; i < out.size(); ++i) {
            const float want = (i >= 25UZ && i < 35UZ) ? static_cast<float>(i - 24UZ) : 0.0f;
            expect(out[i] == want) << "sample" << i;
        }
    };

    "a gap between records is exactly its stated length of zeros"_test = [] {
        const std::vector<float> out = run(30UZ, {record(10ULL, {1.f, 2.f, 3.f, 4.f, 5.f}), record(20ULL, {6.f, 7.f, 8.f, 9.f, 10.f})});
        expect(eq(out.size(), 50UZ));
        for (std::size_t i = 0UZ; i < out.size(); ++i) {
            float want = 0.0f;
            if (i >= 10UZ && i < 15UZ) {
                want = static_cast<float>(i - 9UZ);
            } else if (i >= 20UZ && i < 25UZ) {
                want = static_cast<float>(i - 14UZ);
            }
            expect(out[i] == want) << "sample" << i;
        }
    };

    "overlapping records resolve in arrival order"_test = [] {
        // The first record covers positions 10..14; the second, arriving after it, covers
        // 12..16. Positions 10..14 come from the first, 15..16 from the second's tail.
        const std::vector<float> out = run(15UZ, {record(10ULL, {1.f, 2.f, 3.f, 4.f, 5.f}), record(12ULL, {6.f, 7.f, 8.f, 9.f, 10.f})});
        expect(eq(out.size(), 25UZ));
        const std::vector<float> want{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 9, 10, 0, 0, 0, 0, 0, 0, 0, 0};
        for (std::size_t i = 0UZ; i < out.size(); ++i) {
            expect(out[i] == want[i]) << "sample" << i;
        }
    };

    "late records are counted, wholly or by their overlap"_test = [] {
        ClockedDataSetToStream<float> block;
        block._emitted = 1000ULL;
        block.absorb(record(500ULL, std::vector<float>(300UZ, 1.f)));
        expect(eq(block.nLateSamples(), 300ULL)) << "wholly in the past: the whole record";
        expect(block._pending.empty());
        block.absorb(record(900ULL, std::vector<float>(300UZ, 1.f)));
        expect(eq(block.nLateSamples(), 400ULL)) << "partly late: the hundred passed samples";
        expect(eq(block._pending.size(), 1UZ)) << "the remainder still plays";
    };

    "the ratio is honored when set through settings"_test = [] {
        const std::vector<float> out = run(10UZ, {record(3ULL, {1.f, 2.f})}, gr::property_map{{"input_chunk_size", 1U}, {"output_chunk_size", 2U}});
        expect(eq(out.size(), 20UZ)) << "ten clock items at 1:2 make twenty samples";
        for (std::size_t i = 0UZ; i < out.size(); ++i) {
            const float want = (i == 3UZ) ? 1.f : (i == 4UZ ? 2.f : 0.0f);
            expect(out[i] == want) << "sample" << i;
        }
    };

    "a record with no samples or no metadata is skipped and counted"_test = [] {
        gr::DataSet<float> empty;
        empty.meta_information.resize(1UZ);
        gr::DataSet<float> mapless;
        mapless.signal_values = {1.f};

        ClockedDataSetToStream<float> block;
        block.absorb(empty);
        block.absorb(mapless);
        expect(eq(block.nRecordsUnplaced(), 2ULL));
        expect(block._pending.empty());

        const std::vector<float> out = run(6UZ, {record(0ULL, {}), record(2ULL, {1.f})});
        expect(eq(out.size(), 10UZ)) << "the empty record stops nothing";
        expect(out[2UZ] == 1.f);
    };

    return 0;
}
