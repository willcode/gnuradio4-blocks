#include <boost/ut.hpp>

#include <gnuradio-4.0/fileio/SigMfIo.hpp>

#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// A recording is what one program hands another, so the assertions below are byte assertions at both ends: the
// canonical recording is read into samples and tags and compared value by value, then written back out and compared
// against the same 422 metadata bytes and 64 dataset bytes. Every refusal is driven through a real graph, and the
// pool-contention case is the measured BasicFileSink deadlock turned into a regression test.

namespace {

using namespace boost::ut;
using namespace std::string_literals;
using namespace std::string_view_literals;
using namespace std::chrono_literals;

using gr::blocks::fileio::SigMfSink;
using gr::blocks::fileio::SigMfSource;
namespace keys = gr::blocks::fileio::sigmf_keys;

template<typename T>
using TagSinkFor = gr::blocks::testing::TagSink<T, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>;

std::string gProgramName = "qa_SigMfIo";

/// A temporary directory named for this binary, removed when the test that owns it ends. A fixed path under /tmp is
/// a parallel-ctest hazard, so nothing here uses one.
struct Workspace {
    std::filesystem::path dir;

    explicit Workspace(std::string_view testName) : dir(std::filesystem::temp_directory_path() / std::filesystem::path(gProgramName).filename() / testName) {
        std::error_code status;
        std::filesystem::remove_all(dir, status);
        std::filesystem::create_directories(dir, status);
    }
    ~Workspace() {
        std::error_code status;
        std::filesystem::remove_all(dir, status);
    }
    Workspace(const Workspace&)            = delete;
    Workspace& operator=(const Workspace&) = delete;
    Workspace(Workspace&&)                 = delete;
    Workspace& operator=(Workspace&&)      = delete;

    [[nodiscard]] std::string base(std::string_view name) const { return (dir / name).string(); }
};

void writeBytes(const std::string& path, std::span<const std::byte> bytes) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

void writeText(const std::string& path, std::string_view text) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file.write(text.data(), static_cast<std::streamsize>(text.size()));
}

[[nodiscard]] std::vector<std::byte> readBytes(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return {};
    }
    const std::string      raw{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    std::vector<std::byte> bytes(raw.size());
    std::memcpy(bytes.data(), raw.data(), raw.size());
    return bytes;
}

[[nodiscard]] std::string readText(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return {};
    }
    return std::string{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

[[nodiscard]] std::string hexOf(std::span<const std::byte> bytes) {
    std::string out;
    for (const std::byte value : bytes) {
        out.append(std::format("{:02X}", std::to_integer<unsigned>(value)));
    }
    return out;
}

[[nodiscard]] bool mentions(std::string_view message, std::string_view refusal) { return message.find(refusal) != std::string_view::npos; }

// ─── anchor A ────────────────────────────────────────────────────────────────────────────────────────────────────

constexpr std::array<std::complex<float>, 8> kAnchorSamples{std::complex<float>{0.0f, 0.0f}, std::complex<float>{1.0f, 0.0f}, std::complex<float>{0.0f, 1.0f}, std::complex<float>{-1.0f, 0.0f}, //
    std::complex<float>{0.0f, -1.0f}, std::complex<float>{0.5f, 0.5f}, std::complex<float>{-0.5f, 0.5f}, std::complex<float>{0.25f, -0.75f}};

constexpr std::uint64_t kAnchorTimeNs = 1787745600000000000ULL;

constexpr std::string_view kAnchorMetadata = R"({
  "global": {
    "core:datatype": "cf32_le",
    "core:version": "1.2.6",
    "core:sample_rate": 48000,
    "core:recorder": "gnuradio4"
  },
  "captures": [
    {
      "core:sample_start": 0,
      "core:frequency": 433921337,
      "core:datetime": "2026-08-26T12:00:00.000000Z"
    }
  ],
  "annotations": [
    {
      "core:sample_start": 4,
      "core:sample_count": 2,
      "core:label": "burst"
    }
  ]
}
)"sv;

[[nodiscard]] std::vector<std::byte> anchorDataBytes() {
    std::vector<std::byte> bytes(kAnchorSamples.size() * sizeof(std::complex<float>));
    std::memcpy(bytes.data(), kAnchorSamples.data(), bytes.size());
    return bytes;
}

[[nodiscard]] std::string writeAnchor(const Workspace& workspace, std::string_view name, std::string_view metadata = kAnchorMetadata) {
    const std::string base = workspace.base(name);
    writeText(base + ".sigmf-meta", metadata);
    writeBytes(base + ".sigmf-data", anchorDataBytes());
    return base;
}

// ─── tag helpers ─────────────────────────────────────────────────────────────────────────────────────────────────

template<typename T>
[[nodiscard]] std::optional<T> tagValue(const gr::property_map& map, std::string_view key) {
    const gr::pmt::Value* value = gr::blocks::fileio::sigmf_detail::findTag(map, key);
    if (value == nullptr) {
        return std::nullopt;
    }
    if constexpr (std::same_as<T, std::string>) {
        return value->value_or(std::string(""));
    } else {
        const T* held = value->template get_if<T>();
        return held == nullptr ? std::nullopt : std::optional<T>(*held);
    }
}

/// Tags a sink saw, merged by stream index, so an assertion is about what a downstream consumer reads at an index
/// rather than about how many maps the port machinery chose to deliver there.
[[nodiscard]] std::map<std::size_t, gr::property_map> mergedByIndex(std::span<const gr::Tag> tags) {
    std::map<std::size_t, gr::property_map> merged;
    for (const gr::Tag& tag : tags) {
        gr::property_map& slot = merged[tag.index];
        for (const auto& [key, value] : tag.map) {
            slot.insert_or_assign(key, value);
        }
    }
    return merged;
}

[[nodiscard]] gr::property_map anchorIndexZeroTag() {
    gr::property_map map;
    gr::tag::put(map, gr::tag::SAMPLE_RATE, 48000.f);
    gr::tag::put(map, gr::tag::NUM_CHANNELS, gr::Size_t{1U});
    gr::tag::put(map, keys::kDatatype, std::string("cf32_le"));
    gr::tag::put(map, gr::tag::TRIGGER_NAME, std::string("sigmf:capture"));
    gr::tag::put(map, gr::tag::TRIGGER_TIME, kAnchorTimeNs);
    gr::tag::put(map, gr::tag::TRIGGER_OFFSET, 0.f);
    gr::tag::put(map, gr::tag::FREQUENCY, 433921337.0);
    return map;
}

[[nodiscard]] gr::property_map anchorAnnotationTag() {
    gr::property_map map;
    gr::tag::put(map, keys::kAnnotationLabel, std::string("burst"));
    gr::tag::put(map, keys::kAnnotationLength, gr::Size_t{2U});
    return map;
}

// ─── test-only blocks ────────────────────────────────────────────────────────────────────────────────────────────

/// Emits exactly the values and tags a test scripts. It owns no reserved settings key, so nothing but the script
/// reaches the stream.
template<typename T>
struct ScriptedSource : gr::Block<ScriptedSource<T>> {
    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(ScriptedSource, out);

    std::vector<T>       _values{};
    std::vector<gr::Tag> _tags{}; ///< ascending absolute indices
    std::size_t          _produced{0UZ};
    std::size_t          _nextTag{0UZ};

    using gr::Block<ScriptedSource<T>>::Block;

    void start() {
        _produced = 0UZ;
        _nextTag  = 0UZ;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_produced >= _values.size()) {
            outSpan.publish(0UZ);
            return gr::work::Status::DONE;
        }
        const std::size_t count = std::min(outSpan.size(), _values.size() - _produced);
        if (count == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        std::copy_n(_values.begin() + static_cast<std::ptrdiff_t>(_produced), count, outSpan.begin());
        while (_nextTag < _tags.size() && _tags[_nextTag].index < _produced + count) {
            outSpan.publishTag(_tags[_nextTag].map, _tags[_nextTag].index - _produced);
            ++_nextTag;
        }
        _produced += count;
        outSpan.publish(count);
        return _produced >= _values.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// A 1:1 block forwarding through the framework's default forwarder, which filters to the reserved keys.
template<typename T>
struct FilteringPassthrough : gr::Block<FilteringPassthrough<T>> {
    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    GR_MAKE_REFLECTABLE(FilteringPassthrough, in, out);

    using gr::Block<FilteringPassthrough<T>>::Block;

    [[nodiscard]] constexpr T processOne(const T& value) const noexcept { return value; }
};

/// Stands in for a rate changer: it republishes every tag with `sample_rate` rewritten, which is what the standing
/// rescale ruling requires of anything that changes the rate.
template<typename T>
struct RateRewriter : gr::Block<RateRewriter<T>, gr::NoTagPropagation> {
    gr::PortIn<T>  in;
    gr::PortOut<T> out;

    gr::Annotated<float, "new_rate", gr::Doc<"The rate this block claims for its output">> new_rate = 0.f;

    GR_MAKE_REFLECTABLE(RateRewriter, in, out, new_rate);

    using gr::Block<RateRewriter<T>, gr::NoTagPropagation>::Block;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t count = std::min(inSpan.size(), outSpan.size());
        std::copy_n(inSpan.begin(), count, outSpan.begin());
        for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
            const std::size_t offset = relIndex < 0 ? 0UZ : static_cast<std::size_t>(relIndex);
            if (offset >= count) {
                continue;
            }
            gr::property_map rewritten = tagMapRef.get();
            gr::tag::put(rewritten, gr::tag::SAMPLE_RATE, new_rate.value);
            outSpan.publishTag(std::move(rewritten), offset);
        }
        std::ignore = inSpan.consume(count);
        outSpan.publish(count);
        return gr::work::Status::OK;
    }
};

/// Holds shared I/O pool threads with non-returning tasks for the run's duration, which is the graph configuration
/// `BasicFileSink` deadlocks in: it waits on an I/O-pool task that cannot be scheduled because the pool's threads are
/// all held by tasks that never return.
///
/// The count is bounded rather than the pool's own maximum: that maximum is tens of thousands of threads on this
/// host, so occupying it is neither possible nor what the shipped configuration looks like. A handful of
/// non-returning tasks — a socket sender loop and the scheduler's own watchdog — is what the tree actually runs.
template<typename T>
struct IoPoolHolder : gr::Block<IoPoolHolder<T>> {
    gr::PortIn<T> in;

    GR_MAKE_REFLECTABLE(IoPoolHolder, in);

    static constexpr std::size_t kHeldThreads = 8UZ;

    std::shared_ptr<std::atomic_bool> _release{};
    std::shared_ptr<std::atomic_int>  _running{};

    using gr::Block<IoPoolHolder<T>>::Block;

    void start() {
        _release         = std::make_shared<std::atomic_bool>(false);
        _running         = std::make_shared<std::atomic_int>(0);
        auto       pool  = gr::thread_pool::Manager::defaultIoPool();
        const auto count = std::min(kHeldThreads, static_cast<std::size_t>(pool->maxThreads()));
        for (std::size_t i = 0UZ; i < count; ++i) {
            pool->execute([release = _release, running = _running] {
                running->fetch_add(1, std::memory_order_release);
                while (!release->load(std::memory_order_acquire)) {
                    std::this_thread::sleep_for(1ms);
                }
                running->fetch_sub(1, std::memory_order_release);
            });
        }
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (_running->load(std::memory_order_acquire) < static_cast<int>(count) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }
    }

    void stop() {
        if (_release) {
            _release->store(true, std::memory_order_release);
        }
        if (_running) {
            const auto deadline = std::chrono::steady_clock::now() + 5s;
            while (_running->load(std::memory_order_acquire) > 0 && std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
        }
    }

    constexpr void processOne(const T&) const noexcept {}
};

// ─── minimal spans, for driving a block at an exact chunk size ───────────────────────────────────────────────────

struct TagWriterStub : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterStub() = default;
    constexpr explicit TagWriterStub(std::span<gr::Tag> room) : std::span<gr::Tag>(room) {}
    constexpr void publish(std::size_t) const noexcept {}
};

/// A source under test reads `tags.size()` as the room a span has for tags, so the stub carries a real one.
inline constexpr std::size_t kStubTagRoom = 64UZ;

template<typename T>
struct OutputSpanStub : std::span<T> {
    using value_type = T;

    std::vector<gr::Tag>*             sink{nullptr};
    std::array<gr::Tag, kStubTagRoom> tagRoom{};
    TagWriterStub                     tags{};
    std::size_t                       streamIndex{0UZ};
    std::size_t                       count{0UZ};
    bool                              isConnected{true};
    bool                              isSync{true};

    OutputSpanStub(std::span<T> samples, std::size_t at, std::vector<gr::Tag>* published) : std::span<T>(samples), sink(published), streamIndex(at) { tags = TagWriterStub(std::span<gr::Tag>(tagRoom)); }

    constexpr void publish(std::size_t nSamples) noexcept { count = nSamples; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (sink != nullptr) {
            sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
        }
    }
};

template<typename TBlock>
[[nodiscard]] TBlock makeBlock(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

template<typename T>
struct StreamCapture {
    std::vector<T>       samples{};
    std::vector<gr::Tag> tags{};
};

/// Drive a source's `processBulk` at an exact chunk size, without a graph or a scheduler.
template<typename T, typename TBlock>
[[nodiscard]] StreamCapture<T> drainSource(TBlock& block, std::size_t chunkSize, std::size_t callLimit) {
    StreamCapture<T> capture;
    std::vector<T>   scratch(chunkSize);
    for (std::size_t guard = 0UZ; guard < callLimit; ++guard) {
        OutputSpanStub<T> outSpan(std::span<T>(scratch.data(), chunkSize), capture.samples.size(), &capture.tags);
        const auto        status = block.processBulk(outSpan);
        capture.samples.insert(capture.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        if (status == gr::work::Status::DONE) {
            break;
        }
        if (outSpan.count == 0UZ && status != gr::work::Status::OK) {
            break;
        }
    }
    return capture;
}

// ─── graph helpers ───────────────────────────────────────────────────────────────────────────────────────────────

/// Owns the scheduler for as long as the caller inspects the blocks it holds; a reference taken from the graph stays
/// valid only while the scheduler that took ownership is alive.
struct GraphRun {
    std::unique_ptr<gr::scheduler::Simple<>> scheduler{};
    std::expected<void, gr::Error>           result{};

    [[nodiscard]] bool        ok() const { return result.has_value(); }
    [[nodiscard]] std::string message() const { return result ? ""s : result.error().message; }
};

/// Joins on destruction, so an exception leaving `runAndWait` cannot destroy a joinable thread.
struct ThreadJoiner {
    std::thread thread;
    ~ThreadJoiner() {
        if (thread.joinable()) {
            thread.join();
        }
    }
};

[[nodiscard]] GraphRun runGraph(gr::Graph&& flow, std::chrono::seconds timeout = 10s) {
    GraphRun run;
    run.scheduler = std::make_unique<gr::scheduler::Simple<>>();
    if (auto ready = run.scheduler->exchange(std::move(flow)); !ready) {
        run.result = std::unexpected(ready.error());
        return run;
    }
    auto        finished      = std::make_shared<std::atomic_bool>(false);
    auto        watchdogFired = std::make_shared<std::atomic_bool>(false);
    auto* const sched         = run.scheduler.get();

    ThreadJoiner watchdog{std::thread([sched, finished, watchdogFired, timeout] {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (finished->load(std::memory_order_acquire)) {
                return;
            }
            std::this_thread::sleep_for(20ms);
        }
        watchdogFired->store(true, std::memory_order_relaxed);
        sched->requestStop();
    })};

    try {
        run.result = sched->runAndWait();
    } catch (const gr::exception& failure) {
        run.result = std::unexpected(gr::Error(failure.message));
    } catch (const std::exception& failure) {
        run.result = std::unexpected(gr::Error(std::string(failure.what())));
    }
    finished->store(true, std::memory_order_release);
    if (watchdog.thread.joinable()) {
        watchdog.thread.join();
    }
    if (watchdogFired->load(std::memory_order_relaxed)) {
        run.result = std::unexpected(gr::Error("the graph did not finish within its deadline"));
    }
    return run;
}

/// Read a recording through a graph and copy out the samples and tags a downstream sink saw.
template<typename T>
[[nodiscard]] std::expected<StreamCapture<T>, std::string> readRecording(gr::property_map settings, gr::Size_t expected = 0U) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<SigMfSource<T>>(std::move(settings));
    auto&     sink   = flow.emplaceBlock<TagSinkFor<T>>({{"log_tags", true}, {"log_samples", true}, {"n_samples_expected", expected}});
    if (!flow.connect<"out", "in">(source, sink)) {
        return std::unexpected("the ports would not connect"s);
    }
    const GraphRun run = runGraph(std::move(flow));
    if (!run.ok()) {
        return std::unexpected(run.message());
    }
    StreamCapture<T> capture;
    capture.samples.assign(sink._samples.begin(), sink._samples.end());
    capture.tags = sink._tags;
    return capture;
}

/// Start a block the way the framework does and report the refusal it names, or "ok".
///
/// A refusal has to be read here rather than from `runAndWait`: the scheduler logs a block whose `start()` threw and
/// carries on, so the graph's own result does not name the reason. The graph-driven half of the criterion — that a
/// refused block reaches the ERROR state, emits nothing and leaves no file — is asserted separately below.
template<typename TBlock>
[[nodiscard]] std::string startRefusal(gr::property_map settings) {
    try {
        auto block = makeBlock<TBlock>(std::move(settings));
        block.start();
        block.stop();
    } catch (const gr::exception& refused) {
        return refused.message;
    } catch (const std::exception& refused) {
        return std::string(refused.what());
    }
    return "ok"s;
}

[[nodiscard]] std::string sourceRefusal(gr::property_map settings) { return startRefusal<SigMfSource<std::complex<float>>>(std::move(settings)); }

const suite<"SigMF source"> _source = [] {
    "the anchor recording, read"_test = [] {
        const Workspace   workspace{"anchor_read"};
        const std::string base = writeAnchor(workspace, "anchor");

        expect(eq(hexOf(anchorDataBytes()), "00000000000000000000803F00000000000000000000803F000080BF0000000000000000000080BF0000003F0000003F000000BF0000003F0000803E000040BF"s)) << "the dataset is IEEE-754 binary32, interleaved real then imaginary";

        const auto capture = readRecording<std::complex<float>>({{"file_name", base}});
        expect(capture.has_value()) << (capture ? ""s : capture.error());
        expect(eq(capture->samples.size(), 8UZ));
        expect(std::ranges::equal(capture->samples, kAnchorSamples)) << "cf32_le into complex<float> moves bytes and nothing else";

        const auto merged = mergedByIndex(capture->tags);
        expect(eq(merged.size(), 2UZ)) << "one tag index at the capture boundary, one at the annotation, and no other";
        expect(merged.contains(0UZ) && merged.contains(4UZ));

        const gr::property_map& first = merged.at(0UZ);
        expect(tagValue<float>(first, "sample_rate").value_or(0.f) == 48000.f);
        expect(tagValue<double>(first, "frequency").value_or(0.0) == 433921337.0);
        expect(eq(tagValue<std::string>(first, "trigger_name").value_or(""), "sigmf:capture"s));
        expect(tagValue<std::uint64_t>(first, "trigger_time").value_or(0U) == kAnchorTimeNs);
        expect(tagValue<float>(first, "trigger_offset").value_or(1.f) == 0.f);
        expect(eq(tagValue<std::string>(first, keys::kDatatype).value_or(""), "cf32_le"s));
        expect(tagValue<gr::Size_t>(first, "num_channels").value_or(0U) == 1U);

        const gr::property_map& second = merged.at(4UZ);
        expect(eq(tagValue<std::string>(second, keys::kAnnotationLabel).value_or(""), "burst"s));
        expect(tagValue<gr::Size_t>(second, keys::kAnnotationLength).value_or(0U) == 2U);
    };

    "the drop derivation"_test = [] {
        const Workspace workspace{"drop"};

        const auto recordingWith = [&workspace](std::string_view name, std::uint64_t secondGlobalIndex) {
            const std::string base = workspace.base(name);
            writeText(base + ".sigmf-meta", std::format(R"({{"global": {{"core:datatype": "cf32_le", "core:version": "1.2.0"}}, "captures": [{{"core:sample_start": 0, "core:global_index": 1000}}, {{"core:sample_start": 4096, "core:global_index": {}}}], "annotations": []}})", secondGlobalIndex));
            writeBytes(base + ".sigmf-data", std::vector<std::byte>(4097UZ * sizeof(std::complex<float>), std::byte{}));
            return base;
        };

        const auto dropped = readRecording<std::complex<float>>({{"file_name", recordingWith("dropped", 5300U)}});
        expect(dropped.has_value()) << (dropped ? ""s : dropped.error());
        const auto droppedTags = mergedByIndex(dropped->tags);
        expect(droppedTags.contains(4096UZ));
        expect(tagValue<gr::Size_t>(droppedTags.at(4096UZ), "n_dropped_samples").value_or(0U) == 204U) << "5300 - (1000 + 4096) = 204";

        const auto contiguous = readRecording<std::complex<float>>({{"file_name", recordingWith("contiguous", 5096U)}});
        expect(contiguous.has_value()) << (contiguous ? ""s : contiguous.error());
        const auto quietTags = mergedByIndex(contiguous->tags);
        expect(quietTags.contains(4096UZ));
        expect(!tagValue<gr::Size_t>(quietTags.at(4096UZ), "n_dropped_samples").has_value()) << "contiguous segments publish no drop count";

        expect(mentions(sourceRefusal({{"file_name", recordingWith("regressed", 5000U)}}), "global_index_regression"));
    };

    "truncation, both policies"_test = [] {
        const Workspace   workspace{"truncation"};
        const std::string base = writeAnchor(workspace, "short");

        std::vector<std::byte> shortData = anchorDataBytes();
        shortData.resize(40UZ); // five whole complex samples, where the annotation implies six
        writeBytes(base + ".sigmf-data", shortData);

        const std::string refused = sourceRefusal({{"file_name", base}});
        expect(mentions(refused, "data_truncated")) << refused;
        expect(mentions(refused, "implies 6")) << refused;
        expect(mentions(refused, "holds 5")) << refused;
        expect(mentions(refused, "shortfall of 1")) << refused;

        const auto allowed = readRecording<std::complex<float>>({{"file_name", base}, {"truncation", std::string("allow")}});
        expect(allowed.has_value()) << (allowed ? ""s : allowed.error());
        expect(eq(allowed->samples.size(), 5UZ));
        expect(mergedByIndex(allowed->tags).contains(4UZ)) << "the annotation inside the emitted range still appears";

        std::vector<std::byte> partial = anchorDataBytes();
        partial.resize(60UZ); // not a whole multiple of the eight-byte item stride
        writeBytes(base + ".sigmf-data", partial);
        expect(mentions(sourceRefusal({{"file_name", base}}), "data_not_whole_samples"));
        expect(mentions(sourceRefusal({{"file_name", base}, {"truncation", std::string("allow")}}), "data_not_whole_samples")) << "a partial sample is refused in both modes";
    };

    "multi-channel recordings interleave with a count"_test = [] {
        const Workspace   workspace{"multichannel"};
        const std::string base = workspace.base("stereo");
        writeText(base + ".sigmf-meta", R"({"global": {"core:datatype": "ri16_le", "core:version": "1.2.0", "core:num_channels": 2}, "captures": [{"core:sample_start": 0}, {"core:sample_start": 128}], "annotations": []})");
        writeBytes(base + ".sigmf-data", std::vector<std::byte>(129UZ * 2UZ * sizeof(std::int16_t), std::byte{}));

        const auto capture = readRecording<std::int16_t>({{"file_name", base}});
        expect(capture.has_value()) << (capture ? ""s : capture.error());
        expect(eq(capture->samples.size(), 258UZ)) << "129 samples of two interleaved channels are 258 items";
        const auto merged = mergedByIndex(capture->tags);
        expect(merged.contains(0UZ));
        expect(tagValue<gr::Size_t>(merged.at(0UZ), "num_channels").value_or(0U) == 2U);
        expect(merged.contains(256UZ)) << "a capture at sample 128 of two channels is stream index 256";
    };

    "an empty captures array streams as one segment from sample zero"_test = [] {
        const Workspace   workspace{"implied_capture"};
        const std::string base = workspace.base("implied");
        writeText(base + ".sigmf-meta", R"({"global": {"core:datatype": "cf32_le", "core:version": "1.2.6", "core:sample_rate": 48000}, "captures": [], "annotations": []})");
        writeBytes(base + ".sigmf-data", anchorDataBytes());

        const auto capture = readRecording<std::complex<float>>({{"file_name", base}});
        expect(capture.has_value()) << (capture ? ""s : capture.error());
        expect(std::ranges::equal(capture->samples, kAnchorSamples)) << "every sample is described by the implied segment";

        const auto merged = mergedByIndex(capture->tags);
        expect(eq(merged.size(), 1UZ)) << "one segment implies one boundary and no more";
        expect(merged.contains(0UZ)) << "and it sits at sample zero";
        expect(eq(tagValue<std::string>(merged.at(0UZ), "trigger_name").value_or(""), "sigmf:capture"s));
        expect(eq(tagValue<float>(merged.at(0UZ), "sample_rate").value_or(0.f), 48000.f));
        expect(!tagValue<std::uint64_t>(merged.at(0UZ), "trigger_time").has_value()) << "the implied segment states no time, and none is invented on read";
    };

    "repeat republishes the schedule at each wrap"_test = [] {
        const Workspace   workspace{"repeat"};
        const std::string base = writeAnchor(workspace, "looped");

        const auto capture = readRecording<std::complex<float>>({{"file_name", base}, {"repeat", true}}, 24U);
        expect(capture.has_value()) << (capture ? ""s : capture.error());
        expect(eq(capture->samples.size(), 24UZ));

        std::vector<std::size_t> boundaries;
        for (const auto& [index, map] : mergedByIndex(capture->tags)) {
            if (tagValue<std::string>(map, "trigger_name").has_value()) {
                boundaries.push_back(index);
            }
            expect(!tagValue<gr::Size_t>(map, "n_dropped_samples").has_value()) << "a wrap drops nothing; the recording ended and began again";
        }
        expect(std::ranges::equal(boundaries, std::vector<std::size_t>{0UZ, 8UZ, 16UZ}));
    };

    "the 2026-08-28 tolerances, each counted"_test = [] {
        const Workspace workspace{"tolerances"};

        const auto unsortedAnnotations = [&workspace](std::string_view name, std::size_t count) {
            const std::string base = workspace.base(name);
            std::string       body;
            for (std::size_t i = 0UZ; i < count; ++i) {
                body.append(std::format(R"({}{{"core:sample_start": {}}})", i == 0UZ ? "" : ", ", count - i));
            }
            writeText(base + ".sigmf-meta", std::format(R"({{"global": {{"core:datatype": "cf32_le", "core:version": "1.2.0"}}, "captures": [{{"core:sample_start": 0}}], "annotations": [{}]}})", body));
            writeBytes(base + ".sigmf-data", std::vector<std::byte>((count + 1UZ) * sizeof(std::complex<float>), std::byte{}));
            return base;
        };

        {
            // driven without a graph: a thousand tags in one stream exceed what a port's tag ring carries between
            // consumers, and the assertion here is about the repair the reader made, not about that limit
            auto block = makeBlock<SigMfSource<std::complex<float>>>({{"file_name", unsortedAnnotations("repairable", 1024UZ)}});
            block.start();
            const auto capture = drainSource<std::complex<float>>(block, 4096UZ, 256UZ);
            block.stop();

            expect(eq(block.nAnnotationsSorted, std::uint64_t{1U})) << "an unsorted array within the bound is sorted, counted and reported";
            expect(eq(block.nAnnotationsCollided, std::uint64_t{0U})) << "and nothing is dropped by the repair";
            expect(eq(capture.samples.size(), 1025UZ));
            expect(eq(capture.tags.size(), 1025UZ)) << "one capture boundary and a thousand and twenty-four annotations";
            std::vector<std::size_t> indices;
            indices.reserve(capture.tags.size());
            for (const gr::Tag& tag : capture.tags) {
                indices.push_back(tag.index);
            }
            expect(std::ranges::is_sorted(indices)) << "the emitted schedule is the sorted document's";
            expect(eq(indices.front(), 0UZ));
            expect(eq(indices.back(), 1024UZ));
        }
        expect(mentions(sourceRefusal({{"file_name", unsortedAnnotations("refused", 1025UZ)}}), "annotations_unsorted"));

        const std::string gapBase = workspace.base("leading_gap");
        writeText(gapBase + ".sigmf-meta", R"({"global": {"core:datatype": "cf32_le", "core:version": "1.2.0", "core:offset": 0}, "captures": [{"core:sample_start": 96}], "annotations": [{"core:sample_start": 4, "core:label": "inside the gap"}, {"core:sample_start": 100, "core:label": "after it"}]})");
        writeBytes(gapBase + ".sigmf-data", std::vector<std::byte>(104UZ * sizeof(std::complex<float>), std::byte{}));

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", gapBase}});
        auto&     sink   = flow.emplaceBlock<TagSinkFor<std::complex<float>>>({{"log_tags", true}, {"log_samples", true}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << "the leading gap is skipped, not refused: " << run.message();
        expect(eq(source.nLeadingItemsSkipped, std::uint64_t{96U}));
        expect(eq(source.nSamplesEmitted, std::uint64_t{8U})) << "the emitted count excludes the gap";
        expect(eq(source.nAnnotationsCollided, std::uint64_t{1U})) << "the annotation inside the gap is dropped and counted";

        const auto merged = mergedByIndex(sink._tags);
        expect(merged.contains(0UZ)) << "the first capture boundary is stream index 0";
        expect(merged.contains(4UZ));
        expect(eq(tagValue<std::string>(merged.at(4UZ), keys::kAnnotationLabel).value_or(""), "after it"s));
    };

    "chunk independence"_test = [] {
        const Workspace   workspace{"chunks"};
        const std::string base = writeAnchor(workspace, "chunked");

        std::optional<StreamCapture<std::complex<float>>> reference;
        for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 4096UZ}) {
            auto block = makeBlock<SigMfSource<std::complex<float>>>({{"file_name", base}});
            block.start();
            const auto capture = drainSource<std::complex<float>>(block, chunk, 64UZ);
            block.stop();
            if (!reference) {
                reference = capture;
                expect(eq(capture.samples.size(), 8UZ));
                continue;
            }
            expect(std::ranges::equal(capture.samples, reference->samples)) << std::format("samples differ at a chunk size of {}", chunk);
            expect(eq(capture.tags.size(), reference->tags.size())) << std::format("tag count differs at a chunk size of {}", chunk);
            for (std::size_t i = 0UZ; i < std::min(capture.tags.size(), reference->tags.size()); ++i) {
                expect(eq(capture.tags[i].index, reference->tags[i].index)) << std::format("tag {} moved at a chunk size of {}", i, chunk);
            }
        }
    };

    "every source refusal, driven through a real graph"_test = [] {
        const Workspace workspace{"refusals"};

        expect(mentions(sourceRefusal({{"file_name", std::string("")}}), "no_file_name"));
        expect(mentions(sourceRefusal({{"file_name", workspace.base("x.sigmf")}}), "archive_unsupported"));
        expect(mentions(sourceRefusal({{"file_name", workspace.base("absent")}}), "meta_not_found"));

        const std::string lonely = workspace.base("lonely");
        writeText(lonely + ".sigmf-meta", R"({"global": {"core:datatype": "cf32_le", "core:version": "1.2.0"}, "captures": [{"core:sample_start": 0}], "annotations": []})");
        expect(mentions(sourceRefusal({{"file_name", lonely}}), "data_not_found"));

        const std::string anchor = writeAnchor(workspace, "anchor");
        expect(mentions(sourceRefusal({{"file_name", anchor}, {"max_metadata_bytes", gr::Size_t{16U}}}), "meta_too_large"));

        const std::string nonconforming = workspace.base("nonconforming");
        writeText(nonconforming + ".sigmf-meta", R"({"global": {"core:datatype": "cf32_le", "core:version": "1.2.0", "core:dataset": "nowhere.bin"}, "captures": [{"core:sample_start": 0}], "annotations": []})");
        expect(mentions(sourceRefusal({{"file_name", nonconforming}}), "nonconforming_dataset"));

        const std::string unreadable = workspace.base("unreadable");
        std::filesystem::create_directories(unreadable + ".sigmf-meta");
        expect(mentions(sourceRefusal({{"file_name", unreadable}}), "path_unreadable"));

        const std::string realFormat = workspace.base("real");
        writeText(realFormat + ".sigmf-meta", R"({"global": {"core:datatype": "rf32_le", "core:version": "1.2.0"}, "captures": [{"core:sample_start": 0}], "annotations": []})");
        writeBytes(realFormat + ".sigmf-data", anchorDataBytes());
        expect(mentions(sourceRefusal({{"file_name", realFormat}}), "datatype_domain")) << "a real datatype is not a complex stream, and neither conversion is invented";

        const std::string broken = workspace.base("broken");
        writeText(broken + ".sigmf-meta", "{ this is not JSON");
        writeBytes(broken + ".sigmf-data", anchorDataBytes());
        expect(mentions(sourceRefusal({{"file_name", broken}}), "meta_unparsable"));

        const std::string future = workspace.base("future");
        writeText(future + ".sigmf-meta", R"({"global": {"core:datatype": "cf32_le", "core:version": "2.0.0"}, "captures": [{"core:sample_start": 0}], "annotations": []})");
        writeBytes(future + ".sigmf-data", anchorDataBytes());
        expect(mentions(sourceRefusal({{"file_name", future}}), "future_version"));

        const std::string wide = workspace.base("wide");
        writeText(wide + ".sigmf-meta", R"({"global": {"core:datatype": "ci64_le", "core:version": "1.2.0"}, "captures": [{"core:sample_start": 0}], "annotations": []})");
        writeBytes(wide + ".sigmf-data", anchorDataBytes());
        expect(mentions(sourceRefusal({{"file_name", wide}}), "datatype_unparsable")) << "a 64-bit integer spelling is not in the grammar, so it is refused as any unknown spelling is";

        const std::string bulky = workspace.base("bulky");
        const std::string padding(4096UZ, 'x');
        writeText(bulky + ".sigmf-meta", std::format(R"({{"global": {{"core:datatype": "cf32_le", "core:version": "1.2.0", "vendor:note": "{}"}}, "captures": [{{"core:sample_start": 0}}], "annotations": []}})", padding));
        writeBytes(bulky + ".sigmf-data", anchorDataBytes());
        expect(mentions(sourceRefusal({{"file_name", bulky}, {"max_extra_bytes", gr::Size_t{64U}}}), "extras_too_large")) << "a carried document is refused, never truncated";
        expect(eq(sourceRefusal({{"file_name", bulky}, {"carry_unknown_fields", false}}), "ok"s)) << "discarding is a stated decision the operator can make";

        expect(!std::filesystem::exists(workspace.base("absent") + ".sigmf-data")) << "a refusal creates no file";
    };

    "an integer datatype of another width is refused by name"_test = [] {
        const Workspace   workspace{"integer_width"};
        const std::string base = workspace.base("sixteen");
        writeText(base + ".sigmf-meta", R"({"global": {"core:datatype": "ri16_le", "core:version": "1.2.0"}, "captures": [{"core:sample_start": 0}], "annotations": []})");
        writeBytes(base + ".sigmf-data", std::vector<std::byte>(16UZ, std::byte{}));

        expect(mentions(startRefusal<SigMfSource<std::int32_t>>({{"file_name", base}}), "integer_width"));
    };

    "a refused source reaches the error state and emits nothing"_test = [] {
        const Workspace   workspace{"refused_graph"};
        const std::string base = workspace.base("truncated");
        writeText(base + ".sigmf-meta", kAnchorMetadata);
        std::vector<std::byte> shortData = anchorDataBytes();
        shortData.resize(16UZ);
        writeBytes(base + ".sigmf-data", shortData);

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", base}});
        auto&     sink   = flow.emplaceBlock<TagSinkFor<std::complex<float>>>({{"log_samples", true}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        std::ignore        = run.ok(); // the scheduler logs a refused start and carries on; the block state is the signal

        expect(eq(source.nSamplesEmitted, std::uint64_t{0U})) << "a block that cannot do its job refuses to start";
        expect(eq(sink._samples.size(), 0UZ)) << "and nothing downstream has samples to account for";
    };
};

const suite<"SigMF sink"> _sink = [] {
    "the anchor recording, written"_test = [] {
        const Workspace workspace{"anchor_write"};

        {
            gr::Graph flow;
            auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
            source._values.assign(kAnchorSamples.begin(), kAnchorSamples.end());
            source._tags.emplace_back(0UZ, anchorIndexZeroTag());
            source._tags.emplace_back(4UZ, anchorAnnotationTag());
            auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("anchor")}});
            expect(flow.connect<"out", "in">(source, sink).has_value());
            const GraphRun run = runGraph(std::move(flow));
            expect(run.ok()) << run.message();
            expect(eq(sink.nSamplesClipped, std::uint64_t{0U}));
        }
        expect(eq(readText(workspace.base("anchor") + ".sigmf-meta"), std::string(kAnchorMetadata))) << "the 422 metadata bytes, exactly";
        expect(eq(hexOf(readBytes(workspace.base("anchor") + ".sigmf-data")), hexOf(anchorDataBytes()))) << "the 64 dataset bytes, exactly";

        {
            gr::Graph flow;
            auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
            source._values.assign(kAnchorSamples.begin(), kAnchorSamples.end());
            source._tags.emplace_back(0UZ, anchorIndexZeroTag());
            source._tags.emplace_back(4UZ, anchorAnnotationTag());
            auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("narrowed")}, {"datatype", std::string("ci16_le")}});
            expect(flow.connect<"out", "in">(source, sink).has_value());
            const GraphRun run = runGraph(std::move(flow));
            expect(run.ok()) << run.message();
            expect(eq(sink.nSamplesClipped, std::uint64_t{2U})) << "+1.0 is not representable, at either width";
        }
        expect(eq(hexOf(readBytes(workspace.base("narrowed") + ".sigmf-data")), "00000000FF7F00000000FF7F00800000000000800040004000C00040002000A0"s)) << "anchor B's thirty-two bytes";

        std::string expectedMetadata(kAnchorMetadata);
        expectedMetadata.replace(expectedMetadata.find("cf32_le"), "cf32_le"sv.size(), "ci16_le");
        expect(eq(readText(workspace.base("narrowed") + ".sigmf-meta"), expectedMetadata)) << "the metadata differs only in core:datatype";
    };

    "a rate that changes cannot be recorded, and the sink says so"_test = [] {
        const Workspace workspace{"rate_changes"};

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
        source._values.assign(kAnchorSamples.begin(), kAnchorSamples.end());
        for (const auto& [index, rate] : std::array<std::pair<std::size_t, float>, 3>{std::pair<std::size_t, float>{0UZ, 48000.f}, std::pair<std::size_t, float>{4UZ, 96000.f}, std::pair<std::size_t, float>{6UZ, 192000.f}}) {
            gr::property_map map;
            gr::tag::put(map, gr::tag::SAMPLE_RATE, rate);
            source._tags.emplace_back(index, std::move(map));
        }
        auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("rates")}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << run.message();
        expect(eq(sink.nSampleRateChangesIgnored, std::uint64_t{2U}));

        expect(mentions(readText(workspace.base("rates") + ".sigmf-meta"), R"("core:sample_rate": 48000)")) << "the recording keeps the first rate and means it";
    };

    "every sink refusal leaves the directory as it found it"_test = [] {
        const Workspace workspace{"sink_refusals"};

        const auto refusal = [](gr::property_map settings) { return startRefusal<SigMfSink<std::complex<float>>>(std::move(settings)); };

        expect(mentions(refusal({{"file_name", std::string("")}}), "no_file_name"));
        expect(mentions(refusal({{"file_name", workspace.base("x.sigmf")}}), "archive_unsupported"));
        expect(mentions(refusal({{"file_name", workspace.base("out")}, {"datatype", std::string("cx32_le")}}), "datatype_unknown"));
        expect(mentions(refusal({{"file_name", workspace.base("out")}, {"datatype", std::string("rf32_le")}}), "datatype_domain"));
        expect(!std::filesystem::exists(workspace.base("out") + ".sigmf-data")) << "a refused datatype touches no file";

        const std::string existing = workspace.base("existing");
        writeText(existing + ".sigmf-data", "keep me");
        expect(mentions(refusal({{"file_name", existing}}), "exists"));
        expect(eq(readText(existing + ".sigmf-data"), "keep me"s)) << "the existing recording is byte-identical afterwards";
        expect(eq(refusal({{"file_name", existing}, {"overwrite", true}}), "ok"s));

        const std::string blocked = workspace.base("blocked");
        writeText(blocked, "a file where a directory would have to be");
        expect(mentions(refusal({{"file_name", blocked + "/inner"}}), "path_unwritable"));
    };

    "a non-finite frequency is refused on write"_test = [] {
        const Workspace workspace{"non_finite"};

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
        source._values.assign(kAnchorSamples.begin(), kAnchorSamples.end());
        gr::property_map map;
        gr::tag::put(map, gr::tag::FREQUENCY, std::numeric_limits<double>::infinity());
        source._tags.emplace_back(0UZ, std::move(map));
        auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("infinite")}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        std::ignore        = run.ok();

        expect(!std::filesystem::exists(workspace.base("infinite") + ".sigmf-meta")) << "a frequency of infinity is not a recording anyone can use, and no document is left claiming it";
    };

    "a stream carrying a drop count reproduces the derivation"_test = [] {
        const Workspace workspace{"drop_write"};

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
        source._values.assign(4097UZ, std::complex<float>{});
        source._tags.emplace_back(0UZ, anchorIndexZeroTag());
        gr::property_map dropTag;
        gr::tag::put(dropTag, gr::tag::TRIGGER_NAME, std::string("sigmf:capture"));
        gr::tag::put(dropTag, gr::tag::N_DROPPED_SAMPLES, gr::Size_t{204U});
        source._tags.emplace_back(4096UZ, std::move(dropTag));
        auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("dropped")}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << run.message();

        const std::string metadata = readText(workspace.base("dropped") + ".sigmf-meta");
        expect(mentions(metadata, R"("core:sample_start": 4096)"));
        expect(mentions(metadata, R"("core:global_index": 4300)")) << "the counter advances by the items written plus the samples the hardware lost";
        expect(mentions(metadata, R"("core:global_index": 0)")) << "and the first segment anchors it at what this run observed";
    };

    "the metadata is rewritten on a period"_test = [] {
        const Workspace workspace{"rewrite"};

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
        source._values.assign(64UZ, std::complex<float>{});
        auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("rolling")}, {"meta_rewrite_period", gr::Size_t{8U}}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << run.message();

        const std::string metadata = readText(workspace.base("rolling") + ".sigmf-meta");
        expect(!metadata.empty()) << "a run interrupted after a rewrite leaves a readable pair";
        expect(mentions(metadata, R"("core:datatype": "cf32_le")"));
    };

    "max_bytes ends the recording cleanly"_test = [] {
        const Workspace workspace{"max_bytes"};

        gr::Graph flow;
        auto&     source = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
        source._values.assign(kAnchorSamples.begin(), kAnchorSamples.end());
        auto& sink = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", workspace.base("capped")}, {"max_bytes", std::uint64_t{32U}}});
        expect(flow.connect<"out", "in">(source, sink).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << run.message();
        expect(eq(sink.nSamplesWritten, std::uint64_t{4U}));
        expect(eq(readBytes(workspace.base("capped") + ".sigmf-data").size(), 32UZ));
    };
};

const suite<"SigMF round trip"> _roundTrip = [] {
    "source into sink, with no intermediate block"_test = [] {
        const Workspace   workspace{"round_trip"};
        const std::string source = writeAnchor(workspace, "in");
        const std::string target = workspace.base("out");

        gr::Graph flow;
        auto&     reader = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", source}});
        auto&     writer = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", target}});
        expect(flow.connect<"out", "in">(reader, writer).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << run.message();

        expect(eq(hexOf(readBytes(target + ".sigmf-data")), hexOf(anchorDataBytes()))) << "every sample bit-equal";
        expect(eq(readText(target + ".sigmf-meta"), std::string(kAnchorMetadata))) << "and the metadata byte-identical";
    };

    "an intermediate block keeps the reserved six and loses the sigmf_ eleven"_test = [] {
        const Workspace   workspace{"round_trip_chain"};
        const std::string source = writeAnchor(workspace, "in");
        const std::string target = workspace.base("out");

        gr::Graph flow;
        auto&     reader = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", source}});
        auto&     middle = flow.emplaceBlock<FilteringPassthrough<std::complex<float>>>();
        auto&     writer = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", target}});
        expect(flow.connect<"out", "in">(reader, middle).has_value());
        expect(flow.connect<"out", "in">(middle, writer).has_value());
        const GraphRun run = runGraph(std::move(flow));
        expect(run.ok()) << run.message();

        const std::string metadata = readText(target + ".sigmf-meta");
        expect(mentions(metadata, R"("core:sample_rate": 48000)")) << "the reserved keys survive the default forwarder";
        expect(mentions(metadata, R"("core:frequency": 433921337)"));
        expect(mentions(metadata, R"("core:datetime": "2026-08-26T12:00:00.000000Z")"));
        expect(mentions(metadata, R"("annotations": [])")) << "and the sigmf_ keys do not, which is the contract until the retrofit reaches the block in between";
    };

    "the rate reconciliation, both branches"_test = [] {
        const Workspace workspace{"rate_reconciliation"};

        std::string awkward(kAnchorMetadata);
        awkward.replace(awkward.find("48000"), "48000"sv.size(), "61440001");
        const std::string source = writeAnchor(workspace, "awkward", awkward);

        {
            const std::string target = workspace.base("straight");
            gr::Graph         flow;
            auto&             reader = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", source}});
            auto&             writer = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", target}});
            expect(flow.connect<"out", "in">(reader, writer).has_value());
            const GraphRun run = runGraph(std::move(flow));
            expect(run.ok()) << run.message();

            expect(eq(reader.nSampleRateNarrowed, std::uint64_t{1U})) << "61440001 does not survive the float";
            expect(eq(writer.nCarriedRateSuperseded, std::uint64_t{0U}));
            expect(mentions(readText(target + ".sigmf-meta"), R"("core:sample_rate": 61440001)")) << "the carried original wins when the stream confirms nothing touched it";
        }

        {
            const std::string target = workspace.base("resampled");
            gr::Graph         flow;
            auto&             reader   = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", source}});
            auto&             rewriter = flow.emplaceBlock<RateRewriter<std::complex<float>>>({{"new_rate", 30720000.f}});
            auto&             writer   = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", target}});
            expect(flow.connect<"out", "in">(reader, rewriter).has_value());
            expect(flow.connect<"out", "in">(rewriter, writer).has_value());
            const GraphRun run = runGraph(std::move(flow));
            expect(run.ok()) << run.message();

            expect(eq(writer.nCarriedRateSuperseded, std::uint64_t{1U})) << "the observed rate wins the moment anything touched it";
            expect(mentions(readText(target + ".sigmf-meta"), R"("core:sample_rate": 30720000)"));
        }
    };
};

const suite<"SigMF lifecycle and I/O discipline"> _lifecycle = [] {
    "start and stop leave no descriptor behind"_test = [] {
        const Workspace   workspace{"lifecycle"};
        const std::string base   = writeAnchor(workspace, "cycled");
        const std::string target = workspace.base("written");

        const auto openDescriptors = [] {
            std::error_code status;
            std::size_t     count = 0UZ;
            for (auto it = std::filesystem::directory_iterator("/proc/self/fd", status); !status && it != std::filesystem::directory_iterator{}; it.increment(status)) {
                ++count;
            }
            return count;
        };

        auto reader = makeBlock<SigMfSource<std::complex<float>>>({{"file_name", base}});
        auto writer = makeBlock<SigMfSink<std::complex<float>>>({{"file_name", target}, {"overwrite", true}});

        reader.start();
        reader.stop();
        writer.start();
        writer.stop();
        const std::size_t before = openDescriptors();
        for (std::size_t i = 0UZ; i < 10UZ; ++i) {
            reader.start();
            reader.stop();
            writer.start();
            writer.stop();
        }
        expect(eq(openDescriptors(), before)) << "ten start/stop cycles on each block leave the descriptor count unchanged";
        expect(!readText(target + ".sigmf-meta").empty()) << "a sink stopped without a sample still describes what it wrote";
    };

    "no shared pool is touched"_test = [] {
        const Workspace   workspace{"pool"};
        const std::string base = writeAnchor(workspace, "contended");

        for (std::size_t attempt = 0UZ; attempt < 20UZ; ++attempt) {
            const std::string target = workspace.base(std::format("out_{}", attempt));

            gr::Graph flow;
            auto&     reader = flow.emplaceBlock<SigMfSource<std::complex<float>>>({{"file_name", base}});
            auto&     writer = flow.emplaceBlock<SigMfSink<std::complex<float>>>({{"file_name", target}});
            expect(flow.connect<"out", "in">(reader, writer).has_value());

            auto& ticker = flow.emplaceBlock<ScriptedSource<std::complex<float>>>();
            ticker._values.assign(64UZ, std::complex<float>{});
            auto& holder = flow.emplaceBlock<IoPoolHolder<std::complex<float>>>();
            expect(flow.connect<"out", "in">(ticker, holder).has_value());

            const GraphRun run = runGraph(std::move(flow), 8s);
            expect(run.ok()) << std::format("attempt {} did not complete: {}", attempt, run.message());
            expect(eq(hexOf(readBytes(target + ".sigmf-data")), hexOf(anchorDataBytes()))) << std::format("attempt {} wrote the wrong bytes", attempt);
        }
    };
};

} // namespace

int main(int argc, char* argv[]) {
    if (argc > 0 && argv[0] != nullptr) {
        gProgramName = argv[0];
    }
    return 0;
}
