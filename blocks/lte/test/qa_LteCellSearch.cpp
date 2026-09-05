#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/rng/GaussianNoise.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/channel/Multipath.hpp>
#include <gnuradio-4.0/lte/CellSearch.hpp>

#include "LteDownlinkScene.hpp"

using namespace boost::ut;
using gr::blocks::lte::CellSearch;
using gr::lte::CyclicPrefix;
using gr::lte::DuplexMode;
using gr::test::lte::makeDownlink;
using gr::test::lte::Scene;
using gr::test::lte::SceneConfig;

namespace {

using Complex = std::complex<float>;

/// A source that hands the graph a fixed stream, at most `_chunk` samples per call so that a test can pin the
/// scheduler's window rather than take whatever the buffer happens to offer.
struct ChunkedSource : gr::Block<ChunkedSource> {
    gr::PortOut<Complex> out;
    GR_MAKE_REFLECTABLE(ChunkedSource, out);

    std::vector<Complex>                                  _data{};
    std::size_t                                           _chunk{0UZ}; ///< 0 takes whatever the span offers
    std::vector<std::pair<std::size_t, gr::property_map>> _tags{};
    std::size_t                                           _at{0UZ};

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        std::size_t n = std::min(outSpan.size(), _data.size() - _at);
        if (_chunk != 0UZ) {
            n = std::min(n, _chunk);
        }
        if (n == 0UZ) {
            outSpan.publish(0UZ);
            return _at == _data.size() ? gr::work::Status::DONE : gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        for (const auto& [index, map] : _tags) {
            if (index >= _at && index < _at + n) {
                outSpan.publishTag(map, index - _at);
            }
        }
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_at), n, outSpan.begin());
        outSpan.publish(n);
        _at += n;
        return _at == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<float>> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// What the channel does to the scene before the block sees it; every field's zero is the identity.
struct Impairments {
    double                  frequencyOffsetHz{0.};
    double                  noisePower{0.}; ///< linear mean power per sample; the scene's own mean power is one
    std::uint64_t           seed{1ULL};
    std::vector<gr::Size_t> delays{}; ///< empty leaves the channel out of the graph
    std::vector<double>     powersDb{};
};

/// Everything one run of the block reported.
struct Run {
    std::vector<gr::DataSet<float>>             records{};
    std::uint64_t                               samples{0ULL};
    std::uint64_t                               halfFrames{0ULL};
    std::uint64_t                               halfFramesSkipped{0ULL};
    std::uint64_t                               positions{0ULL};
    std::uint64_t                               pssDetections{0ULL};
    std::uint64_t                               sssRejected{0ULL};
    std::uint64_t                               secondaryOffStream{0ULL};
    std::uint64_t                               published{0ULL};
    std::uint64_t                               rateRefused{0ULL};
    std::uint64_t                               tagsDropped{0ULL};
    std::uint64_t                               searchRebuilds{0ULL};
    std::array<double, CellSearch::kSlotValues> slot{};
    std::uint64_t                               slotFilled{0ULL};
    std::uint64_t                               watchReads{0ULL};        ///< slot reads another thread took while the graph ran
    std::uint64_t                               watchFilled{0ULL};       ///< of those, the ones that carried a record
    std::uint64_t                               watchInconsistent{0ULL}; ///< of those, the ones whose nine values disagreed
};

/// A block between the source and the identifier substitutes its own value into any tag naming a setting it also
/// has, so a test about what the identifier does with a tag wires the source straight to it.
enum class Chain : std::uint8_t { Channel = 0, Bare };

[[nodiscard]] Run drive(std::vector<Complex> samples, gr::property_map settings, const Impairments& channel = {}, std::size_t chunk = 0UZ, std::vector<std::pair<std::size_t, gr::property_map>> tags = {}, Chain chain = Chain::Channel, bool watch = false) {
    settings.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(gr::lte::kSampleRate));

    gr::Graph flow;
    auto&     source = flow.emplaceBlock<ChunkedSource>();
    source._data     = std::move(samples);
    source._chunk    = chunk;
    source._tags     = std::move(tags);
    auto& search     = flow.emplaceBlock<CellSearch>(settings);
    auto& sink       = flow.emplaceBlock<RecordSink>();

    // The scene is the transmitted signal; the channel is delay spread first, then the carrier offset, then the
    // noise, so a stated per-sample ratio is against a signal of unit mean power however the taps are arranged.
    if (chain == Chain::Bare) {
        expect(flow.connect<"out", "in">(source, search).has_value());
    } else {
        auto& carrier = flow.emplaceBlock<gr::blocks::channel::FrequencyOffset<Complex>>({{"sample_rate", gr::lte::kSampleRate}, {"frequency_offset", channel.frequencyOffsetHz}});
        if (channel.delays.empty()) {
            expect(flow.connect<"out", "in">(source, carrier).has_value());
        } else {
            auto& fading = flow.emplaceBlock<gr::blocks::channel::FadingChannel<Complex>>({{"sample_rate", gr::lte::kSampleRate}, {"delays", channel.delays}, {"powers_db", channel.powersDb}, {"max_doppler", 0.}, {"k_factor", 1e6}, {"normalize", true}, {"seed", channel.seed}});
            expect(flow.connect<"out", "in">(source, fading).has_value());
            expect(flow.connect<"out", "in">(fading, carrier).has_value());
        }
        auto& noise = flow.emplaceBlock<gr::blocks::channel::AwgnChannel<Complex>>({{"noise_power", channel.noisePower}, {"seed", channel.seed}});
        expect(flow.connect<"out", "in">(carrier, noise).has_value());
        expect(flow.connect<"out", "in">(noise, search).has_value());
    }
    expect(flow.connect<"out", "in">(search, sink).has_value());

    Run                     run;
    gr::scheduler::Simple<> scheduler;
    expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });

    // The measurement slot is read from a thread of its own while the graph runs, which is the only way to test
    // that a reader outside the owning thread sees nine values that belong to one record rather than a mixture.
    std::thread reader;
    if (watch) {
        reader = std::thread([&search = search, &done, &run] {
            while (!done.load()) {
                const auto [values, count] = search._slot.read();
                ++run.watchReads;
                if (count == 0ULL) {
                    continue;
                }
                ++run.watchFilled;
                const auto identity = static_cast<std::uint32_t>(values[CellSearch::kCellIdAt]);
                const auto group    = static_cast<std::uint32_t>(values[CellSearch::kNId1At]);
                const auto root     = static_cast<std::uint32_t>(values[CellSearch::kNId2At]);
                if (identity != 3U * group + root || group > 167U || root > 2U || !(values[CellSearch::kSssMetricAt] > 0.)) {
                    ++run.watchInconsistent;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        });
    }
    const auto start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(300)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!done.load()) {
        scheduler.requestStop();
        expect(false) << "the graph did not finish within five minutes";
    }
    runner.join();
    if (reader.joinable()) {
        reader.join();
    }

    run.records            = sink._records;
    run.samples            = search.nSamples;
    run.halfFrames         = search.nHalfFrames;
    run.halfFramesSkipped  = search.nHalfFramesSkipped;
    run.positions          = search.nPositions;
    run.pssDetections      = search.nPssDetections;
    run.sssRejected        = search.nSssRejected;
    run.secondaryOffStream = search.nSecondaryOffStream;
    run.published          = search.nPublished;
    run.rateRefused        = search.nRateRefused;
    run.tagsDropped        = search.nTagsDropped;
    run.searchRebuilds     = search.nSearchRebuilds;
    for (std::size_t i = 0UZ; i < CellSearch::kSlotValues; ++i) {
        run.slot[i] = 0.;
    }
    run.slot[CellSearch::kCellIdAt]     = static_cast<double>(search.cellId());
    run.slot[CellSearch::kNId1At]       = static_cast<double>(search.nId1());
    run.slot[CellSearch::kNId2At]       = static_cast<double>(search.nId2());
    run.slot[CellSearch::kFrameStartAt] = static_cast<double>(search.frameStart());
    run.slot[CellSearch::kFrequencyAt]  = search.frequencyOffsetHz();
    run.slot[CellSearch::kPssMetricAt]  = search.pssMetric();
    run.slot[CellSearch::kSssMetricAt]  = search.sssMetric();
    run.slot[CellSearch::kDuplexAt]     = static_cast<double>(search.duplex());
    run.slot[CellSearch::kCyclicAt]     = static_cast<double>(search.cyclicPrefix());
    run.slotFilled                      = search.nRecords();
    return run;
}

[[nodiscard]] const gr::property_map& metaOf(const gr::DataSet<float>& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

[[nodiscard]] std::uint64_t metaU64(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? ~0ULL : entry->second.value_or(std::uint64_t{~0ULL});
}

[[nodiscard]] gr::Size_t metaSize(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<gr::Size_t>::max() : entry->second.value_or(std::numeric_limits<gr::Size_t>::max());
}

[[nodiscard]] float metaFloat(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::numeric_limits<float>::quiet_NaN() : entry->second.value_or(std::numeric_limits<float>::quiet_NaN());
}

[[nodiscard]] std::string metaString(const gr::DataSet<float>& record, std::string_view key) {
    const gr::property_map& map   = metaOf(record);
    const auto              entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

/// Candidate primary positions a stream of @p samples samples holds: every one of them but the last 127, at which
/// the stream no longer carries a whole primary symbol. This is the invariant, not a count of whole windows — the
/// block examines the stream's last positions as one short window rather than dropping them.
[[nodiscard]] std::size_t evaluatedPositions(std::size_t samples) { return samples < gr::lte::kSymbolSamples ? 0UZ : samples - (gr::lte::kSymbolSamples - 1UZ); }

/// The primary symbols of @p scene the block can evaluate: those whose 128 useful samples the stream carries whole.
[[nodiscard]] std::size_t reachablePrimaries(const Scene& scene, std::size_t samples) {
    return static_cast<std::size_t>(std::ranges::count_if(scene.pss, [samples](std::size_t p) { return p + gr::lte::kSymbolSamples <= samples; }));
}

/// What a reported carrier offset may be on a clean scene. The two half-symbol correlations the estimate is built
/// from are not orthogonal to the ten subcarriers outside the sequence, though the full-symbol correlation is, so
/// the scene sets a deterministic floor no signal-to-noise ratio removes — 530 Hz for root 25, 228 Hz for the other
/// two — and the tolerance is that bound plus the noise term.
constexpr float kOffsetToleranceHz = 600.f;

/// The noise power a stated per-sample signal-to-noise ratio implies against the scene's unit mean power.
[[nodiscard]] double noisePowerFor(double snrDb) { return std::pow(10., -snrDb / 10.); }

/// Check one record against the scene it came from; returns the empty string when everything matched.
[[nodiscard]] std::string mismatch(const gr::DataSet<float>& record, const Scene& scene, const SceneConfig& config, float toleranceHz) {
    const std::uint64_t position = metaU64(record, "pss_position");
    const auto          at       = std::ranges::find(scene.pss, static_cast<std::size_t>(position));
    if (at == scene.pss.end()) {
        return std::format("primary position {} is not one of the scene's", position);
    }
    const std::size_t index = static_cast<std::size_t>(std::distance(scene.pss.begin(), at));

    std::string wrong;
    const auto  check = [&wrong](bool ok, std::string text) {
        if (!ok) {
            wrong += (wrong.empty() ? "" : "; ") + text;
        }
    };
    const gr::Size_t expectedId = gr::lte::cellIdentity(config.nId1, config.nId2);
    check(metaSize(record, "cell_id") == expectedId, std::format("cell_id {} wanted {}", metaSize(record, "cell_id"), expectedId));
    check(metaSize(record, "n_id_1") == config.nId1, std::format("n_id_1 {} wanted {}", metaSize(record, "n_id_1"), config.nId1));
    check(metaSize(record, "n_id_2") == config.nId2, std::format("n_id_2 {} wanted {}", metaSize(record, "n_id_2"), config.nId2));
    check(metaString(record, "duplex") == (config.duplex == DuplexMode::Tdd ? "tdd" : "fdd"), std::format("duplex '{}'", metaString(record, "duplex")));
    check(metaString(record, "cyclic_prefix") == (config.cyclicPrefix == CyclicPrefix::Extended ? "extended" : "normal"), std::format("cyclic_prefix '{}'", metaString(record, "cyclic_prefix")));
    check(metaSize(record, "half_frame") == scene.halfFrame[index], std::format("half_frame {} wanted {}", metaSize(record, "half_frame"), scene.halfFrame[index]));
    check(metaU64(record, "frame_start") == scene.frameStart[index], std::format("frame_start {} wanted {}", metaU64(record, "frame_start"), scene.frameStart[index]));
    check(std::abs(metaFloat(record, "frequency_offset_hz")) < toleranceHz, std::format("frequency_offset_hz {:.1f}", metaFloat(record, "frequency_offset_hz")));
    check(metaFloat(record, "sss_metric") > 0.9f, std::format("sss_metric {:.4f}", metaFloat(record, "sss_metric")));
    check(metaFloat(record, "sample_rate") == gr::lte::kSampleRate, "sample_rate");
    check(record.signal_values.size() == gr::lte::kSignalLength, std::format("{} soft values", record.signal_values.size()));
    check(record.extents.size() == 1UZ && record.extents[0UZ] == static_cast<std::int32_t>(gr::lte::kSignalLength), "extents");
    check(record.signal_names.size() == 1UZ && record.signal_names[0UZ] == "sss", "signal_names");
    return wrong;
}

} // namespace

const boost::ut::suite<"LteCellSearch"> _lteCellSearch = [] {
    "a clean downlink identifies exactly, under every structure"_test = [] {
        // Twelve groups spread over the range, including the two either side of the closed form's first fold and
        // both ends, against all three roots: 36 cells under each of the four structures.
        const std::array<std::uint32_t, 12UZ> groups{0U, 1U, 17U, 29U, 30U, 31U, 60U, 83U, 100U, 137U, 166U, 167U};
        gr::rng::Xoshiro256pp                 rng(0xc0ffeeULL);
        std::size_t                           runs          = 0UZ;
        std::size_t                           records       = 0UZ;
        float                                 worstOffset   = 0.f;
        double                                squaredOffset = 0.;

        for (const gr::lte::FrameGeometry& geometry : gr::lte::kStructures) {
            for (const std::uint32_t group : groups) {
                for (std::uint32_t root = 0U; root < 3U; ++root) {
                    SceneConfig config;
                    config.nId1         = group;
                    config.nId2         = root;
                    config.duplex       = geometry.duplex;
                    config.cyclicPrefix = geometry.cyclicPrefix;
                    config.frames       = 2UZ;
                    config.timingOffset = rng() % gr::lte::kFrameSamples;
                    config.seed         = 0x1000ULL + group * 3U + root;

                    const Scene scene = makeDownlink(config);
                    const Run   run   = drive(scene.samples, {}, Impairments{0., noisePowerFor(30.), config.seed, {}, {}});

                    // Two radio frames carry four primary symbols and the block reports all four: the positions the
                    // whole windows leave over are examined as one short window rather than dropped.
                    const std::size_t expected = reachablePrimaries(scene, scene.samples.size());
                    expect(eq(expected, scene.pss.size())) << "a two-frame scene puts every primary symbol inside the stream";
                    expect(eq(run.records.size(), expected)) << std::format("cell ({},{}) offset {}: one record per primary symbol the stream carries", config.nId1, config.nId2, config.timingOffset);
                    expect(eq(run.positions, evaluatedPositions(scene.samples.size()))) << std::format("cell ({},{}) offset {}: every position evaluated exactly once", config.nId1, config.nId2, config.timingOffset);
                    for (const gr::DataSet<float>& record : run.records) {
                        const std::string wrong = mismatch(record, scene, config, kOffsetToleranceHz);
                        expect(wrong.empty()) << std::format("cell ({},{}) offset {}: {}", config.nId1, config.nId2, config.timingOffset, wrong);
                        const float reported = metaFloat(record, "frequency_offset_hz");
                        worstOffset          = std::max(worstOffset, std::abs(reported));
                        squaredOffset += static_cast<double>(reported) * static_cast<double>(reported);
                        ++records;
                    }
                    ++runs;
                }
            }
        }
        expect(eq(runs, 4UZ * 36UZ)) << "36 cells under each of the four structures";
        std::println("criterion 2: {} records over {} scenes at 30 dB, reported carrier offset {:.1f} Hz rms and {:.1f} Hz at the extreme", records, runs, records == 0UZ ? 0. : std::sqrt(squaredOffset / static_cast<double>(records)), worstOffset);
    };

    "every position is evaluated exactly once, the stream's last ones included"_test = [] {
        SceneConfig config;
        config.nId1       = 44U;
        config.nId2       = 0U;
        config.frames     = 2UZ;
        config.seed       = 0xf200ULL;
        const Scene scene = makeDownlink(config);

        // Lengths either side of a whole window's own boundary: one that ends exactly on it, one that leaves a
        // single position over, one that leaves the whole of a primary symbol's tail, and one that ends mid-stream.
        constexpr std::size_t whole = gr::lte::kHalfFrameSamples + gr::lte::kSymbolSamples - 1UZ; // 9727, one window's worth
        for (const std::size_t length : {whole, whole + 1UZ, whole + gr::lte::kSymbolSamples - 1UZ, 2UZ * gr::lte::kHalfFrameSamples + gr::lte::kSymbolSamples - 1UZ, 24'000UZ, scene.samples.size()}) {
            const std::vector<Complex> cut(scene.samples.begin(), scene.samples.begin() + static_cast<std::ptrdiff_t>(length));
            for (const std::size_t chunk : {0UZ, 137UZ, 9601UZ}) {
                const Run run = drive(cut, {}, Impairments{0., noisePowerFor(30.), 0xf300ULL, {}, {}}, chunk);
                expect(eq(run.samples, length)) << std::format("length {} chunk {}: every sample read", length, chunk);
                expect(eq(run.positions, evaluatedPositions(length))) << std::format("length {} chunk {}: every position but the last 127 evaluated, each once", length, chunk);
                expect(eq(run.records.size(), reachablePrimaries(scene, length))) << std::format("length {} chunk {}: every primary symbol the stream carries whole is reported", length, chunk);
            }
        }
    };

    "a secondary symbol before the stream is refused rather than read from the pad"_test = [] {
        // Unpaired spectrum with the extended prefix reaches 480 samples back for its secondary symbol, so a
        // stream cut one sample into that symbol carries a primary symbol whose identity cannot be decided.
        SceneConfig config;
        config.nId1         = 91U;
        config.nId2         = 0U;
        config.duplex       = DuplexMode::Tdd;
        config.cyclicPrefix = CyclicPrefix::Extended;
        config.frames       = 2UZ;
        config.seed         = 0xf400ULL;
        const Scene scene   = makeDownlink(config);

        const gr::lte::FrameGeometry geometry{DuplexMode::Tdd, CyclicPrefix::Extended};
        const std::size_t            first     = scene.pss[0UZ];
        const auto                   secondary = static_cast<std::size_t>(static_cast<std::ptrdiff_t>(first) + geometry.secondaryOffset());
        const std::size_t            cut       = secondary + 1UZ;
        const std::vector<Complex>   truncated(scene.samples.begin() + static_cast<std::ptrdiff_t>(cut), scene.samples.end());
        const std::size_t            at = first - cut;
        expect(at < gr::lte::kMaxSecondaryLookBehind) << "the primary symbol sits inside the look-behind the structure needs";

        // The instrument, verified positive: the kernel knows nothing of a stream's beginning and reads the one
        // missing sample out of the window's own pad, on which it still confirms the cell.
        std::vector<Complex> window(gr::lte::kMaxSecondaryLookBehind, Complex(0.f, 0.f));
        window.insert(window.end(), truncated.begin(), truncated.begin() + static_cast<std::ptrdiff_t>(gr::lte::CellDetector::windowFor(gr::lte::kHalfFrameSamples) - gr::lte::kMaxSecondaryLookBehind));
        gr::lte::CellDetector                         detector(0.f);
        gr::lte::ExamineCounts                        counts;
        const std::span<const gr::lte::CellDetection> found  = detector.examine(window, gr::lte::kHalfFrameSamples, 20.f, 0.5f, counts);
        const bool                                    padded = std::ranges::any_of(found, [at](const gr::lte::CellDetection& detection) { return detection.primary.position == at && detection.secondary.geometry.duplex == DuplexMode::Tdd && detection.secondary.geometry.cyclicPrefix == CyclicPrefix::Extended; });
        expect(padded) << "the kernel confirms the cell from the pad, which is what the block has to refuse";

        const Run run = drive(truncated, {}, {}, 0UZ, {}, Chain::Bare);
        expect(eq(run.secondaryOffStream, 1ULL)) << "and the block refuses that detection, counting it";
        for (const gr::DataSet<float>& record : run.records) {
            expect(metaU64(record, "pss_position") != at) << "no identity is decided on samples the stream does not carry";
        }
        expect(!run.records.empty()) << "while every later primary symbol is still reported";
    };

    "a setting changed mid-stream invalidates only what it is an input to"_test = [] {
        SceneConfig config;
        config.nId1       = 66U;
        config.nId2       = 1U;
        config.frames     = 4UZ;
        config.seed       = 0xf500ULL;
        const Scene scene = makeDownlink(config);

        const auto sequences = [](const Run& run) {
            std::vector<std::uint64_t> out;
            for (const gr::DataSet<float>& record : run.records) {
                out.push_back(metaU64(record, "sequence"));
            }
            return out;
        };
        const auto contiguous = [](const std::vector<std::uint64_t>& values) {
            for (std::size_t i = 0UZ; i < values.size(); ++i) {
                if (values[i] != static_cast<std::uint64_t>(i)) {
                    return false;
                }
            }
            return !values.empty();
        };

        const Run untouched = drive(scene.samples, {}, {}, 0UZ, {}, Chain::Bare);
        expect(contiguous(sequences(untouched))) << "the undisturbed run numbers its records from zero without a gap";
        expect(eq(untouched.searchRebuilds, 0ULL)) << "and never rebuilds the hypotheses";

        // A threshold is read where it is used, so a tag carrying one leaves the window, the counters and the
        // record numbering exactly where it found them.
        std::vector<std::pair<std::size_t, gr::property_map>> threshold;
        threshold.emplace_back(12'000UZ, gr::property_map{{gr::property_map::key_type("pss_threshold"), gr::pmt::Value(25.f)}});
        const Run tagged = drive(scene.samples, {}, {}, 0UZ, threshold, Chain::Bare);
        expect(eq(tagged.samples, untouched.samples)) << "the buffered stream survives a threshold change";
        expect(eq(tagged.halfFrames, untouched.halfFrames));
        expect(eq(tagged.positions, untouched.positions)) << "and no position is evaluated twice or lost";
        expect(eq(tagged.searchRebuilds, 0ULL)) << "a threshold is not an input to the frequency hypotheses";
        expect(contiguous(sequences(tagged))) << "and the record numbering runs on";
        expect(eq(tagged.records.size(), untouched.records.size())) << "a threshold of 25 still admits this cell";

        // The search width is an input to them, so changing it rebuilds them - and nothing else.
        std::vector<std::pair<std::size_t, gr::property_map>> width;
        width.emplace_back(12'000UZ, gr::property_map{{gr::property_map::key_type("frequency_search_hz"), gr::pmt::Value(15'000.f)}});
        const Run widened = drive(scene.samples, {}, {}, 0UZ, width, Chain::Bare);
        expect(eq(widened.searchRebuilds, 1ULL)) << "the width is, and rebuilds them once";
        expect(eq(widened.samples, untouched.samples)) << "while the stream is neither dropped nor re-read";
        expect(eq(widened.positions, untouched.positions));
        expect(contiguous(sequences(widened)));
    };

    "the primary position and the frame start keep the structure's own distance"_test = [] {
        for (const gr::lte::FrameGeometry& geometry : gr::lte::kStructures) {
            for (const std::size_t offset : {0UZ, 1UZ, 959UZ, 9599UZ, 13337UZ}) {
                SceneConfig config;
                config.nId1         = 55U;
                config.nId2         = 2U;
                config.duplex       = geometry.duplex;
                config.cyclicPrefix = geometry.cyclicPrefix;
                config.frames       = 2UZ;
                config.timingOffset = offset;
                config.seed         = 0x2000ULL + offset;

                const Scene scene = makeDownlink(config);
                const Run   run   = drive(scene.samples, {}, Impairments{0., noisePowerFor(30.), config.seed, {}, {}});
                expect(!run.records.empty()) << std::format("offset {} produced records", offset);
                for (const gr::DataSet<float>& record : run.records) {
                    const std::string wrong = mismatch(record, scene, config, kOffsetToleranceHz);
                    expect(wrong.empty()) << std::format("offset {}: {}", offset, wrong);
                    const std::int64_t distance = static_cast<std::int64_t>(metaU64(record, "pss_position")) - static_cast<std::int64_t>(metaU64(record, "frame_start"));
                    expect(eq(distance, -geometry.frameStartOffset(metaSize(record, "half_frame")))) << std::format("offset {}: the primary symbol sits the structure's own distance into the frame", offset);
                }
            }
        }
    };

    "a carrier offset inside one hypothesis is read back"_test = [] {
        // The hundred draws are a hundred half-frames of one continuous stream rather than a hundred graphs: the
        // noise is drawn afresh for each of them either way, and a graph costs more to build than a half-frame
        // costs to search.
        SceneConfig config;
        config.nId1                = 12U;
        config.nId2                = 0U;
        config.frames              = 50UZ;
        config.seed                = 0x3000ULL;
        const Scene       scene    = makeDownlink(config);
        const std::size_t expected = reachablePrimaries(scene, scene.samples.size());

        for (const double offset : {7000., -7000.}) {
            const Run run     = drive(scene.samples, {}, Impairments{offset, noisePowerFor(10.), 0x4000ULL, {}, {}});
            double    squared = 0.;
            for (const gr::DataSet<float>& record : run.records) {
                expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(12U, 0U))) << std::format("offset {} Hz", offset);
                const double error = static_cast<double>(metaFloat(record, "frequency_offset_hz")) - offset;
                squared += error * error;
            }
            expect(eq(run.records.size(), expected)) << std::format("every half-frame at {} Hz identified", offset);
            const double rms = run.records.empty() ? 0. : std::sqrt(squared / static_cast<double>(run.records.size()));
            std::println("criterion 4a: offset {:+.0f} Hz at 10 dB, {} draws, rms error {:.1f} Hz", offset, run.records.size(), rms);
            expect(rms <= 380.) << std::format("offset {} Hz rms error {:.1f} within 380 Hz", offset, rms);
        }
    };

    "a carrier offset beyond one hypothesis needs the search, and is refused without it"_test = [] {
        SceneConfig config;
        config.nId1       = 12U;
        config.nId2       = 1U;
        config.frames     = 1UZ;
        config.seed       = 0x3100ULL;
        const Scene scene = makeDownlink(config);

        const Run searched = drive(scene.samples, {{"frequency_search_hz", 45'000.f}}, Impairments{40'000., noisePowerFor(10.), 0x4100ULL, {}, {}});
        expect(eq(searched.records.size(), reachablePrimaries(scene, scene.samples.size()))) << "a +/-45 kHz search finds a 40 kHz offset in every primary symbol the stream carries";
        for (const gr::DataSet<float>& record : searched.records) {
            expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(12U, 1U)));
            const float reported = metaFloat(record, "frequency_offset_hz");
            std::println("criterion 4b: 40 kHz offset read back as {:.1f} Hz", reported);
            expect(std::abs(reported - 40'000.f) <= 380.f) << std::format("reported {:.1f} Hz", reported);
            // 40 kHz is 5.33 steps of 7.5 kHz, so the 37.5 kHz hypothesis wins and the fractional part carries 2.5 kHz
            expect(std::abs(reported - 37'500.f) < 4'000.f) << "the hypothesis the fraction is measured from";
        }

        const Run unsearched = drive(scene.samples, {}, Impairments{40'000., noisePowerFor(10.), 0x4100ULL, {}, {}});
        expect(eq(unsearched.records.size(), 0UZ)) << "no record without the search";
        expect(eq(unsearched.pssDetections, 0ULL)) << "and no primary detection either: the arm that could not see the signal took that path";
        expect(unsearched.halfFrames > 0ULL) << "while still examining the stream";
    };

    "noise alone reports nothing"_test = [] {
        // Ten seconds of noise, driven through the kernel so that the largest metric the search saw can be
        // reported: the block publishes records, not the statistic it rejected them on.
        const std::size_t             total = 10UZ * static_cast<std::size_t>(gr::lte::kSampleRate);
        gr::rng::Xoshiro256pp         rng(0x5000ULL);
        gr::rng::GaussianNoise<float> draw(rng);
        std::vector<Complex>          noise(total);
        for (Complex& sample : noise) {
            sample = draw.complexSample();
        }

        // The wide search takes a shorter run: a maximum of exponentially distributed correlation powers grows as
        // the logarithm of how many were drawn, so ten seconds of it is its one-second figure plus ln(10) = 2.3,
        // and thirteen times the arithmetic to watch that happen buys nothing the arithmetic does not already say.
        for (const auto& [width, seconds] : {std::pair{0.f, 10UZ}, std::pair{45'000.f, 1UZ}}) {
            gr::lte::PssCorrelator correlator(width);
            constexpr std::size_t  window   = gr::lte::kMaxSecondaryLookBehind + gr::lte::kHalfFrameSamples + gr::lte::kSymbolSamples - 1UZ;
            const std::size_t      span     = std::min(noise.size(), seconds * static_cast<std::size_t>(gr::lte::kSampleRate));
            float                  worst    = 0.f;
            std::size_t            examined = 0UZ;
            for (std::size_t at = 0UZ; at + window <= span; at += gr::lte::kHalfFrameSamples) {
                const std::span<const Complex>               positions(noise.data() + at + gr::lte::kMaxSecondaryLookBehind, window - gr::lte::kMaxSecondaryLookBehind);
                const std::array<gr::lte::PssDetection, 3UZ> found = correlator.search(positions, gr::lte::kHalfFrameSamples);
                for (const gr::lte::PssDetection& detection : found) {
                    worst = std::max(worst, detection.metric);
                }
                ++examined;
            }
            const double drawn    = static_cast<double>(examined) * 3. * static_cast<double>(gr::lte::kHalfFrameSamples) * static_cast<double>(correlator.hypotheses());
            const double expected = std::log(drawn) + 0.5772;
            std::println("criterion 5: {} hypotheses, {} s, {} half-frames, {:.3g} correlation powers drawn, largest metric {:.2f} against the ln(N)+gamma expectation {:.2f}", correlator.hypotheses(), seconds, examined, drawn, worst, expected);
            expect(static_cast<double>(worst) < expected + 6.) << std::format("the largest metric sits where the extreme-value expectation puts it, got {:.2f} against {:.2f}", worst, expected);
        }

        // and the block itself publishes nothing over a second of the same noise, through the whole path
        const std::vector<Complex> second(noise.begin(), noise.begin() + static_cast<std::ptrdiff_t>(gr::lte::kSampleRate));
        const Run                  narrow = drive(second, {}, {});
        expect(eq(narrow.published, 0ULL)) << "nothing published over noise at one hypothesis";
        expect(eq(narrow.records.size(), 0UZ));
        const Run wide = drive(second, {{"frequency_search_hz", 45'000.f}}, {});
        expect(eq(wide.published, 0ULL)) << "nor over thirteen";
    };

    "the sweep says where identification holds"_test = [] {
        SceneConfig config;
        config.nId1                = 88U;
        config.nId2                = 2U;
        config.frames              = 50UZ;
        config.seed                = 0x6000ULL;
        const Scene       scene    = makeDownlink(config);
        const gr::Size_t  identity = gr::lte::cellIdentity(88U, 2U);
        const std::size_t trials   = reachablePrimaries(scene, scene.samples.size());

        std::println("criterion 6: SNR sweep, cell {} paired normal, {} half-frames each carrying one primary symbol", identity, trials);
        for (const double snrDb : {-10., -7., -5., 0., 5., 10.}) {
            const Run   run   = drive(scene.samples, {}, Impairments{0., noisePowerFor(snrDb), 0x7000ULL, {}, {}});
            std::size_t right = 0UZ;
            std::size_t wrong = 0UZ;
            for (const gr::DataSet<float>& record : run.records) {
                (metaSize(record, "cell_id") == identity ? right : wrong)++;
            }
            const std::size_t none = trials > right + wrong ? trials - right - wrong : 0UZ;
            std::println("  {:+5.1f} dB: identified {:3}, wrong {:3}, not reported {:3}", snrDb, right, wrong, none);
            if (snrDb >= 0.) {
                expect(eq(right, trials)) << std::format("{} dB identifies every half-frame", snrDb);
            }
            expect(eq(wrong, 0UZ)) << std::format("{} dB: a reported identity is never a wrong one", snrDb);
        }
    };

    "two cells on different roots are both reported, or the stronger alone"_test = [] {
        SceneConfig first;
        first.nId1         = 30U;
        first.nId2         = 0U;
        first.frames       = 2UZ;
        first.seed         = 0x8000ULL;
        SceneConfig second = first;
        second.nId1        = 77U;
        second.nId2        = 1U;
        second.seed        = 0x8100ULL;

        const Scene a = makeDownlink(first);
        const Scene b = makeDownlink(second);
        // Equal power, the second cell's frame 3000 samples behind the first's; the sum is what one antenna sees.
        const std::vector<Complex> mixed = gr::test::lte::mix(a.samples, b.samples, 3000UZ, 1.f);

        const Run all = drive(mixed, {{"report", std::string("all")}}, Impairments{0., noisePowerFor(10.), 0x8200ULL, {}, {}});
        expect(all.records.size() >= 4UZ) << std::format("two cells over the examined half-frames, got {}", all.records.size());
        std::size_t sawFirst = 0UZ;
        std::size_t sawOther = 0UZ;
        for (const gr::DataSet<float>& record : all.records) {
            if (metaSize(record, "n_id_2") == 0U) {
                ++sawFirst;
                expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(30U, 0U)));
            } else {
                ++sawOther;
                expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(77U, 1U)));
            }
        }
        expect(sawFirst > 0UZ && sawOther > 0UZ) << "both cells reported under 'all'";

        const Run best = drive(mixed, {{"report", std::string("best")}}, Impairments{0., noisePowerFor(10.), 0x8200ULL, {}, {}});
        expect(best.records.size() < all.records.size()) << std::format("'best' keeps one of each half-frame's, got {} against {}", best.records.size(), all.records.size());
        for (const gr::DataSet<float>& record : best.records) {
            // The half-frame a record came from is the one its own primary symbol sits in, and 'best' kept the
            // strongest of the records 'all' reported from that same half-frame.
            const std::uint64_t half       = metaU64(record, "pss_position") / gr::lte::kHalfFrameSamples;
            float               strongest  = 0.f;
            std::size_t         candidates = 0UZ;
            for (const gr::DataSet<float>& candidate : all.records) {
                if (metaU64(candidate, "pss_position") / gr::lte::kHalfFrameSamples == half) {
                    strongest = std::max(strongest, metaFloat(candidate, "pss_metric"));
                    ++candidates;
                }
            }
            expect(candidates > 0UZ) << std::format("half-frame {} was reported under 'all' too", half);
            expect(metaFloat(record, "pss_metric") >= strongest - 1e-3f) << std::format("half-frame {}: 'best' kept the strongest of its {} records", half, candidates);
            expect(std::ranges::count_if(best.records, [half](const gr::DataSet<float>& other) { return metaU64(other, "pss_position") / gr::lte::kHalfFrameSamples == half; }) == 1) << std::format("half-frame {} produced one record under 'best'", half);
        }
    };

    "two cells on one root are one detection, and the stronger wins it"_test = [] {
        SceneConfig strong;
        strong.nId1      = 5U;
        strong.nId2      = 2U;
        strong.frames    = 2UZ;
        strong.seed      = 0x9000ULL;
        SceneConfig weak = strong;
        weak.nId1        = 140U;
        weak.seed        = 0x9100ULL;

        const Scene                a     = makeDownlink(strong);
        const Scene                b     = makeDownlink(weak);
        const std::vector<Complex> mixed = gr::test::lte::mix(a.samples, b.samples, 3000UZ, std::pow(10.f, -6.f / 20.f));

        const Run run = drive(mixed, {}, Impairments{0., noisePowerFor(10.), 0x9200ULL, {}, {}});
        expect(!run.records.empty());
        for (const gr::DataSet<float>& record : run.records) {
            expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(5U, 2U))) << "the stronger cell is the one identified";
            expect(metaSize(record, "n_id_1") != 140U) << "the weaker cell on the same root is not separated";
        }
    };

    "delay spread does not move the identity"_test = [] {
        SceneConfig config;
        config.nId1       = 64U;
        config.nId2       = 0U;
        config.frames     = 2UZ;
        config.seed       = 0xa000ULL;
        const Scene scene = makeDownlink(config);

        const Impairments channel{0., noisePowerFor(10.), 0xa100ULL, {0U, 4U, 9U}, {0., -6., -10.}};
        const Run         run = drive(scene.samples, {}, channel);
        expect(!run.records.empty()) << "a three-path channel is still identified";
        for (const gr::DataSet<float>& record : run.records) {
            expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(64U, 0U)));
            const std::uint64_t position = metaU64(record, "pss_position");
            const auto          at       = std::ranges::find_if(scene.pss, [position](std::size_t p) { return position >= p && position <= p + 9UZ; });
            expect(at != scene.pss.end()) << std::format("primary position {} sits on one of the paths", position);
            if (at != scene.pss.end()) {
                const std::size_t  index = static_cast<std::size_t>(std::distance(scene.pss.begin(), at));
                const std::int64_t slip  = static_cast<std::int64_t>(metaU64(record, "frame_start")) - static_cast<std::int64_t>(scene.frameStart[index]);
                expect(slip >= 0 && slip <= 9) << std::format("frame start within the delay spread, off by {}", slip);
            }
        }
    };

    "the records do not change with the chunk size"_test = [] {
        SceneConfig config;
        config.nId1         = 0U;
        config.nId2         = 0U;
        config.frames       = 2UZ;
        config.timingOffset = 4321UZ;
        config.seed         = 0xb000ULL;
        const Scene scene   = makeDownlink(config);

        std::vector<gr::DataSet<float>> reference;
        for (const std::size_t chunk : {0UZ, 1UZ, 137UZ, 9600UZ, 9601UZ}) {
            const Run run = drive(scene.samples, {}, Impairments{0., noisePowerFor(30.), 0xb100ULL, {}, {}}, chunk);
            if (chunk == 0UZ) {
                reference = run.records;
                expect(!reference.empty()) << "the reference run produced records";
                continue;
            }
            expect(eq(run.records.size(), reference.size())) << std::format("chunk {}", chunk);
            for (std::size_t i = 0UZ; i < std::min(run.records.size(), reference.size()); ++i) {
                expect(eq(metaU64(run.records[i], "sequence"), metaU64(reference[i], "sequence"))) << std::format("chunk {} record {} sequence", chunk, i);
                expect(eq(metaU64(run.records[i], "frame_start"), metaU64(reference[i], "frame_start"))) << std::format("chunk {} record {} frame start", chunk, i);
                expect(eq(metaU64(run.records[i], "pss_position"), metaU64(reference[i], "pss_position"))) << std::format("chunk {} record {} primary position", chunk, i);
                expect(eq(metaSize(run.records[i], "cell_id"), metaSize(reference[i], "cell_id"))) << std::format("chunk {} record {} identity", chunk, i);
                expect(std::abs(metaFloat(run.records[i], "frequency_offset_hz") - metaFloat(reference[i], "frequency_offset_hz")) < 1.f) << std::format("chunk {} record {} offset", chunk, i);
                expect(that % (run.records[i].signal_values == reference[i].signal_values)) << std::format("chunk {} record {} soft values", chunk, i);
            }
        }
    };

    "the reader carries the last confirmed detection"_test = [] {
        SceneConfig config;
        config.nId1       = 167U;
        config.nId2       = 1U;
        config.frames     = 8UZ;
        config.seed       = 0xc000ULL;
        const Scene scene = makeDownlink(config);
        const Run   run   = drive(scene.samples, {}, Impairments{0., noisePowerFor(30.), 0xc100ULL, {}, {}}, 0UZ, {}, Chain::Channel, true);

        // Read while the graph runs, from a thread that owns nothing: every read either carries no record yet or
        // carries nine values that belong to one record, which is what the slot is for.
        std::println("criterion 11: {} slot reads from another thread while the graph ran, {} of them carrying a record", run.watchReads, run.watchFilled);
        expect(run.watchReads > 0ULL) << "the reader thread ran alongside the graph";
        expect(run.watchFilled > 0ULL) << "and saw records while it did";
        expect(eq(run.watchInconsistent, 0ULL)) << "every read was one record's own nine values";

        expect(!run.records.empty());
        if (run.records.empty()) {
            return;
        }
        const gr::DataSet<float>& last = run.records.back();
        expect(eq(run.slotFilled, run.published)) << "the slot's fill count is the records published";
        expect(eq(static_cast<gr::Size_t>(run.slot[CellSearch::kCellIdAt]), metaSize(last, "cell_id")));
        expect(eq(static_cast<gr::Size_t>(run.slot[CellSearch::kNId1At]), metaSize(last, "n_id_1")));
        expect(eq(static_cast<gr::Size_t>(run.slot[CellSearch::kNId2At]), metaSize(last, "n_id_2")));
        expect(eq(static_cast<std::uint64_t>(run.slot[CellSearch::kFrameStartAt]), metaU64(last, "frame_start")));
        expect(std::abs(run.slot[CellSearch::kFrequencyAt] - static_cast<double>(metaFloat(last, "frequency_offset_hz"))) < 1e-3);
        expect(std::abs(run.slot[CellSearch::kPssMetricAt] - static_cast<double>(metaFloat(last, "pss_metric"))) < 1e-3);
        expect(std::abs(run.slot[CellSearch::kSssMetricAt] - static_cast<double>(metaFloat(last, "sss_metric"))) < 1e-6);
        expect(eq(run.slot[CellSearch::kDuplexAt], metaString(last, "duplex") == "tdd" ? 1. : 0.));
        expect(eq(run.slot[CellSearch::kCyclicAt], metaString(last, "cyclic_prefix") == "extended" ? 1. : 0.));
    };

    "what the block refuses, it refuses by name"_test = [] {
        const auto staged = [](gr::property_map settings) {
            settings.insert_or_assign(gr::property_map::key_type("sample_rate"), gr::pmt::Value(gr::lte::kSampleRate));
            CellSearch block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&staged] { staged({{"frequency_search_hz", 250'000.f}}); })) << "a quarter-megahertz search is refused at staging";
        expect(throws([&staged] { staged({{"search_interval", gr::Size_t(0)}}); })) << "a zero interval is refused at staging";
        expect(throws([&staged] { staged({{"report", std::string("some")}}); })) << "an unknown report rule is refused at staging";
        expect(nothrow([&staged] { staged({{"frequency_search_hz", 200'000.f}}); })) << "the bound itself is allowed";

        for (const float rate : {1'919'999.f, 1'920'001.f}) {
            CellSearch block({{"sample_rate", rate}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            expect(throws([&block] { block.start(); })) << std::format("{} refuses to start", rate);
        }
        expect(nothrow([] {
            CellSearch block({{"sample_rate", gr::lte::kSampleRate}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            block.start();
        })) << "and 1.92 MS/s starts";
    };

    "a rate tag naming another rate is counted and the stream continues"_test = [] {
        SceneConfig config;
        config.nId1       = 3U;
        config.nId2       = 0U;
        config.frames     = 2UZ;
        config.seed       = 0xd000ULL;
        const Scene scene = makeDownlink(config);

        std::vector<std::pair<std::size_t, gr::property_map>> tags;
        tags.emplace_back(12'000UZ, gr::property_map{{gr::property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), gr::pmt::Value(2'000'000.f)}});
        tags.emplace_back(13'000UZ, gr::property_map{{gr::property_map::key_type("trigger_name"), gr::pmt::Value(std::string("elsewhere"))}});

        const Run run = drive(scene.samples, {}, {}, 0UZ, tags, Chain::Bare);
        expect(eq(run.rateRefused, 1ULL)) << "the rate tag is a counted stated drop";
        expect(eq(run.tagsDropped, 1ULL)) << "and the other tag is dropped and counted";
        expect(!run.records.empty()) << "while the stream is read on as 1.92 MS/s";
        for (const gr::DataSet<float>& record : run.records) {
            expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(3U, 0U)));
            expect(eq(metaFloat(record, "sample_rate"), gr::lte::kSampleRate)) << "and the record still says so";
        }
    };

    "a search interval examines one half-frame in N"_test = [] {
        SceneConfig config;
        config.nId1       = 21U;
        config.nId2       = 1U;
        config.frames     = 4UZ;
        config.seed       = 0xe000ULL;
        const Scene scene = makeDownlink(config);

        const Run every = drive(scene.samples, {}, Impairments{0., noisePowerFor(30.), 0xe100ULL, {}, {}});
        const Run third = drive(scene.samples, {{"search_interval", gr::Size_t(3)}}, Impairments{0., noisePowerFor(30.), 0xe100ULL, {}, {}});

        expect(eq(third.halfFrames, every.halfFrames)) << "the same half-frames pass either way";
        expect(eq(third.halfFramesSkipped, third.halfFrames - (third.halfFrames + 2ULL) / 3ULL)) << "two of every three are skipped";
        expect(third.records.size() < every.records.size()) << std::format("{} records against {}", third.records.size(), every.records.size());
        for (const gr::DataSet<float>& record : third.records) {
            expect(eq(metaSize(record, "cell_id"), gr::lte::cellIdentity(21U, 1U)));
            expect(eq(metaU64(record, "pss_position") / gr::lte::kHalfFrameSamples % 3ULL, 0ULL)) << "and every record comes from an examined half-frame";
        }
    };
};

int main() { /* not needed for UT */ }
