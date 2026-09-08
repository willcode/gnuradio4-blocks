#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <string>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/fourier/window.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_spectral {

using gr::blocks::measurement::Spectrogram;
using gr::blocks::measurement::WelchPsd;
using CF = std::complex<float>;

constexpr std::size_t kFft        = 256UZ;
constexpr float       kSampleRate = 48000.f;

/// Emits a fixed sequence in bursts of a stated size, then ends the stream. The burst size is what makes chunk
/// independence testable: the same samples presented differently must produce the same records. `tags` are stamped at
/// their absolute sample index, which is how a setting is moved on a block inside a running graph.
template<typename T>
struct BurstSource : gr::Block<BurstSource<T>> {
    gr::PortOut<T> out;

    std::vector<T>       samples{};
    std::vector<gr::Tag> tags{}; ///< index is an absolute sample index
    std::size_t          burst = 4096UZ;
    std::size_t          at    = 0UZ;

    GR_MAKE_REFLECTABLE(BurstSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        for (const gr::Tag& tag : tags) {
            if (tag.index >= at && tag.index < at + take) {
                outSpan.publishTag(tag.map, tag.index - at);
            }
        }
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

/// Takes at most `stride` records a call, so a small stride puts the block under back-pressure and makes the
/// no-record-lost invariant testable: the records must be the same ones a sink that keeps up receives.
struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::vector<gr::DataSet<float>> records{};
    std::size_t                     stride = std::numeric_limits<std::size_t>::max();

    GR_MAKE_REFLECTABLE(RecordSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        const std::size_t take = std::min(inSpan.size(), stride);
        records.insert(records.end(), inSpan.begin(), inSpan.begin() + static_cast<std::ptrdiff_t>(take));
        std::ignore = inSpan.consume(take);
        return gr::work::Status::OK;
    }
};

[[nodiscard]] std::vector<CF> tone(std::size_t count, double binsPerFft, std::size_t fftSize, float amplitude = 1.f) {
    std::vector<CF>  data(count);
    constexpr double twoPi = 2. * std::numbers::pi;
    for (std::size_t k = 0UZ; k < count; ++k) {
        const double phase = twoPi * binsPerFft * static_cast<double>(k) / static_cast<double>(fftSize);
        data[k]            = CF(amplitude * static_cast<float>(std::cos(phase)), amplitude * static_cast<float>(std::sin(phase)));
    }
    return data;
}

/// Emits a fixed list of density records, then ends the stream. What feeds a detector its input.
struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<gr::DataSet<float>, gr::Async> out;

    std::vector<gr::DataSet<float>> records{};
    std::size_t                     at = 0UZ;

    GR_MAKE_REFLECTABLE(RecordSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= records.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min(records.size() - at, outSpan.size());
        for (std::size_t k = 0UZ; k < take; ++k) {
            outSpan[k] = records[at + k];
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

/// @brief Seeded standard normal pairs, which is what a complex white-noise sample is made of.
struct GaussianNoise {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }

    [[nodiscard]] CF operator()() {
        const double radius = std::sqrt(-2. * std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }
};

/**
 * @brief The five facts `DataSetToStream` admits a record on, in its own order. Returns the reason it would reject
 * the record, or nullptr.
 *
 * That predicate is a private static member of the block, so it cannot be called from here; these are the same five
 * conditions, restated. A record this rejects cannot be read by the Tier-1 consumer, whatever else it carries.
 */
[[nodiscard]] const char* admissionFailure(const gr::DataSet<float>& ds, std::size_t signalIndex = 0UZ) {
    if (ds.signal_names.empty()) {
        return "no_signals";
    }
    if (signalIndex >= ds.signal_names.size()) {
        return "signal_index_out_of_range";
    }
    if (ds.extents.size() != 1UZ) {
        return "not_one_dimensional";
    }
    if (ds.extents[0UZ] <= 0) {
        return "empty_or_negative_extent";
    }
    if (ds.signal_values.size() != ds.signal_names.size() * static_cast<std::size_t>(ds.extents[0UZ])) {
        return "inconsistent_extent";
    }
    return nullptr;
}

[[nodiscard]] double metaNumber(const gr::DataSet<float>& record, std::string_view key, double fallback = -1.) {
    if (record.meta_information.empty()) {
        return fallback;
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return fallback;
    }
    if (const auto* asU64 = it->second.template get_if<std::uint64_t>()) {
        return static_cast<double>(*asU64);
    }
    if (const auto* asFloat = it->second.template get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    if (const auto* asDouble = it->second.template get_if<double>()) {
        return *asDouble;
    }
    return fallback;
}

/// @brief Runs one source-through-block-into-sink graph and returns the records it produced.
template<typename TBlock, typename T>
[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, std::vector<T> samples, std::size_t burst, std::size_t sinkStride = std::numeric_limits<std::size_t>::max()) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<BurstSource<T>>();
    auto&                 block  = test.emplace<TBlock>(std::move(settings));
    auto&                 sink   = test.emplace<RecordSink>();
    source.samples               = std::move(samples);
    source.burst                 = burst;
    sink.stride                  = sinkStride;

    if (!test.connect(source, "out", block, "in").has_value() || !test.connect(block, "out", sink, "in").has_value()) {
        return {};
    }
    std::ignore = test.run(); // the source ends the stream itself, so no wall-clock deadline is involved
    return sink.records;
}

/// @brief Runs a records-in, records-out block over a list of records and returns what it produced.
template<typename TBlock>
[[nodiscard]] std::vector<gr::DataSet<float>> collectFromRecords(gr::property_map settings, std::vector<gr::DataSet<float>> records) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<RecordSource>();
    auto&                 block  = test.emplace<TBlock>(std::move(settings));
    auto&                 sink   = test.emplace<RecordSink>();
    source.records               = std::move(records);

    if (!test.connect(source, "out", block, "in").has_value() || !test.connect(block, "out", sink, "in").has_value()) {
        return {};
    }
    std::ignore = test.run();
    return sink.records;
}

/// @brief The relative variance of a record's bins — the estimator variance Welch's method trades against resolution,
/// read across the bins of white noise, which are independent draws of the same distribution.
[[nodiscard]] double relativeVariance(const gr::DataSet<float>& record) {
    const auto& values = record.signal_values;
    double      mean   = 0.;
    for (const float value : values) {
        mean += static_cast<double>(value);
    }
    mean /= static_cast<double>(values.size());

    double variance = 0.;
    for (const float value : values) {
        const double deviation = static_cast<double>(value) - mean;
        variance += deviation * deviation;
    }
    variance /= static_cast<double>(values.size() - 1UZ);
    return variance / (mean * mean);
}

/// @brief The frequency of the strongest detection in a detection record, in bins of a `fftSize` transform.
[[nodiscard]] double strongestPeakBin(const gr::DataSet<float>& detections, float sampleRate, std::size_t fftSize) {
    const std::size_t n    = detections.signal_values.size() / 3UZ;
    std::size_t       best = 0UZ;
    for (std::size_t k = 1UZ; k < n; ++k) {
        if (detections.signal_values[n + k] > detections.signal_values[n + best]) {
            best = k;
        }
    }
    return static_cast<double>(detections.signal_values[best]) * static_cast<double>(fftSize) / static_cast<double>(sampleRate);
}

/// @brief What a record states its computation cost, beside the wall clock of the single `processBulk` call that
/// produced it. The call brackets everything the stated figure covers and a little more — the segment's copy into the
/// accumulator's buffer and the span bookkeeping — so a truthful figure lies inside the call and is a large part of it.
/// `settings` must name a length and whatever else makes one whole segment complete one record.
///
/// The reading is taken after `warmup` records, because the first record a block makes carries the transform's plan and
/// the first touch of every buffer with it and states a cost tens of times the standing one.
template<typename TBlock>
[[nodiscard]] std::pair<double, double> oneRecordCost(gr::property_map settings, std::size_t fftSize, std::size_t warmup = 2UZ) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();

    const std::vector<CF>           samples = tone(fftSize + 1UZ, 32., fftSize);
    std::vector<gr::DataSet<float>> room(2UZ);

    std::pair<double, double> reading{-1., 0.};
    for (std::size_t pass = 0UZ; pass <= warmup; ++pass) {
        gr::blocks::testing::span::InputSpan<CF>                  in{std::span<const CF>(samples)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> out{std::span<gr::DataSet<float>>(room)};

        const auto started   = std::chrono::steady_clock::now();
        std::ignore          = block.processBulk(in, out);
        const double outside = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

        reading = {out.count == 0UZ ? -1. : metaNumber(room[0UZ], "compute_seconds", -1.), outside};
    }
    return reading;
}

/// @brief The same two figures for a graph that runs one record end to end, which is the reading a consumer of the
/// record can take for itself: the graph's wall clock carries the scheduler's start-up, the source's copies and the
/// record's copy out to the sink as well as the block's own computation.
template<typename TBlock>
[[nodiscard]] std::pair<double, double> oneRecordGraphCost(gr::property_map settings, std::size_t fftSize) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<BurstSource<CF>>();
    auto&                 block  = test.emplace<TBlock>(std::move(settings));
    auto&                 sink   = test.emplace<RecordSink>();
    source.samples               = tone(fftSize + 1UZ, 32., fftSize);
    source.burst                 = 65536UZ;

    if (!test.connect(source, "out", block, "in").has_value() || !test.connect(block, "out", sink, "in").has_value()) {
        return {-1., 0.};
    }
    const auto started   = std::chrono::steady_clock::now();
    std::ignore          = test.run();
    const double outside = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

    return {sink.records.empty() ? -1. : metaNumber(sink.records.front(), "compute_seconds", -1.), outside};
}

} // namespace qa_spectral

const boost::ut::suite<"SpectralEstimate"> spectralTests = [] {
    using namespace boost::ut;
    using namespace qa_spectral;

    "a record states the segment count that made it, and carries the tier's keys"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);

        expect(!records.empty()) << "an eight-segment input at four averages must produce records";
        if (records.empty()) {
            return;
        }
        const auto& first = records.front();
        expect(eq(first.signal_values.size(), kFft)) << "a complex input's record is two-sided";
        expect(eq(metaNumber(first, "n_averaged"), 4.)) << "the record states the segments that went into it";
        expect(approx(metaNumber(first, "overlap"), 0.5, 1e-9)) << "and the fraction they shared, which is what says how many independent looks the count is worth";
        expect(eq(metaNumber(first, "fft_size"), static_cast<double>(kFft)));
        expect(approx(metaNumber(first, "sample_rate"), static_cast<double>(kSampleRate), 1e-3));
        expect(metaNumber(first, "enbw_bins") > 1.) << "a Hann window's noise bandwidth exceeds a bin";
        expect(eq(metaNumber(first, "sample_start"), 0.)) << "the first record starts at the stream's own origin";
        expect(eq(first.axis_names.size(), 1UZ) && eq(first.axis_units.size(), 1UZ));
        expect(eq(first.axis_units[0UZ], std::string("Hz")));
        expect(eq(first.signal_units[0UZ], std::string("1/Hz"))) << "the unit states the stored form, which is linear density";
    };

    "the axis is uniform and runs negative to positive for a complex input"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 4UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        const auto& axis = records.front().axis_values[0UZ];
        expect(eq(axis.size(), kFft));
        const float step = axis[1UZ] - axis[0UZ];
        expect(approx(step, kSampleRate / static_cast<float>(kFft), 1e-3f));
        for (std::size_t k = 1UZ; k < axis.size(); ++k) {
            expect(std::abs((axis[k] - axis[k - 1UZ]) - step) < 1e-6f * std::abs(step) + 1e-6f) << "the axis must be uniform to the landed tolerance";
        }
        expect(axis.front() < 0.f && axis.back() > 0.f) << "a two-sided axis straddles DC";
    };

    "a tone reads its own power through the stated noise-bandwidth correction"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 16UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        const auto& values   = records.front().signal_values;
        const float peak     = *std::ranges::max_element(values);
        const float binWidth = kSampleRate / static_cast<float>(kFft);
        const float power    = peak * static_cast<float>(metaNumber(records.front(), "enbw_bins")) * binWidth;
        expect(std::abs(10.f * std::log10(power)) < 0.6f) << "a full-scale tone reads 0 dBFS within the window's own spreading";
    };

    "chunk independence: the same samples in different bursts give the same records"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 8UZ, 32., kFft);
        const auto             wide    = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        const auto             narrow  = collect<WelchPsd<CF>, CF>(settings, samples, 37UZ);

        expect(eq(wide.size(), narrow.size())) << "the record count cannot depend on how the stream arrived";
        for (std::size_t r = 0UZ; r < std::min(wide.size(), narrow.size()); ++r) {
            expect(eq(metaNumber(wide[r], "sample_start"), metaNumber(narrow[r], "sample_start"))) << "the window grid is stream-absolute";
            expect(eq(wide[r].signal_values.size(), narrow[r].signal_values.size()));
            for (std::size_t k = 0UZ; k < wide[r].signal_values.size(); ++k) {
                expect(wide[r].signal_values[k] == narrow[r].signal_values[k]) << "record " << r << " bin " << k << " differs between chunkings";
            }
        }
    };

    "a real input emits the one-sided record, DC through Nyquist"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        std::vector<float>     samples(kFft * 4UZ);
        for (std::size_t k = 0UZ; k < samples.size(); ++k) {
            samples[k] = std::cos(2.f * std::numbers::pi_v<float> * 32.f * static_cast<float>(k) / static_cast<float>(kFft));
        }
        const auto records = collect<WelchPsd<float>, float>(settings, samples, 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        expect(eq(records.front().signal_values.size(), kFft / 2UZ + 1UZ));
        expect(approx(records.front().axis_values[0UZ].front(), 0.f, 1e-3f)) << "a one-sided axis starts at DC";
        expect(approx(records.front().axis_values[0UZ].back(), kSampleRate / 2.f, 1.f)) << "and ends at Nyquist";
    };

    "the spectrogram emits one record per hop, timestamped by the hop"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             records = collect<Spectrogram<CF>, CF>(settings, tone(kFft * 4UZ, 32., kFft), 4096UZ);
        expect(records.size() >= 3UZ) << "four segments at half overlap yield several rows";
        if (records.size() < 2UZ) {
            return;
        }
        const double hop = static_cast<double>(kFft) / 2.;
        for (std::size_t r = 1UZ; r < records.size(); ++r) {
            expect(eq(metaNumber(records[r], "sample_start") - metaNumber(records[r - 1UZ], "sample_start"), hop)) << "rows advance by exactly one hop";
            expect(eq(metaNumber(records[r], "n_averaged"), 1.)) << "a spectrogram row is one transform";
            expect(approx(metaNumber(records[r], "overlap"), 0.5, 1e-9)) << "and it states the hop it was taken at, as the averaged records do";
        }
    };

    "a hop above fft_size transforms one segment in every hop and consumes the rest"_test = [] {
        // The duty-cycle contract: 256 points out of every 1024 samples. What the block must NOT do is transform the
        // other 768, and what it must still do is count them, so the rows stay where the stream put them.
        constexpr std::size_t  kHop     = 1024UZ;
        constexpr std::size_t  kSamples = kHop * 10UZ;
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{kHop}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kSamples, 32., kFft), 4096UZ);

        expect(eq(records.size(), 10UZ)) << "one segment starts at every hop the stream has room for, made " << records.size();
        for (std::size_t r = 0UZ; r < records.size(); ++r) {
            expect(eq(metaNumber(records[r], "sample_start"), static_cast<double>(r * kHop))) << "record " << r << " lost the samples the hop skipped over";
            expect(eq(metaNumber(records[r], "hop"), static_cast<double>(kHop))) << "the record states the hop it was taken at";
            expect(eq(metaNumber(records[r], "overlap"), 0.)) << "segments a gap apart share nothing";
            expect(eq(metaNumber(records[r], "fft_size"), static_cast<double>(kFft))) << "the transform is the stated length, not the hop";
            expect(eq(records[r].signal_values.size(), kFft));
        }
    };

    "a skipped gap changes nothing about the segments that are transformed"_test = [] {
        // The same stream read at hop 1024 and at hop 256: the hop-1024 records must be bit-identical to every fourth
        // hop-256 record, since each is the same 256 samples through the same window.
        const auto             samples = tone(kFft * 16UZ, 32., kFft);
        const gr::property_map gapped{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{kFft * 4UZ}}, {"sample_rate", kSampleRate}};
        const gr::property_map dense{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};

        const auto sparse = collect<WelchPsd<CF>, CF>(gapped, samples, 4096UZ);
        const auto every  = collect<WelchPsd<CF>, CF>(dense, samples, 4096UZ);
        expect(eq(sparse.size(), 4UZ)) << "sixteen segments' worth of stream holds four segments a gap apart";
        expect(eq(every.size(), 16UZ));
        for (std::size_t r = 0UZ; r < std::min(sparse.size(), every.size() / 4UZ); ++r) {
            const auto& skipped = sparse[r];
            const auto& whole   = every[r * 4UZ];
            expect(eq(metaNumber(skipped, "sample_start"), metaNumber(whole, "sample_start")));
            for (std::size_t k = 0UZ; k < skipped.signal_values.size(); ++k) {
                expect(skipped.signal_values[k] == whole.signal_values[k]) << "record " << r << " bin " << k << " moved because samples were skipped around it";
            }
        }
    };

    "chunk independence holds across a skipped gap"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"hop", gr::Size_t{1000U}}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(12000UZ, 32., kFft);
        const auto             wide    = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        const auto             narrow  = collect<WelchPsd<CF>, CF>(settings, samples, 37UZ);

        expect(!wide.empty()) << "a hop that is not a multiple of anything still has to produce records";
        expect(eq(wide.size(), narrow.size())) << "the record count cannot depend on how the stream arrived";
        for (std::size_t r = 0UZ; r < std::min(wide.size(), narrow.size()); ++r) {
            expect(eq(metaNumber(wide[r], "sample_start"), metaNumber(narrow[r], "sample_start"))) << "record " << r << " moved in the stream";
            for (std::size_t k = 0UZ; k < wide[r].signal_values.size(); ++k) {
                expect(wide[r].signal_values[k] == narrow[r].signal_values[k]) << "record " << r << " bin " << k << " differs between chunkings";
            }
        }
    };

    "the spectrogram's rows may be a gap apart, and stay on the stream's own time axis"_test = [] {
        constexpr std::size_t  kHop = 1024UZ;
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"hop", gr::Size_t{kHop}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<Spectrogram<CF>, CF>(settings, tone(kHop * 6UZ, 32., kFft), 4096UZ);

        expect(eq(records.size(), 6UZ)) << "six hops fit in the stream, made " << records.size();
        for (std::size_t r = 1UZ; r < records.size(); ++r) {
            expect(eq(metaNumber(records[r], "sample_start") - metaNumber(records[r - 1UZ], "sample_start"), static_cast<double>(kHop))) << "rows advance by exactly one hop";
            expect(eq(metaNumber(records[r], "hop"), static_cast<double>(kHop)));
        }
    };

    "a hop below fft_size is an overlap named in samples"_test = [] {
        // The point of the samples spelling: a hop a fraction cannot express. 100 out of 256 is one of them.
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"hop", gr::Size_t{100U}}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);

        expect(records.size() >= 3UZ);
        if (records.size() < 2UZ) {
            return;
        }
        expect(eq(metaNumber(records[1UZ], "sample_start") - metaNumber(records[0UZ], "sample_start"), 100.)) << "the stated hop governs, not the overlap";
        expect(approx(metaNumber(records[0UZ], "hop"), 100., 1e-9));
        expect(approx(metaNumber(records[0UZ], "overlap"), 1. - 100. / static_cast<double>(kFft), 1e-9)) << "and the record states the fraction that hop shares";
    };

    "the hop is live, and moving it restarts the accumulation without moving the stream"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block._core.hop, kFft / 2UZ)) << "with no hop stated the overlap sets it";

        block._core.streamAt = 4096ULL;
        block._core.segments = 3UZ;
        block._core.skipping = 700UZ; // a gap left over from the hop that is about to be replaced

        std::ignore = block.settings().set({{"hop", gr::Size_t{5000U}}});
        std::ignore = block.settings().activateContext();
        std::ignore = block.settings().applyStagedParameters();

        expect(eq(block._core.hop, 5000UZ)) << "the new hop is in force";
        expect(eq(block._core.segments, 0UZ)) << "and the accumulation it changes the meaning of is discarded";
        expect(eq(block._core.skipping, 0UZ)) << "a gap measured against the old hop does not survive it";
        expect(eq(block._core.gridStart, std::uint64_t{4096ULL})) << "a new hop is a new grid, anchored where the stream stands";
        expect(eq(block._core.streamAt, std::uint64_t{4096ULL})) << "the stream position is not a setting";

        // A change that leaves the hop alone leaves the grid alone with it, gap and all.
        block._core.skipping = 700UZ;
        std::ignore          = block.settings().set({{"window", std::string("Hamming")}});
        std::ignore          = block.settings().activateContext();
        std::ignore          = block.settings().applyStagedParameters();
        expect(eq(block._core.skipping, 700UZ)) << "a window change does not move the grid, so the gap it was going to skip stands";
        expect(eq(block._core.gridStart, std::uint64_t{4096ULL}));
    };

    "window_param builds the window the library builds, and the record says which one it is"_test = [] {
        // gqrx4's display is calibrated at Kaiser beta 6.76; the library's own default is 1.6, which costs about 45 dB
        // of sidelobe. A block that cannot be told the beta cannot produce that display's spectrum.
        constexpr float kBeta = 6.76f;

        const auto build = [](gr::property_map settings) {
            auto block = std::make_unique<WelchPsd<CF>>(std::move(settings));
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            return block;
        };
        const auto stated  = build({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}, {"window_param", kBeta}});
        const auto omitted = build({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}});

        const auto direct  = gr::algorithm::window::create<float>(gr::algorithm::window::Type::Kaiser, kFft, kBeta);
        const auto library = gr::algorithm::window::create<float>(gr::algorithm::window::Type::Kaiser, kFft);

        expect(eq(stated->_core.window.size(), direct.size()));
        for (std::size_t k = 0UZ; k < std::min(stated->_core.window.size(), direct.size()); ++k) {
            expect(stated->_core.window[k] == direct[k]) << "tap " << k << " is not what window::create builds at beta " << kBeta;
        }
        std::size_t differing = 0UZ;
        for (std::size_t k = 0UZ; k < stated->_core.window.size(); ++k) {
            differing += stated->_core.window[k] != library[k] ? 1UZ : 0UZ;
        }
        expect(differing > kFft / 2UZ) << "a stated beta has to be a different window from the default, " << differing << " taps differ";

        for (std::size_t k = 0UZ; k < omitted->_core.window.size(); ++k) {
            expect(omitted->_core.window[k] == library[k]) << "tap " << k << " does not match the library's own default";
        }

        // The window changes the noise bandwidth, so a record that names the window without the parameter under-states
        // its own calibration; both facts ride in the metadata.
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}, {"window_param", kBeta}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        expect(approx(metaNumber(records.front(), "window_param"), static_cast<double>(kBeta), 1e-5)) << "the record states the beta it was taken at";

        const gr::property_map defaulted{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}, {"window", std::string("Kaiser")}};
        const auto             plain = collect<WelchPsd<CF>, CF>(defaulted, tone(kFft * 8UZ, 32., kFft), 4096UZ);
        expect(!plain.empty());
        if (plain.empty()) {
            return;
        }
        expect(approx(metaNumber(plain.front(), "window_param"), 1.6, 1e-5)) << "and with no parameter stated the record names the default the library used, not the zero the setting held";

        // The calibration follows the parameter, which is the reason the record has to state it: a larger beta trades
        // sidelobe level for a wider main lobe, so the noise bandwidth the reader divides out is a different number.
        // The relationship is what is asserted; the figures are recorded rather than pinned.
        const double statedEnbw  = metaNumber(records.front(), "enbw_bins");
        const double defaultEnbw = metaNumber(plain.front(), "enbw_bins");
        std::println("window_param: Kaiser ENBW {:.4f} bins at beta {:.2f}, {:.4f} at the default 1.6", statedEnbw, static_cast<double>(kBeta), defaultEnbw);
        expect(statedEnbw > defaultEnbw) << "beta 6.76 has to spread a tone over more bins than beta 1.6, measured " << statedEnbw << " against " << defaultEnbw;
    };

    "a window parameter outside what the window accepts is refused, naming the setting"_test = [] {
        const auto refused = [](gr::property_map settings) {
            settings.insert({std::pmr::string("sample_rate"), gr::pmt::Value(kSampleRate)});
            WelchPsd<CF> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"window", std::string("Gaussian")}, {"window_param", 0.9f}}); })) << "a Gaussian sigma is a fraction of the half-length";
        expect(throws([&] { refused({{"window", std::string("Tukey")}, {"window_param", 2.0f}}); })) << "a Tukey alpha is a fraction";
        expect(throws([&] { refused({{"window", std::string("Kaiser")}, {"window_param", -1.0f}}); })) << "a Kaiser beta is not negative";
        expect(nothrow([&] { refused({{"window", std::string("Hann")}, {"window_param", 3.0f}}); })) << "a window with no parameter ignores one, as the library does";
    };

    "the ceiling is a receiver's largest display size, and what it costs is stated"_test = [] {
        // Accepting 2^22 is what the test can afford: a 2^22-point transform is about 15 ms and its record is 32 MiB,
        // which belongs in a bench and not in a test budget. What must hold here is that the setting is taken, that
        // the accumulator is built for the length, and that a size well above the old 65536 ceiling runs end to end.
        constexpr gr::Size_t kCeiling = 4194304U;

        WelchPsd<CF> atCeiling({{"fft_size", kCeiling}, {"n_averages", gr::Size_t{1U}}, {"sample_rate", kSampleRate}});
        expect(nothrow([&] {
            atCeiling.settings().init();
            std::ignore = atCeiling.settings().applyStagedParameters();
        })) << "the largest size gqrx4's dock offers has to be a size this block accepts";
        expect(eq(atCeiling._core.fftSize, static_cast<std::size_t>(kCeiling)));
        expect(eq(atCeiling._core.window.size(), static_cast<std::size_t>(kCeiling))) << "and the window is built for it";

        const auto refused = [](gr::Size_t size) {
            WelchPsd<CF> block({{"fft_size", size}, {"sample_rate", kSampleRate}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused(kCeiling * 2U); })) << "and one size above it is not";

        // A mid size, run whole: 65536 was the old ceiling, so this is the first length the amendment admits into a
        // running graph rather than merely into a settings map.
        constexpr std::size_t  kMid = 65536UZ;
        const gr::property_map settings{{"fft_size", gr::Size_t{static_cast<unsigned>(kMid)}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kMid * 3UZ, 1024., kMid), 65536UZ);
        expect(eq(records.size(), 3UZ)) << "three whole transforms of the old ceiling's length, made " << records.size();
        if (!records.empty()) {
            expect(eq(records.front().signal_values.size(), kMid));
            expect(eq(records.front().axis_values[0UZ].size(), kMid)) << "the axis is the other half of a record's bulk";
        }
    };

    "settings that cannot describe a measurement are refused"_test = [] {
        const auto refused = [](gr::property_map settings) {
            settings.insert({std::pmr::string("sample_rate"), gr::pmt::Value(kSampleRate)});
            WelchPsd<CF> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"fft_size", gr::Size_t{100U}}}); })) << "a transform length that is not a power of two";
        expect(throws([&] { refused({{"fft_size", gr::Size_t{32U}}}); })) << "below the stated floor";
        expect(throws([&] { refused({{"overlap", 1.0}}); })) << "a segment cannot wholly overlap its successor";
        expect(throws([&] { refused({{"overlap", -0.1}}); }));
        expect(throws([&] { refused({{"n_averages", gr::Size_t{0U}}}); }));
        expect(throws([&] { refused({{"mode", std::string("median")}}); })) << "a mode the accumulation has no rule for";
        expect(throws([&] { refused({{"window", std::string("Gaussian2")}}); })) << "a window the vocabulary has no name for";
    };

    "a stream ending mid-average flushes what it has, marked with its count"_test = [] {
        // five whole segments where the setting asks for eight: the one record that comes out is the flush
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 5UZ, 32., kFft), 4096UZ);

        expect(eq(records.size(), 1UZ)) << "a partial accumulation is reported, never suppressed";
        if (records.empty()) {
            return;
        }
        expect(eq(metaNumber(records.front(), "n_averaged"), 5.)) << "and it states the segments that actually reached it";
        expect(eq(metaNumber(records.front(), "sample_start"), 0.));
    };

    "the spectrogram's last hop is emitted, not left in the buffer"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             records = collect<Spectrogram<CF>, CF>(settings, tone(kFft * 5UZ, 32., kFft), 4096UZ);
        expect(eq(records.size(), 5UZ)) << "five whole transforms fit in the stream and five rows come out";
    };

    "no record is lost behind a sink that takes one at a time"_test = [] {
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 400UZ, 32., kFft);
        const auto             quick   = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        const auto             slow    = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ, 1UZ);

        expect(quick.size() >= 190UZ) << "the run has to be long enough to fill the record buffer, made " << quick.size();
        expect(eq(slow.size(), quick.size())) << "back-pressure delays records; it does not drop them";
        for (std::size_t r = 0UZ; r < std::min(slow.size(), quick.size()); ++r) {
            expect(eq(metaNumber(slow[r], "sample_start"), metaNumber(quick[r], "sample_start"))) << "record " << r << " lost its place in the stream";
        }
    };

    "a starved call says it lacks input rather than answering OK"_test = [] {
        // `in.min_samples` does not keep an empty span away from this block: an asynchronous output port is by
        // itself reason enough for the framework to run the block with no input at all, which is the path that
        // lets a full accumulation flush. A call that neither takes nor makes anything and still answers OK tells
        // the scheduler that it made progress, so the scheduler re-runs it at once and never parks -- and the
        // framework's own zero-progress watch then names this block for a stall that is upstream of it.
        const auto fresh = [] {
            auto block = std::make_unique<WelchPsd<CF>>(gr::property_map{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}});
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };

        std::vector<gr::DataSet<float>> room(4UZ);

        auto                                                      starved = fresh();
        gr::blocks::testing::span::InputSpan<CF>                  noInput{std::span<const CF>{}};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outEmpty{std::span<gr::DataSet<float>>(room)};
        expect(starved->processBulk(noInput, outEmpty) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "an empty span is a lack of input, not progress";
        expect(eq(outEmpty.count, 0UZ)) << "and nothing is published on that path";

        // One sample is still nothing this block can use: it holds a sample back so the end-of-stream epilogue
        // has a span to run on, which is what `in.min_samples = 2` asks the framework for.
        const std::vector<CF>                                     single{CF{1.f, 0.f}};
        gr::blocks::testing::span::InputSpan<CF>                  oneSample{std::span<const CF>(single)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outOne{std::span<gr::DataSet<float>>(room)};
        expect(starved->processBulk(oneSample, outOne) == gr::work::Status::INSUFFICIENT_INPUT_ITEMS) << "one sample is the held-back sample and nothing else";
        expect(eq(oneSample.consumed, 0UZ)) << "and it stays in the buffer";

        // A span it can work with answers OK, so the status distinguishes the two cases rather than reporting one.
        auto                                                      fed     = fresh();
        const std::vector<CF>                                     samples = tone(kFft + 1UZ, 32., kFft);
        gr::blocks::testing::span::InputSpan<CF>                  hasInput{std::span<const CF>(samples)};
        gr::blocks::testing::span::OutputSpan<gr::DataSet<float>> outFed{std::span<gr::DataSet<float>>(room)};
        expect(fed->processBulk(hasInput, outFed) == gr::work::Status::OK) << "a span it can take from is progress";
        expect(gt(hasInput.consumed, 0UZ)) << "and it took from it";
    };

    "a live setting restarts the accumulation and keeps the stream-absolute grid"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();

        block._core.pending.assign(100UZ, CF{1.f, 0.f});
        block._core.streamAt      = 4096ULL;
        block._core.recordStartAt = 4096ULL;
        block._core.segments      = 3UZ;

        std::ignore = block.settings().set({{"n_averages", gr::Size_t{4U}}, {"window", std::string("Hamming")}});
        std::ignore = block.settings().activateContext();
        std::ignore = block.settings().applyStagedParameters();

        expect(eq(block._core.streamAt, std::uint64_t{4096ULL})) << "the window grid stays anchored where the stream anchored it";
        expect(eq(block._core.pending.size(), 100UZ)) << "and no buffered sample is dropped";
        expect(eq(block._core.segments, 0UZ)) << "the half-averaged spectrum is discarded, since two windows do not average";
        expect(eq(block._core.recordStartAt, std::uint64_t{4096ULL})) << "the next record starts where the stream now is";
        expect(eq(block._core.nAverages, 4UZ)) << "and the new setting is in force";

        // A transaction carrying only a setting the accumulation does not depend on leaves the accumulation alone.
        // `activateContext()` restages the whole context, so this case stages the one key and applies it directly.
        block._core.segments = 2UZ;
        std::ignore          = block.settings().set({{"signal_name", std::string("psd_b")}});
        std::ignore          = block.settings().applyStagedParameters();
        expect(eq(block._core.segments, 2UZ)) << "a transaction naming no accumulation setting leaves the accumulation alone";
    };

    "fft_size moves under a running block, keeping the stream and losing no buffered sample"_test = [] {
        const auto running = [](gr::property_map settings) {
            auto block = std::make_unique<WelchPsd<CF>>(std::move(settings));
            block->settings().init();
            std::ignore = block->settings().applyStagedParameters();
            block->start();
            return block;
        };
        const auto live = [](auto& block, gr::property_map changes) {
            std::ignore = block.settings().set(std::move(changes));
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        };

        {
            auto block = running({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}});
            expect(nothrow([&] { live(*block, {{"overlap", 0.75}}); })) << "the overlap is live";
            expect(eq(block->_core.hop, kFft / 4UZ));
        }
        {
            auto block = running({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
            block->_core.pending.assign(60UZ, CF{1.f, 0.f});
            block->_core.streamAt      = 9000ULL;
            block->_core.recordStartAt = 9000ULL;
            block->_core.segments      = 5UZ;

            expect(nothrow([&] { live(*block, {{"fft_size", gr::Size_t{512U}}}); })) << "a running block takes a new transform length";
            expect(eq(block->_core.fftSize, 512UZ)) << "and is built for it";
            expect(eq(block->_core.window.size(), 512UZ));
            expect(eq(block->_core.windowed.size(), 512UZ)) << "the transform's own buffers move with the length";
            expect(eq(block->_core.accumulator.size(), 512UZ));
            expect(eq(block->_core.segments, 0UZ)) << "the five segments at the old resolution are discarded, not averaged with the new";
            expect(eq(block->_core.pending.size(), 60UZ)) << "no buffered sample is dropped";
            expect(eq(block->_core.streamAt, std::uint64_t{9000ULL})) << "and the stream position is not a setting";
            expect(eq(block->_core.gridStart, std::uint64_t{9000ULL})) << "the new grid is anchored where the stream stands, and the record says so";
        }
    };

    "a live fft_size change reaches a block inside a running graph, through a tag"_test = [] {
        // The framework's own path for a setting that moves mid-stream: a tag whose key names a setting the block did
        // not have written at construction. fft_size is therefore left at its 1024 default here, which is what makes
        // it auto-updatable; the source stamps the change at a known sample and the records either side are read.
        constexpr std::size_t kFirst  = 1024UZ; // the block's default
        constexpr std::size_t kSecond = 256UZ;
        constexpr std::size_t kAt     = 4096UZ; // the change lands here, on a segment boundary of the old grid

        gr::property_map settings{{"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
        gr::property_map moveTo{{"fft_size", gr::Size_t{static_cast<unsigned>(kSecond)}}};

        gr::test::RuntimeTest test;
        auto&                 source = test.emplace<BurstSource<CF>>();
        auto&                 block  = test.emplace<WelchPsd<CF>>(std::move(settings));
        auto&                 sink   = test.emplace<RecordSink>();
        source.samples               = tone(8192UZ, 32., kFirst);
        source.burst                 = 512UZ;
        source.tags.push_back(gr::Tag{kAt, std::move(moveTo)});

        expect(test.connect(source, "out", block, "in").has_value() && test.connect(block, "out", sink, "in").has_value()) << fatal;
        std::ignore = test.run();

        const auto& records = sink.records;
        expect(records.size() > 4UZ) << "the run has to produce records at both lengths, made " << records.size();

        std::size_t atFirst  = 0UZ;
        std::size_t atSecond = 0UZ;
        for (const auto& record : records) {
            const auto size = static_cast<std::size_t>(metaNumber(record, "fft_size"));
            if (size == kFirst) {
                ++atFirst;
                expect(eq(metaNumber(record, "grid_start"), 0.)) << "records before the change are on the grid the stream started";
            } else if (size == kSecond) {
                ++atSecond;
                expect(eq(record.signal_values.size(), kSecond)) << "a record's length is the transform that made it";
                expect(metaNumber(record, "grid_start") > 0.) << "and records after it say the grid moved";
                expect(metaNumber(record, "sample_start") >= metaNumber(record, "grid_start")) << "no segment starts before the grid it is on";
            }
            expect(eq(metaNumber(record, "n_averaged"), 1.)) << "no record is a partial average across the change";
        }
        expect(atFirst > 0UZ) << "records at the first length";
        expect(atSecond > 0UZ) << "and records at the second, which is the whole point";

        // The stream position keeps counting the same stream across the change: sample_start never goes backwards.
        for (std::size_t r = 1UZ; r < records.size(); ++r) {
            expect(metaNumber(records[r], "sample_start") > metaNumber(records[r - 1UZ], "sample_start")) << "record " << r << " went backwards in the stream";
        }
    };

    "a restart clears the flush latch, so the second run reports its own partial record"_test = [] {
        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();

        block.start();
        block._flushed       = true; // what the end of a first run leaves behind
        block._core.streamAt = 999ULL;
        block._core.pending.assign(7UZ, CF{});

        block.start(); // the framework's own second run; the block keeps no state that a stop would have to clear
        expect(!block._flushed) << "a second run's flush must not be suppressed by the first run's";
        expect(eq(block._core.streamAt, std::uint64_t{0ULL})) << "and the grid is anchored at the new stream's origin";
        expect(block._core.pending.empty());
    };

    "criterion 2: halving n_averages doubles the estimator variance, within an envelope"_test = [] {
        // Welch's own statistics: n independent periodograms of white noise average to a per-bin estimate whose
        // variance is the square of its mean over n. The ratio between 32 and 64 averages is therefore 2, and the
        // measured figure is recorded rather than merely bounded. Segments do not overlap, so they are independent.
        GaussianNoise   noise;
        std::vector<CF> samples(kFft * 64UZ * 4UZ);
        std::ranges::generate(samples, [&noise] { return noise(); });

        const auto measure = [&samples](gr::Size_t averages) {
            const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", averages}, {"overlap", 0.0}, {"sample_rate", kSampleRate}};
            const auto             records = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
            double                 total   = 0.;
            std::size_t            counted = 0UZ;
            for (const auto& record : records) {
                if (metaNumber(record, "n_averaged") == static_cast<double>(averages)) { // whole records only, not the flush
                    total += relativeVariance(record);
                    ++counted;
                }
            }
            return counted == 0UZ ? 0. : total / static_cast<double>(counted);
        };

        const double at64  = measure(64U);
        const double at32  = measure(32U);
        const double ratio = at32 / at64;
        std::println("criterion 2: relative estimator variance {:.5f} at 64 averages, {:.5f} at 32, ratio {:.3f} (nominal 1/64, 1/32, 2)", at64, at32, ratio);

        expect(at64 > 0. && at32 > 0.) << "both settings have to produce whole records";
        expect(ratio > 1.5 && ratio < 2.7) << "the variance envelope is a factor of two, measured " << ratio;
    };

    "criterion 3: a windowed tone's sub-bin frequency, with the bias each window costs"_test = [] {
        // The fixture is the real thing: a complex exponential through the block's own window and transform, read by
        // PeakDetect's three-point parabolic refinement. The refinement is exact for a parabola and a windowed main
        // lobe is not one, so the bias is measured per window and recorded rather than assumed away.
        constexpr double kCenterBin = 32.;
        constexpr double kMargin    = 0.06; // bins; the measured bias at 0.3 and 0.5 sits well inside this

        for (const std::string& windowName : {std::string("Hann"), std::string("Blackman")}) {
            for (const double offset : {0.0, 0.3, 0.5}) {
                const gr::property_map psdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"window", windowName}};
                const auto             psd = collect<WelchPsd<CF>, CF>(psdSettings, tone(kFft * 16UZ, kCenterBin + offset, kFft), 4096UZ);
                expect(!psd.empty()) << "the tone has to produce a density record";
                if (psd.empty()) {
                    continue;
                }
                const auto detections = collectFromRecords<gr::blocks::measurement::PeakDetect>({{"threshold_db", 20.0}, {"reference", std::string("above_median")}}, {psd.front()});
                expect(eq(detections.size(), 1UZ)) << "one tone is one detection record, window " << windowName << " offset " << offset;
                if (detections.empty()) {
                    continue;
                }
                const double measured = strongestPeakBin(detections.front(), kSampleRate, kFft);
                const double bias     = measured - (kCenterBin + offset);
                std::println("criterion 3: {:<9} tone at bin {:+.1f}: parabolic estimate off by {:+.5f} bins", windowName, offset, bias);

                if (offset == 0.0) {
                    expect(std::abs(bias) < 1e-4) << "at a bin center the lobe is symmetric and the refinement adds nothing, measured " << bias;
                } else {
                    expect(std::abs(bias) < kMargin) << "off-center bias for " << windowName << " at " << offset << " bins measured " << bias;
                }
            }
        }
    };

    "criterion 6: every emitted record passes the tier's admission predicates and carries the section 0 keys"_test = [] {
        const gr::property_map psdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}};
        const gr::property_map rowSettings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};

        std::vector<gr::DataSet<float>> everything;
        for (const auto& record : collect<WelchPsd<CF>, CF>(psdSettings, tone(kFft * 12UZ, 32., kFft), 4096UZ)) {
            everything.push_back(record);
        }
        std::vector<float> real(kFft * 12UZ);
        for (std::size_t k = 0UZ; k < real.size(); ++k) {
            real[k] = std::cos(2.f * std::numbers::pi_v<float> * 32.f * static_cast<float>(k) / static_cast<float>(kFft));
        }
        for (const auto& record : collect<WelchPsd<float>, float>(psdSettings, real, 4096UZ)) {
            everything.push_back(record);
        }
        for (const auto& record : collect<Spectrogram<CF>, CF>(rowSettings, tone(kFft * 8UZ, 32., kFft), 4096UZ)) {
            everything.push_back(record);
        }
        expect(everything.size() >= 8UZ) << "the three shapes have to have produced records to judge";

        for (std::size_t r = 0UZ; r < everything.size(); ++r) {
            const auto& ds     = everything[r];
            const char* reason = admissionFailure(ds);
            expect(reason == nullptr) << "record " << r << " would be rejected as " << (reason == nullptr ? "" : reason);

            expect(metaNumber(ds, "sample_rate") > 0.) << "record " << r << " states no rate";
            expect(metaNumber(ds, "sample_start") >= 0.) << "record " << r << " states no place in the stream";
            expect(metaNumber(ds, "n_averaged") >= 1.) << "record " << r << " states no segment count";
            expect(metaNumber(ds, "overlap") >= 0. && metaNumber(ds, "overlap") < 1.) << "record " << r << " states no segment overlap";
            expect(metaNumber(ds, "enbw_bins") > 0.) << "record " << r << " states no noise bandwidth";

            const auto& axis = ds.axis_values[0UZ];
            expect(eq(axis.size(), ds.signal_values.size())) << "record " << r << " has an axis of a different length from its values";
            const float step = axis[1UZ] - axis[0UZ];
            for (std::size_t k = 1UZ; k < axis.size(); ++k) {
                expect(std::abs((axis[k] - axis[k - 1UZ]) - step) <= 1e-6f * std::abs(step)) << "record " << r << " axis is not uniform at bin " << k;
            }
        }
    };

    "criterion 7: chunk independence holds for the spectrogram and for a real input"_test = [] {
        const gr::property_map rowSettings{{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             samples = tone(kFft * 8UZ, 32., kFft);
        const auto             wideRow = collect<Spectrogram<CF>, CF>(rowSettings, samples, 4096UZ);
        const auto             thinRow = collect<Spectrogram<CF>, CF>(rowSettings, samples, 37UZ);

        expect(!wideRow.empty());
        expect(eq(wideRow.size(), thinRow.size())) << "the row count cannot depend on how the stream arrived";
        for (std::size_t r = 0UZ; r < std::min(wideRow.size(), thinRow.size()); ++r) {
            expect(eq(metaNumber(wideRow[r], "sample_start"), metaNumber(thinRow[r], "sample_start"))) << "row " << r << " moved on the time axis";
            for (std::size_t k = 0UZ; k < wideRow[r].signal_values.size(); ++k) {
                expect(wideRow[r].signal_values[k] == thinRow[r].signal_values[k]) << "row " << r << " bin " << k << " differs between chunkings";
            }
        }

        const gr::property_map psdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{2U}}, {"sample_rate", kSampleRate}};
        std::vector<float>     real(kFft * 8UZ);
        for (std::size_t k = 0UZ; k < real.size(); ++k) {
            real[k] = std::cos(2.f * std::numbers::pi_v<float> * 32.f * static_cast<float>(k) / static_cast<float>(kFft));
        }
        const auto wideReal = collect<WelchPsd<float>, float>(psdSettings, real, 4096UZ);
        const auto thinReal = collect<WelchPsd<float>, float>(psdSettings, real, 37UZ);

        expect(!wideReal.empty());
        expect(eq(wideReal.size(), thinReal.size())) << "a real stream's record count cannot depend on its chunking either";
        for (std::size_t r = 0UZ; r < std::min(wideReal.size(), thinReal.size()); ++r) {
            expect(eq(metaNumber(wideReal[r], "sample_start"), metaNumber(thinReal[r], "sample_start")));
            for (std::size_t k = 0UZ; k < wideReal[r].signal_values.size(); ++k) {
                expect(wideReal[r].signal_values[k] == thinReal[r].signal_values[k]) << "real record " << r << " bin " << k << " differs between chunkings";
            }
        }
    };

    "max_hold keeps the largest density each bin reached, never the average"_test = [] {
        // a tone for the first half of the stream and silence after: the mean falls, the hold does not
        std::vector<CF> samples = tone(kFft * 8UZ, 32., kFft);
        std::fill(samples.begin() + static_cast<std::ptrdiff_t>(kFft * 4UZ), samples.end(), CF{});

        const gr::property_map meanSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}, {"mode", std::string("mean")}};
        const gr::property_map holdSettings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}, {"mode", std::string("max_hold")}};

        const auto meanRecords = collect<WelchPsd<CF>, CF>(meanSettings, samples, 4096UZ);
        const auto holdRecords = collect<WelchPsd<CF>, CF>(holdSettings, samples, 4096UZ);
        expect(!meanRecords.empty() && !holdRecords.empty());
        if (meanRecords.empty() || holdRecords.empty()) {
            return;
        }
        const float meanPeak = *std::ranges::max_element(meanRecords.front().signal_values);
        const float holdPeak = *std::ranges::max_element(holdRecords.front().signal_values);
        expect(holdPeak > meanPeak) << "the hold keeps the loud half, the mean divides it away";
    };

    // `threads` buys the transform threads and changes nothing else. At 2^18 the second thread moves the transform
    // off SimdFFT and onto the four-step split, which is a different factorization of the same DFT, so the records
    // agree to the two engines' rounding rather than bit for bit -- the bound is against the record's own peak,
    // because a bin far down the skirt has no absolute scale of its own.
    "threads change what a transform costs and not what it says"_test = [] {
        constexpr std::size_t kLong = 1UZ << 18UZ;

        const auto samples = tone(kLong * 2UZ, 1024., kLong);
        const auto records = [&samples](gr::Size_t threads) {
            const gr::property_map settings{{"fft_size", gr::Size_t{static_cast<gr::Size_t>(kLong)}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}, {"threads", threads}};
            return collect<WelchPsd<CF>, CF>(settings, samples, 65536UZ);
        };

        const auto single = records(1U);
        const auto many   = records(4U);
        expect(!single.empty() && eq(single.size(), many.size())) << "the thread count cannot change how many records a stream makes";
        if (single.empty() || single.size() != many.size()) {
            return;
        }

        for (std::size_t r = 0UZ; r < single.size(); ++r) {
            expect(eq(single[r].signal_values.size(), many[r].signal_values.size()));
            expect(eq(metaNumber(single[r], "sample_start"), metaNumber(many[r], "sample_start"))) << "and cannot move the window grid";

            const float peak      = *std::ranges::max_element(single[r].signal_values);
            float       deviation = 0.f;
            for (std::size_t k = 0UZ; k < single[r].signal_values.size(); ++k) {
                deviation = std::max(deviation, std::abs(single[r].signal_values[k] - many[r].signal_values[k]));
            }
            expect(lt(deviation, peak * 1.e-5f)) << std::format("record {} worst deviation {} against a peak of {}", r, deviation, peak);
        }
    };

    "threads is live on its own and survives a length rebuild"_test = [] {
        const auto live = [](auto& block, gr::property_map changes) {
            std::ignore = block.settings().set(std::move(changes));
            std::ignore = block.settings().activateContext();
            std::ignore = block.settings().applyStagedParameters();
        };

        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block._core.threads, 1UZ)) << "one thread is the default";
        expect(eq(block._core.transform.threads, 1UZ)) << "and it reaches the transform";

        block._core.pending.assign(60UZ, CF{1.f, 0.f});
        block._core.streamAt = 9000ULL;
        block._core.segments = 5UZ;

        expect(nothrow([&] { live(block, {{"threads", gr::Size_t{4U}}}); })) << "a running block takes a new thread count";
        expect(eq(block._core.transform.threads, 4UZ)) << "which reaches the transform";
        expect(eq(block._core.pending.size(), 60UZ)) << "and the buffered samples survive it";
        expect(eq(block._core.streamAt, std::uint64_t{9000ULL})) << "and the stream position";

        // Whether the accumulation survives is the block's own rule, and it is pinned against the callback rather
        // than against settings(): a change through settings() re-stages every key the block was explicitly given,
        // not only the one that moved, so that path cannot show which key the block acted on.
        block._core.segments = 5UZ;
        block.threads        = 8U;
        expect(nothrow([&] { block.settingsChanged({}, gr::property_map{{"threads", gr::Size_t{8U}}}); }));
        expect(eq(block._core.transform.threads, 8UZ)) << "the count reaches the transform";
        expect(eq(block._core.segments, 5UZ)) << "and on its own it does not restart the accumulation -- it changes cost, not meaning";

        live(block, {{"fft_size", gr::Size_t{512U}}});
        expect(eq(block._core.fftSize, 512UZ));
        expect(eq(block._core.transform.threads, 4UZ)) << "a length rebuild makes a fresh transform, which is re-told the thread count in force";

        expect(throws([&] { live(block, {{"threads", gr::Size_t{0U}}}); })) << "zero threads is not a thread count";
    };

    // The tuning belongs in the record, not in the signal name: a consumer that stacks records on an absolute axis
    // reads one number rather than parsing one out of a string. It changes nothing about the estimate, so like
    // `signal_name` it must not restart an average that is part way through -- a receiver retunes mid-stream.
    "center_frequency reaches the record and restarts nothing"_test = [] {
        constexpr double       kCenter = 435.5e6;
        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"sample_rate", kSampleRate}, {"center_frequency", kCenter}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, tone(kFft * 8UZ, 32., kFft), 4096UZ);
        expect(!records.empty());
        if (records.empty()) {
            return;
        }
        expect(eq(metaNumber(records.front(), "center_frequency"), kCenter)) << "the record carries the tuning beside sample_rate";
        expect(approx(records.front().axis_values[0UZ].front(), -kSampleRate / 2.f, 1.f)) << "and the axis itself stays baseband";

        const auto rows = collect<Spectrogram<CF>, CF>({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"center_frequency", kCenter}}, tone(kFft * 4UZ, 32., kFft), 4096UZ);
        expect(!rows.empty());
        if (!rows.empty()) {
            expect(eq(metaNumber(rows.front(), "center_frequency"), kCenter)) << "and so does a spectrogram row";
        }

        WelchPsd<CF> block({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{8U}}, {"sample_rate", kSampleRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        block._core.pending.assign(60UZ, CF{1.f, 0.f});
        block._core.streamAt = 9000ULL;
        block._core.segments = 5UZ;

        const auto live = [](auto& target, gr::property_map changes) {
            std::ignore = target.settings().set(std::move(changes));
            std::ignore = target.settings().activateContext();
            std::ignore = target.settings().applyStagedParameters();
        };
        expect(nothrow([&] { live(block, {{"center_frequency", 88.5e6}}); })) << "a running block retunes";
        expect(eq(block.center_frequency.value, 88.5e6)) << "and the new tuning is in force";
        expect(eq(block._core.pending.size(), 60UZ)) << "with every buffered sample kept";
        expect(eq(block._core.gridStart, std::uint64_t{0ULL})) << "and the grid where it was";

        // as for `threads`: the block's own rule is pinned against the callback, because a change through settings()
        // re-stages every key the block was explicitly given rather than only the one that moved
        block._core.segments   = 5UZ;
        block.center_frequency = 144.5e6;
        expect(nothrow([&] { block.settingsChanged({}, gr::property_map{{"center_frequency", 144.5e6}}); }));
        expect(eq(block._core.segments, 5UZ)) << "a retune on its own does not restart the average in progress";

        expect(throws([&] { live(block, {{"center_frequency", std::numeric_limits<double>::quiet_NaN()}}); })) << "a tuning that is not a number is refused";
    };

    "the spectrogram takes the same thread setting"_test = [] {
        Spectrogram<CF> block({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}, {"threads", gr::Size_t{2U}}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block._core.transform.threads, 2UZ)) << "the setting reaches the transform at construction";
    };

    // A consumer pacing itself by what a record costs the block cannot read that cost off the thread it waits on:
    // above one thread the transform runs on a pool and the calling thread's processor clock stops counting it. The
    // record states a wall-clock reading around its whole computation instead, which covers every thread.
    "every record states the wall clock computing it cost"_test = [] {
        const auto samples = tone(kFft * 16UZ, 32., kFft);

        const gr::property_map settings{{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{4U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}};
        const auto             records = collect<WelchPsd<CF>, CF>(settings, samples, 4096UZ);
        expect(records.size() >= 2UZ) << "the run has to make more than one record";
        for (std::size_t r = 0UZ; r < records.size(); ++r) {
            expect(metaNumber(records[r], "compute_seconds", -1.) > 0.) << std::format("record {} states no compute cost", r);
        }

        const auto rows = collect<Spectrogram<CF>, CF>({{"fft_size", gr::Size_t{kFft}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}}, samples, 4096UZ);
        expect(rows.size() >= 2UZ) << "and so has the spectrogram's";
        for (std::size_t r = 0UZ; r < rows.size(); ++r) {
            expect(metaNumber(rows[r], "compute_seconds", -1.) > 0.) << std::format("row {} states no compute cost", r);
        }
    };

    "the stated cost is the block's own compute, and grows with the transform"_test = [] {
        const auto check = [](std::string_view name, auto costAt) {
            constexpr std::size_t kShort = 1UZ << 10UZ;
            constexpr std::size_t kLong  = 1UZ << 20UZ;
            constexpr double      kShare = 0.25; // the least of a bare call's wall clock the computation may be

            const auto [shortCost, shortCall] = costAt(kShort, gr::Size_t{1U});
            const auto [longCost, longCall]   = costAt(kLong, gr::Size_t{1U});
            const auto [manyCost, manyCall]   = costAt(kLong, gr::Size_t{4U});
            std::println("{}: compute_seconds {:.6e} s at {} points (call {:.6e} s), {:.6e} s at {} points (call {:.6e} s), {:.6e} s on four threads (call {:.6e} s)", name, shortCost, kShort, shortCall, longCost, kLong, longCall, manyCost, manyCall);

            expect(shortCost > 0. && longCost > 0.) << name << ": a record that cost nothing was not computed";
            expect(shortCost <= shortCall && longCost <= longCall) << name << ": the figure is measured inside the call and cannot exceed it";
            expect(shortCost >= kShare * shortCall && longCost >= kShare * longCall) << name << ": the computation is the bulk of what a bare call does";
            expect(longCost > 10. * shortCost) << name << ": a transform a thousand times longer must cost more";
            expect(manyCost > 0. && manyCost <= manyCall) << name << ": a threaded transform still states what the record cost";
        };

        check("WelchPsd", [](std::size_t size, gr::Size_t threads) { return oneRecordCost<WelchPsd<CF>>({{"fft_size", static_cast<gr::Size_t>(size)}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"threads", threads}}, size); });
        check("Spectrogram", [](std::size_t size, gr::Size_t threads) { return oneRecordCost<Spectrogram<CF>>({{"fft_size", static_cast<gr::Size_t>(size)}, {"overlap", 0.0}, {"sample_rate", kSampleRate}, {"threads", threads}}, size); });
    };

    // The reading a consumer can take for itself is the graph's own wall clock, and a record's figure is one part of
    // it: the scheduler's start-up, the source's copies and the record's copy out to the sink are the rest.
    "the stated cost is consistent with an outside reading of a single-record graph"_test = [] {
        constexpr std::size_t kSize = 1UZ << 18UZ;

        const auto check = [](std::string_view name, std::pair<double, double> reading) {
            constexpr double kFloor = 0.02; // the least of a single-record graph's wall clock the record may state

            const auto [cost, run] = reading;
            std::println("{}: a single-record graph ran in {:.6e} s and its record states {:.6e} s", name, run, cost);
            expect(cost > 0.) << name << ": the graph made a record and it states a cost";
            expect(cost <= run) << name << ": a record cannot have cost more than the run that made it";
            expect(cost >= kFloor * run) << name << ": and it is a stated fraction of that run";
        };

        check("WelchPsd", oneRecordGraphCost<WelchPsd<CF>>({{"fft_size", static_cast<gr::Size_t>(kSize)}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", kSampleRate}}, kSize));
        check("Spectrogram", oneRecordGraphCost<Spectrogram<CF>>({{"fft_size", static_cast<gr::Size_t>(kSize)}, {"overlap", 0.0}, {"sample_rate", kSampleRate}}, kSize));
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
