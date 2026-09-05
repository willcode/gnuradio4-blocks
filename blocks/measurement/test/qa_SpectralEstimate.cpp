#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <print>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>

namespace qa_spectral {

using gr::blocks::measurement::Spectrogram;
using gr::blocks::measurement::WelchPsd;
using CF = std::complex<float>;

constexpr std::size_t kFft        = 256UZ;
constexpr float       kSampleRate = 48000.f;

/// Emits a fixed sequence in bursts of a stated size, then ends the stream. The burst size is what makes chunk
/// independence testable: the same samples presented differently must produce the same records.
template<typename T>
struct BurstSource : gr::Block<BurstSource<T>> {
    gr::PortOut<T> out;

    std::vector<T> samples{};
    std::size_t    burst = 4096UZ;
    std::size_t    at    = 0UZ;

    GR_MAKE_REFLECTABLE(BurstSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
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

    "fft_size is refused while the block runs and moves again once it is stopped"_test = [] {
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
            expect(throws([&] { live(*block, {{"fft_size", gr::Size_t{512U}}}); })) << "a running block refuses a new transform length";
        }
        {
            auto block = running({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}});
            expect(nothrow([&] { live(*block, {{"overlap", 0.75}}); })) << "the overlap is live";
            expect(eq(block->_core.hop, kFft / 4UZ));
        }
        {
            auto block = running({{"fft_size", gr::Size_t{kFft}}, {"sample_rate", kSampleRate}});
            block->stop();
            expect(nothrow([&] { live(*block, {{"fft_size", gr::Size_t{512U}}}); })) << "stopped, the length moves: the contract is staged-restart, not immutable";
            expect(eq(block->_core.fftSize, 512UZ));
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
        block.stop();

        block.start();
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
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
