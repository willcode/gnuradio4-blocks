#include <algorithm>
#include <cmath>
#include <cstdint>
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
#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/measurement/OccupiedBandwidth.hpp>

namespace qa_detect {

using gr::blocks::measurement::CfarDetect;
using gr::blocks::measurement::OccupiedBandwidth;
using gr::blocks::measurement::PeakDetect;

constexpr float       kSampleRate = 48000.f;
constexpr std::size_t kBins       = 1024UZ;

/// Emits a fixed list of density records, then ends the stream.
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

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<float>, gr::Async> in;

    std::vector<gr::DataSet<float>> records{};

    GR_MAKE_REFLECTABLE(RecordSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        records.insert(records.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief A density record on a two-sided axis, with the values the caller supplies.
[[nodiscard]] gr::DataSet<float> densityRecord(std::vector<float> values, float sampleRate = kSampleRate) {
    gr::DataSet<float> ds;
    const std::size_t  n = values.size();
    ds.extents           = {static_cast<std::int32_t>(n)};
    ds.layout            = gr::LayoutRight{};
    ds.axis_names        = {"Frequency"};
    ds.axis_units        = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(n);
    const float binWidth = sampleRate / static_cast<float>(n);
    const float offset   = static_cast<float>(n / 2UZ) * binWidth;
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) * binWidth - offset;
    }
    ds.signal_names      = {"psd"};
    ds.signal_quantities = {"PowerSpectralDensity"};
    ds.signal_units      = {"1/Hz"};
    ds.signal_values     = std::move(values);
    ds.signal_ranges.resize(1UZ);
    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = gr::property_map{{std::pmr::string("sample_rate"), gr::pmt::Value(sampleRate)}, {std::pmr::string("sample_start"), gr::pmt::Value(std::uint64_t{7ULL})}};
    ds.timing_events.resize(1UZ);
    return ds;
}

template<typename TBlock>
[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, std::vector<gr::DataSet<float>> records) {
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

/// @brief As `collect`, and also the two counters the detector keeps of what it did and did not emit.
template<typename TBlock>
[[nodiscard]] std::pair<std::vector<gr::DataSet<float>>, std::pair<std::uint64_t, std::uint64_t>> collectCounted(gr::property_map settings, std::vector<gr::DataSet<float>> records) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<RecordSource>();
    auto&                 block  = test.emplace<TBlock>(std::move(settings));
    auto&                 sink   = test.emplace<RecordSink>();
    source.records               = std::move(records);
    if (!test.connect(source, "out", block, "in").has_value() || !test.connect(block, "out", sink, "in").has_value()) {
        return {};
    }
    std::ignore = test.run();
    return {sink.records, {block.nRecords(), block.nEmptyResults()}};
}

/// @brief Column `c` of a detection record, whose three columns are laid end to end.
[[nodiscard]] std::span<const float> column(const gr::DataSet<float>& record, std::size_t c) {
    const std::size_t n = record.signal_values.size() / 3UZ;
    return std::span<const float>(record.signal_values).subspan(c * n, n);
}

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

/// @brief Exponentially distributed power of unit mean, which is what a squared-magnitude noise bin is.
struct ExponentialNoise {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] float operator()() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        const double uniform = (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
        return static_cast<float>(-std::log(uniform));
    }
};

} // namespace qa_detect

const boost::ut::suite<"Detectors"> detectorTests = [] {
    using namespace boost::ut;
    using namespace qa_detect;

    "the parabolic refinement is exact on a lobe that is a parabola in decibels"_test = [] {
        // A Gaussian lobe is an exact parabola in decibels, so this fixture bounds the arithmetic rather than the
        // window: criterion 3's bias against a real windowed tone is measured in qa_SpectralEstimate.
        const auto lobeAt = [](double centerBin) {
            std::vector<float> values(kBins, 1e-9f);
            for (std::size_t k = 0UZ; k < kBins; ++k) {
                const double d = (static_cast<double>(k) - centerBin) / 1.5; // a Hann-like main lobe about three bins wide
                values[k] += static_cast<float>(std::exp(-d * d));
            }
            return values;
        };
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        for (const double offset : {0.0, 0.3, -0.3, 0.5}) {
            const double centre  = 300. + offset;
            const auto   records = collect<PeakDetect>({{"threshold_db", 20.0}, {"reference", std::string("above_median")}}, {densityRecord(lobeAt(centre))});
            expect(eq(records.size(), 1UZ));
            if (records.empty()) {
                continue;
            }
            const auto frequencies = column(records.front(), 0UZ);
            expect(eq(frequencies.size(), 1UZ)) << "one lobe is one detection, at offset " << offset;
            if (frequencies.size() != 1UZ) {
                continue;
            }
            const float expected  = (static_cast<float>(centre) - static_cast<float>(kBins / 2UZ)) * binWidth;
            const float errorBins = std::abs(frequencies[0UZ] - expected) / binWidth;
            if (offset == 0.0) {
                expect(errorBins < 1e-4f) << "at a bin center the refinement must add nothing, measured " << errorBins << " bins";
            } else {
                expect(errorBins < 0.05f) << "off-center bias at offset " << offset << " measured " << errorBins << " bins";
            }
        }
    };

    "peaks closer than the stated separation collapse to the strongest"_test = [] {
        std::vector<float> values(kBins, 1e-9f);
        values[400UZ]        = 1.0f;
        values[401UZ]        = 0.5f; // a shoulder, not a second signal
        values[402UZ]        = 0.25f;
        values[600UZ]        = 0.8f; // a genuine second emitter, far away
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        const auto loose = collect<PeakDetect>({{"threshold_db", 20.0}, {"min_distance_hz", 0.0}}, {densityRecord(values)});
        const auto tight = collect<PeakDetect>({{"threshold_db", 20.0}, {"min_distance_hz", static_cast<double>(10.f * binWidth)}}, {densityRecord(values)});
        expect(!loose.empty() && !tight.empty());
        if (loose.empty() || tight.empty()) {
            return;
        }
        expect(column(tight.front(), 0UZ).size() == 2UZ) << "the shoulder collapses, the far emitter survives";
        expect(column(loose.front(), 0UZ).size() >= column(tight.front(), 0UZ).size());
    };

    "an empty result emits no record and is counted, so nothing-found still differs from nothing-ran"_test = [] {
        // An empty DataSet fails the tier's admission predicates, whose first question is whether the extent is
        // positive, so the empty result cannot travel as a record; the counter carries it instead.
        std::vector<gr::DataSet<float>> quiet;
        for (std::size_t r = 0UZ; r < 3UZ; ++r) {
            quiet.push_back(densityRecord(std::vector<float>(kBins, 1.f)));
        }
        const auto [records, counts] = collectCounted<PeakDetect>({{"threshold_db", 60.0}}, std::move(quiet));
        expect(records.empty()) << "no detection passed the threshold, so no record was emitted";
        expect(eq(counts.first, std::uint64_t{0ULL})) << "and none is counted as emitted";
        expect(eq(counts.second, std::uint64_t{3ULL})) << "the three inputs that found nothing are counted as such";

        std::vector<float> withPeak(kBins, 1e-9f);
        withPeak[500UZ]                = 1.f;
        const auto [found, foundCount] = collectCounted<PeakDetect>({{"threshold_db", 20.0}}, {densityRecord(std::move(withPeak))});
        expect(eq(found.size(), 1UZ)) << "a record that finds something does emit one";
        expect(eq(foundCount.first, std::uint64_t{1ULL}));
        expect(eq(foundCount.second, std::uint64_t{0ULL}));
    };

    "the CFAR detector counts its own empty results too"_test = [] {
        // pure noise at a design rate of 10^-9 finds nothing at all in a single record
        ExponentialNoise   noise;
        std::vector<float> values(kBins);
        std::ranges::generate(values, [&noise] { return noise(); });

        const auto [records, counts] = collectCounted<CfarDetect>({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-9}}, {densityRecord(std::move(values))});
        expect(records.empty()) << "nothing crossed a threshold set nine decades down";
        expect(eq(counts.first, std::uint64_t{0ULL}));
        expect(eq(counts.second, std::uint64_t{1ULL}));
    };

    "criterion 4: the measured false-alarm rate matches the design, on noise alone"_test = [] {
        constexpr double      kPfa     = 1e-3;
        constexpr std::size_t kRecords = 1024UZ; // about 10^6 tested cells at the margin below
        ExponentialNoise      noise;

        std::vector<gr::DataSet<float>> records;
        records.reserve(kRecords);
        for (std::size_t r = 0UZ; r < kRecords; ++r) {
            std::vector<float> values(kBins);
            std::ranges::generate(values, [&noise] { return noise(); });
            records.push_back(densityRecord(std::move(values)));
        }

        const gr::property_map settings{{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", kPfa}};
        const auto [out, counts] = collectCounted<CfarDetect>(settings, std::move(records));
        expect(eq(counts.first + counts.second, static_cast<std::uint64_t>(kRecords))) << "every record in is accounted for, as a record out or as an empty result";
        expect(eq(out.size(), static_cast<std::size_t>(counts.first))) << "and the records that came out are the ones counted";

        std::size_t detections = 0UZ;
        for (const auto& record : out) {
            detections += record.signal_values.size() / 3UZ;
        }
        const std::size_t margin   = 16UZ + 2UZ;
        const std::size_t tested   = kRecords * (kBins - 2UZ * margin);
        const double      measured = static_cast<double>(detections) / static_cast<double>(tested);
        expect(tested > 900000UZ) << "the claim is made over about a million cells, tested " << tested;
        expect(measured < 2. * kPfa && measured > 0.5 * kPfa) << "measured false-alarm rate " << measured << " against a design " << kPfa;
    };

    "criterion 4: a tone well over the noise is detected, and its guard cells stay clean"_test = [] {
        ExponentialNoise   noise;
        std::vector<float> values(kBins);
        std::ranges::generate(values, [&noise] { return noise(); });
        values[512UZ] += 10.f; // ten times the mean noise power, which is 10 dB over

        const gr::property_map settings{{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}};
        const auto             out = collect<CfarDetect>(settings, {densityRecord(std::move(values))});
        expect(eq(out.size(), 1UZ));
        if (out.empty()) {
            return;
        }
        const auto  frequencies = column(out.front(), 0UZ);
        const float binWidth    = kSampleRate / static_cast<float>(kBins);
        const float expected    = (512.f - static_cast<float>(kBins / 2UZ)) * binWidth;
        expect(std::ranges::any_of(frequencies, [&](float f) { return std::abs(f - expected) < 0.5f * binWidth; })) << "the tone is found";
        expect(frequencies.size() <= 3UZ) << "and it is not accompanied by a crowd of false alarms";
    };

    "the detector states the multiplier and the cells it can test"_test = [] {
        CfarDetect block({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        // alpha = N * (pfa^(-1/N) - 1) at N = 32
        const double expected = 32. * (std::pow(1e-3, -1. / 32.) - 1.);
        expect(std::abs(block.alpha() - expected) < 1e-9) << "the closed cell-averaging form, stated";
        expect(eq(block.testableCells(kBins), kBins - 36UZ)) << "the untested margin is the training and guard bands";
        expect(eq(block.testableCells(10UZ), 0UZ)) << "a record shorter than the margin tests nothing";
    };

    "a flat band with no transition reads its own edges"_test = [] {
        // a two-sided band 200 bins wide, flat, with nothing outside it
        constexpr std::size_t kLow  = 412UZ;
        constexpr std::size_t kHigh = 612UZ;
        std::vector<float>    values(kBins, 0.f);
        std::fill(values.begin() + kLow, values.begin() + kHigh, 1.f);
        const float binWidth = kSampleRate / static_cast<float>(kBins);

        const auto full = collect<OccupiedBandwidth>({{"fraction", 1.0}}, {densityRecord(values)});
        expect(eq(full.size(), 1UZ));
        if (full.empty()) {
            return;
        }
        const float measured = full.front().signal_values[0UZ];
        const float designed = static_cast<float>(kHigh - kLow - 1UZ) * binWidth;
        expect(std::abs(measured - designed) < 2.f * binWidth) << "at fraction 1 the band is the whole occupied span, measured " << measured << " against " << designed;

        const auto ninetyNine = collect<OccupiedBandwidth>({{"fraction", 0.99}}, {densityRecord(values)});
        expect(!ninetyNine.empty());
        if (!ninetyNine.empty()) {
            const float narrowed = ninetyNine.front().signal_values[0UZ];
            expect(narrowed < measured) << "trimming a percent of the power narrows the band";
            expect(narrowed > 0.97f * measured) << "but only by about a percent of it";
        }
    };

    "criterion 5: a shaped noise band of stated transition reads its design bandwidth"_test = [] {
        // A rectangle with no transition and no noise is not the fixture criterion 5 asks for. This band is seeded
        // noise shaped by a raised-cosine edge of a stated width, which is what a designed lowpass gives a spectrum;
        // the claim is then that the 99 % bandwidth lands within that transition width of the design bandwidth.
        constexpr double      kBandBins       = 400.;
        constexpr double      kTransitionBins = 40.;
        constexpr std::size_t kTrials         = 32UZ;

        const auto shape = [](std::size_t k) {
            const double d     = std::abs(static_cast<double>(k) - static_cast<double>(kBins / 2UZ));
            const double inner = 0.5 * (kBandBins - kTransitionBins);
            const double outer = 0.5 * (kBandBins + kTransitionBins);
            if (d <= inner) {
                return 1.;
            }
            if (d >= outer) {
                return 1e-6; // a real receiver's floor rather than a mathematical zero
            }
            return 0.5 * (1. + std::cos(std::numbers::pi * (d - inner) / kTransitionBins));
        };

        ExponentialNoise                noise;
        std::vector<gr::DataSet<float>> band;
        band.reserve(kTrials);
        for (std::size_t r = 0UZ; r < kTrials; ++r) {
            std::vector<float> values(kBins);
            for (std::size_t k = 0UZ; k < kBins; ++k) {
                values[k] = static_cast<float>(shape(k)) * noise();
            }
            band.push_back(densityRecord(std::move(values)));
        }

        const auto out = collect<OccupiedBandwidth>({{"fraction", 0.99}}, std::move(band));
        expect(eq(out.size(), kTrials));
        if (out.empty()) {
            return;
        }
        const float binWidth = kSampleRate / static_cast<float>(kBins);
        double      mean     = 0.;
        double      lowest   = 1e300;
        double      highest  = 0.;
        for (const auto& record : out) {
            const double bins = static_cast<double>(record.signal_values[0UZ]) / static_cast<double>(binWidth);
            mean += bins;
            lowest  = std::min(lowest, bins);
            highest = std::max(highest, bins);
        }
        mean /= static_cast<double>(out.size());
        std::println("criterion 5: 99 % bandwidth of a {:.0f}-bin band with a {:.0f}-bin transition: mean {:.2f} bins over {} trials, range {:.2f} to {:.2f}", kBandBins, kTransitionBins, mean, out.size(), lowest, highest);

        expect(std::abs(mean - kBandBins) < kTransitionBins) << "the 99 % bandwidth must land within the transition width of the design, measured " << mean;
        expect(highest - lowest < 2. * kTransitionBins) << "and no single realization may wander further than the transition itself, spread " << (highest - lowest);
    };

    "criterion 6: every detection and bandwidth record passes the tier's admission predicates"_test = [] {
        std::vector<float> withPeaks(kBins, 1e-6f);
        for (const std::size_t at : {200UZ, 512UZ, 800UZ}) {
            for (std::size_t k = at - 3UZ; k <= at + 3UZ; ++k) {
                withPeaks[k] += static_cast<float>(std::exp(-0.5 * static_cast<double>((k - at) * (k - at))));
            }
        }

        std::vector<gr::DataSet<float>> emitted;
        for (const auto& record : collect<PeakDetect>({{"threshold_db", 20.0}}, {densityRecord(withPeaks)})) {
            emitted.push_back(record);
        }
        for (const auto& record : collect<CfarDetect>({{"n_train", gr::Size_t{16U}}, {"n_guard", gr::Size_t{2U}}, {"pfa", 1e-3}}, {densityRecord(withPeaks)})) {
            emitted.push_back(record);
        }
        for (const auto& record : collect<OccupiedBandwidth>({{"fraction", 0.99}}, {densityRecord(withPeaks)})) {
            emitted.push_back(record);
        }
        expect(eq(emitted.size(), 3UZ)) << "all three blocks have to have emitted a record to judge";

        for (std::size_t r = 0UZ; r < emitted.size(); ++r) {
            const auto& ds     = emitted[r];
            const char* reason = admissionFailure(ds);
            expect(reason == nullptr) << "record " << r << " would be rejected as " << (reason == nullptr ? "" : reason);
            expect(eq(ds.meta_information.size(), ds.signal_names.size())) << "record " << r << " does not state the conditions on every channel";

            const auto& meta = ds.meta_information[0UZ];
            expect(meta.contains(std::pmr::string("sample_rate"))) << "record " << r << " states no rate";
            expect(meta.contains(std::pmr::string("sample_start"))) << "record " << r << " states no place in the stream";

            const auto& axis = ds.axis_values[0UZ];
            expect(eq(axis.size(), static_cast<std::size_t>(ds.extents[0UZ]))) << "record " << r << " has an axis of a different length from its extent";
            if (axis.size() >= 2UZ) {
                const float step = axis[1UZ] - axis[0UZ];
                for (std::size_t k = 1UZ; k < axis.size(); ++k) {
                    expect(std::abs((axis[k] - axis[k - 1UZ]) - step) <= 1e-6f * std::abs(step)) << "record " << r << " axis is not uniform at " << k;
                }
            }
        }
    };

    "the occupied bandwidth is pollable, and reads nothing before the first record or after a restart"_test = [] {
        OccupiedBandwidth block({{"fraction", 0.99}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();
        expect(eq(block.nRecords(), std::uint64_t{0ULL})) << "a poll before any record says so rather than inventing a number";
        expect(eq(block.bandwidth(), 0.));

        block._slot.publish({1234.0, -600.0, 634.0}, 5ULL); // what a first run leaves behind
        expect(eq(block.nRecords(), std::uint64_t{5ULL}));
        block.start();
        expect(eq(block.nRecords(), std::uint64_t{0ULL})) << "a second run does not report the first run's records";
        expect(eq(block.bandwidth(), 0.)) << "nor the first run's bandwidth";
        expect(eq(block.lowerEdge(), 0.) && eq(block.upperEdge(), 0.));
    };

    "the scalar record carries the source record's own place in the stream"_test = [] {
        const auto records = collect<OccupiedBandwidth>({{"fraction", 0.99}}, {densityRecord(std::vector<float>(kBins, 1.f))});
        expect(eq(records.size(), 1UZ));
        if (records.empty()) {
            return;
        }
        const auto& ds = records.front();
        expect(eq(ds.extents.size(), 1UZ) && eq(ds.extents[0UZ], 1));
        expect(eq(ds.signal_names.size(), 3UZ) && eq(ds.signal_values.size(), 3UZ));
        expect(eq(ds.axis_names[0UZ], std::string("Measurement")));
        expect(eq(ds.axis_units[0UZ], std::string("index")));
        expect(eq(ds.meta_information.size(), 3UZ)) << "every channel states the conditions, so one signal is enough to read";
        const auto& meta = ds.meta_information[0UZ];
        expect(meta.contains(std::pmr::string("fraction")));
        const auto start = meta.find(std::pmr::string("sample_start"));
        expect(start != meta.end()) << "the record is placed where the density record it measured was";
        if (start != meta.end()) {
            const auto* index = start->second.get_if<std::uint64_t>();
            expect(index != nullptr && *index == 7ULL) << "and at that record's own first input sample";
        }
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
