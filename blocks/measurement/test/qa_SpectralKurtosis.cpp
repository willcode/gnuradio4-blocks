#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/measurement/SpectralKurtosis.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_sk {

using gr::blocks::measurement::SpectralKurtosis;
using gr::blocks::measurement::WelchPsd;
namespace shim = gr::blocks::testing::span;

/// @brief Seeded uniform, exponential, Gamma and Gaussian draws from xorshift64.
struct Rng {
    std::uint64_t state;

    explicit Rng(std::uint64_t seed = 0xd1b54a32d192ed03ULL) : state(seed) {}

    [[nodiscard]] double uniform() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return (static_cast<double>(state >> 11U) + 0.5) / static_cast<double>(1ULL << 53U);
    }

    /// @brief One Exponential(1) draw.
    [[nodiscard]] double exponential() { return -std::log(uniform()); }

    /// @brief One Gamma(d, 1) draw for integer d, as the sum of d independent Exponential(1) draws.
    [[nodiscard]] double gammaInt(std::size_t d) {
        double total = 0.;
        for (std::size_t k = 0UZ; k < d; ++k) {
            total += exponential();
        }
        return total;
    }

    /// @brief One standard-normal draw, Box-Muller.
    [[nodiscard]] double normal() {
        const double radius = std::sqrt(-2. * std::log(uniform()));
        const double angle  = 2. * std::numbers::pi * uniform();
        return radius * std::cos(angle);
    }

    /// @brief `|A + n|^2` with `n` circular complex Gaussian of total power `noisePower`: one periodogram bin of a
    /// carrier of amplitude `A` in noise.
    [[nodiscard]] double cwBin(double amplitude, double noisePower) {
        const double sigma = std::sqrt(noisePower / 2.);
        const double re    = amplitude + sigma * normal();
        const double im    = sigma * normal();
        return re * re + im * im;
    }
};

/// @brief Emits a fixed list of records, `perCall` at a time, then ends the stream.
struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<gr::DataSet<float>, gr::Async> out;

    std::vector<gr::DataSet<float>> records{};
    std::size_t                     perCall = 4096UZ;
    std::size_t                     at      = 0UZ;

    GR_MAKE_REFLECTABLE(RecordSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= records.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        const std::size_t take = std::min({perCall, records.size() - at, outSpan.size()});
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

/// @brief A density record of `values.size()` bins with five of the metadata keys `WelchPsd` writes, in its types.
[[nodiscard]] gr::DataSet<float> spectrum(std::vector<float> values, std::uint64_t nAveraged, double overlap, std::uint64_t sampleStart = 0ULL) {
    gr::DataSet<float> ds;
    const std::size_t  n = values.size();
    ds.extents           = {static_cast<std::int32_t>(n)};
    ds.layout            = gr::LayoutRight{};
    ds.axis_names        = {"Frequency"};
    ds.axis_units        = {"Hz"};
    ds.axis_values.resize(1UZ);
    ds.axis_values[0UZ].resize(n);
    for (std::size_t k = 0UZ; k < n; ++k) {
        ds.axis_values[0UZ][k] = static_cast<float>(k) - static_cast<float>(n / 2UZ);
    }
    ds.signal_names      = {"psd"};
    ds.signal_quantities = {"PowerSpectralDensity"};
    ds.signal_units      = {"1/Hz"};
    ds.signal_values     = std::move(values);
    ds.signal_ranges.resize(1UZ);
    ds.meta_information.resize(1UZ);
    ds.meta_information[0UZ] = gr::property_map{
        {std::pmr::string("sample_rate"), gr::pmt::Value(48000.f)},
        {std::pmr::string("sample_start"), gr::pmt::Value(sampleStart)},
        {std::pmr::string("n_averaged"), gr::pmt::Value(nAveraged)},
        {std::pmr::string("overlap"), gr::pmt::Value(overlap)},
        {std::pmr::string("window"), gr::pmt::Value(std::string("Hann"))},
    };
    ds.timing_events.resize(1UZ);
    return ds;
}

/// @brief The record with the metadata key @p key removed.
[[nodiscard]] gr::DataSet<float> withoutMeta(gr::DataSet<float> record, std::string_view key) {
    if (!record.meta_information.empty()) {
        record.meta_information[0UZ].erase(std::pmr::string(key));
    }
    return record;
}

/// @brief @p count copies of one record.
[[nodiscard]] std::vector<gr::DataSet<float>> repeated(const gr::DataSet<float>& record, std::size_t count) { return std::vector<gr::DataSet<float>>(count, record); }

/// @brief The records and the block's counters after one run, read while the scheduler holds the graph.
struct Run {
    std::vector<gr::DataSet<float>> records;

    std::uint64_t nRecords{};
    std::uint64_t nRefusedRecords{};
    std::uint64_t nDegenerateBins{};
    std::uint64_t worstBin{};
    double        worstBinSk{};
    double        coverage{};
};

/// @brief Runs `records` through a source, `SpectralKurtosis` and a record sink. Returns the records and the
/// counters.
[[nodiscard]] Run runWith(gr::property_map settings, const std::vector<gr::DataSet<float>>& records, std::size_t perCall = 4096UZ) {
    gr::Graph graph;
    auto&     source = graph.emplaceBlock<RecordSource>();
    auto&     block  = graph.emplaceBlock<SpectralKurtosis>(std::move(settings));
    auto&     sink   = graph.emplaceBlock<RecordSink>();
    source.records   = records;
    source.perCall   = perCall;

    if (!graph.connect<"out", "spectra">(source, block).has_value() || !graph.connect<"measurements", "in">(block, sink).has_value()) {
        return {};
    }
    gr::scheduler::Simple scheduler;
    if (!scheduler.exchange(std::move(graph)).has_value()) {
        return {};
    }
    std::ignore = scheduler.runAndWait(); // the source ends the stream

    Run result;
    result.records         = std::move(sink.records);
    result.nRecords        = block.nRecords();
    result.nRefusedRecords = block.nRefusedRecords();
    result.nDegenerateBins = block.nDegenerateBins();
    result.worstBin        = block.worstBin();
    result.worstBinSk      = block.worstBinSk();
    result.coverage        = block.coverage();
    return result;
}

[[nodiscard]] std::vector<gr::DataSet<float>> collect(gr::property_map settings, const std::vector<gr::DataSet<float>>& records, std::size_t perCall = 4096UZ) { return runWith(std::move(settings), records, perCall).records; }

/// @brief Applies the staged settings and starts the block, as the scheduler does before the first work call.
template<typename TBlock>
void startBlock(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

/// @brief Stages @p settings on a started block and applies them. Returns true when the block refuses them.
[[nodiscard]] bool refuses(SpectralKurtosis& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    try {
        std::ignore = block.settings().applyStagedParameters();
    } catch (const gr::exception&) {
        return true;
    }
    return false;
}

/// @brief One work call over @p rows. Returns the records it emits.
[[nodiscard]] std::vector<gr::DataSet<float>> feed(SpectralKurtosis& block, std::span<const gr::DataSet<float>> rows) {
    std::vector<gr::DataSet<float>>      out(rows.size() + 1UZ);
    shim::InputSpan<gr::DataSet<float>>  inSpan{rows};
    shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(out)};
    std::ignore = block.processBulk(inSpan, outSpan);
    out.resize(outSpan.count);
    return out;
}

/// @brief The record with the metadata key @p key set to @p value.
template<typename V>
[[nodiscard]] gr::DataSet<float> withMeta(gr::DataSet<float> record, std::string_view key, V value) {
    record.meta_information[0UZ].insert_or_assign(std::pmr::string(key), gr::pmt::Value(value));
    return record;
}

/// @brief The large-`M` limit `L` of `SK` at `d = 1` for a carrier of bin power `S`, present a fraction `p` of the
/// time, over noise of unit power, and the first-order coefficient `b1` of its finite-`M` bias, `E[SK] = L + b1/M`.
/// The mixture's raw moments are `m1`, `m2` and `m3`, with `v = m2 - m1^2`. The delta method on `M*S2/S1^2` gives
/// `b1 = 2L + 3 m2 v/m1^4 - 2 (m3 - m1 m2)/m1^3`. On noise alone `b1` is zero.
struct PulsedLimit {
    double limit{0.};
    double bias{0.};
};

[[nodiscard]] constexpr PulsedLimit pulsedLimit(double p, double s) noexcept {
    const double m1 = p * (s + 1.) + (1. - p);
    const double m2 = p * (s * s + 4. * s + 2.) + (1. - p) * 2.;
    const double m3 = p * (s * s * s + 9. * s * s + 18. * s + 6.) + (1. - p) * 6.;
    const double v  = m2 - m1 * m1;
    const double l  = m2 / (m1 * m1) - 1.;
    return {.limit = l, .bias = 2. * l + 3. * m2 * v / (m1 * m1 * m1 * m1) - 2. * (m3 - m1 * m2) / (m1 * m1 * m1)};
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
    if (const auto* asBool = it->second.template get_if<bool>()) {
        return *asBool ? 1. : 0.;
    }
    if (const auto* asFloat = it->second.template get_if<float>()) {
        return static_cast<double>(*asFloat);
    }
    if (const auto* asDouble = it->second.template get_if<double>()) {
        return *asDouble;
    }
    return fallback;
}

[[nodiscard]] std::string metaString(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return {};
    }
    if (const auto* asString = it->second.template get_if<std::pmr::string>()) {
        return std::string(std::string_view(*asString));
    }
    return {};
}

/// @brief `m` records of `nBins` independent Gamma(d) draws, the distribution of a noise bin summed over `d`
/// periodograms of circular complex Gaussian noise.
[[nodiscard]] std::vector<gr::DataSet<float>> noiseRows(Rng& rng, std::size_t m, std::size_t nBins, std::size_t d) {
    std::vector<gr::DataSet<float>> rows;
    rows.reserve(m);
    for (std::size_t row = 0UZ; row < m; ++row) {
        std::vector<float> values(nBins);
        for (float& v : values) {
            v = static_cast<float>(rng.gammaInt(d));
        }
        rows.push_back(spectrum(std::move(values), static_cast<std::uint64_t>(d), 0.0));
    }
    return rows;
}

[[nodiscard]] double mean(const std::vector<float>& values) {
    double total = 0.;
    for (const float v : values) {
        total += static_cast<double>(v);
    }
    return values.empty() ? 0. : total / static_cast<double>(values.size());
}

[[nodiscard]] double sampleStdDev(const std::vector<float>& values, double meanValue) {
    if (values.size() < 2UZ) {
        return 0.;
    }
    double sumSq = 0.;
    for (const float v : values) {
        const double d = static_cast<double>(v) - meanValue;
        sumSq += d * d;
    }
    return std::sqrt(sumSq / static_cast<double>(values.size() - 1UZ));
}

} // namespace qa_sk

const boost::ut::suite<"SpectralKurtosis"> spectralKurtosisTests = [] {
    using namespace boost::ut;
    using namespace qa_sk;

    // SK reads 1 on noise at d = 1, 4 and 16. Each bin of one accumulation is an independent reading. The mean over
    // the bins lies within four standard errors of 1, and their spread lies within 15% of the closed form.
    "spectral kurtosis reads 1 on noise, with the closed-form spread"_test = [] {
        constexpr std::size_t kM    = 64UZ;
        constexpr std::size_t kBins = 2048UZ;
        Rng                   rng(0x2545f4914f6cdd1dULL);

        // The closed-form spreads at M = 64 are literals: 0.24251, 0.19728 and 0.18321 at d = 1, 4 and 16.
        struct Shape {
            std::size_t d;
            double      spread;
        };
        const std::array<Shape, 3UZ> shapes{{{1UZ, 0.24251}, {4UZ, 0.19728}, {16UZ, 0.18321}}};

        for (const Shape& shape : shapes) {
            const auto rows    = noiseRows(rng, kM, kBins, shape.d);
            const auto records = collect({{"n_spectra", gr::Size_t{kM}}, {"shape", gr::Size_t{static_cast<gr::Size_t>(shape.d)}}}, rows, 4096UZ);
            expect(eq(records.size(), 1UZ)) << std::format("d={}", shape.d);
            if (records.empty()) {
                continue;
            }
            const auto&  values     = records.front().signal_values;
            const double measured   = mean(values);
            const double measuredSd = sampleStdDev(values, measured);
            const double sem        = 4. * shape.spread / std::sqrt(static_cast<double>(kBins));

            std::println("noise: d={} mean {:.5f} (band 1 +- {:.5f}), spread {:.5f} (closed form {:.5f})", shape.d, measured, sem, measuredSd, shape.spread);
            expect(std::abs(measured - 1.) < sem) << std::format("d={}: mean SK too far from 1", shape.d);
            expect(std::abs(measuredSd - shape.spread) < 0.15 * shape.spread) << std::format("d={}: spread too far from the closed-form value", shape.d);

            // The record states the closed-form spread and the expectation.
            std::println("noise: d={} record sk_std {:.5f}, sk_expectation {:.5f}", shape.d, metaNumber(records.front(), "sk_std"), metaNumber(records.front(), "sk_expectation"));
            expect(std::abs(metaNumber(records.front(), "sk_std") - shape.spread) < 5e-5) << std::format("d={}: the record's sk_std must be the closed-form spread", shape.d);
            expect(eq(metaNumber(records.front(), "sk_expectation"), 1.)) << "sk_expectation is exactly 1, at every M and every d";
        }
    };

    // A noiseless tone gives identical bin powers and SK = 0 at every M. The comparison is exact.
    "a noiseless tone reads SK = 0 exactly, at every M"_test = [] {
        for (const std::size_t m : {8UZ, 64UZ, 1024UZ}) {
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t row = 0UZ; row < m; ++row) {
                rows.push_back(spectrum(std::vector<float>{3.5f, 3.5f, 3.5f, 3.5f}, 1ULL, 0.0));
            }
            const auto records = collect({{"n_spectra", gr::Size_t{static_cast<gr::Size_t>(m)}}, {"shape", gr::Size_t{1U}}}, rows, 4096UZ);
            expect(eq(records.size(), 1UZ)) << std::format("M={}", m);
            if (!records.empty()) {
                for (const float v : records.front().signal_values) {
                    expect(eq(v, 0.f)) << std::format("M={}: identical bin powers must give SK exactly 0", m);
                }
            }
        }
    };

    // A carrier in noise at bin ratios rho = 1, 2, 10 and 100 reads its closed form within 0.01 at M = 1024. A pulsed
    // signal reads its large-M limit with the first-order bias, within four standard errors of the bin mean.
    "a carrier in noise reads its closed form, and a pulsed signal reads its limit with the first-order bias"_test = [] {
        constexpr std::size_t kM    = 1024UZ;
        constexpr std::size_t kBins = 400UZ;
        Rng                   rng(0x9e3779b97f4a7c15ULL);

        const auto cwSk = [](double rho) { return (2. * rho + 1.) / ((rho + 1.) * (rho + 1.)); };
        for (const double rho : {1., 2., 10., 100.}) {
            std::vector<gr::DataSet<float>> rows;
            const double                    amplitude = std::sqrt(rho); // N0 = 1
            for (std::size_t row = 0UZ; row < kM; ++row) {
                std::vector<float> values(kBins);
                for (float& v : values) {
                    v = static_cast<float>(rng.cwBin(amplitude, 1.));
                }
                rows.push_back(spectrum(std::move(values), 1ULL, 0.0));
            }
            const auto records = collect({{"n_spectra", gr::Size_t{kM}}, {"shape", gr::Size_t{1U}}}, rows, 4096UZ);
            expect(eq(records.size(), 1UZ)) << std::format("rho={}", rho);
            if (records.empty()) {
                continue;
            }
            const double measured = mean(records.front().signal_values);
            const double expected = cwSk(rho);
            std::println("CW rho={:g} measured {:.5f}, closed form {:.5f}", rho, measured, expected);
            expect(std::abs(measured - expected) < 0.01) << std::format("rho={}: CW arm outside 0.01 absolute", rho);
        }

        // A pulsed signal at S/N0 = 100 and p = 0.1, at M = 1024 and M = 8192, and at p = 0.25 and 0.5 at M = 8192.
        const double kSN0       = 100.;
        const double kAmp       = std::sqrt(kSN0);
        const auto   pulsedMean = [&](double p, std::size_t m) -> std::pair<double, double> {
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t row = 0UZ; row < m; ++row) {
                std::vector<float> values(kBins);
                for (float& v : values) {
                    v = rng.uniform() < p ? static_cast<float>(rng.cwBin(kAmp, 1.)) : static_cast<float>(rng.gammaInt(1UZ));
                }
                rows.push_back(spectrum(std::move(values), 1ULL, 0.0));
            }
            const auto records = collect({{"n_spectra", gr::Size_t{static_cast<gr::Size_t>(m)}}, {"shape", gr::Size_t{1U}}}, rows, 4096UZ);
            if (records.empty()) {
                return {0., 0.};
            }
            const double measured = mean(records.front().signal_values);
            return {measured, sampleStdDev(records.front().signal_values, measured) / std::sqrt(static_cast<double>(kBins))};
        };

        struct Arm {
            double      p;
            std::size_t m;
        };
        for (const Arm arm : {Arm{0.1, 1024UZ}, Arm{0.1, 8192UZ}, Arm{0.25, 8192UZ}, Arm{0.5, 8192UZ}}) {
            const PulsedLimit closed             = pulsedLimit(arm.p, kSN0);
            const double      expected           = closed.limit + closed.bias / static_cast<double>(arm.m);
            const auto [measured, standardError] = pulsedMean(arm.p, arm.m);
            std::println("pulsed p={:g} M={}: measured {:.4f}, L + b1/M = {:.4f} (L {:.4f}), band +- {:.4f}", arm.p, arm.m, measured, expected, closed.limit, 4. * standardError);
            expect(std::abs(measured - expected) < 4. * standardError) << std::format("p={} M={}: outside four standard errors of L + b1/M", arm.p, arm.m);
        }
    };

    // The block folds a record only when it carries `overlap` and that value is exactly zero.
    "a record is folded only when its overlap metadata is present and zero"_test = [] {
        const gr::DataSet<float> clean = spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0);

        {
            std::vector<gr::DataSet<float>> rows{spectrum({1.f, 2.f, 3.f}, 1ULL, 0.5)};
            for (std::size_t k = 0UZ; k < 8UZ; ++k) {
                rows.push_back(clean);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "the overlapping record must be refused and counted";
            expect(eq(r.records.size(), 1UZ)) << "the eight remaining records still complete the n_spectra=8 accumulation";
        }
        {
            std::vector<gr::DataSet<float>> rows{withoutMeta(clean, "overlap")};
            for (std::size_t k = 0UZ; k < 8UZ; ++k) {
                rows.push_back(clean);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "a record without overlap metadata is refused";
            expect(eq(r.records.size(), 1UZ));
        }
    };

    // The other refusals. Each is counted, and the block measures again after each.
    "the further refusals: shape, absent n_averaged, a negative bin, a changed bin count"_test = [] {
        const gr::DataSet<float> shape1 = spectrum({1.f, 2.f, 3.f, 4.f}, 1ULL, 0.0);
        const gr::DataSet<float> shape4 = spectrum({1.f, 2.f, 3.f, 4.f}, 4ULL, 0.0);

        { // a `shape` setting that disagrees with the record it receives
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{4U}}}, repeated(shape1, 8UZ));
            expect(eq(r.nRefusedRecords, std::uint64_t{8ULL})) << "every record disagreeing with the shape setting is refused";
            expect(eq(r.records.size(), 0UZ));
        }
        { // shape = 0 reads the shape from the record, and a record without `n_averaged` is refused
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, repeated(withoutMeta(shape1, "n_averaged"), 8UZ));
            expect(eq(r.nRefusedRecords, std::uint64_t{8ULL}));
            expect(eq(r.records.size(), 0UZ));
        }
        { // a shape change inside an accumulation discards it and refuses only that record
            std::vector<gr::DataSet<float>> rows = repeated(shape1, 2UZ);
            rows.push_back(shape4);
            for (const auto& row : repeated(shape4, 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "only the record that disagreed is refused";
            expect(eq(r.records.size(), 1UZ)) << "the eight records at the new shape complete a fresh accumulation";
            if (!r.records.empty()) {
                expect(eq(metaNumber(r.records.front(), "shape"), 4.)) << "and that record is measured under the new shape";
            }
        }
        { // a negative bin
            std::vector<gr::DataSet<float>> rows{spectrum({1.f, -1.f, 3.f, 4.f}, 1ULL, 0.0)};
            for (const auto& row : repeated(shape1, 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "a negative bin refuses the record and folds nothing";
            expect(eq(r.records.size(), 1UZ));
        }
        { // a bin count that does not match the accumulation in progress
            std::vector<gr::DataSet<float>> rows = repeated(shape1, 2UZ);
            rows.push_back(spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0));
            for (const auto& row : repeated(spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0), 8UZ)) {
                rows.push_back(row);
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{8U}}}, rows);
            expect(eq(r.nRefusedRecords, std::uint64_t{1ULL}));
            expect(eq(r.records.size(), 1UZ));
            if (!r.records.empty()) {
                expect(eq(r.records.front().signal_values.size(), 3UZ)) << "the accumulation restarts at the new bin count";
            }
        }
    };

    "n_spectra below 8 or above 2^20 refuses when applied"_test = [] {
        const auto staged = [](gr::property_map settings) {
            SpectralKurtosis block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { staged({{"n_spectra", gr::Size_t{7U}}}); })) << "below the floor of 8";
        expect(throws([&] { staged({{"n_spectra", gr::Size_t{(1U << 20U) + 1U}}}); })) << "above the ceiling of 2^20";
        expect(nothrow([&] { staged({{"n_spectra", gr::Size_t{8U}}}); })) << "the floor itself is admitted";
        expect(nothrow([&] { staged({{"n_spectra", gr::Size_t{1U << 20U}}}); })) << "the ceiling itself is admitted";
    };

    // The record copies the input axis and states its conditions. `valid` is false only when every bin is degenerate.
    "the record states its conditions, and a dead bin leaves it valid"_test = [] {
        constexpr std::size_t kM = 8UZ;

        { // one dead bin among three is counted and written 0, and the record stays valid
            const Run r = runWith({{"n_spectra", gr::Size_t{kM}}}, repeated(spectrum({1.f, 2.f, 0.f}, 1ULL, 0.0, 4242ULL), kM));
            expect(eq(r.records.size(), 1UZ));
            expect(eq(r.nDegenerateBins, std::uint64_t{1ULL})) << "the all-zero bin is counted degenerate";
            if (!r.records.empty()) {
                const auto& record = r.records.front();
                expect(eq(record.signal_values.at(2UZ), 0.f)) << "a bin with no defined ratio is written 0";
                expect(eq(metaNumber(record, "valid"), 1.)) << "a single dead bin leaves the record valid";
                expect(eq(metaNumber(record, "n_spectra"), 8.));
                expect(eq(metaNumber(record, "shape"), 1.));
                expect(eq(metaNumber(record, "sample_start"), 4242.)) << "the first input record's sample_start";
                expect(std::abs(metaNumber(record, "sample_rate") - 48000.) < 1e-3) << "the rate is copied from the input record";
                expect(eq(metaString(record, "window"), std::string("Hann"))) << "the window name is copied from the input record";
                expect(std::abs(metaNumber(record, "sk_std") - 0.5766) < 1e-4) << "the closed-form spread at M=8, d=1 is 0.5766";
                expect(eq(record.signal_names.at(0UZ), std::string("spectral_kurtosis")));
                expect(eq(record.axis_values.at(0UZ).size(), 3UZ)) << "the input's frequency axis";
            }
        }
        { // with every bin dead the record is marked invalid
            const Run r = runWith({{"n_spectra", gr::Size_t{kM}}}, repeated(spectrum({0.f, 0.f, 0.f}, 1ULL, 0.0), kM));
            expect(eq(r.records.size(), 1UZ));
            expect(eq(r.nDegenerateBins, std::uint64_t{3ULL}));
            if (!r.records.empty()) {
                expect(eq(metaNumber(r.records.front(), "valid"), 0.)) << "a record whose every bin is degenerate is invalid";
            }
        }
        { // the worst bin lies furthest from 1, on either side
            // Bin 0 alternates 0 and 1 over M = 8. S1 = 4 and S2 = 4, M*S2/S1^2 = 2, and SK = (9/7)*(2-1) = 1.2857.
            // Bin 1 is constant, and its SK is 0. Bin 1 departs further from 1.
            std::vector<gr::DataSet<float>> rows;
            for (std::size_t k = 0UZ; k < kM; ++k) {
                rows.push_back(spectrum({k % 2UZ == 0UZ ? 0.f : 1.f, 2.5f}, 1ULL, 0.0));
            }
            const Run r = runWith({{"n_spectra", gr::Size_t{kM}}}, rows);
            expect(eq(r.records.size(), 1UZ));
            std::println("record: worstBin {} worstBinSk {:.5f}, bin 0 SK {:.5f} (closed form 9/7 = {:.5f})", r.worstBin, r.worstBinSk, r.records.empty() ? 0.f : r.records.front().signal_values.at(0UZ), 9. / 7.);
            expect(eq(r.worstBin, std::uint64_t{1ULL})) << "the constant bin departs from 1 by the most";
            expect(std::abs(r.worstBinSk) < 1e-6) << "and its SK is exactly 0";
            if (!r.records.empty()) {
                expect(std::abs(static_cast<double>(r.records.front().signal_values.at(0UZ)) - 9. / 7.) < 1e-5) << "the alternating bin reads the closed 9/7";
            }
        }
    };

    "emit_records = false measures without publishing"_test = [] {
        Rng       rng(0x3c6ef372fe94f82bULL);
        const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}, {"emit_records", false}}, noiseRows(rng, 16UZ, 8UZ, 1UZ));
        expect(eq(r.records.size(), 0UZ)) << "no record is published";
        expect(eq(r.nRecords, std::uint64_t{2ULL})) << "and both accumulations are measured and counted";
        expect(eq(r.nRefusedRecords, std::uint64_t{0ULL}));
    };

    // The same input records, delivered a different number per call, give bit-identical records and counters.
    "the records and counters do not depend on how many input records arrive per call"_test = [] {
        Rng                             rng(0x1122334455667788ULL);
        std::vector<gr::DataSet<float>> rows = noiseRows(rng, 96UZ, 64UZ, 1UZ);
        rows.push_back(spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0)); // one refusal, for the refusal counter
        const auto settings  = [] { return gr::property_map{{"n_spectra", gr::Size_t{32U}}, {"shape", gr::Size_t{1U}}}; };
        const Run  reference = runWith(settings(), rows, 4096UZ);
        expect(!reference.records.empty());
        expect(eq(reference.nRefusedRecords, std::uint64_t{1ULL}));

        for (const std::size_t perCall : {1UZ, 7UZ, 1000UZ, 12345UZ}) {
            const Run candidate = runWith(settings(), rows, perCall);
            expect(eq(candidate.records.size(), reference.records.size())) << std::format("perCall {}", perCall);
            expect(eq(candidate.nRecords, reference.nRecords)) << std::format("perCall {}: nRecords", perCall);
            expect(eq(candidate.nRefusedRecords, reference.nRefusedRecords)) << std::format("perCall {}: nRefusedRecords", perCall);
            expect(eq(candidate.nDegenerateBins, reference.nDegenerateBins)) << std::format("perCall {}: nDegenerateBins", perCall);
            expect(eq(candidate.worstBin, reference.worstBin)) << std::format("perCall {}: worstBin", perCall);
            expect(eq(candidate.worstBinSk, reference.worstBinSk)) << std::format("perCall {}: worstBinSk", perCall);
            for (std::size_t r = 0UZ; r < std::min(candidate.records.size(), reference.records.size()); ++r) {
                expect(std::ranges::equal(candidate.records[r].signal_values, reference.records[r].signal_values)) << std::format("perCall {}: record {}", perCall, r);
            }
        }
    };

    // A started block takes a refused application and then more records. Its readings match a block that saw no
    // refusal, and a valid n_spectra after the refusal applies.
    "a refused n_spectra on a running block changes nothing it computes, and a later valid one applies"_test = [] {
        Rng                                       rng(0x243f6a8885a308d3ULL);
        const std::vector<gr::DataSet<float>>     rows = noiseRows(rng, 40UZ, 16UZ, 1UZ);
        const std::span<const gr::DataSet<float>> all(rows);
        const auto                                settings = [] { return gr::property_map{{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}; };

        SpectralKurtosis reference(settings());
        startBlock(reference);
        expect(eq(feed(reference, all.first(8UZ)).size(), 1UZ));
        const auto referenceTail = feed(reference, all.subspan(8UZ));
        expect(eq(referenceTail.size(), 4UZ));

        const std::array<gr::property_map, 3UZ> refusals{{
            {{"n_spectra", gr::Size_t{0U}}},
            {{"n_spectra", gr::Size_t{(1U << 20U) + 1U}}},
            {{"n_spectra", gr::Size_t{1U}}, {"emit_records", false}, {"shape", gr::Size_t{4U}}},
        }};
        for (std::size_t r = 0UZ; r < refusals.size(); ++r) {
            SpectralKurtosis block(settings());
            startBlock(block);
            expect(eq(feed(block, all.first(8UZ)).size(), 1UZ));
            expect(refuses(block, refusals[r])) << std::format("refusal {} is refused", r);
            const auto tail = feed(block, all.subspan(8UZ));
            expect(eq(tail.size(), referenceTail.size())) << std::format("refusal {}: the same readings", r);
            for (std::size_t k = 0UZ; k < std::min(tail.size(), referenceTail.size()); ++k) {
                expect(std::ranges::equal(tail[k].signal_values, referenceTail[k].signal_values)) << std::format("refusal {}: reading {}", r, k);
                expect(eq(metaNumber(tail[k], "n_spectra"), 8.)) << std::format("refusal {}: reading {} n_spectra", r, k);
                expect(eq(metaNumber(tail[k], "sk_std"), metaNumber(referenceTail[k], "sk_std"))) << std::format("refusal {}: reading {} sk_std", r, k);
            }
            expect(eq(block.nRecords(), reference.nRecords())) << std::format("refusal {}: nRecords", r);
            expect(eq(block.nRefusedRecords(), std::uint64_t{0ULL})) << std::format("refusal {}: no record refused", r);
        }

        SpectralKurtosis block(settings());
        startBlock(block);
        std::ignore = feed(block, all.first(8UZ));
        expect(refuses(block, {{"n_spectra", gr::Size_t{0U}}}));
        expect(eq(block.n_spectra.value, gr::Size_t{0U})) << "the member holds the refused value";
        expect(!refuses(block, {{"n_spectra", gr::Size_t{16U}}})) << "a valid n_spectra is accepted";
        const auto afterChange = feed(block, all.subspan(8UZ));
        expect(eq(afterChange.size(), 2UZ)) << "32 records make two readings of 16";
        for (const auto& record : afterChange) {
            expect(eq(metaNumber(record, "n_spectra"), 16.));
        }
    };

    // Welch estimates of real Gaussian noise at fft_size 64: 33 one-sided bins, bin 0 at DC and bin 32 at Nyquist.
    // Those two bins are real, with Gamma shape 1/2 in one periodogram. The block reads them at that shape. They read
    // 1 within four standard errors of the mean over the readings. Over 200 readings the worst bin lies at an edge no
    // more often than chance allows. Under chance the count is binomial with p = 2/33, and the bound is four standard
    // deviations above its mean.
    "on a one-sided spectrum of real noise the DC and Nyquist bins read 1, and the worst bin is no edge bin more than chance"_test = [] {
        constexpr std::size_t kFft      = 64UZ;
        constexpr std::size_t kBins     = kFft / 2UZ + 1UZ;
        constexpr std::size_t kM        = 64UZ;
        constexpr std::size_t kReadings = 200UZ;

        Rng                rng(0x6c62272e07bb0142ULL);
        std::vector<float> samples(kReadings * kM * kFft + 1UZ);
        for (float& sample : samples) {
            sample = static_cast<float>(rng.normal());
        }

        WelchPsd<float> welch({{"fft_size", gr::Size_t{kFft}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.0}, {"sample_rate", 1.f}, {"one_record_per_call", false}});
        startBlock(welch);
        std::vector<gr::DataSet<float>> spectra(kReadings * kM + 4UZ);
        {
            shim::InputSpan<float>               inSpan{std::span<const float>(samples)};
            shim::OutputSpan<gr::DataSet<float>> outSpan{std::span<gr::DataSet<float>>(spectra)};
            std::ignore = welch.processBulk(inSpan, outSpan);
            spectra.resize(outSpan.count);
        }
        expect(fatal(eq(spectra.size(), kReadings * kM)));
        expect(fatal(eq(spectra.front().signal_values.size(), kBins)));

        SpectralKurtosis sk({{"n_spectra", gr::Size_t{kM}}});
        startBlock(sk);
        double      dc       = 0.;
        double      nyquist  = 0.;
        std::size_t edgeHits = 0UZ;
        for (std::size_t r = 0UZ; r < kReadings; ++r) {
            const auto readings = feed(sk, std::span<const gr::DataSet<float>>(spectra).subspan(r * kM, kM));
            expect(fatal(eq(readings.size(), 1UZ)));
            dc += static_cast<double>(readings.front().signal_values.front()) / static_cast<double>(kReadings);
            nyquist += static_cast<double>(readings.front().signal_values.back()) / static_cast<double>(kReadings);
            edgeHits += sk.worstBin() == 0UZ || sk.worstBin() == kBins - 1UZ ? 1UZ : 0UZ;
            if (r == 0UZ) {
                const auto& record = readings.front();
                expect(eq(metaNumber(record, "one_sided"), 1.)) << "the record keeps the input's sidedness";
                expect(std::abs(metaNumber(record, "sk_std_edge") - gr::measurement::spectralKurtosisSpread(kM, 0.5)) < 1e-12) << "the record states the edge bins' spread";
            }
        }
        const double edgeSpread = gr::measurement::spectralKurtosisSpread(kM, 0.5);
        const double band       = 4. * edgeSpread / std::sqrt(static_cast<double>(kReadings));
        const double chance     = 2. / static_cast<double>(kBins);
        const double hitBound   = static_cast<double>(kReadings) * chance + 4. * std::sqrt(static_cast<double>(kReadings) * chance * (1. - chance));
        std::println("one-sided real noise: DC {:.4f}, Nyquist {:.4f} (band 1 +- {:.4f}); worst bin at an edge in {} of {} readings (bound {:.1f})", dc, nyquist, band, edgeHits, kReadings, hitBound);
        expect(std::abs(dc - 1.) < band) << "the DC bin reads 1 on noise";
        expect(std::abs(nyquist - 1.) < band) << "the Nyquist bin reads 1 on noise";
        expect(static_cast<double>(edgeHits) < hitBound) << "the worst bin is an edge bin no more often than chance";
    };

    "a record that states a mode other than the mean is refused"_test = [] {
        const gr::DataSet<float>        clean = spectrum({1.f, 2.f, 3.f}, 1ULL, 0.0);
        std::vector<gr::DataSet<float>> rows  = repeated(withMeta(clean, "mode", std::string("max_hold")), 8UZ);
        for (const auto& row : repeated(withMeta(clean, "mode", std::string("mean")), 8UZ)) {
            rows.push_back(row);
        }
        const Run r = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
        expect(eq(r.nRefusedRecords, std::uint64_t{8ULL})) << "every max_hold record is refused";
        expect(eq(r.records.size(), 1UZ)) << "the eight mean records make one reading";
    };

    "a record with an infinite bin is refused"_test = [] {
        Rng  rng(0xa4093822299f31d0ULL);
        auto rows                    = noiseRows(rng, 9UZ, 16UZ, 1UZ);
        rows[3UZ].signal_values[5UZ] = std::numeric_limits<float>::infinity();
        const Run r                  = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, rows);
        expect(eq(r.nRefusedRecords, std::uint64_t{1ULL})) << "the record with the infinite bin is refused";
        expect(eq(r.records.size(), 1UZ));
        if (!r.records.empty()) {
            expect(std::ranges::all_of(r.records.front().signal_values, [](float v) { return std::isfinite(v); })) << "every bin of the reading is finite";
        }
    };

    "at the end of the stream the partial accumulation goes out, valid from eight spectra"_test = [] {
        Rng rng(0x082efa98ec4e6c89ULL);
        for (const std::size_t tail : {3UZ, 10UZ}) {
            const Run r = runWith({{"n_spectra", gr::Size_t{16U}}, {"shape", gr::Size_t{1U}}}, noiseRows(rng, 16UZ + tail, 8UZ, 1UZ));
            expect(eq(r.records.size(), 2UZ)) << std::format("tail {}: one reading and one flush", tail);
            expect(eq(r.nRecords, std::uint64_t{1ULL})) << "nRecords counts the full accumulation alone";
            if (r.records.size() == 2UZ) {
                expect(eq(metaNumber(r.records[1UZ], "n_spectra"), static_cast<double>(tail))) << "the flush states its spectra";
                expect(eq(metaNumber(r.records[1UZ], "valid"), tail >= 8UZ ? 1. : 0.)) << std::format("tail {}: valid only from eight spectra", tail);
            }
        }
    };

    "the record keeps the input's center frequency and sidedness, and the record port may stay unconnected"_test = [] {
        const gr::DataSet<float> twoSided = withMeta(spectrum({1.f, 2.f, 3.f, 4.f}, 1ULL, 0.0), "center_frequency", 1.42e9);
        const Run                plain    = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, repeated(twoSided, 8UZ));
        expect(fatal(eq(plain.records.size(), 1UZ)));
        expect(eq(metaNumber(plain.records.front(), "center_frequency"), 1.42e9)) << "the input's center frequency";
        expect(eq(metaNumber(plain.records.front(), "one_sided"), 0.)) << "a record without one_sided is read as two-sided";
        expect(eq(metaNumber(plain.records.front(), "sk_std_edge"), -1.)) << "and states no edge spread";

        // Five bins, one-sided, from a transform of eight samples: bins 0 and 4 are real.
        const gr::DataSet<float> oneSided = withMeta(withMeta(spectrum({1.f, 2.f, 3.f, 4.f, 5.f}, 1ULL, 0.0), "one_sided", true), "fft_size", std::uint64_t{8ULL});
        const Run                real     = runWith({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}}, repeated(oneSided, 8UZ));
        expect(fatal(eq(real.records.size(), 1UZ)));
        expect(eq(metaNumber(real.records.front(), "one_sided"), 1.));
        expect(std::abs(metaNumber(real.records.front(), "sk_std_edge") - gr::measurement::spectralKurtosisSpread(8UZ, 0.5)) < 1e-12);

        gr::Graph graph;
        auto&     source = graph.emplaceBlock<RecordSource>();
        auto&     block  = graph.emplaceBlock<SpectralKurtosis>({{"n_spectra", gr::Size_t{8U}}, {"shape", gr::Size_t{1U}}});
        Rng       rng(0x3f84d5b5b5470917ULL);
        source.records = noiseRows(rng, 16UZ, 8UZ, 1UZ);
        expect(fatal(graph.connect<"out", "spectra">(source, block).has_value()));
        gr::scheduler::Simple scheduler;
        expect(fatal(scheduler.exchange(std::move(graph)).has_value()));
        expect(scheduler.runAndWait().has_value()) << "the graph runs with the record port unconnected";
        expect(eq(block.nRecords(), std::uint64_t{2ULL})) << "and the block measures every accumulation";
    };
};

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
