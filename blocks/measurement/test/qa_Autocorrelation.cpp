#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
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
#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/measurement/Autocorrelation.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>

namespace qa_autocorrelation {

using gr::blocks::measurement::Autocorrelation;
using gr::blocks::measurement::PeakDetect;
using CF = std::complex<float>;

constexpr double kPi    = std::numbers::pi;
constexpr double kTwoPi = 2. * std::numbers::pi;

/// Emits a fixed sequence in bursts of a stated size, then ends the stream. The burst size is what makes chunk
/// independence testable: the same samples presented differently must produce the same records.
template<typename T>
struct BurstSource : gr::Block<BurstSource<T>> {
    gr::PortOut<T> out;

    std::vector<T> samples{};
    std::size_t    burst = 4096UZ;
    std::size_t    at    = 0UZ;

    std::vector<std::pair<std::size_t, gr::property_map>> tags{}; ///< each tag and the absolute sample it sits on, in order
    std::size_t                                           next = 0UZ;

    GR_MAKE_REFLECTABLE(BurstSource, out);

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (at >= samples.size()) {
            outSpan.publish(0UZ);
            this->requestStop();
            return gr::work::Status::DONE;
        }
        std::size_t take = std::min({burst, samples.size() - at, outSpan.size()});
        if (next < tags.size() && tags[next].first > at && tags[next].first - at < take) {
            take = tags[next].first - at; // a tag is a boundary, so a burst never straddles it
        }
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(at), take, outSpan.begin());
        if (next < tags.size() && tags[next].first == at) {
            outSpan.publishTag(tags[next].second, 0UZ);
            ++next;
        }
        at += take;
        outSpan.publish(take);
        return gr::work::Status::OK;
    }
};

/// Keeps every record a run produced, which is what the criteria read.
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

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
    return block;
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
    if (const auto* asBool = it->second.template get_if<bool>()) {
        return *asBool ? 1. : 0.;
    }
    return fallback;
}

[[nodiscard]] std::string metaText(const gr::DataSet<float>& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto& map = record.meta_information[0UZ];
    const auto  it  = map.find(std::pmr::string(key));
    if (it == map.end()) {
        return {};
    }
    const std::pmr::string* asString = it->second.template get_if<std::pmr::string>();
    return asString == nullptr ? std::string{} : std::string(*asString);
}

[[nodiscard]] bool hasKey(const gr::DataSet<float>& record, std::string_view key) { return !record.meta_information.empty() && record.meta_information[0UZ].contains(std::pmr::string(key)); }

/// @brief Run one source-through-block-into-sink graph and hand back the records the sink saw, and the block.
template<typename T>
struct Run {
    std::vector<gr::DataSet<float>> records{};
    std::vector<gr::DataSet<float>> phase{};
    std::uint64_t                   nWindows{};
    std::uint64_t                   nRecords{};
    std::uint64_t                   nDegenerate{};
    std::uint64_t                   nPartialFlushes{};
    std::uint64_t                   nPartialWindows{};
    std::uint64_t                   nWindowResets{};
    std::uint64_t                   nRateRefused{};
    std::uint64_t                   nTailDropped{};
    std::uint64_t                   nSamples{};
};

template<typename T>
[[nodiscard]] Run<T> collect(gr::property_map settings, std::vector<T> samples, std::size_t burst = 65536UZ, bool withPhase = false, std::vector<std::pair<std::size_t, gr::property_map>> tags = {}) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<BurstSource<T>>();
    auto&                 block  = test.emplace<Autocorrelation<T>>(std::move(settings));
    auto&                 sink   = test.emplace<RecordSink>();
    source.samples               = std::move(samples);
    source.burst                 = burst;
    source.tags                  = std::move(tags);

    boost::ut::expect(test.connect(source, "out", block, "in").has_value());
    boost::ut::expect(test.connect(block, "out", sink, "in").has_value());
    RecordSink* phaseSink = nullptr;
    if (withPhase) {
        phaseSink = &test.template emplace<RecordSink>();
        boost::ut::expect(test.connect(block, "phase", *phaseSink, "in").has_value());
    }
    std::ignore = test.run();

    Run<T> result;
    result.records         = sink.records;
    result.phase           = phaseSink == nullptr ? std::vector<gr::DataSet<float>>{} : phaseSink->records;
    result.nWindows        = block.nWindows();
    result.nRecords        = block.nRecords();
    result.nDegenerate     = block.nDegenerate();
    result.nPartialFlushes = block.nPartialFlushes();
    result.nPartialWindows = block.nPartialWindows();
    result.nWindowResets   = block.nWindowResets();
    result.nRateRefused    = block.nRateRefused();
    result.nTailDropped    = block.nTailDropped();
    result.nSamples        = block.nSamples();
    return result;
}

/// @brief The largest value over lags `[first, last]`, and where it sits.
[[nodiscard]] std::pair<std::size_t, double> peakOver(const gr::DataSet<float>& record, std::size_t first, std::size_t last) {
    std::size_t at   = first;
    double      best = -1.;
    for (std::size_t lag = first; lag <= last && lag < record.signal_values.size(); ++lag) {
        if (static_cast<double>(record.signal_values[lag]) > best) {
            best = static_cast<double>(record.signal_values[lag]);
            at   = lag;
        }
    }
    return {at, best};
}

struct Rng {
    std::uint64_t state = 0x243f6a8885a308d3ULL;

    [[nodiscard]] std::uint64_t next() {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] double uniform() { return (static_cast<double>(next() >> 11U) + 0.5) / static_cast<double>(1ULL << 53U); }
    [[nodiscard]] CF     gaussian() {
        const double radius = std::sqrt(-std::log(uniform()));
        const double angle  = kTwoPi * uniform();
        return CF(static_cast<float>(radius * std::cos(angle)), static_cast<float>(radius * std::sin(angle)));
    }
};

[[nodiscard]] std::vector<CF> complexNoise(std::size_t n, std::uint64_t seed) {
    Rng             rng{seed};
    std::vector<CF> out(n);
    for (CF& sample : out) {
        sample = rng.gaussian();
    }
    return out;
}

// ---- scenes -------------------------------------------------------------------------------------------------

/// @brief An OFDM stream: random quadrature symbols on every bin, transformed to time, each symbol preceded by a
/// copy of its own last `nCp` samples. The prefix is what puts an exact repeat at the useful symbol's length.
[[nodiscard]] std::vector<CF> ofdmScene(std::size_t nFft, std::size_t nCp, std::size_t symbols, std::uint64_t seed, double offsetPerSample = 0.) {
    Rng                                                                                              rng{seed};
    gr::algorithm::FFT<std::complex<float>, std::complex<float>, gr::algorithm::Direction::Backward> inverse{};
    std::vector<CF>                                                                                  bins(nFft);
    std::vector<CF>                                                                                  time(nFft);
    std::vector<CF>                                                                                  out;
    out.reserve(symbols * (nFft + nCp));

    for (std::size_t symbol = 0UZ; symbol < symbols; ++symbol) {
        for (CF& bin : bins) {
            const std::size_t quadrant = static_cast<std::size_t>(rng.next() % 4ULL);
            const float       re       = (quadrant & 1ULL) != 0UZ ? 1.f : -1.f;
            const float       im       = (quadrant & 2ULL) != 0UZ ? 1.f : -1.f;
            bin                        = CF(re, im);
        }
        inverse.compute(bins, time);
        out.insert(out.end(), time.end() - static_cast<std::ptrdiff_t>(nCp), time.end());
        out.insert(out.end(), time.begin(), time.end());
    }
    if (offsetPerSample != 0.) {
        for (std::size_t k = 0UZ; k < out.size(); ++k) {
            const double phase = kTwoPi * offsetPerSample * static_cast<double>(k);
            out[k] *= CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
        }
    }
    return out;
}

/// @brief A raised-cosine transmit pulse, `spanSymbols` symbols long at `sps` samples a symbol.
[[nodiscard]] std::vector<float> raisedCosine(std::size_t sps, double beta, std::size_t spanSymbols) {
    const std::size_t  taps   = spanSymbols * sps + 1UZ;
    const auto         center = static_cast<double>(taps - 1UZ) / 2.;
    std::vector<float> h(taps);
    for (std::size_t k = 0UZ; k < taps; ++k) {
        const double t = (static_cast<double>(k) - center) / static_cast<double>(sps);
        double       value{};
        if (std::abs(t) < 1e-12) {
            value = 1.;
        } else if (beta > 0. && std::abs(std::abs(2. * beta * t) - 1.) < 1e-9) {
            const double u = 1. / (2. * beta);
            value          = (kPi / 4.) * std::sin(kPi * u) / (kPi * u);
        } else {
            const double sinc = std::sin(kPi * t) / (kPi * t);
            value             = sinc * std::cos(kPi * beta * t) / (1. - 4. * beta * beta * t * t);
        }
        h[k] = static_cast<float>(value);
    }
    return h;
}

/// @brief Antipodal symbols at `sps` samples a symbol, shaped by @p pulse; an empty pulse holds each symbol flat,
/// which is the rectangular case.
[[nodiscard]] std::vector<CF> linearModulation(std::size_t symbols, std::size_t sps, std::span<const float> pulse, std::uint64_t seed) {
    Rng                rng{seed};
    std::vector<float> antipodal(symbols);
    for (float& symbol : antipodal) {
        symbol = (rng.next() & 1ULL) != 0ULL ? 1.f : -1.f;
    }
    if (pulse.empty()) {
        std::vector<CF> out(symbols * sps);
        for (std::size_t k = 0UZ; k < symbols; ++k) {
            for (std::size_t s = 0UZ; s < sps; ++s) {
                out[k * sps + s] = CF(antipodal[k], 0.f);
            }
        }
        return out;
    }
    std::vector<CF> out(symbols * sps, CF{});
    for (std::size_t k = 0UZ; k < symbols; ++k) {
        const std::size_t base = k * sps;
        for (std::size_t t = 0UZ; t < pulse.size(); ++t) {
            const std::size_t at = base + t;
            if (at < out.size()) {
                out[at] += CF(antipodal[k] * pulse[t], 0.f);
            }
        }
    }
    return out;
}

/// @brief The C4FM scene: four-level frequency modulation at 4800 symbols a second and 20 samples a symbol, the
/// dibits shaped by the Nyquist raised cosine the standard specifies and scaled so the outer level deviates 1800 Hz
/// and the inner 600 Hz, then integrated into a phase.
///
/// The shaping is the transmit filter itself and not a continuous-phase modulator's frequency pulse: the two are
/// different functions that share a name, and only the Nyquist one — `sinc(t/T) cos(pi a t/T) / (1 - (2 a t/T)^2)` —
/// leaves a symbol-rate structure in the envelope once a receive filter converts frequency to amplitude.
[[nodiscard]] std::vector<CF> c4fmScene(std::size_t symbols, std::uint64_t seed) {
    constexpr std::size_t sps  = 20UZ;
    constexpr double      rate = 96000.;
    Rng                   rng{seed};

    const std::vector<float> pulse = raisedCosine(sps, 0.2, 8UZ);
    std::vector<double>      frequency(symbols * sps, 0.);
    for (std::size_t k = 0UZ; k < symbols; ++k) {
        const auto        rank      = static_cast<double>(rng.next() % 4ULL);
        const double      deviation = (2. * rank - 3.) * 600.; // the odd grid times 600 Hz: +/-1800 and +/-600
        const std::size_t base      = k * sps;
        for (std::size_t t = 0UZ; t < pulse.size(); ++t) {
            const std::size_t at = base + t;
            if (at < frequency.size()) {
                frequency[at] += deviation * static_cast<double>(pulse[t]);
            }
        }
    }
    std::vector<CF> out(frequency.size());
    double          phase = 0.;
    for (std::size_t k = 0UZ; k < frequency.size(); ++k) {
        phase += kTwoPi * frequency[k] / rate;
        out[k] = CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
    }
    return out;
}

/// @brief The valid part of a convolution, which is what a channel filter hands on.
[[nodiscard]] std::vector<CF> applyFir(std::span<const CF> in, std::span<const float> taps) {
    if (in.size() < taps.size()) {
        return {};
    }
    std::vector<CF> out(in.size() - taps.size() + 1UZ);
    for (std::size_t k = 0UZ; k < out.size(); ++k) {
        CF sum{};
        for (std::size_t t = 0UZ; t < taps.size(); ++t) {
            sum += in[k + t] * taps[taps.size() - 1UZ - t];
        }
        out[k] = sum;
    }
    return out;
}

/// The Mode S preamble in half-microsecond slots: pulses at 0.0, 1.0, 3.5 and 4.5 microseconds.
constexpr std::array<int, 16> kPreamble{1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0};

/// @brief One long Mode S frame at two slots a microsecond: sixteen preamble slots then two slots a bit, a pulse in
/// the first half for a one and in the second for a zero.
[[nodiscard]] std::vector<CF> modeSFrame(Rng& rng, bool withPreamble, bool withData) {
    std::vector<CF> wave;
    if (withPreamble) {
        for (const int pulse : kPreamble) {
            wave.emplace_back(pulse != 0 ? 1.f : 0.f, 0.f);
        }
    }
    if (withData) {
        for (std::size_t bit = 0UZ; bit < 112UZ; ++bit) {
            const bool one = (rng.next() & 1ULL) != 0ULL;
            wave.emplace_back(one ? 1.f : 0.f, 0.f);
            wave.emplace_back(one ? 0.f : 1.f, 0.f);
        }
    }
    return wave;
}

/// @brief The scatter and threshold a record states, recomputed from the keys it publishes.
struct HandComputed {
    double scatter{};
    double threshold{};
};

[[nodiscard]] HandComputed recomputed(const gr::DataSet<float>& record) {
    const auto pairs = static_cast<std::size_t>(metaNumber(record, "n_pairs"));
    const auto lags  = static_cast<std::size_t>(metaNumber(record, "max_lag"));
    const auto pFa   = metaNumber(record, "false_alarm_rate");
    const bool real  = metaNumber(record, "real_valued") > 0.5;
    return HandComputed{
        .scatter   = gr::analysis::acfScatter(pairs),
        .threshold = real ? gr::analysis::acfRealThreshold(pairs, lags, pFa) : gr::analysis::acfThreshold(pairs, lags, pFa),
    };
}

} // namespace qa_autocorrelation

using namespace boost::ut;
using namespace qa_autocorrelation;

const suite<"autocorrelation record and settings"> _acfRecord = [] {
    "criterion 12: the lag axis, the rate, and every metadata key"_test = [] {
        const std::vector<CF> input = complexNoise(40000UZ, 0x1111ULL);
        for (const float rate : {2.0e6f, 125000.f}) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(100)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(4)}, {"sample_rate", rate}}, input);
            expect(fatal(ge(run.records.size(), 1UZ))) << "the stream is long enough for at least one record";
            const gr::DataSet<float>& record = run.records[0UZ];

            expect(eq(record.axis_names.size(), 1UZ));
            expect(record.axis_names[0UZ] == std::string("Lag"));
            expect(record.axis_units[0UZ] == std::string("s"));
            expect(eq(record.extents.size(), 1UZ));
            expect(eq(record.extents[0UZ], 101));
            expect(eq(record.signal_values.size(), 101UZ));
            expect(record.signal_quantities[0UZ] == std::string("Autocorrelation"));
            expect(record.signal_names[0UZ] == std::string("acf"));
            for (std::size_t k = 0UZ; k <= 100UZ; ++k) {
                expect(record.axis_values[0UZ][k] == static_cast<float>(k) / rate) << std::format("lag {} at rate {}", k, rate);
            }
            expect(approx(metaNumber(record, "sample_rate"), static_cast<double>(rate), 1e-3));

            for (const std::string_view key : {"sample_rate", "sample_start", "sequence", "kind", "normalization", "window_length", "max_lag", "hop", "overlap", "window", "n_averaged", "n_samples", "n_pairs", "mean_removed", "real_valued", "signal_power", "scatter", "detection_threshold", "false_alarm_rate", "peak_threshold_db", "transform_length"}) {
                expect(hasKey(record, key)) << std::format("the record carries '{}'", key);
            }
            expect(eq(static_cast<std::size_t>(metaNumber(record, "window_length")), 4096UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "max_lag")), 100UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "hop")), 2048UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "transform_length")), 8192UZ)) << "the next power of two at or above N + L";
            expect(eq(static_cast<std::size_t>(metaNumber(record, "n_averaged")), 4UZ));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "n_samples")), 4096UZ + 3UZ * 2048UZ));
            expect(metaText(record, "kind") == std::string("complex"));
            expect(metaText(record, "normalization") == std::string("unbiased"));
            expect(metaText(record, "window") == std::string("Rectangular"));
            expect(eq(static_cast<std::size_t>(metaNumber(record, "sequence")), 0UZ));
            expect(record.signal_values[0UZ] == 1.f) << "lag zero anchors the axis at one";
        }
    };

    "criterion 14: refusals, each by name"_test = [] {
        const auto refused = [](gr::property_map settings) {
            Autocorrelation<CF> block(std::move(settings));
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        };
        expect(throws([&] { refused({{"max_lag", gr::Size_t(4096)}, {"window_length", gr::Size_t(4096)}}); })) << "a lag of a whole window";
        expect(throws([&] { refused({{"max_lag", gr::Size_t(5000)}, {"window_length", gr::Size_t(4096)}}); })) << "a lag beyond a window";
        expect(throws([&] { refused({{"max_lag", gr::Size_t(0)}}); }));
        expect(throws([&] { refused({{"window_length", gr::Size_t(8)}, {"max_lag", gr::Size_t(4)}}); }));
        expect(throws([&] { refused({{"n_averages", gr::Size_t(0)}}); }));
        expect(throws([&] { refused({{"overlap", 1.0}}); }));
        expect(throws([&] { refused({{"overlap", -0.1}}); }));
        expect(throws([&] { refused({{"kind", std::string("envelop")}}); }));
        expect(throws([&] { refused({{"normalization", std::string("none")}}); }));
        expect(throws([&] { refused({{"window", std::string("HannExp")}}); }));
        expect(throws([&] { refused({{"false_alarm_rate", 0.0}}); }));
        expect(throws([&] { refused({{"sample_rate", 0.f}}); }));

        // the refusal names both offending values, which is what makes it actionable
        try {
            refused({{"max_lag", gr::Size_t(4096)}, {"window_length", gr::Size_t(1024)}});
            expect(false) << "a lag beyond the window is refused";
        } catch (const std::exception& error) {
            const std::string what(error.what());
            expect(what.find("4096") != std::string::npos && what.find("1024") != std::string::npos) << std::format("the message names both values: {}", what);
        }
    };

    "criterion 14: the staged-restart keys refuse while the block runs and move once it is stopped"_test = [] {
        const auto running = [](gr::property_map settings) {
            auto block = std::make_unique<Autocorrelation<CF>>(std::move(settings));
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
        const gr::property_map base{{"window_length", gr::Size_t(1024)}, {"max_lag", gr::Size_t(64)}, {"sample_rate", 48000.f}};

        const std::array<gr::property_map, 7> kStaged{gr::property_map{{"window_length", gr::Size_t(2048)}}, gr::property_map{{"max_lag", gr::Size_t(128)}}, gr::property_map{{"kind", std::string("complex")}}, gr::property_map{{"normalization", std::string("biased")}}, gr::property_map{{"overlap", 0.25}}, gr::property_map{{"window", std::string("Hann")}}, gr::property_map{{"remove_mean", false}}};
        for (const gr::property_map& change : kStaged) {
            auto block = running(base);
            expect(throws([&] { live(*block, change); })) << std::format("a running block refuses '{}'", change.begin()->first.c_str());
        }
        const std::array<gr::property_map, 5> kLive{gr::property_map{{"n_averages", gr::Size_t(4)}}, gr::property_map{{"false_alarm_rate", 1e-2}}, gr::property_map{{"emit_phase", true}}, gr::property_map{{"signal_name", std::string("r")}}, gr::property_map{{"sample_rate", 96000.f}}};
        for (const gr::property_map& change : kLive) {
            auto block = running(base);
            expect(nothrow([&] { live(*block, change); })) << std::format("'{}' is live", change.begin()->first.c_str());
        }
        {
            auto block = running(base);
            block->stop();
            expect(nothrow([&] { live(*block, {{"window_length", gr::Size_t(2048)}}); })) << "stopped, the geometry moves: the contract is staged-restart, not immutable";
        }
        {
            auto block = running(base);
            expect(nothrow([&] { live(*block, {{"sample_rate", 0.f}}); })) << "a tagged rate that cannot scale an axis is counted, not thrown";
            expect(eq(block->nRateRefused(), std::uint64_t{1ULL}));
            expect(block->sample_rate.value == 48000.f) << "and the previous rate stands";
        }
    };

    "criterion 13: a sample_rate tag mid-stream"_test = [] {
        constexpr std::size_t length = 1024UZ;
        constexpr std::size_t tagAt  = 4600UZ;
        const std::vector<CF> input  = complexNoise(16384UZ, 0x2222ULL);

        // Both rates arrive as tags, the first on the stream's own first sample: the framework stops auto-updating a
        // parameter a graph author has set by name, so a block that follows a tagged rate only ever sees one when the
        // rate was not also written into its settings. The geometry settings are in samples, so neither tag re-plans
        // the transform.
        const gr::property_map first{{gr::tag::SAMPLE_RATE.shortKey(), 48000.f}};
        const gr::property_map second{{gr::tag::SAMPLE_RATE.shortKey(), 96000.f}};
        const gr::property_map geometry{{"window_length", gr::Size_t(length)}, {"max_lag", gr::Size_t(64)}, {"kind", std::string("complex")}, {"n_averages", gr::Size_t(8)}, {"overlap", 0.0}};
        const Run<CF>          run = collect<CF>(geometry, input, 512UZ, false, {{0UZ, first}, {tagAt, second}});

        expect(eq(run.nWindowResets, std::uint64_t{1ULL})) << "the window straddling the change is discarded and counted";
        // the change flushes one short record; the stream's own end flushes whatever the last group holds
        expect(ge(run.nPartialFlushes, std::uint64_t{1ULL})) << "the accumulation so far is flushed rather than dropped";
        expect(le(run.nPartialFlushes, std::uint64_t{2ULL})) << "and nothing else flushes short";
        expect(fatal(ge(run.records.size(), 2UZ)));
        const gr::DataSet<float>& flushed = run.records[0UZ];
        expect(eq(static_cast<std::size_t>(metaNumber(flushed, "n_averaged")), 4UZ)) << "four whole windows preceded the change";
        expect(approx(metaNumber(flushed, "sample_rate"), 48000., 1e-3)) << "carried on the old rate's axis";
        expect(approx(static_cast<double>(flushed.axis_values[0UZ][1UZ]), 1. / 48000., 1e-12));
        expect(eq(static_cast<std::size_t>(metaNumber(flushed, "n_samples")), length + 3UZ * length));
        const HandComputed hand = recomputed(flushed);
        expect(approx(metaNumber(flushed, "scatter"), hand.scatter, 1e-12)) << "the short record's scatter is computed from what it holds";
        expect(approx(metaNumber(flushed, "detection_threshold"), hand.threshold, 1e-12));

        const gr::DataSet<float>& next = run.records[1UZ];
        expect(approx(metaNumber(next, "sample_rate"), 96000., 1e-3)) << "the next record carries the new rate";
        expect(eq(static_cast<std::size_t>(metaNumber(next, "n_averaged")), 8UZ));
        std::println("criterion 13: flushed n_averaged={} at {} Hz, then n_averaged={} at {} Hz, window resets {}", metaNumber(flushed, "n_averaged"), metaNumber(flushed, "sample_rate"), metaNumber(next, "n_averaged"), metaNumber(next, "sample_rate"), run.nWindowResets);

        // a tag that cannot scale an axis is counted and the old rate stands
        const gr::property_map badTag{{gr::tag::SAMPLE_RATE.shortKey(), 0.f}};
        const Run<CF>          refusedRun = collect<CF>(geometry, input, 512UZ, false, {{0UZ, first}, {tagAt, badTag}});
        expect(eq(refusedRun.nRateRefused, std::uint64_t{1ULL}));
        expect(eq(refusedRun.nWindowResets, std::uint64_t{0ULL})) << "a refused rate disturbs no window";
        for (const gr::DataSet<float>& record : refusedRun.records) {
            expect(approx(metaNumber(record, "sample_rate"), 48000., 1e-3));
        }
    };

    "criterion 15: chunk independence, and the counter identity"_test = [] {
        constexpr std::size_t                sps     = 8UZ;
        constexpr std::size_t                symbols = 8000UZ;
        const std::vector<float>             pulse   = raisedCosine(sps, 0.35, 8UZ);
        const std::vector<CF>                scene   = linearModulation(symbols, sps, std::span<const float>(pulse), 0x3333ULL);
        constexpr std::array<std::size_t, 6> kBursts{1UZ, 7UZ, 4095UZ, 4096UZ, 4097UZ, 1UZ << 20U};

        std::vector<gr::DataSet<float>> reference;
        for (const std::size_t burst : kBursts) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(256)}, {"n_averages", gr::Size_t(8)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}}, scene, burst);
            expect(fatal(ge(run.records.size(), 1UZ))) << std::format("burst {}", burst);
            expect(eq(run.nWindows, 8ULL * run.nRecords + run.nPartialWindows + run.nDegenerate)) << std::format("burst {}: the window count closes", burst);
            expect(eq(run.nSamples, scene.size())) << std::format("burst {}: every sample was taken", burst);
            expect(lt(run.nTailDropped, 4096ULL)) << "the undecided remainder is shorter than a window";
            if (reference.empty()) {
                reference = run.records;
                continue;
            }
            expect(eq(run.records.size(), reference.size())) << std::format("burst {}: the same number of records", burst);
            for (std::size_t r = 0UZ; r < std::min(run.records.size(), reference.size()); ++r) {
                expect(run.records[r].signal_values == reference[r].signal_values) << std::format("burst {}: record {} is identical", burst, r);
                expect(eq(metaNumber(run.records[r], "sample_start"), metaNumber(reference[r], "sample_start")));
            }
        }
    };

    "criterion 16: the phase port"_test = [] {
        const std::vector<CF> input     = complexNoise(40000UZ, 0x4444ULL);
        const Run<CF>         withPhase = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(100)}, {"kind", std::string("complex")}, {"emit_phase", true}, {"n_averages", gr::Size_t(4)}, {"sample_rate", 48000.f}}, input, 65536UZ, true);
        expect(fatal(ge(withPhase.phase.size(), 1UZ))) << "the phase port is fed for the complex kind";
        expect(eq(withPhase.phase.size(), withPhase.records.size()));
        const gr::DataSet<float>& correlation = withPhase.records[0UZ];
        const gr::DataSet<float>& phase       = withPhase.phase[0UZ];
        expect(phase.extents == correlation.extents);
        expect(phase.axis_values == correlation.axis_values);
        expect(phase.signal_units[0UZ] == std::string("rad"));
        expect(phase.signal_quantities[0UZ] == std::string("Phase"));
        expect(phase.signal_names[0UZ] == std::string("acf_phase"));
        expect(phase.meta_information[0UZ] == correlation.meta_information[0UZ]) << "the same facts describe both records";
        for (const float value : phase.signal_values) {
            expect(std::abs(value) <= static_cast<float>(kPi) + 1e-5f);
        }

        const Run<CF> envelope = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(100)}, {"kind", std::string("envelope")}, {"emit_phase", true}, {"n_averages", gr::Size_t(4)}, {"sample_rate", 48000.f}}, input, 65536UZ, true);
        expect(envelope.phase.empty()) << "the envelope kind publishes no phase";
        expect(eq(envelope.records.size(), withPhase.records.size())) << "and emit_phase does not disturb the correlation port";
    };
};

const suite<"autocorrelation scenes"> _acfScenes = [] {
    "criterion 7: OFDM, both kinds, against the closed form"_test = [] {
        constexpr std::size_t nFft   = 2048UZ;
        constexpr std::size_t nCp    = 144UZ;
        constexpr double      closed = static_cast<double>(nCp) / static_cast<double>(nCp + nFft);
        const std::vector<CF> scene  = ofdmScene(nFft, nCp, 790UZ, 0x5555ULL);

        for (const std::string& kind : {std::string("complex"), std::string("envelope")}) {
            const Run<CF> run = collect<CF>({{"window_length", gr::Size_t(16384)}, {"max_lag", gr::Size_t(2200)}, {"kind", kind}, {"normalization", std::string("unbiased")}, {"n_averages", gr::Size_t(200)}, {"sample_rate", 30.72e6f}}, scene);
            expect(fatal(ge(run.records.size(), 1UZ))) << kind;
            const gr::DataSet<float>& record = run.records[0UZ];
            const auto [at, height]          = peakOver(record, 2000UZ, 2100UZ);
            const double scatter             = metaNumber(record, "scatter");
            std::println("criterion 7: OFDM {} kind, maximum over [2000, 2100] at lag {} reading {:.5f} against the closed form {:.5f} (scatter {:.5f})", kind, at, height, closed, scatter);
            expect(eq(at, nFft)) << std::format("{}: the maximum sits at the useful symbol's length", kind);
            expect(std::abs(height - closed) < 3. * scatter) << std::format("{}: {:.5f} against {:.5f} within three scatter units of {:.5f}", kind, height, closed, scatter);
            const HandComputed hand = recomputed(record);
            expect(approx(metaNumber(record, "detection_threshold"), hand.threshold, 1e-12)) << "criterion 21: the threshold is what the published keys say it is";
            expect(approx(metaNumber(record, "scatter"), hand.scatter, 1e-12));
            expect(height > metaNumber(record, "detection_threshold")) << "and the peak clears it";
            const double power = metaNumber(record, "signal_power");
            expect(power > 0.) << "criterion 21: the scale the ratio throws away is kept";
        }

        const Run<CF> biased                = collect<CF>({{"window_length", gr::Size_t(16384)}, {"max_lag", gr::Size_t(2200)}, {"kind", std::string("complex")}, {"normalization", std::string("biased")}, {"n_averages", gr::Size_t(200)}, {"sample_rate", 30.72e6f}}, scene);
        const auto [biasedAt, biasedHeight] = peakOver(biased.records[0UZ], 2000UZ, 2100UZ);
        const double biasedClosed           = closed * (1. - static_cast<double>(nFft) / 16384.);
        std::println("criterion 7: under biased the same peak sits at lag {} reading {:.5f} against {:.5f}", biasedAt, biasedHeight, biasedClosed);
        expect(eq(biasedAt, nFft));
        expect(std::abs(biasedHeight - biasedClosed) < 3. * metaNumber(biased.records[0UZ], "scatter"));

        // a carrier offset leaves the magnitude alone and turns the phase into a frequency estimate
        constexpr double      offset        = 0.3 / static_cast<double>(nFft);
        const std::vector<CF> offsetScene   = ofdmScene(nFft, nCp, 790UZ, 0x5555ULL, offset);
        const Run<CF>         withOffset    = collect<CF>({{"window_length", gr::Size_t(16384)}, {"max_lag", gr::Size_t(2200)}, {"kind", std::string("complex")}, {"emit_phase", true}, {"n_averages", gr::Size_t(200)}, {"sample_rate", 30.72e6f}}, offsetScene, 65536UZ, true);
        const auto [offsetAt, offsetHeight] = peakOver(withOffset.records[0UZ], 2000UZ, 2100UZ);
        expect(eq(offsetAt, nFft));
        expect(std::abs(offsetHeight - closed) < 3. * metaNumber(withOffset.records[0UZ], "scatter")) << "a carrier offset does not move the magnitude";
        expect(fatal(ge(withOffset.phase.size(), 1UZ)));
        const double measuredPhase = static_cast<double>(withOffset.phase[0UZ].signal_values[nFft]);
        const double expectedPhase = kTwoPi * offset * static_cast<double>(nFft);
        std::println("criterion 7: with a {:.4f}/T_u offset the phase at lag {} reads {:.5f} rad against {:.5f}", 0.3, nFft, measuredPhase, expectedPhase);
        expect(std::abs(measuredPhase - expectedPhase) < 0.01 * std::abs(expectedPhase)) << std::format("phase {:.5f} against {:.5f}", measuredPhase, expectedPhase);
    };

    "criterion 8: a rectangular-pulse envelope is nothing, and the instrument is positive on the same run"_test = [] {
        constexpr std::size_t  sps     = 8UZ;
        constexpr std::size_t  symbols = 20000UZ;
        const std::vector<CF>  flat    = linearModulation(symbols, sps, std::span<const float>{}, 0x6666ULL);
        const gr::property_map settings{{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(256)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(8)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}};

        const Run<CF> rectangular = collect<CF>(settings, flat);
        expect(rectangular.records.empty()) << "a constant-modulus envelope publishes nothing";
        expect(gt(rectangular.nDegenerate, std::uint64_t{0ULL}));
        expect(eq(rectangular.nDegenerate, rectangular.nWindows)) << "every window of it is degenerate";
        std::println("criterion 8: rectangular-pulse BPSK, {} windows, {} degenerate, {} records", rectangular.nWindows, rectangular.nDegenerate, rectangular.nRecords);

        const std::vector<float> pulse     = raisedCosine(sps, 0.35, 8UZ);
        const std::vector<CF>    shaped    = linearModulation(symbols, sps, std::span<const float>(pulse), 0x6666ULL);
        const Run<CF>            shapedRun = collect<CF>(settings, shaped);
        expect(fatal(ge(shapedRun.records.size(), 1UZ))) << "the same symbols through a band-limited pulse do have an envelope";
        const auto [at, height]      = peakOver(shapedRun.records[0UZ], 4UZ, 12UZ);
        const double atPeriod        = static_cast<double>(shapedRun.records[0UZ].signal_values[sps]);
        const double shapedThreshold = metaNumber(shapedRun.records[0UZ], "detection_threshold");
        std::println("criterion 8: the same symbols through a raised cosine put a maximum at lag {} reading {:.4f}, and lag {} itself reads {:.4f} against the record's threshold {:.4f}", at, height, sps, atPeriod, shapedThreshold);
        expect(le(at > sps ? at - sps : sps - at, 1UZ)) << "within one lag step of one symbol period";
        expect(atPeriod > shapedThreshold) << "and one symbol period clears the threshold";
    };

    "criterion 9: shaped BPSK puts the envelope peak at one symbol period"_test = [] {
        constexpr std::size_t           sps     = 8UZ;
        constexpr std::size_t           symbols = 40000UZ;
        constexpr std::array<double, 3> kBetas{0.2, 0.35, 0.5};
        for (const double beta : kBetas) {
            const std::vector<float> pulse = raisedCosine(sps, beta, 8UZ);
            const std::vector<CF>    scene = linearModulation(symbols, sps, std::span<const float>(pulse), 0x7777ULL);
            const Run<CF>            run   = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(256)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}}, scene);
            expect(fatal(ge(run.records.size(), 1UZ))) << std::format("beta {}", beta);
            const gr::DataSet<float>& record = run.records[0UZ];
            const auto [at, height]          = peakOver(record, 4UZ, 12UZ);
            const double       atPeriod      = static_cast<double>(record.signal_values[sps]);
            const HandComputed hand          = recomputed(record);
            // The envelope's feature at one symbol period is a negative lobe, so the published magnitude runs
            // through a null a few lags short of it and its own maximum sits one lag below the period; the height
            // the closed forms state is the value at the period itself.
            std::println("criterion 9: beta {:.2f} puts the maximum at lag {} reading {:.4f}; lag {} itself reads {:.4f} (bracket 0.31 to 0.38), threshold {:.4f}, scatter {:.5f}", beta, at, height, sps, atPeriod, hand.threshold, hand.scatter);
            expect(le(at > sps ? at - sps : sps - at, 1UZ)) << std::format("beta {}: within one lag step of {}", beta, sps);
            expect(atPeriod > 0.31 && atPeriod < 0.38) << std::format("beta {}: height {:.4f} outside the bracket", beta, atPeriod);
            expect(approx(metaNumber(record, "detection_threshold"), hand.threshold, 1e-12)) << "criterion 21";
            expect(approx(metaNumber(record, "scatter"), hand.scatter, 1e-12));
            expect(atPeriod > hand.threshold);
        }
        // beta = 0 keeps the feature: only the periodic mean vanishes there, and two other terms do not
        const std::vector<float> flatPulse = raisedCosine(sps, 0., 8UZ);
        const std::vector<CF>    flatScene = linearModulation(symbols, sps, std::span<const float>(flatPulse), 0x7777ULL);
        const Run<CF>            flatRun   = collect<CF>({{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(256)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"sample_rate", static_cast<float>(sps * 1000UZ)}}, flatScene);
        const auto [flatAt, flatHeight]    = peakOver(flatRun.records[0UZ], 4UZ, 12UZ);
        const double flatAtPeriod          = static_cast<double>(flatRun.records[0UZ].signal_values[sps]);
        std::println("criterion 9: beta 0.00 puts the maximum at lag {} reading {:.4f} and lag {} reads {:.4f}, which the criterion does not claim to be zero", flatAt, flatHeight, sps, flatAtPeriod);
        expect(le(flatAt > sps ? flatAt - sps : sps - flatAt, 1UZ));
        expect(flatAtPeriod > metaNumber(flatRun.records[0UZ], "detection_threshold"));
    };

    "criterion 10: C4FM, unfiltered and through the channel filter"_test = [] {
        constexpr std::size_t  sps   = 20UZ;
        constexpr float        rate  = 96000.f;
        constexpr std::size_t  lags  = 128UZ;
        const std::vector<CF>  scene = c4fmScene(40000UZ, 0x8888ULL);
        const gr::property_map settings{{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"sample_rate", rate}};

        const Run<CF> unfiltered = collect<CF>(settings, scene);
        expect(unfiltered.records.empty()) << "a constant-modulus waveform has no envelope autocorrelation at all";
        expect(eq(unfiltered.nDegenerate, unfiltered.nWindows));
        std::println("criterion 10: unfiltered C4FM, {} windows, all degenerate", unfiltered.nWindows);

        // The receive filter has to be one whose impulse response does not ring: a windowed-sinc channel filter
        // rings at fs/(2 fc), and that period, not the symbol period, is what its own envelope autocorrelation
        // reports. A Gaussian filter has no ringing, so what is left in the envelope is the signal's.
        constexpr std::array<std::pair<double, int>, 4> kBandwidths{{{6250., 21}, {5000., 25}, {4000., 31}, {3000., 41}}};
        for (const auto& [bandwidth, taps] : kBandwidths) {
            const std::vector<float> channel  = gr::filter::design::gaussianPulse(taps, static_cast<double>(sps), bandwidth / 4800.);
            const std::vector<CF>    filtered = applyFir(std::span<const CF>(scene), std::span<const float>(channel));
            const Run<CF>            run      = collect<CF>(settings, filtered);
            expect(fatal(ge(run.records.size(), 1UZ))) << std::format("{:.0f} Hz: a filter converts frequency modulation to amplitude modulation", bandwidth);
            const gr::DataSet<float>& record = run.records[0UZ];
            const auto [at, height]          = peakOver(record, sps - 6UZ, sps + 6UZ);
            const bool isLocal               = at > 0UZ && at + 1UZ < record.signal_values.size() && record.signal_values[at] > record.signal_values[at - 1UZ] && record.signal_values[at] > record.signal_values[at + 1UZ];
            std::println("criterion 10: Gaussian channel filter at {:.0f} Hz ({} taps): maximum over [{}, {}] at lag {} reading {:.4f}, a local maximum: {}; threshold {:.4f}", bandwidth, channel.size(), sps - 6UZ, sps + 6UZ, at, height, isLocal, metaNumber(record, "detection_threshold"));
            if (bandwidth == 6250.) {
                expect(le(at > sps ? at - sps : sps - at, 1UZ)) << std::format("within one lag step of {} samples, which is 208.333 us at {} S/s", sps, static_cast<std::size_t>(rate));
                expect(isLocal) << "and it is a local maximum rather than a point on a decay";
                expect(height > metaNumber(record, "detection_threshold")) << "clearing the threshold";
            }
        }
    };

    "criterion 11: Mode S, exactly"_test = [] {
        // the preamble alone: a deterministic sequence, so there is no tolerance to argue about
        Rng                   rng{0x9999ULL};
        const std::vector<CF> preamble = modeSFrame(rng, true, false);
        expect(eq(preamble.size(), 16UZ));

        gr::analysis::AcfConfig preambleConfig{};
        preambleConfig.windowLength = 16UZ;
        preambleConfig.maxLag       = 8UZ;
        preambleConfig.kind         = gr::analysis::AcfKind::Envelope;
        preambleConfig.overlap      = 0.;
        preambleConfig.nAverages    = 1UZ;
        gr::analysis::Autocorrelation<CF> kernel;
        kernel.prepare(preambleConfig);
        std::vector<float> preambleMagnitude;
        std::ignore = kernel.process(std::span<const CF>(preamble), [&preambleMagnitude](const gr::analysis::AcfResult& result) { preambleMagnitude.assign(result.magnitude.begin(), result.magnitude.end()); });
        expect(eq(preambleMagnitude.size(), 9UZ));
        std::println("criterion 11: the preamble alone reads |R(1.0 us)|/R(0) = {:.6f} against 3/7 = {:.6f}", preambleMagnitude[2UZ], 3. / 7.);
        expect(std::abs(static_cast<double>(preambleMagnitude[2UZ]) - 3. / 7.) < 1e-5) << std::format("{:.7f} against {:.7f}", preambleMagnitude[2UZ], 3. / 7.);

        // the data field alone, averaged over 200 seeded frames
        constexpr std::size_t frames = 200UZ;
        std::vector<CF>       dataFields;
        for (std::size_t frame = 0UZ; frame < frames; ++frame) {
            const std::vector<CF> field = modeSFrame(rng, false, true);
            dataFields.insert(dataFields.end(), field.begin(), field.end());
        }
        expect(eq(dataFields.size(), frames * 224UZ));
        const Run<CF> data = collect<CF>({{"window_length", gr::Size_t(224)}, {"max_lag", gr::Size_t(12)}, {"kind", std::string("envelope")}, {"overlap", 0.0}, {"n_averages", gr::Size_t(frames)}, {"sample_rate", 2.0e6f}}, dataFields);
        expect(fatal(ge(data.records.size(), 1UZ)));
        const gr::DataSet<float>& dataRecord = data.records[0UZ];
        const double              scatter    = metaNumber(dataRecord, "scatter");
        const double              atHalf     = static_cast<double>(dataRecord.signal_values[1UZ]);
        const double              atOne      = static_cast<double>(dataRecord.signal_values[2UZ]);
        std::println("criterion 11: the data field alone reads |R(0.5 us)|/R(0) = {:.5f} against 0.5 and |R(1.0 us)|/R(0) = {:.5f} against 0, scatter {:.5f}", atHalf, atOne, scatter);
        const HandComputed hand = recomputed(dataRecord);
        expect(std::abs(atHalf - 0.5) < 3. * scatter) << std::format("{:.5f} against 0.5 within three scatter units of {:.5f}", atHalf, scatter);
        // The null is read against the record's own detection threshold rather than against a fixed multiple of the
        // scatter: a magnitude drawn from a zero-mean real estimate is folded, so three scatter units is a bound one
        // realization in three hundred crosses on nothing but luck, while the threshold is the level the record
        // itself says a peak must reach.
        expect(atOne < hand.threshold) << std::format("the derived null reads {:.5f} ({:.2f} scatter units) against the record's threshold {:.5f}", atOne, atOne / scatter, hand.threshold);
        expect(approx(metaNumber(dataRecord, "detection_threshold"), hand.threshold, 1e-12)) << "criterion 21";
        expect(atHalf > hand.threshold) << "the instrument is positive on the same run that carries the null";

        // the whole 240-sample frame blends the two in the ratio 224:16
        std::vector<CF> whole;
        for (std::size_t frame = 0UZ; frame < frames; ++frame) {
            const std::vector<CF> field = modeSFrame(rng, true, true);
            whole.insert(whole.end(), field.begin(), field.end());
        }
        const Run<CF> wholeRun = collect<CF>({{"window_length", gr::Size_t(240)}, {"max_lag", gr::Size_t(16)}, {"kind", std::string("envelope")}, {"overlap", 0.0}, {"n_averages", gr::Size_t(frames)}, {"sample_rate", 2.0e6f}}, whole);
        expect(fatal(ge(wholeRun.records.size(), 1UZ)));
        std::println("criterion 11: the whole 240-sample frame reads |R(0.5 us)|/R(0) = {:.5f} and |R(1.0 us)|/R(0) = {:.5f}, scatter {:.5f}", wholeRun.records[0UZ].signal_values[1UZ], wholeRun.records[0UZ].signal_values[2UZ], metaNumber(wholeRun.records[0UZ], "scatter"));
    };

    "criterion 22: PeakDetect over the record"_test = [] {
        constexpr std::size_t    sps   = 8UZ;
        constexpr float          rate  = static_cast<float>(sps * 1000UZ);
        constexpr std::size_t    lags  = 256UZ;
        constexpr double         pFa   = 1e-3;
        const std::vector<float> pulse = raisedCosine(sps, 0.35, 8UZ);
        const std::vector<CF>    scene = linearModulation(40000UZ, sps, std::span<const float>(pulse), 0xAAAAULL);

        const gr::property_map shapedSettings{{"window_length", gr::Size_t(4096)}, {"max_lag", gr::Size_t(lags)}, {"kind", std::string("envelope")}, {"n_averages", gr::Size_t(32)}, {"false_alarm_rate", pFa}, {"sample_rate", rate}};

        /// @brief The chain of the reader: the estimate, then a peak detector reading the margin the record states.
        const auto detect = [](gr::property_map settings, std::vector<CF> samples, double thresholdDb) {
            gr::test::RuntimeTest test;
            auto&                 source   = test.emplace<BurstSource<CF>>();
            auto&                 acf      = test.emplace<Autocorrelation<CF>>(std::move(settings));
            auto&                 detector = test.emplace<PeakDetect>({{"threshold_db", thresholdDb}, {"reference", std::string("above_median")}, {"min_distance_hz", 0.0}});
            auto&                 sink     = test.emplace<RecordSink>();
            source.samples                 = std::move(samples);
            expect(test.connect(source, "out", acf, "in").has_value());
            expect(test.connect(acf, "out", detector, "in").has_value());
            expect(test.connect(detector, "out", sink, "in").has_value());
            std::ignore = test.run();
            return sink.records;
        };

        /// @brief How far the nearest detection of @p records sits from @p wanted, in seconds, and how many there were.
        const auto nearest = [](const std::vector<gr::DataSet<float>>& records, double wanted) {
            std::pair<double, std::size_t> result{1., 0UZ};
            if (records.empty()) {
                return result;
            }
            result.second = static_cast<std::size_t>(records[0UZ].extents[0UZ]);
            for (std::size_t k = 0UZ; k < result.second; ++k) {
                result.first = std::min(result.first, std::abs(static_cast<double>(records[0UZ].signal_values[k]) - wanted));
            }
            return result;
        };

        /// @brief The decibel margin a record's own largest value stands at over its own median, which is the
        /// quantity a detector reading `above_median` compares against its threshold.
        const auto marginDb = [](const gr::DataSet<float>& record) {
            std::vector<float> sorted(record.signal_values);
            if (sorted.size() < 3UZ) {
                return 0.;
            }
            double top = 0.; // lag zero is one by construction and is not a peak the detector can report
            for (std::size_t k = 1UZ; k < sorted.size(); ++k) {
                top = std::max(top, static_cast<double>(sorted[k]));
            }
            const std::size_t middle = sorted.size() / 2UZ;
            std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(middle), sorted.end());
            const double median = static_cast<double>(sorted[middle]);
            return median > 0. ? 10. * std::log10(top / median) : 0.;
        };

        // A period the sample grid does not land on is what the sub-bin refinement is for: a sinusoidal envelope
        // modulation at 8.4 samples has its correlation maximum at 8.4, between two published lags.
        //
        // Its threshold is a stated margin and not the record's own derived figure, and the reason is a property of
        // the reference rather than of the estimate: a sinusoidal modulation correlates at every lag, so the record
        // carries the same cosine from lag 1 to lag L and its median is the median of that cosine, which stands
        // 10 log10(1/cos(pi/4)) = 1.51 dB under the peak. A margin derived for a record whose median is noise cannot
        // be met by a record that has no noise-only lag in it, and the arm's subject is the interpolation, not the
        // threshold - the realized rate the threshold does claim is the third arm's.
        {
            constexpr double period    = 8.4;
            constexpr double statedDb  = 1.0;
            std::vector<CF>  modulated = complexNoise(4096UZ * 40UZ, 0xCCCCULL);
            for (std::size_t k = 0UZ; k < modulated.size(); ++k) {
                modulated[k] *= static_cast<float>(std::sqrt(1. + 0.9 * std::cos(kTwoPi * static_cast<double>(k) / period)));
            }
            const Run<CF> plain = collect<CF>(shapedSettings, modulated);
            expect(fatal(ge(plain.records.size(), 1UZ)));

            const std::vector<gr::DataSet<float>> detections = detect(shapedSettings, modulated, statedDb);
            const auto [closest, count]                      = nearest(detections, period / static_cast<double>(rate));
            std::println("criterion 22: a {:.1f}-sample sinusoidal envelope modulation stands {:.3f} dB over its record's own median against the derived {:.3f} dB; at a stated {:.1f} dB it gives {} detections and the nearest sits {:.4f} lag steps from the period after parabolic interpolation", period, marginDb(plain.records[0UZ]), metaNumber(plain.records[0UZ], "peak_threshold_db"), statedDb, count, closest * static_cast<double>(rate));
            expect(ge(detections.size(), 1UZ)) << "the injected period is reported as a detection";
            expect(closest <= 0.5 / static_cast<double>(rate)) << "within half a lag step, which is what makes a period between two lags resolvable";
        }

        // the symbol period of a shaped stream, whose envelope feature is a negative lobe and so sits a fifth of a
        // lag below the period itself
        {
            const Run<CF> plain = collect<CF>(shapedSettings, scene);
            expect(fatal(ge(plain.records.size(), 1UZ)));
            const double                          thresholdDb = metaNumber(plain.records[0UZ], "peak_threshold_db");
            const std::vector<gr::DataSet<float>> detections  = detect(shapedSettings, scene, thresholdDb);
            const auto [closest, count]                       = nearest(detections, static_cast<double>(sps) / static_cast<double>(rate));
            std::println("criterion 22: over shaped BPSK the record stands {:.3f} dB over its own median against the derived {:.3f} dB; {} detections, the closest to the symbol period is {:.3f} lag steps away", marginDb(plain.records[0UZ]), thresholdDb, count, closest * static_cast<double>(rate));
            expect(ge(detections.size(), 1UZ)) << "the symbol period is reported as a detection";
            expect(closest <= 1.5 / static_cast<double>(rate)) << "the symbol period is reported within the lobe the envelope actually puts there";
        }

        // the realized rate on noise alone, which is the whole of what a threshold in decibels over the median claims
        {
            constexpr std::size_t noiseRecords = 2000UZ;
            for (const std::string& kind : {std::string("complex"), std::string("envelope")}) {
                const gr::property_map settings{{"window_length", gr::Size_t(1024)}, {"max_lag", gr::Size_t(lags)}, {"kind", kind}, {"n_averages", gr::Size_t(1)}, {"overlap", 0.0}, {"false_alarm_rate", pFa}, {"sample_rate", 48000.f}};
                const Run<CF>          probe = collect<CF>(settings, complexNoise(1024UZ * 8UZ + 16UZ, 0xDDDDULL));
                expect(fatal(ge(probe.records.size(), 1UZ)));
                const double thresholdDb = metaNumber(probe.records[0UZ], "peak_threshold_db");

                const std::vector<gr::DataSet<float>> detections = detect(settings, complexNoise(noiseRecords * 1024UZ + 16UZ, 0xBBBBULL), thresholdDb);
                const double                          measured   = static_cast<double>(detections.size()) / static_cast<double>(noiseRecords);
                const double                          spread     = 3. * std::sqrt(pFa * (1. - pFa) / static_cast<double>(noiseRecords));
                std::println("criterion 22: {} kind at {:.3f} dB over the median, {} of {} noise records produced a detection: rate {:.5f} against the design {:.5f} (+/- {:.5f})", kind, thresholdDb, detections.size(), noiseRecords, measured, pFa, spread);
                expect(measured <= pFa + spread) << std::format("{}: realized rate {:.5f} against design {:.5f}", kind, measured, pFa);
            }
        }
    };
};

int main() { /* not needed for UT */ }
