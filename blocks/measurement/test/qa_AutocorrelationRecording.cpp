/* The recorded-air legs for the autocorrelation: four captures, each read for the period it is known to carry.
 *
 * They stand beside the synthetic criteria and never instead of them, and they live in their own executable because
 * a skip is the whole binary's exit status: folding them into the synthetic gate would let an absent recording skip
 * that gate too. Three of the four captures have no metadata file, so those are read as the raw interleaved pairs
 * their names describe; the LTE downlink has a metadata sidecar and is read through the SigMF source.
 *
 * The directory arrives as GR4_RECORDINGS_DIR. With the directory or a file absent the run prints the path it
 * looked for and exits 77, which CTest is told is a skip.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/algorithm/analysis/Autocorrelation.hpp>
#include <gnuradio-4.0/algorithm/digital/Ppm.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/analog/QuadratureDemod.hpp>
#include <gnuradio-4.0/fileio/SigMfIo.hpp>

namespace qa_acf_recording {

using CF = std::complex<float>;
using gr::analysis::AcfConfig;
using gr::analysis::AcfKind;
using gr::analysis::AcfNormalization;
using gr::analysis::AcfResult;

constexpr double kTwoPi = 2. * std::numbers::pi;

constexpr std::string_view kFmFirst  = "20260726_102344_99400000_2048000_fc.raw";
constexpr std::string_view kFmSecond = "20260726_102407_100600000_2048000_fc.raw";
constexpr std::string_view kLte      = "20260730_182327_757000000_25000000_fc.sigmf-meta";
constexpr std::string_view kAdsb     = "ADSB_20260812_182544_1090000000_2000000_cs16.sigmf-data";
constexpr std::string_view kP25      = "wpd_20260811_152707_483125000_125000_cf32.sigmf-data";

[[nodiscard]] std::string recordingsDirectory() {
    const char* fromEnvironment = std::getenv("GR4_RECORDINGS_DIR");
    return fromEnvironment == nullptr ? std::string{} : std::string(fromEnvironment);
}

[[nodiscard]] std::filesystem::path capturePath(std::string_view name) { return std::filesystem::path(recordingsDirectory()) / name; }

/// @brief Interleaved IEEE float pairs, which is what the `_fc.raw` and `_cf32` captures hold.
[[nodiscard]] std::vector<CF> readCf32(const std::filesystem::path& path, std::size_t items, std::size_t skipItems = 0UZ) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return {};
    }
    file.seekg(static_cast<std::streamoff>(skipItems * 2UZ * sizeof(float)));
    std::vector<CF>    out;
    std::vector<float> raw(2UZ * 65536UZ);
    out.reserve(items);
    while (out.size() < items && file) {
        const std::size_t want = std::min(raw.size() / 2UZ, items - out.size());
        file.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(want * 2UZ * sizeof(float)));
        const auto pairs = static_cast<std::size_t>(file.gcount()) / (2UZ * sizeof(float));
        if (pairs == 0UZ) {
            break;
        }
        for (std::size_t k = 0UZ; k < pairs; ++k) {
            out.emplace_back(raw[2UZ * k], raw[2UZ * k + 1UZ]);
        }
    }
    return out;
}

/// @brief Interleaved little-endian int16 pairs, read as magnitudes: the envelope kind squares them anyway, and a
/// magnitude stream is half the memory of the complex one over a thirty-second capture.
[[nodiscard]] std::size_t forEachCs16Magnitude(const std::filesystem::path& path, std::size_t items, auto&& onBatch) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return 0UZ;
    }
    std::vector<std::int16_t> raw(2UZ * 65536UZ);
    std::vector<float>        batch;
    std::size_t               total = 0UZ;
    while (total < items && file) {
        const std::size_t want = std::min(raw.size() / 2UZ, items - total);
        file.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(want * 2UZ * sizeof(std::int16_t)));
        const auto pairs = static_cast<std::size_t>(file.gcount()) / (2UZ * sizeof(std::int16_t));
        if (pairs == 0UZ) {
            break;
        }
        batch.clear();
        batch.reserve(pairs);
        for (std::size_t k = 0UZ; k < pairs; ++k) {
            const auto real = static_cast<float>(raw[2UZ * k]);
            const auto imag = static_cast<float>(raw[2UZ * k + 1UZ]);
            batch.push_back(std::sqrt(real * real + imag * imag));
        }
        onBatch(std::span<const float>(batch), total);
        total += pairs;
    }
    return total;
}

/// @brief Multiply by a complex exponential, which puts a station at @p offsetHz at zero frequency.
[[nodiscard]] std::vector<CF> tune(std::span<const CF> in, double offsetHz, double sampleRate) {
    std::vector<CF> out(in.size());
    const double    step = -kTwoPi * offsetHz / sampleRate;
    for (std::size_t k = 0UZ; k < in.size(); ++k) {
        const double phase = step * static_cast<double>(k);
        out[k]             = in[k] * CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
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

[[nodiscard]] double meanPower(std::span<const CF> in) {
    double total = 0.;
    for (const CF& sample : in) {
        total += static_cast<double>(sample.real()) * static_cast<double>(sample.real()) + static_cast<double>(sample.imag()) * static_cast<double>(sample.imag());
    }
    return in.empty() ? 0. : total / static_cast<double>(in.size());
}

/// One averaged estimate, which is what every leg below reports.
struct Estimate {
    std::vector<float> magnitude{};
    double             scatter{};
    double             threshold{};
    double             power{};
    std::size_t        windows{};
    std::size_t        pairs{};
};

/// @brief Run one configuration over one span and take the first record, or the flushed partial one.
template<typename T>
[[nodiscard]] Estimate estimate(const AcfConfig& config, std::span<const T> in) {
    gr::analysis::Autocorrelation<T> kernel;
    kernel.prepare(config);
    Estimate   result;
    const auto sink = [&result](const AcfResult& record) {
        if (!result.magnitude.empty()) {
            return;
        }
        result.magnitude.assign(record.magnitude.begin(), record.magnitude.end());
        result.scatter   = record.scatter;
        result.threshold = record.threshold;
        result.power     = record.power;
        result.windows   = record.nAveraged;
        result.pairs     = record.nPairs;
    };
    std::size_t at = 0UZ;
    while (at < in.size()) {
        const std::size_t used = kernel.process(in.subspan(at), sink);
        if (used == 0UZ) {
            break;
        }
        at += used;
    }
    std::ignore = kernel.flush(sink);
    return result;
}

/// @brief Whether @p lag is a strict local maximum of the estimate.
[[nodiscard]] bool isLocalMaximum(std::span<const float> magnitude, std::size_t lag) { return lag > 0UZ && lag + 1UZ < magnitude.size() && magnitude[lag] > magnitude[lag - 1UZ] && magnitude[lag] > magnitude[lag + 1UZ]; }

/// @brief The largest value over `[first, last]` and where it sits.
[[nodiscard]] std::pair<std::size_t, double> peakOver(std::span<const float> magnitude, std::size_t first, std::size_t last) {
    std::size_t at   = first;
    double      best = -1.;
    for (std::size_t lag = first; lag <= last && lag < magnitude.size(); ++lag) {
        if (static_cast<double>(magnitude[lag]) > best) {
            best = static_cast<double>(magnitude[lag]);
            at   = lag;
        }
    }
    return {at, best};
}

/// @brief Whether a local maximum sits within one lag step of @p lag, and where.
[[nodiscard]] std::pair<bool, std::size_t> maximumNear(std::span<const float> magnitude, std::size_t lag) {
    for (const std::size_t candidate : {lag, lag - 1UZ, lag + 1UZ}) {
        if (isLocalMaximum(magnitude, candidate)) {
            return {true, candidate};
        }
    }
    return {false, lag};
}

/// Reads a stated number of items out of a source into memory, then ends the stream.
template<typename T>
struct SampleSink : gr::Block<SampleSink<T>> {
    gr::PortIn<T> in;

    std::vector<T> samples{};

    GR_MAKE_REFLECTABLE(SampleSink, in);

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        samples.insert(samples.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief Read the head of a SigMF recording through the source block that owns its metadata.
[[nodiscard]] std::vector<CF> readThroughSigMf(const std::filesystem::path& path, std::size_t items) {
    gr::test::RuntimeTest test;
    auto&                 source = test.emplace<gr::blocks::fileio::SigMfSource<CF>>({{"file_name", path.string()}, {"length", static_cast<gr::Size_t>(items)}, {"emit_annotations", false}});
    auto&                 sink   = test.emplace<SampleSink<CF>>();
    if (!test.connect(source, "out", sink, "in").has_value()) {
        return {};
    }
    std::ignore = test.run();
    return sink.samples;
}

} // namespace qa_acf_recording

using namespace boost::ut;
using namespace qa_acf_recording;

const suite<"autocorrelation recording legs"> _acfRecording = [] {
    "every capture the legs read is present"_test = [] {
        for (const std::string_view name : {kFmFirst, kFmSecond, kLte, kAdsb, kP25}) {
            const std::filesystem::path path = capturePath(name);
            if (recordingsDirectory().empty() || !std::filesystem::exists(path)) {
                std::println("SKIP: no capture at '{}'; configure with -DGR4_RECORDINGS_DIR=<dir> holding {}", path.string(), name);
                std::fflush(nullptr);
                std::_Exit(77);
            }
        }
    };

    "criterion 17: the FM leg, the 19 kHz pilot at 52.632 us"_test = [] {
        constexpr double      rate     = 2.048e6;
        constexpr std::size_t length   = 16384UZ;
        constexpr std::size_t lags     = 300UZ;
        constexpr std::size_t windows  = 300UZ;
        constexpr double      period   = 1. / 19000.;
        const double          expected = period * rate; // 107.79 samples
        const auto            near     = static_cast<std::size_t>(std::llround(expected));

        const std::vector<float> narrow = gr::filter::design::designLowpass({.sampleRate = rate, .cutoff = 100000., .transitionWidth = 40000., .attenuationDb = 60.});
        const std::vector<float> wide   = gr::filter::design::designLowpass({.sampleRate = rate, .cutoff = 200000., .transitionWidth = 40000., .attenuationDb = 60.});

        for (const std::string_view name : {kFmFirst, kFmSecond}) {
            const std::vector<CF> raw = readCf32(capturePath(name), windows * length + 1024UZ); // room for the channel filter's own group delay
            expect(fatal(ge(raw.size(), windows * length))) << std::format("{}: enough samples", name);

            // Which station the leg reads is the capture's own answer, not an assumption: broadcast channels sit at
            // odd tenths of a megahertz, so every carrier in a 2.048 MHz span around a whole tenth is an odd
            // multiple of 100 kHz away from it, and the strongest of those after the channel filter is the one whose
            // pilot is worth looking for.
            double                    bestOffset = 100000.;
            double                    bestPower  = -1.;
            const std::span<const CF> probeSpan(raw.data(), std::min<std::size_t>(raw.size(), 262144UZ));
            for (int step = -5; step <= 4; ++step) {
                const double          offset = static_cast<double>(step) * 200000. + 100000.;
                const std::vector<CF> probe  = applyFir(std::span<const CF>(tune(probeSpan, offset, rate)), std::span<const float>(narrow));
                const double          power  = meanPower(std::span<const CF>(probe));
                if (power > bestPower) {
                    bestPower  = power;
                    bestOffset = offset;
                }
            }

            const std::vector<CF> tuned   = tune(std::span<const CF>(raw), bestOffset, rate);
            const std::vector<CF> channel = applyFir(std::span<const CF>(tuned), std::span<const float>(narrow));
            const std::vector<CF> widened = applyFir(std::span<const CF>(tuned), std::span<const float>(wide));

            AcfConfig config{};
            config.windowLength = length;
            config.maxLag       = lags;
            config.overlap      = 0.;
            config.nAverages    = windows;

            config.kind                = AcfKind::Complex;
            const Estimate complexArm  = estimate<CF>(config, std::span<const CF>(channel));
            config.kind                = AcfKind::Envelope;
            const Estimate envelopeArm = estimate<CF>(config, std::span<const CF>(channel));
            const Estimate wideArm     = estimate<CF>(config, std::span<const CF>(widened));
            const Estimate untunedArm  = estimate<CF>(config, std::span<const CF>(raw));

            // the discriminator's own output, which is where the pilot is simply a tone
            gr::blocks::analog::QuadratureDemod<float> demod;
            std::vector<float>                         discriminated(channel.size());
            std::ignore                     = demod.processBulk(std::span<const CF>(channel), std::span<float>(discriminated));
            config.kind                     = AcfKind::Complex;
            const Estimate discriminatorArm = estimate<float>(config, std::span<const float>(discriminated));

            std::println("criterion 17: {} tuned {:+.0f} kHz (mean power {:.5f} after the channel filter), N={} K={} L={}, expected lag {:.2f}, scatter {:.6f}", name, bestOffset / 1000., bestPower, length, complexArm.windows, lags, expected, complexArm.scatter);
            for (const auto& [label, arm] : {std::pair{"complex ACF, +/-100 kHz    ", &complexArm}, std::pair{"envelope ACF, +/-100 kHz   ", &envelopeArm}, std::pair{"envelope ACF, +/-200 kHz   ", &wideArm}, std::pair{"envelope ACF, untuned span ", &untunedArm}, std::pair{"ACF of the discriminator   ", &discriminatorArm}}) {
                const auto [isMax, at] = maximumNear(std::span<const float>(arm->magnitude), near);
                std::println("    {} lag 107 {:.4f}  lag 108 {:.4f}  local maximum near {}: {}  threshold {:.4f}", label, arm->magnitude[near], arm->magnitude[near + 1UZ], near, isMax ? std::format("yes at {}", at) : std::string("no"), arm->threshold);
            }

            const auto [envelopeIsMax, envelopeAt] = maximumNear(std::span<const float>(envelopeArm.magnitude), near);
            expect(envelopeIsMax) << std::format("{}: the envelope kind after a channel filter narrower than the signal puts a local maximum within one lag step of {:.2f}", name, expected);
            expect(static_cast<double>(envelopeArm.magnitude[envelopeAt]) > envelopeArm.threshold) << std::format("{}: and it clears the threshold", name);

            const auto [demodIsMax, demodAt] = maximumNear(std::span<const float>(discriminatorArm.magnitude), near);
            expect(demodIsMax) << std::format("{}: the discriminator's own autocorrelation puts a local maximum there", name);
            expect(static_cast<double>(discriminatorArm.magnitude[demodAt]) > discriminatorArm.threshold) << std::format("{}: overwhelmingly", name);
            const auto [argmaxAt, argmaxValue] = peakOver(std::span<const float>(complexArm.magnitude), 20UZ, lags);
            std::println("    the complex IQ arm's argmax over lags [20, {}] sits at {} reading {:.4f}, which is the program's own decorrelation and not the pilot", lags, argmaxAt, argmaxValue);
        }
    };

    "criterion 18: the LTE leg, the useful symbol at 66.667 us"_test = [] {
        constexpr double      rate   = 25.0e6;
        constexpr std::size_t length = 32768UZ;
        constexpr std::size_t lags   = 2000UZ;
        constexpr std::size_t full   = 300UZ;
        constexpr std::size_t tunedK = 50UZ;
        constexpr double      closed = 1024. / 15360.;

        const std::vector<CF> samples = readThroughSigMf(capturePath(kLte), full * length);
        expect(fatal(ge(samples.size(), full * length))) << "the SigMF source delivered the whole span the leg asks for";

        AcfConfig config{};
        config.windowLength     = length;
        config.maxLag           = lags;
        config.kind             = AcfKind::Complex;
        config.overlap          = 0.;
        config.nAverages        = full;
        const Estimate fullSpan = estimate<CF>(config, std::span<const CF>(samples));
        std::println("criterion 18: the whole 25 MHz span, N={} K={}, scatter {:.6f}: |R(1666)|/R(0) = {:.4f}, |R(1667)|/R(0) = {:.4f}", length, fullSpan.windows, fullSpan.scatter, fullSpan.magnitude[1666UZ], fullSpan.magnitude[1667UZ]);

        const std::vector<float>  channel = gr::filter::design::designLowpass({.sampleRate = rate, .cutoff = 4.5e6, .transitionWidth = 1.0e6, .attenuationDb = 60.});
        const std::size_t         need    = tunedK * length + channel.size();
        const std::span<const CF> head(samples.data(), std::min(samples.size(), need));

        config.nAverages = tunedK;
        std::println("criterion 18: the offset sweep, N={} K={}, +/-4.5 MHz channel filter ({} taps)", length, tunedK, channel.size());
        double bestHeight = -1.;
        double bestOffset = 0.;
        for (int step = -10; step <= 10; ++step) {
            const double          offset   = static_cast<double>(step) * 1.0e6;
            const std::vector<CF> filtered = applyFir(std::span<const CF>(tune(head, offset, rate)), std::span<const float>(channel));
            const Estimate        arm      = estimate<CF>(config, std::span<const CF>(filtered));
            const auto [at, height]        = peakOver(std::span<const float>(arm.magnitude), 1600UZ, 1740UZ);
            std::println("    offset {:+6.1f} MHz: maximum over [1600, 1740] at lag {} reading {:.4f} (scatter {:.6f})", offset / 1e6, at, height, arm.scatter);
            if (offset == 4.0e6) {
                expect(at == 1666UZ || at == 1667UZ) << std::format("the tuned arm's maximum sits at 1666 or 1667, got {}", at);
                expect(std::abs(height - closed) < 0.15 * closed) << std::format("the tuned arm reads {:.4f} against the closed form {:.4f} within 15%", height, closed);
                expect(height > arm.threshold) << "and it clears the record's own threshold";
            }
            if (height > bestHeight) {
                bestHeight = height;
                bestOffset = offset;
            }
        }
        std::println("criterion 18: the strongest arm of the sweep sits at {:+.1f} MHz reading {:.4f} against the closed form 1024/15360 = {:.4f}", bestOffset / 1e6, bestHeight, closed);
    };

    "criterion 19: the ADS-B leg, 0.5 us and 1 us, gated and ungated"_test = [] {
        constexpr std::size_t ungatedLength  = 8192UZ;
        constexpr std::size_t ungatedWindows = 900UZ;
        constexpr std::size_t frameLength    = 240UZ;
        constexpr std::size_t frameLags      = 16UZ;
        constexpr std::size_t gateSpan       = 40'000'000UZ;

        // the ungated window, which is arithmetic rather than analysis: a burst at a 1.3e-4 duty cycle contributes
        // 1.3e-4 of a window's variance, and what the estimate reads instead is the receiver's own envelope wander
        std::vector<float> head;
        head.reserve(ungatedWindows * ungatedLength);
        std::ignore = forEachCs16Magnitude(capturePath(kAdsb), ungatedWindows * ungatedLength, [&head](std::span<const float> batch, std::size_t /*at*/) { head.insert(head.end(), batch.begin(), batch.end()); });
        expect(fatal(ge(head.size(), ungatedWindows * ungatedLength)));

        AcfConfig ungated{};
        ungated.windowLength      = ungatedLength;
        ungated.maxLag            = 8UZ;
        ungated.kind              = AcfKind::Envelope;
        ungated.overlap           = 0.;
        ungated.nAverages         = ungatedWindows;
        const Estimate ungatedArm = estimate<float>(ungated, std::span<const float>(head));
        std::println("criterion 19: ungated, N={} K={}, scatter {:.6f}", ungatedLength, ungatedArm.windows, ungatedArm.scatter);
        std::print("    ");
        for (std::size_t lag = 1UZ; lag <= 6UZ; ++lag) {
            std::print("{:.1f} us: {:.4f}   ", 0.5 * static_cast<double>(lag), ungatedArm.magnitude[lag]);
        }
        std::println("");
        const double flatSpread = *std::ranges::max_element(ungatedArm.magnitude.begin() + 1, ungatedArm.magnitude.begin() + 7) - *std::ranges::min_element(ungatedArm.magnitude.begin() + 1, ungatedArm.magnitude.begin() + 7);
        std::println("    no lag between 0.5 and 3.0 us is distinguishable: the whole run spans {:.4f} on a curve that never falls to the scatter", flatSpread);
        expect(static_cast<double>(ungatedArm.magnitude[2UZ]) > 20. * ungatedArm.scatter) << "the derived null at 1 us is nowhere to be seen in an ungated window";

        // the crude magnitude gate: eight times the median of the span it opens over
        std::vector<float> sampled;
        sampled.reserve(gateSpan / 64UZ + 1UZ);
        const std::size_t seen = forEachCs16Magnitude(capturePath(kAdsb), gateSpan, [&sampled](std::span<const float> batch, std::size_t at) {
            for (std::size_t k = (64UZ - at % 64UZ) % 64UZ; k < batch.size(); k += 64UZ) {
                sampled.push_back(batch[k]);
            }
        });
        expect(fatal(gt(seen, 0UZ)));
        std::ranges::nth_element(sampled, sampled.begin() + static_cast<std::ptrdiff_t>(sampled.size() / 2UZ));
        const double median  = static_cast<double>(sampled[sampled.size() / 2UZ]);
        const auto   opening = static_cast<float>(8. * median);
        const auto   closing = static_cast<float>(4. * median);

        std::vector<float> crude;
        bool               open        = false;
        std::size_t        openings    = 0UZ;
        std::size_t        wantMore    = 0UZ;
        std::ignore                    = forEachCs16Magnitude(capturePath(kAdsb), gateSpan, [&](std::span<const float> batch, std::size_t /*at*/) {
            for (const float value : batch) {
                if (wantMore > 0UZ) {
                    crude.push_back(value);
                    --wantMore;
                    continue;
                }
                if (!open && value > opening) {
                    open = true;
                    ++openings;
                    crude.push_back(value);
                    wantMore = frameLength - 1UZ;
                } else if (open && value < closing) {
                    open = false;
                }
            }
        });
        const std::size_t crudeWindows = crude.size() / frameLength;
        crude.resize(crudeWindows * frameLength);

        AcfConfig gated{};
        gated.windowLength      = frameLength;
        gated.maxLag            = frameLags;
        gated.kind              = AcfKind::Envelope;
        gated.overlap           = 0.;
        gated.nAverages         = crudeWindows;
        const Estimate crudeArm = estimate<float>(gated, std::span<const float>(crude));
        std::println("criterion 19: the crude magnitude gate at eight times the median ({:.1f}) opened {} times over {} samples, K={}, scatter {:.5f}", 8. * median, openings, seen, crudeArm.windows, crudeArm.scatter);
        std::print("    ");
        for (std::size_t lag = 1UZ; lag <= 5UZ; ++lag) {
            std::print("{:.1f} us: {:.4f}   ", 0.5 * static_cast<double>(lag), crudeArm.magnitude[lag]);
        }
        std::println("");

        // the framer's own gate: the positions the pulse-position scanner nominates as whole long frames
        gr::digital::PpmScanner scanner;
        scanner.prepare(gr::digital::modeS(), 1UZ);
        scanner.threshold = 2.0F;

        std::vector<float> framed;   ///< each nominated frame's own 240 samples, as the receiver delivered them
        std::vector<float> rendered; ///< the same frames' decoded bits rendered as the ideal pulse-position envelope
        std::vector<float> work;
        std::size_t        workStart  = 0UZ;
        std::size_t        admitted   = 0UZ;
        std::size_t        longFrames = 0UZ;
        std::ignore                   = forEachCs16Magnitude(capturePath(kAdsb), std::numeric_limits<std::size_t>::max(), [&](std::span<const float> batch, std::size_t /*at*/) {
            work.insert(work.end(), batch.begin(), batch.end());
            const auto sink = [&](const gr::digital::PpmFrame& frame) {
                if (frame.bits != 112UZ) {
                    return;
                }
                ++longFrames;
                admitted += frame.admitted() ? 1UZ : 0UZ;
                const std::size_t offset = frame.position - workStart;
                if (offset + frameLength > work.size()) {
                    return;
                }
                const auto whole = std::span<const float>(work).subspan(offset, frameLength);
                framed.insert(framed.end(), whole.begin(), whole.end());
                // the same frame's own bits, rendered as the envelope the pulse-position rule defines: the control
                // that says whether the derived pair is absent from the capture or from the arithmetic
                for (std::size_t bit = 0UZ; bit < 112UZ; ++bit) {
                    const bool one = whole[16UZ + 2UZ * bit] > whole[16UZ + 2UZ * bit + 1UZ];
                    rendered.push_back(one ? 1.f : 0.f);
                    rendered.push_back(one ? 0.f : 1.f);
                }
            };
            scanner.seek(workStart);
            const std::size_t used = scanner.consume(std::span<const float>(work), sink);
            work.erase(work.begin(), work.begin() + static_cast<std::ptrdiff_t>(used));
            workStart += used;
        });

        const std::size_t framedWindows = framed.size() / frameLength;
        framed.resize(framedWindows * frameLength);
        expect(fatal(ge(framedWindows, 5UZ))) << std::format("the framer nominated {} long frames, {} of them admitted by the parity", longFrames, admitted);

        gated.nAverages          = framedWindows;
        const Estimate framedArm = estimate<float>(gated, std::span<const float>(framed));
        std::println("criterion 19: the framer's gate nominated {} long frames ({} admitted by the parity), K={}, scatter {:.5f}, threshold {:.5f}", longFrames, admitted, framedArm.windows, framedArm.scatter, framedArm.threshold);
        std::print("    ");
        for (std::size_t lag = 1UZ; lag <= 5UZ; ++lag) {
            std::print("{:.1f} us: {:.4f}   ", 0.5 * static_cast<double>(lag), framedArm.magnitude[lag]);
        }
        std::println("");

        // The same frames' own bits, rendered as the envelope the pulse-position rule defines, are the control: what
        // they read is a closed form of the frames' own bit statistics, so what the capture's arm reads instead is a
        // fact about the receiver's envelope and not about the arithmetic.
        //
        // Over a 112-bit data field the mean-removed envelope is exactly +/-1/2 in each half slot, so with `b` the
        // excess of equal over unequal adjacent bit pairs as a fraction of the 111 pairs a frame holds, the unbiased
        // estimate over K such frames is exactly
        //     |R(1.0 us)|/R(0) = |b|      and      |R(0.5 us)|/R(0) = [112 + 111 b] / 223.
        // The pulse-position rule's derived pair, a maximum of 1/2 and a null at one microsecond, is the balanced
        // case b = 0; a finite set of frames is not balanced, and the one-microsecond reading is that imbalance and
        // nothing else.
        std::size_t adjacent    = 0UZ;
        std::size_t transitions = 0UZ;
        for (std::size_t frame = 0UZ; frame + 224UZ <= rendered.size(); frame += 224UZ) {
            for (std::size_t bit = 0UZ; bit + 1UZ < 112UZ; ++bit) {
                ++adjacent;
                transitions += rendered[frame + 2UZ * bit] != rendered[frame + 2UZ * (bit + 1UZ)] ? 1UZ : 0UZ;
            }
        }
        const double balance     = adjacent == 0UZ ? 0. : 1. - 2. * static_cast<double>(transitions) / static_cast<double>(adjacent);
        const double derivedHalf = (112. + 111. * balance) / 223.;
        const double derivedOne  = std::abs(balance);

        AcfConfig ideal{};
        ideal.windowLength      = 224UZ;
        ideal.maxLag            = 12UZ;
        ideal.kind              = AcfKind::Envelope;
        ideal.overlap           = 0.;
        ideal.nAverages         = rendered.size() / 224UZ;
        const Estimate idealArm = estimate<float>(ideal, std::span<const float>(rendered));
        std::println("criterion 19: the same frames' decoded bits rendered as the ideal envelope, K={}, scatter {:.5f}, threshold {:.5f}: 0.5 us {:.5f} against the closed form {:.5f}, 1.0 us {:.5f} against {:.5f}, over {} adjacent bit pairs {:.1f}% of which differ", idealArm.windows, idealArm.scatter, idealArm.threshold, idealArm.magnitude[1UZ], derivedHalf, idealArm.magnitude[2UZ], derivedOne, adjacent, 100. * static_cast<double>(transitions) / static_cast<double>(adjacent));
        expect(static_cast<double>(idealArm.magnitude[1UZ]) > idealArm.threshold) << "the rendered frames put a maximum at half a microsecond";
        expect(std::abs(static_cast<double>(idealArm.magnitude[1UZ]) - derivedHalf) < 1e-3) << std::format("the half-microsecond maximum is the closed form {:.5f}", derivedHalf);
        expect(std::abs(static_cast<double>(idealArm.magnitude[2UZ]) - derivedOne) < 1e-3) << std::format("the one-microsecond reading is the frames' own bit imbalance {:.5f} and nothing else", derivedOne);
        expect(static_cast<double>(idealArm.magnitude[2UZ]) < 0.25 * static_cast<double>(idealArm.magnitude[1UZ])) << "so the pulse-position rule's null stands far below its maximum";

        // What gating buys on the capture itself is structure where there was none: the ungated curve never falls,
        // and the gated one does.
        double ungatedFloor = 1.;
        for (std::size_t lag = 1UZ; lag <= 6UZ; ++lag) {
            ungatedFloor = std::min(ungatedFloor, static_cast<double>(ungatedArm.magnitude[lag]));
        }
        std::println("criterion 19: gating drops the curve from a floor of {:.4f} over 0.5 to 3.0 us to {:.4f} at 3.0 us; the capture's own pair at 0.5 and 1.0 us reads {:.5f} and {:.5f} against the ideal {:.5f} and {:.5f}", ungatedFloor, framedArm.magnitude[6UZ], framedArm.magnitude[1UZ], framedArm.magnitude[2UZ], idealArm.magnitude[1UZ], idealArm.magnitude[2UZ]);
        expect(static_cast<double>(framedArm.magnitude[6UZ]) < ungatedFloor) << "the gated estimate falls where the ungated one never does";
    };

    "criterion 20: the P25 leg, recorded and not asserted"_test = [] {
        constexpr double      rate    = 125000.;
        constexpr std::size_t length  = 4096UZ;
        constexpr std::size_t lags    = 60UZ;
        constexpr std::size_t windows = 45UZ;
        constexpr double      symbol  = rate / 4800.; // 26.042 samples

        // the loudest block of the capture, which is where a channel is actually keyed
        constexpr std::size_t blockSize = 100'000UZ;
        constexpr std::size_t blocks    = 90UZ;
        std::size_t           bestBlock = 0UZ;
        double                bestPower = -1.;
        for (std::size_t block = 0UZ; block < blocks; ++block) {
            const std::vector<CF> probe = readCf32(capturePath(kP25), 8192UZ, block * blockSize);
            if (probe.empty()) {
                break;
            }
            const double power = meanPower(std::span<const CF>(probe));
            if (power > bestPower) {
                bestPower = power;
                bestBlock = block;
            }
        }

        const std::vector<float> channel = gr::filter::design::designLowpass({.sampleRate = rate, .cutoff = 6250., .transitionWidth = 3000., .attenuationDb = 60.});
        const std::vector<CF>    block   = readCf32(capturePath(kP25), blockSize + channel.size(), bestBlock * blockSize);
        expect(fatal(ge(block.size(), blockSize))) << "the loudest block is readable";
        std::println("criterion 20: the loudest 0.8 s block starts at sample {} with mean power {:.4g}; the symbol period is {:.3f} samples at {} S/s", bestBlock * blockSize, bestPower, symbol, static_cast<std::size_t>(rate));

        AcfConfig config{};
        config.windowLength = length;
        config.maxLag       = lags;
        config.overlap      = 0.5;
        config.nAverages    = windows;

        for (int step = -8; step <= 8; ++step) {
            const double          offset   = static_cast<double>(step) * 2500.;
            const std::vector<CF> filtered = applyFir(std::span<const CF>(tune(std::span<const CF>(block), offset, rate)), std::span<const float>(channel));
            config.kind                    = AcfKind::Envelope;
            const Estimate envelopeArm     = estimate<CF>(config, std::span<const CF>(filtered));
            config.kind                    = AcfKind::Complex;
            const Estimate complexArm      = estimate<CF>(config, std::span<const CF>(filtered));
            if (envelopeArm.magnitude.empty() || complexArm.magnitude.empty()) {
                std::println("    offset {:+6.1f} kHz: no record — the filtered signal is constant modulus over every window", offset / 1000.);
                continue;
            }
            const auto [envelopeAt, envelopeHeight] = peakOver(std::span<const float>(envelopeArm.magnitude), 10UZ, lags);
            const auto [complexAt, complexHeight]   = peakOver(std::span<const float>(complexArm.magnitude), 10UZ, lags);
            const auto [isMax, at]                  = maximumNear(std::span<const float>(envelopeArm.magnitude), 26UZ);
            std::println("    offset {:+6.1f} kHz: envelope lag 26 {:.4f} lag 27 {:.4f}, local maximum near 26: {}; envelope argmax over [10, {}] at {} reading {:.4f}; complex argmax at {} reading {:.4f} (scatter {:.5f})", offset / 1000., envelopeArm.magnitude[26UZ], envelopeArm.magnitude[27UZ], isMax ? std::format("yes at {}", at) : std::string("no"), lags, envelopeAt, envelopeHeight, complexAt, complexHeight, envelopeArm.scatter);
        }
        std::println("criterion 20: recorded and not asserted — a negative here is a recorded negative, because no arm of this capture put a local maximum at the symbol period when the row was specified either");
    };
};

int main() { /* not needed for UT */ }
