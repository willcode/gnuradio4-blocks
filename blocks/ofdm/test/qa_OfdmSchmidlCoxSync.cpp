#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/ofdm/SchmidlCoxSync.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_sync {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::FrequencyOffset;
using gr::blocks::ofdm::CarrierAllocator;
using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::SchmidlCoxSync;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t  kFft   = 64U;
constexpr gr::Size_t  kCp    = 16U;
constexpr std::size_t kLead  = 137UZ; ///< silence before the frame, so the trigger is not at sample zero
constexpr std::size_t kFrame = 4UZ;

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    if constexpr (requires { block.start(); }) {
        block.start();
    }
    return block;
}

/// @brief The transform, the preamble's prefix, the occupied carriers `-edge..edge` without DC, and the pilots among them.
struct Numerology {
    gr::Size_t                fft;
    gr::Size_t                cp;
    std::int32_t              edge;
    std::vector<std::int32_t> pilots;
};

[[nodiscard]] Numerology narrow() { return {kFft, kCp, 26, {-21, -7, 7, 21}}; }

[[nodiscard]] std::vector<std::int32_t> dataCarriers(const Numerology& numerology) {
    std::vector<std::int32_t> carriers;
    for (std::int32_t c = -numerology.edge; c <= numerology.edge; ++c) {
        if (c == 0 || std::ranges::find(numerology.pilots, c) != numerology.pilots.end()) {
            continue;
        }
        carriers.push_back(c);
    }
    return carriers;
}

[[nodiscard]] std::vector<float> interleave(std::span<const CF> values) {
    std::vector<float> flat(2UZ * values.size());
    for (std::size_t k = 0UZ; k < values.size(); ++k) {
        flat[2UZ * k]       = values[k].real();
        flat[2UZ * k + 1UZ] = values[k].imag();
    }
    return flat;
}

/// @brief One transmitted frame, a Schmidl-Cox preamble and `kFrame` QPSK-loaded data symbols, with silence in front.
///
/// The frame's first sample, the first sample of the preamble's own prefix, is at index `kLead`. Every timing
/// assertion here is measured against it.
[[nodiscard]] std::vector<CF> transmit(const Numerology& numerology = narrow()) {
    const auto      data     = dataCarriers(numerology);
    const auto      preamble = gr::ofdm::schmidlCoxPreamble(static_cast<std::size_t>(numerology.fft), 2UZ * static_cast<std::size_t>(numerology.edge), 0xC0FFEEULL).value();
    std::vector<CF> pilotValues{CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)};

    CarrierAllocator allocator = make<CarrierAllocator>({{"fft_len", numerology.fft}, {"data_carriers", data}, {"pilot_carriers", numerology.pilots}, //
        {"pilot_symbols", interleave(pilotValues)}, {"sync_words", interleave(preamble)}, {"frame_len", static_cast<gr::Size_t>(kFrame)}});

    std::vector<CF> payload(kFrame * data.size());
    std::uint32_t   state = 12345U;
    for (CF& symbol : payload) {
        state          = state * 1664525U + 1013904223U;
        const float re = ((state >> 16U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
        const float im = ((state >> 17U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
        symbol         = CF(re, im);
    }

    std::vector<gr::DataSet<CF>> records;
    std::vector<gr::DataSet<CF>> scratch(64UZ);
    {
        shim::InputSpan<CF>               inSpan(std::span<const CF>(payload), 0UZ);
        shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
        std::ignore = allocator.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
        shim::InputSpan<CF>               tail(std::span<const CF>(payload).last(payload.size() - inSpan.consumed), inSpan.consumed);
        shim::OutputSpan<gr::DataSet<CF>> outTail{std::span<gr::DataSet<CF>>(scratch)};
        std::ignore = allocator.processEpilogue(tail, outTail);
        for (std::size_t k = 0UZ; k < outTail.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
    }

    CpInsert        cp = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{numerology.cp}}});
    std::vector<CF> stream(kLead, CF{});
    std::vector<CF> body(8192UZ);
    {
        shim::InputSpan<gr::DataSet<CF>> inSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
        shim::OutputSpan<CF>             outSpan{std::span<CF>(body)};
        std::ignore = cp.processBulk(inSpan, outSpan);
        stream.insert(stream.end(), body.begin(), body.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    }
    // two symbols of trailing silence, longer than the lag: the delayed output then carries the whole frame
    stream.insert(stream.end(), 2UZ * static_cast<std::size_t>(numerology.fft + numerology.cp), CF{});
    return stream;
}

[[nodiscard]] double meanPower(std::span<const CF> stream) {
    double sum = 0.;
    for (const CF& sample : stream) {
        sum += static_cast<double>(std::norm(sample));
    }
    return sum / static_cast<double>(stream.size());
}

/// The energy floor of the metric, set from the scene's signal: a quarter of the energy that a window of fft_len/2
/// preamble samples holds.
[[nodiscard]] float energyFloor(std::span<const CF> stream, const Numerology& numerology = narrow()) {
    const double perSample = meanPower(stream.subspan(kLead, static_cast<std::size_t>(numerology.cp + numerology.fft)));
    return static_cast<float>(0.25 * perSample * static_cast<double>(numerology.fft / 2U));
}

struct Detection {
    std::vector<std::int64_t>  biases{}; ///< trigger sample minus the constructed frame start
    std::vector<double>        cfo{};
    std::vector<std::uint32_t> plateau{}; ///< samples the metric stayed above the threshold
};

[[nodiscard]] Detection sync(SchmidlCoxSync& block, std::span<const CF> stream, std::size_t chunk, std::vector<CF>* corrected = nullptr) {
    const auto result = shim::run<CF>(block, stream, chunk);
    Detection  found;
    for (const gr::Tag& tag : result.tags) {
        const auto entry = tag.map.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
        if (entry == tag.map.end()) {
            continue;
        }
        found.biases.push_back(static_cast<std::int64_t>(tag.index) - static_cast<std::int64_t>(block.delaySamples()) - static_cast<std::int64_t>(kLead));
        const auto meta = tag.map.find(std::pmr::string(gr::tag::TRIGGER_META_INFO.shortKey()));
        if (meta != tag.map.end()) {
            if (const auto* map = meta->second.get_if<gr::property_map>(); map != nullptr) {
                const auto value = map->find(std::pmr::string("cfo_fractional"));
                if (value != map->end()) {
                    if (const auto* eps = value->second.get_if<float>(); eps != nullptr) {
                        found.cfo.push_back(static_cast<double>(*eps));
                    }
                }
                const auto width = map->find(std::pmr::string("plateau_samples"));
                if (width != map->end()) {
                    if (const auto* samples = width->second.get_if<gr::Size_t>(); samples != nullptr) {
                        found.plateau.push_back(*samples);
                    }
                }
            }
        }
    }
    if (corrected != nullptr) {
        *corrected = result.samples;
    }
    return found;
}

/// @brief The samples and tags of a run in the framework's order of calls.
struct Driven {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

/**
 * @brief Drives the block in the framework's order: `forwardTags()` then `processBulk()` on each chunk, and the
 * end-of-stream call last.
 *
 * The tag cursor hands each call the tags up to the end of its window and retires them by the rule a released span
 * applies. The end-of-stream call passes `processedIn` 0 and spans of no samples. Its input span holds every tag the
 * cursor has not retired.
 */
struct Drive {
    SchmidlCoxSync&     block;
    std::span<const CF> input;
    shim::TagCursor     cursor;
    Driven              out{};
    std::size_t         position = 0UZ;

    void to(std::size_t end, std::size_t chunk) {
        std::vector<CF> scratch(chunk);
        while (position < end) {
            const std::size_t    count = std::min(chunk, end - position);
            shim::InputSpan<CF>  inSpan(input.subspan(position, count), position, cursor.window(position, count));
            shim::OutputSpan<CF> outSpan(std::span<CF>(scratch.data(), count), position, &out.tags);
            auto                 inputs  = std::tie(inSpan);
            auto                 outputs = std::tie(outSpan);
            block.forwardTags(inputs, outputs, count);
            std::ignore = block.processBulk(inSpan, outSpan);
            cursor.retire(inSpan);
            out.samples.insert(out.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
            position += count;
        }
    }

    void finish() {
        shim::InputSpan<CF>  inSpan(std::span<const CF>{}, position, cursor.window(position, std::numeric_limits<std::size_t>::max() - position));
        shim::OutputSpan<CF> outSpan(std::span<CF>{}, position, &out.tags);
        auto                 inputs  = std::tie(inSpan);
        auto                 outputs = std::tie(outSpan);
        block.forwardTags(inputs, outputs, 0UZ);
    }
};

/// @brief The `mark` and `trigger_name` tags of a run, as `key=value@index` in the order the block published them.
[[nodiscard]] std::vector<std::string> tagList(std::span<const gr::Tag> tags) {
    std::vector<std::string> list;
    for (const gr::Tag& tag : tags) {
        for (const auto& [key, value] : tag.map) {
            if (key == "mark" || key == std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())) {
                list.push_back(std::format("{}={}@{}", std::string(key), value.value_or(std::string{}), tag.index));
            }
        }
    }
    return list;
}

/// @brief The residual offset a stream still carries, read off the preamble's own repeated halves.
///
/// The preamble at `at` is `cp_len` prefix samples followed by two identical halves. Correlating the halves gives
/// `pi * eps`, as in the block's own estimator.
[[nodiscard]] double residualCfo(std::span<const CF> stream, std::size_t at) {
    const std::size_t    half  = static_cast<std::size_t>(kFft) / 2UZ;
    const std::size_t    start = at + static_cast<std::size_t>(kCp);
    std::complex<double> sum{};
    for (std::size_t m = 0UZ; m < half; ++m) {
        sum += std::conj(std::complex<double>(stream[start + m])) * std::complex<double>(stream[start + m + half]);
    }
    return std::atan2(sum.imag(), sum.real()) / std::numbers::pi;
}

const boost::ut::suite<"OFDM Schmidl-Cox sync"> _sync = [] {
    using namespace boost::ut;

    const std::vector<CF> clean = transmit();
    const float           floor = energyFloor(std::span<const CF>(clean));

    /**
     * The exact-correlation plateau runs from the frame's first sample `t0` to `cp_len` samples later, and its
     * midpoint is `t0 + cp_len/2`. The threshold crossings are wider than that plateau, and their position depends on
     * the preamble's energy over time. At `d = t0 - delta` the correlation holds the energy of the window's last
     * `L - delta` samples, and the reference holds all `L`. The rising edge is early by the samples that carry the
     * first fifth of the symbol's energy. The falling edge is late by the matching count at the other end, plus the
     * incoherent terms against the next symbol. This bias belongs to the sequence. The test asserts that it stays
     * inside a quarter of the prefix and that it does not depend on how the stream is chunked.
     */
    "the trigger lands within a quarter prefix of the constructed frame start, at any chunking"_test = [&clean, floor] {
        std::vector<std::int64_t> biases;
        for (const std::size_t chunk : {1UZ, 5UZ, 64UZ, 331UZ, 4096UZ}) {
            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            const Detection found = sync(block, std::span<const CF>(clean), chunk);
            expect(eq(found.biases.size(), 1UZ)) << std::format("chunk {}: one preamble, one trigger", chunk);
            if (found.biases.empty()) {
                continue;
            }
            biases.push_back(found.biases[0UZ]);
            expect(le(4 * std::abs(found.biases[0UZ]), static_cast<std::int64_t>(kCp))) << std::format("chunk {}: plateau-midpoint bias", chunk);
            expect(eq(block.nShortPlateaus(), std::uint64_t{0}));
            if (chunk == 4096UZ) {
                std::println("plateau-midpoint timing bias, no impairment: {:+} samples of a {}-sample prefix, the crossings {} samples wide against the {}-sample exact plateau", //
                    found.biases[0UZ], kCp, found.plateau[0UZ], kCp + 1U);
            }
        }
        expect(eq(std::ranges::count(biases, biases.front()), std::ssize(biases))) << "the same stream chunked differently triggers at the same sample";
    };

    /**
     * A prefix much shorter than the transform leaves the crossings far wider than the exact plateau. The sample that
     * ends such a crossing arrives long after the frame's first sample, and a call that starts at that sample has
     * already published every output before it.
     */
    "a short prefix loses no frame at any chunk size"_test = [] {
        for (const Numerology& numerology : {Numerology{256U, 16U, 100, {-75, -25, 25, 75}}, Numerology{512U, 32U, 200, {-150, -50, 50, 150}}}) {
            const std::vector<CF>     stream = transmit(numerology);
            const float               rFloor = energyFloor(std::span<const CF>(stream), numerology);
            std::vector<std::int64_t> biases;
            std::uint32_t             width = 0U;
            for (std::size_t chunk = 1UZ; chunk <= 64UZ; ++chunk) {
                SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", numerology.fft}, {"cp_len", numerology.cp}, {"r_floor", rFloor}, {"correct_cfo", false}});
                const Detection found = sync(block, std::span<const CF>(stream), chunk);
                expect(eq(found.biases.size(), 1UZ)) << std::format("fft_len {} cp_len {} chunk {}: one preamble, one trigger", numerology.fft, numerology.cp, chunk);
                expect(eq(block.nLate(), std::uint64_t{0})) << std::format("fft_len {} cp_len {} chunk {}: no detection is late", numerology.fft, numerology.cp, chunk);
                if (!found.biases.empty()) {
                    biases.push_back(found.biases[0UZ]);
                    width = found.plateau[0UZ];
                }
            }
            expect(eq(biases.size(), 64UZ));
            if (biases.empty()) {
                continue;
            }
            expect(eq(std::ranges::count(biases, biases.front()), std::ssize(biases))) << "the same stream chunked differently triggers at the same sample";
            std::println("fft_len {} cp_len {}: trigger bias {:+} samples, the crossings {} samples wide against the {}-sample exact plateau", numerology.fft, numerology.cp, biases.front(), width, numerology.cp + 1U);
        }
    };

    "a crossing wider than any preamble's plateau emits no trigger"_test = [&clean, floor] {
        // a tone repeats at every lag, and M stays at 1 for as long as it lasts
        const auto      amplitude = static_cast<float>(std::sqrt(meanPower(std::span<const CF>(clean).subspan(kLead, static_cast<std::size_t>(kCp + kFft)))));
        std::vector<CF> tone(kLead, CF{});
        for (std::size_t n = 0UZ; n < 1000UZ; ++n) {
            tone.push_back(std::polar(amplitude, static_cast<float>(0.1 * std::numbers::pi * static_cast<double>(n))));
        }
        tone.insert(tone.end(), 256UZ, CF{});
        for (const std::size_t chunk : {1UZ, 64UZ, 4096UZ}) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            std::ignore          = shim::run<CF>(block, std::span<const CF>(tone), chunk);
            expect(eq(block.nTriggers(), std::uint64_t{0})) << std::format("chunk {}", chunk);
            expect(eq(block.nWidePlateaus(), std::uint64_t{1})) << std::format("chunk {}: the tone's crossing is refused", chunk);
        }
    };

    "silence emits no trigger"_test = [floor] {
        const std::vector<CF> quiet(2048UZ, CF{});
        SchmidlCoxSync        block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
        std::ignore                 = shim::run<CF>(block, std::span<const CF>(quiet), 256UZ);
        expect(eq(block.nTriggers(), std::uint64_t{0}));
    };

    "a NaN or infinite sample costs no later frame"_test = [&clean, floor] {
        std::vector<CF> twice = clean;
        twice.insert(twice.end(), clean.begin(), clean.end());
        for (const CF bad : {CF(std::numeric_limits<float>::quiet_NaN(), 0.f), CF(std::numeric_limits<float>::infinity(), 0.f)}) {
            std::vector<CF> input = twice;
            input[10UZ]           = bad; // in the silence before the first frame
            SchmidlCoxSync block  = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            std::ignore           = shim::run<CF>(block, std::span<const CF>(input), 64UZ);
            expect(eq(block.nTriggers(), std::uint64_t{2})) << std::format("{} at sample 10: two frames, two triggers", bad.real());
        }
    };

    "the fractional offset is estimated and corrected to under a thousandth of a spacing"_test = [&clean, floor] {
        for (const double eps : {-0.4, 0.25}) {
            // at a sample_rate of fft_len one subcarrier spacing is one hertz, and frequency_offset equals eps
            FrequencyOffset<CF> impair  = make<FrequencyOffset<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"frequency_offset", eps}});
            const auto          shifted = shim::run<CF>(impair, std::span<const CF>(clean), 512UZ);
            const double        truth   = residualCfo(std::span<const CF>(shifted.samples), kLead);

            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
            std::vector<CF> corrected;
            const Detection found = sync(block, std::span<const CF>(shifted.samples), 97UZ, &corrected);

            expect(eq(found.cfo.size(), 1UZ)) << std::format("eps {}", eps);
            if (found.cfo.empty()) {
                continue;
            }
            const double estimateError = std::abs(found.cfo[0UZ] - eps);
            const double residual      = std::abs(residualCfo(std::span<const CF>(corrected), kLead + block.delaySamples()));
            std::println("eps {:+.2f}: the stream carries {:+.6f}, the estimate is {:+.6f} (error {:.2e}), the corrected stream leaves {:.2e}", eps, truth, found.cfo[0UZ], estimateError, residual);
            expect(lt(estimateError, 1e-3)) << std::format("eps {}: the estimate", eps);
            expect(lt(residual, 1e-3)) << std::format("eps {}: what the correction left", eps);
            expect(le(4 * std::abs(found.biases[0UZ]), static_cast<std::int64_t>(kCp))) << std::format("eps {}: an offset does not move the timing", eps);
        }
    };

    "detection statistics through additive noise at 10 dB"_test = [&clean, floor] {
        constexpr std::size_t kTrials = 200UZ;
        const double          signal  = meanPower(std::span<const CF>(clean).subspan(kLead, kFrame * static_cast<std::size_t>(kCp + kFft)));
        const double          noise   = signal / 10.; // 10 dB, both quadratures together

        std::size_t               detected = 0UZ;
        std::size_t               spurious = 0UZ;
        std::vector<std::int64_t> biases;
        std::vector<double>       errors;
        for (std::size_t trial = 0UZ; trial < kTrials; ++trial) {
            AwgnChannel<CF> channel = make<AwgnChannel<CF>>({{"noise_power", noise}, {"seed", std::uint64_t{trial + 1UZ}}});
            const auto      noisy   = shim::run<CF>(channel, std::span<const CF>(clean), 1024UZ);

            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            const Detection found = sync(block, std::span<const CF>(noisy.samples), 512UZ);
            if (found.biases.empty()) {
                continue;
            }
            ++detected;
            spurious += found.biases.size() - 1UZ;
            biases.push_back(found.biases[0UZ]);
            errors.push_back(found.cfo.empty() ? 0. : found.cfo[0UZ]);
        }

        std::ranges::sort(biases);
        double mean = 0.;
        for (const std::int64_t bias : biases) {
            mean += static_cast<double>(bias);
        }
        mean /= static_cast<double>(biases.size());
        double variance = 0.;
        double worstCfo = 0.;
        for (std::size_t k = 0UZ; k < biases.size(); ++k) {
            variance += (static_cast<double>(biases[k]) - mean) * (static_cast<double>(biases[k]) - mean);
            worstCfo = std::max(worstCfo, std::abs(errors[k]));
        }
        variance /= static_cast<double>(biases.size());

        std::println("AWGN at 10 dB, {} trials: detected {}, spurious {}; timing bias mean {:+.2f} samples, sigma {:.2f}, range [{}, {}]; worst |cfo| {:.4f} spacings", //
            kTrials, detected, spurious, mean, std::sqrt(variance), biases.front(), biases.back(), worstCfo);

        expect(eq(detected, kTrials)) << "every frame is found at 10 dB";
        expect(eq(spurious, 0UZ)) << "and found once";
        expect(lt(4. * std::abs(mean), static_cast<double>(kCp))) << "the plateau midpoint stays inside a quarter of the prefix";
        expect(lt(std::sqrt(variance), 3.0)) << "and its spread is a small fraction of the prefix";
        expect(lt(worstCfo, 0.1)) << "no trial's fractional estimate is out by a tenth of a spacing";
    };

    /**
     * Two frames back to back put the second preamble `kFrame + 1` symbols after the first. A dead time shorter than
     * that finds both preambles, and a longer one refuses the second.
     */
    "the dead time refuses a preamble inside it"_test = [&clean, floor] {
        const std::size_t frameSamples = (kFrame + 1UZ) * static_cast<std::size_t>(kCp + kFft);
        const auto        frame        = std::span<const CF>(clean).subspan(kLead, frameSamples);
        std::vector<CF>   stream(kLead, CF{});
        stream.insert(stream.end(), frame.begin(), frame.end());
        stream.insert(stream.end(), frame.begin(), frame.end());
        stream.insert(stream.end(), clean.size() - kLead - frameSamples, CF{});

        for (const auto& [gap, triggers] : {std::pair{gr::Size_t{1U}, 2UZ}, std::pair{static_cast<gr::Size_t>(kFrame + 2UZ), 1UZ}}) {
            SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"min_gap", gap}});
            const Detection found = sync(block, std::span<const CF>(stream), 256UZ);
            expect(eq(found.biases.size(), triggers)) << std::format("min_gap {}: two preambles {} samples apart", gap, frameSamples);
            if (found.biases.size() == 2UZ) {
                expect(le(4 * std::abs(found.biases[1UZ] - std::ssize(frame)), static_cast<std::int64_t>(kCp))) << "the second trigger lands within a quarter prefix of its frame";
            }
        }
    };

    "a burst that ends with the stream leaves its last delaySamples() inputs in the delay line"_test = [&clean, floor] {
        const std::span<const CF>  burst = std::span<const CF>(clean).first(kLead + (kFrame + 1UZ) * static_cast<std::size_t>(kCp + kFft));
        const std::vector<gr::Tag> inTags{gr::Tag{burst.size() - 1UZ, gr::property_map{{"mark", std::string("last")}}}};
        SchmidlCoxSync             block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
        Drive                      drive{block, burst, shim::TagCursor(std::span<const gr::Tag>(inTags))};
        drive.to(burst.size(), 64UZ);
        drive.finish();

        const std::size_t         delay   = block.delaySamples();
        const std::span<const CF> out     = drive.out.samples;
        const auto                trigger = std::ranges::find_if(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
        expect(eq(out.size(), burst.size())) << "one output per input";
        expect(std::ranges::all_of(out.first(delay), [](const CF& sample) { return sample == CF{}; })) << "the delay line starts empty";
        expect(std::ranges::equal(out.subspan(delay), burst.first(burst.size() - delay))) << "each input comes out delaySamples() later";
        expect(trigger != drive.out.tags.end()) << "the frame's trigger is output";
        if (trigger == drive.out.tags.end()) {
            return;
        }
        expect(le(4 * std::abs(static_cast<std::int64_t>(trigger->index - delay) - static_cast<std::int64_t>(kLead)), static_cast<std::int64_t>(kCp)));
        const std::vector<std::string> expected{std::format("trigger_name=ofdm_frame@{}", trigger->index), std::format("mark=last@{}", burst.size())};
        expect(tagList(drive.out.tags) == expected) << std::format("the tag on the last input leaves at the end-of-stream index: {}", tagList(drive.out.tags));
    };

    /**
     * In the framework's order of calls, an input tag near the frame's start leaves on its own sample `delaySamples()`
     * later. A tag held past the last output and a tag at the end-of-stream index leave at that index in the order of
     * their samples.
     */
    "input tags keep their samples beside a detection, at any chunking"_test = [&clean, floor] {
        const std::size_t          end = clean.size();
        const std::vector<gr::Tag> inTags{gr::Tag{kLead + 10UZ, gr::property_map{{"mark", std::string("frame")}}}, //
            gr::Tag{end - 5UZ, gr::property_map{{"mark", std::string("tail")}}},                                   //
            gr::Tag{end, gr::property_map{{"mark", std::string("end")}}},                                          //
            gr::Tag{end, gr::property_map{{static_cast<std::pmr::string>(gr::tag::END_OF_STREAM), true}}}};
        for (const std::size_t chunk : {1UZ, 7UZ, 64UZ, 4096UZ}) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            Drive          drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
            drive.to(end, chunk);
            drive.finish();

            const auto trigger = std::ranges::find_if(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
            expect(trigger != drive.out.tags.end()) << std::format("chunk {}: the frame's trigger is output", chunk);
            if (trigger == drive.out.tags.end()) {
                continue;
            }
            const std::vector<std::string> expected{std::format("trigger_name=ofdm_frame@{}", trigger->index), std::format("mark=frame@{}", kLead + 10UZ + block.delaySamples()), //
                std::format("mark=tail@{}", end), std::format("mark=end@{}", end)};
            expect(tagList(drive.out.tags) == expected) << std::format("chunk {}: {} against {}", chunk, tagList(drive.out.tags), expected);
            expect(std::ranges::none_of(drive.out.tags, [](const gr::Tag& tag) { return tag.map.contains(static_cast<std::pmr::string>(gr::tag::END_OF_STREAM)); })) << std::format("chunk {}: the input's end_of_stream key is dropped", chunk);
        }
    };

    /**
     * At sample 240 the block has confirmed the frame that starts at 137 and has not yet output its trigger's sample.
     * It also holds the input tag on sample 150. A change of a key the lag does not depend on keeps all of that.
     */
    "a threshold, min_gap or r_floor change keeps the delay line, the held detection and the held tags"_test = [&clean, floor] {
        const std::vector<gr::Tag> inTags{gr::Tag{150UZ, gr::property_map{{"mark", std::string("held")}}}};
        constexpr std::size_t      kSplit = 240UZ;

        struct Run {
            Driven        out{};
            std::uint64_t triggersAtSplit = 0ULL;
        };
        const auto run = [&clean, &inTags, floor](const gr::property_map& change) {
            SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"correct_cfo", false}});
            Drive          drive{block, std::span<const CF>(clean), shim::TagCursor(std::span<const gr::Tag>(inTags))};
            drive.to(kSplit, 64UZ);
            Run result;
            result.triggersAtSplit = block.nTriggers();
            if (!change.empty()) {
                std::ignore = block.settings().setStaged(change);
                std::ignore = block.settings().applyStagedParameters();
            }
            drive.to(clean.size(), 64UZ);
            drive.finish();
            result.out = std::move(drive.out);
            return result;
        };

        const Run  reference = run({});
        const auto expected  = tagList(reference.out.tags);
        const auto trigger   = std::ranges::find_if(reference.out.tags, [](const gr::Tag& tag) { return tag.map.contains(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey())); });
        expect(eq(reference.triggersAtSplit, std::uint64_t{1})) << "the detection is confirmed before the change";
        expect(trigger != reference.out.tags.end() && trigger->index >= kSplit) << "and its sample is output after it";
        expect(eq(expected.size(), 2UZ)) << "one trigger and one input tag";
        for (const gr::property_map& change : {gr::property_map{{"threshold", 0.65f}}, gr::property_map{{"min_gap", gr::Size_t{2U}}}, gr::property_map{{"r_floor", 0.5f * floor}}}) {
            const Run  changed = run(change);
            const auto key     = std::string(change.begin()->first);
            expect(changed.out.samples == reference.out.samples) << std::format("{}: the delay line", key);
            expect(tagList(changed.out.tags) == expected) << std::format("{}: the held detection and tag, {} against {}", key, tagList(changed.out.tags), expected);
        }
    };

    "settings the estimator cannot work under are refused by name"_test = [] {
        expect(throws([] { std::ignore = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", gr::Size_t{80U}}}); })) << "a prefix longer than the symbol";
        expect(throws([] { std::ignore = make<SchmidlCoxSync>({{"fft_len", kFft}, {"threshold", 1.f}}); })) << "a threshold M cannot reach";
        expect(throws([] { std::ignore = make<SchmidlCoxSync>({{"fft_len", kFft}, {"r_floor", 0.f}}); })) << "a floor of zero";
    };

    "start() throws nothing after a refused setting"_test = [floor] {
        SchmidlCoxSync block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
        std::ignore          = block.settings().setStaged({{"threshold", 1.5f}});
        expect(throws([&block] { std::ignore = block.settings().applyStagedParameters(); })) << "the settings application refuses it";
        expect(nothrow([&block] { block.start(); })) << "a restart throws nothing";
    };

    "a block left at its default settings finds the frame"_test = [&clean] {
        SchmidlCoxSync  block = make<SchmidlCoxSync>({});
        const Detection found = sync(block, std::span<const CF>(clean), 64UZ);
        expect(eq(found.biases.size(), 1UZ));
        if (!found.biases.empty()) {
            expect(le(4 * std::abs(found.biases[0UZ]), static_cast<std::int64_t>(kCp)));
        }
    };
};

} // namespace qa_ofdm_sync

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
