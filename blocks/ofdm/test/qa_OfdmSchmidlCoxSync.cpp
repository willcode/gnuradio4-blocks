#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <print>
#include <span>
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

[[nodiscard]] std::vector<std::int32_t> pilotCarriers() { return {-21, -7, 7, 21}; }

[[nodiscard]] std::vector<std::int32_t> dataCarriers() {
    std::vector<std::int32_t> carriers;
    const auto                pilots = pilotCarriers();
    for (std::int32_t c = -26; c <= 26; ++c) {
        if (c == 0 || std::ranges::find(pilots, c) != pilots.end()) {
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

/// @brief One transmitted frame: a Schmidl-Cox preamble and `kFrame` QPSK-loaded data symbols, with silence in front.
///
/// The frame's first sample -- the first sample of the preamble's own prefix -- is at index `kLead`, which is what
/// every timing assertion here is measured against.
[[nodiscard]] std::vector<CF> transmit() {
    const auto      data     = dataCarriers();
    const auto      pilots   = pilotCarriers();
    const auto      preamble = gr::ofdm::schmidlCoxPreamble(static_cast<std::size_t>(kFft), 52UZ, 0xC0FFEEULL);
    std::vector<CF> pilotValues{CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)};

    CarrierAllocator allocator = make<CarrierAllocator>({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, //
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

    CpInsert        cp = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{kCp}}});
    std::vector<CF> stream(kLead, CF{});
    std::vector<CF> body(8192UZ);
    {
        shim::InputSpan<gr::DataSet<CF>> inSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
        shim::OutputSpan<CF>             outSpan{std::span<CF>(body)};
        std::ignore = cp.processBulk(inSpan, outSpan);
        stream.insert(stream.end(), body.begin(), body.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    }
    stream.insert(stream.end(), 96UZ, CF{}); // trailing silence, so the delayed output carries the whole frame
    return stream;
}

[[nodiscard]] double meanPower(std::span<const CF> stream) {
    double sum = 0.;
    for (const CF& sample : stream) {
        sum += static_cast<double>(std::norm(sample));
    }
    return sum / static_cast<double>(stream.size());
}

/// The energy floor the metric's ratio is guarded by, set from the signal the scene actually carries: a fraction of
/// what a correlation window of fft_len/2 samples holds when the preamble is present.
[[nodiscard]] float energyFloor(std::span<const CF> stream) {
    const double perSample = meanPower(stream.subspan(kLead, static_cast<std::size_t>(kCp + kFft)));
    return static_cast<float>(0.25 * perSample * static_cast<double>(kFft / 2U));
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

/// @brief The residual offset a stream still carries, read off the preamble's own repeated halves.
///
/// The preamble at `at` is `cp_len` prefix samples followed by two identical halves; correlating the halves gives
/// `pi * eps` exactly as the block's own estimator does, so this measures what the correction left behind.
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
     * The exact-correlation plateau runs from the frame's first sample to `cp_len` samples later, so its own midpoint
     * is `t0 + cp_len/2` and the rule the block applies is unbiased against it. The threshold crossings are wider than
     * that plateau, and where they fall depends on the preamble's time-domain energy profile: at `d = t0 - delta` the
     * correlation holds the energy of the window's last `L - delta` samples while the reference holds all `L`, so the
     * rising edge is early by however many samples carry the first fifth of the symbol's energy, and the falling edge
     * is late by the matching count at the other end plus what the incoherent terms against the next symbol add. The
     * residue is a property of the sequence and not of the estimator, so what is asserted is that it stays inside a
     * quarter of the prefix -- where the prefix itself absorbs it, and `timing_offset` is the setting for the rest --
     * and that it does not depend on how the stream is chunked.
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

    "silence emits no trigger"_test = [floor] {
        const std::vector<CF> quiet(2048UZ, CF{});
        SchmidlCoxSync        block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}});
        std::ignore                 = shim::run<CF>(block, std::span<const CF>(quiet), 256UZ);
        expect(eq(block.nTriggers(), std::uint64_t{0}));
    };

    "the fractional offset is estimated and corrected to under a thousandth of a spacing"_test = [&clean, floor] {
        for (const double eps : {-0.4, 0.25}) {
            // sample_rate fft_len makes one subcarrier spacing exactly one hertz, so the offset is eps itself
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

    "the dead time keeps one frame from being found twice"_test = [&clean, floor] {
        std::vector<CF> pair = clean;
        pair.insert(pair.end(), clean.begin(), clean.end());

        SchmidlCoxSync  block = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"min_gap", gr::Size_t{5U}}});
        const Detection found = sync(block, std::span<const CF>(pair), 256UZ);
        expect(eq(found.biases.size(), 2UZ)) << "two frames, two triggers";
        if (found.biases.size() == 2UZ) {
            expect(eq(found.biases[1UZ] - found.biases[0UZ], std::ssize(clean)));
        }
    };

    "settings the estimator cannot work under are refused by name"_test = [] {
        expect(throws([] { std::ignore = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", gr::Size_t{80U}}}); })) << "a prefix longer than the symbol";
        expect(throws([] { std::ignore = make<SchmidlCoxSync>({{"fft_len", kFft}, {"threshold", 1.f}}); })) << "a threshold M cannot reach";
        expect(throws([] { std::ignore = make<SchmidlCoxSync>({{"fft_len", kFft}, {"r_floor", 0.f}}); })) << "a floor that guards nothing";
    };
};

} // namespace qa_ofdm_sync

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
