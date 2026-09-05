#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/channel/DelayProfile.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/digital/ConstellationBlocks.hpp>
#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/ChannelEqualizer.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/ofdm/SchmidlCoxSync.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

/**
 * The tier's OFDM loopback, rehearsed whole: bits through the constellation encoder, the carrier allocator, the
 * prefix and its inverse transform, a channel of static multipath, a carrier offset and additive noise, then the
 * Schmidl-Cox sync, prefix removal with its forward transform, the equalizer and the decoder, and the bit error rate
 * out the far end at two pinned operating points.
 *
 * The chain the spec names has a separate IFFT and FFT between the allocator and the prefix blocks. The fourier
 * module's FFT block carries no complex symbol record on either side and the module has no inverse block at all, so
 * the transforms are inside CpInsert and CpRemove instead -- the same library kernel, called once each, which is
 * what the section-2 heading already calls those two: the blocks where the domains meet.
 */
namespace qa_ofdm_loopback {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::FrequencyOffset;
using gr::blocks::digital::ConstellationDecoder;
using gr::blocks::digital::ConstellationEncoder;
using gr::blocks::filter::FirFilter;
using gr::blocks::ofdm::CarrierAllocator;
using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using gr::blocks::ofdm::OfdmChannelEqualizer;
using gr::blocks::ofdm::SchmidlCoxSync;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t  kFft    = 64U;
constexpr gr::Size_t  kCp     = 16U;
constexpr gr::Size_t  kSync   = 2U;
constexpr std::size_t kFrame  = 20UZ;
constexpr std::size_t kFrames = 8UZ;
constexpr std::size_t kLead   = 200UZ;
constexpr std::size_t kTrail  = 400UZ;

/// The cut is biased into the prefix by enough to clear both the channel's delay spread and the plateau-midpoint
/// bias the sync leaves: the window then starts inside the prefix, where the symbol's own tail already sits, and the
/// per-carrier rotation it costs is one the least-squares estimate absorbs along with the channel.
constexpr std::int32_t kTimingOffset = -6;

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

[[nodiscard]] std::vector<CF> channelSounding() {
    std::vector<CF> word(kFft, CF{});
    std::uint32_t   state = 987654321U;
    const auto      place = [&word, &state](std::int32_t carrier) {
        state                                            = state * 1664525U + 1013904223U;
        const float re                                   = ((state >> 16U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
        const float im                                   = ((state >> 17U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
        word[gr::ofdm::CarrierMap::binOf(kFft, carrier)] = CF(re, im);
    };
    for (const std::int32_t carrier : dataCarriers()) {
        place(carrier);
    }
    for (const std::int32_t carrier : pilotCarriers()) {
        place(carrier);
    }
    return word;
}

[[nodiscard]] std::vector<CF> pilotCycle() { return {CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)}; }

[[nodiscard]] std::vector<CF> multipathTaps() {
    constexpr double          rate = static_cast<double>(kFft);
    const std::vector<double> delays{0., 2. / rate, 5. / rate, 9. / rate};
    const std::vector<double> powers{0., -3., -6., -9.};
    return gr::channel::tapsFromProfile(std::span<const double>(delays), std::span<const double>(powers), rate, true);
}

/// @brief Everything from bits to the transmitted stream, with no trigger tag: a receiver finds its own frames.
[[nodiscard]] std::vector<CF> transmit(std::span<const std::uint8_t> labels) {
    ConstellationEncoder<float> encoder = make<ConstellationEncoder<float>>({{"constellation", std::string("qpsk")}});
    const auto                  symbols = shim::run<CF>(encoder, labels, 4096UZ);

    const auto      preamble = gr::ofdm::schmidlCoxPreamble(static_cast<std::size_t>(kFft), 52UZ, 0xC0FFEEULL);
    const auto      sounding = channelSounding();
    std::vector<CF> words(preamble.begin(), preamble.end());
    words.insert(words.end(), sounding.begin(), sounding.end());

    CarrierAllocator allocator = make<CarrierAllocator>({{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, //
        {"pilot_symbols", interleave(pilotCycle())}, {"sync_words", interleave(words)}, {"frame_len", static_cast<gr::Size_t>(kFrame)}});

    std::vector<gr::DataSet<CF>>      records;
    std::vector<gr::DataSet<CF>>      scratch(512UZ);
    shim::InputSpan<CF>               inSpan(symbols.samples, 0UZ);
    shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = allocator.processBulk(inSpan, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }
    shim::InputSpan<CF>               tail(std::span<const CF>(symbols.samples).last(symbols.samples.size() - inSpan.consumed), inSpan.consumed);
    shim::OutputSpan<gr::DataSet<CF>> outTail{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = allocator.processEpilogue(tail, outTail);
    for (std::size_t k = 0UZ; k < outTail.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }

    // emit_trigger off: a transmit-side marker would reach the receiver's prefix removal alongside the sync's own
    // and restart the frame at whichever came second
    CpInsert                         cp = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{kCp}}, {"emit_trigger", false}});
    std::vector<CF>                  body(1UZ << 17);
    shim::InputSpan<gr::DataSet<CF>> recordSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
    shim::OutputSpan<CF>             streamSpan{std::span<CF>(body)};
    std::ignore = cp.processBulk(recordSpan, streamSpan);

    std::vector<CF> stream(kLead, CF{});
    stream.insert(stream.end(), body.begin(), body.begin() + static_cast<std::ptrdiff_t>(streamSpan.count));
    stream.insert(stream.end(), kTrail, CF{});
    return stream;
}

struct Result {
    std::size_t frames    = 0UZ;
    std::size_t symbols   = 0UZ;
    std::size_t bitErrors = 0UZ;
    std::size_t bits      = 0UZ;
    double      evmDb     = 0.;
    double      ber       = 0.;
};

[[nodiscard]] Result receive(std::span<const CF> stream, std::span<const std::uint8_t> labels, float floor) {
    SchmidlCoxSync sync   = make<SchmidlCoxSync>({{"fft_len", kFft}, {"cp_len", kCp}, {"r_floor", floor}, {"min_gap", static_cast<gr::Size_t>(kFrame)}});
    const auto     synced = shim::run<CF>(sync, stream, 4096UZ);

    CpRemove remove = make<CpRemove>({{"fft_len", kFft}, {"cp_len", std::vector<gr::Size_t>{kCp}}, {"n_sync", kSync}, //
        {"frame_len", static_cast<gr::Size_t>(kFrame)}, {"timing_offset", kTimingOffset}});

    std::vector<gr::DataSet<CF>>      records;
    std::vector<gr::DataSet<CF>>      scratch(512UZ);
    shim::InputSpan<CF>               inSpan(std::span<const CF>(synced.samples), 0UZ, std::span<const gr::Tag>(synced.tags));
    shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = remove.processBulk(inSpan, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }

    OfdmChannelEqualizer equalizer = make<OfdmChannelEqualizer>({{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, //
        {"pilot_symbols", interleave(pilotCycle())}, {"sync_word", interleave(channelSounding())},                                                          //
        {"n_sync", kSync}, {"sync_index", gr::Size_t{1U}}, {"tracking", std::string("cpe")}});

    std::vector<gr::DataSet<CF>>         equalized;
    std::vector<gr::DataSet<float>>      estimates;
    shim::InputSpan<gr::DataSet<CF>>     recordSpan(std::span<const gr::DataSet<CF>>(records), 0UZ);
    shim::OutputSpan<gr::DataSet<CF>>    equalSpan{std::span<gr::DataSet<CF>>(scratch)};
    shim::OutputSpan<gr::DataSet<float>> channelSpan(std::span<gr::DataSet<float>>{}, 0UZ, nullptr, false);
    std::ignore = equalizer.processBulk(recordSpan, equalSpan, channelSpan);
    for (std::size_t k = 0UZ; k < equalSpan.count; ++k) {
        equalized.push_back(std::move(scratch[k]));
    }

    std::vector<CF> decided;
    for (const gr::DataSet<CF>& symbol : equalized) {
        decided.insert(decided.end(), symbol.signal_values.begin(), symbol.signal_values.end());
    }
    ConstellationDecoder<float> decoder = make<ConstellationDecoder<float>>({{"constellation", std::string("qpsk")}});
    const auto                  bits    = shim::run<std::uint8_t>(decoder, std::span<const CF>(decided), 4096UZ);

    const gr::digital::Constellation<float> points = gr::digital::Constellation<float>::qpsk();
    Result                                  result;
    result.frames  = sync.nTriggers();
    result.symbols = decided.size();
    double error   = 0.;
    double signal  = 0.;
    for (std::size_t k = 0UZ; k < std::min(bits.samples.size(), labels.size()); ++k) {
        result.bitErrors += static_cast<std::size_t>(std::popcount(static_cast<unsigned>(bits.samples[k] ^ labels[k]) & 3U));
        result.bits += 2UZ;
        const CF nearest = points.point(points.hardDecision(decided[k]));
        error += static_cast<double>(std::norm(decided[k] - nearest));
        signal += static_cast<double>(std::norm(nearest));
    }
    result.evmDb = 10. * std::log10(std::max(error / std::max(signal, 1e-30), 1e-30));
    result.ber   = result.bits == 0UZ ? 1. : static_cast<double>(result.bitErrors) / static_cast<double>(result.bits);
    return result;
}

/// @brief What zero forcing costs on this profile: the response over the data carriers and the noise it multiplies.
struct Selectivity {
    double minDb  = 0.;
    double maxDb  = 0.;
    double meanDb = 0.; ///< 10*log10 of the mean of 1/|H|^2, which is the factor zero forcing multiplies the noise by
};

[[nodiscard]] Selectivity selectivity() {
    const auto  taps = multipathTaps();
    Selectivity result{1e30, -1e30, 0.};
    double      sum  = 0.;
    const auto  data = dataCarriers();
    for (const std::int32_t carrier : data) {
        std::complex<double> h{};
        for (std::size_t d = 0UZ; d < taps.size(); ++d) {
            const double angle = -2. * std::numbers::pi * static_cast<double>(carrier) * static_cast<double>(d) / static_cast<double>(kFft);
            h += std::complex<double>(taps[d]) * std::complex<double>(std::cos(angle), std::sin(angle));
        }
        const double magnitude = std::abs(h);
        result.minDb           = std::min(result.minDb, 20. * std::log10(magnitude));
        result.maxDb           = std::max(result.maxDb, 20. * std::log10(magnitude));
        sum += 1. / (magnitude * magnitude);
    }
    result.meanDb = 10. * std::log10(sum / static_cast<double>(data.size()));
    return result;
}

const boost::ut::suite<"OFDM tier gate"> _loopback = [] {
    using namespace boost::ut;

    std::vector<std::uint8_t> labels(kFrames * kFrame * dataCarriers().size());
    std::uint32_t             state = 13579U;
    for (std::uint8_t& label : labels) {
        state = state * 1664525U + 1013904223U;
        label = static_cast<std::uint8_t>((state >> 16U) & 3U);
    }

    const std::vector<CF> clean = transmit(std::span<const std::uint8_t>(labels));

    double meanPower = 0.;
    for (std::size_t k = kLead; k + kTrail < clean.size(); ++k) {
        meanPower += static_cast<double>(std::norm(clean[k]));
    }
    meanPower /= static_cast<double>(clean.size() - kLead - kTrail);
    const float floor = static_cast<float>(0.25 * meanPower * static_cast<double>(kFft / 2U));

    /**
     * The two operating points are recorded with their envelopes rather than against a bound argued in advance,
     * because what sets the rate here is not the additive noise alone. Zero forcing divides by the channel, so a
     * carrier the profile puts in a null carries its own noise multiplied by the same amount; the mean of 1/|H|^2
     * over the data carriers is what that costs, and it is printed beside the rates. The same chain over a flat
     * channel is run at each point too, so the share belonging to the profile can be read off the difference rather
     * than inferred. The bounds are set a factor above what the seeded scenes measure: a regression shows and
     * ordinary movement does not.
     */
    "criterion 5: the whole chain, at two pinned operating points"_test = [&clean, &labels, floor, meanPower] {
        const Selectivity profile = selectivity();
        std::println("the static profile over the 48 data carriers: |H| from {:.1f} to {:+.1f} dB, and zero forcing multiplies the noise by {:+.1f} dB on average", //
            profile.minDb, profile.maxDb, profile.meanDb);

        struct Point {
            double      snrDb;
            double      eps;
            double      berBound;
            const char* name;
        };
        constexpr std::array<Point, 2UZ> points{Point{20., 0.25, 1e-3, "20 dB, eps +0.25"}, Point{12., -0.40, 5e-2, "12 dB, eps -0.40"}};

        for (const Point& point : points) {
            for (const bool fading : {false, true}) {
                std::vector<CF> carried = clean;
                if (fading) {
                    FirFilter<CF, CF> channel = make<FirFilter<CF, CF>>({{"taps", multipathTaps()}});
                    carried                   = shim::run<CF>(channel, std::span<const CF>(clean), 4096UZ).samples;
                }
                FrequencyOffset<CF> offset  = make<FrequencyOffset<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"frequency_offset", point.eps}});
                const auto          shifted = shim::run<CF>(offset, std::span<const CF>(carried), 4096UZ);
                AwgnChannel<CF>     noise   = make<AwgnChannel<CF>>({{"noise_power", meanPower / std::pow(10., point.snrDb / 10.)}, {"seed", std::uint64_t{424242}}});
                const auto          noisy   = shim::run<CF>(noise, std::span<const CF>(shifted.samples), 4096UZ);

                const Result got = receive(std::span<const CF>(noisy.samples), std::span<const std::uint8_t>(labels), floor);
                std::println("criterion 5 at {} over a {} channel: {} of {} frames found, {} symbols decoded, EVM {:.1f} dB, {} bit errors of {} -> BER {:.2e}", //
                    point.name, fading ? "multipath" : "flat", got.frames, kFrames, got.symbols, got.evmDb, got.bitErrors, got.bits, got.ber);

                expect(eq(got.frames, kFrames)) << std::format("{}: every frame is found", point.name);
                expect(eq(got.symbols, labels.size())) << std::format("{}: every data carrier is decoded", point.name);
                if (fading) {
                    expect(le(got.ber, point.berBound)) << std::format("{}: bit error rate", point.name);
                }
            }
        }
    };
};

} // namespace qa_ofdm_loopback

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
