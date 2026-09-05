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
#include <gnuradio-4.0/algorithm/channel/DelayProfile.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/CarrierImpairments.hpp>
#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/ChannelEqualizer.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_equalizer {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::channel::PhaseNoise;
using gr::blocks::filter::FirFilter;
using gr::blocks::ofdm::CarrierAllocator;
using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using gr::blocks::ofdm::OfdmChannelEqualizer;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t  kFft   = 64U;
constexpr gr::Size_t  kCp    = 16U;
constexpr gr::Size_t  kSync  = 2U; ///< a Schmidl-Cox preamble for timing, then the known symbol least squares reads
constexpr std::size_t kFrame = 20UZ;

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

[[nodiscard]] CF qpsk(std::uint32_t& state) {
    state          = state * 1664525U + 1013904223U;
    const float re = ((state >> 16U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
    const float im = ((state >> 17U) & 1U) != 0U ? 0.70710678f : -0.70710678f;
    return CF(re, im);
}

/// The known full symbol least squares divides by: every occupied carrier carries a point, which is what makes the
/// estimate defined everywhere the data is.
[[nodiscard]] std::vector<CF> channelSounding() {
    std::vector<CF> word(kFft, CF{});
    std::uint32_t   state = 987654321U;
    for (const std::int32_t carrier : dataCarriers()) {
        word[gr::ofdm::CarrierMap::binOf(kFft, carrier)] = qpsk(state);
    }
    for (const std::int32_t carrier : pilotCarriers()) {
        word[gr::ofdm::CarrierMap::binOf(kFft, carrier)] = qpsk(state);
    }
    return word;
}

[[nodiscard]] std::vector<CF> pilotCycle() { return {CF(1.f, 0.f), CF(1.f, 0.f), CF(1.f, 0.f), CF(-1.f, 0.f)}; }

[[nodiscard]] gr::property_map numerology() {
    const auto      preamble = gr::ofdm::schmidlCoxPreamble(static_cast<std::size_t>(kFft), 52UZ, 0xC0FFEEULL);
    const auto      sounding = channelSounding();
    std::vector<CF> words(preamble.begin(), preamble.end());
    words.insert(words.end(), sounding.begin(), sounding.end());
    return {{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, //
        {"pilot_symbols", interleave(pilotCycle())}, {"sync_words", interleave(words)}, {"frame_len", static_cast<gr::Size_t>(kFrame)}};
}

[[nodiscard]] gr::property_map equalizerSettings(std::string tracking) {
    return {{"fft_len", kFft}, {"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, //
        {"pilot_symbols", interleave(pilotCycle())}, {"sync_word", interleave(channelSounding())},     //
        {"n_sync", kSync}, {"sync_index", gr::Size_t{1U}}, {"tracking", std::move(tracking)}};
}

struct Modulated {
    std::vector<CF>              payload{};
    std::vector<gr::DataSet<CF>> records{};
};

/// @brief One frame of QPSK data through the allocator: the payload it was given and the symbols it produced.
[[nodiscard]] Modulated modulate(std::size_t frames = 1UZ) {
    CarrierAllocator allocator = make<CarrierAllocator>(numerology());

    Modulated     result;
    std::uint32_t state = 24680U;
    result.payload.resize(frames * kFrame * dataCarriers().size());
    for (CF& symbol : result.payload) {
        symbol = qpsk(state);
    }

    std::vector<gr::DataSet<CF>>      scratch(256UZ);
    shim::InputSpan<CF>               inSpan(std::span<const CF>(result.payload), 0UZ);
    shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = allocator.processBulk(inSpan, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        result.records.push_back(std::move(scratch[k]));
    }
    shim::InputSpan<CF>               tail(std::span<const CF>(result.payload).last(result.payload.size() - inSpan.consumed), inSpan.consumed);
    shim::OutputSpan<gr::DataSet<CF>> outTail{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = allocator.processEpilogue(tail, outTail);
    for (std::size_t k = 0UZ; k < outTail.count; ++k) {
        result.records.push_back(std::move(scratch[k]));
    }
    return result;
}

struct Stream {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

[[nodiscard]] Stream toStream(std::span<const gr::DataSet<CF>> records) {
    CpInsert                         cp = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{kCp}}});
    Stream                           result;
    std::vector<CF>                  body(1UZ << 16);
    shim::InputSpan<gr::DataSet<CF>> inSpan(records, 0UZ);
    shim::OutputSpan<CF>             outSpan{std::span<CF>(body), 0UZ, &result.tags};
    std::ignore = cp.processBulk(inSpan, outSpan);
    result.samples.assign(body.begin(), body.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
    return result;
}

[[nodiscard]] std::vector<gr::DataSet<CF>> toRecords(const Stream& stream) {
    CpRemove block = make<CpRemove>({{"fft_len", kFft}, {"cp_len", std::vector<gr::Size_t>{kCp}}, {"n_sync", kSync}, {"frame_len", static_cast<gr::Size_t>(kFrame)}});

    std::vector<gr::DataSet<CF>>      records;
    std::vector<gr::DataSet<CF>>      scratch(256UZ);
    shim::InputSpan<CF>               inSpan(std::span<const CF>(stream.samples), 0UZ, std::span<const gr::Tag>(stream.tags));
    shim::OutputSpan<gr::DataSet<CF>> outSpan{std::span<gr::DataSet<CF>>(scratch)};
    std::ignore = block.processBulk(inSpan, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }
    return records;
}

struct Equalized {
    std::vector<gr::DataSet<CF>>    symbols{};
    std::vector<gr::DataSet<float>> estimates{};
};

[[nodiscard]] Equalized equalize(OfdmChannelEqualizer& block, std::span<const gr::DataSet<CF>> records, bool wantEstimate = false) {
    Equalized                       result;
    std::vector<gr::DataSet<CF>>    scratch(256UZ);
    std::vector<gr::DataSet<float>> estimateScratch(16UZ);

    shim::InputSpan<gr::DataSet<CF>>     inSpan(records, 0UZ);
    shim::OutputSpan<gr::DataSet<CF>>    outSpan{std::span<gr::DataSet<CF>>(scratch)};
    shim::OutputSpan<gr::DataSet<float>> channelSpan(wantEstimate ? std::span<gr::DataSet<float>>(estimateScratch) : std::span<gr::DataSet<float>>{}, 0UZ, nullptr, wantEstimate);
    std::ignore = block.processBulk(inSpan, outSpan, channelSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        result.symbols.push_back(std::move(scratch[k]));
    }
    for (std::size_t k = 0UZ; k < channelSpan.count; ++k) {
        result.estimates.push_back(std::move(estimateScratch[k]));
    }
    return result;
}

[[nodiscard]] float metaFloat(const gr::DataSet<CF>& record, std::string_view key) {
    const auto entry = record.meta_information[0UZ].find(std::pmr::string(key));
    if (entry == record.meta_information[0UZ].end()) {
        return 0.f;
    }
    const auto* value = entry->second.get_if<float>();
    return value == nullptr ? 0.f : *value;
}

/// @brief The error vector magnitude of a whole run, in dB, against the nearest QPSK point.
[[nodiscard]] double errorVectorDb(std::span<const gr::DataSet<CF>> symbols) {
    const gr::digital::Constellation<float> points = gr::digital::Constellation<float>::qpsk();
    double                                  error  = 0.;
    double                                  signal = 0.;
    for (const gr::DataSet<CF>& symbol : symbols) {
        for (const CF& value : symbol.signal_values) {
            const CF nearest = points.point(points.hardDecision(value));
            error += static_cast<double>(std::norm(value - nearest));
            signal += static_cast<double>(std::norm(nearest));
        }
    }
    return 10. * std::log10(std::max(error / signal, 1e-30));
}

[[nodiscard]] std::size_t symbolErrors(std::span<const gr::DataSet<CF>> symbols, std::span<const CF> payload) {
    const gr::digital::Constellation<float> points = gr::digital::Constellation<float>::qpsk();
    std::size_t                             wrong  = 0UZ;
    std::size_t                             at     = 0UZ;
    for (const gr::DataSet<CF>& symbol : symbols) {
        for (const CF& value : symbol.signal_values) {
            if (at >= payload.size()) {
                return wrong;
            }
            if (points.hardDecision(value) != points.hardDecision(payload[at])) {
                ++wrong;
            }
            ++at;
        }
    }
    return wrong;
}

/// The static profile every impaired scene here runs through: four paths over nine samples, which at 64 samples a
/// symbol is a delay spread of an eighth of the transform and well inside the prefix.
[[nodiscard]] std::vector<CF> multipathTaps() {
    constexpr double          rate = static_cast<double>(kFft);
    const std::vector<double> delays{0., 2. / rate, 5. / rate, 9. / rate};
    const std::vector<double> powers{0., -3., -6., -9.};
    return gr::channel::tapsFromProfile(std::span<const double>(delays), std::span<const double>(powers), rate, true);
}

const boost::ut::suite<"OFDM channel equalizer"> _equalizer = [] {
    using namespace boost::ut;

    "criterion 1: on an ideal channel the data comes back as it went in"_test = [] {
        const Modulated      sent  = modulate();
        OfdmChannelEqualizer block = make<OfdmChannelEqualizer>(equalizerSettings("cpe"));
        const Equalized      got   = equalize(block, std::span<const gr::DataSet<CF>>(sent.records), true);

        expect(eq(got.symbols.size(), kFrame)) << "one output record per data symbol, and none for the sync words";
        expect(eq(block.nSkipped(), std::uint64_t{1})) << "the preamble is passed over; the sounding symbol is read";
        expect(eq(block.nUntrained(), std::uint64_t{0}));
        expect(eq(block.nEstimates(), std::uint64_t{1}));
        expect(eq(got.estimates.size(), 1UZ)) << "one channel-estimate record per frame";

        const auto  data  = dataCarriers();
        double      worst = 0.;
        std::size_t at    = 0UZ;
        for (const gr::DataSet<CF>& symbol : got.symbols) {
            expect(eq(symbol.signal_values.size(), data.size()));
            for (std::size_t k = 0UZ; k < data.size(); ++k) {
                worst = std::max(worst, static_cast<double>(std::abs(symbol.signal_values[k] - sent.payload[at + k])));
                expect(symbol.axis_values[0UZ][k] == CF(static_cast<float>(data[k]), 0.f)) << "the axis states the carrier each value came from";
            }
            at += data.size();
        }
        std::println("criterion 1, ideal channel: worst data-carrier error {:.3e}, EVM {:.1f} dB", worst, errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols)));
        expect(lt(worst, 1e-5)) << "the data reproduces to float rounding";

        // the estimate of an ideal channel is unit magnitude and zero phase on every occupied carrier
        const gr::DataSet<float>& estimate = got.estimates[0UZ];
        expect(eq(estimate.signal_values.size(), 2UZ * (data.size() + pilotCarriers().size())));
        for (std::size_t k = 0UZ; k < estimate.signal_values.size() / 2UZ; ++k) {
            expect(lt(std::abs(estimate.signal_values[k] - 1.f), 1e-5f));
        }
    };

    "criterion 4: static multipath, with and without additive noise"_test = [] {
        const Modulated sent  = modulate();
        const Stream    clean = toStream(std::span<const gr::DataSet<CF>>(sent.records));
        const auto      taps  = multipathTaps();

        FirFilter<CF, CF> channelFilter = make<FirFilter<CF, CF>>({{"taps", taps}});
        Stream            faded;
        faded.tags    = clean.tags;
        faded.samples = shim::run<CF>(channelFilter, std::span<const CF>(clean.samples), 4096UZ).samples;

        {
            OfdmChannelEqualizer block = make<OfdmChannelEqualizer>(equalizerSettings("cpe"));
            const Equalized      got   = equalize(block, std::span<const gr::DataSet<CF>>(toRecords(faded)));
            const double         evm   = errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols));
            std::println("criterion 4, static multipath alone (0/-3/-6/-9 dB at 0/2/5/9 samples): EVM {:.1f} dB, {} symbol errors", evm, symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)));
            expect(lt(evm, -60.)) << "the prefix makes the channel circular and zero forcing inverts it outright";
            expect(eq(symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)), 0UZ));
        }

        double meanPower = 0.;
        for (const CF& sample : faded.samples) {
            meanPower += static_cast<double>(std::norm(sample));
        }
        meanPower /= static_cast<double>(faded.samples.size());

        for (const double snrDb : {20., 15.}) {
            AwgnChannel<CF> noise = make<AwgnChannel<CF>>({{"noise_power", meanPower / std::pow(10., snrDb / 10.)}, {"seed", std::uint64_t{7}}});
            Stream          noisy;
            noisy.tags    = faded.tags;
            noisy.samples = shim::run<CF>(noise, std::span<const CF>(faded.samples), 4096UZ).samples;

            OfdmChannelEqualizer block = make<OfdmChannelEqualizer>(equalizerSettings("cpe"));
            const Equalized      got   = equalize(block, std::span<const gr::DataSet<CF>>(toRecords(noisy)));
            const double         evm   = errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols));
            const std::size_t    wrong = symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload));
            std::println("criterion 4, the same multipath at {:.0f} dB SNR: EVM {:.1f} dB, {} of {} symbols wrong", snrDb, evm, wrong, kFrame * dataCarriers().size());
            expect(lt(evm, -snrDb + 12.)) << "zero forcing costs noise enhancement at the profile's nulls, bounded here";
        }
    };

    "criterion 4: phase noise is held by cpe tracking and is not held without it"_test = [] {
        const Modulated sent  = modulate();
        const Stream    clean = toStream(std::span<const gr::DataSet<CF>>(sent.records));

        const auto through = [&clean](double linewidth, const std::string& tracking) {
            PhaseNoise<CF> jitter = make<PhaseNoise<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"linewidth", linewidth}, {"seed", std::uint64_t{2026}}});
            Stream         noisy;
            noisy.tags    = clean.tags;
            noisy.samples = shim::run<CF>(jitter, std::span<const CF>(clean.samples), 4096UZ).samples;

            OfdmChannelEqualizer block = make<OfdmChannelEqualizer>(equalizerSettings(tracking));
            return equalize(block, std::span<const gr::DataSet<CF>>(toRecords(noisy)));
        };

        constexpr double kLinewidth = 0.02; // hertz, against a subcarrier spacing of one: a slow oscillator
        const Equalized  tracked    = through(kLinewidth, "cpe");
        const Equalized  untracked  = through(kLinewidth, "none");

        const double      trackedEvm     = errorVectorDb(std::span<const gr::DataSet<CF>>(tracked.symbols));
        const double      untrackedEvm   = errorVectorDb(std::span<const gr::DataSet<CF>>(untracked.symbols));
        const std::size_t trackedWrong   = symbolErrors(std::span<const gr::DataSet<CF>>(tracked.symbols), std::span<const CF>(sent.payload));
        const std::size_t untrackedWrong = symbolErrors(std::span<const gr::DataSet<CF>>(untracked.symbols), std::span<const CF>(sent.payload));

        float lastCpe = 0.f;
        for (const gr::DataSet<CF>& symbol : tracked.symbols) {
            lastCpe = metaFloat(symbol, "cpe_rad");
        }
        std::println("criterion 4, phase noise of {} Hz against a 1 Hz spacing over {} symbols: cpe EVM {:.1f} dB with {} symbol errors, no tracking EVM {:.1f} dB with {} errors; the last symbol was turned by {:+.3f} rad", //
            kLinewidth, kFrame, trackedEvm, trackedWrong, untrackedEvm, untrackedWrong, lastCpe);

        // The strict inequality the spec asks for is the symbol count and not the error vector: once the phase has
        // turned past a decision boundary the nearest point is a different point, so the error vector stops growing
        // while every decision under it is wrong. What cpe leaves is the phase noise inside a symbol rather than
        // between symbols, which turns each carrier against its neighbors and no common phase can remove.
        expect(lt(trackedEvm, untrackedEvm)) << "tracking lowers the error vector";
        expect(eq(trackedWrong, 0UZ)) << "cpe holds the constellation";
        expect(gt(untrackedWrong, kFrame * dataCarriers().size() / 4UZ)) << "and without it the frame provably loses lock";
    };

    "the interpolated update follows a channel the sync word alone does not"_test = [] {
        const Modulated sent  = modulate();
        const Stream    clean = toStream(std::span<const gr::DataSet<CF>>(sent.records));

        // A frequency ramp is the impairment a per-carrier update answers and a common phase does not: it turns each
        // carrier by its own angle, which is what the interpolation between pilots is there to follow.
        PhaseNoise<CF> jitter = make<PhaseNoise<CF>>({{"sample_rate", static_cast<float>(kFft)}, {"linewidth", 0.05}, {"seed", std::uint64_t{99}}});
        Stream         noisy;
        noisy.tags         = clean.tags;
        noisy.samples      = shim::run<CF>(jitter, std::span<const CF>(clean.samples), 4096UZ).samples;
        const auto records = toRecords(noisy);

        double best = 0.;
        for (const std::string& tracking : {std::string("none"), std::string("cpe"), std::string("cpe_interp")}) {
            OfdmChannelEqualizer block = make<OfdmChannelEqualizer>(equalizerSettings(tracking));
            const Equalized      got   = equalize(block, std::span<const gr::DataSet<CF>>(records));
            const double         evm   = errorVectorDb(std::span<const gr::DataSet<CF>>(got.symbols));
            std::println("phase noise of 0.05 Hz, tracking '{}': EVM {:.1f} dB, {} symbol errors", tracking, evm, symbolErrors(std::span<const gr::DataSet<CF>>(got.symbols), std::span<const CF>(sent.payload)));
            if (tracking == "none") {
                best = evm;
            } else {
                expect(lt(evm, best)) << std::format("tracking '{}' is better than none", tracking);
            }
        }
    };

    "a numerology the estimator cannot work on is refused by name"_test = [] {
        expect(throws([] {
            gr::property_map settings                  = equalizerSettings("cpe");
            std::vector<CF>  word                      = channelSounding();
            word[gr::ofdm::CarrierMap::binOf(kFft, 3)] = CF{};
            settings["sync_word"]                      = interleave(word);
            std::ignore                                = make<OfdmChannelEqualizer>(settings);
        })) << "a sync word silent on an occupied carrier";
        expect(throws([] {
            gr::property_map settings = equalizerSettings("cpe");
            settings["sync_index"]    = gr::Size_t{4U};
            std::ignore               = make<OfdmChannelEqualizer>(settings);
        })) << "a sync index outside the frame's head";
        expect(throws([] {
            gr::property_map settings = equalizerSettings("wander");
            std::ignore               = make<OfdmChannelEqualizer>(settings);
        })) << "a tracking mode the block has no rule for";
        expect(throws([] {
            gr::property_map settings = equalizerSettings("cpe");
            settings["alpha"]         = 0.f;
            std::ignore               = make<OfdmChannelEqualizer>(settings);
        })) << "a smoother that never moves";
    };
};

} // namespace qa_ofdm_equalizer

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
