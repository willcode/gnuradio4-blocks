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
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_cp {

using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr std::size_t kFft = 64UZ;

/// The reference inverse transform: the library kernel the block itself uses, scaled the way the block scales it, so
/// a comparison against it is exact rather than approximate.
[[nodiscard]] std::vector<CF> inverseTransform(std::span<const CF> spectrum) {
    gr::algorithm::FFT<CF, CF, gr::algorithm::Direction::Backward> inverse{};
    std::vector<CF>                                                time(spectrum.size());
    inverse.compute(spectrum, std::span<CF>(time));
    const float scale = 1.f / static_cast<float>(spectrum.size());
    for (CF& value : time) {
        value *= scale;
    }
    return time;
}

[[nodiscard]] std::vector<CF> forwardTransform(std::span<const CF> time) {
    gr::algorithm::FFT<CF, CF, gr::algorithm::Direction::Forward> forward{};
    std::vector<CF>                                               spectrum(time.size());
    forward.compute(time, std::span<CF>(spectrum));
    return spectrum;
}

/// A frequency-domain symbol whose every bin is distinguishable, so a misplaced sample names itself.
[[nodiscard]] gr::DataSet<CF> symbol(std::size_t index, std::size_t symbolInFrame, std::size_t fftLength = kFft) {
    std::vector<CF> values(fftLength);
    for (std::size_t k = 0UZ; k < fftLength; ++k) {
        values[k] = CF(static_cast<float>(index + 1UZ) * 0.125f * static_cast<float>(k + 1UZ), 0.25f * static_cast<float>(k) - static_cast<float>(index));
    }
    std::vector<int> carriers(fftLength);
    for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
        carriers[bin] = gr::ofdm::CarrierMap::carrierOf(fftLength, bin);
    }
    return gr::blocks::ofdm::detail::makeSymbolRecord(std::move(values), std::span<const int>(carriers), "data", "test", //
        gr::property_map{{std::pmr::string("symbol_in_frame"), gr::pmt::Value(static_cast<std::uint64_t>(symbolInFrame))}});
}

struct Stream {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Drive a records-in, stream-out block, re-presenting whatever a call did not consume.
[[nodiscard]] Stream insert(CpInsert& block, std::span<const gr::DataSet<CF>> records, std::size_t outRoom) {
    Stream          result;
    std::vector<CF> scratch(outRoom);

    std::size_t consumed = 0UZ;
    while (consumed < records.size()) {
        shim::InputSpan<gr::DataSet<CF>> inSpan(records.subspan(consumed), consumed);
        shim::OutputSpan<CF>             outSpan(std::span<CF>(scratch.data(), outRoom), result.samples.size(), &result.tags);
        std::ignore = block.processBulk(inSpan, outSpan);
        result.samples.insert(result.samples.end(), scratch.begin(), scratch.begin() + static_cast<std::ptrdiff_t>(outSpan.count));
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ) {
            break;
        }
        consumed += inSpan.consumed;
    }
    return result;
}

/// @brief Drive a stream-in, records-out block over @p chunk samples a call, tags placed at absolute indices.
[[nodiscard]] std::vector<gr::DataSet<CF>> remove(CpRemove& block, const Stream& input, std::size_t chunk, std::size_t outRoom = 64UZ) {
    std::vector<gr::DataSet<CF>> records;
    std::vector<gr::DataSet<CF>> scratch(outRoom);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.samples.size()) {
        fed              = std::min(input.samples.size(), fed + chunk);
        const auto first = std::ranges::lower_bound(input.tags, consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(input.tags, fed, std::ranges::less{}, &gr::Tag::index);

        shim::InputSpan<CF>               inSpan(std::span<const CF>(input.samples).subspan(consumed, fed - consumed), consumed, std::span<const gr::Tag>(first, last));
        shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == input.samples.size()) {
            break;
        }
        consumed += inSpan.consumed;
    }
    return records;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// The stream a prefix cycle and a list of symbols must produce, built independently of the block.
[[nodiscard]] std::vector<CF> expectedStream(std::span<const gr::DataSet<CF>> records, std::span<const gr::Size_t> cycle) {
    std::vector<CF> stream;
    for (std::size_t s = 0UZ; s < records.size(); ++s) {
        const std::vector<CF> time   = inverseTransform(records[s].signal_values);
        const std::size_t     prefix = static_cast<std::size_t>(cycle[s % cycle.size()]);
        stream.insert(stream.end(), time.end() - static_cast<std::ptrdiff_t>(prefix), time.end());
        stream.insert(stream.end(), time.begin(), time.end());
    }
    return stream;
}

const boost::ut::suite<"OFDM cyclic prefix"> _cyclicPrefix = [] {
    using namespace boost::ut;

    "the prefix is the symbol's own tail, for a scalar and for a cycle"_test = [] {
        for (const std::vector<gr::Size_t>& cycle : {std::vector<gr::Size_t>{16U}, std::vector<gr::Size_t>{16U, 4U, 8U}}) {
            std::vector<gr::DataSet<CF>> records;
            for (std::size_t s = 0UZ; s < 6UZ; ++s) {
                records.push_back(symbol(s, s));
            }
            CpInsert     block  = make<CpInsert>({{"cp_len", cycle}});
            const Stream stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

            const std::vector<CF> expected = expectedStream(std::span<const gr::DataSet<CF>>(records), std::span<const gr::Size_t>(cycle));
            expect(eq(stream.samples.size(), expected.size()));
            expect(std::ranges::equal(stream.samples, expected)) << "the emitted stream is bit-identical to the prefix algebra";
            expect(eq(stream.tags.size(), 1UZ)) << "one frame start, at symbol_in_frame 0";
            expect(eq(stream.tags[0UZ].index, 0UZ));
        }
    };

    "the prefix cycle restarts at each frame"_test = [] {
        const std::vector<gr::Size_t> cycle{16U, 4U};
        std::vector<gr::DataSet<CF>>  records;
        for (std::size_t frame = 0UZ; frame < 2UZ; ++frame) {
            for (std::size_t s = 0UZ; s < 3UZ; ++s) {
                records.push_back(symbol(3UZ * frame + s, s)); // symbol_in_frame runs 0,1,2 in both frames
            }
        }
        CpInsert     block  = make<CpInsert>({{"cp_len", cycle}});
        const Stream stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        // 16, 4, 16 in the first frame and again in the second: the cycle follows symbol_in_frame, not the record count
        const std::size_t expectedLength = 6UZ * kFft + 2UZ * (16UZ + 4UZ + 16UZ);
        expect(eq(stream.samples.size(), expectedLength));
        expect(eq(stream.tags.size(), 2UZ));
        expect(eq(stream.tags[1UZ].index, 3UZ * kFft + 16UZ + 4UZ + 16UZ));
    };

    "CpInsert into CpRemove is exact identity on the kept samples, at any chunking"_test = [] {
        for (const std::vector<gr::Size_t>& cycle : {std::vector<gr::Size_t>{16U}, std::vector<gr::Size_t>{16U, 4U, 8U}}) {
            std::vector<gr::DataSet<CF>> records;
            for (std::size_t s = 0UZ; s < 3UZ; ++s) {
                records.push_back(symbol(s, s));
            }
            CpInsert     insertBlock = make<CpInsert>({{"cp_len", cycle}});
            const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

            // What the cut must contain: the samples of the stream the cadence names, taken from the stream itself.
            std::vector<std::vector<CF>> expected;
            std::size_t                  at = 0UZ;
            for (std::size_t s = 0UZ; s < records.size(); ++s) {
                const std::size_t prefix = static_cast<std::size_t>(cycle[s % cycle.size()]);
                expected.push_back(std::vector<CF>(stream.samples.begin() + static_cast<std::ptrdiff_t>(at + prefix), stream.samples.begin() + static_cast<std::ptrdiff_t>(at + prefix + kFft)));
                at += prefix + kFft;
            }

            for (const std::size_t chunk : {1UZ, 3UZ, 17UZ, 80UZ, 4096UZ}) {
                CpRemove   removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", cycle}, {"frame_len", gr::Size_t{3U}}});
                const auto out         = remove(removeBlock, stream, chunk);
                expect(eq(out.size(), records.size())) << std::format("chunk {}", chunk);
                for (std::size_t s = 0UZ; s < out.size(); ++s) {
                    const std::vector<CF> reference = forwardTransform(std::span<const CF>(expected[s]));
                    expect(std::ranges::equal(out[s].signal_values, reference)) << std::format("chunk {} symbol {}", chunk, s);
                }
            }
        }
    };

    "the round trip returns the symbol it started from"_test = [] {
        std::vector<gr::DataSet<CF>> records;
        for (std::size_t s = 0UZ; s < 3UZ; ++s) {
            records.push_back(symbol(s, s));
        }
        CpInsert     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        CpRemove     removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{3U}}});
        const auto   out         = remove(removeBlock, stream, 37UZ);

        expect(eq(out.size(), 3UZ));
        double worst = 0.;
        for (std::size_t s = 0UZ; s < out.size(); ++s) {
            for (std::size_t k = 0UZ; k < kFft; ++k) {
                worst = std::max(worst, static_cast<double>(std::abs(out[s].signal_values[k] - records[s].signal_values[k])));
            }
        }
        std::println("round trip through the inverse and forward transforms: worst bin error {:.3e}", worst);
        expect(lt(worst, 1e-3)) << "the scaled backward transform is the forward one's inverse";
    };

    "a negative timing_offset rotates each carrier by the slope it states"_test = [] {
        constexpr std::int32_t       kOffset = -4;
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ)};

        CpInsert     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        CpRemove     removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"timing_offset", kOffset}});
        const auto   out         = remove(removeBlock, stream, 11UZ);

        expect(eq(out.size(), 1UZ));
        double worst = 0.;
        for (std::size_t bin = 0UZ; bin < kFft; ++bin) {
            const int    carrier = gr::ofdm::CarrierMap::carrierOf(kFft, bin);
            const double phase   = 2. * std::numbers::pi * static_cast<double>(carrier) * static_cast<double>(kOffset) / static_cast<double>(kFft);
            const CF     want    = records[0UZ].signal_values[bin] * CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
            worst                = std::max(worst, static_cast<double>(std::abs(out[0UZ].signal_values[bin] - want)));
        }
        std::println("timing_offset {} against the stated slope 2*pi*k*offset/fft_len: worst bin error {:.3e}", kOffset, worst);
        expect(lt(worst, 1e-3)) << "the bias into the prefix is a pure per-carrier rotation, which the equalizer absorbs";
    };

    "between frames the block discards, and counts what it discarded"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        Stream                       stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        // a gap of silence before the frame, and a tail after it, neither of which the cadence claims
        Stream padded;
        padded.samples.assign(25UZ, CF{});
        padded.samples.insert(padded.samples.end(), stream.samples.begin(), stream.samples.end());
        padded.samples.insert(padded.samples.end(), 40UZ, CF{});
        padded.tags.push_back(gr::Tag{25UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}});

        CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}});
        const auto out   = remove(block, padded, 23UZ);

        expect(eq(out.size(), 2UZ));
        expect(eq(block.nDiscarded(), std::uint64_t{65})) << "25 samples before the trigger and 40 after the frame";
        expect(eq(gr::blocks::ofdm::detail::metaCount(out[1UZ], "discarded_samples", 0ULL), std::uint64_t{25})) << "what the record states is what had been discarded when it was cut";
    };

    "a trigger label the block does not carry starts no frame"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}, {"trigger_label", std::string("other")}});
        const Stream                 stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        CpRemove labeled = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"trigger_label", std::string("ofdm_frame")}});
        expect(eq(remove(labeled, stream, 100UZ).size(), 0UZ));
        expect(eq(labeled.nDiscarded(), stream.samples.size()));

        CpRemove any = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}});
        expect(eq(remove(any, stream, 100UZ).size(), 1UZ)) << "an empty trigger_label accepts any trigger";
    };

    "edge windowing narrows the spectrum, measured"_test = [] {
        // F5: turning the edge on is a measured improvement, not a given. The scene is 24 occupied carriers of an
        // fft_len of 64, which leaves a guard band wide enough for the shoulder the windowing attacks to be visible
        // well away from the transition; the analysis is Hann-windowed so that its own leakage does not stand in for
        // the signal's.
        constexpr std::size_t        kOccupiedHalf = 12UZ;
        constexpr std::size_t        kSymbols      = 48UZ;
        std::vector<gr::DataSet<CF>> records;
        std::vector<int>             carriers(kFft);
        for (std::size_t bin = 0UZ; bin < kFft; ++bin) {
            carriers[bin] = gr::ofdm::CarrierMap::carrierOf(kFft, bin);
        }
        for (std::size_t s = 0UZ; s < kSymbols; ++s) {
            std::vector<CF> values(kFft, CF{});
            for (std::size_t k = 1UZ; k <= kOccupiedHalf; ++k) {
                const float sign = ((s + k) % 2UZ) == 0UZ ? 1.f : -1.f;
                values[k]        = CF(sign, -sign);
                values[kFft - k] = CF(-sign, sign);
            }
            records.push_back(gr::blocks::ofdm::detail::makeSymbolRecord(std::move(values), std::span<const int>(carriers), "data", "test", //
                gr::property_map{{std::pmr::string("symbol_in_frame"), gr::pmt::Value(static_cast<std::uint64_t>(s))}}));
        }

        constexpr std::size_t kAnalysis = 2048UZ;
        std::vector<float>    hann(kAnalysis);
        for (std::size_t n = 0UZ; n < kAnalysis; ++n) {
            hann[n] = static_cast<float>(0.5 * (1.0 - std::cos(2. * std::numbers::pi * static_cast<double>(n) / static_cast<double>(kAnalysis))));
        }

        const auto shoulderDb = [&records, &hann](gr::Size_t edge) {
            CpInsert     block  = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}, {"window_len", edge}});
            const Stream stream = insert(block, std::span<const gr::DataSet<CF>>(records), 16384UZ);

            std::vector<CF> segment(kAnalysis);
            for (std::size_t n = 0UZ; n < kAnalysis; ++n) {
                segment[n] = stream.samples[256UZ + n] * hann[n];
            }
            const std::vector<CF> spectrum = forwardTransform(std::span<const CF>(segment));

            // The occupied band is |carrier| <= 12 of 64; everything past |carrier| 20 is the shoulder, with the
            // carriers between left out so the transition counts as neither.
            constexpr std::size_t inEdge  = kOccupiedHalf * kAnalysis / kFft;
            constexpr std::size_t outEdge = 20UZ * kAnalysis / kFft;
            double                inBand  = 0.;
            double                out     = 0.;
            for (std::size_t bin = 0UZ; bin < kAnalysis; ++bin) {
                const std::size_t distance = std::min(bin, kAnalysis - bin);
                const double      power    = static_cast<double>(std::norm(spectrum[bin]));
                if (distance <= inEdge) {
                    inBand += power;
                } else if (distance >= outEdge) {
                    out += power;
                }
            }
            return 10. * std::log10(out / inBand);
        };

        const double plain = shoulderDb(0U);
        for (const gr::Size_t edge : {gr::Size_t{4U}, gr::Size_t{8U}, gr::Size_t{16U}}) {
            const double windowed = shoulderDb(edge);
            std::println("out-of-band power relative to in-band: {:.2f} dB unwindowed, {:.2f} dB with a {}-sample raised-cosine edge, a {:.2f} dB improvement", plain, windowed, edge, plain - windowed);
            expect(lt(windowed, plain)) << "a raised-cosine edge lowers the shoulder";
        }
    };

    "a prefix or an offset the algebra cannot hold is refused by name"_test = [] {
        expect(throws([] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{-17}}}); })) << "past the prefix";
        expect(throws([] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{1}}}); })) << "into the next symbol";
        expect(throws([] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{}}}); })) << "an empty prefix cycle";
        expect(throws([] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{80U}}}); })) << "a prefix longer than the symbol";
        expect(throws([] { std::ignore = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{4U}}, {"window_len", gr::Size_t{8U}}}); })) << "an edge longer than the prefix it overlaps";
    };
};

} // namespace qa_ofdm_cp

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
