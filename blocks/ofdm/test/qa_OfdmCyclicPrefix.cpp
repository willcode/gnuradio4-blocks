#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/fourier/fft.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_cp {

using gr::blocks::ofdm::CpInsert;
using gr::blocks::ofdm::CpRemove;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr std::size_t kFft = 64UZ;

/// The reference inverse transform: the library FFT the block uses, scaled the same way.
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

/// A frequency-domain symbol of distinct values. A record without @p symbolInFrame carries no `symbol_in_frame`.
[[nodiscard]] gr::DataSet<CF> symbol(std::size_t index, std::optional<std::size_t> symbolInFrame, std::size_t fftLength = kFft) {
    std::vector<CF> values(fftLength);
    for (std::size_t k = 0UZ; k < fftLength; ++k) {
        values[k] = CF(static_cast<float>(index + 1UZ) * 0.125f * static_cast<float>(k + 1UZ), 0.25f * static_cast<float>(k) - static_cast<float>(index));
    }
    std::vector<int> carriers(fftLength);
    for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
        carriers[bin] = gr::ofdm::CarrierMap::carrierOf(fftLength, bin).value();
    }
    gr::property_map meta;
    if (symbolInFrame.has_value()) {
        meta[std::pmr::string("symbol_in_frame")] = gr::pmt::Value(static_cast<std::uint64_t>(*symbolInFrame));
    }
    return gr::blocks::ofdm::detail::makeSymbolRecord(std::move(values), std::span<const int>(carriers), "data", "test", std::move(meta));
}

struct Stream {
    std::vector<CF>      samples{};
    std::vector<gr::Tag> tags{};
};

/// @brief Drives `CpInsert` until it has consumed every record.
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

/// @brief Drives `CpRemove` over samples @p begin to @p end of @p input, @p chunk samples a call, with each tag at its
/// absolute index.
[[nodiscard]] std::vector<gr::DataSet<CF>> remove(CpRemove& block, const Stream& input, std::size_t begin, std::size_t end, std::size_t chunk, std::size_t outRoom = 64UZ) {
    std::vector<gr::DataSet<CF>> records;
    std::vector<gr::DataSet<CF>> scratch(outRoom);

    std::size_t consumed = begin;
    std::size_t fed      = begin;
    while (consumed < end) {
        fed              = std::min(end, fed + chunk);
        const auto first = std::ranges::lower_bound(input.tags, consumed, std::ranges::less{}, &gr::Tag::index);
        const auto last  = std::ranges::lower_bound(input.tags, fed, std::ranges::less{}, &gr::Tag::index);

        shim::InputSpan<CF>               inSpan(std::span<const CF>(input.samples).subspan(consumed, fed - consumed), consumed, std::span<const gr::Tag>(first, last));
        shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && fed == end) {
            break;
        }
        consumed += inSpan.consumed;
    }
    return records;
}

[[nodiscard]] std::vector<gr::DataSet<CF>> remove(CpRemove& block, const Stream& input, std::size_t chunk, std::size_t outRoom = 64UZ) { return remove(block, input, 0UZ, input.samples.size(), chunk, outRoom); }

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// @brief Applies @p settings to a running block, as a settings change between two calls does.
template<typename TBlock>
void change(TBlock& block, gr::property_map settings) {
    std::ignore = block.settings().setStaged(std::move(settings));
    std::ignore = block.settings().applyStagedParameters();
}

/// @brief The message a refused setting throws, or an empty string when nothing throws.
template<typename TFunction>
[[nodiscard]] std::string refusal(TFunction&& function) {
    try {
        function();
    } catch (const gr::exception& e) {
        return std::string(e.what());
    }
    return {};
}

[[nodiscard]] std::string signalName(const gr::DataSet<CF>& record) { return record.signal_names.empty() ? std::string{} : record.signal_names[0UZ]; }

/// @brief Runs a source, a `CarrierAllocator`, `CpInsert` and a sink under the framework's scheduler. Returns the
/// samples the sink received.
[[nodiscard]] std::size_t runToSink(std::size_t nInput, std::size_t sampleEdge) {
    using gr::blocks::testing::ProcessFunction;
    std::vector<std::int32_t> carriers;
    for (std::int32_t carrier = -26; carrier <= 26; ++carrier) {
        if (carrier != 0) {
            carriers.push_back(carrier);
        }
    }

    gr::Graph graph;
    auto&     source   = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(nInput)}, {"mark_tag", false}});
    auto&     allocate = graph.emplaceBlock<gr::blocks::ofdm::CarrierAllocator>({{"fft_len", gr::Size_t{kFft}}, {"data_carriers", carriers}});
    auto&     insert   = graph.emplaceBlock<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
    auto&     sink     = graph.emplaceBlock<gr::blocks::testing::TagSink<CF, ProcessFunction::USE_PROCESS_BULK>>({{"log_samples", false}, {"log_tags", false}});
    boost::ut::expect(graph.connect<"out", "in">(source, allocate).has_value());
    boost::ut::expect(graph.connect<"out", "in">(allocate, insert).has_value());
    boost::ut::expect(graph.connect<"out", "in">(insert, sink, gr::EdgeParameters{.minBufferSize = sampleEdge}).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return static_cast<std::size_t>(sink._nSamplesProduced);
}

/// The stream a prefix cycle and a list of symbols must produce, built without the block.
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

        // 16, 4, 16 in both frames. The cycle follows symbol_in_frame, not the record count.
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

            // Each cut holds the samples the cadence names, read from the stream itself.
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

    "a negative timing_offset rotates carrier k by 2*pi*k*offset/fft_len"_test = [] {
        constexpr std::int32_t       kOffset = -4;
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ)};

        CpInsert     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        CpRemove     removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"timing_offset", kOffset}});
        const auto   out         = remove(removeBlock, stream, 11UZ);

        expect(eq(out.size(), 1UZ));
        double worst = 0.;
        for (std::size_t bin = 0UZ; bin < kFft; ++bin) {
            const int    carrier = gr::ofdm::CarrierMap::carrierOf(kFft, bin).value();
            const double phase   = 2. * std::numbers::pi * static_cast<double>(carrier) * static_cast<double>(kOffset) / static_cast<double>(kFft);
            const CF     want    = records[0UZ].signal_values[bin] * CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
            worst                = std::max(worst, static_cast<double>(std::abs(out[0UZ].signal_values[bin] - want)));
        }
        std::println("timing_offset {} against the stated slope 2*pi*k*offset/fft_len: worst bin error {:.3e}", kOffset, worst);
        expect(lt(worst, 1e-3)) << "the bias into the prefix is a pure per-carrier rotation";
    };

    "between frames the block discards, and counts what it discarded"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        Stream                       stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        // silence before the frame and after it, outside the cadence
        Stream padded;
        padded.samples.assign(25UZ, CF{});
        padded.samples.insert(padded.samples.end(), stream.samples.begin(), stream.samples.end());
        padded.samples.insert(padded.samples.end(), 40UZ, CF{});
        padded.tags.push_back(gr::Tag{25UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("ofdm_frame")}}});

        CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}});
        const auto out   = remove(block, padded, 23UZ);

        expect(eq(out.size(), 2UZ));
        expect(eq(block.nDiscarded(), std::uint64_t{65})) << "25 samples before the trigger and 40 after the frame";
        expect(eq(gr::blocks::ofdm::detail::metaCount(out[1UZ], "discarded_samples", 0ULL), std::uint64_t{25})) << "the record states the samples discarded before it was cut";
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

    "edge windowing lowers the out-of-band power, measured"_test = [] {
        // 24 occupied carriers of 64. The guard band is wide, and the shoulder shows well away from the band edge. A
        // Hann window on the analysis keeps its own leakage below the signal's.
        constexpr std::size_t        kOccupiedHalf = 12UZ;
        constexpr std::size_t        kSymbols      = 48UZ;
        std::vector<gr::DataSet<CF>> records;
        std::vector<int>             carriers(kFft);
        for (std::size_t bin = 0UZ; bin < kFft; ++bin) {
            carriers[bin] = gr::ofdm::CarrierMap::carrierOf(kFft, bin).value();
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

            // The occupied band is |carrier| <= 12 of 64. Everything past |carrier| 20 is the shoulder. The carriers
            // between are left out, and the transition counts as neither.
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
        const auto names = [](std::string_view setting, auto&& function) {
            const std::string message = refusal(function);
            expect(message.contains(setting)) << std::format("the refusal names {}: '{}'", setting, message);
        };
        names("timing_offset", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{-17}}}); });
        names("timing_offset", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{1}}}); });
        names("cp_len", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{}}}); });
        names("cp_len", [] { std::ignore = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{80U}}}); });
        names("window_len", [] { std::ignore = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{4U}}, {"window_len", gr::Size_t{8U}}}); });
    };

    "a record shorter than its prefix is dropped and counted"_test = [] {
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ, 8UZ), symbol(2UZ, 1UZ)};
        CpInsert                           block  = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream                       stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        const std::vector<gr::DataSet<CF>> kept{records[0UZ], records[2UZ]};
        const std::vector<gr::Size_t>      cycle{16U};
        expect(std::ranges::equal(stream.samples, expectedStream(std::span<const gr::DataSet<CF>>(kept), std::span<const gr::Size_t>(cycle)))) << "the stream holds the two whole symbols";
        expect(eq(block.nMalformed(), std::uint64_t{1}));
        expect(eq(block.nUnmarked(), std::uint64_t{0}));
    };

    "a record without symbol_in_frame takes the next cycle position and is counted"_test = [] {
        const std::vector<gr::Size_t>      cycle{16U, 4U, 8U};
        const std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, std::nullopt), symbol(2UZ, 2UZ)};
        CpInsert                           block  = make<CpInsert>({{"cp_len", cycle}});
        const Stream                       stream = insert(block, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        expect(std::ranges::equal(stream.samples, expectedStream(std::span<const gr::DataSet<CF>>(records), std::span<const gr::Size_t>(cycle)))) << "the unmarked record takes prefix 4, the second entry";
        expect(eq(block.nUnmarked(), std::uint64_t{1}));
        expect(eq(block.nMalformed(), std::uint64_t{0}));
    };

    "CpInsert publishes the last record whole through the framework's end of stream, whatever room its output has"_test = [] {
        // 52 data carriers make one symbol of 64 + 16 samples. The sink's edge of 100 samples fills before the last
        // symbol has gone out at some stream lengths, and the block then publishes the rest on later calls.
        constexpr std::size_t kSymbolSamples = kFft + 16UZ;
        std::size_t           short_         = 0UZ;
        for (std::size_t nSymbols = 1UZ; nSymbols <= 40UZ; ++nSymbols) {
            const std::size_t received = runToSink(52UZ * nSymbols, 100UZ);
            if (received != nSymbols * kSymbolSamples) {
                ++short_;
                std::println("{} symbols: the sink received {} of {} samples", nSymbols, received, nSymbols * kSymbolSamples);
            }
        }
        expect(eq(short_, 0UZ)) << "every stream length reaches the sink whole";
    };

    "a trigger on the cut's first sample opens its frame once, whatever room the output has"_test = [] {
        // Two frames of one symbol each. With timing_offset at -cp_len the cut starts on the trigger's own sample.
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 0UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream                 stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        expect(eq(stream.tags.size(), 2UZ));

        for (const std::size_t outRoom : {1UZ, 64UZ}) {
            CpRemove   block = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{1U}}, {"timing_offset", std::int32_t{-16}}});
            const auto out   = remove(block, stream, 4096UZ, outRoom);
            expect(eq(out.size(), 2UZ)) << std::format("output room {}", outRoom);
            for (std::size_t r = 0UZ; r < out.size(); ++r) {
                expect(eq(gr::blocks::ofdm::detail::metaCount(out[r], "frame_index", 99ULL), std::uint64_t{r})) << std::format("output room {} record {}", outRoom, r);
            }
        }
    };

    "a CpInsert setting given its current value keeps the windowed edge"_test = [] {
        std::vector<gr::DataSet<CF>> records;
        for (std::size_t s = 0UZ; s < 4UZ; ++s) {
            records.push_back(symbol(s, s));
        }
        const gr::property_map settings{{"cp_len", std::vector<gr::Size_t>{16U}}, {"window_len", gr::Size_t{4U}}};

        CpInsert     reference = make<CpInsert>(settings);
        const Stream expected  = insert(reference, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        CpInsert block  = make<CpInsert>(settings);
        Stream   stream = insert(block, std::span<const gr::DataSet<CF>>(records).first(2UZ), 4096UZ);
        change(block, settings);
        const Stream rest = insert(block, std::span<const gr::DataSet<CF>>(records).subspan(2UZ), 4096UZ);
        stream.samples.insert(stream.samples.end(), rest.samples.begin(), rest.samples.end());
        expect(std::ranges::equal(stream.samples, expected.samples)) << "the third symbol's head still adds the second symbol's extension";
    };

    "a cut setting given its current value keeps the partial cut, and a signal_name change reaches the next record"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};
        CpInsert                     insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream                 stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records), 4096UZ);
        const gr::property_map       settings{{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}};

        CpRemove   reference = make<CpRemove>(settings);
        const auto whole     = remove(reference, stream, 4096UZ);
        expect(eq(whole.size(), 2UZ));

        const std::vector<gr::property_map> changes{{{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"timing_offset", std::int32_t{0}}}, {{"signal_name", std::string("renamed")}}};
        for (const gr::property_map& changed : changes) {
            // 100 samples hold the first symbol and the second symbol's prefix, with 4 samples of its cut
            CpRemove block = make<CpRemove>(settings);
            auto     out   = remove(block, stream, 0UZ, 100UZ, 4096UZ);
            change(block, changed);
            const auto rest = remove(block, stream, 100UZ, stream.samples.size(), 4096UZ);
            out.insert(out.end(), rest.begin(), rest.end());

            const bool renamed = changed.contains("signal_name");
            expect(eq(out.size(), 2UZ));
            expect(eq(block.nDiscarded(), std::uint64_t{0}));
            if (out.size() == 2UZ && whole.size() == 2UZ) {
                expect(std::ranges::equal(out[1UZ].signal_values, whole[1UZ].signal_values)) << "the cut in hand is complete and unchanged";
                expect(eq(signalName(out[0UZ]), std::string("ofdm_symbol")));
                expect(eq(signalName(out[1UZ]), std::string(renamed ? "renamed" : "ofdm_symbol")));
            }
        }
    };

    "a refused setting leaves the block on the settings it last accepted"_test = [] {
        std::vector<gr::DataSet<CF>> records{symbol(0UZ, 0UZ), symbol(1UZ, 1UZ)};

        CpInsert     reference = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        const Stream expected  = insert(reference, std::span<const gr::DataSet<CF>>(records), 4096UZ);

        CpInsert insertBlock = make<CpInsert>({{"cp_len", std::vector<gr::Size_t>{16U}}});
        Stream   stream      = insert(insertBlock, std::span<const gr::DataSet<CF>>(records).first(1UZ), 4096UZ);
        expect(refusal([&insertBlock] { change(insertBlock, {{"cp_len", std::vector<gr::Size_t>{}}}); }).contains("cp_len"));
        const Stream second = insert(insertBlock, std::span<const gr::DataSet<CF>>(records).subspan(1UZ), 4096UZ);
        stream.samples.insert(stream.samples.end(), second.samples.begin(), second.samples.end());
        expect(std::ranges::equal(stream.samples, expected.samples)) << "CpInsert keeps a prefix of 16";

        CpRemove removeBlock = make<CpRemove>({{"fft_len", gr::Size_t{kFft}}, {"cp_len", std::vector<gr::Size_t>{16U}}, {"frame_len", gr::Size_t{2U}}});
        expect(refusal([&removeBlock] { change(removeBlock, {{"timing_offset", std::int32_t{-20}}}); }).contains("timing_offset"));
        const auto out = remove(removeBlock, expected, 37UZ);
        expect(eq(out.size(), 2UZ)) << "CpRemove keeps a timing_offset of 0";
        if (out.size() == 2UZ) {
            const std::vector<CF> cut(expected.samples.begin() + 16, expected.samples.begin() + 16 + static_cast<std::ptrdiff_t>(kFft));
            expect(std::ranges::equal(out[0UZ].signal_values, forwardTransform(std::span<const CF>(cut))));
        }
    };

    "an fft_len change during a cut ends the frame, and the partial cut counts as discarded"_test = [] {
        // The cut of 64 holds 50 samples when fft_len becomes 32. The address sanitizer run of this case checks that no
        // write leaves the cut buffer.
        Stream input;
        input.samples.assign(214UZ, CF{1.f, 0.f});
        input.tags.push_back(gr::Tag{0UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("frame")}}});
        input.tags.push_back(gr::Tag{166UZ, gr::property_map{{gr::tag::TRIGGER_NAME.shortKey(), std::string("frame")}}});

        CpRemove block = make<CpRemove>({{"fft_len", gr::Size_t{64U}}, {"cp_len", std::vector<gr::Size_t>{16U}}});
        expect(eq(remove(block, input, 0UZ, 66UZ, 4096UZ).size(), 0UZ));
        change(block, {{"fft_len", gr::Size_t{32U}}});

        expect(eq(remove(block, input, 66UZ, 166UZ, 4096UZ).size(), 0UZ)) << "no frame is open until the next trigger";
        expect(eq(block.nDiscarded(), std::uint64_t{150})) << "the 50 samples of the partial cut and the 100 after it";

        const auto out = remove(block, input, 166UZ, input.samples.size(), 4096UZ);
        expect(eq(out.size(), 1UZ)) << "the next trigger opens a frame cut at the new length";
        if (!out.empty()) {
            expect(eq(out[0UZ].signal_values.size(), 32UZ));
            expect(eq(gr::blocks::ofdm::detail::metaCount(out[0UZ], "fft_len", 0ULL), std::uint64_t{32}));
            expect(lt(std::abs(out[0UZ].signal_values[0UZ] - CF(32.f, 0.f)), 1e-4f)) << "a cut of ones puts 32 on DC";
        }
    };
};

} // namespace qa_ofdm_cp

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
