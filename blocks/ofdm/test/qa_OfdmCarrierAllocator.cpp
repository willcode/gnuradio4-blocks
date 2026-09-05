#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <span>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_allocator {

using gr::blocks::ofdm::CarrierAllocator;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t kFft = 64U;

/// The QA numerology: 64 carriers, 52 occupied, 4 of them pilots. A test shape, explicitly not an
/// interoperability claim, and the one the whole module's QA is measured on.
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

/// A stream whose every sample is distinguishable, so a misplaced carrier names itself.
[[nodiscard]] std::vector<CF> ramp(std::size_t count) {
    std::vector<CF> data(count);
    for (std::size_t k = 0UZ; k < count; ++k) {
        data[k] = CF(static_cast<float>(k + 1UZ), -static_cast<float>(k + 1UZ));
    }
    return data;
}

/**
 * @brief Drive the block over @p input in chunks, then run the end-of-stream epilogue over what it held back.
 *
 * The block leaves the last sample of every call unconsumed so the framework's epilogue has a span to run on, so the
 * driver re-presents what a call did not take rather than advancing by the chunk size.
 */
[[nodiscard]] std::vector<gr::DataSet<CF>> drive(CarrierAllocator& block, std::span<const CF> input, std::size_t chunk, std::size_t outRoom = 128UZ) {
    std::vector<gr::DataSet<CF>> records;
    std::vector<gr::DataSet<CF>> scratch(outRoom);

    std::size_t consumed = 0UZ;
    std::size_t fed      = 0UZ;
    while (consumed < input.size()) {
        fed = std::min(input.size(), fed + chunk);
        shim::InputSpan<CF>               inSpan(input.subspan(consumed, fed - consumed), consumed);
        shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
        std::ignore = block.processBulk(inSpan, outSpan);
        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
        consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && fed == input.size()) {
            break; // only the sample the block holds back for its epilogue is left
        }
    }

    shim::InputSpan<CF>               tail(input.subspan(consumed), consumed);
    shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
    std::ignore = block.processEpilogue(tail, outSpan);
    for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
        records.push_back(std::move(scratch[k]));
    }
    return records;
}

[[nodiscard]] CarrierAllocator make(gr::property_map settings) {
    CarrierAllocator block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

[[nodiscard]] std::uint64_t meta(const gr::DataSet<CF>& record, std::string_view key) {
    const auto entry = record.meta_information[0UZ].find(std::pmr::string(key));
    if (entry == record.meta_information[0UZ].end()) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    const auto* value = entry->second.get_if<std::uint64_t>();
    return value == nullptr ? std::numeric_limits<std::uint64_t>::max() : *value;
}

[[nodiscard]] std::string kindOf(const gr::DataSet<CF>& record) {
    const auto  entry = record.meta_information[0UZ].find(std::pmr::string("symbol_kind"));
    const auto* value = entry->second.get_if<std::pmr::string>();
    return value == nullptr ? std::string{} : std::string(value->begin(), value->end());
}

const boost::ut::suite<"OFDM carrier allocator"> _allocator = [] {
    using namespace boost::ut;

    "a symbol places data, pilots and guards where the signed map says"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> pilotValues{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f), CF(0.f, -1.f)};

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(pilotValues)}});

        const std::vector<CF> input   = ramp(data.size());
        const auto            records = drive(block, std::span<const CF>(input), 7UZ);

        expect(eq(records.size(), 1UZ)) << "48 data carriers in, one symbol out";
        const auto& values = records[0UZ].signal_values;
        expect(eq(values.size(), static_cast<std::size_t>(kFft)));

        for (std::size_t k = 0UZ; k < data.size(); ++k) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[k]);
            expect(values[bin] == input[k]) << std::format("data carrier {} landed in bin {}", data[k], bin);
        }
        for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]);
            expect(values[bin] == pilotValues[p]) << std::format("pilot carrier {} landed in bin {}", pilots[p], bin);
        }

        expect(values[0UZ] == CF{}) << "DC is a guard in this numerology and holds nothing";
        expect(values[32UZ] == CF{}) << "bin 32 is carrier -32, the Nyquist carrier, and is guarded here";
        expect(values[27UZ] == CF{}) << "carrier +27 is outside the occupied band";
        expect(values[37UZ] == CF{}) << "carrier -27 is outside the occupied band";

        // the record's own axis states the mapping, so a consumer never re-derives it
        expect(records[0UZ].axis_values[0UZ][0UZ] == CF(0.f, 0.f));
        expect(records[0UZ].axis_values[0UZ][1UZ] == CF(1.f, 0.f));
        expect(records[0UZ].axis_values[0UZ][32UZ] == CF(-32.f, 0.f));
        expect(records[0UZ].axis_values[0UZ][63UZ] == CF(-1.f, 0.f));
    };

    "+fft_len/2 is refused, not folded onto the Nyquist bin"_test = [] {
        std::vector<std::int32_t> data = dataCarriers();
        data.push_back(32);
        expect(throws([&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}}); }));
    };

    "the pilot cycle runs at two levels"_test = [] {
        const auto      data   = dataCarriers();
        const auto      pilots = pilotCarriers();
        std::vector<CF> cycle(6UZ); // not a multiple of n_pilots, so the pilots rotate from symbol to symbol
        for (std::size_t k = 0UZ; k < cycle.size(); ++k) {
            cycle[k] = CF(static_cast<float>(k), 0.f);
        }

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(cycle)}});

        const std::vector<CF> input   = ramp(3UZ * data.size());
        const auto            records = drive(block, std::span<const CF>(input), 100UZ);
        expect(eq(records.size(), 3UZ));

        for (std::size_t s = 0UZ; s < 3UZ; ++s) {
            for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
                const std::size_t bin      = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]);
                const std::size_t expected = (s * pilots.size() + p) % cycle.size();
                expect(records[s].signal_values[bin] == cycle[expected]) << std::format("symbol {} pilot slot {}", s, p);
            }
        }
    };

    "sync words open a frame verbatim and the cadence repeats"_test = [] {
        const auto      data = dataCarriers();
        std::vector<CF> word(kFft);
        for (std::size_t k = 0UZ; k < word.size(); ++k) {
            word[k] = CF(static_cast<float>(k), 0.5f);
        }

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}});

        const std::vector<CF> input   = ramp(4UZ * data.size());
        const auto            records = drive(block, std::span<const CF>(input), 13UZ);

        expect(eq(records.size(), 6UZ)) << "two frames of one sync word and two data symbols";
        for (const std::size_t at : {0UZ, 3UZ}) {
            expect(eq(kindOf(records[at]), std::string("sync")));
            expect(std::ranges::equal(records[at].signal_values, word)) << "the sync word is emitted verbatim";
            expect(eq(meta(records[at], "symbol_in_frame"), std::uint64_t{0}));
        }
        expect(eq(meta(records[1UZ], "symbol_in_frame"), std::uint64_t{1}));
        expect(eq(meta(records[2UZ], "symbol_in_frame"), std::uint64_t{2}));
        expect(eq(meta(records[3UZ], "frame_index"), std::uint64_t{1}));
        expect(eq(meta(records[5UZ], "symbol_index"), std::uint64_t{5}));
    };

    "the same stream chunked differently gives the same records"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> pilotValues{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f)};
        std::vector<CF>       word(kFft, CF(0.25f, -0.25f));

        const gr::property_map settings{{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(pilotValues)}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}};

        const std::vector<CF>        input = ramp(7UZ * data.size() + 5UZ); // ends mid-symbol and mid-frame
        std::vector<gr::DataSet<CF>> reference;
        for (const std::size_t chunk : {1UZ, 7UZ, 48UZ, 4096UZ}) {
            CarrierAllocator block   = make(settings);
            auto             records = drive(block, std::span<const CF>(input), chunk);
            if (reference.empty()) {
                reference = std::move(records);
                expect(gt(reference.size(), 0UZ));
                continue;
            }
            expect(eq(records.size(), reference.size())) << std::format("chunk {}", chunk);
            for (std::size_t r = 0UZ; r < records.size(); ++r) {
                expect(std::ranges::equal(records[r].signal_values, reference[r].signal_values)) << std::format("chunk {} record {}", chunk, r);
                expect(eq(meta(records[r], "symbol_index"), meta(reference[r], "symbol_index")));
                expect(eq(meta(records[r], "pad_carriers"), meta(reference[r], "pad_carriers")));
            }
        }
    };

    "end of stream pads the open symbol and the open frame, counted and stated"_test = [] {
        const auto      data = dataCarriers();
        std::vector<CF> word(kFft, CF(1.f, 0.f));

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});

        const std::size_t     partial = 5UZ;
        const std::vector<CF> input   = ramp(data.size() + partial); // one whole symbol and a fragment
        const auto            records = drive(block, std::span<const CF>(input), 9UZ);

        expect(eq(records.size(), 4UZ)) << "the sync word plus the frame's three data symbols";
        expect(eq(meta(records[1UZ], "pad_carriers"), std::uint64_t{0}));
        expect(eq(meta(records[2UZ], "pad_carriers"), data.size() - partial));
        expect(eq(meta(records[3UZ], "pad_carriers"), data.size()));
        expect(eq(block.nPadded(), 2UZ * data.size() - partial));

        // what was invented is zero, and what arrived is untouched
        const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[partial]);
        expect(records[2UZ].signal_values[bin] == CF{});
        const std::size_t kept = gr::ofdm::CarrierMap::binOf(kFft, data[partial - 1UZ]);
        expect(records[2UZ].signal_values[kept] == input[data.size() + partial - 1UZ]);
    };

    "an unframed stream pads only the open symbol"_test = [] {
        const auto            data    = dataCarriers();
        CarrierAllocator      block   = make({{"fft_len", kFft}, {"data_carriers", data}});
        const std::vector<CF> input   = ramp(data.size() + 2UZ);
        const auto            records = drive(block, std::span<const CF>(input), 5UZ);

        expect(eq(records.size(), 2UZ));
        expect(eq(meta(records[1UZ], "pad_carriers"), data.size() - 2UZ));
        expect(eq(block.nPadded(), data.size() - 2UZ));
    };

    "a numerology or a frame the block cannot serve is refused by name"_test = [] {
        const auto data = dataCarriers();

        expect(throws([&data] { std::ignore = make({{"fft_len", gr::Size_t{63U}}, {"data_carriers", data}}); })) << "an odd transform";
        expect(throws([] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", std::vector<std::int32_t>{}}}); })) << "no data carriers";
        expect(throws([&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", std::vector<std::int32_t>{1}}}); })) << "a pilot on a data carrier";
        expect(throws([&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilotCarriers()}}); })) << "pilots with no pilot_symbols";
        expect(throws([&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", std::vector<float>(2UZ * kFft - 4UZ, 0.f)}, {"frame_len", gr::Size_t{2U}}}); })) << "a sync word that is not a whole symbol";
        expect(throws([&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", std::vector<float>(2UZ * kFft, 0.f)}}); })) << "sync words on an unframed stream";
    };
};

} // namespace qa_ofdm_allocator

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
