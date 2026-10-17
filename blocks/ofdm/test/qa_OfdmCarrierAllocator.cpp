#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/ut.hpp>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace qa_ofdm_allocator {

using gr::blocks::ofdm::CarrierAllocator;
using CF       = std::complex<float>;
namespace shim = gr::blocks::testing::span;

constexpr gr::Size_t kFft = 64U;

/// The test numerology: 64 carriers, 52 of them occupied, 4 of those pilots.
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

/// A stream of distinct samples. A misplaced carrier shows which sample it holds.
[[nodiscard]] std::vector<CF> ramp(std::size_t count) {
    std::vector<CF> data(count);
    for (std::size_t k = 0UZ; k < count; ++k) {
        data[k] = CF(static_cast<float>(k + 1UZ), -static_cast<float>(k + 1UZ));
    }
    return data;
}

/// @brief Drives the block through span shims and keeps the stream index across calls.
struct Driver {
    CarrierAllocator&            block;
    std::size_t                  outRoom = 128UZ;
    std::size_t                  at      = 0UZ; ///< stream index of the next sample
    std::vector<gr::DataSet<CF>> records{};

    /// Feeds @p input in chunks. Each call starts at the first sample the previous call did not consume. Returns the
    /// samples the block consumed.
    std::size_t feed(std::span<const CF> input, std::size_t chunk) {
        std::vector<gr::DataSet<CF>> scratch(outRoom);
        std::size_t                  consumed = 0UZ;
        std::size_t                  fed      = 0UZ;
        while (consumed < input.size()) {
            fed = std::min(input.size(), fed + chunk);
            shim::InputSpan<CF>               inSpan(input.subspan(consumed, fed - consumed), at + consumed);
            shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), outRoom));
            std::ignore = block.processBulk(inSpan, outSpan);
            collect(scratch, outSpan.count);
            consumed += inSpan.consumed;
            if (inSpan.consumed == 0UZ && fed == input.size()) {
                break; // the block takes nothing more while its output is full
            }
        }
        at += consumed;
        return consumed;
    }

    /// Runs the end-of-stream epilogue over @p tail, with room for @p room records.
    void end(std::span<const CF> tail, std::size_t room) {
        std::vector<gr::DataSet<CF>>      scratch(room);
        shim::InputSpan<CF>               inSpan(tail, at);
        shim::OutputSpan<gr::DataSet<CF>> outSpan(std::span<gr::DataSet<CF>>(scratch.data(), room));
        std::ignore = block.processEpilogue(inSpan, outSpan);
        collect(scratch, outSpan.count);
    }

    void collect(std::vector<gr::DataSet<CF>>& scratch, std::size_t count) {
        for (std::size_t k = 0UZ; k < count; ++k) {
            records.push_back(std::move(scratch[k]));
        }
    }
};

/// @brief Drives the block over @p input in chunks, then runs the end-of-stream epilogue over the samples it left.
[[nodiscard]] std::vector<gr::DataSet<CF>> drive(CarrierAllocator& block, std::span<const CF> input, std::size_t chunk, std::size_t outRoom = 128UZ) {
    Driver            driver{block, outRoom};
    const std::size_t consumed = driver.feed(input, chunk);
    driver.end(input.subspan(consumed), outRoom);
    return std::move(driver.records);
}

[[nodiscard]] CarrierAllocator make(gr::property_map settings) {
    CarrierAllocator block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
    return block;
}

/// @brief Applies @p settings to a running block, as a settings change between two calls does.
void change(CarrierAllocator& block, gr::property_map settings) {
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

/// @brief Runs a source of @p nInput samples into the block and a record counter under the framework's scheduler. A
/// @p recordEdge of 0 keeps the default edge. Returns the records the counter received and the block's `nTruncated()`.
[[nodiscard]] std::pair<std::size_t, std::uint64_t> runToCounter(std::size_t nInput, gr::property_map settings, std::size_t recordEdge) {
    using gr::blocks::testing::ProcessFunction;
    gr::Graph graph;
    auto&     source   = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(nInput)}, {"mark_tag", false}});
    auto&     allocate = graph.emplaceBlock<CarrierAllocator>(std::move(settings));
    auto&     counter  = graph.emplaceBlock<gr::blocks::testing::CountingSink<gr::DataSet<CF>>>({});
    boost::ut::expect(graph.connect<"out", "in">(source, allocate).has_value());
    boost::ut::expect((recordEdge > 0UZ ? graph.connect<"out", "in">(allocate, counter, gr::EdgeParameters{.minBufferSize = recordEdge}) : graph.connect<"out", "in">(allocate, counter)).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    boost::ut::expect(scheduler.runAndWait().has_value());
    return {static_cast<std::size_t>(counter.count.value), allocate.nTruncated()};
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

    "a symbol holds data, pilots and guards in the bins of their signed carriers"_test = [] {
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
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[k]).value();
            expect(values[bin] == input[k]) << std::format("data carrier {} landed in bin {}", data[k], bin);
        }
        for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
            expect(values[bin] == pilotValues[p]) << std::format("pilot carrier {} landed in bin {}", pilots[p], bin);
        }

        expect(values[0UZ] == CF{}) << "DC is a guard in this numerology and holds nothing";
        expect(values[32UZ] == CF{}) << "bin 32 is carrier -32, the Nyquist carrier, and is guarded here";
        expect(values[27UZ] == CF{}) << "carrier +27 is outside the occupied band";
        expect(values[37UZ] == CF{}) << "carrier -27 is outside the occupied band";

        // the record's axis states the carrier of each bin
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
        std::vector<CF> cycle(6UZ); // a length that is not a multiple of n_pilots rotates the pilots
        for (std::size_t k = 0UZ; k < cycle.size(); ++k) {
            cycle[k] = CF(static_cast<float>(k), 0.f);
        }

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(cycle)}});

        const std::vector<CF> input   = ramp(3UZ * data.size());
        const auto            records = drive(block, std::span<const CF>(input), 100UZ);
        expect(eq(records.size(), 3UZ));

        for (std::size_t s = 0UZ; s < 3UZ; ++s) {
            for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
                const std::size_t bin      = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
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

        // the padded carriers are zero, and the carriers that arrived are untouched
        const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[partial]).value();
        expect(records[2UZ].signal_values[bin] == CF{});
        const std::size_t kept = gr::ofdm::CarrierMap::binOf(kFft, data[partial - 1UZ]).value();
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
        const auto data  = dataCarriers();
        const auto names = [](std::string_view setting, auto&& function) {
            const std::string message = refusal(function);
            expect(message.contains(setting)) << std::format("the refusal names {}: '{}'", setting, message);
        };

        names("fft_len", [&data] { std::ignore = make({{"fft_len", gr::Size_t{63U}}, {"data_carriers", data}}); });
        names("data_carriers", [] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", std::vector<std::int32_t>{}}}); });
        names("pilot_carriers", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", std::vector<std::int32_t>{1}}}); });
        names("pilot_symbols", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilotCarriers()}}); });
        names("sync_words", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", std::vector<float>(2UZ * kFft - 4UZ, 0.f)}, {"frame_len", gr::Size_t{2U}}}); });
        names("sync_words", [&data] { std::ignore = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", std::vector<float>(2UZ * kFft, 0.f)}}); });
    };

    "processBulk takes the whole span, and the epilogue alone pads"_test = [] {
        const auto            data  = dataCarriers();
        CarrierAllocator      block = make({{"fft_len", kFft}, {"data_carriers", data}});
        const std::vector<CF> input = ramp(data.size() + 2UZ);

        Driver driver{block};
        expect(eq(driver.feed(std::span<const CF>(input), 4096UZ), input.size())) << "no sample is held back";
        expect(eq(driver.records.size(), 1UZ));
        driver.end({}, 128UZ);
        expect(eq(driver.records.size(), 2UZ)) << "an epilogue over an empty span emits the open symbol";
        expect(eq(meta(driver.records[1UZ], "pad_carriers"), data.size() - 2UZ));
    };

    "an end-of-stream output with too little room ends the frame short and counts what it lacks"_test = [] {
        const auto      data = dataCarriers();
        std::vector<CF> word(kFft, CF(1.f, 0.f));

        CarrierAllocator      block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});
        const std::vector<CF> input = ramp(data.size() + 5UZ);

        Driver driver{block};
        std::ignore = driver.feed(std::span<const CF>(input), 4096UZ);
        expect(eq(driver.records.size(), 2UZ)) << "the sync word and the first data symbol";
        driver.end({}, 1UZ);
        expect(eq(driver.records.size(), 3UZ)) << "room for the open symbol alone";
        expect(eq(block.nTruncated(), std::uint64_t{1})) << "the frame lacks its third data symbol";
        expect(eq(block.nPadded(), data.size() - 5UZ)) << "only emitted records count as padded";
    };

    "the end of stream through the framework completes the frame into the room the output has, and counts what it lacks"_test = [] {
        // One sync word and 200 data symbols make a frame of 201 records. The stream carries one symbol and 5 samples.
        const auto              data = dataCarriers();
        const std::vector<CF>   word(kFft, CF(1.f, 0.f));
        const gr::property_map  settings{{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{200U}}};
        constexpr std::uint64_t kFrame = 201ULL;

        const auto [records, truncated] = runToCounter(data.size() + 5UZ, settings, 0UZ);
        std::println("default record edge: {} records out, {} counted as lacking", records, truncated);
        expect(gt(truncated, 0ULL)) << "the default record edge holds fewer records than the frame";
        expect(eq(static_cast<std::uint64_t>(records) + truncated, kFrame)) << "the records out and the records lacking make the frame";

        const auto [wideRecords, wideTruncated] = runToCounter(data.size() + 5UZ, settings, 1024UZ);
        expect(eq(static_cast<std::uint64_t>(wideRecords), kFrame)) << "a record edge of 1024 takes the whole frame";
        expect(eq(wideTruncated, 0ULL));
    };

    "a setting given its current value keeps the pending samples, the open frame and the indices"_test = [] {
        const auto             data   = dataCarriers();
        const auto             pilots = pilotCarriers();
        const std::vector<CF>  pilotValues{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f)};
        const std::vector<CF>  word(kFft, CF(0.25f, -0.25f));
        const gr::property_map settings{{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(pilotValues)}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}};
        const std::vector<CF>  input = ramp(5UZ * data.size());

        CarrierAllocator reference = make(settings);
        const auto       expected  = drive(reference, std::span<const CF>(input), 4096UZ);

        CarrierAllocator  block = make(settings);
        Driver            driver{block};
        const std::size_t half = data.size() + data.size() / 2UZ;
        expect(eq(driver.feed(std::span<const CF>(input).first(half), 4096UZ), half));
        change(block, settings);
        expect(eq(driver.feed(std::span<const CF>(input).subspan(half), 4096UZ), input.size() - half));
        driver.end({}, 128UZ);

        expect(eq(driver.records.size(), expected.size()));
        for (std::size_t r = 0UZ; r < std::min(driver.records.size(), expected.size()); ++r) {
            expect(std::ranges::equal(driver.records[r].signal_values, expected[r].signal_values)) << std::format("record {}", r);
            for (const std::string_view key : {"symbol_index", "frame_index", "symbol_in_frame", "sample_start", "pad_carriers"}) {
                expect(eq(meta(driver.records[r], key), meta(expected[r], key))) << std::format("record {} {}", r, key);
            }
        }
    };

    "pilot_symbols and signal_name changes apply from the next record and keep the frame"_test = [] {
        const auto            data   = dataCarriers();
        const auto            pilots = pilotCarriers();
        const std::vector<CF> first{CF(1.f, 0.f), CF(0.f, 1.f), CF(-1.f, 0.f), CF(0.f, -1.f)};
        const std::vector<CF> second{CF(2.f, 0.f), CF(0.f, 2.f), CF(-2.f, 0.f), CF(0.f, -2.f)};
        const std::vector<CF> word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF> input = ramp(3UZ * data.size());

        CarrierAllocator  block = make({{"fft_len", kFft}, {"data_carriers", data}, {"pilot_carriers", pilots}, {"pilot_symbols", interleave(first)}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});
        Driver            driver{block};
        const std::size_t half = data.size() + data.size() / 2UZ;
        std::ignore            = driver.feed(std::span<const CF>(input).first(half), 4096UZ);
        expect(eq(driver.records.size(), 2UZ)) << "the sync word and the first data symbol";
        change(block, {{"pilot_symbols", interleave(second)}, {"signal_name", std::string("renamed")}});
        std::ignore = driver.feed(std::span<const CF>(input).subspan(half), 4096UZ);

        expect(eq(driver.records.size(), 4UZ)) << "one frame of a sync word and three data symbols";
        expect(eq(block.nDropped(), std::uint64_t{0}));
        for (std::size_t r = 0UZ; r < std::min(driver.records.size(), 4UZ); ++r) {
            const auto& record = driver.records[r];
            expect(eq(meta(record, "symbol_index"), std::uint64_t{r}));
            expect(eq(meta(record, "frame_index"), std::uint64_t{0}));
            expect(eq(meta(record, "symbol_in_frame"), std::uint64_t{r}));
            expect(eq(signalName(record), std::string(r < 2UZ ? "ofdm_symbol" : "renamed"))) << std::format("record {}", r);
            if (r == 0UZ) {
                continue;
            }
            const std::vector<CF>& cycle = r < 2UZ ? first : second;
            for (std::size_t p = 0UZ; p < pilots.size(); ++p) {
                const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, pilots[p]).value();
                expect(record.signal_values[bin] == cycle[p]) << std::format("record {} pilot slot {}", r, p);
            }
            for (std::size_t k = 0UZ; k < data.size(); ++k) {
                const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, data[k]).value();
                expect(record.signal_values[bin] == input[(r - 1UZ) * data.size() + k]) << std::format("record {} data carrier {}", r, data[k]);
            }
        }
    };

    "a new numerology ends the open frame, keeps the samples that fit and counts the samples dropped"_test = [] {
        const auto            data = dataCarriers();
        const std::vector<CF> word(kFft, CF(0.5f, 0.5f));
        const std::size_t     half  = data.size() + data.size() / 2UZ; // one symbol and 24 samples
        const std::vector<CF> input = ramp(half + 32UZ);

        for (const std::size_t nNew : {16UZ, 32UZ}) {
            std::vector<std::int32_t> carriers; // +1, -1, +2, -2, ...
            for (std::size_t k = 0UZ; k < nNew; ++k) {
                const auto magnitude = static_cast<std::int32_t>(k / 2UZ + 1UZ);
                carriers.push_back(k % 2UZ == 0UZ ? magnitude : -magnitude);
            }
            CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{3U}}});
            Driver           driver{block};
            std::ignore = driver.feed(std::span<const CF>(input).first(half), 4096UZ);
            expect(eq(driver.records.size(), 2UZ));
            change(block, {{"data_carriers", carriers}});
            std::ignore = driver.feed(std::span<const CF>(input).subspan(half), 4096UZ);

            const bool        dropped = nNew < 24UZ;
            const std::size_t first   = dropped ? half : data.size(); // the input index of the new frame's first data sample
            expect(eq(block.nDropped(), dropped ? std::uint64_t{24} : std::uint64_t{0})) << std::format("{} new data carriers", nNew);
            expect(ge(driver.records.size(), 4UZ)) << "a new frame of a sync word and a data symbol";
            if (driver.records.size() < 4UZ) {
                continue;
            }
            const auto& sync = driver.records[2UZ];
            const auto& next = driver.records[3UZ];
            expect(eq(kindOf(sync), std::string("sync")));
            expect(eq(meta(sync, "frame_index"), std::uint64_t{1})) << "the open frame ended";
            expect(eq(meta(sync, "symbol_in_frame"), std::uint64_t{0}));
            expect(eq(meta(sync, "symbol_index"), std::uint64_t{2})) << "the indices continue";
            expect(eq(meta(next, "symbol_index"), std::uint64_t{3}));
            expect(eq(meta(next, "sample_start"), std::uint64_t{first}));
            const std::size_t bin = gr::ofdm::CarrierMap::binOf(kFft, carriers[0UZ]).value();
            expect(next.signal_values[bin] == input[first]) << std::format("{} new data carriers", nNew);
        }
    };

    "a refused setting leaves the block on the settings it last accepted"_test = [] {
        const auto            data = dataCarriers();
        const std::vector<CF> word(kFft, CF(0.5f, 0.5f));
        const std::vector<CF> input = ramp(4UZ * data.size());

        CarrierAllocator block = make({{"fft_len", kFft}, {"data_carriers", data}, {"sync_words", interleave(word)}, {"frame_len", gr::Size_t{2U}}});
        expect(refusal([&block] { change(block, {{"frame_len", gr::Size_t{0U}}}); }).contains("sync_words")) << "an unframed stream with sync words";
        const auto records = drive(block, std::span<const CF>(input), 4096UZ);
        expect(eq(records.size(), 6UZ)) << "two frames of a sync word and two data symbols";
        for (const std::size_t at : {0UZ, 3UZ}) {
            if (at < records.size()) {
                expect(eq(kindOf(records[at]), std::string("sync")));
            }
        }
    };
};

} // namespace qa_ofdm_allocator

int main() { return boost::ut::cfg<boost::ut::override>.run(); }
