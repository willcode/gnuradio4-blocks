#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <print>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/fec/LdpcBlocks.hpp>
#include <gnuradio-4.0/fec/PolarBlocks.hpp>

/*
 * The two wrapped families are exercised through the scheduler, because what this module owns is a
 * record contract and a wall, not a decoder. AFF3CT's own correctness is its business and its test
 * suite's; what is pinned here is the layer between it and this tree.
 *
 * The sign is the first of those and the one that could fail silently. This tree's soft convention
 * is that a positive value carries a one; AFF3CT's is the opposite, and the wall negates. A wrap
 * that got that backwards would decode every frame to something wrong in a way no round trip
 * through the same wrap would show, so the anchor below decodes one known word under the bridge and
 * again under a deliberately inverted one, and asserts that the second does not recover it.
 */
namespace {

using gr::blocks::fec::LdpcDecode;
using gr::blocks::fec::LdpcEncode;
using gr::blocks::fec::PolarDecode;
using gr::blocks::fec::PolarEncode;

using Bits = gr::DataSet<std::uint8_t>;
using Soft = gr::DataSet<float>;

std::uint64_t rng = 0x9E3779B97F4A7C15ULL;

[[nodiscard]] std::uint64_t next() {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng >> 17U;
}

[[nodiscard]] std::vector<std::uint8_t> randomBits(std::size_t count) {
    std::vector<std::uint8_t> bits(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        bits[i] = static_cast<std::uint8_t>(next() & 1ULL);
    }
    return bits;
}

//! A record of @p values, shaped as the record producers in this tree shape one.
template<typename T>
[[nodiscard]] gr::DataSet<T> record(std::vector<T> values, gr::property_map meta = {}) {
    gr::DataSet<T> r;
    r.signal_values = std::move(values);
    r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
    r.signal_names.emplace_back("fec");
    r.timing_events.resize(1UZ);
    if (!meta.empty()) {
        r.meta_information.push_back(std::move(meta));
    }
    return r;
}

/*!
 * @brief Coded bits as soft values in this tree's sense: a one becomes @p magnitude and a zero
 * becomes its negative.
 *
 * The magnitude is confidence and no decoder here is scale sensitive in a way that changes a
 * decision, so the same vector at magnitude 1 is the hard entry point and at magnitude 8 the
 * strong-LLR one; both are exercised.
 */
[[nodiscard]] std::vector<float> asSoft(const std::vector<std::uint8_t>& coded, float magnitude) {
    std::vector<float> values(coded.size());
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        values[i] = ((coded[i] & 1U) != 0U) ? magnitude : -magnitude;
    }
    return values;
}

//! Items of @p a and @p b that differ.
[[nodiscard]] std::size_t differences(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
    std::size_t count = 0UZ;
    for (std::size_t i = 0UZ; i < std::min(a.size(), b.size()); ++i) {
        count += (a[i] != b[i]) ? 1UZ : 0UZ;
    }
    return count + (a.size() > b.size() ? a.size() - b.size() : b.size() - a.size());
}

//! A record's count under @p key, with the key's absence reported rather than defaulted away.
[[nodiscard]] gr::Size_t metaCount(const Bits& r, const char* key) {
    boost::ut::expect(!r.meta_information.empty()) << "the record carries a metadata map";
    if (r.meta_information.empty()) {
        return 0U;
    }
    const auto& map   = r.meta_information[0UZ];
    const auto  entry = map.find(gr::property_map::key_type(key));
    boost::ut::expect(entry != map.end()) << key;
    return entry == map.end() ? 0U : entry->second.value_or(gr::Size_t{0U});
}

template<typename T>
struct RecordSource : gr::Block<RecordSource<T>> {
    gr::PortOut<gr::DataSet<T>, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<gr::DataSet<T>>    _records;
    std::size_t                    _pos{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Bits, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Bits>              _records;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

//! Run @p flow to completion under the simple scheduler, then call @p after while its blocks are
//! still alive.
template<typename After>
void runFlow(gr::Graph&& flow, After&& after) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(60)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!done.load()) {
        scheduler.requestStop();
    }
    runner.join();
    boost::ut::expect(done.load());
    after();
}

template<typename TBlock, typename TIn, typename Inspect>
[[nodiscard]] std::vector<Bits> runBlock(gr::property_map settings, std::vector<gr::DataSet<TIn>> records, Inspect&& inspect) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource<TIn>>();
    src._records  = std::move(records);
    auto& coder   = flow.emplaceBlock<TBlock>(std::move(settings));
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, coder).has_value());
    boost::ut::expect(flow.connect<"out", "in">(coder, sink).has_value());

    std::vector<Bits> published;
    runFlow(std::move(flow), [&] {
        inspect(coder);
        published = std::move(sink._records);
    });
    return published;
}

template<typename TBlock, typename TIn>
[[nodiscard]] std::vector<Bits> runBlock(gr::property_map settings, std::vector<gr::DataSet<TIn>> records) {
    return runBlock<TBlock, TIn>(std::move(settings), std::move(records), [](const TBlock&) {});
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

//! The one payload record of @p payload, encoded under @p settings.
template<typename TEncode>
[[nodiscard]] std::vector<std::uint8_t> encoded(gr::property_map settings, const std::vector<std::uint8_t>& payload) {
    const std::vector<Bits> out = runBlock<TEncode, std::uint8_t>(std::move(settings), {record<std::uint8_t>(payload)});
    boost::ut::expect(boost::ut::eq(out.size(), 1UZ));
    return out.empty() ? std::vector<std::uint8_t>{} : out[0UZ].signal_values;
}

//! The v1 LDPC constructions, and what each carries.
struct LdpcShape {
    const char* standard;
    std::size_t k;
    std::size_t n;
};
constexpr std::array<LdpcShape, 2UZ> kLdpcShapes{{{"ccsds_128_64", 64UZ, 128UZ}, {"wimax_576_288", 288UZ, 576UZ}}};

[[nodiscard]] gr::property_map ldpcEncode(const char* standard) { return {{"standard", std::string(standard)}}; }

[[nodiscard]] gr::property_map ldpcDecode(const char* standard, const char* decoder, gr::Size_t iterations = 50U) { return {{"standard", std::string(standard)}, {"decoder", std::string(decoder)}, {"n_iterations", iterations}}; }

//! The v1 Polar constructions: the 5G sequence at the tier's stated shape, and a small aided one.
[[nodiscard]] gr::property_map polarEncode5g() { return {{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}}; }
[[nodiscard]] gr::property_map polarDecode5g() { return {{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}, {"decoder", std::string("sc")}}; }

//! CRC-16/CCITT-FALSE in this tree's own vocabulary, which is what the aided list decoder checks with.
[[nodiscard]] gr::property_map crcSettings() {
    return {{"crc_width", gr::Size_t{16U}}, {"crc_poly", std::uint64_t{0x1021ULL}}, {"crc_initial_value", std::uint64_t{0xFFFFULL}}, //
        {"crc_final_xor", std::uint64_t{0ULL}}, {"crc_input_reflected", false}, {"crc_result_reflected", false}};
}

[[nodiscard]] gr::property_map merged(gr::property_map base, const gr::property_map& more) {
    for (const auto& [key, value] : more) {
        base[key] = value;
    }
    return base;
}

[[nodiscard]] gr::property_map polarEncodeAided() { return merged({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"frozen_construction", std::string("ga")}, {"design_snr_db", 2.5}}, crcSettings()); }

[[nodiscard]] gr::property_map polarDecodeAided(gr::Size_t list = 4U) {
    return merged({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"frozen_construction", std::string("ga")}, {"design_snr_db", 2.5}, //
                      {"decoder", std::string("ca_scl")}, {"list_size", list}},
        crcSettings());
}

} // namespace

int main() {
    using namespace boost::ut;

    /*
     * Criterion 6, first part: the wall is real. No installed header of this module names AFF3CT,
     * which is what makes an AFF3CT version bump a change to one translation unit rather than to
     * every consumer that ever included one of these blocks.
     */
    "no installed header of this module includes anything of AFF3CT's"_test = [] {
        for (const std::string name : {"Aff3ctWall.hpp", "LdpcBlocks.hpp", "PolarBlocks.hpp", "FecBlocks.hpp", "ConvBlocks.hpp", "InterleaveBlocks.hpp", "PunctureBlocks.hpp", "RsBlocks.hpp"}) {
            const std::string path = std::string(GR4_FEC_INCLUDE_DIR) + "/gnuradio-4.0/fec/" + name;
            std::ifstream     header(path);
            expect(header.is_open()) << path;
            std::string line;
            std::size_t at = 0UZ;
            while (std::getline(header, line)) {
                ++at;
                const bool include = line.find("#include") != std::string::npos;
                const bool foreign = line.find("aff3ct") != std::string::npos || line.find("AFF3CT") != std::string::npos || line.find("streampu") != std::string::npos || line.find("mipp") != std::string::npos;
                // The wall's own file name and prose say AFF3CT often; only an include line matters.
                expect(!(include && foreign)) << name << "line" << at << line;
            }
        }
    };

    // Criterion 6, second part: a configuration the wall cannot honor raises the graph's own
    // exception type, naming the family, rather than letting a foreign one out.
    "a broken configuration is refused by the family that could not honor it"_test = [] {
        const auto says = [](const auto& call, std::string_view family) {
            try {
                call();
            } catch (const gr::exception& refusal) {
                const std::string what(refusal.what());
                expect(what.find(family) != std::string::npos) << family << what;
                return;
            } catch (...) {
                expect(false) << "a foreign exception crossed the wall";
                return;
            }
            expect(false) << "the configuration should have been refused";
        };

        says([] { std::ignore = make<LdpcEncode>({{"standard", std::string("")}}); }, "LDPC");
        says([] { std::ignore = make<LdpcEncode>({{"alist_path", std::string("/nonexistent/matrix.alist")}}); }, "LDPC");
        says([] { std::ignore = make<LdpcDecode>({{"standard", std::string("no_such_code")}}); }, "LDPC");
        says([] { std::ignore = make<LdpcDecode>(ldpcDecode("ccsds_128_64", "gallager_z")); }, "LDPC");
        says([] { std::ignore = make<PolarEncode>({{"n", gr::Size_t{1000U}}, {"k", gr::Size_t{500U}}}); }, "Polar");
        says([] { std::ignore = make<PolarDecode>({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{256U}}}); }, "Polar");
        says([] { std::ignore = make<PolarDecode>({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"decoder", std::string("ca_scl")}}); }, "Polar");
        // The tree's CRC kernel reads whole bytes, so an aided payload that is not a whole number of
        // them is refused at configure rather than silently truncated.
        says([] { std::ignore = make<PolarEncode>(merged({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{125U}}}, crcSettings())); }, "Polar");
        // A construction that should be available says so by name if it is not, because "the release
        // ships this" is exactly the claim a wrap is least able to check by reading.
        const auto builds = [](const auto& call, std::string_view what) {
            try {
                call();
            } catch (const std::exception& refusal) {
                std::println("the wrap cannot build {}: {}", what, refusal.what());
                expect(false) << what;
            }
        };
        builds([] { std::ignore = make<LdpcEncode>(ldpcEncode("ccsds_128_64")); }, "LDPC ccsds_128_64");
        builds([] { std::ignore = make<LdpcEncode>(ldpcEncode("wimax_576_288")); }, "LDPC wimax_576_288");
        builds([] { std::ignore = make<PolarEncode>(polarEncode5g()); }, "Polar (1024, 512) 5G");
        builds([] { std::ignore = make<PolarEncode>(polarEncodeAided()); }, "Polar (256, 128) GA with a CRC");
        builds([] { std::ignore = make<PolarDecode>(polarDecodeAided()); }, "Polar (256, 128) CA-SCL");
    };

    /*
     * Criterion 1, the sign anchor. One known word, strong LLRs, one coded bit flipped. Under the
     * wall's bridge the decode returns the word; under a bridge negated end to end it does not, and
     * that single vector is what makes a silent convention flip impossible.
     */
    "the LLR sign is anchored, per family"_test = [] {
        {
            const std::vector<std::uint8_t> payload = randomBits(64UZ);
            std::vector<std::uint8_t>       coded   = encoded<LdpcEncode>(ldpcEncode("ccsds_128_64"), payload);
            expect(eq(coded.size(), 128UZ));
            if (coded.size() != 128UZ) {
                return;
            }
            coded[7UZ] = static_cast<std::uint8_t>(coded[7UZ] ^ 1U);

            const std::vector<float> right = asSoft(coded, 8.0F);
            std::vector<float>       wrong(right.size());
            std::ranges::transform(right, wrong.begin(), [](float value) { return -value; });

            const std::vector<Bits> asMeant = runBlock<LdpcDecode, float>(ldpcDecode("ccsds_128_64", "normalized_min_sum"), {record<float>(right)});
            const std::vector<Bits> flipped = runBlock<LdpcDecode, float>(ldpcDecode("ccsds_128_64", "normalized_min_sum"), {record<float>(wrong)});
            expect(eq(asMeant.size(), 1UZ));
            expect(eq(flipped.size(), 1UZ));
            if (asMeant.size() == 1UZ && flipped.size() == 1UZ) {
                expect(eq(differences(asMeant[0UZ].signal_values, payload), 0UZ)) << "LDPC under the wall's bridge";
                const std::size_t underInversion = differences(flipped[0UZ].signal_values, payload);
                std::println("the LDPC sign anchor: 0 payload bit errors under the bridge, {} under its inverse", underInversion);
                expect(gt(underInversion, 0UZ)) << "LDPC under an inverted bridge must not recover the word";
            }
        }
        {
            const std::vector<std::uint8_t> payload = randomBits(512UZ);
            std::vector<std::uint8_t>       coded   = encoded<PolarEncode>(polarEncode5g(), payload);
            expect(eq(coded.size(), 1024UZ));
            if (coded.size() != 1024UZ) {
                return;
            }
            coded[11UZ] = static_cast<std::uint8_t>(coded[11UZ] ^ 1U);

            const std::vector<float> right = asSoft(coded, 8.0F);
            std::vector<float>       wrong(right.size());
            std::ranges::transform(right, wrong.begin(), [](float value) { return -value; });

            const std::vector<Bits> asMeant = runBlock<PolarDecode, float>(polarDecode5g(), {record<float>(right)});
            const std::vector<Bits> flipped = runBlock<PolarDecode, float>(polarDecode5g(), {record<float>(wrong)});
            expect(eq(asMeant.size(), 1UZ));
            expect(eq(flipped.size(), 1UZ));
            if (asMeant.size() == 1UZ && flipped.size() == 1UZ) {
                expect(eq(differences(asMeant[0UZ].signal_values, payload), 0UZ)) << "Polar under the wall's bridge";
                const std::size_t underInversion = differences(flipped[0UZ].signal_values, payload);
                std::println("the Polar sign anchor: 0 payload bit errors under the bridge, {} under its inverse", underInversion);
                expect(gt(underInversion, 0UZ)) << "Polar under an inverted bridge must not recover the word";
            }
        }
    };

    /*
     * Criterion 2: the clean round trip, at both entry points. The hard entry is the coded bits
     * presented at unit magnitude, which is the bridge a hard-decision receiver crosses; the soft
     * entry is the same vector at a strong magnitude. Neither changes a decision, and the test says
     * so rather than assuming it.
     */
    "every shipped construction round-trips clean at both entry points"_test = [] {
        for (const LdpcShape& shape : kLdpcShapes) {
            const std::vector<std::uint8_t> payload = randomBits(shape.k * 2UZ); // two frames a record
            const std::vector<std::uint8_t> coded   = encoded<LdpcEncode>(ldpcEncode(shape.standard), payload);
            expect(eq(coded.size(), shape.n * 2UZ)) << shape.standard;

            for (const float magnitude : {1.0F, 8.0F}) {
                const std::vector<Bits> out = runBlock<LdpcDecode, float>(ldpcDecode(shape.standard, "normalized_min_sum"), {record<float>(asSoft(coded, magnitude))}, [](const LdpcDecode& block) {
                    expect(eq(block.nFrames, std::uint64_t{2ULL}));
                    expect(eq(block.nCorrectedErrors, std::uint64_t{0ULL}));
                    expect(eq(block.nUncorrectableFrames, std::uint64_t{0ULL}));
                });
                expect(eq(out.size(), 1UZ)) << shape.standard << magnitude;
                if (!out.empty()) {
                    expect(eq(differences(out[0UZ].signal_values, payload), 0UZ)) << shape.standard << magnitude;
                    expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{0U})) << shape.standard;
                    expect(eq(metaCount(out[0UZ], "uncorrectable_errors"), gr::Size_t{0U})) << shape.standard;
                }
            }
        }

        {
            const std::vector<std::uint8_t> payload = randomBits(512UZ);
            const std::vector<std::uint8_t> coded   = encoded<PolarEncode>(polarEncode5g(), payload);
            for (const float magnitude : {1.0F, 8.0F}) {
                const std::vector<Bits> out = runBlock<PolarDecode, float>(polarDecode5g(), {record<float>(asSoft(coded, magnitude))});
                expect(eq(out.size(), 1UZ));
                if (!out.empty()) {
                    expect(eq(differences(out[0UZ].signal_values, payload), 0UZ)) << "Polar sc" << magnitude;
                    // Without a CRC there is no refusal to report, and the key is absent rather than zero.
                    expect(out[0UZ].meta_information[0UZ].find(gr::property_map::key_type("uncorrectable_errors")) == out[0UZ].meta_information[0UZ].end());
                }
            }
        }
        {
            // The same shape without a CRC, decoded by the list decoder alone: it separates a fault in
            // the list search from a fault in the CRC that chooses among its survivors.
            const std::vector<std::uint8_t> payload = randomBits(128UZ);
            const std::vector<std::uint8_t> coded   = encoded<PolarEncode>({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"design_snr_db", 2.5}}, payload);
            expect(eq(coded.size(), 256UZ));
            for (const std::string decoder : {"sc", "scl"}) {
                const std::vector<Bits> out = runBlock<PolarDecode, float>({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"design_snr_db", 2.5}, {"decoder", decoder}, {"list_size", gr::Size_t{4U}}}, //
                    {record<float>(asSoft(coded, 8.0F))});
                expect(eq(out.size(), 1UZ)) << decoder;
                if (!out.empty()) {
                    expect(eq(differences(out[0UZ].signal_values, payload), 0UZ)) << "Polar (256, 128)" << decoder;
                }
            }
        }
        {
            const std::vector<std::uint8_t> payload = randomBits(112UZ);
            const std::vector<std::uint8_t> coded   = encoded<PolarEncode>(polarEncodeAided(), payload);
            expect(eq(coded.size(), 256UZ));
            for (const float magnitude : {1.0F, 8.0F}) {
                const std::vector<Bits> out = runBlock<PolarDecode, float>(polarDecodeAided(), {record<float>(asSoft(coded, magnitude))});
                expect(eq(out.size(), 1UZ));
                if (!out.empty()) {
                    expect(eq(differences(out[0UZ].signal_values, payload), 0UZ)) << "Polar ca_scl" << magnitude;
                    expect(eq(metaCount(out[0UZ], "uncorrectable_errors"), gr::Size_t{0U})) << "the CRC this tree computed checks the path this tree decoded";
                }
            }
        }
    };

    /*
     * Criterion 3: both counters exercised, per family. Errors inside the code's strength are
     * corrected and `corrected_errors` reports exactly how many coded bits the received word and the
     * word decoded disagreed on; a saturated frame is refused, counted, and still published.
     */
    "correction and refusal are both counted, per family"_test = [] {
        {
            const std::vector<std::uint8_t>        payload = randomBits(288UZ);
            std::vector<std::uint8_t>              coded   = encoded<LdpcEncode>(ldpcEncode("wimax_576_288"), payload);
            constexpr std::array<std::size_t, 4UZ> flips{{3UZ, 97UZ, 254UZ, 501UZ}};
            const std::uint64_t                    kFlips = flips.size();
            for (const std::size_t at : flips) {
                coded[at] = static_cast<std::uint8_t>(coded[at] ^ 1U);
            }
            const std::vector<Bits> out = runBlock<LdpcDecode, float>(ldpcDecode("wimax_576_288", "normalized_min_sum"), {record<float>(asSoft(coded, 4.0F))}, [kFlips](const LdpcDecode& block) {
                expect(eq(block.nUncorrectableFrames, std::uint64_t{0ULL}));
                expect(eq(block.nCorrectedErrors, kFlips));
            });
            expect(eq(out.size(), 1UZ));
            if (!out.empty()) {
                expect(eq(differences(out[0UZ].signal_values, payload), 0UZ)) << "four scattered errors are inside this code's strength";
                expect(eq(metaCount(out[0UZ], "corrected_errors"), static_cast<gr::Size_t>(kFlips)));
                expect(eq(metaCount(out[0UZ], "uncorrectable_errors"), gr::Size_t{0U}));
            }

            // A frame of pure noise: the syndrome cannot be met, the count says so, and the estimate
            // still rides out so that the graph does not stall on a bad frame.
            std::vector<float> saturated(576UZ);
            for (std::size_t i = 0UZ; i < saturated.size(); ++i) {
                saturated[i] = ((next() & 1ULL) != 0ULL) ? 0.4F : -0.4F;
            }
            const std::vector<Bits> refused = runBlock<LdpcDecode, float>(ldpcDecode("wimax_576_288", "normalized_min_sum"), {record<float>(saturated)}, [](const LdpcDecode& block) {
                expect(eq(block.nUncorrectableFrames, std::uint64_t{1ULL}));
                expect(eq(block.nRecords, std::uint64_t{1ULL}));
            });
            expect(eq(refused.size(), 1UZ)) << "the record still publishes";
            if (!refused.empty()) {
                expect(eq(refused[0UZ].signal_values.size(), 288UZ));
                expect(eq(metaCount(refused[0UZ], "uncorrectable_errors"), gr::Size_t{1U}));
            }
        }
        {
            const std::vector<std::uint8_t> payload = randomBits(112UZ);
            std::vector<std::uint8_t>       coded   = encoded<PolarEncode>(polarEncodeAided(), payload);
            expect(eq(coded.size(), 256UZ));
            if (coded.size() != 256UZ) {
                return;
            }
            coded[19UZ]                 = static_cast<std::uint8_t>(coded[19UZ] ^ 1U);
            coded[140UZ]                = static_cast<std::uint8_t>(coded[140UZ] ^ 1U);
            const std::vector<Bits> out = runBlock<PolarDecode, float>(polarDecodeAided(), {record<float>(asSoft(coded, 4.0F))}, [](const PolarDecode& block) {
                expect(eq(block.nUncorrectableFrames, std::uint64_t{0ULL}));
                expect(eq(block.nCorrectedErrors, std::uint64_t{2ULL}));
            });
            expect(eq(out.size(), 1UZ));
            if (!out.empty()) {
                expect(eq(differences(out[0UZ].signal_values, payload), 0UZ)) << "two errors are inside this list decoder's reach";
                expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{2U}));
            }

            std::vector<float> saturated(256UZ);
            for (std::size_t i = 0UZ; i < saturated.size(); ++i) {
                saturated[i] = ((next() & 1ULL) != 0ULL) ? 0.4F : -0.4F;
            }
            const std::vector<Bits> refused = runBlock<PolarDecode, float>(polarDecodeAided(), {record<float>(saturated)}, [](const PolarDecode& block) {
                expect(eq(block.nUncorrectableFrames, std::uint64_t{1ULL})) << "no path in the list passes this tree's CRC";
                expect(eq(block.nRecords, std::uint64_t{1ULL}));
            });
            expect(eq(refused.size(), 1UZ)) << "the record still publishes";
            if (!refused.empty()) {
                expect(eq(refused[0UZ].signal_values.size(), 112UZ));
                expect(eq(metaCount(refused[0UZ], "uncorrectable_errors"), gr::Size_t{1U}));
            }
        }
    };

    /*
     * The wrap's own contract: the AFF3CT objects are constructed once and reused, so a decoder must
     * reset itself between frames. Three identical frames in one record decode identically; if any
     * state leaked from one frame to the next they would not.
     */
    "a decoder built once decodes identical frames identically"_test = [] {
        const std::vector<std::uint8_t> payload = randomBits(64UZ);
        std::vector<std::uint8_t>       one     = encoded<LdpcEncode>(ldpcEncode("ccsds_128_64"), payload);
        expect(eq(one.size(), 128UZ));
        if (one.size() != 128UZ) {
            return;
        }
        one[5UZ] = static_cast<std::uint8_t>(one[5UZ] ^ 1U);

        std::vector<float>       values;
        const std::vector<float> soft = asSoft(one, 4.0F);
        for (std::size_t f = 0UZ; f < 3UZ; ++f) {
            values.insert(values.end(), soft.begin(), soft.end());
        }
        const std::vector<Bits> out = runBlock<LdpcDecode, float>(ldpcDecode("ccsds_128_64", "normalized_min_sum"), {record<float>(values)});
        expect(eq(out.size(), 1UZ));
        if (out.empty()) {
            return;
        }
        expect(eq(out[0UZ].signal_values.size(), 192UZ));
        for (std::size_t f = 0UZ; f < 3UZ; ++f) {
            const std::vector<std::uint8_t> got(out[0UZ].signal_values.begin() + static_cast<std::ptrdiff_t>(f * 64UZ), out[0UZ].signal_values.begin() + static_cast<std::ptrdiff_t>((f + 1UZ) * 64UZ));
            expect(eq(differences(got, payload), 0UZ)) << "frame" << f;
        }
        expect(eq(metaCount(out[0UZ], "corrected_errors"), gr::Size_t{3U})) << "one flipped bit in each of three frames";
    };

    // Adapter conformance: a misaligned record is a counted, stated drop and the next record is
    // coded normally; a record's other facts cross verbatim.
    "a misaligned record is a counted, stated drop"_test = [] {
        const std::vector<std::uint8_t> whole   = randomBits(128UZ);
        const std::vector<std::uint8_t> partial = randomBits(100UZ);
        const std::vector<Bits>         out     = runBlock<LdpcEncode, std::uint8_t>(ldpcEncode("ccsds_128_64"), //
                        {record<std::uint8_t>(partial), record<std::uint8_t>(whole, {{"who", std::string("earlier stage")}})}, [](const LdpcEncode& block) {
                expect(eq(block.nRecordsRefused, std::uint64_t{1ULL}));
                expect(eq(block.nRecords, std::uint64_t{1ULL}));
                expect(eq(block.nFrames, std::uint64_t{2ULL}));
            });
        expect(eq(out.size(), 1UZ));
        if (out.empty()) {
            return;
        }
        expect(eq(out[0UZ].signal_values.size(), 256UZ));
        const auto& map = out[0UZ].meta_information[0UZ];
        const auto  who = map.find(gr::property_map::key_type("who"));
        expect(who != map.end());
        if (who != map.end()) {
            expect(eq(who->second.value_or(std::string{}), std::string("earlier stage")));
        }
    };

    return 0;
}
