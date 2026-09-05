#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <print>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/channel/NoisePower.hpp>
#include <gnuradio-4.0/algorithm/digital/Crc.hpp>

#include <gnuradio-4.0/ax25/Ax25.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/digital/CpmModulate.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>
#include <gnuradio-4.0/digital/DifferentialCoding.hpp>
#include <gnuradio-4.0/sync/PreambleTiming.hpp>
#include <gnuradio-4.0/sync/SymbolSync.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

namespace {

using gr::blocks::ax25::Ax25Decode;
using gr::blocks::ax25::Ax25Encode;
using gr::blocks::basic::DataSetToStream;
using gr::blocks::channel::AwgnChannel;
using gr::blocks::digital::CpmModulate;
using gr::blocks::digital::CrcAppend;
using gr::blocks::digital::DelimiterFramer;
using gr::blocks::digital::DifferentialEncoder;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSource;

using CF     = std::complex<float>;
using Record = gr::DataSet<std::uint8_t>;

/// ITU-R M.1371 Annex 2 section 3: 9600 bit/s GMSK at BT 0.4, modulation index 0.5, NRZI, HDLC framing.
constexpr float       kSampleRate    = 48000.F; // an integer five samples a symbol, and a rate a discriminator is delivered at
constexpr float       kSymbolRate    = 9600.F;
constexpr double      kModIndex      = 0.5;
constexpr double      kBt            = 0.4;
constexpr std::size_t kSps           = 5UZ;
constexpr std::size_t kTraining      = 24UZ; ///< alternating symbols, a tone at half the bit rate, for the loop to settle on
constexpr std::size_t kPayloadOctets = 21UZ; ///< 168 bits, the default AIS message length

/// The 16-bit frame check of ISO/IEC 13239, which is the AX.25 FCS and the AIS FCS alike.
[[nodiscard]] const gr::digital::Crc& frameCheck() {
    static const gr::digital::Crc crc(16U, 0x1021ULL, 0xFFFFULL, 0xFFFFULL, true, true);
    return crc;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
    [[nodiscard]] std::size_t below(std::size_t bound) noexcept { return next() % bound; }
};

[[nodiscard]] std::vector<std::uint8_t> flagBits() { return {0U, 1U, 1U, 1U, 1U, 1U, 1U, 0U}; }

/// @brief One slot's HDLC bit stream: opening flag, the octets least significant bit first and stuffed, closing flag.
[[nodiscard]] std::vector<std::uint8_t> hdlcBits(std::span<const std::uint8_t> payload) {
    std::vector<std::uint8_t> octets(payload.begin(), payload.end());
    const auto                fcs = static_cast<std::uint16_t>(frameCheck().compute(payload));
    octets.push_back(static_cast<std::uint8_t>(fcs & 0xFFU)); // the check sequence travels least significant octet first
    octets.push_back(static_cast<std::uint8_t>(fcs >> 8U));

    std::vector<std::uint8_t> wire = flagBits();
    unsigned                  ones = 0U;
    for (const std::uint8_t octet : octets) {
        for (unsigned bit = 0U; bit < 8U; ++bit) {
            const std::uint8_t value = (octet >> bit) & 1U;
            wire.push_back(value);
            if (value != 0U) {
                if (++ones == 5U) { // a zero after five ones, so the payload can never forge the flag
                    wire.push_back(0U);
                    ones = 0U;
                }
            } else {
                ones = 0U;
            }
        }
    }
    const std::vector<std::uint8_t> closing = flagBits();
    wire.insert(wire.end(), closing.begin(), closing.end());
    return wire;
}

/**
 * @brief One slot as channel bits: the training tone, then the NRZI-coded frame.
 *
 * NRZI holds the level on a one and transitions on a zero, so the encoder's state entering the flag decides whether
 * the alternation breaks there. Starting it at the training sequence's last symbol makes the flag's leading zero
 * one more transition, and the tone runs unbroken into the frame.
 */
[[nodiscard]] std::vector<std::uint8_t> slotBits(std::span<const std::uint8_t> payload) {
    std::vector<std::uint8_t> channel;
    std::uint8_t              level = 0U;
    for (std::size_t k = 0UZ; k < kTraining; ++k) {
        channel.push_back(level);
        level = static_cast<std::uint8_t>(level ^ 1U);
    }
    std::uint8_t previous = channel.back();
    for (const std::uint8_t bit : hdlcBits(payload)) {
        previous = static_cast<std::uint8_t>(bit != 0U ? previous : (previous ^ 1U));
        channel.push_back(previous);
    }
    return channel;
}

/// @brief The two-level PAM grid the modulator reads: one is +1, zero is -1.
[[nodiscard]] std::vector<float> symbolsOf(std::span<const std::uint8_t> bits) {
    std::vector<float> symbols(bits.size());
    std::ranges::transform(bits, symbols.begin(), [](std::uint8_t bit) { return bit != 0U ? 1.F : -1.F; });
    return symbols;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

/// @brief Runs @p symbols through `CpmModulate` at the AIS constants and hands back the complex baseband.
///
/// The modulator is driven directly rather than under a scheduler: one burst is a few hundred symbols, and standing
/// a graph up for each of two hundred of them costs far more than the modulation does.
[[nodiscard]] std::vector<CF> modulate(std::span<const float> symbols) {
    CpmModulate<float> shaper = make<CpmModulate<float>>({{"pulse", std::string("gaussian")}, {"bt", kBt}, {"modulation_index", kModIndex}, {"samples_per_symbol", static_cast<gr::Size_t>(kSps)}});
    shaper.start();
    std::vector<CF> baseband(symbols.size() * kSps);
    std::ignore = shaper.processBulk(symbols, std::span<CF>(baseband));
    return baseband;
}

[[nodiscard]] gr::PluginLoader recipeLoader() {
    static gr::SchedulerRegistry          schedulerRegistry;
    static const std::vector<std::string> paths{std::string(RECIPES_SOURCE_PATH)};
    return gr::PluginLoader(gr::globalBlockRegistry(), schedulerRegistry, paths);
}

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Record> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        if (inSpan.size() == 0UZ) { // an empty call is not progress, and saying so is what lets the graph finish
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t k = 0UZ; k < n; ++k) {
            outSpan[k] = _records[_pos + k];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// What one receive run left on each of the composite's three record ports, and what its extractor counted.
struct Received {
    std::vector<Record> ok{};
    std::vector<Record> fail{};
    std::vector<Record> reject{};
    std::uint64_t       aborts = ~0ULL;
};

/**
 * @brief The abort runs the composite's own extractor counted, read off the interior block the recipe names.
 *
 * An abort abandons the frame in progress and produces no record, so it leaves by no port and the counter is the
 * only place it is stated. Reaching it goes through the subgraph the composite holds, matching the recipe's own
 * block name and checking the erased type before the cast; a run that finds neither answers with a value no run
 * can produce, so a failure to reach the block fails the assertion rather than passing it.
 */
[[nodiscard]] std::uint64_t abortsOf(const std::shared_ptr<gr::BlockModel>& composite) {
    using Extractor = gr::blocks::digital::DelimiterExtractor<std::uint8_t>;
    if (composite == nullptr || composite->graph() == nullptr) {
        return ~0ULL;
    }
    for (const auto& child : composite->graph()->blocks()) {
        if (child != nullptr && child->name() == "extractor" && child->typeName() == gr::meta::type_name<Extractor>()) {
            return static_cast<Extractor*>(child->raw())->nAborts;
        }
    }
    return ~0ULL;
}

/**
 * @brief `FskDemod` into `HdlcDeframe` over @p stream, with white Gaussian noise added at @p esN0_db.
 *
 * The noise is added to the whole stream, so the silence either side of a burst is noise and the receiver has to
 * find the burst in it rather than being told where it is.
 */
[[nodiscard]] Received receive(std::span<const CF> stream, double noiseBandwidth, double esN0_db, std::uint64_t seed, std::uint32_t preambleSymbols = 0U) {
    auto loader = recipeLoader();
    auto demod  = loader.instantiate("gr::recipes::FskDemod", {{"sample_rate", kSampleRate}, {"symbol_rate", kSymbolRate}, {"modulation_index", kModIndex}, {"noise_bandwidth", noiseBandwidth}, {"preamble_symbols", preambleSymbols}});
    boost::ut::expect(demod != nullptr) << "gr::recipes::FskDemod" << boost::ut::fatal;
    auto deframe = loader.instantiate("gr::recipes::HdlcDeframe", {{"max_payload_items", std::uint32_t{64U}}});
    boost::ut::expect(deframe != nullptr) << "gr::recipes::HdlcDeframe" << boost::ut::fatal;

    gr::Graph  graph;
    const auto values  = gr::Tensor<CF>(stream.begin(), stream.end());
    auto&      source  = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}, {"mark_tag", false}});
    auto&      channel = graph.emplaceBlock<AwgnChannel<CF>>({{"noise_power", static_cast<double>(gr::channel::noisePowerFor(static_cast<float>(esN0_db), 1.F, static_cast<float>(kSps)))}, {"seed", seed}});
    const auto front   = graph.addBlock(std::move(demod));
    const auto framing = graph.addBlock(std::move(deframe));
    auto&      good    = graph.emplaceBlock<RecordSink>();
    auto&      bad     = graph.emplaceBlock<RecordSink>();
    auto&      refused = graph.emplaceBlock<RecordSink>();

    const auto sourceModel  = gr::graph::findBlock(graph, source);
    const auto channelModel = gr::graph::findBlock(graph, channel);
    const auto goodModel    = gr::graph::findBlock(graph, good);
    const auto badModel     = gr::graph::findBlock(graph, bad);
    const auto refusedModel = gr::graph::findBlock(graph, refused);
    boost::ut::expect(sourceModel.has_value() && channelModel.has_value() && goodModel.has_value() && badModel.has_value() && refusedModel.has_value()) << boost::ut::fatal;

    boost::ut::expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, *channelModel, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(*channelModel, gr::PortDefinition{"out"}, front, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(front, gr::PortDefinition{"out"}, framing, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(framing, gr::PortDefinition{"out"}, *goodModel, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(framing, gr::PortDefinition{"fail"}, *badModel, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(framing, gr::PortDefinition{"reject"}, *refusedModel, gr::PortDefinition{"in"}).has_value());

    Received              result;
    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    const auto finished = scheduler.runAndWait();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);
    result.ok     = good._records;
    result.fail   = bad._records;
    result.reject = refused._records;
    result.aborts = abortsOf(framing);
    return result;
}

struct LabelSink : gr::Block<LabelSink> {
    gr::PortIn<std::uint8_t> in;
    GR_MAKE_REFLECTABLE(LabelSink, in);
    std::vector<std::uint8_t> _labels{};

    [[nodiscard]] gr::work::Status processBulk(std::span<const std::uint8_t> input) {
        _labels.insert(_labels.end(), input.begin(), input.end());
        return gr::work::Status::OK;
    }
};

/// @brief What the composite's own preamble stage counted, read off the interior block the recipe names.
[[nodiscard]] std::pair<std::uint64_t, std::uint64_t> detectionsOf(const std::shared_ptr<gr::BlockModel>& composite) {
    using Detector = gr::blocks::sync::PreambleTiming<float>;
    if (composite == nullptr || composite->graph() == nullptr) {
        return {~0ULL, ~0ULL};
    }
    for (const auto& child : composite->graph()->blocks()) {
        if (child != nullptr && child->name() == "preamble" && child->typeName() == gr::meta::type_name<Detector>()) {
            const auto* block = static_cast<Detector*>(child->raw());
            return {block->nDetections, block->nSuppressed};
        }
    }
    return {~0ULL, ~0ULL};
}

/// @brief What the composite's own timing loop refused, read off the interior block the recipe names.
[[nodiscard]] std::uint64_t ignoredBy(const std::shared_ptr<gr::BlockModel>& composite) {
    using Loop = gr::blocks::sync::SymbolSync<float>;
    if (composite == nullptr || composite->graph() == nullptr) {
        return ~0ULL;
    }
    for (const auto& child : composite->graph()->blocks()) {
        if (child != nullptr && child->name() == "timing" && child->typeName() == gr::meta::type_name<Loop>()) {
            return static_cast<const Loop*>(child->raw())->ignoredTagPayloads();
        }
    }
    return ~0ULL;
}

/// @brief One `FskDemod` run: the labels the deframer would be handed, and what the stage inside it counted.
struct Demodulated {
    std::vector<std::uint8_t> labels{};
    std::uint64_t             detections = 0ULL;
    std::uint64_t             suppressed = 0ULL;
    std::uint64_t             refused    = 0ULL; ///< timing payloads the loop declined
};

/// @brief `FskDemod` alone over @p stream: the sliced labels the deframer would be handed.
[[nodiscard]] Demodulated demodulate(std::span<const CF> stream, double noiseBandwidth, double esN0_db, std::uint64_t seed, std::uint32_t preambleSymbols) {
    auto loader = recipeLoader();
    auto demod  = loader.instantiate("gr::recipes::FskDemod", {{"sample_rate", kSampleRate}, {"symbol_rate", kSymbolRate}, {"modulation_index", kModIndex}, {"noise_bandwidth", noiseBandwidth}, {"preamble_symbols", preambleSymbols}});
    boost::ut::expect(demod != nullptr) << boost::ut::fatal;

    gr::Graph  graph;
    const auto values  = gr::Tensor<CF>(stream.begin(), stream.end());
    auto&      source  = graph.emplaceBlock<TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}, {"mark_tag", false}});
    auto&      channel = graph.emplaceBlock<AwgnChannel<CF>>({{"noise_power", static_cast<double>(gr::channel::noisePowerFor(static_cast<float>(esN0_db), 1.F, static_cast<float>(kSps)))}, {"seed", seed}});
    const auto front   = graph.addBlock(std::move(demod));
    auto&      sink    = graph.emplaceBlock<LabelSink>();

    const auto sourceModel  = gr::graph::findBlock(graph, source);
    const auto channelModel = gr::graph::findBlock(graph, channel);
    const auto sinkModel    = gr::graph::findBlock(graph, sink);
    boost::ut::expect(sourceModel.has_value() && channelModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
    boost::ut::expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, *channelModel, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(*channelModel, gr::PortDefinition{"out"}, front, gr::PortDefinition{"in"}).has_value());
    boost::ut::expect(graph.connect(front, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
    std::ignore                         = scheduler.runAndWait();
    const auto [detections, suppressed] = detectionsOf(front);
    return Demodulated{std::move(sink._labels), detections, suppressed, ignoredBy(front)};
}

/// @brief A stream of @p count seeded slots, each starting at a random symbol phase inside a noise-only gap.
struct Scene {
    std::vector<CF>                        stream{};
    std::vector<std::vector<std::uint8_t>> payloads{};
    std::vector<std::size_t>               starts{};
};

[[nodiscard]] Scene bursts(std::size_t count, std::uint64_t seed, std::size_t gapSymbols = 40UZ) {
    Rng   rng{seed};
    Scene scene;
    scene.stream.insert(scene.stream.end(), gapSymbols * kSps, CF{});
    for (std::size_t k = 0UZ; k < count; ++k) {
        std::vector<std::uint8_t> payload(kPayloadOctets);
        for (std::uint8_t& octet : payload) {
            octet = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
        }
        const std::vector<CF> burst = modulate(std::span<const float>(symbolsOf(std::span<const std::uint8_t>(slotBits(std::span<const std::uint8_t>(payload))))));
        scene.stream.insert(scene.stream.end(), rng.below(kSps), CF{}); // the burst starts at a random symbol phase
        scene.starts.push_back(scene.stream.size());
        scene.stream.insert(scene.stream.end(), burst.begin(), burst.end());
        scene.stream.insert(scene.stream.end(), gapSymbols * kSps, CF{});
        scene.payloads.push_back(std::move(payload));
    }
    return scene;
}

/// @brief What the sliced label stream says about one arm's slots, read against the bits transmitted.
struct Sliced {
    std::size_t labels        = 0UZ; ///< labels the demodulator produced over the whole stream
    std::size_t perfect       = 0UZ; ///< slots reproduced symbol for symbol, training sequence included
    std::size_t frameClean    = 0UZ; ///< slots whose frame region, from the start flag on, carries no error
    std::size_t frameErrors   = 0UZ; ///< symbol errors inside those frame regions, over every slot
    std::size_t flagWrong     = 0UZ; ///< slots whose start flag's first bit is wrong
    std::size_t alignmentSpan = 0UZ; ///< symbols between the earliest slot alignment and the latest
};

/// @brief Each slot at its own best alignment in @p labels: where the demodulator put it, and what it got wrong.
[[nodiscard]] Sliced slice(const Scene& scene, std::span<const std::uint8_t> labels) {
    constexpr long kSearch = 40L; // the chain's group delay is about fourteen symbols and every slot carries the same one
    Sliced         result{};
    long           earliest = std::numeric_limits<long>::max();
    long           latest   = std::numeric_limits<long>::min();
    result.labels           = labels.size();
    for (std::size_t k = 0UZ; k < scene.payloads.size(); ++k) {
        const std::vector<std::uint8_t> bits   = slotBits(std::span<const std::uint8_t>(scene.payloads[k]));
        const long                      center = static_cast<long>(scene.starts[k] / kSps);
        long                            best   = -1L;
        std::size_t                     fewest = bits.size() + 1UZ;
        for (long a = center - kSearch; a <= center + kSearch; ++a) {
            if (a < 0L || static_cast<std::size_t>(a) + bits.size() > labels.size()) {
                continue;
            }
            std::size_t errors = 0UZ;
            for (std::size_t i = 0UZ; i < bits.size(); ++i) {
                errors += labels[static_cast<std::size_t>(a) + i] != bits[i] ? 1UZ : 0UZ;
            }
            if (errors < fewest) {
                fewest = errors;
                best   = a;
            }
        }
        if (best < 0L) {
            continue;
        }
        std::size_t frameErrors = 0UZ;
        for (std::size_t i = kTraining; i < bits.size(); ++i) {
            frameErrors += labels[static_cast<std::size_t>(best) + i] != bits[i] ? 1UZ : 0UZ;
        }
        result.perfect += fewest == 0UZ ? 1UZ : 0UZ;
        result.frameClean += frameErrors == 0UZ ? 1UZ : 0UZ;
        result.frameErrors += frameErrors;
        result.flagWrong += labels[static_cast<std::size_t>(best) + kTraining] != bits[kTraining] ? 1UZ : 0UZ;
        earliest = std::min(earliest, best - center);
        latest   = std::max(latest, best - center);
    }
    result.alignmentSpan = latest >= earliest ? static_cast<std::size_t>(latest - earliest) : 0UZ;
    return result;
}

/// @brief How many of @p expected payloads appear among @p received, each counted once.
[[nodiscard]] std::size_t matched(std::span<const std::vector<std::uint8_t>> expected, std::span<const Record> received) {
    std::set<std::vector<std::uint8_t>> seen;
    for (const Record& record : received) {
        seen.insert(record.signal_values);
    }
    std::size_t hits = 0UZ;
    for (const std::vector<std::uint8_t>& payload : expected) {
        hits += seen.contains(payload) ? 1UZ : 0UZ;
    }
    return hits;
}

[[nodiscard]] std::vector<std::string> exportedNames(const gr::property_map& portsMap) {
    std::vector<std::string> names;
    for (const auto& [blockName, portInfoValue] : portsMap) {
        const auto* portMap = portInfoValue.get_if<gr::property_map>();
        if (portMap == nullptr) {
            continue;
        }
        for (const auto& [internalName, exportInfoValue] : *portMap) {
            const auto* exportMap = exportInfoValue.get_if<gr::property_map>();
            if (exportMap == nullptr) {
                continue;
            }
            if (const auto it = exportMap->find("exportedName"); it != exportMap->end()) {
                names.emplace_back(it->second.value_or(std::string_view{}));
            }
        }
    }
    return names;
}

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    if (record.meta_information.empty()) {
        return {};
    }
    const auto& map   = record.meta_information.front();
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : std::string(entry->second.value_or(std::string_view{}));
}

[[nodiscard]] Record recordOf(std::vector<std::uint8_t> items) {
    Record record;
    record.extents.push_back(static_cast<std::int32_t>(items.size()));
    record.signal_values = std::move(items);
    record.signal_names.emplace_back("payload");
    record.signal_quantities.emplace_back("");
    record.signal_units.emplace_back("");
    record.meta_information.emplace_back();
    record.timing_events.emplace_back();
    return record;
}

/// The AX.25 anchor's information field, and the addressing that frames it.
constexpr std::array<std::uint8_t, 5UZ> kInfo{{0x3AU, 0x54U, 0x45U, 0x53U, 0x54U}};

} // namespace

const boost::ut::suite<"hdlc bursts"> hdlcBurstTests = [] {
    using namespace boost::ut;

    "HdlcDeframe demands its bound and exports both failure ports"_test = [] {
        auto loader = recipeLoader();
        expect(loader.instantiate("gr::recipes::HdlcDeframe") == nullptr) << "the extractor's bound has no default, and the recipe inherits the clause";

        auto composite = loader.instantiate("gr::recipes::HdlcDeframe", {{"max_payload_items", std::uint32_t{64U}}});
        expect(composite != nullptr) << fatal;

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end());
        expect(eq(outputs.size(), 3UZ)) << "a discard leaves by a port with its reason, so neither failure port is hidden";
        for (const std::string_view name : {"out", "fail", "reject"}) {
            expect(std::ranges::find(outputs, name) != outputs.end()) << name;
        }
    };

    "the AX.25 chain reproduces its result with HdlcDeframe in place of its three blocks"_test = [] {
        const gr::property_map addressing{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}, {"control_type", std::string("UI")}, //
            {"pid", gr::Size_t{0xF0}}, {"command", true}, {"poll_final", false}};
        const gr::property_map fcs{{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, {"final_xor", std::uint64_t{0xFFFF}}, //
            {"input_reflected", true}, {"result_reflected", true}, {"crc_byte_order", std::string("little")}};
        const gr::property_map framing{{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"bits_per_item", gr::Size_t{1}}, //
            {"max_payload_items", gr::Size_t{8192}}, {"payload_pack_bits", gr::Size_t{8}}, {"payload_bit_order", std::string("lsb_first")}};

        auto loader  = recipeLoader();
        auto deframe = loader.instantiate("gr::recipes::HdlcDeframe", {{"max_payload_items", std::uint32_t{1024U}}});
        expect(deframe != nullptr) << fatal;

        gr::Graph flow;
        auto&     source    = flow.emplaceBlock<RecordSource>();
        source._records     = {recordOf(std::vector<std::uint8_t>(kInfo.begin(), kInfo.end()))};
        auto&      encode   = flow.emplaceBlock<Ax25Encode>(addressing);
        auto&      append   = flow.emplaceBlock<CrcAppend>(fcs);
        auto&      framer   = flow.emplaceBlock<DelimiterFramer>(framing);
        auto&      wire     = flow.emplaceBlock<DataSetToStream<std::uint8_t>>({{"boundary_label", std::string("")}});
        auto&      line     = flow.emplaceBlock<DifferentialEncoder<std::uint8_t>>({{"coding", std::string("nrzi")}});
        const auto framing_ = flow.addBlock(std::move(deframe));
        auto&      decode   = flow.emplaceBlock<Ax25Decode>();
        auto&      passed   = flow.emplaceBlock<RecordSink>();

        expect(flow.connect<"out", "in">(source, encode).has_value());
        expect(flow.connect<"out", "in">(encode, append).has_value());
        expect(flow.connect<"out", "in">(append, framer).has_value());
        expect(flow.connect<"out", "in">(framer, wire).has_value());
        expect(flow.connect<"out", "in">(wire, line).has_value());
        const auto lineModel   = gr::graph::findBlock(flow, line);
        const auto decodeModel = gr::graph::findBlock(flow, decode);
        expect(lineModel.has_value() && decodeModel.has_value()) << fatal;
        expect(flow.connect(*lineModel, gr::PortDefinition{"out"}, framing_, gr::PortDefinition{"in"}).has_value());
        expect(flow.connect(framing_, gr::PortDefinition{"out"}, *decodeModel, gr::PortDefinition{"in"}).has_value());
        expect(flow.connect<"out", "in">(decode, passed).has_value());

        std::vector<Record>   received;
        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(flow)).has_value());
        const auto finished = scheduler.runAndWait();
        expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);
        received = passed._records;

        expect(eq(received.size(), 1UZ)) << "the anchor survives NRZI coding, the flag framing and the frame check";
        if (received.size() == 1UZ) {
            expect(that % (received[0UZ].signal_values == std::vector<std::uint8_t>(kInfo.begin(), kInfo.end())));
            expect(eq(metaString(received[0UZ], "ax25_destination"), std::string("APRS")));
            expect(eq(metaString(received[0UZ], "ax25_source"), std::string("N0CALL")));
            expect(eq(metaString(received[0UZ], "ax25_type"), std::string("UI")));
        }
    };

    "the burst decodes, and the loop bandwidth the training sequence admits is measured"_test = [] {
        constexpr std::size_t kBursts = 200UZ;
        const Scene           scene   = bursts(kBursts, 0xA15ULL);
        std::println("[record] {} AIS slots at 48 kHz, {} samples", kBursts, scene.stream.size());

        const Received    atDefault = receive(std::span<const CF>(scene.stream), 0.002, 20.0, 0xC0FFEEULL);
        const std::size_t byDefault = matched(std::span<const std::vector<std::uint8_t>>(scene.payloads), std::span<const Record>(atDefault.ok));
        std::println("[record] Es/N0 20 dB, noise_bandwidth 0.002 (FskDemod's default): {} of {} payloads reproduced, {} failed the check, {} refused by the extractor", byDefault, kBursts, atDefault.fail.size(), atDefault.reject.size());

        std::size_t best             = 0UZ;
        double      bestWidth        = 0.0;
        double      smallestComplete = 0.0;
        for (const double width : {0.002, 0.01, 0.02, 0.05, 0.1}) {
            const Received    run  = receive(std::span<const CF>(scene.stream), width, 20.0, 0xC0FFEEULL);
            const std::size_t hits = matched(std::span<const std::vector<std::uint8_t>>(scene.payloads), std::span<const Record>(run.ok));
            std::println("[record] Es/N0 20 dB, noise_bandwidth {:.3f}: {} of {} payloads, {} failed, {} refused", width, hits, kBursts, run.fail.size(), run.reject.size());
            if (hits > best) {
                best      = hits;
                bestWidth = width;
            }
            if (hits == kBursts && smallestComplete == 0.0) {
                smallestComplete = width;
            }
        }
        std::println("[record] best at Es/N0 20 dB: {} of {} payloads at noise_bandwidth {:.3f}; smallest bandwidth decoding all {}: {:.3f} (0 means none did)", best, kBursts, bestWidth, kBursts, smallestComplete);
        // a second-order loop at critical damping settles in about 3/(Bn*T) symbols, so the 32 symbols of training and
        // flag ahead of the data admit Bn*T of about 0.1, and the 0.002 a continuous link is sized for settles in the
        // middle of the payload instead
        expect(ge(best, byDefault)) << "a burst does not decode better on a loop sized for a continuous link";
        expect(gt(best, kBursts / 2UZ)) << "the training sequence is long enough for a loop that is sized for it";

        const Received    weak    = receive(std::span<const CF>(scene.stream), bestWidth, 10.0, 0xBEEFULL);
        const std::size_t atTenDb = matched(std::span<const std::vector<std::uint8_t>>(scene.payloads), std::span<const Record>(weak.ok));
        std::println("[record] Es/N0 10 dB at noise_bandwidth {:.3f}: {} of {} payloads", bestWidth, atTenDb, kBursts);
        expect(le(atTenDb, best)) << "ten decibels less signal cannot decode more";
    };

    "two slots back to back both decode, and a truncated slot leaves nothing on out"_test = [] {
        Rng                                    rng{0x5107ULL};
        std::vector<std::vector<std::uint8_t>> payloads;
        std::vector<std::uint8_t>              adjacentBits;
        for (std::size_t k = 0UZ; k < 2UZ; ++k) {
            std::vector<std::uint8_t> payload(kPayloadOctets);
            for (std::uint8_t& octet : payload) {
                octet = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
            }
            const std::vector<std::uint8_t> bits = slotBits(std::span<const std::uint8_t>(payload));
            adjacentBits.insert(adjacentBits.end(), bits.begin(), bits.end()); // the second slot opens where the first closed
            payloads.push_back(std::move(payload));
        }

        std::vector<CF>       stream(40UZ * kSps, CF{});
        const std::vector<CF> pair = modulate(std::span<const float>(symbolsOf(std::span<const std::uint8_t>(adjacentBits))));
        stream.insert(stream.end(), pair.begin(), pair.end());
        stream.insert(stream.end(), 40UZ * kSps, CF{});

        // the same two slots with a slot's worth of silence between them, as the control: at 20 dB the loop, not the
        // noise, decides whether a slot decodes, so "both decode" is only meaningful against what a gap gives
        std::vector<CF> spaced(40UZ * kSps, CF{});
        for (std::size_t k = 0UZ; k < 2UZ; ++k) {
            const std::vector<CF> burst = modulate(std::span<const float>(symbolsOf(std::span<const std::uint8_t>(slotBits(std::span<const std::uint8_t>(payloads[k]))))));
            spaced.insert(spaced.end(), burst.begin(), burst.end());
            spaced.insert(spaced.end(), 40UZ * kSps, CF{});
        }

        const Received    back      = receive(std::span<const CF>(stream), 0.05, 20.0, 0xC0FFEEULL);
        const Received    apart     = receive(std::span<const CF>(spaced), 0.05, 20.0, 0xC0FFEEULL);
        const std::size_t adjacent  = matched(std::span<const std::vector<std::uint8_t>>(payloads), std::span<const Record>(back.ok));
        const std::size_t separated = matched(std::span<const std::vector<std::uint8_t>>(payloads), std::span<const Record>(apart.ok));
        std::println("[record] two slots at noise_bandwidth 0.050, Es/N0 20 dB: {} of 2 back to back, {} of 2 with a slot of silence between them", adjacent, separated);
        expect(eq(adjacent, 2UZ)) << "a slot that opens where the last one closed is not hidden by it";
        expect(ge(adjacent, separated));

        // the transmitter stops after 100 bits of data: no closing flag arrives, so nothing may reach `out`
        std::vector<std::uint8_t> payload(kPayloadOctets);
        for (std::uint8_t& octet : payload) {
            octet = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
        }
        std::vector<std::uint8_t> bits = slotBits(std::span<const std::uint8_t>(payload));
        bits.resize(kTraining + 8UZ + 100UZ);
        std::vector<CF>       cut(40UZ * kSps, CF{});
        const std::vector<CF> burst = modulate(std::span<const float>(symbolsOf(std::span<const std::uint8_t>(bits))));
        cut.insert(cut.end(), burst.begin(), burst.end());
        cut.insert(cut.end(), 200UZ * kSps, CF{});

        const Received truncated = receive(std::span<const CF>(cut), 0.05, 20.0, 0xC0FFEEULL);
        expect(eq(matched(std::span<const std::vector<std::uint8_t>>(std::span(&payload, 1UZ)), std::span<const Record>(truncated.ok)), 0UZ)) << "an unclosed frame is not a frame";
        std::println("[record] a slot cut after 100 data bits: {} on out, {} failed the check, {} refused by the extractor, {} aborts counted", truncated.ok.size(), truncated.fail.size(), truncated.reject.size(), truncated.aborts);
        expect(gt(truncated.reject.size() + truncated.fail.size(), 0UZ)) << "the region the abandoned frame left reaches a failure port";
        expect(eq(truncated.aborts, 1ULL)) << "the abort itself leaves by no port, so the extractor's own counter is where the reason is stated";
    };

    "the burst timing preset takes the loop bandwidth out of the decision"_test = [] {
        // The same 200 slots and the same seeds as the leg above, which is the negative control: that leg runs the
        // chain with the preset stage a wire and its counts are the ones this arm is read against. What the preset
        // claims is not a better best but a flat table -- the bandwidth stops deciding whether a burst acquires --
        // so the spread across the sweep is asserted as well as the counts.
        constexpr std::size_t kBursts = 200UZ;
        const Scene           scene   = bursts(kBursts, 0xA15ULL);

        const Received    reference = receive(std::span<const CF>(scene.stream), 0.05, 20.0, 0xC0FFEEULL);
        const std::size_t control   = matched(std::span<const std::vector<std::uint8_t>>(scene.payloads), std::span<const Record>(reference.ok));
        std::println("[record] the control's best bandwidth, noise_bandwidth 0.050 with the stage a wire: {} of {} payloads", control, kBursts);

        std::size_t fewest = kBursts;
        std::size_t most   = 0UZ;
        for (const double width : {0.002, 0.01, 0.02, 0.05}) {
            const Received    run  = receive(std::span<const CF>(scene.stream), width, 20.0, 0xC0FFEEULL, static_cast<std::uint32_t>(kTraining));
            const std::size_t hits = matched(std::span<const std::vector<std::uint8_t>>(scene.payloads), std::span<const Record>(run.ok));
            std::println("[record] preamble_symbols {}, noise_bandwidth {:.3f}: {} of {} payloads, {} failed, {} refused, {} aborts", kTraining, width, hits, kBursts, run.fail.size(), run.reject.size(), run.aborts);
            fewest = std::min(fewest, hits);
            most   = std::max(most, hits);
            expect(ge(hits, control)) << "every bandwidth decodes at least what the best one does without the preset, at " << width;
        }
        std::println("[record] preamble_symbols {} over the four narrow bandwidths: {} to {} of {}, a spread of {}", kTraining, fewest, most, kBursts, most - fewest);
        expect(lt(most - fewest, kBursts / 16UZ)) << "and the sweep is flat, against the 42 slots the control spreads over";

        const Received    wide     = receive(std::span<const CF>(scene.stream), 0.1, 20.0, 0xC0FFEEULL, static_cast<std::uint32_t>(kTraining));
        const std::size_t atWidest = matched(std::span<const std::vector<std::uint8_t>>(scene.payloads), std::span<const Record>(wide.ok));
        std::println("[record] preamble_symbols {}, noise_bandwidth 0.100: {} of {} payloads", kTraining, atWidest, kBursts);
        expect(ge(atWidest, control)) << "the widest bandwidth's loss is tracking jitter, and the preset carries it too";
    };

    "the preset re-times the symbol stream and does not shorten it"_test = [] {
        // A decode count cannot say why a slot was lost, so this reads the sliced labels against the transmitted
        // bits directly. Two properties decide it. The stream must stay one label per symbol: a preset that takes a
        // sample out of it walks every later slot's alignment earlier and costs one label a burst. And the last
        // training symbol must be taken at the preset phase, because a differential decoder reads it together with
        // the start flag's first bit, so slicing it at the phase the loop was holding inverts the flag.
        constexpr std::size_t kBursts = 200UZ;
        const Scene           scene   = bursts(kBursts, 0xA15ULL);

        std::array<Sliced, 2UZ> arms{};
        for (const std::size_t arm : {0UZ, 1UZ}) {
            const std::uint32_t training = arm == 0UZ ? 0U : static_cast<std::uint32_t>(kTraining);
            const Demodulated   run      = demodulate(std::span<const CF>(scene.stream), 0.002, 20.0, 0xC0FFEEULL, training);
            arms[arm]                    = slice(scene, std::span<const std::uint8_t>(run.labels));
            std::println("[record] preamble_symbols {}: {} labels, {} of {} slots symbol-perfect, {} with the whole frame region clean, {} symbol errors in the frame regions, {} slots with the flag's first bit wrong, alignment over {} symbols", training, arms[arm].labels, arms[arm].perfect, kBursts, arms[arm].frameClean, arms[arm].frameErrors, arms[arm].flagWrong, arms[arm].alignmentSpan);
            expect(eq(run.detections, training == 0U ? 0ULL : static_cast<std::uint64_t>(kBursts))) << "one tag a burst and none from a payload, at preamble_symbols " << training;
            expect(eq(run.suppressed, 0ULL)) << "and none of them inside a hold-off";
            expect(eq(run.refused, 0ULL)) << "and the loop honored every payload it was handed";
        }

        expect(lt(arms[0UZ].labels, arms[1UZ].labels + kBursts / 4UZ)) << "the preset re-times the stream without taking a symbol out of it, or the count falls by about one a burst";
        expect(le(arms[1UZ].alignmentSpan, arms[0UZ].alignmentSpan)) << "and every slot lands where the control puts it, rather than walking a symbol earlier burst by burst";
        expect(lt(arms[1UZ].flagWrong, arms[0UZ].flagWrong / 4UZ)) << "the last training symbol is taken at the preset phase, so the differential decoder reads the flag's first bit right";
        expect(gt(arms[1UZ].frameClean, arms[0UZ].frameClean)) << "and more slots reach the deframer with the whole frame region symbol-perfect";
    };
};

int main() { /* not needed for UT */ }
