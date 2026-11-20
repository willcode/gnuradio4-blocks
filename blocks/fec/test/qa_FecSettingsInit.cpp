#include <boost/ut.hpp>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <expected>
#include <format>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/fec/ConvBlocks.hpp>
#include <gnuradio-4.0/fec/FecBlocks.hpp>
#include <gnuradio-4.0/fec/InterleaveBlocks.hpp>
#include <gnuradio-4.0/fec/PunctureBlocks.hpp>
#include <gnuradio-4.0/fec/RsBlocks.hpp>

#include <gnuradio-4.0/testing/InitChecks.hpp>

// The fec blocks at their defaults and with each setting off its default, initialized as a graph's addition does. Every
// block of the family has a setting with no default (the code, the kind, the pattern), and is not ready until a caller
// gives it; a setting that only has meaning beside that one is checked in a whole configuration that names both.

namespace {

using namespace gr::blocks::fec;
using gr::testing::OffDefault;
using gr::testing::requiredInitFailures;

[[nodiscard]] std::string failuresOf(const std::vector<std::string>& failures) { return std::format("{}", failures); }

template<typename TBlock>
void expectInitializes(gr::property_map constructed, std::string_view what) {
    boost::ut::expect(!gr::testing::initRefused<TBlock>(std::move(constructed))) << std::format("{} initializes", what);
}

[[nodiscard]] std::string reasonsOf(const gr::settings::Rejected& refused) {
    std::string reasons;
    for (const gr::settings::Rejection& entry : refused) {
        reasons += std::format("'{}': {}; ", entry.key, entry.reason);
    }
    return reasons;
}

[[nodiscard]] std::vector<std::string> keysOf(const gr::settings::Rejected& refused) {
    std::vector<std::string> keys;
    for (const gr::settings::Rejection& entry : refused) {
        keys.push_back(entry.key);
    }
    return keys;
}

} // namespace

namespace qa_fec_settings_init {

// emits `n_records` records of 50 information bits, then ends the stream
struct BitRecords : gr::Block<BitRecords> {
    gr::PortOut<gr::DataSet<std::uint8_t>, gr::Async> out;

    struct Settings {
        gr::Size_t n_records = 3U;

        GR_MAKE_REFLECTABLE(Settings, n_records);
    } settings;

    GR_MAKE_REFLECTABLE(BitRecords, out, settings);

    std::size_t      _sent = 0UZ;
    std::atomic<int> starts{0};

    void start() {
        ++starts;
        _sent = 0UZ;
    }

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), static_cast<std::size_t>(settings.n_records) - _sent);
        for (std::size_t i = 0UZ; i < n; ++i) {
            gr::DataSet<std::uint8_t> record;
            record.signal_values.assign(50UZ, std::uint8_t{1U});
            outSpan[i] = std::move(record);
        }
        outSpan.publish(n);
        _sent += n;
        return _sent >= settings.n_records ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

// counts the records it receives
struct RecordTally : gr::Block<RecordTally> {
    gr::PortIn<gr::DataSet<std::uint8_t>, gr::Async> in;

    GR_MAKE_REFLECTABLE(RecordTally, in);

    std::atomic<std::size_t> received{0UZ};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        received.fetch_add(inSpan.size(), std::memory_order_relaxed);
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

// an encoder built with nothing given, between a source and a sink, under a scheduler
struct EncoderChain {
    gr::scheduler::Simple<> scheduler;
    BitRecords*             source  = nullptr;
    ConvEncode*             encoder = nullptr;
    RecordTally*            sink    = nullptr;

    EncoderChain() {
        using boost::ut::expect;
        using boost::ut::fatal;
        gr::Graph flow;
        source  = std::addressof(flow.emplaceBlock<BitRecords>());
        encoder = std::addressof(flow.emplaceBlock<ConvEncode>());
        sink    = std::addressof(flow.emplaceBlock<RecordTally>());
        expect(fatal(flow.connect<"out", "in">(*source, *encoder).has_value()));
        expect(fatal(flow.connect<"out", "in">(*encoder, *sink).has_value()));
        expect(fatal(scheduler.exchange(std::move(flow)).has_value()));
    }
};

} // namespace qa_fec_settings_init

namespace {

[[nodiscard]] std::string encoderGraph(std::string_view code) {
    return std::format(R"yaml(blocks:
  - id: gr::blocks::fec::ConvEncode
    parameters:
      name: encoder
      code: {}
)yaml",
        code);
}

} // namespace

const boost::ut::suite<"fec settings at the initialization"> fecInitTests = [] {
    using namespace boost::ut;

    "the convolutional blocks"_test = [] {
        // a convolutional block has no code until one is given, so each is built with a whole code
        const gr::property_map named{{"code", std::string("ccsds")}, {"invert_outputs", gr::Size_t{2U}}};
        const gr::property_map explicitCode{{"constraint_length", gr::Size_t{7U}}, {"polynomials", std::vector<gr::Size_t>{121U, 91U}}, {"invert_outputs", gr::Size_t{1U}}};

        expectInitializes<ConvEncode>(named, "an encoder of a named code");
        expectInitializes<ConvEncode>(explicitCode, "an encoder of an explicit code");

        const gr::property_map decoding{{"code", std::string("ccsds")}, {"invert_outputs", gr::Size_t{2U}}, {"termination", std::string("open")}};
        expectInitializes<ViterbiDecode>(decoding, "a decoder of a named code, open");
        expectInitializes<ViterbiDecode>(explicitCode, "a decoder of an explicit code");
        expectInitializes<ViterbiDecodeSoft>(decoding, "a soft decoder of a named code, open");
        expectInitializes<ViterbiDecodeSoft>(explicitCode, "a soft decoder of an explicit code");
    };

    "the block codes"_test = [] {
        const std::vector<std::string> fecEncoder = requiredInitFailures<FecEncode>({{"code", std::string("golay24")}}, {});
        expect(fecEncoder.empty()) << failuresOf(fecEncoder);
        const std::vector<std::string> fecDecoder = requiredInitFailures<FecDecode>({{"code", std::string("golay24")}}, {});
        expect(fecDecoder.empty()) << failuresOf(fecDecoder);

        const std::vector<OffDefault>  reedSolomon{{"pad", gr::Size_t{4U}}, {"interleave", gr::Size_t{2U}}};
        const std::vector<std::string> rsEncoder = requiredInitFailures<RsEncode>({{"roots", gr::Size_t{16U}}}, reedSolomon);
        expect(rsEncoder.empty()) << failuresOf(rsEncoder);
        const std::vector<std::string> rsDecoder = requiredInitFailures<RsDecode>({{"roots", gr::Size_t{16U}}}, reedSolomon);
        expect(rsDecoder.empty()) << failuresOf(rsDecoder);
        expectInitializes<RsEncode>({{"code", std::string("ccsds_255_223")}, {"basis", std::string("dual")}, {"interleave", gr::Size_t{4U}}, {"pad", gr::Size_t{10U}}}, "a CCSDS Reed-Solomon encoder");
        expectInitializes<RsDecode>({{"code", std::string("ccsds_255_223")}, {"basis", std::string("dual")}, {"interleave", gr::Size_t{4U}}, {"pad", gr::Size_t{10U}}}, "a CCSDS Reed-Solomon decoder");

        // a GF(64) code refuses a basis: the family has no dual basis
        const auto expectBasisRefused = [](auto& block, std::string_view which) {
            const auto checked = block.check({});
            expect(fatal(!checked.has_value())) << std::format("{} takes a basis", which);
            expect(keysOf(checked.error()) == std::vector<std::string>{"basis"}) << std::format("{}: {}", which, reasonsOf(checked.error()));
            expect(reasonsOf(checked.error()).contains("must be unset under code gf64")) << std::format("{}: {}", which, reasonsOf(checked.error()));
        };
        RsEncode gf64Encoder({{"roots", gr::Size_t{16U}}, {"basis", std::string("dual")}});
        RsDecode gf64Decoder({{"roots", gr::Size_t{16U}}, {"basis", std::string("dual")}});
        expectBasisRefused(gf64Encoder, "a GF(64) encoder");
        expectBasisRefused(gf64Decoder, "a GF(64) decoder");
    };

    "the interleavers and the puncturers"_test = [] {
        const gr::property_map         permutation{{"kind", std::string("permutation")}, {"table", std::vector<gr::Size_t>{1U, 0U}}};
        const std::vector<OffDefault>  interleaving{{"rows", gr::Size_t{2U}}, {"cols", gr::Size_t{8U}}, {"branches", gr::Size_t{4U}}, {"unit_delay", gr::Size_t{2U}}, {"fill_value", gr::Size_t{7U}}};
        const std::vector<std::string> interleave = requiredInitFailures<Interleave<std::uint8_t>>(permutation, interleaving);
        expect(interleave.empty()) << failuresOf(interleave);
        const std::vector<std::string> deinterleave = requiredInitFailures<Deinterleave<std::uint8_t>>(permutation, interleaving);
        expect(deinterleave.empty()) << failuresOf(deinterleave);
        expectInitializes<Interleave<std::uint8_t>>({{"kind", std::string("block")}, {"rows", gr::Size_t{2U}}, {"cols", gr::Size_t{8U}}, {"window_offset", gr::Size_t{2U}}, {"window_length", gr::Size_t{8U}}}, "a windowed block interleaver");
        expectInitializes<Interleave<std::uint8_t>>({{"kind", std::string("convolutional")}, {"branches", gr::Size_t{4U}}, {"unit_delay", gr::Size_t{2U}}, {"fill_value", gr::Size_t{7U}}}, "a convolutional interleaver");
        expectInitializes<Interleave<std::uint8_t>>({{"kind", std::string("permutation")}, {"table", std::vector<gr::Size_t>{1U, 0U, 3U, 2U}}}, "a permutation interleaver");
        expectInitializes<Deinterleave<std::uint8_t>>({{"kind", std::string("block")}, {"rows", gr::Size_t{2U}}, {"cols", gr::Size_t{8U}}, {"output_offset", gr::Size_t{2U}}, {"output_length", gr::Size_t{8U}}}, "a windowed block deinterleaver");
        expectInitializes<Deinterleave<std::uint8_t>>({{"kind", std::string("permutation")}, {"table", std::vector<gr::Size_t>{1U, 0U, 3U, 2U}}}, "a permutation deinterleaver");
        expectInitializes<Deinterleave<std::uint8_t>>({{"kind", std::string("convolutional")}, {"branches", gr::Size_t{4U}}, {"unit_delay", gr::Size_t{2U}}}, "a convolutional deinterleaver");
        // a convolutional deinterleaver refuses an output window: the window addresses a frame
        for (const std::string_view key : {std::string_view("output_offset"), std::string_view("output_length")}) {
            Deinterleave<std::uint8_t> windowed({{"kind", std::string("convolutional")}, {"branches", gr::Size_t{4U}}, {"unit_delay", gr::Size_t{2U}}, {std::pmr::string(key), gr::Size_t{1U}}});
            const auto                 checked = windowed.check({});
            expect(fatal(!checked.has_value())) << std::format("a convolutional deinterleaver takes {}", key);
            expect(keysOf(checked.error()) == std::vector<std::string>{std::string(key)}) << reasonsOf(checked.error());
            expect(reasonsOf(checked.error()).contains("must be 0 when kind is convolutional")) << reasonsOf(checked.error());
        }

        const std::vector<std::string> puncture = requiredInitFailures<Puncture>({{"pattern", std::string("110")}}, {});
        expect(puncture.empty()) << failuresOf(puncture);
        const std::vector<std::string> depuncture = requiredInitFailures<Depuncture>({{"pattern", std::string("110")}}, {});
        expect(depuncture.empty()) << failuresOf(depuncture);
    };
};

const boost::ut::suite<"a convolutional block with no code"> notReadyTests = [] {
    using namespace boost::ut;

    "each block built with nothing given is in its graph and not ready, with the check's reason"_test = [] {
        gr::Graph          flow;
        ConvEncode&        encoder = flow.emplaceBlock<ConvEncode>();
        ViterbiDecode&     hard    = flow.emplaceBlock<ViterbiDecode>();
        ViterbiDecodeSoft& soft    = flow.emplaceBlock<ViterbiDecodeSoft>();
        expect(eq(flow.blocks().size(), 3UZ));
        const auto expectNotReady = [](const std::expected<void, gr::settings::Rejected>& ready, std::string_view which) {
            expect(fatal(!ready.has_value())) << std::format("{} is ready", which);
            expect(keysOf(ready.error()) == std::vector<std::string>{"code"}) << std::format("{}: {}", which, reasonsOf(ready.error()));
            expect(reasonsOf(ready.error()).contains("required")) << std::format("{}: {}", which, reasonsOf(ready.error()));
        };
        expectNotReady(encoder.ready(), "the encoder");
        expectNotReady(hard.ready(), "the hard decoder");
        expectNotReady(soft.ready(), "the soft decoder");
    };

    "a graph that holds the encoder not ready is refused at its start, naming it"_test = [] {
        qa_fec_settings_init::EncoderChain   chain;
        const std::expected<void, gr::Error> ran = chain.scheduler.runAndWait();
        expect(fatal(!ran.has_value())) << "a graph started with an encoder that has no code";
        expect(ran.error().message.contains(std::format("{}.code: required", chain.encoder->unique_name))) << ran.error().message;
        expect(eq(chain.source->starts.load(), 0)) << "a block started before every block was checked";
        expect(eq(chain.sink->received.load(), 0UZ));
    };

    "one set of a code name makes the encoder ready, and the graph starts"_test = [] {
        qa_fec_settings_init::EncoderChain chain;
        expect(fatal(chain.encoder->set(gr::property_map{{"code", std::string("ccsds")}}).has_value()));
        expect(chain.encoder->ready().has_value());
        expect(eq(chain.encoder->_code.constraintLength, 7UZ)) << "the apply hook built the code";

        const std::expected<void, gr::Error> ran = chain.scheduler.runAndWait();
        expect(fatal(ran.has_value())) << (ran ? std::string{} : ran.error().message);
        expect(eq(chain.sink->received.load(), 3UZ));
        expect(eq(chain.encoder->nRecords, std::uint64_t{3U}) && eq(chain.encoder->nInfoBits, std::uint64_t{150U}));
    };

    "a set of a constraint length alone is refused and keeps no value"_test = [] {
        gr::Graph   flow;
        ConvEncode& encoder = flow.emplaceBlock<ConvEncode>();
        const auto  refused = encoder.set(gr::property_map{{"constraint_length", gr::Size_t{7U}}});
        expect(fatal(!refused.has_value()));
        expect(keysOf(refused.error()) == std::vector<std::string>{"polynomials"}) << reasonsOf(refused.error());
        expect(reasonsOf(refused.error()).contains("must have >= 2 and <= 4 entries")) << reasonsOf(refused.error());
        expect(eq(encoder.settings.constraint_length.value, 0U));
        expect(eq(encoder.get<gr::Size_t>("constraint_length").value_or(1U), 0U)) << "a refused set left a value behind";
        expect(!encoder.ready().has_value());
    };

    "a graph file that gives a code name no convention has fails to load"_test = [] {
        gr::BlockRegistry& registry = gr::globalBlockRegistry();
        std::ignore                 = registry.insert<ConvEncode>();
        gr::PluginLoader& loader    = gr::globalPluginLoader();

        auto good = gr::loadGrc(loader, encoderGraph("ccsds"));
        expect(fatal(eq(good->blocks().size(), 1UZ)));
        expect(good->blocks().front()->ready().has_value()) << "a named convention loads ready";

        std::string reason;
        try {
            std::ignore = gr::loadGrc(loader, encoderGraph("voyager"));
        } catch (const std::exception& refused) {
            reason = refused.what();
        }
        expect(reason.contains("code: must be one of: custom, ccsds, ccsds_uninverted, nasa_dsn, nasa_dsn_uninverted")) << reason;
    };
};

int main() { /* not needed for ut */ }
