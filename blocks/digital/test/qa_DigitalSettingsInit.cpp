#include <boost/ut.hpp>

#include <cstdint>
#include <format>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/BitPacking.hpp>
#include <gnuradio-4.0/digital/ConstellationBlocks.hpp>
#include <gnuradio-4.0/digital/CpmModulate.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>
#include <gnuradio-4.0/digital/DifferentialCoding.hpp>
#include <gnuradio-4.0/digital/LengthHeaderFramer.hpp>
#include <gnuradio-4.0/digital/LinearEqualizer.hpp>
#include <gnuradio-4.0/digital/Manchester.hpp>
#include <gnuradio-4.0/digital/ManchesterCombine.hpp>
#include <gnuradio-4.0/digital/PacketFramer.hpp>
#include <gnuradio-4.0/digital/PamSymbols.hpp>
#include <gnuradio-4.0/digital/PreambleCorrelator.hpp>
#include <gnuradio-4.0/digital/Scrambler.hpp>
#include <gnuradio-4.0/digital/SignalQuality.hpp>

#include <gnuradio-4.0/testing/InitChecks.hpp>

// Each block is initialized at its defaults and once per setting at a value other than its default, as a graph that
// adds the block does: the settings hook runs then with every setting changed. A setting that only has meaning beside
// another (a header format's field, a transparency's parameters) is left out of the list, and is checked in a whole
// configuration that names both.
// A block with a required setting that has no default is not ready at its defaults, and is checked beside that setting.

namespace {

using namespace gr::blocks::digital;
using gr::testing::OffDefault;

[[nodiscard]] std::string failuresOf(const std::vector<std::string>& failures) { return std::format("{}", failures); }

/// The settings that name a constellation, each off its default, in the spellings every block that carries them takes.
[[nodiscard]] std::vector<OffDefault> constellationSettings() {
    return {
        {"constellation", std::string("bpsk")},
        {"arity", gr::Size_t{16U}},
        {"phase_offset", 0.1f},
        {"label_xor", gr::Size_t{1U}},
        {"points", std::vector<float>{1.f, 0.f, -1.f, 0.f}},
        {"normalization", std::string("Amplitude")},
    };
}

[[nodiscard]] std::vector<OffDefault> with(std::vector<OffDefault> settings, const std::vector<OffDefault>& more) {
    settings.insert(settings.end(), more.begin(), more.end());
    return settings;
}

template<typename TBlock>
void expectInitializes(gr::property_map constructed, std::string_view what) {
    boost::ut::expect(!gr::testing::initRefused<TBlock>(std::move(constructed))) << std::format("{} initializes", what);
}

} // namespace

const boost::ut::suite<"digital settings at the initialization"> digitalInitTests = [] {
    using namespace boost::ut;

    "the correlators and the framers"_test = [] {
        const std::vector<std::string> correlator = gr::testing::requiredInitFailures<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("01011011")}}, {
                                                                                                                                                                          {"access_code", std::string("01011011")},
                                                                                                                                                                          {"max_errors", gr::Size_t{1U}},
                                                                                                                                                                          {"trigger_label", std::string("sync")},
                                                                                                                                                                          {"bits_per_item", gr::Size_t{2U}},
                                                                                                                                                                          {"code_stride", gr::Size_t{2U}},
                                                                                                                                                                          {"tag_at", std::string("code_start")},
                                                                                                                                                                      });
        expect(correlator.empty()) << failuresOf(correlator);
        expectInitializes<AccessCodeCorrelator<std::uint8_t>>({{"access_code", std::string("0101101100110110")}, {"bits_per_item", gr::Size_t{2U}}, {"max_errors", gr::Size_t{1U}}, {"code_stride", gr::Size_t{3U}}, {"tag_at", std::string("code_start")}}, "a correlator of 2-bit symbols at stride 3");

        const std::vector<std::string> preamble = gr::testing::requiredInitFailures<PreambleCorrelator<float>>({{"sequence", std::vector<float>{1.f, 0.f, 0.f, 1.f}}, {"threshold", 1.5f}}, {
                                                                                                                                                                                                {"sequence", std::vector<float>{1.f, 0.f, 0.f, 1.f, -1.f, 0.f, 0.f, -1.f}},
                                                                                                                                                                                                {"threshold", 0.5f},
                                                                                                                                                                                                {"threshold_mode", std::string("relative")},
                                                                                                                                                                                                {"samples_per_symbol", 2.0f},
                                                                                                                                                                                                {"trigger_label", std::string("sync")},
                                                                                                                                                                                            });
        expect(preamble.empty()) << failuresOf(preamble);

        const std::vector<std::string> packet = gr::testing::requiredInitFailures<PacketFramer<std::uint8_t>>({{"max_payload_items", gr::Size_t{64U}}}, {
                                                                                                                                                            {"max_payload_items", gr::Size_t{64U}},
                                                                                                                                                            {"header_format", std::string("length_repeated")},
                                                                                                                                                            {"trigger_to_header", gr::Size_t{4U}},
                                                                                                                                                            {"trigger", std::string("sync")},
                                                                                                                                                        });
        expect(packet.empty()) << failuresOf(packet);
        expectInitializes<PacketFramer<std::uint8_t>>({{"max_payload_items", gr::Size_t{64U}}, {"header_format", std::string("fixed_length")}, {"fixed_payload_items", gr::Size_t{8U}}}, "a fixed-length packet framer");
        expectInitializes<PacketFramer<std::uint8_t>>({{"max_payload_items", gr::Size_t{64U}}, {"header_format", std::string("length_plain")}, {"length_bits", gr::Size_t{16U}}, {"byte_order", std::string("Little")}, {"count_covers", std::string("Payload")}, {"count_unit_items", gr::Size_t{1U}}, {"check_items", gr::Size_t{2U}}}, "a plain-length packet framer");
        expectInitializes<PacketFramer<std::uint8_t>>({{"max_payload_items", gr::Size_t{64U}}, {"header_format", std::string("length_golay24")}, {"length_bits", gr::Size_t{12U}}, {"flags_position", std::string("Low")}, {"count_covers", std::string("WholeFrame")}, {"frame_prefix_items", gr::Size_t{4U}}}, "a Golay-header packet framer");

        const std::vector<std::string> length = gr::testing::requiredInitFailures<LengthHeaderFramer>({{"max_payload_items", gr::Size_t{64U}}}, {
                                                                                                                                                    {"sync_word", std::string("0101")},
                                                                                                                                                    {"header_format", std::string("length_repeated")},
                                                                                                                                                    {"max_payload_items", gr::Size_t{64U}},
                                                                                                                                                    {"payload_pack_bits", gr::Size_t{8U}},
                                                                                                                                                    {"payload_bit_order", std::string("LsbFirst")},
                                                                                                                                                });
        expect(length.empty()) << failuresOf(length);
        expectInitializes<LengthHeaderFramer>({{"header_format", std::string("fixed_length")}, {"fixed_payload_items", gr::Size_t{8U}}, {"max_payload_items", gr::Size_t{64U}}}, "a fixed-length framer");
        expectInitializes<LengthHeaderFramer>({{"header_format", std::string("length_plain")}, {"length_bits", gr::Size_t{16U}}, {"byte_order", std::string("Little")}, {"count_covers", std::string("Payload")}, {"count_unit_items", gr::Size_t{1U}}, {"check_items", gr::Size_t{2U}}, {"max_payload_items", gr::Size_t{64U}}}, "a plain-length framer");
        expectInitializes<LengthHeaderFramer>({{"header_format", std::string("length_golay24")}, {"length_bits", gr::Size_t{12U}}, {"flags_position", std::string("Low")}, {"count_covers", std::string("WholeFrame")}, {"frame_prefix_items", gr::Size_t{4U}}, {"max_payload_items", gr::Size_t{64U}}}, "a Golay-header framer");
    };

    "the delimiter blocks"_test = [] {
        const std::vector<std::string> extractor = gr::testing::requiredInitFailures<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{64U}}}, {
                                                                                                                                                                                                                 {"end_delimiter", std::string("01111110")},
                                                                                                                                                                                                                 {"frame_open", std::string("Immediate")},
                                                                                                                                                                                                                 {"bits_per_item", gr::Size_t{2U}},
                                                                                                                                                                                                                 {"bit_order", std::string("LsbFirst")},
                                                                                                                                                                                                                 {"transparency", std::string("BitStuffing")},
                                                                                                                                                                                                                 {"max_payload_items", gr::Size_t{64U}},
                                                                                                                                                                                                                 {"min_payload_items", gr::Size_t{2U}},
                                                                                                                                                                                                                 {"payload_pack_bits", gr::Size_t{8U}},
                                                                                                                                                                                                                 {"payload_bit_order", std::string("LsbFirst")},
                                                                                                                                                                                                                 {"trigger", std::string("sync")},
                                                                                                                                                                                                                 {"trigger_resets", false},
                                                                                                                                                                                                                 {"frame_label", std::string("frame")},
                                                                                                                                                                                                                 {"start_delimiter", std::string("0111")},
                                                                                                                                                                                                                 {"stuff_after_ones", gr::Size_t{4U}},
                                                                                                                                                                                                                 {"abort_ones", gr::Size_t{8U}},
                                                                                                                                                                                                                 {"escape_item", gr::Size_t{0xDBU}},
                                                                                                                                                                                                                 {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}},
                                                                                                                                                                                                             });
        expect(extractor.empty()) << failuresOf(extractor);
        expectInitializes<DelimiterExtractor<std::uint8_t>>({{"frame_open", std::string("Start")}, {"start_delimiter", std::string("0111")}, {"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{64U}}}, "an extractor with a start delimiter");
        expectInitializes<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("BitStuffing")}, {"stuff_after_ones", gr::Size_t{4U}}, {"abort_ones", gr::Size_t{8U}}, {"max_payload_items", gr::Size_t{64U}}}, "an extractor's bit stuffing");
        expectInitializes<DelimiterExtractor<std::uint8_t>>({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8U}}, {"transparency", std::string("ByteEscape")}, {"escape_item", gr::Size_t{0xDBU}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", gr::Size_t{64U}}}, "an extractor's byte escaping");

        const std::vector<std::string> framer = gr::testing::requiredInitFailures<DelimiterFramer>({{"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{64U}}}, {
                                                                                                                                                                                             {"end_delimiter", std::string("01111110")},
                                                                                                                                                                                             {"frame_open", std::string("Immediate")},
                                                                                                                                                                                             {"bits_per_item", gr::Size_t{2U}},
                                                                                                                                                                                             {"bit_order", std::string("LsbFirst")},
                                                                                                                                                                                             {"transparency", std::string("BitStuffing")},
                                                                                                                                                                                             {"max_payload_items", gr::Size_t{64U}},
                                                                                                                                                                                             {"payload_pack_bits", gr::Size_t{8U}},
                                                                                                                                                                                             {"payload_bit_order", std::string("LsbFirst")},
                                                                                                                                                                                             {"start_delimiter", std::string("0111")},
                                                                                                                                                                                             {"stuff_after_ones", gr::Size_t{4U}},
                                                                                                                                                                                             {"abort_ones", gr::Size_t{8U}},
                                                                                                                                                                                             {"escape_item", gr::Size_t{0xDBU}},
                                                                                                                                                                                             {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}},
                                                                                                                                                                                         });
        expect(framer.empty()) << failuresOf(framer);
        expectInitializes<DelimiterFramer>({{"frame_open", std::string("Start")}, {"start_delimiter", std::string("0111")}, {"end_delimiter", std::string("01111110")}, {"max_payload_items", gr::Size_t{64U}}}, "a framer with a start delimiter");
        expectInitializes<DelimiterFramer>({{"end_delimiter", std::string("01111110")}, {"transparency", std::string("BitStuffing")}, {"stuff_after_ones", gr::Size_t{4U}}, {"abort_ones", gr::Size_t{8U}}, {"max_payload_items", gr::Size_t{64U}}}, "a framer's bit stuffing");
        expectInitializes<DelimiterFramer>({{"end_delimiter", std::string("11000000")}, {"bits_per_item", gr::Size_t{8U}}, {"transparency", std::string("ByteEscape")}, {"escape_item", gr::Size_t{0xDBU}}, {"escape_map", std::vector<gr::Size_t>{0xDCU, 0xC0U, 0xDDU, 0xDBU}}, {"max_payload_items", gr::Size_t{64U}}}, "a framer's byte escaping");
    };

    "the bit and symbol blocks"_test = [] {
        const std::vector<OffDefault> packing{{"k", gr::Size_t{4U}}, {"bit_order", std::string("LsbFirst")}, {"flush_partial", false}};
        expect(gr::testing::initFailures<PackBits>(packing).empty());
        expect(gr::testing::initFailures<UnpackBits>(packing).empty());
        const std::vector<OffDefault> repacking{{"bits_in", gr::Size_t{2U}}, {"bits_out", gr::Size_t{4U}}, {"input_bit_order", std::string("LsbFirst")}, {"output_bit_order", std::string("LsbFirst")}};
        expect(gr::testing::initFailures<RepackBits>(with(repacking, {{"flush_partial", false}})).empty());
        expect(gr::testing::initFailures<RecordRepackBits>(repacking).empty());

        const std::vector<OffDefault> coding{{"modulus", gr::Size_t{4U}}, {"coding", std::string("Nrzi")}, {"initial_state", gr::Size_t{1U}}};
        expect(gr::testing::initFailures<DifferentialEncoder<std::uint8_t>>(coding).empty());
        expect(gr::testing::initFailures<DifferentialDecoder<std::uint8_t>>(coding).empty());
        expect(gr::testing::initFailures<DifferentialPhasor<float>>({}).empty());

        expect(gr::testing::requiredInitFailures<ManchesterEncoder>({{"convention", std::string("Ieee8023")}}, {{"convention", std::string("GeThomas")}, {"bits_per_item", gr::Size_t{4U}}, {"bit_order", std::string("LsbFirst")}}).empty());
        const std::vector<std::string> decoder = gr::testing::requiredInitFailures<ManchesterDecoder>({{"convention", std::string("Ieee8023")}}, {
                                                                                                                                                      {"convention", std::string("GeThomas")},
                                                                                                                                                      {"bits_per_item", gr::Size_t{4U}},
                                                                                                                                                      {"bit_order", std::string("LsbFirst")},
                                                                                                                                                      {"chip_phase", gr::Size_t{1U}},
                                                                                                                                                      {"align_tag_key", std::string("sync")},
                                                                                                                                                      {"align_tag_value", std::string("frame")},
                                                                                                                                                      {"flush_partial", false},
                                                                                                                                                  });
        expect(decoder.empty()) << failuresOf(decoder);
        expect(gr::testing::initFailures<ManchesterCombine>({{"input", std::string("Chips")}, {"averaging_symbols", gr::Size_t{128U}}, {"hold_symbols", gr::Size_t{32U}}, {"phase", std::string("odd")}}).empty());

        expect(gr::testing::initFailures<LevelTracker<float>>({{"n_levels", gr::Size_t{8U}}, {"spread_rate", 0.02}, {"offset_rate", 0.2}, {"spread_limit", 0.1}}).empty());
        expect(gr::testing::initFailures<PamSlicer<float>>({{"n_levels", gr::Size_t{8U}}, {"labels", std::vector<gr::Size_t>{3U, 2U, 0U, 1U}}}).empty());
        expect(gr::testing::initRefused<PamSlicer<float>>({{"n_levels", gr::Size_t{3U}}})) << "an odd level count is refused at the initialization";
    };

    "the scramblers"_test = [] {
        const std::vector<std::string> additive = gr::testing::requiredInitFailures<AdditiveScrambler<std::uint8_t>>({{"taps", std::string("1,3,5,8")}, {"seed", std::string("11111111")}}, {
                                                                                                                                                                                                {"sequence_repeat", false},
                                                                                                                                                                                                {"bits_per_item", gr::Size_t{1U}},
                                                                                                                                                                                                {"bit_order", std::string("LsbFirst")},
                                                                                                                                                                                                {"reset_tag_key", std::string("sync")},
                                                                                                                                                                                                {"reset_tag_value", std::string("frame")},
                                                                                                                                                                                                {"reset_period", gr::Size_t{100U}},
                                                                                                                                                                                            });
        expect(additive.empty()) << failuresOf(additive);
        expectInitializes<AdditiveScrambler<std::uint8_t>>({{"profile", std::string("ccsds131")}}, "an additive scrambler by its profile");
        expectInitializes<AdditiveScrambler<std::uint8_t>>({{"sequence", std::string("FF E1 1D 9A")}}, "an additive scrambler by its sequence");

        const gr::property_map        taps{{"taps", std::string("18,23")}};
        const std::vector<OffDefault> multiplicative{
            {"seed", std::string("00000000000000000000001")},
            {"bits_per_item", gr::Size_t{1U}},
            {"bit_order", std::string("LsbFirst")},
            {"invert_output", true},
            {"reset_tag_key", std::string("sync")},
            {"reset_tag_value", std::string("frame")},
            {"reset_period", gr::Size_t{100U}},
        };
        const std::vector<std::string> scrambler = gr::testing::requiredInitFailures<MultiplicativeScrambler>(taps, multiplicative);
        expect(scrambler.empty()) << failuresOf(scrambler);
        const std::vector<std::string> descrambler = gr::testing::requiredInitFailures<MultiplicativeDescrambler>(taps, multiplicative);
        expect(descrambler.empty()) << failuresOf(descrambler);
        expectInitializes<MultiplicativeScrambler>({{"profile", std::string("g3ruh")}}, "a multiplicative scrambler by its profile");
        expectInitializes<MultiplicativeScrambler>({{"taps", std::string("18,23")}, {"force_transition_after", gr::Size_t{32U}}, {"monitor_delays", std::string("1,23")}}, "a multiplicative scrambler with a forced transition");
        expectInitializes<MultiplicativeDescrambler>({{"taps", std::string("18,23")}, {"force_transition_after", gr::Size_t{32U}}, {"monitor_delays", std::string("1,23")}}, "a multiplicative descrambler with a forced transition");
    };

    "the constellation, modulation and measurement blocks"_test = [] {
        const std::vector<std::string> decoder = gr::testing::initFailures<ConstellationDecoder<float>>(constellationSettings());
        expect(decoder.empty()) << failuresOf(decoder);
        const std::vector<std::string> encoder = gr::testing::initFailures<ConstellationEncoder<float>>(constellationSettings());
        expect(encoder.empty()) << failuresOf(encoder);
        const std::vector<std::string> soft = gr::testing::initFailures<ConstellationSoftDecoder<float>>(with(constellationSettings(), {{"noise_power", 0.5f}, {"soft_algorithm", std::string("Exact")}}));
        expect(soft.empty()) << failuresOf(soft);
        expect(gr::testing::initFailures<SymbolMap>({{"map", std::vector<std::uint8_t>{1U, 0U}}}).empty());

        const std::vector<std::string> cpm = gr::testing::initFailures<CpmModulate<float>>({
            {"modulation_index", 1.0},
            {"samples_per_symbol", gr::Size_t{4U}},
            {"pulse", std::string("RaisedCosine")},
            {"pulse_length", gr::Size_t{2U}},
            {"bt", 0.5},
            {"amplitude", 2.0f},
        });
        expect(cpm.empty()) << failuresOf(cpm);

        const std::vector<std::string> equalizer = gr::testing::initFailures<LinearEqualizer<float>>(with(constellationSettings(), {
                                                                                                                                       {"num_taps", gr::Size_t{7U}},
                                                                                                                                       {"algorithm", std::string("Nlms")},
                                                                                                                                       {"step_size", 0.05},
                                                                                                                                       {"samples_per_symbol", gr::Size_t{2U}},
                                                                                                                                       {"training_sequence", std::vector<float>{1.f, 0.f, -1.f, 0.f}},
                                                                                                                                       {"freeze", true},
                                                                                                                                       {"divergence_bound", 50.0},
                                                                                                                                   }));
        expect(equalizer.empty()) << failuresOf(equalizer);
        expectInitializes<LinearEqualizer<float>>({{"reference", std::string("training")}, {"training_sequence", std::vector<float>{1.f, 0.f, -1.f, 0.f}}}, "an equalizer trained on a sequence");

        const std::vector<std::string> evm = gr::testing::initFailures<EvmMeter<float>>(with(constellationSettings(), {{"window", gr::Size_t{500U}}}));
        expect(evm.empty()) << failuresOf(evm);
        const std::vector<std::string> snr = gr::testing::initFailures<SnrEstimator<float>>(with(constellationSettings(), {{"method", std::string("decision_directed")}, {"window", gr::Size_t{1024U}}, {"ceiling_db", 50.0}}));
        expect(snr.empty()) << failuresOf(snr);

        const std::vector<OffDefault> crc{
            {"width", gr::Size_t{64U}},
            {"poly", std::uint64_t{0x1EDC6F41ULL}},
            {"initial_value", std::uint64_t{0ULL}},
            {"final_xor", std::uint64_t{0ULL}},
            {"input_reflected", false},
            {"result_reflected", false},
            {"crc_byte_order", std::string("Little")},
            {"skip_header_bytes", gr::Size_t{2U}},
        };
        const std::vector<std::string> append = gr::testing::initFailures<CrcAppend>(crc);
        expect(append.empty()) << failuresOf(append);
        const std::vector<std::string> check = gr::testing::initFailures<CrcCheck>(with(crc, {{"discard_crc", true}}));
        expect(check.empty()) << failuresOf(check);
    };
};

int main() { /* not needed for ut */ }
