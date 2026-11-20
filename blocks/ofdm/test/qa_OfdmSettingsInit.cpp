#include <boost/ut.hpp>

#include <cstddef>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/ofdm/CarrierAllocator.hpp>
#include <gnuradio-4.0/ofdm/ChannelEqualizer.hpp>
#include <gnuradio-4.0/ofdm/CyclicPrefix.hpp>
#include <gnuradio-4.0/ofdm/SchmidlCoxSync.hpp>

#include <gnuradio-4.0/testing/InitChecks.hpp>

// The ofdm blocks at their defaults and with each setting off its default, initialized as a graph's addition does. A
// setting that only has meaning beside another (the carriers of a numerology and its pilots and sync words) is checked
// in a whole configuration that names them together. The allocator and the equalizer have no data carriers by default
// and are not ready until a host gives them.

namespace {

using namespace gr::blocks::ofdm;
using gr::testing::requiredInitFailures;

[[nodiscard]] std::string failuresOf(const std::vector<std::string>& failures) { return std::format("{}", failures); }

template<typename TBlock>
void expectInitializes(gr::property_map constructed, std::string_view what) {
    boost::ut::expect(!gr::testing::initRefused<TBlock>(std::move(constructed))) << std::format("{} initializes", what);
}

[[nodiscard]] std::vector<std::int32_t> dataCarriers() { return {-2, -1, 1, 2}; }
[[nodiscard]] std::vector<std::int32_t> pilotCarriers() { return {-3, 3}; }

/// One symbol of @p bins bins, interleaved re,im, that carries 1 on every bin.
[[nodiscard]] std::vector<float> flatSymbol(std::size_t bins = 64UZ) {
    std::vector<float> word(2UZ * bins, 0.f);
    for (std::size_t bin = 0UZ; bin < bins; ++bin) {
        word[2UZ * bin] = 1.f;
    }
    return word;
}

} // namespace

const boost::ut::suite<"ofdm settings at the initialization"> ofdmInitTests = [] {
    using namespace boost::ut;

    "CarrierAllocator"_test = [] {
        const gr::property_map         numerology{{"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", std::vector<float>{1.f, 0.f}}};
        const std::vector<std::string> failures = requiredInitFailures<CarrierAllocator>(numerology, {
                                                                                                         {"fft_len", gr::Size_t{128U}},
                                                                                                         {"frame_len", gr::Size_t{2U}},
                                                                                                         {"signal_name", std::string("symbol")},
                                                                                                     });
        expect(failures.empty()) << failuresOf(failures);
        expectInitializes<CarrierAllocator>({{"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", std::vector<float>{1.f, 0.f, -1.f, 0.f}}, {"frame_len", gr::Size_t{1U}}, {"sync_words", flatSymbol()}}, "a framed allocator with pilots and a sync word");
    };

    "ChannelEqualizer"_test = [] {
        const gr::property_map         numerology{{"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", std::vector<float>{1.f, 0.f}}, {"sync_word", flatSymbol()}};
        const std::vector<std::string> failures = requiredInitFailures<ChannelEqualizer>(numerology, {
                                                                                                         {"n_sync", gr::Size_t{2U}},
                                                                                                         {"tracking", std::string("none")},
                                                                                                         {"alpha", 0.2f},
                                                                                                         {"signal_name", std::string("equalized")},
                                                                                                         {"constellation", std::string("bpsk")},
                                                                                                         {"arity", gr::Size_t{16U}},
                                                                                                         {"phase_offset", 0.1f},
                                                                                                         {"label_xor", gr::Size_t{1U}},
                                                                                                         {"normalization", std::string("amplitude")},
                                                                                                     });
        expect(failures.empty()) << failuresOf(failures);
        expectInitializes<ChannelEqualizer>({{"fft_len", gr::Size_t{128U}}, {"data_carriers", dataCarriers()}, {"sync_word", flatSymbol(128UZ)}, {"tracking", std::string("none")}}, "an equalizer of 128 bins");
        expectInitializes<ChannelEqualizer>({{"data_carriers", dataCarriers()}, {"pilot_carriers", pilotCarriers()}, {"pilot_symbols", std::vector<float>{1.f, 0.f, -1.f, 0.f}}, {"sync_word", flatSymbol()}, {"n_sync", gr::Size_t{2U}}, {"sync_index", gr::Size_t{1U}}, {"tracking", std::string("cpe_interp")}}, "an equalizer with pilots and a sync word");
    };

    "CpInsert and CpRemove"_test = [] {
        const std::vector<std::string> insert = gr::testing::initFailures<CpInsert>({
            {"cp_len", std::vector<gr::Size_t>{8U}},
            {"window_len", gr::Size_t{4U}},
            {"emit_trigger", false},
            {"trigger_label", std::string("frame")},
        });
        expect(insert.empty()) << failuresOf(insert);
        const std::vector<std::string> remove = gr::testing::initFailures<CpRemove>({
            {"fft_len", gr::Size_t{128U}},
            {"cp_len", std::vector<gr::Size_t>{8U, 4U}},
            {"n_sync", gr::Size_t{1U}},
            {"frame_len", gr::Size_t{4U}},
            {"timing_offset", std::int32_t{-2}},
            {"trigger_label", std::string("frame")},
            {"signal_name", std::string("symbol")},
        });
        expect(remove.empty()) << failuresOf(remove);
    };

    "SchmidlCoxSync"_test = [] {
        const std::vector<std::string> failures = gr::testing::initFailures<SchmidlCoxSync>({
            {"fft_len", gr::Size_t{128U}},
            {"cp_len", gr::Size_t{8U}},
            {"threshold", 0.5f},
            {"min_gap", gr::Size_t{4U}},
            {"correct_cfo", false},
            {"r_floor", 1e-6f},
            {"trigger_label", std::string("frame")},
        });
        expect(failures.empty()) << failuresOf(failures);
    };
};

int main() { /* not needed for ut */ }
