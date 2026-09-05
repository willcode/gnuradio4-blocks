// GENERATED FILE — do not edit. Source of truth: blocks/recipes/BpskDemod.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_BPSKDEMOD_HPP
#define GNURADIO_RECIPES_BPSKDEMOD_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct BpskDemod {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(float sample_rate_, float symbol_rate_) : sample_rate(std::move(sample_rate_)), symbol_rate(std::move(symbol_rate_)) {}
        float sample_rate; // input sample rate in hertz; required
        float symbol_rate; // symbol rate in hertz; required
        double frequency_offset = 0.0; // where the carrier sits relative to the front end's center, in hertz
        std::uint32_t decimation = std::uint32_t{1}; // decimation in the channel filter; pick it for about four samples per symbol after it
        double channel_bandwidth = 0.75; // cutoff of the channel filter as a multiple of the symbol rate
        double rolloff = 0.35; // excess bandwidth of the transmit shaping filter, in [0, 1]
        double fll_noise_bandwidth = 0.01; // closed-loop noise bandwidth of the frequency-locked loop, normalized: Bn*T per sample
        std::uint32_t fll_filter_length = std::uint32_t{45}; // taps in each band-edge filter
        std::string timing_detector = std::string("gardner"); // timing error detector; it runs BEFORE carrier recovery, so it must be one that needs no decision: gardner, zero_crossing or early_late
        double timing_noise_bandwidth = 0.002; // closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate
        double costas_noise_bandwidth = 0.01; // closed-loop noise bandwidth of the carrier phase loop, normalized to the symbol rate
        double agc_reference_db = 0.0; // target output level of the front end's AGC, in decibels
        double agc_attack_symbols = 256.0; // gain-control time constant when the gain must decrease, in SYMBOL PERIODS; the recipe divides by symbol_rate, because a gain loop reading the instantaneous magnitude follows the modulation's own envelope unless its constant is long against a symbol
        double agc_decay_symbols = 512.0; // gain-control time constant when the gain must increase, in SYMBOL PERIODS; twice the attack, so a level that rises is followed half as fast as one that falls
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::BpskDemod";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("input sample rate in hertz; required"));
            m5[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m5[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("sample_rate"));
            e4 = gr::pmt::Value(std::move(m5));
            t3.push_back(std::move(e4));
            gr::pmt::Value e6;
            gr::property_map m7;
            m7[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("symbol rate in hertz; required"));
            m7[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float32"));
            m7[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("symbol_rate"));
            e6 = gr::pmt::Value(std::move(m7));
            t3.push_back(std::move(e6));
            gr::pmt::Value e8;
            gr::property_map m9;
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("where the carrier sits relative to the front end's center, in hertz"));
            m9[std::pmr::string("default")] = gr::pmt::Value(0.0);
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("frequency_offset"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("decimation in the channel filter; pick it for about four samples per symbol after it"));
            m11[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("decimation"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("cutoff of the channel filter as a multiple of the symbol rate"));
            m13[std::pmr::string("default")] = gr::pmt::Value(0.75);
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel_bandwidth"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("excess bandwidth of the transmit shaping filter, in [0, 1]"));
            m15[std::pmr::string("default")] = gr::pmt::Value(0.35);
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("rolloff"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            gr::pmt::Value e16;
            gr::property_map m17;
            m17[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("closed-loop noise bandwidth of the frequency-locked loop, normalized: Bn*T per sample"));
            m17[std::pmr::string("default")] = gr::pmt::Value(0.01);
            m17[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m17[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("fll_noise_bandwidth"));
            e16 = gr::pmt::Value(std::move(m17));
            t3.push_back(std::move(e16));
            gr::pmt::Value e18;
            gr::property_map m19;
            m19[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("taps in each band-edge filter"));
            m19[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{45});
            m19[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m19[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("fll_filter_length"));
            e18 = gr::pmt::Value(std::move(m19));
            t3.push_back(std::move(e18));
            gr::pmt::Value e20;
            gr::property_map m21;
            m21[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("timing error detector; it runs BEFORE carrier recovery, so it must be one that needs no decision: gardner, zero_crossing or early_late"));
            m21[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("gardner"));
            m21[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m21[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing_detector"));
            e20 = gr::pmt::Value(std::move(m21));
            t3.push_back(std::move(e20));
            gr::pmt::Value e22;
            gr::property_map m23;
            m23[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate"));
            m23[std::pmr::string("default")] = gr::pmt::Value(0.002);
            m23[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m23[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing_noise_bandwidth"));
            e22 = gr::pmt::Value(std::move(m23));
            t3.push_back(std::move(e22));
            gr::pmt::Value e24;
            gr::property_map m25;
            m25[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("closed-loop noise bandwidth of the carrier phase loop, normalized to the symbol rate"));
            m25[std::pmr::string("default")] = gr::pmt::Value(0.01);
            m25[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m25[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("costas_noise_bandwidth"));
            e24 = gr::pmt::Value(std::move(m25));
            t3.push_back(std::move(e24));
            gr::pmt::Value e26;
            gr::property_map m27;
            m27[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("target output level of the front end's AGC, in decibels"));
            m27[std::pmr::string("default")] = gr::pmt::Value(0.0);
            m27[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m27[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("agc_reference_db"));
            e26 = gr::pmt::Value(std::move(m27));
            t3.push_back(std::move(e26));
            gr::pmt::Value e28;
            gr::property_map m29;
            m29[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("gain-control time constant when the gain must decrease, in SYMBOL PERIODS; the recipe divides by symbol_rate, because a gain loop reading the instantaneous magnitude follows the modulation's own envelope unless its constant is long against a symbol"));
            m29[std::pmr::string("default")] = gr::pmt::Value(256.0);
            m29[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m29[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("agc_attack_symbols"));
            e28 = gr::pmt::Value(std::move(m29));
            t3.push_back(std::move(e28));
            gr::pmt::Value e30;
            gr::property_map m31;
            m31[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("gain-control time constant when the gain must increase, in SYMBOL PERIODS; twice the attack, so a level that rises is followed half as fast as one that falls"));
            m31[std::pmr::string("default")] = gr::pmt::Value(512.0);
            m31[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m31[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("agc_decay_symbols"));
            e30 = gr::pmt::Value(std::move(m31));
            t3.push_back(std::move(e30));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m32;
            m32[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("bpsk_demod"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m32));
            gr::property_map m33;
            gr::Tensor<gr::pmt::Value> t34;
            gr::pmt::Value e35;
            gr::Tensor<gr::pmt::Value> t36;
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::pmr::string("translate"));
            t36.push_back(std::move(e37));
            gr::pmt::Value e38;
            e38 = gr::pmt::Value(std::pmr::string("INPUT"));
            t36.push_back(std::move(e38));
            gr::pmt::Value e39;
            e39 = gr::pmt::Value(std::pmr::string("in"));
            t36.push_back(std::move(e39));
            gr::pmt::Value e40;
            e40 = gr::pmt::Value(std::pmr::string("in"));
            t36.push_back(std::move(e40));
            e35 = gr::pmt::Value(std::move(t36));
            t34.push_back(std::move(e35));
            gr::pmt::Value e41;
            gr::Tensor<gr::pmt::Value> t42;
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::pmr::string("decision"));
            t42.push_back(std::move(e43));
            gr::pmt::Value e44;
            e44 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t42.push_back(std::move(e44));
            gr::pmt::Value e45;
            e45 = gr::pmt::Value(std::pmr::string("real"));
            t42.push_back(std::move(e45));
            gr::pmt::Value e46;
            e46 = gr::pmt::Value(std::pmr::string("out"));
            t42.push_back(std::move(e46));
            e41 = gr::pmt::Value(std::move(t42));
            t34.push_back(std::move(e41));
            m33[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t34));
            gr::Tensor<gr::pmt::Value> t47;
            gr::pmt::Value e48;
            gr::Tensor<gr::pmt::Value> t49;
            gr::pmt::Value e50;
            e50 = gr::pmt::Value(std::pmr::string("translate"));
            t49.push_back(std::move(e50));
            gr::pmt::Value e51;
            e51 = gr::pmt::Value(std::int64_t{0});
            t49.push_back(std::move(e51));
            gr::pmt::Value e52;
            e52 = gr::pmt::Value(std::pmr::string("channel"));
            t49.push_back(std::move(e52));
            gr::pmt::Value e53;
            e53 = gr::pmt::Value(std::int64_t{0});
            t49.push_back(std::move(e53));
            e48 = gr::pmt::Value(std::move(t49));
            t47.push_back(std::move(e48));
            gr::pmt::Value e54;
            gr::Tensor<gr::pmt::Value> t55;
            gr::pmt::Value e56;
            e56 = gr::pmt::Value(std::pmr::string("channel"));
            t55.push_back(std::move(e56));
            gr::pmt::Value e57;
            e57 = gr::pmt::Value(std::int64_t{0});
            t55.push_back(std::move(e57));
            gr::pmt::Value e58;
            e58 = gr::pmt::Value(std::pmr::string("agc"));
            t55.push_back(std::move(e58));
            gr::pmt::Value e59;
            e59 = gr::pmt::Value(std::int64_t{0});
            t55.push_back(std::move(e59));
            e54 = gr::pmt::Value(std::move(t55));
            t47.push_back(std::move(e54));
            gr::pmt::Value e60;
            gr::Tensor<gr::pmt::Value> t61;
            gr::pmt::Value e62;
            e62 = gr::pmt::Value(std::pmr::string("agc"));
            t61.push_back(std::move(e62));
            gr::pmt::Value e63;
            e63 = gr::pmt::Value(std::int64_t{0});
            t61.push_back(std::move(e63));
            gr::pmt::Value e64;
            e64 = gr::pmt::Value(std::pmr::string("fll"));
            t61.push_back(std::move(e64));
            gr::pmt::Value e65;
            e65 = gr::pmt::Value(std::int64_t{0});
            t61.push_back(std::move(e65));
            e60 = gr::pmt::Value(std::move(t61));
            t47.push_back(std::move(e60));
            gr::pmt::Value e66;
            gr::Tensor<gr::pmt::Value> t67;
            gr::pmt::Value e68;
            e68 = gr::pmt::Value(std::pmr::string("fll"));
            t67.push_back(std::move(e68));
            gr::pmt::Value e69;
            e69 = gr::pmt::Value(std::int64_t{0});
            t67.push_back(std::move(e69));
            gr::pmt::Value e70;
            e70 = gr::pmt::Value(std::pmr::string("timing"));
            t67.push_back(std::move(e70));
            gr::pmt::Value e71;
            e71 = gr::pmt::Value(std::int64_t{0});
            t67.push_back(std::move(e71));
            e66 = gr::pmt::Value(std::move(t67));
            t47.push_back(std::move(e66));
            gr::pmt::Value e72;
            gr::Tensor<gr::pmt::Value> t73;
            gr::pmt::Value e74;
            e74 = gr::pmt::Value(std::pmr::string("timing"));
            t73.push_back(std::move(e74));
            gr::pmt::Value e75;
            e75 = gr::pmt::Value(std::int64_t{0});
            t73.push_back(std::move(e75));
            gr::pmt::Value e76;
            e76 = gr::pmt::Value(std::pmr::string("costas"));
            t73.push_back(std::move(e76));
            gr::pmt::Value e77;
            e77 = gr::pmt::Value(std::int64_t{0});
            t73.push_back(std::move(e77));
            e72 = gr::pmt::Value(std::move(t73));
            t47.push_back(std::move(e72));
            gr::pmt::Value e78;
            gr::Tensor<gr::pmt::Value> t79;
            gr::pmt::Value e80;
            e80 = gr::pmt::Value(std::pmr::string("costas"));
            t79.push_back(std::move(e80));
            gr::pmt::Value e81;
            e81 = gr::pmt::Value(std::int64_t{0});
            t79.push_back(std::move(e81));
            gr::pmt::Value e82;
            e82 = gr::pmt::Value(std::pmr::string("decision"));
            t79.push_back(std::move(e82));
            gr::pmt::Value e83;
            e83 = gr::pmt::Value(std::int64_t{0});
            t79.push_back(std::move(e83));
            e78 = gr::pmt::Value(std::move(t79));
            t47.push_back(std::move(e78));
            m33[std::pmr::string("connections")] = gr::pmt::Value(std::move(t47));
            gr::Tensor<gr::pmt::Value> t84;
            gr::pmt::Value e85;
            gr::property_map m86;
            gr::property_map m87;
            m87[std::pmr::string("frequency_shift")] = gr::pmt::Value(std::pmr::string("=-frequency_offset"));
            m87[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m87[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("translate"));
            m86[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m87));
            m86[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::math::Rotator<complex<float32>>"));
            e85 = gr::pmt::Value(std::move(m86));
            t84.push_back(std::move(e85));
            gr::pmt::Value e88;
            gr::property_map m89;
            gr::property_map m90;
            m90[std::pmr::string("decimation")] = gr::pmt::Value(std::pmr::string("=decimation"));
            m90[std::pmr::string("transition_width")] = gr::pmt::Value(std::pmr::string("=0.5 * channel_bandwidth * symbol_rate"));
            m90[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m90[std::pmr::string("cutoff")] = gr::pmt::Value(std::pmr::string("=channel_bandwidth * symbol_rate"));
            m90[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m90[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel"));
            m89[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m90));
            m89[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<complex<float32>, float32>"));
            e88 = gr::pmt::Value(std::move(m89));
            t84.push_back(std::move(e88));
            gr::pmt::Value e91;
            gr::property_map m92;
            gr::property_map m93;
            m93[std::pmr::string("attack_s")] = gr::pmt::Value(std::pmr::string("=agc_attack_symbols / symbol_rate"));
            m93[std::pmr::string("reference_db")] = gr::pmt::Value(std::pmr::string("=agc_reference_db"));
            m93[std::pmr::string("decay_s")] = gr::pmt::Value(std::pmr::string("=agc_decay_symbols / symbol_rate"));
            m93[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate / decimation"));
            m93[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("agc"));
            m92[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m93));
            m92[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::Agc<complex<float32>>"));
            e91 = gr::pmt::Value(std::move(m92));
            t84.push_back(std::move(e91));
            gr::pmt::Value e94;
            gr::property_map m95;
            gr::property_map m96;
            m96[std::pmr::string("noise_bandwidth")] = gr::pmt::Value(std::pmr::string("=fll_noise_bandwidth"));
            m96[std::pmr::string("rolloff")] = gr::pmt::Value(std::pmr::string("=rolloff"));
            m96[std::pmr::string("samples_per_symbol")] = gr::pmt::Value(std::pmr::string("=sample_rate / (decimation * symbol_rate)"));
            m96[std::pmr::string("filter_length")] = gr::pmt::Value(std::pmr::string("=fll_filter_length"));
            m96[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("fll"));
            m95[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m96));
            m95[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::FllBandEdge"));
            e94 = gr::pmt::Value(std::move(m95));
            t84.push_back(std::move(e94));
            gr::pmt::Value e97;
            gr::property_map m98;
            gr::property_map m99;
            m99[std::pmr::string("noise_bandwidth")] = gr::pmt::Value(std::pmr::string("=timing_noise_bandwidth"));
            m99[std::pmr::string("rolloff")] = gr::pmt::Value(std::pmr::string("=rolloff"));
            m99[std::pmr::string("samples_per_symbol")] = gr::pmt::Value(std::pmr::string("=sample_rate / (decimation * symbol_rate)"));
            m99[std::pmr::string("constellation")] = gr::pmt::Value(std::pmr::string("bpsk"));
            m99[std::pmr::string("detector")] = gr::pmt::Value(std::pmr::string("=timing_detector"));
            m99[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing"));
            m98[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m99));
            m98[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::SymbolSync<complex<float32>>"));
            e97 = gr::pmt::Value(std::move(m98));
            t84.push_back(std::move(e97));
            gr::pmt::Value e100;
            gr::property_map m101;
            gr::property_map m102;
            m102[std::pmr::string("detector_gain")] = gr::pmt::Value(1.0);
            m102[std::pmr::string("noise_bandwidth")] = gr::pmt::Value(std::pmr::string("=costas_noise_bandwidth"));
            m102[std::pmr::string("order")] = gr::pmt::Value(std::uint32_t{2});
            m102[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("costas"));
            m101[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m102));
            m101[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::CostasLoop"));
            e100 = gr::pmt::Value(std::move(m101));
            t84.push_back(std::move(e100));
            gr::pmt::Value e103;
            gr::property_map m104;
            gr::property_map m105;
            m105[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("decision"));
            m104[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m105));
            m104[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::Real<complex<float32>>"));
            e103 = gr::pmt::Value(std::move(m104));
            t84.push_back(std::move(e103));
            m33[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t84));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m33));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m106;
            m106[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m106[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m106[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m106[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m106[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::BpskDemod"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m106));
            return def;
        }();
        return kDefinition;
    }

    // Builds the composite through the same instantiation path the loader uses — the
    // bindings attach identically, so live parameter changes behave identically — and
    // adds it to `graph`. No YAML is parsed and no file is read.
    static std::shared_ptr<gr::BlockModel> emplace(gr::Graph& graph, Parameters parameters) {
        gr::property_map values;
        values[std::pmr::string("sample_rate")] = parameters.sample_rate;
        values[std::pmr::string("symbol_rate")] = parameters.symbol_rate;
        values[std::pmr::string("frequency_offset")] = parameters.frequency_offset;
        values[std::pmr::string("decimation")] = parameters.decimation;
        values[std::pmr::string("channel_bandwidth")] = parameters.channel_bandwidth;
        values[std::pmr::string("rolloff")] = parameters.rolloff;
        values[std::pmr::string("fll_noise_bandwidth")] = parameters.fll_noise_bandwidth;
        values[std::pmr::string("fll_filter_length")] = parameters.fll_filter_length;
        values[std::pmr::string("timing_detector")] = std::pmr::string(parameters.timing_detector);
        values[std::pmr::string("timing_noise_bandwidth")] = parameters.timing_noise_bandwidth;
        values[std::pmr::string("costas_noise_bandwidth")] = parameters.costas_noise_bandwidth;
        values[std::pmr::string("agc_reference_db")] = parameters.agc_reference_db;
        values[std::pmr::string("agc_attack_symbols")] = parameters.agc_attack_symbols;
        values[std::pmr::string("agc_decay_symbols")] = parameters.agc_decay_symbols;
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_BPSKDEMOD_HPP
