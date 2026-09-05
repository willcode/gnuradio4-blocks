// GENERATED FILE — do not edit. Source of truth: blocks/recipes/AfskDemod.yaml.
// Regenerate with gr4-recipe-gen; qa_Recipes diffs this file against a fresh emission.
#ifndef GNURADIO_RECIPES_AFSKDEMOD_HPP
#define GNURADIO_RECIPES_AFSKDEMOD_HPP

#include <memory>
#include <string>
#include <utility>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>

namespace gr::recipes {

struct AfskDemod {
    struct Parameters {
        // required parameters are constructor arguments: omitting one is a compile error,
        // the same requirement the loader enforces at run time
        Parameters(float sample_rate_, float symbol_rate_, double mark_hz_, double space_hz_) : sample_rate(std::move(sample_rate_)), symbol_rate(std::move(symbol_rate_)), mark_hz(std::move(mark_hz_)), space_hz(std::move(space_hz_)) {}
        float sample_rate; // input audio sample rate in hertz; required
        float symbol_rate; // symbol rate in hertz; required
        double mark_hz; // the tone that carries a ONE, in hertz; required, and there is no default tone pair
        double space_hz; // the tone that carries a ZERO, in hertz; required. Its position relative to mark_hz is the output polarity
        std::uint32_t decimation = std::uint32_t{1}; // decimation in the channel filter; pick it for about eight samples per symbol after it
        std::uint32_t hilbert_taps = std::uint32_t{127}; // length of the Hilbert transformer, odd; 127 keeps the band from 0.0125 to 0.4875 of the rate within 2 percent of unity
        double channel_bandwidth = 0.9167; // cutoff of the filter ahead of the discriminator, as a multiple of the symbol rate; half of Carson's bandwidth, |h|/2 + 0.5
        double lowpass_bandwidth = 0.5; // cutoff of the post-discriminator lowpass, as a multiple of the symbol rate
        double noise_bandwidth = 0.002; // closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate
        std::string detector = std::string("gardner"); // timing error detector: mueller_muller, modified_mueller_muller, zero_crossing, gardner, early_late, signal_slope_ml or signum_slope_ml
    };

    [[nodiscard]] static const gr::detail::YamlDefinitionsLoader::Definition& definition() {
        static const gr::detail::YamlDefinitionsLoader::Definition kDefinition = [] {
            gr::detail::YamlDefinitionsLoader::Definition def;
            def.metadata.block_type = "gr::recipes::AfskDemod";
            gr::Tensor<gr::pmt::Value> t0;
            gr::pmt::Value e1;
            gr::property_map m2;
            gr::Tensor<gr::pmt::Value> t3;
            gr::pmt::Value e4;
            gr::property_map m5;
            m5[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("input audio sample rate in hertz; required"));
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
            m9[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the tone that carries a ONE, in hertz; required, and there is no default tone pair"));
            m9[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m9[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("mark_hz"));
            e8 = gr::pmt::Value(std::move(m9));
            t3.push_back(std::move(e8));
            gr::pmt::Value e10;
            gr::property_map m11;
            m11[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("the tone that carries a ZERO, in hertz; required. Its position relative to mark_hz is the output polarity"));
            m11[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m11[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("space_hz"));
            e10 = gr::pmt::Value(std::move(m11));
            t3.push_back(std::move(e10));
            gr::pmt::Value e12;
            gr::property_map m13;
            m13[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("decimation in the channel filter; pick it for about eight samples per symbol after it"));
            m13[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{1});
            m13[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m13[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("decimation"));
            e12 = gr::pmt::Value(std::move(m13));
            t3.push_back(std::move(e12));
            gr::pmt::Value e14;
            gr::property_map m15;
            m15[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("length of the Hilbert transformer, odd; 127 keeps the band from 0.0125 to 0.4875 of the rate within 2 percent of unity"));
            m15[std::pmr::string("default")] = gr::pmt::Value(std::uint32_t{127});
            m15[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("uint32"));
            m15[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("hilbert_taps"));
            e14 = gr::pmt::Value(std::move(m15));
            t3.push_back(std::move(e14));
            gr::pmt::Value e16;
            gr::property_map m17;
            m17[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("cutoff of the filter ahead of the discriminator, as a multiple of the symbol rate; half of Carson's bandwidth, |h|/2 + 0.5"));
            m17[std::pmr::string("default")] = gr::pmt::Value(0.9167);
            m17[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m17[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel_bandwidth"));
            e16 = gr::pmt::Value(std::move(m17));
            t3.push_back(std::move(e16));
            gr::pmt::Value e18;
            gr::property_map m19;
            m19[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("cutoff of the post-discriminator lowpass, as a multiple of the symbol rate"));
            m19[std::pmr::string("default")] = gr::pmt::Value(0.5);
            m19[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m19[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("lowpass_bandwidth"));
            e18 = gr::pmt::Value(std::move(m19));
            t3.push_back(std::move(e18));
            gr::pmt::Value e20;
            gr::property_map m21;
            m21[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("closed-loop noise bandwidth of the timing recovery, normalized to the symbol rate"));
            m21[std::pmr::string("default")] = gr::pmt::Value(0.002);
            m21[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("float64"));
            m21[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("noise_bandwidth"));
            e20 = gr::pmt::Value(std::move(m21));
            t3.push_back(std::move(e20));
            gr::pmt::Value e22;
            gr::property_map m23;
            m23[std::pmr::string("doc")] = gr::pmt::Value(std::pmr::string("timing error detector: mueller_muller, modified_mueller_muller, zero_crossing, gardner, early_late, signal_slope_ml or signum_slope_ml"));
            m23[std::pmr::string("default")] = gr::pmt::Value(std::pmr::string("gardner"));
            m23[std::pmr::string("type")] = gr::pmt::Value(std::pmr::string("string"));
            m23[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("detector"));
            e22 = gr::pmt::Value(std::move(m23));
            t3.push_back(std::move(e22));
            m2[std::pmr::string("exported_parameters")] = gr::pmt::Value(std::move(t3));
            gr::property_map m24;
            m24[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("afsk_demod"));
            m2[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m24));
            gr::property_map m25;
            gr::Tensor<gr::pmt::Value> t26;
            gr::pmt::Value e27;
            gr::Tensor<gr::pmt::Value> t28;
            gr::pmt::Value e29;
            e29 = gr::pmt::Value(std::pmr::string("split"));
            t28.push_back(std::move(e29));
            gr::pmt::Value e30;
            e30 = gr::pmt::Value(std::pmr::string("INPUT"));
            t28.push_back(std::move(e30));
            gr::pmt::Value e31;
            e31 = gr::pmt::Value(std::pmr::string("in"));
            t28.push_back(std::move(e31));
            gr::pmt::Value e32;
            e32 = gr::pmt::Value(std::pmr::string("in"));
            t28.push_back(std::move(e32));
            e27 = gr::pmt::Value(std::move(t28));
            t26.push_back(std::move(e27));
            gr::pmt::Value e33;
            gr::Tensor<gr::pmt::Value> t34;
            gr::pmt::Value e35;
            e35 = gr::pmt::Value(std::pmr::string("timing"));
            t34.push_back(std::move(e35));
            gr::pmt::Value e36;
            e36 = gr::pmt::Value(std::pmr::string("OUTPUT"));
            t34.push_back(std::move(e36));
            gr::pmt::Value e37;
            e37 = gr::pmt::Value(std::pmr::string("out"));
            t34.push_back(std::move(e37));
            gr::pmt::Value e38;
            e38 = gr::pmt::Value(std::pmr::string("out"));
            t34.push_back(std::move(e38));
            e33 = gr::pmt::Value(std::move(t34));
            t26.push_back(std::move(e33));
            m25[std::pmr::string("exported_ports")] = gr::pmt::Value(std::move(t26));
            gr::Tensor<gr::pmt::Value> t39;
            gr::pmt::Value e40;
            gr::Tensor<gr::pmt::Value> t41;
            gr::pmt::Value e42;
            e42 = gr::pmt::Value(std::pmr::string("split"));
            t41.push_back(std::move(e42));
            gr::pmt::Value e43;
            e43 = gr::pmt::Value(std::int64_t{0});
            t41.push_back(std::move(e43));
            gr::pmt::Value e44;
            e44 = gr::pmt::Value(std::pmr::string("hilbert"));
            t41.push_back(std::move(e44));
            gr::pmt::Value e45;
            e45 = gr::pmt::Value(std::int64_t{0});
            t41.push_back(std::move(e45));
            e40 = gr::pmt::Value(std::move(t41));
            t39.push_back(std::move(e40));
            gr::pmt::Value e46;
            gr::Tensor<gr::pmt::Value> t47;
            gr::pmt::Value e48;
            e48 = gr::pmt::Value(std::pmr::string("split"));
            t47.push_back(std::move(e48));
            gr::pmt::Value e49;
            e49 = gr::pmt::Value(std::int64_t{0});
            t47.push_back(std::move(e49));
            gr::pmt::Value e50;
            e50 = gr::pmt::Value(std::pmr::string("delay"));
            t47.push_back(std::move(e50));
            gr::pmt::Value e51;
            e51 = gr::pmt::Value(std::int64_t{0});
            t47.push_back(std::move(e51));
            e46 = gr::pmt::Value(std::move(t47));
            t39.push_back(std::move(e46));
            gr::pmt::Value e52;
            gr::Tensor<gr::pmt::Value> t53;
            gr::pmt::Value e54;
            e54 = gr::pmt::Value(std::pmr::string("delay"));
            t53.push_back(std::move(e54));
            gr::pmt::Value e55;
            e55 = gr::pmt::Value(std::int64_t{0});
            t53.push_back(std::move(e55));
            gr::pmt::Value e56;
            e56 = gr::pmt::Value(std::pmr::string("analytic"));
            t53.push_back(std::move(e56));
            gr::pmt::Value e57;
            e57 = gr::pmt::Value(std::int64_t{0});
            t53.push_back(std::move(e57));
            e52 = gr::pmt::Value(std::move(t53));
            t39.push_back(std::move(e52));
            gr::pmt::Value e58;
            gr::Tensor<gr::pmt::Value> t59;
            gr::pmt::Value e60;
            e60 = gr::pmt::Value(std::pmr::string("hilbert"));
            t59.push_back(std::move(e60));
            gr::pmt::Value e61;
            e61 = gr::pmt::Value(std::int64_t{0});
            t59.push_back(std::move(e61));
            gr::pmt::Value e62;
            e62 = gr::pmt::Value(std::pmr::string("analytic"));
            t59.push_back(std::move(e62));
            gr::pmt::Value e63;
            e63 = gr::pmt::Value(std::int64_t{1});
            t59.push_back(std::move(e63));
            e58 = gr::pmt::Value(std::move(t59));
            t39.push_back(std::move(e58));
            gr::pmt::Value e64;
            gr::Tensor<gr::pmt::Value> t65;
            gr::pmt::Value e66;
            e66 = gr::pmt::Value(std::pmr::string("analytic"));
            t65.push_back(std::move(e66));
            gr::pmt::Value e67;
            e67 = gr::pmt::Value(std::int64_t{0});
            t65.push_back(std::move(e67));
            gr::pmt::Value e68;
            e68 = gr::pmt::Value(std::pmr::string("translate"));
            t65.push_back(std::move(e68));
            gr::pmt::Value e69;
            e69 = gr::pmt::Value(std::int64_t{0});
            t65.push_back(std::move(e69));
            e64 = gr::pmt::Value(std::move(t65));
            t39.push_back(std::move(e64));
            gr::pmt::Value e70;
            gr::Tensor<gr::pmt::Value> t71;
            gr::pmt::Value e72;
            e72 = gr::pmt::Value(std::pmr::string("translate"));
            t71.push_back(std::move(e72));
            gr::pmt::Value e73;
            e73 = gr::pmt::Value(std::int64_t{0});
            t71.push_back(std::move(e73));
            gr::pmt::Value e74;
            e74 = gr::pmt::Value(std::pmr::string("channel"));
            t71.push_back(std::move(e74));
            gr::pmt::Value e75;
            e75 = gr::pmt::Value(std::int64_t{0});
            t71.push_back(std::move(e75));
            e70 = gr::pmt::Value(std::move(t71));
            t39.push_back(std::move(e70));
            gr::pmt::Value e76;
            gr::Tensor<gr::pmt::Value> t77;
            gr::pmt::Value e78;
            e78 = gr::pmt::Value(std::pmr::string("channel"));
            t77.push_back(std::move(e78));
            gr::pmt::Value e79;
            e79 = gr::pmt::Value(std::int64_t{0});
            t77.push_back(std::move(e79));
            gr::pmt::Value e80;
            e80 = gr::pmt::Value(std::pmr::string("discriminator"));
            t77.push_back(std::move(e80));
            gr::pmt::Value e81;
            e81 = gr::pmt::Value(std::int64_t{0});
            t77.push_back(std::move(e81));
            e76 = gr::pmt::Value(std::move(t77));
            t39.push_back(std::move(e76));
            gr::pmt::Value e82;
            gr::Tensor<gr::pmt::Value> t83;
            gr::pmt::Value e84;
            e84 = gr::pmt::Value(std::pmr::string("discriminator"));
            t83.push_back(std::move(e84));
            gr::pmt::Value e85;
            e85 = gr::pmt::Value(std::int64_t{0});
            t83.push_back(std::move(e85));
            gr::pmt::Value e86;
            e86 = gr::pmt::Value(std::pmr::string("lowpass"));
            t83.push_back(std::move(e86));
            gr::pmt::Value e87;
            e87 = gr::pmt::Value(std::int64_t{0});
            t83.push_back(std::move(e87));
            e82 = gr::pmt::Value(std::move(t83));
            t39.push_back(std::move(e82));
            gr::pmt::Value e88;
            gr::Tensor<gr::pmt::Value> t89;
            gr::pmt::Value e90;
            e90 = gr::pmt::Value(std::pmr::string("lowpass"));
            t89.push_back(std::move(e90));
            gr::pmt::Value e91;
            e91 = gr::pmt::Value(std::int64_t{0});
            t89.push_back(std::move(e91));
            gr::pmt::Value e92;
            e92 = gr::pmt::Value(std::pmr::string("timing"));
            t89.push_back(std::move(e92));
            gr::pmt::Value e93;
            e93 = gr::pmt::Value(std::int64_t{0});
            t89.push_back(std::move(e93));
            e88 = gr::pmt::Value(std::move(t89));
            t39.push_back(std::move(e88));
            m25[std::pmr::string("connections")] = gr::pmt::Value(std::move(t39));
            gr::Tensor<gr::pmt::Value> t94;
            gr::pmt::Value e95;
            gr::property_map m96;
            gr::property_map m97;
            m97[std::pmr::string("delay")] = gr::pmt::Value(std::uint32_t{0});
            m97[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("split"));
            m96[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m97));
            m96[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::SampleDelay<float32>"));
            e95 = gr::pmt::Value(std::move(m96));
            t94.push_back(std::move(e95));
            gr::pmt::Value e98;
            gr::property_map m99;
            gr::property_map m100;
            m100[std::pmr::string("taps")] = gr::pmt::Value(std::pmr::string("=hilbert_taps"));
            m100[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("hilbert"));
            m100[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m100[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("hilbert"));
            m99[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m100));
            m99[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<float32, float32>"));
            e98 = gr::pmt::Value(std::move(m99));
            t94.push_back(std::move(e98));
            gr::pmt::Value e101;
            gr::property_map m102;
            gr::property_map m103;
            m103[std::pmr::string("delay")] = gr::pmt::Value(std::pmr::string("=(hilbert_taps - 1) / 2"));
            m103[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("delay"));
            m102[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m103));
            m102[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::SampleDelay<float32>"));
            e101 = gr::pmt::Value(std::move(m102));
            t94.push_back(std::move(e101));
            gr::pmt::Value e104;
            gr::property_map m105;
            gr::property_map m106;
            m106[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("analytic"));
            m105[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m106));
            m105[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::basic::RealImagToComplex<float32>"));
            e104 = gr::pmt::Value(std::move(m105));
            t94.push_back(std::move(e104));
            gr::pmt::Value e107;
            gr::property_map m108;
            gr::property_map m109;
            m109[std::pmr::string("frequency_shift")] = gr::pmt::Value(std::pmr::string("=-(mark_hz + space_hz) / 2"));
            m109[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m109[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("translate"));
            m108[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m109));
            m108[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::math::Rotator<complex<float32>>"));
            e107 = gr::pmt::Value(std::move(m108));
            t94.push_back(std::move(e107));
            gr::pmt::Value e110;
            gr::property_map m111;
            gr::property_map m112;
            m112[std::pmr::string("decimation")] = gr::pmt::Value(std::pmr::string("=decimation"));
            m112[std::pmr::string("transition_width")] = gr::pmt::Value(std::pmr::string("=0.5 * channel_bandwidth * symbol_rate"));
            m112[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m112[std::pmr::string("cutoff")] = gr::pmt::Value(std::pmr::string("=channel_bandwidth * symbol_rate"));
            m112[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate"));
            m112[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("channel"));
            m111[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m112));
            m111[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<complex<float32>, float32>"));
            e110 = gr::pmt::Value(std::move(m111));
            t94.push_back(std::move(e110));
            gr::pmt::Value e113;
            gr::property_map m114;
            gr::property_map m115;
            m115[std::pmr::string("gain")] = gr::pmt::Value(std::pmr::string("=sample_rate / (decimation * pi * (mark_hz - space_hz))"));
            m115[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("discriminator"));
            m114[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m115));
            m114[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::analog::QuadratureDemod<float32>"));
            e113 = gr::pmt::Value(std::move(m114));
            t94.push_back(std::move(e113));
            gr::pmt::Value e116;
            gr::property_map m117;
            gr::property_map m118;
            m118[std::pmr::string("transition_width")] = gr::pmt::Value(std::pmr::string("=0.5 * lowpass_bandwidth * symbol_rate"));
            m118[std::pmr::string("profile")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m118[std::pmr::string("cutoff")] = gr::pmt::Value(std::pmr::string("=lowpass_bandwidth * symbol_rate"));
            m118[std::pmr::string("sample_rate")] = gr::pmt::Value(std::pmr::string("=sample_rate / decimation"));
            m118[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("lowpass"));
            m117[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m118));
            m117[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::filter::DesignedFilter<float32, float32>"));
            e116 = gr::pmt::Value(std::move(m117));
            t94.push_back(std::move(e116));
            gr::pmt::Value e119;
            gr::property_map m120;
            gr::property_map m121;
            m121[std::pmr::string("noise_bandwidth")] = gr::pmt::Value(std::pmr::string("=noise_bandwidth"));
            m121[std::pmr::string("samples_per_symbol")] = gr::pmt::Value(std::pmr::string("=sample_rate / (decimation * symbol_rate)"));
            m121[std::pmr::string("detector")] = gr::pmt::Value(std::pmr::string("=detector"));
            m121[std::pmr::string("name")] = gr::pmt::Value(std::pmr::string("timing"));
            m120[std::pmr::string("parameters")] = gr::pmt::Value(std::move(m121));
            m120[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("gr::blocks::sync::SymbolSync<float32>"));
            e119 = gr::pmt::Value(std::move(m120));
            t94.push_back(std::move(e119));
            m25[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t94));
            m2[std::pmr::string("graph")] = gr::pmt::Value(std::move(m25));
            m2[std::pmr::string("id")] = gr::pmt::Value(std::pmr::string("SUBGRAPH"));
            e1 = gr::pmt::Value(std::move(m2));
            t0.push_back(std::move(e1));
            def.definition[std::pmr::string("blocks")] = gr::pmt::Value(std::move(t0));
            gr::property_map m122;
            m122[std::pmr::string("plugin_version")] = gr::pmt::Value(std::pmr::string("2026-09-02"));
            m122[std::pmr::string("plugin_license")] = gr::pmt::Value(std::pmr::string("MIT"));
            m122[std::pmr::string("plugin_author")] = gr::pmt::Value(std::pmr::string("gnuradio4 recipes"));
            m122[std::pmr::string("plugin_name")] = gr::pmt::Value(std::pmr::string("GrRecipes"));
            m122[std::pmr::string("block_type")] = gr::pmt::Value(std::pmr::string("gr::recipes::AfskDemod"));
            def.definition[std::pmr::string("definition_metadata")] = gr::pmt::Value(std::move(m122));
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
        values[std::pmr::string("mark_hz")] = parameters.mark_hz;
        values[std::pmr::string("space_hz")] = parameters.space_hz;
        values[std::pmr::string("decimation")] = parameters.decimation;
        values[std::pmr::string("hilbert_taps")] = parameters.hilbert_taps;
        values[std::pmr::string("channel_bandwidth")] = parameters.channel_bandwidth;
        values[std::pmr::string("lowpass_bandwidth")] = parameters.lowpass_bandwidth;
        values[std::pmr::string("noise_bandwidth")] = parameters.noise_bandwidth;
        values[std::pmr::string("detector")] = std::pmr::string(parameters.detector);
        auto composite = gr::detail::instantiateBlockFromYamlDefinition(gr::globalPluginLoader(), definition(), values);
        if (!composite.has_value()) {
            return nullptr;
        }
        return graph.addBlock(*composite);
    }
};

} // namespace gr::recipes

#endif // GNURADIO_RECIPES_AFSKDEMOD_HPP
