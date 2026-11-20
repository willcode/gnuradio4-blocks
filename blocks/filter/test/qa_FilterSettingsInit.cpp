#include <boost/ut.hpp>

#include <complex>
#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/Tensor.hpp>
#include <gnuradio-4.0/filter/ArbitraryRateResampler.hpp>
#include <gnuradio-4.0/filter/Channelizer.hpp>
#include <gnuradio-4.0/filter/DcBlocker.hpp>
#include <gnuradio-4.0/filter/DesignedFilter.hpp>
#include <gnuradio-4.0/filter/FirFilter.hpp>
#include <gnuradio-4.0/filter/FrequencyEstimator.hpp>
#include <gnuradio-4.0/filter/RationalResampler.hpp>
#include <gnuradio-4.0/filter/SavitzkyGolayFilter.hpp>
#include <gnuradio-4.0/filter/StagedDecimator.hpp>
#include <gnuradio-4.0/filter/SvdDenoiser.hpp>
#include <gnuradio-4.0/filter/time_domain_filter.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

// Each filter block with a settings hook initializes at its defaults and with each setting alone off its default,
// inside its limits: its hook runs at every initialization with every setting changed.
const boost::ut::suite<"filter blocks initialize at each setting"> settingsInitTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::filter;
    using gr::Size_t;
    using gr::testing::initFailures;
    using CF                 = std::complex<float>;
    const auto report        = [](const std::vector<std::string>& failures) { return std::format("{}", failures); };
    const auto shortLowpass  = std::vector<float>{0.25f, 0.5f, 0.25f};
    const auto channelBranch = std::vector<float>{0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f};

    "resamplers"_test = [&] {
        const auto arbitrary = initFailures<ArbitraryRateResampler<float>>({
            {"rate", 0.5},
            {"min_rate", 0.25},
            {"bank_size", Size_t{32U}},
            {"interpolation", std::string("cubic")},
            {"taps", shortLowpass},
            {"rolloff", 0.3f},
            {"attenuation_db", 50.f},
            {"max_ripple_db", 0.2f},
        });
        expect(arbitrary.empty()) << report(arbitrary);
        const auto rational = initFailures<RationalResampler<float>>({
            {"interpolation", Size_t{3U}},
            {"decimation", Size_t{2U}},
            {"taps", shortLowpass},
            {"rolloff", 0.3f},
            {"attenuation_db", 50.f},
            {"max_ripple_db", 0.2f},
        });
        expect(rational.empty()) << report(rational);
        const auto staged = initFailures<StagedDecimator<float>>({
            {"decimation", Size_t{8U}},
            {"passband_width", 0.7f},
            {"attenuation_db", 70.f},
            {"ripple_db", 0.1f},
            {"factor_odd", false},
            {"max_odd_factor", Size_t{9U}},
        });
        expect(staged.empty()) << report(staged);
    };

    "channelizer banks"_test = [&] {
        const std::vector<gr::testing::OffDefault> offDefault{
            {"n_channels", Size_t{8U}},
            {"taps", channelBranch},
            {"attenuation_db", 70.0},
            {"transition", 0.4},
            {"prototype", std::string("lowpass")},
            {"span", Size_t{8U}},
            {"oversample", Size_t{2U}},
        };
        const auto analysis = initFailures<PolyphaseChannelizer<float>>(offDefault);
        expect(analysis.empty()) << report(analysis);
        const auto synthesis = initFailures<PolyphaseSynthesizer<float>>(offDefault);
        expect(synthesis.empty()) << report(synthesis);
    };

    "FIR filters"_test = [&] {
        const auto dcBlocker = initFailures<DcBlocker<float>>({{"length", Size_t{16U}}, {"long_form", false}, {"reseed_interval", Size_t{1024U}}});
        expect(dcBlocker.empty()) << report(dcBlocker);
        // the defaults state no band, so the block is not ready until a whole design is given
        const gr::property_map lowpass{{"profile", std::string("lowpass")}, {"sample_rate", 48000.f}, {"cutoff", 1000.0}, {"transition_width", 500.0}, {"gain", 2.0}, {"attenuation_db", 50.0}, {"window", std::string("hann")}};
        const auto             designed = gr::testing::requiredInitFailures<DesignedFilter<float, float>>(lowpass, {{"decimation", Size_t{2U}}});
        expect(designed.empty()) << report(designed);
        expect(gr::testing::initRefused<DesignedFilter<float, float>>({{"profile", std::string("lowpass")}, {"sample_rate", 48000.f}})) << "a design key given at construction is designed at the initialization";
        const auto fir = initFailures<FirFilter<CF, float>>({{"taps", shortLowpass}, {"decimation", Size_t{2U}}});
        expect(fir.empty()) << report(fir);
        const auto savitzkyGolay = initFailures<SavitzkyGolayFilter<float>>({
            {"window_size", Size_t{15U}},
            {"poly_order", Size_t{2U}},
            {"deriv_order", Size_t{1U}},
            {"sample_rate", 100.f},
            {"alignment", std::string("causal")},
        });
        expect(savitzkyGolay.empty()) << report(savitzkyGolay);
        const auto svd = initFailures<SvdDenoiser<float>>({
            {"window_size", Size_t{32U}},
            {"hankel_rows", Size_t{8U}},
            {"max_rank", Size_t{4U}},
            {"relative_threshold", 0.01f},
            {"absolute_threshold", 0.001f},
            {"energy_fraction", 0.9f},
            {"hop_fraction", 0.5f},
        });
        expect(svd.empty()) << report(svd);
    };

    "time-domain filters"_test = [&] {
        const auto fir = initFailures<fir_filter<float>>({{"b", gr::Tensor<float>(shortLowpass.begin(), shortLowpass.end())}});
        expect(fir.empty()) << report(fir);
        const auto iir = initFailures<IirFilterDirectForm2<float>>({{"b", gr::Tensor<float>(shortLowpass.begin(), shortLowpass.end())}, {"a", gr::Tensor<float>{1.f, -0.5f}}});
        expect(iir.empty()) << report(iir);
        const auto basic = initFailures<BasicDecimatingFilter<float>>({
            {"filter_type", std::string("fir")},
            {"filter_response", std::string("highpass")},
            {"filter_order", Size_t{4U}},
            {"f_low", 0.05f},
            {"f_high", 0.3f},
            {"sample_rate", 2.f},
            {"decimate", Size_t{2U}},
            {"iir_design_method", std::string("bessel")},
            {"fir_design_method", std::string("hamming")},
        });
        expect(basic.empty()) << report(basic);
        const auto decimator = initFailures<Decimator<float>>({{"decim", Size_t{4U}}});
        expect(decimator.empty()) << report(decimator);
    };

    "frequency estimators"_test = [&] {
        const auto timeDomain = initFailures<FrequencyEstimatorTimeDomain<float>>({
            {"sample_rate", 2000.f},
            {"f_min", 30.f},
            {"f_expected", 55.f},
            {"f_max", 70.f},
            {"n_periods", Size_t{6U}},
            {"epsilon", 1e-6f},
        });
        expect(timeDomain.empty()) << report(timeDomain);
        const auto frequencyDomain = initFailures<FrequencyEstimatorFrequencyDomain<float>>({
            {"sample_rate", 2000.f},
            {"f_min", 30.f},
            {"f_expected", 55.f},
            {"f_max", 70.f},
            {"min_fft_size", Size_t{512U}},
            {"epsilon", 1e-6f},
        });
        expect(frequencyDomain.empty()) << report(frequencyDomain);
        const auto demodulator = initFailures<IQDemodulator<float>>({
            {"sample_rate", 1e6f},
            {"f_high_pass", 200.f},
            {"f_low_pass", 20000.f},
            {"phase_unit", std::string("degrees")},
            {"invert_phase", true},
            {"derivative_method", std::string("savitzky_golay5")},
            {"epsilon", 1e-9f},
        });
        expect(demodulator.empty()) << report(demodulator);
    };
};

int main() { /* not needed for UT */ }
