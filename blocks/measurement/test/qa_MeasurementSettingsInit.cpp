#include <boost/ut.hpp>

#include <complex>
#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/measurement/Autocorrelation.hpp>
#include <gnuradio-4.0/measurement/Detectors.hpp>
#include <gnuradio-4.0/measurement/Kurtosis.hpp>
#include <gnuradio-4.0/measurement/OccupiedBandwidth.hpp>
#include <gnuradio-4.0/measurement/PhaseUnwrap.hpp>
#include <gnuradio-4.0/measurement/PolarizationCombiner.hpp>
#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/measurement/SpectralKurtosis.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

// Each measurement block with a settings hook initializes at its defaults and with each setting alone off its default,
// inside its limits: its hook runs at every initialization with every setting changed.
const boost::ut::suite<"measurement blocks initialize at each setting"> settingsInitTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::measurement;
    using gr::Size_t;
    using gr::testing::initFailures;
    using CF          = std::complex<float>;
    const auto report = [](const std::vector<std::string>& failures) { return std::format("{}", failures); };

    "Autocorrelation"_test = [&] {
        const auto failures = initFailures<Autocorrelation<CF>>({
            {"window_length", Size_t{2048U}},
            {"max_lag", Size_t{512U}},
            {"kind", std::string("Complex")},
            {"normalization", std::string("Biased")},
            {"n_averages", Size_t{4U}},
            {"overlap", 0.25},
            {"remove_mean", false},
            {"window", std::string("Hann")},
            {"false_alarm_rate", 1e-2},
            {"emit_phase", true},
            {"sample_rate", 48000.f},
            {"signal_name", std::string("r")},
        });
        expect(failures.empty()) << report(failures);
    };

    "detectors"_test = [&] {
        const auto peak = initFailures<PeakDetect>({{"threshold_db", 6.0}, {"reference", std::string("absolute")}, {"min_distance_hz", 100.0}, {"max_peaks", Size_t{3U}}});
        expect(peak.empty()) << report(peak);
        const auto cfar = initFailures<CfarDetect>({{"n_train", Size_t{8U}}, {"n_guard", Size_t{4U}}, {"pfa", 1e-2}});
        expect(cfar.empty()) << report(cfar);
        const auto occupied = initFailures<OccupiedBandwidth>({{"fraction", 0.9}});
        expect(occupied.empty()) << report(occupied);
    };

    "kurtosis and phase"_test = [&] {
        const auto kurtosis = initFailures<Kurtosis<CF>>({{"window", Size_t{2048U}}, {"sample_rate", 48000.f}, {"emit_records", false}, {"signal_name", std::string("k")}});
        expect(kurtosis.empty()) << report(kurtosis);
        const auto spectral = initFailures<SpectralKurtosis>({{"n_spectra", Size_t{16U}}, {"shape", Size_t{1U}}, {"emit_records", false}});
        expect(spectral.empty()) << report(spectral);
        const auto unwrap = initFailures<PhaseUnwrap>({{"origin", std::string("zero")}, {"max_step_fraction", 0.5}, {"reset_on_discontinuity", false}});
        expect(unwrap.empty()) << report(unwrap);
        const auto combiner = initFailures<PolarizationCombiner>({
            {"mode", std::string("selection")},
            {"window", Size_t{1024U}},
            {"sample_rate", 48000.f},
            {"noise_powers", std::vector<double>{0.1, 0.5}},
            {"normalize", std::string("unit_signal")},
            {"weight_smoothing", 0.5},
            {"emit_orthogonal", false},
            {"emit_records", false},
        });
        expect(combiner.empty()) << report(combiner);
    };

    "spectral estimates"_test = [&] {
        const auto welch = initFailures<WelchPsd<CF>>({
            {"fft_size", Size_t{512U}},
            {"window", std::string("Kaiser")},
            {"window_param", 6.f},
            {"overlap", 0.25},
            {"hop", Size_t{128U}},
            {"n_averages", Size_t{4U}},
            {"mode", std::string("max_hold")},
            {"sample_rate", 48000.f},
            {"signal_name", std::string("p")},
            {"center_frequency", 100e6},
            {"one_record_per_call", false},
        });
        expect(welch.empty()) << report(welch);
        const auto spectrogram = initFailures<Spectrogram<CF>>({
            {"fft_size", Size_t{512U}},
            {"window", std::string("Kaiser")},
            {"window_param", 6.f},
            {"overlap", 0.25},
            {"hop", Size_t{128U}},
            {"sample_rate", 48000.f},
            {"signal_name", std::string("s")},
            {"center_frequency", 100e6},
            {"one_record_per_call", false},
        });
        expect(spectrogram.empty()) << report(spectrogram);
    };
};

int main() { /* not needed for UT */ }
