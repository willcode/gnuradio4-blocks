#include <boost/ut.hpp>

#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/fourier/FrequencyCompressor.hpp>
#include <gnuradio-4.0/fourier/fft.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

// Each fourier block with a settings hook initializes at its defaults and with each setting alone off its default,
// inside its limits: its hook runs at every initialization with every setting changed.
const boost::ut::suite<"fourier blocks initialize at each setting"> settingsInitTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::fourier;
    using gr::Size_t;
    using gr::testing::initFailures;
    const auto report = [](const std::vector<std::string>& failures) { return std::format("{}", failures); };

    "FFT"_test = [&] {
        const auto failures = initFailures<FFT<float>>({
            {"fftSize", Size_t{512U}},
            {"window", std::string("Hamming")},
            {"outputInDb", true},
            {"outputInDeg", true},
            {"unwrapPhase", true},
            {"outputPhase", false},
            {"sample_rate", 48000.f},
            {"signal_name", std::string("x")},
            {"signal_unit", std::string("V")},
            {"signal_min", -1.f},
            {"signal_max", 1.f},
        });
        expect(failures.empty()) << report(failures);
    };

    "FrequencyCompressor"_test = [&] {
        const auto failures = initFailures<FrequencyCompressor>({{"divisor", Size_t{4U}}, {"frame", Size_t{1024U}}, {"sample_rate", 48000.f}});
        expect(failures.empty()) << report(failures);
    };
};

int main() { /* not needed for UT */ }
