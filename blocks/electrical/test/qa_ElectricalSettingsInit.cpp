#include <boost/ut.hpp>

#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/electrical/PowerEstimators.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

// Each electrical block with a settings hook initializes at its defaults and with each setting alone off its default,
// inside its limits: its hook runs at every initialization with every setting changed.
const boost::ut::suite<"electrical blocks initialize at each setting"> settingsInitTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::electrical;
    using gr::Size_t;
    using gr::testing::initFailures;
    const auto report = [](const std::vector<std::string>& failures) { return std::format("{}", failures); };

    "PowerMetrics"_test = [&] {
        const auto failures = initFailures<PowerMetrics<float, 3UZ>>({{"sample_rate", 20'000.f}, {"high_pass", 5.f}, {"low_pass", 50.f}, {"decimate", Size_t{50U}}});
        expect(failures.empty()) << report(failures);
    };
};

int main() { /* not needed for UT */ }
