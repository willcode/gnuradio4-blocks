#include <boost/ut.hpp>

#include <complex>
#include <cstdint>
#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/basic/ClockSource.hpp>
#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/basic/DataSetToStream.hpp>
#include <gnuradio-4.0/basic/DataSink.hpp>
#include <gnuradio-4.0/basic/Decimators.hpp>
#include <gnuradio-4.0/basic/IqCorrection.hpp>
#include <gnuradio-4.0/basic/MessageDebug.hpp>
#include <gnuradio-4.0/basic/RateProbe.hpp>
#include <gnuradio-4.0/basic/RealToComplex.hpp>
#include <gnuradio-4.0/basic/SampleDelay.hpp>
#include <gnuradio-4.0/basic/Selector.hpp>
#include <gnuradio-4.0/basic/SignalGenerator.hpp>
#include <gnuradio-4.0/basic/StreamToDataSet.hpp>
#include <gnuradio-4.0/basic/SyncBlock.hpp>
#include <gnuradio-4.0/basic/Throttle.hpp>
#include <gnuradio-4.0/basic/Trigger.hpp>
#include <gnuradio-4.0/basic/VectorSource.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

// Each block with a settings hook initializes at its defaults, and with each setting alone at a value other than its
// default inside its limits. The hook runs at every initialization with every setting changed.
const boost::ut::suite<"basic blocks initialize at each setting"> settingsInitTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;
    using gr::testing::initFailures;
    using gr::testing::OffDefault;
    using gr::Size_t;
    const auto report = [](const std::vector<std::string>& failures) { return std::format("{}", failures); };

    "ClockSource"_test = [&report] {
        const auto failures = initFailures<ClockSource<float>>({
            {"n_samples_max", Size_t{10U}},
            {"sample_rate", 2000.f},
            {"chunk_size", Size_t{10U}},
            {"tag_times", std::vector<std::uint64_t>{1U, 2U}},
            {"tag_values", std::vector<std::string>{"a", "b"}},
            {"repeat_period", std::uint64_t{10U}},
            {"do_zero_order_hold", true},
            {"use_internal_thread", false},
        });
        expect(failures.empty()) << report(failures);
    };

    "DataSetToPacket and DataSetToStream"_test = [&report] {
        const auto toPacket = initFailures<DataSetToPacket<float>>({
            {"signal_index", Size_t{1U}},
            {"protocol_label", std::string("protocol")},
            {"source_label", std::string("source")},
            {"frequency_hz", 1e6},
            {"axis_rate_tolerance", 1e-3f},
        });
        expect(toPacket.empty()) << report(toPacket);
        const auto toStream = initFailures<DataSetToStream<float>>({
            {"signal_index", Size_t{1U}},
            {"boundary_label", std::string("record")},
            {"axis_rate_tolerance", 1e-3f},
        });
        expect(toStream.empty()) << report(toStream);
    };

    "DataSink and DataSetSink"_test = [&report] {
        const auto sink = initFailures<DataSink<float>>({
            {"sample_rate", 2.f},
            {"signal_name", std::string("settings init stream")},
            {"signal_quantity", std::string("voltage")},
            {"signal_unit", std::string("V")},
            {"signal_min", -2.f},
            {"signal_max", 2.f},
        });
        expect(sink.empty()) << report(sink);
        const auto dataSetSink = initFailures<DataSetSink<float>>({{"signal_name", std::string("settings init records")}});
        expect(dataSetSink.empty()) << report(dataSetSink);
    };

    "KeepOneInN and KeepMInN"_test = [&report] {
        const auto oneInN = initFailures<KeepOneInN<float>>({{"n", Size_t{3U}}});
        expect(oneInN.empty()) << report(oneInN);
        // m and offset alone at another value break offset + m <= n against the default n of 1
        const auto mInN = initFailures<KeepMInN<float>>({{"n", Size_t{4U}}});
        expect(mInN.empty()) << report(mInN);
    };

    "FrontEndCorrection"_test = [&report] {
        const auto failures = initFailures<FrontEndCorrection<std::complex<float>>>({
            {"swap_iq", true},
            {"enable_dc", true},
            {"sample_rate", 48000.f},
            {"tau", 0.5},
            {"enable_iq", true},
            {"iq_tau", 0.2},
        });
        expect(failures.empty()) << report(failures);
    };

    "MessageDebug, RateProbe, RealToComplex and SampleDelay"_test = [&report] {
        const auto debug = initFailures<MessageDebug>({{"display", false}, {"store", Size_t{2U}}});
        expect(debug.empty()) << report(debug);
        const auto probe = initFailures<RateProbe<float>>({{"alpha", 0.5f}, {"min_update_ms", 100.f}});
        expect(probe.empty()) << report(probe);
        const auto toComplex = initFailures<RealToComplex<float>>({{"transition", 0.1f}});
        expect(toComplex.empty()) << report(toComplex);
        const auto delay = initFailures<SampleDelay<float>>({{"delay", Size_t{4U}}});
        expect(delay.empty()) << report(delay);
    };

    "Selector and SyncBlock"_test = [&report] {
        // a map alone at another value names a port the default port counts of 0 do not have
        const auto selector = initFailures<Selector<float>>({
            {"n_inputs", Size_t{2U}},
            {"n_outputs", Size_t{2U}},
            {"back_pressure", true},
            {"sync_combined_ports", false},
        });
        expect(selector.empty()) << report(selector);
        const auto sync = initFailures<SyncBlock<float>>({
            {"n_ports", Size_t{2U}},
            {"max_history_size", Size_t{1000U}},
            {"filter", std::string("trigger")},
            {"tolerance", std::uint64_t{10U}},
        });
        expect(sync.empty()) << report(sync);
    };

    "SignalGenerator"_test = [&report] {
        const auto failures = initFailures<SignalGenerator<float>>({
            {"sample_rate", 2000.f},
            {"chunk_size", Size_t{10U}},
            {"signal_type", std::string("cos")},
            {"tone_frequency", 2.f},
            {"amplitude", 2.f},
            {"offset", 1.f},
            {"phase", 0.5f},
            {"seed", std::uint64_t{7U}},
        });
        expect(failures.empty()) << report(failures);
    };

    "StreamToDataSet and StreamFilter"_test = [&report] {
        const std::vector<OffDefault> offDefault{
            {"filter", std::string("trigger")},
            {"n_pre", Size_t{10U}},
            {"n_post", Size_t{10U}},
            {"n_max", Size_t{100U}},
            {"sample_rate", 2.f},
            {"signal_name", std::string("name")},
            {"signal_quantity", std::string("voltage")},
            {"signal_unit", std::string("V")},
        };
        const auto toDataSet = initFailures<StreamToDataSet<float>>(offDefault);
        expect(toDataSet.empty()) << report(toDataSet);
    };

    "Throttle, SchmittTrigger and VectorSource"_test = [&report] {
        const auto throttle = initFailures<Throttle<float>>({{"sample_rate", 1000.f}, {"max_items_per_chunk", Size_t{10U}}, {"max_sleep_s", 0.05}});
        expect(throttle.empty()) << report(throttle);
        const auto trigger = initFailures<SchmittTrigger<float>>({
            {"offset", 0.5f},
            {"threshold", 0.5f},
            {"trigger_name_rising_edge", std::string("up")},
            {"trigger_name_falling_edge", std::string("down")},
            {"sample_rate", 2.f},
            {"forward_tag", false},
            {"trigger_name", std::string("trigger")},
            {"trigger_offset", 0.25f},
        });
        expect(trigger.empty()) << report(trigger);
        const auto source = initFailures<VectorSource<float>>({
            {"values", std::vector<float>{1.f, 2.f}},
            {"repeat", std::uint64_t{2U}},
            {"repeat_tags", true},
            {"tags", gr::property_map{{"0", gr::property_map{{"mark", true}}}}},
        });
        expect(source.empty()) << report(source);
    };
};

int main() { /* not needed for UT */ }
