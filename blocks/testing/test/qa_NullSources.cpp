#include <boost/ut.hpp>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>

#include <gnuradio-4.0/testing/NullSources.hpp>

const boost::ut::suite<"Null[..] and Testing Blocks"> nullSourcesTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::testing;

    constexpr auto kTestTypes = std::tuple<uint8_t, int16_t, int32_t, float>();

    "NullSource->CountingSink"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 12;

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<NullSource<T>>();
        auto&                 sink = test.emplace<CountingSink<T>>(property_map{{"n_samples_max", N}});

        expect(test.connect(src, "out", sink, "in").has_value());
        expect(test.run().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "CountingSource->NullSink"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N_total     = 7;
        constexpr T          start_value = T(3);

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<CountingSource<T>>(property_map{{"default_value", start_value}, {"n_samples_max", N_total}});
        auto&                 sink = test.emplace<NullSink<T>>();

        expect(test.connect(src, "out", sink, "in").has_value());
        expect(test.run().has_value());
    } | kTestTypes;

    "ConstantSource->NullSink"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 5;

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<ConstantSource<T>>(property_map{{"default_value", typename ConstantSource<T>::value_t(99)}, {"n_samples_max", N}});
        auto&                 sink = test.emplace<NullSink<T>>();

        expect(test.connect(src, "out", sink, "in").has_value());
        expect(test.run().has_value());
    } | kTestTypes;

    "SlowSource->CountingSink"_test = []<typename T>(const T&) {
        constexpr gr::Size_t N = 3;

        gr::test::RuntimeTest test;
        auto&                 src  = test.emplace<SlowSource<T>>(property_map{{"default_value", typename SlowSource<T>::value_t(77)}, {"delay", 10U}});
        auto&                 sink = test.emplace<CountingSink<T>>(property_map{{"n_samples_max", N}});

        expect(test.connect(src, "out", sink, "in").has_value());
        expect(test.run().has_value());
        expect(eq(sink.count, N));
    } | kTestTypes;

    "double header type smoke test"_test = [] {
        static_assert(BlockLike<ConstantSource<double>>);
        static_assert(BlockLike<NullSink<double>>);

        ConstantSource<double> source;
        NullSink<double>       sink;
        source.default_value = 1.5;
        expect(eq(source.processOne(), 1.5));
        sink.processOne(1.5);
    };

    "complex canonical type smoke test"_test = [] {
        using T = std::complex<float>;

        static_assert(BlockLike<ConstantSource<T>>);
        static_assert(BlockLike<NullSink<T>>);

        ConstantSource<T> source(property_map{{"default_value", 1.5F}});
        NullSink<T>       sink;
        source.init(source.progress);
        expect(eq(source.processOne(), T{1.5F, 0.F}));
        sink.processOne(T{1.5F, -2.F});
    };
};

int main() { /* not needed for UT */ }
