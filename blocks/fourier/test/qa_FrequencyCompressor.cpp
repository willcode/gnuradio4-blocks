#include <boost/ut.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <format>
#include <numbers>
#include <span>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/fourier/FrequencyCompressor.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using CF = std::complex<float>;

template<typename T>
struct FiniteSource : gr::Block<FiniteSource<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<T>                 _data;
    std::size_t                    _pos{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _pos);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_pos), n, outSpan.begin());
        outSpan.publish(n);
        _pos += n;
        return _pos == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename T>
struct CollectSink : gr::Block<CollectSink<T>> {
    gr::PortIn<T> in;
    GR_MAKE_REFLECTABLE(CollectSink, in);
    std::vector<T>                 _got;
    [[nodiscard]] gr::work::Status processBulk(std::span<const T> input) {
        _got.insert(_got.end(), input.begin(), input.end());
        return gr::work::Status::OK;
    }
};

[[nodiscard]] std::vector<CF> tone(std::size_t count, double cycles) {
    std::vector<CF> outData(count);
    for (std::size_t n = 0UZ; n < count; ++n) {
        const double phase = 2.0 * std::numbers::pi * cycles * static_cast<double>(n);
        outData[n]         = CF{static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))};
    }
    return outData;
}

/// @brief One call into the block: returns what it published and any tag it placed.
struct Call {
    std::size_t          made{};
    std::size_t          consumed{};
    std::vector<gr::Tag> tags{};
};

/// The settings change is placed exactly between two calls: the scheduler cannot be asked to change a setting at a
/// chosen sample, and a test that waited for one would be timing-dependent.
[[nodiscard]] Call drive(gr::blocks::fourier::FrequencyCompressor& block, std::span<const CF> input, std::span<CF> output) {
    namespace span = gr::blocks::testing::span;
    std::vector<gr::Tag> published;
    span::InputSpan<CF>  inSpan{input};
    span::OutputSpan<CF> outSpan{output, 0UZ, &published};
    std::ignore = block.processBulk(inSpan, outSpan);
    return Call{.made = outSpan.count, .consumed = inSpan.consumed, .tags = published};
}

/// @brief Apply a live settings change the way a running graph does.
void applyLive(gr::blocks::fourier::FrequencyCompressor& block, gr::property_map changes) {
    std::ignore = block.settings().set(std::move(changes));
    std::ignore = block.settings().activateContext(); // settings stage here and apply below, as they do inside work()
    std::ignore = block.settings().applyStagedParameters();
}

/// Run source → compressor → sink to completion on the Simple scheduler; empty on a hang.
[[nodiscard]] std::vector<CF> runGraph(gr::property_map settings, std::vector<CF> data) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<FiniteSource<CF>>();
    src._data     = std::move(data);
    auto& comp    = flow.emplaceBlock<gr::blocks::fourier::FrequencyCompressor>(std::move(settings));
    auto& sink    = flow.emplaceBlock<CollectSink<CF>>();
    boost::ut::expect(flow.connect<"out", "in">(src, comp).has_value());
    boost::ut::expect(flow.connect<"out", "in">(comp, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const bool finished = done.load();
    if (!finished) {
        scheduler.requestStop();
    }
    runner.join();
    return finished ? sink._got : std::vector<CF>{};
}

} // namespace

const boost::ut::suite<"FrequencyCompressor"> frequencyCompressorTests = [] {
    using namespace boost::ut;
    using gr::blocks::fourier::FrequencyCompressor;

    "a finite graph divides the stream and ends on its own"_test = [] {
        constexpr std::size_t kFrame   = 1024UZ;
        constexpr std::size_t kDivisor = 8UZ;
        const std::size_t     n        = 128UZ * kFrame;
        const auto            got      = runGraph({{"divisor", gr::Size_t(kDivisor)}, {"frame", gr::Size_t(kFrame)}}, tone(n, 0.2));

        expect(!got.empty()) << "the graph terminated with output";
        const auto want = static_cast<double>(n) / static_cast<double>(kDivisor);
        expect(lt(std::abs(static_cast<double>(got.size()) - want), static_cast<double>(kFrame))) << std::format("{} outputs of {} expected", got.size(), want);
        double power = 0.0;
        for (std::size_t i = got.size() / 2UZ; i < got.size(); ++i) {
            power += static_cast<double>(std::norm(got[i]));
        }
        power /= static_cast<double>(got.size() - got.size() / 2UZ);
        expect(lt(std::abs(10.0 * std::log10(power)), 0.5)) << std::format("settled level {} dB against the unit tone", 10.0 * std::log10(power));
    };

    "divisor 1 is a bit-exact passthrough"_test = [] {
        const auto x   = tone(10000UZ, 0.05);
        const auto got = runGraph({{"divisor", gr::Size_t(1)}}, x);
        expect(that % (got == x)) << "every sample as it arrived";
    };

    "refusals fire by name"_test = [] {
        const auto refused = [](gr::property_map settings) {
            return boost::ut::expect(throws([&settings] {
                FrequencyCompressor block{gr::property_map(settings)};
                block.settings().init();
                std::ignore = block.settings().applyStagedParameters();
                block.start();
            }));
        };
        refused({{"divisor", gr::Size_t(0)}}) << "divisor 0";
        refused({{"frame", gr::Size_t(1000)}}) << "frame not a power of two";
        refused({{"divisor", gr::Size_t(512)}, {"frame", gr::Size_t(1024)}}) << "a divisor past a quarter frame";
    };

    "the divisor moves while the block runs, re-plans, and states the new rate"_test = [] {
        constexpr float     kRate = 48000.f;
        FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(1024)}, {"sample_rate", kRate}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();

        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);

        // run until the first samples come out, which is where the block states its configured rate
        std::vector<gr::Tag> firstTags;
        std::size_t          madeBefore = 0UZ;
        for (std::size_t call = 0UZ; call < 8UZ && madeBefore == 0UZ; ++call) {
            const Call c = drive(block, std::span<const CF>(input), std::span<CF>(output));
            madeBefore   = c.made;
            firstTags    = c.tags;
        }
        expect(madeBefore > 0UZ) << "the block produces output at divisor 4";
        expect(eq(firstTags.size(), 1UZ)) << "and states the rate it was configured for";
        if (firstTags.size() == 1UZ) {
            const auto it = firstTags[0UZ].map.find(static_cast<std::pmr::string>(gr::tag::SAMPLE_RATE));
            expect(it != firstTags[0UZ].map.end());
            if (it != firstTags[0UZ].map.end()) {
                expect(approx(it->second.value_or(0.f), kRate / 4.f, 1e-3f));
            }
        }

        applyLive(block, {{"divisor", gr::Size_t(8)}});
        expect(eq(block.divisor.value, gr::Size_t(8))) << "the change is accepted rather than refused";

        std::vector<gr::Tag> afterTags;
        std::size_t          madeAfter = 0UZ;
        for (std::size_t call = 0UZ; call < 8UZ && madeAfter == 0UZ; ++call) {
            const Call c = drive(block, std::span<const CF>(input), std::span<CF>(output));
            madeAfter    = c.made;
            afterTags    = c.tags;
        }
        expect(madeAfter > 0UZ) << "and the block keeps producing under the new divisor";
        expect(eq(afterTags.size(), 1UZ)) << "the new rate is stated once, on the first sample after the change";
        if (afterTags.size() == 1UZ) {
            const auto it = afterTags[0UZ].map.find(static_cast<std::pmr::string>(gr::tag::SAMPLE_RATE));
            expect(it != afterTags[0UZ].map.end());
            if (it != afterTags[0UZ].map.end()) {
                expect(approx(it->second.value_or(0.f), kRate / 8.f, 1e-3f)) << "which is the input rate over the new divisor";
            }
        }

        const Call quiet = drive(block, std::span<const CF>(input), std::span<CF>(output));
        expect(quiet.tags.empty()) << "and only once: a rate that has not moved is not restated";
    };

    "the frame moves while the block runs, and says nothing about the rate"_test = [] {
        FrequencyCompressor block({{"divisor", gr::Size_t(4)}, {"frame", gr::Size_t(1024)}, {"sample_rate", 48000.f}});
        block.settings().init();
        std::ignore = block.settings().applyStagedParameters();
        block.start();

        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);
        for (std::size_t call = 0UZ; call < 8UZ; ++call) {
            std::ignore = drive(block, std::span<const CF>(input), std::span<CF>(output));
        }

        applyLive(block, {{"frame", gr::Size_t(2048)}});
        expect(eq(block.frame.value, gr::Size_t(2048))) << "a new analysis frame is accepted while running";

        std::size_t          made = 0UZ;
        std::vector<gr::Tag> tags;
        for (std::size_t call = 0UZ; call < 8UZ && made == 0UZ; ++call) {
            const Call c = drive(block, std::span<const CF>(input), std::span<CF>(output));
            made         = c.made;
            tags         = c.tags;
        }
        expect(made > 0UZ) << "the block re-plans and keeps producing";
        expect(tags.empty()) << "a frame change moves the resolution, not the rate, so nothing is stated";
    };

    "the output rate follows the divisor across a live change"_test = [] {
        const auto      input = tone(1UZ << 16, 0.05);
        std::vector<CF> output(1UZ << 16);

        const auto ratioAt = [&](gr::Size_t first, gr::Size_t second) {
            FrequencyCompressor block({{"divisor", first}, {"frame", gr::Size_t(1024)}});
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
            block.start();
            if (second != first) {
                for (std::size_t call = 0UZ; call < 4UZ; ++call) {
                    std::ignore = drive(block, std::span<const CF>(input), std::span<CF>(output));
                }
                applyLive(block, {{"divisor", second}});
            }
            std::size_t made = 0UZ;
            std::size_t took = 0UZ;
            for (std::size_t call = 0UZ; call < 24UZ; ++call) {
                const Call c = drive(block, std::span<const CF>(input), std::span<CF>(output));
                made += c.made;
                took += c.consumed;
            }
            return took > 0UZ ? static_cast<double>(made) / static_cast<double>(took) : 0.;
        };

        expect(std::abs(ratioAt(gr::Size_t(4), gr::Size_t(4)) - 0.25) < 0.02) << "a divisor of four produces a quarter of the samples";
        expect(std::abs(ratioAt(gr::Size_t(4), gr::Size_t(8)) - 0.125) < 0.02) << "and after a change to eight, an eighth of them";
    };
};

int main() { /* not needed for UT */ }
