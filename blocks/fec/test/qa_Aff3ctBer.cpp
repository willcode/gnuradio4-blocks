#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <print>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/fec/LdpcBlocks.hpp>
#include <gnuradio-4.0/fec/PolarBlocks.hpp>

/*
 * Tier 2's coded-loopback acceptance gate for the two wrapped families: encode, a seeded additive
 * white Gaussian noise channel, decode, and the information bit-error rate read against the rate
 * the pinned release's own reference tables publish for the same code, decoder and iteration count.
 *
 * The discipline is `spec-conv-viterbi.md` C1's, verbatim. The operating points, the published
 * readings, the envelope and the frame counts were fixed before the first run and none was changed
 * after it. A fixed seed measures one realization rather than an ensemble, and a frame error puts a
 * burst of bits wrong at once, so the estimator's spread at these frame counts is wide; the envelope
 * is a factor of four either way, asserted in both directions, so a rate far below the published
 * curve fails as loudly as one above it. What a factor of four still catches is every way a wrap can
 * be wrong — a sign inverted, a frozen set that is not the standard's, an iteration count that is
 * not being honored — since each of those lands orders of magnitude away.
 *
 * The channel is the tree's own `AwgnChannel<float>`, driven directly rather than wired into the
 * graph: it is a stream block and these are record blocks, so a graph would need a pair of
 * record-to-stream adapters between them and would measure those as well as this.
 */
namespace {

using gr::blocks::channel::AwgnChannel;
using gr::blocks::fec::LdpcDecode;
using gr::blocks::fec::LdpcEncode;
using gr::blocks::fec::PolarDecode;
using gr::blocks::fec::PolarEncode;

using Bits = gr::DataSet<std::uint8_t>;
using Soft = gr::DataSet<float>;

[[nodiscard]] bool longRun() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

//! Frames a leg. The long arm is what the recorded readings were taken at.
[[nodiscard]] std::size_t frames() { return longRun() ? 600UZ : 120UZ; }

//! The seed the whole gate runs at, so every rerun measures the same realization.
constexpr std::uint64_t kSeed = 20260902ULL;

//! The envelope, a factor either way, asserted in both directions.
constexpr double kEnvelope = 4.0;

std::uint64_t rng = 0x243F6A8885A308D3ULL;

[[nodiscard]] std::vector<std::uint8_t> randomBits(std::size_t count) {
    std::vector<std::uint8_t> bits(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        rng     = rng * 6364136223846793005ULL + 1442695040888963407ULL;
        bits[i] = static_cast<std::uint8_t>((rng >> 33U) & 1ULL);
    }
    return bits;
}

template<typename T>
[[nodiscard]] gr::DataSet<T> record(std::vector<T> values) {
    gr::DataSet<T> r;
    r.signal_values = std::move(values);
    r.extents.push_back(static_cast<std::int32_t>(r.signal_values.size()));
    r.signal_names.emplace_back("fec");
    r.timing_events.resize(1UZ);
    return r;
}

template<typename T>
struct RecordSource : gr::Block<RecordSource<T>> {
    gr::PortOut<gr::DataSet<T>, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<gr::DataSet<T>>    _records;
    std::size_t                    _pos{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Bits, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Bits>              _records;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

template<typename TBlock, typename TIn>
[[nodiscard]] std::vector<std::uint8_t> runBlock(gr::property_map settings, gr::DataSet<TIn> input) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<RecordSource<TIn>>();
    src._records  = {std::move(input)};
    auto& coder   = flow.emplaceBlock<TBlock>(std::move(settings));
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, coder).has_value());
    boost::ut::expect(flow.connect<"out", "in">(coder, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(600)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!done.load()) {
        scheduler.requestStop();
    }
    runner.join();
    boost::ut::expect(done.load());

    std::vector<std::uint8_t> out;
    if (!sink._records.empty()) {
        out = std::move(sink._records[0UZ].signal_values);
    }
    return out;
}

/*!
 * @brief Antipodal signaling of @p coded at unit energy a coded bit through the tree's AWGN channel,
 * returned as log-likelihood ratios in this tree's sense.
 *
 * A one is transmitted as +1 and a zero as -1, which is the sign convention this tree's soft carrier
 * states. The noise variance is the one that makes @p ebn0Db true at the code rate @p rate, since
 * `Eb = Es / rate` and `N0 = 2 * sigma^2`. The ratio handed to the decoder is `2 y / sigma^2`, the
 * exact channel LLR: the min-sum family is invariant to a positive scale but the sum-product family
 * is not, and a wrap that fed it an unscaled amplitude would be measuring a different decoder.
 */
[[nodiscard]] std::vector<float> throughChannel(const std::vector<std::uint8_t>& coded, double rate, double ebn0Db) {
    const double sigmaSquared = 1.0 / (2.0 * rate * std::pow(10.0, ebn0Db / 10.0));

    std::vector<float> clean(coded.size());
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        clean[i] = ((coded[i] & 1U) != 0U) ? 1.0F : -1.0F;
    }

    AwgnChannel<float> channel({{"noise_power", sigmaSquared}, {"seed", kSeed}});
    channel.settings().init();
    std::ignore = channel.settings().applyStagedParameters();
    channel.start();

    std::vector<float> noisy(coded.size());
    std::ignore = channel.processBulk(std::span<const float>(clean), std::span<float>(noisy));
    channel.stop();

    const float scale = static_cast<float>(2.0 / sigmaSquared);
    for (float& value : noisy) {
        value *= scale;
    }
    return noisy;
}

[[nodiscard]] std::size_t differences(const std::vector<std::uint8_t>& a, const std::vector<std::uint8_t>& b) {
    std::size_t count = 0UZ;
    for (std::size_t i = 0UZ; i < std::min(a.size(), b.size()); ++i) {
        count += (a[i] != b[i]) ? 1UZ : 0UZ;
    }
    return count;
}

//! One leg: encode, channel, decode, and the information bit-error rate it measured.
struct Reading {
    std::size_t errors = 0UZ;
    std::size_t bits   = 0UZ;

    [[nodiscard]] double ber() const { return bits == 0UZ ? 0.0 : static_cast<double>(errors) / static_cast<double>(bits); }
};

template<typename TEncode, typename TDecode>
[[nodiscard]] Reading measure(gr::property_map encodeSettings, gr::property_map decodeSettings, std::size_t payloadBits, std::size_t codedBits, double ebn0Db) {
    const std::size_t               legFrames = frames();
    const std::vector<std::uint8_t> payload   = randomBits(payloadBits * legFrames);
    const std::vector<std::uint8_t> coded     = runBlock<TEncode, std::uint8_t>(std::move(encodeSettings), record<std::uint8_t>(payload));
    boost::ut::expect(boost::ut::eq(coded.size(), codedBits * legFrames));
    if (coded.size() != codedBits * legFrames) {
        return {};
    }

    const double                    rate = static_cast<double>(payloadBits) / static_cast<double>(codedBits);
    const std::vector<float>        llr  = throughChannel(coded, rate, ebn0Db);
    const std::vector<std::uint8_t> got  = runBlock<TDecode, float>(std::move(decodeSettings), record<float>(llr));
    boost::ut::expect(boost::ut::eq(got.size(), payload.size()));
    return {differences(got, payload), payload.size()};
}

//! Report @p reading against @p published and assert it inside the envelope, in both directions.
void judge(std::string_view leg, double ebn0Db, const Reading& reading, double published) {
    using namespace boost::ut;
    const double measured = reading.ber();
    std::println("{} at Eb/N0 {:.2f} dB: {} information bit errors in {} bits, BER {:.3e}, published {:.3e}, ratio {:.2f}", //
        leg, ebn0Db, reading.errors, reading.bits, measured, published, measured / published);
    expect(gt(measured, published / kEnvelope)) << leg << "far below the published curve is as wrong as far above it";
    expect(lt(measured, published * kEnvelope)) << leg << "above the published curve";
}

//! Report @p reading where the release publishes no curve for the configuration.
void note(std::string_view leg, double ebn0Db, const Reading& reading) {
    std::println("{} at Eb/N0 {:.2f} dB: {} information bit errors in {} bits, BER {:.3e} (no published curve for this configuration)", //
        leg, ebn0Db, reading.errors, reading.bits, reading.ber());
}

} // namespace

int main() {
    using namespace boost::ut;

    /*
     * LDPC (576, 288), the WiMAX matrix the release ships, sum-product flooding at 100 iterations —
     * the release's own reference run, decoder and iteration count included, so the readings compare
     * against a curve produced by the same code rather than an adjacent one.
     */
    "the LDPC loopback lands on the release's published waterfall"_test = [] {
        struct Point {
            double ebn0;
            double published;
        };
        constexpr std::array<Point, 2UZ> points{{{1.50, 8.64e-03}, {2.00, 1.19e-03}}};
        for (const Point& point : points) {
            const Reading reading = measure<LdpcEncode, LdpcDecode>({{"standard", std::string("wimax_576_288")}}, //
                {{"standard", std::string("wimax_576_288")}, {"decoder", std::string("bp_flooding")}, {"n_iterations", gr::Size_t{100U}}}, 288UZ, 576UZ, point.ebn0);
            judge("LDPC (576, 288) WiMAX SPA 100", point.ebn0, reading, point.published);
        }
    };

    /*
     * Polar (1024, 512) under the release's own 5G reliability sequence, successive cancellation —
     * again the configuration the release's reference table was produced at.
     */
    "the Polar loopback lands on the release's published waterfall"_test = [] {
        struct Point {
            double ebn0;
            double published;
        };
        constexpr std::array<Point, 2UZ> points{{{2.00, 8.15e-03}, {2.50, 9.02e-04}}};
        for (const Point& point : points) {
            const gr::property_map construction{{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}};
            gr::property_map       decode = construction;
            decode["decoder"]             = std::string("sc");
            const Reading reading         = measure<PolarEncode, PolarDecode>(construction, std::move(decode), 512UZ, 1024UZ, point.ebn0);
            judge("Polar (1024, 512) 5G SC", point.ebn0, reading, point.published);
        }
    };

    /*
     * The two configurations the tier named, for which the release publishes no curve: the normalized
     * min-sum schedule at fifty iterations, and the CRC-aided list decoder. Both are recorded rather
     * than judged against a reading, and the list decoder carries the one assertion that can be made
     * without one — that it is strictly better than the plain successive cancellation it extends.
     */
    "the tier's own configurations are recorded, and the list decoder beats what it extends"_test = [] {
        if (!longRun()) { // the readings below are the long arm's; the short arm states the shape and stops
            std::println("the tier's own configurations are measured under ENABLE_LONG_TESTS");
            return;
        }
        const Reading nms = measure<LdpcEncode, LdpcDecode>({{"standard", std::string("wimax_576_288")}}, //
            {{"standard", std::string("wimax_576_288")}, {"decoder", std::string("normalized_min_sum")}, {"n_iterations", gr::Size_t{50U}}}, 288UZ, 576UZ, 2.00);
        note("LDPC (576, 288) WiMAX NMS 50", 2.00, nms);

        const gr::property_map crc{{"crc_width", gr::Size_t{16U}}, {"crc_poly", std::uint64_t{0x1021ULL}}, {"crc_initial_value", std::uint64_t{0xFFFFULL}}};
        gr::property_map       aidedEncode{{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}};
        gr::property_map       aidedDecode = aidedEncode;
        for (const auto& [key, value] : crc) {
            aidedEncode[key] = value;
            aidedDecode[key] = value;
        }
        aidedDecode["decoder"]   = std::string("ca_scl");
        aidedDecode["list_size"] = gr::Size_t{8U};

        // 496 payload bits a frame, the sixteen the signature takes coming out of the same k.
        const Reading aided = measure<PolarEncode, PolarDecode>(std::move(aidedEncode), std::move(aidedDecode), 496UZ, 1024UZ, 2.00);
        note("Polar (1024, 512) 5G CA-SCL 8, CRC-16", 2.00, aided);

        const gr::property_map plain{{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}};
        gr::property_map       plainDecode = plain;
        plainDecode["decoder"]             = std::string("sc");
        const Reading bare                 = measure<PolarEncode, PolarDecode>(plain, std::move(plainDecode), 512UZ, 1024UZ, 2.00);
        expect(lt(aided.ber(), bare.ber())) << "a CRC-aided list of eight must beat the successive cancellation it extends";
    };

    return 0;
}
