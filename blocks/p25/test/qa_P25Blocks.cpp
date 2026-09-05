#include <boost/ut.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/vocoder/Imbe.hpp>
#include <gnuradio-4.0/p25/Deframer.hpp>
#include <gnuradio-4.0/p25/ImbeFrame.hpp>
#include <gnuradio-4.0/p25/PayloadDecode.hpp>
#include <gnuradio-4.0/p25/PayloadGen.hpp>
#include <gnuradio-4.0/vocoder/ImbeDecode.hpp>

namespace {

using gr::blocks::p25::P25Deframer;
using gr::blocks::p25::P25PayloadDecode;
using gr::blocks::vocoder::ImbeDecode;

//! Eighteen consecutive voice codewords a real P25 transmitter sent — two whole voice frames
//! in transmitted order. The codec decodes each set against the one before it, so order is
//! significant here.
constexpr std::array<std::array<std::uint16_t, 8>, 18> kAirParameters{{
    {{402, 301, 1264, 3494, 50, 58, 508, 26}},
    {{400, 762, 3246, 1855, 58, 1816, 1065, 91}},
    {{410, 698, 1037, 1925, 33, 822, 2044, 114}},
    {{408, 639, 3900, 3520, 27, 2031, 41, 115}},
    {{408, 2913, 2294, 2365, 46, 1953, 90, 50}},
    {{415, 1924, 864, 3648, 34, 462, 239, 27}},
    {{2343, 1712, 1267, 3079, 0, 919, 155, 42}},
    {{399, 3968, 1600, 2634, 60, 35, 1722, 19}},
    {{386, 2365, 921, 2456, 53, 754, 2009, 114}},
    {{391, 523, 673, 2472, 63, 1516, 488, 83}},
    {{388, 619, 3760, 3706, 57, 684, 1899, 106}},
    {{386, 2111, 2045, 2800, 55, 1154, 393, 35}},
    {{389, 476, 713, 3296, 40, 1504, 1693, 122}},
    {{390, 809, 1685, 2416, 35, 563, 618, 59}},
    {{388, 111, 4080, 2538, 47, 553, 390, 74}},
    {{389, 1349, 2648, 2698, 45, 876, 1812, 123}},
    {{2703, 1926, 3900, 1202, 0, 877, 463, 52}},
    {{399, 1936, 1455, 2808, 6, 630, 253, 11}},
}};

//! An LDU frame from the generator with nine chosen codewords written into its voice slots.
[[nodiscard]] std::vector<std::uint8_t> voiceFrame(std::vector<std::uint8_t> raw, std::size_t firstCodeword) {
    for (std::size_t n = 0UZ; n < gr::p25::kImbeCodewordsPerFrame; ++n) {
        gr::p25::ImbeParameters parameters;
        parameters.u = kAirParameters[firstCodeword + n];
        gr::p25::imbeEncodeVoiceCodeword(raw.data(), n, parameters);
    }
    return raw;
}

struct FiniteSource : gr::Block<FiniteSource> {
    gr::PortOut<std::uint8_t> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<std::uint8_t>      _data;
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
struct RecordSink : gr::Block<RecordSink<T>> {
    gr::PortIn<gr::DataSet<T>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<T>>    _records;
    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

struct ChainResult {
    std::vector<gr::DataSet<std::uint16_t>> messages;
    std::vector<gr::DataSet<float>>         pcm;
    bool                                    finished = false;
};

//! Run dibits through deframer -> payload -> vocoder to completion.
[[nodiscard]] ChainResult runChain(std::vector<std::uint8_t> dibits) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<FiniteSource>();
    src._data     = std::move(dibits);
    auto& frames  = flow.emplaceBlock<P25Deframer>();
    auto& payload = flow.emplaceBlock<P25PayloadDecode>();
    auto& voice   = flow.emplaceBlock<ImbeDecode>();
    auto& msgSink = flow.emplaceBlock<RecordSink<std::uint16_t>>();
    auto& pcmSink = flow.emplaceBlock<RecordSink<float>>();
    boost::ut::expect(flow.connect<"out", "in">(src, frames).has_value());
    boost::ut::expect(flow.connect<"out", "in">(frames, payload).has_value());
    boost::ut::expect(flow.connect<"out", "in">(payload, voice).has_value());
    boost::ut::expect(flow.connect<"out", "in">(payload, msgSink).has_value());
    boost::ut::expect(flow.connect<"out", "in">(voice, pcmSink).has_value());

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
    ChainResult result;
    result.finished = done.load();
    if (!result.finished) {
        scheduler.requestStop();
    }
    runner.join();
    result.messages = std::move(msgSink._records);
    result.pcm      = std::move(pcmSink._records);
    return result;
}

template<typename V>
[[nodiscard]] V meta(const gr::DataSet<std::uint16_t>& record, const char* key, V fallback) {
    const auto& map = record.meta_information[0UZ];
    if (const auto entry = map.find(gr::property_map::key_type(key)); entry != map.end()) {
        return entry->second.value_or(V(fallback));
    }
    return fallback;
}

//! A whole clear transmission: header, both voice frames carrying the air fixture, terminator.
[[nodiscard]] std::vector<std::uint8_t> clearTransmission(std::uint8_t algid = gr::p25::kAlgidClear) {
    std::uint64_t             rng = 99991ULL;
    std::vector<std::uint8_t> stream(200UZ, 0U);

    gr::p25::P25HeaderMessage header;
    header.encryption_sync.algid         = algid;
    header.encryption_sync.keyid         = 0x1234U;
    header.talkgroup                     = 1234U;
    const std::vector<std::uint8_t> hdu  = gr::p25::buildHduFrame(0x293U, header, rng);
    const std::vector<std::uint8_t> ldu1 = voiceFrame(gr::p25::buildLdu1Frame(0x293U, gr::p25::groupVoiceLinkControl(1234U, 56789U), rng), 0UZ);
    gr::p25::P25EncryptionSync      es;
    es.algid                             = algid;
    es.keyid                             = 0x1234U;
    const std::vector<std::uint8_t> ldu2 = voiceFrame(gr::p25::buildLdu2Frame(0x293U, es, rng), 9UZ);

    stream.insert(stream.end(), hdu.begin(), hdu.end());
    stream.insert(stream.end(), ldu1.begin(), ldu1.end());
    stream.insert(stream.end(), ldu2.begin(), ldu2.end());
    std::vector<std::uint8_t> tail;
    gr::p25::appendP25Frame(tail, 0x293U, static_cast<std::uint8_t>(gr::p25::P25Duid::Tdu), rng);
    stream.insert(stream.end(), tail.begin(), tail.end());
    stream.insert(stream.end(), 200UZ, 0U);
    return stream;
}

} // namespace

int main() {
    using namespace boost::ut;

    "a clear transmission decodes end to end"_test = [] {
        const ChainResult result = runChain(clearTransmission());
        expect(result.finished);
        expect(eq(result.messages.size(), 4UZ)) << "header, two voice frames, terminator";
        if (result.messages.size() != 4UZ) {
            return;
        }
        const auto& hdu  = result.messages[0];
        const auto& ldu1 = result.messages[1];
        const auto& ldu2 = result.messages[2];
        const auto& tdu  = result.messages[3];

        expect(eq(meta<gr::Size_t>(hdu, "duid", 99U), 0U));
        expect(eq(meta<std::string>(hdu, "state", ""), std::string("decoded")));
        expect(eq(meta<gr::Size_t>(hdu, "nac", 0U), 0x293U));
        expect(eq(meta<gr::Size_t>(hdu, "talkgroup", 0U), 1234U));
        expect(eq(meta<gr::Size_t>(hdu, "algid", 0U), gr::Size_t(gr::p25::kAlgidClear)));
        expect(eq(meta<gr::Size_t>(hdu, "keyid", 0U), 0x1234U));

        expect(eq(meta<gr::Size_t>(ldu1, "duid", 99U), 5U));
        expect(eq(meta<std::string>(ldu1, "state", ""), std::string("decoded")));
        expect(eq(meta<gr::Size_t>(ldu1, "talkgroup", 0U), 1234U));
        expect(eq(meta<gr::Size_t>(ldu1, "source_unit", 0U), 56789U));
        expect(meta<bool>(ldu1, "clear", false));
        expect(eq(meta<gr::Size_t>(ldu2, "duid", 99U), 10U));
        expect(meta<bool>(ldu2, "clear", false));

        // Each voice record carries its position on the voice clock: ceil(frame_end * 5/3),
        // the values the vocoder's records are asserted to land on below.
        expect(eq(meta<std::uint64_t>(ldu1, "voice_sample_start", 0ULL), (static_cast<std::uint64_t>(200U + 396U + 864U) * 5ULL + 2ULL) / 3ULL));
        expect(eq(meta<std::uint64_t>(ldu2, "voice_sample_start", 0ULL), (static_cast<std::uint64_t>(200U + 396U + 864U + 864U) * 5ULL + 2ULL) / 3ULL));
        expect(eq(meta<std::string>(tdu, "state", ""), std::string("no_payload")));
        expect(eq(tdu.signal_values.size(), 0UZ));

        // The voice records carry exactly the parameter words the transmitter sent: the
        // codeword codes round-trip clean on an undamaged stream.
        for (std::size_t frame = 0UZ; frame < 2UZ; ++frame) {
            const auto& record = result.messages[1UZ + frame];
            expect(eq(record.signal_values.size(), 72UZ));
            if (record.signal_values.size() != 72UZ) {
                continue;
            }
            for (std::size_t n = 0UZ; n < 9UZ; ++n) {
                for (std::size_t w = 0UZ; w < 8UZ; ++w) {
                    expect(eq(record.signal_values[n * 8UZ + w], kAirParameters[frame * 9UZ + n][w])) << "frame" << frame << "codeword" << n << "word" << w;
                }
            }
        }

        // The vocoder's records: 1440 samples each, back to back on the voice clock, and
        // byte-identical to the kernel fed the same eighteen codewords in order.
        expect(eq(result.pcm.size(), 2UZ));
        if (result.pcm.size() != 2UZ) {
            return;
        }
        gr::vocoder::ImbeDecoder reference;
        std::vector<float>       expected(18UZ * 160UZ);
        for (std::size_t n = 0UZ; n < 18UZ; ++n) {
            reference.decode(std::span<const std::uint16_t, 8UZ>(kAirParameters[n]), std::span<float, 160UZ>(expected.data() + n * 160UZ, 160UZ));
        }
        std::uint64_t placed = 0ULL;
        for (std::size_t frame = 0UZ; frame < 2UZ; ++frame) {
            const auto& record = result.pcm[frame];
            expect(eq(record.signal_values.size(), 1440UZ));
            for (std::size_t i = 0UZ; i < record.signal_values.size(); ++i) {
                expect(record.signal_values[i] == expected[frame * 1440UZ + i]) << "sample" << i;
            }
            const auto& map   = record.meta_information[0UZ];
            const auto  entry = map.find(gr::property_map::key_type("sample_start"));
            expect(entry != map.end());
            const std::uint64_t at = entry == map.end() ? 0ULL : entry->second.value_or(std::uint64_t{0ULL});
            if (frame == 0UZ) {
                // ceil of the first voice frame's end, 200 idle + 396 header + 864, times 5/3.
                expect(eq(at, (static_cast<std::uint64_t>(200U + 396U + 864U) * 5ULL + 2ULL) / 3ULL));
            } else {
                expect(eq(at, placed + 1440ULL)) << "consecutive frames join without a gap";
            }
            placed = at;
        }
    };

    "voice with no encryption state stays silent"_test = [] {
        std::uint64_t                   rng = 424242ULL;
        std::vector<std::uint8_t>       stream(200UZ, 0U);
        const std::vector<std::uint8_t> ldu1 = voiceFrame(gr::p25::buildLdu1Frame(0x293U, gr::p25::groupVoiceLinkControl(1234U, 56789U), rng), 0UZ);
        stream.insert(stream.end(), ldu1.begin(), ldu1.end());
        stream.insert(stream.end(), 200UZ, 0U);

        const ChainResult result = runChain(std::move(stream));
        expect(result.finished);
        expect(eq(result.messages.size(), 1UZ));
        if (!result.messages.empty()) {
            expect(!meta<bool>(result.messages[0], "clear", true)) << "unknown is not clear";
            expect(eq(result.messages[0].signal_values.size(), 72UZ)) << "the parameters still ride the record";
        }
        expect(eq(result.pcm.size(), 0UZ)) << "nothing is vocoded";
    };

    "an encrypted transmission is silent and is not a failure"_test = [] {
        const ChainResult result = runChain(clearTransmission(0x84U));
        expect(result.finished);
        expect(eq(result.messages.size(), 4UZ));
        if (result.messages.size() == 4UZ) {
            expect(eq(meta<gr::Size_t>(result.messages[0], "algid", 0U), 0x84U));
            expect(!meta<bool>(result.messages[1], "clear", true));
            expect(!meta<bool>(result.messages[2], "clear", true));
        }
        expect(eq(result.pcm.size(), 0UZ));
    };

    "a sync inside a broken frame supersedes it"_test = [] {
        std::uint64_t             rng = 7777ULL;
        std::vector<std::uint8_t> stream(200UZ, 0U);
        std::vector<std::uint8_t> ldu1 = voiceFrame(gr::p25::buildLdu1Frame(0x293U, gr::p25::groupVoiceLinkControl(1234U, 56789U), rng), 0UZ);
        ldu1.resize(300UZ); // the stream breaks 300 symbols in
        stream.insert(stream.end(), ldu1.begin(), ldu1.end());
        std::vector<std::uint8_t> tdu;
        gr::p25::appendP25Frame(tdu, 0x293U, static_cast<std::uint8_t>(gr::p25::P25Duid::Tdu), rng);
        stream.insert(stream.end(), tdu.begin(), tdu.end());
        stream.insert(stream.end(), 200UZ, 0U);

        const ChainResult result = runChain(std::move(stream));
        expect(result.finished);
        expect(eq(result.messages.size(), 2UZ));
        if (result.messages.size() == 2UZ) {
            expect(eq(meta<std::string>(result.messages[0], "state", ""), std::string("truncated")));
            expect(eq(meta<gr::Size_t>(result.messages[0], "duid", 99U), 5U));
            expect(eq(result.messages[0].signal_values.size(), 0UZ));
            expect(eq(meta<std::string>(result.messages[1], "state", ""), std::string("no_payload")));
            expect(eq(meta<gr::Size_t>(result.messages[1], "duid", 99U), 3U));
        }
        expect(eq(result.pcm.size(), 0UZ));
    };

    "a stream ending mid-frame still terminates"_test = [] {
        std::uint64_t             rng = 31337ULL;
        std::vector<std::uint8_t> stream(200UZ, 0U);
        std::vector<std::uint8_t> ldu1 = voiceFrame(gr::p25::buildLdu1Frame(0x293U, gr::p25::groupVoiceLinkControl(1234U, 56789U), rng), 0UZ);
        ldu1.resize(300UZ);
        stream.insert(stream.end(), ldu1.begin(), ldu1.end());

        const ChainResult result = runChain(std::move(stream));
        expect(result.finished) << "a pending frame must not stall end-of-stream";
        expect(eq(result.messages.size(), 0UZ)) << "the frame never completed and never resolved";
        expect(eq(result.pcm.size(), 0UZ));
    };

    return 0;
}
