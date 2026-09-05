#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/ccsds/Frames.hpp>
#include <gnuradio-4.0/ccsds/SpacePackets.hpp>
#include <gnuradio-4.0/ccsds/TimeCodes.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

/*
 * The module's regression guard rather than a budget: these blocks run per frame or per packet, not per bit, so no
 * figure here gates anything and the bench exists to catch a change that turns a per-frame cost into a per-byte one.
 * Take every figure pinned to one named performance core; a figure taken unpinned mixes core types and compares with
 * nothing, including itself.
 */

using Record = gr::DataSet<std::uint8_t>;
using gr::blocks::testing::span::InputSpan;
using gr::blocks::testing::span::OutputSpan;

namespace {

struct Arm {
    std::string             label;
    std::function<double()> body;
};

void report(std::span<Arm> arms, std::size_t unitsPerCall, std::size_t repeats, std::string_view unit) {
    std::vector<double> best(arms.size(), 1e300);
    std::vector<double> worst(arms.size(), 0.0);
    double              checksum = 0.0;

    for (std::size_t repeat = 0UZ; repeat <= repeats; ++repeat) {
        for (std::size_t a = 0UZ; a < arms.size(); ++a) {
            const auto start = std::chrono::steady_clock::now();
            checksum += arms[a].body();
            const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - start).count() / static_cast<double>(unitsPerCall);
            if (repeat > 0UZ) {
                best[a]  = std::min(best[a], ns);
                worst[a] = std::max(worst[a], ns);
            }
        }
    }

    std::println("{} {}s per call, best of {} interleaved runs after one discarded warm-up", unitsPerCall, unit, repeats);
    for (std::size_t a = 0UZ; a < arms.size(); ++a) {
        std::println("{:<44} {:9.1f} ns/{}  (spread {:9.1f})", arms[a].label, best[a], unit, worst[a] - best[a]);
    }
    std::println("[checksum {:g}]", checksum);
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

[[nodiscard]] Record recordOf(std::vector<std::uint8_t> bytes) {
    Record record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.meta_information.emplace_back();
    record.timing_events.emplace_back();
    return record;
}

constexpr std::size_t kFramesPerCall = 256UZ;

/// A 2046-octet TM frame, the longest zone a first header pointer can address: primary header, no secondary header, no OCF.
[[nodiscard]] std::vector<Record> tmFrames() {
    std::vector<Record> frames;
    frames.reserve(kFramesPerCall);
    for (std::size_t i = 0UZ; i < kFramesPerCall; ++i) {
        std::vector<std::uint8_t>  frame(2046UZ, static_cast<std::uint8_t>(i));
        gr::ccsds::TmPrimaryHeader header{.version = 0, .spacecraft_id = 42, .virtual_channel = 1, .ocf_present = false, .master_frame_count = static_cast<std::uint8_t>(i), .vc_frame_count = static_cast<std::uint8_t>(i), .secondary_header = false, .sync_flag = false, .packet_order = false, .segment_length_id = 3, .first_header_pointer = 0};
        std::ignore = gr::ccsds::writeTmPrimaryHeader(header, frame);
        frames.push_back(recordOf(frame));
    }
    return frames;
}

/// A 2046-octet zone tiled exactly by 62 whole 33-octet space packets, so the walk ends on the zone's last octet and
/// no fragment is carried between calls: the figure is the extraction walk and nothing else. 33 divides 2046, and 2046
/// is the longest zone whose every position a first header pointer can name.
[[nodiscard]] std::vector<std::uint8_t> extractionZone() {
    constexpr std::size_t     kPacketOctets = 33UZ;
    std::vector<std::uint8_t> zone;
    zone.reserve(2046UZ);
    std::uint16_t seq = 0U;
    while (zone.size() + kPacketOctets <= 2046UZ) {
        gr::ccsds::SpacePacketHeader header{};
        std::ignore = gr::ccsds::headerForPayload(7U, false, false, 3U, seq++, kPacketOctets - gr::ccsds::kSpacePacketHeaderSize, header);
        std::vector<std::uint8_t> packet(kPacketOctets, static_cast<std::uint8_t>(seq));
        std::ignore = gr::ccsds::writeSpacePacketHeader(header, packet);
        zone.insert(zone.end(), packet.begin(), packet.end());
    }
    return zone;
}

} // namespace

int main() {
    using gr::blocks::ccsds::TmFrameDecode;

    const std::vector<Record>       frames = tmFrames();
    const std::vector<std::uint8_t> zone   = extractionZone();

    TmFrameDecode       decode = make<TmFrameDecode>({{"frame_length", gr::Size_t{2046}}});
    std::vector<Record> outBuf(kFramesPerCall);
    std::vector<Record> shBuf(1UZ);
    std::vector<Record> ocfBuf(1UZ);

    const auto driveTm = [&] {
        InputSpan<Record>  inSpan{std::span<const Record>(frames)};
        OutputSpan<Record> outSpan{std::span<Record>(outBuf)};
        OutputSpan<Record> shSpan{std::span<Record>(shBuf), 0UZ, nullptr, false};
        OutputSpan<Record> ocfSpan{std::span<Record>(ocfBuf), 0UZ, nullptr, false};
        std::ignore = decode.processBulk(inSpan, outSpan, shSpan, ocfSpan);
        return static_cast<double>(outSpan.count);
    };

    gr::ccsds::PacketExtractor extractor;
    const auto                 driveExtract = [&] {
        std::size_t count = 0UZ;
        extractor.reset();
        for (std::size_t i = 0UZ; i < kFramesPerCall; ++i) {
            extractor.feed(zone, 0U, static_cast<std::uint32_t>(i), [&count](std::span<const std::uint8_t>) { ++count; });
        }
        return static_cast<double>(count);
    };

    gr::ccsds::Layout cucLayout{};
    cucLayout.kind           = gr::ccsds::TimeCodeKind::cuc;
    cucLayout.coarse_octets  = 4U;
    cucLayout.fine_octets    = 0U;
    cucLayout.t_field_octets = 4UZ;
    std::vector<std::uint8_t> cucField(4UZ, 0U);
    const auto                driveCuc = [&] {
        double sum = 0.0;
        for (std::uint32_t i = 0U; i < kFramesPerCall; ++i) {
            cucField[0] = static_cast<std::uint8_t>(i >> 24U);
            cucField[1] = static_cast<std::uint8_t>(i >> 16U);
            cucField[2] = static_cast<std::uint8_t>(i >> 8U);
            cucField[3] = static_cast<std::uint8_t>(i);
            gr::ccsds::Instant instant{};
            std::ignore = gr::ccsds::decode(cucField, cucLayout, -gr::ccsds::kEpoch1958Ns, 0, instant);
            sum += static_cast<double>(instant.ns);
        }
        return sum;
    };

    std::vector<Arm> arms;
    arms.push_back({"TmFrameDecode::processBulk, 2046-octet frame", driveTm});
    arms.push_back({"PacketExtractor::feed, 2046-octet zone", driveExtract});
    arms.push_back({"gr::ccsds::decode, CUC, 4 coarse octets", driveCuc});

    report(std::span<Arm>(arms), kFramesPerCall, 10UZ, "op");
    return 0;
}
