#include "Interleaved.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/ax25/Ax25.hpp>
#include <gnuradio-4.0/ax25/Kiss.hpp>

/*
 * The module's regression guard rather than a budget. The address arithmetic is a few shifts a byte over frames of
 * tens of bytes at kilobit link rates, so no figure here gates anything; what the arms are for is that a change
 * turning a per-frame cost into a per-byte one shows up as a number that moved.
 *
 * The unit is one frame, because that is what these blocks process: a record is a frame whatever its length, and a
 * per-byte figure would average the fixed address work over a payload size nobody chose.
 */
namespace {

using gr::blocks::ax25::Ax25Decode;
using gr::blocks::ax25::Ax25Encode;
using gr::blocks::ax25::KissDecode;
using gr::blocks::ax25::KissEncode;

using Record = gr::DataSet<std::uint8_t>;

constexpr std::size_t kFramesPerCall = 4096UZ;
constexpr std::size_t kRepeats       = 9UZ;

/// The reader and writer spans a `processBulk` needs, with no tag handling: the measurement is the record path.
struct TagSpan : std::span<const gr::Tag> {
    using value_type = gr::Tag;
    bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriteSpan : std::span<gr::Tag> {
    using value_type = gr::Tag;
    void publish(std::size_t) const noexcept {}
};

template<typename T>
struct ReaderSpan : std::span<const T> {
    using value_type = T;

    TagSpan     rawTags{};
    std::size_t streamIndex = 0UZ;
    std::size_t consumed    = 0UZ;
    bool        isConnected = true;
    bool        isSync      = true;

    explicit ReaderSpan(std::span<const T> records) : std::span<const T>(records) {}

    bool consume(std::size_t nRecords) noexcept {
        consumed = nRecords;
        return true;
    }
    void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::vector<std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>> tags() const { return {}; }
    [[nodiscard]] std::vector<std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>> tags(std::size_t) const { return {}; }
};

template<typename T>
struct WriterSpan : std::span<T> {
    using value_type = T;

    TagWriteSpan tags{};
    std::size_t  streamIndex = 0UZ;
    std::size_t  count       = 0UZ;
    bool         isConnected = true;
    bool         isSync      = true;

    explicit WriterSpan(std::span<T> room) : std::span<T>(room) {}

    void publish(std::size_t nRecords) noexcept { count = nRecords; }
    void publishTag(const gr::property_map&, std::size_t = 0UZ) noexcept {}
};

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

struct Rng {
    std::uint64_t state = 0x9e3779b97f4a7c15ULL;

    [[nodiscard]] std::uint64_t next() noexcept {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        return state;
    }
};

/// One call's worth of information fields, each of @p infoBytes seeded bytes.
[[nodiscard]] std::vector<Record> payloads(std::size_t infoBytes) {
    Rng                 rng;
    std::vector<Record> batch(kFramesPerCall);
    for (Record& record : batch) {
        record.signal_values.resize(infoBytes);
        for (std::uint8_t& byte : record.signal_values) {
            byte = static_cast<std::uint8_t>(rng.next() & 0xFFULL);
        }
        record.extents.push_back(static_cast<std::int32_t>(infoBytes));
        record.signal_names.emplace_back("payload");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
    }
    return batch;
}

/// The same payloads as assembled frames, which is what the decode arms read.
[[nodiscard]] std::vector<Record> frames(const gr::property_map& settings, std::size_t infoBytes) {
    Ax25Encode          block  = make<Ax25Encode>(settings);
    std::vector<Record> source = payloads(infoBytes);
    std::vector<Record> built(kFramesPerCall);

    ReaderSpan<Record> inSpan{std::span<const Record>(source)};
    WriterSpan<Record> outSpan{std::span<Record>(built)};
    std::ignore = block.processBulk(inSpan, outSpan);
    built.resize(outSpan.count);
    return built;
}

/// The same frames with a KISS data command byte in front, which is what `KissDecode` reads.
[[nodiscard]] std::vector<Record> kissFrames(const gr::property_map& settings, std::size_t infoBytes) {
    KissEncode          block  = make<KissEncode>({{"kiss_port", gr::Size_t{2}}});
    std::vector<Record> source = frames(settings, infoBytes);
    std::vector<Record> built(kFramesPerCall);

    ReaderSpan<Record> inSpan{std::span<const Record>(source)};
    WriterSpan<Record> outSpan{std::span<Record>(built)};
    std::ignore = block.processBulk(inSpan, outSpan);
    built.resize(outSpan.count);
    return built;
}

} // namespace

int main() {
    const gr::property_map kSettings{{"destination", std::string("APRS")}, {"source", std::string("N0CALL")}, {"via", std::string("WIDE1-1,WIDE2-2")}};
    const std::size_t      kInfoBytes = 64UZ;

    std::vector<Record> produced(kFramesPerCall);

    const std::vector<Record> info       = payloads(kInfoBytes);
    const std::vector<Record> assembled  = frames(kSettings, kInfoBytes);
    const std::vector<Record> kissFramed = kissFrames(kSettings, kInfoBytes);

    Ax25Encode encode = make<Ax25Encode>(kSettings);
    Ax25Decode decode;
    KissEncode kissEncode = make<KissEncode>({{"kiss_port", gr::Size_t{2}}});
    KissDecode kissDecode;

    const auto drive = [&produced](auto& block, const std::vector<Record>& source) {
        ReaderSpan<Record> inSpan{std::span<const Record>(source)};
        WriterSpan<Record> outSpan{std::span<Record>(produced)};
        std::ignore = block.processBulk(inSpan, outSpan);
        return static_cast<double>(outSpan.count);
    };

    std::vector<gr::blocks::ax25::bench::Arm> arms;
    arms.push_back({"Ax25Encode, two hops, 64 info bytes", [&] { return drive(encode, info); }});
    arms.push_back({"Ax25Decode, two hops, 64 info bytes", [&] { return drive(decode, assembled); }});
    arms.push_back({"KissEncode", [&] { return drive(kissEncode, assembled); }});
    arms.push_back({"KissDecode", [&] { return drive(kissDecode, kissFramed); }});

    gr::blocks::ax25::bench::report(std::span<gr::blocks::ax25::bench::Arm>(arms), kFramesPerCall, kRepeats, "frame");
}
