// The Reed-Solomon profile surface's hot paths, per symbol at a stated codeblock count: the dual-basis
// recoding (one table lookup a symbol), the interleave gather, and the decode itself, measured through
// the block so the staging copies the type erasure costs are inside the figure.
#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/fec/RsBlocks.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::fec::RsDecode;
using gr::blocks::fec::RsEncode;
using gr::blocks::testing::bench::Arm;
namespace shim = gr::blocks::testing::span;

using Record = gr::DataSet<std::uint8_t>;

constexpr std::size_t kCodeblocks = 8UZ; ///< codeblocks a record, which is one call's work
constexpr std::size_t kRepeats    = 9UZ;

template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

[[nodiscard]] Record record(std::size_t items) {
    Record r;
    r.signal_values.resize(items);
    for (std::size_t i = 0UZ; i < items; ++i) {
        r.signal_values[i] = static_cast<std::uint8_t>((i * 37UZ + 11UZ) & 0xFFUZ);
    }
    r.extents.push_back(static_cast<std::int32_t>(items));
    r.signal_names.emplace_back("fec");
    r.timing_events.resize(1UZ);
    return r;
}

template<typename TBlock>
[[nodiscard]] double sweep(TBlock& block, std::span<const Record> input, std::span<Record> output) {
    shim::InputSpan<Record>  inSpan(input, 0UZ);
    shim::OutputSpan<Record> outSpan{output};
    std::ignore = block.processBulk(inSpan, outSpan);
    return static_cast<double>(output[0UZ].signal_values[0UZ]);
}

} // namespace

int main() {
    const gr::property_map conventional{{"code", std::string("ccsds_255_223")}, {"basis", std::string("conventional")}, {"interleave", gr::Size_t{5U}}};
    const gr::property_map dual{{"code", std::string("ccsds_255_223")}, {"basis", std::string("dual")}, {"interleave", gr::Size_t{5U}}};
    const gr::property_map flat{{"code", std::string("ccsds_255_223")}, {"basis", std::string("conventional")}};

    RsEncode encodePlain(conventional);
    RsEncode encodeDual(dual);
    RsEncode encodeFlat(flat);
    RsDecode decodePlain(conventional);
    RsDecode decodeDual(dual);
    RsDecode decodeFlat(flat);
    init(encodePlain);
    init(encodeDual);
    init(encodeFlat);
    init(decodePlain);
    init(decodeDual);
    init(decodeFlat);

    const std::vector<Record> info{record(kCodeblocks * 223UZ * 5UZ)};
    std::vector<Record>       coded(1UZ);
    std::ignore = sweep(encodePlain, std::span<const Record>(info), std::span<Record>(coded));
    std::vector<Record> codedDual(1UZ);
    std::ignore = sweep(encodeDual, std::span<const Record>(info), std::span<Record>(codedDual));
    const std::vector<Record> flatInfo{record(kCodeblocks * 223UZ * 5UZ)}; // the same items as the I=5 arms, as I=1 codeblocks
    std::vector<Record>       flatCoded(1UZ);
    std::ignore = sweep(encodeFlat, std::span<const Record>(flatInfo), std::span<Record>(flatCoded));
    std::vector<Record> sink(1UZ);

    std::vector<Arm> arms{
        {"RsEncode ccsds_255_223 conventional I=5", [&] { return sweep(encodePlain, std::span<const Record>(info), std::span<Record>(sink)); }},
        {"RsEncode ccsds_255_223 dual I=5 (the recoding on top)", [&] { return sweep(encodeDual, std::span<const Record>(info), std::span<Record>(sink)); }},
        {"RsDecode ccsds_255_223 conventional I=1 (the decode alone)", [&] { return sweep(decodeFlat, std::span<const Record>(flatCoded), std::span<Record>(sink)); }},
        {"RsDecode ccsds_255_223 conventional I=5 (the gather on top)", [&] { return sweep(decodePlain, std::span<const Record>(coded), std::span<Record>(sink)); }},
        {"RsDecode ccsds_255_223 dual I=5 (recoding and gather)", [&] { return sweep(decodeDual, std::span<const Record>(codedDual), std::span<Record>(sink)); }},
    };

    gr::blocks::testing::bench::report(std::span<Arm>(arms), kCodeblocks * 255UZ * 5UZ, kRepeats);
    return 0;
}
