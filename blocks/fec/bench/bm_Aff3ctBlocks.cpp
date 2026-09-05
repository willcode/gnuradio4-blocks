#include <cstddef>
#include <cstdint>
#include <print>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/fec/LdpcBlocks.hpp>
#include <gnuradio-4.0/fec/PolarBlocks.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::fec::LdpcDecode;
using gr::blocks::fec::LdpcEncode;
using gr::blocks::fec::PolarDecode;
using gr::blocks::fec::PolarEncode;
using gr::blocks::testing::bench::Arm;
namespace shim = gr::blocks::testing::span;

using Bits = gr::DataSet<std::uint8_t>;
using Soft = gr::DataSet<float>;

constexpr std::size_t kRepeats = 4UZ;

template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

std::uint32_t state = 987654321U;

[[nodiscard]] std::vector<std::uint8_t> randomBits(std::size_t count) {
    std::vector<std::uint8_t> bits(count);
    for (std::size_t i = 0UZ; i < count; ++i) {
        state   = state * 1664525U + 1013904223U;
        bits[i] = static_cast<std::uint8_t>((state >> 16U) & 1U);
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

//! One encode call, and the coded bits it produced.
template<typename TEncode>
[[nodiscard]] std::vector<std::uint8_t> encodeOnce(TEncode& block, const std::vector<std::uint8_t>& payload, std::size_t codedBits) {
    const std::vector<Bits> input{record<std::uint8_t>(payload)};
    std::vector<Bits>       sink(1UZ);
    shim::InputSpan<Bits>   inSpan(std::span<const Bits>(input), 0UZ);
    shim::OutputSpan<Bits>  outSpan{std::span<Bits>(sink)};
    std::ignore                     = block.processBulk(inSpan, outSpan);
    std::vector<std::uint8_t> coded = sink[0UZ].signal_values;
    coded.resize(codedBits);
    return coded;
}

[[nodiscard]] std::vector<float> asSoft(const std::vector<std::uint8_t>& coded) {
    std::vector<float> values(coded.size());
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        values[i] = ((coded[i] & 1U) != 0U) ? 4.0F : -4.0F;
    }
    return values;
}

//! One call of a decode block over one record, which is the shape a graph calls it in.
template<typename TBlock>
[[nodiscard]] double sweepDecode(TBlock& block, std::span<const Soft> input, std::span<Bits> output) {
    shim::InputSpan<Soft>  inSpan(input, 0UZ);
    shim::OutputSpan<Bits> outSpan{output};
    std::ignore = block.processBulk(inSpan, outSpan);
    return static_cast<double>(output[0UZ].signal_values[0UZ]);
}

template<typename TBlock>
[[nodiscard]] double sweepEncode(TBlock& block, std::span<const Bits> input, std::span<Bits> output) {
    shim::InputSpan<Bits>  inSpan(input, 0UZ);
    shim::OutputSpan<Bits> outSpan{output};
    std::ignore = block.processBulk(inSpan, outSpan);
    return static_cast<double>(output[0UZ].signal_values[0UZ]);
}

} // namespace

int main() {
    // Frames a record, chosen so every arm moves a comparable number of bits and the report's
    // denominator — information bits a call — is the figure a chain is budgeted in.
    constexpr std::size_t kLdpcFrames  = 16UZ;
    constexpr std::size_t kPolarFrames = 16UZ;
    constexpr std::size_t kAidedFrames = 16UZ;

    LdpcEncode  ldpcEncoder({{"standard", std::string("wimax_576_288")}});
    LdpcDecode  ldpcNms({{"standard", std::string("wimax_576_288")}, {"decoder", std::string("normalized_min_sum")}, {"n_iterations", gr::Size_t{50U}}});
    LdpcDecode  ldpcSpa({{"standard", std::string("wimax_576_288")}, {"decoder", std::string("bp_flooding")}, {"n_iterations", gr::Size_t{50U}}});
    LdpcDecode  ldpcLayered({{"standard", std::string("wimax_576_288")}, {"decoder", std::string("bp_horizontal_layered")}, {"n_iterations", gr::Size_t{50U}}});
    PolarEncode polarEncoder({{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}});
    PolarDecode polarSc({{"n", gr::Size_t{1024U}}, {"k", gr::Size_t{512U}}, {"frozen_construction", std::string("5g")}, {"decoder", std::string("sc")}});
    PolarEncode aidedEncoder({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"crc_width", gr::Size_t{16U}}, {"crc_poly", std::uint64_t{0x1021ULL}}, {"crc_initial_value", std::uint64_t{0xFFFFULL}}});
    PolarDecode aidedScl({{"n", gr::Size_t{256U}}, {"k", gr::Size_t{128U}}, {"decoder", std::string("ca_scl")}, {"list_size", gr::Size_t{8U}}, //
        {"crc_width", gr::Size_t{16U}}, {"crc_poly", std::uint64_t{0x1021ULL}}, {"crc_initial_value", std::uint64_t{0xFFFFULL}}});
    init(ldpcEncoder);
    init(ldpcNms);
    init(ldpcSpa);
    init(ldpcLayered);
    init(polarEncoder);
    init(polarSc);
    init(aidedEncoder);
    init(aidedScl);

    const std::vector<std::uint8_t> ldpcPayload  = randomBits(288UZ * kLdpcFrames);
    const std::vector<std::uint8_t> polarPayload = randomBits(512UZ * kPolarFrames);
    const std::vector<std::uint8_t> aidedPayload = randomBits(112UZ * kAidedFrames);

    const std::vector<Bits> ldpcInput{record<std::uint8_t>(ldpcPayload)};
    const std::vector<Bits> polarInput{record<std::uint8_t>(polarPayload)};
    const std::vector<Bits> aidedInput{record<std::uint8_t>(aidedPayload)};

    const std::vector<Soft> ldpcSoft{record<float>(asSoft(encodeOnce(ldpcEncoder, ldpcPayload, 576UZ * kLdpcFrames)))};
    const std::vector<Soft> polarSoft{record<float>(asSoft(encodeOnce(polarEncoder, polarPayload, 1024UZ * kPolarFrames)))};
    const std::vector<Soft> aidedSoft{record<float>(asSoft(encodeOnce(aidedEncoder, aidedPayload, 256UZ * kAidedFrames)))};

    std::vector<Bits> sink(1UZ);

    // Every arm is read against the information bits its call carries, which is the figure a link
    // budget is written in; the shapes differ, so the denominator is stated per arm.
    std::vector<Arm> ldpcArms{
        {"LdpcEncode (576, 288)", [&] { return sweepEncode(ldpcEncoder, std::span<const Bits>(ldpcInput), std::span<Bits>(sink)); }},
        {"LdpcDecode (576, 288) NMS 50", [&] { return sweepDecode(ldpcNms, std::span<const Soft>(ldpcSoft), std::span<Bits>(sink)); }},
        {"LdpcDecode (576, 288) SPA flooding 50", [&] { return sweepDecode(ldpcSpa, std::span<const Soft>(ldpcSoft), std::span<Bits>(sink)); }},
        {"LdpcDecode (576, 288) SPA layered 50", [&] { return sweepDecode(ldpcLayered, std::span<const Soft>(ldpcSoft), std::span<Bits>(sink)); }},
    };
    std::println("LDPC, ns per information bit, {} frames of (576, 288) a call, early exit on at a clean word", kLdpcFrames);
    gr::blocks::testing::bench::report(std::span<Arm>(ldpcArms), 288UZ * kLdpcFrames, kRepeats);

    std::vector<Arm> polarArms{
        {"PolarEncode (1024, 512) 5G", [&] { return sweepEncode(polarEncoder, std::span<const Bits>(polarInput), std::span<Bits>(sink)); }},
        {"PolarDecode (1024, 512) SC", [&] { return sweepDecode(polarSc, std::span<const Soft>(polarSoft), std::span<Bits>(sink)); }},
    };
    std::println("Polar, ns per information bit, {} frames of (1024, 512) a call", kPolarFrames);
    gr::blocks::testing::bench::report(std::span<Arm>(polarArms), 512UZ * kPolarFrames, kRepeats);

    std::vector<Arm> aidedArms{
        {"PolarEncode (256, 128) GA, CRC-16", [&] { return sweepEncode(aidedEncoder, std::span<const Bits>(aidedInput), std::span<Bits>(sink)); }},
        {"PolarDecode (256, 128) CA-SCL 8", [&] { return sweepDecode(aidedScl, std::span<const Soft>(aidedSoft), std::span<Bits>(sink)); }},
    };
    std::println("Polar CRC-aided list, ns per payload bit, {} frames of (256, 128) a call, 112 payload bits a frame.", kAidedFrames);
    std::println("The list decoders this release ships in a fast form are broken at the pinned tag, so this is the naive");
    std::println("reference implementation and its cost is the reference implementation's, not the family's floor.");
    gr::blocks::testing::bench::report(std::span<Arm>(aidedArms), 112UZ * kAidedFrames, kRepeats);
    return 0;
}
