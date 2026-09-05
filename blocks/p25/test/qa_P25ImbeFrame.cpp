#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/p25/ImbeFrame.hpp>
#include <gnuradio-4.0/p25/PayloadGen.hpp>
#include <gnuradio-4.0/p25/PayloadLayer.hpp>

/*
 * qa for the P25 voice codeword: the interleave, the scrambling and the codes that protect
 * the 88 bits of IMBE speech parameters inside 144 transmitted ones.
 *
 * The strongest assertions here are absolute rather than comparative. The interleave has to be a
 * permutation of the codeword -- a bijection, no bit carried twice and none lost -- and the
 * nine codewords, the six hexbit groups, the low speed data and the header have to tile a
 * voice frame's 1680 information bits exactly, covering every one of them once. Neither
 * statement is a value this code produced; both are fixed by how long the fields are, so a
 * layout that got an origin wrong fails them however self-consistent it is.
 *
 * The interleave's purpose is also pinned as a number. Its job is to keep a burst of channel
 * errors out of any single error-correcting word, and the measure of how well it does that is
 * the closest two bits of one coded word ever come in the transmitted frame. That distance is
 * five, so a burst up to five bits long can put at most one error into any one coded word --
 * which every code in the codeword corrects -- and six can put two, which the Hamming words
 * cannot. A test that only round-tripped would pass with the interleave replaced by the
 * identity; this one would not.
 *
 * Three codewords come off the air. Each is 144 bits a real P25 transmitter sent, pinned
 * as a literal alongside the parameters it carries. Each must decode with zero corrections
 * and re-encode to exactly the bits that were received. That joint claim is not reachable by
 * a wrong implementation: a deinterleave, a scrambling sequence or a code bit order that was
 * off anywhere would be reading unrelated bits, and unrelated bits do not decode for free.
 */
namespace {

using namespace gr::p25;

//! Which of the eight parameter words codeword bit `index` protects.
int codedWordOf(std::size_t index) {
    if (index < 92U) {
        return static_cast<int>(index / 23U);
    }
    if (index < 137U) {
        return 4 + static_cast<int>((index - 92U) / 15U);
    }
    return 7;
}

std::uint64_t rng = 0x5175F5FF77FFULL;
std::uint32_t nextRandom() {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<std::uint32_t>(rng >> 33);
}

ImbeParameters randomParameters() {
    ImbeParameters p;
    for (std::size_t i = 0U; i < kImbeParameterWords; ++i) {
        p.u[i] = static_cast<std::uint16_t>(nextRandom() & ((1U << kImbeParameterBits[i]) - 1U));
    }
    return p;
}

//! Three codewords a real P25 transmitter sent, with where in the capture each was found.
struct AirCodeword {
    const char*                                    where;
    std::array<std::uint8_t, kImbeCodewordBits>    bits;
    std::array<std::uint16_t, kImbeParameterWords> u;
};

constexpr std::array<AirCodeword, 3> kAir{{
    {"LDU1 frame at symbol 188744, voice codeword 8", {{0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0}}, {{398, 1333, 3825, 441, 0, 1733, 1011, 114}}},
    {"LDU2 frame at symbol 189608, voice codeword 0", {{0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1}}, {{390, 952, 3781, 3808, 57, 1090, 1301, 19}}},
    {"LDU2 frame at symbol 189608, voice codeword 1", {{0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0}}, {{389, 333, 713, 2232, 54, 1523, 133, 66}}},
}};

} // namespace

const boost::ut::suite<"P25ImbeFrame"> p25ImbeFrameTests = [] {
    using namespace boost::ut;
    using namespace gr::p25;

    "the interleave is a permutation: every codeword bit lands somewhere in the 144, and every one of the 144 receives exactly one codeword bit"_test = [] {
        std::array<int, kImbeCodewordBits> hits{};
        bool                               inRange = true;
        for (std::size_t i = 0U; i < kImbeCodewordBits; ++i) {
            const std::size_t at = imbeInterleavedBit(i);
            if (at >= kImbeCodewordBits) {
                inRange = false;
                continue;
            }
            ++hits[at];
        }
        expect(that % inRange) << "no codeword bit is placed outside the codeword";
        expect(std::all_of(hits.begin(), hits.end(), [](int h) { return h == 1; })) << "the interleave is a bijection over the codeword's 144 bits";
    };

    "what the interleave is for: the closest two bits of one coded word ever come is the longest burst that cannot put two errors into any single code"_test = [] {
        std::size_t closest = kImbeCodewordBits;
        for (std::size_t a = 0U; a < kImbeCodewordBits; ++a) {
            for (std::size_t b = a + 1U; b < kImbeCodewordBits; ++b) {
                if (codedWordOf(a) != codedWordOf(b)) {
                    continue;
                }
                const std::size_t pa = imbeInterleavedBit(a);
                const std::size_t pb = imbeInterleavedBit(b);
                closest              = std::min(closest, pa > pb ? pa - pb : pb - pa);
            }
        }
        expect(eq(closest, std::size_t{5U})) << "a burst of five transmitted bits reaches at most one bit of any coded word";

        // And the burst that does defeat a code, stated the same way rather than assumed.
        std::array<int, 8> worst{};
        for (std::size_t start = 0U; start + 6U <= kImbeCodewordBits; ++start) {
            std::array<int, 8> hit{};
            for (std::size_t i = 0U; i < kImbeCodewordBits; ++i) {
                const std::size_t at = imbeInterleavedBit(i);
                if (at >= start && at < start + 6U) {
                    ++hit[static_cast<std::size_t>(codedWordOf(i))];
                }
            }
            for (std::size_t w = 0U; w < 8U; ++w) {
                worst[w] = std::max(worst[w], hit[w]);
            }
        }
        expect(eq(*std::max_element(worst.begin(), worst.end()), 2)) << "and a burst of six reaches two, which the Hamming words cannot correct";
    };

    "the frame's information bits are tiled: the header, nine voice codewords, six hexbit groups and the low speed data cover all 1680 of them, each exactly once"_test = [] {
        constexpr std::size_t kFrameInfoBits = 1680U;
        std::vector<int>      cover(kFrameInfoBits, 0);
        const auto            mark = [&cover](std::size_t at, std::size_t count) {
            for (std::size_t i = 0U; i < count; ++i) {
                if (at + i < cover.size()) {
                    ++cover[at + i];
                }
            }
        };
        mark(0U, kFrameHeaderInfoBits);
        for (const std::size_t origin : kImbeCodewordOrigin) {
            mark(origin, kImbeCodewordBits);
        }
        for (std::size_t g = 0U; g < kLduGroups; ++g) {
            mark(kLduFirstGroupOrigin + g * kLduGroupStride, kLduCodewordsPerGroup * kLduCodewordBits);
        }
        mark(kLduLowSpeedDataOrigin, kLduLowSpeedDataBits);

        expect(kFrameHeaderInfoBits + kImbeCodewordsPerFrame * kImbeCodewordBits + kLduGroups * kLduCodewordsPerGroup * kLduCodewordBits + kLduLowSpeedDataBits == kFrameInfoBits) << "the fields' lengths add up to a voice frame's information bits";
        expect(std::all_of(cover.begin(), cover.end(), [](int c) { return c == 1; })) << "and they tile it: nothing overlaps and nothing is left out";
    };

    "encode and decode are inverses, and an undamaged codeword costs the codes nothing"_test = [] {
        std::size_t trials = 0U;
        std::size_t exact  = 0U;
        for (std::size_t t = 0U; t < 20000U; ++t) {
            const ImbeParameters                        in = randomParameters();
            std::array<std::uint8_t, kImbeCodewordBits> bits{};
            imbeEncodeCodeword(in, bits);
            const ImbeParameters out = imbeDecodeCodeword(bits);
            ++trials;
            exact += static_cast<std::size_t>(out.u == in.u && out.errors == 0U);
        }
        expect(eq(exact, trials)) << "every parameter set survives encoding and decoding untouched";
    };

    "each Golay word carries three errors and each Hamming word one, and the parameters still come back; the error count has to be the number actually injected"_test = [] {
        const ImbeParameters in = randomParameters();
        for (std::size_t w = 0U; w < 4U; ++w) {
            std::array<std::uint8_t, kImbeCodewordBits> bits{};
            imbeEncodeCodeword(in, bits);
            for (std::size_t e = 0U; e < 3U; ++e) {
                const std::size_t at = 23U * w + 3U * e + 1U;
                bits[at]             = static_cast<std::uint8_t>(bits[at] ^ 1U);
            }
            const ImbeParameters out = imbeDecodeCodeword(bits);
            expect(out.u == in.u) << "three errors in a Golay word are corrected";
            expect(eq(out.errors, 3U)) << "and counted";
        }
        for (std::size_t w = 0U; w < 3U; ++w) {
            std::array<std::uint8_t, kImbeCodewordBits> bits{};
            imbeEncodeCodeword(in, bits);
            const std::size_t at     = 92U + 15U * w + 4U;
            bits[at]                 = static_cast<std::uint8_t>(bits[at] ^ 1U);
            const ImbeParameters out = imbeDecodeCodeword(bits);
            expect(out.u == in.u) << "one error in a Hamming word is corrected";
            expect(eq(out.errors, 1U)) << "and counted";
        }
    };

    "the seventh word is carried bare: a bit changed there is a bit changed in the parameter, with no code to notice and no other word disturbed"_test = [] {
        const ImbeParameters                        in = randomParameters();
        std::array<std::uint8_t, kImbeCodewordBits> bits{};
        imbeEncodeCodeword(in, bits);
        bits[140]                = static_cast<std::uint8_t>(bits[140] ^ 1U);
        const ImbeParameters out = imbeDecodeCodeword(bits);
        expect(eq(out.errors, 0U)) << "an error in the unprotected word is not seen";
        expect(out.u[7] != in.u[7]) << "it lands in the parameter";
        for (std::size_t i = 0U; i < 7U; ++i) {
            expect(out.u[i] == in.u[i]) << "and every other word is unchanged";
        }
    };

    "the scrambling is real and is seeded by u0: damaging u0 past what its Golay code can repair makes the sequence the other six words are descrambled with the wrong one, so they come back wrong too"_test = [] {
        const ImbeParameters                        in = randomParameters();
        std::array<std::uint8_t, kImbeCodewordBits> bits{};
        imbeEncodeCodeword(in, bits);
        for (std::size_t e = 0U; e < 5U; ++e) {
            bits[e * 4U] = static_cast<std::uint8_t>(bits[e * 4U] ^ 1U);
        }
        const ImbeParameters out = imbeDecodeCodeword(bits);
        expect(out.u[0] != in.u[0]) << "five errors are past the first Golay word's power";
        std::size_t disturbed = 0U;
        for (std::size_t i = 1U; i < 7U; ++i) {
            disturbed += static_cast<std::size_t>(out.u[i] != in.u[i]);
        }
        expect(ge(disturbed, std::size_t{5U})) << "an unrecoverable first word carries the rest down with it";

        // The generator runs on across the words rather than restarting: the sequence the
        // second word is masked with cannot be the one the first is masked with.
        ImbeScrambler       pn{in.u[0]};
        const std::uint32_t m1 = pn.next(23U);
        const std::uint32_t m2 = pn.next(23U);
        const std::uint32_t m3 = pn.next(23U);
        expect(m1 != m2 && m2 != m3 && m1 != m3) << "each scrambled word gets its own stretch of the sequence";
        expect(eq(ImbeScrambler{in.u[0]}.next(23U), m1)) << "and the sequence is a function of the first word alone";
    };

    "off the air: each of these is 144 bits a real transmitter sent"_test = [] {
        for (const AirCodeword& air : kAir) {
            const ImbeParameters p = imbeDecodeCodeword(air.bits);
            expect(eq(p.errors, 0U)) << "an undamaged codeword off the air costs the codes nothing";
            expect(p.u == air.u) << "and decodes to the parameters it was sent carrying";

            std::array<std::uint8_t, kImbeCodewordBits> back{};
            imbeEncodeCodeword(p, back);
            expect(back == air.bits) << "re-encoding reproduces the transmitted bits exactly";
        }
    };

    "in a whole frame, through the status-symbol geometry: voice and the link control occupy the same frame and must not touch, both have to come back"_test = [] {
        std::uint64_t             seed  = 20260811U;
        const auto                lc    = groupVoiceLinkControl(1U, 9681U);
        std::vector<std::uint8_t> frame = buildLdu1Frame(0x692U, lc, seed);

        std::array<ImbeParameters, kImbeCodewordsPerFrame> sent{};
        for (std::size_t n = 0U; n < kImbeCodewordsPerFrame; ++n) {
            sent[n] = randomParameters();
            imbeEncodeVoiceCodeword(frame.data(), n, sent[n]);
        }

        std::size_t recovered = 0U;
        for (std::size_t n = 0U; n < kImbeCodewordsPerFrame; ++n) {
            const ImbeParameters got = imbeDecodeVoiceCodeword(frame.data(), n);
            recovered += static_cast<std::size_t>(got.u == sent[n].u && got.errors == 0U);
        }
        expect(eq(recovered, kImbeCodewordsPerFrame)) << "all nine codewords survive being placed in a frame";

        P25PayloadLayer layer{};
        P25Frame        f{};
        f.duid                   = static_cast<std::uint8_t>(P25Duid::Ldu1);
        f.nac                    = 0x692U;
        const P25Message message = layer.decodePayload(f, frame.data());
        expect(that % message.decoded()) << "and the link control in the same frame still decodes";
        expect(message.has_link_control && message.link_control.talkgroup == 1U) << "with its talkgroup intact";
        expect(eq(message.link_control.source_unit, std::uint32_t{9681U})) << "and its source unit";
        expect(message.inner_corrected == 0U && message.inner_refused == 0U) << "the voice codewords do not reach into the hexbit groups";
    };
};

int main() { /* tests are automatically registered and run */ }
