#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cstdint>

#include <gnuradio-4.0/p25/EncryptionSync.hpp>
#include <gnuradio-4.0/p25/FrameGen.hpp>
#include <gnuradio-4.0/p25/FrameLayer.hpp>
#include <gnuradio-4.0/p25/FrameSync.hpp>
#include <gnuradio-4.0/p25/ImbeFrame.hpp>
#include <gnuradio-4.0/p25/LinkControl.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>
#include <gnuradio-4.0/p25/PayloadBits.hpp>
#include <gnuradio-4.0/p25/PayloadGen.hpp>
#include <gnuradio-4.0/p25/PayloadLayer.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

/*
 * A smoke test for the P25 protocol layer's eleven headers: it checks that they compile and
 * link together and pins a handful of cheap protocol anchors, one per layer. Deeper coverage
 * of each header's own logic lives in the layer-specific qa files; this one only guards that
 * the headers work together as a whole.
 */

const boost::ut::suite<"P25Protocol"> p25ProtocolTests = [] {
    using namespace boost::ut;
    using namespace gr::p25;

    "the IMBE codeword interleave is a bijection over its 144 bit positions"_test = [] {
        std::array<bool, kImbeCodewordBits> seen{};
        bool                                inRange  = true;
        bool                                distinct = true;
        for (std::size_t i = 0U; i < kImbeCodewordBits; ++i) {
            const std::size_t mapped = imbeInterleavedBit(i);
            if (mapped >= kImbeCodewordBits) {
                inRange = false;
                continue;
            }
            if (seen[mapped]) {
                distinct = false;
            }
            seen[mapped] = true;
        }
        expect(that % inRange) << "every mapped position lies inside the codeword";
        expect(that % distinct) << "no two source bits land on the same position";
        expect(std::ranges::all_of(seen, [](bool b) { return b; })) << "every position is reached, so the map is onto and therefore a bijection";
    };

    "ImbeScrambler seeded 0x123 is a deterministic function of its seed"_test = [] {
        ImbeScrambler       a{0x123U};
        ImbeScrambler       b{0x123U};
        const std::uint32_t wordA = a.next(23U);
        const std::uint32_t wordB = b.next(23U);
        expect(eq(wordA, wordB)) << "two instances seeded alike produce the same first word";

        // The documented recurrence, computed independently of ImbeScrambler itself: state <-
        // (173*state + 13849) mod 65536, seeded with u0 shifted up four places, each output bit
        // the top bit of the new state, most significant first.
        std::uint32_t state    = 0x123U << 4U;
        std::uint32_t expected = 0U;
        for (unsigned i = 0U; i < 23U; ++i) {
            state    = (173U * state + 13849U) & 0xFFFFU;
            expected = (expected << 1) | ((state >> 15) & 1U);
        }
        expect(eq(wordA, expected)) << "the scrambler follows the documented linear congruence exactly";
    };

    "imbePackUVector and imbeUnpackUVector round-trip a known parameter set"_test = [] {
        ImbeParameters p;
        p.u = {0x0ABU, 0x0CDU, 0x0EFU, 0x012U, 0x345U, 0x678U, 0x7FFU, 0x5AU};

        std::array<std::uint8_t, kImbeUVectorBytes> bytes{};
        imbePackUVector(p, bytes.data());
        const ImbeParameters roundTripped = imbeUnpackUVector(bytes.data());

        expect(std::ranges::equal(p.u, roundTripped.u)) << "every parameter word survives a pack/unpack round trip";
    };

    "decodeNid round-trips a NID built by the frame generator's encode path"_test = [] {
        constexpr std::uint16_t nac  = 0x3A5U;
        constexpr std::uint8_t  duid = static_cast<std::uint8_t>(P25Duid::Ldu1);

        const std::uint64_t nid     = buildNid(nac, duid);
        const P25Nid        decoded = decodeNid(nid);

        expect(that % decoded.valid) << "an undamaged, freshly built NID decodes";
        expect(eq(decoded.nac, nac));
        expect(eq(decoded.duid, duid));
        expect(that % decoded.parity_ok);
        expect(that % decoded.duid_defined);
        expect(eq(decoded.bch_errors, 0U)) << "nothing was damaged, so nothing was corrected";
    };

    "duidTransmittedDibits reports the standard's fixed voice-frame length"_test = [] {
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Ldu1)), std::size_t{864U}));
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Ldu2)), std::size_t{864U}));
        expect(neq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Ldu1)), std::size_t{0U}));
        expect(neq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Ldu2)), std::size_t{0U}));
    };

    "a generated LDU1 frame carries a Link Control word the payload layer recovers"_test = [] {
        constexpr std::uint16_t nac = 0x1B2U;
        std::uint64_t           rng = 42U;

        const auto octets = groupVoiceLinkControl(/*talkgroup=*/12345U, /*sourceUnit=*/678901U);
        const auto raw    = buildLdu1Frame(nac, octets, rng);

        P25FrameLayer layer{};
        bool          frameSeen = false;
        P25Frame      frame{};
        for (std::size_t i = 0U; i < raw.size(); ++i) {
            layer.push(raw[i], [&](const P25Frame& f) {
                frameSeen = true;
                frame     = f;
                return P25FrameAction::Continue;
            });
        }
        expect(that % frameSeen) << "the frame layer finds the sync and decodes the identifier";
        expect(eq(frame.nac, nac));
        expect(eq(frame.duid, static_cast<std::uint8_t>(P25Duid::Ldu1)));

        P25PayloadLayer  payload{};
        const P25Message message = payload.decodePayload(frame, raw.data());
        expect(that % message.decoded()) << "the Reed-Solomon block over the Link Control hexbits decodes";
        expect(that % message.has_link_control);
        expect(eq(message.link_control.talkgroup, std::uint16_t{12345U}));
        expect(eq(message.link_control.source_unit, std::uint32_t{678901U}));
    };
};

int main() { /* tests are automatically registered and run */ }
