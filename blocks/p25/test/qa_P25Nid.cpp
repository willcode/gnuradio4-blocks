#include <boost/ut.hpp>

#include <cstdint>

#include <gnuradio-4.0/algorithm/fec/Bch.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

/*
 * qa for the P25 network identifier's protocol fields: Nid.hpp, on top of the BCH(63,16,23)
 * decoder it is built from. The code itself -- generator derivation, linearity, minimum
 * distance, correction radius -- is pinned once, library-side, by
 * gnuradio4-library/algorithm/test/qa_Bch.cpp; what belongs here is what TIA-102 does with
 * the codeword once it decodes: where the network access code and the data unit identifier
 * sit inside it, the independent trailing parity bit, which identifiers the standard defines,
 * the fixed frame lengths those identifiers carry, and the coarse classification a consumer
 * dispatches on.
 */
const boost::ut::suite<"P25Nid"> p25NidTests = [] {
    using namespace boost::ut;
    using namespace gr::p25;

    "the identifier's fields come out where TIA-102 puts them"_test = [] {
        const std::uint16_t nac  = 0x692U;
        const std::uint8_t  duid = static_cast<std::uint8_t>(P25Duid::Ldu1);
        const std::uint64_t nid  = (gr::fec::bch63Encode(static_cast<std::uint16_t>((nac << 4U) | duid)) << 1U) | duidParityBit(duid);
        const P25Nid        n    = decodeNid(nid);
        expect(that % n.valid) << "a well-formed identifier decodes";
        expect(eq(n.nac, nac)) << "the network access code is the identifier's top 12 bits";
        expect(eq(n.duid, duid)) << "the data unit identifier is the next 4";
        expect(that % n.parity_ok) << "the trailing parity bit agrees";
        expect(that % n.duid_defined) << "a voice frame's identifier is one TIA-102 defines";
        expect(eq(n.bch_errors, 0U)) << "nothing was corrected";
    };

    "the fields are not transposed"_test = [] {
        // Swapping the two fields must not be silently survivable: a 12-bit code in the low
        // bits and a 4-bit identifier in the high ones is a different identifier entirely.
        const P25Nid n = decodeNid((gr::fec::bch63Encode(0x6925U) << 1U) | duidParityBit(0x5U));
        expect(n.nac == 0x692U && n.duid == 0x5U) << "the fields are not transposed";
    };

    "the trailing parity bit is a function of the identifier alone"_test = [] {
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Hdu)), 0U)) << "a header's parity bit is 0";
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Tdu)), 0U)) << "a terminator's is 0";
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Tsbk)), 0U)) << "a trunking block's is 0";
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Pdu)), 0U)) << "packet data's is 0";
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Tdu15)), 0U)) << "a terminator with link control's is 0";
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Ldu1)), 1U)) << "a voice frame carrying link control has 1";
        expect(eq(duidParityBit(static_cast<std::uint8_t>(P25Duid::Ldu2)), 1U)) << "a voice frame carrying the encryption sync has 1";

        const std::uint8_t  duid = static_cast<std::uint8_t>(P25Duid::Ldu2);
        const std::uint64_t good = (gr::fec::bch63Encode(static_cast<std::uint16_t>((0x692U << 4U) | duid)) << 1U) | duidParityBit(duid);
        expect(that % decodeNid(good).parity_ok) << "the built identifier's parity bit agrees";
        expect(that % !decodeNid(good ^ 1ULL).parity_ok) << "flipping the trailing bit alone is noticed";
        expect(that % decodeNid(good ^ 1ULL).valid) << "and does not disturb the codeword, being outside it";
    };

    "the nine identifiers TIA-102 leaves undefined are recognized as such"_test = [] {
        bool right = true;
        for (unsigned d = 0U; d < 16U; ++d) {
            const bool defined = (d == 0x0U || d == 0x3U || d == 0x5U || d == 0x7U || d == 0xAU || d == 0xCU || d == 0xFU);
            right              = right && (duidIsDefined(static_cast<std::uint8_t>(d)) == defined);
        }
        expect(that % right) << "exactly seven of the sixteen identifiers are defined";
    };

    "the fixed frame lengths, which the status period has to divide"_test = [] {
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Hdu)), std::size_t{396U})) << "a header is 396 transmitted symbols";
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Tdu)), std::size_t{72U})) << "a terminator is 72";
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Ldu1)), std::size_t{864U})) << "a voice frame is 864";
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Ldu2)), std::size_t{864U})) << "both voice frames are 864";
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Tdu15)), std::size_t{216U})) << "a terminator with link control is 216";
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Tsbk)), std::size_t{0U})) << "a trunking frame's length is not fixed";
        expect(eq(duidTransmittedDibits(static_cast<std::uint8_t>(P25Duid::Pdu)), std::size_t{0U})) << "a data frame's length is not fixed";
        bool divides = true;
        for (unsigned d = 0U; d < 16U; ++d) {
            const std::size_t n = duidTransmittedDibits(static_cast<std::uint8_t>(d));
            divides             = divides && (n % kStatusSymbolPeriod == 0U);
        }
        expect(that % divides) << "every fixed frame length is a whole multiple of the 36-symbol status period";
    };

    "the coarse classification a consumer dispatches on"_test = [] {
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Hdu)) == P25FrameKind::VoiceHeader) << "a header opens voice";
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Ldu1)) == P25FrameKind::Voice) << "LDU1 is voice";
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Ldu2)) == P25FrameKind::Voice) << "LDU2 is voice";
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Tdu)) == P25FrameKind::Terminator) << "TDU terminates";
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Tdu15)) == P25FrameKind::Terminator) << "TDU15 terminates";
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Tsbk)) == P25FrameKind::Control) << "TSBK is control";
        expect(duidKind(static_cast<std::uint8_t>(P25Duid::Pdu)) == P25FrameKind::Data) << "PDU is data";
        expect(duidKind(0x1U) == P25FrameKind::Unknown) << "an undefined identifier classifies as unknown";
    };
};

int main() { /* tests are automatically registered and run */ }
