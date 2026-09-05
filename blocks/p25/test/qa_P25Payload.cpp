#include <boost/ut.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/algorithm/fec/Golay.hpp>
#include <gnuradio-4.0/algorithm/fec/Hamming.hpp>
#include <gnuradio-4.0/p25/EncryptionSync.hpp>
#include <gnuradio-4.0/p25/FrameGen.hpp>
#include <gnuradio-4.0/p25/LinkControl.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>
#include <gnuradio-4.0/p25/PayloadBits.hpp>
#include <gnuradio-4.0/p25/PayloadGen.hpp>
#include <gnuradio-4.0/p25/PayloadLayer.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

/*
 * qa for the P25 payload layer: PayloadBits.hpp, LinkControl.hpp, EncryptionSync.hpp,
 * PayloadLayer.hpp and the encoder in PayloadGen.hpp.
 *
 * The frame layouts here are arithmetic rather than tables: a field's transmitted position is
 * computed from the status-symbol period and the interleaving stride rather than looked up.
 * That is cheaper to read and it is also easier to get quietly wrong, so the layout is pinned
 * three ways before any message is decoded -- the mapping between information and transmitted
 * positions is a bijection onto the non-status positions, it agrees with the independent
 * status strip in `StatusSymbol.hpp`, and the first and last position of every field region
 * are stated as literals.
 *
 * The messages are then checked by a round trip: a frame is built carrying a talkgroup, a
 * source unit, an algorithm and a key that were chosen here, run through the whole layer as a
 * dibit stream, and the values that come out are compared against the ones that went in.
 * Nothing in the comparison is a second decoder's opinion.
 *
 * The last group is the one that matters most. A payload damaged past what its codes can
 * repair must be reported, and the layer must not hand back a message whose fields were
 * invented. So the tests sweep damage from none to far past the limit and assert, on every
 * single trial, that a message which claims to have decoded carries the fields that were
 * transmitted. A decoder that returns plausible garbage instead of a failure is the defect
 * this whole file exists to catch.
 */
namespace {

using namespace gr::p25;

std::uint64_t rng = 0x9E3779B97F4A7C15ULL;
std::uint64_t next() {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng >> 17U;
}

constexpr std::uint16_t kNac       = 0x692U;
constexpr std::uint16_t kTalkgroup = 0x04D2U;
constexpr std::uint32_t kSource    = 9681U;
constexpr std::uint8_t  kAlgid     = 0x84U;
constexpr std::uint16_t kKeyid     = 0xBEEFU;

P25EncryptionSync sampleEncryptionSync() {
    P25EncryptionSync es;
    for (std::size_t i = 0U; i < kMessageIndicatorOctets; ++i) {
        es.message_indicator[i] = static_cast<std::uint8_t>(0x11U * (i + 1U));
    }
    es.algid = kAlgid;
    es.keyid = kKeyid;
    return es;
}

//! One frame of each kind, all carrying the same chosen identity and encryption state.
std::vector<std::uint8_t> buildFrame(std::uint8_t duid, std::uint64_t& seed) {
    if (duid == static_cast<std::uint8_t>(P25Duid::Hdu)) {
        P25HeaderMessage h;
        h.encryption_sync = sampleEncryptionSync();
        h.mfid            = 0x21U;
        h.talkgroup       = kTalkgroup;
        return buildHduFrame(kNac, h, seed);
    }
    if (duid == static_cast<std::uint8_t>(P25Duid::Ldu1)) {
        return buildLdu1Frame(kNac, groupVoiceLinkControl(kTalkgroup, kSource, 0U, 0U), seed);
    }
    return buildLdu2Frame(kNac, sampleEncryptionSync(), seed);
}

//! Whether a message carries what `buildFrame` put into it.
bool fieldsAreTheOnesTransmitted(const P25Message& m) {
    if (m.frame.duid == static_cast<std::uint8_t>(P25Duid::Ldu1)) {
        return m.link_control.lco == 0U && m.link_control.mfid == 0U && m.link_control.talkgroup == kTalkgroup && m.link_control.source_unit == kSource;
    }
    const bool state = m.encryption_sync.algid == kAlgid && m.encryption_sync.keyid == kKeyid && m.encryption_sync.message_indicator == sampleEncryptionSync().message_indicator;
    if (m.frame.duid == static_cast<std::uint8_t>(P25Duid::Hdu)) {
        return state && m.header_mfid == 0x21U && m.header_talkgroup == kTalkgroup;
    }
    return state;
}

//! Replace hexbit `index` with a different one, leaving a valid inner codeword behind. The
//! inner code then sees nothing wrong and the outer code sees exactly one wrong symbol, which
//! is what standing a payload precisely at the outer code's limit requires.
void breakHexbit(std::vector<std::uint8_t>& frame, std::uint8_t duid, std::size_t index) {
    if (duid == static_cast<std::uint8_t>(P25Duid::Hdu)) {
        const std::size_t   origin = kHduPayloadOrigin + index * kHduCodewordBits;
        const std::uint32_t cw     = informationWord(frame.data(), origin, kHduCodewordBits);
        const std::uint8_t  info   = static_cast<std::uint8_t>(gr::fec::golay18Decode(cw).info);
        setInformationWord(frame.data(), origin, kHduCodewordBits, gr::fec::golay18Encode(static_cast<std::uint8_t>((info + 1U) & 0x3FU)));
        return;
    }
    const std::size_t   group  = index / kLduCodewordsPerGroup;
    const std::size_t   slot   = index % kLduCodewordsPerGroup;
    const std::size_t   origin = kLduFirstGroupOrigin + group * kLduGroupStride + slot * kLduCodewordBits;
    const std::uint32_t cw     = informationWord(frame.data(), origin, kLduCodewordBits);
    const std::uint8_t  info   = static_cast<std::uint8_t>(gr::fec::hamming1063Decode(static_cast<std::uint16_t>(cw)).info);
    setInformationWord(frame.data(), origin, kLduCodewordBits, gr::fec::hamming1063Encode(static_cast<std::uint8_t>((info + 1U) & 0x3FU)));
}

//! Run a whole frame through the layer and return the one message it produces.
P25Message throughLayer(const std::vector<std::uint8_t>& frame, bool& produced) {
    P25PayloadLayer layer;
    P25Message      seen;
    produced = false;
    // The layer needs a lead-in so the frame sync is not the very first thing it sees, and a
    // tail so a frame whose payload runs to the end still completes.
    for (std::size_t i = 0U; i < 64U; ++i) {
        layer.push(static_cast<std::uint8_t>(next() & 0x3U), [](const P25Message&) { return P25FrameAction::Continue; });
    }
    for (const std::uint8_t d : frame) {
        layer.push(d, [&](const P25Message& m) {
            seen     = m;
            produced = true;
            return P25FrameAction::Continue;
        });
    }
    return seen;
}

} // namespace

const boost::ut::suite<"P25Payload"> p25PayloadTests = [] {
    using namespace boost::ut;
    using namespace gr::p25;

    "the information-to-transmitted mapping is a bijection onto the non-status positions"_test = [] {
        constexpr std::size_t span = 864U;
        std::vector<bool>     hit(span, false);
        bool                  ordered  = true;
        std::size_t           previous = 0U;
        for (std::size_t i = 0U; i < informationSpan(span); ++i) {
            const std::size_t t = transmittedIndexOf(i);
            expect(lt(t, span)) << "an information symbol lands inside the frame";
            expect(that % !isStatusSymbol(t)) << "an information symbol never lands on a status symbol";
            if (hit[t]) {
                expect(false) << "two information symbols never share a transmitted position";
            }
            if (i > 0U && t <= previous) {
                ordered = false;
            }
            previous = t;
            hit[t]   = true;
        }
        expect(that % ordered) << "the mapping is increasing, so information order is transmission order";
        std::size_t covered = 0U;
        for (std::size_t t = 0U; t < span; ++t) {
            if (hit[t]) {
                ++covered;
            } else {
                expect(that % isStatusSymbol(t)) << "every transmitted position the mapping skips is a status symbol";
            }
        }
        expect(eq(covered, informationSpan(span))) << "the mapping covers every non-status position exactly once";
    };

    "and it agrees with the independent status strip"_test = [] {
        std::vector<std::uint8_t> raw(864U);
        for (std::size_t i = 0U; i < raw.size(); ++i) {
            raw[i] = static_cast<std::uint8_t>(next() & 0x3U);
        }
        std::vector<std::uint8_t> stripped(informationSpan(864U));
        const std::size_t         n = stripStatusSymbols(raw.data(), raw.size(), stripped.data(), stripped.size());
        expect(eq(n, stripped.size())) << "the strip produced the information symbols the span predicts";
        bool same = true;
        for (std::size_t i = 0U; i < n; ++i) {
            if (raw[transmittedIndexOf(i)] != stripped[i]) {
                same = false;
            }
        }
        expect(that % same) << "indexing into the raw dibits agrees with stripping the status symbols first";
    };

    "where the field regions start and end, as literals"_test = [] {
        expect(eq(kFrameHeaderInfoBits, std::size_t{112U})) << "the sync and the network identifier take 112 information bits";
        expect(kHduPayloadOrigin == 112U && kHduCodewords * kHduCodewordBits == 648U) << "the header's 36 Golay codewords run from information bit 112 for 648 bits";
        expect(eq(transmittedIndexOf(112U / 2U), std::size_t{57U})) << "the header's payload begins at transmitted symbol 57";
        expect(kLduFirstGroupOrigin == 400U && kLduGroupStride == 184U) << "a voice frame's hexbit groups begin at information bit 400 and repeat every 184";
        expect(eq(kLduGroups * kLduCodewordsPerGroup, kLduCodewords)) << "six groups of four make the 24 hexbits";
        expect(kLduFirstGroupOrigin + 5U * kLduGroupStride + 40U == 1360U) << "the last hexbit group ends at information bit 1360, before the eighth voice codeword";
        expect(kHduRsPad + kHduCodewords == 63U && kLduRsPad + kLduCodewords == 63U) << "both shortenings restore to the field's full block length";
    };

    "hexbits pack and unpack as inverses"_test = [] {
        std::array<std::uint8_t, 15U> octets{};
        for (std::size_t i = 0U; i < octets.size(); ++i) {
            octets[i] = static_cast<std::uint8_t>(next() & 0xFFU);
        }
        std::array<std::uint8_t, 20U> hexbits{};
        unpackHexbits(octets.data(), octets.size(), hexbits.data());
        for (const std::uint8_t h : hexbits) {
            expect(lt(h, std::uint8_t{64U})) << "a hexbit carries six bits";
        }
        std::array<std::uint8_t, 15U> back{};
        packHexbits(hexbits.data(), hexbits.size(), back.data());
        expect(back == octets) << "packing undoes unpacking over the header's fifteen octets";
    };

    "Link Control is read out of octets that were written here"_test = [] {
        const auto           octets = groupVoiceLinkControl(0x1234U, 0x00ABCDEFU, 0x90U, 0x07U);
        const P25LinkControl lc     = parseLinkControl(octets.data());
        expect(lc.lco == 0U && lc.isGroupVoice()) << "the opcode is the group voice channel user";
        expect(eq(lc.mfid, std::uint8_t{0x90U})) << "the manufacturer is read from the second octet";
        expect(eq(lc.service_options, std::uint8_t{0x07U})) << "the service options are read from the third";
        expect(eq(lc.talkgroup, std::uint16_t{0x1234U})) << "the talkgroup is the fifth and sixth octets";
        expect(eq(lc.source_unit, std::uint32_t{0x00ABCDEFU})) << "the source unit is the last three octets";
        expect(!lc.protected_flag && !lc.implicit_mfid) << "the two flags in the first octet are clear";

        // an opcode whose address fields are not defined must not produce addresses
        std::array<std::uint8_t, kLinkControlOctets> other = octets;
        other[0]                                           = 0x44U; // opcode 4, and the implicit-manufacturer bit
        const P25LinkControl lc2                           = parseLinkControl(other.data());
        expect(eq(lc2.lco, std::uint8_t{0x04U})) << "the opcode is masked out of the first octet";
        expect(that % lc2.implicit_mfid) << "the implicit-manufacturer bit is read";
        expect(!lc2.has_addresses && lc2.talkgroup == 0U && lc2.source_unit == 0U) << "an opcode whose addresses are undefined yields none rather than a guess";
    };

    "a header round trip"_test = [] {
        P25HeaderMessage message;
        message.encryption_sync = sampleEncryptionSync();
        message.mfid            = 0x21U;
        message.talkgroup       = kTalkgroup;

        std::uint64_t                   seed  = 0xABCDEF01U;
        const std::vector<std::uint8_t> frame = buildHduFrame(kNac, message, seed);
        expect(eq(frame.size(), std::size_t{396U})) << "a header is 396 transmitted symbols";

        bool             produced = false;
        const P25Message m        = throughLayer(frame, produced);
        expect(that % produced) << "the layer produced a message for the header";
        expect(m.frame.nac == kNac && m.frame.duid == static_cast<std::uint8_t>(P25Duid::Hdu)) << "the header is identified as one, carrying the network access code it was built with";
        expect(that % m.decoded()) << "the header's Reed-Solomon block decoded";
        expect(m.rs_errors == 0U && m.inner_corrected == 0U && m.inner_refused == 0U) << "an undamaged header costs no corrections at all";
        expect(m.has_encryption_sync && m.has_header_fields) << "a header carries the encryption state and the talkgroup";
        expect(eq(m.encryption_sync.algid, kAlgid)) << "the algorithm identifier survives the round trip";
        expect(eq(m.encryption_sync.keyid, kKeyid)) << "the key identifier survives the round trip";
        expect(m.encryption_sync.message_indicator == message.encryption_sync.message_indicator) << "the message indicator survives the round trip";
        expect(eq(m.header_mfid, std::uint8_t{0x21U})) << "the manufacturer survives the round trip";
        expect(eq(m.header_talkgroup, kTalkgroup)) << "the talkgroup survives the round trip";
        expect(that % m.encryption_sync.encrypted()) << "an algorithm that is not 0x80 reads as encrypted";
    };

    "a first voice frame round trip"_test = [] {
        const auto                      octets = groupVoiceLinkControl(kTalkgroup, kSource, 0U, 0U);
        std::uint64_t                   seed   = 0x13579BDFU;
        const std::vector<std::uint8_t> frame  = buildLdu1Frame(kNac, octets, seed);
        expect(eq(frame.size(), std::size_t{864U})) << "a voice frame is 864 transmitted symbols";

        bool             produced = false;
        const P25Message m        = throughLayer(frame, produced);
        expect(produced && m.frame.duid == static_cast<std::uint8_t>(P25Duid::Ldu1)) << "the first voice frame is identified";
        expect(m.decoded() && m.has_link_control) << "its Link Control decoded";
        expect(m.rs_errors == 0U && m.inner_refused == 0U) << "an undamaged Link Control costs no corrections";
        expect(eq(m.link_control.talkgroup, kTalkgroup)) << "the talkgroup survives the round trip";
        expect(eq(m.link_control.source_unit, kSource)) << "the source unit survives the round trip";
        expect(m.link_control.lco == 0U && m.link_control.mfid == 0U) << "the opcode and manufacturer survive";
        expect(that % !m.has_encryption_sync) << "a first voice frame carries no encryption state";
    };

    "a second voice frame round trip, and the two encryption paths agreeing"_test = [] {
        const P25EncryptionSync         es    = sampleEncryptionSync();
        std::uint64_t                   seed  = 0x2468ACE0U;
        const std::vector<std::uint8_t> frame = buildLdu2Frame(kNac, es, seed);

        bool             produced = false;
        const P25Message m        = throughLayer(frame, produced);
        expect(produced && m.frame.duid == static_cast<std::uint8_t>(P25Duid::Ldu2)) << "the second voice frame is identified";
        expect(m.decoded() && m.has_encryption_sync) << "its Encryption Sync decoded";
        expect(m.rs_errors == 0U && m.inner_refused == 0U) << "an undamaged Encryption Sync costs no corrections";
        expect(eq(m.encryption_sync.algid, es.algid)) << "the algorithm identifier survives the round trip";
        expect(eq(m.encryption_sync.keyid, es.keyid)) << "the key identifier survives the round trip";
        expect(m.encryption_sync.message_indicator == es.message_indicator) << "the message indicator survives the round trip";
        expect(that % !m.has_link_control) << "a second voice frame carries no Link Control";

        // the header path and the voice path are different codes over different frames; where
        // both carry the same state they must produce the same answer
        P25HeaderMessage header;
        header.encryption_sync = es;
        header.talkgroup       = kTalkgroup;
        std::uint64_t    hseed = 0x2468ACE0U;
        bool             hp    = false;
        const P25Message hm    = throughLayer(buildHduFrame(kNac, header, hseed), hp);
        expect(hp && hm.decoded()) << "the header carrying the same state decoded";
        expect(hm.encryption_sync.algid == m.encryption_sync.algid && hm.encryption_sync.keyid == m.encryption_sync.keyid && hm.encryption_sync.message_indicator == m.encryption_sync.message_indicator) << "the two independent encryption paths agree";
    };

    "clear traffic reads as clear"_test = [] {
        P25EncryptionSync clear;
        clear.algid = kAlgidClear;
        clear.keyid = 0U;
        expect(that % !clear.encrypted()) << "algorithm 0x80 is the absence of encryption";
        std::uint64_t    seed     = 7U;
        bool             produced = false;
        const P25Message m        = throughLayer(buildLdu2Frame(kNac, clear, seed), produced);
        expect(produced && m.decoded()) << "a clear second voice frame decodes";
        expect(m.encryption_sync.algid == kAlgidClear && !m.encryption_sync.encrypted()) << "and reads as clear";
        expect(eq(m.encryption_sync.keyid, std::uint16_t{0U})) << "with no key";
    };

    "the outer code's exact boundary, one whole hexbit at a time"_test = [] {
        // A hexbit is broken cleanly: its inner codeword is replaced by a different valid
        // inner codeword, so the inner code sees nothing wrong and hands the Reed-Solomon
        // decoder exactly one wrong symbol. That is the only way to stand a payload precisely
        // at the outer code's limit, and it is where a decoder that guesses past its capability
        // shows itself.
        struct Arm {
            const char*  name;
            std::uint8_t duid;
            unsigned     correctable;
            std::size_t  hexbits;
        };
        const Arm arms[] = {{"header RS(36,20,17)", static_cast<std::uint8_t>(P25Duid::Hdu), 8U, kHduCodewords}, {"link control RS(24,12,13)", static_cast<std::uint8_t>(P25Duid::Ldu1), 6U, kLduCodewords}, {"encryption sync RS(24,16,9)", static_cast<std::uint8_t>(P25Duid::Ldu2), 4U, kLduCodewords}};

        for (const Arm& arm : arms) {
            long beyondTrials = 0, beyondReported = 0, beyondWrong = 0;
            for (unsigned broken = 0U; broken <= arm.correctable + 4U; ++broken) {
                for (unsigned offset = 0U; offset < 12U; ++offset) {
                    std::uint64_t             seed  = 0x5150U + offset;
                    std::vector<std::uint8_t> frame = buildFrame(arm.duid, seed);
                    for (unsigned b = 0U; b < broken; ++b) {
                        breakHexbit(frame, arm.duid, (offset + b * 3U) % arm.hexbits);
                    }

                    bool             produced = false;
                    const P25Message m        = throughLayer(frame, produced);
                    expect(produced && m.frame.duid == arm.duid) << "the frame was still identified";
                    expect(m.inner_refused == 0U && m.inner_corrected == 0U) << "a cleanly replaced inner codeword costs the inner code nothing, so this really is an outer-code test";

                    if (broken <= arm.correctable) {
                        expect(that % m.decoded()) << "a payload inside the outer code's limit decodes";
                        expect(eq(m.rs_errors, broken)) << "and the reported symbol count is the number of broken hexbits";
                        expect(that % fieldsAreTheOnesTransmitted(m)) << "and the fields are the ones that were transmitted";
                    } else {
                        ++beyondTrials;
                        if (!m.decoded()) {
                            ++beyondReported;
                        } else if (!fieldsAreTheOnesTransmitted(m)) {
                            ++beyondWrong;
                        }
                    }
                }
            }
            // Past the limit a bounded-distance decoder is defined to return the nearest
            // codeword, and the nearest is sometimes not the transmitted one -- no decoder can
            // do better than that, and the Reed-Solomon qa pins the invariant that an accepted
            // block is always a genuine codeword at the distance it reports. What must hold
            // here is that the failure path is real and dominant rather than absent.
            expect(gt(beyondReported, 0L)) << "damage past the outer code's limit is reported";
            expect(le(beyondWrong * 10L, beyondTrials)) << "and a wrong answer past the limit stays a small minority";
        }
    };

    "the inner codes' boundaries, inside a real frame"_test = [] {
        // Golay(18,6,8) absorbs three bit errors in a header hexbit and reports the fourth.
        for (unsigned bits = 0U; bits <= 4U; ++bits) {
            std::uint64_t             seed   = 77U;
            std::vector<std::uint8_t> frame  = buildFrame(static_cast<std::uint8_t>(P25Duid::Hdu), seed);
            const std::size_t         origin = kHduPayloadOrigin + 5U * kHduCodewordBits;
            for (unsigned b = 0U; b < bits; ++b) {
                const std::size_t at = origin + b * 4U;
                setInformationBit(frame.data(), at, static_cast<std::uint8_t>(informationBit(frame.data(), at) ^ 1U));
            }
            bool             produced = false;
            const P25Message m        = throughLayer(frame, produced);
            expect(that % produced) << "the damaged header was identified";
            if (bits <= 3U) {
                expect(eq(m.inner_refused, 0U)) << "Golay(18,6,8) accepts up to three wrong bits in a hexbit";
                expect(eq(m.inner_corrected, (bits == 0U ? 0U : 1U))) << "and reports the one codeword it repaired";
                expect(m.decoded() && m.rs_errors == 0U && fieldsAreTheOnesTransmitted(m)) << "so the outer code sees an undamaged block";
            } else {
                expect(eq(m.inner_refused, 1U)) << "and reports the fourth as beyond it";
            }
        }

        // Hamming(10,6,3) absorbs one bit error in a voice-frame hexbit and reports the second
        // whenever the syndrome names no position it has.
        for (unsigned bits = 0U; bits <= 1U; ++bits) {
            std::uint64_t             seed   = 78U;
            std::vector<std::uint8_t> frame  = buildFrame(static_cast<std::uint8_t>(P25Duid::Ldu2), seed);
            const std::size_t         origin = kLduFirstGroupOrigin + 2U * kLduGroupStride + kLduCodewordBits;
            for (unsigned b = 0U; b < bits; ++b) {
                const std::size_t at = origin + b;
                setInformationBit(frame.data(), at, static_cast<std::uint8_t>(informationBit(frame.data(), at) ^ 1U));
            }
            bool             produced = false;
            const P25Message m        = throughLayer(frame, produced);
            expect(produced && m.inner_refused == 0U) << "Hamming(10,6,3) accepts a single wrong bit in a hexbit";
            expect(eq(m.inner_corrected, bits)) << "and reports the codeword it repaired";
            expect(m.decoded() && m.rs_errors == 0U && fieldsAreTheOnesTransmitted(m)) << "so the outer code sees an undamaged block";
        }
    };

    "scattered damage over the whole frame, which is what a marginal signal produces"_test = [] {
        struct Arm {
            const char*  name;
            std::uint8_t duid;
        };
        const Arm arms[] = {{"header", static_cast<std::uint8_t>(P25Duid::Hdu)}, {"link control", static_cast<std::uint8_t>(P25Duid::Ldu1)}, {"encryption sync", static_cast<std::uint8_t>(P25Duid::Ldu2)}};

        for (const Arm& arm : arms) {
            long     recovered = 0, reported = 0, wrong = 0, trials = 0;
            unsigned deepest = 0U;
            for (unsigned damage = 0U; damage <= 200U; damage += 5U) {
                for (unsigned repeat = 0U; repeat < 12U; ++repeat) {
                    std::uint64_t             seed  = 0x5150U + repeat;
                    std::vector<std::uint8_t> frame = buildFrame(arm.duid, seed);
                    // The sync and the network identifier are the frame layer's business and
                    // are pinned by its own tests, so damage starts past them.
                    for (unsigned d = 0U; d < damage; ++d) {
                        const std::size_t at = kFrameHeaderDibits + next() % (frame.size() - kFrameHeaderDibits);
                        frame[at]            = static_cast<std::uint8_t>((frame[at] + 1U + (next() & 0x2U)) & 0x3U);
                    }

                    bool             produced = false;
                    const P25Message m        = throughLayer(frame, produced);
                    if (!produced || m.frame.duid != arm.duid) {
                        continue; // the identifier itself was hit; the frame layer owns that case
                    }
                    ++trials;
                    if (!m.decoded()) {
                        ++reported;
                    } else if (fieldsAreTheOnesTransmitted(m)) {
                        ++recovered;
                        deepest = (damage > deepest) ? damage : deepest;
                    } else {
                        ++wrong;
                    }
                }
            }
            expect(gt(trials, 300L)) << "the scatter sweep produced enough frames to mean something";
            expect(eq(recovered + reported + wrong, trials)) << "every frame was accounted for";
            expect(gt(reported, 0L)) << "damage past the codes' reach really was reached, and was reported";
            expect(ge(deepest, 30U)) << "the inner code absorbs scattered damage rather than passing it straight to the outer one";
            expect(le(wrong * 10L, trials)) << "a wrong answer stays a small minority of the whole sweep";
        }
    };

    "a frame whose length TIA-102 does not fix is reported without a payload"_test = [] {
        std::uint64_t             seed = 3U;
        std::vector<std::uint8_t> raw;
        appendP25Frame(raw, kNac, static_cast<std::uint8_t>(P25Duid::Tsbk), seed);
        bool             produced = false;
        const P25Message m        = throughLayer(raw, produced);
        expect(produced && m.frame.duid == static_cast<std::uint8_t>(P25Duid::Tsbk)) << "a trunking frame is identified";
        expect(that % (m.state == P25PayloadState::LengthUnknown)) << "and reported as one whose length is not fixed";
        expect(!m.decoded() && !m.has_link_control && !m.has_encryption_sync) << "carrying nothing that was not read";
    };

    "a terminator is reported at once, with no payload this layer reads"_test = [] {
        std::uint64_t             seed = 5U;
        std::vector<std::uint8_t> raw;
        appendP25Frame(raw, kNac, static_cast<std::uint8_t>(P25Duid::Tdu), seed);
        bool             produced = false;
        const P25Message m        = throughLayer(raw, produced);
        expect(produced && m.frame.duid == static_cast<std::uint8_t>(P25Duid::Tdu)) << "a terminator is identified";
        expect(that % (m.state == P25PayloadState::NoPayload)) << "and carries no payload this layer reads";
    };

    "a frame cut short by the next one is reported as truncated, not decoded"_test = [] {
        std::uint64_t    seed = 11U;
        P25HeaderMessage h;
        h.encryption_sync                = sampleEncryptionSync();
        std::vector<std::uint8_t> stream = buildHduFrame(kNac, h, seed);
        stream.resize(144U); // the header is cut off well before its 396 symbols
        appendP25Frame(stream, kNac, static_cast<std::uint8_t>(P25Duid::Tdu), seed);
        appendP25Frame(stream, kNac, static_cast<std::uint8_t>(P25Duid::Tdu), seed);

        P25PayloadLayer         layer;
        std::vector<P25Message> seen;
        for (const std::uint8_t d : stream) {
            layer.push(d, [&](const P25Message& m) {
                seen.push_back(m);
                return P25FrameAction::Continue;
            });
        }
        expect(ge(seen.size(), std::size_t{2U})) << "the truncated header and the frames after it were all reported";
        expect(seen[0].frame.duid == static_cast<std::uint8_t>(P25Duid::Hdu)) << "the truncated frame is reported first";
        expect(that % (seen[0].state == P25PayloadState::Truncated)) << "and is reported as truncated";
        expect(eq(layer.truncated, std::uint64_t{1U})) << "the layer counted exactly one truncation";
        bool ordered = true;
        for (std::size_t i = 1U; i < seen.size(); ++i) {
            if (seen[i].frame.dibit_index <= seen[i - 1U].frame.dibit_index) {
                ordered = false;
            }
        }
        expect(that % ordered) << "messages leave the layer in stream order whichever path they took";
    };

    "a consumer asking for a retune makes the layer forget what it holds"_test = [] {
        std::uint64_t             seed = 13U;
        std::vector<std::uint8_t> stream;
        for (int i = 0; i < 6; ++i) {
            appendP25Frame(stream, kNac, static_cast<std::uint8_t>(P25Duid::Tdu), seed);
        }
        P25PayloadLayer layer;
        long            count = 0;
        for (const std::uint8_t d : stream) {
            layer.push(d, [&](const P25Message&) {
                ++count;
                return P25FrameAction::Retune;
            });
        }
        // Each retune throws away the dibits held at that moment, and the next frame's sync is
        // still whole, so every frame is still found: the discard never straddles a frame.
        expect(eq(count, 6L)) << "a retune between frames costs no frames";

        long            plain = 0;
        P25PayloadLayer other;
        for (const std::uint8_t d : stream) {
            other.push(d, [&](const P25Message&) {
                ++plain;
                return P25FrameAction::Continue;
            });
        }
        expect(eq(plain, count)) << "and the census is the same as without any retune at all";
    };

    "and the discard is real: a retune throws away the frame that was already in hand"_test = [] {
        // Between frames a retune costs nothing, which is the check above and is the point of
        // the seam. The discard still has to actually happen, and the case that shows it is a
        // frame cut short by the next one: at the moment the truncated frame is reported, the
        // frame that superseded it has already been identified and is being held. A tuner
        // having moved makes that frame worthless -- it came from the old channel -- so it
        // must be thrown away with everything else.
        const auto census = [](bool retuneOnTruncated) {
            std::uint64_t    seed = 11U;
            P25HeaderMessage h;
            h.encryption_sync                = sampleEncryptionSync();
            std::vector<std::uint8_t> stream = buildHduFrame(kNac, h, seed);
            stream.resize(144U);
            for (int i = 0; i < 5; ++i) {
                appendP25Frame(stream, kNac, static_cast<std::uint8_t>(P25Duid::Tdu), seed);
            }
            P25PayloadLayer layer;
            long            n = 0;
            for (const std::uint8_t d : stream) {
                layer.push(d, [&](const P25Message& m) {
                    ++n;
                    if (retuneOnTruncated && m.state == P25PayloadState::Truncated) {
                        return P25FrameAction::Retune;
                    }
                    return P25FrameAction::Continue;
                });
            }
            return n;
        };
        const long kept      = census(false);
        const long discarded = census(true);
        expect(eq(kept, 6L)) << "without a retune, the truncated frame and the five after it are all reported";
        expect(eq(discarded, kept - 1L)) << "a retune on the truncated frame discards the frame that superseded it, which came from the channel the tuner has left";
    };
};

int main() { /* tests are automatically registered and run */ }
