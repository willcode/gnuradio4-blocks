#include <boost/ut.hpp>

#include <cstdint>
#include <vector>

#include <gnuradio-4.0/p25/FrameGen.hpp>
#include <gnuradio-4.0/p25/FrameLayer.hpp>
#include <gnuradio-4.0/p25/Nid.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

/*
 * qa for the P25 Phase 1 frame layer: FrameLayer.hpp, driven by the transmitted dibit streams
 * FrameGen.hpp builds.
 *
 * Every identity assertion here is absolute: the stream is built from a network access code
 * and a sequence of data unit identifiers chosen in this file, and the layer has to give those
 * same values back. Nothing is compared against a second decoder's opinion, and nothing is
 * compared against a value this code measured.
 *
 * The stream carries its status symbols, so the layer only reads the right identifier if it
 * removes them from the right places -- which is why there is no separate "does it strip"
 * check here. A wrong period or a wrong phase shifts the identifier's 64 bits and the BCH
 * decode fails, so the census going to zero is that failure's signature.
 *
 * The last group is the one that matters most. The frame sync is composed entirely of the two
 * outer symbols, so a demodulator whose level tracking has widened in noise produces
 * outer-symbol runs that reach the pattern by accident. Feeding the layer exactly that -- a
 * stream of nothing but outer symbols -- is the adversarial case, and the requirement is that
 * candidates arrive and frames do not.
 */
namespace {

using namespace gr::p25;

struct Caught {
    std::vector<P25Frame> frames;
    P25FrameLayer         layer;
};

//! Run a stream through a layer, collecting every frame it reports.
Caught run(const std::vector<std::uint8_t>& dibits, unsigned maxSync = 4U, P25FrameAction action = P25FrameAction::Continue) {
    Caught c;
    c.layer.max_sync_errors = maxSync;
    for (const std::uint8_t d : dibits) {
        c.layer.push(d, [&](const P25Frame& f) {
            c.frames.push_back(f);
            return action;
        });
    }
    return c;
}

const std::uint16_t             kNac = 0x692U;
const std::vector<std::uint8_t> kDuids{
    static_cast<std::uint8_t>(P25Duid::Hdu),
    static_cast<std::uint8_t>(P25Duid::Ldu1),
    static_cast<std::uint8_t>(P25Duid::Ldu2),
    static_cast<std::uint8_t>(P25Duid::Ldu1),
    static_cast<std::uint8_t>(P25Duid::Ldu2),
    static_cast<std::uint8_t>(P25Duid::Tdu15),
    static_cast<std::uint8_t>(P25Duid::Tdu),
    static_cast<std::uint8_t>(P25Duid::Tdu),
    static_cast<std::uint8_t>(P25Duid::Tsbk),
    static_cast<std::uint8_t>(P25Duid::Tdu),
};

} // namespace

const boost::ut::suite<"P25FrameLayer"> p25FrameLayerTests = [] {
    using namespace boost::ut;
    using namespace gr::p25;

    "a clean stream gives back exactly the frames it was built from"_test = [] {
        const auto   stream = p25FrameStream(kNac, kDuids, 777U);
        const Caught c      = run(stream);

        expect(eq(c.frames.size(), kDuids.size())) << "every frame written is reported once";
        expect(eq(c.layer.candidates, kDuids.size())) << "and every sync in a clean stream is a candidate";
        expect(eq(c.layer.rejected(), std::uint64_t{0U})) << "nothing is refused in a clean stream";

        bool ids   = true;
        bool order = true;
        for (std::size_t i = 0U; i < c.frames.size() && i < kDuids.size(); ++i) {
            ids   = ids && (c.frames[i].nac == kNac) && (c.frames[i].bch_errors == 0U) && c.frames[i].parity_ok;
            order = order && (c.frames[i].duid == kDuids[i]);
        }
        expect(that % ids) << "every frame carries the network access code the stream was written with";
        expect(that % order) << "the data unit identifiers come back in the order they were written";

        // The reported position is the frame sync's first dibit, which the builder knows.
        std::size_t at    = 0U;
        bool        where = true;
        for (std::size_t i = 0U; i < c.frames.size() && i < kDuids.size(); ++i) {
            where                 = where && (c.frames[i].dibit_index == at);
            const std::size_t len = duidTransmittedDibits(kDuids[i]);
            at += (len != 0U) ? len : 180U;
        }
        expect(that % where) << "each frame is reported at the dibit its sync started on";

        expect(eq(c.layer.duid_census[static_cast<unsigned>(P25Duid::Ldu1)], std::uint64_t{2U})) << "the census counts both LDU1 frames";
        expect(eq(c.layer.duid_census[static_cast<unsigned>(P25Duid::Tdu)], std::uint64_t{3U})) << "and all three terminators";
        expect(c.frames[0].kind == P25FrameKind::VoiceHeader) << "the header classifies as a voice header";
        expect(c.frames[1].kind == P25FrameKind::Voice) << "a voice frame classifies as voice";
        expect(c.frames[8].kind == P25FrameKind::Control) << "a trunking frame classifies as control";
    };

    "the status symbol's value is immaterial: what matters is that it is removed"_test = [] {
        auto stream = p25FrameStream(kNac, kDuids, 777U);
        for (std::size_t i = 0U; i < stream.size(); ++i) {
            if (isStatusSymbol(i)) {
                stream[i] = static_cast<std::uint8_t>((i / 36U) & 0x3U); // cycle through all four
            }
        }
        const Caught c = run(stream);
        expect(eq(c.frames.size(), kDuids.size())) << "changing every status symbol changes nothing";
    };

    "damage inside a sync is tolerated up to the limit, and the frame still identifies"_test = [] {
        for (unsigned k = 0U; k <= 6U; ++k) {
            auto stream = p25FrameStream(kNac, kDuids, 31337U);
            // Damage only the second frame's sync, so the rest of the census stays a control.
            const std::size_t at = duidTransmittedDibits(kDuids[0]);
            for (unsigned i = 0U; i < k; ++i) {
                stream[at + i] = static_cast<std::uint8_t>(stream[at + i] ^ 0x3U);
            }
            const Caught c    = run(stream, 4U);
            const bool   kept = (c.frames.size() == kDuids.size());
            if (k <= 4U) {
                expect(that % kept) << "a sync damaged within tolerance still yields its frame";
                bool found = false;
                for (const P25Frame& f : c.frames) {
                    if (f.dibit_index == at) {
                        found = (f.nac == kNac) && (f.duid == kDuids[1]) && (f.sync_errors == k);
                    }
                }
                expect(that % found) << "and it identifies correctly, reporting the damage it carried";
            } else {
                expect(that % !kept) << "a sync damaged past tolerance is not found";
            }
        }
    };

    "damage inside the identifier is what the BCH is there for"_test = [] {
        // Wrong bits inside the identifier still identify the frame, up to the accepted
        // radius. Damaging whole dibits is the honest way to do it, since a wrong symbol is
        // what actually happens, and it costs two bits at a time.
        for (unsigned k = 1U; k * 2U <= P25FrameLayer{}.max_bch_errors; ++k) {
            auto              stream = p25FrameStream(kNac, kDuids, 4242U);
            const std::size_t at     = duidTransmittedDibits(kDuids[0]);
            for (unsigned i = 0U; i < k; ++i) {
                const std::size_t j = at + kFrameSyncDibits + i * 3U; // inside the identifier
                stream[j]           = static_cast<std::uint8_t>(stream[j] ^ 0x3U);
            }
            const Caught c  = run(stream);
            bool         ok = false;
            for (const P25Frame& f : c.frames) {
                if (f.dibit_index == at) {
                    ok = (f.nac == kNac) && (f.duid == kDuids[1]) && (f.bch_errors == 2U * k);
                }
            }
            expect(that % ok) << "an identifier with whole dibits wrong is corrected back to what was written";
        }

        {
            // Past the code's power the frame is refused rather than misreported.
            auto              stream = p25FrameStream(kNac, kDuids, 4242U);
            const std::size_t at     = duidTransmittedDibits(kDuids[0]);
            for (unsigned i = 0U; i < 12U; ++i) {
                const std::size_t j = at + kFrameSyncDibits + i;
                stream[j]           = static_cast<std::uint8_t>(~stream[j] & 0x3U);
            }
            const Caught c        = run(stream);
            bool         reported = false;
            for (const P25Frame& f : c.frames) {
                reported = reported || (f.dibit_index == at);
            }
            expect(that % !reported) << "an identifier damaged past the code's power is refused, not guessed at";
            expect(gt(c.layer.candidates, c.layer.frames)) << "and the refusal is counted";
        }
    };

    "outer-symbol noise reaches the sync pattern, but the identifier gate stops it there"_test = [] {
        std::uint64_t             rng = 0xDEADBEEFULL;
        std::vector<std::uint8_t> noise(4000000U);
        for (std::size_t i = 0U; i < noise.size(); ++i) {
            rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
            // Only the two outer symbols, which is what a widened level tracker emits: this is
            // the most favorable stream a false sync could possibly be drawn from.
            noise[i] = ((rng >> 40U) & 1U) ? 1U : 3U;
        }
        P25FrameLayer layer;
        layer.max_sync_errors = 4U;
        std::uint64_t frames  = 0U;
        for (const std::uint8_t d : noise) {
            layer.push(d, [&](const P25Frame&) {
                ++frames;
                return P25FrameAction::Continue;
            });
        }
        expect(gt(layer.candidates, std::uint64_t{500U})) << "outer-symbol noise does reach the sync pattern, as it must for this to test anything";
        // The identifier's refusal is statistical, not structural: a field that was never
        // transmitted still has a small chance of lying inside the code's accepted radius. The
        // bound is what that chance supports, and it is three orders of magnitude below the
        // candidate count, so a layer that stopped checking the identifier fails it by a mile.
        expect(lt(frames * 1000U, layer.candidates)) << "the identifier refuses at least 99.9 % of them";
        expect(le(frames, std::uint64_t{2U})) << "and the handful it cannot refuse stays inside what the code's radius predicts";
    };

    "acting on a retune request costs nothing at any frame spacing TIA-102 defines"_test = [] {
        const auto   stream = p25FrameStream(kNac, kDuids, 909U);
        const Caught plain  = run(stream, 4U, P25FrameAction::Continue);
        const Caught moved  = run(stream, 4U, P25FrameAction::Retune);

        expect(eq(moved.layer.retunes, moved.layer.frames)) << "every retune request is acted on";
        expect(eq(moved.frames.size(), plain.frames.size())) << "and discarding the held dibits loses no frame";
        bool same = true;
        for (std::size_t i = 0U; i < moved.frames.size() && i < plain.frames.size(); ++i) {
            same = same && (moved.frames[i].duid == plain.frames[i].duid) && (moved.frames[i].dibit_index == plain.frames[i].dibit_index);
        }
        expect(that % same) << "the frames are the same frames";
    };

    "and the discard itself really discards: a frame straddling it is not reported"_test = [] {
        const auto            stream = p25FrameStream(kNac, kDuids, 5150U);
        P25FrameLayer         layer;
        std::vector<P25Frame> got;
        for (std::size_t i = 0U; i < stream.size(); ++i) {
            // Reset one dibit before the first frame would be reported, so its sync is in the
            // delay line and its identifier is not yet complete.
            if (i == kFrameHeaderDibits - 1U) {
                layer.reset();
            }
            layer.push(stream[i], [&](const P25Frame& f) {
                got.push_back(f);
                return P25FrameAction::Continue;
            });
        }
        expect(eq(got.size(), kDuids.size() - 1U)) << "the frame whose identifier straddled the discard is gone";
        expect(!got.empty() && got[0].duid == kDuids[1]) << "and the next frame is the first one reported";
    };
};

int main() { /* tests are automatically registered and run */ }
