#include <boost/ut.hpp>

#include <array>
#include <cstdint>
#include <vector>

#include <gnuradio-4.0/p25/FrameSync.hpp>
#include <gnuradio-4.0/p25/StatusSymbol.hpp>

/*
 * qa for the P25 frame-sync correlator and the status-symbol arithmetic: FrameSync.hpp and
 * StatusSymbol.hpp.
 *
 * Two things are pinned here and both are absolute rather than comparative. The correlator's
 * distance is counted in dibits, so a symbol decision that gets both bits of a dibit wrong is
 * one error and not two -- that distinction is invisible to a test that only ever damages one
 * bit at a time, so it is damaged deliberately both ways. And the status-symbol period and
 * phase are checked against the frame lengths TIA-102 fixes: a terminator is 72 transmitted
 * symbols, a voice frame 864, a header 396, a terminator with link control 216. Those numbers
 * come from the standard, not from this code, and they only come out right if a frame carrying
 * 35k information symbols is transmitted as exactly 36k.
 */
namespace {

using namespace gr::p25;

//! Pack a dibit sequence into the correlator's 48-bit window, most significant pair first.
std::uint64_t pack(const std::vector<std::uint8_t>& d) {
    std::uint64_t w = 0U;
    for (const std::uint8_t v : d) {
        w = ((w << 2) | (v & 0x3U)) & kFrameSyncMask;
    }
    return w;
}

} // namespace

const boost::ut::suite<"P25FrameSync"> p25FrameSyncTests = [] {
    using namespace boost::ut;
    using namespace gr::p25;

    "the pattern is 24 dibits and every one of them is an outer symbol"_test = [] {
        const auto sync = frameSyncDibits();
        expect(eq(sync.size(), std::size_t{24U})) << "the frame sync is 24 dibits";

        bool allOuter = true;
        for (const std::uint8_t d : sync) {
            allOuter = allOuter && (d == 1U || d == 3U);
        }
        expect(that % allOuter) << "every frame sync dibit is +3 or -3, never an inner symbol";

        expect(eq(pack(std::vector<std::uint8_t>(sync.begin(), sync.end())), kFrameSyncMagic)) << "the unpacked dibits repack to the TIA-102 pattern 0x5575F5FF77FF";
    };

    "distance is zero on the pattern and counts dibits, not bits"_test = [] {
        const auto sync = frameSyncDibits();
        expect(eq(frameSyncDibitErrors(kFrameSyncMagic), 0U)) << "the pattern is at distance zero from itself";

        {
            // Dibit 0 of the pattern is 01. Making it 10 gets both bits wrong; making it 11 gets
            // one bit wrong. Both are one wrong symbol and count the same.
            std::vector<std::uint8_t> both(sync.begin(), sync.end());
            std::vector<std::uint8_t> one(sync.begin(), sync.end());
            expect(eq(both[0], std::uint8_t{1U})) << "the first sync dibit is 01, which the two damages below rely on";
            both[0] = 2U;
            one[0]  = 3U;
            expect(eq(frameSyncDibitErrors(pack(both)), 1U)) << "a dibit with both bits wrong is one error";
            expect(eq(frameSyncDibitErrors(pack(one)), 1U)) << "a dibit with one bit wrong is one error";
        }
        {
            // Damaging k distinct dibits gives exactly k, across the whole window.
            bool exact = true;
            for (unsigned k = 0U; k <= 24U; ++k) {
                std::vector<std::uint8_t> d(sync.begin(), sync.end());
                for (unsigned i = 0U; i < k; ++i) {
                    d[i] = static_cast<std::uint8_t>(d[i] ^ 0x3U); // both bits, the worst case
                }
                exact = exact && (frameSyncDibitErrors(pack(d)) == k);
            }
            expect(that % exact) << "damaging k dibits gives a distance of exactly k, for every k up to 24";
        }
        expect(eq(frameSyncDibitErrors(~kFrameSyncMagic & kFrameSyncMask), 24U)) << "the pattern's complement is 24 dibits away";
    };

    "the sliding search reports the sync where it actually is"_test = [] {
        const auto sync = frameSyncDibits();
        {
            std::vector<std::uint8_t> stream(40U, 0U); // inner symbols, far from the pattern
            for (std::size_t i = 0U; i < sync.size(); ++i) {
                stream[13U + i] = sync[i];
            }
            stream.resize(80U, 0U);

            P25FrameSyncSearch s;
            s.max_dibit_errors = 0U;
            std::vector<std::size_t> found;
            for (std::size_t i = 0U; i < stream.size(); ++i) {
                unsigned e = 0U;
                if (s.push(stream[i], e)) {
                    found.push_back(i + 1U - kFrameSyncDibits);
                }
            }
            expect(eq(found.size(), std::size_t{1U})) << "one exact match in a stream carrying one sync";
            expect(!found.empty() && found[0] == 13U) << "the match is reported at the sync's first dibit";
        }
        {
            // Nothing is reported before 24 dibits have been seen, whatever they are.
            P25FrameSyncSearch s;
            s.max_dibit_errors = 24U; // would match anything, if it reported at all
            bool early         = false;
            for (std::size_t i = 0U; i < kFrameSyncDibits - 1U; ++i) {
                unsigned e = 0U;
                early      = early || s.push(1U, e);
            }
            expect(that % !early) << "no match is reported until the window holds 24 dibits";
        }
    };

    "tolerance admits damage up to its limit and no further"_test = [] {
        const auto sync = frameSyncDibits();
        for (unsigned tol = 0U; tol <= 6U; ++tol) {
            bool right = true;
            for (unsigned k = 0U; k <= 8U; ++k) {
                std::vector<std::uint8_t> d(sync.begin(), sync.end());
                for (unsigned i = 0U; i < k; ++i) {
                    d[i] = static_cast<std::uint8_t>(d[i] ^ 0x3U);
                }
                P25FrameSyncSearch s;
                s.max_dibit_errors = tol;
                bool     hit       = false;
                unsigned e         = 0U;
                for (const std::uint8_t v : d) {
                    hit = s.push(v, e);
                }
                right = right && (hit == (k <= tol));
            }
            expect(that % right) << "a sync is a candidate exactly while its damage is within tolerance";
        }
    };

    "status symbols: period 36, phase 35, counted from the sync's first symbol"_test = [] {
        expect(eq(kStatusSymbolPeriod, std::size_t{36U})) << "the status period is 36 transmitted symbols";
        expect(that % isStatusSymbol(35U)) << "the first status symbol is at index 35";
        expect(that % isStatusSymbol(71U)) << "the second is at index 71";
        expect(that % isStatusSymbol(107U)) << "and the third at 107";
        {
            bool clean = true;
            for (std::size_t i = 0U; i < kFrameSyncDibits; ++i) {
                clean = clean && !isStatusSymbol(i);
            }
            expect(that % clean) << "no status symbol falls inside the 24 dibits of the frame sync itself";
        }
        expect(!isStatusSymbol(34U) && !isStatusSymbol(36U)) << "35 is the only status index in its period";
    };

    "the frame lengths TIA-102 fixes come out of that period exactly"_test = [] {
        expect(eq(transmittedSpan(70U), std::size_t{72U})) << "a terminator's 70 information symbols are transmitted as 72";
        expect(eq(transmittedSpan(840U), std::size_t{864U})) << "a voice frame's 840 are transmitted as 864";
        expect(eq(transmittedSpan(385U), std::size_t{396U})) << "a header's 385 are transmitted as 396";
        expect(eq(transmittedSpan(210U), std::size_t{216U})) << "a terminator with link control's 210 are transmitted as 216";
        expect(eq(transmittedSpan(175U), std::size_t{180U})) << "a trunking block's 175 are transmitted as 180";
        expect(eq(informationSpan(72U), std::size_t{70U})) << "72 transmitted symbols carry 70 information symbols";
        expect(eq(informationSpan(864U), std::size_t{840U})) << "864 carry 840";
        expect(eq(informationSpan(396U), std::size_t{385U})) << "396 carry 385";
        expect(eq(informationSpan(216U), std::size_t{210U})) << "216 carry 210";
        expect(eq(informationSpan(180U), std::size_t{175U})) << "180 carry 175";
        expect(eq(informationSpan(57U), std::size_t{56U})) << "the sync and the network identifier occupy 57 transmitted symbols";
    };

    "stripping removes only the status positions"_test = [] {
        {
            std::vector<std::uint8_t> raw(72U);
            for (std::size_t i = 0U; i < raw.size(); ++i) {
                raw[i] = isStatusSymbol(i) ? 3U : 1U;
            }
            std::vector<std::uint8_t> out(72U, 9U);
            const std::size_t         n = stripStatusSymbols(raw.data(), raw.size(), out.data(), out.size());
            expect(eq(n, std::size_t{70U})) << "stripping a terminator's 72 transmitted symbols leaves 70";
            bool none = true;
            for (std::size_t i = 0U; i < n; ++i) {
                none = none && (out[i] == 1U);
            }
            expect(that % none) << "no status symbol survives the strip";
        }
        {
            // The strip must keep the information symbols in order, not merely the right count.
            std::vector<std::uint8_t> raw;
            std::vector<std::uint8_t> want;
            for (std::size_t i = 0U; i < 144U; ++i) {
                if (isStatusSymbol(i)) {
                    raw.push_back(2U);
                } else {
                    const auto v = static_cast<std::uint8_t>(want.size() % 3U); // never 3, so distinct from the marker
                    raw.push_back(v);
                    want.push_back(v);
                }
            }
            std::vector<std::uint8_t> out(want.size(), 9U);
            const std::size_t         n = stripStatusSymbols(raw.data(), raw.size(), out.data(), out.size());
            expect(n == want.size() && out == want) << "the surviving symbols keep their order and their values";
        }
    };
};

int main() { /* tests are automatically registered and run */ }
