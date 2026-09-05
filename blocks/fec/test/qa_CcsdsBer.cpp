/* The tier gate for the CCSDS coding row: the concatenated code — inner constraint-length-7
 * rate-1/2 code under the 'ccsds' convention, outer Reed-Solomon (255,223) at interleave 5 —
 * over a seeded additive white Gaussian noise channel, the decoded information bit error rate
 * read against the performance CCSDS itself publishes.
 *
 * The published curve: CCSDS 130.1-G-3 (June 2020), figure 6-7, the I = 5 trace. Its stated
 * assumptions are matched here — unquantized soft-decision Viterbi decoding, ideal
 * synchronization, Eb/N0 counted per information bit so the two code rates are inside it. The
 * operating points, the readings taken off the curve, the envelope and the frame counts were
 * fixed before the first run and none was changed after it. The envelope is asserted in both
 * directions: a rate far below the curve is as wrong as one far above it.
 *
 * The decode is the record-native shape the recipes run, not a streaming approximation: the
 * encoded marker's 52 state-independent symbols open each record and are the trellis's run-up,
 * twelve margin symbols close it so the codeblock's last bits keep their future, and the trim
 * discards exactly what the run-up and the margin decoded. What this gate certifies is that the
 * record-native chain sits on the published curve — that cutting the stream into records with
 * marker run-up costs nothing the curve would show.
 */
#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <format>
#include <numbers>
#include <print>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/algorithm/fec/Convolutional.hpp>
#include <gnuradio-4.0/algorithm/fec/ReedSolomon.hpp>

namespace {

using Rs = gr::fec::ReedSolomonCcsds255_223;

constexpr std::size_t kInterleave = 5UZ;
constexpr std::size_t kInfoBytes  = 223UZ * kInterleave; // 1115: the transfer frame at pad 0
constexpr std::size_t kWireBytes  = 255UZ * kInterleave; // 1275: the codeblock on the wire
constexpr double      kRate       = (223.0 / 255.0) * 0.5;

constexpr std::string_view kAsm = "00011010110011111111110000011101";

//! The operating points, read off figure 6-7's I = 5 trace at its 0.1 dB grid. The two-sided
//! points sit where the estimator has power: a Reed-Solomon frame fails as a burst of hundreds
//! of information bits at once, so a published BER of 2.2e-5 is a frame error rate near 4e-4 —
//! a rate no frame count a qa can afford resolves in both directions. The 2.3 dB point is
//! therefore judged from above only: a broken chain reads orders of magnitude high there, and
//! reading low is the expected value of a sound one at this frame count.
struct Point {
    double ebn0Db;
    double published;
    bool   twoSided;
};
constexpr std::array<Point, 3UZ> kPoints{{{2.0, 2.9e-3, true}, {2.1, 8.0e-4, true}, {2.3, 2.2e-5, false}}};

//! The envelope, a factor either way, asserted in both directions.
constexpr double kEnvelope = 4.0;

[[nodiscard]] bool longRun() { return std::getenv("ENABLE_LONG_TESTS") != nullptr; }

//! Frames a leg. The long arm is what the recorded readings were taken at.
[[nodiscard]] std::size_t frames() { return longRun() ? 400UZ : 40UZ; }

std::uint64_t rng = 0x243F6A8885A308D3ULL;

[[nodiscard]] std::uint64_t next() {
    rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return rng >> 11U;
}

//! One zero-mean unit-variance Gaussian draw, from the same seeded generator as the data.
[[nodiscard]] double gaussian() {
    const double u1 = (static_cast<double>(next() & 0xFFFFFFFFFFFFFULL) + 1.0) / 4503599627370497.0;
    const double u2 = static_cast<double>(next() & 0xFFFFFFFFFFFFFULL) / 4503599627370496.0;
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

struct Reading {
    std::size_t bitErrors   = 0UZ;
    std::size_t bits        = 0UZ;
    std::size_t frameErrors = 0UZ;
    std::size_t frameCount  = 0UZ;

    [[nodiscard]] double ber() const { return bits == 0UZ ? 0.0 : static_cast<double>(bitErrors) / static_cast<double>(bits); }
};

//! One leg: seeded frames through the whole transmit and record-native receive chain at @p ebn0Db.
[[nodiscard]] Reading runLeg(double ebn0Db) {
    gr::fec::ConvolutionalCode inner;
    boost::ut::expect(gr::fec::configureConvention(inner, "ccsds"));
    gr::fec::ViterbiDecoder viterbi;
    boost::ut::expect(viterbi.configure(inner, gr::fec::ConvTermination::Open));

    const double esOverN0 = kRate * std::pow(10.0, ebn0Db / 10.0);
    const double sigma    = std::sqrt(1.0 / (2.0 * esOverN0));

    Reading reading;
    reading.frameCount = frames();

    std::vector<std::uint8_t> frame(kInfoBytes);
    std::vector<std::uint8_t> plainWords(kInfoBytes);
    std::vector<std::uint8_t> wireWords(kWireBytes);
    std::vector<std::uint8_t> codeblock(kWireBytes);
    std::vector<std::uint8_t> txBits;
    std::vector<std::uint8_t> coded;
    std::vector<float>        soft;
    std::vector<std::uint8_t> decodedBits;
    std::vector<std::uint8_t> rxWords(kWireBytes);
    std::vector<std::uint8_t> rxPlain(kWireBytes);

    for (std::size_t f = 0UZ; f < reading.frameCount; ++f) {
        for (std::uint8_t& b : frame) {
            b = static_cast<std::uint8_t>(next());
        }

        // outer code: five codewords, interleaved onto the wire
        gr::fec::deinterleaveCodewords(frame, plainWords, 223UZ, kInterleave);
        for (std::size_t w = 0UZ; w < kInterleave; ++w) {
            Rs::Block block{};
            std::copy_n(plainWords.begin() + static_cast<std::ptrdiff_t>(w * 223UZ), 223UZ, block.begin());
            Rs::encode(block, 0UZ);
            std::copy_n(block.begin(), 255UZ, wireWords.begin() + static_cast<std::ptrdiff_t>(w * 255UZ));
        }
        gr::fec::interleaveCodewords(wireWords, codeblock, 255UZ, kInterleave);

        // inner code: the marker, the codeblock's bits and a margin, one terminated stretch
        txBits.clear();
        for (const char bit : kAsm) {
            txBits.push_back(bit == '1' ? 1U : 0U);
        }
        for (const std::uint8_t byte : codeblock) {
            for (unsigned j = 8U; j-- > 0U;) {
                txBits.push_back(static_cast<std::uint8_t>((byte >> j) & 1U));
            }
        }
        for (std::size_t j = 0UZ; j < 6UZ; ++j) {
            txBits.push_back(static_cast<std::uint8_t>(next() & 1ULL));
        }
        coded.assign(gr::fec::convolutionalEncodedBits(inner, txBits.size()), 0U);
        boost::ut::expect(boost::ut::eq(gr::fec::convolutionalEncode(inner, txBits, coded), coded.size()));

        soft.assign(coded.size(), 0.0f);
        for (std::size_t k = 0UZ; k < coded.size(); ++k) {
            soft[k] = static_cast<float>(((coded[k] & 1U) != 0U ? 1.0 : -1.0) + sigma * gaussian());
        }

        // the record: from the marker's 52 state-independent symbols through the codeblock and the margin
        const std::size_t recordSymbols = 52UZ + kWireBytes * 8UZ * 2UZ + 12UZ;
        const std::size_t recordBits    = recordSymbols / 2UZ;
        decodedBits.assign(recordBits, 0U);
        std::ignore = viterbi.decodeSoft(std::span<const float>(soft).subspan(12UZ, recordSymbols), decodedBits);

        // the trim drops the run-up and the margin; what is left is the codeblock, bit for bit
        const std::span<const std::uint8_t> codeblockBits = std::span<const std::uint8_t>(decodedBits).subspan(26UZ, kWireBytes * 8UZ);
        std::vector<std::uint8_t>           rxBytes(kWireBytes, 0U);
        for (std::size_t i = 0UZ; i < codeblockBits.size(); ++i) {
            rxBytes[i / 8UZ] = static_cast<std::uint8_t>((rxBytes[i / 8UZ] << 1U) | (codeblockBits[i] & 1U));
        }

        gr::fec::deinterleaveCodewords(rxBytes, rxWords, 255UZ, kInterleave);
        bool frameBad = false;
        for (std::size_t w = 0UZ; w < kInterleave; ++w) {
            Rs::Block block{};
            std::copy_n(rxWords.begin() + static_cast<std::ptrdiff_t>(w * 255UZ), 255UZ, block.begin());
            const gr::fec::RsResult result = Rs::decode(block, 0UZ);
            frameBad                       = frameBad || !result.valid;
            std::copy_n(block.begin(), 223UZ, rxPlain.begin() + static_cast<std::ptrdiff_t>(w * 223UZ));
        }
        std::vector<std::uint8_t> recovered(kInfoBytes);
        gr::fec::interleaveCodewords(std::span<const std::uint8_t>(rxPlain.data(), kInfoBytes), recovered, 223UZ, kInterleave);

        for (std::size_t i = 0UZ; i < kInfoBytes; ++i) {
            reading.bitErrors += static_cast<std::size_t>(std::popcount(static_cast<unsigned>(recovered[i] ^ frame[i])));
        }
        reading.bits += kInfoBytes * 8UZ;
        if (frameBad || !std::ranges::equal(recovered, frame)) {
            ++reading.frameErrors;
        }
    }
    return reading;
}

} // namespace

int main() {
    using namespace boost::ut;

    "the concatenated chain lands on the green book's I = 5 curve"_test = [] {
        for (const Point& point : kPoints) {
            const Reading reading  = runLeg(point.ebn0Db);
            const double  measured = reading.ber();
            std::println("concatenated (255,223) I=5 + K7 r1/2 at Eb/N0 {:.2f} dB: {} information bit errors in {} bits, BER {:.3e}, published {:.3e}, ratio {:.2f}, {} of {} frames in error", //
                point.ebn0Db, reading.bitErrors, reading.bits, measured, point.published, measured / point.published, reading.frameErrors, reading.frameCount);
            if (point.twoSided) {
                expect(gt(measured, point.published / kEnvelope)) << "far below the published curve is as wrong as far above it";
            }
            expect(lt(measured, point.published * kEnvelope)) << "above the published curve";
        }
    };

    return 0;
}
