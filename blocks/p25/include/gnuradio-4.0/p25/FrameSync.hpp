#ifndef GNURADIO_P25_FRAME_SYNC_HPP
#define GNURADIO_P25_FRAME_SYNC_HPP

// Frame-synchronization search for P25 Phase 1, per TIA-102.BAAA.
//
// Every frame opens with the same 48-bit pattern, 0x5575F5FF77FF, carried as 24 symbols. The
// pattern is composed entirely of the two outer deviations -- it contains only the 01 and 11
// dibits, never 00 or 10 -- which makes it maximally distant from the inner symbols but also
// means a demodulator whose decisions are pushed outward by noise can reach it by accident.
// A match is therefore a candidate to be confirmed by decoding the network identifier that
// follows it, not an event in its own right.
//
// The search is tolerant rather than exact. A symbol decision that goes wrong corrupts a whole
// dibit, so the natural distance measure is the number of dibits that differ, not the number
// of bits: two wrong bits inside one symbol are one error, not two. The window is kept as a
// 48-bit word and the whole distance is four machine instructions -- exclusive-or against the
// pattern, fold each dibit's two bits together, mask, population count:
//
//   x = window ^ pattern             a bit is set where the two disagree
//   (x | (x >> 1)) & 0x5555...       one bit per dibit, set where either of its bits disagrees
//
// Raising the tolerance recovers frames whose sync arrived damaged and costs candidates that
// have to be rejected downstream. It cannot be raised without limit: past about six dibits the
// pattern stops being distinctive and wrong answers start reaching the identifier decoder with
// enough of the sync intact to look plausible.

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <string>

namespace gr::p25 {

// The sync pattern itself, and its length in dibits.
inline constexpr std::uint64_t kFrameSyncMagic  = 0x5575F5FF77FFULL;
inline constexpr std::size_t   kFrameSyncDibits = 24U;

//! The frame synchronization pattern unpacked into dibits, most significant pair first.
[[nodiscard]] inline std::array<std::uint8_t, kFrameSyncDibits> frameSyncDibits() noexcept {
    std::array<std::uint8_t, kFrameSyncDibits> d{};
    for (std::size_t i = 0; i < kFrameSyncDibits; ++i) {
        const unsigned shift = static_cast<unsigned>(2U * (kFrameSyncDibits - 1U - i));
        d[i]                 = static_cast<std::uint8_t>((kFrameSyncMagic >> shift) & 0x3ULL);
    }
    return d;
}

//! The frame sync as an `AccessCodeCorrelator` access-code string at `bits_per_item = 2`:
//! each transmitted dibit's two bits as '0'/'1' characters, in the order the correlator
//! shifts them in, so a chain built from this cannot disagree with the pattern above.
[[nodiscard]] inline std::string frameSyncAccessCode() {
    std::string code;
    code.reserve(2U * kFrameSyncDibits);
    for (const std::uint8_t dibit : frameSyncDibits()) {
        code.push_back(((dibit >> 1U) & 1U) != 0U ? '1' : '0');
        code.push_back((dibit & 1U) != 0U ? '1' : '0');
    }
    return code;
}

//! One bit per dibit of a 48-bit sync window, selecting the low bit of each pair.
inline constexpr std::uint64_t kDibitMask48 = 0x555555555555ULL;

//! Mask covering the 24 dibits of a frame sync.
inline constexpr std::uint64_t kFrameSyncMask = 0xFFFFFFFFFFFFULL;

//! How many of a window's 24 dibits differ from the frame sync pattern. A dibit counts once
//! however many of its two bits are wrong, because one wrong symbol decision is one error.
[[nodiscard]] inline constexpr unsigned frameSyncDibitErrors(std::uint64_t window) noexcept {
    const std::uint64_t x = (window ^ kFrameSyncMagic) & kFrameSyncMask;
    return static_cast<unsigned>(std::popcount((x | (x >> 1)) & kDibitMask48));
}

//! A sliding 24-dibit correlator. Dibits are pushed one at a time in the order they were
//! received; `push` reports the distance from the sync pattern to the 24 most recent of them.
struct P25FrameSyncSearch {
    //! Dibits of the 24 that may differ and still leave a candidate worth decoding.
    unsigned max_dibit_errors{4U};

    std::uint64_t window{0U};
    std::size_t   filled{0U};

    void reset() noexcept {
        window = 0U;
        filled = 0U;
    }

    //! Absorb one dibit. Returns true when the 24 most recent dibits are within tolerance of
    //! the frame sync pattern, writing how many of them differ.
    [[nodiscard]] bool push(std::uint8_t dibit, unsigned& errors) noexcept {
        window = ((window << 2) | static_cast<std::uint64_t>(dibit & 0x3U)) & kFrameSyncMask;
        if (filled < kFrameSyncDibits) {
            ++filled;
            if (filled < kFrameSyncDibits) {
                return false;
            }
        }
        errors = frameSyncDibitErrors(window);
        return errors <= max_dibit_errors;
    }
};

} // namespace gr::p25

#endif // GNURADIO_P25_FRAME_SYNC_HPP
