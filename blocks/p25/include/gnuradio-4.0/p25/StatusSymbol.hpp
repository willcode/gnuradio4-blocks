#ifndef GNURADIO_P25_STATUS_SYMBOL_HPP
#define GNURADIO_P25_STATUS_SYMBOL_HPP

// Status-symbol interleaving for P25 Phase 1, per TIA-102.BAAA.
//
// The transmitter interleaves one status symbol into the outgoing symbol stream after every
// 35 information symbols, so the stream a receiver slices is 36/35 longer than the frame
// content it carries. Status symbols report the state of the corresponding inbound channel;
// they are not part of any frame's content and every field of a frame -- including the network
// identifier, into which the first one falls -- must be reassembled with them taken out.
//
// Counting the first symbol of the frame sync as index 0, a status symbol sits at every index
// congruent to 35 modulo 36:
//
//   index   0 .. 23   frame sync, 24 symbols, no status symbol inside it
//   index  24 .. 34   the first 11 symbols of the network identifier
//   index  35         status symbol
//   index  36 .. 56   the remaining 21 symbols of the network identifier
//   index  71         status symbol
//   ...
//
// so the 32-symbol network identifier spans 33 transmitted symbols, and a frame carrying
// 35*k information symbols is transmitted as exactly 36*k. That is why every frame length
// TIA-102 defines is a whole multiple of 36 symbols: 72 for a terminator, 216 for a
// terminator with link control, 396 for a header, 864 for a voice frame.
//
// Anchoring the count to each detected frame sync rather than to a free-running counter is
// what makes the strip self-correcting. The counter and the frame boundaries carry the same
// information because the lengths are all multiples of 36, so nothing is lost by re-deriving
// the phase at every frame, and a transmission that begins mid-stream -- or a receiver that
// loses and regains the signal between two bursts -- needs no separate reacquisition.

#include <cstddef>
#include <cstdint>

namespace gr::p25 {

//! Transmitted symbols per status symbol.
inline constexpr std::size_t kStatusSymbolPeriod = 36U;

//! Where the status symbol sits inside each period, counting the frame sync's first symbol
//! as index 0.
inline constexpr std::size_t kStatusSymbolPhase = 35U;

//! Information symbols carried by one period.
inline constexpr std::size_t kInformationPerPeriod = kStatusSymbolPeriod - 1U;

//! Whether the symbol at `index` transmitted symbols past a frame sync is a status symbol.
[[nodiscard]] inline constexpr bool isStatusSymbol(std::size_t index) noexcept { return (index % kStatusSymbolPeriod) == kStatusSymbolPhase; }

//! Transmitted symbols needed to carry `information` information symbols from a frame sync.
//! A whole multiple of the period carries its information exactly, with no trailing status
//! symbol beyond the last information symbol.
[[nodiscard]] inline constexpr std::size_t transmittedSpan(std::size_t information) noexcept {
    const std::size_t whole = information / kInformationPerPeriod;
    const std::size_t rest  = information % kInformationPerPeriod;
    return whole * kStatusSymbolPeriod + rest;
}

//! Information symbols inside the first `transmitted` symbols from a frame sync.
[[nodiscard]] inline constexpr std::size_t informationSpan(std::size_t transmitted) noexcept { return transmitted - (transmitted / kStatusSymbolPeriod); }

//! Copy the information symbols out of `raw`, which must begin at a frame sync, writing at
//! most `capacity` of them. Returns how many were written.
inline std::size_t stripStatusSymbols(const std::uint8_t* raw, std::size_t count, std::uint8_t* out, std::size_t capacity) noexcept {
    std::size_t n = 0U;
    for (std::size_t i = 0U; i < count && n < capacity; ++i) {
        if (!isStatusSymbol(i)) {
            out[n++] = raw[i];
        }
    }
    return n;
}

} // namespace gr::p25

#endif // GNURADIO_P25_STATUS_SYMBOL_HPP
