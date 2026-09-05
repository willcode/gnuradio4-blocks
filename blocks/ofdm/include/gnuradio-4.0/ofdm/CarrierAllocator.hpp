#ifndef GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP
#define GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <print>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::CarrierAllocator)

/**
 * @brief Maps a stream of data-carrier symbols onto OFDM symbols: data, pilots and sync words in bin order.
 *
 * One output record is one symbol, `fft_len` values in FFT bin order, and its axis carries the signed logical index
 * of each value. `data_carriers` and `pilot_carriers` name carriers by signed index; the conversion to bins happens
 * once, in `gr::ofdm::CarrierMap`, and everything here reads the bins it produced.
 *
 * Pilots are read from `pilot_symbols` by the two-level cycle the family states: symbol `s`, pilot slot `p` takes
 * `pilot_symbols[(s * n_pilots + p) % len]`. A cycle whose length is a multiple of `n_pilots` therefore gives every
 * symbol the same pilots, and one that is not rotates them from symbol to symbol. `s` is the symbol's data index
 * within its frame, so a receiver that has found the frame can reproduce the sequence from the frame alone; unframed,
 * `s` is stream-absolute, there being no frame to count from.
 *
 * `sync_words` are whole symbols emitted verbatim as a frame's first symbols. They are a concatenation, since a
 * setting carries no vector of vectors, and a length that is not a whole number of symbols is refused. A Schmidl-Cox
 * preamble is one of these — a sync word whose time-domain form repeats its lower half in its upper — which
 * `gr::ofdm::schmidlCoxPreamble` builds for a caller who has no standard to follow.
 *
 * `frame_len` is the data symbols in a frame; 0 is unframed and carries no sync words. A frame's sync words are
 * emitted only once a whole data symbol is ready for it, so a stream that ends on a frame boundary leaves no preamble
 * standing in front of nothing.
 *
 * End of stream fills what is missing with zeros: the trailing partial symbol always, and, when framed, the rest of
 * the open frame, so a receiver's `n_sync + frame_len` cut is always satisfied. Every record states the carriers that
 * were invented for it under `pad_carriers`, and the total is `nPadded()`.
 */
struct CarrierAllocator : Block<CarrierAllocator, NoTagPropagation> {
    using Description = Doc<"OFDM carrier allocator: a stream of data-carrier symbols in, one DataSet<complex<float>> symbol record of fft_len values in bin order out. Pilots follow the two-level pilot_symbols cycle, sync_words open each frame verbatim, and the trailing partial symbol and frame are zero-padded with the invented carriers counted in each record's pad_carriers">;

    PortIn<Complex>                  in;
    PortOut<DataSet<Complex>, Async> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length, a power of two; the record's length in bins">>                       fft_len = 64U;
    Annotated<std::vector<std::int32_t>, "data_carriers", Visible, Doc<"signed logical carrier indices the data stream fills, in order">> data_carriers{};
    Annotated<std::vector<std::int32_t>, "pilot_carriers", Visible, Doc<"signed logical carrier indices the pilot cycle fills">>          pilot_carriers{};
    Annotated<std::vector<float>, "pilot_symbols", Doc<"interleaved re,im; read by (s * n_pilots + p) % len">>                            pilot_symbols{};
    Annotated<std::vector<float>, "sync_words", Doc<"interleaved re,im, a whole number of fft_len-carrier symbols, emitted verbatim">>    sync_words{};
    Annotated<gr::Size_t, "frame_len", Visible, Doc<"data symbols per frame; 0 is unframed and carries no sync words">>                   frame_len   = 0U;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                        signal_name = std::string("ofdm_symbol");

    GR_MAKE_REFLECTABLE(CarrierAllocator, in, out, fft_len, data_carriers, pilot_carriers, pilot_symbols, sync_words, frame_len, signal_name);

    std::optional<gr::ofdm::CarrierMap> _map{};
    std::vector<Complex>                _pilots{};      ///< `pilot_symbols` as complex values
    std::vector<std::vector<Complex>>   _syncWords{};   ///< the frame's opening symbols, whole
    std::vector<int>                    _binCarriers{}; ///< the signed carrier each bin holds, which is the record's axis
    std::vector<Complex>                _pending{};     ///< data-carrier symbols not yet placed in a symbol
    std::vector<Complex>                _symbol{};      ///< one symbol under construction, reused across records
    detail::SymbolRecordShape           _shape{};       ///< the invariant part of a record, assigned into the slot the port recycles

    std::size_t   _nData           = 0UZ;
    std::size_t   _syncLeft        = 0UZ;  ///< sync words still owed to the frame being opened
    std::uint64_t _symbolIndex     = 0ULL; ///< stream-absolute count of emitted records
    std::uint64_t _frameIndex      = 0ULL;
    std::size_t   _symbolInFrame   = 0UZ;  ///< sync words included, so it indexes the frame's symbol cadence
    std::size_t   _dataInFrame     = 0UZ;  ///< data symbols emitted into the open frame
    std::uint64_t _pilotPhase      = 0ULL; ///< the `s` of the pilot cycle: within the frame when framed, absolute when not
    std::uint64_t _streamAt        = 0ULL; ///< absolute input index of `_pending`'s first sample
    std::uint64_t _padded          = 0ULL; ///< carriers invented at end of stream, cumulative
    std::size_t   _padThisSymbol   = 0UZ;
    bool          _flushed         = false;
    bool          _warnedTruncated = false;

    /// @brief The data-carrier slots zero-filled at end of stream, over the whole run.
    [[nodiscard]] std::uint64_t nPadded() const noexcept { return _padded; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_len", "data_carriers", "pilot_carriers", "pilot_symbols", "sync_words", "frame_len"};
        if (_map.has_value() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        rebuild();
    }

    void rebuild() {
        gr::ofdm::CarrierMap map = detail::buildMap(fft_len, std::span<const std::int32_t>(data_carriers.value), std::span<const std::int32_t>(pilot_carriers.value));

        std::vector<Complex> pilots = detail::complexFrom(std::span<const float>(pilot_symbols.value), "pilot_symbols");
        try {
            map.validatePilotCycle(pilots.size());
        } catch (const std::invalid_argument& error) {
            throw gr::exception(error.what());
        }

        std::vector<std::vector<Complex>> words = detail::syncWordsFrom(std::span<const float>(sync_words.value), static_cast<std::size_t>(fft_len));
        if (frame_len.value == 0U && !words.empty()) {
            throw gr::exception(std::format("frame_len is 0, which is unframed, but sync_words holds {} symbols: a sync word marks a frame's start and an unframed stream has none", words.size()));
        }

        const std::size_t fftLength = static_cast<std::size_t>(fft_len.value);
        _binCarriers.resize(fftLength);
        for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
            _binCarriers[bin] = gr::ofdm::CarrierMap::carrierOf(fftLength, bin);
        }

        _nData = map.nData();
        _map.emplace(std::move(map));
        _pilots    = std::move(pilots);
        _syncWords = std::move(words);
        _symbol.assign(fftLength, Complex{});
        _shape.build(std::span<const int>(_binCarriers), signal_name.value, property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(static_cast<std::uint64_t>(fft_len.value))}});
        reset();
    }

    void reset() {
        _pending.clear();
        _syncLeft      = _syncWords.size();
        _symbolIndex   = 0ULL;
        _frameIndex    = 0ULL;
        _symbolInFrame = 0UZ;
        _dataInFrame   = 0UZ;
        _pilotPhase    = 0ULL;
        _streamAt      = 0ULL;
        _padded        = 0ULL;
        _padThisSymbol = 0UZ;
        _flushed       = false;
    }

    void start() {
        if (!_map.has_value()) {
            rebuild();
        }
        reset();
        // The framework runs `processEpilogue` only over a non-empty trailing span, and that epilogue is what pads the
        // last symbol. `processBulk` therefore always leaves one sample unconsumed, and asking for two keeps that from
        // stalling the steady state.
        in.min_samples = 2UZ;
    }

    /// @brief Fold whole data symbols out of the stream, taking no sample the output has no room for the record of.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const Progress tally = fold(inSpan, inSpan.size() > 0UZ ? inSpan.size() - 1UZ : 0UZ, outSpan);
        if (tally.outputFull && tally.made == 0UZ && tally.taken == 0UZ) {
            outSpan.publish(0UZ);
            std::ignore = inSpan.consume(0UZ);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        outSpan.publish(tally.made);
        std::ignore = inSpan.consume(tally.taken);
        return work::Status::OK;
    }

    /// @brief End of stream: fold the trailing samples, then zero-fill the open symbol and, when framed, the open frame.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        Progress tally = fold(inSpan, inSpan.size(), outSpan);
        if (!_flushed) {
            flush(tally, outSpan);
        }
        outSpan.publish(tally.made);
        return work::Status::OK;
    }

private:
    /// @brief What one call got through.
    struct Progress {
        std::size_t taken      = 0UZ;
        std::size_t made       = 0UZ;
        bool        outputFull = false;
    };

    [[nodiscard]] Progress fold(InputSpanLike auto& inSpan, std::size_t offer, OutputSpanLike auto& outSpan) {
        Progress tally{};
        if (_nData == 0UZ) {
            return tally;
        }
        for (;;) {
            if (_pending.size() < _nData) {
                if (tally.taken == offer) {
                    return tally;
                }
                const std::size_t take  = std::min(_nData - _pending.size(), offer - tally.taken);
                const auto        first = inSpan.begin() + static_cast<std::ptrdiff_t>(tally.taken);
                if (_pending.empty()) {
                    _streamAt = static_cast<std::uint64_t>(inSpan.streamIndex) + static_cast<std::uint64_t>(tally.taken);
                }
                _pending.insert(_pending.end(), first, first + static_cast<std::ptrdiff_t>(take));
                tally.taken += take;
                continue;
            }
            // A whole data symbol is ready, so the frame it belongs to exists and its sync words go first.
            if (tally.made == outSpan.size()) {
                tally.outputFull = true;
                return tally;
            }
            if (_syncLeft > 0UZ) {
                emitSyncWord(outSpan[tally.made]);
            } else {
                emitDataSymbol(outSpan[tally.made]);
            }
            ++tally.made;
        }
    }

    /// @brief Zero-fill what the stream did not deliver, counting every invented carrier.
    void flush(Progress& tally, OutputSpanLike auto& outSpan) {
        const bool frameOpen = frame_len.value > 0U && (_dataInFrame > 0UZ || !_pending.empty());
        if (_pending.empty() && !frameOpen) {
            _flushed = true;
            return;
        }

        std::size_t owed = _syncLeft;
        if (!_pending.empty() || frameOpen) {
            _padThisSymbol = _nData - _pending.size();
            _pending.resize(_nData, Complex{});
            _padded += static_cast<std::uint64_t>(_padThisSymbol);
            owed += 1UZ;
            if (frame_len.value > 0U) {
                owed += static_cast<std::size_t>(frame_len.value) - _dataInFrame - 1UZ;
            }
        }

        while (tally.made < outSpan.size() && owed > 0UZ) {
            if (_syncLeft > 0UZ) {
                emitSyncWord(outSpan[tally.made]);
            } else {
                if (_pending.empty()) { // a whole symbol the frame owes and the stream never carried
                    _pending.assign(_nData, Complex{});
                    _padThisSymbol = _nData;
                    _padded += static_cast<std::uint64_t>(_nData);
                }
                emitDataSymbol(outSpan[tally.made]);
            }
            ++tally.made;
            --owed;
        }

        if (owed > 0UZ && !_warnedTruncated) {
            _warnedTruncated = true;
            std::println(stderr, "gr::blocks::ofdm::CarrierAllocator: the end-of-stream span held room for {} records, so {} of the final frame's symbols were not emitted", outSpan.size(), owed);
        }
        _flushed = true;
    }

    void emitSyncWord(DataSet<Complex>& target) {
        const std::size_t index = _syncWords.size() - _syncLeft;
        --_syncLeft;
        stamp(target, _syncWords[index], detail::kKindSync, 0UZ);
        ++_symbolIndex;
        ++_symbolInFrame;
    }

    void emitDataSymbol(DataSet<Complex>& target) {
        const std::size_t  nPilots   = _map->nPilots();
        const std::size_t* dataBins  = _map->dataBins().data();
        const std::size_t* pilotBins = _map->pilotBins().data();
        const Complex*     data      = _pending.data();

        std::ranges::fill(_symbol, Complex{});
        for (std::size_t k = 0UZ; k < _nData; ++k) {
            _symbol[dataBins[k]] = data[k];
        }
        if (nPilots > 0UZ) {
            const Complex*      pilots = _pilots.data();
            const std::size_t   cycle  = _pilots.size();
            const std::uint64_t phase  = _pilotPhase;
            for (std::size_t p = 0UZ; p < nPilots; ++p) {
                _symbol[pilotBins[p]] = pilots[gr::ofdm::CarrierMap::pilotSymbolIndex(static_cast<std::size_t>(phase), p, nPilots, cycle)];
            }
        }

        stamp(target, _symbol, detail::kKindData, _padThisSymbol);
        _padThisSymbol = 0UZ;
        _pending.clear();
        ++_symbolIndex;
        ++_symbolInFrame;
        ++_dataInFrame;
        ++_pilotPhase;
        if (frame_len.value > 0U && _dataInFrame == static_cast<std::size_t>(frame_len.value)) {
            ++_frameIndex;
            _symbolInFrame = 0UZ;
            _dataInFrame   = 0UZ;
            _pilotPhase    = 0ULL;
            _syncLeft      = _syncWords.size();
        }
    }

    void stamp(DataSet<Complex>& target, const std::vector<Complex>& values, std::string_view kind, std::size_t padCarriers) const {
        property_map& meta                              = _shape.emitInto(target, std::span<const Complex>(values), kind);
        meta[std::pmr::string(detail::kSymbolIndexKey)] = pmt::Value(_symbolIndex);
        meta[std::pmr::string(detail::kFrameIndexKey)]  = pmt::Value(_frameIndex);
        meta[std::pmr::string(detail::kSymbolInFrame)]  = pmt::Value(static_cast<std::uint64_t>(_symbolInFrame));
        meta[std::pmr::string(detail::kSampleStartKey)] = pmt::Value(_streamAt);
        meta[std::pmr::string(detail::kPadCarriersKey)] = pmt::Value(static_cast<std::uint64_t>(padCarriers));
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP
