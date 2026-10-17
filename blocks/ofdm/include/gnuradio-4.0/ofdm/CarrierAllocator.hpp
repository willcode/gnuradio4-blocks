#ifndef GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP
#define GNURADIO_OFDM_CARRIER_ALLOCATOR_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
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
 * @brief Maps a stream of data-carrier symbols onto OFDM symbols of data, pilots and sync words in bin order.
 *
 * One output record is one symbol of `fft_len` values in FFT bin order. Its axis carries the signed logical index of
 * each value. `data_carriers` and `pilot_carriers` name carriers by signed index. `gr::ofdm::CarrierMap` converts
 * them to bins once, when the settings are applied.
 *
 * Pilots come from `pilot_symbols` by a two-level cycle. Symbol `s`, pilot slot `p` takes
 * `pilot_symbols[(s * n_pilots + p) % len]`. A cycle whose length is a multiple of `n_pilots` gives every symbol the
 * same pilots. Any other length rotates them from symbol to symbol. When framed, `s` is the symbol's data index within
 * its frame. Without framing, `s` counts from the start of the stream.
 *
 * `sync_words` holds whole symbols as one concatenation. The block emits them verbatim at the head of each frame. A
 * length that is not a whole number of symbols is refused. `gr::ofdm::schmidlCoxPreamble` builds a preamble that can
 * serve as a sync word.
 *
 * `frame_len` is the number of data symbols in a frame. A value of 0 means unframed, with no sync words. The block
 * emits a frame's sync words only once a whole data symbol is ready for that frame. A stream that ends on a frame
 * boundary therefore ends on a data symbol.
 *
 * At end of stream the block zero-fills the trailing partial symbol. When framed, it also emits zero-filled symbols to
 * complete the open frame. Each record states its zero-filled carriers under `pad_carriers`, and `nPadded()` gives the
 * total. These records go into the room the output has when the stream ends. A frame that needs more records than that
 * room ends short, and `nTruncated()` counts the records it lacks.
 *
 * Settings may change while the block runs. A change of `fft_len`, `data_carriers` or `pilot_carriers` is a new
 * numerology. It ends the open frame without padding, and the next record opens a new frame. The data samples not yet
 * placed stay when they fit in one symbol of the new numerology. More than that are dropped, and `nDropped()` counts
 * them. A change of `pilot_symbols` applies from the next data symbol, and a change of `sync_words` from the next sync
 * word. A change of `frame_len` sets the length of the open frame. A frame that already holds that many data symbols
 * ends at once, and a stream that ran unframed opens a new frame. The indices and the counters continue across every
 * change. A setting given its current value changes nothing. A refused setting leaves the block on the settings it
 * last accepted.
 *
 * The block forwards no input tag.
 */
struct CarrierAllocator : Block<CarrierAllocator, NoTagPropagation> {
    using Description = Doc<"Maps a stream of data-carrier symbols onto DataSet<complex<float>> symbol records of fft_len values in bin order. Pilots follow the two-level pilot_symbols cycle, and sync_words open each frame verbatim. The trailing partial symbol and frame are zero-padded, and each record's pad_carriers counts its padded carriers">;

    PortIn<Complex>                  in;
    PortOut<DataSet<Complex>, Async> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length and record length in bins, a power of two">>                          fft_len = 64U;
    Annotated<std::vector<std::int32_t>, "data_carriers", Visible, Doc<"signed logical carrier indices the data stream fills, in order">> data_carriers{};
    Annotated<std::vector<std::int32_t>, "pilot_carriers", Visible, Doc<"signed logical carrier indices the pilot cycle fills">>          pilot_carriers{};
    Annotated<std::vector<float>, "pilot_symbols", Doc<"interleaved re,im pilots, read by (s * n_pilots + p) % len">>                     pilot_symbols{};
    Annotated<std::vector<float>, "sync_words", Doc<"interleaved re,im, a whole number of fft_len-carrier symbols, emitted verbatim">>    sync_words{};
    Annotated<gr::Size_t, "frame_len", Visible, Doc<"data symbols per frame; 0 means unframed">>                                          frame_len   = 0U;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                        signal_name = std::string("ofdm_symbol");

    GR_MAKE_REFLECTABLE(CarrierAllocator, in, out, fft_len, data_carriers, pilot_carriers, pilot_symbols, sync_words, frame_len, signal_name);

    std::optional<gr::ofdm::CarrierMap> _map{};
    std::vector<Complex>                _pilots{};      ///< `pilot_symbols` as complex values
    std::vector<std::vector<Complex>>   _syncWords{};   ///< the sync words, one symbol each
    std::vector<int>                    _binCarriers{}; ///< the signed carrier each bin holds, which is the record's axis
    std::vector<Complex>                _pending{};     ///< data-carrier symbols not yet placed in a symbol
    std::vector<Complex>                _symbol{};      ///< one symbol under construction, reused across records
    detail::SymbolRecordShape           _shape{};       ///< the invariant part of a record

    std::size_t   _nData         = 0UZ;
    std::size_t   _frameLength   = 0UZ;  ///< `frame_len` as last applied
    std::size_t   _syncLeft      = 0UZ;  ///< sync words still owed to the frame being opened
    std::uint64_t _symbolIndex   = 0ULL; ///< stream-absolute count of emitted records
    std::uint64_t _frameIndex    = 0ULL;
    std::size_t   _symbolInFrame = 0UZ;  ///< the next record's position in its frame, sync words included
    std::size_t   _dataInFrame   = 0UZ;  ///< data symbols emitted into the open frame
    std::uint64_t _pilotPhase    = 0ULL; ///< the `s` of the pilot cycle, within the frame when framed and absolute when not
    std::uint64_t _streamAt      = 0ULL; ///< absolute input index of `_pending`'s first sample
    std::uint64_t _padded        = 0ULL; ///< carriers zero-filled at end of stream, cumulative
    std::uint64_t _truncated     = 0ULL; ///< records the final frame lacked for want of output room
    std::uint64_t _dropped       = 0ULL; ///< data samples a change of numerology dropped
    std::size_t   _padThisSymbol = 0UZ;
    bool          _flushed       = false;

    /// @brief The data-carrier slots zero-filled at end of stream, over the whole run.
    [[nodiscard]] std::uint64_t nPadded() const noexcept { return _padded; }

    /// @brief The records the final frame lacked because the output had no room for them at end of stream.
    [[nodiscard]] std::uint64_t nTruncated() const noexcept { return _truncated; }

    /// @brief The data samples dropped by changes of numerology, over the whole run.
    [[nodiscard]] std::uint64_t nDropped() const noexcept { return _dropped; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_len", "data_carriers", "pilot_carriers", "pilot_symbols", "sync_words", "frame_len"};
        if (!_map.has_value() || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            rebuild();
        } else if (newSettings.contains("signal_name")) {
            buildShape();
        }
    }

    /// @brief Builds the numerology, the pilots, the sync words and the record shape from the members.
    ///
    /// Every check runs before the block changes. A refused setting throws and leaves the block as it was.
    void rebuild() {
        gr::ofdm::CarrierMap map = detail::buildMap(fft_len, std::span<const std::int32_t>(data_carriers.value), std::span<const std::int32_t>(pilot_carriers.value));

        std::vector<Complex> pilots = detail::complexFrom(std::span<const float>(pilot_symbols.value), "pilot_symbols");
        detail::requireValid(map.validatePilotCycle(pilots.size()));

        std::vector<std::vector<Complex>> words = detail::syncWordsFrom(std::span<const float>(sync_words.value), static_cast<std::size_t>(fft_len));
        if (frame_len.value == 0U && !words.empty()) {
            throw gr::exception(std::format("frame_len is 0, which is unframed, but sync_words holds {} symbols: a sync word marks a frame's start and an unframed stream has none", words.size()));
        }

        const bool        newNumerology  = !_map.has_value() || map.fftLength() != _map->fftLength() || !std::ranges::equal(map.dataCarriers(), _map->dataCarriers()) || !std::ranges::equal(map.pilotCarriers(), _map->pilotCarriers());
        const std::size_t oldFrameLength = _frameLength;
        const std::size_t syncSent       = _syncWords.size() - _syncLeft;

        if (newNumerology) {
            const std::size_t fftLength = map.fftLength();
            _binCarriers.resize(fftLength);
            for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
                _binCarriers[bin] = detail::carrierAt(fftLength, bin);
            }
            _symbol.assign(fftLength, Complex{});
        }
        _nData = map.nData();
        _map.emplace(std::move(map));
        _pilots      = std::move(pilots);
        _syncWords   = std::move(words);
        _frameLength = static_cast<std::size_t>(frame_len.value);
        buildShape();

        // The open stream follows the new settings. Before `start()` it holds nothing, and `start()` clears it.
        if (newNumerology) {
            if (_pending.size() > _nData) {
                _dropped += _pending.size();
                _pending.clear();
            }
            if (oldFrameLength > 0UZ || _frameLength > 0UZ) {
                openNextFrame();
                return;
            }
        } else if (_frameLength != oldFrameLength && (oldFrameLength == 0UZ || (_frameLength > 0UZ && _dataInFrame >= _frameLength))) {
            openNextFrame();
            return;
        }
        if (_dataInFrame == 0UZ) { // the open frame's sync words continue with the new ones
            _syncLeft = _syncWords.size() > syncSent ? _syncWords.size() - syncSent : 0UZ;
        }
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
        _truncated     = 0ULL;
        _dropped       = 0ULL;
        _padThisSymbol = 0UZ;
        _flushed       = false;
    }

    void start() {
        if (!_map.has_value()) {
            rebuild();
        }
        reset();
    }

    /**
     * @brief Folds whole data symbols out of the stream.
     *
     * The sample that completes a symbol stays in the input until the output has taken every record of that symbol.
     * The framework keeps calling the block while its input holds samples.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        Progress tally = fold(inSpan, inSpan.size(), outSpan);
        if (tally.outputFull && tally.taken > 0UZ && _pending.size() == _nData) {
            _pending.pop_back(); // the next call takes this sample again
            --tally.taken;
        }
        outSpan.publish(tally.made);
        std::ignore = inSpan.consume(tally.taken);
        if (tally.outputFull && tally.made == 0UZ && tally.taken == 0UZ) {
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /**
     * @brief Folds the trailing samples at end of stream, then zero-fills the open symbol and any open frame.
     *
     * The framework calls it once per stream, with the samples the block has not consumed. The span may be empty.
     */
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        Progress tally = fold(inSpan, inSpan.size(), outSpan);
        if (!_flushed) {
            flush(tally, outSpan);
        }
        outSpan.publish(tally.made);
        return work::Status::OK;
    }

private:
    /// @brief The samples taken and the records made by one call.
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
            // A whole data symbol is ready. Its frame's sync words go out first.
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

    /// @brief Zero-fills what the stream did not deliver, into the room the output has, and counts what has no room.
    void flush(Progress& tally, OutputSpanLike auto& outSpan) {
        _flushed             = true;
        const bool framed    = _frameLength > 0UZ;
        const bool frameOpen = framed && (_dataInFrame > 0UZ || !_pending.empty());
        if (_pending.empty() && !frameOpen) {
            return;
        }

        // the sync words still owed, the open symbol, and the zero-filled symbols that complete the frame
        std::size_t owed = _syncLeft + 1UZ + (framed ? _frameLength - _dataInFrame - 1UZ : 0UZ);
        _padThisSymbol   = _nData - _pending.size();
        _pending.resize(_nData, Complex{});

        while (tally.made < outSpan.size() && owed > 0UZ) {
            if (_syncLeft > 0UZ) {
                emitSyncWord(outSpan[tally.made]);
            } else {
                if (_pending.empty()) { // a whole symbol the frame needs and the stream did not carry
                    _pending.assign(_nData, Complex{});
                    _padThisSymbol = _nData;
                }
                emitDataSymbol(outSpan[tally.made]);
            }
            ++tally.made;
            --owed;
        }
        _truncated += owed;
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
            const Complex*    pilots = _pilots.data();
            const std::size_t cycle  = _pilots.size();
            for (std::size_t p = 0UZ; p < nPilots; ++p) {
                _symbol[pilotBins[p]] = pilots[gr::ofdm::CarrierMap::pilotSymbolIndex(_pilotPhase, p, nPilots, cycle)];
            }
        }

        stamp(target, _symbol, detail::kKindData, _padThisSymbol);
        _padded += static_cast<std::uint64_t>(_padThisSymbol);
        _padThisSymbol = 0UZ;
        _pending.clear();
        ++_symbolIndex;
        ++_symbolInFrame;
        ++_dataInFrame;
        ++_pilotPhase;
        if (_frameLength > 0UZ && _dataInFrame >= _frameLength) {
            openNextFrame();
        }
    }

    /// @brief Ends the open frame where it stands. The next record opens a new frame.
    void openNextFrame() {
        if (_symbolInFrame > 0UZ) {
            ++_frameIndex;
        }
        _symbolInFrame = 0UZ;
        _dataInFrame   = 0UZ;
        _pilotPhase    = 0ULL;
        _syncLeft      = _syncWords.size();
    }

    void buildShape() {
        const std::uint64_t fftLength = _symbol.size();
        _shape.build(std::span<const int>(_binCarriers), signal_name.value, property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(fftLength)}});
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
