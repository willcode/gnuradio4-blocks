#ifndef GNURADIO_OFDM_CYCLIC_PREFIX_HPP
#define GNURADIO_OFDM_CYCLIC_PREFIX_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::CpInsert)

/**
 * @brief Frequency-domain symbol records in, a time-domain stream with cyclic prefixes out.
 *
 * This is one of the two blocks where the domains meet, so it carries the inverse transform as well as the prefix.
 * The fourier module's `FFT` block takes a stream and emits a four-signal real `DataSet` of magnitude, phase and
 * parts; it neither accepts nor produces a `DataSet<complex<float>>` symbol, and the module has no inverse block at
 * all. Composing it between a symbol source and this one is therefore not available, and the transform is taken here
 * on the library kernel — the same kernel the fourier block itself uses, so this is a reuse and not a second FFT.
 *
 * The transform is the inverse DFT with its `1/fft_len`: the library's transforms are unnormalized, so the scaling is
 * applied here and `CpRemove`'s forward transform returns the record it started from.
 *
 * Each symbol's tail `cp_len` samples are copied in front of it. `cp_len` is a scalar or a per-symbol cycle — 802.11's
 * long-then-short opening is a two-entry cycle — and the cycle position is the record's own `symbol_in_frame`, so it
 * restarts with every frame rather than drifting against it. A record that carries no `symbol_in_frame` is counted in
 * `nUnmarked()` and takes the position the block was already at.
 *
 * `window_len` raised-cosine samples smooth each symbol's edges into its neighbor's: the symbol is extended
 * cyclically by that many samples, the extension falls while the next symbol's head rises, and the two are added. The
 * stream advances by `cp_len + fft_len` per symbol either way, so windowing costs no rate. It is off by default,
 * because it is a deliberate departure from the exact prefix algebra and its benefit — a narrower spectrum — is a
 * measurement rather than a given.
 *
 * A frame's first sample carries a `trigger_name` tag, which is what lets `CpRemove` align a loopback without a
 * detector in between. It is a transmit-side marker that nothing on air ever sees; `emit_trigger` turns it off.
 */
struct CpInsert : Block<CpInsert, NoTagPropagation> {
    using Description = Doc<"OFDM cyclic-prefix insertion: DataSet<complex<float>> symbol records in, a time-domain complex stream out. Carries the inverse transform, since the fourier module's FFT block neither takes nor returns a complex symbol record. cp_len is a scalar or a per-symbol cycle restarting at each frame; window_len raised-cosine samples optionally smooth the symbol edges; a frame's first sample carries a trigger tag">;

    PortIn<DataSet<Complex>, Async> in;
    PortOut<Complex>                out;

    Annotated<std::vector<gr::Size_t>, "cp_len", Visible, Doc<"prefix samples: one entry is a constant, several are a per-symbol cycle restarting at each frame">> cp_len        = std::vector<gr::Size_t>{16U};
    Annotated<gr::Size_t, "window_len", Visible, Doc<"raised-cosine edge samples overlapped with the next symbol; 0 is off">>                                      window_len    = 0U;
    Annotated<bool, "emit_trigger", Doc<"tag the first sample of every frame with trigger_name">>                                                                  emit_trigger  = true;
    Annotated<std::string, "trigger_label", Doc<"the label written under the trigger_name key of the emitted tags">>                                               trigger_label = std::string("ofdm_frame");

    GR_MAKE_REFLECTABLE(CpInsert, in, out, cp_len, window_len, emit_trigger, trigger_label);

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Backward> _inverse{};

    std::vector<Complex> _time{};    ///< one symbol, transformed, before the prefix is put in front of it
    std::vector<Complex> _out{};     ///< the symbol as a stream: prefix, symbol and the windowed head folded in
    std::vector<Complex> _overlap{}; ///< the previous symbol's falling extension, waiting for this one's head
    std::vector<float>   _ramp{};    ///< the rising raised-cosine edge; the falling one is its complement

    std::size_t   _fftLength    = 0UZ;
    std::size_t   _outAt        = 0UZ;  ///< samples of `_out` already published
    std::size_t   _cyclePos     = 0UZ;  ///< the prefix cycle's position, which is the symbol's index in its frame
    std::uint64_t _unmarked     = 0ULL; ///< records that named no `symbol_in_frame`
    std::uint64_t _malformed    = 0ULL; ///< records too short to hold the prefix they would be given
    bool          _triggerReady = false;

    /// @brief Records that carried no `symbol_in_frame`, and so took the cycle position the block was already at.
    [[nodiscard]] std::uint64_t nUnmarked() const noexcept { return _unmarked; }

    /// @brief Records shorter than the prefix they would have been given, which are dropped rather than read past.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"cp_len", "window_len"};
        if (!_ramp.empty() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        if (cp_len.value.empty()) {
            throw gr::exception("cp_len is the prefix length, a scalar or a per-symbol cycle, and must hold at least one entry");
        }
        const gr::Size_t shortest = *std::ranges::min_element(cp_len.value);
        if (window_len.value > shortest) {
            throw gr::exception(std::format("window_len is {} and the shortest cp_len is {}: an edge longer than the prefix it overlaps would reach past the symbol it belongs to", window_len.value, shortest));
        }
        buildRamp();
    }

    void start() {
        buildRamp();
        _time.clear();
        _out.clear();
        _outAt        = 0UZ;
        _cyclePos     = 0UZ;
        _unmarked     = 0ULL;
        _malformed    = 0ULL;
        _triggerReady = false;
        std::ranges::fill(_overlap, Complex{});
    }

    /**
     * @brief Drain the symbol under construction, then build the next one, for as long as both spans allow.
     *
     * A record is consumed only once its whole stream form has been published, so a call that runs out of output room
     * leaves the record where it is and resumes on the next one; nothing of a symbol is ever lost between calls.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t written  = 0UZ;
        std::size_t consumed = 0UZ;

        for (;;) {
            if (_outAt < _out.size()) {
                if (written == outSpan.size()) {
                    break;
                }
                if (_outAt == 0UZ && _triggerReady) {
                    outSpan.publishTag(property_map{{gr::tag::TRIGGER_NAME.shortKey(), trigger_label.value}, {gr::tag::TRIGGER_OFFSET.shortKey(), 0.f}}, written);
                    _triggerReady = false;
                }
                const std::size_t take = std::min(_out.size() - _outAt, outSpan.size() - written);
                std::copy_n(_out.begin() + static_cast<std::ptrdiff_t>(_outAt), take, outSpan.begin() + static_cast<std::ptrdiff_t>(written));
                _outAt += take;
                written += take;
                continue;
            }
            if (consumed == inSpan.size() || written == outSpan.size()) {
                break;
            }
            build(inSpan[consumed]);
            ++consumed;
        }

        outSpan.publish(written);
        std::ignore = inSpan.consume(consumed);
        if (written == 0UZ && consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    void buildRamp() {
        const std::size_t edge = static_cast<std::size_t>(window_len.value);
        _ramp.resize(edge);
        for (std::size_t n = 0UZ; n < edge; ++n) {
            // half-sample centered, so the rise and the fall it complements are symmetric about the boundary
            _ramp[n] = static_cast<float>(0.5 * (1.0 - std::cos(std::numbers::pi * (static_cast<double>(n) + 0.5) / static_cast<double>(edge))));
        }
        _overlap.assign(edge, Complex{});
    }

    /// @brief One record as a stretch of stream: the inverse transform, its prefix, and the windowed edges.
    void build(const DataSet<Complex>& record) {
        const std::size_t fftLength = record.signal_values.size();
        const std::size_t longest   = static_cast<std::size_t>(*std::ranges::max_element(cp_len.value));
        if (fftLength < longest || fftLength == 0UZ) {
            // A prefix is a copy of the symbol's own tail, so a record with less symbol than prefix is not one. It is
            // dropped and counted rather than read past its end.
            ++_malformed;
            _out.clear();
            _outAt = 0UZ;
            return;
        }
        if (fftLength != _fftLength) {
            _fftLength = fftLength;
            _time.assign(fftLength, Complex{});
        }

        const std::uint64_t marker = detail::metaCount(record, detail::kSymbolInFrame, std::numeric_limits<std::uint64_t>::max());
        if (marker == std::numeric_limits<std::uint64_t>::max()) {
            ++_unmarked;
        } else {
            _cyclePos = static_cast<std::size_t>(marker);
        }
        if (_cyclePos == 0UZ && emit_trigger.value) {
            _triggerReady = true;
        }

        _inverse.compute(record.signal_values, std::span<Complex>(_time));
        const float scale = 1.f / static_cast<float>(fftLength);
        for (Complex& value : _time) {
            value *= scale;
        }

        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(cp_len.value), _cyclePos);
        const std::size_t edge   = _ramp.size();

        _out.resize(prefix + fftLength);
        std::copy_n(_time.end() - static_cast<std::ptrdiff_t>(prefix), prefix, _out.begin());
        std::copy_n(_time.begin(), fftLength, _out.begin() + static_cast<std::ptrdiff_t>(prefix));

        if (edge > 0UZ) {
            // The symbol continues cyclically past its end, so the extension is its own head; it falls while this
            // symbol's head rises, and the previous symbol's extension is added into that head.
            for (std::size_t n = 0UZ; n < edge; ++n) {
                _out[n] = _out[n] * _ramp[n] + _overlap[n];
            }
            for (std::size_t n = 0UZ; n < edge; ++n) {
                _overlap[n] = _time[n] * (1.f - _ramp[n]);
            }
        }

        _outAt = 0UZ;
        ++_cyclePos;
    }
};

GR_REGISTER_BLOCK(gr::blocks::ofdm::CpRemove)

/**
 * @brief A time-domain stream in, one frequency-domain symbol record per symbol out, aligned by a trigger tag.
 *
 * The alignment convention is pinned so that this block and `CpInsert` meet exactly: **the trigger marks the first
 * sample of the first symbol's cyclic prefix**. On a trigger the block skips `cp_len` samples and cuts the
 * `fft_len` that follow — equivalently, it takes `cp_len + fft_len` and keeps the last `fft_len` — then repeats at
 * the symbol cadence for `n_sync + frame_len` symbols, or until the next trigger, whichever comes first. Between
 * frames it discards, and `nDiscarded()` says how many samples, as does every record's `discarded_samples`.
 *
 * `timing_offset` biases the cut into the prefix, the standard margin against a channel's delay spread: a negative
 * value starts the window that many samples early, inside the prefix, where the symbol's own tail already sits. The
 * consequence is a per-carrier phase slope of `-2*pi*k*timing_offset/fft_len`, a rotation the equalizer's channel
 * estimate absorbs along with the channel's own — it is not an error and must not be "fixed" anywhere downstream.
 * The range is `[-min(cp_len), 0]`: outside it the window leaves the symbol and the algebra no longer holds.
 *
 * The forward transform is taken here for the reason `CpInsert` takes the inverse: the fourier module's `FFT` block
 * carries no complex symbol record on either side. It is the library kernel, unnormalized, which is the exact inverse
 * of `CpInsert`'s scaled backward transform.
 */
struct CpRemove : Block<CpRemove, NoTagPropagation> {
    using Description = Doc<"OFDM cyclic-prefix removal: a complex stream in, one DataSet<complex<float>> symbol record of fft_len bins out per symbol. Aligned by a trigger tag marking the first sample of the first symbol's prefix; cuts fft_len after cp_len, biased by timing_offset; discards between frames and counts what it discarded">;

    PortIn<Complex>                  in;
    PortOut<DataSet<Complex>, Async> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length, a power of two; the record's length in bins">>                                                       fft_len       = 64U;
    Annotated<std::vector<gr::Size_t>, "cp_len", Visible, Doc<"prefix samples: one entry is a constant, several are a per-symbol cycle restarting at each frame">>        cp_len        = std::vector<gr::Size_t>{16U};
    Annotated<gr::Size_t, "n_sync", Visible, Doc<"sync symbols at a frame's head, which are cut like any other">>                                                         n_sync        = 0U;
    Annotated<gr::Size_t, "frame_len", Visible, Doc<"data symbols per frame; with n_sync it sets the cadence a trigger starts. 0 cuts until the next trigger">>           frame_len     = 0U;
    Annotated<std::int32_t, "timing_offset", Visible, Unit<"samples">, Doc<"signed bias of the cut, in [-min(cp_len), 0]; negative starts the window inside the prefix">> timing_offset = 0;
    Annotated<std::string, "trigger_label", Doc<"the trigger_name a tag must carry to start a frame; empty accepts any trigger">>                                         trigger_label = std::string("");
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                                                        signal_name   = std::string("ofdm_symbol");

    GR_MAKE_REFLECTABLE(CpRemove, in, out, fft_len, cp_len, n_sync, frame_len, timing_offset, trigger_label, signal_name);

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Forward> _forward{};

    std::vector<Complex>      _cut{};         ///< the window under collection
    std::vector<Complex>      _spectrum{};    ///< the transform's output
    std::vector<int>          _binCarriers{}; ///< the signed carrier each bin holds, which is the record's axis
    detail::SymbolRecordShape _shape{};       ///< the invariant part of a record, assigned into the slot the port recycles

    bool          _collecting    = false; ///< inside a frame; otherwise the block is discarding
    std::uint64_t _symbolStart   = 0ULL;  ///< absolute index of the current symbol's first prefix sample
    std::uint64_t _windowStart   = 0ULL;  ///< absolute index of the current cut's first sample
    std::size_t   _have          = 0UZ;   ///< samples of the cut already collected
    std::size_t   _symbolInFrame = 0UZ;
    std::uint64_t _symbolIndex   = 0ULL;
    std::uint64_t _frameIndex    = 0ULL;
    std::uint64_t _discarded     = 0ULL;

    /// @brief Samples thrown away between frames, over the whole run.
    [[nodiscard]] std::uint64_t nDiscarded() const noexcept { return _discarded; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_len", "cp_len", "timing_offset"};
        if (!_cut.empty() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        detail::requireFftLength(fft_len);
        detail::requireCyclicPrefix(std::span<const gr::Size_t>(cp_len.value), fft_len);

        const auto shortest = static_cast<std::int32_t>(*std::ranges::min_element(cp_len.value));
        if (timing_offset.value > 0 || timing_offset.value < -shortest) {
            throw gr::exception(std::format("timing_offset is {} and must lie in [{}, 0]: outside it the cut leaves the symbol whose prefix it is biased into", timing_offset.value, -shortest));
        }

        const std::size_t fftLength = static_cast<std::size_t>(fft_len.value);
        _cut.assign(fftLength, Complex{});
        _spectrum.assign(fftLength, Complex{});
        _binCarriers.resize(fftLength);
        for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
            _binCarriers[bin] = gr::ofdm::CarrierMap::carrierOf(fftLength, bin);
        }
        _shape.build(std::span<const int>(_binCarriers), signal_name.value,
            property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(static_cast<std::uint64_t>(fft_len.value))}, //
                {std::pmr::string(detail::kTimingOffsetKey), pmt::Value(static_cast<std::int64_t>(timing_offset.value))}});
    }

    void start() {
        _collecting    = false;
        _have          = 0UZ;
        _symbolInFrame = 0UZ;
        _symbolIndex   = 0ULL;
        _frameIndex    = 0ULL;
        _discarded     = 0ULL;
    }

    /**
     * @brief Walk the call's samples, cutting where the cadence says and discarding where it does not.
     *
     * Only the tags inside the region this call consumes are read: one further on is offered again next call, and
     * acting on it now would restart the frame at a sample the block has not reached. The span is walked in stretches
     * rather than sample by sample — a skip over a prefix, a copy into a cut, a discard between frames — so the cost
     * is the copy and nothing per sample beyond it.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::uint64_t        base = static_cast<std::uint64_t>(inSpan.streamIndex);
        std::vector<std::uint64_t> triggers;
        for (const auto& [relIndex, tagMap] : inSpan.tags(inSpan.size())) {
            if (relIndex < 0 || !isTrigger(tagMap.get())) {
                continue;
            }
            triggers.push_back(base + static_cast<std::uint64_t>(relIndex));
        }
        std::ranges::sort(triggers);

        const std::size_t fftLength = _cut.size();
        std::size_t       at        = 0UZ;
        std::size_t       made      = 0UZ;
        std::size_t       nextTrig  = 0UZ;

        while (at < inSpan.size()) {
            const std::uint64_t here = base + static_cast<std::uint64_t>(at);
            while (nextTrig < triggers.size() && triggers[nextTrig] < here) {
                ++nextTrig; // a trigger the walk has already passed, which only a second tag on one sample produces
            }
            if (nextTrig < triggers.size() && triggers[nextTrig] == here) {
                openFrame(here);
                ++nextTrig;
            }
            const std::uint64_t untilTrigger = nextTrig < triggers.size() ? triggers[nextTrig] - here : inSpan.size() - at;
            const std::size_t   room         = static_cast<std::size_t>(std::min(static_cast<std::uint64_t>(inSpan.size() - at), untilTrigger));

            if (!_collecting) {
                _discarded += static_cast<std::uint64_t>(room);
                at += room;
                continue;
            }
            if (here < _windowStart) { // the prefix, and whatever `timing_offset` left of it
                const std::size_t skip = static_cast<std::size_t>(std::min(static_cast<std::uint64_t>(room), _windowStart - here));
                at += skip;
                continue;
            }

            const std::size_t need = fftLength - _have;
            if (need <= room && made == outSpan.size()) {
                break; // the samples that complete this cut stay unconsumed until there is room for its record
            }
            const std::size_t take = std::min(need, room);
            std::copy_n(inSpan.begin() + static_cast<std::ptrdiff_t>(at), take, _cut.begin() + static_cast<std::ptrdiff_t>(_have));
            _have += take;
            at += take;
            if (_have == fftLength) {
                emit(outSpan[made]);
                ++made;
                advance();
            }
        }

        outSpan.publish(made);
        std::ignore = inSpan.consume(at);
        if (at == 0UZ && made == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    [[nodiscard]] bool isTrigger(const property_map& tagMap) const {
        const auto entry = tagMap.find(std::pmr::string(gr::tag::TRIGGER_NAME.shortKey()));
        if (entry == tagMap.end()) {
            return false;
        }
        if (trigger_label.value.empty()) {
            return true;
        }
        const auto* label = entry->second.get_if<std::pmr::string>();
        return label != nullptr && std::string_view(label->data(), label->size()) == std::string_view(trigger_label.value);
    }

    void openFrame(std::uint64_t at) {
        if (_collecting || _symbolIndex > 0ULL) {
            ++_frameIndex;
        }
        _collecting    = true;
        _symbolInFrame = 0UZ;
        _have          = 0UZ;
        _symbolStart   = at;
        _windowStart   = windowOf(at, 0UZ);
    }

    [[nodiscard]] std::uint64_t windowOf(std::uint64_t symbolStart, std::size_t symbolInFrame) const {
        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(cp_len.value), symbolInFrame);
        return symbolStart + static_cast<std::uint64_t>(static_cast<std::int64_t>(prefix) + timing_offset.value);
    }

    /// @brief Step the cadence to the next symbol, or leave the frame when it has given all its symbols.
    void advance() {
        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(cp_len.value), _symbolInFrame);
        _symbolStart += prefix + _cut.size();
        ++_symbolInFrame;
        ++_symbolIndex;
        _have = 0UZ;

        const std::size_t perFrame = static_cast<std::size_t>(n_sync.value) + static_cast<std::size_t>(frame_len.value);
        if (perFrame > 0UZ && _symbolInFrame >= perFrame) {
            _collecting = false;
            return;
        }
        _windowStart = windowOf(_symbolStart, _symbolInFrame);
    }

    void emit(DataSet<Complex>& target) {
        _forward.compute(_cut, std::span<Complex>(_spectrum));
        const std::string_view kind                     = _symbolInFrame < static_cast<std::size_t>(n_sync.value) ? detail::kKindSync : detail::kKindData;
        property_map&          meta                     = _shape.emitInto(target, std::span<const Complex>(_spectrum), kind);
        meta[std::pmr::string(detail::kSymbolIndexKey)] = pmt::Value(_symbolIndex);
        meta[std::pmr::string(detail::kFrameIndexKey)]  = pmt::Value(_frameIndex);
        meta[std::pmr::string(detail::kSymbolInFrame)]  = pmt::Value(static_cast<std::uint64_t>(_symbolInFrame));
        meta[std::pmr::string(detail::kSampleStartKey)] = pmt::Value(_windowStart);
        meta[std::pmr::string(detail::kDiscardedKey)]   = pmt::Value(_discarded);
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CYCLIC_PREFIX_HPP
