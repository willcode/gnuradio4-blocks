#ifndef GNURADIO_OFDM_CYCLIC_PREFIX_HPP
#define GNURADIO_OFDM_CYCLIC_PREFIX_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
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
 * @brief Converts frequency-domain symbol records into a time-domain stream with cyclic prefixes.
 *
 * The block takes the inverse transform of each record and puts a cyclic prefix in front of it. The transform runs on
 * the library FFT and is scaled by `1/fft_len`. The library's transforms are unnormalized. With this scaling the
 * forward transform in `CpRemove` returns the record the block started from.
 *
 * The prefix is a copy of the symbol's last `cp_len` samples. `cp_len` is a scalar or a per-symbol cycle. The cycle
 * position is the record's own `symbol_in_frame`. The cycle therefore restarts with every frame. A record without
 * `symbol_in_frame` takes the position after the previous record's, and `nUnmarked()` counts it.
 *
 * `window_len` raised-cosine samples blend the edges of neighboring symbols. The block extends each symbol cyclically
 * by that many samples. The extension falls while the next symbol's head rises, and the two are added. The stream
 * advances by `cp_len + fft_len` per symbol either way. Windowing is off by default. A windowed symbol departs from the
 * exact prefix algebra in its first `window_len` samples and has lower out-of-band power.
 *
 * With `emit_trigger` set, the block tags the first sample of each frame with `trigger_name` set to `trigger_label`.
 * `CpRemove` starts a frame on such a tag. The block forwards no input tag.
 *
 * Settings may change while the block runs. A change of `cp_len` applies from the next record. A change of
 * `window_len` also applies from the next record, whose head rises without the previous symbol's extension. A setting
 * given its current value changes nothing. A refused setting leaves the block on the settings it last accepted.
 */
struct CpInsert : Block<CpInsert, NoTagPropagation> {
    using Description = Doc<"Converts DataSet<complex<float>> symbol records into a time-domain complex stream. It takes the inverse transform of each record and puts a cyclic prefix in front of it. cp_len is a scalar or a per-symbol cycle that restarts at each frame. Optional window_len raised-cosine samples smooth the symbol edges. The first sample of each frame carries a trigger tag">;

    PortIn<DataSet<Complex>, Async> in;
    PortOut<Complex>                out;

    Annotated<std::vector<gr::Size_t>, "cp_len", Visible, Doc<"prefix samples, a constant or a per-symbol cycle restarting each frame">> cp_len        = std::vector<gr::Size_t>{16U};
    Annotated<gr::Size_t, "window_len", Visible, Doc<"raised-cosine edge samples shared with the next symbol; 0 disables">>              window_len    = 0U;
    Annotated<bool, "emit_trigger", Doc<"tag the first sample of every frame with trigger_name">>                                        emit_trigger  = true;
    Annotated<std::string, "trigger_label", Doc<"the label written under the trigger_name key of the emitted tags">>                     trigger_label = std::string("ofdm_frame");

    GR_MAKE_REFLECTABLE(CpInsert, in, out, cp_len, window_len, emit_trigger, trigger_label);

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Backward> _inverse{};

    std::vector<Complex>    _time{};     ///< one symbol, transformed, before the prefix is put in front of it
    std::vector<Complex>    _out{};      ///< the symbol as a stream, with prefix, symbol and the windowed head
    std::vector<Complex>    _overlap{};  ///< the previous symbol's falling extension, added into the next symbol's head
    std::vector<float>      _ramp{};     ///< the rising raised-cosine edge, whose complement is the falling one
    std::vector<gr::Size_t> _prefixes{}; ///< `cp_len` as last accepted

    std::size_t   _longestPrefix = 0UZ;
    std::size_t   _fftLength     = 0UZ;
    std::size_t   _outAt         = 0UZ;  ///< samples of `_out` already published
    std::size_t   _cyclePos      = 0UZ;  ///< the prefix cycle's position, which is the symbol's index in its frame
    std::uint64_t _unmarked      = 0ULL; ///< records that named no `symbol_in_frame`
    std::uint64_t _malformed     = 0ULL; ///< records too short to hold their prefix
    bool          _triggerReady  = false;

    /// @brief Records that carried no `symbol_in_frame`.
    [[nodiscard]] std::uint64_t nUnmarked() const noexcept { return _unmarked; }

    /// @brief Records shorter than their prefix. The block drops them.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"cp_len", "window_len"};
        if (!_prefixes.empty() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        acceptShape();
    }

    /// @brief Checks `cp_len` and `window_len` and takes them. A refused setting throws and leaves the block as it was.
    void acceptShape() {
        if (cp_len.value.empty()) {
            throw gr::exception("cp_len is the prefix length, a scalar or a per-symbol cycle, and must hold at least one entry");
        }
        const gr::Size_t shortest = *std::ranges::min_element(cp_len.value);
        if (window_len.value > shortest) {
            throw gr::exception(std::format("window_len is {} and the shortest cp_len is {}: an edge longer than the prefix it overlaps would reach past the symbol it belongs to", window_len.value, shortest));
        }
        _prefixes      = cp_len.value;
        _longestPrefix = static_cast<std::size_t>(*std::ranges::max_element(_prefixes));
        if (static_cast<std::size_t>(window_len.value) != _ramp.size()) {
            buildRamp();
        }
    }

    void start() {
        if (_prefixes.empty()) {
            acceptShape();
        }
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
     * @brief Publishes the samples of the record in hand, then builds the next record, while both spans allow.
     *
     * The block builds a record's samples into a buffer. The record stays in the input until its last sample is
     * published. A call that runs out of output room publishes the rest of the buffer on the next call. The framework
     * keeps calling the block while its input holds a record.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t written  = 0UZ;
        std::size_t consumed = 0UZ;

        for (;;) {
            if (_outAt < _out.size()) { // the samples of the record at `consumed`
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
                if (_outAt == _out.size()) {
                    ++consumed;
                }
                continue;
            }
            if (consumed == inSpan.size() || written == outSpan.size()) {
                break;
            }
            build(inSpan[consumed]);
            if (_out.empty()) { // a malformed record builds no samples
                ++consumed;
            }
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
            // centered on half samples: the rise and its complement are symmetric about the symbol boundary
            _ramp[n] = static_cast<float>(0.5 * (1.0 - std::cos(std::numbers::pi * (static_cast<double>(n) + 0.5) / static_cast<double>(edge))));
        }
        _overlap.assign(edge, Complex{});
    }

    /// @brief Builds one record's samples: the inverse transform, the prefix and the windowed edges.
    void build(const DataSet<Complex>& record) {
        const std::size_t fftLength = record.signal_values.size();
        if (fftLength < _longestPrefix || fftLength == 0UZ) {
            // A record shorter than its prefix is dropped and counted.
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

        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(_prefixes), _cyclePos);
        const std::size_t edge   = _ramp.size();

        _out.resize(prefix + fftLength);
        std::copy_n(_time.end() - static_cast<std::ptrdiff_t>(prefix), prefix, _out.begin());
        std::copy_n(_time.begin(), fftLength, _out.begin() + static_cast<std::ptrdiff_t>(prefix));

        if (edge > 0UZ) {
            // The cyclic extension past the symbol's end is a copy of its head. This symbol's head rises while the
            // previous symbol's extension falls, and the two are added.
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
 * @brief Cuts a time-domain stream into frequency-domain symbol records, one per symbol, aligned by a trigger tag.
 *
 * A trigger marks the first sample of the first symbol's cyclic prefix, the sample `CpInsert` tags. On a trigger the
 * block skips `cp_len` samples and cuts the `fft_len` samples that follow. It repeats at the symbol cadence for
 * `n_sync + frame_len` symbols or until the next trigger, whichever comes first. Between frames it discards samples.
 * `nDiscarded()` counts them, and each record's `discarded_samples` holds the total since start.
 *
 * `timing_offset` moves the cut into the prefix, a margin against the channel's delay spread. A negative value starts
 * the cut that many samples early, inside the prefix, where the symbol's own tail sits. The result is a per-carrier
 * phase of `2*pi*k*timing_offset/fft_len` on carrier `k`. The range is `[-min(cp_len), 0]`.
 *
 * The block takes the forward transform of each cut on the library FFT, unnormalized. It inverts the scaled backward
 * transform in `CpInsert`.
 *
 * Settings may change while the block runs. A change of `fft_len`, `cp_len` or `timing_offset` ends the open frame.
 * The samples of its partial cut count as discarded, and the block discards until the next trigger. A change of
 * `n_sync` or `frame_len` sets the length of the open frame, and a change of `signal_name` applies from the next
 * record. A setting given its current value changes nothing. A refused setting leaves the block on the settings it
 * last accepted.
 *
 * The block reads trigger tags and forwards no input tag.
 */
struct CpRemove : Block<CpRemove, NoTagPropagation> {
    using Description = Doc<"Cuts a complex stream into one DataSet<complex<float>> record of fft_len bins per symbol and takes its forward transform. A trigger tag marks the first sample of the first symbol's prefix. The block cuts fft_len samples after cp_len, moved by timing_offset. It discards samples between frames and counts them">;

    PortIn<Complex>                  in;
    PortOut<DataSet<Complex>, Async> out;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length and record length in bins, a power of two">>                         fft_len       = 64U;
    Annotated<std::vector<gr::Size_t>, "cp_len", Visible, Doc<"prefix samples, a constant or a per-symbol cycle restarting each frame">> cp_len        = std::vector<gr::Size_t>{16U};
    Annotated<gr::Size_t, "n_sync", Visible, Doc<"sync symbols at a frame's head, cut like data symbols">>                               n_sync        = 0U;
    Annotated<gr::Size_t, "frame_len", Visible, Doc<"data symbols per frame; 0 cuts until the next trigger">>                            frame_len     = 0U;
    Annotated<std::int32_t, "timing_offset", Visible, Unit<"samples">, Doc<"signed bias of the cut, in [-min(cp_len), 0]">>              timing_offset = 0;
    Annotated<std::string, "trigger_label", Doc<"trigger_name that starts a frame; empty accepts any trigger">>                          trigger_label = std::string("");
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                       signal_name   = std::string("ofdm_symbol");

    GR_MAKE_REFLECTABLE(CpRemove, in, out, fft_len, cp_len, n_sync, frame_len, timing_offset, trigger_label, signal_name);

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Forward> _forward{};

    std::vector<Complex>      _cut{};         ///< the samples of the cut collected so far
    std::vector<Complex>      _spectrum{};    ///< the transform's output
    std::vector<int>          _binCarriers{}; ///< the signed carrier each bin holds, which is the record's axis
    std::vector<gr::Size_t>   _prefixes{};    ///< `cp_len` as last accepted
    std::int32_t              _offset = 0;    ///< `timing_offset` as last accepted
    detail::SymbolRecordShape _shape{};       ///< the invariant part of a record

    bool          _collecting    = false; ///< inside a frame, and discarding when false
    std::uint64_t _symbolStart   = 0ULL;  ///< absolute index of the current symbol's first prefix sample
    std::uint64_t _windowStart   = 0ULL;  ///< absolute index of the current cut's first sample
    std::size_t   _have          = 0UZ;   ///< samples of the cut already collected
    std::size_t   _symbolInFrame = 0UZ;
    std::uint64_t _symbolIndex   = 0ULL;
    std::uint64_t _frameIndex    = 0ULL;
    std::uint64_t _discarded     = 0ULL;

    /// @brief Samples discarded between frames, over the whole run.
    [[nodiscard]] std::uint64_t nDiscarded() const noexcept { return _discarded; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_len", "cp_len", "timing_offset"};
        if (_cut.empty() || std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            rebuild();
        } else if (newSettings.contains("signal_name")) {
            buildShape();
        }
    }

    /// @brief Builds the cut, the transform buffers and the record shape from the members.
    ///
    /// `start()` runs it as well. A block whose settings never change gets its cut there. Every check runs before the
    /// block changes. A refused setting throws and leaves the block as it was.
    void rebuild() {
        detail::requireFftLength(fft_len);
        detail::requireCyclicPrefix(std::span<const gr::Size_t>(cp_len.value), fft_len);

        const auto shortest = static_cast<std::int32_t>(*std::ranges::min_element(cp_len.value));
        if (timing_offset.value > 0 || timing_offset.value < -shortest) {
            throw gr::exception(std::format("timing_offset is {} and must lie in [{}, 0]: outside it the cut leaves the symbol whose prefix it is biased into", timing_offset.value, -shortest));
        }

        const std::size_t fftLength = static_cast<std::size_t>(fft_len.value);
        const bool        newCut    = fftLength != _cut.size() || cp_len.value != _prefixes || timing_offset.value != _offset;
        if (newCut && _collecting) { // the open frame's cadence and its partial cut belong to the old settings
            _discarded += static_cast<std::uint64_t>(_have);
            _have       = 0UZ;
            _collecting = false;
        }
        _prefixes = cp_len.value;
        _offset   = timing_offset.value;
        if (fftLength != _cut.size()) {
            _cut.assign(fftLength, Complex{});
            _spectrum.assign(fftLength, Complex{});
            _binCarriers.resize(fftLength);
            for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
                _binCarriers[bin] = detail::carrierAt(fftLength, bin);
            }
        }
        buildShape();
    }

    void start() {
        if (_cut.empty()) {
            rebuild();
        }
        _collecting    = false;
        _have          = 0UZ;
        _symbolInFrame = 0UZ;
        _symbolIndex   = 0ULL;
        _frameIndex    = 0ULL;
        _discarded     = 0ULL;
    }

    /**
     * @brief Walks the call's samples in stretches: a skip over a prefix, a copy into a cut, or a discard between frames.
     *
     * A trigger opens a frame when the walk reaches its sample and the output has room for a record. A call that stops
     * early leaves the rest of the span, with its tags, to the next call.
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
                ++nextTrig; // a second trigger on a sample the walk has passed
            }
            if (nextTrig < triggers.size() && triggers[nextTrig] == here) {
                if (made == outSpan.size()) {
                    break; // the trigger stays in the span, and the next call opens its frame
                }
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
            if (here < _windowStart) { // the part of the prefix before the cut
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
        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(_prefixes), symbolInFrame);
        return symbolStart + static_cast<std::uint64_t>(static_cast<std::int64_t>(prefix) + _offset);
    }

    /// @brief Steps the cadence to the next symbol, or ends the frame after its last symbol.
    void advance() {
        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(_prefixes), _symbolInFrame);
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

    void buildShape() {
        const std::uint64_t fftLength = _cut.size();
        _shape.build(std::span<const int>(_binCarriers), signal_name.value,
            property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(fftLength)}, //
                {std::pmr::string(detail::kTimingOffsetKey), pmt::Value(static_cast<std::int64_t>(_offset))}});
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CYCLIC_PREFIX_HPP
