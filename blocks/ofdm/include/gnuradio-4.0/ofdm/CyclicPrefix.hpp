#ifndef GNURADIO_OFDM_CYCLIC_PREFIX_HPP
#define GNURADIO_OFDM_CYCLIC_PREFIX_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/MemoryAllocators.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fourier/fft.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::CpInsert)

/**
 * @brief Turns frequency-domain symbol records into a time-domain stream, each symbol behind a cyclic prefix.
 *
 * The block closes the frequency-domain part of an OFDM transmitter. It follows `CarrierAllocator` and puts out the
 * complex baseband stream. Its input is one `DataSet<std::complex<float>>` record per symbol, with its values in FFT
 * bin order. The record's length is the transform length. The block takes the inverse transform of the record, scaled
 * by one over the record's length, and emits the symbol behind its prefix. `CpRemove`'s forward transform of the
 * symbol returns the record's values. The block reads `symbol_in_frame` from the record's meta and forwards no input
 * tag.
 *
 * With `emit_trigger` set, the block tags the first sample of each frame, the first prefix sample of the symbol at
 * position 0. The tag holds `gr:trigger_name` set to `trigger_label` and `gr:trigger_offset` 0. `CpRemove` and
 * `SchmidlCoxSync` take the same label by default.
 *
 * The prefix is a copy of the symbol's last samples. `cp_len` holds one prefix length or a cycle of lengths. The block
 * reads the cycle at the record's `symbol_in_frame`, and the cycle restarts with each frame. A record without
 * `symbol_in_frame` takes the position after the previous record's. A record shorter than the longest `cp_len` entry is
 * dropped.
 *
 * `window_len` above 0 blends neighboring symbols over that many samples. The block extends each symbol cyclically by
 * `window_len` samples with a falling raised-cosine edge, and adds the extension to the next symbol's rising head. The
 * edges lower the out-of-band power and change the first `window_len` samples of each symbol. The stream still advances
 * by `cp_len + fft_len` samples per symbol.
 *
 * `cp_len` holds at least one entry, and `window_len` is at most the shortest entry. The block refuses a change that
 * breaks either rule. A change of `window_len` that needs a change of `cp_len` goes with it in one application.
 *
 * A change of `cp_len` or `window_len` applies from the next record. After a change of `window_len`, the next head
 * rises without the previous symbol's extension. A change of `emit_trigger` applies from the next record, and a change
 * of `trigger_label` from the next tag.
 *
 * `nUnmarked()` counts the records without `symbol_in_frame`, and `nMalformed()` counts the dropped records. `start()`
 * sets both to 0 and restarts the cycle.
 */
struct CpInsert : Block<CpInsert, NoTagPropagation> {
    using Description = Doc<"Turns DataSet<complex<float>> symbol records into a time-domain complex stream. The block takes the inverse transform of each record, scaled by 1/N for a record of N values, and puts a cyclic prefix in front of it. cp_len holds one length or a cycle read at the record's symbol_in_frame. Optional window_len raised-cosine samples smooth the symbol edges, and the first sample of each frame carries a trigger tag. `window_len` raised-cosine edge samples are shared with the next symbol. With `emit_trigger`, the first sample of every frame carries `trigger_label` under gr:trigger_name">;

    PortIn<DataSet<Complex>, Async> in;
    PortOut<Complex>                out;

    struct Settings {
        Annotated<std::vector<gr::Size_t>, Unit<"samples">, Visible, Doc<"prefix length">>   cp_len        = std::vector<gr::Size_t>{16U};
        Annotated<gr::Size_t, Unit<"samples">, Visible, Doc<"edge window length (0 = off)">> window_len    = 0U;
        Annotated<bool, Doc<"frame trigger tags">>                                           emit_trigger  = true;
        Annotated<std::string, Doc<"emitted trigger name">>                                  trigger_label = std::string("ofdm_frame");

        GR_MAKE_REFLECTABLE(Settings, cp_len, window_len, emit_trigger, trigger_label);
    } settings;

    GR_MAKE_REFLECTABLE(CpInsert, in, out, settings);

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Backward> _inverse{};

    std::vector<Complex> _time{};    ///< one symbol, transformed, before the prefix is put in front of it
    std::vector<Complex> _out{};     ///< the symbol as a stream, with prefix, symbol and the windowed head
    std::vector<Complex> _overlap{}; ///< the previous symbol's falling extension, added into the next symbol's head
    std::vector<float>   _ramp{};    ///< the rising raised-cosine edge of `window_len` samples

    std::size_t   _fftLength     = 0UZ;
    std::size_t   _longestPrefix = 0UZ;  ///< the longest entry of `cp_len`
    std::size_t   _outAt         = 0UZ;  ///< samples of `_out` already published
    std::uint64_t _cyclePos      = 0ULL; ///< the prefix cycle's position, which is the symbol's index in its frame
    std::uint64_t _unmarked      = 0ULL; ///< records that named no `symbol_in_frame`
    std::uint64_t _malformed     = 0ULL; ///< records too short to hold their prefix
    bool          _triggerReady  = false;

    /// @brief Records that carried no `symbol_in_frame`.
    [[nodiscard]] std::uint64_t nUnmarked() const noexcept { return _unmarked; }

    /// @brief Records shorter than their prefix. The block drops them.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed; }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.cp_len.value.empty()) {
            return gr::Refused(s.cp_len, "required");
        }
        if (s.window_len > std::ranges::min(s.cp_len.value)) {
            return gr::Refused(s.window_len, "must be <= min(cp_len)");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        if (s.window_len.changed()) {
            buildRamp(s.window_len);
        }
        _longestPrefix = std::ranges::max(s.cp_len.value);
    }

    void start() {
        _out.clear();
        _outAt        = 0UZ;
        _cyclePos     = 0ULL;
        _unmarked     = 0ULL;
        _malformed    = 0ULL;
        _triggerReady = false;
        std::ranges::fill(_overlap, Complex{});
    }

    /**
     * @brief Publishes the samples of the record in hand, then builds the next record, while both spans allow.
     *
     * The block builds a record's samples into a buffer. The record stays in the input until its last sample is
     * published. A call that runs out of output room publishes the rest of the buffer on the next call.
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
                    outSpan.publishTag(property_map{{gr::tag::TRIGGER_NAME.key(), settings.trigger_label.value}, {gr::tag::TRIGGER_OFFSET.key(), 0.f}}, written);
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
    void buildRamp(std::size_t edge) {
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
            _cyclePos = marker;
        }
        if (_cyclePos == 0ULL && settings.emit_trigger.value) {
            _triggerReady = true;
        }

        _inverse.compute(record.signal_values, std::span<Complex>(_time));
        const float scale = 1.f / static_cast<float>(fftLength);
        for (Complex& value : _time) {
            value *= scale;
        }

        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(settings.cp_len.value), _cyclePos);
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
 * @brief Cuts a time-domain stream into frequency-domain symbol records, one per symbol, from a frame trigger.
 *
 * The block opens the frequency-domain part of an OFDM receiver. It follows `SchmidlCoxSync` and feeds
 * `ChannelEqualizer`. Its input is the complex baseband stream. Its output is one `DataSet<std::complex<float>>` record
 * per symbol: the unnormalized forward transform of the symbol's `fft_len` samples, in FFT bin order. Its axis holds
 * each value's signed carrier index, and `signal_name` names its signal. Its meta holds `fft_len`, `timing_offset`,
 * `symbol_kind` (`sync` for the first `n_sync` symbols of a frame and `data` after them), `symbol_index` (the record's
 * index in the stream), `frame_index`, `symbol_in_frame`, `sample_start` (the input index of the cut's first sample)
 * and `discarded_samples` (the `nDiscarded()` total at the cut). The block forwards no input tag.
 *
 * A frame starts at a trigger, a tag whose `gr:trigger_name` equals `trigger_label`, on the first prefix sample of the
 * frame's first symbol. `CpInsert` and `SchmidlCoxSync` put their triggers there. An empty `trigger_label` accepts
 * every trigger. From the trigger the block skips each symbol's prefix and cuts the `fft_len` samples after it.
 * `cp_len` holds one prefix length or a cycle of lengths, read at the symbol's position in the frame. A frame runs
 * until the next trigger. With `frame_len` above 0 it also ends after `n_sync + frame_len` symbols. The block discards
 * the samples between frames, the partial cut a trigger interrupts and the partial cut at the end of the stream.
 *
 * `timing_offset`, from `-min(cp_len)` to 0, starts each cut that many samples early, inside the prefix. The early cut
 * leaves a margin against the channel's delay spread and turns carrier `k` by `2*pi*k*timing_offset/fft_len`. `fft_len`
 * is a power of two from 8 to 65536, and `cp_len` holds at least one entry, none longer than `fft_len`. The block
 * refuses a value outside these ranges. When a new value fits its range only beside a new value of another setting,
 * both go in one application.
 *
 * A change of `fft_len`, `cp_len` or `timing_offset` ends the open frame, and the block discards samples until the next
 * trigger. A change of `n_sync` or `frame_len` sets the length of the open frame, and a frame that already holds that
 * many symbols ends at once. A change of `trigger_label` applies from the next trigger, and a change of `signal_name`
 * from the next record.
 *
 * `nDiscarded()` counts the discarded samples. `start()` sets it and the indices to 0.
 */
struct CpRemove : Block<CpRemove, NoTagPropagation> {
    using Description = Doc<"Cuts a complex stream into one DataSet<complex<float>> record of fft_len bins per symbol and takes its forward transform. A frame starts at a tag whose gr:trigger_name equals trigger_label, on the first prefix sample of its first symbol. The block skips each prefix, cuts fft_len samples moved by timing_offset and discards the samples between frames. `cp_len` is one length or a per-symbol cycle. The `n_sync` sync symbols at a frame's head are cut like data symbols. `timing_offset` is the signed bias of the cut">;

    PortIn<Complex>                  in;
    PortOut<DataSet<Complex>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Visible, Doc<"FFT length">, detail::FftLengthLimits>                  fft_len       = 64U;
        Annotated<std::vector<gr::Size_t>, Unit<"samples">, Visible, Doc<"prefix length">>          cp_len        = std::vector<gr::Size_t>{16U};
        Annotated<gr::Size_t, Visible, Doc<"sync symbols per frame">>                               n_sync        = 0U;
        Annotated<gr::Size_t, Visible, Doc<"data symbols per frame (0 = to next trigger)">>         frame_len     = 0U;
        Annotated<std::int32_t, Visible, Unit<"samples">, Doc<"cut timing offset">, Limits<Max<0>>> timing_offset = 0;
        Annotated<std::string, Doc<"frame trigger name (empty = any)">>                             trigger_label = std::string("ofdm_frame");
        Annotated<std::string, Doc<"signal name">>                                                  signal_name   = std::string("ofdm_symbol");

        GR_MAKE_REFLECTABLE(Settings, fft_len, cp_len, n_sync, frame_len, timing_offset, trigger_label, signal_name);
    } settings;

    GR_MAKE_REFLECTABLE(CpRemove, in, out, settings);

    gr::algorithm::FFT<Complex, Complex, gr::algorithm::Direction::Forward> _forward{};

    std::vector<Complex, gr::allocator::Aligned<Complex>> _cut{};         ///< the samples of the cut collected so far
    std::vector<Complex, gr::allocator::Aligned<Complex>> _spectrum{};    ///< the transform's output
    std::vector<int>                                      _binCarriers{}; ///< the signed carrier each bin holds, which is the record's axis
    detail::SymbolRecordShape                             _shape{};       ///< the invariant part of a record

    bool          _collecting    = false; ///< inside a frame, and discarding when false
    std::uint64_t _symbolStart   = 0ULL;  ///< absolute index of the current symbol's first prefix sample
    std::uint64_t _windowStart   = 0ULL;  ///< absolute index of the current cut's first sample
    std::size_t   _have          = 0UZ;   ///< samples of the cut already collected
    std::uint64_t _symbolInFrame = 0ULL;
    std::uint64_t _symbolIndex   = 0ULL;
    std::uint64_t _frameIndex    = 0ULL;
    std::uint64_t _discarded     = 0ULL;

    /// @brief Samples discarded between frames, over the whole run.
    [[nodiscard]] std::uint64_t nDiscarded() const noexcept { return _discarded; }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (!std::has_single_bit(s.fft_len.value)) {
            return gr::Refused(s.fft_len, "must be a power of 2");
        }
        if (s.cp_len.value.empty()) {
            return gr::Refused(s.cp_len, "required");
        }
        const gr::Size_t shortest = std::ranges::min(s.cp_len.value);
        if (std::ranges::max(s.cp_len.value) > s.fft_len) {
            return gr::Refused(s.cp_len, "must be <= fft_len");
        }
        if (s.timing_offset < -static_cast<std::int32_t>(shortest)) {
            return gr::Refused(s.timing_offset, "must be >= -min(cp_len)");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        const bool newCut = s.fft_len.changed() || s.cp_len.changed() || s.timing_offset.changed();
        if (newCut && _collecting) { // the open frame's cadence and its partial cut belong to the old settings
            endFrame();
        }
        prepare(newCut || s.signal_name.changed());
        if (_collecting && s.frame_len > 0U && _symbolInFrame >= std::uint64_t{s.n_sync} + s.frame_len) {
            endFrame();
        }
    }

    void start() {
        _collecting    = false;
        _have          = 0UZ;
        _symbolInFrame = 0ULL;
        _symbolIndex   = 0ULL;
        _frameIndex    = 0ULL;
        _discarded     = 0ULL;
    }

    /**
     * @brief Cuts the call's samples into records.
     *
     * A call that stops early leaves the rest of the span, with its tags, to the next call.
     */
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const Walk walked = walk(inSpan, outSpan);
        outSpan.publish(walked.made);
        std::ignore = inSpan.consume(walked.taken);
        if (walked.taken == 0UZ && walked.made == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /**
     * @brief Cuts the samples the stream ends with, then ends the open frame.
     *
     * The partial cut and the samples left for want of output room count as discarded.
     */
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const Walk walked = walk(inSpan, outSpan);
        _discarded += static_cast<std::uint64_t>(inSpan.size() - walked.taken);
        endFrame();
        outSpan.publish(walked.made);
        std::ignore = inSpan.consume(inSpan.size());
        return work::Status::OK;
    }

private:
    /// @brief The samples one walk took and the records it made.
    struct Walk {
        std::size_t taken = 0UZ;
        std::size_t made  = 0UZ;
    };

    /**
     * @brief Walks the span in stretches: a skip over a prefix, a copy into a cut, or a discard between frames.
     *
     * A trigger opens a frame when the walk reaches its sample and the output has room for a record.
     */
    [[nodiscard]] Walk walk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
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
        return Walk{at, made};
    }

    [[nodiscard]] bool isTrigger(const property_map& tagMap) const {
        const auto entry = tagMap.find(std::pmr::string(gr::tag::TRIGGER_NAME.key()));
        if (entry == tagMap.end()) {
            return false;
        }
        if (settings.trigger_label.value.empty()) {
            return true;
        }
        const auto* label = entry->second.get_if<std::pmr::string>();
        return label != nullptr && std::string_view(label->data(), label->size()) == std::string_view(settings.trigger_label.value);
    }

    /// @brief Sizes the cut and the transform buffers for `fft_len`, and builds the record shape when @p newShape is set.
    void prepare(bool newShape) {
        const std::size_t fftLength = settings.fft_len;
        if (fftLength != _cut.size()) {
            _cut.assign(fftLength, Complex{});
            _spectrum.assign(fftLength, Complex{});
            _binCarriers.resize(fftLength);
            for (std::size_t bin = 0UZ; bin < fftLength; ++bin) {
                _binCarriers[bin] = detail::carrierAt(fftLength, bin);
            }
        }
        if (newShape) {
            buildShape();
        }
    }

    /// @brief Ends the open frame. The samples of its partial cut count as discarded.
    void endFrame() {
        _discarded += static_cast<std::uint64_t>(_have);
        _have       = 0UZ;
        _collecting = false;
    }

    void openFrame(std::uint64_t at) {
        if (_collecting || _symbolIndex > 0ULL) {
            ++_frameIndex;
        }
        _discarded += static_cast<std::uint64_t>(_have); // the partial cut of the frame this trigger ends
        _collecting    = true;
        _symbolInFrame = 0ULL;
        _have          = 0UZ;
        _symbolStart   = at;
        _windowStart   = windowOf(at, 0ULL);
    }

    [[nodiscard]] std::uint64_t windowOf(std::uint64_t symbolStart, std::uint64_t symbolInFrame) const {
        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(settings.cp_len.value), symbolInFrame);
        return symbolStart + static_cast<std::uint64_t>(static_cast<std::int64_t>(prefix) + settings.timing_offset.value);
    }

    /// @brief Steps the cadence to the next symbol, or ends the frame after its last symbol.
    void advance() {
        const std::size_t prefix = detail::cyclicPrefixLength(std::span<const gr::Size_t>(settings.cp_len.value), _symbolInFrame);
        _symbolStart += prefix + _cut.size();
        ++_symbolInFrame;
        ++_symbolIndex;
        _have = 0UZ;

        if (settings.frame_len.value > 0U && _symbolInFrame >= std::uint64_t{settings.n_sync.value} + settings.frame_len.value) {
            _collecting = false;
            return;
        }
        _windowStart = windowOf(_symbolStart, _symbolInFrame);
    }

    void emit(DataSet<Complex>& target) {
        _forward.compute(_cut, std::span<Complex>(_spectrum));
        const std::string_view kind                     = _symbolInFrame < settings.n_sync.value ? detail::kKindSync : detail::kKindData;
        property_map&          meta                     = _shape.emitInto(target, std::span<const Complex>(_spectrum), kind);
        meta[std::pmr::string(detail::kSymbolIndexKey)] = pmt::Value(_symbolIndex);
        meta[std::pmr::string(detail::kFrameIndexKey)]  = pmt::Value(_frameIndex);
        meta[std::pmr::string(detail::kSymbolInFrame)]  = pmt::Value(_symbolInFrame);
        meta[std::pmr::string(detail::kSampleStartKey)] = pmt::Value(_windowStart);
        meta[std::pmr::string(detail::kDiscardedKey)]   = pmt::Value(_discarded);
    }

    void buildShape() {
        const std::uint64_t fftLength = _cut.size();
        _shape.build(std::span<const int>(_binCarriers), settings.signal_name.value,
            property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(fftLength)}, //
                {std::pmr::string(detail::kTimingOffsetKey), pmt::Value(static_cast<std::int64_t>(settings.timing_offset.value))}});
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CYCLIC_PREFIX_HPP
