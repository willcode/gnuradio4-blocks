#ifndef GNURADIO_FEC_INTERLEAVE_BLOCKS_HPP
#define GNURADIO_FEC_INTERLEAVE_BLOCKS_HPP

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fec/Interleaver.hpp>

#include <gnuradio-4.0/fec/RecordShape.hpp>

/**
 * The record-native adapters over the interleaver families in `gnuradio-4.0/algorithm/fec`.
 *
 * An interleaver spreads a burst. A channel that destroys a run of adjacent items hands a decoder
 * a run of adjacent errors, which is the one shape a code with a small correcting radius cannot
 * take; moving the items apart before the channel and back afterwards turns that run into single
 * errors scattered over a frame, which is the shape every code is designed for. Nothing here
 * reads an item's value, so the whole layer is addressing arithmetic and the item type is the
 * carrier's business rather than the family's.
 *
 * The carrier is the bit record the other adapters in this module speak: a
 * `DataSet<std::uint8_t>` with one item per bit or per symbol, the value untouched from one end
 * to the other. A symbol-width consumer packs before this block and unpacks after it — the
 * families move items and a wider item is a different packing, not a different interleaver.
 * The framed kinds also carry `DataSet<float>`, the soft record a soft-decision chain
 * deinterleaves before its decoder: a permutation never reads a value, so the float
 * registration is addressing arithmetic over a different carrier and nothing more.
 *
 * One block pair carries all three families, selected by `kind`. The record contract, the counted
 * drops and the counters are the same for all three; the settings each family reads are below.
 *
 * - **`block`** takes `rows` and `cols`, and — on `Interleave` only — optionally the
 *   `window_offset` and `window_length` of a readout that covers only part of the interleaved
 *   frame. One record must hold a whole number of frames.
 * - **`convolutional`** takes `branches` and `unit_delay`. It is the one family here whose state
 *   crosses a record boundary: it is stream-shaped, any record length is valid, and splitting a
 *   stream into different records gives the same output items. The end-to-end
 *   interleave-to-deinterleave delay is `branches * (branches - 1) * unit_delay` items and the
 *   initial fill emits `fill_value`, so the output is 1:1 with the input from the first item.
 * - **`permutation`** takes an explicit `table`, required with no default: a published table is
 *   an interoperability constant.
 */
namespace gr::blocks::fec {

namespace detail {

//! What one output record inherits: the facts of the record the kernel drew its first item from.
struct InterleaverOrigin {
    std::size_t  begin = 0UZ; //!< the absolute item index at which that record's items started
    std::string  name{};      //!< its signal name, or the module's label where it named none
    property_map meta{};      //!< its metadata map, or an empty one where it carried none
};

//! The family settings one kernel is built from; the readout window is Interleave's alone.
struct InterleaverSettings {
    gr::fec::InterleaverKind kind         = gr::fec::InterleaverKind::unset;
    gr::Size_t               rows         = 0U;
    gr::Size_t               cols         = 0U;
    gr::Size_t               windowOffset = 0U;
    gr::Size_t               windowLength = 0U; //!< 0 reads out the whole frame
    gr::Size_t               branches     = 0U;
    gr::Size_t               unitDelay    = 0U;
    std::vector<gr::Size_t>  table{};
    gr::Size_t               fillValue = 0U;
};

//! The kernel the family settings name, for settings `checkInterleaver` accepts. The kernel's constructor judges how
//! the values combine (the frame ceiling, the window, a table that is no permutation) and throws.
template<typename T>
[[nodiscard]] inline gr::fec::Interleaver<T> interleaverKernel(const InterleaverSettings& settings) {
    const T fill = static_cast<T>(settings.fillValue);
    if (settings.kind == gr::fec::InterleaverKind::block) {
        const std::size_t frame  = std::size_t{settings.rows} * settings.cols;
        const std::size_t length = settings.windowLength == 0U ? frame : settings.windowLength;
        return gr::fec::Interleaver<T>::block(settings.rows, settings.cols, settings.windowOffset, length, fill);
    }
    if constexpr (std::same_as<T, std::uint8_t>) {
        if (settings.kind == gr::fec::InterleaverKind::convolutional) {
            return gr::fec::Interleaver<T>::convolutional(settings.branches, settings.unitDelay, fill);
        }
    }
    std::vector<std::size_t> indices(settings.table.begin(), settings.table.end());
    return gr::fec::Interleaver<T>::permutation(std::span<const std::size_t>(indices));
}

//! The fault in the settings a family requires, under the setting at fault, or nothing when the family has them.
template<typename T, typename TSettings>
[[nodiscard]] std::optional<gr::Refused> checkInterleaver(const TSettings& s) {
    switch (s.kind.value) {
    case gr::fec::InterleaverKind::block:
        if (s.rows.value == 0U) {
            return gr::Refused(s.rows, "required");
        }
        if (s.cols.value == 0U) {
            return gr::Refused(s.cols, "required");
        }
        return {};
    case gr::fec::InterleaverKind::convolutional:
        if (!std::same_as<T, std::uint8_t>) {
            return gr::Refused(s.kind, "convolutional requires byte items");
        }
        if (s.branches.value < 2U) {
            return gr::Refused(s.branches, "must be >= 2 when kind is convolutional");
        }
        if (s.unit_delay.value == 0U) {
            return gr::Refused(s.unit_delay, "required");
        }
        return {};
    case gr::fec::InterleaverKind::permutation:
        if (s.table.value.empty()) {
            return gr::Refused(s.table, "required");
        }
        return {};
    case gr::fec::InterleaverKind::unset: break;
    }
    return gr::Refused(s.kind, "required");
}

/*!
 * @brief The state the convolutional kind needs beyond the kernel's own: which record each output
 * span begins in.
 *
 * The kernel holds the delay lines and the commutator, which is everything the items need. What it
 * cannot hold is the record vocabulary, because it has never heard of a record. The item at
 * absolute output position `p` left branch `p mod B`, so it entered the lines `delay(p mod B)`
 * items earlier; the record that held that item is the one whose facts the output span inherits.
 * Keeping the origins in a queue costs one entry per record in flight, which the latency bounds.
 */
class OriginQueue {
public:
    void reset() noexcept {
        _origins.clear();
        _items = 0UZ;
    }

    //! Note that @p record's items begin at the current stream position, and hand back that position.
    template<typename T>
    [[nodiscard]] std::size_t admit(const DataSet<T>& record) {
        const std::size_t begin = _items;
        _origins.push_back(InterleaverOrigin{begin, originName(record), originMeta(record)});
        _items += record.signal_values.size();
        return begin;
    }

    /*!
     * @brief The origin of the item at absolute output position @p start, given a delay of
     * @p delayItems items and a longest delay of @p maxDelay.
     *
     * A delay reaching back before the first item is the initial fill: no record held that item, and
     * the record being consumed is the only one there is to name.
     *
     * The queue is retired here rather than by walking it forward, because the delay is the
     * commutator's and therefore cycles: consecutive spans reach back by different amounts and the
     * source index is not monotonic. What no future span can reach is everything older than
     * `start - maxDelay`, and that is the only bound safe to drop by.
     */
    [[nodiscard]] const InterleaverOrigin& originAt(std::size_t start, std::size_t delayItems, std::size_t maxDelay) noexcept {
        const std::size_t horizon = (start > maxDelay) ? start - maxDelay : 0UZ;
        while (_origins.size() > 1UZ && _origins[1UZ].begin <= horizon) {
            _origins.pop_front();
        }
        if (delayItems > start) {
            return _origins.back(); // still filling: the span opens on fill values, which came from no record
        }
        const std::size_t source = start - delayItems;
        std::size_t       pick   = 0UZ;
        while (pick + 1UZ < _origins.size() && _origins[pick + 1UZ].begin <= source) {
            ++pick;
        }
        return _origins[pick];
    }

private:
    std::deque<InterleaverOrigin> _origins{};
    std::size_t                   _items{0UZ};
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fec::Interleave, [T], [ std::uint8_t, float ])

template<typename T>
requires std::same_as<T, std::uint8_t> || std::same_as<T, float>
struct Interleave : Block<Interleave<T>> {
    using Description = Doc<R""(
@brief Interleave: item records in, the same items in the permuted order of the family `kind` names out.

The framed kinds, `block` and `permutation`, turn one record into one record. A record's length must be a nonzero
multiple of the family's frame, and each frame becomes `rows * cols` items, or the window's length where `kind` is
`block` and a window is set. A record whose length fails that test is dropped and counted in `nRecordsRefused`, one
line at `stop()` reports the total, and the record after it is interleaved normally.

The `convolutional` kind is stream-shaped. Any nonzero length is a valid record, the output is 1:1 with the input from
the first item, and the delay lines keep their contents across records, so the same stream cut into different records
interleaves identically. Only an empty record is refused. It carries bytes only: a `float` block refuses it at
configure.

An interleaver has no status to report, so the record's facts cross unchanged; its signal name and its single-map
shape follow it, and the output record's extent names its own length. A framed output carries the facts of the record
it came from; a convolutional output carries those of the record that held the first item of its span, or those of the
record being consumed while the delay lines are still filling. Deinterleave puts the items back.

Under kind block, the rectangle of `rows` by `cols` is written row-major and read column-major, so items adjacent at the
input land `rows` apart at the output; `window_offset` and `window_length` select the part of the readout emitted. Under
convolutional, branch b of the `branches` delay lines holds b * unit_delay cells, and the end-to-end delay is branches *
(branches - 1) * unit_delay items. Under permutation, out[i] = in[table[i]], every index below the frame size once.
)"">;

    PortIn<DataSet<T>, Async>  in;
    PortOut<DataSet<T>, Async> out;

    struct Settings {
        Annotated<gr::fec::InterleaverKind, Doc<"interleaver family">, Visible>   kind          = gr::fec::InterleaverKind::unset;
        Annotated<gr::Size_t, Doc<"block rows">, Visible>                         rows          = 0U;
        Annotated<gr::Size_t, Doc<"block columns">, Visible>                      cols          = 0U;
        Annotated<gr::Size_t, Doc<"readout window offset">>                       window_offset = 0U;
        Annotated<gr::Size_t, Doc<"readout window length (0 = whole frame)">>     window_length = 0U;
        Annotated<gr::Size_t, Doc<"delay line count">, Visible>                   branches      = 0U;
        Annotated<gr::Size_t, Doc<"cells per branch step">, Visible>              unit_delay    = 0U;
        Annotated<std::vector<gr::Size_t>, Doc<"permutation table">>              table{};
        Annotated<gr::Size_t, Doc<"convolutional fill value">, Limits<Max<255U>>> fill_value = 0U;

        GR_MAKE_REFLECTABLE(Settings, kind, rows, cols, window_offset, window_length, branches, unit_delay, table, fill_value);
    } settings;

    GR_MAKE_REFLECTABLE(Interleave, in, out, settings);

    std::optional<gr::fec::Interleaver<T>> interleaver{};
    detail::OriginQueue                    _origins{};

    // Interleave's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nItems          = 0ULL; ///< items those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a whole number of frames

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::optional<gr::Refused> refused = detail::checkInterleaver<T>(s);
        if (refused) {
            return refused;
        }
        std::ignore = kernelOf(s);
        return {};
    }

    void applySettings(const Settings& s) {
        interleaver.emplace(kernelOf(s));
        _origins.reset();
    }

    [[nodiscard]] gr::fec::Interleaver<T> kernelOf(const Settings& s) const {
        return detail::interleaverKernel<T>({.kind = s.kind.value,
            .rows                                  = s.rows.value,
            .cols                                  = s.cols.value,
            .windowOffset                          = s.window_offset.value,
            .windowLength                          = s.window_length.value, //
            .branches                              = s.branches.value,
            .unitDelay                             = s.unit_delay.value,
            .table                                 = s.table.value,
            .fillValue                             = s.fill_value.value});
    }

    // a restart begins the stream again: the delay lines hold their fill and no record is in flight
    void start() {
        interleaver->reset();
        _origins.reset();
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"items", nItems}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::Interleave", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool        stream   = interleaver->kind() == gr::fec::InterleaverKind::convolutional;
        const std::size_t inFrame  = interleaver->frameSize();
        const std::size_t outFrame = interleaver->interleavedSize();

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<T>& record = inSpan[consumed];
            const std::size_t items  = record.signal_values.size();
            if (items == 0UZ || (!stream && items % inFrame != 0UZ)) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<T> moved;
            moved.signal_values.resize(stream ? items : (items / inFrame) * outFrame);
            if (stream) {
                // The span opens at the stream position this record's items open at, so the branch the
                // commutator is on there is what dates the span's first item.
                const std::size_t start                 = _origins.admit(record);
                const std::size_t depth                 = (start % interleaver->branches()) * interleaver->unitDelay() * interleaver->branches();
                std::ignore                             = interleaver->interleave(record.signal_values, moved.signal_values);
                const detail::InterleaverOrigin& origin = _origins.originAt(start, depth, interleaver->latency());
                detail::carryOrigin(moved, origin.name, origin.meta);
            } else {
                std::ignore = interleaver->interleave(record.signal_values, moved.signal_values);
                detail::carryOrigin(moved, detail::originName(record), detail::originMeta(record));
            }

            ++nRecords;
            nItems += moved.signal_values.size();
            outSpan[made] = std::move(moved);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::fec::Deinterleave, [T], [ std::uint8_t, float ])

template<typename T>
requires std::same_as<T, std::uint8_t> || std::same_as<T, float>
struct Deinterleave : Block<Deinterleave<T>> {
    using Description = Doc<R""(
@brief Deinterleave: the interleaved items in, the original order out, under the family `kind` names.

The inverse of Interleave under the same settings, item for item. The `convolutional` kind runs the complementary
delay set, so an interleave followed by a deinterleave delays the stream by exactly
`branches * (branches - 1) * unit_delay` items and changes nothing else about it; it carries bytes only, as on
Interleave.

`output_offset` and `output_length` window the deinterleaved frame: the emitted record is items
`[output_offset, output_offset + output_length)` of each frame, a length of 0 meaning to the end of the frame, and the
record's extent names the emitted length. A distributed sync word occupies the deinterleaved frame's first items, so
`output_offset` at the word's item count hands the decoder the coded block alone. The window addresses a frame, so the
stream-shaped `convolutional` kind refuses it. Interleave's readout window is a different thing, a partial readout of
the interleaved frame on the transmit side, and has no counterpart here.

Length refusals, the counters and the metadata carry are Interleave's, unchanged.

Under kind block, the rectangle of `rows` by `cols` is written row-major and read column-major. Under the framed kinds,
`output_offset` items are dropped from the front of each deinterleaved frame and `output_length` items kept. Under
convolutional, branch b of the `branches` delay lines holds (branches - 1 - b) * unit_delay cells on this side, and the
end-to-end delay is branches * (branches - 1) * unit_delay items. Under permutation, `table` is the interleaver's table,
inverted once at configure.
)"">;

    PortIn<DataSet<T>, Async>  in;
    PortOut<DataSet<T>, Async> out;

    struct Settings {
        Annotated<gr::fec::InterleaverKind, Doc<"interleaver family">, Visible>   kind          = gr::fec::InterleaverKind::unset;
        Annotated<gr::Size_t, Doc<"block rows">, Visible>                         rows          = 0U;
        Annotated<gr::Size_t, Doc<"block columns">, Visible>                      cols          = 0U;
        Annotated<gr::Size_t, Doc<"output offset">>                               output_offset = 0U;
        Annotated<gr::Size_t, Doc<"output length (0 = to frame end)">>            output_length = 0U;
        Annotated<gr::Size_t, Doc<"delay line count">, Visible>                   branches      = 0U;
        Annotated<gr::Size_t, Doc<"cells per branch step">, Visible>              unit_delay    = 0U;
        Annotated<std::vector<gr::Size_t>, Doc<"permutation table">>              table{};
        Annotated<gr::Size_t, Doc<"convolutional fill value">, Limits<Max<255U>>> fill_value = 0U;

        GR_MAKE_REFLECTABLE(Settings, kind, rows, cols, output_offset, output_length, branches, unit_delay, table, fill_value);
    } settings;

    GR_MAKE_REFLECTABLE(Deinterleave, in, out, settings);

    std::optional<gr::fec::Interleaver<T>> interleaver{};
    detail::OriginQueue                    _origins{};
    std::size_t                            _outOffset = 0UZ; //!< the resolved window, in items of the deinterleaved frame
    std::size_t                            _outLength = 0UZ;
    std::vector<T>                         _whole{}; //!< the full deinterleaved frames, when only a window of them is emitted

    // Deinterleave's counters are plain members, read by the owning thread and reported once at stop().
    std::uint64_t nRecords        = 0ULL; ///< records published on `out`
    std::uint64_t nItems          = 0ULL; ///< items those records carry
    std::uint64_t nRecordsRefused = 0ULL; ///< records whose length was not a whole number of frames

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::optional<gr::Refused> refused = detail::checkInterleaver<T>(s);
        if (refused) {
            return refused;
        }
        const gr::fec::Interleaver<T> kernel = kernelOf(s);
        if (s.kind == gr::fec::InterleaverKind::convolutional) { // stream-shaped: there is no frame for an output window to address
            if (s.output_offset != 0U) {
                return gr::Refused(s.output_offset, "must be 0 when kind is convolutional");
            }
            if (s.output_length != 0U) {
                return gr::Refused(s.output_length, "must be 0 when kind is convolutional");
            }
            return {};
        }
        if (s.output_offset >= kernel.frameSize()) {
            return gr::Refused(s.output_offset, "must be < the frame size");
        }
        if (std::size_t{s.output_offset} + s.output_length > kernel.frameSize()) {
            return gr::Refused(s.output_length, "output_offset + output_length must be <= the frame size");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        interleaver.emplace(kernelOf(s));
        const bool framed = interleaver->kind() != gr::fec::InterleaverKind::convolutional; // the window is read by the framed kinds alone
        _outOffset        = framed ? std::size_t{s.output_offset} : 0UZ;
        _outLength        = framed && s.output_length != 0U ? std::size_t{s.output_length} : interleaver->frameSize() - _outOffset;
        _origins.reset();
    }

    [[nodiscard]] gr::fec::Interleaver<T> kernelOf(const Settings& s) const { return detail::interleaverKernel<T>({.kind = s.kind.value, .rows = s.rows.value, .cols = s.cols.value, .branches = s.branches.value, .unitDelay = s.unit_delay.value, .table = s.table.value, .fillValue = s.fill_value.value}); }

    // a restart returns the delay lines to their fill and forgets the records in flight
    void start() {
        interleaver->reset();
        _origins.reset();
    }

    void stop() {
        const std::array<std::pair<std::string_view, std::uint64_t>, 3UZ> counters{{{"records", nRecords}, {"items", nItems}, {"records refused", nRecordsRefused}}};
        detail::report("gr::blocks::fec::Deinterleave", this->name, counters);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const bool        stream   = interleaver->kind() == gr::fec::InterleaverKind::convolutional;
        const std::size_t inFrame  = interleaver->interleavedSize();
        const std::size_t outFrame = interleaver->frameSize();

        const bool windowed = !stream && !(_outOffset == 0UZ && _outLength == outFrame);

        std::size_t consumed = 0UZ;
        std::size_t made     = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<T>& record = inSpan[consumed];
            const std::size_t items  = record.signal_values.size();
            if (items == 0UZ || (!stream && items % inFrame != 0UZ)) {
                ++nRecordsRefused;
                continue;
            }

            DataSet<T> moved;
            if (stream) {
                // The complementary delay set, so that the two sides together delay every item alike.
                moved.signal_values.resize(items);
                const std::size_t start                 = _origins.admit(record);
                const std::size_t depth                 = (interleaver->branches() - 1UZ - (start % interleaver->branches())) * interleaver->unitDelay() * interleaver->branches();
                std::ignore                             = interleaver->deinterleave(record.signal_values, moved.signal_values);
                const detail::InterleaverOrigin& origin = _origins.originAt(start, depth, interleaver->latency());
                detail::carryOrigin(moved, origin.name, origin.meta);
            } else if (windowed) {
                // deinterleave the whole frames, then emit the window of each: the window addresses the
                // deinterleaved order, which only exists once the frame is whole
                const std::size_t frames = items / inFrame;
                _whole.resize(frames * outFrame);
                std::ignore = interleaver->deinterleave(record.signal_values, _whole);
                moved.signal_values.resize(frames * _outLength);
                for (std::size_t f = 0UZ; f < frames; ++f) {
                    const T* from = _whole.data() + f * outFrame + _outOffset;
                    std::copy_n(from, _outLength, moved.signal_values.data() + f * _outLength);
                }
                detail::carryOrigin(moved, detail::originName(record), detail::originMeta(record));
            } else {
                moved.signal_values.resize((items / inFrame) * outFrame);
                std::ignore = interleaver->deinterleave(record.signal_values, moved.signal_values);
                detail::carryOrigin(moved, detail::originName(record), detail::originMeta(record));
            }

            ++nRecords;
            nItems += moved.signal_values.size();
            outSpan[made] = std::move(moved);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::fec

#endif // GNURADIO_FEC_INTERLEAVE_BLOCKS_HPP
