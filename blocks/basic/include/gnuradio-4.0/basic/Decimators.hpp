#ifndef GNURADIO_DECIMATORS_HPP
#define GNURADIO_DECIMATORS_HPP

#include <algorithm>
#include <complex>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

namespace detail {

/**
 * @brief Republishes input tags at the output offsets where a decimator placed their samples.
 *
 * Each tag moves to the next item the block keeps. The mapping counts items against the block's own phase in integers,
 * so it is exact at any stream offset. A tag with no kept item left in the call waits for the next call. At the end of
 * the stream the block publishes every waiting tag at the end-of-stream index, one past its last output. Order and
 * multiplicity are preserved.
 */
struct TagRelay {
    std::vector<property_map> _carry{};
    std::size_t               _cursor = 0UZ;
    std::size_t               _count  = 0UZ;

    void reset() {
        _carry.clear();
        _cursor = 0UZ;
        _count  = 0UZ;
    }

    template<typename TInSpan>
    void begin(const TInSpan& inSpan) noexcept {
        _cursor = 0UZ;
        _count  = inSpan.rawTags.size();
    }

    template<typename TInSpan>
    void collect(const TInSpan& inSpan, std::size_t index) {
        const auto&       raw   = inSpan.rawTags;
        const std::size_t until = inSpan.streamIndex + index;
        while (_cursor < _count && raw[_cursor].index <= until) {
            if (raw[_cursor].index >= inSpan.streamIndex) {
                _carry.push_back(raw[_cursor].map);
            }
            ++_cursor;
        }
    }

    /// Holds every tag in the tag ring of `port` at or past input offset `from`, without its `end_of_stream` key.
    template<typename TPort>
    void collectPastEnd(TPort& port, std::size_t from) {
        if (!port.isConnected()) {
            return;
        }
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        for (const Tag& ringTag : port.tagReader().get()) {
            if (ringTag.index < from) {
                continue;
            }
            property_map tag(ringTag.map);
            tag.erase(endOfStream);
            if (!tag.empty()) {
                _carry.push_back(std::move(tag));
            }
        }
    }

    template<typename TOutSpan>
    void release(TOutSpan& outSpan, std::size_t at) {
        for (const property_map& forwarded : _carry) {
            outSpan.publishTag(forwarded, at);
        }
        _carry.clear();
    }

    /// True when nothing waits and nothing is left to collect. A run of items can then be copied in one step instead of
    /// item by item. Meaningful only after `begin`.
    [[nodiscard]] bool idle() const noexcept { return _carry.empty() && _cursor >= _count; }
};

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::basic::KeepOneInN, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

/**
 * `n` takes values from one up. The framework refuses a zero before the block sees it, and the member and
 * `settings().get()` keep the accepted `n`. A refused value from a tag is reported in a plain message, and the run goes
 * on. A refused staged value or Set message is reported in an error message, which ends a run whose messages nothing
 * reads. `settings().set()` throws at the call and stores nothing. The block computes on the `n` it last accepted,
 * starting at the default.
 *
 * `reset()` restarts the phase at the accepted `n`, as at the start of a stream. The framework calls it at the
 * lifecycle's reset and at a reset to defaults, after the defaults are applied. The next item out is then the `n`-th
 * input after the reset. The tags waiting for a kept item stay and leave on that item. When the defaults change `n`,
 * the input rate restated for the new `n` leaves there too. `start()` drops the tags an earlier run left waiting and
 * forgets its input rate.
 */
template<typename T>
struct KeepOneInN : Block<KeepOneInN<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Keeps the last item of every group of `n` and drops the rest.

The block suits display downsampling and slow status streams, where no anti-alias filtering is wanted or needed.
The first item out is the `n`-th input, then every `n`-th input after it. A change of `n` and a reset restart the
phase. The counter reloads at once, and the next item out is the `n`-th input after the change or the reset. `n` must
be at least one.

A tag on a dropped item moves forward to the next kept item. The output rate is `1/n` of the input rate. A
`sample_rate` tag leaves with its value divided by `n`. After a change of `n` the next kept item carries the input
rate of the last `sample_rate` tag the block published, divided by the new `n`. That rate leaves ahead of the tags
waiting for the item. A waiting `sample_rate` tag leaves after it and states the newer rate. At the end of the stream
a tag still waiting for a kept item leaves at the end-of-stream index, one past the last output.

The block declares no decimation ratio, and a large `n` needs no buffer of `n` items.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "n", Doc<"group size with the last item of each group kept">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>> n = 1U;

    GR_MAKE_REFLECTABLE(KeepOneInN, in, out, n);

    gr::Size_t       _n         = n.value; ///< the accepted `n`
    gr::Size_t       _counter   = n.value;
    float            _inputRate = 0.f; ///< the input rate of the last `sample_rate` tag published in this run, 0 before the first
    detail::TagRelay _relay{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!newSettings.contains("n") || n.value == _n) {
            return;
        }
        _n       = n.value;
        _counter = _n;
        if (_inputRate > 0.f) {
            _relay._carry.insert(_relay._carry.begin(), property_map{{std::pmr::string(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(_inputRate)}});
        }
    }

    void start() {
        _inputRate = 0.f;
        _relay.reset();
    }

    void reset() { _counter = _n; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(keep(inSpan, outSpan));
        return work::Status::OK;
    }

    /// At the end of the stream the block keeps what the trailing input holds and publishes every waiting tag at the
    /// end-of-stream index. At a stop request it publishes nothing and drops the waiting tags.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (lifecycle::isShuttingDown(this->state())) {
            _relay.reset();
            outSpan.publish(0UZ);
            return work::Status::OK;
        }
        const std::size_t made = keep(inSpan, outSpan);
        _relay.collectPastEnd(in, inSpan.streamIndex + inSpan.size());
        releaseWaiting(outSpan, made);
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::size_t keep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made   = 0UZ;
        std::size_t walked = 0UZ;
        _relay.begin(inSpan);

        for (; walked < inSpan.size() && made < outSpan.size(); ++walked) {
            _relay.collect(inSpan, walked);
            if (--_counter > 0U) {
                continue;
            }
            _counter      = _n;
            outSpan[made] = inSpan[walked];
            releaseWaiting(outSpan, made);
            ++made;
        }

        std::ignore = inSpan.consume(walked);
        return made;
    }

    void releaseWaiting(OutputSpanLike auto& outSpan, std::size_t at) {
        const std::pmr::string sampleRate(gr::tag::SAMPLE_RATE.shortKey());
        for (property_map& tag : _relay._carry) {
            if (auto found = tag.find(sampleRate); found != tag.end()) {
                if (const float* rate = found->second.template get_if<float>(); rate != nullptr) {
                    _inputRate    = *rate;
                    found->second = static_cast<float>(static_cast<double>(*rate) / static_cast<double>(_n));
                } else if (const double* wide = found->second.template get_if<double>(); wide != nullptr) {
                    _inputRate    = static_cast<float>(*wide);
                    found->second = *wide / static_cast<double>(_n);
                }
            }
        }
        _relay.release(outSpan, at);
    }
};

GR_REGISTER_BLOCK(gr::blocks::basic::KeepMInN, [T], [ float, std::complex<float>, std::int16_t, std::uint8_t ])

/**
 * `m` and `n` take values from one up, and the framework refuses a zero of either as `KeepOneInN` refuses a zero `n`.
 * The framework refuses that key alone and applies the rest of the application. The block computes on the `m`, `n`
 * and `offset` it last accepted. It refuses an `offset + m` above `n`, compared in 64 bits on the values an application
 * changes and the accepted values of the others. This refusal covers the whole application and changes nothing the
 * block computes. The refused values stay in their members, and `settings().get()` returns them. The block ignores
 * each one until an application changes that setting again. The framework drops a staged or tagged value equal to its
 * member before the block sees it. A refused value sent again alone therefore passes with no refusal and changes
 * nothing.
 *
 * `reset()` restarts the phase at the accepted slice, as `KeepOneInN` restarts its own. The tags waiting for a kept
 * item stay and leave on the next one. `start()` drops the tags an earlier run left waiting.
 */
template<typename T>
struct KeepMInN : Block<KeepMInN<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Keeps `m` consecutive items out of every `n`, starting `offset` items into each group.

The block extracts a fixed slice from a repeating frame. Examples are a snapshot of a few thousand items in every
period and a fixed window of a repeating burst. The kept items are indices `offset` to `offset + m - 1` of every
group of `n`, and the first group begins at the first input. `m` and `n` must be at least one, and `offset + m` at
most `n`.

The block consumes every span it receives and keeps its place in the group across calls. A group may span many calls,
and a slice may be longer than one span or than the room left in the output. The block declares no decimation ratio,
and a large `n` needs no buffer of `n` items. At the end of the stream the block has emitted the part of the last slice
it saw.

A change of `m`, `n` or `offset` and a reset restart the phase. The first group after the change or the reset begins
at the next input item.

A tag on a dropped item moves forward to the next kept item. For an item past the end of a slice, that is the first
item of the next group. A `sample_rate` tag leaves unchanged and states the spacing of the items inside a slice. At
the end of the stream a tag still waiting for a kept item leaves at the end-of-stream index, one past the last output.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    Annotated<gr::Size_t, "m", Doc<"items kept per group">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>> m      = 1U;
    Annotated<gr::Size_t, "n", Doc<"group size">, Limits<1U, std::numeric_limits<gr::Size_t>::max()>>           n      = 1U;
    Annotated<gr::Size_t, "offset", Doc<"index in the group of the first kept item">>                           offset = 0U;

    GR_MAKE_REFLECTABLE(KeepMInN, in, out, m, n, offset);

    gr::Size_t       _m      = m.value;      ///< the accepted `m`
    gr::Size_t       _n      = n.value;      ///< the accepted `n`
    gr::Size_t       _offset = offset.value; ///< the accepted `offset`
    gr::Size_t       _phase  = 0U;           ///< index within the current group of the next input item
    detail::TagRelay _relay{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        const gr::Size_t nextM      = newSettings.contains("m") ? m.value : _m;
        const gr::Size_t nextN      = newSettings.contains("n") ? n.value : _n;
        const gr::Size_t nextOffset = newSettings.contains("offset") ? offset.value : _offset;
        if (std::uint64_t{nextOffset} + std::uint64_t{nextM} > std::uint64_t{nextN}) {
            throw gr::exception(std::format("offset ({}) + m ({}) must not exceed n ({})", nextOffset, nextM, nextN));
        }
        if (nextM != _m || nextN != _n || nextOffset != _offset) {
            _m      = nextM;
            _n      = nextN;
            _offset = nextOffset;
            _phase  = 0U;
        }
    }

    void start() { _relay.reset(); }

    void reset() { _phase = 0U; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        outSpan.publish(keep(inSpan, outSpan));
        return work::Status::OK;
    }

    /// At the end of the stream the block keeps what the trailing input holds and publishes every waiting tag at the
    /// end-of-stream index. At a stop request it publishes nothing and drops the waiting tags.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (lifecycle::isShuttingDown(this->state())) {
            _relay.reset();
            outSpan.publish(0UZ);
            return work::Status::OK;
        }
        const std::size_t made = keep(inSpan, outSpan);
        _relay.collectPastEnd(in, inSpan.streamIndex + inSpan.size());
        _relay.release(outSpan, made);
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::size_t keep(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::size_t group = static_cast<std::size_t>(_n);
        const std::size_t first = static_cast<std::size_t>(_offset);
        const std::size_t last  = first + static_cast<std::size_t>(_m); // exclusive, and at most the group

        std::size_t walked = 0UZ;
        std::size_t made   = 0UZ;
        _relay.begin(inSpan);

        while (walked < inSpan.size()) {
            const std::size_t phase = static_cast<std::size_t>(_phase);
            if (phase >= first && phase < last) { // inside the slice, copy the run that fits both spans
                const std::size_t run = std::min({last - phase, inSpan.size() - walked, outSpan.size() - made});
                if (run == 0UZ) {
                    break; // output full, the rest of the slice waits for the next call
                }
                if (_relay.idle()) {
                    std::copy_n(inSpan.begin() + static_cast<std::ptrdiff_t>(walked), run, outSpan.begin() + static_cast<std::ptrdiff_t>(made));
                } else {
                    for (std::size_t i = 0UZ; i < run; ++i) {
                        _relay.collect(inSpan, walked + i);
                        outSpan[made + i] = inSpan[walked + i];
                        _relay.release(outSpan, made + i);
                    }
                }
                walked += run;
                made += run;
                _phase = static_cast<gr::Size_t>(phase + run);
            } else { // outside the slice, skip to its start or to the next group in one step
                const std::size_t until = phase < first ? first : group;
                const std::size_t run   = std::min(until - phase, inSpan.size() - walked);
                _relay.collect(inSpan, walked + run - 1UZ); // one call carries every tag of the skipped run forward
                walked += run;
                _phase = static_cast<gr::Size_t>(phase + run);
            }
            if (_phase >= static_cast<gr::Size_t>(group)) {
                _phase = 0U;
            }
        }

        std::ignore = inSpan.consume(walked);
        return made;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_DECIMATORS_HPP
