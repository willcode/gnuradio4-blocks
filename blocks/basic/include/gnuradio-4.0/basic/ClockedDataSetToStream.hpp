#ifndef GNURADIO_CLOCKEDDATASETTOSTREAM_HPP
#define GNURADIO_CLOCKEDDATASETTOSTREAM_HPP

#include <atomic>
#include <cstdint>
#include <deque>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::ClockedDataSetToStream, [T], [float])

/*!
@brief Clock-paced record playback: `DataSet<T>` records in, a continuous stream of `T` out,
each record's samples at the record's stated position and every other position `T{}`.

`DataSetToStream` concatenates records and reports the gaps between them; this block renders
the gaps. A live sink needs a continuous stream in which idle time is explicit, and after the
last record the idle time has no bound, so gap synthesis from record positions alone cannot
know when to stop. Pace therefore arrives as a second input: a clock stream, consumed for
pace only, its values unread. For every `input_chunk_size` clock items consumed,
`output_chunk_size` output samples are published (the framework's runtime resampling
settings; the defaults 3 and 5 name the 8000/4800 voice-to-symbol ratio this block was built
against). Output cannot run ahead of the clock, so a real-time clock branch yields a
real-time output and a file-rate run of the same graph yields the identical sample sequence
faster.

A record's first-sample position on the output stream's own clock is its
`meta_information[0]["sample_start"]`; absent means position 0. The producing chain owns the
translation onto that timebase. A record with no samples or no metadata map is skipped and
counted. A record wholly behind the emitted position is dropped and its length counted in
`nLateSamples`; a partly late record plays its remainder and counts the overlap. Overlapping
records resolve in arrival order. The sample sequence depends only on the records and their
positions, never on the schedule, except where the schedule delivers a record late — which
is exactly what `nLateSamples` counts.

The counters are published through `std::atomic_ref` so a status reader outside the graph's
threads can poll them without a lock.
*/
template<typename T>
struct ClockedDataSetToStream : Block<ClockedDataSetToStream<T>, Resampling<3UZ, 5UZ, false>> {
    using Description = Doc<"Clock-paced record playback: records placed at their stated stream positions, the idle value elsewhere, output advancing at a fixed ratio to a clock input">;

    PortIn<std::uint8_t>      clock; //!< consumed for pace only, values unread
    PortIn<DataSet<T>, Async> in;
    PortOut<T>                out;

    GR_MAKE_REFLECTABLE(ClockedDataSetToStream, clock, in, out);

    std::deque<DataSet<T>> _pending{};
    std::uint64_t          _emitted = 0ULL;

    alignas(8) std::uint64_t _clockShared    = 0ULL;
    alignas(8) std::uint64_t _emittedShared  = 0ULL;
    alignas(8) std::uint64_t _lateShared     = 0ULL;
    alignas(8) std::uint64_t _unplacedShared = 0ULL;

    //! Clock items absorbed; advances through idle time.
    [[nodiscard]] std::uint64_t clockItemsConsumed() const noexcept { return std::atomic_ref<const std::uint64_t>(_clockShared).load(std::memory_order_relaxed); }
    //! Output samples published.
    [[nodiscard]] std::uint64_t samplesEmitted() const noexcept { return std::atomic_ref<const std::uint64_t>(_emittedShared).load(std::memory_order_relaxed); }
    //! Samples of record content the clock had already passed, cumulative.
    [[nodiscard]] std::uint64_t nLateSamples() const noexcept { return std::atomic_ref<const std::uint64_t>(_lateShared).load(std::memory_order_relaxed); }
    //! Records skipped for want of samples or a metadata map, cumulative.
    [[nodiscard]] std::uint64_t nRecordsUnplaced() const noexcept { return std::atomic_ref<const std::uint64_t>(_unplacedShared).load(std::memory_order_relaxed); }

    //! Admit one record: place it in the pending queue at its stated position, or count it
    //! as unplaced or late. Called by `processBulk` for each arriving record.
    void absorb(const DataSet<T>& record) {
        if (record.signal_values.empty() || record.meta_information.empty()) {
            std::atomic_ref<std::uint64_t>(_unplacedShared).fetch_add(1ULL, std::memory_order_relaxed);
            return;
        }
        std::uint64_t start = 0ULL;
        const auto&   map   = record.meta_information[0UZ];
        if (const auto entry = map.find(property_map::key_type("sample_start")); entry != map.end()) {
            start = entry->second.value_or(std::uint64_t{0ULL});
        }
        if (start + record.signal_values.size() <= _emitted) {
            // wholly in the past: the clock outran it
            std::atomic_ref<std::uint64_t>(_lateShared).fetch_add(record.signal_values.size(), std::memory_order_relaxed);
            return;
        }
        if (start < _emitted) {
            // the overlap the clock already passed
            std::atomic_ref<std::uint64_t>(_lateShared).fetch_add(_emitted - start, std::memory_order_relaxed);
        }
        _pending.push_back(record);
        writeStart(_pending.back(), start);
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& clockSpan, InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        for (const auto& record : inSpan) {
            absorb(record);
        }
        std::ignore = inSpan.consume(inSpan.size());

        const std::size_t icz    = static_cast<std::size_t>(this->input_chunk_size);
        const std::size_t ocz    = static_cast<std::size_t>(this->output_chunk_size);
        const std::size_t chunks = std::min(clockSpan.size() / icz, outSpan.size() / ocz);
        const std::size_t made   = chunks * ocz;
        for (std::size_t i = 0UZ; i < made; ++i) {
            outSpan[i] = popSample();
        }
        std::ignore = clockSpan.consume(chunks * icz);
        outSpan.publish(made);

        std::atomic_ref<std::uint64_t>(_clockShared).fetch_add(chunks * icz, std::memory_order_relaxed);
        std::atomic_ref<std::uint64_t>(_emittedShared).store(_emitted, std::memory_order_relaxed);
        if (made == 0UZ && inSpan.size() == 0UZ) {
            return outSpan.size() < ocz ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    //! Keep a record's resolved position in its own metadata, so `popSample` reads one place.
    static void writeStart(DataSet<T>& record, std::uint64_t start) { record.meta_information[0UZ]["sample_start"] = start; }

    [[nodiscard]] static std::uint64_t startOf(const DataSet<T>& record) {
        const auto entry = record.meta_information[0UZ].find(property_map::key_type("sample_start"));
        return entry == record.meta_information[0UZ].end() ? 0ULL : entry->second.value_or(std::uint64_t{0ULL});
    }

    //! The sample at the stream position now leaving, from the earliest-arrived record that
    //! covers it or from the idle value between records.
    [[nodiscard]] T popSample() {
        T value{};
        while (!_pending.empty()) {
            const DataSet<T>&   front = _pending.front();
            const std::uint64_t start = startOf(front);
            if (start + front.signal_values.size() <= _emitted) {
                _pending.pop_front(); // fully behind the stream: spent, or arrived too late
                continue;
            }
            if (_emitted >= start) {
                value = front.signal_values[_emitted - start];
            }
            break;
        }
        ++_emitted;
        return value;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_CLOCKEDDATASETTOSTREAM_HPP
