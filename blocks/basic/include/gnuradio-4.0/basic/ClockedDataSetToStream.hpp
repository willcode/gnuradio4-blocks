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
@brief Plays `DataSet<T>` records into a continuous stream of `T`, paced by a clock input.

Each record's samples appear at the record's stated position. Every other position holds
`T{}`. `DataSetToStream` concatenates records and reports the gaps between them. This block
fills the gaps. A live sink needs a continuous stream in which idle time is explicit. After
the last record the idle time has no bound. Record positions alone cannot tell the block when
to stop. The pace comes from a second input, a clock stream. The block consumes clock items
and does not read their values. For every `input_chunk_size` clock items consumed, it
publishes `output_chunk_size` output samples. These are the framework's runtime resampling
settings. The defaults 3 and 5 turn 4800 clock items into 8000 output samples, a voice rate
from a symbol rate. Output cannot run ahead of the clock. A real-time clock branch gives a
real-time output. A file-rate run of the same graph gives the same sample sequence faster.

A record's first sample goes to the output position in `meta_information[0]["sample_start"]`.
An absent key means position 0. The position is on the output stream's clock, and the block
does not translate it. A record with no samples or no metadata map is skipped and counted. A
record wholly behind the emitted position is dropped, and its length is counted in
`nLateSamples`. A partly late record plays its remainder, and the overlap is counted.
Overlapping records resolve in arrival order. The sample sequence depends only on the records
and their positions. The schedule changes it only by delivering a record late, and
`nLateSamples` counts that case.

The counters are published through `std::atomic_ref`. A status reader outside the graph's
threads can poll them without a lock.
*/
template<typename T>
struct ClockedDataSetToStream : Block<ClockedDataSetToStream<T>, Resampling<3UZ, 5UZ, false>> {
    using Description = Doc<"Plays records into a continuous stream at their stated positions, paced by a clock input. The idle value fills every other position. The output advances at a fixed ratio to the clock.">;

    PortIn<std::uint8_t>      clock; //!< consumed for pace, values not read
    PortIn<DataSet<T>, Async> in;
    PortOut<T>                out;

    GR_MAKE_REFLECTABLE(ClockedDataSetToStream, clock, in, out);

    std::deque<DataSet<T>> _pending{};
    std::uint64_t          _emitted = 0ULL;

    alignas(8) std::uint64_t _clockShared    = 0ULL;
    alignas(8) std::uint64_t _emittedShared  = 0ULL;
    alignas(8) std::uint64_t _lateShared     = 0ULL;
    alignas(8) std::uint64_t _unplacedShared = 0ULL;

    //! Clock items consumed, idle time included.
    [[nodiscard]] std::uint64_t clockItemsConsumed() const noexcept { return std::atomic_ref<const std::uint64_t>(_clockShared).load(std::memory_order_relaxed); }
    //! Output samples published.
    [[nodiscard]] std::uint64_t samplesEmitted() const noexcept { return std::atomic_ref<const std::uint64_t>(_emittedShared).load(std::memory_order_relaxed); }
    //! Samples of record content the clock had already passed, cumulative.
    [[nodiscard]] std::uint64_t nLateSamples() const noexcept { return std::atomic_ref<const std::uint64_t>(_lateShared).load(std::memory_order_relaxed); }
    //! Records skipped for want of samples or a metadata map, cumulative.
    [[nodiscard]] std::uint64_t nRecordsUnplaced() const noexcept { return std::atomic_ref<const std::uint64_t>(_unplacedShared).load(std::memory_order_relaxed); }

    //! Queues one record at its stated position, or counts it as unplaced or late.
    //! `processBulk` calls it for each arriving record.
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
            // wholly behind the emitted position
            std::atomic_ref<std::uint64_t>(_lateShared).fetch_add(record.signal_values.size(), std::memory_order_relaxed);
            return;
        }
        if (start < _emitted) {
            // the part behind the emitted position
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
    //! Stores a record's resolved position in its own metadata, where `popSample` reads it.
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
                _pending.pop_front(); // fully behind the emitted position, played or late
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
