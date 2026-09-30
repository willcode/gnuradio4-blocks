#ifndef GNURADIO_IQ_TAP_HPP
#define GNURADIO_IQ_TAP_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {
using namespace gr;

GR_REGISTER_BLOCK(gr::blocks::basic::IqTap)
GR_REGISTER_BLOCK(gr::blocks::basic::FloatTap)

/*!
 * @brief Ring of the most recent samples, written by the graph and read on demand.
 *
 * A consumer outside the flow graph, such as a display or an analyzer, reads the latest
 * N samples when it asks. It does not receive every sample as it arrives. The graph makes
 * one chunked copy per work call. The consumer runs at its own rate, not at the sample
 * rate.
 *
 * A mutex guards the state. The state lives on the heap behind a shared_ptr. The block
 * stays movable, and the owner can read the ring without touching the block object.
 */
struct TapState {
    std::vector<std::complex<float>> ring;
    std::size_t                      writePos = 0;
    std::uint64_t                    total    = 0;
    /// Incremented each time the ring is reallocated. A reallocation restarts `total` from
    /// zero. A gapless reader with a cursor from before the reallocation has a gap of unknown
    /// size ahead of it. The reader detects that case by comparing `gen` with the value it
    /// started with.
    std::uint64_t      gen = 0;
    mutable std::mutex mtx;

    /*! The position of each gapless reader, and whether the writer waits for them.
     *
     * With `gate` clear the writer does not wait. A reader too slow to keep up loses the
     * samples it did not reach. With `gate` set the writer waits a short time for the
     * readers to make room. A source can then run at full speed without overwriting a
     * stream that a reader follows sample by sample.
     *
     * Each reader has its own slot, and the readers move independently. The writer waits
     * for the reader furthest behind. A slot holding `kIdle` is not reading and holds
     * nothing up. The wait is bounded, and the writer continues when it expires. A reader
     * that stops without calling readerIdle() stalls nothing.
     */
    enum Reader : std::size_t { kAnalysis = 0, kCapture = 1, kReaderCount = 2 };
    static constexpr std::uint64_t kIdle = ~std::uint64_t{0};

    std::atomic<bool>          gate{false};
    std::atomic<std::uint64_t> readCursor[kReaderCount] = {std::atomic<std::uint64_t>{kIdle}, std::atomic<std::uint64_t>{kIdle}};
    std::condition_variable    room;

    /// Records a reader's position and wakes a waiting writer.
    void readerAt(Reader which, std::uint64_t cursor) {
        readCursor[which].store(cursor, std::memory_order_relaxed);
        room.notify_all();
    }

    /// Marks a reader as stopped. A stopped reader does not hold the writer up.
    void readerIdle(Reader which) {
        readCursor[which].store(kIdle, std::memory_order_relaxed);
        room.notify_all();
    }

    /// How far the furthest-behind active reader is from the write cursor, in samples.
    /// Zero when nothing is reading.
    std::uint64_t readerLag() const {
        std::uint64_t worst = 0;
        for (const auto& c : readCursor) {
            const std::uint64_t at = c.load(std::memory_order_relaxed);
            if (at != kIdle && total > at) {
                worst = std::max(worst, total - at);
            }
        }
        return worst;
    }

    void write(std::span<const std::complex<float>> in, std::size_t cap) {
        std::unique_lock lk(mtx);
        if (gate.load(std::memory_order_relaxed) && cap > 0 && ring.size() == cap) {
            // Wait until the readers lag by at most half the ring. The writer fills one
            // half while the readers work through the other.
            room.wait_for(lk, std::chrono::milliseconds(50), [&] { return readerLag() <= cap / 2; });
        }
        if (ring.size() != cap) {
            ring.assign(cap, std::complex<float>{});
            writePos = 0;
            total    = 0;
            ++gen;
        }
        if (cap == 0) {
            return;
        }
        if (in.size() >= cap) { // only the tail is retained
            std::copy_n(in.end() - static_cast<std::ptrdiff_t>(cap), cap, ring.begin());
            writePos = 0;
            total += in.size();
            return;
        }
        const std::size_t n     = in.size();
        const std::size_t first = std::min(n, cap - writePos);
        std::copy_n(in.begin(), first, ring.begin() + static_cast<std::ptrdiff_t>(writePos));
        if (n > first) {
            std::copy_n(in.begin() + static_cast<std::ptrdiff_t>(first), n - first, ring.begin());
        }
        writePos = (writePos + n) % cap;
        total += in.size();
    }

    /// Copy the latest out.size() samples in chronological order. False if not enough yet.
    bool copyLatest(std::span<std::complex<float>> out) const {
        std::lock_guard   lk(mtx);
        const std::size_t n = out.size();
        if (ring.size() < n || total < n) {
            return false;
        }
        const std::size_t start = (writePos + ring.size() - n) % ring.size();
        const std::size_t first = std::min(n, ring.size() - start);
        std::copy_n(ring.begin() + static_cast<std::ptrdiff_t>(start), first, out.begin());
        if (n > first) {
            std::copy_n(ring.begin(), n - first, out.begin() + static_cast<std::ptrdiff_t>(first));
        }
        return true;
    }

    /// The writer's absolute sample count, the cursor a gapless reader starts from.
    std::uint64_t cursorNow() const {
        std::lock_guard lk(mtx);
        return total;
    }

    /// Samples retained. A gapless reader that falls this far behind has lost data, so
    /// divided by the sample rate it is the reader's real-time budget.
    std::size_t capacity() const {
        std::lock_guard lk(mtx);
        return ring.size();
    }

    /// The ring's allocation generation, `gen`.
    std::uint64_t generation() const {
        std::lock_guard lk(mtx);
        return gen;
    }

    /*!
     * @brief Append every sample written since `cursor` to `out`, gapless.
     *
     * `cursor` is an absolute count in the writer's `total` domain. Pass the value a
     * previous call returned, or cursorNow() to start. The samples written since then are
     * appended in chronological order. Returns the new cursor. A reader more than the ring's
     * capacity behind has lost samples. `lost` receives their number when it is not null.
     * The copy resumes from the oldest sample still held.
     */
    std::uint64_t copySince(std::uint64_t cursor, std::vector<std::complex<float>>& out, std::uint64_t* lost = nullptr) const {
        std::lock_guard lk(mtx);
        if (lost != nullptr) {
            *lost = 0;
        }
        const std::size_t cap = ring.size();
        if (cap == 0) {
            return total;
        }
        if (cursor > total) {
            cursor = total; // resync a cursor from a previous ring generation
        }
        std::uint64_t behind = total - cursor;
        if (behind > cap) {
            if (lost != nullptr) {
                *lost = behind - cap;
            }
            behind = cap;
        }
        const std::size_t n = static_cast<std::size_t>(behind);
        if (n == 0) {
            return total;
        }
        const std::size_t start = (writePos + cap - n) % cap;
        const std::size_t first = std::min(n, cap - start);
        const std::size_t base  = out.size();
        out.resize(base + n);
        std::copy_n(ring.begin() + static_cast<std::ptrdiff_t>(start), first, out.begin() + static_cast<std::ptrdiff_t>(base));
        if (n > first) {
            std::copy_n(ring.begin(), n - first, out.begin() + static_cast<std::ptrdiff_t>(base + first));
        }
        return total;
    }
};

struct IqTap : Block<IqTap> {
    using Description = Doc<R""(@brief Retains the latest `capacity` complex samples for a consumer outside the graph.

The block allocates its own TapState. Read it through `sharedState`. copyLatest() copies
the latest N samples. copySince() follows the stream without gaps.)"">;

    PortIn<std::complex<float>>                                       in;
    Annotated<Size_t, "capacity", Doc<"ring buffer size in samples">> capacity = 262144U;

    GR_MAKE_REFLECTABLE(IqTap, in, capacity);

    // Named `sharedState` because the Block base already has a `state()` lifecycle method.
    std::shared_ptr<TapState> sharedState = std::make_shared<TapState>();

    work::Status processBulk(std::span<const std::complex<float>> input) {
        sharedState->write(input, static_cast<std::size_t>(capacity));
        return work::Status::OK;
    }
};

/// The real-valued counterpart of TapState, for a demodulated audio stream. It offers
/// snapshots alone. It has no generation counter and no writer gate.
struct FloatTapState {
    std::vector<float> ring;
    std::size_t        writePos = 0;
    std::uint64_t      total    = 0;
    mutable std::mutex mtx;

    void write(std::span<const float> in, std::size_t cap) {
        std::lock_guard lk(mtx);
        if (ring.size() != cap) {
            ring.assign(cap, 0.0f);
            writePos = 0;
            total    = 0;
        }
        if (cap == 0) {
            return;
        }
        if (in.size() >= cap) {
            std::copy_n(in.end() - static_cast<std::ptrdiff_t>(cap), cap, ring.begin());
            writePos = 0;
            total += in.size();
            return;
        }
        const std::size_t n     = in.size();
        const std::size_t first = std::min(n, cap - writePos);
        std::copy_n(in.begin(), first, ring.begin() + static_cast<std::ptrdiff_t>(writePos));
        if (n > first) {
            std::copy_n(in.begin() + static_cast<std::ptrdiff_t>(first), n - first, ring.begin());
        }
        writePos = (writePos + n) % cap;
        total += in.size();
    }

    bool copyLatest(std::span<float> out) const {
        std::lock_guard   lk(mtx);
        const std::size_t n = out.size();
        if (ring.size() < n || total < n) {
            return false;
        }
        const std::size_t start = (writePos + ring.size() - n) % ring.size();
        const std::size_t first = std::min(n, ring.size() - start);
        std::copy_n(ring.begin() + static_cast<std::ptrdiff_t>(start), first, out.begin());
        if (n > first) {
            std::copy_n(ring.begin(), n - first, out.begin() + static_cast<std::ptrdiff_t>(first));
        }
        return true;
    }
};

struct FloatTap : Block<FloatTap> {
    using Description = Doc<R""(@brief Retains the latest `capacity` real samples for a consumer outside the graph.)"">;

    PortIn<float>                                                     in;
    Annotated<Size_t, "capacity", Doc<"ring buffer size in samples">> capacity = 16384U;

    GR_MAKE_REFLECTABLE(FloatTap, in, capacity);

    std::shared_ptr<FloatTapState> sharedState = std::make_shared<FloatTapState>();

    work::Status processBulk(std::span<const float> input) {
        sharedState->write(input, static_cast<std::size_t>(capacity));
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_IQ_TAP_HPP
