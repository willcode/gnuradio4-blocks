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
 * A consumer that is not part of the flow graph -- a display, an analyzer -- wants the
 * latest N samples whenever it happens to ask, not every sample as it arrives. Keeping
 * only the latest N here costs the graph one chunked copy per work call and lets the
 * consumer run at its own rate rather than the sample rate.
 *
 * Lock-guarded; the state lives on the heap behind a shared_ptr so the block stays
 * movable and the owner can read the ring without touching the block object.
 */
struct TapState {
    std::vector<std::complex<float>> ring;
    std::size_t                      writePos = 0;
    std::uint64_t                    total    = 0;
    /// Bumped every time the ring is re-allocated, which restarts `total` from zero. A
    /// gapless reader holding a cursor from before the bump has an unknowable gap in front
    /// of it, and comparing this against what it started with is the only way it can tell.
    std::uint64_t      gen = 0;
    mutable std::mutex mtx;

    /*! Where each gapless reader has read up to, and whether the writer waits for them.
     *
     * A tap is ordinarily a place to look rather than a queue to drain: the writer never
     * waits, and a reader too slow to keep up loses the samples it did not reach. Gated,
     * the writer instead waits -- briefly -- for the readers to make room, which is what
     * lets a source run flat out without overwriting a stream that is being read through
     * rather than sampled.
     *
     * There is a slot per reader because the tap has more than one and they move
     * independently, and the writer has to wait for whichever is further behind. A slot
     * reading `kIdle` is not reading and holds nothing up. The wait is bounded and the
     * writer proceeds regardless when it expires, so a reader that stops without saying so
     * stalls nothing.
     */
    enum Reader : std::size_t { kAnalysis = 0, kCapture = 1, kReaderCount = 2 };
    static constexpr std::uint64_t kIdle = ~std::uint64_t{0};

    std::atomic<bool>          gate{false};
    std::atomic<std::uint64_t> readCursor[kReaderCount] = {std::atomic<std::uint64_t>{kIdle}, std::atomic<std::uint64_t>{kIdle}};
    std::condition_variable    room;

    /// Say where a reader has reached, and let a waiting writer on.
    void readerAt(Reader which, std::uint64_t cursor) {
        readCursor[which].store(cursor, std::memory_order_relaxed);
        room.notify_all();
    }

    /// Say that a reader has stopped reading, so it holds the writer up no longer.
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
            // Half the ring, so the readers have half to work through while the
            // writer fills the other.
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

    /// The writer's current absolute sample count -- the cursor a gapless reader starts from.
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

    /// The ring's current allocation generation; see `gen`.
    std::uint64_t generation() const {
        std::lock_guard lk(mtx);
        return gen;
    }

    /*!
     * @brief Append every sample written since `cursor` to `out`, gapless.
     *
     * `cursor` is an absolute count in the writer's `total` domain: pass what a previous
     * call returned (or cursorNow() to start), and the samples appended since come back in
     * chronological order. Returns the new cursor. A reader that falls more than the ring's
     * capacity behind has lost samples: `lost` (may be null) reports how many, and
     * the copy resumes from the oldest sample still held.
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
            cursor = total; // a cursor from a previous stream generation: resync
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
    using Description = Doc<R""(@brief retains the latest `capacity` complex samples for a consumer outside the graph.

Assign nothing: the block allocates its own TapState. Read it through `sharedState`, either
by snapshotting the latest N with copyLatest() or by following the stream gaplessly with
copySince().)"">;

    PortIn<std::complex<float>>                                       in;
    Annotated<Size_t, "capacity", Doc<"ring buffer size in samples">> capacity = 262144U;

    GR_MAKE_REFLECTABLE(IqTap, in, capacity);

    // `sharedState` rather than `state`: the Block base already has a `state()` lifecycle method.
    std::shared_ptr<TapState> sharedState = std::make_shared<TapState>();

    work::Status processBulk(std::span<const std::complex<float>> input) {
        sharedState->write(input, static_cast<std::size_t>(capacity));
        return work::Status::OK;
    }
};

/// Real-valued counterpart of TapState, for a demodulated audio stream. Snapshot-only:
/// there is no generation counter and no writer gate, because nothing follows an audio
/// tap gaplessly.
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
    using Description = Doc<R""(@brief retains the latest `capacity` real samples for a consumer outside the graph.)"">;

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
