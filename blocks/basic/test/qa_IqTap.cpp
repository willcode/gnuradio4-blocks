#include <boost/ut.hpp>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/basic/IqTap.hpp>

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdint>
#include <span>
#include <thread>
#include <vector>

namespace gr::blocks::basic::iq_tap_test {

using C = std::complex<float>;

/// Sample `i` carries its own absolute index, so any snapshot can be checked for both
/// content and contiguity without knowing when it was taken.
C value(std::size_t i) { return C{static_cast<float>(i), -static_cast<float>(i)}; }

std::vector<C> seq(std::size_t a, std::size_t b) { // [a, b)
    std::vector<C> v;
    v.reserve(b - a);
    for (std::size_t i = a; i < b; ++i) {
        v.push_back(value(i));
    }
    return v;
}

bool matchesFrom(std::span<const C> got, std::size_t first) {
    for (std::size_t i = 0UZ; i < got.size(); ++i) {
        if (got[i] != value(first + i)) {
            return false;
        }
    }
    return true;
}

/// A snapshot is valid only if it is a run of consecutive indices: a torn copy taken while
/// the writer was mid-chunk would break the step, wrap or not.
bool isConsecutive(std::span<const C> got) {
    for (std::size_t i = 1UZ; i < got.size(); ++i) {
        if (got[i] != value(static_cast<std::size_t>(got[i - 1UZ].real()) + 1UZ)) {
            return false;
        }
    }
    return true;
}

double millisOf(auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); }

struct CountingSource : Block<CountingSource> {
    PortOut<C> out;
    Size_t     n_samples_max = 1024U;
    GR_MAKE_REFLECTABLE(CountingSource, out, n_samples_max);

    std::size_t produced = 0UZ;

    work::Status processBulk(OutputSpanLike auto& outSpan) {
        const std::size_t maxN = static_cast<std::size_t>(n_samples_max);
        if (produced >= maxN) {
            outSpan.publish(0UZ);
            return work::Status::DONE;
        }
        const std::size_t n = std::min(outSpan.size(), maxN - produced);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = value(produced + i);
        }
        outSpan.publish(n);
        produced += n;
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic::iq_tap_test

const boost::ut::suite IqTapTests = [] {
    using namespace boost::ut;
    using namespace gr::blocks::basic;
    using namespace gr::blocks::basic::iq_tap_test;

    "latest-N snapshot, before and after the ring wraps"_test = [] {
        constexpr std::size_t kCap = 8UZ;
        TapState              ts;

        ts.write(seq(0UZ, 5UZ), kCap); // partial fill, no wrap yet
        expect(eq(ts.capacity(), kCap));
        expect(eq(ts.cursorNow(), 5UZ));

        std::vector<C> four(4UZ);
        expect(ts.copyLatest(four));
        expect(matchesFrom(four, 1UZ)) << "the latest 4 of 0..4";

        std::vector<C> full(kCap);
        expect(!ts.copyLatest(full)) << "fewer samples written than asked for";

        ts.write(seq(5UZ, 11UZ), kCap); // writePos wraps the ring end
        expect(eq(ts.cursorNow(), 11UZ));
        expect(ts.copyLatest(full));
        expect(matchesFrom(full, 3UZ)) << "the latest 8 of 0..10, read across the wrap";

        std::vector<C> three(3UZ);
        expect(ts.copyLatest(three));
        expect(matchesFrom(three, 8UZ));

        ts.write(seq(11UZ, 31UZ), kCap); // one chunk larger than the whole ring
        expect(eq(ts.cursorNow(), 31UZ));
        expect(ts.copyLatest(full));
        expect(matchesFrom(full, 23UZ)) << "only the tail of an oversized chunk is retained";
    };

    "generation counter marks the re-allocation a cursor cannot see"_test = [] {
        TapState ts;

        ts.write(seq(0UZ, 100UZ), 64UZ);
        const std::uint64_t gen0 = ts.generation();
        expect(eq(ts.cursorNow(), 100UZ));

        ts.write(seq(100UZ, 110UZ), 64UZ);
        expect(eq(ts.generation(), gen0)) << "same capacity: no re-allocation, no bump";
        expect(eq(ts.cursorNow(), 110UZ));

        const std::uint64_t cursorBefore = ts.cursorNow();

        ts.write(seq(0UZ, 10UZ), 128UZ); // capacity change re-allocates and restarts `total`
        expect(eq(ts.generation(), gen0 + 1UZ));
        expect(eq(ts.cursorNow(), 10UZ)) << "`total` restarted, so the old cursor is now in the future";

        // Cursor arithmetic alone cannot distinguish this from "no new samples": copySince
        // resyncs and reports nothing lost. Only the generation says a gap happened.
        std::vector<C> out;
        std::uint64_t  lost = 0UZ;
        expect(eq(ts.copySince(cursorBefore, out, &lost), 10UZ));
        expect(eq(out.size(), 0UZ));
        expect(eq(lost, 0UZ));
        expect(neq(ts.generation(), gen0));
    };

    "copySince follows the stream gaplessly and counts what it lost"_test = [] {
        constexpr std::size_t kCap = 64UZ;
        TapState              ts;

        ts.write(seq(0UZ, 100UZ), kCap); // 100 written into a 64-sample ring

        std::vector<C> out;
        std::uint64_t  lost   = 0UZ;
        std::uint64_t  cursor = ts.copySince(0UZ, out, &lost);
        expect(eq(cursor, 100UZ));
        expect(eq(out.size(), kCap)) << "a reader more than a ring behind resumes at the oldest held";
        expect(eq(lost, 36UZ));
        expect(matchesFrom(out, 36UZ));

        out.clear();
        ts.write(seq(100UZ, 110UZ), kCap);
        cursor = ts.copySince(cursor, out, &lost);
        expect(eq(cursor, 110UZ));
        expect(eq(out.size(), 10UZ));
        expect(eq(lost, 0UZ)) << "kept up this time";
        expect(matchesFrom(out, 100UZ));

        out.clear();
        cursor = ts.copySince(cursor, out, &lost);
        expect(eq(out.size(), 0UZ)) << "nothing new since the last call";
        expect(eq(cursor, 110UZ));
    };

    "readerLag tracks the furthest-behind active reader, per slot"_test = [] {
        TapState ts;
        ts.write(seq(0UZ, 100UZ), 64UZ); // total = 100

        expect(eq(ts.readerLag(), 0UZ)) << "nothing reading";

        ts.readerAt(TapState::kAnalysis, 90UZ);
        expect(eq(ts.readerLag(), 10UZ));

        ts.readerAt(TapState::kCapture, 40UZ);
        expect(eq(ts.readerLag(), 60UZ)) << "the writer waits on whichever is further behind";

        ts.readerAt(TapState::kAnalysis, 20UZ);
        expect(eq(ts.readerLag(), 80UZ)) << "the slots move independently";

        ts.readerIdle(TapState::kAnalysis);
        expect(eq(ts.readerLag(), 60UZ)) << "an idle slot holds nothing up";

        ts.readerIdle(TapState::kCapture);
        expect(eq(ts.readerLag(), 0UZ));
    };

    "the gate makes the writer wait for a lagging reader, but never indefinitely"_test = [] {
        constexpr std::size_t kCap = 64UZ;
        TapState              ts;
        ts.write(seq(0UZ, 100UZ), kCap); // allocate the ring; total = 100

        // Ungated, a lagging reader is ignored: the writer never waits.
        ts.readerAt(TapState::kCapture, 0UZ);
        expect(gt(ts.readerLag(), kCap / 2UZ));
        auto start = std::chrono::steady_clock::now();
        ts.write(seq(100UZ, 110UZ), kCap);
        const auto ungated = std::chrono::steady_clock::now() - start;
        expect(lt(millisOf(ungated), 40.0)) << "ungated writer must not wait";

        // Gated with the same lagging reader, the writer waits out its bounded timeout and
        // then proceeds regardless, so a reader that stops without saying so stalls nothing.
        ts.gate.store(true);
        expect(gt(ts.readerLag(), kCap / 2UZ));
        start = std::chrono::steady_clock::now();
        ts.write(seq(110UZ, 120UZ), kCap);
        const auto stalled = std::chrono::steady_clock::now() - start;
        expect(ge(millisOf(stalled), 40.0)) << "gated writer must wait on a lagging reader";
        expect(lt(millisOf(stalled), 1000.0)) << "and the wait must be bounded";
        expect(eq(ts.cursorNow(), 120UZ)) << "the write still lands";

        // A reader that catches up, or goes idle, releases the writer immediately.
        ts.readerAt(TapState::kCapture, ts.cursorNow());
        expect(eq(ts.readerLag(), 0UZ));
        start = std::chrono::steady_clock::now();
        ts.write(seq(120UZ, 130UZ), kCap);
        const auto caughtUp = std::chrono::steady_clock::now() - start;
        expect(lt(millisOf(caughtUp), 40.0));

        ts.readerAt(TapState::kCapture, 0UZ); // far behind again, then idle
        ts.readerIdle(TapState::kCapture);
        start = std::chrono::steady_clock::now();
        ts.write(seq(130UZ, 140UZ), kCap);
        const auto idled = std::chrono::steady_clock::now() - start;
        expect(lt(millisOf(idled), 40.0)) << "an idle reader does not pin the writer";
        expect(eq(ts.cursorNow(), 140UZ));
    };

    "a lagging reader releases the gated writer as soon as it advances"_test = [] {
        constexpr std::size_t kCap = 64UZ;
        TapState              ts;
        ts.write(seq(0UZ, 100UZ), kCap);
        ts.readerAt(TapState::kCapture, 0UZ);
        ts.gate.store(true);

        std::thread writer([&] { ts.write(seq(100UZ, 110UZ), kCap); });
        ts.readerAt(TapState::kCapture, 100UZ); // notifies `room`
        writer.join();

        expect(eq(ts.cursorNow(), 110UZ));
        std::vector<C> out(10UZ);
        expect(ts.copyLatest(out));
        expect(matchesFrom(out, 100UZ));
    };

    "concurrent writes and snapshots never tear"_test = [] {
        constexpr std::size_t kCap       = 4096UZ;
        constexpr std::size_t kChunk     = 100UZ;
        constexpr std::size_t kChunks    = 2000UZ;
        constexpr std::size_t kSnapshot  = 64UZ;
        constexpr std::size_t kSnapshots = 2000UZ;

        TapState ts;

        std::thread writer([&] {
            for (std::size_t c = 0UZ; c < kChunks; ++c) {
                ts.write(seq(c * kChunk, (c + 1UZ) * kChunk), kCap);
            }
        });

        std::size_t torn  = 0UZ;
        std::size_t taken = 0UZ;
        for (std::size_t s = 0UZ; s < kSnapshots; ++s) { // a fixed count, so the loop terminates
            std::vector<C> out(kSnapshot);
            if (ts.copyLatest(out)) {
                ++taken;
                if (!isConsecutive(out)) {
                    ++torn;
                }
            }
        }
        writer.join();

        expect(eq(torn, 0UZ)) << "every snapshot must be a consecutive run";
        expect(gt(taken, 0UZ)) << "the reader must have seen something";
        expect(eq(ts.cursorNow(), kChunks * kChunk));

        std::vector<C> tail(kSnapshot);
        expect(ts.copyLatest(tail));
        expect(matchesFrom(tail, kChunks * kChunk - kSnapshot));
    };

    "IqTap retains the tail of what the graph pushed through it"_test = [] {
        constexpr gr::Size_t kSamples = 4096U;
        constexpr gr::Size_t kCap     = 1024U;

        gr::Graph flow;
        auto&     src      = flow.emplaceBlock<CountingSource>({{"n_samples_max", kSamples}});
        auto&     tap      = flow.emplaceBlock<IqTap>({{"capacity", kCap}});
        auto      tapState = tap.sharedState; // the handle a consumer outside the graph holds
        expect(flow.connect<"out", "in">(src, tap).has_value());

        gr::scheduler::Simple<> sched;
        expect(sched.exchange(std::move(flow)).has_value());
        expect(sched.runAndWait().has_value());

        expect(eq(tapState->capacity(), static_cast<std::size_t>(kCap)));
        expect(eq(tapState->cursorNow(), static_cast<std::uint64_t>(kSamples)));

        std::vector<C> out(kCap);
        expect(tapState->copyLatest(out));
        expect(matchesFrom(out, static_cast<std::size_t>(kSamples - kCap)));
    };

    "FloatTapState keeps the latest N across the wrap"_test = [] {
        constexpr std::size_t kCap = 8UZ;
        FloatTapState         ts;

        std::vector<float> in0{0.f, 1.f, 2.f, 3.f, 4.f};
        ts.write(in0, kCap);

        std::vector<float> four(4UZ);
        expect(ts.copyLatest(four));
        expect(four == std::vector<float>{1.f, 2.f, 3.f, 4.f});

        std::vector<float> full(kCap);
        expect(!ts.copyLatest(full));

        std::vector<float> in1{5.f, 6.f, 7.f, 8.f, 9.f, 10.f};
        ts.write(in1, kCap); // wraps
        expect(ts.copyLatest(full));
        expect(full == std::vector<float>{3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f});
    };
};

int main() { /* not needed for UT */ }
