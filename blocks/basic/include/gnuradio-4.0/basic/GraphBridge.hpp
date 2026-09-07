#ifndef GNURADIO_GRAPH_BRIDGE_HPP
#define GNURADIO_GRAPH_BRIDGE_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <source_location>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include <gnuradio-4.0/basic/NamespaceCompatibility.hpp>

namespace gr::blocks::basic {
using namespace gr;

GR_REGISTER_BLOCK(gr::blocks::basic::BridgeSink)
GR_REGISTER_BLOCK(gr::blocks::basic::BridgeSource)

/// What a bridge counts, readable through the registry by name and without a handle on either block:
/// the samples the producer side has dropped since the last configure(), the ring's capacity, the
/// samples waiting in it, and whether the producer has latched end-of-stream.
struct BridgeCounters {
    std::uint64_t overflows = 0;
    std::size_t   capacity  = 0;
    std::size_t   available = 0;
    bool          eos       = false;
};

/*!
 * @brief Bounded ring joining two independently scheduled flow graphs.
 *
 * A front end that must keep running continuously cannot share a scheduler with a
 * consumer chain that is torn down and rebuilt whenever its configuration changes.
 * Two graphs joined only by this ring keep the producer alive across every rebuild of
 * the consumer, and the ring being bounded is also the end-to-end latency bound.
 *
 * Lock-guarded. The owner holds the `shared_ptr<BridgeState>` and injects the same
 * instance into both blocks after `emplaceBlock`, keeping the blocks movable and the
 * overflow counter readable from outside them.
 */
struct BridgeState {
    std::vector<std::complex<float>> ring;
    std::size_t                      cap   = 0; // ring capacity in samples
    std::size_t                      head  = 0; // index of the oldest valid sample
    std::size_t                      count = 0; // number of valid samples
    bool                             eos   = false;
    /// Overflow policy, chosen by what feeds the producer graph. A real-time source cannot
    /// be backpressured, so the sink drops the oldest samples and never blocks the producer.
    /// A file (or any rate-free) source can be: the sink then accepts only what fits and
    /// leaves the rest in the upstream edge buffer, which stalls the source without ever
    /// blocking inside processBulk -- that would trip the scheduler's stuck-block watchdog.
    bool                       backpressure = false;
    std::atomic<std::uint64_t> overflows{0};
    mutable std::mutex         mtx;
    std::condition_variable    cv;

    /// (Re)size the ring and empty it. Called on every producer-graph build. `block`
    /// selects backpressure over drop-oldest.
    void configure(std::size_t capacity, bool block = false) {
        std::lock_guard lk(mtx);
        ring.assign(capacity, std::complex<float>{});
        cap          = capacity;
        head         = 0;
        count        = 0;
        eos          = false;
        backpressure = block;
        overflows.store(0);
        cv.notify_all();
    }

    /// Empty the ring, keeping its capacity. Called on every consumer-graph (re)build.
    void clear() {
        {
            std::lock_guard lk(mtx);
            head  = 0;
            count = 0;
            eos   = false;
        }
        cv.notify_all();
    }

    /// A consistent reading of the counters, taken under the lock.
    [[nodiscard]] BridgeCounters counters() const {
        std::lock_guard lk(mtx);
        return {overflows.load(std::memory_order_relaxed), cap, count, eos};
    }

    /// Latch end-of-stream: the consumer graph drains what is left, then emits EoS.
    void setEos() {
        {
            std::lock_guard lk(mtx);
            eos = true;
        }
        cv.notify_all();
    }

    /// Append exactly `n` samples at the tail, wrapping the ring end. The caller holds
    /// the lock and has ensured the space.
    void appendLocked(std::span<const std::complex<float>> in, std::size_t n) {
        const std::size_t tail  = (head + count) % cap;
        const std::size_t first = std::min(n, cap - tail);
        std::copy_n(in.begin(), first, ring.begin() + static_cast<std::ptrdiff_t>(tail));
        if (n > first) {
            std::copy_n(in.begin() + static_cast<std::ptrdiff_t>(first), n - first, ring.begin());
        }
        count += n;
    }

    /// Producer side, drop-oldest: append `in`, dropping the oldest samples on overflow so
    /// the producer never blocks. Always accepts all of `in`.
    void push(std::span<const std::complex<float>> in) {
        {
            std::lock_guard lk(mtx);
            if (cap == 0) {
                return;
            }
            const std::size_t n = in.size();
            if (n >= cap) { // incoming exceeds the whole ring: keep only its tail
                overflows.fetch_add(count + (n - cap), std::memory_order_relaxed);
                std::copy_n(in.end() - static_cast<std::ptrdiff_t>(cap), cap, ring.begin());
                head  = 0;
                count = cap;
            } else {
                const std::size_t space = cap - count;
                if (n > space) { // drop just enough oldest to fit
                    const std::size_t drop = n - space;
                    head                   = (head + drop) % cap;
                    count -= drop;
                    overflows.fetch_add(drop, std::memory_order_relaxed);
                }
                appendLocked(in, n);
            }
        }
        cv.notify_all();
    }

    /// Producer side, backpressure: accept only what currently fits and return how much
    /// that was, so the caller can consume just that many from the upstream edge. Never
    /// drops. On a full ring it waits briefly -- bounded well under the scheduler's
    /// stuck-block watchdog -- for the consumer to drain; without that wait the scheduler
    /// re-invokes a zero-progress BridgeSink in a busy loop whenever the consumer graph is
    /// the bottleneck.
    std::size_t accept(std::span<const std::complex<float>> in) {
        std::size_t took = 0;
        {
            std::unique_lock lk(mtx);
            if (cap != 0) {
                if (count == cap) {
                    cv.wait_for(lk, std::chrono::milliseconds(10), [this] { return count < cap || eos; });
                }
                took = std::min(in.size(), cap - count);
                if (took > 0) {
                    appendLocked(in.first(took), took);
                }
            }
        }
        if (took > 0) {
            cv.notify_all();
        }
        return took;
    }

    /// Sink one span under the configured policy; returns how many samples to consume from
    /// the upstream edge -- all of `in` for drop-oldest, only what fit for backpressure.
    std::size_t sink(std::span<const std::complex<float>> in) {
        if (backpressure) {
            return accept(in);
        }
        push(in);
        return in.size();
    }

    /// Consumer side: wait up to 100 ms for data, then pop up to `out.size()` samples in
    /// chronological order. Returns how many were popped and reports the current EoS latch
    /// in `eosOut`, so the source can emit DONE once the ring has drained.
    std::size_t popOrWait(std::span<std::complex<float>> out, bool& eosOut) {
        std::size_t n = 0;
        {
            std::unique_lock lk(mtx);
            cv.wait_for(lk, std::chrono::milliseconds(100), [this] { return count > 0 || eos; });
            eosOut = eos;
            if (count == 0 || cap == 0 || out.empty()) {
                return 0;
            }
            n                       = std::min(out.size(), count);
            const std::size_t first = std::min(n, cap - head);
            std::copy_n(ring.begin() + static_cast<std::ptrdiff_t>(head), first, out.begin());
            if (n > first) {
                std::copy_n(ring.begin(), n - first, out.begin() + static_cast<std::ptrdiff_t>(first));
            }
            head = (head + n) % cap;
            count -= n;
        }
        cv.notify_all(); // wake a BridgeSink waiting on a full ring (backpressure mode)
        return n;
    }
};

/*!
 * @brief Bridge rings published by name, so a loaded graph can join one without a C++ handle.
 *
 * A settings map cannot carry a shared pointer, so the two halves of an exported flow graph have
 * no way to name the same ring. The application creates the ring, publishes it here under a name,
 * and both files name that string in their `bridge_name` setting. This is `DataSinkRegistry` for
 * bridges: one global instance, a duplicate name refused, and the counters readable from outside
 * the blocks.
 */
class BridgeRegistry {
    mutable std::mutex                                  _mutex;
    std::map<std::string, std::shared_ptr<BridgeState>> _bridges;

public:
    void publish(std::string_view name, std::shared_ptr<BridgeState> state, std::source_location location = std::source_location::current()) {
        if (name.empty()) {
            throw gr::exception("Failed to publish a bridge under an empty name.", location);
        }
        if (state == nullptr) {
            throw gr::exception(std::format("Failed to publish bridge `{}`. The state is null.", name), location);
        }
        std::lock_guard lg{_mutex};
        if (!_bridges.try_emplace(std::string{name}, std::move(state)).second) {
            throw gr::exception(std::format("Failed to publish bridge `{}`. A bridge of that name is already published.", name), location);
        }
    }

    [[nodiscard]] std::shared_ptr<BridgeState> find(std::string_view name) const {
        std::lock_guard lg{_mutex};
        const auto      it = _bridges.find(std::string{name});
        return it == _bridges.end() ? nullptr : it->second;
    }

    /// `find`, refusing by name rather than returning null: what a block joining by name needs.
    [[nodiscard]] std::shared_ptr<BridgeState> require(std::string_view name, std::string_view blockName, std::source_location location = std::source_location::current()) const {
        std::shared_ptr<BridgeState> state = find(name);
        if (state == nullptr) {
            throw gr::exception(std::format("Block `{}` cannot join bridge `{}`. No bridge of that name is published.", blockName, name), location);
        }
        return state;
    }

    void withdraw(std::string_view name) {
        std::lock_guard lg{_mutex};
        _bridges.erase(std::string{name});
    }

    [[nodiscard]] std::optional<BridgeCounters> counters(std::string_view name) const {
        const std::shared_ptr<BridgeState> state = find(name);
        return state == nullptr ? std::nullopt : std::optional<BridgeCounters>{state->counters()};
    }
};

__attribute__((visibility("default"))) inline BridgeRegistry& globalBridgeRegistry() {
    static BridgeRegistry instance;
    return instance;
}

namespace detail {

// Called from settingsChanged rather than start(): a loaded graph applies its settings at init, and
// a BridgeSource with no ring returns DONE on its first work call, before start() would have run.
inline void rebindBridge(std::shared_ptr<BridgeState>& bridge, const property_map& oldSettings, std::string_view bridgeName, std::string_view blockName) {
    if (!oldSettings.contains("bridge_name")) {
        return;
    }
    if (oldSettings.at("bridge_name").value_or(std::string{}) == bridgeName) {
        return;
    }
    bridge = bridgeName.empty() ? nullptr : globalBridgeRegistry().require(bridgeName, blockName);
}

} // namespace detail

struct BridgeSink : Block<BridgeSink> {
    using Description = Doc<R""(@brief producer-graph terminal of a GraphBridge: pushes its input into the shared BridgeState ring.

Assign the shared `BridgeState` after `emplaceBlock`, or set `bridge_name` to a name the
application has published in the global bridge registry, which is how a graph loaded from a
file reaches a ring no settings map could carry. Without either the block consumes and
discards its input.)"">;

    PortIn<std::complex<float>> in;

    Annotated<std::string, "bridge name", Visible> bridge_name = "";

    GR_MAKE_REFLECTABLE(BridgeSink, in, bridge_name);

    std::shared_ptr<BridgeState> bridge;

    void settingsChanged(const property_map& oldSettings, const property_map& /*newSettings*/) { detail::rebindBridge(bridge, oldSettings, bridge_name.value, this->name.value); }

    void stop() noexcept {
        if (!bridge_name.value.empty()) {
            bridge.reset();
        }
    }

    // InputSpanLike rather than the auto-consume-all span form, so backpressure mode can
    // consume only what fit -- that partial consume is what stalls the upstream source.
    work::Status processBulk(InputSpanLike auto& inSpan) {
        std::size_t took = inSpan.size();
        if (bridge) {
            took = bridge->sink(std::span<const std::complex<float>>(inSpan.data(), inSpan.size()));
        }
        std::ignore = inSpan.consume(took);
        return work::Status::OK;
    }
};

struct BridgeSource : Block<BridgeSource> {
    using Description = Doc<R""(@brief consumer-graph source of a GraphBridge: pops from the shared BridgeState ring.

Waits on a condition variable rather than polling, so an idle bridge never spins the
scheduler. Emits DONE once the ring has drained and the producer side has latched EoS.
Takes its ring the same two ways `BridgeSink` does: assigned after `emplaceBlock`, or named
through `bridge_name` in the global bridge registry.)"">;

    PortOut<std::complex<float>> out;

    Annotated<std::string, "bridge name", Visible> bridge_name = "";

    GR_MAKE_REFLECTABLE(BridgeSource, out, bridge_name);

    std::shared_ptr<BridgeState> bridge;

    void settingsChanged(const property_map& oldSettings, const property_map& /*newSettings*/) { detail::rebindBridge(bridge, oldSettings, bridge_name.value, this->name.value); }

    void stop() noexcept {
        if (!bridge_name.value.empty()) {
            bridge.reset();
        }
    }

    work::Status processBulk(OutputSpanLike auto& outSpan) {
        if (!bridge) {
            outSpan.publish(0);
            return work::Status::DONE;
        }
        bool              eos = false;
        const std::size_t n   = bridge->popOrWait(std::span<std::complex<float>>(outSpan.data(), outSpan.size()), eos);
        outSpan.publish(n);
        if (n == 0 && eos) { // drained and the producer finished: propagate EoS downstream
            return work::Status::DONE;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_GRAPH_BRIDGE_HPP
