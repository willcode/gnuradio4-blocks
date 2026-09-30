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

/// The counters of a bridge. The registry returns them by name, without a handle on either block.
/// `overflows` counts the samples the producer side dropped since the last configure(). `capacity`
/// is the ring's size, `available` the samples waiting in it. `eos` is set once the producer has
/// latched end-of-stream.
struct BridgeCounters {
    std::uint64_t overflows = 0;
    std::size_t   capacity  = 0;
    std::size_t   available = 0;
    bool          eos       = false;
};

/*!
 * @brief Bounded ring joining two independently scheduled flow graphs.
 *
 * The producer graph and the consumer graph run on separate schedulers. The consumer
 * graph can be torn down and rebuilt after a configuration change. The producer graph
 * keeps running through every rebuild. The ring's capacity bounds the end-to-end latency.
 *
 * A mutex guards the state. The owner holds the `shared_ptr<BridgeState>` and assigns the
 * same instance to both blocks after `emplaceBlock`. The blocks stay movable, and the
 * overflow counter stays readable from outside them.
 */
struct BridgeState {
    std::vector<std::complex<float>> ring;
    std::size_t                      cap   = 0; // ring capacity in samples
    std::size_t                      head  = 0; // index of the oldest valid sample
    std::size_t                      count = 0; // number of valid samples
    bool                             eos   = false;
    /// Overflow policy, chosen by the source of the producer graph. A real-time source
    /// cannot take backpressure. With `false` the sink drops the oldest samples and does not
    /// block the producer. A file source, or any source without a fixed rate, can take
    /// backpressure. With `true` the sink accepts only what fits. The rest stays in the
    /// upstream edge buffer and stalls the source. The sink does not block inside
    /// processBulk, where a block would trip the scheduler's stuck-block watchdog.
    bool                       backpressure = false;
    std::atomic<std::uint64_t> overflows{0};
    mutable std::mutex         mtx;
    std::condition_variable    cv;

    /// Sizes the ring and empties it. Call it on each build of the producer graph. `block`
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

    /// Empties the ring and keeps its capacity. Call it on each build of the consumer graph.
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

    /// Latches end-of-stream. The consumer side drains the ring and then emits EoS.
    void setEos() {
        {
            std::lock_guard lk(mtx);
            eos = true;
        }
        cv.notify_all();
    }

    /// Appends exactly `n` samples at the tail and wraps at the ring end. The caller holds
    /// the lock and has checked the space.
    void appendLocked(std::span<const std::complex<float>> in, std::size_t n) {
        const std::size_t tail  = (head + count) % cap;
        const std::size_t first = std::min(n, cap - tail);
        std::copy_n(in.begin(), first, ring.begin() + static_cast<std::ptrdiff_t>(tail));
        if (n > first) {
            std::copy_n(in.begin() + static_cast<std::ptrdiff_t>(first), n - first, ring.begin());
        }
        count += n;
    }

    /// Appends all of `in` under the drop-oldest policy. On overflow the oldest samples are
    /// dropped and counted. The producer does not block.
    void push(std::span<const std::complex<float>> in) {
        {
            std::lock_guard lk(mtx);
            if (cap == 0) {
                return;
            }
            const std::size_t n = in.size();
            if (n >= cap) { // the input fills the whole ring, keep only its tail
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

    /// Appends what fits under the backpressure policy and returns the count taken. The
    /// caller consumes that many samples from the upstream edge. No sample is dropped. On a
    /// full ring the call waits up to 10 ms for the consumer to drain it. The wait is well
    /// under the scheduler's stuck-block watchdog. Without it, the scheduler calls a
    /// BridgeSink that made no progress in a busy loop while the consumer graph is slower.
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

    /// Takes one span under the configured policy. Returns the samples to consume from the
    /// upstream edge. That is all of `in` for drop-oldest and what fit for backpressure.
    std::size_t sink(std::span<const std::complex<float>> in) {
        if (backpressure) {
            return accept(in);
        }
        push(in);
        return in.size();
    }

    /// Waits up to 100 ms for data, then pops up to `out.size()` samples in chronological
    /// order. Returns the count popped. `eosOut` receives the EoS latch. The source emits
    /// DONE once the ring has drained after EoS.
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
 * A settings map cannot carry a shared pointer. The two halves of an exported flow graph need
 * another way to name the same ring. The application creates the ring and publishes it here
 * under a name. Both graph files name that string in their `bridge_name` setting. The registry
 * follows `DataSinkRegistry`. It has one global instance and refuses a duplicate name. The
 * counters are readable from outside the blocks.
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

    /// `find` for a block that joins by name. An unknown name throws an error that names it.
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

// Called from settingsChanged and not from start(). A loaded graph applies its settings at init.
// A BridgeSource without a ring returns DONE on its first work call, before start() would run.
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
    using Description = Doc<R""(@brief Pushes its input into the shared BridgeState ring of a GraphBridge, at the end of the producer graph.

Assign the shared `BridgeState` after `emplaceBlock`, or set `bridge_name` to a name the
application has published in the global bridge registry. A graph loaded from a file reaches
its ring by that name, since a settings map cannot carry the ring. Without either the block
consumes and discards its input.)"">;

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

    // An InputSpanLike span lets backpressure mode consume only what fit. The partial
    // consume stalls the upstream source.
    /// Answers `INSUFFICIENT_OUTPUT_ITEMS` when a backpressured ring takes none of a non-empty
    /// input, and `OK` otherwise.
    work::Status processBulk(InputSpanLike auto& inSpan) {
        std::size_t took = inSpan.size();
        if (bridge) {
            took = bridge->sink(std::span<const std::complex<float>>(inSpan.data(), inSpan.size()));
        }
        std::ignore = inSpan.consume(took);
        return took == 0UZ && inSpan.size() > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::OK;
    }
};

struct BridgeSource : Block<BridgeSource> {
    using Description = Doc<R""(@brief Pops samples from the shared BridgeState ring of a GraphBridge, at the start of the consumer graph.

The block waits on a condition variable and does not poll. An idle bridge does not spin the
scheduler. The block emits DONE once the ring has drained and the producer side has latched
EoS. It gets its ring in the two ways `BridgeSink` does. The ring is assigned after
`emplaceBlock` or named through `bridge_name` in the global bridge registry.)"">;

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

    /// Answers `OK` when it published, `DONE` once the ring has drained after end-of-stream, and
    /// `INSUFFICIENT_INPUT_ITEMS` when the wait ends on an empty ring. The scheduler keeps calling a
    /// block that answers that status. The framework's zero-progress report counts only an `OK`.
    work::Status processBulk(OutputSpanLike auto& outSpan) {
        if (!bridge) {
            outSpan.publish(0);
            return work::Status::DONE;
        }
        bool              eos = false;
        const std::size_t n   = bridge->popOrWait(std::span<std::complex<float>>(outSpan.data(), outSpan.size()), eos);
        outSpan.publish(n);
        if (n == 0 && eos) { // ring drained after the producer finished, propagate EoS
            return work::Status::DONE;
        }
        return n == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_GRAPH_BRIDGE_HPP
