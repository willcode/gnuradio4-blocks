#ifndef GNURADIO_DIGITAL_CSMA_GATE_HPP
#define GNURADIO_DIGITAL_CSMA_GATE_HPP

#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <string_view>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

/**
 * A transmit-side carrier-sense gate, in the record domain a tagged-stream gate's whole logic collapses to one
 * boolean and one counter: a record is atomic, so there is no packet-in-flight to track and no length tag to read.
 *
 * `sense` carries one carrier-sense observation per item, not one sample: non-zero is busy, zero is clear, and the
 * gate keeps only the newest item it has seen, consuming the whole span every call and reading none of the values
 * it discards. `sense` is required rather than `gr::Optional`, because an unconnected sense port has two readings
 * and both are silently wrong — always clear (a gate that never gates) or always busy (a permanent stall) — and
 * refusing to build one is the only answer that cannot be quietly mistaken for the other.
 */
namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::CsmaGate)

/*!
@brief Holds `DataSet<std::uint8_t>` records while `sense` says the channel is busy, releasing them when it is clear.

Each consultation of `sense` (its newest item, read once per call) either holds — nothing is consumed on `in`,
nothing is published, and the call returns `work::Status::INSUFFICIENT_INPUT_ITEMS` rather than sleeping, so the
scheduler's own back-off decides the pace — or grants `burst_records` releases, of which the first is the
consultation and the remaining `burst_records - 1` pass without reading `sense` again, even if it turns busy in the
meantime: the back-to-back exception, spelled as a count whose meaning does not depend on the shape of the input
stream. At most one such grant is made per call, which is what keeps a channel that stays clear from emptying an
unbounded backlog in one call regardless of `burst_records`; a full backlog drains over as many calls as it takes,
each releasing up to the burst.

`nRecordsHeld` counts a record once, on the first call that leaves it waiting in the input buffer instead of passing
it, and never again however many further calls it waits through: the number is records delayed, not record-calls
spent waiting. `nBusyCalls` counts the calls `sense` itself gated. Read together they are how an operator tells a
busy medium from a stalled graph, which a CSMA gate is otherwise indistinguishable from.

The gate is busy until a sense item proves otherwise, at start and again after any settings change: an unconfirmed
channel is the reading that cannot transmit over somebody else. A release allowance outlives neither, and what a
settings change or a `stop()` throws away is counted in `nAllowanceDiscarded` rather than vanishing.
*/
struct CsmaGate : Block<CsmaGate> {
    using Description = Doc<"CSMA gate: holds DataSet<uint8_t> records while the newest 'sense' item says busy, releasing burst_records at a time when it says clear">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortIn<std::uint8_t, Async>           sense;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "burst_records", Doc<"records released per consultation of sense; the next burst_records - 1 pass without re-reading it. Zero is refused">, Visible> burst_records = 1U;

    GR_MAKE_REFLECTABLE(CsmaGate, in, sense, out, burst_records);

    bool        _busy             = true; ///< the newest sense value seen; busy until proven otherwise
    std::size_t _releaseRemaining = 0UZ;  ///< records still owed from the last consultation's grant, 0 meaning the next release needs one
    std::size_t _recordsWaiting   = 0UZ;  ///< records the previous call left in the input buffer, so only new waiters are counted

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecordsPassed      = 0ULL; ///< records published on `out`
    std::uint64_t nRecordsHeld        = 0ULL; ///< records counted once each, on the first call that left them waiting
    std::uint64_t nSenseTransitions   = 0ULL; ///< times the newest sense value differed from the one before it
    std::uint64_t nBusyCalls          = 0ULL; ///< calls that held because sense itself said busy
    std::uint64_t nAllowanceDiscarded = 0ULL; ///< releases granted but never spent, thrown away by a settings change or at stop()

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (burst_records.value == 0U) {
            throw gr::exception("burst_records is records released per consultation of sense and must be at least 1, got 0");
        }
        // a re-configured gate carries neither a release allowance earned under the previous configuration nor a
        // sense reading taken under it: the channel is busy again until an item says otherwise
        discardAllowance();
        _busy = true;
    }

    void stop() {
        discardAllowance();
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        append("records passed", nRecordsPassed);
        append("records held", nRecordsHeld);
        append("sense transitions", nSenseTransitions);
        append("busy calls", nBusyCalls);
        append("allowance discarded", nAllowanceDiscarded);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::digital::CsmaGate '{}': {}", this->name, report);
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, InputSpanLike auto& senseSpan, OutputSpanLike auto& outSpan) {
        if (senseSpan.size() > 0UZ) { // one read of the last item; every other item in the span is consumed and ignored
            const bool newBusy = senseSpan[senseSpan.size() - 1UZ] != 0U;
            if (newBusy != _busy) {
                ++nSenseTransitions;
            }
            _busy = newBusy;
        }
        std::ignore = senseSpan.consume(senseSpan.size());

        std::size_t consumed        = 0UZ;
        std::size_t made            = 0UZ;
        bool        busyHold        = false; ///< the hold was sense itself, this call
        bool        grantedThisCall = false;
        while (consumed < inSpan.size() && made < outSpan.size()) {
            if (_releaseRemaining == 0UZ) {
                if (grantedThisCall) { // this call's one grant is spent; the rest of the backlog waits for the next
                    break;
                }
                if (_busy) {
                    busyHold = true;
                    break;
                }
                _releaseRemaining = static_cast<std::size_t>(burst_records.value);
                grantedThisCall   = true;
            }
            outSpan[made] = inSpan[consumed];
            ++made;
            ++consumed;
            --_releaseRemaining;
        }

        nRecordsPassed += made;
        if (busyHold) {
            ++nBusyCalls;
        }
        // only the records this call left waiting that were not already waiting when it began: a record delayed over
        // twenty calls is one delayed record, and counting it twenty times would say the backlog is twenty deep
        const std::size_t waiting = inSpan.size() - consumed;
        if (waiting > _recordsWaiting) {
            nRecordsHeld += waiting - _recordsWaiting;
        }
        _recordsWaiting = waiting;

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            if (busyHold) {
                return work::Status::INSUFFICIENT_INPUT_ITEMS; // held, not slept: the scheduler's own back-off sets the pace
            }
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Throws away the releases still owed from the last consultation, counting them. No record was withheld
    /// by them — they are a spent allowance rather than a queued value — but an allowance a run ends on is a decision
    /// the channel never got, and a number is what says so.
    void discardAllowance() {
        nAllowanceDiscarded += _releaseRemaining;
        _releaseRemaining = 0UZ;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_CSMA_GATE_HPP
