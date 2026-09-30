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
 * A transmit-side carrier-sense gate for records. In the record domain, the logic of a tagged-stream gate reduces
 * to one boolean and one counter. A record is atomic. There is no packet in flight to track and no length tag to
 * read.
 *
 * `sense` carries one carrier-sense observation per item, not per sample. Non-zero is busy and zero is clear. The gate
 * keeps only the newest item it has seen. It consumes the whole span every call and reads none of the values it
 * discards. `sense` is a required port, not `gr::Optional`. An unconnected sense port has two possible readings, and
 * both are silently wrong. Always clear gives a gate that never gates. Always busy gives a permanent stall. The
 * scheduler still accepts a graph with `sense` unconnected. The gate then stays busy and releases no record.
 */
namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::CsmaGate)

/*!
@brief Holds `DataSet<std::uint8_t>` records while `sense` says the channel is busy and releases them when it is clear.

Each call reads the newest item of `sense` once. That consultation either holds or grants `burst_records` releases.
A hold consumes nothing on `in` and publishes nothing. The call then returns `work::Status::INSUFFICIENT_INPUT_ITEMS`
and does not sleep. The scheduler's back-off sets the pace. In a grant, the first release is the consultation. The
remaining `burst_records - 1` pass without reading `sense` again, even if it turns busy meanwhile. This count is the
back-to-back exception, and its meaning does not depend on the shape of the input stream. A call makes at most one
grant. A channel that stays clear therefore cannot empty an unbounded backlog in one call, whatever `burst_records`
is. A full backlog drains over as many calls as it needs, each releasing up to the burst.

`nRecordsHeld` counts a record once, on the first call that leaves it waiting in the input buffer. Further calls it
waits through do not count it again. The number is records delayed, not calls waited through. `nBusyCalls` counts the
calls that `sense` itself gated. Read together, the two tell a busy medium from a stalled graph. A CSMA gate
otherwise looks the same in both cases.

The gate is busy until a sense item shows a clear channel, at start and after any settings change. Holding while the
channel is unconfirmed never transmits over another station. A settings change or a `stop()` discards any release
allowance. `nAllowanceDiscarded` counts the discarded releases.
*/
struct CsmaGate : Block<CsmaGate> {
    using Description = Doc<"CSMA gate that holds DataSet<uint8_t> records while the newest 'sense' item says busy and releases burst_records at a time when it says clear">;

    PortIn<DataSet<std::uint8_t>, Async>  in;
    PortIn<std::uint8_t, Async>           sense;
    PortOut<DataSet<std::uint8_t>, Async> out;

    Annotated<gr::Size_t, "burst_records", Doc<"records released per consultation of sense, one or more">, Visible> burst_records = 1U;

    GR_MAKE_REFLECTABLE(CsmaGate, in, sense, out, burst_records);

    bool        _busy             = true; ///< the newest sense value seen, busy until a clear item arrives
    std::size_t _releaseRemaining = 0UZ;  ///< records still owed from the last consultation's grant, 0 meaning the next release needs one
    std::size_t _recordsWaiting   = 0UZ;  ///< records the previous call left in the input buffer, so only new waiters are counted

    // Plain members, read by the owning thread and by QA, and reported once at stop().
    std::uint64_t nRecordsPassed      = 0ULL; ///< records published on `out`
    std::uint64_t nRecordsHeld        = 0ULL; ///< records counted once each, on the first call that left them waiting
    std::uint64_t nSenseTransitions   = 0ULL; ///< times the newest sense value differed from the one before it
    std::uint64_t nBusyCalls          = 0ULL; ///< calls that held because sense itself said busy
    std::uint64_t nAllowanceDiscarded = 0ULL; ///< releases granted and not used, discarded by a settings change or at stop()

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (burst_records.value == 0U) {
            throw gr::exception("burst_records is records released per consultation of sense and must be at least 1, got 0");
        }
        // A re-configured gate keeps neither a release allowance nor a sense reading from the previous
        // configuration. The channel is busy again until an item says otherwise.
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
        if (senseSpan.size() > 0UZ) { // one read of the last item, and every other item in the span is consumed unread
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
                if (grantedThisCall) { // this call's one grant is used up, and the rest of the backlog waits for the next call
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
        // Only the records this call left waiting that were not waiting when it began. A record delayed over twenty
        // calls is one delayed record. Counting it twenty times would report a backlog twenty deep.
        const std::size_t waiting = inSpan.size() - consumed;
        if (waiting > _recordsWaiting) {
            nRecordsHeld += waiting - _recordsWaiting;
        }
        _recordsWaiting = waiting;

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            if (busyHold) {
                return work::Status::INSUFFICIENT_INPUT_ITEMS; // held without sleeping, and the scheduler's back-off sets the pace
            }
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Discards the releases still owed from the last consultation and counts them.
    ///
    /// They withheld no record, since they are an unused allowance and not a queued value. An allowance left at the end
    /// of a run is a decision the channel never received, and the count records it.
    void discardAllowance() {
        nAllowanceDiscarded += _releaseRemaining;
        _releaseRemaining = 0UZ;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_DIGITAL_CSMA_GATE_HPP
