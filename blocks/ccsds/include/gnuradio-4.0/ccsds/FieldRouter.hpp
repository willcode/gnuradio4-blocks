#ifndef GNURADIO_CCSDS_FIELD_ROUTER_HPP
#define GNURADIO_CCSDS_FIELD_ROUTER_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/ccsds/RecordHelpers.hpp>

/**
 * @brief `FieldRouter`, routing a record by a CCSDS field the decode already wrote to metadata.
 *
 * One block, two profiles (`"apid"` or `"virtual_channel"`), N output ports plus `other` rather than one port with
 * a metadata filter — `basic::Selector` already carries `std::vector<PortOut<T, Async>>`, the routing is visible
 * in the flowgraph, and the record is parsed once with the branch a port assignment rather than a predicate
 * re-evaluated per candidate. The mitigation for a fixed port set is that the decode has already written the
 * field, so a graph that wants the metadata-filter shape does not need this block at all.
 */
namespace gr::blocks::ccsds {

GR_REGISTER_BLOCK(gr::blocks::ccsds::FieldRouter)

/*!
@brief Routes a `DataSet<std::uint8_t>` by `ccsds_apid` or `ccsds_vcid`, one output port per named value.

A record whose value equals `values[i]` goes to `outputs[i]`; anything else, including a record whose key is
absent or holds something other than a `gr::Size_t`, goes to `other`, counted and never assigned a value: a value
of the wrong type reads as absent, because a record that has crossed a network can carry anything under a key, and
inventing a routing value for one would send it somewhere on the strength of a guess. With `other` unconnected an
unmatched record is a counted drop, which is the one place this block lets a record vanish, and only because the
graph's author declined the port that was offered.
*/
struct FieldRouter : Block<FieldRouter> {
    using Description = Doc<"Routes a CCSDS record by APID or virtual channel identifier, one output port per named value plus an `other` catch-all">;

    PortIn<DataSet<std::uint8_t>, Async>               in;
    std::vector<PortOut<DataSet<std::uint8_t>, Async>> outputs;
    PortOut<DataSet<std::uint8_t>, Async, Optional>    other;

    Annotated<std::string, "field", Doc<"'apid' (reads ccsds_apid) or 'virtual_channel' (reads ccsds_vcid); required">, Visible>                                                            field{};
    Annotated<std::vector<gr::Size_t>, "values", Doc<"the field value routed to each output port, in order; required, refused empty or with duplicates or an out-of-width value">, Visible> values{};

    GR_MAKE_REFLECTABLE(FieldRouter, in, outputs, other, field, values);

    std::vector<std::uint64_t> nRouted{};
    std::uint64_t              nOther      = 0ULL;
    std::uint64_t              nMissingKey = 0ULL;

    bool                      _configured = false;
    const char*               _key        = "ccsds_apid";
    std::vector<std::int32_t> _valueToPort{}; // indexed by field value; -1 means no matching output port

    void settingsChanged(const property_map&, const property_map&) { rebuild(); }
    void start() { rebuild(); }

    void rebuild() {
        _configured = false;
        if (field.value != "apid" && field.value != "virtual_channel") {
            throw gr::exception(std::format("field must be 'apid' or 'virtual_channel' and has no default, got '{}'", field.value));
        }
        if (values.value.empty()) {
            throw gr::exception("values is required and has no default: an empty routing table names no output");
        }
        const gr::Size_t maxValue = field.value == "apid" ? 2047U : 63U;
        for (std::size_t i = 0UZ; i < values.value.size(); ++i) {
            if (values.value[i] > maxValue) {
                throw gr::exception(std::format("values[{}] = {} exceeds the width of '{}' (0 to {})", i, values.value[i], field.value, maxValue));
            }
            for (std::size_t j = 0UZ; j < i; ++j) {
                if (values.value[i] == values.value[j]) {
                    throw gr::exception(std::format("values[{}] and values[{}] are both {}: duplicate routing targets", j, i, values.value[i]));
                }
            }
        }

        _key = field.value == "apid" ? "ccsds_apid" : "ccsds_vcid";
        outputs.resize(values.value.size());
        nRouted.assign(values.value.size(), 0ULL);
        _valueToPort.assign(std::size_t{maxValue} + 1UZ, -1);
        for (std::size_t i = 0UZ; i < values.value.size(); ++i) {
            _valueToPort[values.value[i]] = static_cast<std::int32_t>(i);
        }
        _configured = true;
    }

    void stop() {
        std::string report;
        const auto  append = [&report](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(report), "{}{}: {}", report.empty() ? "" : ", ", label, count);
            }
        };
        for (std::size_t i = 0UZ; i < nRouted.size(); ++i) {
            append(std::format("outputs[{}]", i), nRouted[i]);
        }
        append("other", nOther);
        append("missing key", nMissingKey);
        if (!report.empty()) {
            std::println(stderr, "gr::blocks::ccsds::FieldRouter '{}': {}", this->name, report);
        }
    }

    template<gr::OutputSpanLike TOutSpan>
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, std::span<TOutSpan>& outs, OutputSpanLike auto& otherSpan) {
        if (!_configured) {
            std::ignore = inSpan.consume(0UZ);
            for (auto& outSpan : outs) {
                outSpan.publish(0UZ);
            }
            otherSpan.publish(0UZ);
            return work::Status::ERROR;
        }

        const bool               otherConnected = otherSpan.isConnected;
        std::vector<std::size_t> madePerPort(outs.size(), 0UZ);
        std::size_t              madeOther = 0UZ;
        std::size_t              consumed  = 0UZ;

        while (consumed < inSpan.size()) {
            const DataSet<std::uint8_t>&    record = inSpan[consumed];
            const std::optional<gr::Size_t> value  = detail::readSize(detail::metaOf(record), _key);

            std::optional<std::size_t> portIndex;
            const bool                 missingKey = !value.has_value();
            if (!missingKey && *value < _valueToPort.size() && _valueToPort[*value] >= 0) {
                portIndex = static_cast<std::size_t>(_valueToPort[*value]);
            }

            // The room test comes before every count, so a record held back for want of room is counted once, on
            // the call that routes it, and not again on each call that could not.
            if (portIndex.has_value()) {
                if (madePerPort[*portIndex] >= outs[*portIndex].size()) {
                    break; // no room on the port this record is bound for; retry next call
                }
                outs[*portIndex][madePerPort[*portIndex]] = record;
                ++madePerPort[*portIndex];
                ++nRouted[*portIndex];
            } else {
                if (otherConnected) {
                    if (madeOther >= otherSpan.size()) {
                        break;
                    }
                    otherSpan[madeOther] = record;
                    ++madeOther;
                }
                if (missingKey) {
                    ++nMissingKey;
                } else {
                    ++nOther;
                }
            }
            ++consumed;
        }

        std::ignore = inSpan.consume(consumed);
        for (std::size_t i = 0UZ; i < outs.size(); ++i) {
            outs[i].publish(madePerPort[i]);
        }
        otherSpan.publish(otherConnected ? madeOther : 0UZ);

        if (consumed == 0UZ) {
            const bool anyRoom = otherSpan.size() > 0UZ || std::ranges::any_of(outs, [](const auto& outSpan) { return outSpan.size() > 0UZ; });
            return anyRoom ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::ccsds

#endif // GNURADIO_CCSDS_FIELD_ROUTER_HPP
