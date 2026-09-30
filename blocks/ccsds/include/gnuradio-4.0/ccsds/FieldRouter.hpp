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
 * @brief `FieldRouter` routes a record by a CCSDS field that the decoder has written to metadata.
 *
 * One block serves two profiles, `"apid"` and `"virtual_channel"`. It has N output ports plus `other`, not one port
 * with a metadata filter. The ports are a `std::vector<PortOut<T, Async>>`. The routing is visible in the flowgraph.
 * The block reads the field once, and the branch is a port assignment. It evaluates no predicate per candidate. The
 * port set is fixed.
 */
namespace gr::blocks::ccsds {

GR_REGISTER_BLOCK(gr::blocks::ccsds::FieldRouter)

/*!
@brief Routes a `DataSet<std::uint8_t>` by `ccsds_apid` or `ccsds_vcid`, one output port per named value.

A record whose value equals `values[i]` goes to `outputs[i]`. Any other record goes to `other`. A record whose key is
absent or holds a type other than `gr::Size_t` is counted in `nMissingKey`. Any other unmatched record is counted in
`nOther`. A value of the wrong type reads as absent. A record that has crossed a network can carry anything under a key,
and the block assigns it no guessed value. With `other` unconnected, the block drops an unmatched record and still
counts it. `values` is refused when it is empty, holds duplicates or holds a value wider than the field.
*/
struct FieldRouter : Block<FieldRouter> {
    using Description = Doc<"Routes a CCSDS record by APID or virtual channel identifier, one output port per named value plus an `other` catch-all">;

    PortIn<DataSet<std::uint8_t>, Async>               in;
    std::vector<PortOut<DataSet<std::uint8_t>, Async>> outputs;
    PortOut<DataSet<std::uint8_t>, Async, Optional>    other;

    Annotated<std::string, "field", Doc<"required field, 'apid' (ccsds_apid) or 'virtual_channel' (ccsds_vcid)">, Visible> field{};
    Annotated<std::vector<gr::Size_t>, "values", Doc<"required field value of each output port, in port order">, Visible>  values{};

    GR_MAKE_REFLECTABLE(FieldRouter, in, outputs, other, field, values);

    std::vector<std::uint64_t> nRouted{};
    std::uint64_t              nOther      = 0ULL;
    std::uint64_t              nMissingKey = 0ULL;

    bool                      _configured = false;
    const char*               _key        = "ccsds_apid";
    std::vector<std::int32_t> _valueToPort{}; // indexed by field value, -1 for no matching output port

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

            // The room test comes before every count. A record held back for lack of room is counted once, on the call
            // that routes it.
            if (portIndex.has_value()) {
                if (madePerPort[*portIndex] >= outs[*portIndex].size()) {
                    break; // no room on this record's port, retry next call
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
