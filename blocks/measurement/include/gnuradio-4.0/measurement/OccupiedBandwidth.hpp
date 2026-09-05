#ifndef GNURADIO_MEASUREMENT_OCCUPIED_BANDWIDTH_HPP
#define GNURADIO_MEASUREMENT_OCCUPIED_BANDWIDTH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/MeasurementRecord.hpp>
#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::measurement {

GR_REGISTER_BLOCK(gr::blocks::measurement::OccupiedBandwidth)

/**
 * @brief The band holding a stated fraction of a density record's total power, with its edges.
 *
 * The walk is the cumulative sum from both ends: the lower edge is where `(1 - fraction) / 2` of the total has
 * accumulated from below, the upper edge where the same has accumulated from above, and the bandwidth is the
 * distance between them. That symmetric definition is the one the occupied-bandwidth figure in a regulatory mask
 * means, and it does not assume the signal is centered.
 *
 * The block emits one record per input record and also publishes to a slot a graph can poll — bandwidth, lower edge
 * and upper edge — for consumers that want a reading without consuming records. The reading is whole or absent
 * rather than partial, so there is no coverage to report: `nRecords()` is zero until the first record has been
 * measured, and that zero is what says the three readings have never been written.
 */
struct OccupiedBandwidth : Block<OccupiedBandwidth, NoTagPropagation> {
    using Description = Doc<"Occupied bandwidth of a spectral density record: the band holding a stated fraction of the total power, with its edges, as a record and as a pollable reading">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<double, "fraction", Visible, Doc<"the share of total power the band must hold, in (0, 1]">> fraction = 0.99;

    GR_MAKE_REFLECTABLE(OccupiedBandwidth, in, out, fraction);

    gr::measurement::MeasurementSlot<3UZ> _slot{}; ///< bandwidth, lower edge, upper edge — all in hertz
    std::uint64_t                         _records = 0ULL;

    void settingsChanged(const property_map& /*old*/, const property_map& /*new*/) {
        if (!(fraction > 0.0) || !(fraction <= 1.0)) {
            throw gr::exception(std::format("fraction is the share of power the band holds and must lie in (0, 1], got {}", fraction.value));
        }
    }

    /// @brief A fresh run reads nothing until it has measured something: the slot is republished, not merely counted
    /// down, or the readers would go on returning the last run's numbers.
    void start() {
        _records = 0ULL;
        _slot.publish({0.0, 0.0, 0.0}, 0ULL);
    }

    /// @brief The most recent occupied bandwidth in hertz, or zero before the first record.
    [[nodiscard]] double bandwidth() const noexcept { return _slot.read().first[0UZ]; }

    /// @brief The band's lower edge in hertz, on the record's own axis.
    [[nodiscard]] double lowerEdge() const noexcept { return _slot.read().first[1UZ]; }

    /// @brief The band's upper edge in hertz, on the record's own axis.
    [[nodiscard]] double upperEdge() const noexcept { return _slot.read().first[2UZ]; }

    /// @brief The records measured so far; zero means the readings above have never been written.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _slot.read().second; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made = 0UZ;
        for (std::size_t r = 0UZ; r < inSpan.size() && made < outSpan.size(); ++r, ++made) {
            outSpan[made] = measure(inSpan[r]);
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(made);
        return work::Status::OK;
    }

private:
    [[nodiscard]] DataSet<float> measure(const DataSet<float>& record) {
        const std::span<const float> values(record.signal_values);
        float                        lower = 0.f;
        float                        upper = 0.f;
        float                        width = 0.f;

        if (!values.empty() && !record.axis_values.empty() && record.axis_values[0UZ].size() == values.size()) {
            const std::span<const float> axis(record.axis_values[0UZ]);
            double                       total = 0.;
            for (const float value : values) {
                total += static_cast<double>(std::max(value, 0.f));
            }
            if (total > 0.) {
                const double tail = 0.5 * (1. - fraction.value) * total;

                double      running = 0.;
                std::size_t low     = 0UZ;
                while (low + 1UZ < values.size() && running + static_cast<double>(std::max(values[low], 0.f)) <= tail) {
                    running += static_cast<double>(std::max(values[low], 0.f));
                    ++low;
                }
                running          = 0.;
                std::size_t high = values.size() - 1UZ;
                while (high > 0UZ && running + static_cast<double>(std::max(values[high], 0.f)) <= tail) {
                    running += static_cast<double>(std::max(values[high], 0.f));
                    --high;
                }

                lower = axis[low];
                upper = axis[high];
                width = std::abs(upper - lower);
            }
        }

        ++_records;
        _slot.publish({static_cast<double>(width), static_cast<double>(lower), static_cast<double>(upper)}, _records);

        // The source record's own facts carry through, so the measurement stays placed on the stream it was made of.
        property_map carried;
        if (!record.meta_information.empty()) {
            carried = record.meta_information[0UZ];
        }
        carried.insert_or_assign(property_map::key_type("fraction"), pmt::Value(fraction.value));

        const float         sampleRate  = carriedFloat(carried, "sample_rate");
        const std::uint64_t sampleStart = carriedIndex(carried, "sample_start");

        const std::array<gr::measurement::ScalarChannel, 3UZ> channels{{
            {"occupied_bandwidth", "Bandwidth", "Hz", width},
            {"lower_edge", "Frequency", "Hz", lower},
            {"upper_edge", "Frequency", "Hz", upper},
        }};
        DataSet<float>                                        ds = gr::measurement::makeScalarRecord(std::span<const gr::measurement::ScalarChannel>(channels), sampleRate, sampleStart, std::move(carried));
        ds.timestamp                                             = record.timestamp;
        return ds;
    }

    /// @brief A float the source record stated, or a non-finite value when it stated none — which is what tells
    /// `makeScalarRecord` to leave the key out rather than write a zero a consumer would divide by.
    [[nodiscard]] static float carriedFloat(const property_map& map, std::string_view key) {
        if (const auto it = map.find(property_map::key_type(key)); it != map.end()) {
            if (const auto* value = it->second.get_if<float>()) {
                return *value;
            }
        }
        return std::numeric_limits<float>::quiet_NaN();
    }

    /// @brief A stream index the source record stated, or zero when it stated none.
    [[nodiscard]] static std::uint64_t carriedIndex(const property_map& map, std::string_view key) {
        if (const auto it = map.find(property_map::key_type(key)); it != map.end()) {
            if (const auto* value = it->second.get_if<std::uint64_t>()) {
                return *value;
            }
        }
        return 0ULL;
    }
};

} // namespace gr::blocks::measurement

#endif // GNURADIO_MEASUREMENT_OCCUPIED_BANDWIDTH_HPP
