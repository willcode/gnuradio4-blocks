#ifndef GNURADIO_MEASUREMENT_OCCUPIED_BANDWIDTH_HPP
#define GNURADIO_MEASUREMENT_OCCUPIED_BANDWIDTH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
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
 * @brief Measures the band that holds a stated fraction of a density record's total power, and its edges.
 *
 * The block sums the record's power from each end. The lower edge is the first bin at which the sum from below would
 * exceed `(1 - fraction) / 2` of the total. The upper edge is the same bin from above. The bandwidth is the distance
 * between the two edges. Each tail outside the band holds at most `(1 - fraction) / 2` of the total. A negative bin
 * counts as zero. A bin that is NaN or positive infinity makes the three readings of its record NaN.
 *
 * The block emits one scalar record per input record. It also publishes the bandwidth and both edges to a slot.
 * `bandwidth()`, `lowerEdge()`, `upperEdge()` and `nRecords()` read the slot from any thread, and a read returns the
 * values of one record. `nRecords()` is zero until the first record is measured.
 *
 * The block measures with the `fraction` it last accepted, starting at the default. A refused `fraction` stays in its
 * member and changes no reading. The block stops every input tag.
 */
struct OccupiedBandwidth : Block<OccupiedBandwidth, NoTagPropagation> {
    using Description = Doc<"Measures the occupied bandwidth of a spectral density record: the band that holds a stated fraction of the total power. The block emits the bandwidth and both band edges as one record per input record, and keeps the latest values for polling. The block stops every input tag.">;

    PortIn<DataSet<float>, Async>  in;
    PortOut<DataSet<float>, Async> out;

    Annotated<double, "fraction", Visible, Doc<"share of total power inside the band, in (0, 1]">> fraction = 0.99;

    GR_MAKE_REFLECTABLE(OccupiedBandwidth, in, out, fraction);

    gr::measurement::MeasurementSlot<3UZ> _slot{}; ///< bandwidth, lower edge and upper edge in hertz
    std::uint64_t                         _records  = 0ULL;
    double                                _fraction = fraction.value; ///< `fraction` as last accepted

    void settingsChanged(const property_map& /*old*/, const property_map& /*new*/) {
        if (!(fraction > 0.0) || !(fraction <= 1.0)) {
            throw gr::exception(std::format("fraction is the share of power the band holds and must lie in (0, 1], got {}", fraction.value));
        }
        _fraction = fraction.value;
    }

    /// @brief Clears the record count and publishes zero readings for a new run.
    void start() {
        _records = 0ULL;
        _slot.publish({0.0, 0.0, 0.0}, 0ULL);
    }

    /// @brief The latest occupied bandwidth in hertz, or zero before the first record.
    [[nodiscard]] double bandwidth() const noexcept { return _slot.read().first[0UZ]; }

    /// @brief The latest lower edge in hertz, on the record's axis.
    [[nodiscard]] double lowerEdge() const noexcept { return _slot.read().first[1UZ]; }

    /// @brief The latest upper edge in hertz, on the record's axis.
    [[nodiscard]] double upperEdge() const noexcept { return _slot.read().first[2UZ]; }

    /// @brief Records measured in this run. Zero means no reading is published yet.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _slot.read().second; }

    /// @brief Measures input records while the output has room. A call that moves nothing returns
    /// `INSUFFICIENT_INPUT_ITEMS` for an empty input and `INSUFFICIENT_OUTPUT_ITEMS` for a full output.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made = 0UZ;
        for (std::size_t r = 0UZ; r < inSpan.size() && made < outSpan.size(); ++r, ++made) {
            outSpan[made] = measure(inSpan[r]);
        }
        outSpan.publish(made);
        std::ignore = inSpan.consume(made);
        if (made > 0UZ) {
            return work::Status::OK;
        }
        return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
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
                total += static_cast<double>(std::max(value, 0.f)); // NaN stays NaN
            }
            if (!std::isfinite(total)) {
                lower = std::numeric_limits<float>::quiet_NaN();
                upper = lower;
                width = lower;
            } else if (total > 0.) {
                const double tail = 0.5 * (1. - _fraction) * total;

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

        // The output carries the source record's metadata, its `sample_start` and its timestamp.
        property_map carried;
        if (!record.meta_information.empty()) {
            carried = record.meta_information[0UZ];
        }
        carried.insert_or_assign(property_map::key_type("fraction"), pmt::Value(_fraction));

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

    /// @brief The float under `key`, or NaN when the map holds no float there. `makeScalarRecord` leaves out a
    /// non-finite rate.
    [[nodiscard]] static float carriedFloat(const property_map& map, std::string_view key) {
        if (const auto it = map.find(property_map::key_type(key)); it != map.end()) {
            if (const auto* value = it->second.get_if<float>()) {
                return *value;
            }
        }
        return std::numeric_limits<float>::quiet_NaN();
    }

    /// @brief The stream index under `key`, or zero when the map holds none.
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
