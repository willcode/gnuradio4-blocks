#ifndef GNURADIO_IMCHARTMONITOR_HPP
#define GNURADIO_IMCHARTMONITOR_HPP

#include "gnuradio-4.0/BlockRegistry.hpp"
#include <algorithm>
#include <mutex>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/HistoryBuffer.hpp>
#include <gnuradio-4.0/algorithm/ImChart.hpp>
#include <gnuradio-4.0/algorithm/dataset/DataSetUtils.hpp>

#include <format>
#include <gnuradio-4.0/meta/formatter.hpp>

namespace gr::blocks::testing {

GR_REGISTER_BLOCK("gr::blocks::testing::ImChartMonitor", gr::blocks::testing::ImChartMonitor, [T], [float]);
GR_REGISTER_BLOCK("gr::blocks::testing::ConsoleDebugSink", gr::blocks::testing::ConsoleDebugSink, [T], [float]);

template<typename T, bool drawAsynchronously = true>
requires(std::is_arithmetic_v<T> || gr::DataSetLike<T>)
struct ImChartMonitor : Block<ImChartMonitor<T, drawAsynchronously>, std::conditional_t<drawAsynchronously, Drawable<UICategory::Content, "console">, void>> {
    using ClockSourceType               = std::chrono::system_clock;
    using TimePoint                     = ClockSourceType::time_point;
    constexpr static bool isDataSetLike = gr::DataSetLike<T>;
    // optional shortening
    template<typename U, typename... Arguments>
    using A = Annotated<U, Arguments...>;

    PortIn<T> in;

    struct Settings {
        A<float, Visible, Doc<"sample rate">, Unit<"Hz">, Limits<Min<float(0)>>> sample_rate          = 1000.0f;
        A<std::string, Visible, Doc<"signal name">>                              signal_name          = "unknown signal";
        A<gr::Size_t, Doc<"sample history length">>                              n_history            = isDataSetLike ? 3ULL : 1000ULL;
        A<gr::Size_t, Doc<"tag history length">>                                 n_tag_history        = 20ULL;
        A<bool, Doc<"view reset">>                                               reset_view           = true;
        A<bool, Doc<"data graph">>                                               plot_graph           = true;
        A<bool, Doc<"timing display">>                                           plot_timing          = false;
        A<bool, Doc<"merged timing display">>                                    plot_merged_tags     = false;
        A<std::uint64_t, Unit<"ms">, Doc<"timeout">>                             timeout_ms           = 40ULL;
        A<gr::Size_t, Doc<"chart width">>                                        chart_width          = 130U;
        A<gr::Size_t, Doc<"chart height">>                                       chart_height         = 28U;
        A<bool, Doc<"mountain range view">>                                      mountain_range       = false;
        A<gr::Size_t, Doc<"mountain range horizontal offset">>                   mountain_x_offset    = 2U;
        A<gr::Size_t, Doc<"mountain range vertical offset">>                     mountain_y_offset    = 2U;
        A<gr::Size_t, Doc<"mountain range color (max = rotating)">>              mountain_color_index = std::numeric_limits<gr::Size_t>::max();

        GR_MAKE_REFLECTABLE(Settings, sample_rate, signal_name, n_history, n_tag_history, reset_view, plot_graph, plot_timing, plot_merged_tags, timeout_ms, chart_width, chart_height, mountain_range, mountain_x_offset, mountain_y_offset, mountain_color_index);
    } settings;

    A<int, Doc<"signal index (-1 = all)">> signal_index = -1;

    GR_MAKE_REFLECTABLE(ImChartMonitor, in, settings);

    HistoryBuffer<T> _historyBufferX{settings.n_history};
    HistoryBuffer<T> _historyBufferY{settings.n_history};
    HistoryBuffer<T> _historyBufferTags{settings.n_history};
    std::size_t      _n_samples_total = 0UZ;

    struct TagInfo {
        TimePoint      timestamp;
        property_map   map;
        std::size_t    index;
        std::ptrdiff_t relIndex = 0;
        bool           merged   = false;
    };
    HistoryBuffer<TagInfo> _historyTags{settings.n_tag_history};
    std::size_t            _tagIndex = 0UZ;

    std::source_location _location   = std::source_location::current();
    TimePoint            _lastUpdate = ClockSourceType::now();
    std::mutex           _drawMutex;

    void applySettings(const Settings& s) {
        if (s.n_history.changed()) {
            _historyBufferX.resize(s.n_history);
            _historyBufferY.resize(s.n_history);
            _historyBufferTags.resize(s.n_history);
        }

        if (s.n_tag_history.changed()) {
            _historyTags.resize(s.n_tag_history);
        }

        if constexpr (drawAsynchronously && std::is_arithmetic_v<T>) {
            in.max_samples = std::max(1UZ, static_cast<std::size_t>(2.f * s.sample_rate * static_cast<float>(s.timeout_ms) / 1000.f));
        }
    }

    void start() {
        std::println("started sink {} aka. '{}'", this->unique_name, this->name);
        if constexpr (drawAsynchronously && std::is_arithmetic_v<T>) {
            in.max_samples = std::max(1UZ, static_cast<std::size_t>(2.f * settings.sample_rate * static_cast<float>(settings.timeout_ms) / 1000.f));
        }
    }

    void stop() { std::println("stopped sink {} aka. '{}'", this->unique_name, this->name); }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inData) noexcept {
        if (inData.empty()) {
            return work::Status::OK;
        }

        { // guarded write to local storage section
            const std::lock_guard lock(_drawMutex);
            const TimePoint       nowStamp = ClockSourceType::now();

            // collect per-sample tag positions from the input span
            auto spanTags  = inData.tags();
            using TagEntry = std::pair<std::size_t, property_map>;
            std::vector<TagEntry> localTags;
            for (const auto& [relIndex, tagMapRef] : spanTags) {
                if (relIndex >= 0 && static_cast<std::size_t>(relIndex) < inData.size()) {
                    localTags.emplace_back(static_cast<std::size_t>(relIndex), tagMapRef.get());
                }
            }

            std::size_t tagCursor = 0;
            for (std::size_t i = 0; i < inData.size(); ++i) {
                _n_samples_total++;

                if constexpr (std::is_arithmetic_v<T>) {
                    const T Ts = T(1.0f) / T(settings.sample_rate);
                    _historyBufferX.push_back(static_cast<T>(_n_samples_total) * Ts);
                }
                _historyBufferY.push_back(inData[i]);

                if (tagCursor < localTags.size() && localTags[tagCursor].first == i) {
                    if constexpr (std::is_arithmetic_v<T>) {
                        _historyBufferTags.push_back(inData[i]);
                    }
                    _historyTags.push_back(TagInfo{.timestamp = nowStamp, .map = localTags[tagCursor].second, .index = _tagIndex++});
                    ++tagCursor;
                } else {
                    if constexpr (std::is_floating_point_v<T>) {
                        _historyBufferTags.push_back(std::numeric_limits<T>::quiet_NaN());
                    } else if constexpr (std::is_arithmetic_v<T>) {
                        _historyBufferTags.push_back(std::numeric_limits<T>::lowest());
                    }
                }
            }

            if (settings.plot_merged_tags) {
                property_map mergedMap;
                for (const auto& [relIndex, tagMapRef] : inData.tags()) {
                    if (relIndex == 0) {
                        mergedMap.merge(property_map(tagMapRef.get()));
                    }
                }
                if (!mergedMap.empty()) {
                    _historyTags.push_back(TagInfo{.timestamp = nowStamp, .map = std::move(mergedMap), .index = _tagIndex++, .relIndex = 0, .merged = true});
                }
            }
        } // end lifetime of '_drawMutex' write lock-guard

        if constexpr (!drawAsynchronously) {
            draw();
        }

        return work::Status::OK;
    }

    work::Status draw(const property_map& config = {}, std::source_location location = std::source_location::current()) noexcept {
        const std::lock_guard lock(_drawMutex);
        _location = location;

        if (settings.timeout_ms > 0U) {
            const TimePoint nowStamp = ClockSourceType::now();
            if (nowStamp < (_lastUpdate + std::chrono::milliseconds(settings.timeout_ms))) {
                return lifecycle::isShuttingDown(this->state()) ? work::Status::DONE : work::Status::OK;
            }
            _lastUpdate = nowStamp;
        }

        const bool shouldResetView = config.contains("reset_view");
        if (settings.plot_graph) {
            plotGraph(shouldResetView);
        }
        if (settings.plot_timing) {
            plotTiming();
        }

        return lifecycle::isShuttingDown(this->state()) ? work::Status::DONE : work::Status::OK;
    }

    void plotGraph(bool shouldResetView) {
        if constexpr (std::is_arithmetic_v<T>) {
            const auto [xMin, xMax] = std::ranges::minmax_element(_historyBufferX);
            const auto [yMin, yMax] = std::ranges::minmax_element(_historyBufferY);
            if (_historyBufferX.empty() || *xMin == *xMax || *yMin == *yMax) {
                return; // buffer or axes' ranges are empty -> skip drawing
            }

            if (shouldResetView) {
                gr::graphs::resetView();
            }
            std::println("\nPlot Graph for '{}' - #samples total: {}", settings.signal_name, _n_samples_total);

            auto adjustRange = [](T min, T max) {
                min            = std::min(min, T(0));
                max            = std::max(max, T(0));
                const T margin = (max - min) * static_cast<T>(0.2);
                return std::pair<double, double>{min - margin, max + margin};
            };

            auto chart = gr::graphs::ImChart<std::dynamic_extent, std::dynamic_extent>({{*xMin, *xMax}, adjustRange(*yMin, *yMax)}, static_cast<std::size_t>(settings.chart_width), static_cast<std::size_t>(settings.chart_height));
            chart.draw(_historyBufferX, _historyBufferY, settings.signal_name);
            chart.draw<gr::graphs::Style::Marker>(_historyBufferX, _historyBufferTags, "Tags");
            chart.draw();
        } else if constexpr (isDataSetLike) {
            if (_historyBufferY.empty()) {
                return;
            }

            const auto& latestDataSet   = _historyBufferY[0];
            const bool  hasMultiSignals = latestDataSet.size() > 1UZ;

            if (settings.mountain_range && (hasMultiSignals || _historyBufferY.size() > 1UZ)) {
                // mountain range mode
                dataset::MountainRangeConfig mrConfig{.chart_width = static_cast<std::size_t>(settings.chart_width), .chart_height = static_cast<std::size_t>(settings.chart_height), .x_offset_chars = static_cast<std::size_t>(settings.mountain_x_offset), .y_offset_chars = static_cast<std::size_t>(settings.mountain_y_offset), .color_index = static_cast<std::size_t>(settings.mountain_color_index), .reset_view = shouldResetView ? graphs::ResetChartView::RESET : graphs::ResetChartView::KEEP};

                if (hasMultiSignals) {
                    // multi-signal DataSet: use signals as traces (replace on each update)
                    gr::dataset::drawMountainRange(latestDataSet, mrConfig, _location);
                } else {
                    // single-signal DataSets: accumulate in history as traces
                    std::vector<T> dataSetsVec;
                    dataSetsVec.reserve(_historyBufferY.size());
                    for (const auto& ds : _historyBufferY) {
                        dataSetsVec.push_back(ds);
                    }
                    gr::dataset::drawMountainRange(dataSetsVec, mrConfig, _location);
                }
            } else {
                // regular single-plot mode
                std::size_t signalIdx = signal_index < 0 ? std::numeric_limits<std::size_t>::max() : static_cast<std::size_t>(signal_index);
                gr::dataset::draw(latestDataSet, {.chart_width = static_cast<std::size_t>(settings.chart_width), .chart_height = static_cast<std::size_t>(settings.chart_height), .reset_view = shouldResetView ? graphs::ResetChartView::RESET : graphs::ResetChartView::KEEP}, signalIdx, _location);
            }
        }
    }

    void plotTiming() {
        std::println("\nPast Timing Events for '{}'", settings.signal_name);
        for (auto const& tag : _historyTags) {
            auto isoTime = [](TimePoint time) noexcept {
                return std::format("{}", time); // ms-precision ISO time-format
            };
            std::uint64_t nsSinceEpoch = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(tag.timestamp.time_since_epoch()).count());

            std::println("{:10} {:1} {:24} {} {}", tag.index, tag.merged ? 'M' : 'T', isoTime(tag.timestamp), nsSinceEpoch, tag.map);
        }
    }
};

// the synchronous form: it prints from processBulk instead of offering a drawable surface
template<typename T>
using ConsoleDebugSink = ImChartMonitor<T, false>;

} // namespace gr::blocks::testing

#endif // GNURADIO_IMCHARTMONITOR_HPP
