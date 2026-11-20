#ifndef GNURADIO_CLOCK_SOURCE_HPP
#define GNURADIO_CLOCK_SOURCE_HPP

#include <atomic>
#include <chrono>
#include <format>
#include <ranges>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/BlockingSync.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/TriggerMatcher.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>

namespace gr::blocks::basic {

template<typename T, typename... Arguments>
using A = gr::Annotated<T, Arguments...>;
using namespace gr;

GR_REGISTER_BLOCK("gr::blocks::basic::ClockSource", gr::blocks::basic::DefaultClockSource)

template<typename T, typename ClockSourceType = std::chrono::system_clock>
struct ClockSource : Block<ClockSource<T, ClockSourceType>>, BlockingSync<ClockSource<T, ClockSourceType>, ClockSourceType> {
    using Description = Doc<R""(@brief generates clock signals with specified timing intervals.

Operating modes (controlled by `use_internal_thread`):
  true (default): internal timer thread wakes scheduler every (chunk_size / sample_rate) seconds
  false: block produces samples based on elapsed time when scheduler calls processBulk

The 'tag_times[ns]:tag_value(string)' vectors control the emission of tags at specified times after block start.
The first sample after a start or a change of `sample_rate` carries a `gr:sample_rate` tag.
Terminates when n_samples_max is reached (0 = unlimited).

Each entry of `tag_values` is a tag written '<trigger name>/<ctx>'. A `repeat_period` beyond the last of `tag_times`
restarts the tags after each period. With more `tag_times` than `tag_values`, `do_zero_order_hold` publishes the last
tag again, and otherwise an empty one. `use_internal_thread` paces the block from a timer thread of its own; otherwise
timing comes on demand from outside.
)"">;

    using TimePoint = std::chrono::time_point<ClockSourceType>;

    PortOut<T> out;

    struct Settings {
        A<gr::Size_t, Visible, Doc<"sample limit (0 = unlimited)">>          n_samples_max = 1024;
        A<float, Unit<"Hz">, Visible, Doc<"sample rate">, Limits<Over<0.f>>> sample_rate   = 1000.f;
        A<gr::Size_t, Visible, Doc<"samples per update">, Limits<Min<1U>>>   chunk_size    = 100;
        A<Tensor<std::uint64_t>, Unit<"ns">, Doc<"tag times">>               tag_times;
        A<std::vector<std::string>, Doc<"tag values">>                       tag_values;
        A<std::uint64_t, Unit<"ns">, Visible, Doc<"tag repeat period">>      repeat_period{0U};
        A<bool, Doc<"zero-order hold">>                                      do_zero_order_hold  = false;
        A<bool, Doc<"internal timer thread">>                                use_internal_thread = true;
        A<bool>                                                              verbose_console     = false;

        GR_MAKE_REFLECTABLE(Settings, n_samples_max, sample_rate, chunk_size, tag_times, tag_values, repeat_period, do_zero_order_hold, use_internal_thread, verbose_console);
    } settings;

    GR_MAKE_REFLECTABLE(ClockSource, out, settings);

    std::vector<Tag> tags{};
    gr::Size_t       n_samples_produced{0};

    TimePoint   _beginSequenceTimePoint{};
    bool        _beginSequenceTimePointInitialized{false};
    std::size_t _nextTimeTag{0};
    std::size_t _nextTagIndex{0};
    bool        _rateOwed{true}; // the next published sample carries a `gr:sample_rate` tag

    // ~Block() invokes stop() for a block that is still active, and by then the timer state the mixin's stop()
    // waits on has been released: the active states have to be left here, while the derived object still stands.
    ~ClockSource() {
        if (lifecycle::isActive(this->state())) {
            std::ignore = this->changeStateTo(lifecycle::State::REQUESTED_STOP);
        }
        this->stopTimerAndJoin();
    }

    void start() {
        if (settings.verbose_console) {
            std::println("ClockSource::start() - use_internal_thread: {}", static_cast<bool>(settings.use_internal_thread));
        }
        n_samples_produced                 = 0U;
        _nextTimeTag                       = 0;
        _nextTagIndex                      = 0;
        _beginSequenceTimePointInitialized = false;
        _rateOwed                          = true;

        this->blockingSyncStart();

        if (settings.verbose_console) {
            std::println("started {}", this->name);
        }
    }

    void stop() {
        if (settings.verbose_console) {
            std::println("stopping {}", this->name);
        }
        this->blockingSyncStop();
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const Tensor<std::uint64_t>& times = s.tag_times.value;
        if (std::ranges::adjacent_find(times, std::greater_equal()) != times.end()) {
            return gr::Refused(s.tag_times, "must be strictly ascending");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        if (s.sample_rate.changed()) {
            _rateOwed = true;
        }
    }

    work::Status processBulk(OutputSpanLike auto& outSpan) noexcept {
        if (settings.n_samples_max > 0UZ && n_samples_produced >= settings.n_samples_max) {
            outSpan.publish(0UZ);
            return work::Status::DONE;
        }

        const gr::Size_t remainingSamples = (settings.n_samples_max > 0UZ) ? settings.n_samples_max - n_samples_produced : std::numeric_limits<gr::Size_t>::max();
        const auto       syncDrivenCount  = this->syncSamples(outSpan);
        gr::Size_t       samplesToProduce = std::min({remainingSamples, static_cast<gr::Size_t>(syncDrivenCount)});

        if (samplesToProduce == 0) {
            outSpan.publish(0UZ);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        if (_rateOwed) {
            outSpan.publishTag(property_map{{std::pmr::string(tag::SAMPLE_RATE.key()), settings.sample_rate.value}}, 0UZ);
            _rateOwed = false;
        }

        const auto now = ClockSourceType::now();

        gr::Size_t samplesToNextTimeTag = std::numeric_limits<gr::Size_t>::max();
        if (!settings.tag_times.value.empty()) {
            if (_nextTimeTag == 0 && !_beginSequenceTimePointInitialized) {
                _beginSequenceTimePoint            = this->blockingSyncStartTime();
                _beginSequenceTimePointInitialized = true;
            }
            if (_nextTimeTag >= settings.tag_times.value.size() && settings.repeat_period >= settings.tag_times.value.back()) {
                _beginSequenceTimePoint += std::chrono::microseconds(settings.repeat_period / 1000);
                _beginSequenceTimePointInitialized = true;
                _nextTimeTag                       = 0;
            }
            if (_nextTimeTag < settings.tag_times.value.size()) {
                const auto tagOffsetNs       = std::chrono::duration_cast<std::chrono::nanoseconds>(_beginSequenceTimePoint - this->blockingSyncStartTime()).count() + static_cast<std::int64_t>(settings.tag_times.value[_nextTimeTag]);
                const auto absoluteTagSample = static_cast<gr::Size_t>(std::max(0L, std::lround(static_cast<double>(tagOffsetNs) / 1.e9 * static_cast<double>(settings.sample_rate))));
                samplesToNextTimeTag         = (absoluteTagSample >= n_samples_produced) ? (absoluteTagSample - n_samples_produced) : 0U;
            }
        }

        auto samplesToNextTag = tags.empty() || _nextTagIndex >= tags.size() ? std::numeric_limits<gr::Size_t>::max() : static_cast<gr::Size_t>(tags[_nextTagIndex].index) - n_samples_produced;

        if (samplesToNextTag < samplesToNextTimeTag) {
            if (_nextTagIndex < tags.size() && samplesToNextTag <= samplesToProduce) {
                const auto tagDeltaIndex = tags[_nextTagIndex].index - static_cast<std::size_t>(n_samples_produced);
                if (settings.verbose_console) {
                    const Tag&  tag   = tags[_nextTagIndex];
                    auto        entry = [](const auto& kvp) { return std::make_pair(std::string_view(kvp.first), kvp.second); };
                    std::string mapText{"<empty map>"};
                    if (!tag.map.empty()) {
                        mapText = gr::join(tag.map | std::views::transform(entry), ", ");
                    }
                    std::println("{}::processBulk(...)\t publish tag at {:6} @index= {}: map: {{ {} }}", //
                        this->name, n_samples_produced + tagDeltaIndex, tag.index, mapText);
                }
                outSpan.publishTag(tags[_nextTagIndex].map, tagDeltaIndex);
                samplesToProduce = std::max(samplesToNextTag, gr::Size_t{1});
                _nextTagIndex++;
            }
        } else {
            if (!settings.tag_times.value.empty() && _nextTimeTag < settings.tag_times.value.size() && samplesToNextTimeTag <= samplesToProduce) {
                const std::string value = _nextTimeTag < settings.tag_values.value.size() ? settings.tag_values.value[_nextTimeTag] : (settings.do_zero_order_hold ? settings.tag_values.value.back() : "");

                std::string           triggerName;
                [[maybe_unused]] bool triggerNameNegated;
                std::string           triggerContext;
                [[maybe_unused]] bool triggerContextNegated;
                gr::blocks::basic::trigger::detail::parse(value, triggerName, triggerNameNegated, triggerContext, triggerContextNegated);

                property_map triggerTag;
                uint64_t     triggerTime = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
                triggerTime += static_cast<std::uint64_t>(static_cast<float>(samplesToNextTimeTag) * 1e9f / settings.sample_rate);

                triggerTag[tag::TRIGGER_NAME.key()]      = triggerName;
                triggerTag[tag::TRIGGER_TIME.key()]      = triggerTime;
                triggerTag[tag::TRIGGER_OFFSET.key()]    = 0.f;
                triggerTag[tag::CONTEXT.key()]           = triggerContext;
                triggerTag[tag::TRIGGER_META_INFO.key()] = property_map{};

                if (settings.verbose_console) {
                    std::println("{}::processBulk(...)\t publish tag-time at {:6}, time:{}ns", this->name, samplesToNextTimeTag, settings.tag_times.value[_nextTimeTag]);
                }
                outSpan.publishTag(triggerTag, samplesToNextTimeTag);
                samplesToProduce = std::max(samplesToNextTimeTag, gr::Size_t{1});
                _nextTimeTag++;
            }
        }

        outSpan.publish(static_cast<std::size_t>(samplesToProduce));
        n_samples_produced += samplesToProduce;

        return work::Status::OK;
    }
};

using DefaultClockSource = ClockSource<std::uint8_t, std::chrono::system_clock>;

} // namespace gr::blocks::basic

static_assert(gr::HasProcessBulkFunction<gr::blocks::basic::ClockSource<std::uint8_t>>);

#endif // GNURADIO_CLOCK_SOURCE_HPP
