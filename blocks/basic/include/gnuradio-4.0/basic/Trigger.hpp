#ifndef TRIGGER_HPP
#define TRIGGER_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/SchmittTrigger.hpp>
#include <gnuradio-4.0/meta/UncertainValue.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK("gr::blocks::basic::SchmittTriggerNoInterpolation", gr::blocks::basic::SchmittTriggerNoInterpolation, [T], [ std::int16_t, std::int32_t, float ])
GR_REGISTER_BLOCK("gr::blocks::basic::SchmittTriggerBasic", gr::blocks::basic::SchmittTriggerBasic, [T], [ std::int16_t, std::int32_t, float ])
GR_REGISTER_BLOCK("gr::blocks::basic::SchmittTrigger", gr::blocks::basic::SchmittTrigger, [T], [ std::int16_t, std::int32_t, float ])
GR_REGISTER_BLOCK("gr::blocks::basic::SchmittTriggerPolynomial", gr::blocks::basic::SchmittTriggerPolynomial, [T], [ std::int16_t, std::int32_t, float ])

// the default is the method the plain name is registered under; the three others are named below
template<typename T, gr::trigger::InterpolationMethod Method = gr::trigger::InterpolationMethod::LINEAR_INTERPOLATION>
requires(std::is_arithmetic_v<T> or (UncertainValueLike<T> && std::is_arithmetic_v<meta::fundamental_base_value_type_t<T>>))
struct SchmittTrigger : public gr::Block<SchmittTrigger<T, Method>, NoTagPropagation> {
    using Description = Doc<R""(@brief Digital Schmitt trigger implementation with optional intersample interpolation

@see https://en.wikipedia.org/wiki/Schmitt_trigger

The following sub-sample interpolation methods are supported:
  * NO_INTERPOLATION: nomen est omen
  * BASIC_LINEAR_INTERPOLATION: basic linear interpolation based on the new and previous sample
  * LINEAR_INTERPOLATION: interpolation via linear regression over the samples between when
    the lower and upper threshold has been crossed and vice versa
  * POLYNOMIAL_INTERPOLATION: Savitzky–Golay filter-based methods
    https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter

The block generates the following optional trigger that are controlled by:
 * trigger_name_rising_edge  -> trigger name that is used when a rising edge is detected
 * trigger_name_falling_edge -> trigger name that is used when a falling edge is detected

The trigger time and offset is calculated through sample counting from the last synchronising trigger.
The information is stored (info only) in `trigger_name`, `trigger_time`, `trigger_offset`.

Without tag forwarding, the block emits only the tags of detected edges. The last trigger's UTC time and offset
synchronize time, and sample counting runs from there.
)"">;
    using enum gr::trigger::EdgeDetection;
    using ClockSourceType = std::chrono::system_clock;
    using value_t         = meta::fundamental_base_value_type_t<T>;

    template<typename U, typename... Arguments>
    using A = gr::Annotated<U, Arguments...>;

    constexpr static std::size_t N_HISTORY = 32UZ;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        A<value_t, Doc<"trigger offset">, Visible>                              offset{value_t(0)};
        A<value_t, Doc<"trigger threshold">, Visible>                           threshold{value_t(1)};
        A<std::string, Doc<"rising edge trigger name (empty = off)">, Visible>  trigger_name_rising_edge{magic_enum::enum_name(RISING)};
        A<std::string, Doc<"falling edge trigger name (empty = off)">, Visible> trigger_name_falling_edge{magic_enum::enum_name(FALLING)};
        A<float, Doc<"avg. sample rate">, Visible, Limits<Over<0.f>>>           sample_rate = 1.f;

        A<bool, Doc<"tag forwarding">>                         forward_tag{true};
        A<std::string, Doc<"last synchronizing trigger">>      trigger_name = "";
        A<std::uint64_t, Doc<"last trigger time">, Unit<"ns">> trigger_time{0U};
        A<float, Doc<"last trigger offset">, Unit<"s">>        trigger_offset{0.0f};
        std::string                                            context = "";

        GR_MAKE_REFLECTABLE(Settings, offset, threshold, trigger_name_rising_edge, trigger_name_falling_edge, sample_rate, forward_tag, trigger_name, trigger_time, trigger_offset, context);
    } settings;

    GR_MAKE_REFLECTABLE(SchmittTrigger, in, out, settings);

    gr::trigger::SchmittTrigger<T, Method, N_HISTORY> _trigger{};
    std::uint64_t                                     _period{0U}; // nanoseconds per sample, at the clock `_now` runs on
    std::uint64_t                                     _now{0U};    // nanoseconds since the Unix epoch

    void applySettings(const Settings& s) {
        _period = static_cast<std::uint64_t>(1e9f / s.sample_rate);
        _trigger.setOffset(s.offset);
        _trigger.setThreshold(s.threshold);

        if (s.trigger_time.changed() || s.trigger_offset.changed()) {
            _now = s.trigger_time + static_cast<std::uint64_t>(1e9f * s.trigger_offset);
        }
        if (s.offset.changed() || s.threshold.changed()) {
            _trigger.reset();
        }
    }

    void start() { reset(); }

    void reset() {
        _trigger.reset();
        _now = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(ClockSourceType::now().time_since_epoch()).count());
    }

    gr::work::Status processBulk(InputSpanLike auto& inputSpan, OutputSpanLike auto& outputSpan) {
        const std::optional<std::size_t> nextEoSTag = samples_to_eos_tag(in);
        if (inputSpan.size() < N_HISTORY && !nextEoSTag.has_value()) {
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        const std::size_t nProcessInput = nextEoSTag.has_value() ? inputSpan.size() : (inputSpan.size() > N_HISTORY ? inputSpan.size() - N_HISTORY : 0UZ);
        const std::size_t nProcess      = std::min(nProcessInput, outputSpan.size());

        if (nProcess == 0) {
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        auto forwardTags = [&](std::size_t maxRelIndex) {
            if (!settings.forward_tag) {
                return;
            }
            for (const auto& tag : inputSpan.rawTags) {
                const auto relIndex = tag.index >= inputSpan.streamIndex                                   //
                                          ? static_cast<std::ptrdiff_t>(tag.index - inputSpan.streamIndex) //
                                          : -static_cast<std::ptrdiff_t>(inputSpan.streamIndex - tag.index);
                if (relIndex >= 0 && static_cast<std::size_t>(relIndex) <= maxRelIndex) {
                    outputSpan.publishTag(tag.map, static_cast<std::size_t>(relIndex));
                }
            }
        };

        auto publishEdge = [&](const std::string& triggerName, std::size_t edgePos) {
            forwardTags(edgePos);

            const UncertainValue<float> edgeIdxOffset = UncertainValue<float>{static_cast<float>(_trigger.lastEdgeIdx)} + _trigger.lastEdgeOffset;
            const float                 relOffset     = gr::value(edgeIdxOffset) * static_cast<float>(_period); // nanoseconds
            outputSpan.publishTag(
                property_map{
                    {gr::tag::TRIGGER_NAME.key(), triggerName},                                                                  //
                    {gr::tag::TRIGGER_TIME.key(), _now - static_cast<uint64_t>(relOffset)},                                      //
                    {"trigger_time_error", static_cast<uint64_t>(gr::uncertainty(edgeIdxOffset) * static_cast<float>(_period))}, //
                    {gr::tag::TRIGGER_OFFSET.key(), relOffset / 1e9f},                                                           // seconds, as Tag.hpp declares the key
                    {gr::tag::CONTEXT.key(), settings.context}                                                                   //
                },
                edgePos);

            const std::size_t nPublish = edgePos + 1; // include edge sample
            std::copy_n(inputSpan.begin(), nPublish, outputSpan.begin());
            std::ignore = inputSpan.consume(nPublish);
            outputSpan.publish(nPublish);
            return gr::work::Status::OK;
        };

        for (std::size_t i = 0; i < nProcess; ++i) {
            _now += _period;

            if (_trigger.processOne(inputSpan[i]) != NONE) { // edge detected
                const std::ptrdiff_t edgePosition = static_cast<std::ptrdiff_t>(i) + _trigger.lastEdgeIdx;

                if (edgePosition >= 0 && static_cast<std::size_t>(edgePosition) < nProcess) {
                    if (_trigger.lastEdge == RISING && !settings.trigger_name_rising_edge.value.empty()) {
                        return publishEdge(settings.trigger_name_rising_edge, static_cast<std::size_t>(edgePosition));
                    }
                    if (_trigger.lastEdge == FALLING && !settings.trigger_name_falling_edge.value.empty()) {
                        return publishEdge(settings.trigger_name_falling_edge, static_cast<std::size_t>(edgePosition));
                    }
                }
            }
        }

        // no trigger found - copy samples & tags
        std::copy_n(inputSpan.begin(), nProcess, outputSpan.begin());
        forwardTags(nProcess - 1);
        std::ignore = inputSpan.consume(nProcess);
        outputSpan.publish(nProcess);
        return gr::work::Status::OK;
    }
};

// the interpolation method each registered name stands for; SchmittTrigger itself is the linear one
template<typename T>
using SchmittTriggerNoInterpolation = SchmittTrigger<T, gr::trigger::InterpolationMethod::NO_INTERPOLATION>;
template<typename T>
using SchmittTriggerBasic = SchmittTrigger<T, gr::trigger::InterpolationMethod::BASIC_LINEAR_INTERPOLATION>;
template<typename T>
using SchmittTriggerPolynomial = SchmittTrigger<T, gr::trigger::InterpolationMethod::POLYNOMIAL_INTERPOLATION>;

} // namespace gr::blocks::basic

#endif // TRIGGER_HPP
