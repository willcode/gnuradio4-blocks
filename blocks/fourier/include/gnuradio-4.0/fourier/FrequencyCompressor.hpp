#ifndef GNURADIO_FREQUENCY_COMPRESSOR_HPP
#define GNURADIO_FREQUENCY_COMPRESSOR_HPP

#include <algorithm>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <expected>
#include <format>
#include <limits>
#include <span>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fourier/PhaseVocoder.hpp>

namespace gr::blocks::fourier {

namespace detail {

/// The longest analysis frame accepted.
inline constexpr gr::Size_t kMaxCompressorFrame = 1U << 20U;

/// @brief True for a sample whose two parts are finite.
inline constexpr auto kFiniteSample = [](const std::complex<float>& sample) { return std::isfinite(sample.real()) && std::isfinite(sample.imag()); };

/// @brief True for a sample with a part that is not finite.
inline constexpr auto kNonFiniteSample = [](const std::complex<float>& sample) { return !kFiniteSample(sample); };

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fourier::FrequencyCompressor)

struct FrequencyCompressor : Block<FrequencyCompressor, NoTagPropagation> {
    using Description = Doc<R""(
@brief Divides every frequency of a complex stream by `divisor` and keeps the duration.

The output is the input's spectrum at 1/`divisor` of its frequencies. A band and its harmonics keep their frequency
ratios. A phase vocoder compresses time by `divisor`, and the output rate is `sample_rate / divisor`.

`frame` is the analysis length, a power of two. `fs / frame` is the finest frequency step, and `frame / fs` the time a
short event spreads over and the latency above a divisor of 1. `divisor` is at most `frame / 4`. A `divisor` of 1
copies the input with no latency.

With `sample_rate` above zero, the block publishes `sample_rate / divisor` on its first output sample, and again on the
first sample made after that rate changes. A change of `divisor` or `frame` drops the frame in progress and starts a
new one. The output made before the change still goes out, ahead of the first sample of the new plan. At end of stream
the samples short of a frame are dropped.

Input tags are dropped. An upstream `gr:sample_rate` tag does not set `sample_rate`. A sample with a part that is not
finite enters the vocoder as zero. Both ports are asynchronous, and each call publishes the hops the vocoder completes.
)"">;

    PortIn<std::complex<float>, Async>  in;
    PortOut<std::complex<float>, Async> out;

    struct Settings {
        Annotated<gr::Size_t, Visible, Doc<"frequency divisor (1 = copy)">, Limits<Min<1U>, Max<detail::kMaxCompressorFrame / 4U>>> divisor     = 1U;
        Annotated<gr::Size_t, Doc<"analysis frame length">, Limits<Min<4U>, Max<detail::kMaxCompressorFrame>>>                      frame       = 4096U;
        Annotated<float, Visible, Unit<"Hz">, Doc<"sample rate (0 = unstated)">, Limits<Min<0.f>>>                                  sample_rate = 0.f;

        GR_MAKE_REFLECTABLE(Settings, divisor, frame, sample_rate);
    } settings;

    GR_MAKE_REFLECTABLE(FrequencyCompressor, in, out, settings);

    gr::algorithm::PhaseVocoder      _vocoder;
    std::vector<std::complex<float>> _queue;
    std::vector<std::complex<float>> _made;
    std::vector<std::complex<float>> _finite;            ///< the call's input with non-finite samples set to zero
    float                            _queueRate  = 0.f;  ///< the output rate of the samples in `_queue`, 0 for none
    float                            _statedRate = -1.f; ///< the output rate last stated, 0 for none, -1 before the first output

    /// One call's input bound, so a burst of buffered input cannot queue unbounded output.
    static constexpr std::size_t kMaxTake = 1UZ << 16;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (!std::has_single_bit(s.frame.value)) {
            return gr::Refused(s.frame, "must be a power of 2");
        }
        if (s.divisor > s.frame / 4U) {
            return gr::Refused(s.divisor, "must be <= frame / 4");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        if (s.divisor.changed() || s.frame.changed()) {
            if (const std::optional<gr::Error> failure = replan()) {
                this->fail(s.frame, failure->message);
            }
        }
    }

    /// @brief Plans the vocoder for the frame and the divisor and drops the frame in progress. The queued output stays.
    /// Returns the vocoder's refusal, changing nothing.
    [[nodiscard]] std::optional<gr::Error> replan() {
        const std::size_t frameSize = settings.frame;
        const std::size_t n         = settings.divisor;
        // A quarter-frame analysis hop is the overlap at which the Hann window is well behaved.
        // Shrinking it to a multiple of the divisor keeps the synthesis hop whole. A whole
        // synthesis hop keeps the output rate exactly fs over the divisor.
        std::size_t hopIn = frameSize / 4UZ;
        hopIn -= hopIn % n;
        if (const std::expected<void, gr::Error> planned = _vocoder.configure(frameSize, hopIn, hopIn / n); !planned) {
            return planned.error();
        }
        _made.clear();
        return std::nullopt;
    }

    void start() {
        _queue.clear();
        if (const std::optional<gr::Error> failure = replan()) {
            throw gr::exception(failure->message, failure->sourceLocation);
        }
        _statedRate = -1.f;
    }

    /// The output rate the settings give, 0 for none.
    [[nodiscard]] float outputRate() const noexcept { return settings.sample_rate > 0.f ? settings.sample_rate / static_cast<float>(settings.divisor) : 0.f; }

    /// @brief States @p rate on the first sample of @p outSpan when it differs from the rate last stated.
    void stateRate(OutputSpanLike auto& outSpan, float rate) {
        if (rate == _statedRate) {
            return;
        }
        if (rate > 0.f) {
            outSpan.publishTag(property_map{{static_cast<std::pmr::string>(gr::tag::SAMPLE_RATE), rate}}, 0UZ);
        }
        _statedRate = rate;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (settings.divisor == 1U && _queue.empty()) { // passthrough, with nothing to do and no latency to add
            const std::size_t n = std::min(inSpan.size(), outSpan.size());
            std::copy_n(inSpan.begin(), n, outSpan.begin());
            std::ignore = inSpan.consume(n);
            if (n > 0UZ) {
                stateRate(outSpan, outputRate());
            }
            outSpan.publish(n);
            return n > 0UZ ? work::Status::OK : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        // Input is taken only while nothing waits to go out. The queue stays shallow, and back pressure reaches the
        // source.
        if (_queue.empty()) {
            const std::size_t take = std::min(inSpan.size(), kMaxTake);
            if (take == 0UZ) {
                std::ignore = inSpan.consume(0UZ);
                outSpan.publish(0UZ);
                return work::Status::INSUFFICIENT_INPUT_ITEMS;
            }
            std::span<const std::complex<float>> taken(inSpan.data(), take);
            if (!std::ranges::all_of(taken, detail::kFiniteSample)) {
                _finite.assign(taken.begin(), taken.end());
                std::ranges::replace_if(_finite, detail::kNonFiniteSample, std::complex<float>{});
                taken = _finite;
            }
            _made.clear();
            _vocoder.process(taken, _made);
            _queue.insert(_queue.end(), _made.begin(), _made.end());
            _queueRate  = outputRate();
            std::ignore = inSpan.consume(take);
        } else {
            std::ignore = inSpan.consume(0UZ);
        }

        const std::size_t n = std::min(_queue.size(), outSpan.size());
        std::copy_n(_queue.begin(), n, outSpan.begin());
        _queue.erase(_queue.begin(), _queue.begin() + static_cast<std::ptrdiff_t>(n));
        if (n > 0UZ) {
            stateRate(outSpan, _queueRate);
        }
        outSpan.publish(n);
        return n > 0UZ ? work::Status::OK : work::Status::INSUFFICIENT_INPUT_ITEMS;
    }
};

} // namespace gr::blocks::fourier

#endif // GNURADIO_FREQUENCY_COMPRESSOR_HPP
