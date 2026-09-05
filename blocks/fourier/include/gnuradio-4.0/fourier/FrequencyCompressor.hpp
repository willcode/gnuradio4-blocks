#ifndef GNURADIO_FREQUENCY_COMPRESSOR_HPP
#define GNURADIO_FREQUENCY_COMPRESSOR_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <format>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/fourier/PhaseVocoder.hpp>

namespace gr::blocks::fourier {

GR_REGISTER_BLOCK(gr::blocks::fourier::FrequencyCompressor)

struct FrequencyCompressor : Block<FrequencyCompressor> {
    using Description = Doc<R""(
@brief Divides every frequency in a complex stream by an integer, in real time.

An ultrasonic band is inaudible because of where it sits, not how loud it is; dividing every frequency by N moves
the whole band into hearing at once — five octaves down at N = 32 — where a heterodyne brings down one narrow slice
at a time. Division is a pure TRANSPOSITION: every frequency ratio survives, so a call and its harmonics stay
harmonically related and a five-octave band stays five octaves wide. Nothing here adds a frequency offset,
deliberately — an offset would preserve neither.

"Same duration, frequencies divided by N" is "reinterpret the stream at 1/N of its sample rate, then compress time
by N to give the duration back". The rate reinterpretation is free — the output rate is declared fs/N — and the time
compression is a phase vocoder resynthesizing the unmodified spectrum at 1/N of its analysis hop, each bin's phase
advanced by its measured frequency so partials stay coherent across frames. Level is preserved by normalizing the
overlap-add against the window energy actually laid down.

The analysis frame sets the lowest resolvable output frequency (fs/frame well below it) and how much a transient
smears: compressing a 190 kHz band to end near 300 Hz wants about 16 ms of window at 250 kSps, and shorter events
arrive blurred toward that. Narrowing the input band and lowering the divisor trades reach for sharpness; every
setting stays a pure transposition.

Both settings move while the block runs. A new `divisor` or `frame` re-plans the vocoder in place, and a new divisor
also has the block state its output rate on the next sample it publishes, as `sample_rate / divisor` — so a chain
that derives both this divisor and a downstream ratio from one parameter can move them together in a single settings
transaction instead of being rebuilt. A new frame changes the resolution and the latency but not the rate, and states
nothing. Leaving `sample_rate` at zero leaves the rate unstated and publishes no tag, which is what the block did
before it had the setting.

**A change is audible.** Settings are atomic; a stream is not. Samples already downstream were produced under the
old settings, and the frame in flight is dropped rather than resynthesized under settings it was not analyzed for —
the same rule the block already applies at end of stream, a window that cannot fill resolving no frequency. Crossing
to or from `divisor == 1` also moves the latency between none and a frame, 1 being the bit-exact passthrough that
skips the vocoder entirely.

The ports are Async (an arbitrary integer ratio is not a constant L:M); at end of stream the sub-frame tail is
dropped and the block ends with the stream.
)"">;

    PortIn<std::complex<float>, Async>  in;
    PortOut<std::complex<float>, Async> out;

    Annotated<gr::Size_t, "divisor", Visible, Doc<"frequency divisor; 1 is a bit-exact passthrough with no latency">>                           divisor     = 1U;
    Annotated<gr::Size_t, "frame", Doc<"analysis frame; a power of two, fs/frame well below the lowest output frequency wanted">>               frame       = 4096U;
    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"input sample rate; 0 leaves the output rate unstated and publishes no rate tag">> sample_rate = 0.f;

    GR_MAKE_REFLECTABLE(FrequencyCompressor, in, out, divisor, frame, sample_rate);

    gr::algorithm::PhaseVocoder      _vocoder;
    std::vector<std::complex<float>> _queue;
    std::vector<std::complex<float>> _made;
    bool                             _running       = false;
    gr::Size_t                       _statedDivisor = 0U; /// the divisor the last published rate tag stated; 0 is none

    /// One call's input bound, so a burst of buffered input cannot queue unbounded output.
    static constexpr std::size_t kMaxTake = 1UZ << 16;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (divisor == 0U) {
            throw gr::exception("divisor must be at least 1");
        }
        const auto frameSize = static_cast<std::size_t>(frame.value);
        if (frameSize == 0UZ || (frameSize & (frameSize - 1UZ)) != 0UZ) {
            throw gr::exception(std::format("frame must be a power of two, got {}", frame.value));
        }
        if (static_cast<std::size_t>(divisor.value) > frameSize / 4UZ) {
            throw gr::exception(std::format("divisor ({}) must not exceed a quarter of the frame ({}): the synthesis hop would vanish", divisor.value, frame.value));
        }
        if (_running && (newSettings.contains("divisor") || newSettings.contains("frame"))) {
            replan();
        }
    }

    /// @brief Plan the vocoder for the current frame and divisor, discarding whatever the old settings left behind.
    void replan() {
        const auto frameSize = static_cast<std::size_t>(frame.value);
        const auto n         = static_cast<std::size_t>(divisor.value);
        // A quarter-frame analysis hop is the overlap at which the Hann window is well behaved;
        // shrinking it to a multiple of the divisor keeps the synthesis hop whole, which is what
        // keeps the output rate exactly fs over the divisor.
        std::size_t hopIn = frameSize / 4UZ;
        hopIn -= hopIn % n;
        _vocoder.configure(frameSize, hopIn, hopIn / n);
        _queue.clear();
        _made.clear();
    }

    void start() {
        replan();
        _statedDivisor = 0U; // no divisor has been stated yet, so the first output states the one in force
        _running       = true;
    }

    void stop() { _running = false; }

    /// @brief State the output rate on the next sample published after it moved, if the block was told a rate at all.
    ///
    /// The test is against the divisor last stated rather than against which keys a settings transaction carried:
    /// a transaction can restage a value that did not move, and a redundant rate tag is something downstream has to
    /// decide to ignore. A new frame changes the resolution and the latency but not the rate, and states nothing.
    void stateRate(OutputSpanLike auto& outSpan) {
        if (divisor.value == _statedDivisor) {
            return;
        }
        if (sample_rate > 0.f) {
            outSpan.publishTag(property_map{{static_cast<std::pmr::string>(gr::tag::SAMPLE_RATE), sample_rate.value / static_cast<float>(divisor.value)}}, 0UZ);
        }
        _statedDivisor = divisor.value;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (divisor == 1U) { // passthrough: nothing to do and no latency to add
            const std::size_t n = std::min(inSpan.size(), outSpan.size());
            std::copy_n(inSpan.begin(), n, outSpan.begin());
            std::ignore = inSpan.consume(n);
            if (n > 0UZ) {
                stateRate(outSpan);
            }
            outSpan.publish(n);
            return n > 0UZ ? work::Status::OK : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        // Input is only taken while nothing waits to go out, so the queue stays shallow and
        // back pressure reaches the source instead of accumulating here.
        if (_queue.empty()) {
            const std::size_t take = std::min(inSpan.size(), kMaxTake);
            if (take == 0UZ) {
                std::ignore = inSpan.consume(0UZ);
                outSpan.publish(0UZ);
                return work::Status::INSUFFICIENT_INPUT_ITEMS;
            }
            _made.clear();
            _vocoder.process(std::span<const std::complex<float>>(inSpan.data(), take), _made);
            _queue.insert(_queue.end(), _made.begin(), _made.end());
            std::ignore = inSpan.consume(take);
        } else {
            std::ignore = inSpan.consume(0UZ);
        }

        const std::size_t n = std::min(_queue.size(), outSpan.size());
        std::copy_n(_queue.begin(), n, outSpan.begin());
        _queue.erase(_queue.begin(), _queue.begin() + static_cast<std::ptrdiff_t>(n));
        if (n > 0UZ) {
            stateRate(outSpan);
        }
        outSpan.publish(n);
        // Publishing nothing while the first frame fills is ordinary start-up latency, but it
        // must be reported as wanting input rather than as work done, or the scheduler has no
        // reason to come back with more.
        return n > 0UZ ? work::Status::OK : work::Status::INSUFFICIENT_INPUT_ITEMS;
    }
};

} // namespace gr::blocks::fourier

#endif // GNURADIO_FREQUENCY_COMPRESSOR_HPP
