#ifndef GNURADIO_SIGNAL_GENERATOR_HPP
#define GNURADIO_SIGNAL_GENERATOR_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/BlockingSync.hpp>

#include <gnuradio-4.0/algorithm/signal/SignalGeneratorCore.hpp>

namespace gr::blocks::basic {

using namespace gr;

namespace signal_generator {
using Type = gr::signal::SignalType; // same enum, reused from algorithm core
using enum Type;
constexpr auto                                 TypeList  = magic_enum::enum_values<Type>();
inline static constexpr gr::meta::fixed_string TypeNames = "[Const, Sin, Cos, Square, Saw, Triangle, FastSin, FastCos, UniformNoise, TriangularNoise, GaussianNoise]";

} // namespace signal_generator

GR_REGISTER_BLOCK(gr::blocks::basic::SignalGenerator, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct SignalGenerator : Block<SignalGenerator<T>>, BlockingSync<SignalGenerator<T>> {
    using Description = Doc<R""(@brief generates signal waveforms: sine, cosine, square, saw, triangle, constant, fast sine/cosine, and noise.

Operating modes:
  clk_in connected: generates one sample per clock input sample
  clk_in disconnected: free-running mode synchronised to wall-clock time

The first sample after a start or a change of `sample_rate` carries a `gr:sample_rate` tag.

Signal types (A = amplitude, f = tone_frequency, P = phase, O = offset):
  Sin/Cos:          A * sin/cos(2π·f·t + P) + O  (std::sin/cos, high precision)
  FastSin/FastCos:  A * sin/cos(2π·f·t + P) + O  (recursive phasor, ~10x faster)
  Constant:         A + O
  Square/Saw/Triangle: standard periodic waveforms
  UniformNoise:     uniform random in [-A, +A) + O
  TriangularNoise:  triangular random in [-A, +A) + O
  GaussianNoise:    Gaussian N(0, A²) + O

`chunk_size` is the samples per update in free-running mode. `tone_frequency` is the waveform's own frequency, apart
from the stream's.
)"">;

    PortIn<std::uint8_t, Optional> clk_in;
    PortOut<T>                     out;

    struct Settings {
        Annotated<float, Unit<"Hz">, Visible, Doc<"sample rate">, Limits<Over<0.f>>> sample_rate    = 1000.f;
        Annotated<gr::Size_t, Visible, Doc<"samples per update">>                    chunk_size     = 100;
        Annotated<signal_generator::Type, Visible, Doc<"signal type">>               signal_type    = signal_generator::Type::Sin;
        Annotated<float, Visible, Unit<"Hz">, Doc<"tone frequency">>                 tone_frequency = 1.f;
        Annotated<float, Visible>                                                    amplitude      = 1.f;
        Annotated<float, Visible>                                                    offset         = 0.f;
        Annotated<float, Unit<"rad">, Visible, Doc<"phase">>                         phase          = 0.f;
        Annotated<std::uint64_t, Visible, Doc<"noise seed (0 = fixed default)">>     seed           = 0ULL;

        GR_MAKE_REFLECTABLE(Settings, sample_rate, chunk_size, signal_type, tone_frequency, amplitude, offset, phase, seed);
    } settings;

    GR_MAKE_REFLECTABLE(SignalGenerator, clk_in, out, settings);

    gr::signal::SignalGeneratorCore<T> _core;
    bool                               _rateOwed{true}; // true until a published sample states the current rate

    // ~Block() invokes stop() for a block that is still active, and by then the timer state the mixin's stop()
    // waits on has been released: the active states have to be left here, while the derived object still stands.
    ~SignalGenerator() {
        if (lifecycle::isActive(this->state())) {
            std::ignore = this->changeStateTo(lifecycle::State::REQUESTED_STOP);
        }
        this->stopTimerAndJoin();
    }

    void start() {
        _core.configure(settings.signal_type.value, settings.tone_frequency, settings.sample_rate, settings.phase, settings.amplitude, settings.offset, settings.seed);
        _core.reset();
        _rateOwed = true;
        this->blockingSyncStart();
    }

    void stop() { this->blockingSyncStop(); }

    void applySettings(const Settings& s) {
        _core.configure(s.signal_type, s.tone_frequency, s.sample_rate, s.phase, s.amplitude, s.offset, s.seed);
        if (s.sample_rate.changed()) {
            _rateOwed = true;
        }
    }

    work::Status processBulk(InputSpanLike auto& input, OutputSpanLike auto& output) {
        const auto nSamples = this->syncSamples(input, output);
        if (nSamples == 0) {
            std::ignore = input.consume(0);
            output.publish(0);
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        for (std::size_t i = 0; i < nSamples; ++i) {
            output[i] = _core.generateSample();
        }
        if (_rateOwed) {
            output.publishTag(property_map{{std::pmr::string(tag::SAMPLE_RATE.key()), settings.sample_rate.value}}, 0UZ);
            _rateOwed = false;
        }

        std::ignore = input.consume(this->isFreeRunning() ? 0 : nSamples);
        output.publish(nSamples);
        return work::Status::OK;
    }

    [[nodiscard]] T generateSample() noexcept { return _core.generateSample(); }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_SIGNAL_GENERATOR_HPP
