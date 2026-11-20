#ifndef GNURADIO_RATE_PROBE_HPP
#define GNURADIO_RATE_PROBE_HPP

#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <format>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::RateProbe, [T], [ std::uint8_t, std::int16_t, std::int32_t, std::uint32_t, float, double, std::complex<float>, std::complex<double> ])

template<typename T>
struct RateProbe : Block<RateProbe<T>> {
    using Description = Doc<R""(
@brief Measures the rate at which samples arrive at its input, in samples per second.

The block consumes every sample it receives. `rate` is the arrival rate at this block. A `Throttle` upstream sets
that rate when no block between them changes the rate. Behind a source that runs freely, `rate` reports how fast the
graph runs.

The first interval opens at the work call that delivers the first samples, and each interval counts the samples
that arrive after it opens. A measurement takes place in a work call once `min_update_ms` have passed since the
interval opened. It divides the samples received in that interval by the interval's length on a steady clock and
opens the next interval. `rate_now` holds the quotient. `rate` takes the first result as it is, then moves toward
each new result by the weight `alpha`. Before the first measurement `rate` and `rate_now` are 0. While no samples
arrive, both keep their last values. `n_samples` counts the samples received since start and wraps at 2^32.

`rate`, `rate_now` and `n_samples` are status values: the block reports the three at the start and after each
measurement, where a Get reads them, and sends one notification on `msgOut` with the endpoint `rate`,
carrying the same three values.

`alpha` is the weight of the newest measurement in `rate`.
)"">;

    using Clock = std::chrono::steady_clock;

    PortIn<T> in;

    struct Settings {
        Annotated<float, Doc<"newest measurement weight">, Visible, Limits<Over<0.f>, Max<1.f>>>     alpha         = 0.15f;
        Annotated<float, Unit<"ms">, Doc<"minimum measurement interval">, Visible, Limits<Min<0.f>>> min_update_ms = 500.f;

        GR_MAKE_REFLECTABLE(Settings, alpha, min_update_ms);
    } settings;

    struct Status {
        Annotated<double, Unit<"Hz">, Doc<"smoothed arrival rate">, Visible>      rate      = 0.0;
        Annotated<double, Unit<"Hz">, Doc<"last interval arrival rate">, Visible> rate_now  = 0.0;
        Annotated<gr::Size_t, Doc<"samples received">, Visible>                   n_samples = 0U;

        GR_MAKE_REFLECTABLE(Status, rate, rate_now, n_samples);
    } status;

    GR_MAKE_REFLECTABLE(RateProbe, in, settings, status);

    Clock::time_point _intervalStart   = Clock::now();
    std::uint64_t     _intervalSamples = 0ULL;
    gr::Size_t        _unpublished     = 0U; // samples received since `n_samples` was last reported
    bool              _measuredOnce    = false;
    bool              _receiving       = false;

    void start() {
        _intervalStart   = Clock::now();
        _intervalSamples = 0ULL;
        _unpublished     = 0U;
        _measuredOnce    = false;
        _receiving       = false;
        status.rate      = 0.0;
        status.rate_now  = 0.0;
        status.n_samples = 0U;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan) {
        const std::size_t nSamples = inSpan.size();
        std::ignore                = inSpan.consume(nSamples);
        _unpublished               = static_cast<gr::Size_t>(_unpublished + nSamples);
        if (nSamples == 0UZ) {
            return work::Status::OK;
        }

        const Clock::time_point now = Clock::now();
        if (!_receiving) {
            _receiving     = true;
            _intervalStart = now;
            return work::Status::OK;
        }
        _intervalSamples += nSamples;
        const std::chrono::duration<double> elapsed = now - _intervalStart;
        if (elapsed.count() > 0.0 && elapsed.count() * 1e3 >= static_cast<double>(settings.min_update_ms)) {
            measure(now, elapsed.count());
        }
        return work::Status::OK;
    }

private:
    void measure(Clock::time_point now, double seconds) {
        const gr::Size_t received = status.n_samples.value + _unpublished; // wraps at 2^32
        _unpublished              = 0U;
        const double rateNow      = static_cast<double>(_intervalSamples) / seconds;
        const double weight       = static_cast<double>(settings.alpha.value);
        const double smooth       = _measuredOnce ? weight * rateNow + (1.0 - weight) * status.rate.value : rateNow;
        _measuredOnce             = true;

        _intervalSamples = 0ULL;
        _intervalStart   = now;

        status.rate      = smooth;
        status.rate_now  = rateNow;
        status.n_samples = received;
        this->emitMessage("rate", {{"rate", smooth}, {"rate_now", rateNow}, {"n_samples", received}});
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_RATE_PROBE_HPP
