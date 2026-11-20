#ifndef GNURADIO_CHANNEL_MULTIPATH_HPP
#define GNURADIO_CHANNEL_MULTIPATH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/algorithm/channel/DelayProfile.hpp>
#include <gnuradio-4.0/algorithm/rng/Xoshiro256pp.hpp>
#include <gnuradio-4.0/annotated.hpp>

namespace gr::blocks::channel {

GR_REGISTER_BLOCK(gr::blocks::channel::FadingChannel, [T], [std::complex<float>])

/**
 * @brief Time-varying multipath: a tapped delay line whose tap gains are seeded random processes.
 *
 * Each tap's diffuse component is Clarke's sum of sinusoids, which is deterministic once the seed fixes the
 * arrival angles and phases:
 *
 *     h_m[k] = sqrt(P_m / N) * sum_i exp(j*(2*pi*f_d*cos(theta_i)*k/fs + psi_i))
 *
 * with `theta` and `psi` drawn once per tap at configure time. Tap 0 optionally carries a Rician
 * line-of-sight term at `k_factor`, the diffuse part scaled by `sqrt(1/(K+1))` so the tap's total power is
 * unchanged. Delays are whole samples; fractional-delay arms wait for a consumer.
 *
 * `max_doppler = 0` freezes the process, making the block a static FIR and giving a graph one way to turn
 * the time variation off in place.
 *
 * `delays`, `n_sinusoids` and `seed` draw a new realization and restart the delay line; every other setting is
 * applied to the taps already running, and each oscillator keeps the phase it has reached, so sweeping `max_doppler`
 * or `sample_rate` moves the channel on from the gain it is at instead of stepping it. `powers_db` and `k_factor`
 * scale that gain, which is what those two ask for.
 *
 * The block drops the `freq_est`, `phase_est`, `time_est` and `clock_est` tag keys. The taps change the carrier phase
 * and frequency, and the delays move the symbol timing those keys state.
 */
template<typename T>
requires std::is_same_v<T, std::complex<float>>
struct FadingChannel : gr::Block<FadingChannel<T>, gr::DroppedTagKeys<"freq_est", "phase_est", "time_est", "clock_est">> {
    using Description = Doc<R""(
@brief Rayleigh or Rician time-varying multipath, Clarke sum-of-sinusoids.

Deterministic from `seed`: the arrival angles and phases are drawn once at configure time, so the whole
realization is reproducible and independent of how the stream is chunked. `delays` are in samples.

`k_factor` is the Rician K of the first tap, linear, and `los_doppler_ratio` the line-of-sight Doppler as a fraction of
`max_doppler`. More sinusoids per tap are smoother and cost more.
)"">;

    PortIn<T>  in;
    PortOut<T> out;

    struct Settings {
        Annotated<float, Visible, Unit<"Hz">, Doc<"sample rate">, Limits<Over<0.f>>>           sample_rate       = 1.f;
        Annotated<std::vector<gr::Size_t>, Unit<"samples">, Visible, Doc<"tap delays">>        delays            = std::vector<gr::Size_t>{0U};
        Annotated<std::vector<double>, Unit<"dB">, Visible, Doc<"tap powers">>                 powers_db         = std::vector<double>{0.0};
        Annotated<double, Visible, Unit<"Hz">, Doc<"maximum Doppler (0 = static)">>            max_doppler       = 0.0;
        Annotated<double, Visible, Doc<"first tap K-factor (0 = Rayleigh)">, Limits<Min<0.0>>> k_factor          = 0.0;
        Annotated<double, Visible, Doc<"line-of-sight Doppler ratio">>                         los_doppler_ratio = 0.7;
        Annotated<gr::Size_t, Visible, Doc<"sinusoids per tap">, Limits<Min<4U>>>              n_sinusoids       = 16U;
        Annotated<bool, Visible, Doc<"unit power normalization">>                              normalize         = true;
        Annotated<std::uint64_t, Visible, Doc<"random seed (0 = fixed default)">, StoppedOnly> seed              = 0ULL;

        GR_MAKE_REFLECTABLE(Settings, sample_rate, delays, powers_db, max_doppler, k_factor, los_doppler_ratio, n_sinusoids, normalize, seed);
    } settings;

    GR_MAKE_REFLECTABLE(FadingChannel, in, out, settings);

    struct Tap {
        std::size_t         delay{0UZ};
        double              amplitude{0.};    ///< sqrt of the tap's normalized mean power
        double              diffuseScale{1.}; ///< sqrt(1/(K+1)) on tap 0 under Rician, 1 elsewhere
        double              losScale{0.};     ///< sqrt(K/(K+1)) on tap 0 under Rician, 0 elsewhere
        std::vector<double> theta;            ///< per-sinusoid arrival angle, kept so that a retune can re-derive omega
        std::vector<double> omega;            ///< per-sinusoid phase increment, rad/sample
        std::vector<double> psi;              ///< per-sinusoid phase at sample 0, which a phasor restore reads
        /// The per-sinusoid unit phasor and the constant it is multiplied by each sample. Advancing by multiply costs
        /// one complex product where a phase and `std::polar` cost a sine and a cosine.
        std::vector<std::complex<double>> phasor;
        std::vector<std::complex<double>> rotator;
        double                            losOmega{0.};
        double                            losPsi{0.};
        std::complex<double>              losPhasor{1., 0.};
        std::complex<double>              losRotator{1., 0.};
    };

    std::vector<Tap> _taps{};
    /// A ring, newest at `_cursor`, sized to the longest delay: the line is stepped by moving the cursor rather than
    /// by moving every sample in it.
    std::vector<T> _history{};
    std::size_t    _cursor{0UZ};
    std::uint64_t  _position{0ULL}; ///< samples since the last rebuild; the re-seed cadence is keyed on it

    /// Samples between restorations of the phasors from exact phase. Repeated multiplication drifts in magnitude and
    /// angle; restoring from `psi + omega*k` on a fixed absolute cadence bounds both, and keys the correction to the
    /// stream position rather than to the call boundaries, so the realization stays independent of the chunking.
    static constexpr std::uint64_t kPhasorRestoreInterval = 256ULL;

    void start() { rebuild(); }

    /// Every refusal that does not depend on what was drawn, so a retune refuses what a rebuild refuses.
    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const std::vector<gr::Size_t> tapDelays = s.delays.value;
        if (tapDelays.empty()) {
            return gr::Refused(s.delays, "required");
        }
        if (std::ranges::adjacent_find(tapDelays, std::ranges::greater_equal{}) != tapDelays.end()) {
            return gr::Refused(s.delays, "must be strictly ascending");
        }
        if (s.powers_db.value.size() != tapDelays.size()) {
            return gr::Refused(s.powers_db, "must have the same length as delays");
        }
        return {};
    }

    // a reseed starts a new fading realization
    void applySettings(const Settings& s) {
        // Only a key that changes what is drawn, or how many draws there are, restarts the realization. Every other
        // physical parameter is retuned onto the taps already running, which leaves the delay line, the stream
        // position and the phase every oscillator has reached where they were.
        if (s.seed.changed() || s.delays.changed() || s.n_sinusoids.changed()) {
            rebuild();
        } else {
            retune();
        }
    }

    /// The tap gains at the current instant, for QA and for a consumer that wants to see the channel.
    [[nodiscard]] std::vector<std::complex<double>> currentGains() const {
        std::vector<std::complex<double>> gains(_taps.size());
        for (std::size_t m = 0UZ; m < _taps.size(); ++m) {
            gains[m] = tapGain(_taps[m]);
        }
        return gains;
    }

    [[nodiscard]] work::Status processBulk(std::span<const T> input, std::span<T> output) {
        const std::size_t  nSamples = std::min(input.size(), output.size());
        const std::span<T> written  = output.first(nSamples);

        const std::size_t span = _history.size();
        for (std::size_t k = 0UZ; k < nSamples; ++k) {
            _cursor           = _cursor + 1UZ == span ? 0UZ : _cursor + 1UZ;
            _history[_cursor] = input[k];

            std::complex<double> accumulated{0., 0.};
            for (const Tap& tap : _taps) {
                const T delayed = _history[_cursor >= tap.delay ? _cursor - tap.delay : _cursor + span - tap.delay];
                accumulated += tapGain(tap) * std::complex<double>(delayed);
            }
            written[k] = T(static_cast<float>(accumulated.real()), static_cast<float>(accumulated.imag()));

            advance();
        }
        return work::Status::OK;
    }

private:
    [[nodiscard]] std::complex<double> tapGain(const Tap& tap) const noexcept {
        std::complex<double> diffuse{0., 0.};
        for (const std::complex<double>& phasor : tap.phasor) {
            diffuse += phasor;
        }
        diffuse /= std::sqrt(static_cast<double>(tap.phasor.size()));
        std::complex<double> gain = diffuse * tap.diffuseScale;
        if (tap.losScale != 0.) {
            gain += tap.losScale * tap.losPhasor;
        }
        return gain * tap.amplitude;
    }

    void advance() noexcept {
        ++_position;
        if (_position % kPhasorRestoreInterval == 0ULL) {
            restorePhasors();
            return;
        }
        for (Tap& tap : _taps) {
            for (std::size_t i = 0UZ; i < tap.phasor.size(); ++i) {
                tap.phasor[i] *= tap.rotator[i];
            }
            tap.losPhasor *= tap.losRotator;
        }
    }

    /// Restores every phasor to the unit vector at its exact phase for the current stream position.
    void restorePhasors() noexcept {
        constexpr double twoPi = 2. * std::numbers::pi_v<double>;
        const double     k     = static_cast<double>(_position);
        for (Tap& tap : _taps) {
            for (std::size_t i = 0UZ; i < tap.phasor.size(); ++i) {
                tap.phasor[i] = std::polar(1., std::remainder(tap.psi[i] + tap.omega[i] * k, twoPi));
            }
            tap.losPhasor = std::polar(1., std::remainder(tap.losPsi + tap.losOmega * k, twoPi));
        }
    }

    /// Draws a realization: the arrival angles and the phases at sample 0, and the delay line they run in.
    void rebuild() {
        const double twoPi = 2. * std::numbers::pi_v<double>;

        gr::rng::Xoshiro256pp rng(settings.seed.value);
        _taps.assign(settings.delays.value.size(), Tap{});
        for (std::size_t m = 0UZ; m < _taps.size(); ++m) {
            Tap& tap  = _taps[m];
            tap.delay = static_cast<std::size_t>(settings.delays.value[m]);

            tap.theta.resize(settings.n_sinusoids.value);
            tap.omega.resize(settings.n_sinusoids.value);
            tap.psi.resize(settings.n_sinusoids.value);
            tap.phasor.resize(settings.n_sinusoids.value);
            tap.rotator.resize(settings.n_sinusoids.value);
            for (std::size_t i = 0UZ; i < static_cast<std::size_t>(settings.n_sinusoids.value); ++i) {
                // arrival angles uniform on the circle: Clarke's isotropic scattering assumption
                tap.theta[i] = twoPi * rng.uniform01<double>();
                tap.psi[i]   = twoPi * rng.uniform01<double>() - std::numbers::pi_v<double>;
            }
            tap.losPsi = 0.;
        }

        const std::size_t span = static_cast<std::size_t>(settings.delays.value.back()) + 1UZ;
        _history.assign(span, T{});
        _cursor   = span - 1UZ; // the next write wraps to 0, so an empty line reads as all zeros
        _position = 0ULL;
        retune();
    }

    /// Everything the settings fix without a new draw: the tap powers, the Doppler rates, and the phasors at the
    /// stream position already reached. The arrival angles stay as they were drawn. Each phase at sample 0 absorbs
    /// the phase its sinusoid accumulated at the old rate, so that `psi + omega*k` at the current position k is
    /// still the angle the sinusoid points at: the gain does not step at the update, and every later restore agrees
    /// with the phasor carried forward at the new rate. A line of sight that a `k_factor` change turns on
    /// mid-stream had no rate to carry, so its phasor starts at phase 0 at that position.
    void retune() {
        double totalPower = 0.;
        for (const double db : settings.powers_db.value) {
            totalPower += std::pow(10., db / 10.);
        }
        const double scale = (settings.normalize && totalPower > 0.) ? 1. / totalPower : 1.;

        const double fd    = settings.max_doppler;
        const double fs    = static_cast<double>(settings.sample_rate);
        const double twoPi = 2. * std::numbers::pi_v<double>;

        // The phase at sample 0 that leaves a sinusoid pointing where it points now, once it turns at the new rate:
        // psi + (omega_old - omega_new)*k, so psi_new + omega_new*k == psi_old + omega_old*k. At k = 0 it is the
        // phase that was drawn.
        const double position = static_cast<double>(_position);
        const auto   carried  = [position, twoPi](double psi, double omegaOld, double omegaNew) noexcept { return std::remainder(psi + (omegaOld - omegaNew) * position, twoPi); };

        for (std::size_t m = 0UZ; m < _taps.size(); ++m) {
            Tap& tap      = _taps[m];
            tap.amplitude = std::sqrt(std::pow(10., settings.powers_db.value[m] / 10.) * scale);
            for (std::size_t i = 0UZ; i < tap.theta.size(); ++i) {
                const double omega = twoPi * fd * std::cos(tap.theta[i]) / fs;
                tap.psi[i]         = carried(tap.psi[i], tap.omega[i], omega);
                tap.omega[i]       = omega;
                tap.rotator[i]     = std::polar(1., omega);
            }

            double losOmega = 0.;
            if (m == 0UZ && settings.k_factor > 0.0) {
                tap.diffuseScale = std::sqrt(1. / (1. + settings.k_factor));
                tap.losScale     = std::sqrt(settings.k_factor / (1. + settings.k_factor));
                losOmega         = twoPi * fd * settings.los_doppler_ratio / fs;
            } else {
                tap.diffuseScale = 1.;
                tap.losScale     = 0.;
            }
            tap.losPsi     = carried(tap.losPsi, tap.losOmega, losOmega);
            tap.losOmega   = losOmega;
            tap.losRotator = std::polar(1., losOmega);
        }
        restorePhasors();
    }
};

} // namespace gr::blocks::channel

#endif // GNURADIO_CHANNEL_MULTIPATH_HPP
