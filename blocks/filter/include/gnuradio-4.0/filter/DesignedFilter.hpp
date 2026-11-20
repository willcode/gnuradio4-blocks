#ifndef GNURADIO_DESIGNED_FILTER_HPP
#define GNURADIO_DESIGNED_FILTER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <concepts>
#include <cstdint>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>

#include <gnuradio-4.0/filter/FirFilter.hpp>

namespace gr::blocks::filter {

/// @brief What DesignedFilter designs. The first six take real taps, the two complex ones complex taps.
enum class FilterProfile : std::uint8_t { lowpass, highpass, bandpass, bandstop, root_raised_cosine, hilbert, complex_bandpass, complex_bandstop };

GR_REGISTER_BLOCK(gr::blocks::filter::DesignedFilter, ([T], [U]), [ float, std::complex<float> ], [ float, std::complex<float> ])

template<typename TSample, typename TTap = float>
requires((std::same_as<TSample, float> || std::same_as<TSample, std::complex<float>>) && (std::same_as<TTap, float> || std::same_as<TTap, std::complex<float>>))
struct DesignedFilter : Block<DesignedFilter<TSample, TTap>, Resampling<1UZ, 1UZ, false>>, detail::FirFilterCore<DesignedFilter<TSample, TTap>, TSample, TTap> {
    using TParent     = Block<DesignedFilter<TSample, TTap>, Resampling<1UZ, 1UZ, false>>;
    using TCore       = detail::FirFilterCore<DesignedFilter<TSample, TTap>, TSample, TTap>;
    using TOut        = typename TCore::TOut;
    using Description = Doc<R""(
@brief A FIR filter whose taps are designed at runtime from a stated specification.

Parameters in, taps out: the block owns a design specification as settings - profile, -6 dB cutoffs in hertz, a
transition width or an explicit odd length, a window or a Kaiser attenuation, gain, decimation - designs its taps in
its settings hook through `gr::filter::fir::design`, and filters with the same machinery the raw-taps FirFilter runs. Every
design-affecting setting is live: a redesign keeps the input/output alignment exactly, and a `decimation` change moves
the phase origin. `taps` requests a length, not a vector: the design rounds it up to odd, and to three taps or to five
for a band form, and `designed_taps` reports the length designed.

Each tag moves with all its keys by the design's delay `d = (designed_taps - 1) / 2`. A tag on input `i` leaves on
output `round((i + d) / decimation)`. A tag keeps the output it was given when it crossed, through any later redesign or
decimation change. A tag whose output lies past the stream's last output leaves at the end-of-stream index, one past
that output.

The tap type states which family the block designs: real taps carry `lowpass`, `highpass`, `bandpass`, `bandstop`,
`root_raised_cosine` and `hilbert`; complex taps carry `complex_bandpass` and `complex_bandstop`. A profile outside the
tap type's family is refused, naming the taps it requires. `root_raised_cosine` requires an explicit `taps` length and
a positive `symbol_rate` - no transition-width estimate exists for it.

`hilbert` is the analytic-signal branch: antisymmetric type III taps, structurally zero at DC and at Nyquist, rotating
every positive frequency by -90 degrees. An analytic signal is this output paired with the input delayed by the group
delay `(designed_taps - 1) / 2`, which is an integer because the length is odd. It requires an explicit `taps` and reads
no band edge - a Hilbert transformer covers the whole open band and how much of it is usable follows from the length
alone, which is what `gr::filter::fir::design::scanHilbert` measures. `cutoff` is therefore not read under this profile.

`BasicFilter` is the module's other design-from-parameters filter and the one to reach for where an IIR response
or a named window over a fixed length is enough; `FirFilter` is the one to reach for where the taps are already in
hand. This block is the FIR-only third: an arbitrary length, complex taps, and the Hilbert and root-raised-cosine
profiles neither of the others offers.

A graph may couple `sample_rate` to `gr:sample_rate`. A tag that changes it then redesigns the filter. A passing rate
tag is forwarded divided by the decimation, so downstream reads the rate of the stream this block hands it. A refused
settings change leaves the previous design running.

`sample_rate` can follow gr:sample_rate. `cutoff` is the -6 dB point, the lower edge of a band form, and `high_cutoff`
the upper edge. `transition_width` applies to every edge and sets the length unless `taps` states it. `attenuation_db`
shapes a Kaiser window, which takes its beta from it unless `window_param` states one; a negative `window_param` selects
each window's own default. `symbol_rate` and `alpha` serve root_raised_cosine.
)"">;

    PortIn<TSample> in;
    PortOut<TOut>   out;

    struct Settings {
        Annotated<FilterProfile, Visible, Doc<"filter profile">>                     profile          = FilterProfile::lowpass;
        Annotated<float, Unit<"Hz">, Visible, Doc<"sample rate">, Limits<Over<0.f>>> sample_rate      = 1.0f;
        Annotated<double, Unit<"Hz">, Visible, Doc<"cutoff">>                        cutoff           = 0.0;
        Annotated<double, Unit<"Hz">, Visible, Doc<"upper cutoff">>                  high_cutoff      = 0.0;
        Annotated<double, Unit<"Hz">, Visible, Doc<"transition width">>              transition_width = 0.0;
        Annotated<double, Doc<"gain">>                                               gain             = 1.0;
        Annotated<double, Unit<"dB">, Doc<"Kaiser stopband attenuation">>            attenuation_db   = 60.0;
        Annotated<gr::algorithm::window::Type, Doc<"window">>                        window           = gr::algorithm::window::Type::kaiser;
        Annotated<double, Doc<"window parameter (< 0 = default)">>                   window_param     = -1.0;
        Annotated<gr::Size_t, Doc<"requested length (0 = derived)">>                 taps             = 0U;
        Annotated<double, Unit<"Hz">, Doc<"symbol rate">>                            symbol_rate      = 0.0;
        Annotated<double, Doc<"excess bandwidth">>                                   alpha            = 0.35;
        Annotated<gr::Size_t, Doc<"decimation">, Visible, Limits<Min<1U>>>           decimation       = 1U;

        GR_MAKE_REFLECTABLE(Settings, profile, sample_rate, cutoff, high_cutoff, transition_width, gain, attenuation_db, window, window_param, taps, symbol_rate, alpha, decimation);
    } settings;

    struct Status {
        Annotated<gr::Size_t, Doc<"designed length">> designed_taps = 0U;

        GR_MAKE_REFLECTABLE(Status, designed_taps);
    } status;

    GR_MAKE_REFLECTABLE(DesignedFilter, in, out, settings, status);

    std::vector<TTap> _designed;

    // a design takes well under a millisecond at common lengths, so the check designs and the library refuses the band
    // edges it cannot meet
    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const FilterProfile which = s.profile.value;
        if (std::same_as<TTap, float> && isComplexProfile(which)) {
            return gr::Refused(s.profile, std::format("{} requires complex taps", gr::meta::enumName(which).value_or("")));
        }
        if (!std::same_as<TTap, float> && !isComplexProfile(which)) {
            return gr::Refused(s.profile, std::format("{} requires real taps", gr::meta::enumName(which).value_or("")));
        }
        if ((which == FilterProfile::root_raised_cosine || which == FilterProfile::hilbert) && s.taps.value == 0U) {
            return gr::Refused(s.taps, "required");
        }
        if (which == FilterProfile::root_raised_cosine && s.symbol_rate.value <= 0.0) {
            return gr::Refused(s.symbol_rate, "must be > 0");
        }
        std::ignore = designedTaps(s);
        return {};
    }

    void applySettings(const Settings& s) {
        const bool live = this->coreLive();
        // everything except the decimation is design-affecting, and a redesign replays the alignment
        const bool redesign = s.profile.changed() || s.sample_rate.changed() || s.cutoff.changed() || s.high_cutoff.changed() || s.transition_width.changed() || s.gain.changed() || s.attenuation_db.changed() || s.window.changed() || s.window_param.changed() || s.taps.changed() || s.symbol_rate.changed() || s.alpha.changed();
        if (redesign) {
            _designed            = designedTaps(s);
            status.designed_taps = static_cast<gr::Size_t>(_designed.size());
        }
        if (redesign || s.decimation.changed()) {
            this->coreRebuild(std::span<const TTap>(_designed), s.decimation, live);
        }
        if (live && s.decimation.changed()) {
            this->coreMarkReorigin();
        }
    }

    void start() { this->coreStart(std::span<const TTap>(_designed), settings.decimation); }

    /// @brief The delay every forwarded tag moves by, in input samples. Every profile designs a symmetric or
    /// antisymmetric set.
    [[nodiscard]] double groupDelaySamples() const noexcept { return 0.5 * static_cast<double>(_designed.size() - 1UZ); }

private:
    [[nodiscard]] static bool isComplexProfile(FilterProfile which) { return which == FilterProfile::complex_bandpass || which == FilterProfile::complex_bandstop; }

    /// @brief The taps the settings @p given design, after the check's rules; a band the library cannot design throws.
    [[nodiscard]] std::vector<TTap> designedTaps(const Settings& given) const {
        // every window that reads the parameter refuses a negative one: a negative value is free to mean "derive it"
        const double                        windowParam = given.window_param.value < 0.0 ? std::numeric_limits<double>::quiet_NaN() : given.window_param.value;
        gr::filter::fir::design::FilterSpec spec{
            .sampleRate      = static_cast<double>(given.sample_rate.value),
            .cutoff          = given.cutoff.value,
            .highCutoff      = given.high_cutoff.value,
            .transitionWidth = given.transition_width.value,
            .gain            = given.gain.value,
            .attenuationDb   = given.attenuation_db.value,
            .window          = given.window.value,
            .windowParam     = windowParam,
            .taps            = static_cast<std::size_t>(given.taps.value),
        };

        const FilterProfile which = given.profile.value;
        if constexpr (std::same_as<TTap, float>) {
            switch (which) {
            case FilterProfile::lowpass: return gr::filter::fir::design::designLowpass(spec);
            case FilterProfile::highpass: return gr::filter::fir::design::designHighpass(spec);
            case FilterProfile::bandpass: return gr::filter::fir::design::designBandpass(spec);
            case FilterProfile::bandstop: return gr::filter::fir::design::designBandstop(spec);
            case FilterProfile::root_raised_cosine: return gr::filter::fir::design::designRootRaisedCosine(spec.taps, spec.sampleRate, given.symbol_rate.value, given.alpha.value, spec.gain);
            case FilterProfile::hilbert: return gr::filter::fir::design::hilbert(spec.taps, gr::filter::fir::design::windowOf(spec), spec.gain);
            case FilterProfile::complex_bandpass:
            case FilterProfile::complex_bandstop: break; // the check admits no complex profile with real taps
            }
            return {};
        } else {
            if (which == FilterProfile::complex_bandpass) {
                return gr::filter::fir::design::designComplexBandpass(spec);
            }
            return gr::filter::fir::design::designComplexBandstop(spec); // the check admits no real profile with complex taps
        }
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_DESIGNED_FILTER_HPP
