#ifndef GNURADIO_CHANNELIZER_HPP
#define GNURADIO_CHANNELIZER_HPP

#include <algorithm>
#include <complex>
#include <concepts>
#include <cstddef>
#include <expected>
#include <format>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/algorithm/filter/PolyphaseChannelizer.hpp>

namespace gr::blocks::filter {

namespace detail {

/// The widest bank accepted.
inline constexpr gr::Size_t kMaxChannels = static_cast<gr::Size_t>(gr::filter::PolyphaseChannelizer<float>::kMaxChannels);

/// The longest root-Nyquist span accepted, in channel spacings.
inline constexpr std::size_t kMaxSpan = 64UZ;

/// @brief True for a connected port.
inline constexpr auto kIsConnected = [](const auto& port) { return port.isConnected(); };

/// The settings a bank is built from.
struct ChannelizerSettings {
    std::size_t                      channels = 0UZ;
    std::vector<float>               taps{};
    double                           attenuationDb = 0.;
    double                           transition    = 0.;
    gr::filter::ChannelizerPrototype prototype     = gr::filter::ChannelizerPrototype::root_nyquist;
    std::size_t                      span          = 0UZ;
    std::size_t                      oversample    = 0UZ;
};

/// @brief The bank's part of a block's settings @p s.
template<typename TSettings>
[[nodiscard]] ChannelizerSettings channelizerSettings(const TSettings& s) {
    return {.channels = s.n_channels.value, .taps = s.taps.value, .attenuationDb = s.attenuation_db.value, .transition = s.transition.value, .prototype = s.prototype.value, .span = s.span.value, .oversample = s.oversample.value};
}

/// The setting a refused design fails: the one the prototype comes from.
template<typename TSettings>
[[nodiscard]] std::string_view designKey(const TSettings& s) {
    return s.taps->empty() ? "attenuation_db" : "taps";
}

/// @brief The rules both banks state. A design can take a second at 256 channels, so the apply hook designs and
/// fails a prototype no design meets.
template<typename TSettings>
[[nodiscard]] std::optional<gr::Refused> checkChannelizer(const TSettings& s) {
    if (s.prototype.value == gr::filter::ChannelizerPrototype::root_nyquist && (s.span.value < 2U || s.span.value > kMaxSpan)) {
        return gr::Refused(s.span, "must be >= 2 and <= 64 when prototype is root_nyquist");
    }
    if (s.oversample.value == 2U && s.n_channels.value % 2U != 0U) {
        return gr::Refused(s.n_channels, "must be even when oversample is 2");
    }
    return {};
}

/// @brief Designs the prototype @p settings name, or measures the supplied one, with its stopband and ripple.
[[nodiscard]] inline std::expected<gr::filter::ChannelizerDesign, gr::Error> channelizerPrototype(const ChannelizerSettings& settings) { return settings.taps.empty() ? gr::filter::designChannelizerPrototype(settings.channels, settings.prototype, settings.attenuationDb, settings.transition, settings.span, settings.oversample) : gr::filter::measureChannelizerPrototype(settings.taps, settings.channels, settings.transition, settings.oversample); }

/// @brief Divides each `gr:sample_rate` value of @p tag by @p stride, float or double. Returns the divided entries.
[[nodiscard]] inline property_map divideRate(property_map& tag, std::size_t stride) {
    property_map rates;
    for (auto& [key, value] : tag) {
        if (std::string_view(key) != gr::tag::SAMPLE_RATE.key()) {
            continue;
        }
        if (const float* rate = value.get_if<float>(); rate != nullptr) {
            value = *rate / static_cast<float>(stride);
        } else if (const double* wide = value.get_if<double>(); wide != nullptr) {
            value = *wide / static_cast<double>(stride);
        } else {
            continue;
        }
        rates.insert_or_assign(key, value);
    }
    return rates;
}

/// The setting labels both banks carry.
using ChannelCountDoc = Doc<"channel count">;
using TapsDoc         = Doc<"prototype taps (empty = designed)">;
using DesignedTapsDoc = Doc<"prototype in force">;
using StopbandDoc     = Doc<"worst stopband level">;
using RippleDoc       = Doc<"passband ripple">;
using DesignOkDoc     = Doc<"lowpass target met">;
using AttenuationDoc  = Doc<"lowpass stopband target">;
using TransitionDoc   = Doc<"excess bandwidth">;
using PrototypeDoc    = Doc<"prototype family">;
using SpanDoc         = Doc<"root_nyquist span">;
using OversampleDoc   = Doc<"oversampling factor">;

/// The settings text both banks carry.
inline constexpr gr::meta::fixed_string kChannelizerSettingsText = R""(

`n_channels` is M, from 2 to 256, and is fixed once a port is connected. When `taps` is empty, the bank designs a
prototype in the family `prototype` names. A prototype in `taps` runs in place of the design and stays as supplied.

`root_nyquist`, the default, is a root-raised cosine of rolloff `transition` that covers `span` channel spacings, 2 to
64. A longer span rejects more. Its squared responses sum flat across the bank. `lowpass` is a Kaiser lowpass cut off
at half a channel spacing. Its passband ends at (1 - `transition`)/2 channel spacings. Its stopband starts at the alias
edge, `oversample` channel spacings less the pass edge. The design takes the shortest length that rejects the stopband
by `attenuation_db` with at most 0.1 dB of passband ripple. `boxcar` is M taps of 1/M, and the bank becomes a plain
block transform. `span` applies to `root_nyquist` alone and `attenuation_db` to `lowpass` alone.

`oversample` 1 advances the commutator M samples per step. `oversample` 2 advances it M/2 samples and needs an even M.
`lowpass` isolates the most and does not reconstruct. `boxcar` reconstructs at either `oversample` and isolates little.
`root_nyquist` with `oversample` 2 does both.

A change to any of these settings designs the prototype again, rebuilds the bank and clears its history. A refused
change leaves the bank as it was.

`designed_taps` holds the prototype that runs, zero-padded to a whole number of branches. `stopband_db` is the
prototype's worst level from the alias edge to Nyquist. That band folds into each channel when the bank decimates.
`ripple_db` is the passband ripple. Both figures are measured before the padding, relative to the DC gain.
`design_ok` is false only when a `lowpass` design misses `attenuation_db` or the ripple limit.
)"";

/// @brief `Head` followed by `Tail` in one text. The `+` operators of `gr::meta::fixed_string` build their result
/// through a constructor that folds over every character, and Clang refuses a fold of more than 2048.
template<gr::meta::fixed_string Head, gr::meta::fixed_string Tail>
[[nodiscard]] consteval auto joinedText() {
    gr::meta::fixed_string<Head.size() + Tail.size()> text{};
    for (std::size_t i = 0UZ; i < Head.size(); ++i) {
        text[i] = Head[i];
    }
    for (std::size_t i = 0UZ; i < Tail.size(); ++i) {
        text[Head.size() + i] = Tail[i];
    }
    return text;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::filter::PolyphaseChannelizer, [T], [float])

template<std::floating_point F>
struct PolyphaseChannelizer : Block<PolyphaseChannelizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<detail::joinedText<R""(
@brief Splits one complex stream into `n_channels` equally spaced channels with a polyphase analysis filter bank.

Channel `k` carries the input's band around `k * fs / M`, shifted to baseband and decimated. Indices above `M/2` are
the negative frequencies in the usual transform order. Each output runs at `fs/M`, or twice that when the bank is
oversampled. Each step consumes `M / oversample` input samples and writes one sample to every channel.

An input tag lands on channel zero, on the output of the step that consumes its sample. A `gr:sample_rate` key is
divided by the commutator stride and reaches every channel. The keys the bank holds as settings are dropped. At end of
stream the samples short of a whole step are dropped, and their tags land on channel zero at the end of the stream.)"",
        detail::kChannelizerSettingsText>()>;

    PortIn<std::complex<F>>               in;
    std::vector<PortOut<std::complex<F>>> out;

    struct Settings {
        Annotated<gr::Size_t, Visible, detail::ChannelCountDoc, Limits<Min<gr::Size_t{2U}>, Max<detail::kMaxChannels>>, PortCountFor<"out">> n_channels = 4U;
        Annotated<std::vector<float>, detail::TapsDoc>                                                                                       taps{};
        Annotated<double, Unit<"dB">, detail::AttenuationDoc, Limits<Over<0.0>>>                                                             attenuation_db = 60.0;
        Annotated<double, detail::TransitionDoc, Limits<Over<0.0>, Under<1.0>>>                                                              transition     = 0.5;
        Annotated<gr::filter::ChannelizerPrototype, Visible, detail::PrototypeDoc>                                                           prototype      = gr::filter::ChannelizerPrototype::root_nyquist;
        Annotated<gr::Size_t, detail::SpanDoc>                                                                                               span           = 16U;
        Annotated<gr::Size_t, Visible, detail::OversampleDoc, Limits<Min<gr::Size_t{1U}>, Max<gr::Size_t{2U}>>>                              oversample     = 1U;

        GR_MAKE_REFLECTABLE(Settings, n_channels, taps, attenuation_db, transition, prototype, span, oversample);
    } settings;

    struct Status {
        Annotated<std::vector<float>, detail::DesignedTapsDoc> designed_taps{};
        Annotated<double, Unit<"dB">, detail::StopbandDoc>     stopband_db{};
        Annotated<double, Unit<"dB">, detail::RippleDoc>       ripple_db{};
        Annotated<bool, detail::DesignOkDoc>                   design_ok{};

        GR_MAKE_REFLECTABLE(Status, designed_taps, stopband_db, ripple_db, design_ok);
    } status;

    GR_MAKE_REFLECTABLE(PolyphaseChannelizer, in, out, settings, status);

    gr::filter::PolyphaseChannelizer<F> _bank{};
    std::vector<std::complex<F>>        _step{};
    std::size_t                         _channels = 4UZ;
    std::size_t                         _stride   = 4UZ;

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkChannelizer(s); }

    void applySettings(const Settings& s) {
        const bool rebuild = s.n_channels.changed() || s.taps.changed() || s.attenuation_db.changed() || s.transition.changed() || s.prototype.changed() || s.span.changed() || s.oversample.changed();
        if (!rebuild) {
            return;
        }
        // a connected port cannot be added or removed, so the count stays while any port is connected
        if (s.n_channels != _channels && std::ranges::any_of(out, detail::kIsConnected)) {
            this->fail(s.n_channels, "requires unconnected outputs");
            return;
        }
        const std::expected<gr::filter::ChannelizerDesign, gr::Error> measured = install(detail::channelizerSettings(s));
        if (!measured) { // the design or the bank refuses what only building it shows
            this->fail(detail::designKey(s), measured.error().message);
            return;
        }
        status.designed_taps = std::vector<float>(_bank.prototype().begin(), _bank.prototype().end());
        status.stopband_db   = measured->stopbandDb;
        status.ripple_db     = measured->rippleDb;
        status.design_ok     = measured->ok;
    }

    void start() { _bank.reset(); }

    /// @brief Installs an analysis bank for @p next with its output ports, and returns the design for the status values.
    /// Changes nothing on a refusal.
    [[nodiscard]] std::expected<gr::filter::ChannelizerDesign, gr::Error> install(detail::ChannelizerSettings next) {
        std::expected<gr::filter::ChannelizerDesign, gr::Error> measured = detail::channelizerPrototype(next);
        if (!measured) {
            return measured;
        }
        gr::filter::PolyphaseChannelizer<F>  bank;
        const std::expected<void, gr::Error> configured = bank.configure(next.channels, measured->taps, next.oversample);
        if (!configured) {
            return std::unexpected(configured.error());
        }

        _bank     = std::move(bank);
        _channels = _bank.channels();
        _stride   = _bank.stride();
        _step.assign(_channels, std::complex<F>{});
        out.resize(_channels);

        this->input_chunk_size  = static_cast<gr::Size_t>(_stride);
        this->output_chunk_size = 1U;
        return measured;
    }

    template<typename TOutputSpanType>
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, std::span<TOutputSpanType>& outputs) {
        const std::size_t stride   = _stride;
        const std::size_t channels = _channels;
        std::size_t       nSteps   = inSpan.size() / stride;
        for (const auto& port : outputs) {
            nSteps = std::min(nSteps, port.size());
        }

        for (std::size_t s = 0UZ; s < nSteps; ++s) {
            _bank.step(std::span<const std::complex<F>>(inSpan).subspan(s * stride, stride), _step);
            for (std::size_t c = 0UZ; c < channels; ++c) {
                outputs[c][s] = _step[c];
            }
        }

        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex < 0) {
                continue;
            }
            const std::size_t at = static_cast<std::size_t>(relIndex) / stride;
            if (at < nSteps) {
                forwardTag(tagMap.get(), outputs, at);
            }
        }

        std::ignore = inSpan.consume(nSteps * stride);
        for (auto& port : outputs) {
            port.publish(nSteps);
        }
        return work::Status::OK;
    }

    /// @brief At end of stream, forwards the tags of the samples short of a whole step to the end-of-stream index.
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t processedIn) {
        auto& outputs = std::get<0>(outputSpans);
        if (processedIn > 0UZ || std::ranges::any_of(outputs, [](const auto& output) { return output.size() > 0UZ; })) {
            return; // processBulk forwards the tags of the steps it takes
        }
        const std::pmr::string endOfStream = static_cast<std::pmr::string>(gr::tag::END_OF_STREAM);
        const auto&            input       = std::get<0>(inputSpans);
        for (const Tag& remaining : input.rawTags) {
            if (remaining.index < input.streamIndex) {
                continue;
            }
            property_map tag = remaining.map;
            tag.erase(endOfStream);
            forwardTag(std::move(tag), outputs, 0UZ);
        }
    }

    /// @brief Publishes @p tag at @p offset on channel zero, less the bank's setting keys, and its divided
    /// `gr:sample_rate` on every other channel.
    template<typename TOutputs>
    void forwardTag(property_map tag, TOutputs& outputs, std::size_t offset) {
        if (outputs.empty()) {
            return;
        }
        const std::set<std::string> owned = this->writableMembers();
        std::erase_if(tag, [&owned](const auto& entry) { return owned.contains(std::string(entry.first)); });
        const property_map rates = detail::divideRate(tag, _stride);
        if (!tag.empty()) {
            outputs[0].publishTag(std::move(tag), offset);
        }
        if (!rates.empty()) {
            for (std::size_t c = 1UZ; c < outputs.size(); ++c) {
                outputs[c].publishTag(rates, offset);
            }
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::filter::PolyphaseSynthesizer, [T], [float])

template<std::floating_point F>
struct PolyphaseSynthesizer : Block<PolyphaseSynthesizer<F>, NoTagPropagation, Resampling<1UZ, 1UZ, false>> {
    using Description = Doc<detail::joinedText<R""(
@brief Combines `n_channels` equally spaced channels into one complex stream with a polyphase synthesis filter bank.

It is the transpose of the analysis bank, with the same settings and channel numbering. Channel `k` is placed at
`k * fs / M` of the output rate. Each step takes one sample per channel and writes `M / oversample` output samples.

Fed the channels of an analysis bank with the same settings, it returns that bank's input delayed by `((N-1)/S)*S`
samples and scaled by the cascade's gain. N is the length of `designed_taps`, and S is M / `oversample`. `boxcar`
reconstructs at either `oversample`, and `root_nyquist` with `oversample` 2. A `lowpass` prototype leaves a residual,
and so does a critically sampled `root_nyquist` pair.

Tags on the channel inputs are dropped, and the output carries no `gr:sample_rate`.)"",
        detail::kChannelizerSettingsText>()>;

    std::vector<PortIn<std::complex<F>>> in;
    PortOut<std::complex<F>>             out;

    struct Settings {
        Annotated<gr::Size_t, Visible, detail::ChannelCountDoc, Limits<Min<gr::Size_t{2U}>, Max<detail::kMaxChannels>>, PortCountFor<"in">> n_channels = 4U;
        Annotated<std::vector<float>, detail::TapsDoc>                                                                                      taps{};
        Annotated<double, Unit<"dB">, detail::AttenuationDoc, Limits<Over<0.0>>>                                                            attenuation_db = 60.0;
        Annotated<double, detail::TransitionDoc, Limits<Over<0.0>, Under<1.0>>>                                                             transition     = 0.5;
        Annotated<gr::filter::ChannelizerPrototype, Visible, detail::PrototypeDoc>                                                          prototype      = gr::filter::ChannelizerPrototype::root_nyquist;
        Annotated<gr::Size_t, detail::SpanDoc>                                                                                              span           = 16U;
        Annotated<gr::Size_t, Visible, detail::OversampleDoc, Limits<Min<gr::Size_t{1U}>, Max<gr::Size_t{2U}>>>                             oversample     = 1U;

        GR_MAKE_REFLECTABLE(Settings, n_channels, taps, attenuation_db, transition, prototype, span, oversample);
    } settings;

    struct Status {
        Annotated<std::vector<float>, detail::DesignedTapsDoc> designed_taps{};
        Annotated<double, Unit<"dB">, detail::StopbandDoc>     stopband_db{};
        Annotated<double, Unit<"dB">, detail::RippleDoc>       ripple_db{};
        Annotated<bool, detail::DesignOkDoc>                   design_ok{};

        GR_MAKE_REFLECTABLE(Status, designed_taps, stopband_db, ripple_db, design_ok);
    } status;

    GR_MAKE_REFLECTABLE(PolyphaseSynthesizer, in, out, settings, status);

    gr::filter::PolyphaseSynthesizer<F> _bank{};
    std::vector<std::complex<F>>        _step{};
    std::size_t                         _channels = 4UZ;
    std::size_t                         _stride   = 4UZ;

    std::optional<gr::Refused> checkSettings(const Settings& s) const { return detail::checkChannelizer(s); }

    void applySettings(const Settings& s) {
        const bool rebuild = s.n_channels.changed() || s.taps.changed() || s.attenuation_db.changed() || s.transition.changed() || s.prototype.changed() || s.span.changed() || s.oversample.changed();
        if (!rebuild) {
            return;
        }
        // a port that a graph connected cannot be added or removed, so the count holds while one is connected
        if (s.n_channels != _channels && std::ranges::any_of(in, detail::kIsConnected)) {
            this->fail(s.n_channels, "requires unconnected inputs");
            return;
        }
        const std::expected<gr::filter::ChannelizerDesign, gr::Error> measured = install(detail::channelizerSettings(s));
        if (!measured) { // the design or the bank refuses what only building it shows
            this->fail(detail::designKey(s), measured.error().message);
            return;
        }
        status.designed_taps = std::vector<float>(_bank.prototype().begin(), _bank.prototype().end());
        status.stopband_db   = measured->stopbandDb;
        status.ripple_db     = measured->rippleDb;
        status.design_ok     = measured->ok;
    }

    void start() { _bank.reset(); }

    /// @brief Installs a synthesis bank for @p next with its input ports, and returns the design for the status values.
    /// Changes nothing on a refusal.
    [[nodiscard]] std::expected<gr::filter::ChannelizerDesign, gr::Error> install(detail::ChannelizerSettings next) {
        std::expected<gr::filter::ChannelizerDesign, gr::Error> measured = detail::channelizerPrototype(next);
        if (!measured) {
            return measured;
        }
        gr::filter::PolyphaseSynthesizer<F>  bank;
        const std::expected<void, gr::Error> configured = bank.configure(next.channels, measured->taps, next.oversample);
        if (!configured) {
            return std::unexpected(configured.error());
        }

        _bank     = std::move(bank);
        _channels = _bank.channels();
        _stride   = _bank.stride();
        _step.assign(_channels, std::complex<F>{});
        in.resize(_channels);

        this->input_chunk_size  = 1U;
        this->output_chunk_size = static_cast<gr::Size_t>(_stride);
        return measured;
    }

    template<typename TInputSpanType>
    [[nodiscard]] work::Status processBulk(std::span<TInputSpanType>& inputs, std::span<std::complex<F>> output) {
        const std::size_t stride   = _stride;
        const std::size_t channels = _channels;
        std::size_t       nSteps   = output.size() / stride;
        for (const auto& port : inputs) {
            nSteps = std::min(nSteps, port.size());
        }

        for (std::size_t s = 0UZ; s < nSteps; ++s) {
            for (std::size_t c = 0UZ; c < channels; ++c) {
                _step[c] = inputs[c][s];
            }
            _bank.step(_step, output.subspan(s * stride, stride));
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_CHANNELIZER_HPP
