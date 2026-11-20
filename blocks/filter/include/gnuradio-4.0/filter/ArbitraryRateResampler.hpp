#ifndef GNURADIO_BLOCKS_ARBITRARY_RESAMPLER_HPP
#define GNURADIO_BLOCKS_ARBITRARY_RESAMPLER_HPP

#include <algorithm>
#include <array>
#include <complex>
#include <concepts>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/filter/ArbitraryResampler.hpp>

#include <gnuradio-4.0/filter/TagDelay.hpp>

namespace gr::blocks::filter {

GR_REGISTER_BLOCK(gr::blocks::filter::ArbitraryRateResampler, [T], [ float, std::complex<float> ])

template<typename T>
requires(std::same_as<T, float> || std::same_as<T, std::complex<float>>)
struct ArbitraryRateResampler : Block<ArbitraryRateResampler<T>> {
    using TKernel     = gr::filter::ArbitraryResampler<T>;
    using Description = Doc<R""(
@brief Changes the sample rate by a real number, realized exactly as a stated rational.

A caller whose ratio is a fraction of small terms wants `RationalResampler`, at half the arithmetic and no
interpolation error. What this block buys is a bounded filter: the prototype is designed once for a bank of `L` arms
and its length per arm never follows the ratio's arithmetic.

The rate is a `double` and the phase is fixed-point, so the realized ratio is a rational the block reports through
`realizedRate()`, within `1.2e-10` relative of the request. Lowering `rate` past what the prototype was cut for
rebuilds it; `min_rate` states that floor once and keeps the design search off the settings path. The ports are
`Async`, an arbitrary `double` ratio not being expressible as the framework's constant `L:M`. `rate = 1` is not a
pass-through.

A forwarded `gr:sample_rate` tag is multiplied by `rate`, so downstream reads the rate of the stream this block hands
it.

Each tag moves with all its keys by the prototype's delay `d`. `d` is in samples of the interpolated rate `L*fs_in`. A
tag on input `i` leaves on the output nearest `i*L + d`. A designed or other symmetric prototype of `N` taps delays by
`(N-1)/2`. An asymmetric supplied prototype moves its tags by the centroid of its energy, rounded to the whole
interpolated sample. A tag is placed when its sample is consumed and keeps that output through any later rate change or
rebuild. A tag whose output lies past the stream's last output leaves at the end-of-stream index, one past that output.

Supplied `taps` are the prototype at the interpolated rate L*fs_in and carry their own gain, L included. `rolloff` is
the share of the surviving band spent on the transition, for designed taps only; `attenuation_db` also sizes the bank.
)"">;

    PortIn<T, Async>  in;
    PortOut<T, Async> out;

    struct Settings {
        Annotated<double, Doc<"resampling rate">, Limits<Over<0.0>>>               rate          = 1.0;
        Annotated<double, Doc<"minimum design rate (0 = rate)">>                   min_rate      = 0.0;
        Annotated<gr::Size_t, Doc<"polyphase arm count (0 = derived)">>            bank_size     = 0U;
        Annotated<gr::filter::ArmInterpolation, Doc<"interpolation between arms">> interpolation = gr::filter::ArmInterpolation::linear;
        Annotated<std::vector<float>, Doc<"prototype taps (empty = designed)">>    taps{};
        Annotated<float, Unit<"fraction">, Doc<"transition share">>                rolloff        = 0.2f;
        Annotated<float, Unit<"dB">, Doc<"stopband target">>                       attenuation_db = 60.f;
        Annotated<float, Unit<"dB">, Doc<"passband ripple target">>                max_ripple_db  = 0.1f;

        GR_MAKE_REFLECTABLE(Settings, rate, min_rate, bank_size, interpolation, taps, rolloff, attenuation_db, max_ripple_db);
    } settings;

    GR_MAKE_REFLECTABLE(ArbitraryRateResampler, in, out, settings);

    std::optional<TKernel> _resampler;
    std::size_t            _bankSize    = 1UZ;
    double                 _designedFor = 1.0; /// the `min(1, r)` the prototype was cut for
    std::uint64_t          _inOrigin    = 0ULL;
    std::uint64_t          _outOrigin   = 0ULL;
    std::uint64_t          _stepOrigin  = 0ULL;
    std::int64_t           _phaseOrigin = 0LL;
    std::uint64_t          _twiceDelay  = 0ULL; /// the prototype's delay in half interpolated samples, `twiceTapDelay`
    bool                   _reorigin    = false;
    detail::TagDelayLine   _tags;
    std::uint64_t          _latestHeld = 0ULL; /// the output of the last held tag

    // a design takes a few hundred milliseconds, so the apply hook designs and fails a design no prototype meets
    void applySettings(const Settings& s) {
        const bool live     = _resampler.has_value();
        const bool redesign = s.bank_size.changed() || s.interpolation.changed() || s.taps.changed() || s.rolloff.changed() || s.attenuation_db.changed() || s.max_ripple_db.changed();
        bool       designed = true;
        if (redesign) {
            designed = rebuild();
        } else if (s.rate.changed() || s.min_rate.changed()) {
            designed = retune();
        } else {
            return;
        }
        if (!designed) {
            this->fail(s.attenuation_db, kNoDesign);
            return;
        }
        _reorigin = _reorigin || live;
    }

    void start() {
        if (!rebuild()) {
            throw gr::exception(kNoDesign);
        }
        _tags.reset();
        _latestHeld = 0ULL;
        _inOrigin   = 0ULL;
        _outOrigin  = 0ULL;
        _reorigin   = false;
    }

    /// @brief `L*2^32/step`, the rational the block actually runs at. Exact as a `double` while `step` is below `2^53`.
    [[nodiscard]] double realizedRate() const noexcept { return _resampler->realizedRate(); }
    /// @brief `B`, the taps one arm holds — the wrap term included, so a dot product is this long.
    [[nodiscard]] std::size_t tapsPerArm() const noexcept { return _resampler->tapsPerArm(); }
    [[nodiscard]] std::size_t bankSize() const noexcept { return _bankSize; }
    /// @brief The delay every forwarded tag moves by, in input samples. It is `(N-1)/(2L)` for a symmetric prototype.
    [[nodiscard]] double         groupDelaySamples() const noexcept { return 0.5 * static_cast<double>(_twiceDelay) / static_cast<double>(_bankSize); }
    [[nodiscard]] std::size_t    outputsFor(std::size_t nInput) const noexcept { return _resampler->outputsFor(nInput); }
    [[nodiscard]] std::size_t    inputsFor(std::size_t nOutput) const noexcept { return _resampler->inputsFor(nOutput); }
    [[nodiscard]] const TKernel& kernel() const noexcept { return *_resampler; }

    static constexpr const char* kNoDesign = "must be lower, or max_ripple_db higher, for a prototype under the tap cap";

    /// @brief The prototype for a bank of @p bankSize arms cut for rates down to @p designedFor, or none when no
    /// prototype under the tap cap meets the settings.
    [[nodiscard]] std::optional<std::vector<float>> designedPrototype(std::size_t bankSize, double designedFor) const {
        gr::filter::ResamplerDesign design = gr::filter::designArbitraryResampler(bankSize, designedFor, static_cast<double>(settings.rolloff.value), static_cast<double>(settings.attenuation_db.value), static_cast<double>(settings.max_ripple_db.value));
        if (!design.ok) {
            return std::nullopt;
        }
        return std::move(design.taps);
    }

    /// @brief Builds the bank the settings name; false, changing nothing, when no prototype meets them.
    [[nodiscard]] bool rebuild() {
        const int         order    = gr::filter::interpolationOrder(settings.interpolation.value);
        const std::size_t bankSize = settings.bank_size > 0U ? static_cast<std::size_t>(settings.bank_size) : gr::filter::arbitraryBankSize(static_cast<double>(settings.attenuation_db.value), static_cast<double>(settings.rolloff.value), order);

        std::vector<float> prototype   = settings.taps;
        double             designedFor = 0.0; // supplied taps are never redesigned; the caller owns the rate they are valid for
        if (prototype.empty()) {
            designedFor                                = designTarget();
            std::optional<std::vector<float>> designed = designedPrototype(bankSize, designedFor);
            if (!designed) {
                return false;
            }
            prototype = std::move(*designed);
        }
        _bankSize    = bankSize;
        _designedFor = designedFor;

        _resampler.emplace(settings.rate, _bankSize, order, std::span<const float>(prototype));
        _stepOrigin  = _resampler->step();
        _phaseOrigin = _resampler->phase();
        _twiceDelay  = detail::twiceTapDelay(std::span<const float>(prototype));
        return true;
    }

    /**
     * @brief Change the rate, keeping the position, and redesign only where the new rate would under-filter.
     *
     * The design law depends on the rate only through `min(1, r)`, so a rate that rises inside the designed band leaves
     * the prototype narrower than it needs to be, which costs bandwidth and never aliases; only a fall below what it
     * was cut for folds anything in.
     */
    [[nodiscard]] bool retune() {
        const double wanted = designTarget();
        if (settings.taps.value.empty() && wanted < _designedFor) {
            const std::optional<std::vector<float>> prototype = designedPrototype(_bankSize, wanted);
            if (!prototype) {
                return false;
            }
            _resampler->setTaps(*prototype); // keeps the phase and as much of the window as the new B holds
            _designedFor = wanted;
            _twiceDelay  = detail::twiceTapDelay(std::span<const float>(*prototype));
        }
        _resampler->setRate(settings.rate);
        return true;
    }

    /**
     * @brief Take the origin of a new rate regime, and keep the framework's own forwarding off this block's tags.
     *
     * Defining this at all is what replaces the default forwarder, whose output index matches its input index and is
     * right only at a rate of one. The tags themselves are mapped in `processBulk`; what belongs here is the origin,
     * because this is the one hook the framework calls after a settings change has been applied and before a sample
     * of the call is processed, and the one that sees every port's span. A rate change is applied on the settings
     * path, between calls, where neither absolute offset is knowable.
     */
    template<typename TInputSpans, typename TOutputSpans>
    void forwardTags(TInputSpans& inputSpans, TOutputSpans& outputSpans, std::size_t /*processedIn*/) {
        if (!_reorigin) {
            return;
        }
        gr::for_each_reader_span([this](auto& span) { _inOrigin = static_cast<std::uint64_t>(span.streamIndex); }, inputSpans);
        gr::for_each_writer_span([this](auto& span) { _outOrigin = static_cast<std::uint64_t>(span.streamIndex); }, outputSpans);
        _stepOrigin  = _resampler->step();
        _phaseOrigin = _resampler->phase();
        _reorigin    = false;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const auto [nIn, made] = fitToRoom(inSpan.size(), outSpan.size());

        mapTags(inSpan, nIn);
        std::ignore = _resampler->process(std::span<const T>(inSpan.data(), nIn), std::span<T>(outSpan.data(), made));
        _tags.release(outSpan, made, false);

        std::ignore = inSpan.consume(nIn);
        inSpan.consumeTags(nIn);
        outSpan.publish(made);

        if (made == 0UZ && nIn == 0UZ) {
            // above unity one input sample can carry two outputs, so a room of one is not short of input: it is short
            // of room, and reporting that is what stops the scheduler re-offering the same window
            return _resampler->outputsFor(inSpan.size()) > 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

    /// @brief Make as many of the outputs of the stream's last input as the output span holds, and publish every held
    /// tag, at the end-of-stream index for a tag past the last output.
    template<InputSpanLike TInput, OutputSpanLike TOutput>
    [[nodiscard]] work::Status processEpilogue(TInput& inSpan, TOutput& outSpan) {
        if (detail::dropAtStop(*this, _tags, outSpan)) {
            return work::Status::OK;
        }
        const auto [nIn, made] = fitToRoom(inSpan.size(), outSpan.size());

        mapTags(inSpan, nIn);
        std::ignore = _resampler->process(std::span<const T>(inSpan.data(), nIn), std::span<T>(outSpan.data(), made));
        _tags.takeRemainder(in, static_cast<std::uint64_t>(inSpan.streamIndex) + static_cast<std::uint64_t>(nIn), [this](property_map& tag) { scaleSampleRate(tag); });
        _tags.release(outSpan, made, true);

        std::ignore = inSpan.consume(nIn);
        inSpan.consumeTags(nIn);
        outSpan.publish(made);
        return work::Status::OK;
    }

private:
    /// @brief Rewrite a forwarded rate by `rate`. The framework's chunk-ratio scaling covers only a declared `L:M`,
    /// which an arbitrary real ratio is not, so the same rule is applied here from the setting itself.
    void scaleSampleRate(property_map& parameters) const noexcept {
        const auto it = parameters.find(gr::tag::SAMPLE_RATE.key());
        if (it == parameters.end()) {
            return;
        }
        if (const float* inputRate = it->second.get_if<float>(); inputRate != nullptr) {
            it->second = static_cast<float>(settings.rate * static_cast<double>(*inputRate));
        }
    }

    /// @brief The inputs a call takes of @p available and the outputs they make, at most @p room outputs.
    [[nodiscard]] std::pair<std::size_t, std::size_t> fitToRoom(std::size_t available, std::size_t room) const {
        std::size_t nIn  = available;
        std::size_t made = _resampler->outputsFor(nIn);
        if (made > room) {
            // the largest input count whose outputs still fit: `inputsFor` is the smallest count reaching a given
            // output, so one short of what reaches `room + 1` is exactly it
            nIn  = _resampler->inputsFor(room + 1UZ) - 1UZ;
            made = _resampler->outputsFor(nIn);
        }
        return {nIn, made};
    }

    /// @brief The output that input @p at maps to under the regime in force now.
    [[nodiscard]] std::uint64_t outputOf(std::uint64_t at) const noexcept {
        const std::int64_t delayed = _phaseOrigin - static_cast<std::int64_t>(_twiceDelay << (gr::filter::kArbitraryFractionBits - 1)); // the first output's position less the delay, in 2^-F interpolated samples
        return _outOrigin + gr::filter::mapArbitraryOffset(at - _inOrigin, _bankSize, _stepOrigin, delayed);
    }

    /// @brief The `min(1, r)` the prototype has to cover: the smaller of the current rate and the stated floor, where there is one.
    [[nodiscard]] double designTarget() const noexcept {
        const double floorRate = settings.min_rate > 0.0 ? static_cast<double>(settings.min_rate) : 1.0;
        return std::min({1.0, floorRate, static_cast<double>(settings.rate)}); // AppleClang 21 reports nested two-argument std::min calls here as returning the address of a temporary
    }

    /**
     * @brief Place the tags of the samples this call consumes at their output offsets, under the regime in force now.
     *
     * Each tag moves with all its keys. Input `i` maps to the output nearest the interpolated position `i*L + d`, where
     * `d` is the prototype's delay. A half rounds up. Tags leave in the order they arrived, also across a rate change
     * or a rebuild.
     *
     * A tag's mapping is committed here, where its sample is consumed, and not where the tag first becomes visible.
     * An `Async` port is presented every sample it holds and the block consumes a prefix of them, so a tag past that
     * prefix is presented again next call, and `rate` may have changed in between: mapping it on sight would fix its
     * offset, and scale its `gr:sample_rate`, under a regime that no longer governs its sample. Taking only the tags of
     * `[first, last)` is also what maps each one exactly once, `consumeTags` having retired everything below `first`,
     * and an index carrying more than one tag still maps all of them.
     */
    void mapTags(InputSpanLike auto& inSpan, std::size_t nIn) {
        if (nIn == 0UZ || !inSpan.isConnected) {
            return;
        }
        const std::uint64_t first = static_cast<std::uint64_t>(inSpan.streamIndex);
        const std::uint64_t last  = first + static_cast<std::uint64_t>(nIn);
        for (const gr::Tag& tag : inSpan.rawTags) {
            const std::uint64_t at = static_cast<std::uint64_t>(tag.index);
            if (at < first || at >= last) {
                continue;
            }
            property_map forwarded(tag.map);
            scaleSampleRate(forwarded); // the rate that consumes the sample, which is the rate the tag describes
            detail::holdTag(_tags.held, _latestHeld, outputOf(at), std::move(forwarded));
        }
    }
};

} // namespace gr::blocks::filter

#endif // GNURADIO_BLOCKS_ARBITRARY_RESAMPLER_HPP
