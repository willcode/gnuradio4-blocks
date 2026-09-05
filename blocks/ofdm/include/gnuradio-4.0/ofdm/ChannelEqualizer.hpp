#ifndef GNURADIO_OFDM_CHANNEL_EQUALIZER_HPP
#define GNURADIO_OFDM_CHANNEL_EQUALIZER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/Constellation.hpp>
#include <gnuradio-4.0/digital/ConstellationSettings.hpp>

#include <gnuradio-4.0/ofdm/Numerology.hpp>

namespace gr::blocks::ofdm {

GR_REGISTER_BLOCK(gr::blocks::ofdm::OfdmChannelEqualizer)

/**
 * @brief Post-transform symbol records in, equalized data carriers out.
 *
 * A frame opens with its sync symbols, and the one at `sync_index` is the known full symbol this block divides by:
 * `H[c] = Y[c] / X[c]` on every occupied carrier. Guards hold no estimate and are never divided. A `sync_word` that
 * leaves an occupied carrier empty is refused at configure rather than producing a carrier with no channel behind it.
 *
 * Each data symbol is then tracked and equalized:
 *
 * - `cpe` takes the common phase error the pilots agree on,
 *   `theta_s = angle( sum_p Y_s[p] * conj(H[p] * X_s[p]) )`, and applies `e^{-j theta_s}` to the whole symbol. It is
 *   the correction a phase-noise process needs and the one a residual frequency offset needs, and it costs one
 *   arctangent a symbol.
 * - `cpe_interp` does that and then follows the channel itself: the residual each pilot sees after the common phase
 *   is removed is interpolated across the occupied carriers, linearly in magnitude and in unwrapped phase, and fed
 *   into a single-pole per-carrier smoother of coefficient `alpha`. The interpolation weights are a table built once
 *   at configure; what is left per symbol is one `polar` a carrier, which is the transcendental a phase
 *   interpolation cannot be written without.
 * - `none` equalizes on the sync word's estimate alone, which is what a scene measures tracking against.
 *
 * Equalization is zero-forcing, `Y[c]/H[c]`, with the reciprocal kept beside the estimate so a symbol costs a
 * multiply a carrier and no divide. An MMSE variant and a decision-directed channel update are both strategy swaps
 * on this structure and neither is here.
 *
 * Output records carry the data carriers alone, in `data_carriers` order, ready for a constellation decoder, and
 * their axis states which carrier each value came from. `cpe_rad` is the phase this symbol was turned by, `evm_db`
 * is the symbol's own error vector magnitude against the nearest constellation point, and `frame_evm_db` is the same
 * measure accumulated over the frame so far, which on a frame's last symbol is the frame's figure.
 *
 * The channel estimate is readable on an optional record port, one record per estimate, carrying its magnitude and
 * phase against the carrier axis.
 */
struct OfdmChannelEqualizer : Block<OfdmChannelEqualizer, NoTagPropagation> {
    using Description = Doc<"OFDM channel equalizer: post-transform DataSet<complex<float>> symbol records in, equalized data carriers out in data_carriers order. Least squares against the frame's known sync word, common-phase-error tracking on the pilots with an optional interpolated per-carrier update, and zero-forcing equalization. Records carry cpe_rad, evm_db and frame_evm_db; the channel estimate is readable on an optional record port">;

    PortIn<DataSet<Complex>, Async>          in;
    PortOut<DataSet<Complex>, Async>         out;
    PortOut<DataSet<float>, Optional, Async> channel;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length, a power of two; the record's length in bins">>                               fft_len = 64U;
    Annotated<std::vector<std::int32_t>, "data_carriers", Visible, Doc<"signed logical carrier indices the output holds, in order">>              data_carriers{};
    Annotated<std::vector<std::int32_t>, "pilot_carriers", Visible, Doc<"signed logical carrier indices the tracking reads">>                     pilot_carriers{};
    Annotated<std::vector<float>, "pilot_symbols", Doc<"interleaved re,im; read by (s * n_pilots + p) % len with s the data index in the frame">> pilot_symbols{};
    Annotated<std::vector<float>, "sync_word", Doc<"interleaved re,im, one whole fft_len symbol: the known symbol least squares divides by">>     sync_word{};
    Annotated<gr::Size_t, "n_sync", Visible, Doc<"sync symbols at a frame's head; the data index of a symbol is symbol_in_frame - n_sync">>       n_sync      = 1U;
    Annotated<gr::Size_t, "sync_index", Doc<"which of the frame's sync symbols is the known one; the others are passed over">>                    sync_index  = 0U;
    Annotated<std::string, "tracking", Visible, Doc<"'cpe', 'cpe_interp' or 'none'">>                                                             tracking    = std::string("cpe");
    Annotated<float, "alpha", Visible, Doc<"single-pole coefficient of the per-carrier update, in (0, 1]; read by 'cpe_interp'">>                 alpha       = 0.1f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                                signal_name = std::string("ofdm_equalized");

    Annotated<std::string, "constellation", gr::blocks::digital::detail::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, for 'psk' and 'qam'">>                          arity         = 4U;
    Annotated<float, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>                   phase_offset  = 0.f;
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>                          label_xor     = 0U;
    Annotated<std::vector<float>, "points", Doc<"interleaved re,im for 'custom'">>                          points{};
    Annotated<std::string, "normalization", Doc<"'power', 'amplitude' or 'none'">>                          normalization = std::string("power");

    GR_MAKE_REFLECTABLE(OfdmChannelEqualizer, in, out, channel, fft_len, data_carriers, pilot_carriers, pilot_symbols, sync_word, n_sync, sync_index, tracking, alpha, signal_name, constellation, arity, phase_offset, label_xor, points, normalization);

    /// Where an occupied carrier sits between two pilots, as the weights the interpolation applies. Built once at
    /// configure, because the geometry is a property of the numerology and not of any symbol.
    struct Between {
        std::size_t left{0UZ};  ///< pilot slot at or below this carrier
        std::size_t right{0UZ}; ///< pilot slot at or above it
        float       weight{0.f};
    };

    std::optional<gr::ofdm::CarrierMap> _map{};
    gr::digital::Constellation<float>   _constellation = gr::digital::Constellation<float>::qpsk();

    std::vector<Complex>      _pilots{};    ///< `pilot_symbols` as complex values
    std::vector<Complex>      _sync{};      ///< the known symbol, whole
    std::vector<Complex>      _h{};         ///< the channel estimate, per bin
    std::vector<Complex>      _hInv{};      ///< its reciprocal, so a symbol costs no divide
    std::vector<Complex>      _corrected{}; ///< one symbol after the common phase is taken out
    std::vector<Complex>      _values{};    ///< the data carriers of one output record
    std::vector<std::size_t>  _occupied{};  ///< occupied bins, ascending in signed carrier index
    std::vector<Between>      _between{};   ///< one entry per `_occupied` bin
    std::vector<Complex>      _pilotEstimate{};
    std::vector<int>          _dataAxis{}; ///< the signed carrier of each output value
    std::vector<int>          _occupiedAxis{};
    std::vector<float>        _pilotMagnitude{}; ///< |H| at each pilot, taken once a symbol rather than once a carrier
    std::vector<float>        _pilotAngle{};
    detail::SymbolRecordShape _shape{}; ///< the invariant part of a record, assigned into the slot the port recycles

    bool          _interp        = false;
    bool          _track         = true;
    bool          _trained       = false;
    std::size_t   _symbolInFrame = 0UZ; ///< the position a record that names none is taken to hold
    double        _frameError    = 0.;
    double        _frameSignal   = 0.;
    std::uint64_t _untrained     = 0ULL; ///< data symbols equalized before any sync word was seen
    std::uint64_t _skipped       = 0ULL; ///< sync symbols that were not the known one
    std::uint64_t _estimates     = 0ULL;

    /// @brief Data symbols equalized on the identity channel because no sync word had arrived yet.
    [[nodiscard]] std::uint64_t nUntrained() const noexcept { return _untrained; }
    /// @brief Sync symbols passed over: a frame's head holds more than the one least squares reads.
    [[nodiscard]] std::uint64_t nSkipped() const noexcept { return _skipped; }
    /// @brief Least-squares estimates taken over the run, which is one per frame.
    [[nodiscard]] std::uint64_t nEstimates() const noexcept { return _estimates; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kRebuildKeys{"fft_len", "data_carriers", "pilot_carriers", "pilot_symbols", "sync_word", "n_sync", "sync_index", "tracking", "alpha", "constellation", "arity", "phase_offset", "label_xor", "points", "normalization"};
        if (_map.has_value() && !std::ranges::any_of(kRebuildKeys, [&newSettings](std::string_view key) { return newSettings.contains(key); })) {
            return;
        }
        rebuild();
    }

    void start() {
        if (!_map.has_value()) {
            rebuild();
        }
        resetState();
    }

    void rebuild() {
        gr::ofdm::CarrierMap map = detail::buildMap(fft_len, std::span<const std::int32_t>(data_carriers.value), std::span<const std::int32_t>(pilot_carriers.value));

        std::vector<Complex> pilots = detail::complexFrom(std::span<const float>(pilot_symbols.value), "pilot_symbols");
        try {
            map.validatePilotCycle(pilots.size());
        } catch (const std::invalid_argument& error) {
            throw gr::exception(error.what());
        }

        std::vector<Complex> known = detail::complexFrom(std::span<const float>(sync_word.value), "sync_word");
        try {
            map.validateSyncWord(std::span<const Complex>(known));
        } catch (const std::invalid_argument& error) {
            throw gr::exception(error.what());
        }

        if (tracking.value != "cpe" && tracking.value != "cpe_interp" && tracking.value != "none") {
            throw gr::exception(std::format("tracking must be 'cpe', 'cpe_interp' or 'none', got '{}'", tracking.value));
        }
        if (!(alpha.value > 0.f) || !(alpha.value <= 1.f)) {
            throw gr::exception(std::format("alpha is the single-pole coefficient of the per-carrier update and must lie in (0, 1], got {}", alpha.value));
        }
        if (sync_index.value >= n_sync.value) {
            throw gr::exception(std::format("sync_index is {} and n_sync is {}: the known symbol has to be one of the frame's sync symbols", sync_index.value, n_sync.value));
        }
        if (tracking.value != "none" && map.nPilots() == 0UZ) {
            throw gr::exception(std::format("tracking is '{}' but the numerology has no pilot carriers to track on", tracking.value));
        }

        // A carrier the sync word leaves empty is one least squares cannot estimate, and an unestimated occupied
        // carrier would reach the output divided by nothing. That is a numerology to fix, not a case to handle.
        const std::size_t fftLength = static_cast<std::size_t>(fft_len.value);
        for (const std::size_t bin : map.dataBins()) {
            if (known[bin] == Complex{}) {
                throw gr::exception(std::format("sync_word carries nothing on data carrier {}, so the channel cannot be estimated there", gr::ofdm::CarrierMap::carrierOf(fftLength, bin)));
            }
        }
        for (const std::size_t bin : map.pilotBins()) {
            if (known[bin] == Complex{}) {
                throw gr::exception(std::format("sync_word carries nothing on pilot carrier {}, so the channel cannot be estimated there", gr::ofdm::CarrierMap::carrierOf(fftLength, bin)));
            }
        }

        _constellation = gr::blocks::digital::detail::build<float>(constellation.value, arity, phase_offset, label_xor, std::span<const float>(points.value), normalization.value);

        _dataAxis.assign(map.dataCarriers().begin(), map.dataCarriers().end());
        buildInterpolation(map);

        _interp = tracking.value == "cpe_interp";
        _track  = tracking.value != "none";
        _map.emplace(std::move(map));
        _pilots = std::move(pilots);
        _sync   = std::move(known);
        _h.assign(fftLength, Complex{1.f, 0.f});
        _hInv.assign(fftLength, Complex{1.f, 0.f});
        _corrected.assign(fftLength, Complex{});
        _values.assign(_map->nData(), Complex{});
        _pilotEstimate.assign(_map->nPilots(), Complex{1.f, 0.f});
        _pilotMagnitude.assign(_map->nPilots(), 1.f);
        _pilotAngle.assign(_map->nPilots(), 0.f);
        _shape.build(std::span<const int>(_dataAxis), signal_name.value, property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(static_cast<std::uint64_t>(fft_len.value))}});
        resetState();
    }

    void resetState() {
        std::ranges::fill(_h, Complex{1.f, 0.f});
        std::ranges::fill(_hInv, Complex{1.f, 0.f});
        _trained       = false;
        _symbolInFrame = 0UZ;
        _frameError    = 0.;
        _frameSignal   = 0.;
        _untrained     = 0ULL;
        _skipped       = 0ULL;
        _estimates     = 0ULL;
    }

    /// @brief One record in, at most one record out, taking nothing the output has no room for.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& channelSpan) {
        const bool  wantEstimate = channelSpan.isConnected;
        std::size_t consumed     = 0UZ;
        std::size_t made         = 0UZ;
        std::size_t estimates    = 0UZ;

        while (consumed < inSpan.size()) {
            const DataSet<Complex>& record   = inSpan[consumed];
            const std::uint64_t     position = detail::metaCount(record, detail::kSymbolInFrame, static_cast<std::uint64_t>(_symbolInFrame));
            const bool              isSync   = position < static_cast<std::uint64_t>(n_sync.value);
            if (!isSync && made == outSpan.size()) {
                break; // its equalized form has nowhere to go, so the record stays where it is
            }
            if (isSync && position == static_cast<std::uint64_t>(sync_index.value) && wantEstimate && estimates == channelSpan.size()) {
                break; // the estimate it produces has nowhere to go
            }

            if (position == 0ULL) {
                _frameError  = 0.;
                _frameSignal = 0.;
            }
            if (isSync) {
                if (position == static_cast<std::uint64_t>(sync_index.value)) {
                    estimate(record);
                    if (wantEstimate) {
                        channelSpan[estimates] = channelRecord(record);
                        ++estimates;
                    }
                } else {
                    ++_skipped;
                }
            } else {
                equalize(outSpan[made], record, position - static_cast<std::uint64_t>(n_sync.value));
                ++made;
            }
            _symbolInFrame = static_cast<std::size_t>(position) + 1UZ;
            ++consumed;
        }

        outSpan.publish(made);
        channelSpan.publish(estimates);
        std::ignore = inSpan.consume(consumed);
        if (consumed == 0UZ) {
            return inSpan.size() == 0UZ ? work::Status::INSUFFICIENT_INPUT_ITEMS : work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    /// @brief Precompute which pilots each occupied carrier sits between, and how far along.
    void buildInterpolation(const gr::ofdm::CarrierMap& map) {
        const std::size_t fftLength = map.fftLength();
        std::vector<int>  carriers(map.dataCarriers().begin(), map.dataCarriers().end());
        carriers.insert(carriers.end(), map.pilotCarriers().begin(), map.pilotCarriers().end());
        std::ranges::sort(carriers);

        std::vector<int>         pilotOrder(map.pilotCarriers().begin(), map.pilotCarriers().end());
        std::vector<std::size_t> slotOf(pilotOrder.size());
        for (std::size_t p = 0UZ; p < pilotOrder.size(); ++p) {
            slotOf[p] = p;
        }
        std::ranges::sort(slotOf, [&pilotOrder](std::size_t a, std::size_t b) { return pilotOrder[a] < pilotOrder[b]; });

        _occupied.clear();
        _occupiedAxis.clear();
        _between.clear();
        _occupied.reserve(carriers.size());
        _between.reserve(carriers.size());
        for (const int carrier : carriers) {
            _occupied.push_back(gr::ofdm::CarrierMap::binOf(fftLength, carrier));
            _occupiedAxis.push_back(carrier);

            Between where{};
            if (slotOf.empty()) {
                _between.push_back(where);
                continue;
            }
            std::size_t upper = 0UZ;
            while (upper < slotOf.size() && pilotOrder[slotOf[upper]] < carrier) {
                ++upper;
            }
            if (upper == 0UZ) { // below every pilot: the nearest one's value is held
                where = Between{slotOf.front(), slotOf.front(), 0.f};
            } else if (upper == slotOf.size()) {
                where = Between{slotOf.back(), slotOf.back(), 0.f};
            } else {
                const int low  = pilotOrder[slotOf[upper - 1UZ]];
                const int high = pilotOrder[slotOf[upper]];
                where          = Between{slotOf[upper - 1UZ], slotOf[upper], static_cast<float>(carrier - low) / static_cast<float>(high - low)};
            }
            _between.push_back(where);
        }
    }

    /// @brief Least squares against the known symbol: divide once, keep the reciprocal.
    void estimate(const DataSet<Complex>& record) {
        const Complex* observed = record.signal_values.data();
        const Complex* known    = _sync.data();
        for (const std::size_t bin : _occupied) {
            const Complex h = detail::divide(observed[bin], known[bin]);
            _h[bin]         = h;
            _hInv[bin]      = detail::reciprocal(h);
        }
        _trained = true;
        ++_estimates;
    }

    /// @brief The pilot value symbol @p dataIndex carries in slot @p slot, by the family's two-level cycle.
    [[nodiscard]] Complex pilotOf(std::uint64_t dataIndex, std::size_t slot) const noexcept { return _pilots[gr::ofdm::CarrierMap::pilotSymbolIndex(static_cast<std::size_t>(dataIndex), slot, _map->nPilots(), _pilots.size())]; }

    void equalize(DataSet<Complex>& target, const DataSet<Complex>& record, std::uint64_t dataIndex) {
        if (!_trained) {
            ++_untrained;
        }
        const Complex*     observed  = record.signal_values.data();
        const std::size_t* pilotBins = _map->pilotBins().data();
        const std::size_t  nPilots   = _map->nPilots();

        double theta = 0.;
        if (_track) {
            std::complex<double> sum{};
            for (std::size_t p = 0UZ; p < nPilots; ++p) {
                const std::size_t bin      = pilotBins[p];
                const Complex     expected = detail::multiply(_h[bin], pilotOf(dataIndex, p));
                sum += std::complex<double>(observed[bin]) * std::conj(std::complex<double>(expected));
            }
            theta = std::atan2(sum.imag(), sum.real());
        }
        const Complex rotate(static_cast<float>(std::cos(-theta)), static_cast<float>(std::sin(-theta)));

        for (const std::size_t bin : _occupied) {
            _corrected[bin] = detail::multiply(observed[bin], rotate);
        }
        if (_interp) {
            interpolate(dataIndex);
        }

        const std::size_t* dataBins = _map->dataBins().data();
        const std::size_t  nData    = _map->nData();
        const Complex*     inverse  = _hInv.data();
        Complex*           values   = _values.data();
        double             error    = 0.;
        double             signal   = 0.;
        for (std::size_t k = 0UZ; k < nData; ++k) {
            const std::size_t bin = dataBins[k];
            const Complex     z   = detail::multiply(_corrected[bin], inverse[bin]);
            values[k]             = z;
            const Complex point   = _constellation.point(_constellation.hardDecision(z));
            error += static_cast<double>(std::norm(z - point));
            signal += static_cast<double>(std::norm(point));
        }
        _frameError += error;
        _frameSignal += signal;

        const double symbolEvm = signal > 0. ? 10. * std::log10(std::max(error / signal, 1e-30)) : 0.;
        const double frameEvm  = _frameSignal > 0. ? 10. * std::log10(std::max(_frameError / _frameSignal, 1e-30)) : 0.;

        property_map& meta                             = _shape.emitInto(target, std::span<const Complex>(_values), detail::kKindEqualized);
        meta[std::pmr::string(detail::kSymbolInFrame)] = pmt::Value(dataIndex + static_cast<std::uint64_t>(n_sync.value));
        meta[std::pmr::string(detail::kCpeRadKey)]     = pmt::Value(static_cast<float>(theta));
        meta[std::pmr::string(detail::kEvmDbKey)]      = pmt::Value(static_cast<float>(symbolEvm));
        meta[std::pmr::string(detail::kFrameEvmDbKey)] = pmt::Value(static_cast<float>(frameEvm));
        meta[std::pmr::string(detail::kFrameIndexKey)] = pmt::Value(detail::metaCount(record, detail::kFrameIndexKey, 0ULL));
    }

    /**
     * @brief Follow the channel between sync words: each pilot's residual, interpolated and smoothed per carrier.
     *
     * The common phase has already been taken out of `_corrected`, so what a pilot sees now is the channel itself and
     * the smoother's input stays near its previous value rather than chasing a rotation. Magnitude and phase are
     * interpolated apart, because a linear interpolation of two complex numbers shrinks toward zero wherever they
     * disagree in phase, and it is exactly the phase that a delay spread makes disagree.
     */
    void interpolate(std::uint64_t dataIndex) {
        const std::size_t* pilotBins = _map->pilotBins().data();
        const std::size_t  nPilots   = _map->nPilots();
        for (std::size_t p = 0UZ; p < nPilots; ++p) {
            const Complex estimate = detail::divide(_corrected[pilotBins[p]], pilotOf(dataIndex, p));
            _pilotEstimate[p]      = estimate;
            // taken once a pilot, not once a carrier: there are four of these and fifty-two of those, and an
            // arctangent is the most expensive thing on the path
            _pilotMagnitude[p] = std::abs(estimate);
            _pilotAngle[p]     = std::arg(estimate);
        }

        const float mix = alpha.value;
        for (std::size_t k = 0UZ; k < _occupied.size(); ++k) {
            const Between& where    = _between[k];
            const float    lowMag   = _pilotMagnitude[where.left];
            const float    lowPhase = _pilotAngle[where.left];

            const float magnitude = lowMag + where.weight * (_pilotMagnitude[where.right] - lowMag);
            // the shorter way round, so a pair straddling the branch cut interpolates through it rather than the far side
            const float step  = std::remainder(_pilotAngle[where.right] - lowPhase, 2.f * std::numbers::pi_v<float>);
            const float phase = lowPhase + where.weight * step;

            const std::size_t bin      = _occupied[k];
            const Complex     target   = std::polar(magnitude, phase);
            const Complex     previous = _h[bin];
            const Complex     blended  = Complex(previous.real() * (1.f - mix) + target.real() * mix, previous.imag() * (1.f - mix) + target.imag() * mix);
            _h[bin]                    = blended;
            _hInv[bin]                 = detail::reciprocal(blended);
        }
    }

    /// @brief The channel estimate as a measurement record: magnitude and phase against the carrier axis.
    [[nodiscard]] DataSet<float> channelRecord(const DataSet<Complex>& source) const {
        const std::size_t count = _occupied.size();
        DataSet<float>    ds;

        ds.extents = {static_cast<std::int32_t>(count)};
        ds.layout  = gr::LayoutRight{};

        ds.axis_names = {"Carrier"};
        ds.axis_units = {"1"};
        ds.axis_values.resize(1UZ);
        ds.axis_values[0UZ].resize(count);
        for (std::size_t k = 0UZ; k < count; ++k) {
            ds.axis_values[0UZ][k] = static_cast<float>(_occupiedAxis[k]);
        }

        ds.signal_names      = {"ChannelMagnitude", "ChannelPhase"};
        ds.signal_quantities = {"Magnitude", "Phase"};
        ds.signal_units      = {"1", "rad"};
        ds.signal_values.resize(2UZ * count);
        for (std::size_t k = 0UZ; k < count; ++k) {
            ds.signal_values[k]         = std::abs(_h[_occupied[k]]);
            ds.signal_values[count + k] = std::arg(_h[_occupied[k]]);
        }
        ds.signal_ranges.resize(2UZ);
        for (std::size_t s = 0UZ; s < 2UZ; ++s) {
            const auto range    = std::minmax_element(ds.signal_values.begin() + static_cast<std::ptrdiff_t>(s * count), ds.signal_values.begin() + static_cast<std::ptrdiff_t>((s + 1UZ) * count));
            ds.signal_ranges[s] = {*range.first, *range.second};
        }

        const property_map meta{
            {std::pmr::string(detail::kFftLenKey), pmt::Value(static_cast<std::uint64_t>(fft_len.value))},
            {std::pmr::string(detail::kFrameIndexKey), pmt::Value(detail::metaCount(source, detail::kFrameIndexKey, 0ULL))},
            {std::pmr::string(detail::kSampleStartKey), pmt::Value(detail::metaCount(source, detail::kSampleStartKey, 0ULL))},
            {std::pmr::string("n_averaged"), pmt::Value(std::uint64_t{1})},
            {std::pmr::string("estimator"), pmt::Value(std::string("least_squares"))},
        };
        ds.meta_information.assign(2UZ, meta);
        ds.timing_events.resize(2UZ);
        ds.timestamp = 0;
        return ds;
    }
};

} // namespace gr::blocks::ofdm

#endif // GNURADIO_OFDM_CHANNEL_EQUALIZER_HPP
