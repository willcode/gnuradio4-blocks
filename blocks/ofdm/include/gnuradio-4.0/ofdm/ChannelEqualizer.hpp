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

GR_REGISTER_BLOCK(gr::blocks::ofdm::ChannelEqualizer)

/**
 * @brief Equalizes post-transform symbol records and emits their data carriers.
 *
 * A frame opens with `n_sync` sync symbols. The block divides the one at `sync_index` by the known `sync_word`,
 * `H[c] = Y[c] / X[c]` on every occupied carrier, and counts the others in `nSkipped()`. Guard carriers hold no
 * estimate. A `sync_word` that leaves an occupied carrier empty is refused when the settings are applied. A symbol's
 * data index is `symbol_in_frame - n_sync`, and the pilot values follow the two-level cycle over that index. Every
 * pilot value is finite and nonzero. A record whose length is not `fft_len` is dropped and counted in `nMalformed()`.
 *
 * The block then tracks and equalizes each data symbol in one of three modes.
 *
 * - `cpe` takes the common phase error the pilots agree on,
 *   `theta_s = angle( sum_p Y_s[p] * conj(H[p] * X_s[p]) )`, and applies `e^{-j theta_s}` to the whole symbol. It
 *   removes the common part of phase noise and of a residual frequency offset. It costs one arctangent per symbol.
 * - `cpe_interp` does the same and then follows the channel itself. After the common phase is removed, each pilot
 *   gives its ratio to the current estimate, `r[p] = Y_s[p] * e^{-j theta_s} / (H[p] * X_s[p])`. The ratios are
 *   interpolated across the occupied carriers, linearly in magnitude and in unwrapped phase. Each carrier's estimate
 *   is multiplied by `1 - alpha + alpha * r[c]`, a single-pole step toward `H[c] * r[c]`. On a static channel without
 *   noise every ratio is 1, and the estimate keeps its least-squares shape between the pilots. The interpolation
 *   weights are a table built when the settings are applied. Each symbol costs one `std::polar` per carrier.
 * - `none` equalizes on the sync word's estimate alone.
 *
 * Settings may change while the block runs. A change of `fft_len`, `data_carriers`, `pilot_carriers` or `sync_word`
 * resets the channel estimate. A change of any other setting keeps it. The counters run from `start()` across every
 * change.
 *
 * A refused setting throws and changes nothing the block computes. The members keep the values of the refused
 * application, and a later change the block accepts can take them. Any change of its own settings checks and takes
 * `tracking`, `alpha`, `signal_name` and the constellation settings. A change of `fft_len`, the carrier sets,
 * `pilot_symbols`, `sync_word`, `n_sync` or `sync_index` checks every setting again and takes them all when they pass.
 *
 * The default settings name no data carriers. A block that has accepted no numerology emits an error from
 * `processBulk()` and returns `ERROR`.
 *
 * Equalization is zero-forcing, `Y[c]/H[c]`. The block keeps the reciprocal beside the estimate, and a symbol costs
 * one multiply per carrier and no divide.
 *
 * Output records carry the data carriers alone, in `data_carriers` order. Their axis states the carrier each value
 * came from. `cpe_rad` is the phase the block removed from the symbol. `evm_db` is the symbol's error vector magnitude
 * against the nearest constellation point. `frame_evm_db` is the same measure over the frame so far, and on a frame's
 * last symbol it covers the whole frame.
 *
 * The optional `channel` port emits one record per estimate, with magnitude and phase against the carrier axis.
 */
struct ChannelEqualizer : Block<ChannelEqualizer, NoTagPropagation> {
    using Description = Doc<"Equalizes post-transform DataSet<complex<float>> symbol records and emits the data carriers in data_carriers order. It estimates the channel by least squares against the frame's known sync word, tracks the common phase error on the pilots with an optional interpolated per-carrier update, and equalizes by zero forcing. Records carry cpe_rad, evm_db and frame_evm_db. The optional channel port carries the channel estimate">;

    PortIn<DataSet<Complex>, Async>          in;
    PortOut<DataSet<Complex>, Async>         out;
    PortOut<DataSet<float>, Optional, Async> channel;

    Annotated<gr::Size_t, "fft_len", Visible, Doc<"transform length and record length in bins, a power of two">>                     fft_len = 64U;
    Annotated<std::vector<std::int32_t>, "data_carriers", Visible, Doc<"signed logical carrier indices the output holds, in order">> data_carriers{};
    Annotated<std::vector<std::int32_t>, "pilot_carriers", Visible, Doc<"signed logical carrier indices the tracking reads">>        pilot_carriers{};
    Annotated<std::vector<float>, "pilot_symbols", Doc<"interleaved re,im nonzero pilots, read by (s*n_pilots + p) % len">>          pilot_symbols{};
    Annotated<std::vector<float>, "sync_word", Doc<"interleaved re,im known symbol of fft_len carriers">>                            sync_word{};
    Annotated<gr::Size_t, "n_sync", Visible, Doc<"sync symbols at a frame's head">>                                                  n_sync      = 1U;
    Annotated<gr::Size_t, "sync_index", Doc<"index of the known symbol among the frame's sync symbols">>                             sync_index  = 0U;
    Annotated<std::string, "tracking", Visible, Doc<"phase tracking mode: 'cpe', 'cpe_interp' or 'none'">>                           tracking    = std::string("cpe");
    Annotated<float, "alpha", Visible, Doc<"single-pole coefficient of the 'cpe_interp' update, in (0, 1]">>                         alpha       = 0.1f;
    Annotated<std::string, "signal_name", Doc<"the emitted record's signal name">>                                                   signal_name = std::string("ofdm_equalized");

    Annotated<std::string, "constellation", gr::blocks::digital::ConstellationSettingsDoc, Visible> constellation = std::string("qpsk");
    Annotated<gr::Size_t, "arity", Unit<"symbols">, Doc<"M, read by 'psk' and 'qam'">>              arity         = 4U;
    Annotated<float, "phase_offset", Unit<"rad">, Doc<"rotation of the 'psk' point set">>           phase_offset  = 0.f;
    Annotated<gr::Size_t, "label_xor", Doc<"label offset of the 'psk' point set">>                  label_xor     = 0U;
    Annotated<std::vector<float>, "points", Doc<"interleaved re,im points of 'custom'">>            points{};
    Annotated<std::string, "normalization", Doc<"point scaling: 'power', 'amplitude' or 'none'">>   normalization = std::string("power");

    GR_MAKE_REFLECTABLE(ChannelEqualizer, in, out, channel, fft_len, data_carriers, pilot_carriers, pilot_symbols, sync_word, n_sync, sync_index, tracking, alpha, signal_name, constellation, arity, phase_offset, label_xor, points, normalization);

    /// The two pilots around an occupied carrier and the carrier's interpolation weight between them. The table
    /// depends on the numerology alone.
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
    std::vector<Complex>      _hInv{};      ///< its reciprocal
    std::vector<Complex>      _corrected{}; ///< one symbol after the common phase is taken out
    std::vector<Complex>      _values{};    ///< the data carriers of one output record
    std::vector<std::size_t>  _occupied{};  ///< occupied bins, ascending in signed carrier index
    std::vector<Between>      _between{};   ///< one entry per `_occupied` bin
    std::vector<int>          _dataAxis{};  ///< the signed carrier of each output value
    std::vector<int>          _occupiedAxis{};
    std::vector<float>        _pilotMagnitude{}; ///< |r| at each pilot, taken once per symbol instead of once per carrier
    std::vector<float>        _pilotAngle{};
    detail::SymbolRecordShape _shape{}; ///< the invariant part of a record

    bool          _interp        = false;
    bool          _track         = true;
    bool          _trained       = false;
    std::size_t   _symbolInFrame = 0UZ;  ///< the position of a record that carries no `symbol_in_frame`
    std::uint64_t _nSync         = 0ULL; ///< `n_sync` as last accepted
    std::uint64_t _syncIndex     = 0ULL; ///< `sync_index` as last accepted
    float         _alpha         = 0.f;  ///< `alpha` as last accepted
    double        _frameError    = 0.;
    double        _frameSignal   = 0.;
    std::uint64_t _untrained     = 0ULL; ///< data symbols equalized with no estimate in force
    std::uint64_t _skipped       = 0ULL; ///< sync symbols other than the known one
    std::uint64_t _estimates     = 0ULL;
    std::uint64_t _malformed     = 0ULL; ///< records whose length is not `fft_len`

    /// @brief Data symbols equalized while no channel estimate is in force, on the identity channel.
    [[nodiscard]] std::uint64_t nUntrained() const noexcept { return _untrained; }
    /// @brief Sync symbols other than the one at `sync_index`.
    [[nodiscard]] std::uint64_t nSkipped() const noexcept { return _skipped; }
    /// @brief Least-squares estimates taken over the run, one per frame.
    [[nodiscard]] std::uint64_t nEstimates() const noexcept { return _estimates; }
    /// @brief Records whose length is not `fft_len`. The block drops them.
    [[nodiscard]] std::uint64_t nMalformed() const noexcept { return _malformed; }

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        static constexpr std::array kNumerologyKeys{"fft_len", "data_carriers", "pilot_carriers", "pilot_symbols", "sync_word", "n_sync", "sync_index"};
        static constexpr std::array kEqualizationKeys{"tracking", "alpha", "signal_name", "constellation", "arity", "phase_offset", "label_xor", "points", "normalization"};
        const auto                  changed = [&newSettings](std::span<const char* const> keys) { return std::ranges::any_of(keys, [&newSettings](std::string_view key) { return newSettings.contains(key); }); };
        if (!_map.has_value() || changed(kNumerologyKeys)) {
            rebuild();
        } else if (changed(kEqualizationKeys)) {
            applyEqualization(validatedEqualization(_map->nPilots()));
        }
    }

    void start() { resetState(); }

    /// @brief Builds the numerology and every setting on it. Every check runs before the block changes. A new
    /// numerology or sync word resets the channel estimate.
    void rebuild() {
        gr::ofdm::CarrierMap map = detail::buildMap(fft_len, std::span<const std::int32_t>(data_carriers.value), std::span<const std::int32_t>(pilot_carriers.value));

        std::vector<Complex> pilots = detail::complexFrom(std::span<const float>(pilot_symbols.value), "pilot_symbols");
        detail::requireValid(map.validatePilotCycle(std::span<const Complex>(pilots)));

        std::vector<Complex> known = detail::complexFrom(std::span<const float>(sync_word.value), "sync_word");
        detail::requireValid(map.validateSyncWord(std::span<const Complex>(known)));

        if (sync_index.value >= n_sync.value) {
            throw gr::exception(std::format("sync_index is {} and n_sync is {}: the known symbol has to be one of the frame's sync symbols", sync_index.value, n_sync.value));
        }
        gr::digital::Constellation<float> decisions = validatedEqualization(map.nPilots());

        // Least squares cannot estimate a carrier the sync word leaves empty.
        const std::size_t fftLength = static_cast<std::size_t>(fft_len.value);
        for (const std::size_t bin : map.dataBins()) {
            if (known[bin] == Complex{}) {
                throw gr::exception(std::format("sync_word carries nothing on data carrier {}, so the channel cannot be estimated there", detail::carrierAt(fftLength, bin)));
            }
        }
        for (const std::size_t bin : map.pilotBins()) {
            if (known[bin] == Complex{}) {
                throw gr::exception(std::format("sync_word carries nothing on pilot carrier {}, so the channel cannot be estimated there", detail::carrierAt(fftLength, bin)));
            }
        }

        const bool newNumerology = !_map.has_value() || map.fftLength() != _map->fftLength() || !std::ranges::equal(map.dataCarriers(), _map->dataCarriers()) || !std::ranges::equal(map.pilotCarriers(), _map->pilotCarriers());
        const bool newEstimate   = newNumerology || known != _sync;
        if (newNumerology) {
            _dataAxis.assign(map.dataCarriers().begin(), map.dataCarriers().end());
            buildInterpolation(map);
            _map.emplace(std::move(map));
            _h.assign(fftLength, Complex{1.f, 0.f});
            _hInv.assign(fftLength, Complex{1.f, 0.f});
            _corrected.assign(fftLength, Complex{});
            _values.assign(_map->nData(), Complex{});
            _pilotMagnitude.assign(_map->nPilots(), 1.f);
            _pilotAngle.assign(_map->nPilots(), 0.f);
        }
        _pilots    = std::move(pilots);
        _sync      = std::move(known);
        _nSync     = n_sync.value;
        _syncIndex = sync_index.value;
        applyEqualization(std::move(decisions));
        if (newEstimate) {
            resetEstimate();
        }
    }

    /// @brief Resets the channel estimate, the frame's position and error sums, and the counters.
    void resetState() {
        resetEstimate();
        _symbolInFrame = 0UZ;
        _frameError    = 0.;
        _frameSignal   = 0.;
        _untrained     = 0ULL;
        _skipped       = 0ULL;
        _estimates     = 0ULL;
        _malformed     = 0ULL;
    }

    /// @brief One record in, at most one record out, taking nothing the output has no room for.
    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan, OutputSpanLike auto& channelSpan) {
        if (!_map.has_value()) {
            this->emitErrorMessage("processBulk", gr::Error("gr::blocks::ofdm::ChannelEqualizer: the block holds no numerology. Its default settings name no data carriers, and no settings application has built one"));
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            channelSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const bool        wantEstimate = channelSpan.isConnected;
        const std::size_t fftLength    = _map->fftLength();
        std::size_t       consumed     = 0UZ;
        std::size_t       made         = 0UZ;
        std::size_t       estimates    = 0UZ;

        while (consumed < inSpan.size()) {
            const DataSet<Complex>& record     = inSpan[consumed];
            const std::uint64_t     position   = detail::metaCount(record, detail::kSymbolInFrame, static_cast<std::uint64_t>(_symbolInFrame));
            const bool              isSync     = position < _nSync;
            const bool              wellFormed = record.signal_values.size() == fftLength;
            if (wellFormed && !isSync && made == outSpan.size()) {
                break; // no room for its equalized record, which leaves it on the input
            }
            if (wellFormed && isSync && position == _syncIndex && wantEstimate && estimates == channelSpan.size()) {
                break; // no room for its channel record
            }

            if (position == 0ULL) {
                _frameError  = 0.;
                _frameSignal = 0.;
            }
            if (!wellFormed) {
                ++_malformed;
            } else if (isSync) {
                if (position == _syncIndex) {
                    estimate(record);
                    if (wantEstimate) {
                        channelSpan[estimates] = channelRecord(record);
                        ++estimates;
                    }
                } else {
                    ++_skipped;
                }
            } else {
                equalize(outSpan[made], record, position - _nSync);
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
    /// @brief Checks `tracking` and `alpha` for a numerology of @p nPilots pilots and builds the constellation.
    [[nodiscard]] gr::digital::Constellation<float> validatedEqualization(std::size_t nPilots) const {
        if (tracking.value != "cpe" && tracking.value != "cpe_interp" && tracking.value != "none") {
            throw gr::exception(std::format("tracking must be 'cpe', 'cpe_interp' or 'none', got '{}'", tracking.value));
        }
        if (!(alpha.value > 0.f) || !(alpha.value <= 1.f)) {
            throw gr::exception(std::format("alpha is the single-pole coefficient of the per-carrier update and must lie in (0, 1], got {}", alpha.value));
        }
        if (tracking.value != "none" && nPilots == 0UZ) {
            throw gr::exception(std::format("tracking is '{}' but the numerology has no pilot carriers to track on", tracking.value));
        }
        return gr::blocks::digital::constellationFromSettings<float>(constellation.value, arity, phase_offset, label_xor, std::span<const float>(points.value), normalization.value);
    }

    /// @brief Applies the tracking mode, alpha, the constellation and the signal name. The channel estimate stays.
    void applyEqualization(gr::digital::Constellation<float> decisions) {
        _constellation = std::move(decisions);
        _interp        = tracking.value == "cpe_interp";
        _track         = tracking.value != "none";
        _alpha         = alpha.value;
        _shape.build(std::span<const int>(_dataAxis), signal_name.value, property_map{{std::pmr::string(detail::kFftLenKey), pmt::Value(std::uint64_t{_map->fftLength()})}});
    }

    /// @brief Builds the table of the pilots around each occupied carrier and its weight between them.
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
            _occupied.push_back(detail::binAt(fftLength, carrier));
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
            if (upper == 0UZ) { // below every pilot, holding the nearest pilot's value
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

    /// @brief Puts the identity channel in place of the estimate until the next sync symbol.
    void resetEstimate() {
        std::ranges::fill(_h, Complex{1.f, 0.f});
        std::ranges::fill(_hInv, Complex{1.f, 0.f});
        _trained = false;
    }

    /// @brief Least squares against the known symbol. Divides once and keeps the reciprocal.
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

    /// @brief The pilot value symbol @p dataIndex carries in slot @p slot, by the two-level cycle.
    [[nodiscard]] Complex pilotOf(std::uint64_t dataIndex, std::size_t slot) const noexcept { return _pilots[gr::ofdm::CarrierMap::pilotSymbolIndex(dataIndex, slot, _map->nPilots(), _pilots.size())]; }

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
        meta[std::pmr::string(detail::kSymbolInFrame)] = pmt::Value(dataIndex + _nSync);
        meta[std::pmr::string(detail::kCpeRadKey)]     = pmt::Value(static_cast<float>(theta));
        meta[std::pmr::string(detail::kEvmDbKey)]      = pmt::Value(static_cast<float>(symbolEvm));
        meta[std::pmr::string(detail::kFrameEvmDbKey)] = pmt::Value(static_cast<float>(frameEvm));
        meta[std::pmr::string(detail::kFrameIndexKey)] = pmt::Value(detail::metaCount(record, detail::kFrameIndexKey, 0ULL));
    }

    /**
     * @brief Updates the channel estimate between sync words from the pilots, interpolated across the carriers and
     * smoothed.
     *
     * `_corrected` has the common phase removed. Magnitude and phase of the ratios are interpolated separately. A
     * linear interpolation of two complex values shrinks toward zero where their phases differ. A timing drift turns
     * the phase linearly across the carriers, and the phase interpolation follows it.
     */
    void interpolate(std::uint64_t dataIndex) {
        const std::size_t* pilotBins = _map->pilotBins().data();
        const std::size_t  nPilots   = _map->nPilots();
        for (std::size_t p = 0UZ; p < nPilots; ++p) {
            const std::size_t bin   = pilotBins[p];
            const Complex     ratio = detail::divide(_corrected[bin], detail::multiply(_h[bin], pilotOf(dataIndex, p)));
            _pilotMagnitude[p]      = std::abs(ratio);
            _pilotAngle[p]          = std::arg(ratio);
        }

        const float mix = _alpha;
        for (std::size_t k = 0UZ; k < _occupied.size(); ++k) {
            const Between& where    = _between[k];
            const float    lowMag   = _pilotMagnitude[where.left];
            const float    lowPhase = _pilotAngle[where.left];

            const float magnitude = lowMag + where.weight * (_pilotMagnitude[where.right] - lowMag);
            // the shorter way round, across the branch cut where the pair straddles it
            const float step  = std::remainder(_pilotAngle[where.right] - lowPhase, 2.f * std::numbers::pi_v<float>);
            const float phase = lowPhase + where.weight * step;

            const Complex     ratio = std::polar(magnitude, phase);
            const Complex     factor(1.f - mix + mix * ratio.real(), mix * ratio.imag());
            const std::size_t bin = _occupied[k];
            _h[bin]               = detail::multiply(_h[bin], factor);
            _hInv[bin]            = detail::reciprocal(_h[bin]);
        }
    }

    /// @brief The channel estimate as a measurement record, with magnitude and phase against the carrier axis.
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
            {std::pmr::string(detail::kFftLenKey), pmt::Value(std::uint64_t{_map->fftLength()})},
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
