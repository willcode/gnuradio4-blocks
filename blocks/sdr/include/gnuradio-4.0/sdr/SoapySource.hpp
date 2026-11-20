#ifndef GNURADIO_SOAPY_SOURCE_HPP
#define GNURADIO_SOAPY_SOURCE_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/thread/thread_pool.hpp>

#include <gnuradio-4.0/algorithm/SampleRateEstimator.hpp>

#include <gnuradio-4.0/sdr/DcBlocker.hpp>
#include <gnuradio-4.0/sdr/SoapyRaiiWrapper.hpp>

#include <bit>
#include <chrono>
#include <optional>
#include <thread>

namespace gr::blocks::sdr {

namespace detail {
inline bool equalWithinOnePercent(const auto& a, const auto& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](double x, double y) { return std::abs(x - y) <= 0.01 * std::max(std::abs(x), std::abs(y)); });
}
inline std::uint64_t wallClockNs() { return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()); }
} // namespace detail

GR_REGISTER_BLOCK("gr::blocks::sdr::SoapySource", gr::blocks::sdr::SoapySimpleSource, [T], [ uint8_t, int16_t, std::complex<float> ])
GR_REGISTER_BLOCK("gr::blocks::sdr::SoapyDualSource", gr::blocks::sdr::SoapyDualSource, [T], [ uint8_t, int16_t, std::complex<float> ])

template<typename T, std::size_t nPorts = std::dynamic_extent>
struct SoapySource : Block<SoapySource<T, nPorts>> {
    using Description = Doc<R"(SoapySDR source block for SDR hardware.
Supports single and multi-channel RX via SoapySDR's device-agnostic API.
Uses a dedicated IO thread to decouple USB/hardware latency from the scheduler.

Operating modes:
  clk_in connected: forwards external timing tags (GPS/PPS) with clock-offset interpolation
  clk_in disconnected: emits best-effort wall-clock timestamps on every chunk

What reaches the device: sample_rate and frequency always, because the block publishes both as stream
metadata; every other device setting only when the caller gives it a value, which leaves the device's own
gain, bandwidth, antenna, correction and AGC state unchanged. A setting the caller gives is applied
whatever that value is, so gain_mode=false switches AGC off on a device whose AGC starts on and
frequency_correction=0 resets a device's ppm. A block restored from a fully serialized settings map has
been given every setting that map holds, and a setting changed while the block runs is given by that
change.

Gain: rx_gains is one overall value per channel, which the driver distributes across its gain elements.
On a driver whose elements are attenuations that distribution is not monotonic in gain. rx_gain_elements
names the elements directly (IFGR: 20) and is applied in the order the driver lists them, after the AGC
state, because a driver may refuse a gain write while its AGC is on. Where both are given, the elements
are applied last and take precedence.

A read takes at most half the output buffer, whatever max_chunk_size is.

Tested with RTL-SDR and LimeSDR drivers.

The master clock rate is set before the sample rate. `dc_offset` and `iq_balance` are written I0, Q0, I1, Q1, ... per
channel. `stream_args`, `tune_args` and `device_settings` are comma-separated key=value lists, and `rx_gain_elements`
maps a gain element's name to its gain. The overall gain per channel is distributed by the driver. The DC blocker is an
IIR high-pass for complex<float> streams. A drift beyond `ppm_tag_threshold` emits the corrected frequency and rate.
)">;

    using TBasePort = PortOut<T>;
    using TPortType = std::conditional_t<nPorts == 1U, TBasePort, std::conditional_t<nPorts == std::dynamic_extent, std::vector<TBasePort>, std::array<TBasePort, nPorts>>>;

    gr::PortIn<std::uint8_t, Optional> clk_in;
    TPortType                          out;

    struct Settings {
        Annotated<std::string, Visible, Doc<"SoapySDR driver name">>                                      device;
        Annotated<std::string, Visible, Doc<"driver parameters">>                                         device_parameter;
        Annotated<double, Unit<"Hz">, Doc<"master clock rate (0 = auto)">>                                master_clock_rate = 0.0;
        Annotated<std::string, Doc<"clock source">>                                                       clock_source;
        Annotated<float, Unit<"Hz">, Visible, Doc<"sample rate">, Limits<Over<0.f>>>                      sample_rate  = 1'000'000.f;
        Annotated<gr::Size_t, Visible, Doc<"channel count">>                                              num_channels = 1U;
        Annotated<std::vector<std::string>, Visible, Doc<"antenna per channel">>                          rx_antennae;
        Annotated<std::vector<double>, Unit<"Hz">, Visible, Doc<"center frequency per channel">>          frequency = initDefaultValues(107'000'000.);
        Annotated<std::vector<double>, Unit<"Hz">, Visible, Doc<"RF bandwidth per channel (0 = driver)">> rx_bandwidths;
        Annotated<std::vector<double>, Unit<"dB">, Visible, Doc<"gain per channel">>                      rx_gains;
        Annotated<property_map, Unit<"dB">, Visible, Doc<"gain per element">>                             rx_gain_elements{};
        Annotated<bool, Doc<"automatic gain control">>                                                    gain_mode            = false;
        Annotated<double, Unit<"ppm">, Doc<"frequency correction">>                                       frequency_correction = 0.0;
        Annotated<bool, Doc<"automatic DC offset removal">>                                               dc_offset_mode       = false;
        Annotated<std::vector<double>, Doc<"DC offset correction">>                                       dc_offset;
        Annotated<std::vector<double>, Doc<"IQ balance correction">>                                      iq_balance;
        Annotated<std::string, Doc<"time source">>                                                        time_source;
        Annotated<double, Unit<"Hz">, Doc<"reference clock rate (0 = auto)">>                             reference_clock_rate = 0.0;
        Annotated<std::string, Doc<"stream arguments">>                                                   stream_args;
        Annotated<std::string, Doc<"tuning arguments per channel">>                                       tune_args;
        Annotated<std::string, Doc<"frontend mapping">>                                                   frontend_mapping;
        Annotated<std::string, Doc<"device settings">>                                                    device_settings;
        Annotated<std::uint32_t, Doc<"samples per read limit">, Visible>                                  max_chunk_size       = 512U << 4U;
        Annotated<std::uint32_t, Unit<"us">, Doc<"polling timeout">>                                      max_time_out_us      = 1'000;
        Annotated<gr::Size_t, Doc<"overflow limit (0 = unlimited)">>                                      max_overflow_count   = 10U;
        Annotated<gr::Size_t, Doc<"fragment limit (0 = unlimited)">>                                      max_fragment_count   = 100U;
        Annotated<bool, Doc<"overflow logging">>                                                          verbose_overflow     = false;
        Annotated<bool, Doc<"device state logging">>                                                      log_device_state     = true;
        Annotated<std::string, Doc<"free-running trigger name">>                                          trigger_name         = std::string("SDR_WALLCLOCK");
        Annotated<bool, Doc<"timing tag output">>                                                         emit_timing_tags     = true;
        Annotated<bool, Doc<"timing tag metadata">>                                                       emit_meta_info       = true;
        Annotated<float, Unit<"s">, Doc<"timing tag interval (0 = every chunk)">>                         tag_interval         = 1.0f;
        Annotated<bool, Doc<"DC blocker">>                                                                dc_blocker_enabled   = false;
        Annotated<float, Unit<"Hz">, Doc<"DC blocker cutoff">>                                            dc_blocker_cutoff    = 10.f;
        Annotated<float, Unit<"Hz">, Doc<"rate estimator cutoff (0 = off)">>                              ppm_estimator_cutoff = 0.f;
        Annotated<float, Unit<"ppm">, Doc<"drift tag threshold">>                                         ppm_tag_threshold    = 0.1f;

        GR_MAKE_REFLECTABLE(Settings, device, device_parameter, master_clock_rate, clock_source, sample_rate, num_channels, rx_antennae, frequency, rx_bandwidths, rx_gains, rx_gain_elements, gain_mode, frequency_correction, dc_offset_mode, dc_offset, iq_balance, time_source, reference_clock_rate, stream_args, tune_args, frontend_mapping, device_settings, max_chunk_size, max_time_out_us, max_overflow_count, max_fragment_count, verbose_overflow, log_device_state, trigger_name, emit_timing_tags, emit_meta_info, tag_interval, dc_blocker_enabled, dc_blocker_cutoff, ppm_estimator_cutoff, ppm_tag_threshold);
    } settings;

    GR_MAKE_REFLECTABLE(SoapySource, clk_in, out, settings);

    soapy::Device                               _device{};
    soapy::Device::Stream<T, SOAPY_SDR_RX>      _rxStream{};
    soapy::Kwargs                               _devKwargs{};
    bool                                        _ioThreadDone = true;
    std::atomic<gr::Size_t>                     _overflowCount{0U};
    std::atomic<gr::Size_t>                     _fragmentCount{0U};
    std::int64_t                                _clockOffsetNs    = 0;
    bool                                        _clockOffsetValid = false;
    std::string                                 _clockTriggerName;
    bool                                        _firstEmission  = true;
    std::uint64_t                               _lastTagTimeNs  = 0UL;
    float                                       _prevSampleRate = 0.f;
    double                                      _prevFrequency  = 0.0;
    DcBlocker                                   _dcFilterI;
    DcBlocker                                   _dcFilterQ;
    algorithm::SampleRateEstimator              _rateEstimator;
    float                                       _ppmLastEmitted   = 0.0f;
    bool                                        _clockEosReceived = false;
    std::atomic<bool>                           _ioThreadStarted{false};
    std::atomic<bool>                           _ioStopRequested{false};
    std::atomic<bool>                           _dcFilterDirty{false};
    std::atomic<bool>                           _rateEstimatorDirty{false};
    std::atomic<bool>                           _activationFailed{false};
    gr::Error                                   _activationError{}; // written before _activationFailed is set
    soapy::detail::DeviceRegistry::Registration _activation;

    // The io loop runs for as long as the block is active, so a teardown that never asked it to stop, which
    // is what a scheduler that ends in ERROR leaves behind, waits for a thread that never exits.
    struct IoThreadGuard {
        SoapySource* self;
        ~IoThreadGuard() {
            if (lifecycle::isActive(self->state())) {
                self->requestStop();
            }
            gr::atomic_ref(self->_ioThreadDone).wait(false);
        }
    };
    IoThreadGuard _ioGuard{this};

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (!std::has_single_bit(s.max_chunk_size.value)) {
            return gr::Refused(s.max_chunk_size, "must be a power of 2");
        }
        return {};
    }

    void applySettings(const Settings& s) {
        if (!_device.get()) {
            return;
        }
        if (s.frequency.changed()) {
            applyFrequency();
            if (!s.frequency->empty()) {
                this->forwardTag(tag::FREQUENCY.key(), s.frequency->front());
            }
        }
        if (s.sample_rate.changed()) {
            applySampleRate();
            this->forwardTag(tag::SAMPLE_RATE.key(), s.sample_rate.value);
            _dcFilterDirty.store(true, std::memory_order_release);
            _rateEstimatorDirty.store(true, std::memory_order_release);
        }
        if (s.rx_antennae.changed()) {
            applyAntenna();
        }
        if (s.rx_bandwidths.changed()) {
            applyBandwidth();
        }
        // The AGC state precedes the gains: a driver may refuse a gain write while its AGC is on.
        if (s.gain_mode.changed()) {
            applyGainMode();
        }
        if (s.rx_gains.changed()) {
            applyGain();
        }
        if (s.rx_gain_elements.changed()) {
            applyGainElements();
        }
        if (s.frequency_correction.changed()) {
            applyFrequencyCorrection();
        }
        if (s.dc_offset_mode.changed()) {
            applyDcOffsetMode();
        }
        if (s.dc_offset.changed()) {
            applyDcOffset();
        }
        if (s.iq_balance.changed()) {
            applyIqBalance();
        }
        if (s.device_settings.changed()) {
            applyDeviceSettings();
        }
        if (s.dc_blocker_cutoff.changed() || s.dc_blocker_enabled.changed()) {
            _dcFilterDirty.store(true, std::memory_order_release);
        }
        if (s.ppm_estimator_cutoff.changed() || s.max_chunk_size.changed()) {
            _rateEstimatorDirty.store(true, std::memory_order_release);
        }
    }

    // Throws when the device cannot be opened, configured or activated. On a shared device the stream can activate
    // after start() returns, and work() then reports a refusal as ERROR.
    void start() {
        _overflowCount.store(0U, std::memory_order_relaxed);
        _fragmentCount.store(0U, std::memory_order_relaxed);
        _clockOffsetNs    = 0;
        _clockOffsetValid = false;
        _clockTriggerName.clear();
        _firstEmission    = true;
        _lastTagTimeNs    = 0UL;
        _ppmLastEmitted   = 0.0f;
        _clockEosReceived = false;
        _ioThreadStarted.store(false, std::memory_order_relaxed);
        _ioStopRequested.store(false, std::memory_order_relaxed);
        _dcFilterDirty.store(false, std::memory_order_relaxed);
        _rateEstimatorDirty.store(false, std::memory_order_relaxed);
        _activationFailed.store(false, std::memory_order_relaxed);
        rebuildDcFilter();
        reinitDevice();
        rebuildRateEstimator();
        _activation.activate([this] {
            if (auto r = _rxStream.activate(); !r) {
                _activationError = r.error();
                _activationFailed.store(true, std::memory_order_release);
                return;
            }
            _ioThreadStarted.store(true, std::memory_order_release);
            gr::atomic_ref(_ioThreadDone).store_release(false);
            thread_pool::Manager::defaultIoPool()->execute([this]() { ioReadLoop(); });
        });
        if (_activationFailed.load(std::memory_order_acquire)) {
            failStart(_activationError.message);
        }
    }

    void stop() {
        _activation.reset();
        if (_ioThreadStarted.load(std::memory_order_acquire)) {
            gr::atomic_ref(_ioThreadDone).wait(false);
        }
        _rxStream.reset();
        _device.reset();
    }

    // Releases the device, then throws.
    [[noreturn]] void failStart(std::string_view reason, std::source_location location = std::source_location::current()) {
        stop();
        throw gr::exception(reason, location);
    }

    work::Result work(std::size_t requestedWork = std::numeric_limits<std::size_t>::max()) noexcept {
        if (!lifecycle::isActive(this->state())) {
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        if (_activationFailed.load(std::memory_order_acquire)) {
            this->emitErrorMessage("start()", _activationError);
            return {requestedWork, 0UZ, work::Status::ERROR};
        }
        if (this->disconnect_on_done && this->hasNoDownStreamConnectedChildren()) {
            this->requestStop();
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        if (_ioThreadStarted.load(std::memory_order_acquire) && gr::atomic_ref(_ioThreadDone).load_acquire()) {
            this->requestStop();
            return {requestedWork, 0UZ, work::Status::DONE};
        }
        return {requestedWork, 0UZ, work::Status::OK};
    }

    // The io thread never stops the block itself: requestStop() runs the block's own stop() on the calling thread
    // and stop() waits for the io thread to finish, so the io thread would wait for itself. Whatever the io thread
    // meets that ends the stream is recorded in _ioStopRequested, which ends the read loop, and work() makes the
    // finished io thread the stop, on the scheduler thread.
    [[nodiscard]] bool ioActive() const noexcept { return lifecycle::isActive(this->state()) && !_ioStopRequested.load(std::memory_order_acquire); }

    // The device writes each read straight into the reserved output spans. A read is the smallest of max_chunk_size,
    // half the output buffer and the room every output has, rounded down to whole MTUs where it holds one. The source
    // waits only while an output is full. A consumer waits only while its input chunk exceeds the unread samples, and
    // the output then has room. Until a read finds room, the device's own buffer holds the samples, and the device
    // reports an overflow once that buffer is full.
    [[nodiscard]] static std::size_t readSize(std::size_t maxChunkSize, std::size_t outputCapacity, std::size_t room, std::size_t mtu) noexcept {
        const std::size_t nRead = std::min({maxChunkSize, std::max(outputCapacity / 2UZ, 1UZ), room});
        return (mtu > 0UZ && nRead >= mtu) ? nRead - nRead % mtu : nRead;
    }

    // the size of the smallest output buffer
    [[nodiscard]] std::size_t outputCapacity() {
        if constexpr (nPorts == 1U) {
            return out.streamWriter().buffer().size();
        } else {
            std::size_t capacity = std::numeric_limits<std::size_t>::max();
            for (std::size_t ch = 0UZ; ch < static_cast<std::size_t>(settings.num_channels.value) && ch < out.size(); ++ch) {
                capacity = std::min(capacity, out[ch].streamWriter().buffer().size());
            }
            return capacity;
        }
    }

    void ioReadLoop() {
        thread_pool::thread::setThreadName(std::format("soapy:{}", this->name.value));

        std::size_t       nCh = static_cast<std::size_t>(settings.num_channels.value);
        const std::size_t mtu = _rxStream.mtu();

        auto& clkReader = clk_in.streamReader();
        auto& clkTagRdr = clk_in.tagReader();

        if constexpr (nPorts == 1U) {
            auto&             outWriter = out.streamWriter();
            const std::size_t capacity  = outputCapacity();

            while (ioActive()) {
                this->applyChangedSettings();
                applyDirtyFlags();

                const std::size_t nRead = readSize(settings.max_chunk_size, capacity, outWriter.available(), mtu);
                if (nRead == 0UZ) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }
                auto span = outWriter.template tryReserve<SpanReleasePolicy::ProcessNone>(nRead);
                if (span.empty()) {
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }

                int       flags   = 0;
                long long time_ns = 0;
                int       ret     = _rxStream.readStream(flags, time_ns, settings.max_time_out_us, std::span<T>(span.data(), span.size()));

                if (ret == SOAPY_SDR_TIMEOUT) {
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }
                if (ret < 0) {
                    if (!handleStreamError(ret)) {
                        break;
                    }
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }
                if (ret == 0) {
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }

                _overflowCount.store(0U, std::memory_order_relaxed);
                handleStreamFlags(flags);
                auto nSamples = static_cast<std::size_t>(ret);
                auto tWallNs  = detail::wallClockNs();

                if (settings.ppm_estimator_cutoff > 0.f) {
                    _rateEstimator.update(static_cast<double>(tWallNs) * 1e-9, nSamples);
                }

                drainClockInput(clkReader, clkTagRdr);

                if constexpr (std::is_same_v<T, std::complex<float>>) {
                    if (settings.dc_blocker_enabled) {
                        applyDcBlocker(span.data(), nSamples);
                    }
                }

                if (settings.emit_timing_tags) {
                    auto intervalNs = static_cast<std::uint64_t>(settings.tag_interval.value * 1e9f);
                    if (intervalNs == 0UL || _lastTagTimeNs == 0UL || (tWallNs - _lastTagTimeNs) >= intervalNs) {
                        emitTimingTag(nSamples, tWallNs);
                        _lastTagTimeNs = tWallNs;
                    }
                }

                span.publish(nSamples);
                this->progress->incrementAndGet();
                this->progress->notify_all();
            }
        } else {
            using WriterType = std::remove_reference_t<decltype(out[0].streamWriter())>;
            std::vector<std::reference_wrapper<WriterType>> outWriters;
            const std::size_t                               capacity = outputCapacity();
            for (std::size_t ch = 0UZ; ch < nCh && ch < out.size(); ++ch) {
                outWriters.push_back(std::ref(out[ch].streamWriter()));
            }

            while (ioActive()) {
                this->applyChangedSettings();
                applyDirtyFlags();

                std::size_t room = std::numeric_limits<std::size_t>::max();
                for (auto& w : outWriters) {
                    room = std::min(room, w.get().available());
                }
                const std::size_t nRead = readSize(settings.max_chunk_size, capacity, room, mtu);
                if (nRead == 0UZ) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }

                using OutSpanType = decltype(outWriters[0].get().template tryReserve<SpanReleasePolicy::ProcessNone>(0UZ));
                std::vector<OutSpanType> outSpans;
                outSpans.reserve(outWriters.size());
                for (auto& w : outWriters) {
                    outSpans.push_back(w.get().template tryReserve<SpanReleasePolicy::ProcessNone>(nRead));
                }
                if (std::ranges::any_of(outSpans, [](const auto& s) { return s.empty(); })) {
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }

                int       flags   = 0;
                long long time_ns = 0;
                int       ret     = _rxStream.readStreamIntoBufferList(flags, time_ns, static_cast<long>(settings.max_time_out_us.value), outSpans);

                if (ret == SOAPY_SDR_TIMEOUT) {
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }
                if (ret < 0) {
                    if (!handleStreamError(ret)) {
                        break;
                    }
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }
                if (ret == 0) {
                    drainClockInput(clkReader, clkTagRdr);
                    continue;
                }

                _overflowCount.store(0U, std::memory_order_relaxed);
                handleStreamFlags(flags);
                auto nSamples = static_cast<std::size_t>(ret);
                auto tWallNs  = detail::wallClockNs();

                if (settings.ppm_estimator_cutoff > 0.f) {
                    _rateEstimator.update(static_cast<double>(tWallNs) * 1e-9, nSamples);
                }

                drainClockInput(clkReader, clkTagRdr);

                for (auto& s : outSpans) {
                    s.publish(nSamples);
                }

                if (settings.emit_timing_tags) {
                    auto intervalNs = static_cast<std::uint64_t>(settings.tag_interval.value * 1e9f);
                    if (intervalNs == 0UL || _lastTagTimeNs == 0UL || (tWallNs - _lastTagTimeNs) >= intervalNs) {
                        emitTimingTag(nSamples, tWallNs);
                        _lastTagTimeNs = tWallNs;
                    }
                }

                this->progress->incrementAndGet();
                this->progress->notify_all();
            }
        }

        this->publishEoS();
        if (clk_in.isConnected()) {
            std::ignore = clk_in.disconnect();
        }

        if (auto r = _rxStream.deactivate(); !r) {
            this->emitErrorMessage("ioReadLoop()", r.error());
        }

        gr::atomic_ref(_ioThreadDone).store_release(true);
        gr::atomic_ref(_ioThreadDone).notify_all();
    }

    /// Takes the clock input's tags and samples. A trigger time sets the offset of the clock from the wall clock, a
    /// trigger name sets the name the timing tags carry, and an end-of-stream tag ends the read loop. The samples are
    /// discarded. The read loop calls this on every pass, as the end-of-stream tag can arrive alone after the last
    /// sample.
    void drainClockInput(auto& clkReader, auto& clkTagRdr) {
        if (!clk_in.isConnected()) {
            return;
        }

        auto        tagData       = clkTagRdr.get(clkTagRdr.available());
        std::size_t nTagsConsumed = 0UZ;

        for (const auto& clkTag : tagData) {
            ++nTagsConsumed;
            if (auto it = clkTag.map.find(std::pmr::string(tag::TRIGGER_TIME.key())); it != clkTag.map.end()) {
                if (auto* timePtr = it->second.template get_if<std::uint64_t>()) {
                    auto triggerUtcNs = static_cast<std::int64_t>(*timePtr);
                    _clockOffsetNs    = triggerUtcNs - static_cast<std::int64_t>(detail::wallClockNs());
                    _clockOffsetValid = true;
                }
            }
            if (auto it = clkTag.map.find(std::pmr::string(tag::TRIGGER_NAME.key())); it != clkTag.map.end()) {
                if (auto* namePtr = it->second.template get_if<std::pmr::string>()) {
                    if (!namePtr->empty()) {
                        _clockTriggerName = std::string(*namePtr);
                    }
                }
            }
            static const std::pmr::string kEosKey(tag::END_OF_STREAM.key());
            if (clkTag.map.contains(kEosKey)) {
                _clockEosReceived = true;
            }
        }

        std::ignore = tagData.consume(nTagsConsumed);
        if (const std::size_t nAvailable = clkReader.available(); nAvailable > 0UZ) {
            auto clkSpan = clkReader.get(nAvailable);
            std::ignore  = clkSpan.consume(nAvailable);
        }

        if (_clockEosReceived) {
            _ioStopRequested.store(true, std::memory_order_release);
        }
    }

    void emitTimingTag(std::size_t nSamples, std::uint64_t tWallNs) {
        auto tUtcLastNs      = static_cast<std::int64_t>(tWallNs) + _clockOffsetNs;
        auto chunkDurationNs = static_cast<std::int64_t>(static_cast<double>(nSamples - 1) / static_cast<double>(settings.sample_rate.value) * 1e9);
        auto tUtcFirstNs     = static_cast<std::uint64_t>(std::max(std::int64_t{0}, tUtcLastNs - chunkDurationNs));

        bool hasClock = clk_in.isConnected() && !_clockTriggerName.empty();

        auto publishTag = [&](auto& port) {
            auto tagMap = port.makeTagMap();
            tag::put(tagMap, tag::TRIGGER_NAME, hasClock ? _clockTriggerName : settings.trigger_name.value);
            tag::put(tagMap, tag::TRIGGER_TIME, tUtcFirstNs);
            tag::put(tagMap, tag::TRIGGER_OFFSET, 0.f);

            if (settings.emit_meta_info) {
                auto metaInfo = port.makeTagMap();
                tag::put(metaInfo, "trigger_source", std::string("SoapySource"));

                std::string clockSource = (_clockOffsetValid && !_clockTriggerName.empty()) ? _clockTriggerName : std::string("wallclock");
                tag::put(metaInfo, "clock_source", std::move(clockSource));

                if (_clockOffsetValid) {
                    tag::put(metaInfo, "clock_offset_ns", _clockOffsetNs);
                }

                emitChangedParams(metaInfo);
                tag::put(tagMap, tag::TRIGGER_META_INFO, std::move(metaInfo));
            }

            if (_rateEstimator._initialised && settings.ppm_estimator_cutoff > 0.f) {
                float ppmNow = _rateEstimator.estimatedPpm();
                tag::put(tagMap, tag::SAMPLE_RATE, static_cast<float>(_rateEstimator.estimatedRate()));
                if (!settings.frequency->empty()) {
                    tag::put(tagMap, tag::FREQUENCY, settings.frequency->front() * (1.0 + static_cast<double>(ppmNow) * 1e-6));
                }
                tag::put(tagMap, "ppm_error", ppmNow);
                _ppmLastEmitted = ppmNow;
            }

            port.publishTag(std::move(tagMap), 0UZ);
        };

        if constexpr (nPorts == 1U) {
            publishTag(out);
        } else {
            for (std::size_t ch = 0UZ; ch < out.size(); ++ch) {
                publishTag(out[ch]);
            }
        }
    }

    void emitChangedParams(property_map& metaInfo) {
        if (_firstEmission || _prevSampleRate != settings.sample_rate.value) {
            tag::put(metaInfo, "sample_rate", static_cast<double>(settings.sample_rate.value));
            _prevSampleRate = settings.sample_rate.value;
        }
        if (_firstEmission || (!settings.frequency->empty() && _prevFrequency != settings.frequency->front())) {
            if (!settings.frequency->empty()) {
                tag::put(metaInfo, "frequency", settings.frequency->front());
                _prevFrequency = settings.frequency->front();
            }
        }
        _firstEmission = false;
    }

    void reinitDevice() {
        _rxStream.reset();
        _devKwargs = soapy::Kwargs{{"driver", settings.device.value}};
        if (!settings.device_parameter->empty()) {
            _devKwargs.merge(soapy::parseKwargsString(settings.device_parameter.value));
        }
        _activation = soapy::detail::DeviceRegistry::Registration(_devKwargs);

        auto devResult = soapy::Device::make(_devKwargs);
        if (!devResult) {
            failStart(devResult.error().message);
        }
        _device = std::move(*devResult);

        std::size_t nChannelMax    = _device.getNumChannels(SOAPY_SDR_RX);
        std::size_t nChannelNeeded = (nPorts != std::dynamic_extent) ? nPorts : static_cast<std::size_t>(settings.num_channels.value);
        if (nChannelMax < nChannelNeeded) {
            failStart(std::format("channel mismatch: need {} but device has {}", nChannelNeeded, nChannelMax));
        }

        // The order is what the drivers require: the frontend mapping decides which physical channel an
        // index names, a device setting may share a control with a gain element, a driver may derive its
        // filter from the sample rate, and a driver may refuse a gain write while its AGC is on.
        applyFrontendMapping();
        applyClockConfig();
        applyDeviceSettings();
        applySampleRate();
        applyBandwidth();
        applyAntenna();
        applyFrequency();
        // A bool or a zero carries no "not given" value of its own, so what the caller wrote decides
        // whether these three reach the device. A later change carries that decision in itself.
        if (this->isGiven(settings.frequency_correction)) {
            applyFrequencyCorrection();
        }
        if (this->isGiven(settings.gain_mode)) {
            applyGainMode();
        }
        applyGain();
        applyGainElements();
        if (this->isGiven(settings.dc_offset_mode)) {
            applyDcOffsetMode();
        }
        applyDcOffset();
        applyIqBalance();
        logDeviceState();

        auto        supportedFormats = _device.getStreamFormats(SOAPY_SDR_RX, 0);
        const char* requestedFormat  = soapy::detail::toSoapySDRFormat<T>();
        if (!supportedFormats.empty() && std::ranges::find(supportedFormats, std::string(requestedFormat)) == supportedFormats.end()) {
            failStart(std::format("format '{}' not supported (available: {})", requestedFormat, gr::join(supportedFormats, ", ")));
        }

        std::vector<gr::Size_t> channelIndices(settings.num_channels);
        std::iota(channelIndices.begin(), channelIndices.end(), gr::Size_t{0});
        soapy::Kwargs parsedStreamArgs = settings.stream_args->empty() ? soapy::Kwargs{} : soapy::parseKwargsString(settings.stream_args.value);
        auto          streamResult     = _device.setupStream<T, SOAPY_SDR_RX>(channelIndices, parsedStreamArgs);
        if (!streamResult) {
            failStart(std::format("{} (requested: {})", streamResult.error(), requestedFormat));
        }
        _rxStream = std::move(*streamResult);
    }

    void applyClockConfig() {
        if (!settings.clock_source->empty()) {
            if (auto r = _device.setClockSource(settings.clock_source.value); !r) {
                this->emitErrorMessage("applyClockConfig()", r.error());
            }
        }
        if (!settings.time_source->empty()) {
            if (auto r = _device.setTimeSource(settings.time_source.value); !r) {
                this->emitErrorMessage("applyClockConfig()", r.error());
            }
        }
        if (settings.master_clock_rate > 0.0) {
            if (auto r = _device.setMasterClockRate(settings.master_clock_rate); !r) {
                this->emitErrorMessage("applyClockConfig()", r.error());
            }
        }
        if (settings.reference_clock_rate > 0.0) {
            if (auto r = _device.setReferenceClockRate(settings.reference_clock_rate); !r) {
                this->emitErrorMessage("applyClockConfig()", r.error());
            }
        }
    }

    void applySampleRate() {
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            this->emitErrorMessageIfAny("applySampleRate()", _device.setSampleRate(SOAPY_SDR_RX, i, static_cast<double>(settings.sample_rate)));
        }
        std::vector<double> actual;
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            actual.push_back(_device.getSampleRate(SOAPY_SDR_RX, i));
        }
        if (!detail::equalWithinOnePercent(actual, std::vector<double>(settings.num_channels, static_cast<double>(settings.sample_rate)))) {
            this->emitErrorMessage("applySampleRate()", std::format("mismatch: set {} vs actual {}", settings.sample_rate, gr::join(actual, ", ")));
        }
    }

    // A driver accepts an antenna name it does not have without reporting an error, so the name is checked
    // against the device's own list first.
    void applyAntenna() {
        if (settings.rx_antennae->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            const auto& ant = settings.rx_antennae->at(std::min(static_cast<std::size_t>(i), settings.rx_antennae->size() - 1UZ));
            if (ant.empty()) {
                continue;
            }
            const auto available = _device.listAvailableAntennas(SOAPY_SDR_RX, i);
            if (!available.empty() && std::ranges::find(available, ant) == available.end()) {
                this->emitErrorMessage("applyAntenna()", std::format("channel {} has no antenna '{}' (device has: {})", i, ant, gr::join(available, ", ")));
                continue;
            }
            this->emitErrorMessageIfAny("applyAntenna()", _device.setAntenna(SOAPY_SDR_RX, i, ant));
        }
    }

    // The componentless setFrequency distributes the frequency over every component the driver lists, which
    // writes the tuning residual into the correction component of a driver that has one. The RF component is
    // named where the driver lists it.
    void applyFrequency() {
        if (settings.frequency->empty()) {
            return;
        }
        const soapy::Kwargs tuneArgs = settings.tune_args->empty() ? soapy::Kwargs{} : soapy::parseKwargsString(settings.tune_args.value);
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            double     freq       = settings.frequency->at(std::min(static_cast<std::size_t>(i), settings.frequency->size() - 1UZ));
            const auto components = _device.listFrequencyComponents(SOAPY_SDR_RX, i);
            const bool hasRf      = std::ranges::find(components, std::string("RF")) != components.end();
            if (hasRf) {
                this->emitErrorMessageIfAny("applyFrequency()", _device.setFrequencyComponent(SOAPY_SDR_RX, i, "RF", freq, tuneArgs));
            } else {
                this->emitErrorMessageIfAny("applyFrequency()", _device.setCenterFrequency(SOAPY_SDR_RX, i, freq, tuneArgs));
            }
        }
        std::vector<double> actual;
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            actual.push_back(_device.getCenterFrequency(SOAPY_SDR_RX, i));
        }
        if (!detail::equalWithinOnePercent(actual, settings.frequency.value)) {
            this->emitErrorMessage("applyFrequency()", std::format("mismatch: set {} vs actual {}", gr::join(settings.frequency.value, ", "), gr::join(actual, ", ")));
        }
    }

    void applyBandwidth() {
        if (settings.rx_bandwidths->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            double bw = settings.rx_bandwidths->at(std::min(static_cast<std::size_t>(i), settings.rx_bandwidths->size() - 1UZ));
            this->emitErrorMessageIfAny("applyBandwidth()", _device.setBandwidth(SOAPY_SDR_RX, i, bw));
        }
    }

    void applyGain() {
        if (settings.rx_gains->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            double g = settings.rx_gains->at(std::min(static_cast<std::size_t>(i), settings.rx_gains->size() - 1UZ));
            if (!withinRange(_device.getGainRange(SOAPY_SDR_RX, i), g)) {
                this->emitErrorMessage("applyGain()", std::format("channel {}: {} dB is outside the device's overall gain range {}", i, g, _device.getGainRange(SOAPY_SDR_RX, i)));
                continue;
            }
            this->emitErrorMessageIfAny("applyGain()", _device.setGain(SOAPY_SDR_RX, i, g));
        }
    }

    // Elements are applied in the order the driver lists them, which is the order the driver itself uses when
    // it distributes an overall gain across them.
    void applyGainElements() {
        if (settings.rx_gain_elements->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            const auto available = _device.listAvailableGainElements(SOAPY_SDR_RX, i);
            for (const auto& name : available) {
                const auto entry = settings.rx_gain_elements->find(name);
                if (entry == settings.rx_gain_elements->end()) {
                    continue;
                }
                const auto gain = pmt::convert_safely<double>(entry->second);
                if (!gain) {
                    this->emitErrorMessage("applyGainElements()", std::format("gain element '{}' needs a number: {}", name, gain.error()));
                    continue;
                }
                if (!withinRange(_device.getGainRange(SOAPY_SDR_RX, i, name), *gain)) {
                    this->emitErrorMessage("applyGainElements()", std::format("channel {}: {} dB is outside element '{}' range {}", i, *gain, name, _device.getGainRange(SOAPY_SDR_RX, i, name)));
                    continue;
                }
                this->emitErrorMessageIfAny("applyGainElements()", _device.setGain(SOAPY_SDR_RX, i, *gain, name));
            }
            for (const auto& entry : settings.rx_gain_elements.value) {
                if (std::ranges::find(available, std::string(entry.first)) == available.end()) {
                    this->emitErrorMessage("applyGainElements()", std::format("channel {} has no gain element '{}' (device has: {})", i, entry.first, gr::join(available, ", ")));
                }
            }
        }
    }

    void applyGainMode() {
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            if (!_device.hasAutomaticGainControl(SOAPY_SDR_RX, i)) {
                continue;
            }
            this->emitErrorMessageIfAny("applyGainMode()", _device.setAutomaticGainControl(SOAPY_SDR_RX, i, settings.gain_mode));
        }
    }

    void applyFrequencyCorrection() {
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            if (!_device.hasFrequencyCorrection(SOAPY_SDR_RX, i)) {
                continue;
            }
            this->emitErrorMessageIfAny("applyFrequencyCorrection()", _device.setFrequencyCorrection(SOAPY_SDR_RX, i, settings.frequency_correction));
        }
    }

    void applyDcOffsetMode() {
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            if (!_device.hasDCOffsetMode(SOAPY_SDR_RX, i)) {
                continue;
            }
            this->emitErrorMessageIfAny("applyDcOffsetMode()", _device.setDCOffsetMode(SOAPY_SDR_RX, i, settings.dc_offset_mode));
        }
    }

    void applyDcOffset() {
        if (settings.dc_offset->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            if (!_device.hasDCOffset(SOAPY_SDR_RX, i)) {
                continue;
            }
            auto idx = static_cast<std::size_t>(i) * 2UZ;
            if (idx + 1UZ >= settings.dc_offset->size()) {
                break;
            }
            this->emitErrorMessageIfAny("applyDcOffset()", _device.setDCOffset(SOAPY_SDR_RX, i, settings.dc_offset->at(idx), settings.dc_offset->at(idx + 1UZ)));
        }
    }

    void applyIqBalance() {
        if (settings.iq_balance->empty()) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            if (!_device.hasIQBalance(SOAPY_SDR_RX, i)) {
                continue;
            }
            auto idx = static_cast<std::size_t>(i) * 2UZ;
            if (idx + 1UZ >= settings.iq_balance->size()) {
                break;
            }
            this->emitErrorMessageIfAny("applyIqBalance()", _device.setIQBalance(SOAPY_SDR_RX, i, settings.iq_balance->at(idx), settings.iq_balance->at(idx + 1UZ)));
        }
    }

    void applyFrontendMapping() {
        if (!settings.frontend_mapping->empty()) {
            _device.setFrontendMapping(SOAPY_SDR_RX, settings.frontend_mapping.value);
        }
    }

    // A key the device does not have is ignored by the driver rather than reported, so keys are checked
    // against the device's own list. That list is read only when the caller supplied settings: on some
    // drivers getSettingInfo() reopens the device.
    void applyDeviceSettings() {
        if (settings.device_settings->empty()) {
            return;
        }
        std::vector<std::string> known;
        for (const auto& info : _device.getSettingInfo()) {
            known.push_back(info.key);
        }
        for (const auto& [key, value] : soapy::parseKwargsString(settings.device_settings.value)) {
            if (std::ranges::find(known, key) == known.end()) {
                this->emitErrorMessage("applyDeviceSettings()", known.empty() ? std::format("the device has no settings, so '{}' cannot be written", key) : std::format("the device has no setting '{}' (it has: {})", key, gr::join(known, ", ")));
                continue;
            }
            this->emitErrorMessageIfAny("applyDeviceSettings()", _device.writeSetting(key, value));
        }
    }

    // A driver that reports a degenerate range gives nothing to check against.
    [[nodiscard]] static bool withinRange(const soapy::Range& range, double value) { return range.maximum <= range.minimum || (value >= range.minimum && value <= range.maximum); }

    void logDeviceState() {
        if (!settings.log_device_state) {
            return;
        }
        for (gr::Size_t i = 0U; i < settings.num_channels; i++) {
            std::string gains;
            for (const auto& name : _device.listAvailableGainElements(SOAPY_SDR_RX, i)) {
                gains += std::format("{}{}={:g}", gains.empty() ? "" : " ", name, _device.getGain(SOAPY_SDR_RX, i, name));
            }
            std::string written;
            for (const auto& [key, value] : settings.device_settings->empty() ? soapy::Kwargs{} : soapy::parseKwargsString(settings.device_settings.value)) {
                written += std::format("{}{}={}", written.empty() ? "" : " ", key, _device.readSetting(key));
            }
            std::println(stderr, "[SoapySource] {} ch{}: rate={:g} freq={:g} bw={:g} antenna={} agc={} gain={:g} [{}] corr={:g} settings [{}]", settings.device.value, i, _device.getSampleRate(SOAPY_SDR_RX, i), _device.getCenterFrequency(SOAPY_SDR_RX, i), _device.getBandwidth(SOAPY_SDR_RX, i), _device.getAntenna(SOAPY_SDR_RX, i), _device.isAutomaticGainControl(SOAPY_SDR_RX, i), _device.getGain(SOAPY_SDR_RX, i), gains, _device.getFrequencyCorrection(SOAPY_SDR_RX, i), written);
        }
    }

    void rebuildDcFilter() {
        if constexpr (std::is_same_v<T, std::complex<float>>) {
            if (settings.dc_blocker_enabled && settings.dc_blocker_cutoff > 0.f && settings.sample_rate > 0.f) {
                _dcFilterI.setCutoff(settings.dc_blocker_cutoff.value, settings.sample_rate.value);
                _dcFilterQ.setCutoff(settings.dc_blocker_cutoff.value, settings.sample_rate.value);
            }
        }
    }

    void rebuildRateEstimator() {
        if (settings.ppm_estimator_cutoff > 0.f) {
            // one update per read, each read at the size readSize() gives when every output has room
            const std::size_t nRead         = readSize(settings.max_chunk_size, outputCapacity(), outputCapacity(), _rxStream.mtu());
            double            nomRate       = static_cast<double>(settings.sample_rate.value);
            double            updateHz      = (nomRate > 0.0) ? (nomRate / static_cast<double>(nRead)) : 250.0;
            _rateEstimator.filter_cutoff_hz = settings.ppm_estimator_cutoff;
            _rateEstimator.reset(nomRate, updateHz);
        }
    }

    void applyDcBlocker(std::complex<float>* samples, std::size_t nSamples) {
        for (std::size_t i = 0UZ; i < nSamples; ++i) {
            float filteredI = _dcFilterI.processOne(samples[i].real());
            float filteredQ = _dcFilterQ.processOne(samples[i].imag());
            samples[i]      = {filteredI, filteredQ};
        }
    }

    void emitOverflowTag() {
        if constexpr (nPorts == 1U) {
            auto tagMap = out.makeTagMap();
            tag::put(tagMap, tag::RX_OVERFLOW, true);
            out.publishTag(std::move(tagMap), 0UZ);
        } else {
            for (std::size_t ch = 0UZ; ch < out.size(); ++ch) {
                auto tagMap = out[ch].makeTagMap();
                tag::put(tagMap, tag::RX_OVERFLOW, true);
                out[ch].publishTag(std::move(tagMap), 0UZ);
            }
        }
    }

    bool handleStreamError(int ret) {
        switch (ret) {
        case SOAPY_SDR_OVERFLOW: {
            auto count = _overflowCount.fetch_add(1U, std::memory_order_relaxed) + 1U;
            if (settings.verbose_overflow) {
                std::println(stderr, "[SoapySource] OVERFLOW #{}", count);
            }
            emitOverflowTag();
            if (settings.max_overflow_count > 0 && count >= settings.max_overflow_count) {
                this->emitErrorMessage("ioReadLoop()", std::format("OVERFLOW: {} of max {}", count, settings.max_overflow_count));
                _ioStopRequested.store(true, std::memory_order_release);
                return false;
            }
            return true;
        }
        case SOAPY_SDR_CORRUPTION:
            this->emitErrorMessage("ioReadLoop()", "CORRUPTION");
            _ioStopRequested.store(true, std::memory_order_release);
            return false;
        default:
            this->emitErrorMessage("ioReadLoop()", std::format("stream error: {}", ret));
            _ioStopRequested.store(true, std::memory_order_release);
            return false;
        }
    }

    void handleStreamFlags(int flags) {
        if (settings.max_fragment_count > 0 && (flags & SOAPY_SDR_MORE_FRAGMENTS)) {
            auto count = _fragmentCount.fetch_add(1U, std::memory_order_relaxed) + 1U;
            if (count > settings.max_fragment_count) {
                this->emitErrorMessage("ioReadLoop()", std::format("MORE_FRAGMENTS: {} of max {}", count, settings.max_fragment_count));
                _ioStopRequested.store(true, std::memory_order_release);
            }
        } else {
            _fragmentCount.store(0U, std::memory_order_relaxed);
        }
    }

    void applyDirtyFlags() {
        if (_dcFilterDirty.exchange(false, std::memory_order_acquire)) {
            rebuildDcFilter();
        }
        if (_rateEstimatorDirty.exchange(false, std::memory_order_acquire)) {
            rebuildRateEstimator();
        }
    }

    template<typename U>
    static std::vector<U> initDefaultValues(U initialValue) {
        if constexpr (nPorts != std::dynamic_extent) {
            return std::vector<U>(nPorts, initialValue);
        } else {
            return std::vector<U>(1, initialValue);
        }
    }
};

template<typename T>
using SoapySimpleSource = SoapySource<T, 1UZ>;
template<typename T>
using SoapyDualSource = SoapySource<T, 2UZ>;

} // namespace gr::blocks::sdr

#endif // GNURADIO_SOAPY_SOURCE_HPP
