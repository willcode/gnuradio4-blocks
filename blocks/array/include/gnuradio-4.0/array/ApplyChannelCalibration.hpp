#ifndef GNURADIO_ARRAY_APPLY_CHANNEL_CALIBRATION_HPP
#define GNURADIO_ARRAY_APPLY_CHANNEL_CALIBRATION_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

namespace gr::blocks::array {

GR_REGISTER_BLOCK(gr::blocks::array::ApplyChannelCalibration, [T], [ std::complex<float>, std::complex<double> ])

template<typename T>
struct ApplyChannelCalibration : Block<ApplyChannelCalibration<T>, NoTagPropagation> {
    using Description = Doc<R""(@brief Applies a complex gain and an integer delay to each of N channels.

Input `in#k` reaches output `out#k` multiplied by `gains[k]` and delayed by `delays[k]` samples against the
other channels. The block pays the longest delay D as a lead on every output: output `k` at index `n` is
`gains[k] * in#k[n + D - delays[k]]`. The first `D - delays[k]` samples of input `k` are dropped, and no
output carries inserted zeros. An empty `gains` is a gain of 1 on every channel, and an empty `delays` is no
delay. A list of another length than N, or a negative delay, ends the run with an error that names it.
`gains` takes effect from the next sample processed. `delays` takes effect at the start, and a change while
running ends the run with an error. A tag on input `k` reaches output `k` at the output index of its sample.
A tag on a dropped sample reaches the output at its first sample.
)"">;

    std::vector<PortIn<T>>  in  = std::vector<PortIn<T>>(2UZ);
    std::vector<PortOut<T>> out = std::vector<PortOut<T>>(2UZ);

    Annotated<gr::Size_t, "n_channels", Visible, Doc<"number of channels N">, Limits<1U, 1024U>> n_channels = 2U;
    Annotated<std::vector<T>, "gains", Visible, Doc<"complex gain per channel">>                 gains{};
    Annotated<std::vector<std::int32_t>, "delays", Visible, Doc<"delay per channel in samples">> delays{};

    GR_MAKE_REFLECTABLE(ApplyChannelCalibration, in, out, n_channels, gains, delays);

    bool                       _running = false;
    std::vector<T>             _gains{};
    std::vector<std::size_t>   _skip{};    ///< samples of each input still to drop
    std::vector<std::size_t>   _delays{};  ///< delays in force since the start
    std::vector<property_map>  _pending{}; ///< tags of dropped samples, per channel
    std::optional<std::string> _refusal{};

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& newSettings) {
        if (!_running && newSettings.contains("n_channels")) {
            in.resize(static_cast<std::size_t>(n_channels.value));
            out.resize(static_cast<std::size_t>(n_channels.value));
        }
        if (_running && newSettings.contains("delays")) {
            _refusal = std::string("delays take effect at the start and cannot change while running");
            return;
        }
        _refusal = configure(_running);
    }

    void start() {
        _running = false;
        _refusal = configure(false);
        _running = true;
    }

    void stop() { _running = false; }

    /// Takes the gains, and at the start the delays, or returns the error that refuses them.
    [[nodiscard]] std::optional<std::string> configure(bool running) {
        const std::size_t n = in.size();
        if (!gains->empty() && gains->size() != n) {
            return std::format("gains holds {} entries for {} channels", gains->size(), n);
        }
        if (!delays->empty() && delays->size() != n) {
            return std::format("delays holds {} entries for {} channels", delays->size(), n);
        }
        for (const std::int32_t delay : delays.value) {
            if (delay < 0) {
                return std::format("delays holds the negative delay {}", delay);
            }
        }
        _gains = gains->empty() ? std::vector<T>(n, T{1}) : gains.value;
        if (!running) {
            _delays.assign(n, 0UZ);
            for (std::size_t k = 0UZ; k < delays->size(); ++k) {
                _delays[k] = static_cast<std::size_t>(delays.value[k]);
            }
            const std::size_t lead = std::ranges::max(_delays);
            _skip.resize(n);
            for (std::size_t k = 0UZ; k < n; ++k) {
                _skip[k] = lead - _delays[k];
            }
            _pending.assign(n, property_map{});
        }
        return std::nullopt;
    }

    template<InputSpanLike TInSpan, OutputSpanLike TOutSpan>
    [[nodiscard]] work::Status processBulk(std::span<TInSpan>& inputs, std::span<TOutSpan>& outputs) {
        if (_refusal.has_value()) {
            this->emitErrorMessage("processBulk", *_refusal);
            return work::Status::ERROR;
        }
        if (std::ranges::any_of(_skip, [](std::size_t s) { return s > 0UZ; })) {
            for (std::size_t k = 0UZ; k < inputs.size(); ++k) {
                const std::size_t dropped = std::min(_skip[k], inputs[k].size());
                collectTags(inputs[k], 0UZ, dropped, _pending[k]);
                std::ignore = inputs[k].consume(dropped);
                _skip[k] -= dropped;
            }
            for (auto& output : outputs) {
                output.publish(0UZ);
            }
            return work::Status::OK;
        }

        std::size_t n = std::numeric_limits<std::size_t>::max();
        for (const auto& input : inputs) {
            n = std::min(n, input.size());
        }
        for (const auto& output : outputs) {
            n = std::min(n, output.size());
        }
        for (std::size_t k = 0UZ; k < inputs.size(); ++k) {
            auto&       output = outputs[k];
            const auto& input  = inputs[k];
            const T     gain   = _gains[k];
            for (std::size_t i = 0UZ; i < n; ++i) {
                output[i] = gain * input[i];
            }
            if (!_pending[k].empty() && n > 0UZ) {
                output.publishTag(std::move(_pending[k]), 0UZ);
                _pending[k] = property_map{};
            }
            for (const auto& [relIndex, tagMap] : inputs[k].tags()) {
                if (relIndex >= 0 && static_cast<std::size_t>(relIndex) < n) {
                    output.publishTag(tagMap.get(), static_cast<std::size_t>(relIndex));
                }
            }
        }
        for (auto& input : inputs) {
            std::ignore = input.consume(n);
        }
        for (auto& output : outputs) {
            output.publish(n);
        }
        return work::Status::OK;
    }

    /// Merges the tags of samples [from, to) of `input` into `merged`.
    static void collectTags(auto& input, std::size_t from, std::size_t to, property_map& merged) {
        for (const auto& [relIndex, tagMap] : input.tags()) {
            if (relIndex < 0 || static_cast<std::size_t>(relIndex) < from || static_cast<std::size_t>(relIndex) >= to) {
                continue;
            }
            for (const auto& [key, value] : tagMap.get()) {
                merged.insert_or_assign(key, value);
            }
        }
    }
};

} // namespace gr::blocks::array

#endif // GNURADIO_ARRAY_APPLY_CHANNEL_CALIBRATION_HPP
