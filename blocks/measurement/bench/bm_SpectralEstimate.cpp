#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include <gnuradio-4.0/measurement/SpectralEstimate.hpp>
#include <gnuradio-4.0/testing/BenchInterleaved.hpp>
#include <gnuradio-4.0/testing/TestSpans.hpp>

namespace {

using gr::blocks::measurement::Spectrogram;
using gr::blocks::measurement::WelchPsd;
using gr::blocks::testing::bench::Arm;
using CF = std::complex<float>;

constexpr std::size_t kSamplesPerCall = 1UZ << 16;
constexpr std::size_t kMaxRecords     = 256UZ; // more than the shortest transform and hop can produce in one call
constexpr std::size_t kRepeats        = 9UZ;
constexpr float       kSampleRate     = 1.0e6f;

/// Built in place and started here, so a block that is not movable is measured the same way as one that is.
template<typename TBlock>
void init(TBlock& block) {
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    block.start();
}

template<typename T>
[[nodiscard]] std::vector<T> tone() {
    std::vector<T>   data(kSamplesPerCall);
    constexpr double twoPi = 2. * std::numbers::pi;
    for (std::size_t k = 0UZ; k < data.size(); ++k) {
        const double phase = twoPi * 0.1231 * static_cast<double>(k);
        if constexpr (std::is_same_v<T, CF>) {
            data[k] = CF(static_cast<float>(0.7 * std::cos(phase)), static_cast<float>(0.7 * std::sin(phase)));
        } else {
            data[k] = static_cast<float>(0.7 * std::cos(phase));
        }
    }
    return data;
}

/// @brief One call of a block over the whole sample vector, returning something derived from what it published.
template<typename TBlock, typename T>
[[nodiscard]] double sweep(TBlock& block, std::span<const T> input, std::span<gr::DataSet<float>> records) {
    namespace shim = gr::blocks::testing::span;
    shim::InputSpan<T>                   inSpan(input);
    shim::OutputSpan<gr::DataSet<float>> outSpan(records);
    std::ignore = block.processBulk(inSpan, outSpan);
    return outSpan.count == 0UZ ? 0.0 : static_cast<double>(records[0UZ].signal_values[0UZ]);
}

/// @brief The window, the transform and the power accumulation with no record built: the floor a record is measured
/// against, so criterion 7's question — whether emission dominates the transform — is answered by a difference.
[[nodiscard]] double segmentsOnly(gr::blocks::measurement::detail::SegmentAccumulator<CF>& core, std::span<const CF> input) {
    // The pending buffer is topped up a segment at a time, exactly as the block's own fold does: filling it with the
    // whole call and erasing a hop at a time would make the arm quadratic and measure the wrong thing.
    std::size_t taken = 0UZ;
    for (;;) {
        while (core.pending.size() >= core.fftSize) {
            core.accumulateFront();
            if (core.segments >= core.nAverages) {
                core.segments = 0UZ; // the accumulator is reused in place; nothing is emitted
            }
        }
        if (taken == input.size()) {
            return static_cast<double>(core.accumulator[0UZ]);
        }
        const std::size_t take = std::min(core.fftSize - core.pending.size(), input.size() - taken);
        core.pending.insert(core.pending.end(), input.begin() + static_cast<std::ptrdiff_t>(taken), input.begin() + static_cast<std::ptrdiff_t>(taken + take));
        taken += take;
    }
}

} // namespace

int main() {
    const std::vector<CF>    complexInput = tone<CF>();
    const std::vector<float> realInput    = tone<float>();

    std::vector<gr::DataSet<float>> records(kMaxRecords);

    gr::blocks::measurement::detail::SegmentAccumulator<CF> floorCore;
    floorCore.configure(1024UZ, 512UZ, 16UZ, false, gr::algorithm::window::Type::Hann, kSampleRate);

    WelchPsd<CF>    welch1024({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    WelchPsd<CF>    welchEvery({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{1U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    WelchPsd<CF>    welch8192({{"fft_size", gr::Size_t{8192U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    WelchPsd<float> welchReal({{"fft_size", gr::Size_t{1024U}}, {"n_averages", gr::Size_t{16U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    Spectrogram<CF> spectrogram({{"fft_size", gr::Size_t{1024U}}, {"overlap", 0.5}, {"sample_rate", kSampleRate}});
    init(welch1024);
    init(welchEvery);
    init(welch8192);
    init(welchReal);
    init(spectrogram);

    std::vector<Arm> arms{
        {"window + transform, no record, 1024", [&] { return segmentsOnly(floorCore, std::span<const CF>(complexInput)); }},
        {"WelchPsd<complex> 1024, 16 averages", [&] { return sweep(welch1024, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 1024, 1 average", [&] { return sweep(welchEvery, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<complex> 8192, 16 averages", [&] { return sweep(welch8192, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
        {"WelchPsd<float> 1024, 16 averages", [&] { return sweep(welchReal, std::span<const float>(realInput), std::span<gr::DataSet<float>>(records)); }},
        {"Spectrogram<complex> 1024", [&] { return sweep(spectrogram, std::span<const CF>(complexInput), std::span<gr::DataSet<float>>(records)); }},
    };

    std::println("ns per INPUT sample. A 1024-point transform at half overlap covers 512 input samples, so one record");
    std::println("per hop is one record per 512 samples and one record per 16 averages is one per 8192.");
    gr::blocks::testing::bench::report(std::span<Arm>(arms), kSamplesPerCall, kRepeats);
}
