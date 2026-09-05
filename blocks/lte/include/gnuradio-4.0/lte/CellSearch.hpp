#ifndef GNURADIO_LTE_CELL_SEARCH_HPP
#define GNURADIO_LTE_CELL_SEARCH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <iterator>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/MeasurementSlot.hpp>
#include <gnuradio-4.0/algorithm/lte/SyncSignals.hpp>

namespace gr::blocks::lte {

/// Which confirmed detections of a half-frame become records.
enum class CellReport : std::uint8_t { All = 0, Best };

namespace detail {

[[nodiscard]] inline CellReport parseReport(std::string_view name) {
    if (name == "all") {
        return CellReport::All;
    }
    if (name == "best") {
        return CellReport::Best;
    }
    throw gr::exception(std::format("CellSearch: 'report' must be 'all' or 'best', got '{}'", name));
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::lte::CellSearch)

/**
 * @brief Identifies E-UTRA cells from their two synchronization signals, and nothing else.
 *
 * Per examined half-frame the block finds the primary synchronization signal for each of the three roots, decodes
 * the secondary one beside each detection that clears its threshold, and publishes one record per confirmed
 * identity. What a record carries is exactly what the two signals say by themselves: the physical cell identity
 * `3*N_ID^(1) + N_ID^(2)`, the sample the 10 ms radio frame begins on, the carrier frequency offset, the duplex
 * mode, the cyclic-prefix type and which half-frame the detection came from. Nothing above the synchronization
 * signals is read — no broadcast channel, so no system bandwidth and no system frame number — and nothing is
 * carried between half-frames: every half-frame is decided from scratch, which is what leaves no state to go
 * wrong. A consumer that wants a smoothed answer averages the records.
 *
 * The stream must already be at 1.92 MS/s, the smallest rate the numerology allows, at which the central six
 * resource blocks that carry both signals are the whole band. A stream at any other rate is resampled ahead of
 * this block: an identifier that decimated internally would be carrying a channelizer.
 *
 * Every position is examined exactly once whatever the scheduler's chunking. A half-frame is examined together
 * with the 480 samples before it — the furthest back any of the four structures places the secondary symbol — and
 * the 127 after it, which is what a primary symbol beginning at the window's last position occupies, so a symbol
 * is never split across two windows and never evaluated twice. The end of the stream leaves fewer positions than a
 * half-frame and they are examined as one short window, so the positions evaluated are exactly the ones at which
 * the stream carries a whole primary symbol: every sample but the last 127. How the window is searched is not
 * observable and the metrics and the record are; `search_interval` is the lever for a consumer that cannot afford
 * every half-frame, and it costs exactly a `1/N` share of the work.
 *
 * The look-behind before a stream's first sample is silence rather than signal, so a detection in the first 480
 * samples whose secondary symbol would fall into it is refused: those samples do not exist, and a reading taken
 * from the pad would be an identity decided on nothing.
 *
 * One detection per root per half-frame. Two cells sharing a root are not separated: the stronger wins the single
 * peak and the weaker is not reported, which is the limit an identifier built on one correlation peak has.
 */
struct CellSearch : Block<CellSearch, NoTagPropagation> {
    using Description = Doc<R""(
@brief Reports E-UTRA physical cell identity, frame timing, frequency offset, duplex mode and cyclic-prefix type.

One record per confirmed detection per examined half-frame, from the primary and secondary synchronization signals
alone at 1.92 MS/s. No broadcast channel, no tracking and no cell table: every half-frame is decided from scratch.
)"">;

    PortIn<std::complex<float>> in;
    /// One record per confirmed detection, carrying the 62 soft secondary values the identity was decided on.
    PortOut<DataSet<float>, Async> out;

    Annotated<float, "sample_rate", Visible, Unit<"Hz">, Doc<"stream rate; must be exactly 1920000, resample upstream otherwise">>              sample_rate         = 0.f;
    Annotated<float, "frequency_search_hz", Visible, Unit<"Hz">, Doc<"half-width of the integer frequency search; 0 is the single hypothesis">> frequency_search_hz = 0.f;
    Annotated<float, "pss_threshold", Visible, Doc<"peak-over-mean correlation power below which no primary signal is declared">>               pss_threshold       = 20.f;
    Annotated<float, "sss_threshold", Visible, Doc<"secondary correlation quality below which a detection is dropped as unconfirmed">>          sss_threshold       = 0.5f;
    Annotated<std::string, "report", Visible, Doc<"'all' reports every confirmed root, 'best' only the strongest primary metric">>              report              = std::string("all");
    Annotated<gr::Size_t, "search_interval", Visible, Doc<"examine every Nth half-frame; the others pass unexamined and are counted">>          search_interval     = 1U;

    GR_MAKE_REFLECTABLE(CellSearch, in, out, sample_rate, frequency_search_hz, pss_threshold, sss_threshold, report, search_interval);

    /// The published account, in the order a reader outside C++ reads it.
    static constexpr std::size_t kCellIdAt     = 0UZ;
    static constexpr std::size_t kNId1At       = 1UZ;
    static constexpr std::size_t kNId2At       = 2UZ;
    static constexpr std::size_t kFrameStartAt = 3UZ;
    static constexpr std::size_t kFrequencyAt  = 4UZ;
    static constexpr std::size_t kPssMetricAt  = 5UZ;
    static constexpr std::size_t kSssMetricAt  = 6UZ;
    static constexpr std::size_t kDuplexAt     = 7UZ;
    static constexpr std::size_t kCyclicAt     = 8UZ;
    static constexpr std::size_t kSlotValues   = 9UZ;

    std::uint64_t nSamples             = 0ULL; ///< input samples read
    std::uint64_t nHalfFrames          = 0ULL; ///< whole half-frames completed, examined or not
    std::uint64_t nHalfFramesSkipped   = 0ULL; ///< of those, the ones `search_interval` passed over
    std::uint64_t nPositions           = 0ULL; ///< candidate primary positions evaluated, each exactly once
    std::uint64_t nPssDetections       = 0ULL; ///< primary detections that cleared `pss_threshold`
    std::uint64_t nSssRejected         = 0ULL; ///< of those, the ones no secondary reading confirmed
    std::uint64_t nSecondaryOffStream  = 0ULL; ///< confirmed detections whose secondary symbol lies before the stream's first sample
    std::uint64_t nPublished           = 0ULL; ///< records emitted
    std::uint64_t nRateRefused         = 0ULL; ///< input rate tags naming a rate this block cannot honor
    std::uint64_t nTagsDropped         = 0ULL; ///< other input tags, which a record does not carry
    std::uint64_t nFrameStartOffStream = 0ULL; ///< confirmed detections whose radio frame began before the stream
    std::uint64_t nSearchRebuilds      = 0ULL; ///< times the frequency hypotheses were rebuilt, which only a change of `frequency_search_hz` asks for

    /// The state one examined half-frame leaves behind. Public, as every block's is, so that the type stays an
    /// aggregate and the framework can initialize it from a settings map.
    gr::measurement::MeasurementSlot<kSlotValues> _slot{};
    std::array<double, kSlotValues>               _values{};

    CellReport            _report{CellReport::All};
    float                 _searchWidth{0.f};
    gr::lte::CellDetector _detector{0.f};

    std::vector<std::complex<float>> _buffer{};        ///< the window under examination, its first sample the context before the half-frame
    std::uint64_t                    _halfFrame{0ULL}; ///< index of the half-frame the window's own half covers
    std::uint64_t                    _sequence{0ULL};  ///< records published since start()
    std::vector<DataSet<float>>      _pending{};       ///< records made but not yet published, drained as the port offers room

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (!std::isfinite(frequency_search_hz) || frequency_search_hz < 0.f) {
            throw gr::exception(std::format("CellSearch: 'frequency_search_hz' must be zero or a positive finite width, got {}", frequency_search_hz.value));
        }
        if (frequency_search_hz > kMaxSearchHz) {
            throw gr::exception(std::format("CellSearch: 'frequency_search_hz' is bounded at {} Hz, got {}; a wider search is a receiver's, not an identifier's", kMaxSearchHz, frequency_search_hz.value));
        }
        if (!std::isfinite(pss_threshold) || pss_threshold < 0.f) {
            throw gr::exception(std::format("CellSearch: 'pss_threshold' must be a non-negative finite metric, got {}", pss_threshold.value));
        }
        if (!std::isfinite(sss_threshold) || sss_threshold < 0.f) {
            throw gr::exception(std::format("CellSearch: 'sss_threshold' must be a non-negative finite metric, got {}", sss_threshold.value));
        }
        if (search_interval < 1U) {
            throw gr::exception("CellSearch: 'search_interval' must be at least one");
        }
        _report = detail::parseReport(report);
        // A setting invalidates what it is an input to and nothing else. The frequency hypotheses are derived from
        // the search width, so a change of width rebuilds them; a threshold, a reporting rule and an examination
        // interval are read where they are used, so they leave the window, the counters and the record sequence
        // exactly where a mid-stream change found them.
        if (_searchWidth != frequency_search_hz.value) {
            _searchWidth = frequency_search_hz.value;
            _detector    = gr::lte::CellDetector(_searchWidth);
            ++nSearchRebuilds;
        }
    }

    void start() {
        if (sample_rate != gr::lte::kSampleRate) {
            throw gr::exception(std::format("CellSearch: 'sample_rate' must be exactly {} Hz, got {}; resample upstream", gr::lte::kSampleRate, sample_rate.value));
        }
        reset();
    }

    void stop() {
        std::string line;
        const auto  append = [&line](std::string_view label, std::uint64_t count) {
            if (count > 0ULL) {
                std::format_to(std::back_inserter(line), "{}{}: {}", line.empty() ? "" : ", ", label, count);
            }
        };
        append("samples", nSamples);
        append("half-frames", nHalfFrames);
        append("half-frames skipped", nHalfFramesSkipped);
        append("positions", nPositions);
        append("primary detections", nPssDetections);
        append("secondary rejected", nSssRejected);
        append("secondary off stream", nSecondaryOffStream);
        append("published", nPublished);
        append("rate tags refused", nRateRefused);
        append("tags dropped", nTagsDropped);
        append("frame starts off stream", nFrameStartOffStream);
        if (!line.empty()) {
            std::println(stderr, "gr::blocks::lte::CellSearch '{}': {}", this->name, line);
        }
    }

    /// @brief Forget every buffered sample, count and record. For the owning thread between stop() and start().
    void reset() {
        _buffer.assign(gr::lte::kMaxSecondaryLookBehind, std::complex<float>(0.f, 0.f));
        _halfFrame = 0ULL;
        _sequence  = 0ULL;
        _pending.clear();
        nSamples             = 0ULL;
        nHalfFrames          = 0ULL;
        nHalfFramesSkipped   = 0ULL;
        nPositions           = 0ULL;
        nPssDetections       = 0ULL;
        nSssRejected         = 0ULL;
        nSecondaryOffStream  = 0ULL;
        nPublished           = 0ULL;
        nRateRefused         = 0ULL;
        nTagsDropped         = 0ULL;
        nFrameStartOffStream = 0ULL;
        nSearchRebuilds      = 0ULL;
        _values.fill(0.);
        publishSlot();
    }

    /// @brief The last confirmed detection's physical cell identity. Callable from any thread.
    [[nodiscard]] std::uint32_t cellId() const noexcept { return static_cast<std::uint32_t>(_slot.read().first[kCellIdAt]); }
    /// @brief The last confirmed detection's cell-identity group. Callable from any thread.
    [[nodiscard]] std::uint32_t nId1() const noexcept { return static_cast<std::uint32_t>(_slot.read().first[kNId1At]); }
    /// @brief The last confirmed detection's root index. Callable from any thread.
    [[nodiscard]] std::uint32_t nId2() const noexcept { return static_cast<std::uint32_t>(_slot.read().first[kNId2At]); }
    /// @brief Sample index the last confirmed detection's radio frame begins on. Callable from any thread.
    [[nodiscard]] std::uint64_t frameStart() const noexcept { return static_cast<std::uint64_t>(_slot.read().first[kFrameStartAt]); }
    /// @brief Carrier offset of the last confirmed detection, in Hz. Callable from any thread.
    [[nodiscard]] double frequencyOffsetHz() const noexcept { return _slot.read().first[kFrequencyAt]; }
    /// @brief Primary correlation metric of the last confirmed detection. Callable from any thread.
    [[nodiscard]] double pssMetric() const noexcept { return _slot.read().first[kPssMetricAt]; }
    /// @brief Secondary correlation quality of the last confirmed detection. Callable from any thread.
    [[nodiscard]] double sssMetric() const noexcept { return _slot.read().first[kSssMetricAt]; }
    /// @brief Duplex mode of the last confirmed detection, 0 paired and 1 unpaired. Callable from any thread.
    [[nodiscard]] std::uint32_t duplex() const noexcept { return static_cast<std::uint32_t>(_slot.read().first[kDuplexAt]); }
    /// @brief Cyclic-prefix type of the last confirmed detection, 0 normal and 1 extended. Callable from any thread.
    [[nodiscard]] std::uint32_t cyclicPrefix() const noexcept { return static_cast<std::uint32_t>(_slot.read().first[kCyclicAt]); }
    /// @brief Records published since start(). Callable from any thread.
    [[nodiscard]] std::uint64_t nRecords() const noexcept { return _slot.read().second; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const std::complex<float>> input = std::span<const std::complex<float>>(inSpan);
        countTags(inSpan);
        absorb(input);
        outSpan.publish(drain(outSpan));
        std::ignore = inSpan.consume(input.size());
        return work::Status::OK;
    }

    /// @brief End of stream: take whatever the last call left, then examine the short window it leaves behind.
    ///
    /// The epilogue runs once per stream whatever the tail holds, so the positions a whole-window rule can never
    /// reach are examined here. They are as much the stream's as any other: the only thing the stream does not
    /// carry is a primary symbol beginning within 128 samples of its end.
    [[nodiscard]] work::Status processEpilogue(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const std::span<const std::complex<float>> input = std::span<const std::complex<float>>(inSpan);
        countTags(inSpan);
        absorb(input);
        examineTail();
        outSpan.publish(drain(outSpan));
        return work::Status::OK;
    }

private:
    /// The half-width the search is bounded at: beyond 55 hypotheses the cost is a receiver's search.
    static constexpr float kMaxSearchHz = 200'000.f;
    /// A half-frame with the context the furthest secondary hypothesis needs and the tail a symbol at the last
    /// position occupies, so every position is evaluated exactly once whatever the scheduler hands over.
    static constexpr std::size_t kWindowSamples = gr::lte::CellDetector::windowFor(gr::lte::kHalfFrameSamples);
    /// What a window carries beyond its own positions: the look-behind before them and the symbol after the last.
    static constexpr std::size_t kWindowContext = gr::lte::CellDetector::windowFor(0UZ);

    /// One confirmed identity, held until the half-frame's reporting rule has seen them all.
    struct Confirmed {
        gr::lte::PssDetection primary{};
        gr::lte::SssDecision  secondary{};
        std::uint64_t         position{0ULL};   ///< absolute index of the primary symbol's first useful sample
        std::uint64_t         frameStart{0ULL}; ///< absolute index of the radio frame's first sample
    };

    void countTags(const auto& inSpan) {
        for (const gr::Tag& tag : inSpan.rawTags) {
            bool other = false;
            for (const auto& [key, value] : tag.map) {
                if (std::string_view(key.data(), key.size()) == gr::tag::SAMPLE_RATE.shortKey()) {
                    const float* rate = value.template get_if<float>();
                    if (rate == nullptr || *rate != gr::lte::kSampleRate) {
                        ++nRateRefused; // the block cannot honor another rate and reads the stream as 1.92 MS/s regardless
                    }
                } else {
                    other = true;
                }
            }
            if (other) {
                ++nTagsDropped; // a record is not a sample and carries its own facts, so nothing propagates
            }
        }
    }

    /// Whether the half-frame the window's own half covers is one `search_interval` examines.
    [[nodiscard]] bool examines() const noexcept { return _halfFrame % static_cast<std::uint64_t>(search_interval.value) == 0ULL; }

    /// Take samples into the window and examine every whole window they complete.
    void absorb(std::span<const std::complex<float>> input) {
        _buffer.insert(_buffer.end(), input.begin(), input.end());
        nSamples += input.size();

        while (_buffer.size() >= kWindowSamples) {
            if (examines()) {
                examine(gr::lte::kHalfFrameSamples);
            } else {
                ++nHalfFramesSkipped;
            }
            ++nHalfFrames;
            ++_halfFrame;
            _buffer.erase(_buffer.begin(), _buffer.begin() + static_cast<std::ptrdiff_t>(gr::lte::kHalfFrameSamples));
        }
    }

    /// Examine what the whole windows left: fewer than a half-frame of positions, ending at the last one whose
    /// primary symbol the stream still carries whole.
    void examineTail() {
        if (_buffer.size() <= kWindowContext || !examines()) {
            return;
        }
        examine(_buffer.size() - kWindowContext);
    }

    /// Move as many finished records as the port offers room for.
    [[nodiscard]] std::size_t drain(auto& outSpan) {
        if (!outSpan.isConnected) {
            _pending.clear(); // an unconnected port is not a backlog
            return 0UZ;
        }
        std::size_t       made = 0UZ;
        const std::size_t take = std::min(_pending.size(), outSpan.size());
        for (; made < take; ++made) {
            outSpan[made] = std::move(_pending[made]);
        }
        _pending.erase(_pending.begin(), _pending.begin() + static_cast<std::ptrdiff_t>(made));
        return made;
    }

    void examine(std::size_t positions) {
        const std::span<const std::complex<float>> window(_buffer.data(), gr::lte::CellDetector::windowFor(positions));
        const std::uint64_t                        base = _halfFrame * static_cast<std::uint64_t>(gr::lte::kHalfFrameSamples);
        // Nothing precedes a stream's own first sample, so the look-behind of the first window is silence rather
        // than signal, and a structure whose secondary symbol would be read out of it is refused: the samples it
        // wants are not late, they do not exist.
        const std::ptrdiff_t earliest = _halfFrame == 0ULL ? static_cast<std::ptrdiff_t>(gr::lte::kMaxSecondaryLookBehind) : 0;

        gr::lte::ExamineCounts                        counts;
        const std::span<const gr::lte::CellDetection> found = _detector.examine(window, positions, pss_threshold, sss_threshold, counts);
        nPositions += positions;
        nPssDetections += counts.primaryFound;
        nSssRejected += counts.secondaryRejected;

        std::vector<Confirmed> confirmed;
        for (const gr::lte::CellDetection& detection : found) {
            const std::ptrdiff_t secondary = static_cast<std::ptrdiff_t>(gr::lte::kMaxSecondaryLookBehind + detection.primary.position) + detection.secondary.geometry.secondaryOffset();
            if (secondary < earliest) {
                ++nSecondaryOffStream;
                continue;
            }
            const std::int64_t absolute = static_cast<std::int64_t>(base) + static_cast<std::int64_t>(detection.primary.position);
            const std::int64_t frame    = absolute + detection.secondary.geometry.frameStartOffset(detection.secondary.halfFrame);
            if (frame < 0) {
                // The radio frame began before this stream did, so there is no sample index to name it by.
                ++nFrameStartOffStream;
                continue;
            }
            confirmed.push_back(Confirmed{detection.primary, detection.secondary, static_cast<std::uint64_t>(absolute), static_cast<std::uint64_t>(frame)});
        }

        if (confirmed.empty()) {
            return;
        }
        if (_report == CellReport::Best) {
            const auto      strongest = std::ranges::max_element(confirmed, {}, [](const Confirmed& entry) { return entry.primary.metric; });
            const Confirmed one       = *strongest;
            confirmed.assign(1UZ, one);
        }
        for (const Confirmed& entry : confirmed) {
            emit(entry);
        }
    }

    void emit(const Confirmed& entry) {
        const std::uint32_t nId1     = entry.secondary.nId1;
        const std::uint32_t nId2     = entry.primary.nId2;
        const std::uint32_t identity = gr::lte::cellIdentity(nId1, nId2);
        const bool          tdd      = entry.secondary.geometry.duplex == gr::lte::DuplexMode::Tdd;
        const bool          extended = entry.secondary.geometry.cyclicPrefix == gr::lte::CyclicPrefix::Extended;

        DataSet<float> record;
        record.signal_values.assign(entry.secondary.soft.begin(), entry.secondary.soft.end());
        record.extents.push_back(static_cast<std::int32_t>(entry.secondary.soft.size()));
        record.signal_names.emplace_back("sss");
        record.signal_quantities.emplace_back("");
        record.signal_units.emplace_back("");
        record.meta_information.emplace_back();
        record.timing_events.emplace_back();
        record.timestamp = 0;

        property_map& meta = record.meta_information[0UZ];
        meta.insert_or_assign(property_map::key_type("cell_id"), pmt::Value(static_cast<gr::Size_t>(identity)));
        meta.insert_or_assign(property_map::key_type("n_id_1"), pmt::Value(static_cast<gr::Size_t>(nId1)));
        meta.insert_or_assign(property_map::key_type("n_id_2"), pmt::Value(static_cast<gr::Size_t>(nId2)));
        meta.insert_or_assign(property_map::key_type("duplex"), pmt::Value(std::string(tdd ? "tdd" : "fdd")));
        meta.insert_or_assign(property_map::key_type("cyclic_prefix"), pmt::Value(std::string(extended ? "extended" : "normal")));
        meta.insert_or_assign(property_map::key_type("half_frame"), pmt::Value(static_cast<gr::Size_t>(entry.secondary.halfFrame)));
        meta.insert_or_assign(property_map::key_type("frame_start"), pmt::Value(entry.frameStart));
        meta.insert_or_assign(property_map::key_type("pss_position"), pmt::Value(entry.position));
        meta.insert_or_assign(property_map::key_type("frequency_offset_hz"), pmt::Value(entry.primary.frequencyHz));
        meta.insert_or_assign(property_map::key_type("pss_metric"), pmt::Value(entry.primary.metric));
        meta.insert_or_assign(property_map::key_type("sss_metric"), pmt::Value(entry.secondary.metric));
        meta.insert_or_assign(property_map::key_type(gr::tag::SAMPLE_RATE.shortKey()), pmt::Value(gr::lte::kSampleRate));
        meta.insert_or_assign(property_map::key_type("sequence"), pmt::Value(_sequence));

        _pending.push_back(std::move(record));
        ++_sequence;
        ++nPublished;

        _values[kCellIdAt]     = static_cast<double>(identity);
        _values[kNId1At]       = static_cast<double>(nId1);
        _values[kNId2At]       = static_cast<double>(nId2);
        _values[kFrameStartAt] = static_cast<double>(entry.frameStart);
        _values[kFrequencyAt]  = static_cast<double>(entry.primary.frequencyHz);
        _values[kPssMetricAt]  = static_cast<double>(entry.primary.metric);
        _values[kSssMetricAt]  = static_cast<double>(entry.secondary.metric);
        _values[kDuplexAt]     = tdd ? 1. : 0.;
        _values[kCyclicAt]     = extended ? 1. : 0.;
        publishSlot();
    }

    void publishSlot() noexcept { _slot.publish(_values, nPublished); }
};

} // namespace gr::blocks::lte

#endif // GNURADIO_LTE_CELL_SEARCH_HPP
