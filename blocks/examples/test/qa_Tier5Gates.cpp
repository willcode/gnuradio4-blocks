#include <boost/ut.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <memory>
#include <numbers>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/timing/FrequencySchedule.hpp>

#include <gnuradio-4.0/analog/Agc.hpp>
#include <gnuradio-4.0/ax25/Ax25.hpp>
#include <gnuradio-4.0/basic/ConverterBlocks.hpp>
#include <gnuradio-4.0/basic/SampleDelay.hpp>
#include <gnuradio-4.0/basic/StreamToDataSet.hpp>
#include <gnuradio-4.0/channel/Awgn.hpp>
#include <gnuradio-4.0/channel/DopplerShift.hpp>
#include <gnuradio-4.0/channel/RangeDelay.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/BitPacking.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
#include <gnuradio-4.0/digital/DelimiterFramer.hpp>
#include <gnuradio-4.0/digital/DifferentialCoding.hpp>
#include <gnuradio-4.0/digital/ManchesterCombine.hpp>
#include <gnuradio-4.0/digital/PamSymbols.hpp>
#include <gnuradio-4.0/digital/Scrambler.hpp>
#include <gnuradio-4.0/fec/ConvBlocks.hpp>
#include <gnuradio-4.0/fec/InterleaveBlocks.hpp>
#include <gnuradio-4.0/fec/RsBlocks.hpp>
#include <gnuradio-4.0/fileio/WavBlocks.hpp>
#include <gnuradio-4.0/filter/DcBlocker.hpp>
#include <gnuradio-4.0/filter/DesignedFilter.hpp>
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/recipes/AfskDemod.hpp>
#include <gnuradio-4.0/recipes/DbpskDemod.hpp>
#include <gnuradio-4.0/recipes/FskDemodAudio.hpp>
#include <gnuradio-4.0/recipes/KissFileRead.hpp>
#include <gnuradio-4.0/recipes/KissFileWrite.hpp>

/*
 * Acceptance as composed graphs rather than as a list of blocks: a packet-radio APRS link closed through the
 * KISS file sink and read back, three satellite receive graphs run against off-air audio, and one synthetic orbital
 * pass carrying coded frames from end to end.
 *
 * The APRS graph is the synthetic one. Its transmit side is built from the same landed blocks the receive side
 * inverts -- address encoding, the frame check sequence, HDLC framing, NRZI -- and the tone pair and the noise are the
 * only things the test writes itself. The oracle is closed: the hundred information fields that entered the encoder
 * are compared with the hundred the KISS file gives back.
 *
 * The three satellite graphs read the off-air collection, whose oracle is strong because nobody here wrote it: a frame
 * that passes an AX.25 frame check sequence and decodes to a callsign the satellite is known to transmit was made by a
 * spacecraft. The transmitter facts each graph is configured with -- modulation, rate, tones, deviation, framing --
 * come from the collection's own satellite descriptions and are read as a vendor document is read.
 */
namespace {

using gr::blocks::ax25::Ax25Decode;
using gr::blocks::ax25::Ax25Encode;
using gr::blocks::channel::AwgnChannel;
using gr::blocks::digital::CrcAppend;
using gr::blocks::digital::CrcCheck;
using gr::blocks::digital::DelimiterExtractor;
using gr::blocks::digital::DelimiterFramer;
using gr::blocks::digital::DifferentialDecoder;
using gr::blocks::digital::DifferentialEncoder;
using gr::blocks::digital::PamSlicer;
using gr::blocks::testing::ProcessFunction;
using gr::blocks::testing::TagSource;

using Record = gr::DataSet<std::uint8_t>;

// ─── the wire profiles the AX.25 blocks are configured with ──────────────────────────────────────────────────────

/// The HDLC profile of the delimiter blocks: the flag, bit stuffing, and each payload byte unpacked least significant
/// bit first. One map configures the framer and the extractor, which is what makes them a pair.
[[nodiscard]] gr::property_map hdlc() {
    return {{"end_delimiter", std::string("01111110")}, {"transparency", std::string("bit_stuffing")}, {"stuff_after_ones", gr::Size_t{5}}, {"abort_ones", gr::Size_t{7}}, //
        {"bits_per_item", gr::Size_t{1}}, {"max_payload_items", gr::Size_t{8192}}, {"payload_pack_bits", gr::Size_t{8}}, {"payload_bit_order", std::string("lsb_first")}};
}

/// CRC-16/IBM-SDLC, the AX.25 frame check sequence, with its two bytes least significant first on the wire.
[[nodiscard]] gr::property_map fcs() {
    return {{"width", gr::Size_t{16}}, {"poly", std::uint64_t{0x1021}}, {"initial_value", std::uint64_t{0xFFFF}}, {"final_xor", std::uint64_t{0xFFFF}}, //
        {"input_reflected", true}, {"result_reflected", true}, {"crc_byte_order", std::string("little")}};
}

/// The same six-tuple with the check sequence taken off the record that passes.
[[nodiscard]] gr::property_map fcsChecking() {
    gr::property_map settings = fcs();
    settings["discard_crc"]   = true;
    return settings;
}

// ─── test-local blocks: a record source, a wire flattener, a tap and two sinks ────────────────────────────────────

struct RecordSource : gr::Block<RecordSource> {
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordSource, out);
    std::vector<Record> _records{};
    std::size_t         _pos = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return _pos == _records.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// Flattens each framed record onto the wire item stream the bit-level blocks read. A byte stream has no record
/// boundary of its own, so this is where the framer's records become a link.
struct RecordToStream : gr::Block<RecordToStream> {
    gr::PortIn<Record, gr::Async>        in;
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordToStream, in, out);

    std::vector<std::uint8_t> _pending{};
    std::size_t               _sent = 0UZ;

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        std::size_t consumed = 0UZ;
        if (_sent == _pending.size() && inSpan.size() > 0UZ) {
            _pending = inSpan[0UZ].signal_values;
            _sent    = 0UZ;
            consumed = 1UZ;
        }
        const std::size_t n = std::min(_pending.size() - _sent, outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _pending[_sent + i];
        }
        _sent += n;
        std::ignore = inSpan.consume(consumed);
        outSpan.publish(n);
        return consumed == 0UZ && n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

/// Keeps a copy of every record that crosses it and passes it on, so one graph run can be read at two points.
struct RecordTap : gr::Block<RecordTap> {
    gr::PortIn<Record, gr::Async>  in;
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(RecordTap, in, out);
    std::vector<Record> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            _records.push_back(inSpan[i]);
            outSpan[i] = inSpan[i];
        }
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<Record, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<Record> _records{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const Record& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

struct ByteSink : gr::Block<ByteSink> {
    gr::PortIn<std::uint8_t, gr::Async> in;
    GR_MAKE_REFLECTABLE(ByteSink, in);
    std::vector<std::uint8_t> _items{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        _items.insert(_items.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// Keeps a copy of the stream that crosses it and passes it on, so one graph run can be read at two points.
struct ByteTap : gr::Block<ByteTap> {
    gr::PortIn<std::uint8_t, gr::Async>  in;
    gr::PortOut<std::uint8_t, gr::Async> out;
    GR_MAKE_REFLECTABLE(ByteTap, in, out);
    std::vector<std::uint8_t> _items{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            _items.push_back(inSpan[i]);
            outSpan[i] = inSpan[i];
        }
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

// ─── running a graph ──────────────────────────────────────────────────────────────────────────────────────────────

/// Runs @p flow to completion under the simple scheduler. @p collect runs while the scheduler still owns the graph,
/// because the references `emplaceBlock` returned point into blocks the scheduler destroys with itself.
template<typename TCollect>
void runGraph(gr::Graph flow, TCollect&& collect) {
    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value()) << boost::ut::fatal;
    const auto finished = scheduler.runAndWait();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);
    collect();
}

/// Connects a plain block's output to a composite's input, or the reverse, through the models the graph holds.
void connectToComposite(gr::Graph& flow, auto& block, const std::shared_ptr<gr::BlockModel>& composite) {
    const auto model = gr::graph::findBlock(flow, block);
    boost::ut::expect(model.has_value()) << boost::ut::fatal;
    boost::ut::expect(flow.connect(*model, gr::PortDefinition{"out"}, composite, gr::PortDefinition{"in"}).has_value()) << boost::ut::fatal;
}

void connectFromComposite(gr::Graph& flow, const std::shared_ptr<gr::BlockModel>& composite, auto& block) {
    const auto model = gr::graph::findBlock(flow, block);
    boost::ut::expect(model.has_value()) << boost::ut::fatal;
    boost::ut::expect(flow.connect(composite, gr::PortDefinition{"out"}, *model, gr::PortDefinition{"in"}).has_value()) << boost::ut::fatal;
}

// ─── G-A: the APRS packet-radio graph ─────────────────────────────────────────────────────────────────────────────

constexpr double      kAudioRate    = 48000.;
constexpr double      kAprsBaud     = 1200.;
constexpr double      kMarkHz       = 1200.; ///< Bell 202's tone for a one, the LOWER of the pair
constexpr double      kSpaceHz      = 2200.;
constexpr std::size_t kAprsFrames   = 100UZ;
constexpr std::size_t kKeyUpSymbols = 64UZ; ///< the transmitter's key-up alternation, and the same again at the tail

/// A deterministic APRS information field: a fixed position report and a sequence number, so a failure is the chain's
/// and never the draw's.
[[nodiscard]] std::vector<std::uint8_t> aprsInfo(std::size_t index) {
    const std::string text = std::format("!4903.50N/07201.75W-Tier5 {:03}", index);
    return {text.begin(), text.end()};
}

/// The addressing every APRS frame here carries, and the settings the decoded keys are read against.
[[nodiscard]] gr::property_map aprsAddressing() {
    return {{"destination", std::string("APRS")}, {"source", std::string("N0CALL-9")}, {"via", std::string("WIDE1-1")}, //
        {"control_type", std::string("UI")}, {"pid", gr::Size_t{0xF0}}, {"command", true}, {"poll_final", false}};
}

[[nodiscard]] Record recordOf(std::vector<std::uint8_t> bytes) {
    Record record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("payload");
    record.meta_information.emplace_back();
    record.timing_events.emplace_back();
    return record;
}

[[nodiscard]] const gr::property_map& metaOf(const Record& record) {
    static const gr::property_map kEmpty{};
    return record.meta_information.empty() ? kEmpty : record.meta_information.front();
}

[[nodiscard]] std::string metaString(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry == map.end() ? std::string{} : entry->second.value_or(std::string{});
}

[[nodiscard]] bool metaBool(const Record& record, std::string_view key) {
    const auto& map   = metaOf(record);
    const auto  entry = map.find(gr::property_map::key_type(key));
    return entry != map.end() && entry->second.value_or(false);
}

/// The transmit half: information fields to the NRZI-coded wire bits an AFSK modulator keys.
[[nodiscard]] std::vector<std::uint8_t> aprsWireBits(const std::vector<Record>& payloads) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<RecordSource>();
    source._records  = payloads;
    auto& encode     = flow.emplaceBlock<Ax25Encode>(aprsAddressing());
    auto& append     = flow.emplaceBlock<CrcAppend>(fcs());
    auto& framer     = flow.emplaceBlock<DelimiterFramer>(hdlc());
    auto& wire       = flow.emplaceBlock<RecordToStream>();
    // NRZI is the outermost coding on an AX.25 link: it is applied after the flags and the stuffed bits, and the
    // receiver undoes it before hunting for a flag.
    auto& nrzi = flow.emplaceBlock<DifferentialEncoder<std::uint8_t>>({{"modulus", gr::Size_t{2}}, {"coding", std::string("nrzi")}});
    auto& bits = flow.emplaceBlock<ByteSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, encode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(encode, append).has_value());
    boost::ut::expect(flow.connect<"out", "in">(append, framer).has_value());
    boost::ut::expect(flow.connect<"out", "in">(framer, wire).has_value());
    boost::ut::expect(flow.connect<"out", "in">(wire, nrzi).has_value());
    boost::ut::expect(flow.connect<"out", "in">(nrzi, bits).has_value());

    std::vector<std::uint8_t> out;
    runGraph(std::move(flow), [&out, &bits] { out = bits._items; });
    return out;
}

/// Phase-continuous two-tone audio: the wave a soundcard hands an AFSK receiver. The key-up alternation either side of
/// the data is the transmitter keying its carrier up and down -- it gives the timing loop transitions to acquire on
/// before the first flag arrives, and keeps the chain's filters fed past the last one.
[[nodiscard]] std::vector<float> afskAudio(std::span<const std::uint8_t> bits, std::size_t keyUpSymbols) {
    const auto         perSymbol = static_cast<std::size_t>(std::llround(kAudioRate / kAprsBaud));
    std::vector<float> audio;
    audio.reserve((bits.size() + 2UZ * keyUpSymbols) * perSymbol);

    double     phase = 0.;
    const auto emit  = [&audio, &phase, perSymbol](bool one) {
        const double tone = one ? kMarkHz : kSpaceHz;
        for (std::size_t n = 0UZ; n < perSymbol; ++n) {
            audio.push_back(static_cast<float>(std::cos(phase)));
            phase += 2. * std::numbers::pi * tone / kAudioRate;
            if (phase > 2. * std::numbers::pi) {
                phase -= 2. * std::numbers::pi;
            }
        }
    };

    for (std::size_t k = 0UZ; k < keyUpSymbols; ++k) {
        emit(k % 2UZ == 0UZ);
    }
    for (const std::uint8_t bit : bits) {
        emit(bit != 0U);
    }
    for (std::size_t k = 0UZ; k < keyUpSymbols; ++k) {
        emit(k % 2UZ == 0UZ);
    }
    return audio;
}

/**
 * @brief The noise power that puts a unit-amplitude tone stream at @p esN0_db.
 *
 * A cosine of unit amplitude carries mean power 1/2 per sample, so one symbol's energy is `perSymbol / 2`. Real
 * additive noise of variance `sigma^2` per sample has one-sided density `N0 = 2 sigma^2` across the Nyquist band, so
 * `Es/N0 = perSymbol / (4 sigma^2)` and the wanted variance is `perSymbol / (4 * 10^(esN0_db/10))`. At 40 samples a
 * symbol that is 0.1 at 20 dB and 1.585 at 8 dB.
 */
[[nodiscard]] double toneNoisePower(double esN0_db) {
    const double perSymbol = kAudioRate / kAprsBaud;
    return perSymbol / (4. * std::pow(10., esN0_db / 10.));
}

/// What one run of the APRS receive graph produced.
struct AprsRun {
    std::vector<Record> decoded{};  ///< what left Ax25Decode
    std::vector<Record> readBack{}; ///< what the KISS file gave back through KissFileRead
    std::size_t         softSymbols = 0UZ;
};

/// The receive half at @p esN0_db: audio, noise, AfskDemod, the slicer, NRZI, HDLC, the check sequence, the address
/// decode, and the KISS file sink. The file is then read back through the reader recipe, which closes the oracle.
[[nodiscard]] AprsRun aprsReceive(const std::vector<float>& audio, double esN0_db, const std::string& kissPath) {
    AprsRun run;
    {
        gr::Graph         flow;
        gr::Tensor<float> values(audio.begin(), audio.end());
        auto&             source = flow.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(audio.size())}, {"values", values}, {"mark_tag", false}});
        auto&             noise  = flow.emplaceBlock<AwgnChannel<float>>({{"noise_power", toneNoisePower(esN0_db)}, {"seed", std::uint64_t{20260902ULL}}});
        const auto        demod  = gr::recipes::AfskDemod::emplace(flow, {static_cast<float>(kAudioRate), static_cast<float>(kAprsBaud), kMarkHz, kSpaceHz});
        boost::ut::expect(demod != nullptr) << boost::ut::fatal;
        auto&      slicer    = flow.emplaceBlock<PamSlicer<float>>({{"n_levels", gr::Size_t{2}}});
        auto&      nrzi      = flow.emplaceBlock<DifferentialDecoder<std::uint8_t>>({{"modulus", gr::Size_t{2}}, {"coding", std::string("nrzi")}});
        auto&      extractor = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlc());
        auto&      check     = flow.emplaceBlock<CrcCheck>(fcsChecking());
        auto&      decode    = flow.emplaceBlock<Ax25Decode>();
        auto&      tap       = flow.emplaceBlock<RecordTap>();
        const auto sink      = gr::recipes::KissFileWrite::emplace(flow, {kissPath});
        boost::ut::expect(sink != nullptr) << boost::ut::fatal;

        boost::ut::expect(flow.connect<"out", "in">(source, noise).has_value());
        connectToComposite(flow, noise, demod);
        connectFromComposite(flow, demod, slicer);
        boost::ut::expect(flow.connect<"out", "in">(slicer, nrzi).has_value());
        boost::ut::expect(flow.connect<"out", "in">(nrzi, extractor).has_value());
        boost::ut::expect(flow.connect<"out", "in">(extractor, check).has_value());
        boost::ut::expect(flow.connect<"ok", "in">(check, decode).has_value());
        boost::ut::expect(flow.connect<"out", "in">(decode, tap).has_value());
        connectToComposite(flow, tap, sink);

        runGraph(std::move(flow), [&run, &tap] { run.decoded = tap._records; });
    }

    {
        gr::Graph  flow;
        const auto reader = gr::recipes::KissFileRead::emplace(flow, {kissPath, std::uint32_t{8192}});
        boost::ut::expect(reader != nullptr) << boost::ut::fatal;
        auto& sink = flow.emplaceBlock<RecordSink>();
        connectFromComposite(flow, reader, sink);
        runGraph(std::move(flow), [&run, &sink] { run.readBack = sink._records; });
    }
    std::filesystem::remove(kissPath);
    return run;
}

// ─── the off-air captures, and the skip when the box has none ─────────────────────────────────────────────────────

/// The captures are machine-local, so the build hands their directory to the test through its environment rather than
/// through a graph file.
[[nodiscard]] std::filesystem::path recordingsRoot() {
    const char* dir = std::getenv("GR4_SATELLITE_RECORDINGS_DIR");
    return (dir == nullptr || *dir == '\0') ? std::filesystem::path{} : std::filesystem::path(dir);
}

/// A leg wanted a capture the box does not have. A skip is not a pass, so the process reports one to ctest.
bool g_recordingsAbsent = false;

[[nodiscard]] std::optional<std::filesystem::path> recording(std::string_view name) {
    const std::filesystem::path root = recordingsRoot();
    if (root.empty() || !std::filesystem::is_directory(root)) {
        g_recordingsAbsent = true;
        std::println("[skip] GR4_SATELLITE_RECORDINGS_DIR names no directory; this leg wanted {}", name);
        return std::nullopt;
    }
    std::filesystem::path path = root / name;
    if (!std::filesystem::is_regular_file(path)) {
        g_recordingsAbsent = true;
        std::println("[skip] this leg wanted {}", path.string());
        return std::nullopt;
    }
    return path;
}

/// The timing loop's closed-loop noise bandwidths, normalized to the symbol rate, that the AFSK leg tries. A loop
/// acquires in about the reciprocal of this figure in symbols, so 0.002 is a five-hundred-symbol acquisition and only
/// serves a downlink that stays keyed; a packet burst opens with a preamble of tens of bits and wants a wider loop.
constexpr std::array<double, 3UZ> kAfskLoopBandwidths{{0.002, 0.01, 0.05}};

/// One configuration of the post-detection chain the 9600-baud leg reads a capture under.
struct FskConfig {
    double           lowpassBandwidth; ///< the post-detection cutoff, as a multiple of the symbol rate
    double           noiseBandwidth;   ///< the timing loop's closed-loop noise bandwidth, normalized to the symbol rate
    std::string_view detector;         ///< the timing error detector

    [[nodiscard]] std::string label() const { return std::format("({:.2f}, {:.4f}, {})", lowpassBandwidth, noiseBandwidth, detector); }
};

/**
 * @brief The configurations the 9600-baud leg sweeps, the recipe's own defaults first.
 *
 * A Gardner or Mueller-and-Muller detector rides the spectral component at the symbol rate, whose amplitude is the
 * overlap of the received pulse's spectrum with itself shifted by `Rs` -- so it comes entirely from whatever energy
 * survives ABOVE `Rs/2`. The recipe's cutoff sits at exactly `Rs/2` with a transition of `Rs/4`, which leaves the loop
 * a narrow slice of transition band to steer by: for that 65-tap design the overlap integral normalized by the
 * filter's energy is 0.000 at a cutoff of 0.40 `Rs`, 0.032 at 0.50 and 0.347 at 0.75. Widening the post-detection
 * filter therefore costs noise and buys timing line, and which trade a capture wants is the capture's; the two are
 * swept rather than defaulted. The loop bandwidths run down to 0.0001 for the same reason -- a loop with almost no
 * line to steer by does better the less it tries to steer.
 */
constexpr std::array<FskConfig, 12UZ> kG3ruhConfigs{{
    {0.5, 0.002, "mueller_muller"},
    {0.5, 0.002, "gardner"},
    {0.5, 0.0005, "mueller_muller"},
    {0.5, 0.0005, "gardner"},
    {0.5, 0.0001, "mueller_muller"},
    {0.5, 0.0001, "gardner"},
    {0.75, 0.002, "mueller_muller"},
    {0.75, 0.002, "gardner"},
    {0.75, 0.0005, "mueller_muller"},
    {0.75, 0.0005, "gardner"},
    {0.75, 0.0001, "mueller_muller"},
    {0.75, 0.0001, "gardner"},
}};

/// How many captures of each sweep this chain decodes a frame from. The collection's README says every file holds a
/// decodable packet, so the shortfall is the chain's and each one is named where it is recorded; these are the
/// measured floors, and a chain that reaches fewer than one of them has lost captures it can read.
constexpr std::size_t kAfskServed  = 2UZ;
constexpr std::size_t kG3ruhServed = 9UZ;

/// Captures the conditioning is worth: the DC blocker and the AGC ahead of the recipe decode this many more of the
/// 9600-baud sweep than the same configurations decode without them.
constexpr std::size_t kConditioningWorth = 2UZ;

/// The 65-item sync window the AO-40 correlation must find on the Manchester capture. It is a distance and not a
/// count, so it is pinned exactly: the transcribed vector is either on the air and matched or it is not.
constexpr std::size_t kManchesterSyncErrors = 0UZ;

/// Frames -- not recordings -- the AO-40 leg brings out of Reed-Solomon with every codeword inside the code's
/// correcting power, over the two captures the satellite descriptions name.
constexpr std::size_t kAo40Frames = 0UZ;

/**
 * @brief The settings a capture is read under.
 *
 * The collection is 48 kHz throughout and the stated `sample_rate` records that rather than configuring anything:
 * `WavSource::sample_rate` is annotated read-only and carries the decoded file's own rate from the moment the file is
 * open, so what a caller states there is overwritten before the graph runs and the header is what reaches it.
 */
[[nodiscard]] gr::property_map wavSource(const std::filesystem::path& file) { return {{"uri", file.string()}, {"sample_rate", static_cast<float>(kAudioRate)}}; }

/// One recording's row of the sweep: what the chain got back from it.
struct AirRun {
    std::uint64_t       symbols = 0ULL; ///< symbols the demodulator sliced
    std::size_t         crcOk   = 0UZ;
    std::size_t         crcFail = 0UZ;
    std::vector<Record> decoded{};

    [[nodiscard]] std::string addresses() const {
        std::string line;
        for (const Record& record : decoded) {
            std::format_to(std::back_inserter(line), "{}{}>{}", line.empty() ? "" : ", ", metaString(record, "ax25_source"), metaString(record, "ax25_destination"));
        }
        return line.empty() ? std::string("no callsign") : line;
    }
};

/// A capture the sweep covers, named as the satyaml names its transmitter.
struct Capture {
    std::string_view file;
    std::string_view satellite;
    double           symbolRate;
};

/// Counts the items that cross it, so a leg can say how much the demodulator produced before the framing read it.
/// Counts what crosses it and keeps the first `_keep` items, so a leg can both say how much a stage produced and read
/// a window of it back.
template<typename T>
struct ItemCounter : gr::Block<ItemCounter<T>> {
    gr::PortIn<T, gr::Async>  in;
    gr::PortOut<T, gr::Async> out;
    GR_MAKE_REFLECTABLE(ItemCounter, in, out);
    std::uint64_t  _count = 0ULL;
    std::size_t    _keep  = 0UZ;
    std::vector<T> _kept{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = inSpan[i];
            if (_kept.size() < _keep) {
                _kept.push_back(inSpan[i]);
            }
        }
        _count += n;
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

// ─── G-B: AFSK over AX.25 from real audio ─────────────────────────────────────────────────────────────────────────

/// The AFSK/AX.25 captures: every recording whose satyaml names an AFSK transmitter with AX.25 framing, including
/// both captures of Swiatowid, whose satellite has one. All of them carry the same tone pair -- an audio carrier of
/// 1700 Hz with a deviation of 500 Hz, which is mark 1200 and space 2200 -- and the symbol rate is the transmitter's
/// own, AO-27's being 1240 rather than 1200. `tanusha3_pm.wav` is not among them: its squaring spectrum puts a line
/// 125 times the band's median at 1200.4 Hz and 2.1 and 1.9 per cent of its power in the two Bell 202 tone bands,
/// which is the noise floor, so it is a detected 1200-baud stream and not an AFSK one.
constexpr std::array<Capture, 4UZ> kAfskCaptures{{
    {"ao27.wav", "AO-27", 1240.},
    {"cape3.wav", "CAPE-3", 1200.},
    {"swiatowid-ax25.wav", "Swiatowid", 1200.},
    {"swiatowid.wav", "Swiatowid", 1200.},
}};

/// `WavSource`, `AfskDemod`, the slicer, NRZI, HDLC extraction, the frame check sequence and the address decode.
[[nodiscard]] AirRun afskAx25(const std::filesystem::path& file, double symbolRate, double loopBandwidth) {
    AirRun    run;
    gr::Graph flow;

    auto& source = flow.emplaceBlock<gr::blocks::fileio::WavSource<float>>(wavSource(file));
    // Carson's rule for this tone pair at this rate: the modulation index is |mark - space| / symbol_rate and half the
    // occupied bandwidth is (|h|/2 + 0.5) symbol rates, which at 1240 Bd is 0.903 rather than Bell 202's 0.9167.
    const double index      = std::abs(kMarkHz - kSpaceHz) / symbolRate;
    const auto   decimation = static_cast<std::uint32_t>(std::llround(kAudioRate / (8. * symbolRate)));

    gr::recipes::AfskDemod::Parameters parameters{static_cast<float>(kAudioRate), static_cast<float>(symbolRate), kMarkHz, kSpaceHz};
    parameters.decimation        = decimation;
    parameters.channel_bandwidth = 0.5 * index + 0.5;
    parameters.noise_bandwidth   = loopBandwidth;
    const auto demod             = gr::recipes::AfskDemod::emplace(flow, parameters);
    boost::ut::expect(demod != nullptr) << boost::ut::fatal;

    auto& slicer    = flow.emplaceBlock<PamSlicer<float>>({{"n_levels", gr::Size_t{2}}});
    auto& counter   = flow.emplaceBlock<ItemCounter<std::uint8_t>>();
    auto& nrzi      = flow.emplaceBlock<DifferentialDecoder<std::uint8_t>>({{"modulus", gr::Size_t{2}}, {"coding", std::string("nrzi")}});
    auto& extractor = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlc());
    auto& check     = flow.emplaceBlock<CrcCheck>(fcsChecking());
    auto& decode    = flow.emplaceBlock<Ax25Decode>();
    auto& passed    = flow.emplaceBlock<RecordSink>();
    auto& failed    = flow.emplaceBlock<RecordSink>();

    connectToComposite(flow, source, demod);
    connectFromComposite(flow, demod, slicer);
    boost::ut::expect(flow.connect<"out", "in">(slicer, counter).has_value());
    boost::ut::expect(flow.connect<"out", "in">(counter, nrzi).has_value());
    boost::ut::expect(flow.connect<"out", "in">(nrzi, extractor).has_value());
    boost::ut::expect(flow.connect<"out", "in">(extractor, check).has_value());
    boost::ut::expect(flow.connect<"ok", "in">(check, decode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(decode, passed).has_value());
    boost::ut::expect(flow.connect<"fail", "in">(check, failed).has_value());

    runGraph(std::move(flow), [&run, &counter, &passed, &failed] {
        run.symbols = counter._count;
        run.decoded = passed._records;
        run.crcOk   = passed._records.size();
        run.crcFail = failed._records.size();
    });
    return run;
}

// ─── G-C: 9600-baud G3RUH over AX.25 from real audio ──────────────────────────────────────────────────────────────

/// The FSK 9600 G3RUH captures: every recording whose satyaml names an FSK transmitter at 9600 baud with AX.25 G3RUH
/// framing, all three Astrocast captures among them. A capture whose satyaml calls the same framing BPSK, or names
/// G3RUH at another rate, is a different chain and is not swept here; nor is `tanusha3_pm.wav`, whose squaring
/// spectrum puts its symbol-rate line at 1200.4 Hz where every capture this sweep decodes puts one at 9600.0.
constexpr std::array<Capture, 12UZ> kG3ruhCaptures{{
    {"aalto1.wav", "AALTO-1", 9600.},
    {"astrocast.wav", "Astrocast 0.2", 9600.},
    {"astrocast_9k6.wav", "Astrocast 0.2", 9600.},
    {"astrocast_old.wav", "Astrocast 0.2", 9600.},
    {"az02.wav", "NSIGHT-1", 9600.},
    {"irazu.wav", "IRAZU", 9600.},
    {"koyo.wav", "KOYO", 9600.},
    {"se01.wav", "QBEE", 9600.},
    {"tigrisat.wav", "TIGRISAT", 9600.},
    {"ubakusat.wav", "UBAKUSAT", 9600.},
    {"us01.wav", "US01", 9600.},
    {"us04.wav", "COLUMBIA", 9600.},
}};

/// The G3RUH generator as a delay list: the multiplicative scrambler `1 + x^12 + x^17`. The published profile of that
/// name carries eight bits an item and this stream carries one, so the two taps are spelled out instead.
[[nodiscard]] gr::property_map g3ruh() { return {{"taps", std::string("12,17")}, {"bits_per_item", gr::Size_t{1}}}; }

/**
 * @brief `WavSource`, the post-detection chain, the G3RUH descrambler, NRZI, HDLC and the address decode.
 *
 * @param descrambleFirst false puts NRZI decoding ahead of the descrambler. Both orders recover the same stream, and
 *        the algebra says why: over GF(2) the descrambler is `d[n] = r[n] + r[n-12] + r[n-17]` and NRZI decoding is
 *        `b[n] = 1 + r[n] + r[n-1]`, so composing them either way gives
 *        `1 + r[n] + r[n-1] + r[n-12] + r[n-13] + r[n-17] + r[n-18]` -- the three complements of the second order sum
 *        to one complement, which is the one the first order already carries. The order is therefore a matter of which
 *        stage owns the epoch, not of whether the link decodes, and this arm measures that rather than assuming it.
 * @param condition       true puts a DC blocker and an AGC ahead of the demodulator. A recording arrives at whatever
 *        level the receiver that made it wrote, and a residual carrier offset at that receiver is a constant added to
 *        every sample; the post-detection recipe has neither an AGC nor a DC blocker, so a graph reading real audio
 *        supplies them.
 */
[[nodiscard]] AirRun fskG3ruhAx25(const std::filesystem::path& file, double symbolRate, const FskConfig& config, bool descrambleFirst, bool condition) {
    AirRun    run;
    gr::Graph flow;

    auto&                                  source = flow.emplaceBlock<gr::blocks::fileio::WavSource<float>>(wavSource(file));
    gr::recipes::FskDemodAudio::Parameters parameters{static_cast<float>(kAudioRate), static_cast<float>(symbolRate)};
    parameters.lowpass_bandwidth = config.lowpassBandwidth;
    parameters.noise_bandwidth   = config.noiseBandwidth;
    parameters.detector          = std::string(config.detector);
    const auto demod             = gr::recipes::FskDemodAudio::emplace(flow, parameters);
    boost::ut::expect(demod != nullptr) << boost::ut::fatal;

    auto& counter     = flow.emplaceBlock<ItemCounter<std::uint8_t>>();
    auto& descrambler = flow.emplaceBlock<gr::blocks::digital::MultiplicativeDescrambler>(g3ruh());
    auto& nrzi        = flow.emplaceBlock<DifferentialDecoder<std::uint8_t>>({{"modulus", gr::Size_t{2}}, {"coding", std::string("nrzi")}});
    auto& extractor   = flow.emplaceBlock<DelimiterExtractor<std::uint8_t>>(hdlc());
    auto& check       = flow.emplaceBlock<CrcCheck>(fcsChecking());
    auto& decode      = flow.emplaceBlock<Ax25Decode>();
    auto& passed      = flow.emplaceBlock<RecordSink>();
    auto& failed      = flow.emplaceBlock<RecordSink>();

    if (condition) {
        auto& dc  = flow.emplaceBlock<gr::blocks::filter::DcBlocker<float>>({{"length", gr::Size_t{128}}});
        auto& agc = flow.emplaceBlock<gr::blocks::analog::Agc<float>>({{"sample_rate", static_cast<float>(kAudioRate)}, {"reference_db", 0.0}, {"attack_s", 0.005}, {"decay_s", 0.05}});
        boost::ut::expect(flow.connect<"out", "in">(source, dc).has_value());
        boost::ut::expect(flow.connect<"out", "in">(dc, agc).has_value());
        connectToComposite(flow, agc, demod);
    } else {
        connectToComposite(flow, source, demod);
    }
    connectFromComposite(flow, demod, counter);
    if (descrambleFirst) {
        boost::ut::expect(flow.connect<"out", "in">(counter, descrambler).has_value());
        boost::ut::expect(flow.connect<"out", "in">(descrambler, nrzi).has_value());
        boost::ut::expect(flow.connect<"out", "in">(nrzi, extractor).has_value());
    } else {
        boost::ut::expect(flow.connect<"out", "in">(counter, nrzi).has_value());
        boost::ut::expect(flow.connect<"out", "in">(nrzi, descrambler).has_value());
        boost::ut::expect(flow.connect<"out", "in">(descrambler, extractor).has_value());
    }
    boost::ut::expect(flow.connect<"out", "in">(extractor, check).has_value());
    boost::ut::expect(flow.connect<"ok", "in">(check, decode).has_value());
    boost::ut::expect(flow.connect<"out", "in">(decode, passed).has_value());
    boost::ut::expect(flow.connect<"fail", "in">(check, failed).has_value());

    runGraph(std::move(flow), [&run, &counter, &passed, &failed] {
        run.symbols = counter._count;
        run.decoded = passed._records;
        run.crcOk   = passed._records.size();
        run.crcFail = failed._records.size();
    });
    return run;
}
// ─── G-D: DBPSK with the AO-40 FEC framing behind it ──────────────────────────────────────────────────────────────

/// The AO-40-FEC captures. AO-73 is the only recording whose satyaml names DBPSK with AO-40 FEC framing; the two other
/// AO-40-framed captures are the 400-baud Manchester beacon of the uncoded form and are a different chain.
constexpr std::array<Capture, 1UZ> kAo40Captures{{
    {"ao73.wav", "AO-73", 1200.},
}};

/// QO-100's beacon carries the same coding at the rate it was designed for: 400 bit/s Manchester-coded, so the
/// detector runs at the 800 Hz chip rate and the chips are combined into one soft symbol a bit.
constexpr std::array<Capture, 1UZ> kAo40ManchesterCaptures{{
    {"qo100.wav", "QO-100", 400.},
}};

struct FloatSink : gr::Block<FloatSink> {
    gr::PortIn<float, gr::Async> in;
    GR_MAKE_REFLECTABLE(FloatSink, in);
    std::vector<float> _items{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        _items.insert(_items.end(), inSpan.begin(), inSpan.end());
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// The capture's samples, read through the same source block every leg reads it through.
[[nodiscard]] std::vector<float> readWav(const std::filesystem::path& file) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<gr::blocks::fileio::WavSource<float>>(wavSource(file));
    auto&     sink   = flow.emplaceBlock<FloatSink>();
    boost::ut::expect(flow.connect<"out", "in">(source, sink).has_value());
    std::vector<float> audio;
    runGraph(std::move(flow), [&audio, &sink] { audio = sink._items; });
    return audio;
}

/**
 * @brief Where a suppressed-carrier BPSK signal sits in an audio recording.
 *
 * A real BPSK stream `d(t) cos(2 pi fc t)` with `d` in `{+1,-1}` squares to `(1 + cos(4 pi fc t)) / 2`: the modulation
 * disappears and a line stands at twice the carrier. The scan is over the squared stream with its mean removed, coarse
 * at first and then refined, and half the line it finds is the carrier. AO-73's satyaml states no audio carrier — the
 * carrier is where the receiver that made the recording put it — so it is measured rather than assumed.
 */
[[nodiscard]] double squaringCarrier(std::span<const float> audio, double low, double high) {
    const std::size_t   count = std::min(audio.size(), static_cast<std::size_t>(4. * kAudioRate));
    std::vector<double> squared(count);
    double              mean = 0.;
    for (std::size_t i = 0UZ; i < count; ++i) {
        squared[i] = static_cast<double>(audio[i]) * static_cast<double>(audio[i]);
        mean += squared[i];
    }
    mean /= static_cast<double>(count);
    for (double& value : squared) {
        value -= mean;
    }

    const auto power = [&squared](double frequency) {
        double real = 0.;
        double imag = 0.;
        for (std::size_t i = 0UZ; i < squared.size(); ++i) {
            const double angle = 2. * std::numbers::pi * frequency * static_cast<double>(i) / kAudioRate;
            real += squared[i] * std::cos(angle);
            imag -= squared[i] * std::sin(angle);
        }
        return real * real + imag * imag;
    };

    double best      = 2. * low;
    double bestPower = -1.;
    for (double f = 2. * low; f <= 2. * high; f += 4.) {
        const double p = power(f);
        if (p > bestPower) {
            bestPower = p;
            best      = f;
        }
    }
    for (double f = best - 4.; f <= best + 4.; f += 0.25) {
        const double p = power(f);
        if (p > bestPower) {
            bestPower = p;
            best      = f;
        }
    }
    return 0.5 * best;
}

/**
 * @brief Closed-loop noise bandwidth of the front end's frequency-locked loop, normalized per sample.
 *
 * The one setting of the front end's this leg states for itself, because it is the one no default can carry. The
 * loop's discriminant is two filters sitting on the band edges of a root-raised-cosine spectrum, and neither capture
 * here has one: a Manchester link's spectrum is null at the carrier and stands at the edges, so the discriminant
 * reads the modulation as well as the offset, and a loop wide enough to act on that reading writes it onto the
 * carrier phase -- which a differential detector, measuring the phase moved between two adjacent items, reads as
 * amplitude error on every item. What makes the setting the leg's rather than the recipe's is the other side of it:
 * the recipe's own default is what acquires an offset a receiver has not measured, which is the job a band-edge loop
 * exists for, and these two captures need none of it -- the carrier is measured before the graph is built, by the
 * squaring line, to a quarter of a hertz, so what is left for the loop is drift.
 */
constexpr double kFllNoiseBandwidth = 0.0001;

/// What a Manchester capture's chip stream is turned into behind the detector.
enum class ChipCombine {
    none,   ///< the stream is one soft symbol a bit already, which is every un-Manchester capture
    twoTap, ///< half the difference of an adjacent pair, the textbook soft combination -- the negative control
    parity  ///< ManchesterCombine, which selects the parity that carries the differentially detected bit
};

/// `WavSource`, the Hilbert branch and its matching delay, and `DbpskDemod` tuned to the measured carrier: the front
/// end G-D's chain begins with, up to the soft symbols the coded framing behind it reads.
[[nodiscard]] std::vector<float> dbpskFrontEnd(const std::filesystem::path& file, double symbolRate, double carrierHz, ChipCombine combine = ChipCombine::none) {
    constexpr gr::Size_t kHilbertTaps = 127U;
    gr::Graph            flow;

    auto& source = flow.emplaceBlock<gr::blocks::fileio::WavSource<float>>(wavSource(file));
    // A stream feeds one interior port, so the branch begins at a delay of zero, which is a copy.
    auto& split    = flow.emplaceBlock<gr::blocks::basic::SampleDelay<float>>({{"delay", gr::Size_t{0}}});
    auto& hilbert  = flow.emplaceBlock<gr::blocks::filter::DesignedFilter<float, float>>({{"profile", std::string("hilbert")}, {"sample_rate", static_cast<float>(kAudioRate)}, {"taps", kHilbertTaps}});
    auto& delay    = flow.emplaceBlock<gr::blocks::basic::SampleDelay<float>>({{"delay", gr::Size_t{(kHilbertTaps - 1U) / 2U}}});
    auto& analytic = flow.emplaceBlock<gr::blocks::basic::RealImagToComplex<float>>();

    gr::recipes::DbpskDemod::Parameters parameters{static_cast<float>(kAudioRate), static_cast<float>(symbolRate)};
    parameters.frequency_offset    = carrierHz;
    parameters.decimation          = std::uint32_t{10};
    parameters.fll_noise_bandwidth = kFllNoiseBandwidth;
    const auto demod               = gr::recipes::DbpskDemod::emplace(flow, parameters);
    boost::ut::expect(demod != nullptr) << boost::ut::fatal;
    auto& sink = flow.emplaceBlock<FloatSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, split).has_value());
    boost::ut::expect(flow.connect<"out", "in">(split, hilbert).has_value());
    boost::ut::expect(flow.connect<"out", "in">(split, delay).has_value());
    boost::ut::expect(flow.connect<"out", "real">(delay, analytic).has_value());
    boost::ut::expect(flow.connect<"out", "imag">(hilbert, analytic).has_value());
    connectToComposite(flow, analytic, demod);
    if (combine == ChipCombine::twoTap) {
        // Half the difference of an adjacent pair, as a two-tap filter decimating by two. `taps[0]` multiplies the
        // newest chip, which is the second of the pair, so the earlier chip carries the positive weight. On a stream
        // that reached this point through a DIFFERENTIAL detector the result is unipolar at either pairing, which is
        // what this arm is here to show.
        auto& twoTap = flow.emplaceBlock<gr::blocks::filter::FirFilter<float, float>>({{"taps", std::vector<float>{-0.5f, 0.5f}}, {"decimation", gr::Size_t{2}}});
        connectFromComposite(flow, demod, twoTap);
        boost::ut::expect(flow.connect<"out", "in">(twoTap, sink).has_value());
    } else if (combine == ChipCombine::parity) {
        auto& parity = flow.emplaceBlock<gr::blocks::digital::ManchesterCombine>({{"input", std::string("differential")}});
        connectFromComposite(flow, demod, parity);
        boost::ut::expect(flow.connect<"out", "in">(parity, sink).has_value());
    } else {
        connectFromComposite(flow, demod, sink);
    }

    std::vector<float> soft;
    runGraph(std::move(flow), [&soft, &sink] { soft = sink._items; });
    return soft;
}
// ─── the Doppler leg: a trajectory pass, DBPSK, and the coded frames at the far end ───────────────────────────────

/*
 * The plan's IQ sentence, end to end and synthetic: frames go out over a ten-minute low-orbit pass with the pass's
 * envelope delay and carrier shift applied from a schedule, are corrected from the same schedule, and are recovered
 * through the differential detector and the coded framing. What the recordings vouch for on real air, this vouches
 * for on the tree's own pass, and the two halves meet at the same coded frame.
 *
 * The coding is the AO-40 FEC beacon's, at the geometry and the constants its published description states:
 * P. Karn (KA9Q), "A Proposal for a Coded AO-40 Telemetry Format", the specification the format is defined by, with
 * J. Miller (G3RUH), "Oscar-40 FEC Telemetry", for the interleaver's on-air reading. The beacon itself is 400 bit/s
 * Manchester-coded BPSK received noncoherently as differential BPSK; the FUNcube family carries the same coding at
 * 1200 bit/s, and the framing is unchanged by the rate.
 *
 * The encoder, in order: 256 user bytes, the even ones the data of the first Reed-Solomon codeword and the odd ones
 * the second; two systematic (160,128) codewords over GF(256), the CCSDS (255,223) code shortened by 95 leading
 * virtual-fill zeros in the conventional basis; the CCSDS randomizer over data and parity alike; the two codewords
 * byte-interleaved and encoded by the rate-1/2 constraint-length-7 code with G1 first and G2 inverted, six zero tail
 * bits, 5132 symbols; and an 80-row by 65-column interleaver whose first row holds the sync vector, written by rows
 * from the second row and read by columns, so a sync bit lands every 80th bit of the 5200-bit frame. The decoder is
 * that reversed.
 */

constexpr std::size_t kSyncItems    = 65UZ;                                ///< n, the sync vector's items, one per column of the rectangle
constexpr std::size_t kStride       = 80UZ;                                ///< s, the rectangle's rows, and the items between two sync bits
constexpr std::size_t kFrameItems   = 5200UZ;                              ///< n * s
constexpr std::size_t kRsInterleave = 2UZ;                                 ///< the two codewords, the even and the odd user bytes
constexpr std::size_t kRsData       = 128UZ;                               ///< 223 - 95, one shortened codeword's information bytes
constexpr std::size_t kRsCodeword   = 160UZ;                               ///< 255 - 95
constexpr std::size_t kFrameBytes   = kRsData * kRsInterleave;             ///< the user bytes one frame carries
constexpr std::size_t kCodeBlock    = kRsCodeword * kRsInterleave;         ///< the byte-interleaved codewords the trellis reads
constexpr std::size_t kTrellis      = 5132UZ;                              ///< (8 * 320 + 6) * 2, the terminated code's symbols
constexpr std::size_t kFramePad     = kFrameItems - kSyncItems - kTrellis; ///< places of the last row the frame leaves unused

// The construction's whole arithmetic, so that a change to one number cannot leave the others behind.
static_assert(kSyncItems * kStride == kFrameItems, "the frame is the sync vector's items times their spacing");
static_assert((8UZ * kCodeBlock + 6UZ) * 2UZ == kTrellis, "a terminated rate-1/2 code at K = 7 codes k + 6 steps");
static_assert(kSyncItems + kTrellis + kFramePad == kFrameItems, "the frame holds the vector, the trellis and the rest");
static_assert(kRsCodeword == kRsData + 32UZ, "the CCSDS (255,223) code carries thirty-two parity symbols at any shortening");
static_assert(kFramePad == 3UZ, "the last three places of the rectangle's last row are unused");

constexpr double      kPassSeconds  = 600.;
constexpr double      kKnotSeconds  = 20.;
constexpr double      kClosestRange = 500e3;
constexpr double      kGroundSpeed  = 7000.;
constexpr double      kUplinkHz     = 437e6;
constexpr double      kPskBaud      = 1200.;
constexpr std::size_t kPskSps       = 40UZ; ///< 48000 / 1200
constexpr double      kPskRolloff   = 0.35;
constexpr std::size_t kPskSpan      = 6UZ;
/// Complex noise powers the leg tries. The receiver has no matched filter — it decides on one interpolated sample
/// behind a channel filter at 0.75 symbol rates — so the operating point is not the trajectory pair's own run's, whose
/// receiver is matched-filtered at two samples a symbol and whose per-sample signal power is twenty times this one's.
constexpr std::array<double, 4UZ> kPassNoises{{0.35, 0.1, 0.03, 0.01}};
constexpr std::uint64_t           kPassSeed = 20260902ULL;

/// t, the distance the sync correlation accepts. At n = 65 a uniformly random window falls inside 12 with probability
/// 1.4e-7, which is one false frame per seven million items searched, and the true vector survives a channel at five
/// per cent bit error with probability 1 - 1.7e-5.
constexpr std::size_t kSyncErrors = 12UZ;

/// The AO-40 sync vector: the first 65 bits of the sequence from s(x) = x^7 + x^3 + 1 with the register all ones, as
/// the '0'/'1' string the correlator reads. It occupies the first row of the interleaver's rectangle, which puts it on
/// the air one bit in every 80.
[[nodiscard]] std::string syncVector() { return std::string("11111110000111011110010110010010000001000100110001011101011011000"); }

/// The same vector complemented. A differential detector recovers the transmitted stream up to an overall inversion
/// only when the transmitter differentially encoded it, so a receiver of real air tries both and says which matched.
[[nodiscard]] std::string complemented(std::string word) {
    for (char& bit : word) {
        bit = bit == '1' ? '0' : '1';
    }
    return word;
}

/**
 * @brief The fewest of the sync vector's 65 items any window of @p soft disagrees with, over either polarity.
 *
 * The correlator publishes a detection or nothing, so a chain that lands one item outside the threshold and one that
 * carries no vector at all look alike from outside it. This reads the distance itself, which separates the two and is
 * the sharper measurement of a front end: a window at zero is the transcribed vector matched exactly on real air.
 */
[[nodiscard]] std::size_t bestSyncWindow(std::span<const float> soft) {
    const std::string word = syncVector();
    const std::size_t span = (kSyncItems - 1UZ) * kStride;
    std::size_t       best = kSyncItems;
    if (soft.size() <= span) {
        return best;
    }
    for (std::size_t start = 0UZ; start + span < soft.size(); ++start) {
        std::size_t errors = 0UZ;
        for (std::size_t j = 0UZ; j < kSyncItems; ++j) {
            errors += (soft[start + j * kStride] > 0.f) == (word[j] == '1') ? 0UZ : 1UZ;
        }
        best = std::min({best, errors, kSyncItems - errors});
        if (best == 0UZ) {
            break;
        }
    }
    return best;
}

/// The named conventions the AO-40 chain is built from: the CCSDS randomizer, and the rate-1/2 constraint-length-7
/// code with G1 first and G2 inverted, which is the CCSDS convolutional code.
[[nodiscard]] gr::property_map ao40Randomizer() { return {{"profile", std::string("ccsds131")}}; }
[[nodiscard]] gr::property_map ao40Trellis() { return {{"code", std::string("ccsds")}}; }
[[nodiscard]] gr::property_map ao40ReedSolomon() { return {{"code", std::string("ccsds_255_223")}, {"basis", std::string("conventional")}, {"pad", gr::Size_t{95}}, {"interleave", static_cast<gr::Size_t>(kRsInterleave)}}; }

/// Frame @p index's information field: its own number in the first four bytes, then a stream seeded from it, so a
/// recovered frame names itself and no alignment has to be searched for.
[[nodiscard]] std::vector<std::uint8_t> passPayload(std::size_t index) {
    std::vector<std::uint8_t> bytes(kFrameBytes);
    bytes[0UZ]          = static_cast<std::uint8_t>((index >> 24U) & 0xFFU);
    bytes[1UZ]          = static_cast<std::uint8_t>((index >> 16U) & 0xFFU);
    bytes[2UZ]          = static_cast<std::uint8_t>((index >> 8U) & 0xFFU);
    bytes[3UZ]          = static_cast<std::uint8_t>(index & 0xFFU);
    std::uint64_t state = 0x9E3779B97F4A7C15ULL + static_cast<std::uint64_t>(index);
    for (std::size_t i = 4UZ; i < bytes.size(); ++i) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        bytes[i] = static_cast<std::uint8_t>(state & 0xFFULL);
    }
    return bytes;
}

/// The sync word written into the first `kSyncItems` items of a deinterleaved frame, the coded block behind it, and
/// the frame padded to the rectangle. The interleaver then sends the word to every `stride`-th transmitted item.
struct SyncInsert : gr::Block<SyncInsert> {
    gr::PortIn<Record, gr::Async>  in;
    gr::PortOut<Record, gr::Async> out;
    GR_MAKE_REFLECTABLE(SyncInsert, in, out);
    std::string _word{};

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(inSpan.size(), outSpan.size());
        for (std::size_t k = 0UZ; k < n; ++k) {
            std::vector<std::uint8_t> items;
            items.reserve(kFrameItems);
            for (const char bit : _word) {
                items.push_back(bit == '1' ? 1U : 0U);
            }
            const std::vector<std::uint8_t>& coded = inSpan[k].signal_values;
            items.insert(items.end(), coded.begin(), coded.end());
            items.resize(kFrameItems, 0U);
            outSpan[k] = recordOf(std::move(items));
        }
        std::ignore = inSpan.consume(n);
        outSpan.publish(n);
        return n == 0UZ ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::OK;
    }
};

/// The two bit streams a frame chain produces: the coded frame the far end recovers, and the same stream after the
/// differential encoding the detector undoes.
struct PassBits {
    std::vector<std::uint8_t> coded{}; ///< the interleaved frames, which is what the receiver's correlator reads
    std::vector<std::uint8_t> wire{};  ///< the same stream differentially encoded, which is what is transmitted
};

/// Every frame's transmitted bits, built through the landed coding blocks in one graph: Reed-Solomon, the bit
/// repacking, the terminated convolutional code, the sync word, the rectangle, and the differential encoding the
/// differential detector at the far end undoes.
[[nodiscard]] PassBits passWireBits(std::size_t frames) {
    gr::Graph flow;
    auto&     source = flow.emplaceBlock<RecordSource>();
    for (std::size_t k = 0UZ; k < frames; ++k) {
        source._records.push_back(recordOf(passPayload(k)));
    }
    auto& rs = flow.emplaceBlock<gr::blocks::fec::RsEncode>(ao40ReedSolomon());
    // The randomizer runs over the coded block, data and parity alike, and one record is one epoch.
    auto& random = flow.emplaceBlock<gr::blocks::digital::AdditiveScrambler<Record>>(ao40Randomizer());
    auto& unpack = flow.emplaceBlock<gr::blocks::digital::RecordRepackBits>({{"bits_in", gr::Size_t{8}}, {"bits_out", gr::Size_t{1}}});
    auto& conv   = flow.emplaceBlock<gr::blocks::fec::ConvEncode>(ao40Trellis());
    auto& sync   = flow.emplaceBlock<SyncInsert>();
    sync._word   = syncVector();
    auto& rect   = flow.emplaceBlock<gr::blocks::fec::Interleave<std::uint8_t>>({{"kind", std::string("block")}, {"rows", static_cast<gr::Size_t>(kStride)}, {"cols", static_cast<gr::Size_t>(kSyncItems)}});
    auto& wire   = flow.emplaceBlock<RecordToStream>();
    auto& coded  = flow.emplaceBlock<ByteTap>();
    // The differential detector decides on the product of adjacent symbols, so the transmitter sends the running
    // product: NRZI is that encoding, and a one leaves the transmitted symbol where it was.
    auto& differential = flow.emplaceBlock<DifferentialEncoder<std::uint8_t>>({{"modulus", gr::Size_t{2}}, {"coding", std::string("nrzi")}});
    auto& bits         = flow.emplaceBlock<ByteSink>();

    boost::ut::expect(flow.connect<"out", "in">(source, rs).has_value());
    boost::ut::expect(flow.connect<"out", "in">(rs, random).has_value());
    boost::ut::expect(flow.connect<"out", "in">(random, unpack).has_value());
    boost::ut::expect(flow.connect<"out", "in">(unpack, conv).has_value());
    boost::ut::expect(flow.connect<"out", "in">(conv, sync).has_value());
    boost::ut::expect(flow.connect<"out", "in">(sync, rect).has_value());
    boost::ut::expect(flow.connect<"out", "in">(rect, wire).has_value());
    boost::ut::expect(flow.connect<"out", "in">(wire, coded).has_value());
    boost::ut::expect(flow.connect<"out", "in">(coded, differential).has_value());
    boost::ut::expect(flow.connect<"out", "in">(differential, bits).has_value());

    PassBits out;
    runGraph(std::move(flow), [&out, &coded, &bits] {
        out.coded = coded._items;
        out.wire  = bits._items;
    });
    return out;
}

/// The pass's schedule: a flat ground track at `kGroundSpeed` with `kClosestRange` at closest approach, so the slant
/// range is sqrt(R0^2 + (v (t - T/2))^2) and its rate is v (t - T/2) v / R.
struct PassSchedule {
    std::vector<std::int64_t> times{};
    std::vector<double>       delays{};
    std::vector<double>       offsets{};
};

[[nodiscard]] PassSchedule leoPass() {
    PassSchedule pass;
    const int    knots = static_cast<int>(std::llround(kPassSeconds / kKnotSeconds));
    for (int k = 0; k <= knots; ++k) {
        const double t     = kPassSeconds * static_cast<double>(k) / static_cast<double>(knots);
        const double along = kGroundSpeed * (t - 0.5 * kPassSeconds);
        const double range = std::sqrt(kClosestRange * kClosestRange + along * along);
        pass.times.push_back(std::llround(t * 1e9));
        pass.delays.push_back(gr::timing::delayFor(range));
        pass.offsets.push_back(gr::timing::offsetFor(kGroundSpeed * along / range, kUplinkHz));
    }
    return pass;
}

/**
 * @brief The raised-cosine pulse at `kPskSps` samples a symbol, normalized to unit energy.
 *
 * A RAISED cosine and not its root, because the receiver has no matched filter: `DbpskDemod`'s channel filter is what
 * limits its noise and the decision is taken on one interpolated sample. A root pulse is Nyquist only against a second
 * root, so transmitting one and sampling it directly leaves intersymbol interference at every decision instant; the
 * full raised cosine is Nyquist on its own and its neighbors contribute nothing. The occupied bandwidth is the same
 * `(1 + rolloff) / 2` symbol rates either way.
 */
[[nodiscard]] std::vector<double> nyquistTaps() {
    const std::size_t   taps = kPskSpan * kPskSps + 1UZ;
    std::vector<double> h(taps);
    const double        beta = kPskRolloff;
    const double        pi   = std::numbers::pi;
    for (std::size_t k = 0UZ; k < taps; ++k) {
        const double t = (static_cast<double>(k) - 0.5 * static_cast<double>(taps - 1UZ)) / static_cast<double>(kPskSps);
        if (std::abs(t) < 1e-9) {
            h[k] = 1.;
        } else if (std::abs(std::abs(t) - 1. / (2. * beta)) < 1e-9) {
            h[k] = 0.25 * pi * std::sin(pi / (2. * beta)) / (pi / (2. * beta));
        } else {
            h[k] = (std::sin(pi * t) / (pi * t)) * std::cos(pi * beta * t) / (1. - std::pow(2. * beta * t, 2.));
        }
    }
    double energy = 0.;
    for (const double v : h) {
        energy += v * v;
    }
    for (double& v : h) {
        v /= std::sqrt(energy);
    }
    return h;
}

/// The blocks carry measurement slots and atomic counters, so they are pinned where they are built.
template<typename TBlock>
[[nodiscard]] std::unique_ptr<TBlock> configured(gr::property_map settings) {
    auto block = std::make_unique<TBlock>(std::move(settings));
    block->settings().init();
    std::ignore = block->settings().applyStagedParameters();
    block->start();
    return block;
}

/**
 * @brief The pass as a source: shaped DBPSK, the schedule's delay and shift applied, noise, and the same schedule
 * taken off again. `correct` false leaves the pass uncorrected, which is the negative control.
 *
 * The stream is produced rather than stored: ten minutes at 48 kS/s is 28.8 million complex samples, and the chain
 * either side of it needs none of them twice.
 */
struct PassSource : gr::Block<PassSource> {
    using C = std::complex<float>;
    gr::PortOut<C, gr::Async> out;

    /// A source's declared rate reaches every block behind it, and the demodulation recipe's filters are designed
    /// against it, so the pass states the rate it produces.
    gr::Annotated<float, "sample_rate", gr::Unit<"Hz">, gr::Doc<"rate of the produced stream">> sample_rate = static_cast<float>(kAudioRate);

    GR_MAKE_REFLECTABLE(PassSource, out, sample_rate);

    std::vector<std::uint8_t> _bits{};
    std::size_t               _samples = 0UZ;  ///< samples the pass runs for
    bool                      _impair  = true; ///< false leaves the schedule out entirely, which measures the modem alone
    bool                      _noisy   = true; ///< false leaves the noise out, which separates the schedule from the noise
    bool                      _correct = true;

    std::vector<double>                                   _shape = nyquistTaps();
    std::unique_ptr<gr::blocks::channel::RangeDelay<C>>   _applyDelay{};
    std::unique_ptr<gr::blocks::channel::DopplerShift<C>> _applyShift{};
    std::unique_ptr<gr::blocks::channel::AwgnChannel<C>>  _noise{};
    std::unique_ptr<gr::blocks::channel::DopplerShift<C>> _correctShift{};
    std::unique_ptr<gr::blocks::channel::RangeDelay<C>>   _correctDelay{};
    std::vector<C>                                        _a{};
    std::vector<C>                                        _b{};
    std::size_t                                           _at = 0UZ;

    /// The shaped sample at absolute index @p n: only the symbols whose pulse reaches it contribute.
    [[nodiscard]] double shapedAt(std::size_t n) const {
        double            acc  = 0.;
        const std::size_t span = _shape.size();
        const std::size_t last = n / kPskSps;
        const std::size_t deep = span / kPskSps;
        const std::size_t from = last > deep ? last - deep : 0UZ;
        for (std::size_t s = from; s <= last && s < _bits.size(); ++s) {
            const std::size_t tap = n - s * kPskSps;
            if (tap < span) {
                acc += _shape[tap] * (_bits[s] != 0U ? 1. : -1.);
            }
        }
        return acc;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t n = std::min(outSpan.size(), _samples - _at);
        if (n == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::DONE;
        }
        _a.resize(std::max(_a.size(), n));
        _b.resize(std::max(_b.size(), n));
        for (std::size_t i = 0UZ; i < n; ++i) {
            _a[i] = C(static_cast<float>(shapedAt(_at + i)), 0.f);
        }
        // The stages are optional so that a failure can be attributed: the schedule alone, the noise alone, or both.
        std::span<C> carried(_a.data(), n);
        std::span<C> spare(_b.data(), n);
        const auto   through = [&carried, &spare](auto& block) {
            std::ignore = block->processBulk(std::span<const C>(carried), spare);
            std::swap(carried, spare);
        };
        if (_impair) {
            through(_applyDelay);
            through(_applyShift);
        }
        if (_noisy) {
            through(_noise);
        }
        if (_impair && _correct) {
            through(_correctShift);
            through(_correctDelay);
        }
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = carried[i];
        }
        outSpan.publish(n);
        _at += n;
        return _at == _samples ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

/// What one run of the coded chain recovered, and where the count stood at each stage of it.
struct PassRun {
    std::uint64_t       symbols    = 0ULL; ///< soft symbols the detector produced
    std::uint64_t       detections = 0ULL; ///< times the sync correlation matched, whatever followed it
    std::uint64_t       located    = 0ULL; ///< frames a whole 5200 items of stream followed a match of
    std::uint64_t       decoded    = 0ULL; ///< frames the Viterbi decode published
    std::size_t         frames     = 0UZ;  ///< frames the Reed-Solomon decode published
    std::size_t         valid      = 0UZ;  ///< of those, the ones no codeword of which the code found uncorrectable
    std::size_t         matched    = 0UZ;  ///< of those, the ones that are the frame that was sent
    std::size_t         corrected  = 0UZ;  ///< symbols the Reed-Solomon decode corrected over the valid frames
    double              agreement  = 0.;   ///< the best fraction of a window of soft decisions that are the bit that was sent
    std::vector<Record> records{};

    [[nodiscard]] std::string stages() const { return std::format("{} symbols at {:.4f} agreement, {} sync detections, {} located, {} through the trellis, {} out of Reed-Solomon of which {} valid, {} whole, {} symbols corrected", symbols, agreement, detections, located, decoded, frames, valid, matched, corrected); }
};

/// The best fraction of @p soft whose sign is the bit @p sent carries, over any lag up to @p maxLag. Every chain here
/// carries filter and loop delays nothing compensates, so the alignment is searched rather than derived; what is
/// measured is the agreement and not the lag.
[[nodiscard]] double bestAgreement(std::span<const float> soft, std::span<const std::uint8_t> sent, std::size_t skip, std::size_t maxLag) {
    double best = 0.;
    for (std::size_t lag = 0UZ; lag <= maxLag; ++lag) {
        std::size_t matched = 0UZ;
        std::size_t counted = 0UZ;
        for (std::size_t k = skip; k + lag < soft.size() && k < sent.size(); ++k) {
            matched += ((soft[k + lag] > 0.f) == (sent[k] != 0U)) ? 1UZ : 0UZ;
            ++counted;
        }
        if (counted > 0UZ) {
            best = std::max(best, static_cast<double>(matched) / static_cast<double>(counted));
        }
    }
    return best;
}

/**
 * @brief The coded receive chain, from soft symbols to information records.
 *
 * @param plant  builds whatever produces the soft symbols and connects it to the chain's first block
 */
template<typename TPlant>
[[nodiscard]] PassRun runCodedChain(TPlant&& plant, const std::vector<std::uint8_t>& sent, const std::string& sync = syncVector()) {
    gr::Graph flow;

    auto& symbols               = flow.emplaceBlock<ItemCounter<float>>();
    symbols._keep               = 20000UZ; // enough of a window to read the modem's agreement off, and a rounding error of the run
    auto&            correlator = flow.emplaceBlock<gr::blocks::digital::AccessCodeCorrelator<float>>({{"access_code", sync}, {"stride", static_cast<gr::Size_t>(kStride)}, {"tag_at", std::string("code_start")}, {"max_errors", static_cast<gr::Size_t>(kSyncErrors)}});
    auto&            frames     = flow.emplaceBlock<gr::blocks::basic::StreamToDataSet<float>>({{"filter", std::string("[access_code]")}, {"n_pre", gr::Size_t{0}}, {"n_post", static_cast<gr::Size_t>(kFrameItems)}, {"n_max", static_cast<gr::Size_t>(kFrameItems)}});
    auto&            located    = flow.emplaceBlock<ItemCounter<gr::DataSet<float>>>();
    auto&            deinter    = flow.emplaceBlock<gr::blocks::fec::Deinterleave<float>>({{"kind", std::string("block")}, {"rows", static_cast<gr::Size_t>(kStride)}, {"cols", static_cast<gr::Size_t>(kSyncItems)}, {"output_offset", static_cast<gr::Size_t>(kSyncItems)}, {"output_length", static_cast<gr::Size_t>(kTrellis)}});
    gr::property_map trellis    = ao40Trellis();
    trellis["termination"]      = std::string("terminated");
    auto& viterbi               = flow.emplaceBlock<gr::blocks::fec::ViterbiDecodeSoft>(trellis);
    auto& decoded               = flow.emplaceBlock<ItemCounter<Record>>();
    auto& pack                  = flow.emplaceBlock<gr::blocks::digital::RecordRepackBits>({{"bits_in", gr::Size_t{1}}, {"bits_out", gr::Size_t{8}}});
    auto& derandom              = flow.emplaceBlock<gr::blocks::digital::AdditiveScrambler<Record>>(ao40Randomizer());
    auto& rs                    = flow.emplaceBlock<gr::blocks::fec::RsDecode>(ao40ReedSolomon());
    auto& sink                  = flow.emplaceBlock<RecordSink>();

    plant(flow, symbols);
    boost::ut::expect(flow.connect<"out", "in">(symbols, correlator).has_value());
    boost::ut::expect(flow.connect<"out", "in">(correlator, frames).has_value());
    boost::ut::expect(flow.connect<"out", "in">(frames, located).has_value());
    boost::ut::expect(flow.connect<"out", "in">(located, deinter).has_value());
    boost::ut::expect(flow.connect<"out", "in">(deinter, viterbi).has_value());
    boost::ut::expect(flow.connect<"out", "in">(viterbi, decoded).has_value());
    boost::ut::expect(flow.connect<"out", "in">(decoded, pack).has_value());
    boost::ut::expect(flow.connect<"out", "in">(pack, derandom).has_value());
    boost::ut::expect(flow.connect<"out", "in">(derandom, rs).has_value());
    boost::ut::expect(flow.connect<"out", "in">(rs, sink).has_value());

    PassRun            run;
    std::vector<float> window;
    runGraph(std::move(flow), [&run, &window, &symbols, &correlator, &located, &decoded, &sink] {
        run.detections = correlator.nDetections;
        run.symbols    = symbols._count;
        run.located    = located._count;
        run.decoded    = decoded._count;
        run.records    = sink._records;
        window         = symbols._kept;
    });
    run.agreement                = bestAgreement(std::span<const float>(window), std::span<const std::uint8_t>(sent), 200UZ, 400UZ);
    run.frames                   = run.records.size();
    const std::size_t sentFrames = sent.size() / kFrameItems;
    for (const Record& record : run.records) {
        // The decode publishes a record whether or not the code could correct it, so the frame that counts is the one
        // whose every codeword came back inside the code's correcting power.
        const auto& map   = metaOf(record);
        const auto  bad   = map.find(gr::property_map::key_type("uncorrectable_errors"));
        const auto  fixed = map.find(gr::property_map::key_type("corrected_errors"));
        if (bad != map.end() && static_cast<std::size_t>(bad->second.value_or(gr::Size_t{0})) != 0UZ) {
            continue;
        }
        ++run.valid;
        run.corrected += fixed == map.end() ? 0UZ : static_cast<std::size_t>(fixed->second.value_or(gr::Size_t{0}));
        if (record.signal_values.size() != kFrameBytes) {
            continue;
        }
        const std::size_t index = (static_cast<std::size_t>(record.signal_values[0UZ]) << 24U) | (static_cast<std::size_t>(record.signal_values[1UZ]) << 16U) | (static_cast<std::size_t>(record.signal_values[2UZ]) << 8U) | static_cast<std::size_t>(record.signal_values[3UZ]);
        if (index < sentFrames && record.signal_values == passPayload(index)) {
            ++run.matched;
        }
    }
    return run;
}

/// The coded chain over a stream of soft symbols already in hand.
[[nodiscard]] PassRun runCodedOverSoft(const std::vector<float>& soft, const std::vector<std::uint8_t>& sent, const std::string& sync) {
    return runCodedChain(
        [&soft](gr::Graph& flow, auto& first) {
            gr::Tensor<float> values(soft.begin(), soft.end());
            auto&             source = flow.emplaceBlock<TagSource<float, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(soft.size())}, {"values", values}, {"mark_tag", false}});
            boost::ut::expect(flow.connect<"out", "in">(source, first).has_value());
        },
        sent, sync);
}

/// The coded chain with no channel at all: the frames as soft symbols of unit magnitude, which is what the sign the
/// detector delivers means before any noise reaches it. A failure here is the coding's and not the modem's.
[[nodiscard]] PassRun runCodedNoChannel(const std::vector<std::uint8_t>& coded) {
    std::vector<float> soft(coded.size());
    for (std::size_t i = 0UZ; i < coded.size(); ++i) {
        soft[i] = coded[i] != 0U ? 1.f : -1.f;
    }
    return runCodedOverSoft(soft, coded, syncVector());
}

/// What the link between the two ends carries besides the signal.
enum class PassChannel { none, noiseOnly, passOnly, corrected, uncorrected };

[[nodiscard]] PassRun runPass(const PassSchedule& pass, const PassBits& bits, std::size_t samples, PassChannel channel, double noise) {
    using C = std::complex<float>;

    const auto scheduleOf = [&pass](std::string direction, gr::property_map extra) {
        gr::property_map map{{"sample_rate", static_cast<float>(kAudioRate)}, {"schedule_times_ns", pass.times}, {"direction", std::move(direction)}};
        for (auto& [key, value] : extra) {
            map[key] = value;
        }
        return map;
    };
    const double maxDelay = *std::max_element(pass.delays.begin(), pass.delays.end());

    return runCodedChain(
        [&](gr::Graph& flow, auto& first) {
            auto& source         = flow.emplaceBlock<PassSource>();
            source._bits         = bits.wire;
            source._samples      = samples;
            source._impair       = channel != PassChannel::none && channel != PassChannel::noiseOnly;
            source._noisy        = channel != PassChannel::none && channel != PassChannel::passOnly;
            source._correct      = channel != PassChannel::uncorrected;
            source._applyDelay   = configured<gr::blocks::channel::RangeDelay<C>>(scheduleOf("apply", {{"schedule_delays_s", pass.delays}}));
            source._applyShift   = configured<gr::blocks::channel::DopplerShift<C>>(scheduleOf("apply", {{"schedule_offsets_hz", pass.offsets}}));
            source._noise        = configured<gr::blocks::channel::AwgnChannel<C>>({{"noise_power", noise}, {"seed", kPassSeed}});
            source._correctShift = configured<gr::blocks::channel::DopplerShift<C>>(scheduleOf("correct", {{"schedule_offsets_hz", pass.offsets}}));
            source._correctDelay = configured<gr::blocks::channel::RangeDelay<C>>(scheduleOf("correct", {{"schedule_delays_s", pass.delays}, {"bias_s", std::ceil(maxDelay * kAudioRate) / kAudioRate}}));

            gr::recipes::DbpskDemod::Parameters parameters{static_cast<float>(kAudioRate), static_cast<float>(kPskBaud)};
            parameters.decimation = std::uint32_t{10};
            const auto demod      = gr::recipes::DbpskDemod::emplace(flow, parameters);
            boost::ut::expect(demod != nullptr) << boost::ut::fatal;
            connectToComposite(flow, source, demod);
            connectFromComposite(flow, demod, first);
        },
        bits.coded);
}

} // namespace

const boost::ut::suite<"Tier5Gates"> tier5Gates = [] {
    using namespace boost::ut;

    // A hundred APRS frames out through the tone pair and back in, closed at the KISS file
    "the APRS graph returns every frame at 20 dB, and the KISS file holds what was sent"_test = [] {
        std::vector<Record> payloads;
        for (std::size_t k = 0UZ; k < kAprsFrames; ++k) {
            payloads.push_back(recordOf(aprsInfo(k)));
        }

        const std::vector<std::uint8_t> bits  = aprsWireBits(payloads);
        const std::vector<float>        audio = afskAudio(std::span<const std::uint8_t>(bits), kKeyUpSymbols);
        expect(gt(bits.size(), kAprsFrames * 300UZ)) << std::format("{} wire bits for {} frames", bits.size(), kAprsFrames);

        const std::string path     = std::filesystem::temp_directory_path() / "qa_Tier5Gates_aprs.kiss";
        const AprsRun     run      = aprsReceive(audio, 20., path);
        const std::size_t perFrame = bits.size() / kAprsFrames;
        std::println("[G-A] {} frames, {} wire bits ({} a frame, {:.1f} ms of audio), key-up {} symbols", kAprsFrames, bits.size(), perFrame, 1e3 * static_cast<double>(audio.size()) / kAudioRate, kKeyUpSymbols);
        std::println("[G-A] Es/N0 20.0 dB (noise power {:.4f}): {} frames decoded, {} read back from the KISS file", toneNoisePower(20.), run.decoded.size(), run.readBack.size());

        expect(eq(run.decoded.size(), kAprsFrames)) << "every frame survives the link";
        expect(eq(run.readBack.size(), kAprsFrames)) << "and every one of them reaches the KISS file";

        for (std::size_t k = 0UZ; k < std::min(run.decoded.size(), kAprsFrames); ++k) {
            expect(that % (run.decoded[k].signal_values == payloads[k].signal_values)) << std::format("frame {}: the information field is byte identical", k);
            expect(eq(metaString(run.decoded[k], "ax25_destination"), std::string("APRS"))) << std::format("frame {}", k);
            expect(eq(metaString(run.decoded[k], "ax25_source"), std::string("N0CALL-9"))) << std::format("frame {}", k);
            expect(eq(metaString(run.decoded[k], "ax25_via"), std::string("WIDE1-1"))) << std::format("frame {}", k);
            expect(eq(metaString(run.decoded[k], "ax25_type"), std::string("UI"))) << std::format("frame {}", k);
            expect(metaBool(run.decoded[k], "crc_ok")) << std::format("frame {}", k);
        }
        for (std::size_t k = 0UZ; k < std::min(run.readBack.size(), kAprsFrames); ++k) {
            expect(that % (run.readBack[k].signal_values == payloads[k].signal_values)) << std::format("KISS frame {}: byte identical after KissDecode", k);
        }
    };

    // The count against the noise, the 8 dB point beside its derivation
    "the APRS frame count follows the noise"_test = [] {
        std::vector<Record> payloads;
        for (std::size_t k = 0UZ; k < kAprsFrames; ++k) {
            payloads.push_back(recordOf(aprsInfo(k)));
        }
        const std::vector<std::uint8_t> bits     = aprsWireBits(payloads);
        const std::vector<float>        audio    = afskAudio(std::span<const std::uint8_t>(bits), kKeyUpSymbols);
        const double                    perFrame = static_cast<double>(bits.size()) / static_cast<double>(kAprsFrames);
        const std::string               path     = std::filesystem::temp_directory_path() / "qa_Tier5Gates_aprs_sweep.kiss";

        std::size_t previous = 0UZ;
        for (const double esN0_db : {8., 10., 12., 14., 16., 20.}) {
            const AprsRun run = aprsReceive(audio, esN0_db, path);
            // Bell 202's tones are not orthogonal: at h = 0.8333 their correlation is |sin(pi h)/(pi h)| = 0.190986,
            // so the noncoherent bound is 0.5 exp(-(Eb/N0)(1 - 0.190986)/2). One bit a symbol makes Eb/N0 = Es/N0.
            const double ebN0      = std::pow(10., esN0_db / 10.);
            const double reference = 0.5 * std::exp(-ebN0 * (1. - 0.190986) / 2.);
            const double frameLoss = std::min(1., perFrame * reference);
            std::println("[G-A] Es/N0 {:4.1f} dB: {:3} of {} frames, {} read back; bound {:.3e} a bit gives a frame error rate of about {:.3f} over {:.0f} bits", esN0_db, run.decoded.size(), kAprsFrames, run.readBack.size(), reference, frameLoss, perFrame);

            expect(ge(run.decoded.size(), previous)) << std::format("Es/N0 {} dB: the count rises as the noise falls", esN0_db);
            expect(eq(run.decoded.size(), run.readBack.size())) << std::format("Es/N0 {} dB: the KISS file holds what the decoder published", esN0_db);
            previous = run.decoded.size();
        }
    };

    // Every recording whose satyaml names AFSK with AX.25 framing
    "the AFSK satellite graph decodes a frame from every AFSK AX.25 recording"_test = [] {
        std::size_t swept  = 0UZ;
        std::size_t served = 0UZ;
        for (const Capture& capture : kAfskCaptures) {
            const auto file = recording(capture.file);
            if (!file.has_value()) {
                continue;
            }
            ++swept;
            // A packet downlink is bursts: the timing loop reacquires on every preamble, so the loop bandwidth that
            // serves a continuous pass is not the one that serves a burst, and the sweep says which each file wanted.
            AirRun      run;
            double      chosen = kAfskLoopBandwidths.front();
            bool        first  = true;
            std::string counts;
            for (const double bandwidth : kAfskLoopBandwidths) {
                const AirRun attempt = afskAx25(*file, capture.symbolRate, bandwidth);
                std::format_to(std::back_inserter(counts), "{}{:.3f}:{}", counts.empty() ? "" : " ", bandwidth, attempt.crcOk);
                if (first || attempt.crcOk > run.crcOk) {
                    run    = attempt;
                    chosen = bandwidth;
                    first  = false;
                }
            }
            std::println("[G-B] {:<20} {:<12} {:6.0f} Bd: {:7} symbols, crc_ok by loop bandwidth [{}], best {:3} at {:.3f}, {:3} failed the check; {}", capture.file, capture.satellite, capture.symbolRate, run.symbols, counts, run.crcOk, chosen, run.crcFail, run.addresses());
            served += run.crcOk > 0UZ ? 1UZ : 0UZ;
            expect(gt(run.symbols, 0ULL)) << std::format("{}: the demodulator produced no symbols", capture.file);
        }
        if (swept == 0UZ) {
            return;
        }
        expect(ge(served, kAfskServed)) << std::format("{} of {} AFSK recordings decoded a frame; the collection's README says every file holds one", served, swept);
    };

    // The 9600-baud G3RUH recordings, and the descrambler's place in the chain
    "the FSK G3RUH satellite graph decodes a frame from every 9600-baud recording, the descrambler ahead of NRZI"_test = [] {
        std::size_t swept      = 0UZ;
        std::size_t served     = 0UZ;
        std::size_t bareServed = 0UZ;
        std::size_t reversed   = 0UZ;
        for (const Capture& capture : kG3ruhCaptures) {
            const auto file = recording(capture.file);
            if (!file.has_value()) {
                continue;
            }
            ++swept;
            // The row is one configuration's, not a best of several: the sweep stops at the first configuration that
            // decodes, and that configuration is what the row and the two control arms beside it are run under. No
            // single triple serves every capture -- what the post-detection cutoff buys the timing loop it costs the
            // slicer in noise -- so the sweep is the deliverable and the configuration is part of the measurement.
            AirRun      run;
            std::size_t chosen = 0UZ;
            std::string counts;
            for (std::size_t index = 0UZ; index < kG3ruhConfigs.size(); ++index) {
                run    = fskG3ruhAx25(*file, capture.symbolRate, kG3ruhConfigs[index], true, true);
                chosen = index;
                std::format_to(std::back_inserter(counts), "{}{}:{}", counts.empty() ? "" : " ", kG3ruhConfigs[index].label(), run.crcOk);
                if (run.crcOk > 0UZ) {
                    break;
                }
            }
            const FskConfig& config  = kG3ruhConfigs[chosen];
            const AirRun     bare    = fskG3ruhAx25(*file, capture.symbolRate, config, true, false);
            const AirRun     swapped = fskG3ruhAx25(*file, capture.symbolRate, config, false, true);
            std::println("[G-C] {:<18} {:<14} {:6.0f} Bd: {:7} symbols at {}, crc_ok {:3}, unconditioned {:3}, NRZI first {:3}, {:3} failed the check; {}", capture.file, capture.satellite, capture.symbolRate, run.symbols, config.label(), run.crcOk, bare.crcOk, swapped.crcOk, run.crcFail, run.addresses());
            std::println("[G-C] {:<18} swept [{}]", capture.file, counts);
            served += run.crcOk > 0UZ ? 1UZ : 0UZ;
            bareServed += bare.crcOk > 0UZ ? 1UZ : 0UZ;
            reversed += swapped.crcOk > 0UZ ? 1UZ : 0UZ;
            expect(gt(run.symbols, 0ULL)) << std::format("{}: the demodulator produced no symbols", capture.file);

            // The callsign pairs are the oracle: a frame that passes an AX.25 frame check sequence and decodes to a
            // callsign the satellite is known to transmit was made by a spacecraft. Every capture that yields one is
            // named here, so a chain that decodes the right number of frames from the wrong files still fails.
            for (const auto& [named, expected] : std::array<std::pair<std::string_view, std::string_view>, 8UZ>{{
                     {"aalto1.wav", "OH2A1S-11>OH2AGS"},
                     {"az02.wav", "ON02AZ>ZS1SCS"},
                     {"irazu.wav", "TI0IRA>TI0TEC"},
                     {"koyo.wav", "KOYOSC>GS-H20"},
                     {"tigrisat.wav", "HNATIG>CQ"},
                     {"ubakusat.wav", "YM1RAS>TA2MKA"},
                     {"us01.wav", "CQ>QBUS01"},
                     {"us04.wav", "KD8CJT>CQ"},
                 }}) {
                if (capture.file == named) {
                    expect(run.addresses().contains(expected)) << std::format("{}: expected {}, read {}", capture.file, expected, run.addresses());
                }
            }
        }
        if (swept == 0UZ) {
            return;
        }
        expect(ge(served, kG3ruhServed)) << std::format("{} of {} G3RUH recordings decoded a frame", served, swept);
        expect(eq(reversed, served)) << "the descrambler and the NRZI decode are both affine over GF(2), so the two orders recover the same stream";
        // T12's claim, which nothing measured before: the conditioning is worth exactly this many of the decodes,
        // at the very configurations that produced them
        expect(ge(served, bareServed + kConditioningWorth)) << std::format("{} recordings decode conditioned against {} unconditioned at the same configuration", served, bareServed);
    };

    // The DBPSK graph and the AO-40 coded framing behind it, over real air
    "the AO-40 FEC chain decodes frames from the recordings the satellite descriptions name"_test = [] {
        // The mean magnitude over the root mean square: a stream of clean antipodal decisions reads 1, one that is
        // nothing but Gaussian noise reads sqrt(2/pi) = 0.798, and a unipolar {0, 1} stream reads 1/sqrt(2) = 0.7071.
        // The figure says what kind of variable the chain is handing the slicer without knowing what it decodes to.
        const auto eyeOf = [](std::span<const float> stream) {
            double magnitude = 0.;
            double square    = 0.;
            for (const float value : stream) {
                magnitude += std::abs(static_cast<double>(value));
                square += static_cast<double>(value) * static_cast<double>(value);
            }
            const double count = static_cast<double>(std::max<std::size_t>(stream.size(), 1UZ));
            return square > 0. ? magnitude / (count * std::sqrt(square / count)) : 0.;
        };

        std::size_t frames = 0UZ;
        const auto  sweep  = [&frames, &eyeOf](const Capture& capture, bool manchester) {
            const auto file = recording(capture.file);
            if (!file.has_value()) {
                return;
            }
            const std::vector<float> audio   = readWav(*file);
            const double             seconds = static_cast<double>(audio.size()) / kAudioRate;
            // The chip rate for a Manchester link, the bit rate otherwise: what the detector must run at either way.
            const double             detected = manchester ? 2. * capture.symbolRate : capture.symbolRate;
            const double             carrier  = squaringCarrier(std::span<const float>(audio), 500., 3000.);
            const std::vector<float> soft     = dbpskFrontEnd(*file, detected, carrier, manchester ? ChipCombine::parity : ChipCombine::none);
            const double             eye      = eyeOf(std::span<const float>(soft));
            std::println("[G-D] {:<12} {:<8} {:5.0f} Bd: {:.2f} s of audio, carrier {:.1f} Hz, {} soft symbols against {:.0f} expected, mean over rms {:.3f} against 0.798 for noise", capture.file, capture.satellite, capture.symbolRate, seconds, carrier, soft.size(), seconds * capture.symbolRate, eye);

            if (manchester) {
                // The negative control is the fixed two-tap combine over the same front end. On a differentially
                // detected Manchester stream the two chips of one bit are antipodal, so the detector's intra-bit
                // output is a constant and its inter-bit output is the bit: half the difference of an adjacent pair
                // is then `0.5 (1 - b b')`, unipolar in either pairing, and half its levels sit on the slicer's
                // threshold. Selecting the inter-bit parity is what the block does instead.
                const std::vector<float> control       = dbpskFrontEnd(*file, detected, carrier, ChipCombine::twoTap);
                const double             controlEye    = eyeOf(std::span<const float>(control));
                const std::size_t        window        = bestSyncWindow(std::span<const float>(soft));
                const std::size_t        controlWindow = bestSyncWindow(std::span<const float>(control));
                std::println("[G-D] {:<12} best 65-item sync window: {} errors through the parity selection at mean over rms {:.3f}, {} through the two-tap combine at {:.3f} against 1/sqrt(2) = 0.7071", capture.file, window, eye, controlWindow, controlEye);

                expect(eq(window, kManchesterSyncErrors)) << std::format("{}: the transcribed vector is matched exactly on the air, at {} errors of {}", capture.file, window, kSyncItems);
                expect(gt(controlWindow, kManchesterSyncErrors)) << std::format("{}: the two-tap combine does not match it, at {} errors", capture.file, controlWindow);
                expect(gt(eye, 0.95)) << std::format("{}: the selected parity is an antipodal decision variable, at {:.3f}", capture.file, eye);
                expect(lt(controlEye, 0.75)) << std::format("{}: the two-tap combine's output is unipolar, at {:.3f} against 1/sqrt(2)", capture.file, controlEye);
            }

            // A differential detector recovers the transmitted stream up to an inversion, so both polarities of the
            // sync vector are tried and the one that locates frames is the link's.
            const std::vector<std::uint8_t> nothingSent;
            const PassRun                   upright  = runCodedOverSoft(soft, nothingSent, syncVector());
            const PassRun                   inverted = runCodedOverSoft(soft, nothingSent, complemented(syncVector()));
            const bool                      flipped  = inverted.detections > upright.detections;
            const PassRun&                  best     = flipped ? inverted : upright;
            const char*                     polarity = flipped ? "inverted" : "upright";

            std::string opening;
            for (const Record& record : best.records) {
                const auto& map = metaOf(record);
                const auto  bad = map.find(gr::property_map::key_type("uncorrectable_errors"));
                const bool  ok  = bad == map.end() || static_cast<std::size_t>(bad->second.value_or(gr::Size_t{0})) == 0UZ;
                if (ok && record.signal_values.size() == kFrameBytes && opening.empty()) {
                    for (std::size_t k = 0UZ; k < 8UZ; ++k) {
                        std::format_to(std::back_inserter(opening), "{}{:02X}", opening.empty() ? "" : " ", record.signal_values[k]);
                    }
                }
            }
            // A frame occupies 5200 symbols and the correlation reads its lag out of the 5121 before the code's end,
            // so a capture carries a whole frame only when a detection falls that far from its end.
            const double frameSeconds = static_cast<double>(kFrameItems) / capture.symbolRate;
            std::println("[G-D] {:<12} {} sync, {:.2f} frames of audio: {} detections, {} located, {} through the trellis, {} out of Reed-Solomon of which {} valid, {} symbols corrected; the frame opens {}", capture.file, polarity, seconds / frameSeconds, best.detections, best.located, best.decoded, best.frames, best.valid, best.corrected, opening.empty() ? std::string("no valid frame") : opening);

            expect(lt(std::abs(static_cast<double>(soft.size()) - seconds * capture.symbolRate), 0.02 * seconds * capture.symbolRate)) << std::format("{}: one soft symbol a bit, within two per cent of the transmitter's rate", capture.file);
            expect(ge(best.detections, 1ULL)) << std::format("{}: the sync vector the published description states is on the air", capture.file);
            // A whole frame follows a match only where the capture is long enough to hold one behind the correlation's
            // own lag of `(n - 1) s + 1` items, which is `D + n s = 10 321` symbols. `ao73.wav` carries 6703 and so
            // cannot; `qo100.wav` carries 24 080 and so must.
            if (soft.size() >= (kSyncItems - 1UZ) * kStride + 1UZ + kFrameItems) {
                expect(ge(best.located, 1ULL)) << std::format("{}: the capture is long enough to hold a whole frame behind a match, so one is located", capture.file);
            }
            frames += best.valid;
        };

        for (const Capture& capture : kAo40Captures) {
            sweep(capture, false);
        }
        for (const Capture& capture : kAo40ManchesterCaptures) {
            sweep(capture, true);
        }
        if (recordingsRoot().empty()) {
            return;
        }
        // Frames, not recordings: the two captures between them hold 5.92 frames of audio, and this counts the ones
        // no codeword of which the Reed-Solomon decode found uncorrectable.
        expect(ge(frames, kAo40Frames)) << std::format("{} frames came out of Reed-Solomon with every codeword inside its correcting power", frames);
    };

    // The coded framing on its own, so that a failure of the leg below is the modem's and not this.
    "the AO-40-shaped coded framing round trips with no channel between its ends"_test = [] {
        const PassBits bits = passWireBits(4UZ);
        expect(eq(bits.coded.size(), 4UZ * kFrameItems)) << std::format("{} items for four frames", bits.coded.size());
        expect(eq(bits.wire.size(), bits.coded.size()));

        const PassRun run = runCodedNoChannel(bits.coded);
        std::println("[pass] no channel:  {}", run.stages());
        // The first frame's sync word stands at the stream's first item and the correlation reads its lag out of the
        // items before it, so a stream that opens on a frame opens on the one frame nothing can locate.
        expect(ge(run.matched, 3UZ)) << "every frame after the first comes back whole through the sync vector, the rectangle, the randomizer, the trellis and Reed-Solomon";
        expect(eq(run.corrected, 0UZ)) << "and with nothing for the code to correct";
    };

    // The IQ half. A pass applied and corrected around a coded DBPSK link, and the frames at the far end.
    "a corrected trajectory pass carries coded frames end to end, and the uncorrected pass carries none"_test = [] {
        const PassSchedule pass   = leoPass();
        const std::size_t  frames = static_cast<std::size_t>(kPassSeconds * kAudioRate) / (kFrameItems * kPskSps);
        const PassBits     bits   = passWireBits(frames);
        expect(eq(bits.wire.size(), frames * kFrameItems)) << std::format("{} frames of {} items", frames, kFrameItems);
        const std::size_t samples = bits.wire.size() * kPskSps;

        const double minDelay  = *std::min_element(pass.delays.begin(), pass.delays.end());
        const double maxDelay  = *std::max_element(pass.delays.begin(), pass.delays.end());
        const double minOffset = *std::min_element(pass.offsets.begin(), pass.offsets.end());
        const double maxOffset = *std::max_element(pass.offsets.begin(), pass.offsets.end());

        const auto start = std::chrono::steady_clock::now();

        // The operating point is found rather than assumed: a tenth of the pass over a range of noise powers says
        // where this receiver's threshold is, and the leg then runs the whole pass at the strongest noise that clears
        // it. The threshold is the demodulation's, not the coding's, which the no-channel and pass-only arms fix.
        const std::size_t probeFrames = (samples / 10UZ) / (kFrameItems * kPskSps);

        double chosen = kPassNoises.back();
        for (const double noise : kPassNoises) {
            const PassRun probe = runPass(pass, bits, samples / 10UZ, PassChannel::corrected, noise);
            std::println("[pass] noise {:.2f}:    {}", noise, probe.stages());
            if (probe.matched * 10UZ >= probeFrames * 9UZ) {
                chosen = noise;
                break;
            }
        }

        const PassRun quiet     = runPass(pass, bits, samples, PassChannel::none, chosen);
        const PassRun noisy     = runPass(pass, bits, samples, PassChannel::noiseOnly, chosen);
        const PassRun schedule  = runPass(pass, bits, samples, PassChannel::passOnly, chosen);
        const PassRun corrected = runPass(pass, bits, samples, PassChannel::corrected, chosen);
        const PassRun raw       = runPass(pass, bits, samples, PassChannel::uncorrected, chosen);
        const double  elapsed   = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        std::println("[pass] {} frames of {} bytes over {:.0f} s at {} Bd: envelope delay moves {:.1f} samples, carrier shift {:.1f} to {:.1f} kHz, noise power {:.2f} for Es/N0 {:.2f} dB", frames, kFrameBytes, static_cast<double>(samples) / kAudioRate, kPskBaud, (maxDelay - minDelay) * kAudioRate, 1e-3 * minOffset, 1e-3 * maxOffset, chosen, 10. * std::log10(1. / chosen));
        // The uncorrected pass is not dead everywhere: the carrier shift passes through zero at closest approach, and
        // while it is inside the receiver's channel filter at 0.75 symbol rates the link works. The slope there says
        // for how long, and the frame count is read against that rather than against nothing.
        double steepest = 0.;
        for (std::size_t k = 1UZ; k < pass.offsets.size(); ++k) {
            steepest = std::max(steepest, std::abs(pass.offsets[k] - pass.offsets[k - 1UZ]) / kKnotSeconds);
        }
        const double window = 2. * (0.75 * kPskBaud) / steepest;
        std::println("[pass] no pass:     {}", quiet.stages());
        std::println("[pass] noise only:  {}", noisy.stages());
        std::println("[pass] pass only:   {}", schedule.stages());
        std::println("[pass] corrected:   {}", corrected.stages());
        std::println("[pass] uncorrected: {}; the shift is inside the channel filter for about {:.0f} s of {:.0f}, which is {:.1f} frames; every run in {:.1f} s", raw.stages(), window, static_cast<double>(samples) / kAudioRate, window * kPskBaud / static_cast<double>(kFrameItems), elapsed);

        expect(gt((maxOffset - minOffset) / kPskBaud, 8.)) << "the pass's carrier shift is many symbol rates wide, which no receiver loop can acquire on its own";
        expect(gt(quiet.matched, frames * 9UZ / 10UZ)) << std::format("with neither pass nor noise, {} of {} frames recovered whole", quiet.matched, frames);
        expect(gt(schedule.matched, frames * 9UZ / 10UZ)) << std::format("with the pass applied and corrected and no noise, {} of {} frames recovered whole", schedule.matched, frames);
        expect(gt(corrected.matched, frames * 9UZ / 10UZ)) << std::format("{} of {} frames recovered whole over the corrected pass", corrected.matched, frames);
        expect(gt(corrected.matched, noisy.matched / 2UZ + 1UZ)) << "the corrected pass is not worse than the same noise without a pass";
        expect(lt(raw.matched * 4UZ, corrected.matched)) << std::format("with the pass left uncorrected only {} frames survive against {} corrected, and only around closest approach", raw.matched, corrected.matched);
    };
};

int main() {
    // The suites are run here rather than at exit so that the process can distinguish two outcomes the default runner
    // does not: everything passed, and everything that could run passed while a recording leg had no capture to read.
    // The second is a skip, which ctest is told about by the exit status a skip is registered under.
    const bool failed = boost::ut::cfg<boost::ut::override>.run({.report_errors = true});
    if (failed) {
        return 1;
    }
    return g_recordingsAbsent ? 77 : 0;
}
