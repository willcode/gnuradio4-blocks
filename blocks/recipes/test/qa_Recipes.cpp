/* The recipes gate: every recipe listed in blocks/recipes/index.yaml loads through the
 * standard YAML-definitions machinery and instantiates as a composite block with its
 * exported ports in place. A recipe is data; this is the test that keeps it honest. A
 * general recipe's required parameters are part of the contract: instantiating it bare
 * must refuse by name, and instantiating it with the parameters must derive the interior
 * settings and keep deriving them when a parameter changes live. */
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <filesystem>
#include <format>
#include <mutex>
#include <numbers>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <boost/ut.hpp>

#include <fstream>
#include <sstream>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Graph_yaml_importer.hpp>
#include <gnuradio-4.0/PluginLoader.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

#include <gnuradio-4.0/algorithm/digital/Scrambler.hpp>
#include <gnuradio-4.0/algorithm/fec/Convolutional.hpp>
#include <gnuradio-4.0/algorithm/fec/ReedSolomon.hpp>
#include <gnuradio-4.0/algorithm/filter/FilterDesign.hpp>
#include <gnuradio-4.0/ax25/Kiss.hpp>
#include <gnuradio-4.0/digital/AccessCodeCorrelator.hpp>
#include <gnuradio-4.0/digital/DelimiterExtractor.hpp>
// the socket legs need the optional network family
#ifdef GNURADIO4_HAVE_NETWORK_BLOCKS
#include <gnuradio-4.0/network/TcpByteIo.hpp>
#endif // GNURADIO4_HAVE_NETWORK_BLOCKS
#include <gnuradio-4.0/testing/TagMonitors.hpp>

#include <gnuradio-4.0/recipes/KissFileRead.hpp>
#include <gnuradio-4.0/recipes/KissFileWrite.hpp>
#ifdef GNURADIO4_HAVE_NETWORK_BLOCKS
#include <gnuradio-4.0/recipes/KissServe.hpp>
#endif // GNURADIO4_HAVE_NETWORK_BLOCKS
#include <gnuradio-4.0/recipes/KissStreamDecode.hpp>
#include <gnuradio-4.0/recipes/NbfmDemod.hpp>
#include <gnuradio-4.0/recipes/WbfmMonoDemod.hpp>

#include "RecipeHeaderEmitter.hpp"

namespace {

gr::PluginLoader makeRecipeLoader() {
    static gr::SchedulerRegistry          schedulerRegistry;
    static const std::vector<std::string> paths{std::string(RECIPES_SOURCE_PATH)};
    return gr::PluginLoader(gr::globalBlockRegistry(), schedulerRegistry, paths);
}

// exportedInputPorts()/exportedOutputPorts() return a nested map:
//   { blockUniqueName -> { internalPortName -> { "exportedName" -> name } } }
std::vector<std::string> exportedNames(const gr::property_map& portsMap) {
    std::vector<std::string> names;
    for (const auto& [blockName, portInfoValue] : portsMap) {
        const auto* portMap = portInfoValue.get_if<gr::property_map>();
        if (portMap == nullptr) {
            continue;
        }
        for (const auto& [internalName, exportInfoValue] : *portMap) {
            const auto* exportMap = exportInfoValue.get_if<gr::property_map>();
            if (exportMap == nullptr) {
                continue;
            }
            if (const auto it = exportMap->find("exportedName"); it != exportMap->end()) {
                names.emplace_back(it->second.value_or(std::string_view{}));
            }
        }
    }
    return names;
}

[[nodiscard]] std::shared_ptr<gr::BlockModel> interiorByName(const std::shared_ptr<gr::BlockModel>& composite, std::string_view name) {
    if (composite == nullptr || composite->graph() == nullptr) {
        return nullptr;
    }
    for (const auto& candidate : composite->graph()->blocks()) {
        if (candidate->name() == name) {
            return candidate;
        }
    }
    return nullptr;
}

[[nodiscard]] double numericOf(const gr::pmt::Value& value) { return gr::recipe::detail::doubleOf(value).value_or(0.0); }

[[nodiscard]] std::string stringOf(const gr::pmt::Value& value) { return std::string(value.value_or(std::string_view{})); }

/// what one frequency accounts for in a real stream: its amplitude, and its share of the stream's mean square
struct ToneReading {
    double amplitude{};
    double powerShare{};
};

/// A single-frequency DFT bin evaluated at an arbitrary frequency, so the reading does not depend on the
/// window length landing on a transform grid.
[[nodiscard]] ToneReading readTone(std::span<const float> samples, double frequency, double sampleRate) {
    double real  = 0.0;
    double imag  = 0.0;
    double power = 0.0;
    for (std::size_t i = 0UZ; i < samples.size(); ++i) {
        const double angle = 2.0 * std::numbers::pi * frequency * static_cast<double>(i) / sampleRate;
        const double value = static_cast<double>(samples[i]);
        real += value * std::cos(angle);
        imag -= value * std::sin(angle);
        power += value * value;
    }
    const double count      = static_cast<double>(samples.size());
    const double meanSquare = power / count;
    const double amplitude  = 2.0 * std::hypot(real, imag) / count;
    return {amplitude, meanSquare > 0.0 ? 0.5 * amplitude * amplitude / meanSquare : 0.0};
}

/// @brief Drives @p name over @p input under the scheduler and hands back what the sink saw. A recipe's numbers only
/// mean something if the chain they configure demodulates, so every functional case below goes through a real graph.
template<typename TIn, typename TOut>
[[nodiscard]] std::vector<TOut> runRecipe(std::string_view name, const gr::property_map& parameters, const std::vector<TIn>& input) {
    using gr::blocks::testing::ProcessFunction;

    auto loader    = makeRecipeLoader();
    auto composite = loader.instantiate(std::string(name), parameters);
    boost::ut::expect(composite != nullptr) << name << boost::ut::fatal;

    gr::Graph       graph;
    gr::Tensor<TIn> values(input.begin(), input.end());
    auto&           source = graph.emplaceBlock<gr::blocks::testing::TagSource<TIn, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(input.size())}, {"values", values}, {"mark_tag", false}});
    auto&           sink   = graph.emplaceBlock<gr::blocks::testing::TagSink<TOut, ProcessFunction::USE_PROCESS_BULK>>({{"name", "recipe_out"}});
    const auto      demod  = graph.addBlock(std::move(composite));

    const auto sourceModel = gr::graph::findBlock(graph, source);
    const auto sinkModel   = gr::graph::findBlock(graph, sink);
    boost::ut::expect(sourceModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
    boost::ut::expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, demod, gr::PortDefinition{"in"}).has_value()) << name;
    boost::ut::expect(graph.connect(demod, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value()) << name;

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
    const auto finished = scheduler.runAndWait();
    boost::ut::expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);
    return std::vector<TOut>(sink._samples.begin(), sink._samples.end());
}

/// @brief A sink for a record-typed recipe output, collecting what the chain publishes.
struct CcsdsRecordSink : gr::Block<CcsdsRecordSink> {
    gr::PortIn<gr::DataSet<std::uint8_t>, gr::Async> in;
    GR_MAKE_REFLECTABLE(CcsdsRecordSink, in);
    std::vector<gr::DataSet<std::uint8_t>> _records;
    [[nodiscard]] gr::work::Status         processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& r : inSpan) {
            _records.push_back(r);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

// ─── the four KISS recipes (criterion 14): a record source, a byte source that never ends, a graph runner ────────

/// @brief A source of prepared records, feeding a composite's `in` port and reporting DONE once exhausted.
struct KissRecordSource : gr::Block<KissRecordSource> {
    gr::PortOut<gr::DataSet<std::uint8_t>, gr::Async> out;
    GR_MAKE_REFLECTABLE(KissRecordSource, out);
    std::vector<gr::DataSet<std::uint8_t>> _records{};
    std::size_t                            _pos = 0UZ;
    // A chain that ends in a socket outlives its own record queue: the peer may still be connecting when the last
    // record is consumed, so the source can be told to idle instead of ending the graph under it. Left false it
    // reports the plain, natural DONE that lets a graph writing to a file terminate on its own.
    bool _neverDone = false;

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_pos >= _records.size()) {
            outSpan.publish(0UZ);
            return _neverDone ? gr::work::Status::INSUFFICIENT_INPUT_ITEMS : gr::work::Status::DONE;
        }
        const std::size_t n = std::min(outSpan.size(), _records.size() - _pos);
        for (std::size_t i = 0UZ; i < n; ++i) {
            outSpan[i] = _records[_pos + i];
        }
        outSpan.publish(n);
        _pos += n;
        return gr::work::Status::OK;
    }
};

/// @brief One record, its bytes and its carrier timestamp, shaped as the KISS recipes' `in` port expects.
[[nodiscard]] gr::DataSet<std::uint8_t> kissPayload(std::vector<std::uint8_t> bytes, std::int64_t timestamp = 0) {
    gr::DataSet<std::uint8_t> record;
    record.signal_values = std::move(bytes);
    record.extents.push_back(static_cast<std::int32_t>(record.signal_values.size()));
    record.signal_names.emplace_back("kiss");
    record.timing_events.resize(1UZ);
    record.meta_information.resize(1UZ);
    record.timestamp = timestamp;
    return record;
}

// the socket legs need the optional network family
#ifdef GNURADIO4_HAVE_NETWORK_BLOCKS
/// @brief Runs a graph in a background thread until explicitly stopped — `KissServe`'s sink and its own I/O thread
/// have no natural end, so the test owns the teardown rather than waiting on scheduler.runAndWait() to return.
struct KissGraphRunner {
    gr::scheduler::Simple<> scheduler;
    std::thread             worker;

    explicit KissGraphRunner(gr::Graph&& graph) {
        boost::ut::expect(scheduler.exchange(std::move(graph)).has_value());
        worker = std::thread([this] { std::ignore = scheduler.runAndWait(); });
    }
    KissGraphRunner(const KissGraphRunner&)            = delete;
    KissGraphRunner& operator=(const KissGraphRunner&) = delete;

    void stop() {
        scheduler.requestStop();
        if (worker.joinable()) {
            worker.join();
        }
    }
    ~KissGraphRunner() { stop(); }
};

template<typename F>
[[nodiscard]] bool kissWaitUntil(F&& ready, std::chrono::milliseconds deadline = std::chrono::milliseconds(8000)) {
    const auto until = std::chrono::steady_clock::now() + deadline;
    while (std::chrono::steady_clock::now() < until) {
        if (ready()) {
            return true;
        }
        std::this_thread::yield();
    }
    return ready();
}

/// @brief An ephemeral loopback port: bind a throwaway socket to port 0, read back what the kernel chose, close it —
/// which is what leaves `KissServe`'s own listener free to bind that same port a moment later.
[[nodiscard]] std::uint16_t reserveKissPort() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    boost::ut::expect(fd >= 0) << "could not open a socket to reserve a port";
    ::sockaddr_in address{};
    address.sin_family      = AF_INET;
    address.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    address.sin_port        = 0;
    boost::ut::expect(::bind(fd, reinterpret_cast<const ::sockaddr*>(&address), static_cast<::socklen_t>(sizeof(address))) == 0);
    ::socklen_t length = static_cast<::socklen_t>(sizeof(address));
    boost::ut::expect(::getsockname(fd, reinterpret_cast<::sockaddr*>(&address), &length) == 0);
    const std::uint16_t port = ::ntohs(address.sin_port);
    std::ignore              = ::close(fd);
    return port;
}

/// @brief A byte-stream sink whose collection the test thread may read while the graph is still running.
struct KissByteSink : gr::Block<KissByteSink> {
    gr::PortIn<std::uint8_t> in;
    GR_MAKE_REFLECTABLE(KissByteSink, in);

    mutable std::mutex        _mutex;
    std::vector<std::uint8_t> _bytes{};

    [[nodiscard]] std::size_t count() const {
        std::lock_guard lock(_mutex);
        return _bytes.size();
    }

    [[nodiscard]] std::vector<std::uint8_t> take() const {
        std::lock_guard lock(_mutex);
        return _bytes;
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        {
            std::lock_guard lock(_mutex);
            for (std::size_t i = 0UZ; i < inSpan.size(); ++i) {
                _bytes.push_back(inSpan[i]);
            }
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};
#endif // GNURADIO4_HAVE_NETWORK_BLOCKS

/// @brief The bytes `KissFileWrite` puts in a file for @p records, which is the KISS-over-SLIP wire form of the
/// encode-frame-flatten chain `KissServe` also carries. @p path is written and removed again.
[[nodiscard]] std::vector<std::uint8_t> kissWireForm(const std::vector<gr::DataSet<std::uint8_t>>& records, gr::property_map writeParameters, const std::string& path) {
    std::filesystem::remove(path);
    writeParameters["file_name"] = std::pmr::string(path);
    writeParameters["mode"]      = std::pmr::string("overwrite");

    auto loader    = makeRecipeLoader();
    auto composite = loader.instantiate("gr::recipes::KissFileWrite", writeParameters);
    boost::ut::expect(composite != nullptr) << boost::ut::fatal;

    gr::Graph graph;
    auto&     source       = graph.emplaceBlock<KissRecordSource>();
    source._records        = records;
    const auto chain       = graph.addBlock(std::move(composite));
    const auto sourceModel = gr::graph::findBlock(graph, source);
    boost::ut::expect(sourceModel.has_value()) << boost::ut::fatal;
    boost::ut::expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, chain, gr::PortDefinition{"in"}).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
    boost::ut::expect(scheduler.runAndWait().has_value());

    std::vector<std::uint8_t> wire;
    {
        std::ifstream in(path, std::ios::binary);
        boost::ut::expect(bool(in)) << boost::ut::fatal;
        wire.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    std::filesystem::remove(path);
    return wire;
}

/// @brief What `KissFileRead` decodes back out of @p wire, through a file it is given and then relieved of.
[[nodiscard]] std::vector<gr::DataSet<std::uint8_t>> kissReadBack(std::span<const std::uint8_t> wire, gr::property_map readParameters, const std::string& path) {
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        boost::ut::expect(bool(out)) << boost::ut::fatal;
        out.write(reinterpret_cast<const char*>(wire.data()), static_cast<std::streamsize>(wire.size()));
    }
    readParameters["file_name"] = std::pmr::string(path);

    auto loader    = makeRecipeLoader();
    auto composite = loader.instantiate("gr::recipes::KissFileRead", readParameters);
    boost::ut::expect(composite != nullptr) << boost::ut::fatal;

    gr::Graph  graph;
    auto&      sink      = graph.emplaceBlock<CcsdsRecordSink>();
    const auto chain     = graph.addBlock(std::move(composite));
    const auto sinkModel = gr::graph::findBlock(graph, sink);
    boost::ut::expect(sinkModel.has_value()) << boost::ut::fatal;
    boost::ut::expect(graph.connect(chain, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

    gr::scheduler::Simple scheduler;
    boost::ut::expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
    boost::ut::expect(scheduler.runAndWait().has_value());
    std::filesystem::remove(path);
    return sink._records;
}

/// @brief A deterministic bit stream, so a failure is the chain's and never the draw's.
[[nodiscard]] std::vector<int> sourceBits(std::size_t count, std::uint64_t seed = 0x9e3779b97f4a7c15ULL) {
    std::vector<int> bits(count);
    std::uint64_t    state = seed;
    for (int& bit : bits) {
        state ^= state << 13U;
        state ^= state >> 7U;
        state ^= state << 17U;
        bit = static_cast<int>(state & 1ULL);
    }
    return bits;
}

/// @brief The best agreement between the recovered signs and @p bits over any lag up to @p maxLag, and the lag it was
/// found at. Every chain here carries filter and loop delays the recipe does not compensate, so the alignment is
/// searched rather than derived; what is measured is the agreement, not the lag.
struct Agreement {
    double      fraction = 0.0;
    std::size_t lag      = 0UZ;
    bool        inverted = false;
};

[[nodiscard]] Agreement bestAgreement(std::span<const float> soft, std::span<const int> bits, std::size_t skip, std::size_t maxLag, bool allowInversion) {
    Agreement best;
    for (std::size_t lag = 0UZ; lag <= maxLag; ++lag) {
        for (const bool invert : {false, true}) {
            if (invert && !allowInversion) {
                continue;
            }
            std::size_t matched = 0UZ;
            std::size_t counted = 0UZ;
            for (std::size_t k = skip; k + lag < soft.size() && k < bits.size(); ++k) {
                const int decided = (soft[k + lag] > 0.f) != invert ? 1 : 0;
                matched += decided == bits[k] ? 1UZ : 0UZ;
                ++counted;
            }
            if (counted == 0UZ) {
                continue;
            }
            const double fraction = static_cast<double>(matched) / static_cast<double>(counted);
            if (fraction > best.fraction) {
                best = {fraction, lag, invert};
            }
        }
    }
    return best;
}

/// @brief Phase-continuous binary FSK on a REAL carrier: the audio a soundcard hands an AFSK receiver.
[[nodiscard]] std::vector<float> afskAudio(std::span<const int> bits, double sampleRate, double symbolRate, double markHz, double spaceHz) {
    const auto         perSymbol = static_cast<std::size_t>(sampleRate / symbolRate);
    std::vector<float> audio;
    audio.reserve(bits.size() * perSymbol);
    double phase = 0.0;
    for (const int bit : bits) {
        const double tone = bit == 1 ? markHz : spaceHz;
        for (std::size_t n = 0UZ; n < perSymbol; ++n) {
            audio.push_back(static_cast<float>(std::cos(phase)));
            phase += 2.0 * std::numbers::pi * tone / sampleRate;
        }
    }
    return audio;
}

/// @brief Root-raised-cosine shaped BPSK at complex baseband, with a residual carrier offset the receiver must find.
[[nodiscard]] std::vector<std::complex<float>> bpskBaseband(std::span<const int> symbols, std::size_t samplesPerSymbol, double rolloff, double offsetCyclesPerSample, double phase0) {
    const auto shape = gr::filter::design::designRootRaisedCosine(static_cast<int>(8UZ * samplesPerSymbol + 1UZ), static_cast<double>(samplesPerSymbol), 1.0, rolloff, 1.0);

    std::vector<float> impulses(symbols.size() * samplesPerSymbol, 0.f);
    for (std::size_t k = 0UZ; k < symbols.size(); ++k) {
        impulses[k * samplesPerSymbol] = symbols[k] == 1 ? 1.f : -1.f;
    }

    std::vector<std::complex<float>> out(impulses.size());
    for (std::size_t n = 0UZ; n < impulses.size(); ++n) {
        double sum = 0.0;
        for (std::size_t k = 0UZ; k < shape.size() && k <= n; ++k) {
            sum += static_cast<double>(shape[k]) * static_cast<double>(impulses[n - k]);
        }
        const double angle = 2.0 * std::numbers::pi * offsetCyclesPerSample * static_cast<double>(n) + phase0;
        out[n]             = std::complex<float>(static_cast<float>(sum * std::cos(angle)), static_cast<float>(sum * std::sin(angle)));
    }
    return out;
}

} // namespace

const boost::ut::suite<"recipes"> RecipeTests = [] {
    using namespace boost::ut;

    "every indexed recipe loads; bare instantiation succeeds or refuses only for required parameters"_test = [] {
        auto        loader = makeRecipeLoader();
        const auto& defs   = loader.definitionForBlockName();
        expect(!defs.empty()) << "the index listed no recipes, or was not read";
        for (const auto& [name, definition] : defs) {
            const auto bare = gr::detail::instantiateBlockFromYamlDefinition(loader, definition, {});
            if (!bare.has_value()) {
                expect(bare.error().message.contains("recipe_parameter_required")) << name << ": " << bare.error().message;
            }
        }
    };

    "NbfmDemod demands its rate and deviation, derives, and re-derives live"_test = [] {
        auto loader = makeRecipeLoader();

        auto bare = loader.instantiate("gr::recipes::NbfmDemod");
        expect(bare == nullptr) << "the general recipe must not instantiate with hidden defaults";

        gr::property_map parameters;
        parameters["sample_rate"] = 96000.0f;
        parameters["deviation"]   = 5000.0f;
        auto composite            = loader.instantiate("gr::recipes::NbfmDemod", parameters);
        expect(composite != nullptr) << "instantiate with the required parameters";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr);
        if (discriminator == nullptr) {
            return;
        }
        const auto gain = discriminator->settings().get("gain");
        expect(gain.has_value());
        if (gain.has_value()) {
            expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(96000.0 / (2.0 * std::numbers::pi * 5000.0)))) << "the derivation reached the discriminator";
        }
        const auto deemphasis = interiorByName(composite, "deemphasis");
        expect(deemphasis != nullptr);
        if (deemphasis != nullptr) {
            const auto tau = deemphasis->settings().get("tau");
            expect(tau.has_value());
            if (tau.has_value()) {
                expect(eq(numericOf(*tau), 7.5e-05)) << "the defaulted parameter forwarded";
            }
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr) << "the composite carries the live binding machinery";
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["deviation"] = 2500.0f;
        const auto applied  = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        const auto staged = discriminator->settings().stagedParameters();
        const auto gainIt = staged.find("gain");
        expect(gainIt != staged.end()) << "the re-derived gain is staged";
        if (gainIt != staged.end()) {
            expect(eq(static_cast<float>(numericOf(gainIt->second)), static_cast<float>(96000.0 / (2.0 * std::numbers::pi * 2500.0)))) << "halving the deviation doubles the gain, live";
        }
    };

    "FskDemodAudio demands the rate and the symbol rate, names both when it refuses, and derives its three blocks"_test = [] {
        auto loader = makeRecipeLoader();

        expect(loader.instantiate("gr::recipes::FskDemodAudio") == nullptr) << "the general recipe must not instantiate with hidden defaults";

        // the generic leg above only requires that a refusal carry the family's key; this one requires that THIS
        // recipe refuse, and that the message name both parameters a link cannot be guessed at without
        bool        indexed = false;
        const auto& defs    = loader.definitionForBlockName();
        for (const auto& [name, definition] : defs) {
            if (name != "gr::recipes::FskDemodAudio") {
                continue;
            }
            indexed         = true;
            const auto bare = gr::detail::instantiateBlockFromYamlDefinition(loader, definition, {});
            expect(!bare.has_value()) << "a bare instantiation is refused";
            if (!bare.has_value()) {
                const std::string message = bare.error().message;
                expect(message.contains("recipe_parameter_required")) << message;
                expect(message.contains("sample_rate")) << message;
                expect(message.contains("symbol_rate")) << message;
            }
        }
        expect(indexed) << "the recipe is in index.yaml";

        gr::property_map parameters;
        parameters["sample_rate"] = 48000.0f;
        parameters["symbol_rate"] = 9600.0f;
        auto composite            = loader.instantiate("gr::recipes::FskDemodAudio", parameters);
        expect(composite != nullptr) << "instantiate with the required parameters";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        // the two derivations the recipe exists to save, at the 9600-baud packet radio the arm is written around
        const auto lowpass = interiorByName(composite, "lowpass");
        expect(lowpass != nullptr);
        if (lowpass != nullptr) {
            const auto cutoff = lowpass->settings().get("cutoff");
            expect(cutoff.has_value());
            if (cutoff.has_value()) {
                expect(eq(numericOf(*cutoff), 4800.0)) << "the post-detection cutoff is half the symbol rate";
            }
        }
        const auto timing = interiorByName(composite, "timing");
        expect(timing != nullptr);
        if (timing != nullptr) {
            const auto sps = timing->settings().get("samples_per_symbol");
            expect(sps.has_value());
            if (sps.has_value()) {
                expect(eq(numericOf(*sps), 5.0)) << "9600 baud out of a 48 kHz recording is exactly five samples a symbol";
            }
        }
    };

    "SampleClockOffset turns parts per million into a resampling rate, and re-derives live"_test = [] {
        auto loader = makeRecipeLoader();

        // a clock error has a meaningful zero, so unlike the general demod this one defaults and instantiates
        auto nominal = loader.instantiate("gr::recipes::SampleClockOffset");
        expect(nominal != nullptr) << "a defaulted clock offset is a valid, and inert, channel";

        gr::property_map parameters;
        parameters["ppm"] = 20.0;
        auto composite    = loader.instantiate("gr::recipes::SampleClockOffset", parameters);
        expect(composite != nullptr);
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto clock = interiorByName(composite, "clock");
        expect(clock != nullptr);
        if (clock == nullptr) {
            return;
        }
        const auto rate = clock->settings().get("rate");
        expect(rate.has_value());
        if (rate.has_value()) {
            expect(eq(numericOf(*rate), 1.0 + 20.0e-6)) << "20 ppm fast is 1 + 20e-6 output samples per input";
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr);
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["ppm"]      = -50.0;
        const auto applied = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        const auto staged = clock->settings().stagedParameters();
        const auto rateIt = staged.find("rate");
        expect(rateIt != staged.end()) << "the re-derived rate is staged";
        if (rateIt != staged.end()) {
            expect(eq(numericOf(rateIt->second), 1.0 - 50.0e-6)) << "a slow clock resamples below unity";
        }
    };

    "FskDemod demands the link's own numbers, derives the discriminator gain, and re-derives live"_test = [] {
        auto loader = makeRecipeLoader();

        auto bare = loader.instantiate("gr::recipes::FskDemod");
        expect(bare == nullptr) << "the general recipe must not instantiate with hidden defaults";

        constexpr double kSampleRate      = 48000.0;
        constexpr double kSymbolRate      = 4800.0;
        constexpr double kModulationIndex = 0.5;

        gr::property_map parameters;
        parameters["sample_rate"]      = static_cast<float>(kSampleRate);
        parameters["symbol_rate"]      = static_cast<float>(kSymbolRate);
        parameters["modulation_index"] = kModulationIndex;
        auto composite                 = loader.instantiate("gr::recipes::FskDemod", parameters);
        expect(composite != nullptr) << "instantiate with the required parameters";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr);
        if (discriminator != nullptr) {
            const auto gain = discriminator->settings().get("gain");
            expect(gain.has_value());
            if (gain.has_value()) {
                expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(kSampleRate / (kSymbolRate * std::numbers::pi * kModulationIndex)))) << "the gain hands the transmitted PAM grid back";
            }
        }

        const auto timing = interiorByName(composite, "timing");
        expect(timing != nullptr);
        if (timing != nullptr) {
            const auto sps = timing->settings().get("samples_per_symbol");
            expect(sps.has_value());
            if (sps.has_value()) {
                expect(eq(numericOf(*sps), kSampleRate / kSymbolRate)) << "timing recovery runs at the rate the link states";
            }
            const auto detector = timing->settings().get("detector");
            expect(detector.has_value());
            if (detector.has_value()) {
                expect(eq(stringOf(*detector), std::string("mueller_muller"))) << "a string parameter left at its default is substituted like any other";
            }
        }

        const auto channel = interiorByName(composite, "channel");
        expect(channel != nullptr);
        if (channel != nullptr) {
            const auto cutoff = channel->settings().get("cutoff");
            expect(cutoff.has_value());
            if (cutoff.has_value()) {
                expect(eq(numericOf(*cutoff), 0.6 * kSymbolRate)) << "the filter ahead of the discriminator is what keeps it above the click threshold";
            }
        }

        const auto lowpass = interiorByName(composite, "lowpass");
        expect(lowpass != nullptr);
        if (lowpass != nullptr) {
            const auto cutoff = lowpass->settings().get("cutoff");
            expect(cutoff.has_value());
            if (cutoff.has_value()) {
                expect(eq(numericOf(*cutoff), 0.5 * kSymbolRate)) << "the post-detection lowpass passes half a symbol rate";
            }
        }

        const auto slicer = interiorByName(composite, "slicer");
        expect(slicer != nullptr);
        if (slicer != nullptr) {
            const auto levels = slicer->settings().get("n_levels");
            expect(levels.has_value());
            if (levels.has_value()) {
                expect(eq(numericOf(*levels), 2.0)) << "two levels is the defaulted grid";
            }
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr);
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["modulation_index"] = 1.0;
        change["detector"]         = std::pmr::string("gardner");
        const auto applied         = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        if (discriminator != nullptr) {
            const auto staged = discriminator->settings().stagedParameters();
            const auto gainIt = staged.find("gain");
            expect(gainIt != staged.end()) << "the re-derived gain is staged";
            if (gainIt != staged.end()) {
                expect(eq(static_cast<float>(numericOf(gainIt->second)), static_cast<float>(kSampleRate / (kSymbolRate * std::numbers::pi * 1.0)))) << "doubling the index halves the gain, live";
            }
        }
        if (timing != nullptr) {
            const auto staged     = timing->settings().stagedParameters();
            const auto detectorIt = staged.find("detector");
            expect(detectorIt != staged.end()) << "the re-substituted detector is staged";
            if (detectorIt != staged.end()) {
                expect(eq(stringOf(detectorIt->second), std::string("gardner"))) << "a string parameter re-substitutes live, like a derived number";
            }
        }
    };

    "FskDemod takes the timing detector it is given, a string carried through by substitution"_test = [] {
        auto loader = makeRecipeLoader();

        gr::property_map parameters;
        parameters["sample_rate"]      = 48000.f;
        parameters["symbol_rate"]      = 4800.f;
        parameters["modulation_index"] = 0.5;
        parameters["detector"]         = std::pmr::string("zero_crossing");
        auto composite                 = loader.instantiate("gr::recipes::FskDemod", parameters);
        expect(composite != nullptr) << "a named detector must instantiate";
        if (composite == nullptr) {
            return;
        }

        const auto timing = interiorByName(composite, "timing");
        expect(timing != nullptr);
        if (timing != nullptr) {
            const auto detector = timing->settings().get("detector");
            expect(detector.has_value());
            if (detector.has_value()) {
                expect(eq(stringOf(*detector), std::string("zero_crossing"))) << "the interior block takes the spelling the caller gave";
            }
        }
    };

    "WbfmMonoDemod defaults the service's numbers and demands only the front end's rate"_test = [] {
        auto loader = makeRecipeLoader();

        auto bare = loader.instantiate("gr::recipes::WbfmMonoDemod");
        expect(bare == nullptr) << "the front end's rate is the one number a service recipe still cannot default";

        constexpr double kSampleRate  = 1920000.0;
        constexpr double kDecimation  = 8.0;
        constexpr double kChannelRate = kSampleRate / kDecimation;
        constexpr double kAudioRate   = 48000.0;

        gr::property_map parameters;
        parameters["sample_rate"]        = static_cast<float>(kSampleRate);
        parameters["channel_decimation"] = static_cast<std::uint32_t>(kDecimation);
        auto composite                   = loader.instantiate("gr::recipes::WbfmMonoDemod", parameters);
        expect(composite != nullptr) << "the rate and the decimation are all it needs";
        if (composite == nullptr) {
            return;
        }

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto tuner = interiorByName(composite, "tuner");
        expect(tuner != nullptr);
        if (tuner != nullptr) {
            const auto shift = tuner->settings().get("frequency_shift");
            expect(shift.has_value());
            if (shift.has_value()) {
                expect(eq(numericOf(*shift), 0.0)) << "the tuner is inert at the default offset";
            }
        }

        const auto channel = interiorByName(composite, "channel");
        expect(channel != nullptr);
        if (channel != nullptr) {
            const auto decimation = channel->settings().get("decimation");
            expect(decimation.has_value());
            if (decimation.has_value()) {
                expect(eq(numericOf(*decimation), kDecimation)) << "an integral expression reaches an integral setting whole";
            }
        }

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr);
        if (discriminator != nullptr) {
            const auto gain = discriminator->settings().get("gain");
            expect(gain.has_value());
            if (gain.has_value()) {
                expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(kChannelRate / (2.0 * std::numbers::pi * 75000.0)))) << "the gain is derived at the channel rate, not the front end's";
            }
        }

        const auto audio = interiorByName(composite, "audio");
        expect(audio != nullptr);
        if (audio != nullptr) {
            const auto rate = audio->settings().get("rate");
            expect(rate.has_value());
            if (rate.has_value()) {
                expect(eq(numericOf(*rate), kAudioRate * kDecimation / kSampleRate)) << "the audio ratio follows all three rates";
            }
        }

        const auto deemphasis = interiorByName(composite, "deemphasis");
        expect(deemphasis != nullptr);
        if (deemphasis != nullptr) {
            const auto rate = deemphasis->settings().get("sample_rate");
            const auto tau  = deemphasis->settings().get("tau");
            expect(rate.has_value() && tau.has_value());
            if (rate.has_value() && tau.has_value()) {
                expect(eq(numericOf(*rate), kAudioRate)) << "de-emphasis runs at the audio rate, last in the chain";
                expect(eq(numericOf(*tau), 7.5e-05)) << "75 us is the service's own figure and is defaulted here";
            }
        }

        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr);
        if (wrapper == nullptr) {
            return;
        }
        gr::property_map change;
        change["offset_hz"]  = 250000.0;
        change["audio_rate"] = 32000.0f;
        const auto applied   = wrapper->applyRecipeParameters(change);
        expect(applied.has_value()) << (applied.has_value() ? "" : applied.error().message);
        if (tuner != nullptr) {
            const auto staged  = tuner->settings().stagedParameters();
            const auto shiftIt = staged.find("frequency_shift");
            expect(shiftIt != staged.end()) << "the re-derived shift is staged";
            if (shiftIt != staged.end()) {
                expect(eq(numericOf(shiftIt->second), -250000.0)) << "a station above the center is brought down to it";
            }
        }
        if (audio != nullptr && deemphasis != nullptr) {
            const auto stagedAudio = audio->settings().stagedParameters();
            const auto rateIt      = stagedAudio.find("rate");
            expect(rateIt != stagedAudio.end());
            if (rateIt != stagedAudio.end()) {
                expect(eq(numericOf(rateIt->second), 32000.0 * kDecimation / kSampleRate)) << "a new audio rate moves the resampler, live";
            }
            const auto stagedDeemphasis = deemphasis->settings().stagedParameters();
            const auto deemphasisIt     = stagedDeemphasis.find("sample_rate");
            expect(deemphasisIt != stagedDeemphasis.end());
            if (deemphasisIt != stagedDeemphasis.end()) {
                expect(eq(numericOf(deemphasisIt->second), 32000.0)) << "and retunes the de-emphasis with it";
            }
        }
    };

    "WbfmMonoDemod hands back the modulating tone at the deviation ratio"_test = [] {
        // A recipe's numbers only mean something if the chain they configure demodulates. The signal is one
        // audio tone at a stated peak deviation; the recipe's `deviation` is the reference full deviation, so
        // the audio the chain hands back must be that tone at exactly the ratio of the two, and nothing else.
        using CF = std::complex<float>;
        using gr::blocks::testing::ProcessFunction;

        constexpr double      kSampleRate = 240000.0; // already a channel, so channel_decimation stays at 1
        constexpr double      kAudioRate  = 48000.0;
        constexpr double      kTone       = 1000.0;
        constexpr double      kDeviation  = 15000.0; // what the signal actually swings
        constexpr double      kReference  = 75000.0; // what the recipe scales against
        constexpr std::size_t kInput      = 60000UZ; // 250 ms at the channel rate

        gr::Tensor<CF> values;
        values.reserve(kInput);
        double phase = 0.0;
        for (std::size_t n = 0UZ; n < kInput; ++n) {
            values.push_back(CF(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase))));
            phase += 2.0 * std::numbers::pi * (kDeviation / kSampleRate) * std::sin(2.0 * std::numbers::pi * kTone * static_cast<double>(n) / kSampleRate);
        }

        gr::Graph  graph;
        auto&      source = graph.emplaceBlock<gr::blocks::testing::TagSource<CF, ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(kInput)}, {"values", values}, {"mark_tag", false}});
        const auto demod  = gr::recipes::WbfmMonoDemod::emplace(graph, [] {
            gr::recipes::WbfmMonoDemod::Parameters parameters(static_cast<float>(kSampleRate));
            parameters.tau = 0.0; // bypassed, so the measured amplitude is the discriminator's alone
            return parameters;
        }());
        expect(demod != nullptr) << boost::ut::fatal;
        auto& sink = graph.emplaceBlock<gr::blocks::testing::TagSink<float, ProcessFunction::USE_PROCESS_BULK>>({{"name", "audio"}});

        const auto sourceModel = gr::graph::findBlock(graph, source);
        const auto sinkModel   = gr::graph::findBlock(graph, sink);
        expect(sourceModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
        expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, demod, gr::PortDefinition{"in"}).has_value());
        expect(graph.connect(demod, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

        gr::scheduler::Simple scheduler;
        expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
        const auto finished = scheduler.runAndWait();
        expect(finished.has_value()) << (finished.has_value() ? std::string{} : finished.error().message);

        // the resampler's filter is not compensated, so the first samples are its ramp-up rather than the tone
        constexpr std::size_t kSkip   = 2000UZ;
        constexpr std::size_t kWindow = 9600UZ; // 200 cycles of the tone at the audio rate
        expect(ge(sink._samples.size(), kSkip + kWindow)) << std::format("audio samples produced: {}", sink._samples.size()) << boost::ut::fatal;

        const auto reading = readTone(std::span<const float>(sink._samples.data() + kSkip, kWindow), kTone, kAudioRate);
        const auto ratio   = kDeviation / kReference;
        expect(lt(std::abs(reading.amplitude - ratio), 0.001 * ratio)) << std::format("tone amplitude {:.5f}, expected {:.5f}", reading.amplitude, ratio);
        expect(gt(reading.powerShare, 0.9999)) << std::format("the tone accounts for {:.4f} of the audio's power", reading.powerShare);
    };

    "AfskDemod demands its tone pair, and the deviation's sign is the polarity rule"_test = [] {
        auto loader = makeRecipeLoader();
        expect(loader.instantiate("gr::recipes::AfskDemod") == nullptr) << "the tone pair is an interoperability fact and has no default";

        // Bell 202 at 48 kHz with decimation 5: eight samples per symbol into the timing loop
        constexpr double kSampleRate = 48000.0;
        constexpr double kSymbolRate = 1200.0;
        constexpr double kMark       = 1200.0;
        constexpr double kSpace      = 2200.0;
        constexpr double kDecimation = 5.0;
        constexpr double kHilbert    = 127.0;

        gr::property_map parameters;
        parameters["sample_rate"] = static_cast<float>(kSampleRate);
        parameters["symbol_rate"] = static_cast<float>(kSymbolRate);
        parameters["mark_hz"]     = kMark;
        parameters["space_hz"]    = kSpace;
        parameters["decimation"]  = static_cast<gr::Size_t>(kDecimation);
        auto composite            = loader.instantiate("gr::recipes::AfskDemod", parameters);
        expect(composite != nullptr) << boost::ut::fatal;

        const auto inputs  = exportedNames(composite->exportedInputPorts());
        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ));
        expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end());

        const auto reads = [&composite](std::string_view block, const char* key) {
            const auto interior = interiorByName(composite, block);
            boost::ut::expect(interior != nullptr) << block << boost::ut::fatal;
            const auto value = interior->settings().get(key);
            boost::ut::expect(value.has_value()) << block << "." << key << boost::ut::fatal;
            return numericOf(*value);
        };

        const double expectedGain = kSampleRate / (kDecimation * std::numbers::pi * (kMark - kSpace));
        expect(lt(std::abs(expectedGain + 3.055775), 1e-5)) << std::format("the worked figure is -3.055775, computed as {:.6f}", expectedGain);
        expect(eq(static_cast<float>(reads("discriminator", "gain")), static_cast<float>(expectedGain))) << "a mark below a space is a NEGATIVE deviation and a negative gain";
        expect(eq(static_cast<float>(reads("translate", "frequency_shift")), -1700.f)) << "the band center is (1200 + 2200)/2";
        expect(eq(reads("timing", "samples_per_symbol"), 8.0)) << "48000 / (5 * 1200)";
        expect(eq(reads("delay", "delay"), (kHilbert - 1.0) / 2.0)) << "the real branch takes the Hilbert design's own group delay, as an integer";
        expect(eq(reads("hilbert", "designed_taps"), kHilbert)) << "and the design is the length the parameter states";
        expect(eq(reads("channel", "cutoff"), 0.9167 * kSymbolRate)) << "half of Carson's 2*(500 + 600) = 2200 Hz";
        expect(eq(reads("channel", "decimation"), kDecimation));

        // move the space below the mark and the polarity has to follow, with no other setting touched
        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr) << boost::ut::fatal;
        gr::property_map change;
        change["space_hz"] = 200.0;
        expect(wrapper->applyRecipeParameters(change).has_value());

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr) << boost::ut::fatal;
        const auto staged = discriminator->settings().stagedParameters();
        const auto gainIt = staged.find("gain");
        expect(gainIt != staged.end()) << "the re-derived gain is staged";
        if (gainIt != staged.end()) {
            const double flipped = kSampleRate / (kDecimation * std::numbers::pi * (kMark - 200.0));
            expect(eq(static_cast<float>(numericOf(gainIt->second)), static_cast<float>(flipped))) << std::format("the sign flips to {:+.6f} because the mark is now the higher tone", flipped);
            expect(gt(flipped, 0.0));
        }
    };

    "BpskFrontEnd and its two consumers, one recipe naming another"_test = [] {
        auto loader = makeRecipeLoader();
        for (const char* name : {"gr::recipes::BpskFrontEnd", "gr::recipes::BpskDemod", "gr::recipes::DbpskDemod"}) {
            expect(loader.instantiate(name) == nullptr) << name << ": a general recipe demands the link's rates";
        }

        constexpr double kSampleRate = 200000.0;
        constexpr double kSymbolRate = 12500.0;
        constexpr double kDecimation = 4.0;
        constexpr double kOffset     = 3000.0;

        gr::property_map parameters;
        parameters["sample_rate"]      = static_cast<float>(kSampleRate);
        parameters["symbol_rate"]      = static_cast<float>(kSymbolRate);
        parameters["decimation"]       = static_cast<gr::Size_t>(kDecimation);
        parameters["frequency_offset"] = kOffset;

        for (const char* name : {"gr::recipes::BpskFrontEnd", "gr::recipes::BpskDemod", "gr::recipes::DbpskDemod"}) {
            auto composite = loader.instantiate(name, parameters);
            expect(composite != nullptr) << name << boost::ut::fatal;

            const auto inputs  = exportedNames(composite->exportedInputPorts());
            const auto outputs = exportedNames(composite->exportedOutputPorts());
            expect(eq(inputs.size(), 1UZ) && eq(outputs.size(), 1UZ)) << name;
            expect(std::ranges::find(inputs, "in") != inputs.end() && std::ranges::find(outputs, "out") != outputs.end()) << name;

            // BpskDemod and DbpskDemod write BpskFrontEnd's five stages out rather than naming it: the graph importer
            // creates an interior block from its id alone and never hands it the block's parameters, so a nested
            // recipe with a required parameter refuses. The three must therefore derive the same five settings, and
            // this is the assertion that says the copies have not drifted.
            const auto reads = [&composite, name](std::string_view block, const char* key) {
                const auto interior = interiorByName(composite, block);
                boost::ut::expect(interior != nullptr) << name << ": " << block << boost::ut::fatal;
                const auto value = interior->settings().get(key);
                boost::ut::expect(value.has_value()) << name << ": " << block << "." << key << boost::ut::fatal;
                return numericOf(*value);
            };

            expect(eq(static_cast<float>(reads("translate", "frequency_shift")), static_cast<float>(-kOffset))) << name;
            expect(eq(reads("channel", "cutoff"), 0.75 * kSymbolRate)) << name;
            expect(eq(reads("channel", "decimation"), kDecimation)) << name;
            expect(eq(static_cast<float>(reads("agc", "sample_rate")), static_cast<float>(kSampleRate / kDecimation))) << name << ": the AGC's time constants are measured against the rate it runs at";
            expect(eq(reads("agc", "reference_db"), 0.0)) << name << ": the loops downstream state their gains at unit amplitude";
            expect(eq(reads("fll", "samples_per_symbol"), kSampleRate / (kDecimation * kSymbolRate))) << name;
            expect(eq(reads("timing", "samples_per_symbol"), kSampleRate / (kDecimation * kSymbolRate))) << name;
        }

        // the two detectors are the whole of the difference
        auto coherent = loader.instantiate("gr::recipes::BpskDemod", parameters);
        expect(coherent != nullptr) << boost::ut::fatal;
        const auto costas = interiorByName(coherent, "costas");
        expect(costas != nullptr) << boost::ut::fatal;
        const auto order = costas->settings().get("order");
        const auto kdet  = costas->settings().get("detector_gain");
        expect(order.has_value() && numericOf(*order) == 2.0) << "a settable order would make this a PSK recipe";
        expect(kdet.has_value() && numericOf(*kdet) == 1.0) << "1.0 is the order-2 S-curve slope at the unit amplitude the AGC delivers";

        auto differential = loader.instantiate("gr::recipes::DbpskDemod", parameters);
        expect(differential != nullptr) << boost::ut::fatal;
        expect(interiorByName(differential, "phasor") != nullptr) << "and the differential arm has a phasor where the coherent one has a loop";
        expect(interiorByName(differential, "costas") == nullptr) << "and no carrier loop at all";

        // the limitation the two files are written around, pinned so that it is a finding and not a habit: a nested
        // recipe with a required parameter cannot be built, because the importer never forwards the parameters
        expect(loader.instantiate("gr::recipes::BpskFrontEnd") == nullptr) << "which is the refusal a nested instantiation would run into";
    };

    "FskDemodDcBlock is FskDemod's chain plus the blocker, and hands back soft symbols"_test = [] {
        auto loader = makeRecipeLoader();
        expect(loader.instantiate("gr::recipes::FskDemodDcBlock") == nullptr);

        constexpr double kSampleRate      = 48000.0;
        constexpr double kSymbolRate      = 4800.0;
        constexpr double kModulationIndex = 0.5;

        gr::property_map parameters;
        parameters["sample_rate"]      = static_cast<float>(kSampleRate);
        parameters["symbol_rate"]      = static_cast<float>(kSymbolRate);
        parameters["modulation_index"] = kModulationIndex;
        auto composite                 = loader.instantiate("gr::recipes::FskDemodDcBlock", parameters);
        expect(composite != nullptr) << boost::ut::fatal;

        const auto outputs = exportedNames(composite->exportedOutputPorts());
        expect(eq(outputs.size(), 1UZ) && std::ranges::find(outputs, "out") != outputs.end());
        expect(interiorByName(composite, "slicer") == nullptr) << "soft is the general form; a slicer downstream is one landed block and unslicing is impossible";

        const auto blocker = interiorByName(composite, "dc_block");
        expect(blocker != nullptr) << boost::ut::fatal;
        const auto length = blocker->settings().get("length");
        expect(length.has_value() && numericOf(*length) == 128.0) << "the notch corner is sample_rate/128, which at ten samples per symbol is well under the symbol rate";

        const auto discriminator = interiorByName(composite, "discriminator");
        expect(discriminator != nullptr) << boost::ut::fatal;
        const auto gain = discriminator->settings().get("gain");
        expect(gain.has_value());
        if (gain.has_value()) {
            expect(eq(static_cast<float>(numericOf(*gain)), static_cast<float>(kSampleRate / (kSymbolRate * std::numbers::pi * kModulationIndex)))) << "the gain is FskDemod's, unchanged";
        }

        // and the signed index inverts it, which is the polarity rule FskDemod's header now states
        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(composite.get());
        expect(wrapper != nullptr) << boost::ut::fatal;
        gr::property_map change;
        change["modulation_index"] = -kModulationIndex;
        expect(wrapper->applyRecipeParameters(change).has_value());
        const auto staged = discriminator->settings().stagedParameters();
        const auto gainIt = staged.find("gain");
        expect(gainIt != staged.end());
        if (gainIt != staged.end()) {
            expect(lt(numericOf(gainIt->second), 0.0)) << "a negative index means the higher tone is the zero, and the gain inverts with it";
        }
    };

    "AfskDemod recovers Bell 202 from real audio, the mark positive"_test = [] {
        constexpr double      kSampleRate = 48000.0;
        constexpr double      kSymbolRate = 1200.0;
        constexpr double      kMark       = 1200.0;
        constexpr double      kSpace      = 2200.0;
        constexpr std::size_t kBits       = 400UZ;

        const std::vector<int>   bits  = sourceBits(kBits);
        const std::vector<float> audio = afskAudio(std::span<const int>(bits), kSampleRate, kSymbolRate, kMark, kSpace);

        gr::property_map parameters;
        parameters["sample_rate"] = static_cast<float>(kSampleRate);
        parameters["symbol_rate"] = static_cast<float>(kSymbolRate);
        parameters["mark_hz"]     = kMark;
        parameters["space_hz"]    = kSpace;
        parameters["decimation"]  = static_cast<gr::Size_t>(5);

        const std::vector<float> soft = runRecipe<float, float>("gr::recipes::AfskDemod", parameters, audio);
        expect(ge(soft.size(), kBits - 40UZ)) << std::format("one soft symbol per symbol: {} out of {}", soft.size(), kBits) << boost::ut::fatal;

        // the mark carries a one and is the LOWER tone, so the polarity is right only if the deviation's sign reached
        // the discriminator: an uninverted match is the assertion, and the inverted one is what a wrong sign gives
        const Agreement upright  = bestAgreement(std::span<const float>(soft), std::span<const int>(bits), 8UZ, 40UZ, false);
        const Agreement anyPhase = bestAgreement(std::span<const float>(soft), std::span<const int>(bits), 8UZ, 40UZ, true);
        expect(ge(upright.fraction, 0.99)) << std::format("{:.4f} of symbols recovered at lag {}", upright.fraction, upright.lag);
        expect(that % (!anyPhase.inverted)) << "the best alignment is the uninverted one, which is the deviation-sign polarity rule end to end";
    };

    "BpskFrontEnd levels its input and takes the residual carrier frequency out"_test = [] {
        constexpr double      kSampleRate = 200000.0;
        constexpr double      kSymbolRate = 12500.0;
        constexpr std::size_t kDecimation = 4UZ;
        constexpr std::size_t kSymbols    = 3000UZ;
        constexpr double      kOffsetHz   = 400.0;
        constexpr double      kAmplitude  = 0.05; // well away from unity, so the AGC has something to do

        const std::vector<int>           symbols = sourceBits(kSymbols);
        std::vector<std::complex<float>> wave    = bpskBaseband(std::span<const int>(symbols), static_cast<std::size_t>(kSampleRate / kSymbolRate), 0.35, kOffsetHz / kSampleRate, 0.7);
        for (std::complex<float>& sample : wave) {
            sample *= static_cast<float>(kAmplitude);
        }

        gr::property_map parameters;
        parameters["sample_rate"] = static_cast<float>(kSampleRate);
        parameters["symbol_rate"] = static_cast<float>(kSymbolRate);
        parameters["decimation"]  = static_cast<gr::Size_t>(kDecimation);

        const std::vector<std::complex<float>> out = runRecipe<std::complex<float>, std::complex<float>>("gr::recipes::BpskFrontEnd", parameters, wave);
        expect(ge(out.size(), kSymbols / 2UZ)) << std::format("one complex sample per symbol: {}", out.size()) << boost::ut::fatal;

        // squaring removes the BPSK modulation, so the per-symbol phase advance of z^2 is twice the residual carrier
        const std::size_t settled = out.size() / 2UZ;
        double            level   = 0.0;
        double            advance = 0.0;
        std::size_t       counted = 0UZ;
        for (std::size_t k = settled + 1UZ; k < out.size(); ++k) {
            level += static_cast<double>(std::abs(out[k]));
            const std::complex<double> now{out[k]};
            const std::complex<double> before{out[k - 1UZ]};
            advance += std::arg(now * now * std::conj(before * before));
            ++counted;
        }
        level /= static_cast<double>(counted);
        const double residualHz = (advance / static_cast<double>(counted)) * 0.5 * kSymbolRate / (2.0 * std::numbers::pi);

        // the AGC levels the SAMPLE stream's mean magnitude to one; what is read here is the magnitude at the symbol
        // instants after the matched filter, which sits above that mean for a shaped signal. 1.216 is the measured
        // figure and the bound is around it, not around one.
        expect(that % (std::abs(level - 1.0) < 0.3)) << std::format("the AGC delivers unit amplitude for the loops downstream: measured {:.4f}", level);
        expect(that % (std::abs(residualHz) < 0.1 * kOffsetHz)) << std::format("the frequency-locked loop leaves {:.2f} Hz of a {:.0f} Hz offset", residualHz, kOffsetHz);
    };

    "BpskDemod decides on the axis the Costas loop finds"_test = [] {
        constexpr double      kSampleRate = 200000.0;
        constexpr double      kSymbolRate = 12500.0;
        constexpr std::size_t kDecimation = 4UZ;
        constexpr std::size_t kSymbols    = 3000UZ;

        const std::vector<int>                 bits = sourceBits(kSymbols);
        const std::vector<std::complex<float>> wave = bpskBaseband(std::span<const int>(bits), static_cast<std::size_t>(kSampleRate / kSymbolRate), 0.35, 200.0 / kSampleRate, 1.1);

        gr::property_map parameters;
        parameters["sample_rate"] = static_cast<float>(kSampleRate);
        parameters["symbol_rate"] = static_cast<float>(kSymbolRate);
        parameters["decimation"]  = static_cast<gr::Size_t>(kDecimation);

        const std::vector<float> soft = runRecipe<std::complex<float>, float>("gr::recipes::BpskDemod", parameters, wave);
        expect(ge(soft.size(), kSymbols / 2UZ)) << std::format("soft symbols: {}", soft.size()) << boost::ut::fatal;

        // an order-2 Costas loop has a 180-degree ambiguity, so the recovered stream may be the transmitted one
        // inverted; resolving that is a framing question and not this recipe's
        const std::size_t skip  = soft.size() / 4UZ;
        const Agreement   found = bestAgreement(std::span<const float>(soft).subspan(skip), std::span<const int>(bits).subspan(skip), 0UZ, 40UZ, true);
        expect(ge(found.fraction, 0.99)) << std::format("{:.4f} of symbols recovered at lag {}, {}", found.fraction, found.lag, found.inverted ? "inverted" : "upright");
    };

    "DbpskDemod needs no phase at all, which is the whole of the differential arm"_test = [] {
        constexpr double      kSampleRate = 200000.0;
        constexpr double      kSymbolRate = 12500.0;
        constexpr std::size_t kDecimation = 4UZ;
        constexpr std::size_t kSymbols    = 1500UZ;

        // differentially encoded: the transmitted symbol is the running product, so the receiver's decision on
        // adjacent pairs hands back the source bits without ever learning the carrier phase
        const std::vector<int> bits = sourceBits(kSymbols);
        std::vector<int>       encoded(bits.size());
        int                    running = 1;
        for (std::size_t k = 0UZ; k < bits.size(); ++k) {
            running    = bits[k] == 1 ? running : -running;
            encoded[k] = running > 0 ? 1 : 0;
        }

        gr::property_map parameters;
        parameters["sample_rate"] = static_cast<float>(kSampleRate);
        parameters["symbol_rate"] = static_cast<float>(kSymbolRate);
        parameters["decimation"]  = static_cast<gr::Size_t>(kDecimation);

        std::vector<double> fractions;
        for (const double phase0 : {0.0, std::numbers::pi / 2.0, std::numbers::pi, 3.0 * std::numbers::pi / 4.0}) {
            const std::vector<std::complex<float>> wave = bpskBaseband(std::span<const int>(encoded), static_cast<std::size_t>(kSampleRate / kSymbolRate), 0.35, 0.0, phase0);
            const std::vector<float>               soft = runRecipe<std::complex<float>, float>("gr::recipes::DbpskDemod", parameters, wave);
            expect(ge(soft.size(), kSymbols / 2UZ)) << std::format("phase {:.3f}: soft symbols {}", phase0, soft.size()) << boost::ut::fatal;

            const std::size_t skip  = soft.size() / 4UZ;
            const Agreement   found = bestAgreement(std::span<const float>(soft).subspan(skip), std::span<const int>(bits).subspan(skip), 0UZ, 40UZ, false);
            expect(ge(found.fraction, 0.99)) << std::format("phase {:.3f}: {:.4f} recovered at lag {}", phase0, found.fraction, found.lag);
            fractions.push_back(found.fraction);
        }
        const auto [worst, best] = std::ranges::minmax(fractions);
        expect(that % (best - worst < 0.01)) << std::format("a carrier phase rotation costs the differential arm nothing: {:.4f} to {:.4f} across four rotations", worst, best);
    };

    "FskDemodDcBlock removes the offset FskDemod has no answer to"_test = [] {
        // A discriminator turns a frequency offset into a DC offset, and past half a level every symbol decides
        // wrong. The strict comparison is the only honest justification for an extra block.
        constexpr double      kSampleRate = 96000.0;
        constexpr double      kSymbolRate = 9600.0;
        constexpr double      kIndex      = 0.5;
        constexpr std::size_t kBits       = 600UZ;
        const double          kDeviation  = kIndex * kSymbolRate / 2.0; // 2400 Hz
        const double          kOffset     = 0.45 * kDeviation;          // just under half a level

        const std::vector<int>           bits      = sourceBits(kBits);
        const auto                       perSymbol = static_cast<std::size_t>(kSampleRate / kSymbolRate);
        std::vector<std::complex<float>> wave;
        wave.reserve(bits.size() * perSymbol);
        double phase = 0.0;
        for (const int bit : bits) {
            const double tone = (bit == 1 ? kDeviation : -kDeviation) + kOffset;
            for (std::size_t n = 0UZ; n < perSymbol; ++n) {
                wave.emplace_back(static_cast<float>(std::cos(phase)), static_cast<float>(std::sin(phase)));
                phase += 2.0 * std::numbers::pi * tone / kSampleRate;
            }
        }

        gr::property_map parameters;
        parameters["sample_rate"]      = static_cast<float>(kSampleRate);
        parameters["symbol_rate"]      = static_cast<float>(kSymbolRate);
        parameters["modulation_index"] = kIndex;

        const std::vector<std::uint8_t> plain = runRecipe<std::complex<float>, std::uint8_t>("gr::recipes::FskDemod", parameters, wave);
        const std::vector<float>        fixed = runRecipe<std::complex<float>, float>("gr::recipes::FskDemodDcBlock", parameters, wave);
        expect(ge(plain.size(), kBits / 2UZ) && ge(fixed.size(), kBits / 2UZ)) << boost::ut::fatal;

        std::vector<float> plainSoft(plain.size());
        for (std::size_t k = 0UZ; k < plain.size(); ++k) {
            plainSoft[k] = plain[k] != 0U ? 1.f : -1.f;
        }

        const std::size_t skip      = 40UZ;
        const Agreement   withoutDc = bestAgreement(std::span<const float>(plainSoft), std::span<const int>(bits), skip, 40UZ, true);
        const Agreement   withDc    = bestAgreement(std::span<const float>(fixed), std::span<const int>(bits), skip, 40UZ, true);
        expect(ge(withDc.fraction, 0.99)) << std::format("with the blocker: {:.4f} recovered", withDc.fraction);
        expect(gt(withDc.fraction, withoutDc.fraction + 0.05)) << std::format("without it: {:.4f}; the offset is {:.0f} Hz against a {:.0f} Hz deviation", withoutDc.fraction, kOffset, kDeviation);
    };

    "CcsdsRsFrames recovers the transfer frame, both bases, interleaved and not"_test = [] {
        using Record = gr::DataSet<std::uint8_t>;

        struct Arm {
            const char*   basis;
            std::uint32_t e;
            std::uint32_t depth;
            std::uint32_t frameLength;
            const char*   codeName;
        };
        constexpr Arm kArms[] = {
            {"conventional", 16U, 1U, 223U, "ccsds_255_223"},
            {"dual", 16U, 5U, 1115U, "ccsds_255_223"},
            {"dual", 8U, 1U, 239U, "ccsds_255_239"},
        };

        for (const Arm& arm : kArms) {
            const std::size_t info = 255UZ - 2UZ * arm.e;
            const std::size_t wire = 255UZ;
            expect(eq(static_cast<std::size_t>(arm.frameLength), info * arm.depth)) << "the arm's frame is whole codewords at pad 0";

            std::uint64_t lcg  = 0x9E3779B97F4A7C15ULL + arm.depth;
            const auto    draw = [&lcg] {
                lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
                return static_cast<std::uint8_t>(lcg >> 56U);
            };
            std::vector<std::uint8_t> frame(arm.frameLength);
            for (std::uint8_t& b : frame) {
                b = draw();
            }

            // the transmit side, from the tree's own kernels: the frame's octets are wire-domain strings, so a
            // dual link first maps them to field elements, encodes, and maps the whole codeblock back out
            const bool                dual = std::string_view(arm.basis) == "dual";
            std::vector<std::uint8_t> infoWords(frame);
            if (dual) {
                for (std::uint8_t& b : infoWords) {
                    b = gr::fec::CcsdsDualBasis::fromDual(b);
                }
            }
            std::vector<std::uint8_t> plainWords(infoWords.size());
            gr::fec::deinterleaveCodewords(infoWords, plainWords, info, arm.depth);

            std::vector<std::uint8_t> wireWords(wire * arm.depth);
            for (std::size_t w = 0UZ; w < arm.depth; ++w) {
                if (arm.e == 16U) {
                    gr::fec::ReedSolomonCcsds255_223::Block block{};
                    std::copy_n(plainWords.begin() + static_cast<std::ptrdiff_t>(w * info), info, block.begin());
                    gr::fec::ReedSolomonCcsds255_223::encode(block, 0UZ);
                    std::copy_n(block.begin(), wire, wireWords.begin() + static_cast<std::ptrdiff_t>(w * wire));
                } else {
                    gr::fec::ReedSolomonCcsds255_239::Block block{};
                    std::copy_n(plainWords.begin() + static_cast<std::ptrdiff_t>(w * info), info, block.begin());
                    gr::fec::ReedSolomonCcsds255_239::encode(block, 0UZ);
                    std::copy_n(block.begin(), wire, wireWords.begin() + static_cast<std::ptrdiff_t>(w * wire));
                }
            }
            std::vector<std::uint8_t> codeblock(wire * arm.depth);
            gr::fec::interleaveCodewords(wireWords, codeblock, wire, arm.depth);
            if (dual) {
                for (std::uint8_t& b : codeblock) {
                    b = gr::fec::CcsdsDualBasis::toDual(b);
                }
            }

            // the randomizer, byte for byte: XORing the sequence's bytes MSB first is XORing its bits in order
            gr::digital::ScramblerConfig randomizer{};
            gr::digital::configure(randomizer, gr::digital::standard::ccsds131, gr::digital::seedFromBitString("11111111", 8U), gr::digital::ScramblerMode::Additive, 8U, gr::digital::BitOrder::MsbFirst);
            std::vector<std::uint8_t> randomized(codeblock.size());
            gr::digital::scramble(randomizer, std::span<const std::uint8_t>(codeblock), std::span<std::uint8_t>(randomized));

            // the stream: garbage, the marker, the codeblock's bits, garbage — as soft values, with two of the
            // marker's bits flipped to spend the stated error budget
            constexpr std::string_view kAsm = "00011010110011111111110000011101";
            std::vector<float>         soft;
            for (std::size_t i = 0UZ; i < 41UZ; ++i) {
                soft.push_back((draw() & 1U) != 0U ? 0.9f : -0.9f);
            }
            const std::size_t asmAt = soft.size();
            for (const char bit : kAsm) {
                soft.push_back(bit == '1' ? 0.9f : -0.9f);
            }
            for (const std::uint8_t byte : randomized) {
                for (unsigned j = 8U; j-- > 0U;) {
                    soft.push_back(((byte >> j) & 1U) != 0U ? 0.9f : -0.9f);
                }
            }
            for (std::size_t i = 0UZ; i < 29UZ; ++i) {
                soft.push_back((draw() & 1U) != 0U ? 0.9f : -0.9f);
            }
            soft[asmAt + 3UZ] *= -1.0f;
            soft[asmAt + 20UZ] *= -1.0f;

            const gr::property_map parameters{{"frame_length", arm.frameLength}, {"error_capability", arm.e}, {"code", std::string(arm.codeName)}, //
                {"basis", std::string(arm.basis)}, {"sync_errors", gr::Size_t{2U}}, {"interleave", arm.depth}};

            auto loader    = makeRecipeLoader();
            auto composite = loader.instantiate("gr::recipes::CcsdsRsFrames", parameters);
            expect(composite != nullptr) << boost::ut::fatal;

            gr::Graph         graph;
            gr::Tensor<float> values(soft.begin(), soft.end());
            auto&             source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(soft.size())}, {"values", values}, {"mark_tag", false}});
            auto&             sink   = graph.emplaceBlock<CcsdsRecordSink>();
            const auto        chain  = graph.addBlock(std::move(composite));

            const auto sourceModel = gr::graph::findBlock(graph, source);
            const auto sinkModel   = gr::graph::findBlock(graph, sink);
            expect(sourceModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
            expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, chain, gr::PortDefinition{"in"}).has_value());
            expect(graph.connect(chain, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
            expect(scheduler.runAndWait().has_value());

            expect(eq(sink._records.size(), 1UZ)) << std::format("{} E={} I={}: one marker, one frame", arm.basis, arm.e, arm.depth);
            if (sink._records.size() != 1UZ) {
                continue;
            }
            const Record& seen = sink._records[0UZ];
            expect(std::ranges::equal(seen.signal_values, frame)) << std::format("{} E={} I={}: the transfer frame, byte for byte", arm.basis, arm.e, arm.depth);
            const auto& map       = seen.meta_information[0UZ];
            const auto  corrected = map.find(gr::property_map::key_type("corrected_errors"));
            expect(that % (corrected != map.end() && corrected->second.value_or(gr::Size_t{1U}) == gr::Size_t{0U})) << "a clean codeblock costs the decoder nothing";
            const auto uncorrectable = map.find(gr::property_map::key_type("uncorrectable_errors"));
            expect(that % (uncorrectable != map.end() && uncorrectable->second.value_or(gr::Size_t{1U}) == gr::Size_t{0U}));
        }
    };

    "CcsdsConcatenatedFrames self-aligns: either input offset decodes through one instance"_test = [] {
        using Record = gr::DataSet<std::uint8_t>;

        constexpr std::uint32_t kE           = 16U;
        constexpr std::uint32_t kDepth       = 1U;
        constexpr std::uint32_t kFrameLength = 223U;
        const std::size_t       wireBytes    = 255UZ;

        std::uint64_t lcg  = 0x2545F4914F6CDD1DULL;
        const auto    draw = [&lcg] {
            lcg = lcg * 6364136223846793005ULL + 1442695040888963407ULL;
            return static_cast<std::uint8_t>(lcg >> 56U);
        };

        std::vector<std::uint8_t> frame(kFrameLength);
        for (std::uint8_t& b : frame) {
            b = draw();
        }

        // outer code, conventional basis, then the randomizer, byte for byte
        gr::fec::ReedSolomonCcsds255_223::Block block{};
        std::copy_n(frame.begin(), 223UZ, block.begin());
        gr::fec::ReedSolomonCcsds255_223::encode(block, 0UZ);
        gr::digital::ScramblerConfig randomizer{};
        gr::digital::configure(randomizer, gr::digital::standard::ccsds131, gr::digital::seedFromBitString("11111111", 8U), gr::digital::ScramblerMode::Additive, 8U, gr::digital::BitOrder::MsbFirst);
        std::vector<std::uint8_t> randomized(wireBytes);
        gr::digital::scramble(randomizer, std::span<const std::uint8_t>(block.data(), wireBytes), std::span<std::uint8_t>(randomized));

        // inner code over lead-in, marker, codeblock and margin, one terminated stretch of bits
        std::vector<std::uint8_t> txBits;
        for (std::size_t i = 0UZ; i < 23UZ; ++i) {
            txBits.push_back(static_cast<std::uint8_t>(draw() & 1U));
        }
        const std::size_t asmBitAt = txBits.size();
        for (const char bit : gr::blocks::digital::syncword::ccsds_asm) {
            txBits.push_back(bit == '1' ? 1U : 0U);
        }
        for (const std::uint8_t byte : randomized) {
            for (unsigned j = 8U; j-- > 0U;) {
                txBits.push_back(static_cast<std::uint8_t>((byte >> j) & 1U));
            }
        }
        for (std::size_t i = 0UZ; i < 20UZ; ++i) {
            txBits.push_back(static_cast<std::uint8_t>(draw() & 1U));
        }

        gr::fec::ConvolutionalCode inner;
        expect(gr::fec::configureConvention(inner, "ccsds"));
        std::vector<std::uint8_t> coded(gr::fec::convolutionalEncodedBits(inner, txBits.size()));
        expect(eq(gr::fec::convolutionalEncode(inner, txBits, coded), coded.size()));

        std::vector<float> soft;
        soft.reserve(coded.size() + 60UZ);
        for (const std::uint8_t symbol : coded) {
            soft.push_back((symbol & 1U) != 0U ? 0.9f : -0.9f);
        }
        for (std::size_t i = 0UZ; i < 60UZ; ++i) {
            // the code-start correlator delays its output by the marker's own length, so the framer
            // finishes the record 52 symbols after the stream's payload does — the tail feeds that
            soft.push_back((draw() & 1U) != 0U ? 0.9f : -0.9f);
        }

        const std::string      marker = gr::blocks::digital::syncword::ccsdsEncodedAsm("ccsds");
        const gr::property_map base{{"frame_length", kFrameLength}, {"error_capability", kE}, {"code", std::string("ccsds_255_223")}, {"basis", std::string("conventional")}, //
            {"convolutional", std::string("ccsds")}, {"encoded_marker", marker}, {"sync_errors", gr::Size_t{2U}}, {"interleave", kDepth}};

        // the pairing is recovered from the marker's own position: a one-symbol shift of the whole
        // stream moves the tag by one and changes nothing downstream, so ONE instance decodes at
        // either offset — the ambiguity a parity-anchored chain would need two instances for
        std::ignore             = asmBitAt;
        std::size_t cleanFrames = 0UZ;
        for (const std::size_t offset : {0UZ, 1UZ}) {
            std::vector<float> stream;
            stream.reserve(soft.size() + offset);
            for (std::size_t i = 0UZ; i < offset; ++i) {
                stream.push_back(-0.9f);
            }
            stream.insert(stream.end(), soft.begin(), soft.end());

            auto loader    = makeRecipeLoader();
            auto composite = loader.instantiate("gr::recipes::CcsdsConcatenatedFrames", base);
            expect(composite != nullptr) << boost::ut::fatal;

            gr::Graph         graph;
            gr::Tensor<float> values(stream.begin(), stream.end());
            auto&             source = graph.emplaceBlock<gr::blocks::testing::TagSource<float, gr::blocks::testing::ProcessFunction::USE_PROCESS_BULK>>({{"n_samples_max", static_cast<gr::Size_t>(stream.size())}, {"values", values}, {"mark_tag", false}});
            auto&             sink   = graph.emplaceBlock<CcsdsRecordSink>();
            const auto        chain  = graph.addBlock(std::move(composite));

            const auto sourceModel = gr::graph::findBlock(graph, source);
            const auto sinkModel   = gr::graph::findBlock(graph, sink);
            expect(sourceModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
            expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, chain, gr::PortDefinition{"in"}).has_value());
            expect(graph.connect(chain, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
            expect(scheduler.runAndWait().has_value());

            expect(eq(sink._records.size(), 1UZ)) << std::format("offset {}: one marker, one frame", offset);
            for (const Record& seen : sink._records) {
                const auto& map           = seen.meta_information[0UZ];
                const auto  uncorrectable = map.find(gr::property_map::key_type("uncorrectable_errors"));
                const bool  clean         = uncorrectable != map.end() && uncorrectable->second.value_or(gr::Size_t{1U}) == gr::Size_t{0U} && std::ranges::equal(seen.signal_values, frame);
                expect(clean) << std::format("offset {}: the transfer frame, byte for byte, zero uncorrectable", offset);
                cleanFrames += clean ? 1UZ : 0UZ;
            }
        }
        expect(eq(cleanFrames, 2UZ)) << "one instance, both offsets: the marker's position is the pairing";
    };

    // The four KISS recipes load through both front ends and round-trip.
    "the four KISS recipes load through the YAML front end and the generated typed header"_test = [] {
        auto loader = makeRecipeLoader();

        const std::string tempFile = (std::filesystem::temp_directory_path() / "qa_recipes_kiss_load.bin").string();
        expect(loader.instantiate("gr::recipes::KissFileWrite", {{"file_name", tempFile}}) != nullptr) << "KissFileWrite via the loader";
        expect(loader.instantiate("gr::recipes::KissFileRead", {{"file_name", tempFile}, {"max_payload_items", gr::Size_t{4096U}}}) != nullptr) << "KissFileRead via the loader";
#ifdef GNURADIO4_HAVE_NETWORK_BLOCKS
        expect(loader.instantiate("gr::recipes::KissServe", {{"endpoint", std::string("127.0.0.1:1")}, {"queue_bytes", gr::Size_t{4096U}}}) != nullptr) << "KissServe via the loader";
#endif // GNURADIO4_HAVE_NETWORK_BLOCKS
        expect(loader.instantiate("gr::recipes::KissStreamDecode", {{"max_payload_items", gr::Size_t{4096U}}}) != nullptr) << "KissStreamDecode via the loader";

        gr::Graph gWrite;
        expect(gr::recipes::KissFileWrite::emplace(gWrite, gr::recipes::KissFileWrite::Parameters(tempFile)) != nullptr) << "KissFileWrite via the generated header";
        gr::Graph gRead;
        expect(gr::recipes::KissFileRead::emplace(gRead, gr::recipes::KissFileRead::Parameters(tempFile, std::uint32_t{4096U})) != nullptr) << "KissFileRead via the generated header";
#ifdef GNURADIO4_HAVE_NETWORK_BLOCKS
        gr::Graph gServe;
        expect(gr::recipes::KissServe::emplace(gServe, gr::recipes::KissServe::Parameters(std::string("127.0.0.1:1"), std::uint32_t{4096U})) != nullptr) << "KissServe via the generated header";
#endif // GNURADIO4_HAVE_NETWORK_BLOCKS
        gr::Graph gStream;
        expect(gr::recipes::KissStreamDecode::emplace(gStream, gr::recipes::KissStreamDecode::Parameters(std::uint32_t{4096U})) != nullptr) << "KissStreamDecode via the generated header";
    };

    "KissFileWrite -> KissFileRead reproduces seeded frames over a temporary file, with kiss_port and timestamps"_test = [] {
        const std::string path = (std::filesystem::temp_directory_path() / "qa_recipes_kiss_write_read.bin").string();
        std::filesystem::remove(path);

        const std::vector<gr::DataSet<std::uint8_t>> seeded{
            kissPayload({0x01U, 0x02U, 0x03U}, 1'500'000'000'123'000'000LL),
            kissPayload({0xC0U, 0xDBU, 0xAAU}), // the two SLIP-significant bytes, and no timestamp
            kissPayload({0xFFU, 0x00U, 0x7EU}, 2'000'000'000'000'000'000LL),
        };

        {
            auto                   loader = makeRecipeLoader();
            const gr::property_map writeParams{{"file_name", path}, {"mode", std::string("overwrite")}, {"kiss_port", gr::Size_t{5U}}, {"emit_timestamp", true}};
            auto                   composite = loader.instantiate("gr::recipes::KissFileWrite", writeParams);
            expect(composite != nullptr) << boost::ut::fatal;

            gr::Graph graph;
            auto&     source = graph.emplaceBlock<KissRecordSource>();
            source._records  = seeded;
            const auto chain = graph.addBlock(std::move(composite));

            const auto sourceModel = gr::graph::findBlock(graph, source);
            expect(sourceModel.has_value()) << boost::ut::fatal;
            expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, chain, gr::PortDefinition{"in"}).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
            expect(scheduler.runAndWait().has_value());
        }

        {
            auto                   loader = makeRecipeLoader();
            const gr::property_map readParams{{"file_name", path}, {"max_payload_items", gr::Size_t{4096U}}, {"read_timestamp", true}};
            auto                   composite = loader.instantiate("gr::recipes::KissFileRead", readParams);
            expect(composite != nullptr) << boost::ut::fatal;

            gr::Graph  graph;
            auto&      sink  = graph.emplaceBlock<CcsdsRecordSink>();
            const auto chain = graph.addBlock(std::move(composite));

            const auto sinkModel = gr::graph::findBlock(graph, sink);
            expect(sinkModel.has_value()) << boost::ut::fatal;
            expect(graph.connect(chain, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
            expect(scheduler.runAndWait().has_value());

            expect(eq(sink._records.size(), seeded.size())) << "KissFileWrite -> KissFileRead: frame count";
            if (sink._records.size() == seeded.size()) {
                for (std::size_t i = 0UZ; i < seeded.size(); ++i) {
                    expect(std::ranges::equal(sink._records[i].signal_values, seeded[i].signal_values)) << std::format("frame {}: payload byte for byte", i);
                    const auto& map       = sink._records[i].meta_information[0UZ];
                    const auto  portEntry = map.find(gr::property_map::key_type("kiss_port"));
                    expect(that % (portEntry != map.end() && portEntry->second.value_or(gr::Size_t{0xFFFFU}) == gr::Size_t{5U})) << std::format("frame {}: kiss_port", i);
                    if (seeded[i].timestamp != 0) {
                        expect(eq(sink._records[i].timestamp, (seeded[i].timestamp / 1'000'000LL) * 1'000'000LL)) << std::format("frame {}: timestamp truncated to the millisecond", i);
                    } else {
                        expect(eq(sink._records[i].timestamp, std::int64_t{0LL})) << std::format("frame {}: no timestamp, none invented", i);
                    }
                }
            }
        }
        std::filesystem::remove(path);
    };

#ifdef GNURADIO4_HAVE_NETWORK_BLOCKS
    "KissServe puts KissFileWrite's own bytes on a socket, and KissFileRead reads them back with their timestamps"_test = [] {
        const std::vector<gr::DataSet<std::uint8_t>> seeded{
            kissPayload({0x66U, 0xC0U, 0x77U}, 1'500'000'000'123'000'000LL), // a SLIP-significant byte, and a stamp
            kissPayload({0xDBU, 0x88U}),                                     // the escape introducer, and no stamp
        };
        const gr::property_map encodeParams{{"kiss_port", gr::Size_t{7U}}, {"emit_timestamp", true}};

        // `KissServe` is `KissFileWrite` with the file swapped for a socket, so what the file holds is exactly what
        // the socket must carry: the reference is the sibling recipe's own output, not a wire form spelled out here
        const std::vector<std::uint8_t> wire = kissWireForm(seeded, encodeParams, (std::filesystem::temp_directory_path() / "qa_recipes_kiss_serve_reference.bin").string());
        expect(that % (wire.size() > 4UZ)) << boost::ut::fatal;

        const std::uint16_t port     = reserveKissPort();
        const std::string   endpoint = std::format("127.0.0.1:{}", port);

        auto             loader      = makeRecipeLoader();
        gr::property_map serveParams = encodeParams;
        serveParams["endpoint"]      = std::pmr::string(endpoint);
        serveParams["bind"]          = true;
        serveParams["overflow"]      = std::pmr::string("backpressure");
        serveParams["queue_bytes"]   = gr::Size_t{65536U};
        auto composite               = loader.instantiate("gr::recipes::KissServe", serveParams);
        expect(composite != nullptr) << boost::ut::fatal;

        gr::Graph serverGraph;
        auto&     source       = serverGraph.emplaceBlock<KissRecordSource>();
        source._records        = seeded;
        source._neverDone      = true; // the test owns the teardown; DONE here would tear the sink down before a peer connects
        const auto chain       = serverGraph.addBlock(std::move(composite));
        const auto sourceModel = gr::graph::findBlock(serverGraph, source);
        expect(sourceModel.has_value()) << boost::ut::fatal;
        expect(serverGraph.connect(*sourceModel, gr::PortDefinition{"out"}, chain, gr::PortDefinition{"in"}).has_value());

        gr::Graph tailGraph;
        auto&     tcp      = tailGraph.emplaceBlock<gr::blocks::network::TcpByteSource>({{"endpoint", endpoint}, {"bind", false}, {"overflow", std::string("backpressure")}, {"queue_bytes", gr::Size_t{65536U}}});
        auto&     tailSink = tailGraph.emplaceBlock<KissByteSink>();
        expect(tailGraph.connect<"out", "in">(tcp, tailSink).has_value());

        KissGraphRunner server(std::move(serverGraph));
        KissGraphRunner tail(std::move(tailGraph));
        expect(kissWaitUntil([&tailSink, &wire] { return tailSink.count() >= wire.size(); })) << std::format("only {} of {} bytes crossed the loopback link", tailSink.count(), wire.size());
        server.stop();
        tail.stop();

        const std::vector<std::uint8_t> carried = tailSink.take();
        expect(that % (carried == wire)) << "the socket carried the file recipe's bytes, byte for byte";

        // the decode is `KissFileRead` itself, given the bytes the socket delivered
        const std::vector<gr::DataSet<std::uint8_t>> decoded = kissReadBack(carried, {{"max_payload_items", gr::Size_t{4096U}}, {"read_timestamp", true}}, (std::filesystem::temp_directory_path() / "qa_recipes_kiss_serve_through.bin").string());

        expect(eq(decoded.size(), seeded.size())) << "KissServe -> KissFileRead: frame count";
        if (decoded.size() == seeded.size()) {
            for (std::size_t i = 0UZ; i < seeded.size(); ++i) {
                expect(std::ranges::equal(decoded[i].signal_values, seeded[i].signal_values)) << std::format("frame {}: payload byte for byte, SLIP-significant bytes included", i);
                const auto& map       = decoded[i].meta_information[0UZ];
                const auto  portEntry = map.find(gr::property_map::key_type("kiss_port"));
                expect(that % (portEntry != map.end() && portEntry->second.value_or(gr::Size_t{0xFFFFU}) == gr::Size_t{7U})) << std::format("frame {}: kiss_port", i);
                expect(eq(decoded[i].timestamp, seeded[i].timestamp)) << std::format("frame {}: the stamp the record carried, or none invented for the record that had none", i);
            }
        }
    };
#endif // GNURADIO4_HAVE_NETWORK_BLOCKS

    "KissStreamDecode reassembles a KISS stream split at every stride, mid-frame and mid-escape alike"_test = [] {
        // frameA's payload deliberately contains both SLIP-significant bytes, so the wire form needs escaping —
        // exactly where a split must not desynchronize DelimiterExtractor's machine
        const gr::DataSet<std::uint8_t> frameA = kissPayload({0x11U, 0xC0U, 0x22U, 0xDBU, 0x33U});
        const gr::DataSet<std::uint8_t> frameB = kissPayload({0x44U, 0x55U});

        const std::vector<std::uint8_t> wire = kissWireForm({frameA, frameB}, {{"kiss_port", gr::Size_t{3U}}}, (std::filesystem::temp_directory_path() / "qa_recipes_kiss_stream_decode.bin").string());
        expect(that % (wire.size() > 4UZ)) << boost::ut::fatal;
        expect(that % (std::ranges::count(wire, std::uint8_t{0xDBU}) >= 2)) << "the wire form carries escape pairs, so a split can land inside one";

        auto                   loader = makeRecipeLoader();
        const gr::property_map readParams{{"drop_head", gr::Size_t{0U}}, {"max_payload_items", gr::Size_t{4096U}}};

        // every stride from one byte a record up to one byte short of the whole stream: some of them put a record
        // boundary between an escape introducer and the byte it escapes, and some put one between the two frames,
        // which are the splits a record-domain decoder gets wrong and the ones this recipe exists for
        for (std::size_t stride = 1UZ; stride < wire.size(); ++stride) {
            std::vector<gr::DataSet<std::uint8_t>> fragments;
            for (std::size_t at = 0UZ; at < wire.size(); at += stride) {
                const std::size_t take = std::min(stride, wire.size() - at);
                fragments.push_back(kissPayload(std::vector<std::uint8_t>(wire.begin() + static_cast<std::ptrdiff_t>(at), wire.begin() + static_cast<std::ptrdiff_t>(at + take))));
            }

            auto composite = loader.instantiate("gr::recipes::KissStreamDecode", readParams);
            expect(composite != nullptr) << boost::ut::fatal;

            gr::Graph graph;
            auto&     source       = graph.emplaceBlock<KissRecordSource>();
            source._records        = fragments;
            auto&      sink        = graph.emplaceBlock<CcsdsRecordSink>();
            const auto chain       = graph.addBlock(std::move(composite));
            const auto sourceModel = gr::graph::findBlock(graph, source);
            const auto sinkModel   = gr::graph::findBlock(graph, sink);
            expect(sourceModel.has_value() && sinkModel.has_value()) << boost::ut::fatal;
            expect(graph.connect(*sourceModel, gr::PortDefinition{"out"}, chain, gr::PortDefinition{"in"}).has_value());
            expect(graph.connect(chain, gr::PortDefinition{"out"}, *sinkModel, gr::PortDefinition{"in"}).has_value());

            gr::scheduler::Simple scheduler;
            expect(scheduler.exchange(std::move(graph)).has_value()) << boost::ut::fatal;
            expect(scheduler.runAndWait().has_value());

            expect(eq(sink._records.size(), 2UZ)) << std::format("stride {}: both frames, reassembled across the record boundaries it makes", stride);
            if (sink._records.size() == 2UZ) {
                expect(std::ranges::equal(sink._records[0UZ].signal_values, frameA.signal_values)) << std::format("stride {}: frame A, its escaped bytes included", stride);
                expect(std::ranges::equal(sink._records[1UZ].signal_values, frameB.signal_values)) << std::format("stride {}: frame B", stride);
                for (std::size_t i = 0UZ; i < sink._records.size(); ++i) {
                    const auto& map       = sink._records[i].meta_information[0UZ];
                    const auto  portEntry = map.find(gr::property_map::key_type("kiss_port"));
                    expect(that % (portEntry != map.end() && portEntry->second.value_or(gr::Size_t{0xFFFFU}) == gr::Size_t{3U})) << std::format("stride {}, frame {}", stride, i);
                }
            }
        }
    };

    "the committed typed headers match a fresh emission byte for byte"_test = [] {
        auto        loader = makeRecipeLoader();
        const auto& defs   = loader.definitionForBlockName();
        expect(!defs.empty());
        for (const auto& [name, definition] : defs) {
            const auto lastColon  = name.rfind("::");
            const auto structName = lastColon == std::string::npos ? name : name.substr(lastColon + 2);
            const auto emitted    = gr::recipe::emitter::emitRecipeHeader(definition, structName + ".yaml");
            expect(emitted.has_value()) << name << (emitted.has_value() ? "" : emitted.error().message);
            if (!emitted.has_value()) {
                continue;
            }
            std::ifstream committed(std::string(RECIPES_SOURCE_PATH) + "/include/gnuradio-4.0/recipes/" + structName + ".hpp", std::ios::binary);
            expect(bool(committed)) << structName << ".hpp is not committed — run gr4-recipe-gen";
            if (!committed) {
                continue;
            }
            std::ostringstream content;
            content << committed.rdbuf();
            expect(content.str() == *emitted) << structName << ".hpp drifted from its recipe — regenerate and commit";
        }
    };

    "the generated header and the loader produce equivalent composites"_test = [] {
        auto       loader = makeRecipeLoader();
        gr::Graph  viaHeaderGraph;
        const auto viaHeader = gr::recipes::NbfmDemod::emplace(viaHeaderGraph, gr::recipes::NbfmDemod::Parameters(96000.0f, 5000.0f));
        expect(viaHeader != nullptr) << "the generated emplace must build the composite";

        gr::property_map parameters;
        parameters["sample_rate"] = 96000.0f;
        parameters["deviation"]   = 5000.0f;
        const auto viaLoader      = loader.instantiate("gr::recipes::NbfmDemod", parameters);
        expect(viaLoader != nullptr);
        if (viaHeader == nullptr || viaLoader == nullptr) {
            return;
        }
        for (const char* interiorName : {"discriminator", "deemphasis"}) {
            const auto a = interiorByName(viaHeader, interiorName);
            const auto b = interiorByName(viaLoader, interiorName);
            expect(a != nullptr && b != nullptr) << interiorName;
            if (a == nullptr || b == nullptr) {
                continue;
            }
            for (const char* key : {"gain", "sample_rate", "tau"}) {
                const auto left  = a->settings().get(key);
                const auto right = b->settings().get(key);
                expect(left.has_value() == right.has_value()) << interiorName << "." << key;
                if (left.has_value() && right.has_value()) {
                    expect(eq(numericOf(*left), numericOf(*right))) << interiorName << "." << key << " must agree across front ends";
                }
            }
        }
        auto* wrapper = dynamic_cast<gr::GraphWrapper<gr::Graph>*>(viaHeader.get());
        expect(wrapper != nullptr) << "the generated composite carries the live machinery too";
        if (wrapper != nullptr) {
            gr::property_map change;
            change["deviation"] = 2500.0f;
            expect(wrapper->applyRecipeParameters(change).has_value()) << "live re-evaluation works through the generated front end";
        }
    };
};

int main() { /* not needed for ut */ }
