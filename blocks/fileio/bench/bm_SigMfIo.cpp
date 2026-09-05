#include <algorithm>
#include <array>
#include <chrono>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <limits>
#include <print>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gnuradio-4.0/fileio/SigMfIo.hpp>

// What a SigMF source and sink cost per item end to end, over a real file, at three buffer sizes.
//
// The codec's own cost is measured separately in `algorithm/benchmarks/bm_SigMfCodec.cpp`; the point of this one is
// the difference between that figure and the block's, which is the file I/O and the tag-schedule scan. Both blocks
// hold their own handle and read and write inline, so what is measured here is a `read(2)` or a `write(2)` against
// the page cache plus one indirect call into the selected conversion — and no thread hand-off.
//
// No threshold is asserted. The target is EXCLUDE_FROM_ALL, so it is built by name when a measurement is wanted.

namespace {

using namespace std::chrono;
using gr::blocks::fileio::SigMfSink;
using gr::blocks::fileio::SigMfSource;

constexpr std::array<std::size_t, 3> kBufferSizes{1024UZ, 65536UZ, 1048576UZ};
constexpr std::size_t                kItems = 4UZ * 1024UZ * 1024UZ;

struct TagWriterStub : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterStub() = default;
    constexpr explicit TagWriterStub(std::span<gr::Tag> room) : std::span<gr::Tag>(room) {}
    constexpr void publish(std::size_t) const noexcept {}
};

struct TagReaderStub : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderStub() = default;
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

template<typename T>
struct OutputSpanStub : std::span<T> {
    using value_type = T;

    std::array<gr::Tag, 32> tagRoom{};
    TagWriterStub           tags{};
    std::size_t             streamIndex{0UZ};
    std::size_t             count{0UZ};
    bool                    isConnected{true};
    bool                    isSync{true};

    explicit OutputSpanStub(std::span<T> samples) : std::span<T>(samples) { tags = TagWriterStub(std::span<gr::Tag>(tagRoom)); }

    constexpr void publish(std::size_t nSamples) noexcept { count = nSamples; }
    void           publishTag(const gr::property_map&, std::size_t = 0UZ) {}
};

template<typename T>
struct InputSpanStub : std::span<const T> {
    using value_type = T;

    TagReaderStub rawTags{};
    std::size_t   streamIndex{0UZ};
    std::size_t   consumed{0UZ};
    bool          isConnected{true};
    bool          isSync{true};

    explicit InputSpanStub(std::span<const T> samples) : std::span<const T>(samples) {}

    constexpr bool consume(std::size_t nSamples) noexcept {
        consumed = nSamples;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

    /// The sink reads a tag as `[relativeIndex, mapRef]`, so the view is that pair and not a raw `gr::Tag`.
    using TagView = std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>;

    [[nodiscard]] std::vector<TagView> tags() const {
        std::vector<TagView> view;
        view.reserve(rawTags.size());
        for (const gr::Tag& tag : rawTags) {
            view.emplace_back(static_cast<std::ptrdiff_t>(tag.index), std::cref(tag.map));
        }
        return view;
    }
    [[nodiscard]] std::vector<TagView> tags(std::size_t) const { return tags(); }
};

template<typename TBlock>
[[nodiscard]] TBlock makeBlock(gr::property_map settings) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

void writeRecording(const std::string& base, std::string_view datatype, std::size_t items, std::size_t bytesPerItem) {
    std::ofstream meta(base + ".sigmf-meta", std::ios::binary | std::ios::trunc);
    const auto    text = std::format(R"({{"global": {{"core:datatype": "{}", "core:version": "1.2.0", "core:sample_rate": 61440000}}, "captures": [{{"core:sample_start": 0}}], "annotations": []}})", datatype);
    meta.write(text.data(), static_cast<std::streamsize>(text.size()));
    meta.close();

    std::vector<std::byte> data(items * bytesPerItem, std::byte{0x11});
    std::ofstream          dataset(base + ".sigmf-data", std::ios::binary | std::ios::trunc);
    dataset.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
}

void reportSource(const std::filesystem::path& directory, std::string_view datatype, std::size_t bytesPerItem, std::size_t bufferItems) {
    const std::string base = (directory / "bench").string();
    writeRecording(base, datatype, kItems, bytesPerItem);

    auto block = makeBlock<SigMfSource<std::complex<float>>>({{"file_name", base}});
    block.start();

    std::vector<std::complex<float>> scratch(bufferItems);
    std::size_t                      produced = 0UZ;
    const auto                       start    = steady_clock::now();
    while (produced < kItems) {
        OutputSpanStub<std::complex<float>> outSpan{std::span<std::complex<float>>(scratch)};
        const auto                          status = block.processBulk(outSpan);
        produced += outSpan.count;
        if (outSpan.count == 0UZ || status == gr::work::Status::DONE) {
            break;
        }
    }
    const auto elapsed = duration_cast<nanoseconds>(steady_clock::now() - start).count();
    block.stop();

    std::println("  source {:>8} {:>9} items/call  {:8.3f} ns/item   ({} items)", datatype, bufferItems, static_cast<double>(elapsed) / static_cast<double>(std::max(produced, 1UZ)), produced);
}

void reportSink(const std::filesystem::path& directory, std::string_view datatype, std::size_t bufferItems) {
    const std::string base = (directory / "sink").string();
    std::filesystem::remove(base + ".sigmf-data");
    std::filesystem::remove(base + ".sigmf-meta");

    auto block = makeBlock<SigMfSink<std::complex<float>>>({{"file_name", base}, {"overwrite", true}, {"datatype", std::string(datatype)}});
    block.start();

    const std::vector<std::complex<float>> source(bufferItems, std::complex<float>{0.25f, -0.75f});
    std::size_t                            written = 0UZ;
    const auto                             start   = steady_clock::now();
    while (written < kItems) {
        InputSpanStub<std::complex<float>> inSpan{std::span<const std::complex<float>>(source)};
        std::ignore = block.processBulk(inSpan);
        if (inSpan.consumed == 0UZ) {
            break;
        }
        written += inSpan.consumed;
    }
    const auto elapsed = duration_cast<nanoseconds>(steady_clock::now() - start).count();
    block.stop();

    std::println("  sink   {:>8} {:>9} items/call  {:8.3f} ns/item   ({} items)", datatype, bufferItems, static_cast<double>(elapsed) / static_cast<double>(std::max(written, 1UZ)), written);
}

} // namespace

int main() {
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "bm_SigMfIo";
    std::error_code             status;
    std::filesystem::remove_all(directory, status);
    std::filesystem::create_directories(directory, status);

    std::println("SigMF blocks — end to end over a temporary file, {} items per run", kItems);
    for (const std::size_t bufferItems : kBufferSizes) {
        reportSource(directory, "cf32_le", 8UZ, bufferItems);
        reportSource(directory, "ci16_le", 4UZ, bufferItems);
    }
    std::println("");
    for (const std::size_t bufferItems : kBufferSizes) {
        reportSink(directory, "cf32_le", bufferItems);
        reportSink(directory, "ci16_le", bufferItems);
    }

    std::filesystem::remove_all(directory, status);
    return 0;
}
