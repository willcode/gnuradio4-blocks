#ifndef GNURADIO_FILEIO_SIGMF_IO_HPP
#define GNURADIO_FILEIO_SIGMF_IO_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/algorithm/sigmf/SampleCodec.hpp>
#include <gnuradio-4.0/algorithm/sigmf/SigMfMetadata.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <complex>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <print>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <gnuradio-4.0/fileio/NamespaceCompatibility.hpp>

namespace gr::blocks::fileio {

/**
 * @brief The stream mapping between a SigMF recording and a `Stream<T>`.
 *
 * A recording enters the graph as samples plus the tags and settings the framework already has a
 * vocabulary for. Six reserved keys carry the signal-processing facts; eleven `sigmf_`-prefixed keys
 * carry the annotation fields and the verbatim carriage of everything this tree does not consume.
 */
namespace sigmf_keys {
inline constexpr std::string_view kDatatype            = "sigmf_datatype";
inline constexpr std::string_view kGlobalExtra         = "sigmf_global_extra";
inline constexpr std::string_view kCaptureExtra        = "sigmf_capture_extra";
inline constexpr std::string_view kAnnotationExtra     = "sigmf_annotation_extra";
inline constexpr std::string_view kAnnotationLabel     = "sigmf_annotation_label";
inline constexpr std::string_view kAnnotationLength    = "sigmf_annotation_length";
inline constexpr std::string_view kAnnotationFreqLower = "sigmf_annotation_freq_lower";
inline constexpr std::string_view kAnnotationFreqUpper = "sigmf_annotation_freq_upper";
inline constexpr std::string_view kAnnotationComment   = "sigmf_annotation_comment";
inline constexpr std::string_view kAnnotationGenerator = "sigmf_annotation_generator";
inline constexpr std::string_view kAnnotationUuid      = "sigmf_annotation_uuid";
} // namespace sigmf_keys

namespace sigmf_detail {

/// The five sample types this module registers, matching `BasicFileSink` and `BasicFileSource`.
template<typename T>
concept SigMfSample = std::same_as<T, std::uint8_t> || std::same_as<T, std::int16_t> || std::same_as<T, std::int32_t> || std::same_as<T, float> || std::same_as<T, std::complex<float>>;

/// The key without the `gr:` prefix the framework carries internally.
[[nodiscard]] inline constexpr std::string_view shortTagKey(std::string_view key) noexcept {
    constexpr std::string_view prefix = "gr:";
    return key.starts_with(prefix) ? key.substr(prefix.size()) : key;
}

/// Look a key up in `map`, accepting the `gr:`-prefixed spelling of the same key.
[[nodiscard]] inline const pmt::Value* findTag(const property_map& map, std::string_view key) noexcept {
    if (const auto it = map.find(key); it != map.end()) {
        return &it->second;
    }
    for (const auto& [name, value] : map) {
        if (shortTagKey(std::string_view(name)) == key) {
            return &value;
        }
    }
    return nullptr;
}

struct ResolvedPath {
    std::string base{};
    bool        isArchive{false};
};

/// A recording is named by its base: `foo`, `foo.sigmf-meta` and `foo.sigmf-data` all name the same
/// one. A `.sigmf` suffix names an archive and is reported so the caller can refuse it by name.
[[nodiscard]] inline ResolvedPath resolveBase(std::string_view fileName) {
    if (fileName.ends_with(".sigmf-meta")) {
        return ResolvedPath{std::string(fileName.substr(0UZ, fileName.size() - 11UZ)), false};
    }
    if (fileName.ends_with(".sigmf-data")) {
        return ResolvedPath{std::string(fileName.substr(0UZ, fileName.size() - 11UZ)), false};
    }
    if (fileName.ends_with(".sigmf")) {
        return ResolvedPath{std::string(fileName), true};
    }
    return ResolvedPath{std::string(fileName), false};
}

/// Multiply two index terms, reporting the overflow rather than wrapping.
[[nodiscard]] inline bool checkedMultiply(std::uint64_t lhs, std::uint64_t rhs, std::uint64_t& product) noexcept {
    if (rhs != 0U && lhs > std::numeric_limits<std::uint64_t>::max() / rhs) {
        return false;
    }
    product = lhs * rhs;
    return true;
}

/// Nanoseconds since the Unix epoch, from the host clock.
[[nodiscard]] inline std::uint64_t hostTimeNs() { return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()); }

/// Read a `sigmf_*_extra` string back into a JSON object; an unparsable string is reported.
[[nodiscard]] inline std::optional<gr::sigmf::json::Value> parseExtras(std::string_view text) {
    auto document = gr::sigmf::json::parse(text);
    if (!document || !document->isObject()) {
        return std::nullopt;
    }
    return std::move(*document);
}

/// Append every member of `source` to `destination`, replacing an existing member of the same key.
inline void mergeInto(gr::sigmf::json::Value& destination, const gr::sigmf::json::Value& source) {
    for (std::size_t i = 0UZ; i < source.size(); ++i) {
        destination.set(source.keyAt(i), source.valueAt(i));
    }
}

} // namespace sigmf_detail

GR_REGISTER_BLOCK(gr::blocks::fileio::SigMfSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

/**
 * @brief Reads a SigMF recording and emits its samples as a stream, with its facts as tags.
 *
 * The whole description is validated before one sample is emitted, so a recording this block cannot
 * read faithfully is one it refuses to read at all: `start()` throws, naming the refusal and the
 * offending value. The file handle is owned for the run and read inline, never through a shared I/O
 * pool, so the block cannot deadlock against a graph that also does socket or device work.
 */
template<sigmf_detail::SigMfSample T>
struct SigMfSource : gr::Block<SigMfSource<T>> {
    using Description = Doc<R""(Streams a SigMF recording (a `.sigmf-meta`/`.sigmf-data` pair) as `Stream<T>`.

The recording's sample rate, center frequency, channel count, capture boundaries and dropped-sample
counts become the framework's own reserved tag keys; its annotations and every field this reader does
not consume travel under `sigmf_`-prefixed keys. Datatypes are read at 8, 16 and 32 bits, signed,
unsigned or IEEE float, real or complex, in either byte order, which is every datatype the SigMF
grammar defines. Archives (`.sigmf`) and non-conforming datasets with a per-capture header are not
supported.)"">;

    template<typename U, gr::meta::fixed_string description = "", typename... Arguments>
    using A = gr::Annotated<U, description, Arguments...>;

    using Component                              = gr::sigmf::ComponentOf<T>;
    static constexpr std::size_t kComponentCount = gr::sigmf::componentsPerItem<T>;

    gr::PortOut<T> out;

    A<std::string, "file_name", gr::Visible, Doc<"Path to the recording; a base name or a path ending .sigmf-meta or .sigmf-data">> file_name;
    A<bool, "repeat", Doc<"Restart at the beginning when the recording is exhausted">>                                              repeat               = false;
    A<gr::Size_t, "offset", gr::Visible, Doc<"Items to skip at the start of each pass">>                                            offset               = 0U;
    A<gr::Size_t, "length", gr::Visible, Doc<"Maximum items to emit per pass (0 = all)">>                                           length               = 0U;
    A<std::string, "scaling", Doc<"\"unit\": integers scale to [-1, 1); \"raw\": no scaling">>                                      scaling              = std::string("unit");
    A<std::string, "truncation", Doc<"\"refuse\": a short dataset refuses the start; \"allow\": emit what exists">>                 truncation           = std::string("refuse");
    A<bool, "emit_annotations", Doc<"Publish the recording's annotations as tags">>                                                 emit_annotations     = true;
    A<bool, "carry_unknown_fields", Doc<"Carry every unconsumed metadata field verbatim under the sigmf_*_extra keys">>             carry_unknown_fields = true;
    A<std::string, "capture_label", Doc<"Written under the reserved trigger_name key at every capture boundary">>                   capture_label        = std::string("sigmf:capture");
    A<gr::Size_t, "max_metadata_bytes", Doc<"Largest metadata file this block reads">>                                              max_metadata_bytes   = 16777216U;
    A<gr::Size_t, "max_captures", Doc<"Largest capture-segment count this block reads">>                                            max_captures         = 1048576U;
    A<gr::Size_t, "max_annotations", Doc<"Largest annotation count this block reads">>                                              max_annotations      = 1048576U;
    A<gr::Size_t, "max_extra_bytes", Doc<"Largest carried-extras document this block builds">>                                      max_extra_bytes      = 65536U;

    A<std::string, "datatype", gr::Visible, Doc<"Read-only, updated from the recording's core:datatype">> datatype;
    A<std::string, "sigmf_version", Doc<"Read-only, updated from the recording's core:version">>          sigmf_version;
    A<float, "sample_rate", gr::Visible, gr::Unit<"Hz">, Doc<"Read-only, updated from the recording">>    sample_rate  = 0.f;
    A<gr::Size_t, "num_channels", gr::Visible, Doc<"Read-only, updated from the recording">>              num_channels = 1U;
    A<std::uint64_t, "n_samples", Doc<"Read-only: items one pass of the recording emits">>                n_samples    = 0U;
    A<std::string, "sigmf_description", Doc<"Read-only, updated from the recording's core:description">>  sigmf_description;
    A<std::string, "author", Doc<"Read-only, updated from the recording's core:author">>                  author;
    A<std::string, "recorder", Doc<"Read-only, updated from the recording's core:recorder">>              recorder;
    A<std::string, "hardware", Doc<"Read-only, updated from the recording's core:hw">>                    hardware;
    A<std::string, "license", Doc<"Read-only, updated from the recording's core:license">>                license;
    A<std::string, "sha512", Doc<"Read-only: the recording's core:sha512, reported and never verified">>  sha512;

    GR_MAKE_REFLECTABLE(SigMfSource, out, file_name, repeat, offset, length, scaling, truncation, emit_annotations, carry_unknown_fields, capture_label, //
        max_metadata_bytes, max_captures, max_annotations, max_extra_bytes,                                                                              //
        datatype, sigmf_version, sample_rate, num_channels, n_samples, sigmf_description, author, recorder, hardware, license, sha512);

    // counters — public, monotonic, per run; the stop() line names every non-zero one
    std::uint64_t nSamplesEmitted{0U};
    std::uint64_t nCaptureBoundaries{0U};
    std::uint64_t nAnnotationsEmitted{0U};
    std::uint64_t nAnnotationsCollided{0U};
    std::uint64_t nSampleRateNarrowed{0U};
    std::uint64_t nDropCountSaturated{0U};
    std::uint64_t nShortReads{0U};
    std::uint64_t nIntegralFloatIndices{0U};
    std::uint64_t nCapturesSorted{0U};
    std::uint64_t nAnnotationsSorted{0U};
    std::uint64_t nLeadingItemsSkipped{0U};

    struct ScheduledTag {
        std::uint64_t index{0U};
        property_map  map{};
        bool          fromCapture{false};
        bool          fromAnnotation{false};
    };

    std::ifstream                  _data{};
    std::vector<ScheduledTag>      _schedule{};
    std::vector<std::byte>         _staging{};
    gr::sigmf::DecodeFn<Component> _decode{nullptr};
    std::size_t                    _bytesPerItem{0UZ};
    std::uint64_t                  _itemsPerPass{0U};
    std::uint64_t                  _itemIndex{0U};
    std::size_t                    _nextTag{0UZ};
    std::uint64_t                  _dataStartByte{0U};
    std::uint64_t                  _firstCollisionIndex{0U};
    std::uint64_t                  _shortfallItems{0U};
    bool                           _started{false};
    std::string                    _summary{};

    using gr::Block<SigMfSource<T>>::Block;

    void start() {
        resetRunState();
        openRecording();
        _started = true;
    }

    void stop() {
        _data.close();
        _schedule.clear();
        _staging.clear();
        if (_started) {
            reportRun();
            _started = false;
        }
    }

    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        if (_itemsPerPass == 0U || _decode == nullptr || _itemIndex >= _itemsPerPass) {
            outSpan.publish(0UZ); // a pass that ran out is done, whichever call observes it
            return gr::work::Status::DONE;
        }
        if (outSpan.size() == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }

        const std::uint64_t remaining = _itemsPerPass - _itemIndex;
        std::size_t         count     = static_cast<std::size_t>(std::min<std::uint64_t>(outSpan.size(), remaining));
        count                         = std::min(count, _staging.size() / _bytesPerItem);
        count                         = countFittingTagRoom(count, outSpan.tags.size());
        if (count == 0UZ) {
            outSpan.publish(0UZ);
            return gr::work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }

        const std::size_t wantedBytes = count * _bytesPerItem;
        _data.read(reinterpret_cast<char*>(_staging.data()), static_cast<std::streamsize>(wantedBytes));
        const std::size_t readBytes = static_cast<std::size_t>(_data.gcount());
        if (readBytes < wantedBytes) {
            ++nShortReads;
            _data.clear();
            count = readBytes / _bytesPerItem;
            if (count == 0UZ) {
                _itemsPerPass = _itemIndex; // nothing further will arrive from this handle
                outSpan.publish(0UZ);
                return gr::work::Status::DONE;
            }
        }

        std::ignore = _decode(_staging.data(), componentsOf(outSpan.data()), count * kComponentCount);
        publishScheduledTags(outSpan, count);

        _itemIndex += count;
        nSamplesEmitted += count;
        outSpan.publish(count);

        if (_itemIndex < _itemsPerPass) {
            return gr::work::Status::OK;
        }
        if (repeat) { // a repeat is a discontinuity in content, and the capture tag is what says so
            _itemIndex = 0U;
            _nextTag   = 0UZ;
            _data.clear();
            _data.seekg(static_cast<std::streamoff>(_dataStartByte));
            return gr::work::Status::OK;
        }
        return gr::work::Status::DONE;
    }

private:
    [[noreturn]] void refuse(std::string_view code, std::string_view detail) const { throw gr::exception(std::format("SigMfSource: {}: {}", code, detail)); }

    void resetRunState() {
        nSamplesEmitted       = 0U;
        nCaptureBoundaries    = 0U;
        nAnnotationsEmitted   = 0U;
        nAnnotationsCollided  = 0U;
        nSampleRateNarrowed   = 0U;
        nDropCountSaturated   = 0U;
        nShortReads           = 0U;
        nIntegralFloatIndices = 0U;
        nCapturesSorted       = 0U;
        nAnnotationsSorted    = 0U;
        nLeadingItemsSkipped  = 0U;
        _schedule.clear();
        _staging.clear();
        _decode              = nullptr;
        _bytesPerItem        = 0UZ;
        _itemsPerPass        = 0U;
        _itemIndex           = 0U;
        _nextTag             = 0UZ;
        _dataStartByte       = 0U;
        _firstCollisionIndex = 0U;
        _shortfallItems      = 0U;
        _started             = false;
        _summary.clear();
        _data.close();
        _data.clear();
    }

    /// The scalar components of a run of items; a real `T` is its own component type.
    [[nodiscard]] static Component* componentsOf(T* items) noexcept {
        if constexpr (std::same_as<Component, T>) {
            return items;
        } else {
            return reinterpret_cast<Component*>(items);
        }
    }

    [[nodiscard]] gr::sigmf::Scaling scalingMode() const {
        if (scaling.value == "unit") {
            return gr::sigmf::Scaling::Unit;
        }
        if (scaling.value == "raw") {
            return gr::sigmf::Scaling::Raw;
        }
        throw gr::exception(std::format("SigMfSource: unknown scaling '{}', expected \"unit\" or \"raw\"", scaling.value));
    }

    [[nodiscard]] bool allowTruncation() const {
        if (truncation.value == "refuse") {
            return false;
        }
        if (truncation.value == "allow") {
            return true;
        }
        throw gr::exception(std::format("SigMfSource: unknown truncation '{}', expected \"refuse\" or \"allow\"", truncation.value));
    }

    /// Validate the whole recording, then open the dataset. Every failure here refuses the start.
    void openRecording() {
        if (file_name.value.empty()) {
            refuse("no_file_name", "the 'file_name' setting is empty");
        }
        const auto resolved = sigmf_detail::resolveBase(file_name.value);
        if (resolved.isArchive) {
            refuse("archive_unsupported", std::format("'{}' names a .sigmf archive, which this block does not open", file_name.value));
        }
        const std::filesystem::path metaPath = resolved.base + ".sigmf-meta";
        std::filesystem::path       dataPath = resolved.base + ".sigmf-data";

        std::error_code status;
        if (!std::filesystem::exists(metaPath, status)) {
            refuse("meta_not_found", std::format("'{}' does not exist", metaPath.string()));
        }
        if (!std::filesystem::is_regular_file(metaPath, status)) {
            refuse("path_unreadable", std::format("'{}' is not a regular file", metaPath.string()));
        }
        const std::uintmax_t metaSize = std::filesystem::file_size(metaPath, status);
        if (status) {
            refuse("path_unreadable", std::format("'{}': {}", metaPath.string(), status.message()));
        }
        if (metaSize > static_cast<std::uintmax_t>(max_metadata_bytes.value)) {
            refuse("meta_too_large", std::format("'{}' is {} bytes, above the bound of {}", metaPath.string(), metaSize, max_metadata_bytes.value));
        }
        std::ifstream metaStream(metaPath, std::ios::binary);
        if (!metaStream) {
            refuse("path_unreadable", std::format("'{}' could not be opened for reading", metaPath.string()));
        }
        const std::string text{std::istreambuf_iterator<char>(metaStream), std::istreambuf_iterator<char>()};

        const gr::sigmf::Limits  limits{static_cast<std::uint64_t>(max_captures.value), static_cast<std::uint64_t>(max_annotations.value), gr::sigmf::kMaxUnsortedEntries};
        gr::sigmf::ParseCounters counters{};
        auto                     parsed = gr::sigmf::parse(text, limits, counters);
        nIntegralFloatIndices           = counters.nIntegralFloatIndices;
        nCapturesSorted                 = counters.nCapturesSorted;
        nAnnotationsSorted              = counters.nAnnotationsSorted;
        if (!parsed) {
            refuse(parsed.error().code, parsed.error().message);
        }
        const gr::sigmf::Metadata& metadata = *parsed;

        if (auto badVersion = gr::sigmf::checkVersion(metadata.global.version); badVersion) {
            refuse(badVersion->code, badVersion->message);
        }

        auto fileType = gr::sigmf::parseDatatype(metadata.global.datatype);
        if (!fileType) {
            refuse(fileType.error(), std::format("'core:datatype' is '{}'", metadata.global.datatype));
        }
        if (fileType->domain != gr::sigmf::domainOf<T>()) {
            refuse("datatype_domain", std::format("'core:datatype' is '{}' but the port carries '{}'", metadata.global.datatype, gr::meta::type_name<T>()));
        }
        auto decoder = gr::sigmf::selectDecoder<Component>(*fileType, scalingMode());
        if (!decoder) {
            refuse(decoder.error(), std::format("'core:datatype' is '{}' and the port carries '{}'", metadata.global.datatype, gr::meta::type_name<T>()));
        }
        _decode       = *decoder;
        _bytesPerItem = fileType->bytesPerSample();

        if (metadata.global.metadataOnly.value_or(false)) {
            refuse("metadata_only", "'core:metadata_only' is true, so there is no dataset to stream");
        }
        if (metadata.global.dataset) {
            dataPath = metaPath.parent_path() / *metadata.global.dataset;
            if (!std::filesystem::exists(dataPath, status)) {
                refuse("nonconforming_dataset", std::format("'core:dataset' names '{}', which does not exist", dataPath.string()));
            }
        } else if (!std::filesystem::exists(dataPath, status)) {
            refuse("data_not_found", std::format("'{}' does not exist", dataPath.string()));
        }
        if (!std::filesystem::is_regular_file(dataPath, status)) {
            refuse("path_unreadable", std::format("'{}' is not a regular file", dataPath.string()));
        }

        if (auto badDocument = gr::sigmf::validateForStreaming(metadata); badDocument) {
            refuse(badDocument->code, badDocument->message);
        }

        const std::uintmax_t dataSize = std::filesystem::file_size(dataPath, status);
        if (status) {
            refuse("path_unreadable", std::format("'{}': {}", dataPath.string(), status.message()));
        }
        buildSchedule(metadata, dataPath, static_cast<std::uint64_t>(dataSize));

        _data.open(dataPath, std::ios::binary);
        if (!_data) {
            refuse("path_unreadable", std::format("'{}' could not be opened for reading", dataPath.string()));
        }
        _data.seekg(static_cast<std::streamoff>(_dataStartByte));
    }

    /// Reflect the recording's own description onto the read-only settings members.
    void reflectMetadata(const gr::sigmf::Metadata& metadata) {
        datatype          = metadata.global.datatype;
        sigmf_version     = metadata.global.version;
        sigmf_description = metadata.global.description.value_or("");
        author            = metadata.global.author.value_or("");
        recorder          = metadata.global.recorder.value_or("");
        hardware          = metadata.global.hardware.value_or("");
        license           = metadata.global.license.value_or("");
        sha512            = metadata.global.sha512.value_or("");
        num_channels      = static_cast<gr::Size_t>(metadata.global.numChannels.value_or(1U));

        if (metadata.global.sampleRate) {
            const float narrowed = static_cast<float>(*metadata.global.sampleRate);
            sample_rate          = narrowed;
            if (static_cast<double>(narrowed) != *metadata.global.sampleRate) {
                nSampleRateNarrowed = 1U;
            }
        } else {
            sample_rate = 0.f;
        }
    }

    /// Build the carriage document for one object kind, refusing rather than truncating.
    ///
    /// `core:sample_rate` and `core:frequency` are carried although this block consumes them: the
    /// first loses precision narrowing to the reserved `float` key, so carrying the exact original
    /// is what makes the round trip reconstructible.
    [[nodiscard]] std::string carriageFor(const gr::sigmf::json::Value& extra, std::string_view consumedKey, std::optional<double> consumedValue) {
        if (!carry_unknown_fields) {
            return {};
        }
        gr::sigmf::json::Value document = extra;
        if (consumedValue) {
            document.set(consumedKey, gr::sigmf::json::Value::fromDouble(*consumedValue));
        }
        if (document.size() == 0UZ) {
            return {};
        }
        std::string text = gr::sigmf::json::write(document);
        if (text.size() > static_cast<std::size_t>(max_extra_bytes.value)) {
            refuse("extras_too_large", std::format("a carried document is {} bytes, above the bound of {}", text.size(), max_extra_bytes.value));
        }
        return text;
    }

    void buildSchedule(const gr::sigmf::Metadata& metadata, const std::filesystem::path& dataPath, std::uint64_t dataSize) {
        reflectMetadata(metadata);

        const std::uint64_t trailingBytes = metadata.global.trailingBytes.value_or(0U);
        if (trailingBytes > dataSize) {
            refuse("data_truncated", std::format("'{}' is {} bytes, below the {} trailing bytes the metadata states", dataPath.string(), dataSize, trailingBytes));
        }
        const std::uint64_t usableBytes = dataSize - trailingBytes;
        if (usableBytes % static_cast<std::uint64_t>(_bytesPerItem) != 0U) {
            refuse("data_not_whole_samples", std::format("{} usable bytes are not a whole multiple of the {}-byte item stride", usableBytes, _bytesPerItem));
        }
        const std::uint64_t itemsPerSample = static_cast<std::uint64_t>(num_channels.value);
        const std::uint64_t itemsInFile    = usableBytes / static_cast<std::uint64_t>(_bytesPerItem);
        if (itemsInFile % itemsPerSample != 0U) {
            refuse("data_not_whole_samples", std::format("{} items are not a whole multiple of the {} interleaved channels", itemsInFile, itemsPerSample));
        }

        const std::uint64_t globalOffset = metadata.global.offset.value_or(0U);
        const std::uint64_t streamBase   = metadata.captures.front().sampleStart;
        const std::uint64_t leadingGap   = streamBase - globalOffset; // validateForStreaming proved this is non-negative
        std::uint64_t       leadingItems = 0U;
        if (!sigmf_detail::checkedMultiply(leadingGap, itemsPerSample, leadingItems)) {
            refuse("index_overflow", std::format("the leading gap of {} samples overflows the item index", leadingGap));
        }
        if (leadingItems > itemsInFile) {
            refuse("data_truncated", std::format("the first capture segment begins {} items into a dataset holding {}", leadingItems, itemsInFile));
        }
        nLeadingItemsSkipped = leadingItems;

        const std::uint64_t availableItems = itemsInFile - leadingItems;
        const std::uint64_t impliedSamples = impliedSampleCount(metadata, streamBase);
        std::uint64_t       impliedItems   = 0U;
        if (!sigmf_detail::checkedMultiply(impliedSamples, itemsPerSample, impliedItems)) {
            refuse("index_overflow", std::format("the implied span of {} samples overflows the item index", impliedSamples));
        }
        if (impliedItems > availableItems) {
            if (!allowTruncation()) {
                refuse("data_truncated", std::format("the metadata implies {} items, the dataset holds {}, a shortfall of {}", impliedItems, availableItems, impliedItems - availableItems));
            }
            _shortfallItems = impliedItems - availableItems;
            ++nShortReads;
        }

        const std::uint64_t skipItems = static_cast<std::uint64_t>(offset.value);
        std::uint64_t       emitted   = availableItems > skipItems ? availableItems - skipItems : 0U;
        if (length.value != 0U) {
            emitted = std::min(emitted, static_cast<std::uint64_t>(length.value));
        }
        _itemsPerPass  = emitted;
        n_samples      = emitted;
        _dataStartByte = (leadingItems + skipItems) * static_cast<std::uint64_t>(_bytesPerItem);

        const std::string globalExtra = carriageFor(metadata.global.extra, "core:sample_rate", metadata.global.sampleRate);

        std::map<std::uint64_t, ScheduledTag> byIndex;
        const auto                            slotFor = [&byIndex](std::uint64_t index) -> ScheduledTag& {
            auto [it, inserted] = byIndex.try_emplace(index);
            if (inserted) {
                it->second.index = index;
            }
            return it->second;
        };

        for (std::size_t i = 0UZ; i < metadata.captures.size(); ++i) {
            const gr::sigmf::Capture& capture = metadata.captures[i];
            std::uint64_t             index   = 0U;
            if (!tagIndexFor(capture.sampleStart, streamBase, itemsPerSample, skipItems, index)) {
                continue;
            }
            ScheduledTag& slot = slotFor(index);
            slot.fromCapture   = true;
            gr::tag::put(slot.map, gr::tag::TRIGGER_NAME, capture_label.value);
            gr::tag::put(slot.map, gr::tag::TRIGGER_OFFSET, 0.f);
            if (capture.datetime) {
                if (const auto stamp = gr::sigmf::parseDatetimeNs(*capture.datetime); stamp) {
                    gr::tag::put(slot.map, gr::tag::TRIGGER_TIME, *stamp);
                }
            }
            if (capture.frequency) {
                gr::tag::put(slot.map, gr::tag::FREQUENCY, *capture.frequency);
            }
            if (i > 0UZ) {
                if (const auto dropped = gr::sigmf::droppedBetween(metadata.captures[i - 1UZ], capture); dropped && *dropped > 0U) {
                    constexpr std::uint64_t kMaxDropCount = std::numeric_limits<gr::Size_t>::max();
                    std::uint64_t           value         = *dropped;
                    if (value > kMaxDropCount) {
                        value = kMaxDropCount;
                        ++nDropCountSaturated;
                    }
                    gr::tag::put(slot.map, gr::tag::N_DROPPED_SAMPLES, static_cast<gr::Size_t>(value));
                }
            }
            const std::string captureExtra = carriageFor(capture.extra, "core:frequency", capture.frequency);
            if (!captureExtra.empty()) {
                gr::tag::put(slot.map, sigmf_keys::kCaptureExtra, captureExtra);
            }
        }

        if (emit_annotations) {
            std::vector<std::uint64_t> annotated;
            for (const gr::sigmf::Annotation& annotation : metadata.annotations) {
                std::uint64_t index = 0U;
                if (annotation.sampleStart < streamBase) { // inside the leading gap: it annotates items nobody emits
                    noteDroppedAnnotation(annotation.sampleStart);
                    continue;
                }
                if (!tagIndexFor(annotation.sampleStart, streamBase, itemsPerSample, skipItems, index)) {
                    noteDroppedAnnotation(annotation.sampleStart);
                    continue;
                }
                if (std::ranges::find(annotated, index) != annotated.end()) {
                    noteDroppedAnnotation(annotation.sampleStart); // a property_map cannot hold two values under one key
                    continue;
                }
                annotated.push_back(index);
                ScheduledTag& slot  = slotFor(index);
                slot.fromAnnotation = true;
                if (annotation.label) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationLabel, *annotation.label);
                }
                if (annotation.sampleCount) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationLength, static_cast<gr::Size_t>(std::min<std::uint64_t>(*annotation.sampleCount, std::numeric_limits<gr::Size_t>::max())));
                }
                if (annotation.freqLower) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationFreqLower, *annotation.freqLower);
                }
                if (annotation.freqUpper) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationFreqUpper, *annotation.freqUpper);
                }
                if (annotation.comment) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationComment, *annotation.comment);
                }
                if (annotation.generator) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationGenerator, *annotation.generator);
                }
                if (annotation.uuid) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationUuid, *annotation.uuid);
                }
                const std::string annotationExtra = carriageFor(annotation.extra, {}, std::nullopt);
                if (!annotationExtra.empty()) {
                    gr::tag::put(slot.map, sigmf_keys::kAnnotationExtra, annotationExtra);
                }
            }
        }

        if (_itemsPerPass > 0U) { // the recording's own description rides the tag at stream index 0
            ScheduledTag& slot = slotFor(0U);
            gr::tag::put(slot.map, gr::tag::SAMPLE_RATE, sample_rate.value);
            gr::tag::put(slot.map, gr::tag::NUM_CHANNELS, num_channels.value);
            gr::tag::put(slot.map, sigmf_keys::kDatatype, metadata.global.datatype);
            if (!globalExtra.empty()) {
                gr::tag::put(slot.map, sigmf_keys::kGlobalExtra, globalExtra);
            }
        }

        _schedule.reserve(byIndex.size());
        for (auto& [index, entry] : byIndex) {
            if (index < _itemsPerPass) {
                _schedule.push_back(std::move(entry));
            } else if (entry.fromAnnotation) {
                ++nAnnotationsCollided;
            }
        }

        // one staging allocation for the whole run, sized from the port's own chunk bound with a floor that keeps a
        // block driven outside a graph, where the port reports no size, from reading one item per call
        const std::size_t chunkItems = std::clamp(out.bufferSize(), 4096UZ, 65536UZ);
        _staging.assign(chunkItems * _bytesPerItem, std::byte{});
    }

    void noteDroppedAnnotation(std::uint64_t sampleStart) {
        if (nAnnotationsCollided == 0U) {
            _firstCollisionIndex = sampleStart;
        }
        ++nAnnotationsCollided;
    }

    /// The highest sample index the metadata refers to, relative to the first capture boundary.
    [[nodiscard]] static std::uint64_t impliedSampleCount(const gr::sigmf::Metadata& metadata, std::uint64_t streamBase) {
        std::uint64_t implied = 1U; // the first capture segment's own sample
        for (const gr::sigmf::Capture& capture : metadata.captures) {
            implied = std::max(implied, capture.sampleStart - streamBase + 1U);
        }
        for (const gr::sigmf::Annotation& annotation : metadata.annotations) {
            if (annotation.sampleStart < streamBase) {
                continue;
            }
            const std::uint64_t span = std::max<std::uint64_t>(annotation.sampleCount.value_or(1U), 1U);
            const std::uint64_t end  = annotation.sampleStart - streamBase;
            if (end > std::numeric_limits<std::uint64_t>::max() - span) {
                continue;
            }
            implied = std::max(implied, end + span);
        }
        return implied;
    }

    /// The stream index of a SigMF sample index, or false when it falls outside the emitted range.
    [[nodiscard]] bool tagIndexFor(std::uint64_t sampleStart, std::uint64_t streamBase, std::uint64_t itemsPerSample, std::uint64_t skipItems, std::uint64_t& index) const {
        if (sampleStart < streamBase) {
            return false;
        }
        std::uint64_t items = 0U;
        if (!sigmf_detail::checkedMultiply(sampleStart - streamBase, itemsPerSample, items)) {
            refuse("index_overflow", std::format("a sample index of {} overflows the item index", sampleStart));
        }
        if (items < skipItems) {
            return false;
        }
        index = items - skipItems;
        return true;
    }

    /// Shorten the produced range so that every tag inside it fits the span's own tag capacity. A recording whose
    /// annotations are denser than one span can carry is emitted over more calls rather than losing tags, and the
    /// placement is unchanged because every index is absolute.
    [[nodiscard]] std::size_t countFittingTagRoom(std::size_t count, std::size_t tagRoom) const {
        if (tagRoom == 0UZ) {
            return count; // a span with no room to report tags still carries its samples rather than stalling
        }
        std::size_t admitted = 0UZ;
        for (std::size_t entry = _nextTag; entry < _schedule.size() && _schedule[entry].index < _itemIndex + count; ++entry) {
            if (admitted == tagRoom) {
                return static_cast<std::size_t>(_schedule[entry].index - _itemIndex);
            }
            ++admitted;
        }
        return count;
    }

    void publishScheduledTags(gr::OutputSpanLike auto& outSpan, std::size_t count) {
        while (_nextTag < _schedule.size() && _schedule[_nextTag].index < _itemIndex + count) {
            const ScheduledTag& entry = _schedule[_nextTag];
            if (entry.index >= _itemIndex) {
                outSpan.publishTag(property_map(entry.map), static_cast<std::size_t>(entry.index - _itemIndex));
                if (entry.fromCapture) {
                    ++nCaptureBoundaries;
                }
                if (entry.fromAnnotation) {
                    ++nAnnotationsEmitted;
                }
            }
            ++_nextTag;
        }
    }

    void reportRun() {
        std::string line = std::format("SigMfSource '{}': {} items emitted", this->name, nSamplesEmitted);
        const auto  note = [&line](std::string_view text) { line.append(text); };
        if (nCaptureBoundaries != 0U) {
            note(std::format(", {} capture boundaries", nCaptureBoundaries));
        }
        if (nAnnotationsEmitted != 0U) {
            note(std::format(", {} annotations", nAnnotationsEmitted));
        }
        if (nAnnotationsCollided != 0U) {
            note(std::format(", {} annotations dropped (first at sample {}: an index already carried one, or it fell outside the emitted range)", nAnnotationsCollided, _firstCollisionIndex));
        }
        if (nSampleRateNarrowed != 0U) {
            note(std::format(", the sample rate did not survive narrowing to {}", static_cast<double>(sample_rate.value)));
        }
        if (nDropCountSaturated != 0U) {
            note(std::format(", {} dropped-sample counts saturated", nDropCountSaturated));
        }
        if (nShortReads != 0U) {
            note(std::format(", {} short reads, a shortfall of {} items", nShortReads, _shortfallItems));
        }
        if (nIntegralFloatIndices != 0U) {
            note(std::format(", {} index tokens written with an all-zero fraction", nIntegralFloatIndices));
        }
        if (nCapturesSorted != 0U) {
            note(", 'captures' arrived unsorted and was sorted");
        }
        if (nAnnotationsSorted != 0U) {
            note(", 'annotations' arrived unsorted and was sorted");
        }
        if (nLeadingItemsSkipped != 0U) {
            note(std::format(", {} leading items no capture segment describes were skipped", nLeadingItemsSkipped));
        }
        _summary = std::move(line);
        std::println("{}", _summary);
    }
};

static_assert(gr::BlockLike<SigMfSource<std::complex<float>>>);

GR_REGISTER_BLOCK(gr::blocks::fileio::SigMfSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

/**
 * @brief Writes a stream and the tags describing it as a conforming SigMF recording.
 *
 * The dataset is written inline on the scheduler thread through a handle owned for the run; the
 * metadata document is composed from the accumulated state and written once at `stop()`, after the
 * dataset is flushed and closed, so a reader that sees the metadata file knows the dataset behind it
 * is complete.
 */
template<sigmf_detail::SigMfSample T>
struct SigMfSink : gr::Block<SigMfSink<T>> {
    using Description = Doc<R""(Writes a `Stream<T>` as a SigMF recording (a `.sigmf-meta`/`.sigmf-data` pair).

The reserved `sample_rate`, `num_channels`, `frequency`, `trigger_time` and `n_dropped_samples` keys
become the recording's global fields and capture segments; the `sigmf_annotation_*` keys become its
annotation list; the `sigmf_*_extra` documents are written back verbatim. `core:sha512` is never
written, because a hash of another file's bytes would be a false statement about these. A sample rate
that changes mid-run cannot be expressed in one recording: the first rate is kept and every later
change is counted and named.)"">;

    template<typename U, gr::meta::fixed_string description = "", typename... Arguments>
    using A = gr::Annotated<U, description, Arguments...>;

    using Component                              = gr::sigmf::ComponentOf<T>;
    static constexpr std::size_t kComponentCount = gr::sigmf::componentsPerItem<T>;

    gr::PortIn<T> in;

    A<std::string, "file_name", gr::Visible, Doc<"Base path; the block writes <base>.sigmf-data and <base>.sigmf-meta">>    file_name;
    A<bool, "overwrite", Doc<"When false, an existing output file of either name refuses the start">>                       overwrite     = false;
    A<std::string, "datatype", gr::Visible, Doc<"A SigMF datatype spelling, or \"auto\" to follow the stream">>             datatype      = std::string("auto");
    A<std::string, "scaling", Doc<"\"unit\": floats scale to the integer's full scale; \"raw\": no scaling">>               scaling       = std::string("unit");
    A<float, "sample_rate", gr::Visible, gr::Unit<"Hz">, Doc<"Operator fallback, superseded by the first sample_rate tag">> sample_rate   = 0.f;
    A<gr::Size_t, "num_channels", gr::Visible, Doc<"Operator fallback, superseded by the first num_channels tag">>          num_channels  = 1U;
    A<std::string, "capture_label", Doc<"A trigger_name tag equal to this opens a capture segment">>                        capture_label = std::string("sigmf:capture");
    A<std::string, "recorder", Doc<"Written as core:recorder; always this block's own value">>                              recorder      = std::string("gnuradio4");
    A<std::string, "sigmf_description", Doc<"Written as core:description when non-empty">>                                  sigmf_description;
    A<std::string, "author", Doc<"Written as core:author when non-empty">>                                                  author;
    A<std::string, "hardware", Doc<"Written as core:hw when non-empty">>                                                    hardware;
    A<std::string, "license", Doc<"Written as core:license when non-empty">>                                                license;
    A<bool, "carry_unknown_fields", Doc<"Write the carried sigmf_*_extra documents back verbatim">>                         carry_unknown_fields = true;
    A<bool, "emit_annotations", Doc<"Record the stream's sigmf_annotation_* keys as annotations">>                          emit_annotations     = true;
    A<gr::Size_t, "max_captures", Doc<"Largest capture-segment count this block writes">>                                   max_captures         = 1048576U;
    A<gr::Size_t, "max_annotations", Doc<"Largest annotation count this block writes">>                                     max_annotations      = 1048576U;
    A<std::uint64_t, "max_bytes", Doc<"End the recording at this dataset size (0 = unlimited)">>                            max_bytes            = 0U;
    A<gr::Size_t, "meta_rewrite_period", Doc<"Rewrite the metadata file every this many items (0 = off)">>                  meta_rewrite_period  = 0U;

    GR_MAKE_REFLECTABLE(SigMfSink, in, file_name, overwrite, datatype, scaling, sample_rate, num_channels, capture_label, recorder, sigmf_description, author, hardware, license, //
        carry_unknown_fields, emit_annotations, max_captures, max_annotations, max_bytes, meta_rewrite_period);

    // counters — public, monotonic, per run; the stop() line names every non-zero one
    std::uint64_t nSamplesWritten{0U};
    std::uint64_t nSamplesClipped{0U};
    std::uint64_t nCapturesWritten{0U};
    std::uint64_t nAnnotationsWritten{0U};
    std::uint64_t nCapturesDropped{0U};
    std::uint64_t nAnnotationsDropped{0U};
    std::uint64_t nSampleRateChangesIgnored{0U};
    std::uint64_t nDatatypeChangesIgnored{0U};
    std::uint64_t nCarriedRateSuperseded{0U};
    std::uint64_t nMetaKeysOverridden{0U};
    std::uint64_t nMetaKeysDropped{0U};
    std::uint64_t nExtrasUnparsable{0U};

    std::ofstream                      _data{};
    std::string                        _base{};
    std::vector<std::byte>             _staging{};
    gr::sigmf::EncodeFn<Component>     _encode{nullptr};
    gr::sigmf::Datatype                _fileType{};
    bool                               _datatypeFixed{false};
    std::size_t                        _bytesPerItem{0UZ};
    std::uint64_t                      _itemsWritten{0U};
    std::uint64_t                      _droppedTotal{0U};
    bool                               _anyDropSeen{false};
    bool                               _finished{false};
    std::optional<float>               _observedRate{};
    std::optional<gr::Size_t>          _observedChannels{};
    std::optional<double>              _carriedRate{};
    float                              _lastRate{0.f};
    std::uint64_t                      _itemsSinceRewrite{0U};
    bool                               _started{false};
    std::vector<gr::sigmf::Capture>    _captures{};
    std::vector<gr::sigmf::Annotation> _annotations{};
    gr::sigmf::json::Value             _globalExtra{gr::sigmf::json::Value::makeObject()};
    std::string                        _summary{};

    using gr::Block<SigMfSink<T>>::Block;

    void start() {
        resetRunState();
        if (file_name.value.empty()) {
            refuse("no_file_name", "the 'file_name' setting is empty");
        }
        const auto resolved = sigmf_detail::resolveBase(file_name.value);
        if (resolved.isArchive) {
            refuse("archive_unsupported", std::format("'{}' names a .sigmf archive, which this block does not write", file_name.value));
        }
        _base = resolved.base;

        if (datatype.value != "auto") { // an explicit spelling is checked before any file is touched
            auto parsed = gr::sigmf::parseDatatype(datatype.value);
            if (!parsed) {
                refuse("datatype_unknown", std::format("'{}' is not a datatype this block writes", datatype.value));
            }
            fixDatatype(*parsed);
        }

        const std::filesystem::path dataPath = _base + ".sigmf-data";
        const std::filesystem::path metaPath = _base + ".sigmf-meta";
        if (!overwrite) {
            std::error_code status;
            if (std::filesystem::exists(dataPath, status)) {
                refuse("exists", std::format("'{}' exists and 'overwrite' is false", dataPath.string()));
            }
            if (std::filesystem::exists(metaPath, status)) {
                refuse("exists", std::format("'{}' exists and 'overwrite' is false", metaPath.string()));
            }
        }

        std::error_code status;
        if (dataPath.has_parent_path() && !dataPath.parent_path().empty()) {
            std::filesystem::create_directories(dataPath.parent_path(), status);
        }
        _data.open(dataPath, std::ios::binary | std::ios::trunc);
        if (!_data) {
            refuse("path_unwritable", std::format("'{}' could not be opened for writing", dataPath.string()));
        }
        _started = true;
    }

    void stop() {
        if (!_started) { // a refused start leaves nothing behind, so there is nothing to describe
            return;
        }
        _started = false;
        _data.flush();
        _data.close(); // the dataset is complete before the metadata describing it appears
        writeMetadataFile(true);
        reportRun();
        _base.clear();
    }

    [[nodiscard]] gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        if (!_data.is_open()) {
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::ERROR;
        }
        if (_finished) {
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::DONE;
        }
        if (inSpan.size() == 0UZ) {
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        if (!_datatypeFixed) {
            fixDatatypeFromStream(inSpan);
        }

        std::size_t count = inSpan.size();
        if (max_bytes.value != 0U) {
            const std::uint64_t written = _itemsWritten * static_cast<std::uint64_t>(_bytesPerItem);
            const std::uint64_t room    = max_bytes.value > written ? (max_bytes.value - written) / static_cast<std::uint64_t>(_bytesPerItem) : 0U;
            count                       = static_cast<std::size_t>(std::min<std::uint64_t>(count, room));
            if (count == 0UZ) {
                _finished   = true;
                std::ignore = inSpan.consume(0UZ);
                return gr::work::Status::DONE;
            }
        }

        applyTags(inSpan, count);
        ensureFirstCapture();

        const std::size_t wantedBytes = count * _bytesPerItem;
        if (_staging.size() < wantedBytes) {
            _staging.assign(wantedBytes, std::byte{});
        }
        nSamplesClipped += _encode(componentsOf(inSpan.data()), _staging.data(), count * kComponentCount);
        _data.write(reinterpret_cast<const char*>(_staging.data()), static_cast<std::streamsize>(wantedBytes));
        if (!_data) {
            this->emitErrorMessage("SigMfSink::processBulk()", gr::Error(std::format("writing {} bytes failed (disk full or I/O error)", wantedBytes)));
            std::ignore = inSpan.consume(0UZ);
            return gr::work::Status::ERROR;
        }

        _itemsWritten += count;
        nSamplesWritten += count;
        _itemsSinceRewrite += count;
        if (meta_rewrite_period.value != 0U && _itemsSinceRewrite >= static_cast<std::uint64_t>(meta_rewrite_period.value)) {
            _data.flush();
            writeMetadataFile(false);
            _itemsSinceRewrite = 0U;
        }

        std::ignore = inSpan.consume(count);
        return gr::work::Status::OK;
    }

private:
    [[noreturn]] void refuse(std::string_view code, std::string_view detail) const { throw gr::exception(std::format("SigMfSink: {}: {}", code, detail)); }

    void resetRunState() {
        nSamplesWritten           = 0U;
        nSamplesClipped           = 0U;
        nCapturesWritten          = 0U;
        nAnnotationsWritten       = 0U;
        nCapturesDropped          = 0U;
        nAnnotationsDropped       = 0U;
        nSampleRateChangesIgnored = 0U;
        nDatatypeChangesIgnored   = 0U;
        nCarriedRateSuperseded    = 0U;
        nMetaKeysOverridden       = 0U;
        nMetaKeysDropped          = 0U;
        nExtrasUnparsable         = 0U;
        _captures.clear();
        _annotations.clear();
        _globalExtra = gr::sigmf::json::Value::makeObject();
        _staging.clear();
        _encode        = nullptr;
        _fileType      = {};
        _datatypeFixed = false;
        _bytesPerItem  = 0UZ;
        _itemsWritten  = 0U;
        _droppedTotal  = 0U;
        _anyDropSeen   = false;
        _finished      = false;
        _observedRate.reset();
        _observedChannels.reset();
        _carriedRate.reset();
        _lastRate          = 0.f;
        _itemsSinceRewrite = 0U;
        _started           = false;
        _summary.clear();
        _base.clear();
        _data.close();
        _data.clear();
    }

    /// The scalar components of a run of items; a real `T` is its own component type.
    [[nodiscard]] static const Component* componentsOf(const T* items) noexcept {
        if constexpr (std::same_as<Component, T>) {
            return items;
        } else {
            return reinterpret_cast<const Component*>(items);
        }
    }

    [[nodiscard]] gr::sigmf::Scaling scalingMode() const {
        if (scaling.value == "unit") {
            return gr::sigmf::Scaling::Unit;
        }
        if (scaling.value == "raw") {
            return gr::sigmf::Scaling::Raw;
        }
        throw gr::exception(std::format("SigMfSink: unknown scaling '{}', expected \"unit\" or \"raw\"", scaling.value));
    }

    void fixDatatype(const gr::sigmf::Datatype& fileType) {
        if (fileType.domain != gr::sigmf::domainOf<T>()) {
            refuse("datatype_domain", std::format("the datatype is '{}' but the port carries '{}'", gr::sigmf::spellDatatype(fileType), gr::meta::type_name<T>()));
        }
        auto encoder = gr::sigmf::selectEncoder<Component>(fileType, scalingMode());
        if (!encoder) {
            refuse(encoder.error(), std::format("the datatype is '{}' and the port carries '{}'", gr::sigmf::spellDatatype(fileType), gr::meta::type_name<T>()));
        }
        _encode        = *encoder;
        _fileType      = fileType;
        _bytesPerItem  = fileType.bytesPerSample();
        _datatypeFixed = true;
    }

    /// With `datatype = "auto"` the format is taken from the first span's `sigmf_datatype` tag, and
    /// is fixed permanently by that call: a later tag cannot change a file already being written.
    void fixDatatypeFromStream(gr::InputSpanLike auto& inSpan) {
        std::string spelling(gr::sigmf::canonicalDatatypeFor<T>());
        for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
            if (relIndex > 0) {
                continue;
            }
            if (const pmt::Value* carried = sigmf_detail::findTag(tagMapRef.get(), sigmf_keys::kDatatype); carried != nullptr) {
                spelling = carried->value_or(std::string(spelling));
                break;
            }
        }
        auto parsed = gr::sigmf::parseDatatype(spelling);
        if (!parsed || parsed->domain != gr::sigmf::domainOf<T>()) {
            parsed = gr::sigmf::parseDatatype(std::string(gr::sigmf::canonicalDatatypeFor<T>()));
        }
        fixDatatype(*parsed);
    }

    void applyTags(gr::InputSpanLike auto& inSpan, std::size_t count) {
        for (const auto& [relIndex, tagMapRef] : inSpan.tags()) {
            const std::size_t offsetInSpan = relIndex < 0 ? 0UZ : static_cast<std::size_t>(relIndex);
            if (offsetInSpan >= count && offsetInSpan != 0UZ) {
                continue; // a tag past this call's items is presented again with the items it describes
            }
            applyTag(tagMapRef.get(), _itemsWritten + offsetInSpan);
        }
    }

    void applyTag(const property_map& map, std::uint64_t itemIndex) {
        bool opensSegment = false;

        if (const pmt::Value* rate = sigmf_detail::findTag(map, gr::tag::SAMPLE_RATE.shortKey()); rate != nullptr) {
            if (const float* value = rate->get_if<float>(); value != nullptr) {
                if (!_observedRate) {
                    _observedRate = *value;
                    _lastRate     = *value;
                } else if (*value != *_observedRate) {
                    ++nSampleRateChangesIgnored;
                    _lastRate = *value;
                }
            }
        }
        if (const pmt::Value* channels = sigmf_detail::findTag(map, gr::tag::NUM_CHANNELS.shortKey()); channels != nullptr) {
            if (const gr::Size_t* value = channels->get_if<gr::Size_t>(); value != nullptr) {
                if (!_observedChannels) {
                    _observedChannels = *value;
                } else if (*value != *_observedChannels) {
                    ++nMetaKeysOverridden;
                }
            }
        }
        if (const pmt::Value* carried = sigmf_detail::findTag(map, sigmf_keys::kDatatype); carried != nullptr && _datatypeFixed) {
            // a tag arriving after the format was fixed cannot change a file already being written
            if (carried->value_or(std::string("")) != gr::sigmf::spellDatatype(_fileType)) {
                ++nDatatypeChangesIgnored;
            }
        }
        if (const pmt::Value* carried = sigmf_detail::findTag(map, sigmf_keys::kGlobalExtra); carried != nullptr) {
            mergeExtras(carried->value_or(std::string("")), _globalExtra);
        }

        std::optional<double>        frequency;
        std::optional<std::uint64_t> triggerTime;
        std::optional<std::uint64_t> dropped;
        if (const pmt::Value* value = sigmf_detail::findTag(map, gr::tag::FREQUENCY.shortKey()); value != nullptr) {
            if (const double* carried = value->get_if<double>(); carried != nullptr) {
                frequency    = *carried;
                opensSegment = true;
            }
        }
        if (const pmt::Value* value = sigmf_detail::findTag(map, gr::tag::TRIGGER_NAME.shortKey()); value != nullptr) {
            if (value->value_or(std::string("")) == capture_label.value) {
                opensSegment = true;
            }
        }
        if (const pmt::Value* value = sigmf_detail::findTag(map, gr::tag::TRIGGER_TIME.shortKey()); value != nullptr) {
            if (const std::uint64_t* carried = value->get_if<std::uint64_t>(); carried != nullptr) {
                triggerTime = *carried;
            }
        }
        if (const pmt::Value* value = sigmf_detail::findTag(map, gr::tag::N_DROPPED_SAMPLES.shortKey()); value != nullptr) {
            if (const gr::Size_t* carried = value->get_if<gr::Size_t>(); carried != nullptr && *carried > 0U) {
                dropped      = static_cast<std::uint64_t>(*carried);
                opensSegment = true;
            }
        }

        if (opensSegment) {
            gr::sigmf::Capture* segment = openSegment(itemIndex);
            if (segment != nullptr) {
                if (dropped) {
                    _droppedTotal += *dropped;
                    _anyDropSeen         = true;
                    segment->globalIndex = itemIndex + _droppedTotal;
                }
                if (frequency) {
                    segment->frequency = *frequency;
                }
                if (triggerTime) {
                    segment->datetime = gr::sigmf::formatDatetimeNs(*triggerTime);
                }
                if (const pmt::Value* carried = sigmf_detail::findTag(map, sigmf_keys::kCaptureExtra); carried != nullptr) {
                    mergeExtras(carried->value_or(std::string("")), segment->extra);
                }
            }
        }

        if (emit_annotations) {
            appendAnnotation(map, itemIndex);
        }
    }

    /// Two tags at one index that both open a segment open one segment carrying both facts, because
    /// a segment is identified by its index and two segments at one index cannot be ordered.
    [[nodiscard]] gr::sigmf::Capture* openSegment(std::uint64_t itemIndex) {
        if (!_captures.empty() && _captures.back().sampleStart == itemIndex) {
            return &_captures.back();
        }
        if (_captures.size() >= static_cast<std::size_t>(max_captures.value)) {
            ++nCapturesDropped;
            return nullptr;
        }
        gr::sigmf::Capture segment{};
        segment.sampleStart = itemIndex;
        _captures.push_back(std::move(segment));
        return &_captures.back();
    }

    /// This sink always states a segment at the dataset's first sample rather than leaving the array
    /// empty for a reader to infer one. When no tag supplied that segment, its `core:datetime` is
    /// this block's own clock at the first call, which is the one fact this sink invents.
    void ensureFirstCapture() {
        if (!_captures.empty() && _captures.front().sampleStart == 0U) {
            return;
        }
        gr::sigmf::Capture segment{};
        segment.sampleStart = 0U;
        segment.datetime    = gr::sigmf::formatDatetimeNs(sigmf_detail::hostTimeNs());
        _captures.insert(_captures.begin(), std::move(segment));
    }

    void appendAnnotation(const property_map& map, std::uint64_t itemIndex) {
        gr::sigmf::Annotation annotation{};
        bool                  present    = false;
        const auto            readString = [&map, &present](std::string_view key, std::optional<std::string>& slot) {
            if (const pmt::Value* value = sigmf_detail::findTag(map, key); value != nullptr) {
                slot    = value->value_or(std::string(""));
                present = true;
            }
        };
        const auto readDouble = [&map, &present](std::string_view key, std::optional<double>& slot) {
            if (const pmt::Value* value = sigmf_detail::findTag(map, key); value != nullptr) {
                if (const double* carried = value->get_if<double>(); carried != nullptr) {
                    slot    = *carried;
                    present = true;
                }
            }
        };
        readString(sigmf_keys::kAnnotationLabel, annotation.label);
        readString(sigmf_keys::kAnnotationComment, annotation.comment);
        readString(sigmf_keys::kAnnotationGenerator, annotation.generator);
        readString(sigmf_keys::kAnnotationUuid, annotation.uuid);
        readDouble(sigmf_keys::kAnnotationFreqLower, annotation.freqLower);
        readDouble(sigmf_keys::kAnnotationFreqUpper, annotation.freqUpper);
        if (const pmt::Value* value = sigmf_detail::findTag(map, sigmf_keys::kAnnotationLength); value != nullptr) {
            if (const gr::Size_t* carried = value->get_if<gr::Size_t>(); carried != nullptr) {
                annotation.sampleCount = static_cast<std::uint64_t>(*carried);
                present                = true;
            }
        }
        if (const pmt::Value* value = sigmf_detail::findTag(map, sigmf_keys::kAnnotationExtra); value != nullptr) {
            mergeExtras(value->value_or(std::string("")), annotation.extra);
            present = true;
        }
        if (!present) {
            return;
        }
        if (_annotations.size() >= static_cast<std::size_t>(max_annotations.value)) {
            ++nAnnotationsDropped;
            return;
        }
        annotation.sampleStart = itemIndex;
        _annotations.push_back(std::move(annotation));
    }

    void mergeExtras(std::string_view text, gr::sigmf::json::Value& destination) {
        if (!carry_unknown_fields) {
            return;
        }
        if (text.empty()) {
            return;
        }
        const auto document = sigmf_detail::parseExtras(text);
        if (!document) {
            ++nExtrasUnparsable;
            return;
        }
        sigmf_detail::mergeInto(destination, *document);
    }

    /// Compose the metadata document from the accumulated state, on section 8.5's precedence.
    [[nodiscard]] gr::sigmf::Metadata composeMetadata(bool countReconciliation) {
        gr::sigmf::Metadata metadata{};
        metadata.global.datatype = _datatypeFixed ? gr::sigmf::spellDatatype(_fileType) : std::string(gr::sigmf::canonicalDatatypeFor<T>());
        metadata.global.version  = std::string(gr::sigmf::kWrittenVersion);
        metadata.global.recorder = recorder.value;

        const gr::Size_t channels = _observedChannels.value_or(num_channels.value);
        if (channels != 1U) {
            metadata.global.numChannels = static_cast<std::uint64_t>(channels);
        }

        if (const gr::sigmf::json::Value* carried = _globalExtra.find("core:sample_rate"); carried != nullptr && carried->isNumber()) {
            _carriedRate = carried->number().real;
        }
        const float rate = _observedRate.value_or(sample_rate.value);
        if (rate != 0.f) {
            if (_carriedRate && static_cast<float>(*_carriedRate) == rate) {
                metadata.global.sampleRate = *_carriedRate; // the exact original, reproduced
            } else {
                metadata.global.sampleRate = static_cast<double>(rate);
                if (_carriedRate && countReconciliation) {
                    ++nCarriedRateSuperseded;
                }
            }
        }

        applyCarriedGlobal(metadata, countReconciliation);

        if (!sigmf_description.value.empty()) {
            metadata.global.description = sigmf_description.value;
        }
        if (!author.value.empty()) {
            metadata.global.author = author.value;
        }
        if (!hardware.value.empty()) {
            metadata.global.hardware = hardware.value;
        }
        if (!license.value.empty()) {
            metadata.global.license = license.value;
        }

        metadata.captures = _captures;
        if (!_anyDropSeen) { // without a drop the recording asserts no acquisition continuity
            for (gr::sigmf::Capture& segment : metadata.captures) {
                segment.globalIndex.reset();
            }
        } else {
            for (gr::sigmf::Capture& segment : metadata.captures) {
                if (!segment.globalIndex) {
                    segment.globalIndex = segment.sampleStart;
                }
            }
        }
        for (gr::sigmf::Capture& segment : metadata.captures) {
            liftCarried(segment.extra, gr::sigmf::detail::kCaptureKnownKeys, segment, countReconciliation);
        }
        metadata.annotations = _annotations;
        for (gr::sigmf::Annotation& annotation : metadata.annotations) {
            dropKnown(annotation.extra, gr::sigmf::detail::kAnnotationKnownKeys, countReconciliation);
        }
        return metadata;
    }

    /// Distribute the carried global document: a derived field wins, a never-written field is
    /// dropped, an operator setting wins when it is non-empty, and everything else is written back.
    void applyCarriedGlobal(gr::sigmf::Metadata& metadata, bool countReconciliation) {
        static constexpr std::array<std::string_view, 5> kDerived{"core:datatype", "core:version", "core:num_channels", "core:offset", "core:recorder"};
        static constexpr std::array<std::string_view, 4> kNeverWritten{"core:sha512", "core:dataset", "core:trailing_bytes", "core:metadata_only"};

        gr::sigmf::json::Value remaining = gr::sigmf::json::Value::makeObject();
        for (std::size_t i = 0UZ; i < _globalExtra.size(); ++i) {
            const std::string&            key   = _globalExtra.keyAt(i);
            const gr::sigmf::json::Value& value = _globalExtra.valueAt(i);
            if (std::ranges::find(kDerived, key) != kDerived.end()) {
                if (countReconciliation) {
                    ++nMetaKeysOverridden;
                }
                continue;
            }
            if (std::ranges::find(kNeverWritten, key) != kNeverWritten.end()) {
                if (countReconciliation) {
                    ++nMetaKeysDropped;
                }
                continue;
            }
            if (key == "core:sample_rate") {
                continue; // reconciled against the observed rate before this point
            }
            if (key == "core:description" && value.isString()) {
                metadata.global.description = value.str();
                continue;
            }
            if (key == "core:author" && value.isString()) {
                metadata.global.author = value.str();
                continue;
            }
            if (key == "core:hw" && value.isString()) {
                metadata.global.hardware = value.str();
                continue;
            }
            if (key == "core:license" && value.isString()) {
                metadata.global.license = value.str();
                continue;
            }
            remaining.append(key, value);
        }
        metadata.global.extra = std::move(remaining);
    }

    /// Lift a carried capture field into its typed slot when the stream supplied none; a carried key
    /// this block derives loses to the derived value and is counted.
    void liftCarried(gr::sigmf::json::Value& extra, std::span<const std::string_view> known, gr::sigmf::Capture& segment, bool countReconciliation) {
        gr::sigmf::json::Value remaining = gr::sigmf::json::Value::makeObject();
        for (std::size_t i = 0UZ; i < extra.size(); ++i) {
            const std::string&            key   = extra.keyAt(i);
            const gr::sigmf::json::Value& value = extra.valueAt(i);
            if (std::ranges::find(known, key) == known.end()) {
                remaining.append(key, value);
                continue;
            }
            if (key == "core:frequency" && !segment.frequency && value.isNumber()) {
                segment.frequency = value.number().real;
                continue;
            }
            if (key == "core:datetime" && !segment.datetime && value.isString()) {
                segment.datetime = value.str();
                continue;
            }
            if (countReconciliation) {
                ++nMetaKeysOverridden;
            }
        }
        extra = std::move(remaining);
    }

    void dropKnown(gr::sigmf::json::Value& extra, std::span<const std::string_view> known, bool countReconciliation) {
        gr::sigmf::json::Value remaining = gr::sigmf::json::Value::makeObject();
        for (std::size_t i = 0UZ; i < extra.size(); ++i) {
            if (std::ranges::find(known, extra.keyAt(i)) == known.end()) {
                remaining.append(extra.keyAt(i), extra.valueAt(i));
                continue;
            }
            if (countReconciliation) {
                ++nMetaKeysOverridden;
            }
        }
        extra = std::move(remaining);
    }

    void writeMetadataFile(bool isFinal) {
        if (_base.empty()) {
            return;
        }
        if (isFinal) {
            ensureFirstCapture();
        } else if (_captures.empty()) {
            return;
        }
        const gr::sigmf::Metadata metadata = composeMetadata(isFinal);
        auto                      text     = gr::sigmf::write(metadata);
        if (!text) {
            if (isFinal) {
                refuse(text.error().code, text.error().message);
            }
            return;
        }
        std::ofstream metaStream(_base + ".sigmf-meta", std::ios::binary | std::ios::trunc);
        if (!metaStream) {
            if (isFinal) {
                refuse("path_unwritable", std::format("'{}.sigmf-meta' could not be opened for writing", _base));
            }
            return;
        }
        metaStream.write(text->data(), static_cast<std::streamsize>(text->size()));
        metaStream.flush();
        if (isFinal) {
            nCapturesWritten    = metadata.captures.size();
            nAnnotationsWritten = metadata.annotations.size();
        }
    }

    void reportRun() {
        std::string line = std::format("SigMfSink '{}': {} items written, {} capture segments, {} annotations", this->name, nSamplesWritten, nCapturesWritten, nAnnotationsWritten);
        const auto  note = [&line](std::string_view text) { line.append(text); };
        if (nSamplesClipped != 0U) {
            note(std::format(", {} components saturated", nSamplesClipped));
        }
        if (nCapturesDropped != 0U) {
            note(std::format(", {} capture segments dropped at the bound", nCapturesDropped));
        }
        if (nAnnotationsDropped != 0U) {
            note(std::format(", {} annotations dropped at the bound", nAnnotationsDropped));
        }
        if (nSampleRateChangesIgnored != 0U) {
            note(std::format(", {} sample-rate changes ignored (kept {}, last seen {})", nSampleRateChangesIgnored, static_cast<double>(_observedRate.value_or(0.f)), static_cast<double>(_lastRate)));
        }
        if (nDatatypeChangesIgnored != 0U) {
            note(std::format(", {} datatype changes ignored", nDatatypeChangesIgnored));
        }
        if (nCarriedRateSuperseded != 0U) {
            note(", the carried exact sample rate lost to the observed one");
        }
        if (nMetaKeysOverridden != 0U) {
            note(std::format(", {} carried keys a derived field displaced", nMetaKeysOverridden));
        }
        if (nMetaKeysDropped != 0U) {
            note(std::format(", {} carried keys this sink never writes", nMetaKeysDropped));
        }
        if (nExtrasUnparsable != 0U) {
            note(std::format(", {} carried documents were not a JSON object", nExtrasUnparsable));
        }
        _summary = std::move(line);
        std::println("{}", _summary);
    }
};

static_assert(gr::BlockLike<SigMfSink<std::complex<float>>>);

} // namespace gr::blocks::fileio

#endif // GNURADIO_FILEIO_SIGMF_IO_HPP
