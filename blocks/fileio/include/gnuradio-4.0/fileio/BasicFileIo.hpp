#ifndef BASICFILEIO_HPP
#define BASICFILEIO_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/fileio/FileIo.hpp>
#include <gnuradio-4.0/fileio/FileIoTypes.hpp>
#include <gnuradio-4.0/meta/formatter.hpp>
#include <magic_enum.hpp>

#include <chrono>
#include <complex>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <span>
#include <string_view>

namespace gr::blocks::fileio {

namespace detail {

// the regular files of the file name's directory whose names hold its file name, sorted by path
inline std::vector<std::filesystem::path> getSortedFilesContaining(const std::string& fileName) {
    const std::filesystem::path filePath(fileName);
    const std::filesystem::path directory = parentDirectory(filePath);
    const std::string           baseName  = filePath.filename().string();

    std::error_code                     listError;
    std::filesystem::directory_iterator entries(directory, listError);
    if (listError) {
        throw gr::exception(std::format("cannot list directory '{}' for the file names containing '{}': {}", directory.string(), baseName, listError.message()));
    }

    std::vector<std::filesystem::path> matchingFiles;
    std::copy_if(entries, std::filesystem::directory_iterator{}, std::back_inserter(matchingFiles), //
        [&](const auto& entry) { return entry.is_regular_file() && entry.path().filename().string().contains(baseName); });

    std::sort(matchingFiles.begin(), matchingFiles.end());
    return matchingFiles;
}

[[nodiscard]] inline std::uintmax_t getFileSize(const std::filesystem::path& filePath) {
    if (!std::filesystem::exists(filePath)) {
        throw std::runtime_error(std::format("file '{}' does not exist.", filePath.string()));
    }
    return std::filesystem::file_size(filePath);
}

// Deletes the regular files of the file name's directory whose names hold its file name. A directory that cannot be
// listed counts as empty.
[[maybe_unused]] inline std::vector<std::string> deleteFilesContaining(const std::string& fileName) {
    const std::filesystem::path         filePath(fileName);
    std::error_code                     listError;
    std::filesystem::directory_iterator entries(parentDirectory(filePath), listError);
    if (listError) {
        return {};
    }

    std::vector<std::string> deletedFiles;
    for (const auto& entry : entries) {
        if (entry.is_regular_file() && entry.path().filename().string().contains(filePath.filename().string())) {
            deletedFiles.push_back(entry.path().string());
            std::filesystem::remove(entry.path());
        }
    }

    return deletedFiles;
}

} // namespace detail

GR_REGISTER_BLOCK(gr::blocks::fileio::BasicFileSink, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct BasicFileSink : Block<BasicFileSink<T>> {
    using Description = Doc<R""(A sink block for writing a stream to a binary file.
The file can be played back using a 'BasicFileSource' or read by any program that supports binary files (e.g. Python, C, C++, MATLAB).
For complex types, the binary file contains [float, double]s in IQIQIQ order. No metadata is included with the binary data.)
Important: this implementation assumes a host-order, CPU architecture specific byte order!)"">;
    template<typename U, typename... Arguments>
    using A = gr::Annotated<U, Arguments...>; // optional shortening

    PortIn<T> in;

    struct Settings {
        A<std::string, Doc<"base file name">, Visible>                               file_name;
        A<Mode, Doc<"file mode">, Visible>                                           mode               = Mode::overwrite;
        A<gr::Size_t, Unit<"byte">, Doc<"file size limit (0 = unlimited)">, Visible> max_bytes_per_file = 0U;

        GR_MAKE_REFLECTABLE(Settings, file_name, mode, max_bytes_per_file);
    } settings;

    GR_MAKE_REFLECTABLE(BasicFileSink, in, settings);

    std::size_t _totalBytesWritten{0UZ};
    std::size_t _totalBytesWrittenFile{0UZ};
    std::size_t _fileCounter{0UZ};
    std::string _actualFileName;

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.file_name.value.empty()) {
            return gr::Refused(s.file_name, "required");
        }
        // only Mode::multi has a next file to rotate to: in overwrite mode the cap would truncate the
        // recording in place and in append mode it would do nothing, so reject the combination outright
        if (s.max_bytes_per_file != 0U && s.mode != Mode::multi) {
            return gr::Refused(s.max_bytes_per_file, "a nonzero limit requires mode multi");
        }
        return {};
    }

    // A running sink starts the next file when its name or mode changes. When that file cannot be created, the sink
    // keeps writing the file it had and fails the change.
    void applySettings(const Settings& s) {
        if (_actualFileName.empty()) {
            return; // no file is open: start() opens one
        }
        const bool reopen = s.file_name.changed() || s.mode.changed();
        if (!reopen) {
            return;
        }
        const std::string previousFile         = _actualFileName;
        const std::size_t previousBytesWritten = _totalBytesWrittenFile;
        try {
            openNextFile();
        } catch (const std::exception& error) {
            _actualFileName        = previousFile;
            _totalBytesWrittenFile = previousBytesWritten;
            this->fail(s.file_name, std::format("cannot open the next file: {}", error.what()));
        }
    }

    void start() {
        _totalBytesWritten = 0UZ;
        openNextFile();
    }

    void stop() { closeFile(); }

    [[nodiscard]] constexpr work::Status processBulk(InputSpanLike auto& dataIn) {
        if (dataIn.empty()) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        if (settings.max_bytes_per_file.value != 0U && _totalBytesWrittenFile >= settings.max_bytes_per_file.value) {
            closeFile();
            openNextFile();
        }

        std::size_t nBytesMax = dataIn.size() * sizeof(T);
        if (settings.max_bytes_per_file.value != 0U) {
            nBytesMax = std::min(nBytesMax, static_cast<std::size_t>(settings.max_bytes_per_file.value) - _totalBytesWrittenFile);
        }
        const auto bytes          = std::span<const std::uint8_t>(reinterpret_cast<const std::uint8_t*>(dataIn.data()), nBytesMax);
        auto       writeResultExp = gr::algorithm::fileio::write(_actualFileName, bytes, gr::algorithm::fileio::WriterConfig{.mode = currentWriteMode()});

        if (!writeResultExp.has_value()) {
            throw gr::exception(writeResultExp.error().message, writeResultExp.error().sourceLocation);
        }
        if (!dataIn.consume(nBytesMax / sizeof(T))) {
            throw gr::exception("could not consume input samples");
        }

        _totalBytesWritten += nBytesMax;
        _totalBytesWrittenFile += nBytesMax;

        return work::Status::OK;
    }

private:
    [[nodiscard]] gr::algorithm::fileio::WriteMode currentWriteMode() const { return _totalBytesWrittenFile == 0UZ && settings.mode.value != Mode::append ? gr::algorithm::fileio::WriteMode::overwrite : gr::algorithm::fileio::WriteMode::append; }

    void closeFile() { _actualFileName.clear(); }

    void openNextFile() {
        closeFile();
        _totalBytesWrittenFile = 0UZ;

        detail::ensureDirectoryExists(settings.file_name.value);

        std::filesystem::path filePath(settings.file_name.value);
        // Open file handle based on mode
        switch (settings.mode) {
        case Mode::overwrite: {
            _actualFileName = settings.file_name.value;
        } break;
        case Mode::append: {
            _actualFileName = settings.file_name.value;
        } break;
        case Mode::multi: {
            // _fileCounter ensures that the filenames are unique and still sortable by date-time, with an additional counter to handle rapid successive file creation.
            _actualFileName = (filePath.parent_path() / (gr::time::getIsoTime() + "_" + std::to_string(_fileCounter++) + "_" + filePath.filename().string())).string();
            break;
        }
        default: throw gr::exception("unsupported file mode.");
        }

        auto createResultExp = gr::algorithm::fileio::write(_actualFileName, std::span<const std::uint8_t>{}, gr::algorithm::fileio::WriterConfig{.mode = settings.mode.value == Mode::append ? gr::algorithm::fileio::WriteMode::append : gr::algorithm::fileio::WriteMode::overwrite});
        if (!createResultExp.has_value()) {
            throw gr::exception(createResultExp.error().message, createResultExp.error().sourceLocation);
        }
    }
};

GR_REGISTER_BLOCK(gr::blocks::fileio::BasicFileSource, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
struct BasicFileSource : Block<BasicFileSource<T>> {
    using Description = Doc<R""(A source block for reading a binary file and outputting the data.
This source is the counterpart to 'BasicFileSink'.
For complex types, the binary file contains [float, double]s in IQIQIQ order. No metadata is expected in the binary data.
Important: this implementation assumes a host-order, CPU architecture specific byte order!

With `repeat`, the files are read again back to back. `trigger name` is added to each file chunk.
)"">;

    template<typename U, typename... Arguments>
    using A = gr::Annotated<U, Arguments...>; // optional shortening

    PortOut<T> out;

    struct Settings {
        A<std::string, Doc<"base file name">, Visible, StoppedOnly>  file_name;
        A<Mode, Doc<"file mode">, Visible, StoppedOnly>              mode         = Mode::overwrite;
        A<bool, Doc<"repetition">>                                   repeat       = false;
        A<gr::Size_t, Unit<"samples">, Doc<"start offset">, Visible> offset       = 0U;
        A<gr::Size_t, Doc<"sample limit (0 = unlimited)">, Visible>  length       = 0U;
        A<std::string, Doc<"file chunk trigger name">>               trigger_name = "BasicFileSource::start";

        GR_MAKE_REFLECTABLE(Settings, file_name, mode, repeat, offset, length, trigger_name);
    } settings;

    GR_MAKE_REFLECTABLE(BasicFileSource, out, settings);

    gr::algorithm::fileio::Reader             _reader;
    std::set<std::filesystem::path>           _filesToRead; // sorted by path, so a multi-mode set plays in name order
    std::set<std::filesystem::path>::iterator _nextFile{_filesToRead.end()};
    bool                                      _emittedStartTrigger = false;
    bool                                      _readerActive        = false;
    std::size_t                               _totalBytesRead      = 0UZ;
    std::size_t                               _totalBytesReadFile  = 0UZ;

    ~BasicFileSource() {
        // cancel before ~Block() runs — derived members are destroyed before the CRTP base destructor
        _reader.cancel();
        if (lifecycle::isActive(this->state())) {
            std::ignore = this->changeStateTo(lifecycle::State::REQUESTED_STOP);
            std::ignore = this->changeStateTo(lifecycle::State::STOPPED);
        }
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.file_name.value.empty()) {
            return gr::Refused(s.file_name, "required");
        }
        return {};
    }

    // Throws when there is nothing to read. That is when the reader cannot open the file, or, in multi mode, when the
    // directory cannot be listed or no file name in it holds the base name.
    void start() {
        _totalBytesRead = 0UZ;
        _filesToRead.clear();
        _reader       = {};
        _readerActive = false;

        std::filesystem::path filePath(settings.file_name.value);
        switch (settings.mode) {
        case Mode::overwrite:
        case Mode::append: {
            _filesToRead.insert(filePath);
        } break;
        case Mode::multi: {
            const auto matchingFiles = detail::getSortedFilesContaining(settings.file_name.value);
            if (matchingFiles.empty()) {
                throw gr::exception(std::format("no file in '{}' has a name containing '{}'", detail::parentDirectory(filePath).string(), filePath.filename().string()));
            }
            _filesToRead.insert(matchingFiles.begin(), matchingFiles.end());
        } break;
        default: throw gr::exception("unsupported file mode.");
        }
        _nextFile = _filesToRead.begin();

        openNextFile();
        awaitFirstRead();
    }

    void stop() { closeFile(); }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& dataOut) {
        if (dataOut.empty()) {
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }

        std::size_t nSamplesToPublish = 0UZ;
        if (!_readerActive) {
            dataOut.publish(nSamplesToPublish);
            return work::Status::DONE;
        }
        const std::size_t nOutAvailable = dataOut.size() * sizeof(T);
        if (settings.length.value != 0U) {
            const std::size_t totalBytesToRead = static_cast<std::size_t>(settings.length.value) * sizeof(T);
            if (_totalBytesReadFile >= totalBytesToRead) {
                dataOut.publish(nSamplesToPublish);
                return finishCurrentFile();
            }
        }

        std::optional<gr::Error>   error;
        std::optional<std::size_t> requiredOutputSize;
        bool                       finished = false;

        _reader.poll(
            [&](const auto& res) {
                finished           = res.isFinal;
                requiredOutputSize = res.requiredOutputSize;

                if (res.data.has_value()) {
                    const auto bytes = res.data.value();
                    if (bytes.empty()) {
                        return;
                    }

                    std::size_t bytesToPublish = bytes.size();
                    if (settings.length.value != 0U) {
                        const std::size_t totalBytesToRead = static_cast<std::size_t>(settings.length.value) * sizeof(T);
                        bytesToPublish                     = std::min(bytesToPublish, totalBytesToRead - _totalBytesReadFile);
                    }
                    bytesToPublish -= bytesToPublish % sizeof(T);

                    if (bytesToPublish == 0U) {
                        return;
                    }

                    std::memcpy(dataOut.data(), bytes.data(), bytesToPublish);
                    nSamplesToPublish = bytesToPublish / sizeof(T);
                    _totalBytesRead += bytesToPublish;
                    _totalBytesReadFile += bytesToPublish;
                    return;
                }

                error = res.data.error();
            },
            nOutAvailable, true);

        if (requiredOutputSize.has_value()) {
            dataOut.publish(nSamplesToPublish);
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }
        if (error.has_value()) {
            throw gr::exception(error->message, error->sourceLocation);
        }

        if (nSamplesToPublish > 0U && !_emittedStartTrigger && !settings.trigger_name.value.empty()) {
            dataOut.publishTag(
                property_map{
                    {std::pmr::string(tag::TRIGGER_NAME.key()), settings.trigger_name.value},                                                                                                                   //
                    {std::pmr::string(tag::TRIGGER_TIME.key()), static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count())}, //
                    {std::pmr::string(tag::TRIGGER_OFFSET.key()), 0.f}                                                                                                                                          //
                },
                0UZ);
            _emittedStartTrigger = true;
        }
        dataOut.publish(nSamplesToPublish);

        if (finished || (settings.length.value != 0U && _totalBytesReadFile >= static_cast<std::size_t>(settings.length.value) * sizeof(T))) {
            return finishCurrentFile();
        }

        return work::Status::OK;
    }

private:
    void closeFile() {
        if (_readerActive) {
            _reader.cancel();
        }
        _reader       = {};
        _readerActive = false;
    }

    void openNextFile() {
        if (_nextFile == _filesToRead.end()) {
            return;
        }
        _totalBytesReadFile  = 0UZ;
        _emittedStartTrigger = false;

        // the reader hands one chunk to one processBulk call, so the chunk must fit an output span:
        // size it from the actual ring, capped so a large ring does not turn into a whole-file read
        constexpr std::size_t               kMaxChunkSamples = 64UZ * 1024UZ / sizeof(T);
        gr::algorithm::fileio::ReaderConfig config;
        std::size_t                         chunkSamples = std::clamp(out.bufferSize(), 1UZ, kMaxChunkSamples);
        if (settings.length.value != 0U) {
            chunkSamples = std::min(chunkSamples, static_cast<std::size_t>(settings.length.value));
        }
        const std::size_t chunkBytes = chunkSamples * sizeof(T);

        config.offset              = static_cast<std::size_t>(settings.offset.value) * sizeof(T);
        config.chunkBytes          = chunkBytes;
        config.chunkAlignmentBytes = sizeof(T);

        auto readerExp = gr::algorithm::fileio::readAsync(_nextFile->string(), std::move(config));
        if (!readerExp.has_value()) {
            throw gr::exception(readerExp.error().message, readerExp.error().sourceLocation);
        }
        _reader       = std::move(readerExp.value());
        _readerActive = true;
        ++_nextFile;
    }

    // Waits for the reader's first message, which comes after the reader opens the file. An error refuses the start.
    // A chunk stays queued for processBulk(), since this poll accepts no data. At the end of an empty file the block
    // finishes that file and opens the next one, if any.
    void awaitFirstRead() {
        std::optional<gr::Error> error;
        bool                     finished = false;
        _reader.poll(
            [&](const auto& res) {
                finished = res.isFinal;
                if (!res.data.has_value() && !res.requiredOutputSize.has_value()) {
                    error = res.data.error();
                }
            },
            0UZ, true);
        if (error.has_value()) {
            throw gr::exception(error->message, error->sourceLocation);
        }
        if (finished) {
            std::ignore = finishCurrentFile();
        }
    }

    [[nodiscard]] work::Status finishCurrentFile() {
        closeFile();
        if (_nextFile != _filesToRead.end()) {
            openNextFile();
            return work::Status::OK;
        }
        if (settings.repeat && !_filesToRead.empty()) {
            _nextFile = _filesToRead.begin();
            openNextFile();
            return work::Status::OK;
        }
        return work::Status::DONE;
    }
};

} // namespace gr::blocks::fileio

#endif // BASICFILEIO_HPP
