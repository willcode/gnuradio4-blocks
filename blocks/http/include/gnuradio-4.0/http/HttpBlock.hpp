#ifndef GNURADIO_HTTP_BLOCK_HPP
#define GNURADIO_HTTP_BLOCK_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/algorithm/fileio/FileIo.hpp>
#include <gnuradio-4.0/meta/reflection.hpp>

#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#ifdef __GNUC__
#pragma GCC diagnostic push
#ifndef __clang__
#pragma GCC diagnostic ignored "-Wuseless-cast"
#endif
#pragma GCC diagnostic ignored "-Wsign-conversion"
#endif
#include <magic_enum.hpp>
#include <magic_enum_utility.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

using namespace gr;

namespace gr::blocks::http {

namespace fileio = gr::algorithm::fileio;

enum class SourceMode : char {
    get       = 1,
    subscribe = 2,
};

GR_REGISTER_BLOCK(gr::blocks::http::HttpSource)
struct HttpSource : Block<HttpSource> {
    using Description = Doc<R""(
Read data from an HTTP endpoint.

GET reads one response.
SUBSCRIBE keeps polling and publishes each new response.

Each output item is a PMT map with:
- status: HTTP status code
- raw-data: response body bytes
- mime-type: response content type

Internally this uses FileIo.
)"">;

    PortOut<pmt::Value::Map> out;

    struct Settings {
        gr::Annotated<std::string>                                                               url;
        gr::Annotated<gr::blocks::http::SourceMode, gr::Doc<"request type">>                     type        = gr::blocks::http::SourceMode::get;
        gr::Annotated<gr::Size_t, gr::Unit<"byte">, gr::Doc<"chunk size limit (0 = unlimited)">> chunk_bytes = 0U;

        GR_MAKE_REFLECTABLE(Settings, url, type, chunk_bytes);
    } settings;

    GR_MAKE_REFLECTABLE(HttpSource, out, settings);

    fileio::Reader _reader;
    bool           _emscriptenRunOnMainThread = true; // used in Emscripten unit-tests only

    ~HttpSource() {
        // cancel before ~Block() runs — derived members are destroyed before the CRTP base destructor
        _reader.cancel();
        if (lifecycle::isActive(this->state())) {
            std::ignore = this->changeStateTo(lifecycle::State::REQUESTED_STOP);
            std::ignore = this->changeStateTo(lifecycle::State::STOPPED);
        }
    }

    [[nodiscard]] static pmt::Value::Map makeResultValue(std::span<const std::uint8_t> rawData, int status = 200, std::string_view mimeType = "text/plain") {
        pmt::Value::Map result;
        result["mime-type"] = std::string(mimeType);
        result["status"]    = status;
        result["raw-data"]  = gr::Tensor<std::uint8_t>(rawData.begin(), rawData.end());
        return result;
    }

    [[nodiscard]] fileio::ReaderConfig readerConfig() const {
        fileio::ReaderConfig config;
        if (settings.chunk_bytes.value != 0U) {
            config.chunkBytes = static_cast<std::size_t>(settings.chunk_bytes.value);
        }
        config.longPolling               = settings.type.value == SourceMode::subscribe;
        config.emscriptenRunOnMainThread = _emscriptenRunOnMainThread;
        return config;
    }

    void openReader() {
        auto readerExp = fileio::readAsync(settings.url.value, readerConfig());
        if (!readerExp.has_value()) {
            throw gr::exception(readerExp.error().message, readerExp.error().sourceLocation);
        }
        _reader = std::move(readerExp.value());
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.url.value.empty()) {
            return gr::Refused(s.url, "required");
        }
        return {};
    }

    void start() { openReader(); }

    void stop() { _reader.cancel(); }

    // A running source reads the new endpoint once a key of the read changes. When the reader cannot start, the source
    // keeps the reader it had and refuses the change.
    void applySettings(const Settings& s) {
        if (!lifecycle::isActive(this->state())) {
            return; // start() opens the reader
        }
        const bool reopen = s.url.changed() || s.type.changed() || s.chunk_bytes.changed();
        if (!reopen) {
            return;
        }
        auto readerExp = fileio::readAsync(s.url.value, readerConfig());
        if (!readerExp.has_value()) { // only the endpoint can tell
            this->fail(s.url, std::format("cannot read the endpoint: {}", readerExp.error().message));
            return;
        }
        _reader.cancel();
        _reader = std::move(readerExp.value());
    }

    [[nodiscard]] work::Status processBulk(OutputSpanLike auto& outSpan) {
        if (outSpan.empty()) {
            return work::Status::INSUFFICIENT_OUTPUT_ITEMS;
        }

        bool                           finished = false;
        std::optional<pmt::Value::Map> result;
        std::optional<gr::Error>       error;
        std::size_t                    nSamplesToPublish = 0U;

        _reader.poll(
            [&](const auto& res) {
                finished = res.isFinal;

                if (res.data.has_value()) {
                    const auto bytes = res.data.value();
                    if (!bytes.empty()) {
                        result            = makeResultValue(bytes);
                        nSamplesToPublish = 1U;
                    }
                    return;
                }
                error = res.data.error();
            },
            std::numeric_limits<std::size_t>::max(), false);

        if (error.has_value()) {
            throw gr::exception(error->message, error->sourceLocation);
        }
        if (result.has_value()) {
            outSpan[0] = std::move(*result);
        }
        outSpan.publish(nSamplesToPublish);
        if (finished) {
            return work::Status::DONE;
        }
        return work::Status::OK;
    }
};

GR_REGISTER_BLOCK(gr::blocks::http::HttpSink)
struct HttpSink : Block<HttpSink> {
    using Description = Doc<R""(
Send incoming bytes to an HTTP endpoint with POST.

Each input chunk is sent as one POST request.
Use content_type to set the Content-Type header.

Internally this uses FileIo.
)"">;

    PortIn<std::uint8_t> in;

    struct Settings {
        gr::Annotated<std::string>                          url;
        gr::Annotated<std::string, gr::Doc<"content type">> content_type = "application/octet-stream";

        GR_MAKE_REFLECTABLE(Settings, url, content_type);
    } settings;

    GR_MAKE_REFLECTABLE(HttpSink, in, settings);

    std::optional<fileio::Writer> _writer;
    bool                          _emscriptenRunOnMainThread = true; // used in Emscripten unit-tests only

    ~HttpSink() {
        if (_writer.has_value()) {
            _writer->cancel();
        }
        if (lifecycle::isActive(this->state())) {
            std::ignore = this->changeStateTo(lifecycle::State::REQUESTED_STOP);
            std::ignore = this->changeStateTo(lifecycle::State::STOPPED);
        }
    }

    [[nodiscard]] fileio::WriterConfig writerConfig() const {
        fileio::WriterConfig config;
        if (!settings.content_type.value.empty()) {
            config.httpHeaders.emplace("Content-Type", settings.content_type.value);
        }
        config.emscriptenRunOnMainThread = _emscriptenRunOnMainThread;
        return config;
    }

    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        if (s.url.value.empty()) {
            return gr::Refused(s.url, "required");
        }
        return {};
    }

    void start() { _writer.reset(); }

    void stop() {
        if (_writer.has_value()) {
            _writer->cancel();
        }
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan) {
        if (inSpan.empty()) {
            return work::Status::INSUFFICIENT_INPUT_ITEMS;
        }

        if (_writer.has_value()) {
            if (!_writer->finished()) {
                std::ignore = inSpan.consume(0UZ);
                return work::Status::INSUFFICIENT_INPUT_ITEMS;
            }

            auto writeResultExp = _writer->result();
            _writer.reset();
            if (!writeResultExp.has_value()) {
                throw gr::exception(writeResultExp.error().message, writeResultExp.error().sourceLocation);
            }
        }

        auto bytes     = std::span<const std::uint8_t>(inSpan.data(), inSpan.size());
        auto writerExp = fileio::writeAsync(settings.url.value, bytes, writerConfig());
        if (!writerExp.has_value()) {
            throw gr::exception(writerExp.error().message, writerExp.error().sourceLocation);
        }
        _writer = std::move(writerExp.value());

        return work::Status::OK;
    }
};

} // namespace gr::blocks::http

#endif // GNURADIO_HTTP_BLOCK_HPP
