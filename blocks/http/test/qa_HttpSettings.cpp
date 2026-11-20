#include <boost/ut.hpp>

#include <chrono>
#include <filesystem>
#include <format>
#include <fstream>
#include <string>
#include <vector>

#include <gnuradio-4.0/http/HttpBlock.hpp>
#include <gnuradio-4.0/testing/InitChecks.hpp>

// The settings of the http blocks, read through local files: these cases need no server, so they run where the HTTP
// cases of qa_HttpBlock do not.
const boost::ut::suite<"http block settings"> httpSettingsTests = [] {
    using namespace boost::ut;

    "the http blocks initialize at each setting"_test = [] {
        using gr::testing::requiredInitFailures;
        const auto report   = [](const std::vector<std::string>& failures) { return std::format("{}", failures); };
        auto       failures = requiredInitFailures<gr::blocks::http::HttpSource>({{"url", std::string("http://127.0.0.1:8080/echo")}}, {{"type", std::string("subscribe")}, {"chunk_bytes", gr::Size_t{64U}}});
        expect(failures.empty()) << report(failures);
        failures = requiredInitFailures<gr::blocks::http::HttpSink>({{"url", std::string("http://127.0.0.1:8080/number")}}, {{"content_type", std::string("text/plain")}});
        expect(failures.empty()) << report(failures);
    };

    "a running HttpSource reads a new url, and keeps its reader when the new one cannot start"_test = [] {
        using gr::lifecycle::State;
        const auto                  stamp     = std::chrono::steady_clock::now().time_since_epoch().count();
        const std::filesystem::path directory = std::filesystem::temp_directory_path();
        const std::filesystem::path first     = directory / std::format("gr4-http-first-{}.txt", stamp);
        const std::filesystem::path second    = directory / std::format("gr4-http-second-{}.txt", stamp);
        std::ofstream(first) << "first";
        std::ofstream(second) << "second";
        const std::string firstUrl  = "file:" + first.string();
        const std::string secondUrl = "file:" + second.string();

        gr::blocks::http::HttpSource source({{"url", firstUrl}});
        expect(fatal(source.check({}).has_value()));
        expect(fatal(source.changeStateTo(State::INITIALISED).has_value() && source.changeStateTo(State::RUNNING).has_value()));

        expect(gr::testing::setAndApply(gr::settings::detail::settingsOf(source), {{"url", secondUrl}}).empty()) << "a url the reader takes applies";
        expect(eq(source.settings.url.value, secondUrl));

        const gr::settings::Rejected failed = gr::testing::setAndApply(gr::settings::detail::settingsOf(source), {{"url", std::string("unknown-scheme://nowhere")}});
        expect(eq(failed.size(), 1UZ));
        expect(!failed.empty() && failed.front().key == "url" && failed.front().reason.contains("cannot read the endpoint")) << (failed.empty() ? std::string{} : failed.front().reason);
        expect(eq(source.get<std::string>("url").value_or(std::string{}), secondUrl)) << "the failed key takes back its value";

        expect(source.changeStateTo(State::REQUESTED_STOP).has_value() && source.changeStateTo(State::STOPPED).has_value());
        std::filesystem::remove(first);
        std::filesystem::remove(second);
    };
};

int main() { /* not needed for UT */ }
