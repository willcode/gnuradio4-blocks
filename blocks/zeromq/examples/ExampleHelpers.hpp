#pragma once

#include <chrono>
#include <cstddef>
#include <format>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>

namespace gr::blocks::zeromq::examples {

template<typename T>
struct SequenceSource : gr::Block<SequenceSource<T>> {
    gr::PortOut<T> out;

    struct Settings {
        std::vector<T> values;
        gr::Size_t     startup_delay_ms = 100;
        bool           repeat           = false;

        GR_MAKE_REFLECTABLE(Settings, values, startup_delay_ms, repeat);
    } settings;

    std::size_t index   = 0;
    bool        started = false;

    GR_MAKE_REFLECTABLE(SequenceSource, out, settings);

    [[nodiscard]] T processOne() {
        if (!started) {
            started = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(settings.startup_delay_ms));
        }
        if (settings.values.empty()) {
            this->requestStop();
            return T{};
        }
        if (index >= settings.values.size()) {
            if (!settings.repeat) {
                this->requestStop();
                return T{};
            }
            index = 0;
        }
        return settings.values[index++];
    }
};

template<typename T>
struct PrintingSink : gr::Block<PrintingSink<T>> {
    gr::PortIn<T> in;

    struct Settings {
        gr::Size_t  n_items_max = 16;
        std::string label       = "recv";

        GR_MAKE_REFLECTABLE(Settings, n_items_max, label);
    } settings;

    std::size_t count = 0;

    GR_MAKE_REFLECTABLE(PrintingSink, in, settings);

    void processOne(const T& value) {
        std::cout << settings.label << "[" << count << "] = " << value << '\n';
        ++count;
        if (settings.n_items_max > 0 && count >= settings.n_items_max) {
            this->requestStop();
        }
    }
};

template<typename SchedulerT>
void run_graph(gr::Graph&& graph) {
    SchedulerT scheduler;
    if (auto ret = scheduler.exchange(std::move(graph)); !ret) {
        throw std::runtime_error(std::format("failed to initialize scheduler: {}", ret.error()));
    }
    if (auto ret = scheduler.runAndWait(); !ret) {
        throw std::runtime_error(std::format("scheduler error: {}", ret.error()));
    }
}

inline void require_role(int argc, char** argv, std::string_view usage) {
    (void)argv;
    if (argc < 2) {
        throw std::runtime_error(std::string(usage));
    }
}

} // namespace gr::blocks::zeromq::examples
