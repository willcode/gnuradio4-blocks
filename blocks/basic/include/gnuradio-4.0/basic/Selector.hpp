#ifndef GNURADIO_SELECTOR_HPP
#define GNURADIO_SELECTOR_HPP

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>

#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::basic {
using namespace gr;

GR_REGISTER_BLOCK(gr::blocks::basic::Selector, [T], [ uint8_t, int16_t, int32_t, float ])

template<typename T>
struct Selector : Block<Selector<T>, NoTagPropagation> {
    using Description = Doc<R""(@brief basic multiplexing class to route arbitrary inputs to outputs

See https://wiki.gnuradio.org/index.php/Selector

The selector block allows arbitrary mapping between the input and output ports.

Selector has an arbitrary number of input and output ports, defined by
the `n_inputs` and `n_outputs` properties.

The mapping is defined by a pair of vectors `map_in` and `map_out`, the
corresponding indices from these two vectors define the
(input port index, output port index pairs).

For example, for arrays:

    map_in  = {2, 2, 3, 3}
    map_out = {1, 2, 3, 4}

The mapping is as follows:

     +--------------+
    -|-1   /----> 1-|-
    -|-2 >------> 2-|-
    -|-3 >------> 3-|-
    -|-4   \----> 4-|-
     +--------------+

And for arrays:

    map_in  = {1, 2, 3, 4}
    map_out = {2, 2, 3, 3}

The mapping is as follows:

     +--------------+
    -|-1 >-----\  1-|-
    -|-2 >------> 2-|-
    -|-3 >------> 3-|-
    -|-4 >-----/  4-|-
     +--------------+


When multiple input ports are mapped to a single output port, their samples are synchronized.
This means that all input ports are aligned to have the same number of samples.
It is assumed that these ports should have the same sample rate; otherwise, the buffer may fill up quickly.

Example: Assuming there are 3 input ports with sample counts of 1, 2, and 3, and these are mapped to one output, the output would look like:
`1, 2, 3, 1, 2, 3, 1, 2, 3...`

If you want to change this behavior, set `sync_combined_ports = false;`.
In this case, samples are taken in order from each input, resulting in a sequence that copies samples sequentially from the first input, then the second, and so on.


It also contains two additional ports -- `selectOut` and `monitorOut`. Port
`selectOut` which can be used to define which input port is passed on to the
`monitorOut` port.

For uses where all input ports should be read even if they are not connected
to any output port (thus reading and ignoring all the values from the input),
`back_pressure` false reads them.
)"">;
    // optional shortening
    template<typename U, typename... Arguments>
    using A = Annotated<U, Arguments...>;

    PortIn<gr::Size_t, Async, Optional> select{};  // optional select input
    PortOut<T, Async, Optional>         monitor{}; // optional monitor output (for diagnostics and debugging purposes)
    std::vector<PortIn<T, Async>>       inputs{};  // TODO: need to add exception to pmt_t that this isn't interpreted as a settings type
    std::vector<PortOut<T, Async>>      outputs{};

    struct Settings {
        A<gr::Size_t, Visible, Doc<"input count">, Limits<Max<32U>>, PortCountFor<"inputs">>   n_inputs  = 0U;
        A<gr::Size_t, Visible, Doc<"output count">, Limits<Max<32U>>, PortCountFor<"outputs">> n_outputs = 0U;
        A<Tensor<gr::Size_t>, Visible, Doc<"source input indices">>                            map_in{}; // N.B. need two vectors since pmt_t doesn't support pairs (yet!?!)
        A<Tensor<gr::Size_t>, Visible, Doc<"destination output indices">>                      map_out{};
        A<bool, Visible, Doc<"unrouted input held">>                                           back_pressure       = false;
        A<bool, Doc<"routed inputs synchronized">>                                             sync_combined_ports = true;

        GR_MAKE_REFLECTABLE(Settings, n_inputs, n_outputs, map_in, map_out, back_pressure, sync_combined_ports);
    } settings;

    GR_MAKE_REFLECTABLE(Selector, select, inputs, monitor, outputs, settings);

    std::map<std::size_t, std::vector<std::size_t>> _internalMappingInOut{};
    std::map<std::size_t, std::vector<std::size_t>> _internalMappingOutIn{};

    std::size_t _selectedSrc = 0UZ;

    // The routing depends on the port counts as much as on the maps, so the check reads all four together, and a
    // refused map or port count leaves the ports and the routing unchanged. An index past a port count is reported
    // against the map, the setting a host most likely corrects; the text names the count.
    std::optional<gr::Refused> checkSettings(const Settings& s) const {
        const Tensor<gr::Size_t>& mapIn    = s.map_in;
        const Tensor<gr::Size_t>& mapOut   = s.map_out;
        const gr::Size_t          nInputs  = s.n_inputs;
        const gr::Size_t          nOutputs = s.n_outputs;
        if (mapIn.size() != mapOut.size()) {
            return gr::Refused(s.map_out, "map_in and map_out must have the same length");
        }
        std::set<std::pair<gr::Size_t, gr::Size_t>> pairs{};
        for (std::size_t i = 0UZ; i < mapIn.size(); ++i) {
            if (!pairs.insert({mapIn[i], mapOut[i]}).second) {
                return gr::Refused(s.map_in, "each (map_in, map_out) pair must be unique");
            }
            if (mapIn[i] >= nInputs) {
                return gr::Refused(s.map_in, "each index must be < n_inputs");
            }
            if (mapOut[i] >= nOutputs) {
                return gr::Refused(s.map_out, "each index must be < n_outputs");
            }
        }
        return {};
    }

    void applySettings(const Settings& s) {
        if (s.n_inputs.changed() || s.n_outputs.changed()) {
            std::print("{}: configuration changed: n_inputs {}, n_outputs {}\n", this->name, s.n_inputs, s.n_outputs);
        }
        inputs.resize(s.n_inputs);
        outputs.resize(s.n_outputs);
        _internalMappingInOut = routesOf(s.map_in, s.map_out);
        _internalMappingOutIn = routesOf(s.map_out, s.map_in);
    }

    // for each port index `from` names, the indices `to` pairs it with, in map order; for maps the check accepts
    [[nodiscard]] static std::map<std::size_t, std::vector<std::size_t>> routesOf(const Tensor<gr::Size_t>& from, const Tensor<gr::Size_t>& to) {
        std::map<std::size_t, std::vector<std::size_t>> routes;
        for (std::size_t i = 0UZ; i < from.size(); ++i) {
            routes[from[i]].push_back(to[i]);
        }
        return routes;
    }

    template<gr::InputSpanLike TInSpan, gr::OutputSpanLike TOutSpan>
    gr::work::Status processBulk(InputSpanLike auto& selectSpan, std::span<TInSpan>& ins, OutputSpanLike auto& monOut, std::span<TOutSpan>& outs) {
        if (_internalMappingInOut.empty()) {
            std::ranges::for_each(ins, [this](auto& input) { std::ignore = input.consume(settings.back_pressure ? 0UZ : input.size()); });
            return work::Status::OK;
        }

        if (const auto selectAvailable = selectSpan.size(); selectAvailable > 0) {
            _selectedSrc = selectSpan.back();
            std::ignore  = selectSpan.consume(selectAvailable); // consume all samples on the 'select' streaming input port
        } else {
            _selectedSrc = std::numeric_limits<std::size_t>::max();
        }

        std::vector<std::size_t> outOffsets(outs.size(), 0UZ);
        auto                     copyToOutput = [&outOffsets](std::size_t nSamplesToCopy, auto& inputSpan, auto& outputSpan, std::size_t outIndex) {
            const auto diffCount  = static_cast<std::ptrdiff_t>(nSamplesToCopy);
            const auto diffOffset = static_cast<std::ptrdiff_t>((outIndex == std::numeric_limits<std::size_t>::max()) ? 0U : outOffsets[outIndex]);

            std::copy_n(inputSpan.begin(), diffCount, std::next(outputSpan.begin(), diffOffset));

            if (outIndex != std::numeric_limits<std::size_t>::max()) {
                outOffsets[outIndex] += nSamplesToCopy;
            }

            const auto tags = inputSpan.tags(); // Tag handling
            for (const auto& [normalisedTagIndex, tagMap] : tags) {
                if (normalisedTagIndex < static_cast<std::ptrdiff_t>(nSamplesToCopy)) {
                    const auto tagOffset = normalisedTagIndex > 0 ? static_cast<std::size_t>(normalisedTagIndex) : 0UZ;
                    outputSpan.publishTag(tagMap.get(), tagOffset + static_cast<std::size_t>(diffOffset));
                }
            }
            outputSpan.publish(nSamplesToCopy);
        };

        std::vector<std::size_t> nSamplesToConsume(ins.size(), std::numeric_limits<std::size_t>::max());
        if (settings.sync_combined_ports && std::ranges::any_of(_internalMappingOutIn, [](const auto& pair) { return pair.second.size() > 1; })) {
            for (const auto& [outIndex, inIndices] : _internalMappingOutIn) {
                std::size_t nInSamplesAvailable = std::ranges::min(inIndices | std::views::transform([&ins](std::size_t i) { return ins[i].size(); }));
                nInSamplesAvailable             = std::min(nInSamplesAvailable, outs[outIndex].size() / inIndices.size());
                if (std::ranges::find(inIndices, _selectedSrc) != inIndices.end()) {
                    nInSamplesAvailable = std::min(nInSamplesAvailable, monOut.size());
                }
                for (const std::size_t inIndex : inIndices) {
                    nSamplesToConsume[inIndex] = std::min(nSamplesToConsume[inIndex], nInSamplesAvailable);
                }
            }

            // we need to iterate until no changes occur to include all dependencies -> Think about alternative approach
            bool changed = true;
            while (changed) {
                changed = false;
                for (const auto& [outIndex, inIndices] : _internalMappingOutIn) {
                    const std::size_t currentMin = std::ranges::min(inIndices | std::views::transform([&nSamplesToConsume](std::size_t i) { return nSamplesToConsume[i]; }));
                    for (const std::size_t inIndex : inIndices) {
                        if (nSamplesToConsume[inIndex] > currentMin) {
                            changed = true;
                        }
                        nSamplesToConsume[inIndex] = currentMin;
                    }
                }
            }

            for (const auto& [outIndex, inIndices] : _internalMappingOutIn) {
                if (inIndices.size() == 1) {
                    const std::size_t inIndex = inIndices[0];
                    copyToOutput(nSamplesToConsume[inIndex], ins[inIndex], outs[outIndex], std::numeric_limits<std::size_t>::max());
                } else if (inIndices.size() > 1) {
                    auto&       outSpan           = outs[outIndex];
                    std::size_t nSamplesToPublish = 0UZ;
                    for (std::size_t iS = 0UZ; iS < nSamplesToConsume[inIndices.front()]; iS++) {
                        for (const std::size_t inIndex : inIndices) {
                            outSpan[nSamplesToPublish] = ins[inIndex][iS];
                            for (const auto& tag : ins[inIndex].rawTags) {
                                const auto relIndex = tag.index >= ins[inIndex].streamIndex ? static_cast<std::ptrdiff_t>(tag.index - ins[inIndex].streamIndex) : -static_cast<std::ptrdiff_t>(ins[inIndex].streamIndex - tag.index);
                                if (relIndex == static_cast<std::ptrdiff_t>(iS)) {
                                    outSpan.publishTag(tag.map, nSamplesToPublish);
                                }
                            }
                            nSamplesToPublish++;
                        }
                    }
                    outSpan.publish(nSamplesToPublish);
                }
            }
        } else {
            for (const auto& [inIndex, outIndices] : _internalMappingInOut) {
                InputSpanLike auto& inSpan         = ins[inIndex];
                std::size_t         monOutSize     = _selectedSrc == inIndex ? monOut.size() : std::numeric_limits<std::size_t>::max();
                std::size_t         nSamplesToCopy = std::min({inSpan.size(), monOutSize, //
                            std::ranges::min(outIndices | std::views::transform([&](std::size_t outIndex) { return outs[outIndex].size() - outOffsets[outIndex]; }))});

                for (const std::size_t outIndex : outIndices) {
                    copyToOutput(nSamplesToCopy, inSpan, outs[outIndex], outIndex);
                }
                nSamplesToConsume[inIndex] = nSamplesToCopy;
            }
        }

        if (_selectedSrc < ins.size()) {
            auto&             inSpan         = ins[_selectedSrc];
            const std::size_t nMappedBudget  = nSamplesToConsume[_selectedSrc];
            const std::size_t nSamplesToCopy = std::min({inSpan.size(), monOut.size(), nMappedBudget});
            copyToOutput(nSamplesToCopy, inSpan, monOut, std::numeric_limits<std::size_t>::max());
            // a source routed to an output had its tags published against that output's budget, so only a
            // source the mapping leaves out takes its consume count from the monitor copy
            if (nMappedBudget == std::numeric_limits<std::size_t>::max()) {
                nSamplesToConsume[_selectedSrc] = nSamplesToCopy;
            }
        }

        for (std::size_t inIndex = 0UZ; inIndex < ins.size(); inIndex++) {
            const std::size_t nBackPressure = settings.back_pressure ? 0UZ : ins[inIndex].size();
            const std::size_t nFinal        = nSamplesToConsume[inIndex] == std::numeric_limits<std::size_t>::max() ? nBackPressure : nSamplesToConsume[inIndex];
            std::ignore                     = ins[inIndex].consume(nFinal);
            ins[inIndex].consumeTags(nFinal);
        }
        return work::Status::OK;
    }
};
} // namespace gr::blocks::basic

#endif // include guard
