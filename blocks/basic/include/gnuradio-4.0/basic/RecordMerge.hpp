#ifndef GNURADIO_BASIC_RECORD_MERGE_HPP
#define GNURADIO_BASIC_RECORD_MERGE_HPP

#include <algorithm>
#include <complex>
#include <cstddef>
#include <format>
#include <ranges>
#include <span>
#include <type_traits>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>
#include <gnuradio-4.0/meta/utils.hpp>

namespace gr::blocks::basic {

GR_REGISTER_BLOCK(gr::blocks::basic::RecordMerge, [T], [ uint8_t, int16_t, int32_t, float, std::complex<float> ])

template<typename T>
requires(std::is_arithmetic_v<T> || gr::meta::complex_like<T>)
struct RecordMerge : Block<RecordMerge<T>, NoTagPropagation> {
    using Description = Doc<R""(
@brief Joins the records of `n_inputs` record streams into one, in port order, without dropping or reordering any.

A decoder's tail ends in several record streams that are one stream of events: the frames that passed a check, the
frames that failed it, and the records a framer refused. They are separate ports because each stage states its own
outcome, and each carries the key that says which it is, so a consumer that wants one of them filters rather than
subscribes. Every one of them has to be read: an unread asynchronous output fills its edge and stops the block
writing it, so leaving one unattached stops the chain rather than losing a count.

**Nothing is dropped.** A record is consumed only in the call that publishes it, so a full output span costs the
call and never the record. **Nothing is reordered within an input**: one input's records reach `out` in the order
they arrived. Across inputs the order is the port order of one call, which is the only order a block reading N
independent buffers has: the buffers carry no common clock, and a record states its own time in its own metadata.

Each input's records are counted in `n_records`, in port order, so an input that is silent is distinguishable from
one that is not connected at all. An unconnected input is neither an error nor a stall — it offers nothing and the
asynchronous ports let the block run on whichever inputs do have records.
)"">;

    std::vector<PortIn<DataSet<T>, Async>> inputs;
    PortOut<DataSet<T>, Async>             out;

    Annotated<gr::Size_t, "n_inputs", Visible, Doc<"number of record inputs; fixed once a port is connected">, Limits<1U, 32U>> n_inputs{1U};
    Annotated<std::vector<gr::Size_t>, "n_records", Doc<"read-only: records taken from each input, in port order">>             n_records{};

    GR_MAKE_REFLECTABLE(RecordMerge, inputs, out, n_inputs, n_records);

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void reset() { std::ranges::fill(n_records.value, gr::Size_t{0U}); }

    // `newSettings` names only the settings whose value moved, so the port count is read off the member on every
    // call rather than from the changed set.
    void rebuild() {
        const std::size_t nPorts = static_cast<std::size_t>(n_inputs.value);
        if (inputs.size() != nPorts) {
            if (std::ranges::any_of(inputs, [](const auto& port) { return port.isConnected(); })) {
                throw gr::exception(std::format("n_inputs is {}; the input count cannot change once a port is connected", nPorts));
            }
            inputs.resize(nPorts);
        }
        n_records.value.resize(nPorts, gr::Size_t{0U});
    }

    template<gr::InputSpanLike TInSpan>
    [[nodiscard]] work::Status processBulk(std::span<TInSpan>& ins, OutputSpanLike auto& outSpan) {
        const std::size_t room      = outSpan.isConnected ? outSpan.size() : 0UZ;
        std::size_t       published = 0UZ;

        for (std::size_t port = 0UZ; port < ins.size(); ++port) {
            TInSpan&          inSpan = ins[port];
            const std::size_t take   = std::min(inSpan.size(), room - published);
            for (std::size_t k = 0UZ; k < take; ++k) {
                outSpan[published + k] = inSpan[k]; // read, never moved from: the buffer's record is not this block's
            }
            // a tag at input relative index r annotates record r, and record r of this input leaves at this offset
            for (const auto& [relIndex, tagMap] : inSpan.tags()) {
                if (relIndex >= 0 && relIndex < static_cast<std::ptrdiff_t>(take)) {
                    outSpan.publishTag(tagMap.get(), published + static_cast<std::size_t>(relIndex));
                }
            }
            std::ignore = inSpan.consume(take);
            if (port < n_records.value.size()) {
                n_records.value[port] += static_cast<gr::Size_t>(take);
            }
            published += take;
        }

        outSpan.publish(published);
        if (published == 0UZ) {
            const bool anyOffered = std::ranges::any_of(ins, [](const TInSpan& in) { return in.size() > 0UZ; });
            return anyOffered ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::basic

#endif // GNURADIO_BASIC_RECORD_MERGE_HPP
