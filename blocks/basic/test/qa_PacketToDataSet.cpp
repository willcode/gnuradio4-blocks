#include <boost/ut.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/RuntimeTest.hpp>
#include <gnuradio-4.0/Tag.hpp>

#include <gnuradio-4.0/algorithm/dataset/DataSetHelper.hpp>
#include <gnuradio-4.0/basic/DataSetToPacket.hpp>
#include <gnuradio-4.0/basic/PacketToDataSet.hpp>
#include <gnuradio-4.0/digital/CrcBlocks.hpp>
#include <gnuradio-4.0/testing/NullSources.hpp>

// The forward boundary discards and this one fills, so the properties below are mostly about what the block declines
// to write: no axis, no timing event, no metadata key of its own. Those absences are the contract — a record that
// quietly grew a fabricated time axis would state an origin no packet carries, and would offer the next boundary a
// rate derived from an axis derived from a rate. The other load-bearing half is the narrowing that only exists in
// this direction: `signal_min` and `signal_max` are `float` while a `Range<T>` holds `T`, so the lift back into the
// record's own field is a cast that is undefined outside `T`'s range for a value that came off a wire.

namespace {

using gr::blocks::basic::DataSetToPacket;
using gr::blocks::basic::PacketToDataSet;

template<typename T>
using Record = gr::DataSet<T>;

template<typename T>
using Pkt = gr::Packet<T>;

// ─── a minimal three-port span harness ────────────────────────────────────────────────────────────────────────────

struct TagReaderSpan : std::span<const gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagReaderSpan() = default;
    constexpr TagReaderSpan(std::span<const gr::Tag> tags) : std::span<const gr::Tag>(tags) {}
    constexpr bool consume(std::size_t) const noexcept { return true; }
};

struct TagWriterSpan : std::span<gr::Tag> {
    using value_type          = gr::Tag;
    constexpr TagWriterSpan() = default;
    constexpr TagWriterSpan(std::span<gr::Tag> tags) : std::span<gr::Tag>(tags) {}
    constexpr void publish(std::size_t) const noexcept {}
};

using TagView = std::pair<std::ptrdiff_t, std::reference_wrapper<const gr::property_map>>;

template<typename T>
struct InputSpan : std::span<const T> {
    using value_type = T;

    TagReaderSpan rawTags{};
    std::size_t   streamIndex = 0UZ;
    std::size_t   consumed    = 0UZ;
    bool          isConnected = true;
    bool          isSync      = true;

    InputSpan(std::span<const T> items, std::size_t at = 0UZ, std::span<const gr::Tag> incoming = {}) : std::span<const T>(items), rawTags(incoming), streamIndex(at) {}

    constexpr bool consume(std::size_t nItems) noexcept {
        consumed = nItems;
        return true;
    }
    constexpr void consumeTags(std::size_t) noexcept {}

    [[nodiscard]] std::vector<TagView> tags() const { return tags(this->size()); }

    [[nodiscard]] std::vector<TagView> tags(std::size_t window) const {
        std::vector<TagView> view;
        for (const gr::Tag& tag : rawTags) {
            const auto relIndex = static_cast<std::ptrdiff_t>(tag.index) - static_cast<std::ptrdiff_t>(streamIndex);
            if (relIndex < static_cast<std::ptrdiff_t>(window)) {
                view.emplace_back(relIndex, std::cref(tag.map));
            }
        }
        return view;
    }
};

template<typename T>
struct OutputSpan : std::span<T> {
    using value_type = T;

    std::vector<gr::Tag>* sink = nullptr;
    TagWriterSpan         tags{};
    std::size_t           streamIndex = 0UZ;
    std::size_t           count       = 0UZ;
    bool                  isConnected = true;
    bool                  isSync      = false;

    OutputSpan(std::span<T> items, std::size_t at = 0UZ, std::vector<gr::Tag>* published = nullptr, bool connected = true) : std::span<T>(items), sink(published), streamIndex(at), isConnected(connected) {}

    constexpr void publish(std::size_t nItems) noexcept { count = nItems; }

    void publishTag(const gr::property_map& tagData, std::size_t tagOffset = 0UZ) {
        if (!isConnected || sink == nullptr) {
            return;
        }
        sink->push_back(gr::Tag{streamIndex + tagOffset, tagData});
    }
};

/// @brief Everything one run of the block published, with tag offsets absolute in the port they were published on.
template<typename T>
struct Capture {
    std::vector<Record<T>> records{};
    std::vector<gr::Tag>   recordTags{};
    std::vector<Pkt<T>>    rejected{};
    std::vector<gr::Tag>   rejectTags{};
    std::size_t            consumed = 0UZ;
    std::size_t            calls    = 0UZ;
    bool                   stalled  = false;
};

/**
 * @brief Drive @p block over @p packets with an output span of @p outRoom records.
 *
 * @p outRoom of 0 offers room for every packet in one span. @p inTags carry an absolute *packet* index, which is what
 * a `Packet` port's tag index means. Consumed tags stay in view so that the negative relative index the framework
 * presents an already-visited tag at is exercised rather than assumed away.
 */
template<typename T>
[[nodiscard]] Capture<T> run(PacketToDataSet<T>& block, std::span<const Pkt<T>> packets, std::size_t outRoom = 0UZ, std::span<const gr::Tag> inTags = {}, bool rejectConnected = true, bool outConnected = true) {
    Capture<T>        capture;
    const std::size_t room = outRoom == 0UZ ? std::max(packets.size(), 1UZ) : outRoom;

    std::vector<Record<T>> outScratch(room);
    std::vector<Pkt<T>>    rejectScratch(packets.size() + 1UZ);

    std::size_t consumed = 0UZ;
    while (consumed < packets.size()) {
        InputSpan<Pkt<T>>     inSpan(packets.subspan(consumed), consumed, inTags);
        OutputSpan<Record<T>> outSpan(outConnected ? std::span<Record<T>>(outScratch) : std::span<Record<T>>{}, capture.records.size(), &capture.recordTags, outConnected);
        OutputSpan<Pkt<T>>    rejectSpan(rejectConnected ? std::span<Pkt<T>>(rejectScratch) : std::span<Pkt<T>>{}, capture.rejected.size(), &capture.rejectTags, rejectConnected);

        std::ignore = block.processBulk(inSpan, outSpan, rejectSpan);
        ++capture.calls;

        for (std::size_t k = 0UZ; k < outSpan.count; ++k) {
            capture.records.push_back(std::move(outScratch[k]));
        }
        for (std::size_t k = 0UZ; k < rejectSpan.count; ++k) {
            capture.rejected.push_back(std::move(rejectScratch[k]));
        }
        consumed += inSpan.consumed;
        capture.consumed += inSpan.consumed;
        if (inSpan.consumed == 0UZ && outSpan.count == 0UZ && rejectSpan.count == 0UZ) {
            capture.stalled = true; // no progress against a span it will be offered again: the graph would wedge here
            break;
        }
    }
    return capture;
}

template<typename TBlock>
[[nodiscard]] TBlock make(gr::property_map settings = {}) {
    TBlock block(std::move(settings));
    block.settings().init();
    std::ignore = block.settings().applyStagedParameters();
    return block;
}

// ─── builders ─────────────────────────────────────────────────────────────────────────────────────────────────────

/// @brief `value` as a sample of type @p T, the complex case taking it as the real part; explicit at every step so
/// that -Wconversion has nothing to say about a test's own scaffolding.
template<typename T>
[[nodiscard]] constexpr T sampleValue(std::size_t value) {
    if constexpr (gr::meta::complex_like<T>) {
        return T{static_cast<typename T::value_type>(value), typename T::value_type{}};
    } else {
        return static_cast<T>(value);
    }
}

void putMeta(gr::property_map& map, std::string_view key, gr::pmt::Value value) { map.insert_or_assign(gr::property_map::key_type(key), std::move(value)); }

/// @brief The shape `DataSetToPacket` and `ZmqPacketSource` both produce: a payload and exactly one metadata map.
template<typename T>
[[nodiscard]] Pkt<T> makePacket(std::size_t nSamples) {
    Pkt<T> packet;
    packet.signal_values.resize(nSamples);
    for (std::size_t j = 0UZ; j < nSamples; ++j) {
        packet.signal_values[j] = sampleValue<T>(j);
    }
    packet.meta_information.resize(1UZ);
    return packet;
}

/// @brief A single-signal record of @p nSamples bytes, the length the acceptance pair's transmitter cuts.
[[nodiscard]] Record<std::uint8_t> makeByteRecord(std::size_t nSamples, std::size_t offset) {
    Record<std::uint8_t> record;
    record.signal_values.resize(nSamples);
    for (std::size_t j = 0UZ; j < nSamples; ++j) {
        record.signal_values[j] = static_cast<std::uint8_t>((offset + j) & 0xFFUZ);
    }
    record.extents.push_back(static_cast<std::int32_t>(nSamples));
    record.signal_names.emplace_back("payload");
    record.meta_information.resize(1UZ);
    record.timing_events.resize(1UZ);
    return record;
}

// ─── readers ──────────────────────────────────────────────────────────────────────────────────────────────────────

template<typename TValue>
[[nodiscard]] std::optional<TValue> read(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(key);
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const TValue* value = it->second.template get_if<TValue>(); value != nullptr) {
        return *value;
    }
    return std::nullopt;
}

[[nodiscard]] std::optional<std::string> readString(const gr::property_map& map, std::string_view key) {
    const auto it = map.find(key);
    if (it == map.end()) {
        return std::nullopt;
    }
    if (const std::pmr::string* value = it->second.get_if<std::pmr::string>(); value != nullptr) {
        return std::string(std::string_view(*value));
    }
    return std::nullopt;
}

[[nodiscard]] bool hasKey(const gr::property_map& map, std::string_view key) { return map.find(key) != map.end(); }

// A member's absence is only a substitution failure where the type is dependent, so the two counters this boundary
// does not need are asked for through a concept rather than through a bare requires-expression.
template<typename TBlock>
concept CountsDroppedTimingEvents = requires(TBlock block) { block.nDroppedTimingEvents; };

template<typename TBlock>
concept CountsOverriddenKeys = requires(TBlock block) { block.nMetaKeysOverridden; };

/// @brief The one metadata map a record this block produced carries, or an empty one when the shape is wrong.
template<typename T>
[[nodiscard]] gr::property_map metaOf(const Record<T>& record) {
    return record.meta_information.size() == 1UZ ? record.meta_information.front() : gr::property_map{};
}

[[nodiscard]] std::vector<std::string> keysOf(const gr::property_map& map) {
    std::vector<std::string> keys;
    for (const auto& [key, value] : map) {
        keys.emplace_back(std::string_view(key));
    }
    std::ranges::sort(keys);
    return keys;
}

/// @brief The offsets at which a tag carrying @p key was published, in publication order.
[[nodiscard]] std::vector<std::size_t> offsetsOf(std::span<const gr::Tag> tags, std::string_view key) {
    std::vector<std::size_t> offsets;
    for (const gr::Tag& tag : tags) {
        if (hasKey(tag.map, key)) {
            offsets.push_back(tag.index);
        }
    }
    return offsets;
}

// ─── graph-side blocks, so that Packet<T> is exercised as a real port item and not only through a mock span ───────

template<typename TItem>
struct ItemSource : gr::Block<ItemSource<TItem>> {
    gr::PortOut<TItem> out;
    GR_MAKE_REFLECTABLE(ItemSource, out);

    std::vector<TItem>   _items{};
    std::vector<gr::Tag> _tags{}; ///< index is an absolute item index, ascending
    std::size_t          _emitted = 0UZ;

    gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) {
        const std::size_t nItems = std::min(outSpan.size(), _items.size() - _emitted);
        for (std::size_t k = 0UZ; k < nItems; ++k) {
            for (const gr::Tag& tag : _tags) {
                if (tag.index == _emitted + k) {
                    outSpan.publishTag(tag.map, k);
                }
            }
            outSpan[k] = _items[_emitted + k];
        }
        _emitted += nItems;
        outSpan.publish(nItems);
        return _emitted >= _items.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

template<typename TItem>
struct Collector : gr::Block<Collector<TItem>> {
    gr::PortIn<TItem> in;
    GR_MAKE_REFLECTABLE(Collector, in);

    std::vector<TItem>   _items{};
    std::vector<gr::Tag> _tags{};

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& [relIndex, tagMap] : inSpan.tags()) {
            if (relIndex >= 0 && relIndex < static_cast<std::ptrdiff_t>(inSpan.size())) {
                _tags.push_back(gr::Tag{_items.size() + static_cast<std::size_t>(relIndex), tagMap.get()});
            }
        }
        for (std::size_t k = 0UZ; k < inSpan.size(); ++k) {
            _items.push_back(inSpan[k]);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// @brief Flips one byte of one packet, which is what "corrupted in flight" is: the CRC field still states the
/// original, so the damage is only visible to a receiver that recomputes.
struct PacketCorrupter : gr::Block<PacketCorrupter> {
    gr::PortIn<gr::Packet<std::uint8_t>>             in;
    gr::PortOut<gr::Packet<std::uint8_t>, gr::Async> out;
    GR_MAKE_REFLECTABLE(PacketCorrupter, in, out);

    std::size_t _which = 0UZ; ///< the packet to damage, by arrival order
    std::size_t _seen  = 0UZ;

    gr::work::Status processBulk(gr::InputSpanLike auto& inSpan, gr::OutputSpanLike auto& outSpan) {
        const std::size_t nItems = std::min(inSpan.size(), outSpan.size());
        for (std::size_t k = 0UZ; k < nItems; ++k) {
            gr::Packet<std::uint8_t> packet = inSpan[k];
            if (_seen + k == _which && !packet.signal_values.empty()) {
                packet.signal_values[0UZ] = static_cast<std::uint8_t>(packet.signal_values[0UZ] ^ 0xFFU);
            }
            outSpan[k] = std::move(packet);
        }
        _seen += nItems;
        std::ignore = inSpan.consume(nItems);
        outSpan.publish(nItems);
        return gr::work::Status::OK;
    }
};

} // namespace

const boost::ut::suite<"PacketToDataSet"> packetToDataSetTests = [] {
    using namespace boost::ut;
    using namespace gr;

    // criterion 1 — every record field has one provenance, and four of them are absences
    "every field of the record has one provenance"_test = [] {
        Pkt<float> packet    = makePacket<float>(6UZ);
        packet.timestamp     = -1234567890123LL;
        packet.default_value = 0.5f;
        property_map& map    = packet.meta_information[0UZ];
        putMeta(map, "signal_name", pmt::Value(std::string("chirp")));
        putMeta(map, "signal_quantity", pmt::Value(std::string("voltage")));
        putMeta(map, "signal_unit", pmt::Value(std::string("V")));
        putMeta(map, "signal_min", pmt::Value(-2.f));
        putMeta(map, "signal_max", pmt::Value(2.f));
        putMeta(map, "sample_rate", pmt::Value(48000.f));
        putMeta(map, "sample_start", pmt::Value(std::uint64_t{4096ULL}));
        putMeta(map, "n_pre", pmt::Value(gr::Size_t{7U})); // producer-private, copied unexamined

        auto             block = make<PacketToDataSet<float>>();
        const Pkt<float> input[]{packet};
        const auto       capture = run<float>(block, input);

        expect(eq(capture.records.size(), 1UZ));
        expect(eq(capture.consumed, 1UZ));
        const Record<float>& record = capture.records.front();

        expect(std::ranges::equal(record.signal_values, packet.signal_values)) << "the payload is the record's samples, in order";
        expect(eq(record.extents.size(), 1UZ));
        expect(eq(record.extents.front(), std::int32_t{6}));
        expect(eq(record.timestamp, packet.timestamp)) << "carried into the field of the same name, sign included";
        expect(eq(record.default_value, packet.default_value));
        expect(eq(record.meta_information.size(), 1UZ));
        expect(eq(record.timing_events.size(), 1UZ)) << "one list per signal";
        expect(record.timing_events.front().empty()) << "and nothing ever puts an entry in it";
        expect(eq(record.signal_names.size(), 1UZ));
        expect(eq(record.signal_names.front(), std::string("chirp")));
        expect(eq(record.signal_quantities.size(), 1UZ));
        expect(eq(record.signal_quantities.front(), std::string("voltage")));
        expect(eq(record.signal_units.size(), 1UZ));
        expect(eq(record.signal_units.front(), std::string("V")));
        expect(eq(record.signal_ranges.size(), 1UZ));
        expect(eq(record.signal_ranges.front().min, -2.f));
        expect(eq(record.signal_ranges.front().max, 2.f));

        // the four fields that state nothing, asserted as absences
        expect(record.axis_names.empty()) << "no axis name is invented";
        expect(record.axis_units.empty()) << "no axis unit is invented";
        expect(record.axis_values.empty()) << "and above all no axis values, which would state an origin no packet has";

        // the lifted keys stay in the map, which is the vocabulary's copy default
        const property_map recordMap = metaOf(record);
        expect(eq(readString(recordMap, "signal_name").value_or(""), std::string("chirp")));
        expect(eq(read<float>(recordMap, "sample_rate").value_or(0.f), 48000.f));
        expect(eq(read<std::uint64_t>(recordMap, "sample_start").value_or(0ULL), std::uint64_t{4096ULL})) << "provenance rides as metadata, not as structure";
        expect(eq(read<gr::Size_t>(recordMap, "n_pre").value_or(0U), gr::Size_t{7U})) << "a producer-private key is copied unexamined";

        expect(eq(block.nRejectedPackets, 0ULL));
        expect(eq(block.nMetaKeysDropped, 0ULL));
        expect(eq(block.nSignalNamesSynthesized, 0ULL));
    };

    // criterion 2 — the three clauses, their boundaries, and the untouched refusal
    "three clauses admit and refuse, and a refused packet is republished untouched"_test = [] {
        std::vector<Pkt<float>> packets;
        Pkt<float>              noMap = makePacket<float>(4UZ);
        noMap.meta_information.clear();
        packets.push_back(noMap);
        Pkt<float> twoMaps = makePacket<float>(4UZ);
        twoMaps.meta_information.resize(2UZ);
        packets.push_back(twoMaps);
        Pkt<float> empty = makePacket<float>(0UZ);
        putMeta(empty.meta_information[0UZ], "protocol", pmt::Value(std::string("ack")));
        packets.push_back(empty);
        packets.push_back(makePacket<float>(1UZ)); // the smallest admissible payload

        auto       block   = make<PacketToDataSet<float>>();
        const auto capture = run<float>(block, std::span<const Pkt<float>>(packets));

        expect(eq(capture.records.size(), 1UZ)) << "only the one-item packet is admitted by default";
        expect(eq(capture.rejected.size(), 3UZ));
        expect(eq(block.nRejectedPackets, 3ULL));
        expect(eq(capture.consumed, packets.size()));

        const std::vector<std::string> expectedReasons{"not_one_metadata_map", "not_one_metadata_map", "empty_payload"};
        expect(eq(capture.rejectTags.size(), 3UZ));
        for (std::size_t k = 0UZ; k < std::min(capture.rejectTags.size(), expectedReasons.size()); ++k) {
            expect(eq(readString(capture.rejectTags[k].map, "discard_reason").value_or(""), expectedReasons[k])) << "refusal" << k;
            expect(eq(capture.rejectTags[k].index, k)) << "the reason sits at the packet's own offset";
        }
        // republished untouched: the payload, the map and the carrier fields are the packet's own
        expect(eq(capture.rejected[2UZ].meta_information.size(), 1UZ));
        expect(eq(readString(capture.rejected[2UZ].meta_information[0UZ], "protocol").value_or(""), std::string("ack"))) << "nothing is written into a packet the block refused";
        expect(capture.rejected[0UZ].meta_information.empty()) << "including its metadata vector, which is the field that was wrong";

        { // the setting is the whole of the difference for an empty payload
            auto             permissive = make<PacketToDataSet<float>>({{"allow_empty_payload", true}});
            const Pkt<float> onlyEmpty[]{empty};
            const auto       admitted = run<float>(permissive, onlyEmpty);
            expect(eq(admitted.records.size(), 1UZ));
            expect(eq(admitted.records.front().extents.size(), 1UZ));
            expect(eq(admitted.records.front().extents.front(), std::int32_t{0})) << "an explicit zero extent, which a packet cannot state";
            expect(admitted.records.front().signal_values.empty());
            expect(eq(permissive.nRejectedPackets, 0ULL));
        }

        // Q3 as the predicate it is: the wire's item count is wider than the record's extent field, by a factor of two
        static_assert(std::numeric_limits<std::uint32_t>::max() > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()), "a std::uint32_t item count can state an extent std::int32_t cannot hold");
        expect(eq(static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) - static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()), std::uint64_t{2147483648ULL}));
    };

    // criterion 3 — the vocabulary type rule at a carrier boundary, in the direction where the value came off a wire
    "a mistyped vocabulary key is dropped and a private key of any type is not"_test = [] {
        Pkt<float> packet = makePacket<float>(4UZ);
        putMeta(packet.meta_information[0UZ], "sample_rate", pmt::Value(48000.0)); // a double where the vocabulary says float
        putMeta(packet.meta_information[0UZ], "ctx", pmt::Value(std::uint64_t{9ULL}));

        auto               block = make<PacketToDataSet<float>>();
        const Pkt<float>   input[]{packet};
        const auto         capture   = run<float>(block, input);
        const property_map recordMap = metaOf(capture.records.front());

        expect(!hasKey(recordMap, "sample_rate")) << "a key that reads as nothing does not become a key that reads as nothing in a record";
        expect(eq(read<float>(recordMap, "sample_rate").value_or(-1.f), -1.f));
        expect(eq(block.nMetaKeysDropped, 1ULL));
        expect(eq(read<std::uint64_t>(recordMap, "ctx").value_or(0ULL), std::uint64_t{9ULL})) << "a producer-private key imposes nothing and crosses at whatever type it holds";

        Pkt<float> typed = makePacket<float>(4UZ);
        putMeta(typed.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
        auto             second = make<PacketToDataSet<float>>();
        const Pkt<float> right[]{typed};
        expect(eq(read<float>(metaOf(run<float>(second, right).records.front()), "sample_rate").value_or(0.f), 48000.f));
        expect(eq(second.nMetaKeysDropped, 0ULL));
    };

    // criterion 5 — the annotations were dropped at the forward boundary and stay dropped; the statement crosses
    "dropped_events crosses and no event is recovered"_test = [] {
        Pkt<std::uint8_t> packet = makePacket<std::uint8_t>(8UZ);
        putMeta(packet.meta_information[0UZ], "dropped_events", pmt::Value(gr::Size_t{7U}));

        auto                        block = make<PacketToDataSet<std::uint8_t>>();
        const Pkt<std::uint8_t>     input[]{packet};
        const auto                  capture = run<std::uint8_t>(block, input);
        const Record<std::uint8_t>& record  = capture.records.front();

        expect(eq(read<gr::Size_t>(metaOf(record), "dropped_events").value_or(0U), gr::Size_t{7U})) << "the count of what was lost travels with the record";
        expect(eq(record.timing_events.size(), 1UZ));
        expect(record.timing_events.front().empty()) << "and the events themselves do not come back";
        // no counter exists for an event that never arrived, and its absence is asserted rather than assumed
        static_assert(!CountsDroppedTimingEvents<PacketToDataSet<std::uint8_t>>, "there is nothing at this boundary to drop");
        static_assert(!CountsOverriddenKeys<PacketToDataSet<std::uint8_t>>, "the block derives no key, so nothing is ever displaced");
        static_assert(CountsDroppedTimingEvents<DataSetToPacket<std::uint8_t>>, "the forward boundary does have both, which is what makes the absence a difference");
        static_assert(CountsOverriddenKeys<DataSetToPacket<std::uint8_t>>);
    };

    // criterion 6 — the record's map is the packet's, less the drops, plus the tag's, and nothing else
    "the record's map holds no key the packet did not carry"_test = [] {
        Pkt<float>    packet = makePacket<float>(4UZ);
        property_map& map    = packet.meta_information[0UZ];
        putMeta(map, "protocol", pmt::Value(std::string("packet_link")));
        putMeta(map, "source_id", pmt::Value(std::string("qa")));
        putMeta(map, "sequence", pmt::Value(std::uint64_t{12ULL}));
        putMeta(map, "crc_ok", pmt::Value(true));
        putMeta(map, "crc_value", pmt::Value(std::uint64_t{0xDEADBEEFULL}));
        putMeta(map, "corrected_errors", pmt::Value(gr::Size_t{0U}));
        putMeta(map, "signal_name", pmt::Value(std::string("payload")));
        putMeta(map, "sample_rate", pmt::Value(1.0e6f));
        putMeta(map, "n_max", pmt::Value(gr::Size_t{4096U}));

        auto             block = make<PacketToDataSet<float>>();
        const Pkt<float> input[]{packet};
        const auto       capture = run<float>(block, input);

        expect(std::ranges::equal(keysOf(metaOf(capture.records.front())), keysOf(map))) << "the same key set, exactly: nothing added and nothing consumed";
        expect(!hasKey(metaOf(capture.records.front()), "schema_version")) << "no key whose since is above 1 is written, so none is stated";
        expect(!hasKey(metaOf(capture.records.front()), "dataset_length")) << "the record's length is its extent, and dataset_length belongs to the boundary that needs it";
    };

    // criterion 7 — the four lifted keys, the all-or-nothing range, and the narrowing guard
    "the lifted keys fill the typed fields and the guard keeps a range the type cannot hold out"_test = [] {
        { // a stated name is used and the counter stays at zero
            Pkt<float> named = makePacket<float>(3UZ);
            putMeta(named.meta_information[0UZ], "signal_name", pmt::Value(std::string("chirp")));
            auto             block = make<PacketToDataSet<float>>();
            const Pkt<float> input[]{named};
            const auto       capture = run<float>(block, input);
            expect(eq(capture.records.front().signal_names.front(), std::string("chirp")));
            expect(eq(block.nSignalNamesSynthesized, 0ULL));
            expect(hasKey(metaOf(capture.records.front()), "signal_name")) << "lifted, not consumed";
        }
        { // no name at all falls back to the label, once per record, and is counted
            auto             block = make<PacketToDataSet<float>>({{"signal_label", std::string("from_the_wire")}});
            const Pkt<float> input[]{makePacket<float>(3UZ), makePacket<float>(3UZ)};
            const auto       capture = run<float>(block, input);
            expect(eq(capture.records.front().signal_names.front(), std::string("from_the_wire")));
            expect(eq(block.nSignalNamesSynthesized, 2ULL));
            expect(!hasKey(metaOf(capture.records.front()), "signal_name")) << "the label fills a field and never a key";
        }
        { // one end of a range is not a range, and the key that is present still crosses
            Pkt<float> half = makePacket<float>(3UZ);
            putMeta(half.meta_information[0UZ], "signal_min", pmt::Value(-1.f));
            auto             block = make<PacketToDataSet<float>>();
            const Pkt<float> input[]{half};
            const auto       capture = run<float>(block, input);
            expect(capture.records.front().signal_ranges.empty());
            expect(eq(read<float>(metaOf(capture.records.front()), "signal_min").value_or(0.f), -1.f));
            expect(eq(block.nMetaKeysDropped, 0ULL)) << "nothing was dropped: the fact was never a pair";
        }
        { // the narrowing the forward block's own output can produce: float(INT32_MAX) is INT32_MAX + 1
            expect(gt(static_cast<double>(static_cast<float>(std::numeric_limits<std::int32_t>::max())), static_cast<double>(std::numeric_limits<std::int32_t>::max()))) << "the cast back would be out of range";
            Pkt<std::int32_t> wide = makePacket<std::int32_t>(3UZ);
            putMeta(wide.meta_information[0UZ], "signal_min", pmt::Value(0.f));
            putMeta(wide.meta_information[0UZ], "signal_max", pmt::Value(static_cast<float>(std::numeric_limits<std::int32_t>::max())));
            auto                    block = make<PacketToDataSet<std::int32_t>>();
            const Pkt<std::int32_t> input[]{wide};
            const auto              capture = run<std::int32_t>(block, input);
            expect(capture.records.front().signal_ranges.empty()) << "a limit the record's own type cannot hold is not cast";
            expect(eq(block.nMetaKeysDropped, 1ULL)) << "the fact did not reach the field it names, and that is counted";
            expect(hasKey(metaOf(capture.records.front()), "signal_max")) << "though the producer's own number still crosses in the map";
        }
        { // an integral range that does fit is lifted
            Pkt<std::int16_t> fits = makePacket<std::int16_t>(3UZ);
            putMeta(fits.meta_information[0UZ], "signal_min", pmt::Value(-100.f));
            putMeta(fits.meta_information[0UZ], "signal_max", pmt::Value(100.f));
            auto                    block = make<PacketToDataSet<std::int16_t>>();
            const Pkt<std::int16_t> input[]{fits};
            const auto              capture = run<std::int16_t>(block, input);
            expect(eq(capture.records.front().signal_ranges.size(), 1UZ));
            expect(eq(capture.records.front().signal_ranges.front().min, std::int16_t{-100}));
            expect(eq(capture.records.front().signal_ranges.front().max, std::int16_t{100}));
        }
        { // a complex signal_range is two samples ordered by magnitude, not two limits, so nothing is lifted
            Pkt<std::complex<float>> complexPacket = makePacket<std::complex<float>>(3UZ);
            putMeta(complexPacket.meta_information[0UZ], "signal_min", pmt::Value(-1.f));
            putMeta(complexPacket.meta_information[0UZ], "signal_max", pmt::Value(1.f));
            auto                           block = make<PacketToDataSet<std::complex<float>>>();
            const Pkt<std::complex<float>> input[]{complexPacket};
            expect(run<std::complex<float>>(block, input).records.front().signal_ranges.empty());
        }
    };

    // criterion 8 — a rate and a start are provenance, and provenance is not structure
    "a stated rate builds no axis and stays bit-identical"_test = [] {
        Pkt<float> packet = makePacket<float>(64UZ);
        putMeta(packet.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
        putMeta(packet.meta_information[0UZ], "sample_start", pmt::Value(std::uint64_t{12345ULL}));

        auto                 block = make<PacketToDataSet<float>>();
        const Pkt<float>     input[]{packet};
        const auto           capture = run<float>(block, input);
        const Record<float>& record  = capture.records.front();

        expect(record.axis_values.empty()) << "the rate is not turned into an axis, which would state an origin";
        expect(record.axis_names.empty());
        expect(eq(read<float>(metaOf(record), "sample_rate").value_or(0.f), 48000.f));

        // and back out again: the forward block's route 1 reads the key, so the value is the same float it started as
        auto                       forward = make<DataSetToPacket<float>>();
        const Record<float>        records[]{record};
        std::vector<Pkt<float>>    packetScratch(1UZ);
        std::vector<Record<float>> rejectScratch(1UZ);
        InputSpan<Record<float>>   inSpan{std::span<const Record<float>>(records)};
        OutputSpan<Pkt<float>>     outSpan{std::span<Pkt<float>>(packetScratch)};
        OutputSpan<Record<float>>  rejectSpan{std::span<Record<float>>(rejectScratch)};
        std::ignore = forward.processBulk(inSpan, outSpan, rejectSpan);
        expect(eq(outSpan.count, 1UZ));
        expect(eq(read<float>(packetScratch.front().meta_information.at(0UZ), "sample_rate").value_or(0.f), 48000.f)) << "no axis means no rate derived from an axis derived from a rate";
    };

    // criterion 9 — a tag belongs to its packet and to no other, and nothing is published on out
    "an input tag lands in its packet's record and nowhere else"_test = [] {
        constexpr std::size_t   kPackets = 9UZ;
        std::vector<Pkt<float>> packets;
        for (std::size_t k = 0UZ; k < kPackets; ++k) {
            packets.push_back(makePacket<float>(3UZ));
        }
        std::vector<Tag> tags;
        for (const std::size_t at : {0UZ, 1UZ, 7UZ}) {
            property_map map;
            putMeta(map, "sample_rate", pmt::Value(96000.f));
            putMeta(map, "marker", pmt::Value(static_cast<gr::Size_t>(at)));
            tags.emplace_back(at, std::move(map));
        }

        auto       block   = make<PacketToDataSet<float>>();
        const auto capture = run<float>(block, std::span<const Pkt<float>>(packets), 0UZ, std::span<const Tag>(tags));

        expect(eq(capture.records.size(), kPackets));
        expect(capture.recordTags.empty()) << "no tag is published on out: a record is one item and its map is what carries facts";
        for (std::size_t k = 0UZ; k < capture.records.size(); ++k) {
            const bool tagged = k == 0UZ || k == 1UZ || k == 7UZ;
            expect(eq(hasKey(metaOf(capture.records[k]), "marker"), tagged)) << "record" << k;
            if (tagged) {
                expect(eq(read<gr::Size_t>(metaOf(capture.records[k]), "marker").value_or(999U), static_cast<gr::Size_t>(k))) << "record" << k << "carries its own tag and no other's";
            }
        }

        { // the type rule binds on a tag key too: writing it into a record is a carrier crossing
            std::vector<Tag> mistyped;
            property_map     map;
            putMeta(map, "sample_rate", pmt::Value(96000.0));
            mistyped.emplace_back(0UZ, std::move(map));
            auto             tagBlock = make<PacketToDataSet<float>>();
            const Pkt<float> one[]{makePacket<float>(3UZ)};
            const auto       capture2 = run<float>(tagBlock, one, 0UZ, std::span<const Tag>(mistyped));
            expect(!hasKey(metaOf(capture2.records.front()), "sample_rate"));
            expect(eq(tagBlock.nMetaKeysDropped, 1ULL));
        }
    };

    // criterion 10, compile-time half
    "the block is not admissible for UnfilteredTagPropagation"_test = [] {
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<PacketToDataSet<std::uint8_t>>, "asynchronous outputs and NoTagPropagation each refuse the flag on their own");
        static_assert(!gr::block::kUnfilteredTagPropagationAdmissible<PacketToDataSet<float>>);
        expect(!gr::block::kUnfilteredTagPropagationAdmissible<PacketToDataSet<std::uint8_t>>);
    };

    // criterion 12 — a surprising metadata value is not a malformed packet, and nothing stops the graph
    "ten hostile packets are handled without throwing"_test = [] {
        std::vector<Pkt<float>> packets;
        Pkt<float>              noMap = makePacket<float>(4UZ);
        noMap.meta_information.clear();
        packets.push_back(noMap);
        Pkt<float> twoMaps = makePacket<float>(4UZ);
        twoMaps.meta_information.resize(2UZ);
        packets.push_back(twoMaps);
        packets.push_back(makePacket<float>(0UZ));
        packets.push_back(makePacket<float>(1UZ));

        Pkt<float> everyKeyWrong = makePacket<float>(4UZ);
        for (const std::string_view key : {"sample_rate", "frequency", "signal_name", "protocol", "sequence", "crc_ok", "schema_version"}) {
            putMeta(everyKeyWrong.meta_information[0UZ], key, pmt::Value(std::int8_t{3}));
        }
        packets.push_back(everyKeyWrong);

        Pkt<float>   nested = makePacket<float>(4UZ);
        property_map inner;
        putMeta(inner, "depth", pmt::Value(gr::Size_t{1U}));
        putMeta(nested.meta_information[0UZ], "trigger_meta_info", pmt::Value(inner));
        packets.push_back(nested);

        Pkt<float> nameNotAString = makePacket<float>(4UZ);
        putMeta(nameNotAString.meta_information[0UZ], "signal_name", pmt::Value(gr::Size_t{5U}));
        packets.push_back(nameNotAString);

        Pkt<float> lowOnly = makePacket<float>(4UZ);
        putMeta(lowOnly.meta_information[0UZ], "signal_min", pmt::Value(0.f));
        packets.push_back(lowOnly);

        Pkt<float> infinite = makePacket<float>(4UZ);
        putMeta(infinite.meta_information[0UZ], "signal_min", pmt::Value(0.f));
        putMeta(infinite.meta_information[0UZ], "signal_max", pmt::Value(std::numeric_limits<float>::infinity()));
        packets.push_back(infinite);

        Pkt<float> extremeTime = makePacket<float>(4UZ);
        extremeTime.timestamp  = std::numeric_limits<std::int64_t>::min();
        packets.push_back(extremeTime);

        auto           block = make<PacketToDataSet<float>>();
        Capture<float> capture;
        expect(nothrow([&] { capture = run<float>(block, std::span<const Pkt<float>>(packets)); }));
        expect(!capture.stalled) << "a hostile packet must not wedge the block";
        expect(eq(capture.consumed, packets.size()));
        expect(eq(capture.rejected.size(), 3UZ)) << "exactly the three shape failures";
        expect(eq(block.nRejectedPackets, 3ULL));
        expect(eq(capture.records.size(), packets.size() - 3UZ)) << "every surprising value is admitted";

        // the extreme timestamp crosses as itself, because both fields are std::int64_t
        expect(eq(capture.records.back().timestamp, std::numeric_limits<std::int64_t>::min()));
        expect(eq(capture.records[3UZ].signal_names.front(), std::string("packet"))) << "a mistyped signal_name is dropped, so the label fills the field";
        expect(capture.records[4UZ].signal_ranges.empty()) << "signal_min alone is not a range";
        expect(eq(capture.records[5UZ].signal_ranges.size(), 1UZ)) << "an infinite limit is a legal float and the record's own type holds it, so it is not this block's to refuse";
        expect(std::isinf(capture.records[5UZ].signal_ranges.front().max));
    };

    // criterion 13 — the block holds nothing between calls, so the chunking cannot show
    "the records do not depend on the input chunk size"_test = [] {
        std::vector<Pkt<std::uint8_t>> packets;
        for (std::size_t k = 0UZ; k < 20UZ; ++k) {
            Pkt<std::uint8_t> packet = makePacket<std::uint8_t>(k == 3UZ ? 1UZ : 1UZ + (k % 7UZ));
            putMeta(packet.meta_information[0UZ], "sequence", pmt::Value(static_cast<std::uint64_t>(k)));
            packets.push_back(std::move(packet));
        }

        std::vector<std::vector<std::uint8_t>> payloads;
        std::vector<std::uint64_t>             sequences;
        for (const std::size_t room : {std::size_t{0UZ}, std::size_t{1UZ}, std::size_t{3UZ}, std::size_t{17UZ}}) {
            auto       block   = make<PacketToDataSet<std::uint8_t>>();
            const auto capture = run<std::uint8_t>(block, std::span<const Pkt<std::uint8_t>>(packets), room);
            expect(eq(capture.records.size(), packets.size())) << "room" << room;

            std::vector<std::vector<std::uint8_t>> thesePayloads;
            std::vector<std::uint64_t>             theseSequences;
            for (const Record<std::uint8_t>& record : capture.records) {
                thesePayloads.push_back(record.signal_values);
                theseSequences.push_back(read<std::uint64_t>(metaOf(record), "sequence").value_or(0ULL));
            }
            if (payloads.empty()) {
                payloads  = std::move(thesePayloads);
                sequences = std::move(theseSequences);
            } else {
                const bool samePayloads  = payloads == thesePayloads;
                const bool sameSequences = sequences == theseSequences;
                expect(samePayloads) << "room" << room << ": the payloads are chunk-independent";
                expect(sameSequences) << "room" << room << ": and so is their order";
            }
        }
    };

    // criterion 14 — a port nobody reads costs nothing and changes nothing
    "everything is refused the same way with reject unconnected"_test = [] {
        std::vector<Pkt<float>> packets;
        Pkt<float>              noMap = makePacket<float>(4UZ);
        noMap.meta_information.clear();
        packets.push_back(noMap);
        packets.push_back(makePacket<float>(0UZ));
        packets.push_back(makePacket<float>(4UZ));

        auto           connected = make<PacketToDataSet<float>>();
        const auto     withPort  = run<float>(connected, std::span<const Pkt<float>>(packets));
        auto           orphaned  = make<PacketToDataSet<float>>();
        Capture<float> withoutPort;
        expect(nothrow([&] { withoutPort = run<float>(orphaned, std::span<const Pkt<float>>(packets), 0UZ, {}, false); }));
        expect(eq(withoutPort.records.size(), withPort.records.size()));
        expect(eq(orphaned.nRejectedPackets, connected.nRejectedPackets));
        expect(eq(orphaned.nRejectedPackets, 2ULL));
        expect(eq(withoutPort.rejected.size(), 0UZ)) << "nothing is written to a port nobody reads";
        expect(eq(withoutPort.consumed, packets.size())) << "and the packets are still consumed";

        auto           noOutput = make<PacketToDataSet<float>>();
        Capture<float> withoutOut;
        expect(nothrow([&] { withoutOut = run<float>(noOutput, std::span<const Pkt<float>>(packets), 0UZ, {}, true, false); }));
        expect(eq(withoutOut.records.size(), 0UZ));
        expect(eq(withoutOut.consumed, packets.size()));
        expect(eq(noOutput.nRejectedPackets, 2ULL)) << "the counters are the same with no record reader either";
    };

    "every registered type converts"_test = [] {
        const auto check = []<typename T>() {
            auto   block  = make<PacketToDataSet<T>>();
            Pkt<T> packet = makePacket<T>(5UZ);
            putMeta(packet.meta_information[0UZ], "signal_name", pmt::Value(std::string("signal0")));
            const Pkt<T> input[]{packet};
            const auto   capture = run<T>(block, input);
            expect(eq(capture.records.size(), 1UZ)) << gr::meta::type_name<T>();
            expect(eq(capture.records.front().extents.front(), std::int32_t{5})) << gr::meta::type_name<T>();
            expect(eq(capture.records.front().signal_names.front(), std::string("signal0"))) << gr::meta::type_name<T>();
            expect(std::ranges::equal(capture.records.front().signal_values, packet.signal_values)) << gr::meta::type_name<T>();
        };
        check.template operator()<std::uint8_t>();
        check.template operator()<std::int16_t>();
        check.template operator()<std::int32_t>();
        check.template operator()<float>();
        check.template operator()<std::complex<float>>();
    };
};

// ─── under the scheduler: the round trip, the reject path and the gate's own chain ────────────────────────────────

const boost::ut::suite<"PacketToDataSet under the scheduler"> schedulerTests = [] {
    using namespace boost::ut;
    using namespace gr;
    using namespace gr::blocks::basic;

    // criterion 4 — pinned for what survives, asserted as a difference for what does not, and a fixed point
    "the round trip pins what survives and differs in what it cannot"_test = [] {
        const auto roundTrip = []<typename T>() {
            const std::string label = std::string(gr::meta::type_name<T>());

            Record<T> original;
            original.signal_values.resize(16UZ);
            for (std::size_t j = 0UZ; j < 16UZ; ++j) {
                original.signal_values[j] = sampleValue<T>(j + 1UZ);
            }
            original.extents.push_back(std::int32_t{16});
            original.signal_names.emplace_back("payload");
            original.signal_quantities.emplace_back("voltage");
            original.signal_units.emplace_back("V");
            original.signal_ranges.push_back(gr::Range<T>{sampleValue<T>(0UZ), sampleValue<T>(17UZ)});
            original.timestamp     = 1724630400000000000LL;
            original.default_value = sampleValue<T>(0UZ);
            original.meta_information.resize(1UZ);
            putMeta(original.meta_information[0UZ], "sample_rate", pmt::Value(48000.f));
            putMeta(original.meta_information[0UZ], "sample_start", pmt::Value(std::uint64_t{4096ULL}));
            putMeta(original.meta_information[0UZ], "n_pre", pmt::Value(gr::Size_t{3U}));
            original.axis_names.emplace_back("time");
            original.axis_units.emplace_back("s");
            original.axis_values.resize(1UZ);
            for (std::size_t j = 0UZ; j < 16UZ; ++j) {
                original.axis_values[0UZ].push_back(sampleValue<T>(j));
            }
            original.timing_events.resize(1UZ);
            for (std::size_t j = 0UZ; j < 3UZ; ++j) {
                original.timing_events[0UZ].emplace_back(static_cast<std::ptrdiff_t>(j * 4UZ), property_map{});
            }

            Record<T> fourSignals;
            fourSignals.signal_values.resize(4UZ * 8UZ);
            for (std::size_t signal = 0UZ; signal < 4UZ; ++signal) {
                fourSignals.signal_names.push_back(std::format("signal{}", signal));
                fourSignals.meta_information.emplace_back();
                fourSignals.timing_events.emplace_back();
                for (std::size_t j = 0UZ; j < 8UZ; ++j) {
                    fourSignals.signal_values[signal * 8UZ + j] = sampleValue<T>(100UZ * signal + j);
                }
            }
            fourSignals.extents.push_back(std::int32_t{8});

            gr::test::RuntimeTest test;
            auto&                 source = test.emplace<ItemSource<gr::DataSet<T>>>();
            source._items                = {original, fourSignals};
            auto& forward                = test.emplace<DataSetToPacket<T>>();
            auto& firstPacket            = test.emplace<Collector<gr::Packet<T>>>();
            auto& back                   = test.emplace<PacketToDataSet<T>>();
            auto& returned               = test.emplace<Collector<gr::DataSet<T>>>();
            auto& again                  = test.emplace<DataSetToPacket<T>>();
            auto& lastPacket             = test.emplace<Collector<gr::Packet<T>>>();

            expect(test.connect(source, "out", forward, "in").has_value());
            expect(test.connect(forward, "out", firstPacket, "in").has_value());
            expect(test.connect(forward, "out", back, "in").has_value());
            expect(test.connect(back, "out", returned, "in").has_value());
            expect(test.connect(back, "out", again, "in").has_value());
            expect(test.connect(again, "out", lastPacket, "in").has_value());

            expect(test.run().has_value());

            expect(eq(firstPacket._items.size(), 2UZ)) << label;
            expect(eq(returned._items.size(), 2UZ)) << label;
            expect(eq(lastPacket._items.size(), 2UZ)) << label;
            if (returned._items.size() != 2UZ || firstPacket._items.size() != 2UZ || lastPacket._items.size() != 2UZ) {
                return;
            }

            const gr::Packet<T>&  packet    = firstPacket._items.front();
            const gr::DataSet<T>& reborn    = returned._items.front();
            const property_map    packetMap = packet.meta_information.at(0UZ);
            const property_map    rebornMap = reborn.meta_information.at(0UZ);

            // ── pinned exactly ──
            expect(std::ranges::equal(reborn.signal_values, original.signal_values)) << label << ": the payload, bit for bit";
            expect(eq(reborn.timestamp, original.timestamp)) << label;
            expect(eq(reborn.default_value, original.default_value)) << label;
            expect(eq(reborn.extents.size(), 1UZ)) << label;
            expect(eq(reborn.extents.front(), original.extents.front())) << label;
            expect(eq(reborn.signal_names.front(), original.signal_names.front())) << label;
            expect(eq(reborn.signal_quantities.size(), 1UZ)) << label;
            expect(eq(reborn.signal_quantities.front(), original.signal_quantities.front())) << label;
            expect(eq(reborn.signal_units.front(), original.signal_units.front())) << label;
            expect(eq(reborn.signal_ranges.size(), 1UZ)) << label;
            expect(eq(reborn.signal_ranges.front().min, original.signal_ranges.front().min)) << label;
            expect(eq(reborn.signal_ranges.front().max, original.signal_ranges.front().max)) << label;
            expect(std::ranges::equal(keysOf(rebornMap), keysOf(packetMap))) << label << ": every key the packet carried, and no other";
            expect(eq(read<float>(rebornMap, "sample_rate").value_or(0.f), 48000.f)) << label;
            expect(eq(read<std::uint64_t>(rebornMap, "sample_start").value_or(0ULL), std::uint64_t{4096ULL})) << label;
            expect(eq(read<gr::Size_t>(rebornMap, "n_pre").value_or(0U), gr::Size_t{3U})) << label << ": a private key survives both crossings";

            // ── asserted as a difference ──
            expect(eq(reborn.timing_events.size(), 1UZ)) << label;
            expect(reborn.timing_events.front().empty()) << label << ": the three annotations do not come back";
            expect(eq(read<gr::Size_t>(rebornMap, "dropped_events").value_or(0U), gr::Size_t{3U})) << label << ": and the record says so, which is the whole point of the key";
            expect(reborn.axis_names.empty()) << label << ": the axis is gone and is not rebuilt";
            expect(reborn.axis_units.empty()) << label;
            expect(reborn.axis_values.empty()) << label;
            expect(eq(returned._items[1UZ].signal_names.size(), 1UZ)) << label << ": a four-signal record returns as one signal";
            expect(eq(returned._items[1UZ].signal_values.size(), 8UZ)) << label;
            for (std::size_t j = 0UZ; j < 8UZ; ++j) {
                expect(eq(returned._items[1UZ].signal_values[j], sampleValue<T>(j))) << label << ": and it is signal 0, the other three appearing nowhere";
            }

            // ── the fixed point ──
            expect(std::ranges::equal(keysOf(lastPacket._items.front().meta_information.at(0UZ)), keysOf(packetMap))) << label << ": packet to record to packet is the identity on the map";
            expect(eq(read<float>(lastPacket._items.front().meta_information.at(0UZ), "sample_rate").value_or(0.f), 48000.f)) << label << ": route 1 again, never route 3";
            expect(eq(again.nMetaKeysOverridden, 0ULL)) << label << ": nothing the second forward pass derived disagreed with what it copied";
            expect(eq(back.nMetaKeysDropped, 0ULL)) << label;
            expect(eq(back.nSignalNamesSynthesized, 0ULL)) << label << ": the record's own name came back";

            // the produced record is admissible at both boundary blocks and by the framework's validator
            expect(gr::dataset::checkConsistency(reborn).has_value()) << label << ": an axis-free record validates, which is the landed framing chain's shape too";
        };
        roundTrip.template operator()<std::uint8_t>();
        roundTrip.template operator()<float>();
    };

    // criterion 10, runtime half
    "a rejection reason reaches a sink on reject and does not survive one ordinary block"_test = [] {
        const auto rejections = [](bool intervening) {
            std::vector<gr::Packet<std::uint8_t>> packets;
            gr::Packet<std::uint8_t>              noMap = makePacket<std::uint8_t>(4UZ);
            noMap.meta_information.clear();
            packets.push_back(std::move(noMap));
            packets.push_back(makePacket<std::uint8_t>(4UZ));

            gr::test::RuntimeTest test;
            auto&                 source = test.emplace<ItemSource<gr::Packet<std::uint8_t>>>();
            source._items                = packets;
            auto& convert                = test.emplace<PacketToDataSet<std::uint8_t>>();
            auto& records                = test.emplace<Collector<gr::DataSet<std::uint8_t>>>();
            auto& refused                = test.emplace<Collector<gr::Packet<std::uint8_t>>>();

            expect(test.connect(source, "out", convert, "in").has_value());
            expect(test.connect(convert, "out", records, "in").has_value());
            if (intervening) {
                auto& hop = test.emplace<gr::blocks::testing::Copy<gr::Packet<std::uint8_t>>>();
                expect(test.connect(convert, "reject", hop, "in").has_value());
                expect(test.connect(hop, "out", refused, "in").has_value());
            } else {
                expect(test.connect(convert, "reject", refused, "in").has_value());
            }

            expect(test.run().has_value());
            return std::pair<std::size_t, std::size_t>{refused._items.size(), offsetsOf(std::span<const Tag>(refused._tags), "discard_reason").size()};
        };

        const auto [directPackets, directReasons] = rejections(false);
        expect(eq(directPackets, 1UZ)) << "the refused packet leaves by the reject port";
        expect(eq(directReasons, 1UZ)) << "with its reason on a tag beside it";

        const auto [hoppedPackets, hoppedReasons] = rejections(true);
        expect(eq(hoppedPackets, 1UZ)) << "the packet itself survives the hop";
        expect(eq(hoppedReasons, 0UZ)) << "discard_reason is not a reserved key, so the default forwarder drops it";
    };

    // criterion 11 — the acceptance gate's own chain, in one process, with the leg that could not be built before
    "a payload corrupted in flight leaves by CrcCheck's fail port, exactly once"_test = [] {
        constexpr std::size_t kRecords     = 4UZ;
        constexpr std::size_t kRecordBytes = 250UZ;
        constexpr std::size_t kPacketBytes = kRecordBytes + 4UZ;

        std::vector<gr::DataSet<std::uint8_t>> records;
        for (std::size_t k = 0UZ; k < kRecords; ++k) {
            records.push_back(makeByteRecord(kRecordBytes, k * kRecordBytes));
        }

        gr::test::RuntimeTest test;
        auto&                 source = test.emplace<ItemSource<gr::DataSet<std::uint8_t>>>();
        source._items                = records;
        auto& appender               = test.emplace<gr::blocks::digital::CrcAppend>({{"width", gr::Size_t{32U}}, {"crc_byte_order", std::string("big")}});
        auto& forward                = test.emplace<DataSetToPacket<std::uint8_t>>({{"protocol_label", std::string("packet_link")}});
        auto& damage                 = test.emplace<PacketCorrupter>();
        damage._which                = kRecords - 1UZ;
        auto& back                   = test.emplace<PacketToDataSet<std::uint8_t>>();
        auto& checker                = test.emplace<gr::blocks::digital::CrcCheck>({{"width", gr::Size_t{32U}}, {"crc_byte_order", std::string("big")}});
        auto& verified               = test.emplace<Collector<gr::DataSet<std::uint8_t>>>();
        auto& failed                 = test.emplace<Collector<gr::DataSet<std::uint8_t>>>();

        expect(test.connect(source, "out", appender, "in").has_value());
        expect(test.connect(appender, "out", forward, "in").has_value());
        expect(test.connect(forward, "out", damage, "in").has_value());
        expect(test.connect(damage, "out", back, "in").has_value());
        expect(test.connect(back, "out", checker, "in").has_value());
        expect(test.connect(checker, "ok", verified, "in").has_value());
        expect(test.connect(checker, "fail", failed, "in").has_value());

        expect(test.run().has_value());

        expect(eq(verified._items.size(), kRecords - 1UZ)) << "the intact records verify";
        expect(eq(failed._items.size(), 1UZ)) << "and the corrupted one leaves by the fail port, counted once";
        expect(eq(back.nRejectedPackets, 0ULL)) << "nothing about the damage makes the packet inconvertible";

        for (const gr::DataSet<std::uint8_t>& record : verified._items) {
            expect(eq(record.signal_values.size(), kPacketBytes));
            expect(eq(read<bool>(record.meta_information.at(0UZ), "crc_ok").value_or(false), true));
        }
        expect(eq(failed._items.front().signal_values.size(), kPacketBytes));
        expect(eq(read<bool>(failed._items.front().meta_information.at(0UZ), "crc_ok").value_or(true), false)) << "the record says why it failed";
        expect(eq(failed._items.front().signal_values.front(), static_cast<std::uint8_t>(records.back().signal_values.front() ^ 0xFFU))) << "and it carries the damage rather than hiding it";
    };
};

int main() { /* not needed for UT */ }
