#include <boost/ut.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <gnuradio-4.0/Graph.hpp>
#include <gnuradio-4.0/Scheduler.hpp>
#include <gnuradio-4.0/digital/OffsetWordFraming.hpp>

namespace {

using gr::blocks::digital::GroupAssembler;
using gr::blocks::digital::OffsetWordSync;

// EN 50067's constants, stated once for every leg.
const gr::property_map kRdsSync{{"polynomial", gr::Size_t(0x5B9)}, {"check_bits", gr::Size_t(10)}, {"data_bits", gr::Size_t(16)}, //
    {"offsets", std::vector<gr::Size_t>{0x0FCU, 0x198U, 0x168U, 0x1B4U}}, {"alternate_position", gr::Size_t(2)}, {"alternate_word", gr::Size_t(0x350)}};

/// The framer conformance vector: 446 bits msb-first, four transmitted groups of which the
/// middle-of-the-air's one flipped bit drops exactly one — an oracle this tree did not compute.
constexpr std::string_view kVectorHex  = "2aaaaaaa5bab1302780835babf815115fb5bab13027853a5babf814d51e95bab130a78b2c5babf8121a4405bab130a7832c5babf8121a440";
constexpr std::size_t      kVectorBits = 446UZ;

[[nodiscard]] std::vector<std::uint8_t> vectorBits() {
    std::vector<std::uint8_t> bits;
    bits.reserve(kVectorHex.size() * 4UZ);
    for (const char digit : kVectorHex) {
        const unsigned v = digit <= '9' ? static_cast<unsigned>(digit - '0') : static_cast<unsigned>(digit - 'a') + 10U;
        for (unsigned b = 4U; b-- > 0U;) {
            bits.push_back(static_cast<std::uint8_t>((v >> b) & 1U));
        }
    }
    // The constant is 446 bits; the leading hex digit carries two bits of padding.
    bits.erase(bits.begin(), bits.begin() + static_cast<std::ptrdiff_t>(bits.size() - kVectorBits));
    return bits;
}

const std::vector<std::uint16_t> kVectorWords{0x5babU, 0x09e0U, 0x5babU, 0x5445U, 0x5babU, 0x09e1U, 0x5babU, 0x5354U, 0x5babU, 0x29e0U, 0x5babU, 0x4869U};

template<typename T>
struct FiniteSource : gr::Block<FiniteSource<T>> {
    gr::PortOut<T> out;
    GR_MAKE_REFLECTABLE(FiniteSource, out);
    std::vector<T>                 _data;
    std::size_t                    _pos{0UZ};
    [[nodiscard]] gr::work::Status processBulk(gr::OutputSpanLike auto& outSpan) noexcept {
        const std::size_t n = std::min(outSpan.size(), _data.size() - _pos);
        std::copy_n(_data.begin() + static_cast<std::ptrdiff_t>(_pos), n, outSpan.begin());
        outSpan.publish(n);
        _pos += n;
        return _pos == _data.size() ? gr::work::Status::DONE : gr::work::Status::OK;
    }
};

struct RecordSink : gr::Block<RecordSink> {
    gr::PortIn<gr::DataSet<std::uint16_t>, gr::Async> in;
    GR_MAKE_REFLECTABLE(RecordSink, in);
    std::vector<gr::DataSet<std::uint16_t>> _records;
    [[nodiscard]] gr::work::Status          processBulk(gr::InputSpanLike auto& inSpan) {
        for (const auto& record : inSpan) {
            _records.push_back(record);
        }
        std::ignore = inSpan.consume(inSpan.size());
        return gr::work::Status::OK;
    }
};

/// Run bits through sync -> assembler on the Simple scheduler to completion; empty on a hang.
[[nodiscard]] std::vector<gr::DataSet<std::uint16_t>> runChain(std::vector<std::uint8_t> bits, gr::Size_t minGood = 4U) {
    gr::Graph flow;
    auto&     src = flow.emplaceBlock<FiniteSource<std::uint8_t>>();
    src._data     = std::move(bits);
    auto& sync    = flow.emplaceBlock<OffsetWordSync>(gr::property_map(kRdsSync));
    auto& asm_    = flow.emplaceBlock<GroupAssembler>({{"group_size", gr::Size_t(4)}, {"min_good", minGood}, {"protocol", std::string("rds")}});
    auto& sink    = flow.emplaceBlock<RecordSink>();
    boost::ut::expect(flow.connect<"out", "in">(src, sync).has_value());
    boost::ut::expect(flow.connect<"out", "in">(sync, asm_).has_value());
    boost::ut::expect(flow.connect<"out", "in">(asm_, sink).has_value());

    gr::scheduler::Simple<> scheduler;
    boost::ut::expect(scheduler.exchange(std::move(flow)).has_value());
    std::atomic<bool> done{false};
    std::thread       runner([&scheduler, &done] {
        std::ignore = scheduler.runAndWait();
        done        = true;
    });
    const auto        start = std::chrono::steady_clock::now();
    while (!done.load() && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const bool finished = done.load();
    if (!finished) {
        scheduler.requestStop();
    }
    runner.join();
    return finished ? sink._records : std::vector<gr::DataSet<std::uint16_t>>{};
}

} // namespace

const boost::ut::suite<"OffsetWordFraming"> offsetWordFramingTests = [] {
    using namespace boost::ut;

    "the conformance vector: three groups, the damaged one dropped without losing lock"_test = [] {
        const auto records = runChain(vectorBits());
        expect(eq(records.size(), 3UZ)) << "fifteen blocks validate, one fails, three groups survive";

        std::vector<std::uint16_t> words;
        for (const auto& record : records) {
            expect(eq(record.signal_values.size(), 4UZ));
            words.insert(words.end(), record.signal_values.begin(), record.signal_values.end());
            const auto& meta = record.meta_information[0UZ];
            expect(that % meta.at("crc_ok").value_or(false)) << "an accepted group is clean at min_good 4";
            expect(that % (meta.at("protocol").value_or(std::string{}) == std::string("rds")));
        }
        expect(that % (words == kVectorWords)) << "the words, exactly as the vector states them";
    };

    "acquisition does not depend on where in the stream the lock begins"_test = [] {
        auto bits = vectorBits();
        bits.insert(bits.begin(), 17UZ, std::uint8_t(1)); // an arbitrary misaligning prefix
        const auto records = runChain(std::move(bits));
        expect(eq(records.size(), 3UZ)) << "sliding acquisition finds the same lock through a prefix";
    };

    "min_good below the group size admits the damaged group"_test = [] {
        const auto records = runChain(vectorBits(), 3U);
        expect(eq(records.size(), 4UZ)) << "three-of-four accepts the group the flipped bit damaged";
        std::size_t dirty = 0UZ;
        for (const auto& record : records) {
            if (!record.meta_information[0UZ].at("crc_ok").value_or(true)) {
                ++dirty;
            }
        }
        expect(eq(dirty, 1UZ)) << "exactly one accepted group carries the failure it survived";
    };

    "refusals fire by name"_test = [] {
        const auto refused = [](gr::property_map settings) {
            return boost::ut::expect(throws([&settings] {
                OffsetWordSync block{gr::property_map(settings)};
                block.settings().init();
                std::ignore = block.settings().applyStagedParameters();
                block.start();
            }));
        };
        refused({}) << "no offset cycle";
        gr::property_map badOffset(kRdsSync);
        badOffset.insert_or_assign("offsets", std::vector<gr::Size_t>{0x0FCU, 0x1198U});
        refused(std::move(badOffset)) << "an offset word past the checkword width";

        expect(throws([] {
            GroupAssembler block{{{"group_size", gr::Size_t(4)}, {"min_good", gr::Size_t(5)}, {"protocol", std::string("rds")}}};
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        })) << "min_good past the group size";
        expect(throws([] {
            GroupAssembler block{{{"group_size", gr::Size_t(4)}, {"min_good", gr::Size_t(4)}}};
            block.settings().init();
            std::ignore = block.settings().applyStagedParameters();
        })) << "a record without a protocol name";
    };
};

int main() { /* not needed for UT */ }
