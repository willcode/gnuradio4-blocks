#ifndef GNURADIO_OFFSET_WORD_FRAMING_HPP
#define GNURADIO_OFFSET_WORD_FRAMING_HPP

#include <cstdint>
#include <deque>
#include <format>
#include <string>
#include <vector>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/Tag.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/digital/CyclicSyndrome.hpp>

namespace gr::blocks::digital {

GR_REGISTER_BLOCK(gr::blocks::digital::OffsetWordSync)

struct OffsetWordSync : Block<OffsetWordSync, NoTagPropagation> {
    using Description = Doc<R""(
@brief Block synchronization by coset offset words: bits in, position-labeled data words out.

The scheme EN 50067's Annex C made familiar, as the general pattern it is: a systematic shortened cyclic block whose
checkword is xored with one of a small set of offset words naming the block's position in a cycle. The syndrome of a
valid block IS its offset word, so framing recovery is one polynomial reduction and a lookup — the checkword is the
sync, and there is no preamble.

Unlocked, the register slides bit by bit and locks where the last block_bits validate as position 0. Locked, every
block is tested against the position the cycle expects (or that position's stated alternate, accepted
unconditionally — the bit that selects between a pair may live in a block that was itself lost). Success and failure
BOTH emit the block's data bits, tagged with the position, whether the offset validated, and whether the alternate
was the match: the assembler downstream owns the accept policy, and a doubtful word is worth more to it than a
silent gap. A failure advances the expected position, so a lost block costs a block and not the lock; a run of
`relock_failures` consecutive failures returns to sliding acquisition.

One sharp edge of such constants travels with them: a code of small minimum distance can hold offset pairs one bit
error apart (RDS's C and D differ by exactly the syndrome of bit 18), so a lone validated block is not authoritative
about its position. Lock keys on the offset sequence, never on one checkword.
)"">;

    PortIn<std::uint8_t, Async>   in;
    PortOut<std::uint16_t, Async> out;

    Annotated<gr::Size_t, "polynomial", Doc<"the code's generator, its degree-check_bits term present; there is no default code">>                                                polynomial = 0U;
    Annotated<gr::Size_t, "check_bits", Doc<"checkword width; the offset words live in this many bits">>                                                                          check_bits = 0U;
    Annotated<gr::Size_t, "data_bits", Doc<"data width per block; block_bits = data_bits + check_bits">>                                                                          data_bits  = 0U;
    Annotated<std::vector<gr::Size_t>, "offsets", Doc<"the position cycle's offset words, position 0 first; sliding acquisition locks on position 0; there is no default cycle">> offsets{};
    Annotated<gr::Size_t, "alternate_position", Doc<"cycle position that also accepts alternate_word; any position past the cycle means none">>                                   alternate_position = 0xFFFFU;
    Annotated<gr::Size_t, "alternate_word", Doc<"the alternate offset accepted at alternate_position">>                                                                           alternate_word     = 0U;
    Annotated<gr::Size_t, "relock_failures", Doc<"consecutive failed positions before falling back to sliding acquisition">>                                                      relock_failures    = 8U;
    Annotated<std::string, "trigger_label", Doc<"the label written under the trigger_name key of the emitted tags">>                                                              trigger_label      = std::string("offset_word");

    GR_MAKE_REFLECTABLE(OffsetWordSync, in, out, polynomial, check_bits, data_bits, offsets, alternate_position, alternate_word, relock_failures, trigger_label);

    std::uint64_t _register  = 0ULL;
    std::size_t   _bitsIn    = 0UZ; /// bits shifted since the last accepted boundary (locked) or since reset (sliding)
    bool          _locked    = false;
    std::size_t   _position  = 0UZ; /// the cycle position the next block is expected to carry
    std::size_t   _failures  = 0UZ;
    std::size_t   _blockBits = 26UZ;
    std::uint64_t _blockMask = 0ULL;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) { rebuild(); }

    void start() { rebuild(); }

    void rebuild() {
        if (offsets.value.empty()) {
            throw gr::exception("offsets holds the position cycle and there is no default: the constants select a protocol");
        }
        // The kernel's constructor carries the width and generator refusals; building one here
        // surfaces them at settings time.
        const gr::digital::CyclicSyndrome code(static_cast<std::uint32_t>(polynomial.value), static_cast<unsigned>(check_bits.value), static_cast<unsigned>(data_bits.value));
        for (const gr::Size_t word : offsets.value) {
            if (static_cast<std::uint64_t>(word) >> code.checkBits != 0ULL) {
                throw gr::exception(std::format("offset word {:#x} does not fit the {}-bit checkword", static_cast<std::uint64_t>(word), code.checkBits));
            }
        }
        _blockBits = static_cast<std::size_t>(data_bits.value) + static_cast<std::size_t>(check_bits.value);
        _blockMask = (_blockBits == 64UZ) ? ~0ULL : ((1ULL << _blockBits) - 1ULL);
        reset();
    }

    void reset() {
        _register = 0ULL;
        _bitsIn   = 0UZ;
        _locked   = false;
        _position = 0UZ;
        _failures = 0UZ;
    }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        if (offsets.value.empty()) { // unconfigured: inert rather than framing on an arbitrary code
            std::ignore = inSpan.consume(0UZ);
            outSpan.publish(0UZ);
            return work::Status::ERROR;
        }
        const gr::digital::CyclicSyndrome code(static_cast<std::uint32_t>(polynomial.value), static_cast<unsigned>(check_bits.value), static_cast<unsigned>(data_bits.value));

        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            _register = ((_register << 1U) | static_cast<std::uint64_t>(inSpan[consumed] != 0U ? 1U : 0U)) & _blockMask;
            ++_bitsIn;

            if (!_locked) {
                if (_bitsIn >= _blockBits && code.syndrome(_register) == static_cast<std::uint16_t>(offsets.value[0])) {
                    _locked   = true;
                    _position = 0UZ;
                    _failures = 0UZ;
                    emit(code, outSpan, made, true, false);
                    _bitsIn   = 0UZ;
                    _position = 1UZ % offsets.value.size(); // the locking block WAS position 0; the next is expected one on
                }
                continue;
            }
            if (_bitsIn < _blockBits) {
                continue;
            }

            const std::uint16_t syndrome  = code.syndrome(_register);
            const bool          expected  = syndrome == static_cast<std::uint16_t>(offsets.value[_position]);
            const bool          alternate = _position == static_cast<std::size_t>(alternate_position.value) && syndrome == static_cast<std::uint16_t>(alternate_word.value);
            const bool          good      = expected || alternate;
            emit(code, outSpan, made, good, alternate);
            _bitsIn   = 0UZ;
            _position = (_position + 1UZ) % offsets.value.size();
            _failures = good ? 0UZ : _failures + 1UZ;
            if (_failures >= static_cast<std::size_t>(relock_failures.value)) {
                // Sliding resumes with the register's history intact: the bits already seen may
                // hold the next lock.
                _locked   = false;
                _failures = 0UZ;
                _bitsIn   = _blockBits; // the register is full; every further bit may test
            }
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    void emit(const gr::digital::CyclicSyndrome& code, OutputSpanLike auto& outSpan, std::size_t& made, bool good, bool alternate) {
        // The position reported is the one this block occupies; on the sliding lock that is 0.
        outSpan[made] = static_cast<std::uint16_t>((_register >> code.checkBits) & ((1ULL << code.dataBits) - 1ULL));
        outSpan.publishTag(property_map{{gr::tag::TRIGGER_NAME.shortKey(), trigger_label.value}, //
                               {gr::tag::TRIGGER_OFFSET.shortKey(), 0.0f},                       //
                               {gr::tag::TRIGGER_META_INFO.shortKey(), property_map{{"position", static_cast<gr::Size_t>(_position)}, {"offset_ok", good}, {"alternate", alternate}}}},
            made);
        ++made;
    }
};

GR_REGISTER_BLOCK(gr::blocks::digital::GroupAssembler)

struct GroupAssembler : Block<GroupAssembler, NoTagPropagation> {
    using Description = Doc<R""(
@brief The N-of-M good-block gate: position-labeled words in, one record per completed cycle out.

Consumes the word stream `OffsetWordSync` emits — each word tagged with its cycle position and whether its offset
validated — and assembles one `DataSet` record per completed cycle whose valid count reaches `min_good`. A group
with fewer good words is dropped and counted, never published: a consumer of the record port reads accepted groups
and nothing else. The record's data is the cycle's words in position order; its metadata carries `protocol`,
`crc_ok` (every word valid), `good_words`, `alternate_positions` (which positions validated on their stated
alternate — RDS reads its version bit there when block B was lost), and `sequence`. A word tagged position 0 always
begins a new group, so a relock upstream cannot splice two half groups into one record.
)"">;

    PortIn<std::uint16_t, Async>           in;
    PortOut<DataSet<std::uint16_t>, Async> out;

    Annotated<gr::Size_t, "group_size", Doc<"words per cycle; a record carries exactly this many">>                                  group_size = 4U;
    Annotated<gr::Size_t, "min_good", Doc<"offset-valid words a group needs to publish; group_size accepts only clean groups">>      min_good   = 4U;
    Annotated<std::string, "protocol", Doc<"the record metadata's protocol key, and the record's signal name; there is no default">> protocol{};
    Annotated<std::string, "trigger_label", Doc<"the upstream trigger_name label whose tags carry the position">>                    trigger_label = std::string("offset_word");

    GR_MAKE_REFLECTABLE(GroupAssembler, in, out, group_size, min_good, protocol, trigger_label);

    std::vector<std::uint16_t> _words{};
    std::vector<bool>          _good{};
    std::vector<gr::Size_t>    _alternates{};
    std::size_t                _filled   = 0UZ;
    std::uint64_t              _sequence = 0ULL;
    std::uint64_t              _dropped  = 0ULL;

    void settingsChanged(const property_map& /*oldSettings*/, const property_map& /*newSettings*/) {
        if (group_size < 1U) {
            throw gr::exception("group_size must be at least one");
        }
        if (min_good > group_size) {
            throw gr::exception(std::format("min_good ({}) cannot exceed group_size ({})", min_good.value, group_size.value));
        }
        if (protocol.value.empty()) {
            throw gr::exception("protocol names the records this block emits and there is no default");
        }
        _words.assign(static_cast<std::size_t>(group_size.value), 0U);
        _good.assign(static_cast<std::size_t>(group_size.value), false);
        _alternates.clear();
        _filled = 0UZ;
    }

    [[nodiscard]] std::uint64_t droppedGroups() const noexcept { return _dropped; }

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        const property_map::key_type nameKey{gr::tag::TRIGGER_NAME.shortKey()};
        const property_map::key_type metaKey{gr::tag::TRIGGER_META_INFO.shortKey()};
        const auto                   size = static_cast<std::size_t>(group_size.value);

        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            // The word's annotation arrives as the trigger tag at its own index.
            std::size_t position  = size; // no position: the word cannot be placed
            bool        good      = false;
            bool        alternate = false;
            for (const gr::Tag& tag : inSpan.rawTags) {
                if (static_cast<std::size_t>(tag.index) != static_cast<std::size_t>(inSpan.streamIndex) + consumed) {
                    continue;
                }
                const auto label = tag.map.find(nameKey);
                if (label == tag.map.end() || label->second.value_or(std::string_view{}) != trigger_label.value) {
                    continue;
                }
                if (const auto meta = tag.map.find(metaKey); meta != tag.map.end()) {
                    if (const auto* info = meta->second.get_if<property_map>(); info != nullptr) {
                        if (const auto at = info->find("position"); at != info->end()) {
                            position = static_cast<std::size_t>(at->second.value_or(gr::Size_t(size)));
                        }
                        if (const auto ok = info->find("offset_ok"); ok != info->end()) {
                            good = ok->second.value_or(false);
                        }
                        if (const auto alt = info->find("alternate"); alt != info->end()) {
                            alternate = alt->second.value_or(false);
                        }
                    }
                }
                break;
            }
            if (position >= size) {
                continue; // an unlabeled word belongs to no cycle
            }

            if (position == 0UZ) { // a new cycle always starts clean, whatever was pending
                if (_filled != 0UZ) {
                    ++_dropped;
                }
                std::fill(_good.begin(), _good.end(), false);
                _alternates.clear();
                _filled = 0UZ;
            } else if (position != _filled) {
                // A word out of step means the cycle it belonged to is already lost.
                if (_filled != 0UZ) {
                    ++_dropped;
                    std::fill(_good.begin(), _good.end(), false);
                    _alternates.clear();
                    _filled = 0UZ;
                }
                continue;
            }

            _words[position] = inSpan[consumed];
            _good[position]  = good;
            if (alternate) {
                _alternates.push_back(static_cast<gr::Size_t>(position));
            }
            _filled = position + 1UZ;

            if (_filled == size) {
                std::size_t goodCount = 0UZ;
                for (const bool g : _good) {
                    goodCount += g ? 1UZ : 0UZ;
                }
                if (goodCount >= static_cast<std::size_t>(min_good.value)) {
                    outSpan[made] = assemble(goodCount);
                    ++made;
                } else {
                    ++_dropped;
                }
                std::fill(_good.begin(), _good.end(), false);
                _alternates.clear();
                _filled = 0UZ;
            }
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }

private:
    [[nodiscard]] DataSet<std::uint16_t> assemble(std::size_t goodCount) {
        DataSet<std::uint16_t> record;
        record.signal_values.assign(_words.begin(), _words.end());
        record.extents.push_back(static_cast<std::int32_t>(_words.size()));
        record.signal_names.push_back(protocol.value);
        record.timing_events.resize(1UZ);

        record.meta_information.resize(1UZ);
        property_map& map          = record.meta_information[0UZ];
        map["protocol"]            = protocol.value;
        map["crc_ok"]              = goodCount == _words.size();
        map["good_words"]          = static_cast<gr::Size_t>(goodCount);
        map["alternate_positions"] = _alternates;
        map["sequence"]            = _sequence++;
        return record;
    }
};

} // namespace gr::blocks::digital

#endif // GNURADIO_OFFSET_WORD_FRAMING_HPP
