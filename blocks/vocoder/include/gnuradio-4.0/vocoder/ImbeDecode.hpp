#ifndef GNURADIO_VOCODER_IMBE_DECODE_HPP
#define GNURADIO_VOCODER_IMBE_DECODE_HPP

#include <cstdint>
#include <span>
#include <string>

#include <gnuradio-4.0/Block.hpp>
#include <gnuradio-4.0/BlockRegistry.hpp>
#include <gnuradio-4.0/DataSet.hpp>
#include <gnuradio-4.0/Port.hpp>
#include <gnuradio-4.0/annotated.hpp>

#include <gnuradio-4.0/algorithm/vocoder/Imbe.hpp>

namespace gr::blocks::vocoder {

GR_REGISTER_BLOCK(gr::blocks::vocoder::ImbeDecode)

/*!
@brief IMBE 7200x4400 voice decode: parameter-word records in, deterministically placed PCM
records out.

Each input record carries whole codewords — u0..u7 in transmitted order, eight words per
codeword — and each codeword becomes exactly 160 samples of 8 kHz speech on the int16 grid over
32768, full scale ±1, makeup gain downstream (`gr::vocoder::ImbeDecoder`, the real-arithmetic
IMBE 7200x4400 decoder). One record leaves per record vocoded, its metadata carried through and
its `sample_start` on the 8000 Hz voice clock.

Placement comes from the record itself, so the output is sample-for-sample reproducible on any
schedule: a record's `voice_sample_start` metadata names its position on the voice clock, and
the output lands there, or directly after the previous record's voice where that is later. The
framing layer that produced the record owns the translation from its own timebase to the voice
clock; this block reads only the result. A gap between record positions is time a consumer
renders as silence.

Encrypted traffic is silent, which is not a decoding failure. A record not marked `clear` names
traffic this receiver cannot render as speech — running ciphertext through a vocoder produces
noise at full volume, which is worse than nothing — so with `mute_encrypted` (the default) it is
counted and skipped, and the silence it leaves is the same silence any quiet channel leaves.

Speech state belongs to an utterance rather than to the channel. The codec decodes each
parameter set against the previous set's spectral amplitudes and phases. A record placed more
than `kSpeechGapSamples` past the end of the last vocoded one opens an utterance of its own,
and the carried state is dropped before it is decoded — so decoding the same codewords again
later, offline, reaches the same verdict.
*/
struct ImbeDecode : Block<ImbeDecode> {
    using Description = Doc<"IMBE 7200x4400 voice decode from parameter-word records to PCM records, deterministically placed on the 8 kHz voice clock">;

    PortIn<DataSet<std::uint16_t>, Async> in;
    PortOut<DataSet<float>, Async>        out;

    Annotated<bool, "mute_encrypted", Doc<"skip records not marked clear rather than vocoding ciphertext">> mute_encrypted = true;

    GR_MAKE_REFLECTABLE(ImbeDecode, in, out, mute_encrypted);

    //! The silence that separates one utterance from the next, in voice samples: four voice
    //! frames, longer than any in-transmission gap, far shorter than the pause between
    //! transmissions.
    static constexpr std::uint64_t kSpeechGapSamples = 4ULL * 1440ULL;

    gr::vocoder::ImbeDecoder _decoder{};
    std::uint64_t            _lastVoiceEnd = 0ULL;
    std::uint64_t            _codewords    = 0ULL;
    std::uint64_t            _muted        = 0ULL;

    [[nodiscard]] work::Status processBulk(InputSpanLike auto& inSpan, OutputSpanLike auto& outSpan) {
        std::size_t made     = 0UZ;
        std::size_t consumed = 0UZ;
        for (; consumed < inSpan.size() && made < outSpan.size(); ++consumed) {
            const DataSet<std::uint16_t>& record = inSpan[consumed];
            const std::size_t             words  = record.signal_values.size();
            if (words == 0UZ || words % gr::vocoder::kImbeParameterWords != 0UZ) {
                continue; // no voice rides this record
            }
            const property_map* meta = record.meta_information.empty() ? nullptr : &record.meta_information[0UZ];

            bool clear = false;
            if (meta != nullptr) {
                if (const auto entry = meta->find(property_map::key_type("clear")); entry != meta->end()) {
                    clear = entry->second.value_or(false);
                }
            }
            const std::size_t count = words / gr::vocoder::kImbeParameterWords;
            if (!clear && mute_encrypted.value) {
                _muted += count;
                continue;
            }

            std::uint64_t at = 0ULL;
            if (meta != nullptr) {
                if (const auto entry = meta->find(property_map::key_type("voice_sample_start")); entry != meta->end()) {
                    at = entry->second.value_or(std::uint64_t{0ULL});
                }
            }
            if (at < _lastVoiceEnd) {
                at = _lastVoiceEnd;
            }
            if (!(at >= _lastVoiceEnd && at - _lastVoiceEnd <= kSpeechGapSamples) && _lastVoiceEnd != 0ULL) {
                _decoder.reset();
            }

            DataSet<float> pcm;
            pcm.signal_values.resize(count * gr::vocoder::kImbeSamplesPerFrame);
            for (std::size_t n = 0UZ; n < count; ++n) {
                _decoder.decode(std::span<const std::uint16_t, gr::vocoder::kImbeParameterWords>( //
                                    record.signal_values.data() + n * gr::vocoder::kImbeParameterWords, gr::vocoder::kImbeParameterWords),
                    std::span<float, gr::vocoder::kImbeSamplesPerFrame>(pcm.signal_values.data() + n * gr::vocoder::kImbeSamplesPerFrame, gr::vocoder::kImbeSamplesPerFrame));
            }
            _codewords += count;
            _lastVoiceEnd = at + count * gr::vocoder::kImbeSamplesPerFrame;

            pcm.extents.push_back(static_cast<std::int32_t>(pcm.signal_values.size()));
            pcm.signal_names.emplace_back(record.signal_names.empty() ? std::string("voice") : record.signal_names[0UZ]);
            pcm.timing_events.resize(1UZ);
            pcm.meta_information.resize(1UZ);
            if (meta != nullptr) {
                pcm.meta_information[0UZ] = *meta;
            }
            pcm.meta_information[0UZ]["sample_start"] = at;
            pcm.meta_information[0UZ]["sample_rate"]  = 8000.0f;
            outSpan[made]                             = std::move(pcm);
            ++made;
        }

        std::ignore = inSpan.consume(consumed);
        outSpan.publish(made);
        if (made == 0UZ && consumed == 0UZ) {
            return outSpan.size() == 0UZ ? work::Status::INSUFFICIENT_OUTPUT_ITEMS : work::Status::INSUFFICIENT_INPUT_ITEMS;
        }
        return work::Status::OK;
    }
};

} // namespace gr::blocks::vocoder

#endif // GNURADIO_VOCODER_IMBE_DECODE_HPP
