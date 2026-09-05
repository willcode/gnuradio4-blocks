# Recipes — composed capabilities that need no new code

A recipe is a YAML subgraph definition: a named, loadable block whose implementation is a
composition of existing blocks with the derived values already worked out. Recipes exist
for the compositions a user would otherwise have to re-derive — a discriminator gain from
a rate and a deviation, a de-emphasis constant, a matched filter's construction — and the
scheduler's chain fusion makes the composed form cost what a hand-fused block would.

## What ships

| Recipe                                 | What it composes                                                                                                                                    | Required parameters                                                                                                            |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `gr::recipes::AfskDemod`               | Hilbert branch and matching delay, tuner, channel filter, discriminator, lowpass, timing recovery                                                   | `sample_rate`, `symbol_rate`, `mark_hz`, `space_hz`                                                                            |
| `gr::recipes::BpskDemod`               | `BpskFrontEnd`'s five stages, a Costas loop at order 2, the real part                                                                               | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::BpskFrontEnd`            | tuner, decimating channel filter, AGC, frequency-locked loop, timing recovery                                                                       | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::CcsdsConcatenatedFrames` | access-code correlator, packet framer, soft-decision Viterbi decode, record trim, descrambler, bit repacking, Reed-Solomon decode                   | `frame_length`, `error_capability`, `code`, `basis`, `convolutional`, `encoded_marker`, `sync_errors`                          |
| `gr::recipes::CcsdsRsFrames`           | bit slicer, access-code correlator, packet framer, descrambler, bit repacking, Reed-Solomon decode                                                  | `frame_length`, `error_capability`, `code`, `basis`, `sync_errors`                                                             |
| `gr::recipes::DbpskDemod`              | `BpskFrontEnd`'s five stages, a one-symbol phasor, the real part                                                                                    | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::FskDemodAudio`           | `FskDemod`'s chain from the discriminator on — post-detection lowpass, timing recovery, slicer — for a stream that is already detected              | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::FskDemodDcBlock`         | `FskDemod`'s chain with a DC blocker after the discriminator, soft symbols out                                                                      | `sample_rate`, `symbol_rate`, `modulation_index`                                                                               |
| `gr::recipes::HdlcDeframe`             | NRZI line decoding, HDLC delimiter extraction, the ISO/IEC 13239 frame check; the tail of every HDLC link, AX.25 and the AIS VHF data link included | `max_payload_items`                                                                                                            |
| `gr::recipes::KissFileRead`            | file source, delimiter extraction, KISS decode                                                                                                      | `file_name`, `max_payload_items`                                                                                               |
| `gr::recipes::KissFileWrite`           | KISS encode, delimiter framing, stream flatten, file sink                                                                                           | `file_name`                                                                                                                    |
| `gr::recipes::KissServe`               | KISS encode, delimiter framing, stream flatten, TCP byte sink                                                                                       | `endpoint`, `queue_bytes`                                                                                                      |
| `gr::recipes::KissStreamDecode`        | record trim, stream flatten, delimiter extraction, KISS decode                                                                                      | `max_payload_items`                                                                                                            |
| `gr::recipes::NbfmDemod`               | discriminator and de-emphasis, general form                                                                                                         | `sample_rate`, `deviation`                                                                                                     |
| `gr::recipes::OfdmDemodulator`         | Schmidl-Cox synchronization, cyclic-prefix removal, channel equalization                                                                            | `fft_len`, `data_carriers`, `pilot_carriers`, `pilot_symbols`, `sync_word`, `n_sync`, `frame_len`, `cp_len`, `preamble_cp_len` |
| `gr::recipes::OfdmModulator`           | carrier allocation, cyclic-prefix insertion                                                                                                         | `fft_len`, `data_carriers`, `pilot_carriers`, `pilot_symbols`, `sync_words`, `frame_len`, `cp_len`                             |
| `gr::recipes::SampleClockOffset`       | a resampling by `1 + ppm*1e-6`, tags re-originated                                                                                                  | none                                                                                                                           |
| `gr::recipes::WbfmMonoDemod`           | tuner, decimating channel filter, discriminator, audio resampler, de-emphasis                                                                       | `sample_rate`                                                                                                                  |

A recipe with no required parameter instantiates bare; the rest name what they are missing.

A recipe cannot yet name another indexed recipe as an interior block when that recipe has
a required parameter: the graph importer creates an interior block from its `id` alone and
never hands `PluginLoader::instantiate` the block's `parameters`, so the nested recipe
refuses for the parameters it was never given. `BpskDemod` and `DbpskDemod` therefore write
out the five stages `BpskFrontEnd` holds instead of naming it, and each says so in its
header. `BpskFrontEnd` ships and is usable on its own; when the loader forwards parameters,
those five blocks collapse into one line in each file. It is the same limitation
`blocks/examples/graphs` records for naming a recipe from a graph file.

## The dialect

Each recipe is one file in the core's YAML-definitions form: a `definition_metadata`
header naming the `block_type`, and a single `SUBGRAPH` block whose `graph` lists the
interior blocks by registry id, their `connections`, and the `exported_ports` that become
the composite's own ports. `index.yaml` in this directory is the loader's catalog; a
recipe ships by appearing there, and its `modified` stamp should be bumped with every
edit so the catalog keeps an honest account of each definition's age. A loader reads a
definition that is on disk at every load, so the edit itself reaches the generator and
the tests immediately.

A recipe may declare `exported_parameters` on its `SUBGRAPH`: named, typed values with a
`doc` and an optional `default`, which become the composite's own settings. One without a
default is required, and instantiating the recipe without it is refused by name. An
interior setting written as `=...` is bound to them — a numeric parameter through an
arithmetic expression, so a derivation is written where it can be read and re-derived
whenever a parameter changes; a string- or vector-typed parameter by substitution alone,
`=name` handing the value through as it stands, because there is no arithmetic to do on
one. A literal beginning with `=` is written `\=`.

The derivations themselves belong in the file's header comment: a recipe pins the numbers
_and_ shows where they came from, so a reader can build a variant instead of guessing.

## Reaching recipes from code

Nothing recipe-specific is required. A `gr::PluginLoader` given this directory among its
paths reads `index.yaml` and registers every listed recipe; `loader.instantiate(name)`
then returns the composite block exactly as it returns a compiled one, exported ports in
place. Code that creates blocks by registry name — a YAML chain loader, a palette, a
`Graph` builder — consumes recipes without knowing they are YAML.

## The gate

`test/qa_Recipes.cpp` loads this directory through the standard machinery, instantiates
every indexed recipe, and asserts its exported ports. A recipe that does not load and
instantiate does not merge.
