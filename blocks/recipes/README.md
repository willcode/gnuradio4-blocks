# Recipes

A recipe is a YAML subgraph definition. It is a named, loadable block built from existing
blocks, with the derived values already worked out. Recipes cover the compositions a user
would otherwise derive again. Examples are a discriminator gain from a rate and a
deviation, a de-emphasis constant and a matched filter's construction. The scheduler's
chain fusion makes the composed form run like a hand-fused block.

## What ships

| Recipe                                 | What it composes                                                                                                                  | Required parameters                                                                                                            |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `gr::recipes::AfskDemod`               | Hilbert branch and matching delay, tuner, channel filter, discriminator, lowpass, timing recovery                                 | `sample_rate`, `symbol_rate`, `mark_hz`, `space_hz`                                                                            |
| `gr::recipes::BpskDemod`               | `BpskFrontEnd`'s five stages, a Costas loop at order 2, the real part                                                             | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::BpskFrontEnd`            | tuner, decimating channel filter, AGC, frequency-locked loop, timing recovery                                                     | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::CcsdsConcatenatedFrames` | access-code correlator, packet framer, soft-decision Viterbi decode, record trim, descrambler, bit repacking, Reed-Solomon decode | `frame_length`, `error_capability`, `code`, `basis`, `convolutional`, `encoded_marker`, `sync_errors`                          |
| `gr::recipes::CcsdsRsFrames`           | bit slicer, access-code correlator, packet framer, descrambler, bit repacking, Reed-Solomon decode                                | `frame_length`, `error_capability`, `code`, `basis`, `sync_errors`                                                             |
| `gr::recipes::DbpskDemod`              | `BpskFrontEnd`'s five stages, a one-symbol phasor, the real part                                                                  | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::FskDemod`                | channel filter, discriminator, post-detection lowpass, burst timing preset, timing recovery, slicer                               | `sample_rate`, `symbol_rate`, `modulation_index`                                                                               |
| `gr::recipes::FskDemodAudio`           | post-detection lowpass, timing recovery and slicer, `FskDemod`'s chain after the discriminator, for a stream already detected     | `sample_rate`, `symbol_rate`                                                                                                   |
| `gr::recipes::FskDemodDcBlock`         | `FskDemod`'s chain with a DC blocker after the discriminator, soft symbols out                                                    | `sample_rate`, `symbol_rate`, `modulation_index`                                                                               |
| `gr::recipes::HdlcDeframe`             | NRZI line decoding, HDLC delimiter extraction and the ISO/IEC 13239 frame check, as AX.25 and the AIS VHF data link use them      | `max_payload_items`                                                                                                            |
| `gr::recipes::KissFileRead`            | file source, delimiter extraction, KISS decode                                                                                    | `file_name`, `max_payload_items`                                                                                               |
| `gr::recipes::KissFileWrite`           | KISS encode, delimiter framing, stream flatten, file sink                                                                         | `file_name`                                                                                                                    |
| `gr::recipes::KissServe`               | KISS encode, delimiter framing, stream flatten, TCP byte sink                                                                     | `endpoint`, `queue_bytes`                                                                                                      |
| `gr::recipes::KissStreamDecode`        | record trim, stream flatten, delimiter extraction, KISS decode                                                                    | `max_payload_items`                                                                                                            |
| `gr::recipes::NbfmDemod`               | discriminator and de-emphasis, general form                                                                                       | `sample_rate`, `deviation`                                                                                                     |
| `gr::recipes::OfdmDemodulator`         | Schmidl-Cox synchronization, cyclic-prefix removal, channel equalization                                                          | `fft_len`, `data_carriers`, `pilot_carriers`, `pilot_symbols`, `sync_word`, `n_sync`, `frame_len`, `cp_len`, `preamble_cp_len` |
| `gr::recipes::OfdmModulator`           | carrier allocation, cyclic-prefix insertion                                                                                       | `fft_len`, `data_carriers`, `pilot_carriers`, `pilot_symbols`, `sync_words`, `frame_len`, `cp_len`                             |
| `gr::recipes::SampleClockOffset`       | a resampling by `1 + ppm*1e-6`, tags re-originated                                                                                | none                                                                                                                           |
| `gr::recipes::WbfmMonoDemod`           | tuner, decimating channel filter, discriminator, audio resampler, de-emphasis                                                     | `sample_rate`                                                                                                                  |

A recipe with no required parameter instantiates with no arguments. A recipe missing a
required parameter names the missing one.

`BpskDemod` and `DbpskDemod` write out the five stages `BpskFrontEnd` holds.
`BpskFrontEnd` also runs on its own.

## The dialect

Each recipe is one file in the core's YAML definitions form. A `definition_metadata`
header names the `block_type`. A single `SUBGRAPH` block follows. Its `graph` lists the
interior blocks by registry id, their `connections` and the `exported_ports` that become
the composite's own ports. `index.yaml` in this directory is the loader's catalog. A
recipe ships by appearing there. Bump its `modified` stamp with every edit, and the
catalog keeps a true record of each definition's age. A loader reads each definition from
disk at every load. An edit therefore reaches the generator and the tests at once.

A recipe may declare `exported_parameters` on its `SUBGRAPH`. These are named, typed
values with a `doc` and an optional `default`, and they become the composite's own
settings. A parameter without a default is required. Instantiating the recipe without it
is refused by name. An interior setting written as `=...` is bound to the parameters. A
numeric parameter binds through an arithmetic expression. The derivation is then written
where it can be read, and it is evaluated again whenever a parameter changes. A string or
vector parameter binds by substitution alone. `=name` passes the value through as it
stands, as there is no arithmetic to do on one. A literal beginning with `=` is written
`\=`.

The derivations belong in the file's header comment. A recipe pins the numbers _and_
shows where they came from. A reader can then build a variant without guessing.

## Reaching recipes from code

Code needs nothing specific to recipes. A `gr::PluginLoader` given this directory among
its paths reads `index.yaml` and registers every listed recipe. `loader.instantiate(name)`
then returns the composite block with its exported ports in place, as it returns a
compiled block. Code that creates blocks by registry name uses recipes without knowing
they are YAML. A YAML chain loader, a palette and a `Graph` builder are such code.

## The gate

`test/qa_Recipes.cpp` loads this directory through the standard machinery. It instantiates
every indexed recipe and checks its exported ports. A recipe that does not load and
instantiate fails the test.
