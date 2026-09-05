# Example graphs

Complete flowgraphs in the core's YAML form — the same dialect `gr::loadGrc` reads and the
control plane accepts as inline GRC — so an example is loadable by a graph editor, by a host
application and by the test suite without being compiled into anything.

The files live in `graphs/` and install to `share/gnuradio-4.0/examples/`, beside the recipes.
There was no home for a graph in this repository before: `ENABLE_EXAMPLES` gates example
_programs_ under each family's `src/`, which are C++ and are not loadable.

| Graph                              | What it shows                                                                                                                 |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| `fm_radio_mono.yaml`               | broadcast FM mono, wideband front end to audio: tuner, decimating channel filter, discriminator, audio resampler, de-emphasis |
| `channel_impairment_spectrum.yaml` | the channel models in the order a link applies them, read back as a Welch power spectral density                              |

## They run headless

Every graph loads and runs with no hardware, no sound device and no display, which is what lets
`test/qa_ExampleGraphs.cpp` gate them. The stand-ins are named in each file's header comment
along with what to put in their place:

- a **`SignalGenerator`** stands in for a `gr::blocks::sdr::SoapySource` (`-DGR4_ENABLE_SDR=ON`).
  It is paced to wall-clock time, so it delivers its stated rate the way a radio does.
- a **`DataSink`** or **`DataSetSink`** stands in for `gr::blocks::audio::AudioSink`
  (`-DGR4_ENABLE_AUDIO=ON`) or for one of studio's `gr::studio::*` sinks, which are in
  gnuradio4-studio and not here. Both fork sinks register under their `signal_name`, which is how
  the gate and any host application read the stream out.

## Two things to know when editing one

A studio series or waterfall sink's `in` is a **port collection**. A bare `in` on one connects
nothing and passes no data, silently; those edges are written `in#0`. The fork's own sinks have
plain ports and are written plainly.

A recipe (`gr::recipes::*`) **cannot be named in a graph file yet**. The importer instantiates a
block from its `id` alone and never hands the block's `parameters` to
`PluginLoader::instantiate`, so a recipe with a required exported parameter refuses and the file
fails to load. `fm_radio_mono.yaml` therefore writes `gr::recipes::WbfmMonoDemod`'s chain out and
states the numbers the recipe would have derived.
