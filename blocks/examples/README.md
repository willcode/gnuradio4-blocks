# Example graphs

The example graphs are complete flowgraphs in the core's YAML form. `gr::loadGrc` reads the
same dialect, and the control plane accepts it as inline GRC. A graph editor, a host
application and the test suite can load an example without compiling anything.

The files live in `graphs/` and install to `share/gnuradio-4.0/examples/`, beside the recipes.
`ENABLE_EXAMPLES` gates the example _programs_ under each family's `src/`. Those programs are
C++ and cannot be loaded as graphs.

| Graph                              | What it shows                                                                                                                                  |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `fm_radio_mono.yaml`               | broadcast FM mono from a wideband front end to audio, through tuner, decimating channel filter, discriminator, audio resampler and de-emphasis |
| `channel_impairment_spectrum.yaml` | the channel models in the order a link applies them, read back as a Welch power spectral density                                               |

## The graphs run headless

Each graph loads and runs with no hardware, no sound device and no display.
`test/qa_ExampleGraphs.cpp` can therefore test them. Each file's header comment names its
stand-ins and what to put in their place.

- a **`SignalGenerator`** stands in for a `gr::blocks::sdr::SoapySource` (`-DGR4_ENABLE_SDR=ON`).
  It is paced to wall-clock time, and it delivers its stated rate as a radio does.
- a **`DataSink`** or **`DataSetSink`** stands in for `gr::blocks::audio::AudioSink`
  (`-DGR4_ENABLE_AUDIO=ON`) or for one of studio's `gr::studio::*` sinks. Those sinks are in
  gnuradio4-studio and not here. Both stand-in sinks register under their `signal_name`. The
  test and a host application read the stream through that name.

## Editing a graph

A studio series or waterfall sink's `in` is a **port collection**. A bare `in` on one connects
nothing and passes no data, with no error. Write those edges as `in#0`. The sinks in this
repository have plain ports and are written plainly.

`fm_radio_mono.yaml` writes out the chain of `gr::recipes::WbfmMonoDemod` and states the
numbers the recipe derives.
