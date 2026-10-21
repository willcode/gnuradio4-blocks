# PocketFFT

Upstream project: <https://github.com/mreineck/pocketfft>, `cpp` branch.

Vendored from commit `076cb3d` of 2023-02-14, by way of the distribution package
`pocketfft-devel-1.0^git20230214.076cb3d`. SHA-256 of `pocketfft_hdronly.h` as vendored:
`0eaea21304bf324639ff11022ee63faf48a8fab64afe5c3c2d1efefbe1d7dfa6`. Copied here on 2026-09-06.

The header is taken verbatim and is not reformatted, respelled or otherwise edited; a change here would be an upgrade
to another upstream revision and nothing else. `algorithm/CMakeLists.txt` carries `third_party/include/` as a system
include for the same reason.

The directories above this one mirror the installed layout, so the include path `gnuradio-4.0/third_party/pocketfft/`
is the same in the source tree and in an install. A consumer that finds this project through pkg-config gets only
`-I${includedir}`, and that is all this path needs.

License: BSD-3-Clause, copyright the Max-Planck-Society and Peter Bell, with the odd-length DCT-IV path additionally
copyright Matteo Frigo and the Massachusetts Institute of Technology under the same three clauses. The full notice is
in the header. `LICENSE.md` is upstream's license file.

Used by `gnuradio-4.0/algorithm/fourier/fft.hpp` in `gr::algorithm::FFT`. The `PocketFFT` backend holds its own
`pocketfft_c` or `pocketfft_r` plan. The `FourStep` backend calls `pocketfft::c2c` on a two-dimensional shape and builds
its two plans on each call, since the header's plan cache is compiled out. The passes of the `FourStep`
backend run on the header's thread pool. Include the header through `fft.hpp`, which sets `POCKETFFT_PTHREADS`.
