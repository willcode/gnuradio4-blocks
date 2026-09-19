add_ut_test(qa_LteCellSearch)
target_link_libraries(qa_LteCellSearch PRIVATE gr-lte gr-channel)
# builds its scenes and both kernel references through the library FFT, and sweeps the correlator over hundreds of
# half-frames, so it needs the inliner depth -O2 provides
gr_pin_test_optimization(qa_LteCellSearch -O2)

add_ut_test(qa_LteCellScan)
# the shared block library of every family the scan graph names, so the ids in the file resolve in the global registry
# when the test loads it
target_link_libraries(
  qa_LteCellScan
  PRIVATE gnuradio-core
          gr-lte
          gnuradio4::gr-fileio
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrFileIoBlocksShared
          gnuradio4::GrLteBlocksShared)
target_compile_definitions(qa_LteCellScan
                           PRIVATE EXAMPLE_GRAPHS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../examples/graphs")
# builds its wideband scenes through the library resampler and sweeps the correlator over them, so it needs the inliner
# depth -O2 provides
gr_pin_test_optimization(qa_LteCellScan -O2)

# the recording leg is a unit of its own: a sweep of a wideband capture costs a polyphase design at the capture's own
# rate, which is most of a test binary's budget by itself
add_ut_test(qa_LteScanRecording)
target_link_libraries(qa_LteScanRecording PRIVATE gnuradio-core gr-lte gnuradio4::gr-fileio)
gr_pin_test_optimization(qa_LteScanRecording -O2)
set_property(
  TEST qa_LteScanRecording
  APPEND
  PROPERTY ENVIRONMENT_MODIFICATION "GR4_RECORDINGS_DIR=set:${GR4_RECORDINGS_DIR}")
set_property(TEST qa_LteScanRecording PROPERTY SKIP_RETURN_CODE 77)
