# EXCLUDE_FROM_ALL keeps these out of the default build. A benchmark is built by name when a measurement is wanted. The
# build-time budget of the block families does not cover a benchmark on every build.
function(gr_add_measurement_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-measurement gnuradio4::gr-testing)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

gr_add_measurement_benchmark(bm_Autocorrelation)

gr_add_measurement_benchmark(bm_Detectors)

gr_add_measurement_benchmark(bm_Kurtosis)

gr_add_measurement_benchmark(bm_PhaseUnwrap)

gr_add_measurement_benchmark(bm_PolarizationCombiner)

gr_add_measurement_benchmark(bm_SpectralEstimate)

gr_add_measurement_benchmark(bm_SpectralKurtosis)
