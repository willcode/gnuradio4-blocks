# EXCLUDE_FROM_ALL keeps these out of the default build: a benchmark is built by name when a measurement is wanted, and
# the build budget the block families are written under does not carry one on every build.
function(gr_add_analog_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-analog)
  # the benchmarks drive processBulk through the same span shim the QA uses, so the two measure the same call
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../test)
endfunction()

gr_add_analog_benchmark(bm_Agc)
gr_add_analog_benchmark(bm_FmEmphasis)
gr_add_analog_benchmark(bm_NoiseBlanker)
gr_add_analog_benchmark(bm_PowerMeter)
gr_add_analog_benchmark(bm_QuadratureDemod)
