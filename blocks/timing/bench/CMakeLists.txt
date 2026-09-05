# EXCLUDE_FROM_ALL keeps these out of the default build: a benchmark is built by name when a measurement is wanted, and
# the build budget the block families are written under does not carry one on every build.
function(gr_add_timing_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-timing gnuradio4::gr-testing)
  # the test directory carries the span pair a processBulk is driven through outside a scheduler
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../test)
endfunction()

gr_add_timing_benchmark(bm_DiscontinuityMonitor)
