# EXCLUDE_FROM_ALL keeps these out of the default build: a benchmark is built by name when a measurement is wanted, and
# the build budget the block families are written under does not carry one on every build.
function(gr_add_lte_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-lte)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../test)
endfunction()

gr_add_lte_benchmark(bm_PssCorrelator)
