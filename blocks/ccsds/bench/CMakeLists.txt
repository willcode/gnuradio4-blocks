# EXCLUDE_FROM_ALL keeps these benchmarks out of the default build. A benchmark is built by naming its target.
# Building every benchmark on every build would exceed the build budget of the block families.
function(gr_add_ccsds_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-ccsds gr-testing)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

gr_add_ccsds_benchmark(bm_Ccsds)
