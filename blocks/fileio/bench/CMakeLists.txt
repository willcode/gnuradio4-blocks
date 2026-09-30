# EXCLUDE_FROM_ALL keeps this out of the default build. A benchmark is built by name when a measurement is wanted.
# The build budget of the block families does not include a benchmark on every build.
function(gr_add_fileio_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-fileio)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

gr_add_fileio_benchmark(bm_SigMfIo)
