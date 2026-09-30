# EXCLUDE_FROM_ALL keeps this benchmark out of the default build. Build it by naming its target.
function(gr_add_fileio_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-fileio)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

gr_add_fileio_benchmark(bm_SigMfIo)
