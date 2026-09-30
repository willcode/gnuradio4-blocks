# EXCLUDE_FROM_ALL keeps these benchmarks out of the default build. Build one by naming its target.
function(gr_add_ofdm_benchmark BM_NAME)
  add_executable(${BM_NAME} EXCLUDE_FROM_ALL ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-ofdm gnuradio4::gr-testing)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

gr_add_ofdm_benchmark(bm_Ofdm)
