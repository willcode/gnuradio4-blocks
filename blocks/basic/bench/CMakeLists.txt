function(gr_add_basic_benchmark BM_NAME)
  add_executable(${BM_NAME} ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-basic)
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

gr_add_basic_benchmark(bm_IqCorrection)
gr_add_basic_benchmark(bm_SampleDelay)
