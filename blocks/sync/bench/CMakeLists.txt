function(gr_add_sync_benchmark BM_NAME)
  add_executable(${BM_NAME} ${BM_NAME}.cpp)
  target_link_libraries(${BM_NAME} PRIVATE gnuradio-options gr-sync)
  # the benchmarks drive processBulk through the same span shim the QA uses, so the two measure the same call
  target_include_directories(${BM_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../test)
endfunction()

gr_add_sync_benchmark(bm_Pll)
gr_add_sync_benchmark(bm_CostasLoop)
