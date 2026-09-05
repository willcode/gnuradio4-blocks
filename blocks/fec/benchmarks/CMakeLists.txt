function(add_fec_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-fec
            gnuradio4::gr-testing
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

if(GR4_FEC_HAVE_AFF3CT)
  add_fec_benchmark(bm_Aff3ctBlocks)
endif()
add_fec_benchmark(bm_CcsdsRs)
add_fec_benchmark(bm_InterleaveBlocks)
