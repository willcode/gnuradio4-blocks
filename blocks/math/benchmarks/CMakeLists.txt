function(add_math_benchmark BM_NAME)
  add_benchmark(${BM_NAME})
  target_link_libraries(
    ${BM_NAME}
    PRIVATE gnuradio-options
            gnuradio-core
            gr-math
            ut
            ut-benchmark)
  target_compile_options(${BM_NAME} PRIVATE -O3)
endfunction()

add_math_benchmark(bm_Binary)
add_math_benchmark(bm_BitPacking)
target_link_libraries(bm_BitPacking PRIVATE gnuradio4::gr-testing)
add_math_benchmark(bm_Rotator)
add_math_benchmark(bm_Unary)
