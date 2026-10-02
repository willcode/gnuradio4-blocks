add_ut_test(qa_BitPacking)
target_link_libraries(qa_BitPacking PRIVATE gr-math gr-testing)
add_ut_test(qa_Math)
target_link_libraries(qa_Math PRIVATE gr-math gr-testing)
add_ut_test(qa_MathMultiplyDivide)
target_link_libraries(qa_MathMultiplyDivide PRIVATE gr-math gr-testing)
add_ut_test(qa_MathConst)
target_link_libraries(qa_MathConst PRIVATE gr-math gr-testing)
add_ut_test(qa_ExpressionBlocks)
target_link_libraries(qa_ExpressionBlocks PRIVATE gr-math gr-testing)
add_ut_test(qa_ExpressionBulk)
target_link_libraries(qa_ExpressionBulk PRIVATE gr-math gr-testing)
add_ut_test(qa_Rotator)
target_link_libraries(qa_Rotator PRIVATE gr-math gr-testing)

# one graph file per Unary, Binary, Bits and Reduce alias: each chain one VectorSource per input of the alias into the
# alias into an ExpectSink; the packing round trip, UnpackBits into PackBits; and a Reduce tag placement
foreach(
  _graph
  unary-negate
  unary-abs
  unary-conjugate
  unary-square
  unary-sqrt
  unary-reciprocal
  unary-exp
  unary-log
  unary-log10
  unary-log2
  unary-sin
  unary-cos
  unary-tan
  unary-atan
  unary-sinh
  unary-cosh
  unary-tanh
  unary-power
  unary-floor
  unary-ceil
  unary-round
  unary-sign
  unary-deg-to-rad
  unary-rad-to-deg
  unary-magnitude
  unary-magnitude-squared
  unary-phase
  unary-real
  unary-imag
  binary-multiply-conjugate
  binary-max
  binary-min
  binary-pow
  binary-atan2
  binary-hypot
  binary-modulo
  bits-and
  bits-or
  bits-xor
  bits-not
  bits-and-const
  bits-or-const
  bits-xor-const
  bits-shift-left
  bits-shift-right
  bits-pack-roundtrip
  reduce-sum
  reduce-mean
  reduce-max
  reduce-min
  reduce-argmax
  reduce-argmin
  reduce-mean-square
  reduce-rms
  reduce-tags)
  string(
    REPLACE "-"
            "_"
            _test
            "${_graph}")
  gr_add_graph_test(
    graph_math_${_test}
    graphs/${_graph}.yaml
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrMathBlocksShared>)
endforeach()

# the packing blocks under their compatibility keys, which the digital plugin registers
gr_add_graph_test(
  graph_math_bits_pack_roundtrip_digital
  graphs/bits-pack-roundtrip-digital.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrMathBlocksShared>
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>)
