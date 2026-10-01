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

# one graph file per Unary alias, each chain a VectorSource into the alias into an ExpectSink
foreach(
  _alias
  negate
  abs
  conjugate
  square
  sqrt
  reciprocal
  exp
  log
  log10
  log2
  sin
  cos
  tan
  atan
  sinh
  cosh
  tanh
  power
  floor
  ceil
  round
  sign
  deg-to-rad
  rad-to-deg
  magnitude
  magnitude-squared
  phase
  real
  imag)
  string(
    REPLACE "-"
            "_"
            _test
            "${_alias}")
  gr_add_graph_test(
    graph_math_unary_${_test}
    graphs/unary-${_alias}.yaml
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrMathBlocksShared>)
endforeach()
