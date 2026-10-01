# each graph file runs hand-worked values through an array block into ExpectSinks
foreach(
  _case
  linear-combine-identity
  linear-combine-subtract
  linear-combine-rotate
  linear-combine-weights-tag
  linear-combine-bad-extents
  covariance-identical
  covariance-quadrature
  covariance-exponential
  covariance-conjugate-mismatch
  calibration-gain-delay)
  string(
    REPLACE "-"
            "_"
            _test
            "${_case}")
  gr_add_graph_test(
    graph_array_${_test}
    graphs/${_case}.yaml
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrArrayBlocksShared>)
endforeach()
set_tests_properties(graph_array_linear_combine_bad_extents
                     PROPERTIES PASS_REGULAR_EXPRESSION "weights of extents 3 disagree with 1 outputs by 2 inputs")
set_tests_properties(graph_array_covariance_conjugate_mismatch
                     PROPERTIES PASS_REGULAR_EXPRESSION "value 1 of DataSet 0 is \\(0-1i\\), expected \\(0\\+1i\\)")

# ArraySource steered and nulled by LinearCombine, its magnitude taken by the math family
gr_add_graph_test(
  graph_array_source_steering
  graphs/array-source-steering.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrArrayBlocksShared>
  $<TARGET_FILE_DIR:GrMathBlocksShared>)
