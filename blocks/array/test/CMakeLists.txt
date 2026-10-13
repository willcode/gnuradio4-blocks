# each graph file runs hand-worked values through an array block into ExpectSinks
foreach(
  _case
  linear-combine-identity
  linear-combine-subtract
  linear-combine-rotate
  linear-combine-mixing
  linear-combine-weights-tag
  covariance-identical
  covariance-quadrature
  covariance-exponential
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
# these two graphs end with an error; each case passes when the output holds the error's text
gr_add_graph_test(
  graph_array_linear_combine_bad_extents
  graphs/linear-combine-bad-extents.yaml
  PASS_REGULAR_EXPRESSION
  "weights of extents 3 disagree with 1 outputs by 2 inputs"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrArrayBlocksShared>)
gr_add_graph_test(
  graph_array_covariance_conjugate_mismatch
  graphs/covariance-conjugate-mismatch.yaml
  PASS_REGULAR_EXPRESSION
  "value 1 of DataSet 0 is \\(0-1i\\), expected \\(0\\+1i\\)"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrArrayBlocksShared>)

# ArraySource steered and nulled by LinearCombine, its magnitude taken by the math family
gr_add_graph_test(
  graph_array_source_steering
  graphs/array-source-steering.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrArrayBlocksShared>
  $<TARGET_FILE_DIR:GrMathBlocksShared>)
