# each graph file runs hand-worked values through an array block into ExpectSinks
foreach(
  _case
  linear-combine-identity
  linear-combine-subtract
  linear-combine-rotate
  linear-combine-weights-tag
  linear-combine-bad-extents)
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
