# A graph file is data until something runs it, and nothing in the tree did. The program is generic - it names no block
# family and links none - but it lives here because this module is where the first graph file that needed a shell entry
# point was written; move it up when a second one wants it.
add_executable(gr4-run-graph gr4_run_graph.cpp)
target_link_libraries(gr4-run-graph PRIVATE gnuradio-options gnuradio-core gnuradio-blocklib-core)
# GCC's null-dereference analysis false-positives inside the scheduler's fused-run machinery once it is inlined, the
# same way it does in the inlined library code the unit tests reach; the warning battery reads this property to decide
# whether the analysis is trustworthy for a target, and for one that runs a whole graph it is not. Nothing in this file
# is implicated - every finding is in an installed header this program only includes.
set_target_properties(gr4-run-graph PROPERTIES GR_QA_REDUCED_OPTIMIZATION ON)
