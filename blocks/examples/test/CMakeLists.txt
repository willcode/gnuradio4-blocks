add_ut_test(qa_ExampleGraphs)
# gnuradio-core for the YAML importer and the scheduler; the shared block library of every family a shipped graph names,
# so the ids in the files resolve in the global registry when the test runs.
target_link_libraries(
  qa_ExampleGraphs
  PRIVATE gnuradio-core
          gnuradio4::gr-basic
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrChannelBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrMathBlocksShared
          gnuradio4::GrMeasurementBlocksShared)
target_compile_definitions(qa_ExampleGraphs PRIVATE EXAMPLE_GRAPHS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../graphs")

# The acceptance graphs. They are composed of blocks from most of the tree, and they instantiate recipes through the
# generated recipe headers, so the shared library of every family a graph names has to be loaded for the ids inside a
# recipe to resolve in the global registry.
add_ut_test(qa_Tier5Gates)
target_link_libraries(
  qa_Tier5Gates
  PRIVATE gnuradio-core
          gnuradio4::gr-testing
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrAx25BlocksShared
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrChannelBlocksShared
          gnuradio4::GrDigitalBlocksShared
          gnuradio4::GrFecBlocksShared
          gnuradio4::GrFileIoBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrMathBlocksShared
          gnuradio4::GrSyncBlocksShared)
target_include_directories(qa_Tier5Gates PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../recipes/include")
# The off-air captures are machine-local, so the path reaches the test through its environment rather than through a
# graph file. Empty means the machine has none and the recording legs report a skip.
set_property(
  TEST qa_Tier5Gates
  APPEND
  PROPERTY ENVIRONMENT_MODIFICATION "GR4_SATELLITE_RECORDINGS_DIR=set:${GR4_SATELLITE_RECORDINGS_DIR}")
set_property(TEST qa_Tier5Gates PROPERTY SKIP_RETURN_CODE 77)
