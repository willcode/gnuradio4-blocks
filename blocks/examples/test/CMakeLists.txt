add_ut_test(qa_ExampleGraphs)
# gnuradio-core provides the YAML importer and the scheduler. The shared block library of each family a shipped graph
# names resolves the ids in the files in the global registry when the test runs.
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

# The acceptance graphs use blocks from most families and instantiate recipes through the generated recipe headers. The
# shared library of each family a graph names must be loaded. The ids inside a recipe then resolve in the global
# registry.
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
# The off-air captures are local to a machine. Their path reaches the test through its environment and not through a
# graph file. An empty path means the machine has none, and the recording legs report a skip.
set_property(
  TEST qa_Tier5Gates
  APPEND
  PROPERTY ENVIRONMENT_MODIFICATION "GR4_SATELLITE_RECORDINGS_DIR=set:${GR4_SATELLITE_RECORDINGS_DIR}")
set_property(TEST qa_Tier5Gates PROPERTY SKIP_RETURN_CODE 77)
