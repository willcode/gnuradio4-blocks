# The source-tree cases read the graphs from a header generated here, which embeds the text of every graph file. A graph
# that changes reruns the configure step and regenerates the header.
file(
  GLOB
  _graphFiles
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/../graphs/*.yaml")
list(SORT _graphFiles)
set(GR4_EXAMPLE_GRAPH_ENTRIES "")
foreach(_graphFile IN LISTS _graphFiles)
  set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS "${_graphFile}")
  get_filename_component(_graphName "${_graphFile}" NAME)
  file(READ "${_graphFile}" _graphText)
  string(FIND "${_graphText}" ")gr4yaml\"" _delimiterAt)
  if(NOT
     _delimiterAt
     EQUAL
     -1)
    message(FATAL_ERROR "${_graphFile} holds the raw-string delimiter )gr4yaml\"")
  endif()
  string(APPEND GR4_EXAMPLE_GRAPH_ENTRIES "    ExampleGraph{\"${_graphName}\", R\"gr4yaml(${_graphText})gr4yaml\"},\n")
endforeach()
configure_file(ExampleGraphs.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/ExampleGraphs.hpp" @ONLY)

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

# The install-layout case reads the files where the install rule puts them: a setup test installs this directory's
# component into a prefix inside the build tree, and ctest runs it before the qa. The case is left out where the host
# file system is not readable (Emscripten) and where an absolute data directory puts the files outside that prefix.
if(EMSCRIPTEN)
  message(STATUS "example graphs: the install-layout case is left out under Emscripten")
elseif(IS_ABSOLUTE "${CMAKE_INSTALL_DATADIR}")
  message(STATUS "example graphs: the install-layout case is left out, CMAKE_INSTALL_DATADIR is absolute")
else()
  set(GR4_EXAMPLE_GRAPHS_TEST_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/install-layout")
  add_test(NAME qa_ExampleGraphsInstall COMMAND ${CMAKE_COMMAND} --install ${CMAKE_BINARY_DIR} --prefix
                                                ${GR4_EXAMPLE_GRAPHS_TEST_PREFIX} --component gr4-example-graphs)
  set_tests_properties(qa_ExampleGraphsInstall PROPERTIES FIXTURES_SETUP gr4_example_graphs_installed)
  set_tests_properties(qa_ExampleGraphsInstall PROPERTIES TIMEOUT 60)
  set_tests_properties(qa_ExampleGraphs PROPERTIES FIXTURES_REQUIRED gr4_example_graphs_installed)
  target_compile_definitions(
    qa_ExampleGraphs
    PRIVATE INSTALLED_EXAMPLE_GRAPHS_PATH="${GR4_EXAMPLE_GRAPHS_TEST_PREFIX}/${GR4_EXAMPLE_GRAPHS_INSTALL_DIR}")
endif()
set_tests_properties(qa_ExampleGraphs PROPERTIES TIMEOUT 60)

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
