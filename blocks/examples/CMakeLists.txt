# Example graphs are data, not code: YAML the tree's own importer reads, installed beside the recipes so an installed
# studio, control plane or graph editor pointed at the data directory finds them. Nothing is compiled here;
# qa_ExampleGraphs loads and runs every graph, because an example that no longer loads is the first thing a new user
# meets.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/graphs/" DESTINATION "${CMAKE_INSTALL_DATADIR}/gnuradio-4.0/examples")

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
