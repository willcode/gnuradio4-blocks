# Example graphs are YAML files for the importer and hold no code. They are installed beside the recipes. An installed
# application, control plane or graph editor pointed at the data directory finds them there. Nothing is compiled here.
# qa_ExampleGraphs loads and runs each graph in graphs/. An example that fails to load is the first thing a new user
# meets.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/graphs/" DESTINATION "${CMAKE_INSTALL_DATADIR}/gnuradio-4.0/examples")

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
