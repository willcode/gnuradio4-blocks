add_ut_test(qa_Recipes)
# gnuradio-core for the loader and the emitter; the shared block library so the interior registry ids the recipes name
# are registered when the test runs.
target_link_libraries(
  qa_Recipes
  PRIVATE gnuradio-core
          gnuradio4::gr-testing
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrDigitalBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrMathBlocksShared
          gnuradio4::GrSyncBlocksShared)
target_include_directories(qa_Recipes PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools"
                                              "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_compile_definitions(qa_Recipes PRIVATE RECIPES_SOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/..")

# the network family is optional, so the recipes' socket legs are built only where it is
if(TARGET gnuradio4::GrNetworkBlocksShared)
  target_link_libraries(qa_Recipes PRIVATE gnuradio4::GrNetworkBlocksShared)
  target_compile_definitions(qa_Recipes PRIVATE GNURADIO4_HAVE_NETWORK_BLOCKS)
endif()
