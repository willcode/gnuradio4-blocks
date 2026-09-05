add_ut_test(qa_Recipes)
# gnuradio-core for the loader and the emitter; the shared block library so the interior registry ids the recipes name
# are registered when the test runs.
target_link_libraries(
  qa_Recipes
  PRIVATE gnuradio-core
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrDigitalBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrSyncBlocksShared)
target_include_directories(qa_Recipes PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools"
                                              "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_compile_definitions(qa_Recipes PRIVATE RECIPES_SOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/..")
