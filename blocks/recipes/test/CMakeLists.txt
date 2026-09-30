add_ut_test(qa_Recipes)
# gnuradio-core provides the loader and the emitter. The shared block libraries register the interior registry ids that
# the recipes name when the test runs.
target_link_libraries(
  qa_Recipes
  PRIVATE gnuradio-core
          gnuradio4::gr-testing
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrAx25BlocksShared
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrDigitalBlocksShared
          gnuradio4::GrFecBlocksShared
          gnuradio4::GrFileIoBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrMathBlocksShared
          gnuradio4::GrOfdmBlocksShared
          gnuradio4::GrSyncBlocksShared)
target_include_directories(qa_Recipes PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools"
                                              "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_compile_definitions(qa_Recipes PRIVATE RECIPES_SOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/..")

# The network family is optional. The recipes' socket legs are built only where it is present.
if(TARGET gnuradio4::GrNetworkBlocksShared)
  target_link_libraries(qa_Recipes PRIVATE gnuradio4::GrNetworkBlocksShared)
  target_compile_definitions(qa_Recipes PRIVATE GNURADIO4_HAVE_NETWORK_BLOCKS)
endif()

# The burst gate renders and modulates an AIS slot and takes it back through FskDemod into HdlcDeframe.
add_ut_test(qa_HdlcBursts)
target_link_libraries(
  qa_HdlcBursts
  PRIVATE gnuradio-core
          gnuradio4::gr-testing
          gr-ax25
          gr-basic
          gr-channel
          gr-digital
          gnuradio4::GrAnalogBlocksShared
          gnuradio4::GrAx25BlocksShared
          gnuradio4::GrBasicBlocksShared
          gnuradio4::GrChannelBlocksShared
          gnuradio4::GrDigitalBlocksShared
          gnuradio4::GrFilterBlocksShared
          gnuradio4::GrSyncBlocksShared)
target_compile_definitions(qa_HdlcBursts PRIVATE RECIPES_SOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/..")
