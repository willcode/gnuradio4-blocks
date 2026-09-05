add_library(gr-vocoder INTERFACE)
add_library(gnuradio4::gr-vocoder ALIAS gr-vocoder)
target_link_libraries(gr-vocoder INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-vocoder INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                                $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-vocoder EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/vocoder"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrVocoderBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/vocoder/ImbeDecode.hpp
  LINK_LIBRARIES
  gr-vocoder)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
