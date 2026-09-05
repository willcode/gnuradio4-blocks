add_library(gr-ccsds INTERFACE)
add_library(gnuradio4::gr-ccsds ALIAS gr-ccsds)
target_link_libraries(gr-ccsds INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-ccsds INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                              $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-ccsds EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/ccsds"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrCcsdsBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/ccsds/RecordHelpers.hpp
  include/gnuradio-4.0/ccsds/Frames.hpp
  include/gnuradio-4.0/ccsds/SpacePackets.hpp
  include/gnuradio-4.0/ccsds/FieldRouter.hpp
  include/gnuradio-4.0/ccsds/TimeCodes.hpp
  LINK_LIBRARIES
  gr-ccsds)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(bench)
