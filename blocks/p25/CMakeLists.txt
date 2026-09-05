add_library(gr-p25 INTERFACE)
add_library(gnuradio4::gr-p25 ALIAS gr-p25)
target_link_libraries(gr-p25 INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-p25 INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                            $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-p25 EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/p25"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrP25Blocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/p25/Deframer.hpp
  include/gnuradio-4.0/p25/PayloadDecode.hpp
  LINK_LIBRARIES
  gr-p25)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
