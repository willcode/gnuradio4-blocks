add_library(gr-ax25 INTERFACE)
add_library(gnuradio4::gr-ax25 ALIAS gr-ax25)
target_link_libraries(gr-ax25 INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-ax25 INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                             $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-ax25 EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/ax25"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrAx25Blocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/ax25/Ax25.hpp
  include/gnuradio-4.0/ax25/Kiss.hpp
  LINK_LIBRARIES
  gr-ax25)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(bench)
