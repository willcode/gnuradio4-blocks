add_library(gr-timing INTERFACE)
add_library(gnuradio4::gr-timing ALIAS gr-timing)
target_include_directories(gr-timing INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                               $<INSTALL_INTERFACE:include/>)
target_link_libraries(gr-timing INTERFACE gnuradio-core gnuradio4::gnuradio-algorithm gr-common)
install(TARGETS gr-timing EXPORT gnuradio4Targets)

install(
  FILES include/gnuradio-4.0/GpsSource.hpp
        include/gnuradio-4.0/NMEADevice.hpp
        include/gnuradio-4.0/NMEAParser.hpp
        include/gnuradio-4.0/PpsSource.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/timing"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

# GpsSource and PpsSource are Linux device sources: they refuse to compile off Linux and open /dev/ptpN, /dev/ppsN and
# a serial port. Registering them would put those interfaces in a library every platform loads, which is a decision of
# its own; they are named here so the configure-time audit knows the omission is meant rather than forgotten.
gr_add_block_library(
  GrTimingBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/timing/DiscontinuityMonitor.hpp
  UNREGISTERED_HEADERS
  include/gnuradio-4.0/GpsSource.hpp
  include/gnuradio-4.0/PpsSource.hpp
  LINK_LIBRARIES
  gr-timing)

if(ENABLE_EXAMPLES)
  add_subdirectory(src)
endif()

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(bench)
