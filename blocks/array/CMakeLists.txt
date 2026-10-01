add_library(gr-array INTERFACE)
add_library(gnuradio4::gr-array ALIAS gr-array)
target_link_libraries(gr-array INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-array INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                              $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-array EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/array"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrArrayBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/array/CovarianceEstimate.hpp
  include/gnuradio-4.0/array/LinearCombine.hpp
  LINK_LIBRARIES
  gr-array)

if(TARGET gnuradio4::GrArrayBlocksShared AND ENABLE_TESTING)
  add_subdirectory(test)
endif()
