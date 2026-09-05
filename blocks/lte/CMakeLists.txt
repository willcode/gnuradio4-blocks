add_library(gr-lte INTERFACE)
add_library(gnuradio4::gr-lte ALIAS gr-lte)
target_link_libraries(gr-lte INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-lte INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                            $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-lte EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/lte"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

gr_add_block_library(
  GrLteBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  include/gnuradio-4.0/lte/CellScanner.hpp
  include/gnuradio-4.0/lte/CellSearch.hpp
  LINK_LIBRARIES
  gr-lte)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(ENABLE_EXAMPLES)
  add_subdirectory(src)
endif()

add_subdirectory(bench)
