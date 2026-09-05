if(GR4_ENABLE_AFF3CT)
  foreach(dir "${GR4_AFF3CT_LDPC_CONF_DIR}" "${GR4_AFF3CT_POLAR_CONF_DIR}")
    if(NOT IS_DIRECTORY "${dir}")
    endif()
  endforeach()
endif()

add_library(gr-fec INTERFACE)
add_library(gnuradio4::gr-fec ALIAS gr-fec)
target_link_libraries(gr-fec INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-fec INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                            $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-fec EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/fec"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")
set(GrFecBlocks_HDRS
    include/gnuradio-4.0/fec/ConvBlocks.hpp
    include/gnuradio-4.0/fec/FecBlocks.hpp)
if(GR4_FEC_HAVE_AFF3CT)
else()
endif()

gr_add_block_library(
  GrFecBlocks
  MAKE_SHARED_LIBRARY
  ${GR4_BLOCK_STATIC_LIBRARY}
  HEADERS
  LINK_LIBRARIES
  gr-fec)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
