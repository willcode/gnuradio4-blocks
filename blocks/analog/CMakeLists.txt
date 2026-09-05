add_library(gr-analog INTERFACE)
add_library(gnuradio4::gr-analog ALIAS gr-analog)
target_link_libraries(gr-analog INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-analog INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                               $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-analog EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/analog"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

if(TARGET gnuradio4::GrAnalogBlocksShared AND ENABLE_TESTING)
endif()
