add_library(gr-sync INTERFACE)
add_library(gnuradio4::gr-sync ALIAS gr-sync)
target_link_libraries(gr-sync INTERFACE gnuradio4::gnuradio-core gnuradio4::gnuradio-algorithm)
target_include_directories(gr-sync INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
                                             $<INSTALL_INTERFACE:include/>)
install(TARGETS gr-sync EXPORT gnuradio4Targets)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/sync"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")

if(TARGET gnuradio4::GrSyncBlocksShared AND ENABLE_TESTING)
endif()
