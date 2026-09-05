if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/recipes")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/gnuradio-4.0/recipes"
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gnuradio-4.0")
endif()
add_executable(gr4-recipe-gen tools/gr4-recipe-gen.cpp)
target_link_libraries(gr4-recipe-gen PRIVATE gnuradio-core)

if(ENABLE_TESTING)
  add_subdirectory(test)
endif()
