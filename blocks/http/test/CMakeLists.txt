if(NOT APPLE)
  add_ut_test(qa_HttpBlock)
  target_link_libraries(qa_HttpBlock PRIVATE gr-http gr-testing httplib::httplib)
  # This test and qa_FileIo both listen on 127.0.0.1:8080, so they must not run at the same time.
  set_tests_properties(qa_HttpBlock PROPERTIES RESOURCE_LOCK http_port_8080)

  if(EMSCRIPTEN)
    target_link_options(
      qa_HttpBlock
      PRIVATE
      --pre-js=${CMAKE_CURRENT_SOURCE_DIR}/pre.js
      --emrun)
  endif()
endif()
