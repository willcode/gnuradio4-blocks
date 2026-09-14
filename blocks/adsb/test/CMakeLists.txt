add_ut_test(qa_PpmFramer)
target_link_libraries(qa_PpmFramer PRIVATE gr-adsb gr-testing)

add_ut_test(qa_ModeSDecode)
target_link_libraries(qa_ModeSDecode PRIVATE gr-adsb gr-basic gr-testing)

add_ut_test(qa_AdsbPrinter)
target_link_libraries(qa_AdsbPrinter PRIVATE gr-adsb gr-testing)

# the recorded-air legs, each in its own executable because exit 77 is a whole binary's status: folded into a synthetic
# test an absent capture would skip that test with it
add_ut_test(qa_PpmRecording)
target_link_libraries(qa_PpmRecording PRIVATE gr-adsb gr-testing)
set_tests_properties(
  qa_PpmRecording
  PROPERTIES SKIP_RETURN_CODE
             77
             ENVIRONMENT
             "GR4_RECORDINGS_DIR=${GR4_RECORDINGS_DIR}")

add_ut_test(qa_AdsbRecording)
target_link_libraries(
  qa_AdsbRecording
  PRIVATE gr-adsb
          gr-basic
          gr-fileio
          gr-testing)
set_tests_properties(
  qa_AdsbRecording
  PROPERTIES SKIP_RETURN_CODE
             77
             ENVIRONMENT
             "GR4_RECORDINGS_DIR=${GR4_RECORDINGS_DIR}"
             TIMEOUT
             300)
