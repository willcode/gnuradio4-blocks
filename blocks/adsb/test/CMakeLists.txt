add_ut_test(qa_ModeSDecode)
target_link_libraries(qa_ModeSDecode PRIVATE gr-adsb gr-basic gr-testing)

add_ut_test(qa_AdsbPrinter)
target_link_libraries(qa_AdsbPrinter PRIVATE gr-adsb gr-testing)

# the recorded-air leg, in its own executable because exit 77 is a whole binary's status: folded into qa_ModeSDecode an
# absent capture would skip the synthetic gate with it
add_ut_test(qa_AdsbRecording)
target_link_libraries(
  qa_AdsbRecording
  PRIVATE gr-adsb
          gr-basic
          gr-digital
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
