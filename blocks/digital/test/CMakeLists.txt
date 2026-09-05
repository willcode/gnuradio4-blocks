add_ut_test(qa_AccessCodeCorrelator)
target_link_libraries(qa_AccessCodeCorrelator PRIVATE gr-digital gr-basic gr-testing)

add_ut_test(qa_BitPacking)
target_link_libraries(qa_BitPacking PRIVATE gr-digital gr-testing)

add_ut_test(qa_ConstellationBlocks)
target_link_libraries(qa_ConstellationBlocks PRIVATE gr-digital gr-testing)

add_ut_test(qa_CpmModulate)
target_link_libraries(
  qa_CpmModulate
  PRIVATE gr-digital
          gr-analog
          gr-filter
          gr-sync
          gr-channel
          gr-testing)

add_ut_test(qa_CrcBlocks)
target_link_libraries(qa_CrcBlocks PRIVATE gr-digital gr-testing)

add_ut_test(qa_DelimiterExtractor)
target_link_libraries(qa_DelimiterExtractor PRIVATE gr-digital gr-testing)

add_ut_test(qa_DelimiterFramer)
target_link_libraries(qa_DelimiterFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_DifferentialCoding)
target_link_libraries(qa_DifferentialCoding PRIVATE gr-digital gr-testing)

add_ut_test(qa_LengthHeaderFramer)
target_link_libraries(qa_LengthHeaderFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_LinearEqualizer)
target_link_libraries(qa_LinearEqualizer PRIVATE gr-digital gr-channel gr-testing)

add_ut_test(qa_Manchester)
target_link_libraries(qa_Manchester PRIVATE gr-digital gr-testing)

add_ut_test(qa_ManchesterCombine)
target_link_libraries(qa_ManchesterCombine PRIVATE gr-digital gr-testing)

add_ut_test(qa_PacketFramer)
target_link_libraries(qa_PacketFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_PpmFramer)
target_link_libraries(qa_PpmFramer PRIVATE gr-digital gr-testing)

# the recorded-air leg, in its own executable because exit 77 is a whole binary's status: folded into qa_PpmFramer an
# absent capture would skip the synthetic gate with it
add_ut_test(qa_PpmRecording)
target_link_libraries(qa_PpmRecording PRIVATE gr-digital gr-testing)
set_tests_properties(
  qa_PpmRecording
  PROPERTIES SKIP_RETURN_CODE
             77
             ENVIRONMENT
             "GR4_RECORDINGS_DIR=${GR4_RECORDINGS_DIR}")

add_ut_test(qa_PreambleCorrelator)
target_link_libraries(qa_PreambleCorrelator PRIVATE gr-digital gr-testing)

add_ut_test(qa_SignalQuality)
target_link_libraries(qa_SignalQuality PRIVATE gr-digital gr-channel gr-testing)

add_ut_test(qa_Scrambler)
target_link_libraries(qa_Scrambler PRIVATE gr-digital gr-testing)

add_ut_test(qa_OffsetWordFraming)
target_link_libraries(qa_OffsetWordFraming PRIVATE gr-digital gr-testing)

add_ut_test(qa_PamSymbols)
target_link_libraries(qa_PamSymbols PRIVATE gr-digital gr-testing)

add_ut_test(qa_TeachingModem)
target_link_libraries(
  qa_TeachingModem
  PRIVATE gr-digital
          gr-channel
          gr-filter
          gr-sync
          gr-testing)
