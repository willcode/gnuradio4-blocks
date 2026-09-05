add_ut_test(qa_CcsdsFrames)
target_link_libraries(qa_CcsdsFrames PRIVATE gr-ccsds gr-digital gr-testing)

add_ut_test(qa_CcsdsRouter)
target_link_libraries(qa_CcsdsRouter PRIVATE gr-ccsds gr-digital gr-testing)

add_ut_test(qa_CcsdsPackets)
target_link_libraries(
  qa_CcsdsPackets
  PRIVATE gr-ccsds
          gr-digital
          gr-basic
          gr-testing)

add_ut_test(qa_CcsdsChain)
target_link_libraries(
  qa_CcsdsChain
  PRIVATE gr-ccsds
          gr-digital
          gr-basic
          gr-testing)

add_ut_test(qa_CcsdsTimeCodeBlocks)
target_link_libraries(qa_CcsdsTimeCodeBlocks PRIVATE gr-ccsds gr-digital gr-testing)
