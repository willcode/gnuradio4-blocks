add_ut_test(qa_AccessCodeCorrelator)
target_link_libraries(qa_AccessCodeCorrelator PRIVATE gr-digital gr-basic gr-testing)

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

add_ut_test(qa_CsmaGate)
target_link_libraries(qa_CsmaGate PRIVATE gr-digital gr-testing)

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

add_ut_test(qa_OffsetWordFraming)
target_link_libraries(qa_OffsetWordFraming PRIVATE gr-digital gr-testing)

add_ut_test(qa_PacketFramer)
target_link_libraries(qa_PacketFramer PRIVATE gr-digital gr-testing)

add_ut_test(qa_PamSymbols)
target_link_libraries(qa_PamSymbols PRIVATE gr-digital gr-testing)

add_ut_test(qa_PreambleCorrelator)
target_link_libraries(qa_PreambleCorrelator PRIVATE gr-digital gr-testing)

add_ut_test(qa_Scrambler)
target_link_libraries(qa_Scrambler PRIVATE gr-digital gr-testing)

add_ut_test(qa_SignalQuality)
target_link_libraries(qa_SignalQuality PRIVATE gr-digital gr-channel gr-testing)

add_ut_test(qa_ModemDemo)
target_link_libraries(
  qa_ModemDemo
  PRIVATE gr-digital
          gr-channel
          gr-filter
          gr-sync
          gr-testing)

# CpmModulate, LinearEqualizer, ConstellationSoftDecoder and PreambleCorrelator forward the tags left on their input at
# the end-of-stream index
gr_add_graph_test(
  graph_digital_cpm_modulate_end_tags
  graphs/cpm-modulate-end-tags.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  $<TARGET_FILE_DIR:GrBasicBlocksShared>)
gr_add_graph_test(
  graph_digital_linear_equalizer_end_tags
  graphs/linear-equalizer-end-tags.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>)
gr_add_graph_test(
  graph_digital_constellation_soft_decoder_end_tags
  graphs/constellation-soft-decoder-end-tags.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  $<TARGET_FILE_DIR:GrBasicBlocksShared>)
gr_add_graph_test(
  graph_digital_preamble_correlator_end_tags
  graphs/preamble-correlator-end-tags.yaml
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>)
