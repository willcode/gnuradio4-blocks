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

# The digital family's stream graph cases are run by rungraph. Each case loads the digital plugin beside the testing and
# basic plugins. A graph runs to the end when every sink receives what it expects.
function(gr_digital_stream_graph_test TEST_NAME GRAPH)
  gr_add_graph_test(
    ${TEST_NAME}
    ${GRAPH}
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrDigitalBlocksShared>
    ${ARGN})
endfunction()

# The graph cases of the differential coding blocks.
gr_digital_stream_graph_test(graph_digital_differential_round_trip graphs/differential-round-trip.yaml)
gr_digital_stream_graph_test(graph_digital_differential_wide graphs/differential-wide.yaml)
gr_digital_stream_graph_test(graph_digital_differential_binary graphs/differential-binary.yaml)
gr_digital_stream_graph_test(graph_digital_differential_phasor graphs/differential-phasor.yaml)
gr_digital_stream_graph_test(graph_digital_differential_tags graphs/differential-tags.yaml)
gr_digital_stream_graph_test(graph_digital_differential_settings graphs/differential-settings.yaml)
gr_digital_stream_graph_test(
  graph_digital_accepted_differential_encoder_byte
  graphs/differential-settings.yaml
  SET
  "encoder_block.modulus=!!uint32 256")
gr_digital_stream_graph_test(
  graph_digital_accepted_differential_wide_encoder_whole
  graphs/differential-settings.yaml
  SET
  "wide_encoder_block.modulus=!!uint32 65536")
gr_add_failing_graph_test(
  graph_digital_refused_differential_encoder_zero
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint8>[^\n]* throws: modulus is an alphabet size and must be at least 2, got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "encoder_block.modulus=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_differential_encoder_one
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint8>[^\n]* throws: modulus is an alphabet size and must be at least 2, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "encoder_block.modulus=!!uint32 1")
gr_add_failing_graph_test(
  graph_digital_refused_differential_decoder_zero
  graphs/differential-settings.yaml
  "DifferentialDecoder<uint8>[^\n]* throws: modulus is an alphabet size and must be at least 2, got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "decoder_block.modulus=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_differential_encoder_wide
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint8>[^\n]* throws: modulus 257 exceeds the 256 values the sample type can hold"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "encoder_block.modulus=!!uint32 257")
gr_add_failing_graph_test(
  graph_digital_refused_differential_decoder_wide
  graphs/differential-settings.yaml
  "DifferentialDecoder<uint8>[^\n]* throws: modulus 512 exceeds the 256 values the sample type can hold"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "decoder_block.modulus=!!uint32 512")
gr_add_failing_graph_test(
  graph_digital_refused_differential_wide_encoder_wide
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint16>[^\n]* throws: modulus 65537 exceeds the 65536 values the sample type can hold"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "wide_encoder_block.modulus=!!uint32 65537")
gr_add_failing_graph_test(
  graph_digital_refused_differential_encoder_state
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint8>[^\n]* throws: initial_state 4 must be below modulus 4"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "encoder_block.initial_state=!!uint32 4")
gr_add_failing_graph_test(
  graph_digital_refused_differential_decoder_state
  graphs/differential-settings.yaml
  "DifferentialDecoder<uint8>[^\n]* throws: initial_state 9 must be below modulus 4"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "decoder_block.initial_state=!!uint32 9")
gr_add_failing_graph_test(
  graph_digital_refused_differential_encoder_nrzi
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint8>[^\n]* throws: coding 'nrzi' is a binary code and requires modulus 2, got 4"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "encoder_block.coding=nrzi")
gr_add_failing_graph_test(
  graph_digital_refused_differential_decoder_nrzi
  graphs/differential-settings.yaml
  "DifferentialDecoder<uint8>[^\n]* throws: coding 'nrzi' is a binary code and requires modulus 2, got 4"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "decoder_block.coding=nrzi")
gr_add_failing_graph_test(
  graph_digital_refused_differential_encoder_coding
  graphs/differential-settings.yaml
  "DifferentialEncoder<uint8>[^\n]* throws: coding must be 'differential' or 'nrzi', got 'manchester'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "encoder_block.coding=manchester")
