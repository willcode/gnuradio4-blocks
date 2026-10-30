add_ut_test(qa_ConvBlocks)
target_link_libraries(qa_ConvBlocks PRIVATE gr-fec)

add_ut_test(qa_PunctureBlocks)
target_link_libraries(qa_PunctureBlocks PRIVATE gr-fec)

add_ut_test(qa_InterleaveBlocks)
target_link_libraries(qa_InterleaveBlocks PRIVATE gr-fec)

# The fec family's tests are graph files run by rungraph. Each case loads the fec plugin beside the testing and basic
# plugins.
function(gr_fec_graph_test TEST_NAME GRAPH)
  gr_add_graph_test(
    ${TEST_NAME}
    ${GRAPH}
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrFecBlocksShared>
    ${ARGN})
endfunction()

# Each graph runs to the end when every sink receives what it expects.
gr_fec_graph_test(graph_fec_binary_round_trip graphs/fec-binary-round-trip.yaml)
gr_fec_graph_test(graph_fec_binary_anchors graphs/fec-binary-anchors.yaml)
gr_fec_graph_test(graph_fec_binary_errors graphs/fec-binary-errors.yaml)
gr_fec_graph_test(graph_fec_binary_records graphs/fec-binary-records.yaml)
gr_fec_graph_test(graph_fec_rs_round_trip graphs/fec-rs-round-trip.yaml)
gr_fec_graph_test(graph_fec_rs_errors graphs/fec-rs-errors.yaml)
gr_fec_graph_test(graph_fec_settings graphs/fec-settings.yaml)

# The settings graphs with one setting changed that the block accepts.
gr_fec_graph_test(
  graph_fec_accepted_rs_pad_last
  graphs/fec-settings.yaml
  SET
  "rs_decoder_block.roots=!!uint32 16"
  "rs_decoder_block.pad=!!uint32 46")
gr_fec_graph_test(
  graph_fec_accepted_rs_gf64_depth_7
  graphs/fec-settings.yaml
  SET
  "rs_decoder_block.interleave=!!uint32 7")
gr_fec_graph_test(
  graph_fec_accepted_ccsds_depth_8
  graphs/fec-settings.yaml
  SET
  "ccsds_encoder_block.interleave=!!uint32 8")
gr_fec_graph_test(
  graph_fec_accepted_ccsds_roots_agree
  graphs/fec-settings.yaml
  SET
  "ccsds_encoder_block.roots=!!uint32 32")
gr_fec_graph_test(
  graph_fec_accepted_ccsds_pad_last
  graphs/fec-settings.yaml
  SET
  "ccsds_decoder_block.pad=!!uint32 238")

# The settings graphs with one setting changed that the block refuses, on the block's reason.
gr_add_failing_graph_test(
  graph_fec_refused_fec_code_empty
  graphs/fec-settings.yaml
  "code has no default and must be set to 'golay24'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.code=\"\"")
gr_add_failing_graph_test(
  graph_fec_refused_fec_code_unknown
  graphs/fec-settings.yaml
  "code must be 'golay24', [^\n]*, got 'golay25'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.code=golay25")
gr_add_failing_graph_test(
  graph_fec_refused_rs_roots
  graphs/fec-settings.yaml
  "roots must be 8, 12 or 16, [^\n]*, got 10"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_encoder_block.roots=!!uint32 10")
gr_add_failing_graph_test(
  graph_fec_refused_rs_pad
  graphs/fec-settings.yaml
  "pad is the shortening in leading zero symbols and must be below 47, [^\n]*, got 47"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_decoder_block.roots=!!uint32 16"
  "rs_decoder_block.pad=!!uint32 47")
gr_add_failing_graph_test(
  graph_fec_refused_rs_code_unknown
  graphs/fec-settings.yaml
  "code must be empty, 'ccsds_255_223' or 'ccsds_255_239', got 'rs_experimental'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_encoder_block.code=rs_experimental")
gr_add_failing_graph_test(
  graph_fec_refused_rs_gf64_basis
  graphs/fec-settings.yaml
  "the GF\\(64\\) family has none. got 'dual'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_encoder_block.basis=dual")
gr_add_failing_graph_test(
  graph_fec_refused_rs_gf64_depth
  graphs/fec-settings.yaml
  "interleave must be 1 to 8, got 9"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "rs_decoder_block.interleave=!!uint32 9")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_basis_empty
  graphs/fec-settings.yaml
  "basis has no default under a CCSDS profile"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.basis=\"\"")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_depth_6
  graphs/fec-settings.yaml
  "interleave must be one of 4\\.3\\.5\\.1's depths 1, 2, 3, 4, 5 or 8, got 6"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.interleave=!!uint32 6")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_depth_7
  graphs/fec-settings.yaml
  "interleave must be one of 4\\.3\\.5\\.1's depths 1, 2, 3, 4, 5 or 8, got 7"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.interleave=!!uint32 7")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_roots
  graphs/fec-settings.yaml
  "code 'ccsds_255_223' carries 32 parity symbols and roots says 16"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.roots=!!uint32 16")
gr_add_failing_graph_test(
  graph_fec_refused_ccsds_pad
  graphs/fec-settings.yaml
  "pad is the shortening in leading zero symbols and must be below 239, [^\n]*, got 239"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_decoder_block.pad=!!uint32 239")

# The line a block prints at stop, for the counts no output record carries.
gr_fec_graph_test(
  graph_fec_counters_misaligned_golay24
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecEncode 'misaligned_golay24_coder': records: 1, codewords: 2, records refused: 2")
gr_fec_graph_test(
  graph_fec_counters_misaligned_bch15_11
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecEncode 'misaligned_bch15_11_coder': records: 1, codewords: 2, records refused: 2")
gr_fec_graph_test(
  graph_fec_counters_truncated
  graphs/fec-binary-records.yaml
  PASS_REGULAR_EXPRESSION
  "FecDecode 'truncated_decoder': records: 1, codewords: 1, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_shortened_8
  graphs/fec-rs-round-trip.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'gf64_8_39_decoder': records: 1, codewords: 2\n")
gr_fec_graph_test(
  graph_fec_counters_shortened_12
  graphs/fec-rs-round-trip.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'gf64_12_39_decoder': records: 1, codewords: 2\n")
gr_fec_graph_test(
  graph_fec_counters_shortened_16
  graphs/fec-rs-round-trip.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'gf64_16_27_decoder': records: 1, codewords: 2\n")
gr_fec_graph_test(
  graph_fec_counters_pad
  graphs/fec-rs-errors.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'pad_decoder': records: 1, codewords: 1, corrected errors: 1, uncorrectable codewords: 1, pad corrupted: 1")
gr_fec_graph_test(
  graph_fec_counters_rs_truncated
  graphs/fec-rs-errors.yaml
  PASS_REGULAR_EXPRESSION
  "RsDecode 'truncated_decoder': records: 1, codewords: 1, records refused: 1")
