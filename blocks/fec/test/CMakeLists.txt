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
gr_fec_graph_test(graph_fec_conv_encode graphs/conv-encode.yaml)
gr_fec_graph_test(graph_fec_conv_decode graphs/conv-decode.yaml)
gr_fec_graph_test(graph_fec_conv_records graphs/conv-records.yaml)
gr_fec_graph_test(graph_fec_conv_settings graphs/conv-settings.yaml)
gr_fec_graph_test(graph_fec_puncture_patterns graphs/puncture-patterns.yaml)
gr_fec_graph_test(graph_fec_puncture_chain graphs/puncture-chain.yaml)
gr_fec_graph_test(graph_fec_puncture_records graphs/puncture-records.yaml)
gr_fec_graph_test(graph_fec_puncture_settings graphs/puncture-settings.yaml)

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
gr_fec_graph_test(
  graph_fec_accepted_conv_ccsds_k7
  graphs/conv-settings.yaml
  SET
  "ccsds_encoder_block.constraint_length=!!uint32 7")
gr_fec_graph_test(
  graph_fec_accepted_conv_ccsds_invert_agree
  graphs/conv-settings.yaml
  SET
  "ccsds_encoder_block.invert_outputs=!!uint32 2")
gr_fec_graph_test(
  graph_fec_accepted_conv_soft_open
  graphs/conv-settings.yaml
  SET
  "soft_decoder_block.termination=open")

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
gr_add_failing_graph_test(
  graph_fec_refused_conv_code_empty
  graphs/conv-settings.yaml
  "constraint_length and polynomials have no defaults"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.constraint_length=!!uint32 0"
  "encoder_block.polynomials=!!uint32 []")
gr_add_failing_graph_test(
  graph_fec_refused_conv_k2
  graphs/conv-settings.yaml
  "constraint_length must be 3 to 9, [^\n]*. got constraint_length 2,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.constraint_length=!!uint32 2"
  "decoder_block.polynomials=!!uint32 [3, 2]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_k10
  graphs/conv-settings.yaml
  "constraint_length must be 3 to 9, [^\n]*. got constraint_length 10,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.constraint_length=!!uint32 10"
  "decoder_block.polynomials=!!uint32 [617, 1003]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_one_polynomial
  graphs/conv-settings.yaml
  "polynomials names one generator per coded bit and must hold 2 to 4 of them, got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_polynomial_twice
  graphs/conv-settings.yaml
  "every polynomial must be nonzero, no wider than constraint_length bits and named once[^\n]*. got constraint_length 7,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121, 121]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_polynomial_wide
  graphs/conv-settings.yaml
  "every polynomial must be nonzero, no wider than constraint_length bits and named once[^\n]*. got constraint_length 7,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121, 219]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_polynomial_zero
  graphs/conv-settings.yaml
  "every polynomial must be nonzero, no wider than constraint_length bits and named once[^\n]*. got constraint_length 7,"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "encoder_block.polynomials=!!uint32 [121, 0]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_ccsds_k5
  graphs/conv-settings.yaml
  "code 'ccsds' supplies constraint_length 7 and the explicit setting says 5"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.constraint_length=!!uint32 5")
gr_add_failing_graph_test(
  graph_fec_refused_conv_ccsds_polynomials
  graphs/conv-settings.yaml
  "code 'ccsds' supplies its own polynomials and the explicit setting disagrees"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.polynomials=!!uint32 [121, 91]")
gr_add_failing_graph_test(
  graph_fec_refused_conv_ccsds_invert
  graphs/conv-settings.yaml
  "code 'ccsds' supplies invert_outputs 0b10 and the explicit setting says 0b1:"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "ccsds_encoder_block.invert_outputs=!!uint32 1")
gr_add_failing_graph_test(
  graph_fec_refused_conv_code_unknown
  graphs/conv-settings.yaml
  "code must name one of 'ccsds', [^\n]*, got 'voyager'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.code=voyager")
gr_add_failing_graph_test(
  graph_fec_refused_conv_termination
  graphs/conv-settings.yaml
  "termination must be 'terminated' or 'open', got 'running'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "decoder_block.termination=running")
gr_add_failing_graph_test(
  graph_fec_refused_puncture_empty
  graphs/puncture-settings.yaml
  "fec::Puncture#[0-9]+' throws: pattern has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "puncture_block.pattern=\"\"")
gr_add_failing_graph_test(
  graph_fec_refused_depuncture_empty
  graphs/puncture-settings.yaml
  "fec::Depuncture#[0-9]+' throws: pattern has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "depuncture_block.pattern=\"\"")
gr_add_failing_graph_test(
  graph_fec_refused_puncture_spelling
  graphs/puncture-settings.yaml
  "fec::Puncture#[0-9]+' init\\(\\) throws: pattern spells one keep or delete per coded bit as '1' or '0', got '1102'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "puncture_block.pattern=\"1102\"")
gr_add_failing_graph_test(
  graph_fec_refused_depuncture_spelling
  graphs/puncture-settings.yaml
  "fec::Depuncture#[0-9]+' init\\(\\) throws: pattern spells one keep or delete per coded bit as '1' or '0', got '11 0'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "depuncture_block.pattern=\"11 0\"")
gr_add_failing_graph_test(
  graph_fec_refused_puncture_none_kept
  graphs/puncture-settings.yaml
  "fec::Puncture#[0-9]+' init\\(\\) throws: pattern deletes every position and would carry nothing, got '0000'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "puncture_block.pattern=\"0000\"")
gr_add_failing_graph_test(
  graph_fec_refused_depuncture_none_kept
  graphs/puncture-settings.yaml
  "fec::Depuncture#[0-9]+' init\\(\\) throws: pattern deletes every position and would carry nothing, got '0'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrFecBlocksShared>
  SET
  "depuncture_block.pattern=\"0\"")

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
gr_fec_graph_test(
  graph_fec_counters_refused_encode
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ConvEncode 'refused_encode_coder': frames: 1, information bits: 50, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_refused_hard
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecode 'refused_hard_decoder': frames: 1, information bits: 20, records refused: 2")
gr_fec_graph_test(
  graph_fec_counters_refused_soft
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecodeSoft 'refused_soft_decoder': frames: 1, information bits: 20, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_refused_step
  graphs/conv-records.yaml
  PASS_REGULAR_EXPRESSION
  "ViterbiDecode 'refused_step_decoder': records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_refused_puncture
  graphs/puncture-records.yaml
  PASS_REGULAR_EXPRESSION
  "Puncture 'refused_puncture_puncture': records: 1, bits kept: 6, records refused: 1")
gr_fec_graph_test(
  graph_fec_counters_refused_depuncture
  graphs/puncture-records.yaml
  PASS_REGULAR_EXPRESSION
  "Depuncture 'refused_depuncture_depuncture': records: 1, items emitted: 8, records refused: 2")
