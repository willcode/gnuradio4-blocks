add_ut_test(qa_AccessCodeCorrelator)
target_link_libraries(qa_AccessCodeCorrelator PRIVATE gr-digital gr-testing)

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

# The graph cases of the scramblers.
gr_digital_stream_graph_test(graph_digital_scrambler_ccsds graphs/scrambler-ccsds.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_additive_resets graphs/scrambler-additive-resets.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_additive_stream graphs/scrambler-additive-stream.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_multiplicative graphs/scrambler-multiplicative.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_self_sync graphs/scrambler-self-sync.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_descrambler_resets graphs/scrambler-descrambler-resets.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_family graphs/scrambler-family.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_profiles graphs/scrambler-profiles.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_sequence graphs/scrambler-sequence.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_forced graphs/scrambler-forced.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_records graphs/scrambler-records.yaml)
gr_digital_stream_graph_test(graph_digital_scrambler_settings graphs/scrambler-settings.yaml)
gr_digital_stream_graph_test(
  graph_digital_accepted_scrambler_bits_one_lsb_first
  graphs/scrambler-settings.yaml
  SET
  "additive_block.bits_per_item=!!uint32 1"
  "additive_block.bit_order=LsbFirst")
gr_digital_stream_graph_test(
  graph_digital_accepted_scrambler_profile_si4463
  graphs/scrambler-settings.yaml
  SET
  "additive_block.profile=si4463"
  "additive_block.taps=\"\""
  "additive_block.seed=\"\"")
gr_digital_stream_graph_test(
  graph_digital_accepted_scrambler_sequence_covered
  graphs/scrambler-settings.yaml
  SET
  "sequence_block.reset_period=!!uint32 4")
gr_digital_stream_graph_test(
  graph_digital_accepted_scrambler_sequence_repeated
  graphs/scrambler-settings.yaml
  SET
  "sequence_block.reset_period=!!uint32 8"
  "sequence_block.sequence_repeat=true")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_no_generator
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: taps names the feedback delays as a decimal list such as \\\"1,3,5,8\\\" and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.taps=\"\""
  "additive_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_no_seed
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: seed is the first 8 bits of the sequence and has no default"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_taps_zero
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting taps: gr::digital::tapsFromDelayList: a delay of zero, got '1,0,5'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.taps=\"1,0,5\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_taps_above
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting taps: gr::digital::tapsFromDelayList: a delay above 64, got '1,65'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.taps=\"1,65\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_taps_repeated
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting taps: gr::digital::tapsFromDelayList: a repeated delay, got '3,3'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.taps=\"3,3\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_taps_stray
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting taps: gr::digital::tapsFromDelayList: a character that is neither a digit, a comma nor a space, got '1,3:5'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.taps=\"1,3:5\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_seed_short
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting seed: gr::digital::seedFromBitString: the seed must be exactly 8 bits, got '1111'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.seed=\"1111\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_seed_digit
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting seed: gr::digital::seedFromBitString: the seed must be '0' and '1' characters, got '11112111'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.seed=\"11112111\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_seed_zeros
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: seed \\\"00000000\\\" is all zeros, which is a dead generator"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.seed=\"00000000\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_bits_zero
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: bits_per_item is the significant bits of an item and must be in .1, 8., got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.bits_per_item=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_bits_nine
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: bits_per_item is the significant bits of an item and must be in .1, 8., got 9"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.bits_per_item=!!uint32 9")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_profile_taps
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: profile \\\"ccsds131\\\" names the generator, so taps \\\"4,7\\\" cannot be staged beside it"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.profile=ccsds131"
  "additive_block.taps=\"4,7\""
  "additive_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_profile_seed
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: profile \\\"ccsds131\\\" names the generator, so seed \\\"00001111\\\" cannot be staged beside it"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.profile=ccsds131"
  "additive_block.taps=\"\""
  "additive_block.seed=\"00001111\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_profile_sequence
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: profile \\\"ccsds131\\\" names the generator, so sequence \\\"FF E1\\\" cannot be staged beside it"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.profile=ccsds131"
  "additive_block.taps=\"\""
  "additive_block.seed=\"\""
  "additive_block.sequence=\"FF E1\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_taps_sequence
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: taps \\\"1,3,5,8\\\" walks a recursion and sequence \\\"FF E1\\\" supplies the mask outright"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.sequence=\"FF E1\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_profile_unknown
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting profile: gr::digital::profileByName: unknown profile 'ccsds132', the names are ccsds131, ccsds131_17, pn9, si4463, g3ruh"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.profile=ccsds132"
  "additive_block.taps=\"\""
  "additive_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_additive_g3ruh
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting profile: gr::digital::applyProfile: profile 'g3ruh' is multiplicative .self-synchronizing. and cannot be used additive"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "additive_block.profile=g3ruh"
  "additive_block.taps=\"\""
  "additive_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_multiplicative_ccsds131
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting profile: gr::digital::applyProfile: profile 'ccsds131' is additive and cannot be used multiplicative"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.profile=ccsds131"
  "scrambler_block.taps=\"\""
  "scrambler_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_multiplicative_pn9
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting profile: gr::digital::applyProfile: profile 'pn9' is additive and cannot be used multiplicative"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.profile=pn9"
  "scrambler_block.taps=\"\""
  "scrambler_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_descrambler_ccsds131
  graphs/scrambler-settings.yaml
  "MultiplicativeDescrambler[^\n]* throws: setting profile: gr::digital::applyProfile: profile 'ccsds131' is additive and cannot be used multiplicative"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "descrambler_block.profile=ccsds131"
  "descrambler_block.taps=\"\""
  "descrambler_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_descrambler_pn9
  graphs/scrambler-settings.yaml
  "MultiplicativeDescrambler[^\n]* throws: setting profile: gr::digital::applyProfile: profile 'pn9' is additive and cannot be used multiplicative"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "descrambler_block.profile=pn9"
  "descrambler_block.taps=\"\""
  "descrambler_block.seed=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_multiplicative_seed
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting seed: gr::digital::seedFromBitString: the seed must be exactly 23 bits, got '101'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.taps=\"18,23\""
  "scrambler_block.seed=\"101\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_sequence_overrun
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: sequence holds 32 bits and sequence_repeat is false, so a reset_period of 8 items at 8 bits each leaves the last 32 bits of every epoch unscrambled"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "sequence_block.reset_period=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_sequence_odd
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting sequence: gr::digital::sequenceFromHex: an odd number of hexadecimal digits, got 'FF E'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "sequence_block.sequence=\"FF E\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_sequence_digit
  graphs/scrambler-settings.yaml
  "AdditiveScrambler<uint8>[^\n]* throws: setting sequence: gr::digital::sequenceFromHex: a character that is neither a hexadecimal digit nor a space, got 'FF G1'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "sequence_block.sequence=\"FF G1\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_monitor_disabled
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting the forced-transition group: gr::digital::configureMonitor: monitorDelays describes a disabled mechanism when forceAfter is zero, got '1,2'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.monitor_delays=\"1,2\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_monitor_missing
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting the forced-transition group: gr::digital::tapsFromDelayList: the list must name at least one decimal delay, got ''"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.force_transition_after=!!uint32 8")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_force_one
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting the forced-transition group: gr::digital::configureMonitor: forceAfter must be zero or in .2, 4096., got 1"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.force_transition_after=!!uint32 1"
  "scrambler_block.monitor_delays=\"1,2\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_monitor_three
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting the forced-transition group: gr::digital::configureMonitor: monitorDelays must name exactly two distinct delays, got '1,2,3'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.force_transition_after=!!uint32 8"
  "scrambler_block.monitor_delays=\"1,2,3\"")
gr_add_failing_graph_test(
  graph_digital_refused_scrambler_monitor_one
  graphs/scrambler-settings.yaml
  "MultiplicativeScrambler[^\n]* throws: setting the forced-transition group: gr::digital::configureMonitor: monitorDelays must name exactly two distinct delays, got '2'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "scrambler_block.force_transition_after=!!uint32 8"
  "scrambler_block.monitor_delays=\"2\"")
gr_digital_stream_graph_test(
  graph_digital_counters_scrambler_unscrambled
  graphs/scrambler-sequence.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::AdditiveScrambler 'once_scrambler': 7 item.s. past the end of the explicit sequence left unscrambled"
)
gr_digital_stream_graph_test(
  graph_digital_counters_scrambler_fired_scrambler
  graphs/scrambler-forced.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::MultiplicativeScrambler 'running_scrambler': the forcing term fired 5 time.s.")
gr_digital_stream_graph_test(
  graph_digital_counters_scrambler_fired_descrambler
  graphs/scrambler-forced.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::MultiplicativeDescrambler 'running_descrambler': the forcing term fired 5 time.s.")

# The graph cases of AccessCodeCorrelator.
gr_digital_stream_graph_test(graph_digital_access_code_detection graphs/access-code-detection.yaml)
gr_digital_stream_graph_test(graph_digital_access_code_errors graphs/access-code-errors.yaml)
gr_digital_stream_graph_test(graph_digital_access_code_warmup graphs/access-code-warmup.yaml)
gr_digital_stream_graph_test(graph_digital_access_code_tags graphs/access-code-tags.yaml)
gr_digital_stream_graph_test(graph_digital_access_code_framing graphs/access-code-framing.yaml)
gr_digital_stream_graph_test(graph_digital_access_code_strided graphs/access-code-strided.yaml)
gr_digital_stream_graph_test(graph_digital_access_code_settings graphs/access-code-settings.yaml)
gr_digital_stream_graph_test(
  graph_digital_accepted_access_code_one_limb
  graphs/access-code-settings.yaml
  SET
  "byte_correlator.access_code=\"1111111111111111111111111111111111111111111111111111111111111111\""
  "byte_correlator.max_errors=!!uint32 64")
gr_digital_stream_graph_test(
  graph_digital_accepted_access_code_two_limbs
  graphs/access-code-settings.yaml
  SET
  "byte_correlator.access_code=\"11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111\""
  "byte_correlator.max_errors=!!uint32 128")
gr_digital_stream_graph_test(
  graph_digital_accepted_access_code_stride_cap
  graphs/access-code-settings.yaml
  SET
  "byte_correlator.stride=!!uint32 4096")
gr_digital_stream_graph_test(
  graph_digital_accepted_access_code_strided_limbs
  graphs/access-code-settings.yaml
  SET
  "byte_correlator.access_code=\"11111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111\""
  "byte_correlator.stride=!!uint32 80")
gr_digital_stream_graph_test(
  graph_digital_accepted_access_code_soft_strided
  graphs/access-code-settings.yaml
  SET
  "soft_correlator.stride=!!uint32 80"
  "soft_correlator.tag_at=code_start")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_empty
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: access_code is 0 bits. it must be between 1 and 128, and there is no default sync word"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"\"")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_long
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: access_code is 129 bits. it must be between 1 and 128"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111\""
)
gr_add_failing_graph_test(
  graph_digital_refused_access_code_bound
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: max_errors 5 exceeds the 4 symbols of the access code, which would match everything"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"1011\""
  "byte_correlator.max_errors=!!uint32 5")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_space
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: access_code must be '0' and '1' characters only, got ' ' in \\\"10 11\\\""
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"10 11\"")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_digit
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: access_code must be '0' and '1' characters only, got '2' in \\\"1012\\\""
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"1012\"")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_width_zero
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: bits_per_item must be between 1 and 8, got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"1010\""
  "byte_correlator.bits_per_item=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_width_nine
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: bits_per_item must be between 1 and 8, got 9"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"111111111111111111\""
  "byte_correlator.bits_per_item=!!uint32 9")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_whole_symbols
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: access_code is 5 bits, not a whole number of 2-bit symbols"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"10101\""
  "byte_correlator.bits_per_item=!!uint32 2")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_symbol_bound
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: max_errors 3 exceeds the 2 symbols of the access code"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"1010\""
  "byte_correlator.bits_per_item=!!uint32 2"
  "byte_correlator.max_errors=!!uint32 3")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_soft_width
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<float32>[^\n]* throws: a soft stream carries one bit per item. bits_per_item above 1 needs byte items"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "soft_correlator.access_code=\"1010\""
  "soft_correlator.bits_per_item=!!uint32 2")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_stride_zero
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: stride is the item spacing of a distributed sync word and must be in .1, 4096., got 0"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.stride=!!uint32 0")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_stride_above
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: stride is the item spacing of a distributed sync word and must be in .1, 4096., got 4097"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.stride=!!uint32 4097")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_tag_at
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: tag_at must be 'payload_start' or 'code_start', got 'frame_start'"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.tag_at=frame_start")
gr_add_failing_graph_test(
  graph_digital_refused_access_code_strided_long
  graphs/access-code-settings.yaml
  "AccessCodeCorrelator<uint8>[^\n]* throws: access_code is 129 bits"
  PLUGIN_DIRS
  $<TARGET_FILE_DIR:GrDigitalBlocksShared>
  SET
  "byte_correlator.access_code=\"111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111\""
  "byte_correlator.stride=!!uint32 80")
gr_digital_stream_graph_test(
  graph_digital_counters_access_code_strided_4_3
  graphs/access-code-strided.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::AccessCodeCorrelator 'strided_4_3_correlator': 1 detection.s.")
gr_digital_stream_graph_test(
  graph_digital_counters_access_code_strided_8_5
  graphs/access-code-strided.yaml
  PASS_REGULAR_EXPRESSION
  "gr::blocks::digital::AccessCodeCorrelator 'strided_8_5_correlator': 1 detection.s.")
