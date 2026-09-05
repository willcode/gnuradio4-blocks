add_ut_test(qa_FecBlocks)
target_link_libraries(qa_FecBlocks PRIVATE gr-fec)

add_ut_test(qa_ConvBlocks)
target_link_libraries(qa_ConvBlocks PRIVATE gr-fec)

add_ut_test(qa_PunctureBlocks)
target_link_libraries(qa_PunctureBlocks PRIVATE gr-fec)

add_ut_test(qa_InterleaveBlocks)
target_link_libraries(qa_InterleaveBlocks PRIVATE gr-fec)

# The acceptance gate for the CCSDS coding row: kernels only, judged against the published curve.
add_ut_test(qa_CcsdsBer)
target_link_libraries(qa_CcsdsBer PRIVATE gr-fec)

if(GR4_FEC_HAVE_AFF3CT)
  add_ut_test(qa_Aff3ctBlocks)
  target_link_libraries(qa_Aff3ctBlocks PRIVATE gr-fec)
  # The wall-integrity check reads this module's own headers, so it is told where they are rather than guessing at a
  # path relative to wherever the test binary was run from.
  target_compile_definitions(qa_Aff3ctBlocks PRIVATE GR4_FEC_INCLUDE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../include")

  # The acceptance gate needs a channel to run its loopbacks through, and it is the tree's own.
  add_ut_test(qa_Aff3ctBer)
  target_link_libraries(qa_Aff3ctBer PRIVATE gr-fec gr-channel)
endif()
