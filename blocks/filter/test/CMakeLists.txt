add_ut_test(qa_filter)
target_link_libraries(qa_filter PRIVATE gr-filter gr-testing)

add_ut_test(qa_ArbitraryRateResampler)
target_link_libraries(qa_ArbitraryRateResampler PRIVATE gr-filter gr-testing)
add_ut_test(qa_FirFilter)
target_link_libraries(qa_FirFilter PRIVATE gr-filter gr-testing)
add_ut_test(qa_FrequencyEstimator)
# SimdFFT's always_inline helpers need the inliner depth -O2 provides; at -O1 the unresolved indirect call is a hard
# error.
gr_pin_test_optimization(qa_FrequencyEstimator -O2)
target_link_libraries(qa_FrequencyEstimator PRIVATE gr-filter gr-testing)

add_ut_test(qa_RationalResampler)
target_link_libraries(qa_RationalResampler PRIVATE gr-filter gr-testing)
add_ut_test(qa_SvdDenoiser)
target_link_libraries(qa_SvdDenoiser PRIVATE gr-filter gr-testing)

add_ut_test(qa_SavitzkyGolayFilter)
target_link_libraries(qa_SavitzkyGolayFilter PRIVATE gr-filter gr-testing)
add_ut_test(qa_DesignedFilter)
target_link_libraries(qa_DesignedFilter PRIVATE gr-filter gr-testing)
add_ut_test(qa_StagedDecimator)
gr_pin_test_optimization(qa_StagedDecimator BUILD_TYPE)
target_link_libraries(qa_StagedDecimator PRIVATE gr-filter gr-testing)
add_ut_test(qa_Channelizer)
# The polyphase bank uses SimdFFT. Its always_inline helpers fail to inline at -O1, a hard error.
gr_pin_test_optimization(qa_Channelizer -O2)
target_link_libraries(qa_Channelizer PRIVATE gr-filter gr-testing)
# The sdr module registers qa_DcBlocker for its own block; this one tests filter/DcBlocker.hpp.
add_ut_test(qa_FilterDcBlocker)
target_link_libraries(qa_FilterDcBlocker PRIVATE gr-filter gr-testing)

# each block forwards the tags it holds and the tags past its last input at the end-of-stream index
foreach(_graph fir-filter-end-tags time-domain-fir-end-tags decimator-end-tags)
  string(
    REPLACE "-"
            "_"
            _test
            "${_graph}")
  gr_add_graph_test(
    graph_filter_${_test}
    graphs/${_graph}.yaml
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrFilterBlocksShared>)
endforeach()
foreach(_graph rational-resampler-end-tags staged-decimator-end-tags)
  string(
    REPLACE "-"
            "_"
            _test
            "${_graph}")
  gr_add_graph_test(
    graph_filter_${_test}
    graphs/${_graph}.yaml
    PLUGIN_DIRS
    $<TARGET_FILE_DIR:GrFilterBlocksShared>
    $<TARGET_FILE_DIR:GrBasicBlocksShared>)
endforeach()
