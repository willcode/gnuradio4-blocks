add_ut_test(qa_SpectralEstimate)
# reaches SimdFFT, whose always_inline helpers do not inline at the qa tier's default level
gr_pin_test_optimization(qa_SpectralEstimate -O2)
target_link_libraries(qa_SpectralEstimate PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_Detectors)
target_link_libraries(qa_Detectors PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_Kurtosis)
target_link_libraries(qa_Kurtosis PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_SpectralKurtosis)
target_link_libraries(qa_SpectralKurtosis PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_PhaseUnwrap)
target_link_libraries(qa_PhaseUnwrap PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_PolarizationCombiner)
target_link_libraries(qa_PolarizationCombiner PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_Autocorrelation)
# runs its estimates and its scenes through the library FFT, so it reaches SimdFFT and needs the inliner depth -O2
# provides
gr_pin_test_optimization(qa_Autocorrelation -O2)
target_link_libraries(qa_Autocorrelation PRIVATE gnuradio4::gr-measurement)

# the recorded-air legs, in their own executable because exit 77 is a whole binary's status: folded into
# qa_Autocorrelation an absent recording would skip the synthetic gate with it
add_ut_test(qa_AutocorrelationRecording)
gr_pin_test_optimization(qa_AutocorrelationRecording -O2)
target_link_libraries(qa_AutocorrelationRecording PRIVATE gnuradio4::gr-measurement gnuradio4::gr-analog
                                                          gnuradio4::gr-fileio)
set_property(
  TEST qa_AutocorrelationRecording
  APPEND
  PROPERTY ENVIRONMENT_MODIFICATION "GR4_RECORDINGS_DIR=set:${GR4_RECORDINGS_DIR}")
set_property(TEST qa_AutocorrelationRecording PROPERTY SKIP_RETURN_CODE 77)
