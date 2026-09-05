add_ut_test(qa_SpectralEstimate)
# reaches SimdFFT, whose always_inline helpers do not inline at the qa tier's default level
gr_pin_test_optimization(qa_SpectralEstimate -O2)
target_link_libraries(qa_SpectralEstimate PRIVATE gnuradio4::gr-measurement)

add_ut_test(qa_Detectors)
target_link_libraries(qa_Detectors PRIVATE gnuradio4::gr-measurement)
