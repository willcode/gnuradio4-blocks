add_ut_test(qa_SpectralEstimate)
# Reaches SimdFFT, whose always_inline helpers need the inliner depth -O2 provides.
gr_pin_test_optimization(qa_SpectralEstimate -O2)
target_link_libraries(qa_SpectralEstimate PRIVATE gnuradio4::gr-measurement gnuradio4::gr-testing)
