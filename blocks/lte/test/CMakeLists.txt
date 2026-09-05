add_ut_test(qa_LteCellSearch)
target_link_libraries(qa_LteCellSearch PRIVATE gr-lte gr-channel)
# builds its scenes and both kernel references through the library FFT, and sweeps the correlator over hundreds of
# half-frames, so it needs the inliner depth -O2 provides
gr_pin_test_optimization(qa_LteCellSearch -O2)
