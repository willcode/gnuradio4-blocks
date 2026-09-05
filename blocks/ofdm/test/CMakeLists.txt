add_ut_test(qa_OfdmCarrierAllocator)
target_link_libraries(qa_OfdmCarrierAllocator PRIVATE gnuradio4::gr-ofdm gnuradio4::gr-testing)

add_ut_test(qa_OfdmCyclicPrefix)
# reaches the library FFT, whose always_inline helpers do not inline at the qa tier's default level
gr_pin_test_optimization(qa_OfdmCyclicPrefix -O2)
target_link_libraries(qa_OfdmCyclicPrefix PRIVATE gnuradio4::gr-ofdm gnuradio4::gr-testing)

add_ut_test(qa_OfdmSchmidlCoxSync)
# reaches the library FFT and the phasor, whose always_inline helpers do not inline at the qa tier's default level
gr_pin_test_optimization(qa_OfdmSchmidlCoxSync -O2)
target_link_libraries(qa_OfdmSchmidlCoxSync PRIVATE gnuradio4::gr-ofdm gnuradio4::gr-channel gnuradio4::gr-testing)
# reaches the library FFT, whose always_inline helpers do not inline at the qa tier's default level
# reaches the library FFT and the phasor, whose always_inline helpers do not inline at the qa tier's default level
