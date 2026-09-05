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

add_ut_test(qa_OfdmChannelEqualizer)
# reaches the library FFT, whose always_inline helpers do not inline at the qa tier's default level
gr_pin_test_optimization(qa_OfdmChannelEqualizer -O2)
target_link_libraries(
  qa_OfdmChannelEqualizer
  PRIVATE gnuradio4::gr-ofdm
          gnuradio4::gr-channel
          gnuradio4::gr-digital
          gnuradio4::gr-filter
          gnuradio4::gr-testing)

add_ut_test(qa_OfdmLoopback)
# reaches the library FFT and the phasor, whose always_inline helpers do not inline at the qa tier's default level
gr_pin_test_optimization(qa_OfdmLoopback -O2)
target_link_libraries(
  qa_OfdmLoopback
  PRIVATE gnuradio4::gr-ofdm
          gnuradio4::gr-channel
          gnuradio4::gr-digital
          gnuradio4::gr-filter
          gnuradio4::gr-testing)

# the recipes gate for this family: the composites load, and the numerology reaches every interior block. The shared
# library is linked so the interior registry ids the recipes name are registered when the test runs.
add_ut_test(qa_OfdmRecipes)
target_link_libraries(qa_OfdmRecipes PRIVATE gnuradio-core gnuradio4::gr-ofdm gnuradio4::GrOfdmBlocksShared)
target_compile_definitions(qa_OfdmRecipes PRIVATE RECIPES_SOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../recipes")
