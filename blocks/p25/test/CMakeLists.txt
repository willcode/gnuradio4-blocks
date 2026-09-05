add_ut_test(qa_P25Protocol)
target_link_libraries(qa_P25Protocol PRIVATE gr-p25)

add_ut_test(qa_P25FrameSync)
target_link_libraries(qa_P25FrameSync PRIVATE gr-p25)

add_ut_test(qa_P25FrameLayer)
target_link_libraries(qa_P25FrameLayer PRIVATE gr-p25)

add_ut_test(qa_P25Payload)
target_link_libraries(qa_P25Payload PRIVATE gr-p25)

add_ut_test(qa_P25ImbeFrame)
target_link_libraries(qa_P25ImbeFrame PRIVATE gr-p25)

add_ut_test(qa_P25Nid)
target_link_libraries(qa_P25Nid PRIVATE gr-p25)

