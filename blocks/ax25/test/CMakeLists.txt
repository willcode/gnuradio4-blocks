add_ut_test(qa_Ax25)
target_link_libraries(qa_Ax25 PRIVATE gr-ax25 gr-digital)

add_ut_test(qa_Kiss)
target_link_libraries(qa_Kiss PRIVATE gr-ax25 gr-digital)
