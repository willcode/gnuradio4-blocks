add_ut_test(qa_PowerEstimators)
target_link_libraries(qa_PowerEstimators PRIVATE gr-electrical gr-basic)

add_ut_test(qa_ElectricalSettingsInit)
target_link_libraries(qa_ElectricalSettingsInit PRIVATE gr-electrical gr-basic gr-testing)
