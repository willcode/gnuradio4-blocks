add_ut_test(qa_Recipes)
target_compile_definitions(qa_Recipes PRIVATE RECIPES_SOURCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/..")
