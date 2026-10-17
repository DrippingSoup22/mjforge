FetchContent_Declare(doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.12
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR do-not-add)
FetchContent_MakeAvailable(doctest)

file(GLOB MJF_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_executable(mjforge_tests ${MJF_TEST_SOURCES})
target_include_directories(mjforge_tests PRIVATE ${doctest_SOURCE_DIR})
target_link_libraries(mjforge_tests PRIVATE mjforge_core)
target_compile_definitions(mjforge_tests PRIVATE MJF_TEST_TMP="${CMAKE_BINARY_DIR}/test_tmp")
add_custom_command(TARGET mjforge_tests POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MJFORGE_MUJOCO_RUNTIME_FILE}" "$<TARGET_FILE_DIR:mjforge_tests>"
  VERBATIM)
add_test(NAME mjforge_tests COMMAND mjforge_tests)
