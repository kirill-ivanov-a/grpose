set(TESTS test_camera_model test_metrics test_trajectory)

foreach (CUR_TEST ${TESTS})
    add_executable(${CUR_TEST} ${CUR_TEST}.cpp)
    target_link_libraries(${CUR_TEST} gtest grpose)
    add_test(${CUR_TEST} ${CUR_TEST})
endforeach (CUR_TEST)

# Copy the test timestamped poses file into the build directory.
configure_file(test-file_timestamped-poses.txt test-file_timestamped-poses.txt)
