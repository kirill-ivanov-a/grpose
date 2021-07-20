set(reproj_test_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/dataset/robotcar/reproj_test/main.cpp)
add_executable(reproj_test ${reproj_test_SOURCE_FILES})
target_link_libraries(reproj_test grpose)
