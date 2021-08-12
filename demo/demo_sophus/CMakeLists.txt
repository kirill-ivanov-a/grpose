set(demo_sophus_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/demo_sophus/main.cpp)
add_executable(demo_sophus ${demo_sophus_SOURCE_FILES})
target_link_libraries(demo_sophus grpose)
