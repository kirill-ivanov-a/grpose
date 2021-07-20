set(multicam_demo_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/dataset/multicam/multicam_demo/main.cpp)
add_executable(multicam_demo ${multicam_demo_SOURCE_FILES})
target_link_libraries(multicam_demo grpose)
