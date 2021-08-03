set(autovision_demo_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/dataset/autovision/autovision_demo/main.cpp)
add_executable(autovision_demo ${autovision_demo_SOURCE_FILES})
target_link_libraries(autovision_demo grpose)
