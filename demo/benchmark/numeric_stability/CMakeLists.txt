set(numeric_stability_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/benchmark/numeric_stability/main.cpp)
add_executable(numeric_stability ${numeric_stability_SOURCE_FILES})
target_link_libraries(numeric_stability grpose)
