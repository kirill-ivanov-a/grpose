set(sampson_error_comparison_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/benchmark/sampson_error_comparison/main.cpp)
add_executable(sampson_error_comparison ${sampson_error_comparison_SOURCE_FILES})
target_link_libraries(sampson_error_comparison grpose)
