set(synthetic_ransac_benchmark_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/benchmark/synthetic_ransac_benchmark/main.cpp)
add_executable(synthetic_ransac_benchmark ${synthetic_ransac_benchmark_SOURCE_FILES})
target_link_libraries(synthetic_ransac_benchmark grpose)
