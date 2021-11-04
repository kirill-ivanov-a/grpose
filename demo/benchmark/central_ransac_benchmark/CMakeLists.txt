set(central_ransac_benchmark_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/benchmark/central_ransac_benchmark/main.cpp)
add_executable(central_ransac_benchmark ${central_ransac_benchmark_SOURCE_FILES})
target_link_libraries(central_ransac_benchmark grpose)
