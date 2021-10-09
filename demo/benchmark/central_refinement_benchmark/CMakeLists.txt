set(central_refinement_benchmark_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/benchmark/central_refinement_benchmark/main.cpp)
add_executable(central_refinement_benchmark ${central_refinement_benchmark_SOURCE_FILES})
target_link_libraries(central_refinement_benchmark grpose)
