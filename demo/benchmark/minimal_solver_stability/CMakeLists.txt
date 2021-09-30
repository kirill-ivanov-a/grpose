set(minimal_solver_stability_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/benchmark/minimal_solver_stability/main.cpp)
add_executable(minimal_solver_stability ${minimal_solver_stability_SOURCE_FILES})
target_link_libraries(minimal_solver_stability grpose)
