set(demo_metrics_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/metrics/main.cpp)
add_executable(demo_metrics ${demo_metrics_SOURCE_FILES})
target_link_libraries(demo_metrics generalized_relpose)
