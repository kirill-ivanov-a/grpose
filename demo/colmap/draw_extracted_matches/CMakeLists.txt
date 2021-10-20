set(draw_extracted_matches_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/demo/colmap/draw_extracted_matches/main.cpp)
add_executable(draw_extracted_matches ${draw_extracted_matches_SOURCE_FILES})
target_link_libraries(draw_extracted_matches grpose)
