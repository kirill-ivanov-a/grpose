# Executable(s) for running test cases on purely synthetic correspondences

add_executable(SyntheticInitialization ${PROJECT_SOURCE_DIR}/demo/opengv_initialization/gv_experiments/SyntheticData.cpp)
target_link_libraries(SyntheticInitialization grpose)

add_executable(SyntheticInitializationDegenCase ${PROJECT_SOURCE_DIR}/demo/opengv_initialization/gv_experiments/SyntheticDataDegenCase.cpp)
target_link_libraries(SyntheticInitializationDegenCase grpose)



# Executable(s) for running test cases on arbitrary dataset

add_executable(OpenGVInitialization ${PROJECT_SOURCE_DIR}/demo/opengv_initialization/OpenGVInitialization.cpp)
target_link_libraries(OpenGVInitialization grpose)