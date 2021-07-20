cmake_minimum_required(VERSION 3.13.0)
project(grpose LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

option(ASAN_BUILD "Build with Address Sanitizer" OFF)
option(PROFILE_BUILD "Build with perftools profiler" OFF)

if (ASAN_BUILD)
    add_compile_options("-fsanitize=address")
    link_libraries("-fsanitize=address")
endif()

if (PROFILE_BUILD)
    link_libraries("-lprofiler")
endif()


find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(fmt REQUIRED)

# Find version of OpenCV with free SIFT
# If not found, ask user to provide the path to one
find_package(OpenCV 4.4 PATHS ${OPENCV_INSTALL_DIR} REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR  "OpenCV version 4.4 or compatible not found.")
  if (NOT OpenCV_FOUND)
    message("Could not find OpenCV 4.4 or compatible, consider setting the variable OPENCV_INSTALL_DIR")
  endif()
endif()

add_subdirectory(${PROJECT_SOURCE_DIR}/ext/googletest)
set(BUILD_TESTS OFF)
set(BUILD_EXAMPLES OFF)
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/opengv)
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/Sophus)

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native ")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")

# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -march=native")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g3 -DNDEBUG -march=native")

set(grpose_HEADER_FILES
    ${PROJECT_SOURCE_DIR}/include/types.h
    ${PROJECT_SOURCE_DIR}/include/util.h
    ${PROJECT_SOURCE_DIR}/include/defs.h
    ${PROJECT_SOURCE_DIR}/include/CameraModel.h
    ${PROJECT_SOURCE_DIR}/include/CameraBundle.h
    ${PROJECT_SOURCE_DIR}/include/metrics.h
    ${PROJECT_SOURCE_DIR}/include/trajectory.h

    ${PROJECT_SOURCE_DIR}/include/dataset/DatasetReader.h
    ${PROJECT_SOURCE_DIR}/include/dataset/RobotcarReader.h
    ${PROJECT_SOURCE_DIR}/include/dataset/MultiCamReader.h

    ${PROJECT_SOURCE_DIR}/include/openGVInitialization/FeatureDetectorMatcher.h
    ${PROJECT_SOURCE_DIR}/include/openGVInitialization/NonCentralRelativePoseSolver.h
    ${PROJECT_SOURCE_DIR}/include/openGVInitialization/MultiCamInitSettings.h
    ${PROJECT_SOURCE_DIR}/include/openGVInitialization/PoseFileWriter.h
)

set(grpose_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/util.cpp
    ${PROJECT_SOURCE_DIR}/src/CameraModel.cpp
    ${PROJECT_SOURCE_DIR}/src/CameraBundle.cpp
    ${PROJECT_SOURCE_DIR}/src/metrics.cpp
    ${PROJECT_SOURCE_DIR}/src/trajectory.cpp

    ${PROJECT_SOURCE_DIR}/src/dataset/DatasetReader.cpp
    ${PROJECT_SOURCE_DIR}/src/dataset/RobotcarReader.cpp
    ${PROJECT_SOURCE_DIR}/src/dataset/MultiCamReader.cpp

    ${PROJECT_SOURCE_DIR}/src/openGVInitialization/FeatureDetectorMatcher.cpp
    ${PROJECT_SOURCE_DIR}/src/openGVInitialization/NonCentralRelativePoseSolver.cpp
    ${PROJECT_SOURCE_DIR}/src/openGVInitialization/MultiCamInitSettings.cpp
    ${PROJECT_SOURCE_DIR}/src/openGVInitialization/PoseFileWriter.cpp
)

add_library(grpose
        ${grpose_SOURCE_FILES}
        ${grpose_HEADER_FILES})

target_include_directories(grpose
  PUBLIC
    ${GFLAGS_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/ext/Sophus
)

target_link_libraries(grpose
  PUBLIC
    gflags
    ${OpenCV_LIBS}
    ${GLOG_LIBRARIES}
    opengv
    sophus
)

add_subdirectory(demo)

enable_testing()
add_subdirectory(test)
