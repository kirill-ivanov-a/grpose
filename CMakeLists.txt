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
endif ()

if (PROFILE_BUILD)
    link_libraries("-lprofiler")
endif ()


find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED)

# Find version of OpenCV with free SIFT
# If not found, ask user to provide the path to one
find_package(OpenCV 4.4 PATHS ${OPENCV_INSTALL_DIR} REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV version 4.4 or compatible not found.")
    if (NOT OpenCV_FOUND)
        message("Could not find OpenCV 4.4 or compatible, consider setting the variable OPENCV_INSTALL_DIR")
    endif ()
endif ()

add_subdirectory(${PROJECT_SOURCE_DIR}/ext/googletest)

set(BUILD_TESTS OFF CACHE BOOL "Build OpenGV's and Sophus's tests")
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/opengv)

set(BUILD_EXAMPLES OFF CACHE BOOL "Build Sophus's examples")
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/Sophus)

# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native ")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native")

# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -march=native")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g3 -DNDEBUG -march=native")

set(grpose_HEADER_FILES
        include/util/types.h
        include/util/util.h
        ${PROJECT_SOURCE_DIR}/include/metrics.h
        ${PROJECT_SOURCE_DIR}/include/trajectory.h

        include/camera/camera.h
        include/camera/camera_bundle.h
        include/camera/camera_model.h
        include/camera/camera_model_pinhole.h
        include/camera/camera_model_scaramuzza.h
        include/camera/camera_model_generic.h

        ${PROJECT_SOURCE_DIR}/include/dataset/dataset_reader.h
        ${PROJECT_SOURCE_DIR}/include/dataset/robotcar_reader.h
        ${PROJECT_SOURCE_DIR}/include/dataset/multicam_reader.h
        ${PROJECT_SOURCE_DIR}/include/dataset/autovision_reader.h

        ${PROJECT_SOURCE_DIR}/include/init_opengv/feature_detector_matcher.h
        ${PROJECT_SOURCE_DIR}/include/init_opengv/non_central_relative_pose_solver.h
        ${PROJECT_SOURCE_DIR}/include/init_opengv/multicam_init_settings.h
        ${PROJECT_SOURCE_DIR}/include/init_opengv/write_poses_to_file.h
        )

set(grpose_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/util.cpp
        ${PROJECT_SOURCE_DIR}/src/metrics.cpp
        ${PROJECT_SOURCE_DIR}/src/trajectory.cpp

        src/camera/camera.cpp
        src/camera/camera_bundle.cpp

        ${PROJECT_SOURCE_DIR}/src/dataset/dataset_reader.cpp
        ${PROJECT_SOURCE_DIR}/src/dataset/robotcar_reader.cpp
        ${PROJECT_SOURCE_DIR}/src/dataset/multicam_reader.cpp
        ${PROJECT_SOURCE_DIR}/src/dataset/autovision_reader.cpp

        ${PROJECT_SOURCE_DIR}/src/init_opengv/feature_detector_matcher.cpp
        ${PROJECT_SOURCE_DIR}/src/init_opengv/non_central_relative_pose_solver.cpp
        ${PROJECT_SOURCE_DIR}/src/init_opengv/multicam_init_settings.cpp
        ${PROJECT_SOURCE_DIR}/src/init_opengv/write_poses_to_file.cpp
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

        PRIVATE
        ${Boost_INCLUDE_DIRS}
        )

target_link_libraries(grpose
        PUBLIC
        opengv
        sophus
        fmt::fmt
        gflags
        ${OpenCV_LIBS}
        ${GLOG_LIBRARIES}
        ${CERES_LIBRARIES}
        )

add_subdirectory(demo)

enable_testing()
add_subdirectory(test)
