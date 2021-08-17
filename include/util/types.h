#ifndef GRPOSE_UTIL_TYPES_
#define GRPOSE_UTIL_TYPES_

#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <queue>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

namespace grpose {

using Vector2 = Eigen::Vector2d;
using Vector3 = Eigen::Vector3d;
using Vector6 = Eigen::Matrix<double, 6, 1>;
using VectorX = Eigen::VectorXd;

using Vector2i = Eigen::Vector2i;

using Matrix22 = Eigen::Matrix2d;
using Matrix23 = Eigen::Matrix<double, 2, 3>;
using Matrix32 = Eigen::Matrix<double, 3, 2>;
using Matrix33 = Eigen::Matrix3d;
using Matrix34 = Eigen::Matrix<double, 3, 4>;
using Matrix44 = Eigen::Matrix4d;
using MatrixX2 = Eigen::MatrixX2d;
using MatrixXX = Eigen::MatrixXd;

using Quaternion = Eigen::Quaterniond;

using Sim3 = Sophus::Sim3d;
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

// STL containers with 32-bytes alignment allocation policy.
// Use them whenever you want to store a vectorizable Eigen class or a structure
// with it as a member (see
// https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html). Note that
// Sophus objects are also vectorizable because of quaternions.
template <typename T>
using StdVectorA = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename K, typename T>
using StdMapA = std::map<K, T, std::less<K>,
                         Eigen::aligned_allocator<std::pair<const K, T>>>;

/**
 * Time since the Epoch in microseconds.
 */
using Timestamp = uint64_t;

using ChronoTimePoint =
    std::chrono::time_point<std::chrono::high_resolution_clock>;

namespace fs = std::filesystem;

}  // namespace grpose

#endif
