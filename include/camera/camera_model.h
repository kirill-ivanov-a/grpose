#ifndef GRPOSE_CAMERA_CAMERA_MODEL_
#define GRPOSE_CAMERA_CAMERA_MODEL_

#include <ceres/jet.h>
#include <ceres/local_parameterization.h>

#include "types.h"

namespace grpose {

enum class CameraModelId { kInvalidId = -1, kPinhole };

struct DifferentiatedMapResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector2 point;
  Matrix23 jacobian;
};

/**
 * Camera model class based on CRTP pattern.
 *
 * The architecture is largely based on COLMAP's CameraModel class. It can be
 * found under https://github.com/colmap/colmap .
 *
 * Only Map function is templated and differentiable, since for most uses we
 * only need to differentiate mapping to image, not the other way around.
 *
 * @tparam Derived The concrete CameraModel implementation class.
 */
template <typename Derived>
class CameraModel {
 public:
  static bool IsMappable(const Vector3 &directions,
                         const std::vector<double> &parameters);

  static DifferentiatedMapResult DifferentiateMap(
      const Vector3 &direction, const std::vector<double> &parameters);

  static Vector3 UnmapApproximation(const Vector2 &point,
                                    const std::vector<double> &parameters);

  static Vector3 Unmap(const Vector2 &point,
                       const std::vector<double> &parameters);

  static Vector3 UnmapUnnormalized(const Vector2 &point,
                                   const std::vector<double> &parameters);
};

template <typename Derived>
bool CameraModel<Derived>::IsMappable(const Vector3 &directions,
                                      const std::vector<double> &parameters) {
  return true;
}

template <typename Derived>
DifferentiatedMapResult CameraModel<Derived>::DifferentiateMap(
    const Vector3 &direction, const std::vector<double> &parameters) {
  using Jet3 = ceres::Jet<double, 3>;

  Eigen::Matrix<Jet3, 3, 1> direction_jets = direction.template cast<Jet3>();
  for (int i = 0; i < 3; ++i) direction_jets[i].v[i] = 1.0;

  Eigen::Matrix<Jet3, 2, 1> point_jet =
      Derived::Map(direction_jets, parameters);

  DifferentiatedMapResult result;
  result.point = Vector2(point_jet[0].a, point_jet[1].a);
  result.jacobian << point_jet[0].v.transpose(), point_jet[1].v.transpose();
  return result;
}

template <typename Derived>
Vector3 CameraModel<Derived>::UnmapApproximation(
    const Vector2 &point, const std::vector<double> &parameters) {
  return Vector3(0, 0, 1);
}

template <typename Derived>
Vector3 CameraModel<Derived>::Unmap(const Vector2 &point,
                                    const std::vector<double> &parameters) {
  constexpr int kMaxIterations = 100;
  constexpr double kMinStepSquaredNorm = 1e-6;
  constexpr double kMinDifferenceSquaredNorm = 1e-8;

  ceres::HomogeneousVectorParameterization parameterization(3);

  Vector3 direction = Derived::UnmapApproximation(point, parameters);
  for (int it = 0; it < kMaxIterations; ++it) {
    auto [mapped, map_jacobian] =
        Derived::DifferentiateMap(direction, parameters);
    Vector2 residual = mapped - point;
    if (residual.squaredNorm() < kMinDifferenceSquaredNorm) break;

    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> plus_jacobian;
    parameterization.ComputeJacobian(direction.data(), plus_jacobian.data());

    Matrix22 residual_jacobian = map_jacobian * plus_jacobian;
    Vector2 delta = residual_jacobian.fullPivHouseholderQr().solve(residual);
    Vector3 new_direction;
    parameterization.Plus(direction.data(), delta.data(), new_direction.data());
    direction = new_direction;

    if (delta.squaredNorm() < kMinStepSquaredNorm) break;
  }

  return direction;
}

template <typename Derived>
Vector3 CameraModel<Derived>::UnmapUnnormalized(
    const Vector2 &point, const std::vector<double> &parameters) {
  return Derived::Unmap(point, parameters);
}

}  // namespace grpose

#endif
