#ifndef GRPOSE_CAMERA_CAMERA_MODEL_
#define GRPOSE_CAMERA_CAMERA_MODEL_

#include <type_traits>

#include <ceres/jet.h>
#include <ceres/local_parameterization.h>

#include "util/type_traits.h"
#include "util/types.h"

namespace grpose {

// Numbers after kScaramuzza refer to the unmapping/mapping polynomial degree,
// see include/camera/camera_model_scaramuzza.h for details.
enum class CameraModelId { kInvalid = -1, kPinhole, kMultiFov };

/**
 * The result of differentiating CameraModelImpl::Map(direction, parameters)
 * w.r.t. normalized direction. Note that if the input direction was not
 * normalized, it will still be, and the resulting derivative will be not w.r.t.
 * the original direction.
 */
struct DifferentiatedMapResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Mapping result.
  Vector2 point;

  /**
   * Jacobian w.r.t. normalized direction. Despite being a 2x3 matrix, it is
   * only well-defined on the 2-dimentional tangent space on S^2 to the
   * direction being mapped.
   */
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
  template <typename DirectionDerived>
  static bool IsMappable(const Eigen::MatrixBase<DirectionDerived> &direction,
                         const std::vector<double> &parameters);

  template <typename DirectionDerived>
  static DifferentiatedMapResult DifferentiateMap(
      const Eigen::MatrixBase<DirectionDerived> &direction,
      const std::vector<double> &parameters);

  template <typename PointDerived>
  static Vector3 UnmapApproximate(const Eigen::MatrixBase<PointDerived> &point,
                                  const std::vector<double> &parameters);

  template <typename PointDerived>
  static Vector3 Unmap(const Eigen::MatrixBase<PointDerived> &point,
                       const std::vector<double> &parameters);

  template <typename PointDerived>
  static Vector3 UnmapUnnormalized(const Eigen::MatrixBase<PointDerived> &point,
                                   const std::vector<double> &parameters);
};

// Implementation

template <typename Derived>
template <typename DirectionDerived>
bool CameraModel<Derived>::IsMappable(
    const Eigen::MatrixBase<DirectionDerived> &direction,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR3(direction);

  return true;
}

template <typename Derived>
template <typename DirectionDerived>
DifferentiatedMapResult CameraModel<Derived>::DifferentiateMap(
    const Eigen::MatrixBase<DirectionDerived> &direction,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR3(direction);

  using Jet3 = ceres::Jet<double, 3>;

  const Vector3 direction_normalized = direction.normalized();
  Eigen::Matrix<Jet3, 3, 1> direction_jets =
      direction_normalized.template cast<Jet3>();
  for (int i = 0; i < 3; ++i) direction_jets[i].v[i] = 1.0;

  const Eigen::Matrix<Jet3, 2, 1> point_jet =
      Derived::Map(direction_jets, parameters);

  DifferentiatedMapResult result;
  result.point = Vector2(point_jet[0].a, point_jet[1].a);
  result.jacobian << point_jet[0].v.transpose(), point_jet[1].v.transpose();
  return result;
}

template <typename Derived>
template <typename PointDerived>
Vector3 CameraModel<Derived>::UnmapApproximate(
    const Eigen::MatrixBase<PointDerived> &point,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  return Vector3(0.0, 0.0, 1.0);
}

template <typename Derived>
template <typename PointDerived>
Vector3 CameraModel<Derived>::Unmap(
    const Eigen::MatrixBase<PointDerived> &point,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  constexpr int kMaxIterations = 100;
  constexpr double kMinStepSquaredNorm = 1e-6;
  constexpr double kMinDifferenceSquaredNorm = 1e-8;

  // Each step by \Delta introduces rotation of the direction by the angle
  // |\Delta|/2. We want to constraint this rotation s.t. it never happens more
  // than by \pi/4
  constexpr double kMaxStepNorm = M_PI_2;
  constexpr double kMaxStepSquaredNorm = kMaxStepNorm * kMaxStepNorm;

  ceres::HomogeneousVectorParameterization parameterization(3);

  Vector3 direction = Derived::UnmapApproximate(point, parameters).normalized();
  for (int it = 0; it < kMaxIterations; ++it) {
    const auto [mapped, map_jacobian] =
        Derived::DifferentiateMap(direction, parameters);
    const Vector2 residual = point - mapped;
    if (residual.squaredNorm() < kMinDifferenceSquaredNorm) break;

    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> plus_jacobian;
    parameterization.ComputeJacobian(direction.data(), plus_jacobian.data());

    const Matrix22 residual_jacobian = map_jacobian * plus_jacobian;
    Vector2 delta = residual_jacobian.fullPivHouseholderQr().solve(residual);
    const double squared_delta_norm = delta.squaredNorm();
    if (squared_delta_norm > kMaxStepSquaredNorm)
      delta *= kMaxStepNorm / std::sqrt(squared_delta_norm);

    Vector3 new_direction;
    parameterization.Plus(direction.data(), delta.data(), new_direction.data());
    direction = new_direction;
    if (squared_delta_norm < kMinStepSquaredNorm) break;
  }

  return direction;
}

template <typename Derived>
template <typename PointDerived>
Vector3 CameraModel<Derived>::UnmapUnnormalized(
    const Eigen::MatrixBase<PointDerived> &point,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  return Derived::Unmap(point, parameters);
}

}  // namespace grpose

#endif
