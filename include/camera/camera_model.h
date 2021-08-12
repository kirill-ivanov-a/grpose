#ifndef GRPOSE_CAMERA_CAMERA_MODEL_
#define GRPOSE_CAMERA_CAMERA_MODEL_

#include <type_traits>

#include <ceres/jet.h>
#include <ceres/local_parameterization.h>

#include "util/type_traits.h"
#include "util/types.h"

namespace grpose {

enum class CameraModelId { kInvalid = -1, kPinhole, kMultiFov, kUnified };

/**
 * The result of differentiating CameraModelImpl::Map(direction, parameters)
 * w.r.t. direction.
 */
struct DifferentiatedMapResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Mapping result.
  Vector2 point;

  /// Jacobian w.r.t. direction.
  Matrix23 jacobian;
};

/**
 * Camera model class based on CRTP pattern.
 *
 * The architecture is adapted from COLMAP's CameraModel class. It can be
 * found under https://github.com/colmap/colmap . It follows a similar idea of
 * defining camera models through a vector of parameters and an ID of the model.
 *
 * Only Map function is templated and differentiable, since for most uses we
 * only need to differentiate mapping to image, not the other way around.
 *
 * For a minimal valid implementation one should:
 * 1. Provide a templated static method with the following signature:
 *
 *  template <typename DirectionDerived>
 *  static Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Map(
 *      const Eigen::MatrixBase<DirectionDerived> &direction,
 *      const std::vector<double> &parameters);
 *
 * This method should implement mapping from the camera frame to the image.
 * DirectionDerived::Scalar can be double or ceres::Jet
 * 2. Add a new value to the CameraModelId enum in this file.
 * 3. Register the model in the GRPOSE_DEFINE_CAMERA_MODEL_METHOD macro in
 * camera_model_generic.h
 * 4. Add the camera model implementation to the list of tested classes in
 * test_camera_model.cpp .
 *
 * For an example, see camera_model_pinhole.h .
 *
 * By default, unmapping is implemented as an iterative optimization on S^2.
 * Mapping is differentiated via Ceres's automatic differentiation framework.
 *
 * All methods provided in CameraModel can be overwritten in the implementation
 * by more efficient counterparts. For example, if a closed-form unmapping
 * exists, the implementation can overwrite the Unmap method. Furthermore, if
 * unmapping without normalization can be implemented more efficiently, one can
 * overwrite UnmapUnnormalized. Explicit derivative computation in
 * DifferentiateMap can also provide speedup. In some cases (e.g.
 * CameraModelScaramuzza), an approximate unmapping can be provided to
 * initialize the iterative optimization and thus to help it converge faster. In
 * such cases, UnmapApproximate should be overwritten.
 *
 * @tparam Derived The concrete CameraModel implementation class.
 */
template <typename Derived>
class CameraModel {
 public:
  /**
   * Differentiate the mapping (bearing vectors -> image plane).
   */
  template <typename DirectionDerived>
  static DifferentiatedMapResult DifferentiateMap(
      const Eigen::MatrixBase<DirectionDerived> &direction,
      const std::vector<double> &parameters);

  /**
   * Approximate mapping (image plane -> bearing vectors). It is used in
   * Unmap as an initialization of the optimization.
   * By default, returns (0 0 1)^T.
   */
  template <typename PointDerived>
  static Vector3 UnmapApproximate(const Eigen::MatrixBase<PointDerived> &point,
                                  const std::vector<double> &parameters);

  /**
   * Mapping (image plane -> bearing vectors). The result is normalized.
   */
  template <typename PointDerived>
  static Vector3 Unmap(const Eigen::MatrixBase<PointDerived> &point,
                       const std::vector<double> &parameters);

  /**
   * Mapping (image plane -> bearing vectors). The result is NOT guaranteed to
   * be normalized.
   */
  template <typename PointDerived>
  static Vector3 UnmapUnnormalized(const Eigen::MatrixBase<PointDerived> &point,
                                   const std::vector<double> &parameters);
};

// Implementation

template <typename Derived>
template <typename DirectionDerived>
DifferentiatedMapResult CameraModel<Derived>::DifferentiateMap(
    const Eigen::MatrixBase<DirectionDerived> &direction,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR3(direction);

  using Jet3 = ceres::Jet<double, 3>;

  Eigen::Matrix<Jet3, 3, 1> direction_jets = direction.template cast<Jet3>();
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
  // When the change is less than around 1/40 of a degree, we terminate
  constexpr double kMinStepSquaredNorm = 1e-6;
  constexpr double kMinDifferenceSquaredNorm = 1e-8;

  // Each step by \Delta introduces rotation of the direction by the angle
  // |\Delta|/2. We want to constraint this rotation s.t. it never happens more
  // than by \pi/4
  constexpr double kMaxStepNorm = M_PI_2;
  constexpr double kMaxStepSquaredNorm = kMaxStepNorm * kMaxStepNorm;

  ceres::HomogeneousVectorParameterization parameterization(3);

  Vector3 direction = Derived::UnmapApproximate(point, parameters).normalized();
  int it = 0;
  for (; it < kMaxIterations; ++it) {
    const auto [mapped, map_jacobian] =
        Derived::DifferentiateMap(direction, parameters);
    const Vector2 residual = point - mapped;
    if (residual.squaredNorm() < kMinDifferenceSquaredNorm) {
      break;
    }

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
    if (squared_delta_norm < kMinStepSquaredNorm) {
      break;
    }
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
