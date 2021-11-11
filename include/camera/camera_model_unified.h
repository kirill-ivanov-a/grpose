#ifndef GRPOSE_CAMERA_CAMERA_MODEL_UNIFIED_
#define GRPOSE_CAMERA_CAMERA_MODEL_UNIFIED_

#include "camera/camera_model.h"

namespace grpose {

/**
 * Camera model from the paper
 * Mei, Christopher, and Patrick Rives. "Single view point omnidirectional
 * camera calibration from planar grids." Proceedings 2007 IEEE International
 * Conference on Robotics and Automation. IEEE, 2007.
 */
class CameraModelUnified : public CameraModel<CameraModelUnified> {
 public:
  static constexpr CameraModelId kModelId = CameraModelId::kUnified;
  static constexpr int kNumParameters = 9;

  template <typename DirectionDerived>
  static Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Map(
      const Eigen::MatrixBase<DirectionDerived> &direction,
      const std::vector<double> &parameters);

  template <typename PointDerived>
  static Vector3 UnmapApproximate(const Eigen::MatrixBase<PointDerived> &point,
                                  const std::vector<double> &parameters);

 private:
  static constexpr int fx_id_ = 0, fy_id_ = 1;
  static constexpr int cx_id_ = 2, cy_id_ = 3;
  static constexpr int k1_id_ = 4, k2_id_ = 5;
  static constexpr int p1_id_ = 6, p2_id_ = 7;
  static constexpr int xi_id_ = 8;
};

template <typename DirectionDerived>
Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> CameraModelUnified::Map(
    const Eigen::MatrixBase<DirectionDerived> &direction,
    const std::vector<double> &parameters) {
  using Scalar = typename DirectionDerived::Scalar;
  using Vector2t = Eigen::Matrix<Scalar, 2, 1>;

  const double fx = parameters[fx_id_], fy = parameters[fy_id_];
  const double cx = parameters[cx_id_], cy = parameters[cy_id_];
  const double k1 = parameters[k1_id_], k2 = parameters[k2_id_];
  const double p1 = parameters[p1_id_], p2 = parameters[p2_id_];
  const double xi = parameters[xi_id_];

  // Projection onto the normalized plane.
  const Vector2t raw_uv =
      direction.template head<2>() / (direction[2] + direction.norm() * xi);

  // Distortion
  const Scalar raw_u = raw_uv[0], raw_v = raw_uv[1];
  const Scalar raw_u2 = raw_u * raw_u, raw_v2 = raw_v * raw_v;
  const Scalar raw_u_times_v = raw_u * raw_v;
  const Scalar raw_uv_norm2 = raw_u2 + raw_v2;
  const Scalar raw_uv_norm4 = raw_uv_norm2 * raw_uv_norm2;
  const Scalar radial = k1 * raw_uv_norm2 + k2 * raw_uv_norm4;
  const Scalar du = radial * raw_u + 2.0 * p1 * raw_u_times_v +
                    p2 * (raw_uv_norm2 + 2.0 * raw_u2);
  const Scalar dv = radial * raw_v + 2.0 * p2 * raw_u_times_v +
                    p1 * (raw_uv_norm2 + 2.0 * raw_v2);
  const Scalar u = raw_u + du, v = raw_v + dv;

  // Pinhole
  return Vector2t(fx * u + cx, fy * v + cy);
}

template <typename PointDerived>
Vector3 CameraModelUnified::UnmapApproximate(
    const Eigen::MatrixBase<PointDerived> &point,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  // We omit undistortion here, leaving the generic iterative unmapping to deal
  // with it.
  const double fx = parameters[fx_id_], fy = parameters[fy_id_];
  const double cx = parameters[cx_id_], cy = parameters[cy_id_];
  const double xi = parameters[xi_id_];

  const Vector2 uv((point[0] - cx) / fx, (point[1] - cy) / fy);
  const double uv_norm2 = uv.squaredNorm();
  double discriminant_4 = 1.0 + (1.0 - xi * xi) * uv_norm2;

  // Some points can't be unmapped. The best we can do for them is providing the
  // closest mappable point's direction.
  if (discriminant_4 < 0.0) discriminant_4 = 0.0;

  const double z = (-xi * uv_norm2 + sqrt(discriminant_4)) / (1 + uv_norm2);

  Vector3 direction(0.0, 0.0, z);
  direction.head<2>() = (z + xi) * uv;
  double direction_norm = direction.norm();
  CHECK_NEAR(direction_norm, 1.0, 1e-4);
  return direction / direction_norm;
}

}  // namespace grpose

#endif