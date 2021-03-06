#ifndef GRPOSE_CAMERA_CAMERA_MODEL_SCARAMUZZA_
#define GRPOSE_CAMERA_CAMERA_MODEL_SCARAMUZZA_

#include "camera/camera_model.h"

namespace grpose {

template <int kPolynomialUnmapDegree, int kPolynomialMapDegree>
class CameraModelScaramuzza;

// MultiFoV / MultiCam dataset camera models have these degrees
using CameraModelMultiFov = CameraModelScaramuzza<4, 11>;

template <int kPolynomialUnmapDegree, int kPolynomialMapDegree>
struct ScaramuzzaModelId;

template <>
struct ScaramuzzaModelId<4, 11> {
  static constexpr CameraModelId kModelId = CameraModelId::kMultiFov;
};

/**
 * Omnidirectional camera model as used in
 * https://sites.google.com/site/scarabotix/ocamcalib-toolbox .
 *
 * @tparam kPolynomialUnmapDegree approximate unmapping polynomial degree.
 * @tparam kPolynomialMapDegree mapping polynomial degree.
 */
template <int kPolynomialUnmapDegree, int kPolynomialMapDegree>
class CameraModelScaramuzza
    : public CameraModel<
          CameraModelScaramuzza<kPolynomialUnmapDegree, kPolynomialMapDegree>> {
 public:
  static constexpr CameraModelId kModelId =
      ScaramuzzaModelId<kPolynomialUnmapDegree, kPolynomialMapDegree>::kModelId;
  static constexpr int kNumParameters =
      kPolynomialUnmapDegree + kPolynomialMapDegree + 7;

  template <typename DirectionDerived>
  static Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Map(
      const Eigen::MatrixBase<DirectionDerived> &direction,
      const std::vector<double> &parameters);

  template <typename PointDerived>
  static Vector3 UnmapApproximate(const Eigen::MatrixBase<PointDerived> &point,
                                  const std::vector<double> &parameters);

 private:
  static constexpr int polynomial_unmap_start_ = 0;
  static constexpr int px_id_ = kPolynomialUnmapDegree + 1;
  static constexpr int py_id_ = kPolynomialUnmapDegree + 2;
  static constexpr int c_id_ = kPolynomialUnmapDegree + 3;
  static constexpr int d_id_ = kPolynomialUnmapDegree + 4;
  static constexpr int e_id_ = kPolynomialUnmapDegree + 5;
  static constexpr int polynomial_map_start_ = kPolynomialUnmapDegree + 6;
};

// Implementation

template <int kPolynomialUnmapDegree, int kPolynomialMapDegree>
template <typename DirectionDerived>
Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1>
CameraModelScaramuzza<kPolynomialUnmapDegree, kPolynomialMapDegree>::Map(
    const Eigen::MatrixBase<DirectionDerived> &direction,
    const std::vector<double> &parameters) {
  using Scalar = typename DirectionDerived::Scalar;
  using Vector2t = Eigen::Matrix<Scalar, 2, 1>;

  const double c = parameters[c_id_];
  const double d = parameters[d_id_];
  const double e = parameters[e_id_];
  const double px = parameters[px_id_];
  const double py = parameters[py_id_];

  const Scalar direction_head2_norm = direction.template head<2>().norm();
  if (direction_head2_norm <= std::numeric_limits<double>::epsilon())
    return Vector2t(Scalar(px), Scalar(py));

  const Scalar theta = atan(-direction[2] / direction_head2_norm);

  int i = polynomial_map_start_ + kPolynomialMapDegree;
  Scalar r(parameters[i--]);
  for (; i >= polynomial_map_start_; --i) {
    r = parameters[i] + theta * r;
  }

  const Vector2t raw_point =
      direction.template head<2>() / direction_head2_norm * r;

  const Vector2t point(c * raw_point[0] + d * raw_point[1] + px,
                       e * raw_point[0] + raw_point[1] + py);
  return point;
}

template <int kPolynomialUnmapDegree, int kPolynomialMapDegree>
template <typename PointDerived>
Vector3 CameraModelScaramuzza<kPolynomialUnmapDegree, kPolynomialMapDegree>::
    UnmapApproximate(const Eigen::MatrixBase<PointDerived> &point,
                     const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  const double c = parameters[c_id_];
  const double d = parameters[d_id_];
  const double e = parameters[e_id_];
  const double px = parameters[px_id_];
  const double py = parameters[py_id_];

  Matrix22 affine;
  affine << c, d, e, 1.0;
  Vector2 rectified_point = affine.inverse() * (point - Vector2(px, py));

  const double rectified_norm = rectified_point.norm();

  int i = polynomial_unmap_start_ + kPolynomialUnmapDegree;
  double z = parameters[i--];
  for (; i >= polynomial_unmap_start_; --i) {
    z = parameters[i] + z * rectified_norm;
  }
  z = -z;

  const Vector3 direction(rectified_point[0], rectified_point[1], z);
  return direction.normalized();
}

}  // namespace grpose

#endif