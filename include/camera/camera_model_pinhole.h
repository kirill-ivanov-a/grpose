#ifndef GRPOSE_CAMERA_CAMERAMODELPINHOLE_
#define GRPOSE_CAMERA_CAMERAMODELPINHOLE_

#include "camera/camera_model.h"
#include "util.h"

namespace grpose {

class CameraModelPinhole : public CameraModel<CameraModelPinhole> {
 public:
  static constexpr CameraModelId model_id = CameraModelId::kPinhole;

  /**
   * Generic mapping (bearing vector -> image plane).
   * @param direction 3 coordinates of a direction in the camera frame (norm not
   * important), DirectionDerived::Scalar should be either double or ceres::Jet
   * @return 2D-point on the image
   */
  template <typename DirectionDerived>
  static Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Map(
      const Eigen::MatrixBase<DirectionDerived> &direction,
      const std::vector<double> &parameters);

  template <typename PointDerived>
  static Vector3 UnmapUnnormalized(const Eigen::MatrixBase<PointDerived> &point,
                                   const std::vector<double> &parameters);
  template <typename PointDerived>
  static Vector3 Unmap(const Eigen::MatrixBase<PointDerived> &point,
                       const std::vector<double> &parameters);

 private:
  static constexpr int fx_id_ = 0, fy_id_ = 1;
  static constexpr int px_id_ = 2, py_id_ = 3;
  static constexpr int skew_id_ = 4;
};

// Implementation

template <typename DirectionDerived>
Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> CameraModelPinhole::Map(
    const Eigen::MatrixBase<DirectionDerived> &direction,
    const std::vector<double> &parameters) {
  using Scalar = typename DirectionDerived::Scalar;

  const double fx(parameters[fx_id_]), fy(parameters[fy_id_]);
  const double px(parameters[px_id_]), py(parameters[py_id_]);
  const double skew(parameters[skew_id_]);
  Eigen::Matrix<Scalar, 2, 1> point(
      (fx * direction[0] + skew * direction[1]) / direction[2] + px,
      fy * direction[1] / direction[2] + py);
  return point;
}

template <typename PointDerived>
Vector3 CameraModelPinhole::UnmapUnnormalized(
    const Eigen::MatrixBase<PointDerived> &point,
    const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  const double fx(parameters[fx_id_]), fy(parameters[fy_id_]);
  const double px(parameters[px_id_]), py(parameters[py_id_]);
  const double skew(parameters[skew_id_]);

  Vector3 direction(0.0, 0.0, 1.0);
  direction[1] = (point[1] - py) / fy;
  direction[0] = (point[0] - skew * direction[1] - px) / fx;
  return direction;
}

template <typename PointDerived>
Vector3 CameraModelPinhole::Unmap(const Eigen::MatrixBase<PointDerived> &point,
                                  const std::vector<double> &parameters) {
  GRPOSE_CHECK_IS_VECTOR2(point);

  return UnmapUnnormalized(point, parameters).normalized();
}

}  // namespace grpose

#endif