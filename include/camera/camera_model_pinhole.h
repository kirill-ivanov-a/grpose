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
   * @tparam T double or ceres::Jet
   * @param direction 3 coordinates of a direction in the camera frame (norm not
   * important)
   * @return 2D-point on the image
   */
  template <typename T>
  static Eigen::Matrix<T, 2, 1> Map(const Eigen::Matrix<T, 3, 1> &direction,
                                    const std::vector<double> &parameters);

  static Vector3 Unmap(const Vector2 &point,
                       const std::vector<double> &parameters);


 private:
  static constexpr int fx_id_ = 0, fy_id_ = 1;
  static constexpr int px_id_ = 2, py_id_ = 3;
  static constexpr int skew_id_ = 4;
};

template <typename T>
Eigen::Matrix<T, 2, 1> CameraModelPinhole::Map(
    const Eigen::Matrix<T, 3, 1> &direction,
    const std::vector<double> &parameters) {
  const double fx(parameters[fx_id_]), fy(parameters[fy_id_]);
  const double px(parameters[px_id_]), py(parameters[py_id_]);
  const double skew(parameters[skew_id_]);

  Eigen::Matrix<T, 2, 1> point(
      (fx * direction[0] + skew * direction[1]) / direction[2] + px,
      fy * direction[1] / direction[2] + py);
  return point;
}

}  // namespace grpose

#endif