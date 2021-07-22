#ifndef GRPOSE_CAMERA_CAMERAMODELPINHOLE_
#define GRPOSE_CAMERA_CAMERAMODELPINHOLE_

#include "util.h"

namespace grpose {

class CameraModelPinhole {
 public:
  /**
   * Generic unmapping (image plane -> bearing vector). Was used to interface
   * with ceres's automatic differentiation, can be used again later.
   * @tparam T float, double or ceres::Jet
   * @param point 2 coordinates of a point on the image
   * @return 3D-direction in the camera frame, **UNNORMALIZED**
   */
  template <typename T>
  Eigen::Matrix<T, 3, 1> unmap(const T *point) const {
    using Vector3t = Eigen::Matrix<T, 3, 1>;
    using Vector2t = Eigen::Matrix<T, 2, 1>;
    Eigen::Map<const Vector2t> ptMap(point);
    Vector2t pt = ptMap;
    Vector3t direction(T(0.0), T(0.0), T(1.0));
    direction[1] = (pt[1] - principalPoint_[1]) / fy_;
    direction[0] = (pt[0] - skew_ * direction[1] - principalPoint_[0]) / fx_;

    return direction;
  }

  /**
   * Generic mapping (bearing vector -> image plane). Was used to interface with
   * ceres's automatic differentiation, can be used again later.
   * @tparam T float, double or ceres::Jet
   * @param direction 3 coordinates of a direction in the camera frame (norm not
   * important)
   * @return 2D-point on the image
   *
   * WARNING: as for now, it can only be compiled with T=double because of the
   * temporary incorporation of the Unified camera model.
   */
  template <typename T>
  Eigen::Matrix<T, 2, 1> map(const T *direction) const {
    typedef Eigen::Matrix<T, 3, 1> Vector3t;
    typedef Eigen::Matrix<T, 2, 1> Vector2t;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;

    Vector2t res(
        (T(fx_) * direction[0] + T(skew_) * direction[1]) / direction[2] +
            T(principalPoint_[0]),
        res[1] = T(fy_) * direction[1] / direction[2] + T(principalPoint_[1]));
    return res;
  }

 private:
  double fx_, fy_;
  Vector2 principalPoint_;
  double skew_;
};

}  // namespace grpose

#endif