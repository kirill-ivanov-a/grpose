#ifndef GRPOSE_CAMERA_CAMERA_
#define GRPOSE_CAMERA_CAMERA_

#include <variant>

#include "camera/camera_model_pinhole.h"
#include "camera/camera_model_scaramuzza.h"

namespace grpose {

class Camera {
 public:
  using CameraModelVariants =
      std::variant<CameraModelScaramuzza, CameraModelPinhole>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Camera(int width, int height, const CameraModelVariants &camera_model);

  template <typename T>
  Eigen::Matrix<T, 3, 1> UnmapUnnormalized(const T *point) const {
    return std::visit(
        [&](const auto &cam) { return cam.template Unmap<T>(point); },
        camera_model_);
  }
  template <typename T>
  Eigen::Matrix<T, 2, 1> Map(const T *direction) const {
    return std::visit(
        [&](const auto &cam) { return cam.template Map<T>(direction); },
        camera_model_);
  }
  template <typename T>
  inline Eigen::Matrix<T, 3, 1> UnmapUnnormalized(
      const Eigen::Matrix<T, 2, 1> &point) const {
    return UnmapUnnormalized(point.data());
  }
  template <typename T>
  inline Eigen::Matrix<T, 3, 1> Unmap(
      const Eigen::Matrix<T, 2, 1> &point) const {
    Eigen::Matrix<T, 3, 1> direction = UnmapUnnormalized(point);
    return direction.normalized();
  }
  template <typename T>
  inline Eigen::Matrix<T, 2, 1> Map(
      const Eigen::Matrix<T, 3, 1> &direction) const {
    return Map(direction.data());
  }

  /**
   * Check if the direction can be mapped by this camera model. Note that some
   * directions (most notably, looking straight backward) are outside of the FoV
   * of the camera, and as the mapping polynomial was not fitted to work with
   * those, it may produce incorrect results and even map to the image! Check
   * this before mapping if you are unsure.
   */
  template <typename T>
  bool IsMappable(const Eigen::Matrix<T, 3, 1> &ray) const;

  inline int width() const { return width_; }
  inline int height() const { return height_; }

  // A mask can be used to cancel out pixels that are not providing meaningful
  // information. For example, many wide-angle cameras mounted on a vehicle
  // capture parts of the vehicle itself which negatively affects quality of
  // SLAM providing spurious matches.
  inline void set_mask(const cv::Mat1b &mask) { mask_ = mask; }
  inline const cv::Mat1b &mask() const { return mask_; };

  /**
   * Checks if a point is on image, takes the mask into account if it was
   * provided.
   * @param p a point
   * @param border Additional border inside the image, useful in SLAM
   */
  bool IsOnImage(const Vector2 &p, int border = 0) const;

  /**
   * Undistort cv::Mat image. Can work both with colored and grayscale images.
   * @tparam T cv::Vec3b if colored, unsigned char if grayscale
   * @param img the image to be undistorted
   * @param camera_matrix the target camera matrix, in which we want the image
   * to be drawn. Usually takes the form f 0 cx 0 f cy 0 0  1
   * @return the image as if it was shot with the `cameraMatrix`
   */
  template <typename T>
  cv::Mat_<T> Undistort(const cv::Mat_<T> &img,
                        const Matrix33 &camera_matrix) const {
    Matrix33 Kinv = camera_matrix.inverse();
    cv::Mat result = cv::Mat::zeros(img.rows, img.cols, img.type());

    for (int y = 0; y < result.rows; ++y)
      for (int x = 0; x < result.cols; ++x) {
        Vector3 pnt(static_cast<double>(x), static_cast<double>(y), 1.);
        Vector2 orig_pix = Map((Kinv * pnt).eval());
        int orig_x = orig_pix[0], orig_y = orig_pix[1];
        if (orig_x >= 0 && orig_x < result.cols && orig_y >= 0 &&
            orig_y < result.rows)
          result.at<T>(y, x) = img(orig_y, orig_x);
      }
    return result;
  }

 private:
  CameraModelVariants camera_model_;
  cv::Mat1b mask_;
  int width_, height_;
};

namespace camera_internal {

// Adapted from
// http://blog.abuksigun.com/2018/09/the-simplest-way-to-check-method-exists.html
// This checks if a given class CameraModelT has a method
// CameraModelT::isMappable<ValueT>
template <typename CameraModelT, typename ValueT>
static constexpr bool HasIsMappable(...) {
  return false;
}
template <typename CameraModelT, typename ValueT>
static constexpr bool HasIsMappable(
    int, decltype(std::declval<CameraModelT>().IsMappable(
             Eigen::Matrix<ValueT, 3, 1>())) * = {}) {
  return true;
}

}  // namespace camera_internal

template <typename T>
bool Camera::IsMappable(const Eigen::Matrix<T, 3, 1> &ray) const {
  return std::visit(
      [&](const auto &cam) {
        if constexpr (camera_internal::HasIsMappable<decltype(cam), T>(0)) {
          return cam.IsMappable(ray);
        } else {
          return true;
        }
      },
      camera_model_);
}

}  // namespace grpose

#endif