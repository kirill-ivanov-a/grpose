#ifndef INCLUDE_CAMERA
#define INCLUDE_CAMERA

#include "camera/CameraModelPinhole.h"
#include "camera/CameraModelScaramuzza.h"

#include <variant>

namespace grpose {

class Camera {
 public:
  using CameraModelVariants =
      std::variant<CameraModelScaramuzza, CameraModelPinhole>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Camera(int width, int height, const CameraModelVariants &cameraModel);

  template <typename T>
  Eigen::Matrix<T, 3, 1> unmapUnnormalized(const T *point) const {
    return std::visit(
        [&](const auto &cam) { return cam.template unmap<T>(point); },
        cameraModel_);
  }
  template <typename T>
  Eigen::Matrix<T, 2, 1> map(const T *direction) const {
    return std::visit(
        [&](const auto &cam) { return cam.template map<T>(direction); },
        cameraModel_);
  }
  template <typename T>
  inline Eigen::Matrix<T, 3, 1> unmapUnnormalized(
      const Eigen::Matrix<T, 2, 1> &point) const {
    return unmapUnnormalized(point.data());
  }
  template <typename T>
  inline Eigen::Matrix<T, 3, 1> unmap(
      const Eigen::Matrix<T, 2, 1> &point) const {
    Eigen::Matrix<T, 3, 1> direction = unmapUnnormalized(point);
    return direction.normalized();
  }
  template <typename T>
  inline Eigen::Matrix<T, 2, 1> map(
      const Eigen::Matrix<T, 3, 1> &direction) const {
    return map(direction.data());
  }

  /**
   * Check if the direction can be mapped by this camera model. Note that some
   * directions (most notably, looking straight backward) are outside of the FoV
   * of the camera, and as the mapping polynomial was not fitted to work with
   * those, it may produce incorrect results and even map to the image! Check
   * this before mapping if you are unsure.
   */
  template <typename T>
  bool isMappable(const Eigen::Matrix<T, 3, 1> &ray) const;

  inline int width() const { return width_; }
  inline int height() const { return height_; }

  // A mask can be used to cancel out pixels that are not providing meaningful
  // information. For example, many wide-angle cameras mounted on a vehicle
  // capture parts of the vehicle itself which negatively affects quality of
  // SLAM providing spurious matches.
  inline void setMask(const cv::Mat1b &newMask) { mask_ = newMask; }
  inline const cv::Mat1b &mask() const { return mask_; };

  /**
   * Checks if a point is on image, takes the mask into account if it was
   * provided.
   * @param p a point
   * @param border Additional border inside the image, useful in SLAM
   */
  bool isOnImage(const Vector2 &p, int border = 0) const;

  /**
   * Undistort cv::Mat image. Can work both with colored and grayscale images.
   * @tparam T cv::Vec3b if colored, unsigned char if grayscale
   * @param img the image to be undistorted
   * @param cameraMatrix the target camera matrix, in which we want the image to
   * be drawn. Usually takes the form f 0 cx 0 f cy 0 0  1
   * @return the image as if it was shot with the `cameraMatrix`
   */
  template <typename T>
  cv::Mat undistort(const cv::Mat_<T> &img,
                    const Matrix33 &cameraMatrix) const {
    Matrix33 Kinv = cameraMatrix.inverse();
    cv::Mat result = cv::Mat::zeros(img.rows, img.cols, img.type());

    for (int y = 0; y < result.rows; ++y)
      for (int x = 0; x < result.cols; ++x) {
        Vector3 pnt(static_cast<double>(x), static_cast<double>(y), 1.);
        Vector2 origPix = map((Kinv * pnt).eval());
        int origX = origPix[0], origY = origPix[1];
        if (origX >= 0 && origX < result.cols && origY >= 0 &&
            origY < result.rows)
          result.at<T>(y, x) = img(origY, origX);
      }
    return result;
  }

 private:
  CameraModelVariants cameraModel_;
  cv::Mat1b mask_;
  int width_, height_;
};

namespace {

// Adapted from
// http://blog.abuksigun.com/2018/09/the-simplest-way-to-check-method-exists.html
// This checks if a given class T has a method T::isMappable<ValueT>
template <typename CameraModelT, typename ValueT>
static constexpr bool hasIsMappable(...) {
  return false;
}
template <typename CameraModelT, typename ValueT>
static constexpr bool hasIsMappable(
    int, decltype(std::declval<CameraModelT>().isMappable(
             Eigen::Matrix<ValueT, 3, 1>())) * = {}) {
  return true;
}

}  // namespace

template <typename T>
bool Camera::isMappable(const Eigen::Matrix<T, 3, 1> &ray) const {
  return std::visit(
      [&](const auto &cam) {
        if constexpr (hasIsMappable<decltype(cam), T>(0)) {
          return cam.isMappable(ray);
        } else {
          return true;
        }
      },
      cameraModel_);
}

}  // namespace grpose

#endif