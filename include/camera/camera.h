#ifndef GRPOSE_CAMERA_CAMERA_
#define GRPOSE_CAMERA_CAMERA_

#include <variant>

#include "camera/camera_model_generic.h"

namespace grpose {

class Camera {
 public:
  Camera(int width, int height, CameraModelId model_id,
         const std::vector<double> &parameters);

  template <typename PointDerived>
  inline Vector3 Unmap(const Eigen::MatrixBase<PointDerived> &point) const;

  template <typename DirectionDerived>
  inline Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Map(
      const Eigen::MatrixBase<DirectionDerived> &direction) const;

  /**
   * Check if the direction can be mapped by this camera model. Note that some
   * directions (most notably, looking straight backward) are outside of the FoV
   * of the camera, and as the mapping polynomial was not fitted to work with
   * those, it may produce incorrect results and even map to the image! Check
   * this before mapping if you are unsure.
   */
  template <typename DirectionDerived>
  bool IsMappable(const Eigen::MatrixBase<DirectionDerived> &direction) const;

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
  CameraModelId model_id_;
  std::vector<double> parameters_;
  cv::Mat1b mask_;
  int width_, height_;
};

// Implementation

template <typename PointDerived>
Vector3 Camera::Unmap(const Eigen::MatrixBase<PointDerived> &point) const {
  return CameraModelUnmap(model_id_, parameters_, point);
}

template <typename DirectionDerived>
Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Camera::Map(
    const Eigen::MatrixBase<DirectionDerived> &direction) const {
  return CameraModelMap(model_id_, parameters_, direction);
}

template <typename DirectionDerived>
bool Camera::IsMappable(
    const Eigen::MatrixBase<DirectionDerived> &direction) const {
  return CameraModelIsMappable(model_id_, parameters_, direction);
}

}  // namespace grpose

#endif