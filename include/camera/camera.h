#ifndef GRPOSE_CAMERA_CAMERA_
#define GRPOSE_CAMERA_CAMERA_

#include <variant>

#include "camera/camera_model_generic.h"
#include "colmap_camera_models.h"

namespace grpose {

/**
 * A convenience class that stores camera model parameters, camera mask and
 * provides mapping/unmapping functionality.
 */
class Camera {
 public:
  static Camera ColmapCamera(int width, int height, int colmap_model_id,
                             const std::vector<double> &parameters);

  Camera(int width, int height, CameraModelId model_id,
         const std::vector<double> &parameters);

  /**
   * Mapping (image plane -> bearing vectors).
   *
   * @param point point on the image
   * PointDerived::Scalar should be either double or ceres::Jet.
   * @return Normalized direction in the camera frame.
   */
  template <typename PointDerived>
  inline Vector3 Unmap(const Eigen::MatrixBase<PointDerived> &point) const;

  /**
   * Mapping (bearing vectors -> image plane). The result is normalized.
   *
   * @param direction direction in the camera frame (norm not
   * important), DirectionDerived::Scalar should be either double or ceres::Jet
   * @return 2D-point on the image.
   */
  template <typename DirectionDerived>
  inline Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Map(
      const Eigen::MatrixBase<DirectionDerived> &direction) const;

  /**
   * Mapping (image plane -> bearing vectors). The result is NOT guaranteed to
   * be normalized.
   *
   * @param point point in the image plane.
   * @return 2D-point on the image.
   */
  template <typename PointDerived>
  inline Vector3 UnmapUnnormalized(
      const Eigen::MatrixBase<PointDerived> &point) const;

  /**
   * Differentiate the mapping (bearing vectors -> image plane).
   */
  template <typename DirectionDerived>
  inline DifferentiatedMapResult DifferentiateMap(
      const Eigen::MatrixBase<DirectionDerived> &direction) const;

  inline int width() const { return width_; }
  inline int height() const { return height_; }

  // A mask can be used to cancel out pixels that are not providing meaningful
  // information. For example, many wide-angle cameras mounted on a vehicle
  // capture parts of the vehicle itself which negatively affects quality of
  // SLAM providing spurious matches.
  inline void set_mask(const cv::Mat1b &mask) { mask_ = mask; }
  inline const cv::Mat1b &mask() const { return mask_; };

  inline const std::vector<double> &parameters() const { return parameters_; }

  /**
   * Checks if a point is on image, takes the mask into account if it was
   * provided.
   * @param p a point on the image plane.
   * @param border Additional border inside the image, useful in SLAM.
   */
  bool IsOnImage(const Vector2 &p, int border = 0) const;

  /**
   * Undistort cv::Mat image. Can work both with colored and grayscale images.
   * @tparam T cv::Vec3b if colored, unsigned char if grayscale.
   * @param img the image to be undistorted.
   * @param camera_matrix the target camera matrix, in which we want the image
   * to be drawn. Usually takes the form [f 0 cx; 0 f cy; 0 0 1].
   * @return the image as if it was shot with the `cameraMatrix`.
   */
  template <typename T>
  cv::Mat_<T> Undistort(const cv::Mat_<T> &img,
                        const Matrix33 &camera_matrix) const;

 private:
  static constexpr int kNotColmapCameraId = -2;

  struct ColmapConstructor {};

  Camera(ColmapConstructor, int width, int height, int colmap_model_id,
         const std::vector<double> &parameters);

  int colmap_model_id_ = kNotColmapCameraId;
  CameraModelId model_id_ = CameraModelId::kInvalid;
  std::vector<double> parameters_;
  cv::Mat1b mask_;
  int width_, height_;
};

// Implementation

template <typename PointDerived>
Vector3 Camera::Unmap(const Eigen::MatrixBase<PointDerived> &point) const {
  if (colmap_model_id_ == kNotColmapCameraId) {
    return CameraModelUnmap(model_id_, parameters_, point);
  } else {
    using Scalar = typename PointDerived::Scalar;
    Eigen::Matrix<Scalar, 3, 1> result(0, 0, 1);
    colmap::CameraModelImageToWorld<double, Scalar>(
        colmap_model_id_, parameters_, point[0], point[1], &result[0],
        &result[1]);
    return result.normalized();
  }
}

template <typename DirectionDerived>
Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> Camera::Map(
    const Eigen::MatrixBase<DirectionDerived> &direction) const {
  if (colmap_model_id_ == kNotColmapCameraId) {
    return CameraModelMap(model_id_, parameters_, direction);
  } else {
    using Scalar = typename DirectionDerived::Scalar;
    Eigen::Matrix<Scalar, 2, 1> result(0.0, 0.0);
    colmap::CameraModelWorldToImage(colmap_model_id_, parameters_,
                                    direction[0], direction[1], direction[2],
                                    &result[0], &result[1]);
    return result;
  }
}

template <typename PointDerived>
Vector3 Camera::UnmapUnnormalized(
    const Eigen::MatrixBase<PointDerived> &point) const {
  if (colmap_model_id_ == kNotColmapCameraId) {
    return CameraModelUnmapUnnormalized(model_id_, parameters_, point);
  } else {
    using Scalar = typename PointDerived::Scalar;
    Eigen::Matrix<Scalar, 3, 1> result(0, 0, 1);
    colmap::CameraModelImageToWorld<double, Scalar>(
        colmap_model_id_, parameters_, point[0], point[1], &result[0],
        &result[1]);
    return result;
  }
}

template <typename DirectionDerived>
DifferentiatedMapResult Camera::DifferentiateMap(
    const Eigen::MatrixBase<DirectionDerived> &direction) const {
  if (colmap_model_id_ == kNotColmapCameraId) {
    return CameraModelDifferentiateMap(model_id_, parameters_, direction);
  } else {
    using Jet3 = ceres::Jet<double, 3>;

    Eigen::Matrix<Jet3, 3, 1> direction_jets = direction.template cast<Jet3>();
    for (int i = 0; i < 3; ++i) direction_jets[i].v[i] = 1.0;

    const Eigen::Matrix<Jet3, 2, 1> point_jet = Map(direction_jets);

    DifferentiatedMapResult result;
    result.point = Vector2(point_jet[0].a, point_jet[1].a);
    result.jacobian << point_jet[0].v.transpose(), point_jet[1].v.transpose();
    return result;
  }
}

template <typename T>
cv::Mat_<T> Camera::Undistort(const cv::Mat_<T> &img,
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

}  // namespace grpose

#endif