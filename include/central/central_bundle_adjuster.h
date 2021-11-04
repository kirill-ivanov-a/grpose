#ifndef GRPOSE_CENTRAL_CENTRAL_BUNDLE_ADJUSTER_
#define GRPOSE_CENTRAL_CENTRAL_BUNDLE_ADJUSTER_

#include <ceres/ceres.h>

#include "camera/camera.h"
#include "central/central_point2d_correspondences.h"

namespace grpose {

struct CentralBundleAdjusterSettings {
  static constexpr int kResidualSize = 4;

  enum LossType {
    kSquaredLoss,
    kHuberLoss,
    kCauchyLoss
  } loss_type = kSquaredLoss;

  double pixel_outlier_threshold = 1.0;

  bool fix_points = false;
  bool fix_pose = false;
  ceres::Solver::Options solver_options;
};

class CentralBundleAdjuster {
 public:
  struct RefinementResults {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ceres::Solver::Summary solver_summary;

    SE3 frame1_from_frame2;
    // Every row corresponds to a point
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> points;
    // Every row corresponds to a residual
    Eigen::Matrix<double, Eigen::Dynamic,
                  CentralBundleAdjusterSettings::kResidualSize, Eigen::RowMajor>
        residuals;
  };

  CentralBundleAdjuster(const CentralBundleAdjusterSettings& settings);

  RefinementResults Refine(
      const SE3& frame1_from_frame2_estimate, const Camera& camera1,
      const Camera& camera2,
      const CentralPoint2dCorrespondences& correspondences) const;

 private:
  CentralBundleAdjusterSettings settings_;
};

class ProjectionResidual {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ProjectionResidual(const Camera* camera_1, const Camera* camera_2,
                     const Vector2& point_1, const Vector2& point_2);

  template <typename T>
  bool operator()(const T* const frame1_from_frame2_rotation_ptr,
                  const T* const frame1_from_frame2_translation_ptr,
                  const T* const point3d_ptr, T* residual_ptr) const;

 private:
  const Camera* camera1_;
  const Camera* camera2_;
  const Vector2 point1_, point2_;
};

// Implementation

template <typename T>
bool ProjectionResidual::operator()(
    const T* const frame1_from_frame2_rotation_ptr,
    const T* const frame1_from_frame2_translation_ptr,
    const T* const point3d_ptr, T* residual_ptr) const {
  using SO3t = Sophus::SO3<T>;
  using Vector3t = Eigen::Matrix<T, 3, 1>;
  using Vector4t = Eigen::Matrix<T, 4, 1>;

  const Eigen::Map<const SO3t> R(frame1_from_frame2_rotation_ptr);
  const Eigen::Map<const Vector3t> t(frame1_from_frame2_translation_ptr);
  const Eigen::Map<const Vector3t> point3d(point3d_ptr);
  Eigen::Map<Vector4t> residual(residual_ptr);
  residual.template head<2>() =
      point1_.template cast<T>() - camera1_->Map(point3d);
  residual.template tail<2>() =
      point2_.template cast<T>() - camera2_->Map(R.inverse() * (point3d - t));

  return true;
}

}  // namespace grpose

#endif
