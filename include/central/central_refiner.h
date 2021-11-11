#ifndef GRPOSE_CENTRAL_CENTRAL_REFINER_
#define GRPOSE_CENTRAL_CENTRAL_REFINER_

#include <ceres/ceres.h>

#include "camera/camera.h"
#include "central/central_point2d_correspondences.h"

namespace grpose {

struct CentralRefinerSettings {
  enum RefinementType {
    kSymmetricReprojection,
    kDotProduct,
    kSampsonOnPlane,
    kSampson3d,
  } refinement_type = kSampson3d;

  enum LossType {
    kSquaredLoss,
    kHuberLoss,
    kCauchyLoss
  } loss_type = kSquaredLoss;

  // Triangulation of correspondences using the initial estimate of the epipolar
  // geometry can provide a better linearization point for Sampson Error.
  // Settings this parameter to anything other than kNoTriangulation only makes
  // a difference for Sampson Error-like refinement_type.
  enum PreTriangulationType {
    kNoTriangulation,
    kTriangulation3d
  } pre_triangulation_type = kNoTriangulation;

  double pixel_outlier_threshold = 1.0;

  ceres::Solver::Options solver_options;
};

CentralRefinerSettings::RefinementType ToRefinementType(
    const std::string& name);

class CentralRefiner {
 public:
  using ResidualMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using ReLinearizationAngleMatrix =
      Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

  struct RefinementSummary {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ceres::Solver::Summary solver_summary;
    // Each row represents one residual vector
    ResidualMatrix residuals;
    ReLinearizationAngleMatrix re_linearization_angles;
  };

  CentralRefiner(const CentralRefinerSettings& settings);

  SE3 Refine(const SE3& frame1_from_frame2_estimate, const Camera& camera1,
             const Camera& camera2,
             const CentralPoint2dCorrespondences& correspondences,
             RefinementSummary& summary) const;

  ResidualMatrix CalculateResiduals(
      const SE3& frame1_from_frame2, const Camera& camera1,
      const Camera& camera2,
      const CentralPoint2dCorrespondences& correspondences) const;

 private:
  // NOTE: the user is responsible for the lifetime of these!!
  static std::vector<ceres::CostFunction*> GetCostFunctions(
      const SE3& frame1_from_frame2_estimate, const Camera& camera1,
      const Camera& camera2,
      const CentralPoint2dCorrespondences& correspondences,
      const CentralRefinerSettings& settings,
      ReLinearizationAngleMatrix& re_linearization_angles);
  static ResidualMatrix EvaluateCostFunctions(
      const SE3& frame1_from_frame2,
      const std::vector<ceres::CostFunction*> residual_functions);

  CentralRefinerSettings settings_;
};

class SymmetricReprojectionResidual {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kResidualSize = 4;

  SymmetricReprojectionResidual(const Camera* camera1, const Camera* camera2,
                                const Vector2& point1, const Vector2& point2);

  template <typename T>
  bool operator()(const T* const frame1_from_frame2_rotation_ptr,
                  const T* const frame1_from_frame2_translation_ptr,
                  T* residual_ptr) const;

 private:
  const Camera* camera1_;
  const Camera* camera2_;
  const Vector2 point1_, point2_;
  const Vector3 direction1_, direction2_;
};

class DotProductResidual {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kResidualSize = 2;

  DotProductResidual(const Vector3& direction1, const Vector3& direction2);

  template <typename T>
  bool operator()(const T* const frame1_from_frame2_rotation_ptr,
                  const T* const frame1_from_frame2_translation_ptr,
                  T* residual_ptr) const;

 private:
  Vector3 direction1_, direction2_;
};

class Sampson3dResidual {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int kResidualSize = 4;

  Sampson3dResidual(const Camera* camera1, const Camera* camera2,
                    const Vector2& point1, const Vector2& point2,
                    const Vector3* linearization_direction1 = nullptr,
                    const Vector3* linearization_direction2 = nullptr);

  template <typename T>
  bool operator()(const T* const frame1_from_frame2_rotation_ptr,
                  const T* const frame1_from_frame2_translation_ptr,
                  T* residual_ptr) const;

 private:
  const Camera* camera1_;
  const Camera* camera2_;
  const Vector2 point1_, point2_;
  const Vector2 linearization_point1_, linearization_point2_;
  Vector4 delta_point_;
  const Vector3 direction1_, direction2_;
  const Vector3 linearization_direction1_, linearization_direction2_;
  const Matrix32 J_left1, J_left2;
};

// Implementation

template <typename T>
bool SymmetricReprojectionResidual::operator()(
    const T* const frame1_from_frame2_rotation_ptr,
    const T* const frame1_from_frame2_translation_ptr, T* residual_ptr) const {
  using SO3t = Sophus::SO3<T>;
  using Vector3t = Eigen::Matrix<T, 3, 1>;
  using Vector4t = Eigen::Matrix<T, 4, 1>;

  const Eigen::Map<const SO3t> R(frame1_from_frame2_rotation_ptr);
  const Eigen::Map<const Vector3t> t(frame1_from_frame2_translation_ptr);
  const Vector3t d1 = direction1_.template cast<T>();
  const Vector3t d2 = direction2_.template cast<T>();
  const Vector3t d1_plane =
      R.inverse() * (direction1_.template cast<T>().cross(t));
  const Vector3t d2_projected =
      d2 - d2.dot(d1_plane) / d1_plane.squaredNorm() * d1_plane;
  const Vector3t d2_plane = t.cross(R * d2);
  const Vector3t d1_projected =
      d1 - d1.dot(d2_plane) / d2_plane.squaredNorm() * d2_plane;
  Eigen::Map<Vector4t> residual(residual_ptr);
  residual.template head<2>() =
      point1_.template cast<T>() - camera1_->Map(d1_projected);
  residual.template tail<2>() =
      point2_.template cast<T>() - camera2_->Map(d2_projected);
  return true;
}

template <typename T>
bool DotProductResidual::operator()(
    const T* const frame1_from_frame2_rotation_ptr,
    const T* const frame1_from_frame2_translation_ptr, T* residual_ptr) const {
  using SO3t = Sophus::SO3<T>;
  using Vector2t = Eigen::Matrix<T, 2, 1>;
  using Vector3t = Eigen::Matrix<T, 3, 1>;

  const Eigen::Map<const SO3t> R(frame1_from_frame2_rotation_ptr);
  const Eigen::Map<const Vector3t> t(frame1_from_frame2_translation_ptr);
  const Vector3t d1 = direction1_.template cast<T>();
  const Vector3t d2 = direction2_.template cast<T>();
  const Vector3t d1_plane =
      R.inverse() * (direction1_.template cast<T>().cross(t));
  const Vector3t d2_plane = t.cross(R * d2);

  residual_ptr[0] = d1.dot(d2_plane) / d2_plane.norm();
  residual_ptr[1] = d2.dot(d1_plane) / d1_plane.norm();
  return true;
}

template <typename T>
bool Sampson3dResidual::operator()(
    const T* const frame1_from_frame2_rotation_ptr,
    const T* const frame1_from_frame2_translation_ptr, T* residual_ptr) const {
  using SO3t = Sophus::SO3<T>;
  using Vector3t = Eigen::Matrix<T, 3, 1>;
  using Vector3t = Eigen::Matrix<T, 3, 1>;
  using RowVector4t = Eigen::Matrix<T, 1, 4>;
  using Matrix33t = Eigen::Matrix<T, 3, 3>;

  const Eigen::Map<const SO3t> R(frame1_from_frame2_rotation_ptr);
  const Eigen::Map<const Vector3t> t(frame1_from_frame2_translation_ptr);
  const Vector3t d1 = direction1_.template cast<T>();
  const Vector3t d2 = direction2_.template cast<T>();

  const Matrix33t E = (SO3t::hat(t) * R.matrix()).transpose();
  RowVector4t J;
  J << linearization_direction2_.transpose() * E * J_left1,
      linearization_direction1_.transpose() * E.transpose() * J_left2;

  Eigen::Map<RowVector4t> residual(residual_ptr);
  residual =
      (linearization_direction2_.transpose() * E * linearization_direction1_ +
       J * delta_point_) /
      J.squaredNorm() * J;

  return true;
}

}  // namespace grpose

#endif