#include "central/approximate_central_solution_sampler.h"
#include "central/central_refiner.h"

namespace grpose {

namespace {

using ErrorType = ApproximateCentralSolutionSamplerSettings::ErrorType;

double Error(const ErrorType error_type, const Camera& camera1,
             const Camera& camera2, const Vector2& point1,
             const Vector2& point2, const SE3& frame1_from_frame2) {
  const double* rotation_ptr = frame1_from_frame2.so3().data();
  const double* translation_ptr = frame1_from_frame2.translation().data();

  switch (error_type) {
    case ErrorType::kReprojectionError2D: {
      SymmetricReprojectionResidual functor(&camera1, &camera2, point1, point2);
      Vector4 residual;
      functor(rotation_ptr, translation_ptr, residual.data());
      // sum of pixel reprojection errors on each of 2 images
      return residual.head<2>().norm() + residual.tail<2>().norm();
    }
    case ErrorType::kSumOfAngles: {
      DotProductResidual functor(camera1.Unmap(point1), camera2.Unmap(point2));
      Vector2 residual;
      functor(rotation_ptr, translation_ptr, residual.data());
      return std::asin(std::clamp(residual[0], -1.0, 1.0)) +
             std::asin(std::clamp(residual[1], -1.0, 1.0));
    }
    case ErrorType::kSampson3d: {
      Sampson3dResidual functor(&camera1, &camera2, point1, point2);
      Vector4 residual;
      functor(rotation_ptr, translation_ptr, residual.data());
      return residual.head<2>().norm() + residual.tail<2>().norm();
    }
  }

  return 0.0;
}

CentralPoint2dCorrespondences ComputeInlierCorrespondences(
    const Camera& camera1, const Camera& camera2,
    const CentralPoint2dCorrespondences& point_correspondences,
    const SE3& frame1_from_frame2_true,
    const ApproximateCentralSolutionSamplerSettings& settings) {
  CentralPoint2dCorrespondences inlier_correspondences;

  for (int i = 0; i < point_correspondences.Size(); ++i) {
    Vector2 point1 = point_correspondences.point(0, i);
    Vector2 point2 = point_correspondences.point(1, i);
    double error = Error(settings.error_type, camera1, camera2, point1, point2,
                         frame1_from_frame2_true);
    if (error <= settings.threshold) inlier_correspondences.Add(point1, point2);
  }

  return inlier_correspondences;
}

}  // namespace

ApproximateCentralSolutionSamplerSettings::ErrorType ToErrorType(
    const std::string &name) {
  using ErrorType = ApproximateCentralSolutionSamplerSettings::ErrorType;
  if (name == "reproj_2d")
    return ErrorType::kReprojectionError2D;
  else if (name == "angular")
    return ErrorType::kSumOfAngles;
  else if (name == "sampson_3d")
    return ErrorType::kSampson3d;
  else
    LOG(ERROR) << "Unknown error type " << name << std::endl;

  return ErrorType::kReprojectionError2D;
}


ApproximateCentralSolutionSampler::ApproximateCentralSolutionSampler(
    const Camera& camera1, const Camera& camera2,
    const CentralPoint2dCorrespondences& correspondences,
    const SE3& frame1_from_frame2_true,
    const ApproximateCentralSolutionSamplerSettings& settings)
    : inlier_point_correspondences_(
          ComputeInlierCorrespondences(camera1, camera2, correspondences,
                                       frame1_from_frame2_true, settings)),
      inlier_bv_correspondences_(
          inlier_point_correspondences_.ToCentralBearingVectorCorrespondences(
              camera1, camera2)),
      settings_(settings),
      minimal_solver_(inlier_bv_correspondences_, settings_.algorithm) {
  all_indices_.reserve(inlier_bv_correspondences_.Size());
  for (int i = 0; i < inlier_bv_correspondences_.Size(); ++i)
    all_indices_.push_back(i);
}

}  // namespace grpose
