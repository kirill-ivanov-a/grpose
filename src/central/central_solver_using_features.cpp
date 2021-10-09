#include "central/central_solver_using_features.h"

namespace grpose {

CentralSolverUsingFeatures::CentralSolverUsingFeatures(
    const Camera& camera1, const Camera& camera2,
    const FeatureDetectorAndMatcher& matcher,
    const std::shared_ptr<CentralSolverBvc>& solver_bvc)
    : cameras_{camera1, camera2}, matcher_(matcher), solver_bvc_(solver_bvc) {}

bool CentralSolverUsingFeatures::Solve(const cv::Mat3b& frame1,
                                       const cv::Mat3b& frame2,
                                       SE3& frame1_from_frame2,
                                       SolveInfo* solve_info) const {
  CentralSolverBvc::SolveInfo* solver_bvc_info =
      solve_info ? &solve_info->solve_bvc_info : nullptr;
  const CentralPoint2dCorrespondences point_correspondences =
      matcher_.GetCorrespondences(frame1, frame2, &cameras_[0].mask(),
                                  &cameras_[1].mask());
  const CentralBearingVectorCorrespondences bvc =
      point_correspondences.ToCentralBearingVectorCorrespondences(cameras_[0],
                                                                  cameras_[1]);
  return solver_bvc_->Solve(bvc, frame1_from_frame2, solver_bvc_info);
}

}  // namespace grpose