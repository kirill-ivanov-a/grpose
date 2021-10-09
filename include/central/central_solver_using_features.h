#ifndef GRPOSE_CENTRAL_CENTRAL_SOLVER_USING_FEATURES_
#define GRPOSE_CENTRAL_CENTRAL_SOLVER_USING_FEATURES_

#include <optional>

#include "camera/camera.h"
#include "central/central_solver_bvc.h"
#include "features/feature_detector_and_matcher.h"

namespace grpose {

class CentralSolverUsingFeatures {
 public:
  struct SolveInfo {
    double feature_time;
    // If the solver is implemented using feature matching, then statistics of
    // post-matching estimation operations are stored here
    CentralSolverBvc::SolveInfo solve_bvc_info;
  };

  CentralSolverUsingFeatures(
      const Camera &camera1, const Camera &camera2,
      const FeatureDetectorAndMatcher &matcher,
      const std::shared_ptr<CentralSolverBvc> &solver_bvc);

  bool Solve(const cv::Mat3b &frame1, const cv::Mat3b &frame2,
             SE3 &frame1_from_frame2, SolveInfo *solve_info = nullptr) const;

 private:
  Camera cameras_[2];
  FeatureDetectorAndMatcher matcher_;
  std::shared_ptr<CentralSolverBvc> solver_bvc_;
};

}  // namespace grpose

#endif
