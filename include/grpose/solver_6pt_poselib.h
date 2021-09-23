#ifndef GRPOSE_GRPOSE_SOLVER_6PT_POSELIB_
#define GRPOSE_GRPOSE_SOLVER_6PT_POSELIB_

#include "grpose/bearing_vector_correspondences.h"
#include "grpose/non_central_relative_pose_minimal_solver.h"

namespace grpose {

class Solver6ptPoselib : public NonCentralRelativePoseMinimalSolver {
 public:
  Solver6ptPoselib(
      const std::shared_ptr<BearingVectorCorrespondences> &correspondences,
      const StdVectorA<SE3> &body_from_camera);

  int MinSampleSize() const override;
  bool SolveTimed(const std::vector<int> &correspondence_indices,
                  StdVectorA<SE3> &frame1_from_frame2,
                  double &time_in_seconds) const override;

 private:
  std::shared_ptr<BearingVectorCorrespondences> correspondences_;
  StdVectorA<SE3> body_from_camera_;
};

}  // namespace grpose

#endif