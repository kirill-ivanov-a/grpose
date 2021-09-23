#include "grpose/solver_6pt_poselib.h"

#include <iostream>

#include <PoseLib/gen_relpose_6pt.h>

namespace grpose {

Solver6ptPoselib::Solver6ptPoselib(
    const std::shared_ptr<BearingVectorCorrespondences>& correspondences,
    const StdVectorA<SE3>& body_from_camera)
    : correspondences_(correspondences), body_from_camera_(body_from_camera) {}

int Solver6ptPoselib::MinSampleSize() const { return 6; }

bool Solver6ptPoselib::Solve(const std::vector<int>& correspondence_indices,
                             StdVectorA<SE3>& frame1_from_frame2) const {
  const int kSampleSize = MinSampleSize();  // 6
  // TODO normal assert
  CHECK_EQ(correspondence_indices.size(), kSampleSize);

  std::vector<pose_lib::CameraPose> frame2_from_frame1_poselib;
  std::vector<Vector3> ray_centers[2];
  std::vector<Vector3> ray_directions[2];
  for (int fi : {0, 1}) {
    ray_centers[fi].reserve(kSampleSize);
    ray_directions[fi].reserve(kSampleSize);
  }
  for (int i : correspondence_indices) {
    for (int fi : {0, 1}) {
      const SE3& body_from_current_camera =
          body_from_camera_[correspondences_->camera_index(fi, i)];
      ray_centers[fi].push_back(body_from_current_camera.translation());
      ray_directions[fi].push_back(body_from_current_camera.so3() *
                                   correspondences_->bearing_vector(fi, i));
    }
  }
  const int number_solutions = pose_lib::gen_relpose_6pt(
      ray_centers[0], ray_directions[0], ray_centers[1], ray_directions[1],
      &frame2_from_frame1_poselib);
  CHECK_EQ(frame2_from_frame1_poselib.size(), number_solutions);

  frame1_from_frame2.clear();
  frame1_from_frame2.reserve(frame2_from_frame1_poselib.size());
  for (const pose_lib::CameraPose& solution : frame2_from_frame1_poselib) {
    CHECK_LT(std::abs(solution.alpha - 1.0), 1e-10) << fmt::format(
        "I assumed that gen_relpose_6pt does not set alpha. It did, to {}",
        solution.alpha);
    frame1_from_frame2.push_back(SE3(solution.R, solution.t).inverse());
  }

  return !frame1_from_frame2.empty();
}

}  // namespace grpose