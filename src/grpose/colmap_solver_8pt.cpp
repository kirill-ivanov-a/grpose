#include "grpose/colmap_solver_8pt.h"
#include "util/util.h"

#include <chrono>

#include <colmap_extracts/base/projection.h>
#include <colmap_extracts/estimators/generalized_relative_pose.h>

namespace grpose {

ColmapSolver8pt::ColmapSolver8pt(
    const std::shared_ptr<BearingVectorCorrespondences> &correspondences,
    const StdVectorA<SE3> &body_from_camera)
    : correspondences_(correspondences), body_from_camera_(body_from_camera) {}

int ColmapSolver8pt::MinSampleSize() const { return 8; }

bool ColmapSolver8pt::SolveTimed(const std::vector<int> &correspondence_indices,
                                 StdVectorA<SE3> &frame1_from_frame2,
                                 double &time_in_seconds) const {
  const int kSampleSize = MinSampleSize();

  CHECK_EQ(correspondence_indices.size(), kSampleSize);

  std::vector<colmap::GR6PEstimator::M_t> frame1_from_frame2_matrices;
  std::vector<colmap::GR6PEstimator::X_t> points[2];

  for (int fi : {0, 1}) points[fi].reserve(kSampleSize);

  for (int i : correspondence_indices) {
    for (int fi : {0, 1}) {
      const SE3 &body_from_current_camera =
          body_from_camera_[correspondences_->camera_index(fi, i)];
      auto observation = correspondences_->bearing_vector(fi, i);

      points[fi].emplace_back();
      points[fi].back().rel_tform =
          colmap::InvertProjectionMatrix(body_from_current_camera.matrix3x4());
      points[fi].back().xy = observation.hnormalized();
    }
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  frame1_from_frame2_matrices =
      colmap::GR6PEstimator::Estimate(points[0], points[1]);
  auto end_time = std::chrono::high_resolution_clock::now();
  time_in_seconds = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                               end_time - start_time)
                               .count();

  frame1_from_frame2.clear();
  frame1_from_frame2.reserve(frame1_from_frame2_matrices.size());

  for (const colmap::GR6PEstimator::M_t &solution :
       frame1_from_frame2_matrices) {
    if (!is_nan(solution))
      frame1_from_frame2.push_back(
          SE3(solution.leftCols<3>(), solution.rightCols<1>()).inverse());
  }

  return !frame1_from_frame2.empty();
}

}  // namespace grpose