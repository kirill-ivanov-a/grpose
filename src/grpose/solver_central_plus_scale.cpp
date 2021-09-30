#include "grpose/solver_central_plus_scale.h"

#include <map>
#include <vector>

namespace grpose {

SolverCentralPlusScale::SolverCentralPlusScale(
    const std::shared_ptr<OpengvAdapter> &adapter,
    CentralSolverAlgorithm algorithm, bool deterministic,
    double degenerate_epsilon)
    : algorithm_(algorithm),
      adapter_(adapter),
      opengv_central_solver_(adapter_->GetAdapter(), algorithm_,
                             !deterministic),
      degenerate_epsilon_(degenerate_epsilon) {}

int SolverCentralPlusScale::MinSampleSize() const {
  // TODO OpenGV's CentralRelativePoseSacProblem requires more correspondences
  // then the method it implements, since in the implementation authors do
  // disambiguation with additional correspondences. This is a questionable
  // solution, and maybe in the future we would like to use the corresponding
  // implementations of the methods directly (e.g.
  // opengv::relative_pose::fivept_stewenius)
  return opengv_central_solver_.getSampleSize() + 1;
}

bool SolverCentralPlusScale::SolveTimed(
    const std::vector<int> &correspondence_indices,
    StdVectorA<SE3> &frame1_from_frame2, double &time_in_seconds) const {
  auto start_time = std::chrono::high_resolution_clock::now();

  // TODO normal assert
  CHECK_GE(correspondence_indices.size(), MinSampleSize());

  frame1_from_frame2.clear();

  const BearingVectorCorrespondences &correspondences =
      adapter_->GetCorrespondences();

  std::map<std::pair<int, int>, std::vector<int>> camera_pairs;
  for (int i : correspondence_indices) {
    camera_pairs[{correspondences.camera_index(0, i),
                  correspondences.camera_index(1, i)}]
        .push_back(i);
  }
  CHECK_EQ(camera_pairs.size(), 2);
  int size_per_pair[2] = {
      static_cast<int>(camera_pairs.begin()->second.size()),
      static_cast<int>(camera_pairs.rbegin()->second.size())};
  CHECK(size_per_pair[0] == 1 || size_per_pair[1] == 1) << fmt::format(
      "Expected correspondences from exactly two pairs of cameras, and "
      "one of these pairs should have exactly one correspondence. Got "
      "{} and {} correspondences per pair.",
      size_per_pair[0], size_per_pair[1]);
  std::pair<int, int> pair_central, pair_1;
  std::vector<int> correspondences_central;
  int correspondence_1;
  if (size_per_pair[0] == 1) {
    pair_1 = camera_pairs.begin()->first;
    pair_central = camera_pairs.rbegin()->first;
    correspondence_1 = camera_pairs.begin()->second[0];
    correspondences_central = camera_pairs.rbegin()->second;
  } else {
    pair_1 = camera_pairs.rbegin()->first;
    pair_central = camera_pairs.begin()->first;
    correspondence_1 = camera_pairs.rbegin()->second[0];
    correspondences_central = camera_pairs.begin()->second;
  }

  // frames 1 & 2, camera_c1 -> camera having all correspondences needed for the
  // central solver in frame 1, camera_12 -> camera having 1 scale
  // correspondence in frame 2, etc.
  opengv::transformation_t camera_c1_from_camera_c2_matrix;  // 3x4 matrix
  // TODO almost sure that opengv doesn't take camera offsets relative to
  // viewpoints (frames) for central solvers, but worth double-checking
  if (!opengv_central_solver_.computeModelCoefficients(
          correspondences_central, camera_c1_from_camera_c2_matrix))
    return false;
  if (camera_c1_from_camera_c2_matrix.hasNaN()) return false;

  const SE3 body_from_camera_c1(
      adapter_->GetAdapter().getCamRotation1(correspondences_central[0]),
      adapter_->GetAdapter().getCamOffset1(correspondences_central[0]));
  const SE3 body_from_camera_c2(
      adapter_->GetAdapter().getCamRotation2(correspondences_central[0]),
      adapter_->GetAdapter().getCamOffset2(correspondences_central[0]));
  const SE3 body_from_camera_11(
      adapter_->GetAdapter().getCamRotation1(correspondence_1),
      adapter_->GetAdapter().getCamOffset1(correspondence_1));
  const SE3 body_from_camera_12(
      adapter_->GetAdapter().getCamRotation2(correspondence_1),
      adapter_->GetAdapter().getCamOffset2(correspondence_1));

  const SE3 camera_11_from_camera_c1 =
      body_from_camera_11.inverse() * body_from_camera_c1;
  const SE3 camera_c2_from_camera_12 =
      body_from_camera_c2.inverse() * body_from_camera_12;
  const Matrix33 R0 = camera_11_from_camera_c1.rotationMatrix();
  const Vector3 &t0 = camera_11_from_camera_c1.translation();
  const Matrix33 R1 = camera_c2_from_camera_12.rotationMatrix();
  const Vector3 &t1 = camera_c2_from_camera_12.translation();
  const Matrix33 R = camera_c1_from_camera_c2_matrix.leftCols<3>();
  const Vector3 t = camera_c1_from_camera_c2_matrix.col(3);
  const Vector3 &d0 = correspondences.bearing_vector(0, correspondence_1);
  const Vector3 &d1 = correspondences.bearing_vector(1, correspondence_1);

  const Matrix33 R_tilde = R0 * R * R1;
  const Vector3 t_hat = R0 * t;
  const Vector3 t_star = t0 + R0 * R * t1;

  const double alpha_numerator =
      d0.transpose() * SO3::hat(t_star) * R_tilde * d1;
  const double alpha_denominator =
      d0.transpose() * SO3::hat(t_hat) * R_tilde * d1;

  // In the degenerate case we still cannot reconstruct the scale
  const double alpha = std::abs(alpha_denominator) < degenerate_epsilon_
                           ? 1.0
                           : -alpha_numerator / alpha_denominator;

  const SE3 camera_c1_from_camera_c2(R, alpha * t);
  frame1_from_frame2.push_back(body_from_camera_c1 * camera_c1_from_camera_c2 *
                               body_from_camera_c2.inverse());

  auto end_time = std::chrono::high_resolution_clock::now();
  time_in_seconds = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                               end_time - start_time)
                               .count();

  return true;
}

}  // namespace grpose