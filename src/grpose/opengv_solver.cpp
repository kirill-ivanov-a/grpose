#include "grpose/opengv_solver.h"

#include <opengv/relative_pose/methods.hpp>

namespace grpose {

OpengvSolver::OpengvInternalSolver::algorithm_t OpengvSolver::ToOpengvAlgorithm(
    Algorithm algorithm) {
  switch (algorithm) {
    case OpengvSolver::Algorithm::kSixPoint:
    case OpengvSolver::Algorithm::kRawSixPoint:
      return OpengvInternalSolver::SIXPT;
    case OpengvSolver::Algorithm::kSeventeenPoint:
      return OpengvInternalSolver::SEVENTEENPT;
    case OpengvSolver::Algorithm::kGeneralizedEigensolver:
      return OpengvInternalSolver::GE;
    default:
      throw std::domain_error(
          fmt::format("Unknown algorithm type {}", algorithm));
  }
}

int OpengvSolver::MinSampleSize() const {
  if (algorithm_ == Algorithm::kRawSixPoint)
    return 6;
  else
    return opengv_solver_.getSampleSize();
}

OpengvSolver::OpengvSolver(const std::shared_ptr<OpengvAdapter>& opengv_adapter,
                           OpengvSolver::Algorithm algorithm,
                           bool deterministic)
    : algorithm_(algorithm),
      opengv_adapter_(opengv_adapter),
      opengv_solver_(opengv_adapter_->Get(), ToOpengvAlgorithm(algorithm_),
                     false, !deterministic) {}

bool OpengvSolver::SolveTimed(const std::vector<int>& correspondence_indices,
                              StdVectorA<SE3>& frame1_from_frame2,
                              double& time_in_seconds) const {
  frame1_from_frame2.clear();

  if (algorithm_ == Algorithm::kRawSixPoint)
    return SolveRaw6pt(correspondence_indices, frame1_from_frame2,
                       time_in_seconds);

  opengv::transformation_t frame1_from_frame2_matrix;  // 3x4 matrix

  auto start_time = std::chrono::high_resolution_clock::now();
  bool is_ok = opengv_solver_.computeModelCoefficients(
      correspondence_indices, frame1_from_frame2_matrix);
  auto end_time = std::chrono::high_resolution_clock::now();
  time_in_seconds = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                               end_time - start_time)
                               .count();

  if (!is_ok) return false;
  if (frame1_from_frame2_matrix.hasNaN()) return false;

  frame1_from_frame2.push_back(SE3(frame1_from_frame2_matrix.leftCols<3>(),
                                   frame1_from_frame2_matrix.rightCols<1>()));
  return true;
}

bool OpengvSolver::SolveRaw6pt(const std::vector<int>& correspondence_indices,
                               StdVectorA<SE3>& frame1_from_frame2,
                               double& time_in_seconds) const {
  CHECK_EQ(correspondence_indices.size(), 6);

  auto start_time = std::chrono::high_resolution_clock::now();
  const opengv::rotations_t frame1_from_frame2_rotations =
      opengv::relative_pose::sixpt(opengv_adapter_->Get(),
                                   correspondence_indices);
  auto end_time = std::chrono::high_resolution_clock::now();
  time_in_seconds = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(
                               end_time - start_time)
                               .count();

  for (const opengv::rotation_t& rotation : frame1_from_frame2_rotations)
    if (!rotation.hasNaN())
      frame1_from_frame2.emplace_back(rotation, Vector3::Zero());

  return !frame1_from_frame2.empty();
}

}  // namespace grpose