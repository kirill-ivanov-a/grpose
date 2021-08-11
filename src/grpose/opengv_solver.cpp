#include "grpose/opengv_solver.h"

#include <opengv/relative_pose/methods.hpp>

namespace grpose {

OpengvSolver::OpengvInternalSolver::algorithm_t OpengvSolver::ToOpengvAlgorithm(
    Algorithm algorithm) {
  switch (algorithm) {
    case OpengvSolver::Algorithm::kSixPoint:
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

OpengvSolver::OpengvSolver(const std::shared_ptr<OpengvAdapter>& opengv_adapter,
                           OpengvSolver::Algorithm algorithm,
                           bool deterministic)
    : opengv_adapter_(opengv_adapter),
      opengv_solver_(opengv_adapter_->Get(), ToOpengvAlgorithm(algorithm),
                     false, !deterministic) {}

bool OpengvSolver::Solve(const std::vector<int>& correspondence_indices,
                         StdVectorA<SE3>& frame1_from_frame2) const {
  frame1_from_frame2.clear();

  opengv::transformation_t frame1_from_frame2_matrix;  // 3x4 matrix
  bool is_ok = opengv_solver_.computeModelCoefficients(
      correspondence_indices, frame1_from_frame2_matrix);
  if (!is_ok) return false;

  frame1_from_frame2.push_back(SE3(frame1_from_frame2_matrix.leftCols<3>(),
                                   frame1_from_frame2_matrix.rightCols<1>()));
  return true;
}

}  // namespace grpose