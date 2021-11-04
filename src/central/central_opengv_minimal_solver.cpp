#include "central/central_opengv_minimal_solver.h"

namespace grpose {

CentralOpengvMinimalSolver::CentralOpengvMinimalSolver(
    const CentralBearingVectorCorrespondences& correspondences,
    Algorithm algorithm, bool deterministic)
    : adapter_(correspondences),
      solver_(adapter_.GetAdapter(), algorithm, !deterministic) {}

bool CentralOpengvMinimalSolver::Solve(
    const std::vector<int>& correspondence_indices,
    SE3& frame1_from_frame2) const {
  OpengvInternalMinimalSolver::model_t frame1_from_frame2_matrix;
  bool is_ok = solver_.computeModelCoefficients(correspondence_indices,
                                                frame1_from_frame2_matrix);
  if (!is_ok) return false;
  frame1_from_frame2 = SE3(frame1_from_frame2_matrix.leftCols<3>(),
                           frame1_from_frame2_matrix.col(3));
  return true;
}

int CentralOpengvMinimalSolver::SampleSize() const {
  return solver_.getSampleSize();
}

}  // namespace grpose