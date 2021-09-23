#ifndef GRPOSE_GRPOSE_SOLVER_CENTRAL_PLUS_SCALE_
#define GRPOSE_GRPOSE_SOLVER_CENTRAL_PLUS_SCALE_

#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "grpose/non_central_relative_pose_minimal_solver.h"
#include "grpose/opengv_adapter.h"

namespace grpose {

class SolverCentralPlusScale : public NonCentralRelativePoseMinimalSolver {
 public:
  using OpengvCentralSolver =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using CentralSolverAlgorithm = OpengvCentralSolver::algorithm_t;

  SolverCentralPlusScale(
      const std::shared_ptr<OpengvAdapter> &adapter,
      CentralSolverAlgorithm algorithm = CentralSolverAlgorithm::STEWENIUS,
      bool deterministic = true, double degenerate_epsilon = 1e-10);

  int MinSampleSize() const override;
  bool SolveTimed(const std::vector<int> &correspondence_indices,
                  StdVectorA<SE3> &frame1_from_frame2,
                  double &time_in_seconds) const override;

 private:
  CentralSolverAlgorithm algorithm_;
  std::shared_ptr<OpengvAdapter> adapter_;
  OpengvCentralSolver opengv_central_solver_;
  double degenerate_epsilon_;
};

}  // namespace grpose

#endif