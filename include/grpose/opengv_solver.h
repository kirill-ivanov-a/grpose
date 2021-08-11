#ifndef GRPOSE_GRPOSE_OPENGV_SOLVER_
#define GRPOSE_GRPOSE_OPENGV_SOLVER_

#include <memory>

#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>

#include "grpose/non_central_relative_pose_solver.h"
#include "grpose/opengv_adapter.h"

namespace grpose {

class OpengvSolver : public NonCentralRelativePoseSolver {
 public:
  enum class Algorithm {
    kSixPoint,                // Stewenius 6pt algorithm
    kGeneralizedEigensolver,  // Kneip Generalized Eigensolver
    kSeventeenPoint           // Li 17pt algorithm
  };

  OpengvSolver(const std::shared_ptr<OpengvAdapter> &opengv_adapter,
               Algorithm algorithm, bool deterministic = true);

  int MinimalNeededCorrespondences() const override;
  bool Solve(const std::vector<int> &correspondence_indices,
             StdVectorA<SE3> &frame1_from_frame2) const override;

 private:
  using OpengvInternalSolver =
      opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem;

  static OpengvInternalSolver::algorithm_t ToOpengvAlgorithm(
      Algorithm algorithm);

  Algorithm algorithm_;
  std::shared_ptr<OpengvAdapter> opengv_adapter_;
  OpengvInternalSolver opengv_solver_;
};

}  // namespace grpose

#endif
