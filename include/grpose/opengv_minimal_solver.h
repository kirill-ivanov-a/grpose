#ifndef GRPOSE_GRPOSE_OPENGV_MINIMAL_SOLVER_
#define GRPOSE_GRPOSE_OPENGV_MINIMAL_SOLVER_

#include <memory>

#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>

#include "grpose/minimal_solver.h"
#include "grpose/opengv_adapter.h"

namespace grpose {

class OpengvMinimalSolver : public MinimalSolver {
 public:
  enum class Algorithm {
    kSixPoint,  // Stewenius 6pt algorithm with OpenGV's translation estimation
                // and solution selection based on additional correspondences
    kRawSixPoint,  // Stewenius 6pt algorithm; **WARNING**: ONLY RETURNS VALID
                   // ROTATIONS, TRANSLATIONS SET TO ZERO
    kGeneralizedEigensolver,  // Kneip Generalized Eigensolver
    kSeventeenPoint           // Li 17pt algorithm
  };

  OpengvMinimalSolver(const std::shared_ptr<OpengvAdapter> &opengv_adapter,
                      Algorithm algorithm, bool deterministic = true);

  int MinSampleSize() const override;
  bool SolveTimed(const std::vector<int> &correspondence_indices,
                  StdVectorA<SE3> &frame1_from_frame2,
                  double &time_in_seconds) const override;

 private:
  using OpengvInternalSolver =
      opengv::sac_problems::relative_pose::NoncentralRelativePoseSacProblem;

  static OpengvInternalSolver::algorithm_t ToOpengvAlgorithm(
      Algorithm algorithm);

  bool SolveRaw6pt(const std::vector<int> &correspondence_indices,
                   StdVectorA<SE3> &frame1_from_frame2,
                   double &time_in_seconds) const;

  Algorithm algorithm_;
  std::shared_ptr<OpengvAdapter> opengv_adapter_;
  OpengvInternalSolver opengv_solver_;
};

}  // namespace grpose

#endif
