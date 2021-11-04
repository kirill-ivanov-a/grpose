#ifndef GRPOSE_CENTRAL_CENTRAL_OPENGV_MINIMAL_SOLVER_
#define GRPOSE_CENTRAL_CENTRAL_OPENGV_MINIMAL_SOLVER_

#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "central/central_bearing_vector_correspondences.h"
#include "central/central_opengv_adapter.h"

namespace grpose {

/**
 * A small adapter to OpenGV's central relative minimal solvers.
 */
class CentralOpengvMinimalSolver {
 public:
  using Algorithm = opengv::sac_problems::relative_pose::
      CentralRelativePoseSacProblem::Algorithm;

  CentralOpengvMinimalSolver(
      const CentralBearingVectorCorrespondences &correspondences,
      Algorithm algorithm, bool deterministic = true);

  bool Solve(const std::vector<int> &correspondence_indices,
             SE3 &frame1_from_frame2) const;

  int SampleSize() const;

 private:
  using OpengvInternalMinimalSolver =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;

  CentralOpengvAdapter adapter_;
  OpengvInternalMinimalSolver solver_;
};

}  // namespace grpose

#endif
