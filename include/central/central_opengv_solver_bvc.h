#ifndef GRPOSE_CENTRAL_CENTRAL_OPENGV_SOLVER_BVC_
#define GRPOSE_CENTRAL_CENTRAL_OPENGV_SOLVER_BVC_

#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "central/central_solver_bvc.h"

namespace grpose {

struct CentralOpengvSolverBvcSettings {
  using Algorithm = opengv::sac_problems::relative_pose::
      CentralRelativePoseSacProblem::Algorithm;

  Algorithm algorithm = Algorithm::STEWENIUS;
  int max_iterations = 10000;
  double threshold = 0.0001;
  double probability = 0.99;
  int ransac_verbosity = 0;
};

class CentralOpengvSolverBvc : public CentralSolverBvc {
 public:
  CentralOpengvSolverBvc(const CentralOpengvSolverBvcSettings &settings);

  bool Solve(const CentralBearingVectorCorrespondences &correspondences,
             SE3 &frame1_from_frame2, SolveInfo *solve_info) const override;

 private:
  CentralOpengvSolverBvcSettings settings_;
};

}  // namespace grpose

#endif