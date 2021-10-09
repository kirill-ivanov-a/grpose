#ifndef GRPOSE_GRPOSE_SOLVER_BVC_
#define GRPOSE_GRPOSE_SOLVER_BVC_

#include "grpose/bearing_vector_correspondences.h"

namespace grpose {

/**
 * An interface for generalized relative pose solvers from sets of approximate
 * contaminated bearing vector correspondences (BVC).
 */
class SolverBvc {
 public:
  virtual ~SolverBvc();

  // TODO switch to SolveInfo and make this const
  virtual bool Solve(const BearingVectorCorrespondences &correspondences,
                     SE3 &frame1_from_frame2) = 0;
};

}  // namespace grpose

#endif
