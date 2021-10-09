#ifndef GRPOSE_GRPOSE_OPENGV_SOLVER_BVC_
#define GRPOSE_GRPOSE_OPENGV_SOLVER_BVC_

#include <opengv/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.hpp>

#include "grpose/solver_bvc.h"

namespace grpose {

struct OpengvSolverBvcSettings {
  using Algorithm = opengv::sac_problems::relative_pose::
      MultiNoncentralRelativePoseSacProblem::Algorithm;

  // Same default params as in opengv
  Algorithm algorithm = Algorithm::GE;
  int max_iterations = 10000;
  double threshold = 0.0001;
  double probability = 0.99;
  int ransac_verbosity = 0;
};

/**
 * An adapter to the OpenGV's implementation of RANSAC. Right now it uses the
 * "Multi-" formulation, i.e. the correspondences for the minimal solver are
 * sampled evenly among cameras, no cross-camera correspondences are used.
 *
 * BVC = bearing vector correspondences
 */
class OpengvSolverBvc : public SolverBvc {
 public:
  OpengvSolverBvc(const StdVectorA<SE3> &body_from_cameras,
                  const OpengvSolverBvcSettings &settings);

  bool Solve(const BearingVectorCorrespondences &correspondences,
             SE3 &frame1_from_frame2) override;

  inline int LastNumberOfIterations() const;
  int MinSampleSize() const;

 private:
  using OpengvInternalMinimalSolver = opengv::sac_problems::relative_pose::
      MultiNoncentralRelativePoseSacProblem;

  StdVectorA<SE3> body_from_cameras_;
  OpengvSolverBvcSettings settings_;
  int last_number_of_iterations_ = 0;
};

// Implementation

int OpengvSolverBvc::LastNumberOfIterations() const {
  return last_number_of_iterations_;
}

}  // namespace grpose

#endif