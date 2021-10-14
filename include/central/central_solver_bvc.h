#ifndef GRPOSE_CENTRAL_CENTRAL_SOLVER_BVC_
#define GRPOSE_CENTRAL_CENTRAL_SOLVER_BVC_

#include "central/central_bearing_vector_correspondences.h"

namespace grpose {

class CentralSolverBvc {
 public:
  struct SolveInfo {
    int number_of_correspondences;
    int number_of_inliers;
    int number_of_iterations;
  };

  virtual ~CentralSolverBvc();

  virtual bool Solve(const CentralBearingVectorCorrespondences &correspondences,
                     SE3 &frame1_from_frame2, SolveInfo *solve_info) const = 0;
  bool Solve(const CentralBearingVectorCorrespondences &correspondences,
             SE3 &frame1_from_frame2) const;
};

}  // namespace grpose

#endif