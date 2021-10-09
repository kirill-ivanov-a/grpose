#ifndef GRPOSE_CENTRAL_CENTRAL_SOLVER_FROM_POINTS2D_
#define GRPOSE_CENTRAL_CENTRAL_SOLVER_FROM_POINTS2D_

#include "central/central_point2d_correspondences.h"
#include "util/types.h"

namespace grpose {

class CentralSolverFromPoints2d {
 public:
  /**
   * A struct containing various statistics of a call to
   * CentralSolverFromPoints2d::Solve . For example, timing or number of RANSAC
   * iterations could be stored there. Updated as needed.
   */
  struct SolveInfo {};

  virtual ~CentralSolverFromPoints2d();

  virtual bool Solve(const CentralPoint2dCorrespondences &correspondences,
                     SE3 &frame1_from_frame2, SolveInfo *solve_info) const = 0;
  bool Solve(const CentralPoint2dCorrespondences &correspondences,
             SE3 &frame1_from_frame2) const;
};

}  // namespace grpose

#endif