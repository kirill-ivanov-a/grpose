#include "central/central_solver_from_points2d.h"

namespace grpose {

CentralSolverFromPoints2d::~CentralSolverFromPoints2d() {}

bool CentralSolverFromPoints2d::Solve(
    const CentralPoint2dCorrespondences& correspondences,
    SE3& frame1_from_frame2) const {
  return Solve(correspondences, frame1_from_frame2, nullptr);
}

}  // namespace grpose