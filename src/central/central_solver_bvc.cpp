#include "central/central_solver_bvc.h"

namespace grpose {

CentralSolverBvc::~CentralSolverBvc() {}

bool CentralSolverBvc::Solve(
    const CentralBearingVectorCorrespondences& correspondences,
    SE3& frame1_from_frame2) const {
  return Solve(correspondences, frame1_from_frame2, nullptr);
}

}  // namespace grpose