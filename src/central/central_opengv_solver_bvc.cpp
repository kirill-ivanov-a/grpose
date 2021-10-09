#include "central/central_opengv_solver_bvc.h"

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>

namespace grpose {

grpose::CentralOpengvSolverBvc::CentralOpengvSolverBvc(
    const grpose::CentralOpengvSolverBvcSettings& settings)
    : settings_(settings) {}

bool CentralOpengvSolverBvc::Solve(
    const CentralBearingVectorCorrespondences& correspondences,
    SE3& frame1_from_frame2, SolveInfo* solve_info) const {
  using OpengvInternalMinimalSolver =
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  using OpengvInternalAdapter = opengv::relative_pose::CentralRelativeAdapter;

  OpengvInternalAdapter adapter(correspondences.bearing_vectors(0),
                                correspondences.bearing_vectors(1));
  auto solver = std::make_shared<OpengvInternalMinimalSolver>(
      adapter, settings_.algorithm, true);
  opengv::sac::Ransac<OpengvInternalMinimalSolver> ransac(
      settings_.max_iterations, settings_.threshold, settings_.probability);
  ransac.sac_model_ = solver;

  bool is_ok = ransac.computeModel(settings_.ransac_verbosity);
  if (solve_info) solve_info->number_of_iterations = ransac.iterations_;
  if (!is_ok) return false;

  frame1_from_frame2 = SE3(ransac.model_coefficients_.leftCols<3>(),
                           ransac.model_coefficients_.col(3));

  return true;
}

}  // namespace grpose