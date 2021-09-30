#include "grpose/opengv_solver_bvc.h"

// OpenGV's MultiRansac.hpp is not independent in terms of includes it needs.
#include <climits>
#include <memory>

#include <Eigen/Core>
#include <opengv/sac/MultiRansac.hpp>

#include "grpose/opengv_adapter.h"

namespace grpose {

OpengvSolverBvc::OpengvSolverBvc(const StdVectorA<SE3>& body_from_cameras,
                                 const OpengvSolverBvcSettings& settings)
    : body_from_cameras_(body_from_cameras), settings_(settings) {}

bool OpengvSolverBvc::Solve(const BearingVectorCorrespondences& correspondences,
                            SE3& frame1_from_frame2) {
  // TODO what to do with references here? I want neither copying, nor making
  // the parameter of Solve a shared pointer
  std::shared_ptr correspondences_ptr =
      std::make_shared<BearingVectorCorrespondences>(correspondences);
  OpengvAdapter opengv_adapter(correspondences_ptr, body_from_cameras_);
  std::shared_ptr minimal_problem =
      std::make_shared<OpengvInternalMinimalSolver>(
          opengv_adapter.GetMultiAdapter(), settings_.algorithm);

  // TODO: mb try out the non-balanced ransac as well
  opengv::sac::MultiRansac<OpengvInternalMinimalSolver> ransac(
      settings_.max_iterations, settings_.threshold, settings_.probability);
  ransac.sac_model_ = minimal_problem;

  bool is_ok = ransac.computeModel(settings_.ransac_verbosity);
  last_number_of_iterations_ = ransac.iterations_;
  if (!is_ok) return false;

  frame1_from_frame2 = SE3(ransac.model_coefficients_.leftCols<3>(),
                           ransac.model_coefficients_.col(3));
  return true;
}

int OpengvSolverBvc::MinSampleSize() const {
  switch (settings_.algorithm) {
    case OpengvSolverBvcSettings::Algorithm::SEVENTEENPT:
      return 17;
    case OpengvSolverBvcSettings::Algorithm::GE:
      return 8;
    case OpengvSolverBvcSettings::Algorithm::SIXPT:
      return 6 + 3;
    default:
      throw std::domain_error(
          fmt::format("Unknown algo {}", settings_.algorithm));
  }
}

}  // namespace grpose