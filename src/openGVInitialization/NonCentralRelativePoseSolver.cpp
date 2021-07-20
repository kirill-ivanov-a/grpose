#include "openGVInitialization/NonCentralRelativePoseSolver.h"
#include <util.h>
#include <fstream>
#include <sophus/average.hpp>

namespace grpose {

NonCentralRelativePoseSolver::NonCentralRelativePoseSolver(
    const NonCentralRelativePoseSolverSettings &settings,
    const opengv::translations_t &offsets, const opengv::rotations_t &rotations)
    : settings(settings), extrinsics(offsets, rotations) {
  CHECK_GT(extrinsics.num_cameras, 2)
      << "Number of cameras must be greater than 2 to avoid degenerate non "
         "central "
         "relative pose problem";
}

NonCentralRelativePoseSolver::NonCentralRelativePoseSolver(
    const NonCentralRelativePoseSolverSettings &settings,
    const StdVectorA<SE3> &cam_extrinsics)
    : settings(settings), extrinsics(cam_extrinsics) {
  CHECK_GT(extrinsics.num_cameras, 2)
      << "Number of cameras must be greater than 2 to avoid degenerate non "
         "central "
         "relative pose problem";
}

NonCentralRelativePoseSolver::Extrinsics::Extrinsics(
    const opengv::translations_t &offsets,
    const opengv::rotations_t &rotations) {
  CHECK_EQ(offsets.size(), rotations.size());
  num_cameras = offsets.size();

  cam_offsets = offsets;
  cam_rotations = rotations;

  for (int i = 0; i < num_cameras; i++) {
    bodyFromCams.push_back(SE3(rotations[i], offsets[i]));
  }
}

NonCentralRelativePoseSolver::Extrinsics::Extrinsics(
    const StdVectorA<SE3> &cam_extrinsics) {
  num_cameras = cam_extrinsics.size();
  bodyFromCams = cam_extrinsics;
  for (int i = 0; i < num_cameras; i++) {
    cam_offsets.push_back(bodyFromCams[i].translation());
    cam_rotations.push_back(bodyFromCams[i].rotationMatrix());
  }
}

NonCentralRelativePoseSolution NonCentralRelativePoseSolver::solve(
    const BearingVectorCorrespondences &bvcs, int ransac_runs) {
  // Convert bearing and correspondence vectors format for the multi-camera
  // solver
  multiBearingVectors m_bv1, m_bv2;

  for (int c = 0; c < bvcs.numCameras; c++) {
    m_bv1.push_back(std::make_shared<opengv::bearingVectors_t>());
    m_bv2.push_back(std::make_shared<opengv::bearingVectors_t>());
  }
  for (int i = 0; i < bvcs.numCorrespondences; i++) {
    m_bv1[bvcs.correspondencesCurrent[i]]->push_back(
        bvcs.bearingVectorsCurrent[i]);
    m_bv2[bvcs.correspondencesNext[i]]->push_back(bvcs.bearingVectorsNext[i]);
  }
  return solve(m_bv1, m_bv2, ransac_runs);
}

// Just a convenience definition, should not use this one since doesn't do any
// checks
NonCentralRelativePoseSolution NonCentralRelativePoseSolver::solve(
    const opengv::bearingVectors_t &bearingVectorsCurrent,
    const opengv::bearingVectors_t &bearingVectorsNext,
    const std::vector<int> &correspondencesCurrent,
    const std::vector<int> &correspondencesNext) {
  // Convert bearing and correspondence vectors format for the multi-camera
  // solver
  multiBearingVectors m_bv1, m_bv2;

  for (int c = 0; c < extrinsics.num_cameras; c++) {
    m_bv1.push_back(std::make_shared<opengv::bearingVectors_t>());
    m_bv2.push_back(std::make_shared<opengv::bearingVectors_t>());
  }
  for (int i = 0; i < bearingVectorsCurrent.size(); i++) {
    m_bv1[correspondencesCurrent[i]]->push_back(bearingVectorsCurrent[i]);
    m_bv2[correspondencesNext[i]]->push_back(bearingVectorsNext[i]);
  }

  return solve(m_bv1, m_bv2);
}

// Actual method which solves the relative pose problem using OpenGV's 'multi'
// formulation Uses multi adapter from OpenGV to evenly sample correspondences
// across the cameras in the rig
NonCentralRelativePoseSolution NonCentralRelativePoseSolver::solve(
    const multiBearingVectors &bearingVectorsCurrent,
    const multiBearingVectors &bearingVectorsNext, int ransac_runs) {
  // Check both frames have the same number of cameras
  if (bearingVectorsCurrent.size() != bearingVectorsNext.size()) {
    throw std::runtime_error("Number of cameras not equal between frames");
  }
  int num_cameras = bearingVectorsCurrent.size();

  // choose the solver algorithm
  opengv::sac_problems::relative_pose::MultiNoncentralRelativePoseSacProblem::
      Algorithm algorithm;
  int sample_size;
  switch (settings.algorithm) {
    // 17pt linear
    case SolverAlgorithm::SEVENTEENPT:
      algorithm = opengv::sac_problems::relative_pose::
          MultiNoncentralRelativePoseSacProblem::SEVENTEENPT;
      sample_size =
          17;  // from OpenGV MultiNoncentralRelativePoseSacProblem.cpp
      LOG(INFO) << "Solver using 17pt algorithm";
      break;

    // Stewenius 6pt, OpenGV takes care of checking the 64 possible solutions
    case SolverAlgorithm::SIXPT:
      algorithm = opengv::sac_problems::relative_pose::
          MultiNoncentralRelativePoseSacProblem::SIXPT;
      sample_size =
          6 + 3;  // from OpenGV MultiNoncentralRelativePoseSacProblem.cpp
      LOG(INFO) << "Solver using 6pt algorithm";
      break;

    // Generalized Eigensolver
    // No idea what this actually does, but according to OpenGV paper relative
    // pose can be formulated as an eigenvalue problem
    case SolverAlgorithm::GE:
      algorithm = opengv::sac_problems::relative_pose::
          MultiNoncentralRelativePoseSacProblem::GE;
      sample_size = 8;  // from OpenGV MultiNoncentralRelativePoseSacProblem.cpp
      LOG(INFO) << "Solver using Generalized Eigensolver algorithm";
      break;

    default:
      LOG(FATAL) << "unknown solver algorithm";
      break;
  }
  // Checks we have enough correspondences before solving
  // The multi-camera setup from OpenGV tries to evenly divide up
  // the sample it gets from each camera
  // See MultiNoncentralRelativePoseSacProblem.cpp / getSampleSizes()
  // Also because of the way the multi camera sampling is implemented in OpenGV,
  // it's better to round up and be safe
  int min_cam_correspondences =
      (int)std::ceil(((float)sample_size / (float)num_cameras));

  int total_correspondences = 0;
  for (const auto &c : bearingVectorsCurrent) {
    if (c->size() < min_cam_correspondences) {
      std::stringstream error_msg;
      error_msg << "A camera cannot does not have the "
                << min_cam_correspondences
                << " required minimum correspondences";
      throw std::runtime_error(error_msg.str());
    }
    total_correspondences += c->size();
  }
  if (total_correspondences < sample_size) {
    std::stringstream error_msg;
    error_msg << "Cameras do not have enough correspondences for the RANSAC "
                 "sample size of "
              << sample_size;
    throw std::runtime_error(error_msg.str());
  }

  // create the non-central MULTI relative adapter which holds the data
  opengv::relative_pose::NoncentralRelativeMultiAdapter adapter(
      bearingVectorsCurrent, bearingVectorsNext, extrinsics.cam_offsets,
      extrinsics.cam_rotations);

  // create a MultiNoncentralRelativePoseSacProblem with RANSAC
  opengv::sac::MultiRansac<opengv::sac_problems::relative_pose::
                               MultiNoncentralRelativePoseSacProblem>
      ransac;

  std::vector<int> numberCorrespondences;
  for (size_t i = 0; i < adapter.getNumberPairs(); i++) {
    numberCorrespondences.push_back(adapter.getNumberCorrespondences(i));
  }

  std::shared_ptr<opengv::sac_problems::relative_pose::
                      MultiNoncentralRelativePoseSacProblem>
      relposeproblem_ptr(
          new opengv::sac_problems::relative_pose::
              MultiNoncentralRelativePoseSacProblem(adapter, algorithm));

  // run ransac with the set parameters
  ransac.sac_model_ = relposeproblem_ptr;
  ransac.threshold_ = settings.ransac_threshold;
  ransac.max_iterations_ = settings.ransac_max_iter;
  ransac.probability_ = settings.ransac_probability;

  StdVectorA<Eigen::Vector3d> translation_estimates;
  StdVectorA<SO3> rotation_estimates;

  for (int i = 0; i < ransac_runs; i++) {
    ransac.computeModel(settings.ransac_verbosity_level);

    // // OpenGV only counts the inliers, but actually doesn't refine the ransac
    // solution
    // // with all the inliers found...
    // // Need to do that ourselves
    // opengv::sac_problems::relative_pose::
    //   MultiNoncentralRelativePoseSacProblem::model_t model_all_inliers;
    // ransac.sac_model_->computeModelCoefficients(ransac.inliers_,
    // model_all_inliers);
    /* DON'T DO THIS FOR NOW, since it seems like it actually makes the estimate
       worst I think the reason this is happening is because of imbalance of
       number of features between the cameras. Bascially we can find a lot of
       inliers and this "oh yea this is the best model", but really most of
       those inliers were just from one camera, 'so kinda tricks the RANSAC'
       into thinking it's the best model when really those 'inliers' were all
       from one camera and doesn't really do much to fully constrain all 6-DoF
       of motion For example, if all the 'inliers' were from the rear camera,
       and we're doing a linear solver where all correspondences are weighed
       equally, then it could be that all these 'inliers' we end up taking from
       the rear camera kinda overpowers the rest of the correspondences in their
       effect on the estimate.
    */

    rotation_estimates.push_back(
        SO3(ransac.model_coefficients_.block<3, 3>(0, 0)));
    translation_estimates.push_back(ransac.model_coefficients_.col(3));

    if (settings.solver_verbose) {
      LOG(INFO) << "ran ransac, inliers this time "
                << countMultiCamInliers(ransac.inliers_) << std::endl
                << ransac.model_coefficients_ << std::endl;
    }
  }

  auto mean_rotation = *Sophus::average(
      rotation_estimates);  // Returns a custom optional type, which we
                            // have to dereference to get the value
  Eigen::Vector3d mean_translation;
  for (const auto &t : translation_estimates) {
    mean_translation += (t / ransac_runs);
  }

  // Get the resulting estimated relative pose from the next viewpoint (vp2) to
  // the current viewpoint (vp1) And convert to an SE3
  SE3 vp1_vp2(mean_rotation, mean_translation);

  if (settings.solver_verbose) {
    LOG(INFO) << "average rotation matrix" << std::endl
              << mean_rotation.matrix() << std::endl;
    LOG(INFO) << "average translation" << std::endl
              << mean_translation << std::endl;
    LOG(INFO) << "relative pose estimate " << std::endl
              << vp1_vp2.matrix() << std::endl;
  }

  // Get inliers of the averaged model
  opengv::sac_problems::relative_pose::MultiNoncentralRelativePoseSacProblem::
      model_t averaged_model;
  averaged_model.block<3, 3>(0, 0) = vp1_vp2.rotationMatrix();
  averaged_model.col(3) = vp1_vp2.translation();

  std::vector<std::vector<int>> inliers;
  ransac.sac_model_->selectWithinDistance(averaged_model,
                                          settings.ransac_threshold, inliers);

  // Triangulate 3D points expressed in viewpoint 1 coordinates
  opengv::points_t triangulated_points_vp1 =
      triangulate(bearingVectorsCurrent, bearingVectorsNext, vp1_vp2, inliers);

  NonCentralRelativePoseSolution solution;
  solution.num_inliers = countMultiCamInliers(inliers);
  solution.relative_pose = vp1_vp2;
  solution.triangulated_points = triangulated_points_vp1;
  return solution;
}

opengv::points_t NonCentralRelativePoseSolver::triangulate(
    const multiBearingVectors &bearingVectorsCurrent,
    const multiBearingVectors &bearingVectorsNext, const SE3 &currentFromNext,
    const std::vector<std::vector<int>> &inliers) {
  // Triangulation in OpenGV is only implemented for the central case
  // Meaning we have to triangulate points for each camera seperately,
  // and then transform them to the "world" (vp1) frame

  std::vector<opengv::bearingVectors_t> cam_bv_current(extrinsics.num_cameras);
  std::vector<opengv::bearingVectors_t> cam_bv_next(extrinsics.num_cameras);

  for (int c = 0; c < inliers.size(); c++) {
    for (int i = 0; i < inliers[c].size(); i++) {
      opengv::bearingVector_t bv_current =
          bearingVectorsCurrent[c]->at(inliers[c][i]);

      opengv::bearingVector_t bv_next =
          bearingVectorsNext[c]->at(inliers[c][i]);

      cam_bv_current[c].push_back(bv_current);
      cam_bv_next[c].push_back(bv_next);
    }
  }

  // Get all triangulated points expressed in the body frame of the
  // first/current viewpoint
  opengv::points_t points_current_viewpoint;
  for (int c = 0; c < extrinsics.num_cameras; c++) {
    // get transform from cam in view 2 to cam in view 1
    SE3 viewpoint_from_cam = extrinsics.bodyFromCams[c];
    SE3 cam_curr_from_cam_next =
        viewpoint_from_cam.inverse() * currentFromNext * viewpoint_from_cam;

    opengv::relative_pose::CentralRelativeAdapter triangulate_adapter(
        cam_bv_current[c], cam_bv_next[c], cam_curr_from_cam_next.translation(),
        cam_curr_from_cam_next
            .rotationMatrix());  // pass in the estimated relative
                                 // transformation from ransac

    for (int p = 0; p < cam_bv_current[c].size(); p++) {
      // OpenGV functions only triangulate one point at a time
      // and return the point triangulated in the coordinate frame of the first
      // camera
      opengv::point_t point_cam =
          opengv::triangulation::triangulate(triangulate_adapter, p);
      opengv::point_t point_curr = viewpoint_from_cam * point_cam;
      points_current_viewpoint.push_back(point_curr);
    }
  }

  return points_current_viewpoint;
}

int countMultiCamInliers(std::vector<std::vector<int>> inliers) {
  int num_inliers = 0;
  for (const auto &cam_inliers : inliers) {
    num_inliers += cam_inliers.size();
  }
  return num_inliers;
}

}  // namespace grpose