#include "init_opengv/non_central_relative_pose_solver_old.h"

#include <util/util.h>

#include <fstream>

#include <sophus/average.hpp>

namespace grpose {

NonCentralRelativePoseSolverOld::NonCentralRelativePoseSolverOld(
    const NonCentralRelativePoseSolverSettings &settings,
    const opengv::translations_t &body_from_camera_translations,
    const opengv::rotations_t &body_from_camera_rotations)
    : settings_(settings),
      extrinsics_(body_from_camera_translations, body_from_camera_rotations) {
  CHECK_GT(extrinsics_.number_of_cameras, 2)
      << "Number of cameras must be greater than 2 to avoid degenerate "
         "non-central relative pose problem";
}

NonCentralRelativePoseSolverOld::NonCentralRelativePoseSolverOld(
    const NonCentralRelativePoseSolverSettings &settings,
    const StdVectorA<SE3> &body_from_cameras)
    : settings_(settings), extrinsics_(body_from_cameras) {
  CHECK_GT(extrinsics_.number_of_cameras, 2)
      << "Number of cameras must be greater than 2 to avoid degenerate "
         "non-central relative pose problem";
}

NonCentralRelativePoseSolverOld::Extrinsics::Extrinsics(
    const opengv::translations_t &body_from_camera_translations,
    const opengv::rotations_t &body_from_camera_rotations)
    : body_from_camera_translations(body_from_camera_translations),
      body_from_camera_rotations(body_from_camera_rotations) {
  CHECK_EQ(body_from_camera_translations.size(),
           body_from_camera_rotations.size());
  number_of_cameras = body_from_camera_translations.size();

  for (int i = 0; i < number_of_cameras; i++) {
    body_from_cameras.push_back(
        SE3(body_from_camera_rotations[i], body_from_camera_translations[i]));
  }
}

NonCentralRelativePoseSolverOld::Extrinsics::Extrinsics(
    const StdVectorA<SE3> &body_from_cameras)
    : body_from_cameras(body_from_cameras) {
  number_of_cameras = body_from_cameras.size();
  for (int i = 0; i < number_of_cameras; i++) {
    body_from_camera_translations.push_back(body_from_cameras[i].translation());
    body_from_camera_rotations.push_back(body_from_cameras[i].rotationMatrix());
  }
}

NonCentralRelativePoseSolution NonCentralRelativePoseSolverOld::solve(
    const BearingVectorCorrespondencesOld &bvcs, int ransac_runs) {
  // Convert bearing and correspondence vectors format for the multi-camera
  // solver
  MultiBearingVectors m_bv1, m_bv2;

  for (int c = 0; c < bvcs.number_of_cameras; c++) {
    m_bv1.push_back(std::make_shared<opengv::bearingVectors_t>());
    m_bv2.push_back(std::make_shared<opengv::bearingVectors_t>());
  }
  for (int i = 0; i < bvcs.number_of_correspondences; i++) {
    m_bv1[bvcs.first_correspondences[i]]->push_back(
        bvcs.first_bearing_vectors[i]);
    m_bv2[bvcs.second_correspondences[i]]->push_back(
        bvcs.second_bearing_vectors[i]);
  }
  return solve(m_bv1, m_bv2, ransac_runs);
}

// Just a convenience definition, should not use this one since doesn't do any
// checks
NonCentralRelativePoseSolution NonCentralRelativePoseSolverOld::solve(
    const opengv::bearingVectors_t &first_bearing_vectors,
    const opengv::bearingVectors_t &second_bearing_vectors,
    const std::vector<int> &first_correspondences,
    const std::vector<int> &second_correspondences) {
  // Convert bearing and correspondence vectors format for the multi-camera
  // solver
  MultiBearingVectors m_bv1, m_bv2;

  for (int c = 0; c < extrinsics_.number_of_cameras; c++) {
    m_bv1.push_back(std::make_shared<opengv::bearingVectors_t>());
    m_bv2.push_back(std::make_shared<opengv::bearingVectors_t>());
  }
  for (int i = 0; i < first_bearing_vectors.size(); i++) {
    // TODO Works only if first_correspondences[i] == second_correspondences[i]
    // Otherwise, the correspondence information is lost
    m_bv1[first_correspondences[i]]->push_back(first_bearing_vectors[i]);
    m_bv2[second_correspondences[i]]->push_back(second_bearing_vectors[i]);
  }

  return solve(m_bv1, m_bv2);
}

// Actual method which solves the relative pose problem using OpenGV's 'multi'
// formulation. It uses multi adapter from OpenGV to evenly sample
// correspondences across the cameras in the rig
NonCentralRelativePoseSolution NonCentralRelativePoseSolverOld::solve(
    const MultiBearingVectors &first_bearing_vectors,
    const MultiBearingVectors &second_bearing_vectors, int ransac_runs) {
  // Check both frames have the same number of cameras
  if (first_bearing_vectors.size() != second_bearing_vectors.size()) {
    throw std::runtime_error("Number of cameras not equal between frames");
  }
  int num_cameras = first_bearing_vectors.size();

  // choose the solver algorithm
  opengv::sac_problems::relative_pose::MultiNoncentralRelativePoseSacProblem::
      Algorithm algorithm;
  int sample_size;
  switch (settings_.algorithm) {
    // 17pt linear
    case SolverAlgorithm::kSeventeenPoint:
      algorithm = opengv::sac_problems::relative_pose::
          MultiNoncentralRelativePoseSacProblem::SEVENTEENPT;
      // from OpenGV MultiNoncentralRelativePoseSacProblem.cpp
      sample_size = 17;
      LOG(INFO) << "Solver using 17pt algorithm";
      break;

    // Stewenius 6pt, OpenGV takes care of checking the 64 possible solutions
    case SolverAlgorithm::kSixPoint:
      algorithm = opengv::sac_problems::relative_pose::
          MultiNoncentralRelativePoseSacProblem::SIXPT;
      // from OpenGV MultiNoncentralRelativePoseSacProblem.cpp
      sample_size = 6 + 3;
      LOG(INFO) << "Solver using 6pt algorithm";
      break;

    // Generalized Eigensolver
    case SolverAlgorithm::kGeneralizedEigensolver:
      algorithm = opengv::sac_problems::relative_pose::
          MultiNoncentralRelativePoseSacProblem::GE;
      // from OpenGV MultiNoncentralRelativePoseSacProblem.cpp
      sample_size = 8;
      LOG(INFO) << "Solver using Generalized Eigensolver algorithm";
      break;

    default:
      LOG(FATAL) << "unknown solver algorithm";
      break;
  }
  // Checks that we have enough correspondences before solving
  // The multi-camera setup from OpenGV tries to evenly divide up
  // the sample it gets from each camera
  // See MultiNoncentralRelativePoseSacProblem.cpp / getSampleSizes()
  // Also because of the way the multi camera sampling is implemented in OpenGV,
  // it's better to round up and be safe
  int min_cam_correspondences =
      (int)std::ceil(((float)sample_size / (float)num_cameras));

  int total_correspondences = 0;
  for (const auto &c : first_bearing_vectors) {
    if (c->size() < min_cam_correspondences) {
      // TODO cleanup fmt
      std::stringstream error_msg;
      error_msg << "A camera cannot does not have the "
                << min_cam_correspondences
                << " required minimum correspondences";
      throw std::runtime_error(error_msg.str());
    }
    total_correspondences += c->size();
  }
  if (total_correspondences < sample_size) {
    // TODO cleanup fmt
    std::stringstream error_msg;
    error_msg << "Cameras do not have enough correspondences for the RANSAC "
                 "sample size of "
              << sample_size;
    throw std::runtime_error(error_msg.str());
  }

  // create the non-central MULTI relative adapter which holds the data
  opengv::relative_pose::NoncentralRelativeMultiAdapter adapter(
      first_bearing_vectors, second_bearing_vectors,
      extrinsics_.body_from_camera_translations,
      extrinsics_.body_from_camera_rotations);

  // create a MultiNoncentralRelativePoseSacProblem with RANSAC
  opengv::sac::MultiRansac<opengv::sac_problems::relative_pose::
                               MultiNoncentralRelativePoseSacProblem>
      ransac;

  std::vector<int> number_of_correspondences;
  for (size_t i = 0; i < adapter.getNumberPairs(); i++) {
    number_of_correspondences.push_back(adapter.getNumberCorrespondences(i));
  }

  std::shared_ptr<opengv::sac_problems::relative_pose::
                      MultiNoncentralRelativePoseSacProblem>
      relative_pose_problem_ptr(
          new opengv::sac_problems::relative_pose::
              MultiNoncentralRelativePoseSacProblem(adapter, algorithm));

  // run ransac with the set parameters
  ransac.sac_model_ = relative_pose_problem_ptr;
  ransac.threshold_ = settings_.ransac_threshold;
  ransac.max_iterations_ = settings_.ransac_max_iter;
  ransac.probability_ = settings_.ransac_probability;

  StdVectorA<Eigen::Vector3d> translation_estimates;
  StdVectorA<SO3> rotation_estimates;

  for (int i = 0; i < ransac_runs; i++) {
    ransac.computeModel(settings_.ransac_verbosity_level);

    // // OpenGV only counts the inliers, but actually doesn't refine the ransac
    // solution
    // // with all the inliers found...
    // // Need to do that ourselves
    // opengv::sac_problems::relative_pose::
    //   MultiNoncentralRelativePoseSacProblem::model_t model_all_inliers;
    // ransac.sac_model_->computeModelCoefficients(ransac.inliers_,
    // model_all_inliers);

    /* DON'T DO THIS FOR NOW, since it seems like it actually makes the estimate
       worse. I think the reason this is happening is because of imbalance of
       number of features between the cameras. Basically, we can find a lot of
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

    if (settings_.solver_verbose) {
      LOG(INFO) << "ran ransac, inliers this time "
                << CountMultiCamInliers(ransac.inliers_) << std::endl
                << ransac.model_coefficients_ << std::endl;
    }
  }

  // Returns a custom optional type, which we have to dereference to get the
  // value
  auto mean_rotation = *Sophus::average(rotation_estimates);
  Eigen::Vector3d mean_translation;
  for (const auto &t : translation_estimates) {
    mean_translation += (t / ransac_runs);
  }

  // Get the resulting estimated relative pose from the viewpoint 2 to
  // the viewpoint 1, and convert to SE3
  SE3 first_from_second(mean_rotation, mean_translation);

  if (settings_.solver_verbose) {
    LOG(INFO) << "average rotation matrix" << std::endl
              << mean_rotation.matrix() << std::endl;
    LOG(INFO) << "average translation" << std::endl
              << mean_translation << std::endl;
    LOG(INFO) << "relative pose estimate " << std::endl
              << first_from_second.matrix() << std::endl;
  }

  // Get inliers of the averaged model
  opengv::sac_problems::relative_pose::MultiNoncentralRelativePoseSacProblem::
      model_t averaged_model;
  averaged_model.block<3, 3>(0, 0) = first_from_second.rotationMatrix();
  averaged_model.col(3) = first_from_second.translation();

  std::vector<std::vector<int>> inliers;
  ransac.sac_model_->selectWithinDistance(averaged_model,
                                          settings_.ransac_threshold, inliers);

  // Triangulate 3D points expressed in viewpoint 1 coordinates
  opengv::points_t triangulated_points_vp1 =
      triangulate(first_bearing_vectors, second_bearing_vectors,
                  first_from_second, inliers);

  NonCentralRelativePoseSolution solution;
  solution.number_of_inliers = CountMultiCamInliers(inliers);
  solution.first_frame_from_second_frame = first_from_second;
  solution.triangulated_points = triangulated_points_vp1;
  return solution;
}

opengv::points_t NonCentralRelativePoseSolverOld::triangulate(
    const MultiBearingVectors &first_bearing_vectors,
    const MultiBearingVectors &second_bearing_vectors,
    const SE3 &first_frame_from_second_frame,
    const std::vector<std::vector<int>> &inliers) {
  // Triangulation in OpenGV is only implemented for the central case
  // Meaning we have to triangulate points for each camera seperately,
  // and then transform them to the "world" (vp1) frame

  std::vector<opengv::bearingVectors_t> first_camera_bv(
      extrinsics_.number_of_cameras);
  std::vector<opengv::bearingVectors_t> second_camera_bv(
      extrinsics_.number_of_cameras);

  for (int c = 0; c < inliers.size(); c++) {
    for (int i = 0; i < inliers[c].size(); i++) {
      opengv::bearingVector_t first_bv =
          first_bearing_vectors[c]->at(inliers[c][i]);

      opengv::bearingVector_t second_bv =
          second_bearing_vectors[c]->at(inliers[c][i]);

      first_camera_bv[c].push_back(first_bv);
      second_camera_bv[c].push_back(second_bv);
    }
  }

  // Get all triangulated points expressed in the body frame of the
  // first viewpoint
  opengv::points_t points_first_viewpoint;
  for (int c = 0; c < extrinsics_.number_of_cameras; c++) {
    // get transform from cam in view 2 to cam in view 1
    SE3 viewpoint_from_camera = extrinsics_.body_from_cameras[c];
    SE3 first_camera_from_second_camera = viewpoint_from_camera.inverse() *
                                          first_frame_from_second_frame *
                                          viewpoint_from_camera;

    // pass in the estimated relative transformation from ransac
    opengv::relative_pose::CentralRelativeAdapter triangulate_adapter(
        first_camera_bv[c], second_camera_bv[c],
        first_camera_from_second_camera.translation(),
        first_camera_from_second_camera.rotationMatrix());

    for (int p = 0; p < first_camera_bv[c].size(); p++) {
      // OpenGV functions only triangulate one point at a time
      // and return the point triangulated in the coordinate frame of the first
      // camera
      opengv::point_t point_cam =
          opengv::triangulation::triangulate(triangulate_adapter, p);
      opengv::point_t point_curr = viewpoint_from_camera * point_cam;
      points_first_viewpoint.push_back(point_curr);
    }
  }

  return points_first_viewpoint;
}

int CountMultiCamInliers(std::vector<std::vector<int>> &inliers) {
  int num_inliers = 0;
  for (const auto &cam_inliers : inliers) {
    num_inliers += cam_inliers.size();
  }
  return num_inliers;
}

}  // namespace grpose