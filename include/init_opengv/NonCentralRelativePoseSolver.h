#ifndef GRPOSE_INITOPENGV_NONCENTRALRELATIVEPOSESOLVER_
#define GRPOSE_INITOPENGV_NONCENTRALRELATIVEPOSESOLVER_

#include <glog/logging.h>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>
#include <opengv/types.hpp>

#include <opengv/relative_pose/NoncentralRelativeMultiAdapter.hpp>
#include <opengv/sac/MultiRansac.hpp>
#include <opengv/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.hpp>

#include "init_opengv/FeatureDetectorMatcher.h"
#include "init_opengv/MultiCamInitSettings.h"
#include "types.h"

namespace grpose {

struct NonCentralRelativePoseSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int number_of_inliers;
  SE3 first_frame_from_second_frame;
  // expressed in the coordinates of the first frame
  opengv::points_t triangulated_points;

  SE3 ground_truth_relative_pose;
  std::pair<Timestamp, Timestamp> timestamps;

  enum Status { kSuccessful, kFailed, kPending, kNoGroundTruth };
  Status status;
};

class NonCentralRelativePoseSolver {
 public:
  using SolverAlgorithm = NonCentralRelativePoseSolverSettings::SolverAlgorithm;
  using MultiBearingVectors =
      std::vector<std::shared_ptr<opengv::bearingVectors_t>>;

  NonCentralRelativePoseSolver(
      const NonCentralRelativePoseSolverSettings &settings,
      const opengv::translations_t &body_from_camera_translations,
      const opengv::rotations_t &body_from_camera_rotations);

  NonCentralRelativePoseSolver(
      const NonCentralRelativePoseSolverSettings &settings,
      const StdVectorA<SE3> &body_from_cameras);

  // Solve directly using output struct of correspondence finding module
  NonCentralRelativePoseSolution solve(const BearingVectorCorrespondences &bvcs,
                                       int ransac_runs = 1);

  // Just a convenience definition, should not use this one since doesn't do any
  // checks
  NonCentralRelativePoseSolution solve(
      const opengv::bearingVectors_t &first_bearing_vectors,
      const opengv::bearingVectors_t &second_bearing_vectors,
      const std::vector<int> &first_correspondences,
      const std::vector<int> &second_correspondences);

  // Actual method which solves the relative pose problem using OpenGV's 'multi'
  // formulation
  NonCentralRelativePoseSolution solve(
      const MultiBearingVectors &first_bearing_vectors,
      const MultiBearingVectors &second_bearing_vectors, int ransac_runs = 1);

 private:
  opengv::points_t triangulate(
      const MultiBearingVectors &first_bearing_vectors,
      const MultiBearingVectors &second_bearing_vectors,
      const SE3 &first_frame_from_second_frame,
      const std::vector<std::vector<int>> &inliers);

  struct Extrinsics {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int number_of_cameras;
    StdVectorA<SE3> body_from_cameras;
    // convenience members, represents the same SE3
    // object just in OpenGV format
    opengv::translations_t body_from_camera_translations;
    opengv::rotations_t body_from_camera_rotations;

    //  offsets are the camera origin in the viewpoint frame
    //  rotations are from the camera frame to viewpoint frame
    Extrinsics(const opengv::translations_t &body_from_camera_translations,
               const opengv::rotations_t &body_from_camera_rotations);

    /*
      Assumes SE3s gives the transform from camera frame to viewpoint frame
    */
    Extrinsics(const StdVectorA<SE3> &body_from_cameras);

  } extrinsics_;

  NonCentralRelativePoseSolverSettings settings_;
};

int CountMultiCamInliers(std::vector<std::vector<int>> &inliers);

}  // namespace grpose

#endif