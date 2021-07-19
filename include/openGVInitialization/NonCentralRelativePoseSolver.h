#ifndef INCLUDE_NONCENTRALRELATIVEPOSESOLVER
#define INCLUDE_NONCENTRALRELATIVEPOSESOLVER

#include <glog/logging.h>
#include <types.h>

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

#include "openGVInitialization/FeatureDetectorMatcher.h"
#include "openGVInitialization/MultiCamInitSettings.h"

namespace mcam {

struct NonCentralRelativePoseSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int num_inliers;
  SE3 relative_pose;  // transform from next frame to the current frame
  opengv::points_t
      triangulated_points;  // expressed in the coordinates of the current frame

  SE3 gt_relative_pose;
  std::pair<Timestamp, Timestamp> timestamps;
  enum Status { SUCCESSFUL, FAILED, PENDING, NO_GT };
  Status status;

  // The following are unused for now, but we hope to write std for translation
  // and rotation as well
  //   double std_translation;
  //   double std_rotation;
};

class NonCentralRelativePoseSolver {
 public:
  using SolverAlgorithm = NonCentralRelativePoseSolverSettings::SolverAlgorithm;
  using multiBearingVectors =
      std::vector<std::shared_ptr<opengv::bearingVectors_t>>;

  NonCentralRelativePoseSolver(
      const NonCentralRelativePoseSolverSettings &settings,
      const opengv::translations_t &offsets,
      const opengv::rotations_t &rotations);

  NonCentralRelativePoseSolver(
      const NonCentralRelativePoseSolverSettings &settings,
      const StdVectorA<SE3> &cam_extrinsics);

  // Solve directly using output struct of correspondence finding module
  NonCentralRelativePoseSolution solve(const BearingVectorCorrespondences &bvcs,
                                       int ransac_runs = 1);

  // Just a convenience definition, should not use this one since doesn't do any
  // checks
  NonCentralRelativePoseSolution solve(
      const opengv::bearingVectors_t &bearingVectorsCurrent,
      const opengv::bearingVectors_t &bearingVectorsNext,
      const std::vector<int> &correspondencesCurrent,
      const std::vector<int> &correspondencesNext);

  // Actual method which solves the relative pose problem using OpenGV's 'multi'
  // formulation
  NonCentralRelativePoseSolution solve(
      const multiBearingVectors &bearingVectorsCurrent,
      const multiBearingVectors &bearingVectorsNext, int ransac_runs = 1);

 private:
  opengv::points_t triangulate(const multiBearingVectors &bearingVectorsCurrent,
                               const multiBearingVectors &bearingVectorsNext,
                               const SE3 &currentFromNext,
                               const std::vector<std::vector<int>> &inliers);

  struct Extrinsics {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int num_cameras;
    StdVectorA<SE3> bodyFromCams;
    opengv::translations_t
        cam_offsets;  // convenience members, represents the same SE3
                      // object just in OpenGV format
    opengv::rotations_t cam_rotations;

    /*
      offsets are the camera origin in the viewpoint frame
      rotations are from the camera frame to viewpoint frame
    */
    Extrinsics(const opengv::translations_t &offsets,
               const opengv::rotations_t &rotations);

    /*
      Assumes SE3s gives the transform from camera frame to viewpoint frame
    */
    Extrinsics(const StdVectorA<SE3> &cam_extrinsics);

  } extrinsics;

  NonCentralRelativePoseSolverSettings settings;
};

int countMultiCamInliers(std::vector<std::vector<int>> inliers);

}  // namespace mcam

#endif