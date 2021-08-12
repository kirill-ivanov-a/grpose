#include "init_opengv/multicam_init_settings.h"

#include <math.h>

#include <glog/logging.h>

namespace grpose {

NonCentralRelativePoseSolverSettings::NonCentralRelativePoseSolverSettings(
    double focal_length, bool verbose) {
  UpdateRansacThreshold(ransac_reproj_threshold, focal_length);
  solver_verbose = verbose;
  if (solver_verbose == true) {
    LOG(INFO) << "RANSAC reprojection threshold: " << ransac_reproj_threshold
              << " px";
    LOG(INFO) << "Focal length: " << focal_length << " px";
    LOG(INFO) << "Computed RANSAC threshold: " << ransac_threshold;
  }
}

void NonCentralRelativePoseSolverSettings::UpdateRansacThreshold(
    double reproj_threshold, double focal_length) {
  CHECK_GT(reproj_threshold, 0.0);
  CHECK_GT(focal_length, 0.0);
  ransac_threshold = 1.0 - std::cos(std::atan(reproj_threshold / focal_length));
}

}  // namespace grpose