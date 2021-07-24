#include <iostream>
#include <vector>

#include <glog/logging.h>

#include "metrics.h"
#include "types.h"

using namespace grpose;

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);

  CHECK_EQ(argc, 3);
  fs::path gt_traj_fpath = argv[1];
  fs::path est_traj_fpath = argv[2];

  const Trajectory gt_traj = Trajectory::FromFile(gt_traj_fpath);
  const Trajectory est_traj = Trajectory::FromFile(est_traj_fpath);

  const std::vector<double> ates = AbsoluteTranslationError(gt_traj, est_traj);
  const std::vector<double> ares = AbsoluteRotationError(gt_traj, est_traj);

  std::cout << "Absolute translation errors:" << std::endl << "  ";
  for (const double ate : ates) {
    std::cout << ate << ", ";
  }
  std::cout << std::endl;

  std::cout << "Absolute rotation errors:" << std::endl << "  ";
  for (const double are : ares) {
    std::cout << are << ", ";
  }
  std::cout << std::endl;
}
