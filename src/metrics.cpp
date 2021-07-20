#include "metrics.h"

#include <glog/logging.h>
#include <iostream>
#include <vector>

#include "types.h"

namespace grpose {

std::vector<double> absoluteTranslationError(const Trajectory& groundtruth,
                                             const Trajectory& estimate) {
  // Align the estimate with the groundtruth.
  const Trajectory estimate_aligned = estimate.alignTo(groundtruth);

  std::vector<double> errors;
  for (const Timestamp est_timestamp : estimate_aligned.timestamps()) {
    const SE3& world_from_est =
        estimate_aligned.worldFromFrameAt(est_timestamp);
    const SE3 world_from_gt = groundtruth.worldFromFrameAt(est_timestamp);

    const SE3 gt_from_est = world_from_gt.inverse() * world_from_est;
    errors.emplace_back(gt_from_est.translation().norm());
  }

  return errors;
}

std::vector<double> absoluteRotationError(const Trajectory& groundtruth,
                                          const Trajectory& estimate) {
  // Align the estimate with the groundtruth.
  const Trajectory estimate_aligned = estimate.alignTo(groundtruth);

  std::vector<double> errors;
  for (const Timestamp est_timestamp : estimate_aligned.timestamps()) {
    const SE3& world_from_est =
        estimate_aligned.worldFromFrameAt(est_timestamp);
    const SE3 world_from_gt = groundtruth.worldFromFrameAt(est_timestamp);

    errors.emplace_back(
        (world_from_gt.so3().inverse() * world_from_est.so3()).log().norm());
  }

  return errors;
}

}  // namespace grpose
