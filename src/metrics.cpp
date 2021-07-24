#include "metrics.h"

#include <iostream>
#include <vector>

#include <glog/logging.h>

#include "types.h"

namespace grpose {

std::vector<double> AbsoluteTranslationError(const Trajectory& ground_truth,
                                             const Trajectory& estimate) {
  // Align the estimate with the groundtruth.
  const Trajectory estimate_aligned = estimate.AlignTo(ground_truth);

  std::vector<double> errors;
  for (const Timestamp est_timestamp : estimate_aligned.Timestamps()) {
    const SE3& world_from_est =
        estimate_aligned.WorldFromFrameAt(est_timestamp);
    const SE3 world_from_gt = ground_truth.WorldFromFrameAt(est_timestamp);

    const SE3 gt_from_est = world_from_gt.inverse() * world_from_est;
    errors.emplace_back(gt_from_est.translation().norm());
  }

  return errors;
}

std::vector<double> AbsoluteRotationError(const Trajectory& ground_truth,
                                          const Trajectory& estimate) {
  // Align the estimate with the groundtruth.
  const Trajectory estimate_aligned = estimate.AlignTo(ground_truth);

  std::vector<double> errors;
  for (const Timestamp est_timestamp : estimate_aligned.Timestamps()) {
    const SE3& world_from_est =
        estimate_aligned.WorldFromFrameAt(est_timestamp);
    const SE3 world_from_gt = ground_truth.WorldFromFrameAt(est_timestamp);

    errors.emplace_back(
        (world_from_gt.so3().inverse() * world_from_est.so3()).log().norm());
  }

  return errors;
}

}  // namespace grpose
