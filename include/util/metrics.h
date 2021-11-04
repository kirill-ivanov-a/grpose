#ifndef GRPOSE_UTIL_METRICS_
#define GRPOSE_UTIL_METRICS_

#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>

#include "trajectory.h"
#include "util/types.h"

namespace grpose {

/**
 * @param ground_truth - The groundtruth trajectory.
 * @param estimate - The estimated trajectory.
 * @return - The absolute translation errors between the estimate trajectory and
 * the groundtruth trajectory.
 */
std::vector<double> AbsoluteTranslationError(const Trajectory& ground_truth,
                                             const Trajectory& estimate);

/**
 * @param ground_truth - The groundtruth trajectory.
 * @param estimate - The estimated trajectory.
 * @return - The absolute rotation errors between the estimate trajectory and
 * the groundtruth trajectory.
 */
std::vector<double> AbsoluteRotationError(const Trajectory& ground_truth,
                                          const Trajectory& estimate);

double AbsoluteTranslationError(const SE3& world_from_true_frame,
                                const SE3& world_from_estimate);

double AngularTranslationError(const SE3& world_from_true_frame,
                               const SE3& world_from_estimate,
                               bool degrees = false);

double AbsoluteRotationError(const SE3& world_from_true_frame,
                             const SE3& world_from_estimate,
                             bool degrees = false);

}  // namespace grpose

#endif
