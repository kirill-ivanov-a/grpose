#ifndef GRPOSE_METRICS_
#define GRPOSE_METRICS_

#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "trajectory.h"
#include "types.h"

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

}  // namespace grpose

#endif
