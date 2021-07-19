#ifndef INCLUDE_METRICS
#define INCLUDE_METRICS

#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

#include "trajectory.h"
#include "types.h"

namespace mcam {

/**
 * @param groundtruth - The groundtruth trajectory.
 * @param estimate - The estimated trajectory.
 * @return - The absolute translation errors between the estimate trajectory and
 * the groundtruth trajectory.
 */
std::vector<double> absoluteTranslationError(const Trajectory& groundtruth,
                                             const Trajectory& estimate);

/**
 * @param groundtruth - The groundtruth trajectory.
 * @param estimate - The estimated trajectory.
 * @return - The absolute rotation errors between the estimate trajectory and
 * the groundtruth trajectory.
 */
std::vector<double> absoluteRotationError(const Trajectory& groundtruth,
                                          const Trajectory& estimate);

}  // namespace mcam

#endif
