#ifndef GRPOSE_APPROXIMATE_CENTRAL_SOLUTION_SAMPLER_
#define GRPOSE_APPROXIMATE_CENTRAL_SOLUTION_SAMPLER_

#include <random>

#include "camera/camera.h"
#include "central/central_opengv_minimal_solver.h"
#include "central/central_point2d_correspondences.h"

namespace grpose {

struct ApproximateCentralSolutionSamplerSettings {
  using MinimalSolverAlgorithm = opengv::sac_problems::relative_pose::
      CentralRelativePoseSacProblem::Algorithm;

  MinimalSolverAlgorithm algorithm = MinimalSolverAlgorithm::STEWENIUS;

  enum ErrorType {
    // sum of pixel reprojection errors on each of 2 images
    kReprojectionError2D,
    kSumOfAngles,
    kSampson3d,
  } error_type = kReprojectionError2D;

  double threshold = 1.0;

  int max_num_attempts = 50;
};

ApproximateCentralSolutionSamplerSettings::ErrorType ToErrorType(
    const std::string &name);

class ApproximateCentralSolutionSampler {
 public:
  ApproximateCentralSolutionSampler(
      const Camera &camera1, const Camera &camera2,
      const CentralPoint2dCorrespondences &correspondences,
      const SE3 &frame1_from_frame2_true,
      const ApproximateCentralSolutionSamplerSettings &settings);

  inline CentralPoint2dCorrespondences GetInlierCorrespondences() const;

  template <typename RandomBitsGenerator>
  SE3 SampleFrame1FromFrame2(RandomBitsGenerator &generator) const;

 private:
  CentralPoint2dCorrespondences inlier_point_correspondences_;
  CentralBearingVectorCorrespondences inlier_bv_correspondences_;
  ApproximateCentralSolutionSamplerSettings settings_;
  CentralOpengvMinimalSolver minimal_solver_;
  std::vector<int> all_indices_;
};

// Implementation

template <typename RandomBitsGenerator>
SE3 ApproximateCentralSolutionSampler::SampleFrame1FromFrame2(
    RandomBitsGenerator &generator) const {
  const int kSampleSize = minimal_solver_.SampleSize();
  for (int i = 0; i < settings_.max_num_attempts; ++i) {
    std::vector<int> indices;
    indices.reserve(kSampleSize);
    std::sample(all_indices_.begin(), all_indices_.end(),
                std::back_inserter(indices), kSampleSize, generator);
    SE3 frame1_from_frame2;
    if (minimal_solver_.Solve(indices, frame1_from_frame2))
      return frame1_from_frame2;
  }

  LOG(ERROR) << fmt::format(
      "Minimal solver could not return anything after {} attempts",
      settings_.max_num_attempts);
  return SE3();
}

inline CentralPoint2dCorrespondences
ApproximateCentralSolutionSampler::GetInlierCorrespondences() const {
  return inlier_point_correspondences_;
}

}  // namespace grpose

#endif