#ifndef GRPOSE_SYNTHETIC_RANDOM_SCENE_
#define GRPOSE_SYNTHETIC_RANDOM_SCENE_

#include "synthetic/scene.h"

namespace grpose::synthetic {

/**
 * Samples random 3D points within a cube of predefined radius around origin and
 * a motion with random 3D rotation and random translation with norm within
 * predefined bounds. Camera poses are sampled randomly in an analogous fashion.
 * TODO Distribution of translations is not quite uniform
 */
class RandomScene : public Scene {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RandomScene(unsigned long random_seed = 42);

  BearingVectorCorrespondences GetBearingVectorCorrespondences(
      int number_of_correspondences, double cross_camera_fraction,
      unsigned long random_seed) const override;
  BearingVectorCorrespondences GetOutlierCorrespondences(
      int number_of_correspondences, double cross_camera_fraction = 0.0,
      unsigned long random_seed = 42) const override;

  int NumberOfCameras() const override;
  SE3 GetWorldFromBody(int frame_index) const override;
  StdVectorA<SE3> GetBodyFromCameras() const override;

 private:
  double min_translation_norm_ = 1.0, max_translation_norm_ = 1.1;
  double min_camera_translation_norm_ = 0.1, max_camera_translation_norm_ = 0.2;
  double max_absolute_point_corrdinate_ = 10.0;
  int number_of_cameras_ = 3;

  StdVectorA<SE3> body_from_cameras_;
  SE3 world_from_body_[2];
};

}  // namespace grpose::synthetic

#endif
