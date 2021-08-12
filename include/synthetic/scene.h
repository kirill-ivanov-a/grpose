#ifndef GRPOSE_SYNTHETIC_SCENE_
#define GRPOSE_SYNTHETIC_SCENE_

#include "grpose/bearing_vector_correspondences.h"

namespace grpose::synthetic {

class Scene {
 public:
  virtual ~Scene();

  virtual BearingVectorCorrespondences GetBearingVectorCorrespondences(
      int number_of_correspondences, double cross_camera_fraction = 0.0,
      unsigned long random_seed = 42) const = 0;

  virtual SE3 GetWorldFromBody(int frame_index) const = 0;
  virtual StdVectorA<SE3> GetBodyFromCameras() const = 0;
};

}  // namespace grpose::synthetic

#endif