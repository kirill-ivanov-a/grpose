#include "grpose/bearing_vector_correspondences.h"

namespace grpose {

void BearingVectorCorrespondences::Add(const Vector3 &bearing_vector1,
                                       const Vector3 &bearing_vector2,
                                       int camera_index1, int camera_index2) {
  // TODO proper assert
  if (camera_index1 < 0)
    throw std::domain_error(fmt::format(
        "BearingVectorCorrespondences::AddCorrespondence: camera_index {:d}<0",
        camera_index1));
  if (camera_index2 < 0)
    throw std::domain_error(fmt::format(
        "BearingVectorCorrespondences::AddCorrespondence: camera_index {:d}<0",
        camera_index2));

  const double vector1_norm = bearing_vector1.norm();
  const double vector2_norm = bearing_vector2.norm();
  CHECK_GE(vector1_norm, 1e-10);
  CHECK_GE(vector2_norm, 1e-10);

  bearing_vectors_[0].push_back(bearing_vector1 / vector1_norm);
  bearing_vectors_[1].push_back(bearing_vector2 / vector2_norm);
  camera_indices_[0].push_back(camera_index1);
  camera_indices_[1].push_back(camera_index2);
  number_of_cameras_ = std::max(number_of_cameras_, camera_index1 + 1);
  number_of_cameras_ = std::max(number_of_cameras_, camera_index2 + 1);
}

}  // namespace grpose