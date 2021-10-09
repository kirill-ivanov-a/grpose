#include "central/central_bearing_vector_correspondences.h"

namespace grpose {

void CentralBearingVectorCorrespondences::Add(const Vector3& bearing_vector1,
                                              const Vector3& bearing_vector2) {
  const double vector1_norm = bearing_vector1.norm();
  const double vector2_norm = bearing_vector2.norm();
  CHECK_GE(vector1_norm, 1e-10);
  CHECK_GE(vector2_norm, 1e-10);

  bearing_vectors_[0].push_back(bearing_vector1 / vector1_norm);
  bearing_vectors_[1].push_back(bearing_vector2 / vector2_norm);
}

const Vector3& CentralBearingVectorCorrespondences::bearing_vector(
    int frame_index, int correspondence_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, 2);
  CHECK_GE(correspondence_index, 0);
  CHECK_LT(correspondence_index, Size());

  return bearing_vectors_[frame_index][correspondence_index];
}

}  // namespace grpose
