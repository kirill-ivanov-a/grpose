#ifndef GRPOSE_CENTRAL_CENTRAL_BEARING_VECTOR_CORRESPONDENCES_
#define GRPOSE_CENTRAL_CENTRAL_BEARING_VECTOR_CORRESPONDENCES_

#include "util/types.h"

#include <glog/logging.h>

namespace grpose {

class CentralBearingVectorCorrespondences {
 public:
  CentralBearingVectorCorrespondences() = default;

  void Add(const Vector3 &bearing_vector1, const Vector3 &bearing_vector2);
  const Vector3 &bearing_vector(int frame_index,
                                int correspondence_index) const;
  inline const StdVectorA<Vector3> &bearing_vectors(int frame_index) const;
  inline int Size() const;

 private:
  StdVectorA<Vector3> bearing_vectors_[2];
};

// Implementation

inline int CentralBearingVectorCorrespondences::Size() const {
  return bearing_vectors_[0].size();
}

const StdVectorA<Vector3> &CentralBearingVectorCorrespondences::bearing_vectors(
    int frame_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, 2);
  return bearing_vectors_[frame_index];
}

}  // namespace grpose

#endif