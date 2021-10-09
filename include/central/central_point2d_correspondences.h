#ifndef GRPOSE_CENTRAL_CENTRAL_POINT2D_CORRESPONDENCES_
#define GRPOSE_CENTRAL_CENTRAL_POINT2D_CORRESPONDENCES_

#include "camera/camera.h"
#include "central/central_bearing_vector_correspondences.h"
#include "util/types.h"

namespace grpose {

class CentralPoint2dCorrespondences {
 public:
  CentralPoint2dCorrespondences() = default;

  void Add(const Vector2 &point1, const Vector2 &point2);
  const Vector2 &point(int frame_index, int correspondence_index) const;
  inline int Size() const;

  CentralBearingVectorCorrespondences ToCentralBearingVectorCorrespondences(
      const Camera &camera1, const Camera &camera2) const;

 private:
  StdVectorA<Vector2> points_[2];
};

// Implementation

inline int CentralPoint2dCorrespondences::Size() const {
  return points_[0].size();
}

}  // namespace grpose

#endif
