#include "central/central_point2d_correspondences.h"

#include <glog/logging.h>

namespace grpose {

void CentralPoint2dCorrespondences::Add(const Vector2& point1,
                                        const Vector2& point2) {
  points_[0].push_back(point1);
  points_[1].push_back(point2);
}

const Vector2& CentralPoint2dCorrespondences::point(
    int frame_index, int correspondence_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, 1);
  CHECK_GE(correspondence_index, 0);
  CHECK_LT(correspondence_index, Size());

  return points_[frame_index][correspondence_index];
}

CentralBearingVectorCorrespondences
CentralPoint2dCorrespondences::ToCentralBearingVectorCorrespondences(
    const Camera& camera1, const Camera& camera2) const {
  CentralBearingVectorCorrespondences result;
  for (int i = 0; i < Size(); ++i) {
    if (camera1.IsOnImage(points_[0][i]) && camera2.IsOnImage(points_[1][i]))
      result.Add(camera1.Unmap(points_[0][i]), camera2.Unmap(points_[1][i]));
  }
  return result;
}

}  // namespace grpose
