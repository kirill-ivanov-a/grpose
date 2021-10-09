#include "grpose/point2d_correspondences.h"

namespace grpose {

void Point2dCorrespondences::Add(const Vector2 &point1, const Vector2 &point2,
                                 int camera_index1, int camera_index2) {
  CHECK_GE(camera_index1, 0);
  CHECK_GE(camera_index2, 0);

  points_[0].push_back(point1);
  points_[1].push_back(point2);
  camera_indices_[0].push_back(camera_index1);
  camera_indices_[1].push_back(camera_index2);
  number_of_cameras_ = std::max(number_of_cameras_, camera_index1 + 1);
  number_of_cameras_ = std::max(number_of_cameras_, camera_index2 + 1);
}

}  // namespace grpose