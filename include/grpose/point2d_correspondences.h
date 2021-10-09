#ifndef GRPOSE_GRPOSE_POINT2D_CORRESPONDENCES_
#define GRPOSE_GRPOSE_POINT2D_CORRESPONDENCES_

#include <glog/logging.h>

#include "util/types.h"

namespace grpose {

class Point2dCorrespondences {
 public:
  Point2dCorrespondences() = default;

  void Add(const Vector2 &point1, const Vector2 &point2, int camera_index1,
           int camera_index2);

  inline int Size() const;
  inline int NumberOfCameras() const;

  inline const Vector2 &point(int frame_index, int correspondence_index) const;
  inline int camera_index(int frame_index, int correspondence_index) const;

 private:
  StdVectorA<Vector2> points_[2];
  std::vector<int> camera_indices_[2];
  int number_of_cameras_ = 0;
};

// Implementation

inline int Point2dCorrespondences::Size() const {
  return static_cast<int>(points_[0].size());
}

inline int Point2dCorrespondences::NumberOfCameras() const {
  return number_of_cameras_;
}

inline const Vector2 &Point2dCorrespondences::point(
    int frame_index, int correspondence_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, 2);
  CHECK_GE(correspondence_index, 0);
  CHECK_LT(correspondence_index, Size());

  return points_[frame_index][correspondence_index];
}

inline int Point2dCorrespondences::camera_index(
    int frame_index, int correspondence_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, 2);
  CHECK_GE(correspondence_index, 0);
  CHECK_LT(correspondence_index, Size());

  return camera_indices_[frame_index][correspondence_index];
}

}  // namespace grpose

#endif