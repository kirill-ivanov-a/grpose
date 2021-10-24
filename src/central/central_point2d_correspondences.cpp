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
  CHECK_LT(frame_index, 2);
  CHECK_GE(correspondence_index, 0);
  CHECK_LT(correspondence_index, Size());

  return points_[frame_index][correspondence_index];
}

void CentralPoint2dCorrespondences::FilterByMasks(const cv::Mat1b& mask1,
                                                  const cv::Mat1b& mask2) {
  // TODO check what are the conventions for the camera model (+0.5 or no)
  Eigen::AlignedBox2d box1(Vector2(0.5, 0.5),
                           Vector2(mask1.cols - 1, mask1.rows - 1));
  Eigen::AlignedBox2d box2(Vector2(0.5, 0.5),
                           Vector2(mask2.cols - 1, mask2.rows - 1));
  auto it1 = points_[0].begin(), it2 = points_[1].begin();
  auto new_it1 = points_[0].begin(), new_it2 = points_[1].begin();
  for (; it1 != points_[0].end(); ++it1, ++it2) {
    if (box1.contains(*it1) && box2.contains(*it2) && mask1(ToCvPoint(*it1)) &&
        mask2(ToCvPoint(*it2))) {
      *new_it1 = *it1;
      *new_it2 = *it2;
      new_it1++;
      new_it2++;
    }
  }
  points_[0].erase(new_it1, points_[0].end());
  points_[1].erase(new_it2, points_[1].end());
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
