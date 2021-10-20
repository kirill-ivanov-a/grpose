#include "features/colmap_database.h"

namespace grpose {

ColmapDatabase::ColmapDatabase(
    const fs::path &path, const fs::path &matches_database_image_root,
    const std::shared_ptr<DatasetReader> &dataset_reader)
    : database_(path.string()),
      database_image_root_(matches_database_image_root),
      dataset_reader_(dataset_reader) {
  LOG(INFO) << "Opened a database under " << path.string();
}

CentralPoint2dCorrespondences ColmapDatabase::GetCentralCorrespondences(
    int frame_index1, int camera_index1, int frame_index2,
    int camera_index2) const {
  colmap::image_t colmap_indices[2] = {
      ColmapFromOurIndices(frame_index1, camera_index1),
      ColmapFromOurIndices(frame_index2, camera_index2)};
  colmap::FeatureKeypoints keypoints[2] = {
      database_.ReadKeypoints(colmap_indices[0]),
      database_.ReadKeypoints(colmap_indices[1])};
  colmap::FeatureMatches matches =
      database_.ReadMatches(colmap_indices[0], colmap_indices[1]);

  CentralPoint2dCorrespondences correspondences;
  for (const colmap::FeatureMatch &match : matches) {
    CHECK_LT(match.point2D_idx1, keypoints[0].size());
    CHECK_LT(match.point2D_idx2, keypoints[1].size());
    const auto &kp1 = keypoints[0][match.point2D_idx1];
    const auto &kp2 = keypoints[1][match.point2D_idx2];

    correspondences.Add(Vector2(kp1.x, kp1.y), Vector2(kp2.x, kp2.y));
  }
  return correspondences;
}

colmap::image_t ColmapDatabase::ColmapFromOurIndices(int frame_index,
                                                     int camera_index) const {
  fs::path frame_path = dataset_reader_->FrameFiles(frame_index)[camera_index];
  frame_path = fs::absolute(frame_path);
  fs::path colmap_path;
  CHECK(GetRelativePath(database_image_root_, frame_path, colmap_path))
      << fmt::format("The path {} is not contained in the directory {}!",
                     frame_path.string(), colmap_path.string());

  // TODO: reading all image info to only get the id is inefficient
  colmap::Image colmap_image =
      database_.ReadImageWithName(colmap_path.string());
  return colmap_image.ImageId();
}

}  // namespace grpose