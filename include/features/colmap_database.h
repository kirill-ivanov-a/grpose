#ifndef GRPOSE_FEATURES_COLMAP_DATABASE_
#define GRPOSE_FEATURES_COLMAP_DATABASE_

#include <colmap_extracts/base/database.h>
#include <colmap_extracts/util/types.h>

#include "central/central_point2d_correspondences.h"
#include "dataset/autovision_reader.h"

namespace grpose {

class ColmapDatabase {
 public:
  ColmapDatabase(const fs::path &matches_database_path,
                 const fs::path &matches_database_image_root,
                 const std::shared_ptr<DatasetReader> &dataset_reader);
  ~ColmapDatabase();

  CentralPoint2dCorrespondences GetCentralCorrespondences(
      int frame_index1, int camera_index1, int frame_index2,
      int camera_index2) const;

  void SetDatasetReader(const std::shared_ptr<DatasetReader> &dataset_reader);

 private:
  colmap::image_t ColmapFromOurIndices(int frame_index, int camera_index) const;

  colmap::Database database_;
  fs::path database_image_root_;
  std::shared_ptr<DatasetReader> dataset_reader_;
};

}  // namespace grpose

#endif