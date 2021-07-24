#ifndef GRPOSE_DATASET_MULTICAMREADER_
#define GRPOSE_DATASET_MULTICAMREADER_

#include "DatasetReader.h"

namespace grpose {

struct MultiCamReaderSettings {
  static const std::vector<std::string> default_camera_names;
  std::vector<std::string> camera_names = default_camera_names;

  int NumberOfCameras() const;
};

/**
 * The implementation of `DatasetReader` for the modified Multi-FoV dataset
 * (a.k.a. MultiCam) that was generated for a surround-view-like 4 camera
 * system.
 */
class MultiCamReader : public DatasetReader {
  static constexpr int kNumberOfFrames = 3000;
  static constexpr int kImageWidth = 640, kImageHeight = 480;

  // We use a fixed step between images, and we postulate that it roughly equals
  // 30 frames per second. The whole dataset thus represents 1.5 minutes of
  // driving with really sharp turns without the decrease in speed.
  static constexpr Timestamp kFrameTimestampStep = 30000;

 public:
  static bool IsMultiCam(const fs::path &dataset_directory);

  MultiCamReader(const fs::path &dataset_directory,
                 const MultiCamReaderSettings &settings = {});

  int NumberOfFrames() const override;

  int FirstTimestampToIndex(Timestamp timestamp) const override;

  std::vector<Timestamp> TimestampsFromIndex(int frame_index) const override;

  std::vector<fs::path> FrameFiles(int frame_index) const override;

  std::vector<FrameEntry> Frame(int frame_index) const override;

  CameraBundle GetCameraBundle() const override;

  std::unique_ptr<FrameDepths> GetDepths(int frame_index) const override;

  bool HasWorldFromFrame(int frame_index) const override;

  SE3 WorldFromFrame(int frame_index) const override;

  Trajectory GroundTruthTrajectory() const override;

 private:
  class Depths : public FrameDepths {
   public:
    Depths(const fs::path &dataset_directory, int frame_index,
           const MultiCamReaderSettings &settings);

    std::optional<double> Depth(int camera_index,
                                const Vector2 &point) const override;

   private:
    std::vector<cv::Mat1f> depths_;
    static const Eigen::AlignedBox2d bounding_box_;

    MultiCamReaderSettings settings_;
  };

  static CameraBundle CreateCameraBundle(
      const fs::path &dataset_directory,
      const std::vector<std::string> &camera_names);

  static StdVectorA<SE3> ReadWorldFromBody(const fs::path &dataset_directory);

  MultiCamReaderSettings settings_;
  fs::path dataset_directory_;
  CameraBundle camera_bundle_;
  StdVectorA<SE3> world_from_body_;
};

}  // namespace grpose

#endif
