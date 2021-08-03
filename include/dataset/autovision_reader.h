#ifndef GRPOSE_DATASET_AUTOVISION_READER_
#define GRPOSE_DATASET_AUTOVISION_READER_

#include <filesystem>

#include <opencv2/opencv.hpp>

#include "dataset/dataset_reader.h"

namespace grpose {

struct AutovisionReaderSettings {
  AutovisionReaderSettings(const fs::path &config_filename);

  std::vector<std::string> camera_names;
};

/**
 * The implementation of `DatasetReader` for Autovision data segments.
 */
class AutovisionReader : public DatasetReader {
 public:
  static bool IsAutovision(const fs::path &segment_directory);

  AutovisionReader(const fs::path &segment_directory,
                   const fs::path &calibration_directory,
                   const fs::path &config_filename);

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
  CameraBundle CreateCameraBundle(const fs::path &calibration_directory,
                                  const std::vector<std::string> &camera_names);
  void ReadTimestamps(const fs::path &timestamps_filename);
  void ReadGroundTruth(const fs::path &ground_truth_filename);

  AutovisionReaderSettings settings_;
  fs::path segment_directory_;
  CameraBundle camera_bundle_;

  std::vector<int> frame_ids_;
  std::vector<Timestamp> frame_timestamps_;
  StdVectorA<SE3> ground_truth_world_from_body_;
};

}  // namespace grpose

#endif