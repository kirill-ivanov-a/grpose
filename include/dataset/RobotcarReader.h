#ifndef GRPOSE_DATASET_ROBOTCARREADER_
#define GRPOSE_DATASET_ROBOTCARREADER_

#include <filesystem>

#include <opencv2/opencv.hpp>

#include "dataset/DatasetReader.h"

namespace grpose {

struct RobotcarReaderSettings {
  RobotcarReaderSettings();

  CameraModelScaramuzzaSettings camera_settings;

  static constexpr bool default_fill_odometry_gaps = false;
  bool fill_odometry_gaps = default_fill_odometry_gaps;

  static constexpr bool default_correct_rtk = true;
  bool correct_rtk = default_correct_rtk;

  static constexpr double default_projected_time_window = 2;
  double projected_time_window = default_projected_time_window;

  static constexpr int default_max_projected_points = 20'000;
  int max_projected_points = default_max_projected_points;

  static constexpr double default_box_filter_size = 3;
  double box_filter_size = default_box_filter_size;

  static constexpr char default_output_directory[] = "output/default";
  fs::path output_directory = default_output_directory;
};

/**
 * The implementation of `DatasetReader` for Oxford RobotCar dataset.
 */
class RobotcarReader : public DatasetReader {
 public:
  static constexpr int kImageWidth = 1024, kImageHeight = 1024;
  static constexpr int kNumberOfCameras = 3;

  static bool IsRobotcar(const fs::path &segment_directory);

  RobotcarReader(const fs::path &segment_directory,
                 const fs::path &camera_models_directory,
                 const fs::path &extrinsics_directory,
                 const std::optional<fs::path> &rtk_directory = std::nullopt,
                 const RobotcarReaderSettings &settings = {});

  void ProvideMasks(const fs::path &masks_directory);

  int NumberOfFrames() const override;
  int FirstTimestampToIndex(Timestamp timestamp) const override;
  std::vector<Timestamp> TimestampsFromIndex(int frame_index) const override;
  std::vector<fs::path> FrameFiles(int frame_index) const override;
  std::vector<FrameEntry> Frame(int frame_index) const override;
  CameraBundle GetCameraBundle() const override { return camera_bundle_; }
  std::unique_ptr<FrameDepths> GetDepths(int frame_index) const override;
  bool HasWorldFromFrame(int frame_index) const override;
  SE3 WorldFromFrame(int frame_index) const override;
  Trajectory GroundTruthTrajectory() const override;

  SE3 WorldFromBodyAtTimestamp(Timestamp timestamp,
                               bool use_odometry = false) const;

  // LMS Front, LMS Rear and LDMRS -- three lidars that are provided with the
  // dataset; INS -- inertial sensor; Left, Rear, Right -- three cameras.
  std::vector<Vector3> GetLmsFrontCloud(Timestamp from, Timestamp to,
                                        Timestamp base) const;
  std::vector<Vector3> GetLmsRearCloud(Timestamp from, Timestamp to,
                                       Timestamp base) const;
  std::vector<Vector3> GetLdmrsCloud(Timestamp from, Timestamp to,
                                     Timestamp base) const;

  std::array<StdVectorA<std::pair<Vector2, double>>, kNumberOfCameras> project(
      Timestamp from, Timestamp to, Timestamp base, bool use_lms_front = true,
      bool use_lms_rear = true, bool use_ldmrs = true) const;
  std::array<StdVectorA<std::pair<Vector2, double>>, kNumberOfCameras> project(
      const std::vector<Vector3> &cloud) const;

  inline const std::vector<Timestamp> &lms_front_timestamps() const {
    return lms_front_timestamps_;
  }
  inline const StdVectorA<SE3> &ground_truth_world_from_body() const {
    return ground_truth_world_from_body_;
  }
  inline bool are_masks_provided() const { return are_masks_provided_; }
  Timestamp min_timestamp() const;
  Timestamp max_timestamp() const;

 private:
  static const SE3 kImageFromCamera;

  static CameraBundle CreateCameraBundle(
      const fs::path &models_directory, const SE3 &left_from_body,
      const SE3 &rear_from_body, const SE3 &right_from_body, int w, int h,
      const CameraModelScaramuzzaSettings &camera_settings);

  void SyncTimestamps();

  void GetPointCloudHelper(const fs::path &scan_directory,
                           const SE3 &body_from_sensor, Timestamp base,
                           const std::vector<Timestamp> &timestamps,
                           Timestamp from, Timestamp to, bool is_ldmrs,
                           std::vector<Vector3> &cloud) const;

  void PrintOdometryAndGroundTruth(const fs::path &output_directory) const;

  SE3 left_from_body_, rear_from_body_, right_from_body_;
  SE3 lms_front_from_body_, lms_rear_from_body_;
  SE3 ldmrs_from_body_;
  SE3 ins_from_body_;
  StdVectorA<SE3> ground_truth_world_from_body_;
  StdVectorA<SE3> odometry_world_from_body_;
  CameraBundle camera_bundle_;
  fs::path segment_directory_;
  fs::path left_directory_, rear_directory_, right_directory_;
  fs::path lms_front_directory_, lms_rear_directory_;
  fs::path ldmrs_directory_;
  std::vector<Timestamp> left_timestamps_, rear_timestamps_, right_timestamps_;
  std::vector<Timestamp> lms_front_timestamps_, lms_rear_timestamps_;
  std::vector<Timestamp> ldmrs_timestamps_;
  std::vector<Timestamp> ground_truth_pose_timestamps_;
  std::vector<Timestamp> odometry_pose_timestamps_;
  bool are_masks_provided_;
  RobotcarReaderSettings settings_;
};

}  // namespace grpose

#endif
