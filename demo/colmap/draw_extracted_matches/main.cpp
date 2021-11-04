#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "features/colmap_database.h"
#include "util/util.h"

DEFINE_int32(
    frame1_ind, 0,
    "Index of the first matched frame in the dataset, starting from 0");
DEFINE_int32(camera1_ind, 0,
             "Index of the camera for the first matched frame in the dataset, "
             "starting from 0");
DEFINE_int32(
    frame2_ind, 30,
    "Index of the second matched frame in the dataset, starting from 0");
DEFINE_int32(camera2_ind, 0,
             "Index of the camera for the second matched frame in the dataset, "
             "starting from 0");
DEFINE_int32(point_radius, 5, "Radius of points to draw, in pixels");
DEFINE_bool(mask_matches, true,
            "Do we need to filter out matches outside camera masks?");

using namespace grpose;

void ShowMatches(const std::shared_ptr<DatasetReader> &dataset_reader,
                 const ColmapDatabase &colmap_database) {
  CentralPoint2dCorrespondences correspondences =
      colmap_database.GetCentralCorrespondences(
          FLAGS_frame1_ind, FLAGS_camera1_ind, FLAGS_frame2_ind,
          FLAGS_camera2_ind);

  if (FLAGS_mask_matches) {
    CameraBundle camera_bundle = dataset_reader->GetCameraBundle();
    cv::Mat1b mask1 = camera_bundle.camera(FLAGS_camera1_ind).mask();
    cv::Mat1b mask2 = camera_bundle.camera(FLAGS_camera2_ind).mask();
    correspondences.FilterByMasks(mask1, mask2);
  }

  cv::Mat3b frames[2] = {
      dataset_reader->Frame(FLAGS_frame1_ind)[FLAGS_camera1_ind].frame,
      dataset_reader->Frame(FLAGS_frame2_ind)[FLAGS_camera2_ind].frame};

  cv::Mat3b result_big =
      DrawMatches(correspondences, frames[0], frames[1], FLAGS_point_radius);
  cv::Mat3b result;
  cv::resize(result_big, result, cv::Size(), 0.5, 0.5);
  cv::imshow("Extracted matches", result);
  cv::waitKey();
}

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./draw_extracted_matches autovision_segment_dir autovision_calib_dir autovision_config database database_root
where
  autovision_segment_dir is the path to an AutoVision segment directory
  autovision_calib_dir is the path to the calibration directory
  autovision_config is the path to the configuration JSON file
  database is the path to the COLMAP database containing matches
  database_root is the directory that was used to produce the database.
      )abacaba";

  fs::path output_directory =
      fs::path("output") / ("draw_extracted_maches_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;
  SaveArgv(output_directory / "argv.txt", argc, argv);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  CHECK_EQ(argc, 6) << usage;
  fs::path autovision_segment_dir = argv[1];
  fs::path autovision_calib_dir = argv[2];
  fs::path autovision_config = argv[3];
  fs::path database_path = argv[4];
  fs::path database_root = argv[5];

  std::shared_ptr<DatasetReader> dataset_reader =
      std::make_shared<AutovisionReader>(
          autovision_segment_dir, autovision_calib_dir, autovision_config);
  ColmapDatabase colmap_database(database_path, database_root, dataset_reader);

  ShowMatches(dataset_reader, colmap_database);

  return 0;
}
