#include <opencv2/imgproc.hpp>

#include "dataset/robotcar_reader.h"
#include "util.h"

DEFINE_string(models_dir, "data/models/robotcar",
              "Directory with omnidirectional camera models. It is provided in "
              "our repository. IT IS NOT THE \"models\" DIRECTORY FROM THE "
              "ROBOTCAR DATASET SDK!");
DEFINE_string(extrinsics_dir, "ext/robotcar-dataset-sdk/extrinsics",
              "Directory with RobotCar dataset sensor extrinsics, provided in "
              "the dataset SDK.");
DEFINE_string(masks_dir, "data/masks/robotcar",
              "Directory with RobotCar dataset masks.");
DEFINE_bool(gen_clouds, true,
            "Do we need to generate whole clouds from the dump?");
DEFINE_string(
    out_front, "lidar_front.ply",
    "Output file name, to which the cloud from lms_front will be written "
    "in PLY format.");
DEFINE_string(
    out_rear, "lidar_rear.ply",
    "Output file name, to which the cloud from lms_rear will be written "
    "in PLY format.");
DEFINE_string(out_ldmrs, "lidar_ldmrs.ply",
              "Output file name, to which the cloud from ldmrs will be written "
              "in PLY format.");
DEFINE_string(out_traj_orig, "vo_orig.txt",
              "The file to write original VO trajectory into.");
DEFINE_string(out_traj_interp, "vo_interp.txt",
              "The file to write interpolated VO trajectory into.");
DEFINE_string(
    gt_traj, "gt_traj_tum.txt",
    "The file to write the ground truth trajectory in the TUM format into.");
DEFINE_int32(interp_gran, 1000,
             "Number of poses to be output into interpolated vo trajectory");
DEFINE_int32(max_proj_points, 20'000, "Maximum number of projected points.");
DEFINE_int32(cloud_points, 20'000'000,
             "Number of points to be written into the cloud");
DEFINE_bool(fill_vo_gaps, false,
            "Do we need to fill the gaps in VO trajectory?");
DEFINE_string(
    out_project, "projected.png",
    "Name of the file to output the image with the projected cloud into.");
DEFINE_int32(project_idx, 200,
             "Index of the frame to project point cloud onto.");
DEFINE_double(
    time_win, 5,
    "Lidar scans from the time window [ts - time_win, ts + time_win] will be "
    "used to draw the projected cloud. Here ts is the time, corresponding to "
    "project_idx frame. time_win is measured in seconds.");
DEFINE_double(
    rel_point_size, 0.001,
    "Relative to w+h point size on the images with projected points.");

using namespace grpose;

cv::Mat3b Project(const RobotcarReader &reader, int index) {
  std::cout << "generating projections..." << std::endl;

  auto frame = reader.Frame(index);
  Timestamp base_timestamp = frame[0].timestamp;
  Timestamp time_window = FLAGS_time_win * 1e6;
  Timestamp min_timestamp = base_timestamp - time_window,
            max_timestamp = base_timestamp + time_window;
  std::vector<Vector3> lms_front_cloud =
      reader.GetLmsFrontCloud(min_timestamp, max_timestamp, base_timestamp);
  std::vector<Vector3> lms_rear_cloud =
      reader.GetLmsRearCloud(min_timestamp, max_timestamp, base_timestamp);
  std::vector<Vector3> ldmrs_cloud =
      reader.GetLdmrsCloud(min_timestamp, max_timestamp, base_timestamp);
  std::vector<Vector3> cloud;
  cloud.reserve(lms_front_cloud.size() + lms_rear_cloud.size() +
                ldmrs_cloud.size());
  cloud.insert(cloud.end(), lms_front_cloud.begin(), lms_front_cloud.end());
  cloud.insert(cloud.end(), lms_rear_cloud.begin(), lms_rear_cloud.end());
  cloud.insert(cloud.end(), ldmrs_cloud.begin(), ldmrs_cloud.end());
  cv::Mat3b images[RobotcarReader::kNumberOfCameras];

  int square_size =
      FLAGS_rel_point_size *
      (RobotcarReader::kImageWidth + RobotcarReader::kImageHeight) / 2;

  CameraBundle camera_bundle = reader.GetCameraBundle();
  StdVectorA<Vector2> points[RobotcarReader::kNumberOfCameras];
  std::vector<double> depths[RobotcarReader::kNumberOfCameras];
  std::vector<double> all_depths;
  for (int ci = 0; ci < RobotcarReader::kNumberOfCameras; ++ci) {
    SE3 camera_from_body = camera_bundle.camera_from_body(ci);
    for (const Vector3 &p : cloud) {
      Vector3 moved = camera_from_body * p;
      if (!reader.GetCameraBundle().camera(ci).IsMappable(moved)) continue;

      double depth = moved.norm();
      Vector2 projected = reader.GetCameraBundle().camera(ci).Map(moved);
      depths[ci].push_back(depth);
      all_depths.push_back(depth);
      points[ci].push_back(projected);
    }
  }
  std::sort(all_depths.begin(), all_depths.end());
  double min_depth = all_depths[(int)(0.05 * all_depths.size())];
  double max_depth = all_depths[(int)(0.90 * all_depths.size())];

  for (int ci = 0; ci < RobotcarReader::kNumberOfCameras; ++ci) {
    CHECK(frame[ci].frame.channels() == 3);
    std::vector<cv::Vec3b> colors =
        GetColors(depths[ci], min_depth, max_depth, cv::COLORMAP_JET);
    images[ci] = frame[ci].frame.clone();
    for (int j = 0; j < points[ci].size(); ++j)
      DrawSquare(images[ci], ToCvPoint(points[ci][j]), square_size, colors[j],
                 cv::FILLED);
    cv::bitwise_and(images[ci],
                    ConvertGrayToBgr(camera_bundle.camera(ci).mask()),
                    images[ci]);
  }

  cv::Mat3b result;
  cv::hconcat(images, RobotcarReader::kNumberOfCameras, result);
  return result;
}

void GenerateClouds(const RobotcarReader &reader, Timestamp min_timestamp,
                    Timestamp max_timestamp, Timestamp base_timestamp,
                    const fs::path &output_directory) {
  constexpr int kTotalClouds = 3;

  std::cout << "generating clouds..." << std::endl;

  std::vector<Vector3> lms_front_cloud =
      reader.GetLmsFrontCloud(min_timestamp, max_timestamp, base_timestamp);
  std::vector<Vector3> lms_rear_cloud =
      reader.GetLmsRearCloud(min_timestamp, max_timestamp, base_timestamp);
  std::vector<Vector3> ldmrs_cloud =
      reader.GetLdmrsCloud(min_timestamp, max_timestamp, base_timestamp);
  std::vector<Vector3> *clouds[kTotalClouds] = {&lms_front_cloud,
                                                &lms_rear_cloud, &ldmrs_cloud};
  LOG(INFO) << "total...";
  LOG(INFO) << "points from LMS front: " << lms_front_cloud.size();
  LOG(INFO) << "points from LMS rear : " << lms_rear_cloud.size();
  LOG(INFO) << "points from LDMRS    : " << ldmrs_cloud.size();

  SparsifyVectors(clouds, kTotalClouds, FLAGS_cloud_points);

  LOG(INFO) << "filtered...";
  LOG(INFO) << "points from LMS front: " << lms_front_cloud.size();
  LOG(INFO) << "points from LMS rear : " << lms_rear_cloud.size();
  LOG(INFO) << "points from LDMRS    : " << ldmrs_cloud.size();

  std::cout << "saving clouds..." << std::endl;
  const cv::Vec3b gray(128, 128, 128);
  std::ofstream ofs_front(output_directory / FLAGS_out_front);
  PrintInPly(ofs_front, lms_front_cloud,
             std::vector<cv::Vec3b>(lms_front_cloud.size(), gray));
  std::ofstream ofs_rear(output_directory / FLAGS_out_rear);
  PrintInPly(ofs_rear, lms_rear_cloud,
             std::vector<cv::Vec3b>(lms_rear_cloud.size(), gray));
  std::ofstream ofs_ldmrs(output_directory / FLAGS_out_ldmrs);
  PrintInPly(ofs_ldmrs, ldmrs_cloud,
             std::vector<cv::Vec3b>(ldmrs_cloud.size(), gray));
}

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./lidar_cloud segment_dir rtk_dir
Where segment_dir is a directory with one part from Robotcar
and rtk_dir is a directory with RTK ground-truth for the dataset.
It expects that the chunk has the respective ground-truth.
It expects that the working dir is the root directory of the repository. Otherwise,
you'll also need to provide the `models_dir` flag. See `./lidar_cloud --helpon=main`
for more details on what is in the output and how to control it.
)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  fs::path segment_directory = argv[1];
  fs::path rtk_directory = argv[2];
  fs::path masks_directory = FLAGS_masks_dir;
  CHECK(fs::is_directory(segment_directory));
  CHECK(fs::is_directory(rtk_directory));
  CHECK(fs::is_directory(masks_directory));

  fs::path output_directory =
      fs::path("output") / ("lidar_cloud_" + CurrentTimeBrief());
  fs::create_directories(output_directory);

  RobotcarReaderSettings settings;
  settings.fill_odometry_gaps = FLAGS_fill_vo_gaps;
  settings.projected_time_window = FLAGS_time_win;
  //  rtkDir.reset();
  RobotcarReader reader(segment_directory, FLAGS_models_dir,
                        FLAGS_extrinsics_dir, std::make_optional(rtk_directory),
                        settings);
  reader.ProvideMasks(masks_directory);

  std::cout << "Total frames: " << reader.NumberOfFrames() << std::endl;

  Timestamp min_timestamp = reader.min_timestamp();
  Timestamp max_timestamp = reader.max_timestamp();
  Timestamp base_timestamp = min_timestamp;

  Trajectory ground_truth_trajectory = reader.GroundTruthTrajectory();
  ground_truth_trajectory.SaveToFile(output_directory / FLAGS_gt_traj);

  if (FLAGS_gen_clouds)
    GenerateClouds(reader, min_timestamp, max_timestamp, base_timestamp,
                   output_directory);

  std::ofstream original_trajectory_file(output_directory /
                                         FLAGS_out_traj_orig);
  for (const SE3 &bodyToWorld : reader.ground_truth_world_from_body()) {
    PutInMatrixForm(original_trajectory_file, bodyToWorld);
  }

  std::ofstream interpolated_trajectory_file(output_directory /
                                             FLAGS_out_traj_interp);
  Timestamp total_time = max_timestamp - min_timestamp;
  double step = static_cast<double>(total_time) / FLAGS_interp_gran;
  for (int i = 0; i < FLAGS_interp_gran; ++i) {
    Timestamp ts = min_timestamp + static_cast<Timestamp>(step * i);
    PutInMatrixForm(interpolated_trajectory_file,
                    reader.WorldFromBodyAtTimestamp(ts));
  }

  cv::Mat3b projected = Project(reader, FLAGS_project_idx);
  cv::imwrite((output_directory / FLAGS_out_project).string(), projected);

  return 0;
}
