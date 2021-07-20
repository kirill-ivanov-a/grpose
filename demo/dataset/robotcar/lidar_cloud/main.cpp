#include <opencv2/imgproc.hpp>
#include "dataset/RobotcarReader.h"
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

std::vector<cv::Vec3b> getColors(std::vector<double> vals, double minVal,
                                 double maxVal, const cv::ColormapTypes &cmap) {
  cv::Mat1b valsMat(vals.size(), 1);
  for (int i = 0; i < vals.size(); ++i)
    valsMat(i, 0) = (unsigned char)(std::clamp(
        255 * (vals[i] - minVal) / (maxVal - minVal), 0.1, 254.0));
  cv::Mat3b colorsMat;
  cv::applyColorMap(valsMat, colorsMat, cmap);
  std::vector<cv::Vec3b> colors(vals.size());
  for (int i = 0; i < vals.size(); ++i) colors[i] = colorsMat(i, 0);
  return colors;
}

cv::Mat3b project(const RobotcarReader &reader, int idx) {
  std::cout << "generating projections..." << std::endl;

  auto frame = reader.frame(idx);
  Timestamp baseTs = frame[0].timestamp;
  Timestamp timeWin = FLAGS_time_win * 1e6;
  Timestamp minTs = baseTs - timeWin, maxTs = baseTs + timeWin;
  std::vector<Vector3> lmsFrontCloud =
      reader.getLmsFrontCloud(minTs, maxTs, baseTs);
  std::vector<Vector3> lmsRearCloud =
      reader.getLmsRearCloud(minTs, maxTs, baseTs);
  std::vector<Vector3> ldmrsCloud = reader.getLdmrsCloud(minTs, maxTs, baseTs);
  std::vector<Vector3> cloud;
  cloud.reserve(lmsFrontCloud.size() + lmsRearCloud.size() + ldmrsCloud.size());
  cloud.insert(cloud.end(), lmsFrontCloud.begin(), lmsFrontCloud.end());
  cloud.insert(cloud.end(), lmsRearCloud.begin(), lmsRearCloud.end());
  cloud.insert(cloud.end(), ldmrsCloud.begin(), ldmrsCloud.end());
  cv::Mat3b images[RobotcarReader::numCams];

  int s = FLAGS_rel_point_size *
          (RobotcarReader::imageWidth + RobotcarReader::imageHeight) / 2;

  CameraBundle cam = reader.cam();
  StdVectorA<Vector2> points[RobotcarReader::numCams];
  std::vector<double> depths[RobotcarReader::numCams];
  std::vector<double> allDepths;
  for (int ci = 0; ci < RobotcarReader::numCams; ++ci) {
    SE3 bodyToCam = cam.bodyToCam(ci);
    for (const Vector3 &p : cloud) {
      Vector3 moved = bodyToCam * p;
      if (!reader.cam().cam(ci).isMappable(moved)) continue;

      double depth = moved.norm();
      Vector2 projected = reader.cam().cam(ci).map(moved);
      depths[ci].push_back(depth);
      allDepths.push_back(depth);
      points[ci].push_back(projected);
    }
  }
  std::sort(allDepths.begin(), allDepths.end());
  double minDepth = allDepths[(int)(0.05 * allDepths.size())];
  double maxDepth = allDepths[(int)(0.90 * allDepths.size())];

  for (int ci = 0; ci < RobotcarReader::numCams; ++ci) {
    CHECK(frame[ci].frame.channels() == 3);
    std::vector<cv::Vec3b> colors =
        getColors(depths[ci], minDepth, maxDepth, cv::COLORMAP_JET);
    images[ci] = frame[ci].frame.clone();
    for (int j = 0; j < points[ci].size(); ++j)
      drawSquare(images[ci], toCvPoint(points[ci][j]), s, colors[j],
                 cv::FILLED);
    cv::bitwise_and(images[ci], cvtGrayToBgr(cam.cam(ci).mask()), images[ci]);
  }

  cv::Mat3b result;
  cv::hconcat(images, RobotcarReader::numCams, result);
  return result;
}

void genClouds(const RobotcarReader &reader, Timestamp minTs, Timestamp maxTs,
               Timestamp baseTs, const fs::path &outDir) {
  constexpr int totalClouds = 3;

  std::cout << "generating clouds..." << std::endl;

  std::vector<Vector3> lmsFrontCloud =
      reader.getLmsFrontCloud(minTs, maxTs, baseTs);
  std::vector<Vector3> lmsRearCloud =
      reader.getLmsRearCloud(minTs, maxTs, baseTs);
  std::vector<Vector3> ldmrsCloud = reader.getLdmrsCloud(minTs, maxTs, baseTs);
  std::vector<Vector3> *clouds[totalClouds] = {&lmsFrontCloud, &lmsRearCloud,
                                               &ldmrsCloud};
  LOG(INFO) << "total...";
  LOG(INFO) << "points from LMS front: " << lmsFrontCloud.size();
  LOG(INFO) << "points from LMS rear : " << lmsRearCloud.size();
  LOG(INFO) << "points from LDMRS    : " << ldmrsCloud.size();

  sparsifyVectors(clouds, totalClouds, FLAGS_cloud_points);

  LOG(INFO) << "filtered...";
  LOG(INFO) << "points from LMS front: " << lmsFrontCloud.size();
  LOG(INFO) << "points from LMS rear : " << lmsRearCloud.size();
  LOG(INFO) << "points from LDMRS    : " << ldmrsCloud.size();

  std::cout << "saving clouds..." << std::endl;
  const cv::Vec3b gray(128, 128, 128);
  std::ofstream ofsFront(outDir / FLAGS_out_front);
  printInPly(ofsFront, lmsFrontCloud,
             std::vector<cv::Vec3b>(lmsFrontCloud.size(), gray));
  std::ofstream ofsRear(outDir / FLAGS_out_rear);
  printInPly(ofsRear, lmsRearCloud,
             std::vector<cv::Vec3b>(lmsRearCloud.size(), gray));
  std::ofstream ofsLdmrs(outDir / FLAGS_out_ldmrs);
  printInPly(ofsLdmrs, ldmrsCloud,
             std::vector<cv::Vec3b>(ldmrsCloud.size(), gray));
}

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./lidar_cloud chunk_dir rtk_dir
Where chunk_dir is a directory with one part from Robotcar
and rtk_dir is a directory with RTK ground-truth for the dataset.
It expects that the chunk has the respective ground-truth.
It expects that the working dir is the root directory of the repository. Otherwise,
you'll also need to provide the `models_dir` flag. See `./lidar_cloud --helpon=main`
for more details on what is in the output and how to control it.
)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  fs::path chunkDir = argv[1];
  fs::path rtkDir = argv[2];
  fs::path masksDir = FLAGS_masks_dir;
  CHECK(fs::is_directory(chunkDir));
  CHECK(fs::is_directory(rtkDir));
  CHECK(fs::is_directory(masksDir));

  fs::path outDir = fs::path("output") / ("lidar_cloud_" + curTimeBrief());
  fs::create_directories(outDir);

  RobotcarReaderSettings settings;
  settings.fillVoGaps = FLAGS_fill_vo_gaps;
  settings.projectedTimeWindow = FLAGS_time_win;
  //  rtkDir.reset();
  RobotcarReader reader(chunkDir, FLAGS_models_dir, FLAGS_extrinsics_dir,
                        std::make_optional(rtkDir), settings);
  reader.provideMasks(masksDir);

  std::cout << "Total frames: " << reader.numFrames() << std::endl;

  Timestamp minTs = reader.minTs();
  Timestamp maxTs = reader.maxTs();
  Timestamp baseTs = minTs;

  Trajectory gtTraj = reader.gtTrajectory();
  gtTraj.saveToFile(outDir / FLAGS_gt_traj);

  if (FLAGS_gen_clouds) genClouds(reader, minTs, maxTs, baseTs, outDir);

  std::ofstream ofsTrajOrig(outDir / FLAGS_out_traj_orig);
  for (const SE3 &bodyToWorld : reader.getGtBodyToWorld()) {
    putInMatrixForm(ofsTrajOrig, bodyToWorld);
  }

  std::ofstream ofsTrajInterp(outDir / FLAGS_out_traj_interp);
  Timestamp allTime = maxTs - minTs;
  double step = double(allTime) / FLAGS_interp_gran;
  for (int i = 0; i < FLAGS_interp_gran; ++i) {
    Timestamp ts = minTs + step * i;
    putInMatrixForm(ofsTrajInterp, reader.tsToWorld(ts));
  }

  cv::Mat3b proj = project(reader, FLAGS_project_idx);
  cv::imwrite((outDir / FLAGS_out_project).string(), proj);

  return 0;
}
