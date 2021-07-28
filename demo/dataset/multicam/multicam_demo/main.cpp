#include "dataset/multicam_reader.h"
#include "util/util.h"

DEFINE_int32(frame_ind, 400,
             "The frame from which we want the per-pixel depths");

DEFINE_int32(cloud_first_frame_ind, 400, "The first frame for the point cloud");
DEFINE_int32(cloud_last_frame_ind, 450, "The first frame for the point cloud");
DEFINE_int32(cloud_frame_step, 10,
             "How many frames do we skip for the point cloud");
DEFINE_int32(cloud_points_per_frame, 10000,
             "The approximate amount of points in the cloud per one image");

DEFINE_double(
    min_quot, 0.05,
    "The part of all depths that is below the lowest point on the colormap");
DEFINE_double(
    max_quot, 0.7,
    "The part of all depths that is below the highest point on the colormap");

using namespace grpose;

cv::Mat3b PerPixelDepths(const DatasetReader *reader, int frame_index) {
  CameraBundle camera_bundle = reader->GetCameraBundle();
  int number_of_cameras = camera_bundle.NumberOfCameras();
  auto frame = reader->Frame(frame_index);
  std::vector<cv::Mat3b> images(number_of_cameras);
  for (int ci = 0; ci < number_of_cameras; ++ci) images[ci] = frame[ci].frame;

  std::vector<double> all_depths;
  all_depths.reserve(number_of_cameras * images[0].rows * images[0].cols);
  std::unique_ptr<FrameDepths> frame_depths = reader->GetDepths(frame_index);
  for (int ci = 0; ci < number_of_cameras; ++ci)
    for (int r = 0; r < images[ci].rows; ++r)
      for (int c = 0; c < images[ci].cols; ++c) {
        std::optional<double> depth = frame_depths->Depth(ci, Vector2(c, r));
        all_depths.push_back(depth ? *depth : -1);
      }
  std::vector<double> sorted = all_depths;
  std::sort(sorted.begin(), sorted.end());

  double min_depth = sorted[(int)(FLAGS_min_quot * all_depths.size())];
  double max_depth = sorted[(int)(FLAGS_max_quot * all_depths.size())];
  LOG(INFO) << "depth bounds: " << min_depth << ", " << max_depth << std::endl;

  std::vector<cv::Vec3b> colors =
      GetColors(all_depths, min_depth, max_depth, cv::COLORMAP_JET);

  int i = 0;
  std::vector<cv::Mat3b> depth_images(number_of_cameras);
  for (int ci = 0; ci < number_of_cameras; ++ci) {
    depth_images[ci] = cv::Mat3b(images[ci].rows, images[ci].cols);
    for (int r = 0; r < images[ci].rows; ++r)
      for (int c = 0; c < images[ci].cols; ++c)
        depth_images[ci](r, c) = colors[i++];
  }

  cv::Mat3b all_images, all_depth_images;
  cv::hconcat(images.data(), images.size(), all_images);
  cv::hconcat(depth_images.data(), depth_images.size(), all_depth_images);
  cv::Mat3b result;
  cv::vconcat(all_images, all_depth_images, result);
  return result;
};

void PrintTrajectory(const DatasetReader *reader, const fs::path &filename) {
  int number_of_frames = reader->NumberOfFrames();
  std::ofstream ofs(filename);
  for (int fi = 0; fi < number_of_frames; ++fi)
    PutInMatrixForm(ofs, reader->WorldFromFrame(fi));
}

void CreateCloud(const DatasetReader *reader, int first_frame_index,
                 int last_frame_index, int frame_index_step,
                 int points_per_image, const fs::path &output_directory) {
  CHECK_GE(last_frame_index, first_frame_index);
  CHECK_GT(frame_index_step, 0);

  CameraBundle camera_bundle = reader->GetCameraBundle();
  int number_of_cameras = camera_bundle.NumberOfCameras();
  std::vector<Vector3> cloud;
  std::vector<cv::Vec3b> colors;
  int row_step =
      std::ceil(camera_bundle.camera(0).height() / std::sqrt(points_per_image));
  int col_step =
      std::ceil(camera_bundle.camera(0).width() / std::sqrt(points_per_image));
  LOG(INFO) << "row, col steps = " << row_step << ", " << col_step;
  for (int fi = first_frame_index; fi < last_frame_index;
       fi += frame_index_step) {
    std::unique_ptr<FrameDepths> frame_depths = reader->GetDepths(fi);
    auto frame = reader->Frame(fi);
    SE3 world_from_body = reader->WorldFromFrame(fi);
    for (int ci = 0; ci < number_of_cameras; ++ci) {
      SE3 world_from_camera =
          world_from_body * camera_bundle.body_from_camera(ci);
      for (int r = 0; r < camera_bundle.camera(ci).height(); r += row_step)
        for (int c = 0; c < camera_bundle.camera(ci).width(); c += col_step) {
          Vector2 p(c, r);
          std::optional<double> depth = frame_depths->Depth(ci, p);
          // Eliminating sky with super-high (not inf) depths
          if (depth && *depth < 1e4) {
            cloud.push_back(world_from_camera *
                            (depth.value() *
                             camera_bundle.camera(ci).Unmap(p).normalized()));
            colors.push_back(frame[ci].frame(r, c));
          }
        }
    }
  }

  std::ofstream ofs(output_directory / "cloud.ply");
  PrintInPly(ofs, cloud, colors);
}

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./multicam_demo data_dir
where data_dir is the path to the root MultiCam dataset directory (it should contain
folders "data" and "info").
)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  fs::path output_directory =
      fs::path("output") / ("multicam_demo_" + CurrentTimeBrief());
  fs::create_directories(output_directory);

  LOG(INFO) << "the output directory is " << output_directory.string()
            << std::endl;
  std::cout << "the output directory is " << output_directory.string()
            << std::endl;

  fs::path multicam_directory = argv[1];
  MultiCamReader reader(multicam_directory);

  PrintTrajectory(&reader, output_directory / "gt_traj.txt");
  cv::Mat3b depths = PerPixelDepths(&reader, FLAGS_frame_ind);
  cv::imwrite((output_directory / "alldepths.png").string(), depths);

  CreateCloud(&reader, FLAGS_cloud_first_frame_ind, FLAGS_cloud_last_frame_ind,
              FLAGS_cloud_frame_step, FLAGS_cloud_points_per_frame,
              output_directory);
  return 0;
}
