#include "dataset/MultiCamReader.h"
#include "util.h"

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

cv::Mat3b perPixelDepths(const DatasetReader *reader, int frameInd) {
  CameraBundle cam = reader->cam();
  int numCams = cam.numCams();
  auto frame = reader->frame(frameInd);
  std::vector<cv::Mat3b> images(numCams);
  for (int ci = 0; ci < numCams; ++ci) images[ci] = frame[ci].frame;

  std::vector<double> allDepths;
  allDepths.reserve(numCams * images[0].rows * images[0].cols);
  std::unique_ptr<FrameDepths> frameDepths = reader->depths(frameInd);
  for (int ci = 0; ci < numCams; ++ci)
    for (int r = 0; r < images[ci].rows; ++r)
      for (int c = 0; c < images[ci].cols; ++c) {
        std::optional<double> depth = frameDepths->depth(ci, Vector2(c, r));
        allDepths.push_back(depth ? *depth : -1);
      }
  std::vector<double> sorted = allDepths;
  std::sort(sorted.begin(), sorted.end());

  double minDepth = sorted[(int)(FLAGS_min_quot * allDepths.size())];
  double maxDepth = sorted[(int)(FLAGS_max_quot * allDepths.size())];
  LOG(INFO) << "depth bounds: " << minDepth << ", " << maxDepth << std::endl;

  std::vector<cv::Vec3b> colors =
      getColors(allDepths, minDepth, maxDepth, cv::COLORMAP_JET);

  int i = 0;
  std::vector<cv::Mat3b> depthImgs(numCams);
  for (int ci = 0; ci < numCams; ++ci) {
    depthImgs[ci] = cv::Mat3b(images[ci].rows, images[ci].cols);
    for (int r = 0; r < images[ci].rows; ++r)
      for (int c = 0; c < images[ci].cols; ++c)
        depthImgs[ci](r, c) = colors[i++];
  }

  cv::Mat3b allImages, allDepthImgs;
  cv::hconcat(images.data(), images.size(), allImages);
  cv::hconcat(depthImgs.data(), depthImgs.size(), allDepthImgs);
  cv::Mat3b result;
  cv::vconcat(allImages, allDepthImgs, result);
  return result;
};

void printTrajectory(const DatasetReader *reader, const fs::path &fname) {
  int numFrames = reader->numFrames();
  std::ofstream ofs(fname);
  for (int fi = 0; fi < numFrames; ++fi)
    putInMatrixForm(ofs, reader->frameToWorld(fi));
}

void createCloud(const DatasetReader *reader, int firstFrame, int lastFrame,
                 int frameStep, int pointsPerImage, const fs::path &outDir) {
  CHECK_GE(lastFrame, firstFrame);
  CHECK_GT(frameStep, 0);

  CameraBundle cam = reader->cam();
  int numCams = cam.numCams();
  std::vector<Vector3> cloud;
  std::vector<cv::Vec3b> colors;
  int rowStep = std::ceil(cam.cam(0).height() / std::sqrt(pointsPerImage));
  int colStep = std::ceil(cam.cam(0).width() / std::sqrt(pointsPerImage));
  LOG(INFO) << "row, col steps = " << rowStep << ", " << colStep;
  for (int fi = firstFrame; fi < lastFrame; fi += frameStep) {
    std::unique_ptr<FrameDepths> frameDepths = reader->depths(fi);
    auto frame = reader->frame(fi);
    SE3 bodyToWorld = reader->frameToWorld(fi);
    for (int ci = 0; ci < numCams; ++ci) {
      SE3 camToWorld = bodyToWorld * cam.camToBody(ci);
      for (int r = 0; r < cam.cam(ci).height(); r += rowStep)
        for (int c = 0; c < cam.cam(ci).width(); c += colStep) {
          Vector2 p(c, r);
          std::optional<double> depth = frameDepths->depth(ci, p);
          if (depth &&
              *depth <
                  1e4) {  // Eliminating sky with super-high (not inf) depths
            cloud.push_back(camToWorld * (depth.value() *
                                          cam.cam(ci).unmap(p).normalized()));
            colors.push_back(frame[ci].frame(r, c));
          }
        }
    }
  }

  std::ofstream ofs(outDir / "cloud.ply");
  printInPly(ofs, cloud, colors);
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

  fs::path outDir = fs::path("output") / ("multicam_demo_" + curTimeBrief());
  fs::create_directories(outDir);

  LOG(INFO) << "the output directory is " << outDir.string() << std::endl;
  std::cout << "the output directory is " << outDir.string() << std::endl;

  fs::path mcamDir = argv[1];
  MultiCamReader reader(mcamDir);

  printTrajectory(&reader, outDir / "gt_traj.txt");
  cv::Mat3b depths = perPixelDepths(&reader, FLAGS_frame_ind);
  cv::imwrite((outDir / "alldepths.png").string(), depths);

  createCloud(&reader, FLAGS_cloud_first_frame_ind, FLAGS_cloud_last_frame_ind,
              FLAGS_cloud_frame_step, FLAGS_cloud_points_per_frame, outDir);
  return 0;
}
