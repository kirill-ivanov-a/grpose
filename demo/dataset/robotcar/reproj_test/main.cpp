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

using namespace grpose;

void TestReproj(const CameraBundle &camera_bundle,
                const fs::path &output_directory) {
  for (int ci = 0; ci < camera_bundle.NumberOfCameras(); ++ci) {
    const Camera &camera = camera_bundle.camera(ci);
    int h = camera.height(), w = camera.width();
    cv::Mat1d errors(h, w);
    for (int r = 0; r < h; ++r)
      for (int c = 0; c < w; ++c) {
        Vector2 p(c, r);
        Vector2 repr = camera.Map(camera.Unmap(p));
        errors(c, r) = (p - repr).norm();
      }

    std::ofstream reprojected_file(
        output_directory / fs::path("reproj" + std::to_string(ci) + ".txt"));
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) reprojected_file << errors(r, c) << ' ';
      reprojected_file << '\n';
    }

    cv::imshow("errors" + std::to_string(ci), errors);
  }
  cv::waitKey();
}

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./reproj_test segment_dir rtk_dir
Where segment_dir is a directory with one part from Robotcar
and rtk_dir is a directory with RTK ground-truth for the dataset.
It expects that the chunk has the respective ground-truth.
It expects that the working dir is the root directory of the repository. Otherwise,
you'll also need to provide the `models_dir` flag. See `./reproj_test --helpon=main`
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
      fs::path("output") / ("reproj_test_" + CurrentTimeBrief());
  fs::create_directories(output_directory);

  RobotcarReaderSettings settings;
  //  rtkDir.reset();
  RobotcarReader reader(segment_directory, FLAGS_models_dir,
                        FLAGS_extrinsics_dir, std::make_optional(rtk_directory),
                        settings);
  reader.ProvideMasks(masks_directory);

  TestReproj(reader.GetCameraBundle(), output_directory);

  return 0;
}