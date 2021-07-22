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

void testReproj(const CameraBundle &cameraBundle, const fs::path &outDir) {
  for (int ci = 0; ci < cameraBundle.numCams(); ++ci) {
    Camera cam = cameraBundle.cam(ci);
    int h = cam.height(), w = cam.width();
    cv::Mat1d errors(h, w);
    for (int r = 0; r < h; ++r)
      for (int c = 0; c < w; ++c) {
        Vector2 p(c, r);
        Vector2 repr = cam.map(cam.unmap(p));
        errors(c, r) = (p - repr).norm();
      }

    std::ofstream ofs(outDir /
                      fs::path("reproj" + std::to_string(ci) + ".txt"));
    for (int r = 0; r < h; ++r) {
      for (int c = 0; c < w; ++c) ofs << errors(r, c) << ' ';
      ofs << '\n';
    }

    cv::imshow("errors" + std::to_string(ci), errors);
  }
  cv::waitKey();
}

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./reproj_test chunk_dir rtk_dir
Where chunk_dir is a directory with one part from Robotcar
and rtk_dir is a directory with RTK ground-truth for the dataset.
It expects that the chunk has the respective ground-truth.
It expects that the working dir is the root directory of the repository. Otherwise,
you'll also need to provide the `models_dir` flag. See `./reproj_test --helpon=main`
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

  fs::path outDir = fs::path("output") / ("reproj_test_" + curTimeBrief());
  fs::create_directories(outDir);

  RobotcarReaderSettings settings;
  //  rtkDir.reset();
  RobotcarReader reader(chunkDir, FLAGS_models_dir, FLAGS_extrinsics_dir,
                        std::make_optional(rtkDir), settings);
  reader.provideMasks(masksDir);

  testReproj(reader.cam(), outDir);

  return 0;
}