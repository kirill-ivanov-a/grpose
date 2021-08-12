#include "dataset/autovision_reader.h"

using namespace grpose;

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./autovision_demo segment_dir calib_dir config_fname
where segment_dir is the path to a segment from AutoVision sample data (it should contain
"timestamps.txt", "gt_poses.txt" and directories for the cameras), calib_dir is the path to the directory with the calibration info (it should contain "camera_system_cal.json" and "sensor_system_cal.json"), and config_fname is the path to the JSON config for the dataset.
)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  constexpr int kExpectedArgc = 4;
  if (argc != kExpectedArgc) {
    fmt::print("Expected {:d} arguments, got {:d}", kExpectedArgc, argc);
    return 1;
  }
  fs::path segment_directory = argv[1];
  fs::path calib_directory = argv[2];
  fs::path config_filename = argv[3];

  AutovisionReader reader(segment_directory, calib_directory, config_filename);
  CameraBundle camera_bundle = reader.GetCameraBundle();
  std::cout << "num cams: " << camera_bundle.NumberOfCameras() << std::endl;
  std::cout << "num frames: " << reader.NumberOfFrames() << std::endl;
  fmt::print("camera[0] params: {}",
             fmt::join(camera_bundle.camera(0).parameters(), ", "));
  return 0;
}
