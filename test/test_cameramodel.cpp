#include "camera/camera_model_pinhole.h"

#include <gtest/gtest.h>

using namespace grpose;

template <typename CameraModelT>
class CameraModelTest : public ::testing::Test {};

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
