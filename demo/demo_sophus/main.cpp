#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "types.h"

DEFINE_bool(opencv_demo, false, "If set, the OpenCV demo is run as well");

using namespace grpose;

int main(int argc, char **argv) {
  // Nice bonus: can print this thing when invoked with --help
  gflags::SetUsageMessage("demo_sophus [--opencv_demo]");
  // Put these two lines in the beginning of main() for gflags and glog to work
  // properly
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_opencv_demo) {
    cv::Mat3b image(300, 400, cv::Vec3b(255, 255, 255));
    cv::imshow("A 300x400 white image", image);
    cv::waitKey();
  } else
    std::cout << "The opencv_demo flag is not set" << std::endl;

  // This message will be put into your /tmp directory into a newly created log
  // file. The behavior can be changed via command-line flags, run
  // `demo_sophus --help` for more info.
  LOG(INFO) << "demo_sophus flies";

  Vector3 axis(0, 0, 1);  // rotate around Oz
  double angle = M_PI_2;  // to the angle pi/2

  // exp in so(3) lie algebra corresponds to conversion from axis-angle
  // If you feel uncomfortable with thinking about coordinate transformations as
  // Lie group elements, you might want to check out the following nice intro:
  // http://ethaneade.com/lie.pdf
  SO3 rot_coord1_to_coord2 = SO3::exp(angle * axis);
  Vector3 translation(2, 0, 0);
  SE3 coord1_to_coord2(rot_coord1_to_coord2, translation);

  Vector3 vec_in_coord1(5, 4, 3);
  std::cout << "sample vector in the first coodinate system:\n"
            << vec_in_coord1.transpose() << std::endl;

  std::cout << "rotated by pi/2 around Oz:\n"
            << (rot_coord1_to_coord2 * vec_in_coord1).transpose() << std::endl;

  Vector3 vec_in_coord2 = coord1_to_coord2 * vec_in_coord1;
  std::cout << "converted to coord2:\n"
            << vec_in_coord2.transpose() << std::endl;

  std::cout << "converted back to coord1:\n"
            << (coord1_to_coord2.inverse() * vec_in_coord2).transpose()
            << std::endl;

  LOG(INFO) << "demo_sophus has flown";

  return 0;
}
