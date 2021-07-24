#include <glog/logging.h>
#include <gtest/gtest.h>
#include <random>
#include "camera/CameraModelScaramuzza.h"

using namespace grpose;

TEST(CameraModelScarmuzzaTest, RealCameraReprojection) {
  // some real-world data
  double scale = 604.0;
  Vector2 center(1.58492, 1.07424);
  int unmap_polynomial_degree = 5;
  VectorX unmap_polynomial_coefficients(unmap_polynomial_degree, 1);
  unmap_polynomial_coefficients << 1.14169, -0.203229, -0.362134, 0.351011,
      -0.147191;
  int width = 1920, height = 1208;
  CameraModelScaramuzza camera(width, height, scale, center,
                               unmap_polynomial_coefficients);

  std::srand(42);
  constexpr int kNumberOfTests = 2000;
  double squared_error = 0.0;

  for (int i = 0; i < kNumberOfTests; ++i) {
    Vector2 pnt(double(rand() % width), double(rand() % height));
    Vector3 ray = camera.Unmap(pnt.data());
    ray *= 10.5;

    Vector2 pnt_back = camera.Map(ray.data());
    squared_error += (pnt - pnt_back).squaredNorm();
  }

  double rmse = std::sqrt(squared_error / kNumberOfTests);  // rmse in pixels
  LOG(INFO) << "reprojection rmse = " << rmse << std::endl;
  EXPECT_LT(rmse, 0.1);
}

TEST(CameraModelScarmuzzaTest, SolelyPolynomial) {
  // here camera is initialised so that its mapping only inverses the given
  // polynomial
  double scale = 1;
  Vector2 center(0.0, 0.0);
  int unmap_polynomial_degree = 2;
  VectorX unmap_polynomial_coefficients(unmap_polynomial_degree, 1);
  unmap_polynomial_coefficients << 1.0, -1.0;
  int width = 2, height = 0;
  CameraModelScaramuzza camera(width, height, scale, center,
                               unmap_polynomial_coefficients);

  std::srand(42);
  constexpr int kNumberOfTests = 2000;
  double squared_error = 0.0;
  for (int i = 0; i < kNumberOfTests; ++i) {
    double x = double(std::rand()) / RAND_MAX;
    Vector3 pnt(x, 0, 1 - x * x);
    Vector2 projected = camera.Map(pnt.data());
    squared_error += (projected - Vector2(x, 0)).squaredNorm();
  }
  double rmse = std::sqrt(squared_error / kNumberOfTests);
  LOG(INFO) << "rmse = " << rmse << std::endl;
  EXPECT_LT(rmse, 0.0005);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
