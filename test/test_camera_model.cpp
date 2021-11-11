#include "camera/camera.h"

#include <random>
#include <vector>

#include <ceres/autodiff_cost_function.h>
#include <gtest/gtest.h>

using namespace grpose;

class CameraTest : public ::testing::TestWithParam<Camera> {};

TEST_P(CameraTest, UnmapMap) {
  constexpr int kStepX = 10, kStepY = 10;
  constexpr double kMinSquaredNorm = 1e-6;
  constexpr double kMinUnmapMultiplier = 0.2;
  constexpr double kMaxUnmapMultiplier = 10;

  const Camera &camera = GetParam();

  std::mt19937 mt;
  std::uniform_real_distribution scale_distr(kMinUnmapMultiplier,
                                             kMaxUnmapMultiplier);

  for (int x = 0; x < camera.width(); ++x)
    for (int y = 0; y < camera.height(); ++y) {
      const Vector2 point(static_cast<double>(x), static_cast<double>(x));
      const double scale = scale_distr(mt);
      Vector3 direction = scale * camera.Unmap(point);
      Vector2 backprojected = camera.Map(direction);
      ASSERT_LE((backprojected - point).squaredNorm(), kMinSquaredNorm);
    }
}

TEST_P(CameraTest, MapUnmap) {
  constexpr double kMinSquaredNorm = 1e-7;
  constexpr double kCosAngleError = 1e-6;

  const Camera &camera = GetParam();

  for (int x = 0; x < camera.width(); ++x)
    for (int y = 0; y < camera.height(); ++y) {
      Vector2 point(static_cast<double>(x), static_cast<double>(x));
      Vector3 direction = camera.Unmap(point);
      Vector2 backprojected = camera.Map(direction);
      Vector3 new_direction = camera.Unmap(backprojected);
      ASSERT_NEAR(direction.dot(new_direction), 1.0, kCosAngleError);
    }
}

TEST_P(CameraTest, UnmapNorm) {
  constexpr double kNormDeviation = 1e-6;
  constexpr double kUnmapUnnormalizedCosAngleError = 1e-6;

  const Camera &camera = GetParam();

  for (int x = 0; x < camera.width(); ++x)
    for (int y = 0; y < camera.height(); ++y) {
      Vector2 point(static_cast<double>(x), static_cast<double>(x));
      Vector3 direction = camera.Unmap(point);
      Vector3 unnorm_direction = camera.UnmapUnnormalized(point);
      const double unnorm_norm = unnorm_direction.norm();
      const double direction_norm = direction.norm();
      ASSERT_NEAR(direction_norm, 1.0, kNormDeviation);
      ASSERT_NEAR(unnorm_direction.dot(direction) / unnorm_norm, 1.0,
                  kUnmapUnnormalizedCosAngleError);
    }
}

class MapFunctor {
 public:
  MapFunctor(const Camera &camera) : camera_(camera) {}

  template <typename T>
  bool operator()(const T *const direction, T *point) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> point_map(point);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> direction_map(direction);
    point_map = camera_.Map(direction_map);
    return true;
  }

 private:
  Camera camera_;
};

TEST_P(CameraTest, DifferentiateMap) {
  const Camera &camera = GetParam();

  constexpr double kMapSquaredNormError = 1e-8;
  constexpr double kTangentJacobianSquaredError = 1e-6;

  ceres::AutoDiffCostFunction<MapFunctor, 2, 3> map_function(
      new MapFunctor(camera));

  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> map_jacobian_expected;
  double *j_data = map_jacobian_expected.data();

  for (int x = 0; x < camera.width(); ++x)
    for (int y = 0; y < camera.height(); ++y) {
      const Vector2 point(static_cast<double>(x), static_cast<double>(x));
      const Vector3 direction = camera.Unmap(point);
      const double *d_data = direction.data();
      Vector2 map_expected;
      map_function.Evaluate(&d_data, map_expected.data(), &j_data);

      auto [map_actual, map_jacobian_actual] =
          camera.DifferentiateMap(direction);

      ASSERT_LT((map_actual - map_expected).squaredNorm(),
                kMapSquaredNormError);
      ASSERT_LT((map_jacobian_actual - map_jacobian_expected).squaredNorm(),
                kMapSquaredNormError);
    }
}

Camera GetPinholeCamera() {
  return Camera(1920, 1080, CameraModelId::kPinhole,
                {600.0, 700.0, 960.0, 540.0, 5.0});
}

Camera GetMultiFovCamera() {
  // Parameters in the MultiFov/MultiCam datasets
  return Camera(640, 480, CameraModelId::kMultiFov,
                {-179.471829787234,
                 0.0,
                 0.002316743975,
                 -3.635968439375e-06,
                 2.0546506810625e-08,
                 320.0,
                 240.0,
                 1.0,
                 0.0,
                 0.0,
                 256.2124,
                 138.2261,
                 -3.8287,
                 23.8296,
                 8.0091,
                 -0.5033,
                 6.7625,
                 4.3653,
                 -1.2425,
                 -1.2663,
                 -0.1870,
                 0.0});
}

Camera GetUnifiedCameraWide() {
  return Camera(640, 480, CameraModelId::kUnified,
                {330.0, 250.0, 320.0, 240.0, -0.1, 0.01, 0.001, 0.0015, 0.8});
}

Camera GetUnifiedCameraNarrow() {
  return Camera(640, 480, CameraModelId::kUnified,
                {330.0, 250.0, 320.0, 240.0, -0.1, 0.01, 0.001, 0.0015, 0.4});
}

// COLMAP only supports FoV < 180 degrees
Camera GetColmapUnifiedCamera() {
  return Camera::ColmapCamera(
      640, 480, colmap::UnifiedCameraModel::kModelId,
      {330.0, 250.0, 320.0, 240.0, -0.1, 0.01, 0.001, 0.0015, 0.4});
}

INSTANTIATE_TEST_SUITE_P(CameraTestSuite, CameraTest,
                         testing::Values(GetPinholeCamera(),
                                         GetMultiFovCamera(),
                                         GetUnifiedCameraWide(),
                                         GetUnifiedCameraNarrow(),
                                         GetColmapUnifiedCamera()));

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
