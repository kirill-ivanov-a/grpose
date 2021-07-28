#include "camera/camera_model_pinhole.h"
#include "camera/camera_model_scaramuzza.h"

#include <random>
#include <vector>

#include <ceres/autodiff_cost_function.h>
#include <ceres/local_parameterization.h>
#include <gtest/gtest.h>

using namespace grpose;

template <typename CameraModelT>
struct CameraParameters;

template <>
struct CameraParameters<CameraModelPinhole> {
  std::vector<double> parameters = {600.0, 700.0, 960.0, 540.0, 5.0};
  int width = 1920, height = 1080;
};

template <>
struct CameraParameters<CameraModelMultiFov> {
  // Parameters in the MultiFov/MultiCam datasets
  std::vector<double> parameters = {-179.471829787234,
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
                                    0.0};
  int width = 640, height = 480;
};

template <typename CameraModelT>
class CameraModelTest : public ::testing::Test {
 protected:
  CameraParameters<CameraModelT> camera_parameters;
};

using CameraModels = ::testing::Types<CameraModelMultiFov>;
TYPED_TEST_SUITE(CameraModelTest, CameraModels);

TYPED_TEST(CameraModelTest, UnmapDerivedMap) {
  constexpr int kStepX = 10, kStepY = 10;
  constexpr double kMinSquaredNorm = 1e-7;
  constexpr double kMinUnmapMultiplier = 0.2;
  constexpr double kMaxUnmapMultiplier = 10;

  std::mt19937 mt;
  std::uniform_real_distribution scale_distr(kMinUnmapMultiplier,
                                             kMaxUnmapMultiplier);

  const std::vector<double> &parameters = this->camera_parameters.parameters;

  for (int x = 0; x < this->camera_parameters.width; ++x)
    for (int y = 0; y < this->camera_parameters.height; ++y) {
      const Vector2 point(static_cast<double>(x), static_cast<double>(x));
      const double scale = scale_distr(mt);
      Vector3 direction = scale * TypeParam::Unmap(point, parameters);
      Vector2 backprojected = TypeParam::Map(direction, parameters);
      ASSERT_LE((backprojected - point).squaredNorm(), kMinSquaredNorm);
    }
}

TYPED_TEST(CameraModelTest, UnmapBaseMap) {
  constexpr int kStepX = 10, kStepY = 10;
  constexpr double kMinSquaredNorm = 1e-5;
  constexpr double kMinUnmapMultiplier = 0.2;
  constexpr double kMaxUnmapMultiplier = 10;

  std::mt19937 mt;
  std::uniform_real_distribution scale_distr(kMinUnmapMultiplier,
                                             kMaxUnmapMultiplier);

  const std::vector<double> &parameters = this->camera_parameters.parameters;

  for (int x = 0; x < this->camera_parameters.width; ++x)
    for (int y = 0; y < this->camera_parameters.height; ++y) {
      const Vector2 point(static_cast<double>(x), static_cast<double>(x));
      const double scale = scale_distr(mt);
      const Vector3 direction =
          scale * CameraModel<TypeParam>::Unmap(point, parameters);
      Vector2 backprojected = TypeParam::Map(direction, parameters);
      ASSERT_LE((backprojected - point).squaredNorm(), kMinSquaredNorm)
          << "x=" << x << " y=" << y << std::endl;
    }
}

TYPED_TEST(CameraModelTest, MapDerivedUnmap) {
  constexpr double kMinSquaredNorm = 1e-7;
  constexpr double kCosAngleError = 1e-6;
  const std::vector<double> &parameters = this->camera_parameters.parameters;

  for (int x = 0; x < this->camera_parameters.width; ++x)
    for (int y = 0; y < this->camera_parameters.height; ++y) {
      Vector2 point(static_cast<double>(x), static_cast<double>(x));
      Vector3 direction = TypeParam::Unmap(point, parameters);
      Vector2 backprojected = TypeParam::Map(direction, parameters);
      Vector3 new_direction = TypeParam::Unmap(backprojected, parameters);
      ASSERT_NEAR(direction.dot(new_direction), 1.0, kCosAngleError);
    }
}

TYPED_TEST(CameraModelTest, MapBaseUnmap) {
  constexpr double kMinSquaredNorm = 1e-7;
  constexpr double kCosAngleError = 1e-6;
  const std::vector<double> &parameters = this->camera_parameters.parameters;

  for (int x = 0; x < this->camera_parameters.width; ++x)
    for (int y = 0; y < this->camera_parameters.height; ++y) {
      Vector2 point(static_cast<double>(x), static_cast<double>(x));
      Vector3 direction = CameraModel<TypeParam>::Unmap(point, parameters);
      Vector2 backprojected = TypeParam::Map(direction, parameters);
      Vector3 new_direction =
          CameraModel<TypeParam>::Unmap(backprojected, parameters);
      ASSERT_NEAR(direction.dot(new_direction), 1.0, kCosAngleError);
    }
}

TYPED_TEST(CameraModelTest, UnmapNorm) {
  constexpr double kNormDeviation = 1e-6;
  constexpr double kUnmapUnnormalizedCosAngleError = 1e-6;
  const std::vector<double> &parameters = this->camera_parameters.parameters;

  for (int x = 0; x < this->camera_parameters.width; ++x)
    for (int y = 0; y < this->camera_parameters.height; ++y) {
      Vector2 point(static_cast<double>(x), static_cast<double>(x));
      Vector3 direction = TypeParam::Unmap(point, parameters);
      Vector3 unnorm_direction =
          TypeParam::UnmapUnnormalized(point, parameters);
      const double unnorm_norm = unnorm_direction.norm();
      const double direction_norm = direction.norm();
      ASSERT_NEAR(direction_norm, 1.0, kNormDeviation);
      ASSERT_NEAR(unnorm_direction.dot(direction) / unnorm_norm, 1.0,
                  kUnmapUnnormalizedCosAngleError);
    }
}

template <typename CameraModelT>
class MapFunctor {
 public:
  MapFunctor(const std::vector<double> &parameters) : parameters_(parameters) {}

  template <typename T>
  bool operator()(const T *const direction, T *point) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> point_map(point);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> direction_map(direction);
    point_map = CameraModelT::Map(direction_map, parameters_);
    return true;
  }

 private:
  std::vector<double> parameters_;
};

TYPED_TEST(CameraModelTest, DifferentiateMap) {
  using Functor = MapFunctor<TypeParam>;

  constexpr double kMapSquaredNormError = 1e-8;
  constexpr double kTangentJacobianSquaredError = 1e-6;

  const std::vector<double> &parameters = this->camera_parameters.parameters;
  ceres::AutoDiffCostFunction<Functor, 2, 3> map_function(
      new Functor(parameters));

  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> map_jacobian_expected;
  double *j_data = map_jacobian_expected.data();

  // should fail at compile time with static_assert
  //  Vector2 point = TypeParam::Unmap(Vector3(0.0, 0.0, 0.0), parameters);

  ceres::HomogeneousVectorParameterization parameterization(3);

  for (int x = 0; x < this->camera_parameters.width; ++x)
    for (int y = 0; y < this->camera_parameters.height; ++y) {
      const Vector2 point(static_cast<double>(x), static_cast<double>(x));
      const Vector3 direction = TypeParam::Unmap(point, parameters);
      const double *d_data = direction.data();
      Vector2 map_expected;
      map_function.Evaluate(&d_data, map_expected.data(), &j_data);

      auto [map_actual, map_jacobian_actual] =
          TypeParam::DifferentiateMap(direction, parameters);

      ASSERT_LT((map_actual - map_expected).squaredNorm(),
                kMapSquaredNormError);

      Eigen::Matrix<double, 3, 2, Eigen::RowMajor> tangent;
      parameterization.ComputeJacobian(direction.data(), tangent.data());
      Matrix22 tangent_error =
          (map_jacobian_actual - map_jacobian_expected) * tangent;
      ASSERT_LT(tangent_error.squaredNorm(), kMapSquaredNormError);
    }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
