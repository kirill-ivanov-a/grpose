#ifndef GRPOSE_UTIL_SAMPLING_
#define GRPOSE_UTIL_SAMPLING_

#include <random>

#include <ceres/local_parameterization.h>

#include "util/geometry.h"
#include "util/types.h"

namespace grpose {

template <typename RandomBitsGenerator>
Vector3 AddGaussianDirectionNoise(RandomBitsGenerator &generator,
                                  const Vector3 &direction, double angle_std);

template <typename RandomBitsGenerator>
Vector3 SampleSphereUniform(RandomBitsGenerator &generator,
                            const Vector3 &center = Vector3::Zero(),
                            double radius = 1);

template <typename RandomBitsGenerator>
Vector3 SampleHemisphereUniform(RandomBitsGenerator &generator,
                                const Vector3 &direction = Vector3(0.0, 0.0,
                                                                   1.0),
                                const Vector3 &center = Vector3::Zero(),
                                double radius = 1);

// Implementation

template <typename RandomBitsGenerator>
Vector3 AddGaussianDirectionNoise(RandomBitsGenerator &generator,
                                  const Vector3 &direction, double angle_std) {
  // TODO normal assert
  CHECK_GE(angle_std, 0);
  if (angle_std <= std::numeric_limits<double>::epsilon()) return direction;

  std::uniform_real_distribution<double> phi_distribution(0, 2 * M_PI);
  std::normal_distribution<double> theta_distribution(0, angle_std);
  const double phi = phi_distribution(generator);
  const double theta = theta_distribution(generator);
  // 2*theta since ceres's parameterization rotates by angle |delta|/2
  const double delta[2] = {2 * theta * std::cos(phi),
                           2 * theta * std::sin(phi)};

  Vector3 noisy_direction;
  const ceres::HomogeneousVectorParameterization parameterization(3);
  parameterization.Plus(direction.data(), delta, noisy_direction.data());
  return noisy_direction;
}

namespace {

template <typename RandomBitsGenerator>
Vector3 SampleUnitSphereSectionUniform(RandomBitsGenerator &generator,
                                       double z_min, double z_max) {
  CHECK_LE(z_min, z_max);

  // Here we use Archmedes' hat-box theorem
  std::uniform_real_distribution<double> z_distr(z_min, z_max);
  std::uniform_real_distribution<double> phi_distr(0.0, 2 * M_PI);
  const double z = z_distr(generator), phi = phi_distr(generator);
  const double r = std::sqrt(1.0 - z * z);
  const double x = r * std::cos(phi), y = r * std::sin(phi);
  return Vector3(x, y, z);
}

}  // namespace

template <typename RandomBitsGenerator>
Vector3 SampleSphereUniform(RandomBitsGenerator &generator,
                            const Vector3 &center, double radius) {
  const Vector3 unit_sample =
      SampleUnitSphereSectionUniform(generator, -1.0, 1.0);
  return unit_sample * radius + center;
}

template <typename RandomBitsGenerator>
Vector3 SampleHemisphereUniform(RandomBitsGenerator &generator,
                                const Vector3 &direction, const Vector3 &center,
                                double radius) {
  const Vector3 unit_sample =
      SampleUnitSphereSectionUniform(generator, 0.0, 1.0);

  // construct an SO(3) transformation R s.t. R*(0 0 1)^T = direction
  const Vector3 vx = AnyOrthogonalUnitVector(direction);
  const Vector3 vy = direction.cross(vx);
  Matrix33 R;
  R << vx, vy, direction;
  CHECK_NEAR(R.determinant(), 1.0, 1e-10);

  return radius * (R * unit_sample) + center;
}

}  // namespace grpose

#endif