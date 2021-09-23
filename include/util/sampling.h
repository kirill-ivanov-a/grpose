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

template <typename RandomBitsGenerator>
Vector3 SampleSphereUniform(RandomBitsGenerator &generator,
                            const Vector3 &center, double radius) {
  // Here we use Archmedes' hat-box theorem
  std::uniform_real_distribution<double> z_distr(-1.0, 1.0);
  std::uniform_real_distribution<double> phi_distr(0.0, 2 * M_PI);
  const double z = z_distr(generator), phi = phi_distr(generator);
  const double r = std::sqrt(1.0 - z * z);
  const double x = r * std::cos(phi), y = r * std::sin(phi);
  const Vector3 unit_sample(x, y, z);
  return unit_sample * radius + center;
}

}  // namespace grpose

#endif