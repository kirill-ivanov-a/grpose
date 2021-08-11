#ifndef GRPOSE_UTIL_SAMPLING_
#define GRPOSE_UTIL_SAMPLING_

#include <random>

#include <ceres/local_parameterization.h>

namespace grpose {

template <typename RandomBitsGenerator>
Vector3 AddGaussianDirectionNoise(RandomBitsGenerator &generator,
                                  const Vector3 &direction,
                                  double angle_deviation);

// Implementation

template <typename RandomBitsGenerator>
Vector3 AddGaussianDirectionNoise(RandomBitsGenerator &generator,
                                  const Vector3 &direction,
                                  double angle_deviation) {
  std::uniform_real_distribution<double> phi_distribution(0, 2 * M_PI);
  std::normal_distribution<double> theta_distribution(0, angle_deviation);
  const double phi = phi_distribution(generator);
  const double theta = theta_distribution(generator);
  const double delta[2] = {theta * std::cos(phi), theta * std::sin(phi)};

  Vector3 noisy_direction;
  const ceres::HomogeneousVectorParameterization parameterization(3);
  parameterization.Plus(direction.data(), delta, noisy_direction.data());
  return noisy_direction;
}

}  // namespace grpose

#endif