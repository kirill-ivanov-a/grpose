#include "util/ceres_util.h"

#include "util/types.h"

namespace grpose {

bool SO3LocalParameterization::Plus(const double* x, const double* delta,
                                    double* x_plus_delta) const {
  Eigen::Map<const SO3> x_map(x);
  Eigen::Map<const SO3::Tangent> delta_map(delta);
  Eigen::Map<SO3> x_plus_delta_map(x_plus_delta);
  x_plus_delta_map = x_map * SO3::exp(delta_map);
  return true;
}

bool grpose::SO3LocalParameterization::ComputeJacobian(const double* x,
                                                       double* jacobian) const {
  Eigen::Map<const SO3> x_map(x);
  Eigen::Map<
      Eigen::Matrix<double, SO3::num_parameters, SO3::DoF, Eigen::RowMajor>>
      jacobian_map(jacobian);
  jacobian_map = x_map.Dx_this_mul_exp_x_at_0();
  return true;
}

int SO3LocalParameterization::GlobalSize() const { return SO3::num_parameters; }

int SO3LocalParameterization::LocalSize() const { return SO3::DoF; }

}  // namespace grpose