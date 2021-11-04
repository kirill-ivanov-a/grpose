#include <ceres/ceres.h>

namespace grpose {

class SO3LocalParameterization : public ceres::LocalParameterization {
 public:
  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override;
  bool ComputeJacobian(const double* x, double* jacobian) const override;

  int GlobalSize() const override;
  int LocalSize() const override;
};

}  // namespace grpose