#include "camera/camera_model_pinhole.h"

namespace grpose {

Vector3 CameraModelPinhole::Unmap(const Vector2 &point,
                                  const std::vector<double> &parameters) {
  const double fx(parameters[fx_id_]), fy(parameters[fy_id_]);
  const double px(parameters[px_id_]), py(parameters[py_id_]);
  const double skew(parameters[skew_id_]);

  Vector3 direction(0.0, 0.0, 1.0);
  direction[1] = (point[1] - py) / fy;
  direction[0] = (point[0] - skew * direction[1] - px) / fx;
  return direction.normalized();
}

}  // namespace grpose
