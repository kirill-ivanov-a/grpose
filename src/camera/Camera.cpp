#include "camera/Camera.h"
#include "util.h"

namespace grpose {

Camera::Camera(int width, int height,
               const Camera::CameraModelVariants &camera_model)
    : camera_model_(camera_model), width_(width), height_(height) {}

bool Camera::IsOnImage(const Vector2 &p, int border) const {
  bool is_in_border =
      Eigen::AlignedBox2d(Vector2(border, border),
                          Vector2(width_ - border, height_ - border))
          .contains(p);
  return is_in_border && mask_(ToCvPoint(p));
}

}  // namespace grpose