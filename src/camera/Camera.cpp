#include "camera/Camera.h"
#include "util.h"

namespace grpose {

Camera::Camera(int width, int height,
               const Camera::CameraModelVariants &cameraModel)
    : cameraModel_(cameraModel), width_(width), height_(height) {}

bool Camera::isOnImage(const Vector2 &p, int border) const {
  bool inBorder =
      Eigen::AlignedBox2d(Vector2(border, border),
                          Vector2(width_ - border, height_ - border))
          .contains(p);
  return inBorder && mask_(toCvPoint(p));
}

}  // namespace grpose