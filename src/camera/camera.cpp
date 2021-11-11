#include "camera/camera.h"
#include "util/util.h"

namespace grpose {

Camera Camera::ColmapCamera(int width, int height, int colmap_model_id,
                            const std::vector<double> &parameters) {
  return Camera(ColmapConstructor(), width, height, colmap_model_id,
                parameters);
}

Camera::Camera(int width, int height, CameraModelId model_id,
               const std::vector<double> &parameters)
    : model_id_(model_id),
      parameters_(parameters),
      width_(width),
      height_(height) {}

Camera::Camera(ColmapConstructor, int width, int height, int colmap_model_id,
               const std::vector<double> &parameters)
    : colmap_model_id_(colmap_model_id),
      parameters_(parameters),
      width_(width),
      height_(height) {}

bool Camera::IsOnImage(const Vector2 &p, int border) const {
  bool is_in_border =
      Eigen::AlignedBox2d(Vector2(border, border),
                          Vector2(width_ - border, height_ - border))
          .contains(p);
  return is_in_border && mask_(ToCvPoint(p));
}

}  // namespace grpose