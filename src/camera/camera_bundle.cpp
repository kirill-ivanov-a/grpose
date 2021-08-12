#include "camera/camera_bundle.h"

namespace grpose {

CameraBundle::CameraEntry::CameraEntry(const SE3 &this_from_body,
                                       const Camera &camera)
    : this_from_body(this_from_body),
      body_from_this(this_from_body.inverse()),
      camera(camera) {}

CameraBundle::CameraBundle(SE3 camera_from_body[], Camera cam[], int size) {
  bundle_.reserve(size);
  for (int i = 0; i < size; ++i)
    bundle_.emplace_back(camera_from_body[i], cam[i]);
}

void CameraBundle::set_body_from_camera(int index,
                                        const SE3 &body_from_camera) {
  CHECK_GE(index, 0);
  CHECK_LT(index, bundle_.size());

  bundle_[index].this_from_body = body_from_camera.inverse();
  bundle_[index].body_from_this = body_from_camera;
}

}  // namespace grpose
