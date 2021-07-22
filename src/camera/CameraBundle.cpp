#include "camera/CameraBundle.h"

namespace grpose {

CameraBundle::CameraEntry::CameraEntry(const SE3 &_bodyToThis,
                                       const Camera &cam)
    : bodyToThis(_bodyToThis), thisToBody(_bodyToThis.inverse()), cam(cam) {}

// NOTE: default constructor here for convenience and testing
CameraBundle::CameraBundle() {}

CameraBundle::CameraBundle(SE3 bodyToCam[], Camera cam[], int size) {
  bundle_.reserve(size);
  for (int i = 0; i < size; ++i) bundle_.emplace_back(bodyToCam[i], cam[i]);
}

void CameraBundle::setCamToBody(int ind, const SE3 &camToBody) {
  CHECK_GE(ind, 0);
  CHECK_LT(ind, bundle_.size());

  bundle_[ind].bodyToThis = camToBody.inverse();
  bundle_[ind].thisToBody = camToBody;
}

}  // namespace grpose
