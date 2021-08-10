#ifndef GRPOSE_CAMERA_CAMERA_BUNDLE_
#define GRPOSE_CAMERA_CAMERA_BUNDLE_

#include <glog/logging.h>

#include "camera/camera.h"

namespace grpose {

class CameraBundle {
 public:
  CameraBundle(SE3 camera_from_body[], Camera cam[], int size);

  inline int NumberOfCameras() const { return bundle_.size(); }

  inline Camera &camera(int index) {
    CHECK_GE(index, 0);
    CHECK_LT(index, bundle_.size());

    return bundle_[index].camera;
  }

  inline const Camera &camera(int index) const {
    CHECK_GE(index, 0);
    CHECK_LT(index, bundle_.size());

    return bundle_[index].camera;
  }

  inline SE3 body_from_camera(int index) const {
    CHECK_GE(index, 0);
    CHECK_LT(index, bundle_.size());

    return bundle_[index].body_from_this;
  }

  inline SE3 camera_from_body(int index) const {
    CHECK_GE(index, 0);
    CHECK_LT(index, bundle_.size());

    return bundle_[index].this_from_body;
  }

  void SetBodyFromCamera(int index, const SE3 &body_from_camera);

 private:
  struct CameraEntry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraEntry(const SE3 &this_from_body, const Camera &camera);

    SE3 this_from_body;
    SE3 body_from_this;
    Camera camera;
  };

  StdVectorA<CameraEntry> bundle_;
};

}  // namespace grpose

#endif
