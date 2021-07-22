#ifndef INCLUDE_CAMERABUNDLE
#define INCLUDE_CAMERABUNDLE

#include <glog/logging.h>
#include "camera/Camera.h"

namespace grpose {

class CameraBundle {
 public:
  using CamPyr = StdVectorA<CameraBundle>;

  CameraBundle();  // NOTE: default constructor here for convenience and testing
  CameraBundle(SE3 bodyToCam[], Camera cam[], int size);

  inline int numCams() const { return bundle_.size(); }

  inline Camera &cam(int ind) {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle_.size());

    return bundle_[ind].cam;
  }

  inline const Camera &cam(int ind) const {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle_.size());

    return bundle_[ind].cam;
  }

  inline SE3 camToBody(int ind) const {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle_.size());

    return bundle_[ind].thisToBody;
  }

  inline SE3 bodyToCam(int ind) const {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle_.size());

    return bundle_[ind].bodyToThis;
  }

  void setCamToBody(int ind, const SE3 &camToBody);

 private:
  struct CameraEntry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraEntry(const SE3 &_bodyToThis, const Camera &cam);

    SE3 bodyToThis;
    SE3 thisToBody;
    Camera cam;
  };

  StdVectorA<CameraEntry> bundle_;
};

}  // namespace grpose

#endif
