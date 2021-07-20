#ifndef INCLUDE_CAMERABUNDLE
#define INCLUDE_CAMERABUNDLE

#include <glog/logging.h>
#include "CameraModel.h"

namespace grpose {

class CameraBundle {
 public:
  using CamPyr = StdVectorA<CameraBundle>;

  CameraBundle();  // NOTE: default constructor here for convenience and testing
  CameraBundle(SE3 bodyToCam[], CameraModel cam[], int size);

  inline int numCams() const { return bundle.size(); }

  inline CameraModel &cam(int ind) {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle.size());

    return bundle[ind].cam;
  }

  inline const CameraModel &cam(int ind) const {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle.size());

    return bundle[ind].cam;
  }

  inline SE3 camToBody(int ind) const {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle.size());

    return bundle[ind].thisToBody;
  }

  inline SE3 bodyToCam(int ind) const {
    CHECK_GE(ind, 0);
    CHECK_LT(ind, bundle.size());

    return bundle[ind].bodyToThis;
  }

  void setCamToBody(int ind, const SE3 &camToBody);

  CamPyr camPyr(int levelNum) const;

 private:
  struct CameraEntry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraEntry(const SE3 &_bodyToThis, const CameraModel &cam);

    SE3 bodyToThis;
    SE3 thisToBody;
    CameraModel cam;
  };

  StdVectorA<CameraEntry> bundle;
};

}  // namespace grpose

#endif
