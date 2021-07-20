#include "CameraBundle.h"

namespace mcam {

CameraBundle::CameraEntry::CameraEntry(const SE3 &_bodyToThis,
                                       const CameraModel &cam)
    : bodyToThis(_bodyToThis), thisToBody(_bodyToThis.inverse()), cam(cam) {}

// NOTE: default constructor here for convenience and testing
CameraBundle::CameraBundle() {}

CameraBundle::CameraBundle(SE3 bodyToCam[], CameraModel cam[], int size) {
  bundle.reserve(size);
  for (int i = 0; i < size; ++i) bundle.emplace_back(bodyToCam[i], cam[i]);
}

void CameraBundle::setCamToBody(int ind, const SE3 &camToBody) {
  CHECK_GE(ind, 0);
  CHECK_LT(ind, bundle.size());

  bundle[ind].bodyToThis = camToBody.inverse();
  bundle[ind].thisToBody = camToBody;
}

CameraBundle::CamPyr CameraBundle::camPyr(int pyrLevels) const {
  CHECK_GT(pyrLevels, 0);

  std::vector<CameraModel::CamPyr> pyramids(bundle.size());
  for (int ci = 0; ci < bundle.size(); ++ci)
    pyramids[ci] = bundle[ci].cam.camPyr(pyrLevels);

  StdVectorA<SE3> bodyToCam;
  bodyToCam.reserve(bundle.size());
  for (const auto &e : bundle) bodyToCam.push_back(e.bodyToThis);

  CameraBundle::CamPyr result;
  result.reserve(pyrLevels);
  for (int pl = 0; pl < pyrLevels; ++pl) {
    StdVectorA<CameraModel> cams;
    cams.reserve(bundle.size());
    for (int ci = 0; ci < bundle.size(); ++ci) cams.push_back(pyramids[ci][pl]);
    result.emplace_back(bodyToCam.data(), cams.data(), bundle.size());
  }

  return result;
}

}  // namespace mcam
