#ifndef GRPOSE_GRPOSE_OPENGV_ADAPTER_
#define GRPOSE_GRPOSE_OPENGV_ADAPTER_

#include "grpose/bearing_vector_correspondences.h"

namespace grpose {

/**
 * The class that holds all necessary information for
 * opengv::relative_pose::NoncentralRelativeAdapter. Since the original adapter
 * only holds references, we need to manage lifetime of the relevant objects
 * separately.
 */
class OpengvAdapter {
 public:
  using OpengvInternalAdapter =
      opengv::relative_pose::NoncentralRelativeAdapter;

  // TODO In principle, it should be possible to run OpenGV algorithms for
  // relative pose between different rigs, but then this conversion becomes more
  // cumbersome, since the adapter's class accepts a single set of camera
  // intrinsics.
  OpengvAdapter(
      const std::shared_ptr<BearingVectorCorrespondences> &correspondences,
      const StdVectorA<SE3> &body_from_cameras);

  inline OpengvInternalAdapter &Get();
  inline const OpengvInternalAdapter &Get() const;
  inline BearingVectorCorrespondences &GetCorrespondences();
  inline const BearingVectorCorrespondences &GetCorrespondences() const;

 private:
  std::shared_ptr<BearingVectorCorrespondences> correspondences_;
  opengv::translations_t body_from_camera_translations_;
  opengv::rotations_t body_from_camera_rotations_;
  OpengvInternalAdapter adapter_;
};

// Implementation

OpengvAdapter::OpengvInternalAdapter &OpengvAdapter::Get() { return adapter_; }

const OpengvAdapter::OpengvInternalAdapter &OpengvAdapter::Get() const {
  return adapter_;
}

BearingVectorCorrespondences &OpengvAdapter::GetCorrespondences() {
  return *correspondences_;
}

const BearingVectorCorrespondences &OpengvAdapter::GetCorrespondences() const {
  return *correspondences_;
}

}  // namespace grpose

#endif