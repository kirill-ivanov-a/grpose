#ifndef GRPOSE_GRPOSE_OPENGV_ADAPTER_
#define GRPOSE_GRPOSE_OPENGV_ADAPTER_

#include <opengv/relative_pose/NoncentralRelativeAdapter.hpp>
#include <opengv/relative_pose/NoncentralRelativeMultiAdapter.hpp>

#include "grpose/bearing_vector_correspondences.h"

namespace grpose {

/**
 * The class that holds all necessary information for
 * opengv::relative_pose::NoncentralRelativeAdapter and
 * opengv::relative_pose::NoncentralRelativeMultiAdapter. Since original
 * adapters only hold references, we need to manage lifetime of the relevant
 * objects separately.
 */
class OpengvAdapter {
 public:
  // two different formulations of RANSAC in OpenGV
  using OpengvInternalAdapter =
      opengv::relative_pose::NoncentralRelativeAdapter;
  using OpengvInternalMultiAdapter =
      opengv::relative_pose::NoncentralRelativeMultiAdapter;

  // TODO In principle, it should be possible to run OpenGV algorithms for
  // relative pose between different rigs, but then this conversion becomes more
  // cumbersome, since the adapter's class accepts a single set of camera
  // intrinsics.
  OpengvAdapter(
      const std::shared_ptr<BearingVectorCorrespondences> &correspondences,
      const StdVectorA<SE3> &body_from_cameras);

  inline OpengvInternalAdapter &GetAdapter();
  inline const OpengvInternalAdapter &GetAdapter() const;

  inline OpengvInternalMultiAdapter &GetMultiAdapter();
  inline const OpengvInternalMultiAdapter &GetMultiAdapter() const;

  inline BearingVectorCorrespondences &GetCorrespondences();
  inline const BearingVectorCorrespondences &GetCorrespondences() const;

 private:
  static OpengvInternalMultiAdapter ConstructMultiAdapter(
      const BearingVectorCorrespondences &correspondences,
      const opengv::translations_t &body_from_camera_translations,
      const opengv::rotations_t &body_from_camera_rotations);

  std::shared_ptr<BearingVectorCorrespondences> correspondences_;
  opengv::translations_t body_from_camera_translations_;
  opengv::rotations_t body_from_camera_rotations_;
  OpengvInternalAdapter adapter_;
  OpengvInternalMultiAdapter multi_adapter_;
};

// Implementation

OpengvAdapter::OpengvInternalAdapter &OpengvAdapter::GetAdapter() {
  return adapter_;
}

const OpengvAdapter::OpengvInternalAdapter &OpengvAdapter::GetAdapter() const {
  return adapter_;
}

OpengvAdapter::OpengvInternalMultiAdapter &OpengvAdapter::GetMultiAdapter() {
  return multi_adapter_;
}

const OpengvAdapter::OpengvInternalMultiAdapter &
OpengvAdapter::GetMultiAdapter() const {
  return multi_adapter_;
}

BearingVectorCorrespondences &OpengvAdapter::GetCorrespondences() {
  return *correspondences_;
}

const BearingVectorCorrespondences &OpengvAdapter::GetCorrespondences() const {
  return *correspondences_;
}

}  // namespace grpose

#endif