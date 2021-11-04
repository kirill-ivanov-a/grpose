#ifndef GRPOSE_CENTRAL_CENTRAL_OPENGV_ADAPTER_
#define GRPOSE_CENTRAL_CENTRAL_OPENGV_ADAPTER_

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>

#include "central/central_bearing_vector_correspondences.h"

namespace grpose {

class CentralOpengvAdapter {
 public:
  using OpengvInternalAdapter = opengv::relative_pose::CentralRelativeAdapter;

  CentralOpengvAdapter(
      const CentralBearingVectorCorrespondences &correspondences);

  inline OpengvInternalAdapter &GetAdapter();

 private:
  CentralBearingVectorCorrespondences correspondences_;
  OpengvInternalAdapter adapter_;
};

// Implementation

CentralOpengvAdapter::OpengvInternalAdapter &
CentralOpengvAdapter::GetAdapter() {
  return adapter_;
}

}  // namespace grpose

#endif