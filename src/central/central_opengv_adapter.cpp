#include "central/central_opengv_adapter.h"

namespace grpose {

CentralOpengvAdapter::CentralOpengvAdapter(
    const CentralBearingVectorCorrespondences& correspondences)
    : correspondences_(correspondences),
      adapter_(correspondences_.bearing_vectors(0),
               correspondences_.bearing_vectors(1)) {}

}  // namespace grpose