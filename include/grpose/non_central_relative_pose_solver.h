#ifndef GRPOSE_GRPOSE_NON_CENTRAL_RELATIVE_POSE_SOLVER_
#define GRPOSE_GRPOSE_NON_CENTRAL_RELATIVE_POSE_SOLVER_

#include "util/types.h"

namespace grpose {

class NonCentralRelativePoseSolver {
 public:
  virtual ~NonCentralRelativePoseSolver();

  virtual int MinimalNeededCorrespondences() const = 0;
  virtual bool Solve(const std::vector<int> &correspondence_indices,
                     StdVectorA<SE3> &frame1_from_frame2) const = 0;
};

}  // namespace grpose

#endif
