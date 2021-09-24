#ifndef GRPOSE_GRPOSE_MINIMAL_SOLVER_
#define GRPOSE_GRPOSE_MINIMAL_SOLVER_

#include "util/types.h"

namespace grpose {

class MinimalSolver {
 public:
  virtual ~MinimalSolver();

  virtual int MinSampleSize() const = 0;
  virtual bool SolveTimed(const std::vector<int> &correspondence_indices,
                          StdVectorA<SE3> &frame1_from_frame2,
                          double &time_in_seconds) const = 0;
  virtual bool Solve(const std::vector<int> &correspondence_indices,
                     StdVectorA<SE3> &frame1_from_frame2) const {
    double time_in_seconds;
    return SolveTimed(correspondence_indices, frame1_from_frame2,
                      time_in_seconds);
  }
};

}  // namespace grpose

#endif
