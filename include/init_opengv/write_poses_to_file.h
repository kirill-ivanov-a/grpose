#ifndef GRPOSE_INITOPENGV_WRITEPOSESTOFILE_
#define GRPOSE_INITOPENGV_WRITEPOSESTOFILE_

#include <fstream>
#include <iostream>

#include "init_opengv/non_central_relative_pose_solver.h"

namespace grpose {

void WritePosesToFile(const std::string &path,
                      const NonCentralRelativePoseSolution &solution,
                      int currFrameInd, int nextFrameInd,
                      const std::string &folder_parameters);

}  // namespace grpose

#endif