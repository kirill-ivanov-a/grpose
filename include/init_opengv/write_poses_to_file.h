#ifndef GRPOSE_INIT_OPENGV_WRITE_POSES_TO_FILE_
#define GRPOSE_INIT_OPENGV_WRITE_POSES_TO_FILE_

#include <fstream>
#include <iostream>

#include "init_opengv/non_central_relative_pose_solver_old.h"

namespace grpose {

void WritePosesToFile(const std::string &path,
                      const NonCentralRelativePoseSolution &solution,
                      int currFrameInd, int nextFrameInd,
                      const std::string &folder_parameters);

}  // namespace grpose

#endif