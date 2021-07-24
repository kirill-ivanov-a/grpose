#ifndef GRPOSE_INITOPENGV_POSEFILEWRITER_
#define GRPOSE_INITOPENGV_POSEFILEWRITER_
#include <fstream>
#include <iostream>

#include "init_opengv/NonCentralRelativePoseSolver.h"

namespace grpose {

void WriteToFile(const std::string &path,
                 const NonCentralRelativePoseSolution &solution,
                 int currFrameInd, int nextFrameInd,
                 const std::string &folder_parameters);

}  // namespace grpose

#endif