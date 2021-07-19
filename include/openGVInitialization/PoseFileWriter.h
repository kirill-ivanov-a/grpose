#ifndef INCLUDE_POSEFILEWRITER
#define INCLUDE_POSEFILEWRITER
#include <fstream>
#include <iostream>

#include "openGVInitialization/NonCentralRelativePoseSolver.h"

namespace mcam {

class PoseFileWriter {

public:
  void writeToFile(std::string path, mcam::NonCentralRelativePoseSolution solution, int currFrameInd, int nextFrameInd, std::string folder_parameters);
};

} // namespace mcam

#endif