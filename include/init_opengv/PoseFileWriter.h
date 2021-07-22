#ifndef GRPOSE_INITOPENGV_POSEFILEWRITER_
#define GRPOSE_INITOPENGV_POSEFILEWRITER_
#include <fstream>
#include <iostream>

#include "init_opengv/NonCentralRelativePoseSolver.h"

namespace grpose {

class PoseFileWriter {
 public:
  void writeToFile(std::string path,
                   grpose::NonCentralRelativePoseSolution solution,
                   int currFrameInd, int nextFrameInd,
                   std::string folder_parameters);
};

}  // namespace grpose

#endif