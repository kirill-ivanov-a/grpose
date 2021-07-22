#include <opengv/types.hpp>
#include <sophus/se3.hpp>

#include "init_opengv/FeatureDetectorMatcher.h"
#include "init_opengv/NonCentralRelativePoseSolver.h"
#include "init_opengv/PoseFileWriter.h"

#include "dataset/MultiCamReader.h"
#include "util.h"

grpose::SE3 getMulticamGT(grpose::MultiCamReader mcReader, int currFrameInd,
                          int nextFrameInd) {
  // Getting relative pose to world for first frame
  grpose::SE3 worldFromCurr;
  if (mcReader.hasFrameToWorld(currFrameInd)) {
    worldFromCurr = mcReader.frameToWorld(currFrameInd);
  } else {
    throw std::runtime_error("No relative pose ground truth available");
  }

  // Getting relative pose for second frame
  grpose::SE3 worldFromNext;
  if (mcReader.hasFrameToWorld(nextFrameInd)) {
    worldFromNext = mcReader.frameToWorld(nextFrameInd);
  } else {
    throw std::runtime_error("NO relative pose ground truth available");
  }

  grpose::SE3 currFromNext = worldFromCurr.inverse() * worldFromNext;
  grpose::SE3 gt_relative_poses = currFromNext;

  std::cout << "RAW GROUND TRUTH " << currFrameInd << " " << nextFrameInd
            << std::endl;
  std::cout << worldFromCurr.matrix3x4() << std::endl;
  std::cout << worldFromNext.matrix3x4() << std::endl;

  return gt_relative_poses;
}

void initialize(grpose::MultiCamReader mcReader, int currFrameInd,
                int nextFrameInd, std::string pathToChunk, int ransac_runs,
                bool visualize = false, bool writeOutput = true) {
  std::cout << "Selected frame " << currFrameInd << " in the chunk"
            << std::endl;
  std::vector<grpose::DatasetReader::FrameEntry> curr_frame_bundle =
      mcReader.frame(currFrameInd);
  std::vector<grpose::DatasetReader::FrameEntry> next_frame_bundle =
      mcReader.frame(nextFrameInd);

  /* Get bearing correspondences between frames */
  grpose::FeatureDetectorMatcherSettings fdmSettings;
  grpose::FeatureDetectorMatcher FDM =
      grpose::FeatureDetectorMatcher(fdmSettings, mcReader.cam());
  grpose::BearingVectorCorrespondences bvc =
      FDM.getBearingVectors(curr_frame_bundle, next_frame_bundle);

  /* DEBUG */
  if (visualize) {
    checkCorrespondences(bvc, mcReader.cam(), curr_frame_bundle,
                         next_frame_bundle);
  }

  /* Initialize using relative pose solver */
  // order {0, 1, 2, 3} is {front, right, back, left}
  grpose::StdVectorA<grpose::SE3> camera_extrinsics(4);
  camera_extrinsics[0] = mcReader.cam().camToBody(0);
  camera_extrinsics[1] = mcReader.cam().camToBody(1);
  camera_extrinsics[2] = mcReader.cam().camToBody(2);
  camera_extrinsics[3] = mcReader.cam().camToBody(3);

  const double focal_length = 410;
  const bool solver_verbose = (visualize) ? true : false;
  grpose::NonCentralRelativePoseSolverSettings solverSettings(
      focal_length, solver_verbose);  // what is the focal length for mcam?

  // Solver constructor expect calibrations from image frame to the body frame
  grpose::NonCentralRelativePoseSolver solver(solverSettings,
                                              camera_extrinsics);
  grpose::NonCentralRelativePoseSolution solution;
  try {
    solution = solver.solve(bvc, ransac_runs);

    /* Get ground truth relative pose */
    // Solution is SUCCESSFUL is solver succesful and ground truth is available
    try {
      solution.gt_relative_pose =
          getMulticamGT(mcReader, currFrameInd, nextFrameInd);
      solution.status = solution.SUCCESSFUL;
    } catch (const std::exception &e) {
      LOG(WARNING) << e.what() << std::endl;
      solution.status = solution.NO_GT;
    }

  } catch (const std::exception &e) {
    LOG(WARNING) << e.what() << '\n';
    solution.status = solution.FAILED;
  }

  solution.timestamps = std::pair<grpose::Timestamp, grpose::Timestamp>(
      mcReader.avgTimestamp(mcReader.timestampsFromInd(currFrameInd)),
      mcReader.avgTimestamp(mcReader.timestampsFromInd(nextFrameInd)));

  /* Write the results to a file */
  if (writeOutput) {
    grpose::PoseFileWriter PFW;
    PFW.writeToFile(pathToChunk, solution, currFrameInd, nextFrameInd, "");
  }
}

int main(int argc, char *argv[]) {
  CHECK(argc == 5 || argc == 7);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // set seed for reproducibility
  srand(0);

  std::string pathToChunk = argv[1];  // path to the main data folder
  grpose::fs::path chunkDir(pathToChunk);
  grpose::MultiCamReaderSettings mcReaderSettings;
  grpose::MultiCamReader mcReader(chunkDir, mcReaderSettings);

  int startingFrame =
      std::atoi(argv[2]);  // the starting frame (index for chunk starts at 0)
  CHECK_GE(startingFrame, 0);
  CHECK_LT(startingFrame, mcReader.numFrames());

  if (argc == 5) {
    int endingFrame = std::atoi(argv[3]);
    int ransac_runs = std::atoi(argv[4]);
    CHECK_GT(endingFrame, startingFrame);
    CHECK_LT(endingFrame, mcReader.numFrames());

    LOG(INFO) << "Running for a single pair of frames"
              << "Frame 1: " << startingFrame << "Frame 2: " << endingFrame;

    const bool visualize = true;
    const bool writeOutput = false;
    initialize(mcReader, startingFrame, endingFrame, pathToChunk, ransac_runs,
               visualize, writeOutput);
  } else if (argc == 7) {
    int ransac_runs = std::atoi(argv[3]);       // averages over x iterations
    int skipFrames = std::atoi(argv[4]);        // skip every ith frame
    int framesWindowSize = std::atoi(argv[5]);  // initialize x frames
    int skipFramesWindow = std::atoi(argv[6]);

    CHECK_GT(skipFramesWindow, 0);
    CHECK_LE(skipFramesWindow, framesWindowSize);

    LOG(INFO) << "Running benchmark" << std::endl;

    std::cout << "Read " << mcReader.numFrames() << " frames from mcam"
              << std::endl;
    for (int currFrameInd = startingFrame;
         currFrameInd < mcReader.numFrames() - framesWindowSize;
         currFrameInd += skipFrames) {
      int win_idx = skipFramesWindow;
      while (win_idx <= framesWindowSize) {
        int nextFrameInd = currFrameInd + win_idx;
        initialize(mcReader, currFrameInd, nextFrameInd, pathToChunk,
                   ransac_runs);
        win_idx += skipFramesWindow;
      }
    }
  }
}