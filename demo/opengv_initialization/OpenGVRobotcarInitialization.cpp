#include <opengv/types.hpp>
#include <sophus/se3.hpp>

#include "openGVInitialization/FeatureDetectorMatcher.h"
#include "openGVInitialization/PoseFileWriter.h"

#include "dataset/MultiCamReader.h"
#include "dataset/RobotcarReader.h"
#include "util.h"

DEFINE_string(models_dir, "data/models/robotcar",
              "Directory with omnidirectional camera models. It is provided in "
              "our repository. IT IS NOT THE \"models\" DIRECTORY FROM THE "
              "ROBOTCAR DATASET SDK!");
DEFINE_string(extrinsics_dir, "ext/robotcar-dataset-sdk/extrinsics",
              "Directory with RobotCar dataset sensor extrinsics, provided in "
              "the dataset SDK.");
DEFINE_string(masks_dir, "data/masks/robotcar",
              "Directory with RobotCar dataset masks.");

grpose::SE3 getRobotcarGT(grpose::RobotcarReader rcReader, int currFrameInd,
                        int nextFrameInd) {
  // Getting relative pose to world for first frame
  grpose::SE3 worldFromCurr;
  if (rcReader.hasFrameToWorld(currFrameInd)) {
    worldFromCurr = rcReader.frameToWorld(currFrameInd);
  } else {
    throw std::runtime_error("No relative pose ground truth available");
  }

  // Getting relative pose for second frame
  grpose::SE3 worldFromNext;
  if (rcReader.hasFrameToWorld(nextFrameInd)) {
    worldFromNext = rcReader.frameToWorld(nextFrameInd);
  } else {
    throw std::runtime_error("No relative pose ground truth available");
  }

  grpose::SE3 currFromNext = worldFromCurr.inverse() * worldFromNext;
  grpose::SE3 gt_relative_poses = currFromNext;
  return gt_relative_poses;
}

void initialize(grpose::RobotcarReader rcReader, int currFrameInd,
                int nextFrameInd, std::string pathToChunk, int ransac_runs,
                bool visualize = false, bool writeOutput = true) {
  std::cout << "Selected frame " << currFrameInd << " and frame "
            << nextFrameInd << " in the chunk" << std::endl;

  std::vector<grpose::DatasetReader::FrameEntry> curr_frame_bundle,
      next_frame_bundle;

  try {
    curr_frame_bundle = rcReader.frame(currFrameInd);
    next_frame_bundle = rcReader.frame(nextFrameInd);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Images not read correctly... skipping frame";
    return;
  }

  /* Get bearing correspondences between frames */
  grpose::FeatureDetectorMatcherSettings fdmSettings;
  grpose::FeatureDetectorMatcher FDM =
      grpose::FeatureDetectorMatcher(fdmSettings, rcReader.cam());
  grpose::BearingVectorCorrespondences bvc =
      FDM.getBearingVectors(curr_frame_bundle, next_frame_bundle);

  /* DEBUG: displays the selected frames and found correspondences */
  if (visualize) {
    checkCorrespondences(bvc, rcReader.cam(), curr_frame_bundle,
                         next_frame_bundle);
  }

  /* Initialize using relative pose solver */
  // order {0, 1, 2} is {left, rear, right}
  grpose::StdVectorA<grpose::SE3> camera_extrinsics(3);
  camera_extrinsics[0] = rcReader.cam().camToBody(0);
  camera_extrinsics[1] = rcReader.cam().camToBody(1);
  camera_extrinsics[2] = rcReader.cam().camToBody(2);

  // Rough estimate for the three robotcar cameras
  const double focal_length = 410;
  const bool solver_verbose = (visualize) ? true : false;
  grpose::NonCentralRelativePoseSolverSettings solverSettings(focal_length,
                                                            solver_verbose);
  grpose::NonCentralRelativePoseSolver solver(solverSettings, camera_extrinsics);
  grpose::NonCentralRelativePoseSolution solution;
  try {
    solution = solver.solve(bvc, ransac_runs);

    /* Get ground truth relative pose */
    // Solution is SUCCESSFUL is solver succesful and ground truth is available
    try {
      solution.gt_relative_pose =
          getRobotcarGT(rcReader, currFrameInd, nextFrameInd);
      solution.status = solution.SUCCESSFUL;
    } catch (const std::exception &e) {
      LOG(WARNING) << e.what() << "\n";
      solution.status = solution.NO_GT;
    }

  } catch (const std::exception &e) {
    LOG(WARNING) << e.what() << '\n';
    solution.status = solution.FAILED;
  }

  solution.timestamps = std::pair<grpose::Timestamp, grpose::Timestamp>(
      rcReader.avgTimestamp(rcReader.timestampsFromInd(currFrameInd)),
      rcReader.avgTimestamp(rcReader.timestampsFromInd(nextFrameInd)));

  /* Write the results to a file */
  if (writeOutput) {
    grpose::PoseFileWriter PFW;
    PFW.writeToFile(pathToChunk, solution, currFrameInd, nextFrameInd, "");
  }
}

int main(int argc, char *argv[]) {
  CHECK(argc == 6 || argc == 8);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // set seed for reproducibility
  srand(0);

  std::string pathToChunk = argv[1];  // path to the main data folder
  std::string pathToRTK =
      argv[2];  // path to folder with all RTK for RobotCar chunks
  grpose::fs::path chunkDir(pathToChunk);
  grpose::fs::path rtkDir(pathToRTK);
  grpose::RobotcarReader rcReader(chunkDir, FLAGS_models_dir,
                                FLAGS_extrinsics_dir, rtkDir);
  rcReader.provideMasks(FLAGS_masks_dir);  // Provide the masks, IMPORTANT!!!

  int startingFrame = std::atoi(argv[3]);  // the startingFrame
  CHECK_GE(startingFrame, 0);
  CHECK_LT(startingFrame, rcReader.numFrames());

  if (argc == 6) {
    int endingFrame = std::atoi(argv[4]);
    int ransac_runs = std::atoi(argv[5]);
    CHECK_GT(endingFrame, startingFrame);
    CHECK_LT(endingFrame, rcReader.numFrames());

    LOG(INFO) << "Running for a single pair of frames"
              << "Frame 1: " << startingFrame << "Frame 2: " << endingFrame;

    const bool visualize = true;
    const bool writeOutput = false;
    initialize(rcReader, startingFrame, endingFrame, pathToChunk, ransac_runs,
               visualize, writeOutput);
  } else if (argc == 8) {
    int ransac_runs = std::atoi(argv[4]);       // averages over x iterations
    int skipFrames = std::atoi(argv[5]);        // skip every ith frame
    int framesWindowSize = std::atoi(argv[6]);  // initialize x frames
    int skipFramesWindow = std::atoi(argv[7]);

    CHECK_GT(skipFramesWindow, 0);
    CHECK_LE(skipFramesWindow, framesWindowSize);

    LOG(INFO) << "Running benchmark" << std::endl;

    std::cout << "Read " << rcReader.numFrames() << " frames from Robotcar"
              << std::endl;
    for (int currFrameInd = startingFrame;
         currFrameInd < rcReader.numFrames() - framesWindowSize;
         currFrameInd += skipFrames) {
      int win_idx = skipFramesWindow;
      while (win_idx <= framesWindowSize) {
        int nextFrameInd = currFrameInd + win_idx;
        initialize(rcReader, currFrameInd, nextFrameInd, pathToChunk,
                   ransac_runs);
        win_idx += skipFramesWindow;
      }
    }
  }
}
