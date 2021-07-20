#include <opengv/types.hpp>
#include <sophus/se3.hpp>

#include "openGVInitialization/FeatureDetectorMatcher.h"
#include "openGVInitialization/NonCentralRelativePoseSolver.h"
#include "openGVInitialization/PoseFileWriter.h"

#include "dataset/DatasetReader.h"
#include "dataset/MultiCamReader.h"
#include "dataset/RobotcarReader.h"
#include "util.h"

DEFINE_string(rc_models_dir, "data/models/robotcar",
              "Directory with omnidirectional camera models. It is provided in "
              "our repository. IT IS NOT THE \"models\" DIRECTORY FROM THE "
              "ROBOTCAR DATASET SDK!");
DEFINE_string(rc_masks_dir, "data/masks/robotcar",
              "Directory with RobotCar dataset masks.");
DEFINE_string(rc_extrinsics_dir, "ext/robotcar-dataset-sdk/extrinsics",
              "Directory with RobotCar dataset sensor extrinsics, provided in "
              "the dataset SDK.");

grpose::SE3 getGT(grpose::DatasetReader *datasetReader, int currFrameInd,
                int nextFrameInd) {
  // Getting relative pose to world for first frame
  grpose::SE3 worldFromCurr;
  if (datasetReader->hasFrameToWorld(currFrameInd)) {
    worldFromCurr = datasetReader->frameToWorld(currFrameInd);
  } else {
    throw std::runtime_error("No relative pose ground truth available");
  }

  // Getting relative pose for second frame
  grpose::SE3 worldFromNext;
  if (datasetReader->hasFrameToWorld(nextFrameInd)) {
    worldFromNext = datasetReader->frameToWorld(nextFrameInd);
  } else {
    throw std::runtime_error("NO relative pose ground truth available");
  }

  grpose::SE3 currFromNext = worldFromCurr.inverse() * worldFromNext;
  grpose::SE3 gt_relative_poses = currFromNext;

  //   std::cout << "RAW GROUND TRUTH " << currFrameInd << " " << nextFrameInd
  //             << std::endl;
  //   std::cout << worldFromCurr.matrix3x4() << std::endl;
  //   std::cout << worldFromNext.matrix3x4() << std::endl;

  return gt_relative_poses;
}

void initialize(grpose::DatasetReader *datasetReader, int currFrameInd,
                int nextFrameInd, std::string pathToChunk, int ransac_runs,
                int focal_length, std::string folder_parameters,
                bool visualize = false, bool writeOutput = true) {
  std::cout << "Selected frame " << currFrameInd << " and " << nextFrameInd
            << " in the chunk" << std::endl;

  std::vector<grpose::DatasetReader::FrameEntry> curr_frame_bundle,
      next_frame_bundle;

  try {
    curr_frame_bundle = datasetReader->frame(currFrameInd);
    next_frame_bundle = datasetReader->frame(nextFrameInd);
  } catch (const std::exception &e) {
    LOG(ERROR) << "Images not read correctly... skipping frame";
    return;
  }

  /* Get bearing correspondences between frames */
  grpose::FeatureDetectorMatcherSettings fdmSettings;
  grpose::FeatureDetectorMatcher FDM =
      grpose::FeatureDetectorMatcher(fdmSettings, datasetReader->cam());
  grpose::BearingVectorCorrespondences bvc =
      FDM.getBearingVectors(curr_frame_bundle, next_frame_bundle);

  /* DEBUG */
  if (visualize) {
    checkCorrespondences(bvc, datasetReader->cam(), curr_frame_bundle,
                         next_frame_bundle);
  }

  /* Reading the extrinsic calibrations */
  std::cout << "Reading extrinsics calibrations (camera to body)" << std::endl;
  grpose::CameraBundle cb = datasetReader->cam();

  grpose::StdVectorA<grpose::SE3> camera_extrinsics;
  for (int c = 0; c < cb.numCams(); c++) {
    grpose::SE3 bodyFromCam = cb.camToBody(c);
    camera_extrinsics.push_back(bodyFromCam);
  }

  const bool solver_verbose = (visualize) ? true : false;
  grpose::NonCentralRelativePoseSolverSettings solverSettings(focal_length,
                                                            solver_verbose);

  // Solver constructor expect calibrations from image frame to the body frame
  grpose::NonCentralRelativePoseSolver solver(solverSettings, camera_extrinsics);
  grpose::NonCentralRelativePoseSolution solution;

  // Checking to make sure we're using the right algorithms
  std::cout << "Using feature detect: " << fdmSettings.feature_type
            << std::endl;
  std::cout << "Using algorithm: " << solverSettings.algorithm << std::endl;

  try {
    solution = solver.solve(bvc, ransac_runs);

    /* Get ground truth relative pose */
    // Solution is SUCCESSFUL if solver successful and GT available
    try {
      solution.gt_relative_pose =
          getGT(datasetReader, currFrameInd, nextFrameInd);
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
      datasetReader->avgTimestamp(
          datasetReader->timestampsFromInd(currFrameInd)),
      datasetReader->avgTimestamp(
          datasetReader->timestampsFromInd(nextFrameInd)));

  std::cout << "found solution: \n"
            << solution.relative_pose.matrix3x4() << std::endl;
  std::cout << "gt solution: \n"
            << solution.gt_relative_pose.matrix3x4() << std::endl;

  /* Write the results to a file */
  if (writeOutput) {
    grpose::PoseFileWriter PFW;
    PFW.writeToFile(pathToChunk, solution, currFrameInd, nextFrameInd,
                    folder_parameters);
  }
}

/*
Main method
 */
int main(int argc, char *argv[]) {
  // Should we check the number of arguments?
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Set seed for reproducibility
  srand(0);

  std::string pathToChunk = argv[1];  // Use this to check which dataset
  grpose::fs::path chunkDir(pathToChunk);

  // Check which dataset we are using
  if (grpose::MultiCamReader::isMultiCam(chunkDir)) {
    std::cout << "mcam" << std::endl;
    LOG(INFO) << "Running initialization on mcam dataset" << std::endl;

    CHECK(argc == 5 || argc == 8);
    grpose::MultiCamReaderSettings mcReaderSettings;

    std::unique_ptr<grpose::DatasetReader> datasetReader(
        new grpose::MultiCamReader(chunkDir, mcReaderSettings));

    int startingFrame =
        std::atoi(argv[2]);  // the starting frame (index for chunk starts at 0)
    CHECK_GE(startingFrame, 0);
    CHECK_LT(startingFrame, datasetReader.get()->numFrames());
    int focal_length = 410;

    if (argc == 5) {
      int endingFrame = std::atoi(argv[3]);
      int ransac_runs = std::atoi(argv[4]);
      CHECK_GT(endingFrame, startingFrame);
      CHECK_LT(endingFrame, datasetReader.get()->numFrames());

      LOG(INFO) << "Running for a single pair of frames"
                << "Frame 1: " << startingFrame << "Frame 2: " << endingFrame;

      const bool visualize = true;
      const bool writeOutput = false;
      initialize(datasetReader.get(), startingFrame, endingFrame, pathToChunk,
                 ransac_runs, focal_length, "", visualize, writeOutput);
    } else if (argc == 8) {
      int ransac_runs = std::atoi(argv[3]);       // averages over x iterations
      int skipFrames = std::atoi(argv[4]);        // skip every ith frame
      int framesWindowSize = std::atoi(argv[5]);  // initialize x frames
      int skipFramesWindow = std::atoi(argv[6]);
      std::string folder_parameters = argv[7];

      CHECK_GE(skipFramesWindow, 0);
      CHECK_LE(skipFramesWindow, framesWindowSize);

      LOG(INFO) << "Running benchmark" << std::endl;

      std::cout << "Read " << datasetReader.get()->numFrames()
                << " frames from mcam" << std::endl;
      for (int currFrameInd = startingFrame;
           currFrameInd < datasetReader.get()->numFrames() - framesWindowSize;
           currFrameInd += skipFrames) {
        int win_idx = skipFramesWindow;
        while (win_idx <= framesWindowSize) {
          int nextFrameInd = currFrameInd + win_idx;
          initialize(datasetReader.get(), currFrameInd, nextFrameInd,
                     pathToChunk, ransac_runs, focal_length, folder_parameters);
          win_idx += skipFramesWindow;
        }
      }
    }
  } else if (grpose::RobotcarReader::isRobotcar(chunkDir)) {
    LOG(INFO) << "Running initialization on Robotcar dataset" << std::endl;

    CHECK(argc == 6 || argc == 9);
    std::string pathToRTK =
        argv[2];  // path to folder with all RTK for RobotCar chunks
    grpose::fs::path rtkDir(pathToRTK);
    grpose::RobotcarReader rcReader(chunkDir, FLAGS_rc_models_dir,
                                  FLAGS_rc_extrinsics_dir, rtkDir);
    rcReader.provideMasks(
        FLAGS_rc_masks_dir);  // Provide the masks, IMPORTANT!!!

    std::unique_ptr<grpose::DatasetReader> datasetReader(
        new grpose::RobotcarReader(rcReader));  // Initialize as a pointer

    int startingFrame = std::atoi(argv[3]);  // the startingFrame
    CHECK_GE(startingFrame, 0);
    CHECK_LT(startingFrame, rcReader.numFrames());
    int focal_length = 410;  // is this supposed to be the same as mcam?

    if (argc == 6) {
      int endingFrame = std::atoi(argv[4]);
      int ransac_runs = std::atoi(argv[5]);
      CHECK_GT(endingFrame, startingFrame);
      CHECK_LT(endingFrame, rcReader.numFrames());

      LOG(INFO) << "Running for a single pair of frames"
                << "Frame 1: " << startingFrame << "Frame 2: " << endingFrame;

      const bool visualize = true;
      const bool writeOutput = false;
      initialize(datasetReader.get(), startingFrame, endingFrame, pathToChunk,
                 ransac_runs, focal_length, "", visualize, writeOutput);
    } else if (argc == 9) {
      int ransac_runs = std::atoi(argv[4]);       // averages over x iterations
      int skipFrames = std::atoi(argv[5]);        // skip every ith frame
      int framesWindowSize = std::atoi(argv[6]);  // initialize x frames
      int skipFramesWindow = std::atoi(argv[7]);
      std::string folder_parameters = argv[8];

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
          initialize(datasetReader.get(), currFrameInd, nextFrameInd,
                     pathToChunk, ransac_runs, focal_length, folder_parameters);
          win_idx += skipFramesWindow;
        }
      }
    }
  } else {
    LOG(ERROR) << "Unknown dataset provided in " << chunkDir.string();
  }
}