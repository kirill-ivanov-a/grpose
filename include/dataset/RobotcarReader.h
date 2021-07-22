#ifndef GRPOSE_DATASET_ROBOTCARREADER_
#define GRPOSE_DATASET_ROBOTCARREADER_

#include <filesystem>

#include <opencv2/opencv.hpp>
#include "dataset/DatasetReader.h"

namespace grpose {

struct RobotcarReaderSettings {
  RobotcarReaderSettings();

  CameraModelScaramuzzaSettings cam;

  static constexpr bool default_fillVoGaps = false;
  bool fillVoGaps = default_fillVoGaps;

  static constexpr bool default_correctRtk = true;
  bool correctRtk = default_correctRtk;

  static constexpr double default_projectedTimeWindow = 2;
  double projectedTimeWindow = default_projectedTimeWindow;

  static constexpr int default_maxProjectedPoints = 20'000;
  int maxProjectedPoints = default_maxProjectedPoints;

  static constexpr double default_boxFilterSize = 3;
  double boxFilterSize = default_boxFilterSize;

  static constexpr char default_outputDirectory[] = "output/default";
  fs::path outputDirectory = default_outputDirectory;
};

/**
 * The implementation of `DatasetReader` for Oxford RobotCar dataset.
 */
class RobotcarReader : public DatasetReader {
 public:
  static constexpr int imageWidth = 1024, imageHeight = 1024;
  static constexpr int numCams = 3;

  static bool isRobotcar(const fs::path &chunkDir);

  RobotcarReader(const fs::path &_chunkDir, const fs::path &modelsDir,
                 const fs::path &extrinsicsDir,
                 const std::optional<fs::path> &rtkDir = std::nullopt,
                 const RobotcarReaderSettings &_settings = {});

  void provideMasks(const fs::path &masksDir);

  int numFrames() const override;

  int firstTimestampToInd(Timestamp timestamp) const override;

  std::vector<Timestamp> timestampsFromInd(int frameInd) const override;

  std::vector<fs::path> frameFiles(int frameInd) const override;

  std::vector<FrameEntry> frame(int frameInd) const override;

  CameraBundle cam() const override { return mCam; }

  std::unique_ptr<FrameDepths> depths(int frameInd) const override;

  bool hasFrameToWorld(int frameInd) const override;

  SE3 frameToWorld(int frameInd) const override;

  Trajectory gtTrajectory() const override;

  SE3 tsToWorld(Timestamp ts, bool useVo = false) const;

  std::vector<Vector3> getLmsFrontCloud(Timestamp from, Timestamp to,
                                        Timestamp base) const;

  std::vector<Vector3> getLmsRearCloud(Timestamp from, Timestamp to,
                                       Timestamp base) const;

  std::vector<Vector3> getLdmrsCloud(Timestamp from, Timestamp to,
                                     Timestamp base) const;

  std::array<StdVectorA<std::pair<Vector2, double>>, numCams> project(
      Timestamp from, Timestamp to, Timestamp base, bool useLmsFront = true,
      bool useLmsRear = true, bool useLdmrs = true) const;
  std::array<StdVectorA<std::pair<Vector2, double>>, numCams>

  project(const std::vector<Vector3> &cloud) const;

  inline const std::vector<Timestamp> &lmsFrontTs() const {
    return mLmsFrontTs;
  }

  inline const StdVectorA<SE3> &getGtBodyToWorld() const {
    return gtBodyToWorld;
  }

  inline bool masksProvided() const { return mMasksProvided; }

  Timestamp minTs() const;

  Timestamp maxTs() const;

 private:
  static const SE3 camToImage;

  static CameraBundle createFromData(
      const fs::path &modelsDir, const SE3 &bodyToLeft, const SE3 &bodyToRear,
      const SE3 &bodyToRight, int w, int h,
      const CameraModelScaramuzzaSettings &camSettings);

  void syncTimestamps();

  void getPointCloudHelper(std::vector<Vector3> &cloud, const fs::path &scanDir,
                           const SE3 &sensorToBody, Timestamp base,
                           const std::vector<Timestamp> &timestamps,
                           Timestamp from, Timestamp to, bool isLdmrs) const;

  void printVoAndGT(const fs::path &outputDirectory) const;

  SE3 bodyToLeft, bodyToRear, bodyToRight;
  SE3 bodyToLmsFront, bodyToLmsRear;
  SE3 bodyToLdmrs;
  SE3 bodyToIns;
  StdVectorA<SE3> gtBodyToWorld;
  StdVectorA<SE3> voBodyToWorld;
  CameraBundle mCam;
  fs::path chunkDir;
  fs::path leftDir, rearDir, rightDir;
  fs::path lmsFrontDir, lmsRearDir;
  fs::path ldmrsDir;
  std::vector<Timestamp> mLeftTs, mRearTs, mRightTs;
  std::vector<Timestamp> mLmsFrontTs, mLmsRearTs;
  std::vector<Timestamp> mLdmrsTs;
  std::vector<Timestamp> mGroundTruthTs;
  std::vector<Timestamp> mVoTs;
  bool mMasksProvided;
  RobotcarReaderSettings settings;
};

}  // namespace grpose

#endif
