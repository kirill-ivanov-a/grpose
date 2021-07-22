#ifndef GRPOSE_DATASET_MULTICAMREADER_
#define GRPOSE_DATASET_MULTICAMREADER_

#include "DatasetReader.h"

namespace grpose {

struct MultiCamReaderSettings {
  static const std::vector<std::string> default_camNames;
  std::vector<std::string> camNames = default_camNames;

  int numCams() const;
};

/**
 * The implementation of `DatasetReader` for the modified Multi-FoV dataset that
 * was generated for a surround-view-like 4 camera system.
 */
class MultiCamReader : public DatasetReader {
  static constexpr int mNumFrames = 3000;
  static constexpr int imgWidth = 640, imgHeight = 480;

  // We use a fixed step between images, and we postulate that it roughly equals
  // 30 frames per second. The whole dataset thus represents 1.5 minutes of
  // driving with really sharp turns without the decrease in speed.
  static constexpr Timestamp tsPerFrame = 30000;

 public:
  static bool isMultiCam(const fs::path &datasetDir);

  MultiCamReader(const fs::path &datasetDir,
                 const MultiCamReaderSettings &settings = {});

  int numFrames() const override;

  int firstTimestampToInd(Timestamp timestamp) const override;

  std::vector<Timestamp> timestampsFromInd(int frameInd) const override;

  std::vector<fs::path> frameFiles(int frameInd) const override;

  std::vector<FrameEntry> frame(int frameInd) const override;

  CameraBundle cam() const override;

  std::unique_ptr<FrameDepths> depths(int frameInd) const override;

  bool hasFrameToWorld(int frameInd) const override;

  SE3 frameToWorld(int frameInd) const override;

  Trajectory gtTrajectory() const override;

 private:
  class Depths : public FrameDepths {
   public:
    Depths(const fs::path &datasetDir, int frameInd,
           const MultiCamReaderSettings &newSettings);

    std::optional<double> depth(int camInd,
                                const Vector2 &point) const override;

   private:
    std::vector<cv::Mat1f> depths;
    static const Eigen::AlignedBox2d boundingBox;

    MultiCamReaderSettings settings;
  };

  static CameraBundle createCameraBundle(
      const fs::path &datasetDir, const std::vector<std::string> &camNames);

  static StdVectorA<SE3> readBodyToWorld(const fs::path &datasetDir);

  MultiCamReaderSettings settings;
  fs::path datasetDir;
  CameraBundle mCam;
  StdVectorA<SE3> bodyToWorld;
};

}  // namespace grpose

#endif
