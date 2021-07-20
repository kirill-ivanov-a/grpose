#ifndef INCLUDE_FEATUREDETECTORMATCHER
#define INCLUDE_FEATUREDETECTORMATCHER

#include <types.h>
#include <opencv2/opencv.hpp>
#include <opengv/types.hpp>

#include "CameraBundle.h"
#include "dataset/DatasetReader.h"
#include "openGVInitialization/MultiCamInitSettings.h"

namespace grpose {

struct BearingVectorCorrespondences {
  int numCameras;
  int numCorrespondences;
  std::vector<Timestamp> currentTimestamps, nextTimestamps;
  opengv::bearingVectors_t bearingVectorsCurrent, bearingVectorsNext;
  std::vector<int> correspondencesCurrent, correspondencesNext;

  BearingVectorCorrespondences(int numCameras);

  void setCamTimestamp(int cameraInd, Timestamp currentTimestamp,
                       Timestamp nextTimestamp);

  // Make sure that all bearing vectors pushed are normalized !!!
  void pushBVC(opengv::bearingVector_t bearingVectorCurrent,
               opengv::bearingVector_t bearingVectorNext,
               int cameraCorrespondence);
};

class FeatureDetectorMatcher {
 public:
  using FeatureType = FeatureDetectorMatcherSettings::FeatureType;

  FeatureDetectorMatcher(FeatureDetectorMatcherSettings settings,
                         CameraBundle cameraBundle);

  BearingVectorCorrespondences getBearingVectors(
      const std::vector<DatasetReader::FrameEntry> &frame_bundle,
      const std::vector<DatasetReader::FrameEntry> &next_frame_bundle);

  template <typename T>
  static double mapUnmap(const Eigen::Matrix<T, 2, 1> &point,
                         const CameraModel &cameraModel) {
    return (point - cameraModel.map(cameraModel.unmap(point))).norm();
  }

 private:
  // Deprecated in favour of SIFT extraction and matching
  void descriptorMatching(const cv::Mat &current_frame,
                          const cv::Mat &next_frame, const cv::Mat &mask,
                          cv::Ptr<cv::Feature2D> detector,
                          cv::Ptr<cv::Feature2D> desc_extractor,
                          std::vector<cv::KeyPoint> keypoints_current,
                          std::vector<cv::KeyPoint> &keypoints_next,
                          std::vector<cv::DMatch> &matches);

  // Deprecated in favour of SIFT extraction and matching
  void kltMatching(const cv::Mat &current_frame, const cv::Mat &next_frame,
                   const std::vector<cv::KeyPoint> &keypoints_current,
                   std::vector<cv::KeyPoint> &keypoints_next,
                   std::vector<cv::DMatch> &matches);

  std::vector<cv::KeyPoint> nms(const std::vector<cv::KeyPoint> &keypoints,
                                float n_size);

  std::vector<cv::DMatch> ratioTestMatch(const cv::Mat &desc1,
                                         const cv::Mat &desc2);

  std::vector<cv::DMatch> crossCheckMatches(
      const std::vector<cv::DMatch> &matches1,
      const std::vector<cv::DMatch> &matches2);

  FeatureDetectorMatcherSettings settings;

  CameraBundle cameraBundle;

  cv::Ptr<cv::Feature2D> feat_detector;
  cv::Ptr<cv::DescriptorMatcher> desc_matcher;
};

/* DEBUG functions */
void drawKeypoints(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints);

void drawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches);

/*
  Backproject each keypoint then project it back into the image to create a test
  point compare the geometric error between the keypoint and the test point get
  an average geometric error which get's printed at the end
*/
void checkMapUnmap(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints,
                   const CameraModel &cameraModel);

void checkCorrespondences(
    const BearingVectorCorrespondences &correspondences,
    const CameraBundle &camera_bundle,
    const std::vector<DatasetReader::FrameEntry> &frame_bundle,
    const std::vector<DatasetReader::FrameEntry> &next_frame_bundle);

}  // namespace grpose

#endif