#ifndef GRPOSE_INITOPENGV_FEATUREDETECTORMATCHER_
#define GRPOSE_INITOPENGV_FEATUREDETECTORMATCHER_

#include <opencv2/opencv.hpp>
#include <opengv/types.hpp>

#include "camera/CameraBundle.h"
#include "dataset/DatasetReader.h"
#include "init_opengv/MultiCamInitSettings.h"
#include "types.h"

namespace grpose {

struct BearingVectorCorrespondences {
  BearingVectorCorrespondences(int number_of_cameras);

  void SetCamTimestamp(int camera_index, Timestamp current_timestamp,
                       Timestamp next_timestamp);

  // Make sure that all bearing vectors pushed are normalized !!!
  void PushBvc(const opengv::bearingVector_t &first_bearing_vector,
               const opengv::bearingVector_t &second_bearing_vector,
               int camera_correspondence);

  int number_of_cameras;
  int number_of_correspondences;
  std::vector<Timestamp> first_timestamps, second_timestamps;
  opengv::bearingVectors_t first_bearing_vectors, second_bearing_vectors;
  std::vector<int> first_correspondences, second_correspondences;
};

class FeatureDetectorMatcher {
 public:
  using FeatureType = FeatureDetectorMatcherSettings::FeatureType;

  FeatureDetectorMatcher(const CameraBundle &camera_bundle,
                         const FeatureDetectorMatcherSettings &settings);

  BearingVectorCorrespondences getBearingVectors(
      const std::vector<DatasetReader::FrameEntry> &first_frame_bundle,
      const std::vector<DatasetReader::FrameEntry> &second_frame_bundle);

  template <typename T>
  static double MapUnmapDistance(const Eigen::Matrix<T, 2, 1> &point,
                                 const Camera &camera) {
    return (point - camera.Map(camera.Unmap(point))).norm();
  }

 private:
  // Deprecated in favour of SIFT extraction and matching
  void DescriptorMatching(const cv::Mat &first_frame,
                          const cv::Mat &second_frame, const cv::Mat &mask,
                          cv::Ptr<cv::Feature2D> detector,
                          cv::Ptr<cv::Feature2D> descriptor_extractor,
                          std::vector<cv::KeyPoint> first_keypoints,
                          std::vector<cv::KeyPoint> &second_keypoints,
                          std::vector<cv::DMatch> &matches);

  // Deprecated in favour of SIFT extraction and matching
  void KltMatching(const cv::Mat &first_frame, const cv::Mat &second_frame,
                   const std::vector<cv::KeyPoint> &first_keypoints,
                   std::vector<cv::KeyPoint> &second_keypoints,
                   std::vector<cv::DMatch> &matches);

  std::vector<cv::KeyPoint> NonMaximumSuppression(
      const std::vector<cv::KeyPoint> &keypoints, float n_size);

  std::vector<cv::DMatch> RatioTestMatch(const cv::Mat &descriptors1,
                                         const cv::Mat &descriptors2);

  std::vector<cv::DMatch> CrossCheckMatches(
      const std::vector<cv::DMatch> &matches1,
      const std::vector<cv::DMatch> &matches2);

  CameraBundle camera_bundle_;
  cv::Ptr<cv::Feature2D> feature_detector_;
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher_;
  FeatureDetectorMatcherSettings settings_;
};

/* DEBUG functions */
void DrawKeypoints(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints);

void DrawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches);

//  Backproject each keypoint then project it into the image to create a
//  test point, compare the geometric error between the keypoint and the test
//  point, get an average geometric error which gets printed at the end
void CheckMapUnmap(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints,
                   const Camera &camera);

void CheckCorrespondences(
    const BearingVectorCorrespondences &correspondences,
    const CameraBundle &camera_bundle,
    const std::vector<DatasetReader::FrameEntry> &first_frame_bundle,
    const std::vector<DatasetReader::FrameEntry> &second_frame_bundle);

}  // namespace grpose

#endif