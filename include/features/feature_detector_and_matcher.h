#ifndef GRPOSE_FEATURES_FEATURE_DETECTOR_AND_MATCHER_
#define GRPOSE_FEATURES_FEATURE_DETECTOR_AND_MATCHER_

#include <opencv2/features2d.hpp>

#include "central/central_point2d_correspondences.h"
#include "dataset/dataset_reader.h"
#include "grpose/point2d_correspondences.h"

namespace grpose {

struct SiftSettings {
  int number_of_features = 0;  // 0 means to retain all features
  int number_of_octave_layers = 3;
  double contrast_threshold = 0.04;
  double edge_threshold = 10;
  double sigma = 1.6;

  double min_second_best_ratio = 1.3;
};

struct OrbSettings {
  int number_of_features = 2000;
  float scale_factor = 1.2f;
  int number_of_levels = 8;
  int edge_threshold = 31;
  int first_level = 0;
  int wta_k = 2;
  decltype(cv::ORB::HARRIS_SCORE) score_type = cv::ORB::HARRIS_SCORE;
  int patch_size = 31;
  int fast_threshold = 20;

  // First-to-second-best ratio test is currently not applied for ORB matches
  double min_second_best_ratio = 1.0;
};

struct FeatureDetectorAndMatcherSettings {
  enum FeatureType { kSift, kOrb };
  FeatureType feature_type = kSift;

  SiftSettings sift_settings;
  OrbSettings orb_settings;

  // nms == non-maximum suppression
  float nms_window_size = 5.0;
  bool cross_check_matches = true;
  bool use_non_maximal_suppression = true;
  bool debug_draw_matches = false;

  float min_second_best_ratio() const;
};

class FeatureDetectorAndMatcher {
 public:
  using FeatureType = FeatureDetectorAndMatcherSettings::FeatureType;

  FeatureDetectorAndMatcher(const FeatureDetectorAndMatcherSettings &settings);

  CentralPoint2dCorrespondences GetCorrespondences(
      const cv::Mat3b &frame1, const cv::Mat3b &frame2,
      const cv::Mat1b *mask1 = nullptr, const cv::Mat1b *mask2 = nullptr) const;

 private:
  std::vector<cv::KeyPoint> NonMaximumSuppression(
      const std::vector<cv::KeyPoint> &keypoints, float window_size) const;
  std::vector<cv::DMatch> RatioTestMatch(const cv::Mat &descriptors1,
                                         const cv::Mat &descriptors2) const;
  std::vector<cv::DMatch> CrossCheckMatches(
      const std::vector<cv::DMatch> &matches1,
      const std::vector<cv::DMatch> &matches2) const;
  std::vector<cv::DMatch> Match(const cv::Mat &descriptors1,
                                const cv::Mat &descriptors2) const;

  cv::Ptr<cv::Feature2D> feature_detector_;
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher_;
  FeatureDetectorAndMatcherSettings settings_;
};

}  // namespace grpose

#endif
