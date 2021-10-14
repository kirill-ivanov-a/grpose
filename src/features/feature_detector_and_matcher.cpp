#include "features/feature_detector_and_matcher.h"

namespace grpose {

namespace {

void DrawKeypoints(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints) {
  cv::Mat debug_image;
  cv::drawKeypoints(image, keypoints, debug_image);
  cv::imshow("Detected keypoints", debug_image);
  cv::waitKey(0);
}

void DrawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches) {
  cv::Mat debug_image;
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, debug_image);
  cv::Mat resized;
  cv::resize(debug_image, resized, cv::Size(), 0.5, 0.5);
  cv::imshow("Matched keypoints", resized);
  cv::waitKey(0);
}

}  // namespace

FeatureDetectorAndMatcher::FeatureDetectorAndMatcher(
    const FeatureDetectorAndMatcherSettings &settings)
    : settings_(settings) {
  decltype(cv::DescriptorMatcher::BRUTEFORCE_HAMMING) matcher_type;
  switch (settings.feature_type) {
    case FeatureType::kSift:
      feature_detector_ = cv::SIFT::create(
          settings.sift_settings.number_of_features,
          settings.sift_settings.number_of_octave_layers,
          settings.sift_settings.contrast_threshold,
          settings.sift_settings.edge_threshold, settings_.sift_settings.sigma);
      // For SIFT we do cross-checking manually
      descriptor_matcher_ = cv::BFMatcher::create(cv::NORM_L2, false);
      LOG(INFO) << "Feature detection and matching using SIFT";
      break;

    case FeatureType::kOrb:
      feature_detector_ = cv::ORB::create(
          settings.orb_settings.number_of_features,
          settings.orb_settings.scale_factor,
          settings.orb_settings.number_of_levels,
          settings.orb_settings.edge_threshold,
          settings.orb_settings.first_level, settings.orb_settings.wta_k,
          settings.orb_settings.score_type, settings.orb_settings.patch_size,
          settings.orb_settings.fast_threshold);
      descriptor_matcher_ = cv::BFMatcher::create(
          cv::NORM_HAMMING, settings_.cross_check_matches);
      LOG(INFO) << "Feature detection and matching using ORB";
      break;

    default:
      throw std::domain_error(
          fmt::format("Unknown feature type {}", settings_.feature_type));
  }
}

CentralPoint2dCorrespondences FeatureDetectorAndMatcher::GetCorrespondences(
    const cv::Mat3b &frame1, const cv::Mat3b &frame2, const cv::Mat1b *mask1,
    const cv::Mat1b *mask2) const {
  cv::Mat3b frames[2] = {frame1, frame2};
  const cv::Mat1b *masks[2] = {mask1, mask2};
  cv::Mat1b frames_gray[2];
  std::vector<cv::KeyPoint> keypoints[2];
  cv::Mat descriptors[2];
  for (int frame_index : {0, 1}) {
    cv::cvtColor(frames[frame_index], frames_gray[frame_index],
                 cv::COLOR_BGR2GRAY);
    feature_detector_->detect(
        frames_gray[frame_index], keypoints[frame_index],
        masks[frame_index] ? *(masks[frame_index]) : cv::noArray());
    // For ORB: Remove overlapping detections
    // FOR SIFT: Clean an area of dense detections
    if (settings_.use_non_maximal_suppression)
      keypoints[frame_index] = NonMaximumSuppression(keypoints[frame_index],
                                                     settings_.nms_window_size);
    feature_detector_->compute(frames_gray[frame_index], keypoints[frame_index],
                               descriptors[frame_index]);
  }

  std::vector<cv::DMatch> matches = Match(descriptors[0], descriptors[1]);
  std::cout << "Total matches: " << matches.size() << std::endl;
  LOG(INFO) << "Total matches: " << matches.size() << std::endl;
  if (settings_.debug_draw_matches)
    DrawMatches(frame1, keypoints[0], frame2, keypoints[1], matches);

  CentralPoint2dCorrespondences correspondences;
  for (const auto &m : matches) {
    const cv::KeyPoint kp1 = keypoints[0][m.queryIdx];
    const cv::KeyPoint kp2 = keypoints[1][m.trainIdx];
    correspondences.Add(ToVector2(kp1.pt), ToVector2(kp2.pt));
  }

  return correspondences;
}

std::vector<cv::DMatch> FeatureDetectorAndMatcher::RatioTestMatch(
    const cv::Mat &descriptors1, const cv::Mat &descriptors2) const {
  CHECK_GE(settings_.min_second_best_ratio(), 1.0);
  std::vector<std::vector<cv::DMatch>> matches;
  descriptor_matcher_->knnMatch(descriptors1, descriptors2, matches, 2);

  std::vector<cv::DMatch> good_matches;
  for (const auto &match : matches) {
    if (match.size() == 1) {
      good_matches.push_back(match[0]);
    } else if (match.size() == 2) {
      if (match[1].distance / match[0].distance >
          settings_.min_second_best_ratio()) {
        good_matches.push_back(match[0]);
      }
    }
  }
  return good_matches;
}

std::vector<cv::DMatch> FeatureDetectorAndMatcher::CrossCheckMatches(
    const std::vector<cv::DMatch> &matches1,
    const std::vector<cv::DMatch> &matches2) const {
  std::vector<cv::DMatch> symmetric_matches;
  // TODO test this
  std::set<std::pair<int, int>> indices;
  for (const cv::DMatch &m : matches2) indices.emplace(m.trainIdx, m.queryIdx);
  for (const cv::DMatch &m : matches1)
    if (indices.find({m.queryIdx, m.trainIdx}) != indices.end())
      symmetric_matches.push_back(m);
  return symmetric_matches;
}

std::vector<cv::KeyPoint> FeatureDetectorAndMatcher::NonMaximumSuppression(
    const std::vector<cv::KeyPoint> &keypoints, float window_size) const {
  std::vector<cv::KeyPoint> nms_keypoints;

  // naive implementation with two for loops
  // TODO sliding window approach
  for (const auto &kp1 : keypoints) {
    bool is_max = true;
    for (const auto &kp2 : keypoints) {
      float dist = std::max(std::abs(kp1.pt.x - kp2.pt.x),
                            std::abs(kp1.pt.y - kp2.pt.y));  // L_inf norm

      if (dist <= window_size) {
        if (kp1.response < kp2.response) {
          is_max = false;
          break;
        }
      }
    }
    if (is_max) {
      nms_keypoints.push_back(kp1);
    }
  }
  return nms_keypoints;
}

std::vector<cv::DMatch> FeatureDetectorAndMatcher::Match(
    const cv::Mat &descriptors1, const cv::Mat &descriptors2) const {
  switch (settings_.feature_type) {
    case FeatureDetectorAndMatcherSettings::kSift: {
      if (settings_.cross_check_matches)
        return CrossCheckMatches(RatioTestMatch(descriptors1, descriptors2),
                                 RatioTestMatch(descriptors2, descriptors1));
      else
        return RatioTestMatch(descriptors1, descriptors2);
    }
    case FeatureDetectorAndMatcherSettings::kOrb: {
      std::vector<std::vector<cv::DMatch>> matches_long;
      // cross-check is performed internally by OpenCV in this case
      descriptor_matcher_->knnMatch(descriptors1, descriptors2, matches_long,
                                    1);
      std::vector<cv::DMatch> matches;
      matches.reserve(matches_long.size());
      for (const auto &v : matches_long) {
        CHECK_LE(v.size(), 1);
        if (!v.empty()) matches.push_back(v[0]);
      }
      return matches;
    }
    default:
      throw std::domain_error(
          fmt::format("Unknown feature type {}", settings_.feature_type));
  }
}

float FeatureDetectorAndMatcherSettings::min_second_best_ratio() const {
  switch (feature_type) {
    case kSift:
      return sift_settings.min_second_best_ratio;
    case kOrb:
      return orb_settings.min_second_best_ratio;
    default:
      throw std::domain_error(
          fmt::format("Unknown feature type {}", feature_type));
  }
}

}  // namespace grpose