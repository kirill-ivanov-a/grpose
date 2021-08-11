#include "init_opengv/feature_detector_matcher.h"

namespace grpose {

BearingVectorCorrespondencesOld::BearingVectorCorrespondencesOld(
    int number_of_cameras)
    : number_of_cameras(number_of_cameras),
      number_of_correspondences(0),
      first_timestamps(number_of_cameras),
      second_timestamps(number_of_cameras),
      first_bearing_vectors(),
      second_bearing_vectors(),
      first_correspondences(),
      second_correspondences() {}

void BearingVectorCorrespondencesOld::SetCamTimestamp(
    int camera_index, Timestamp first_timestamp, Timestamp second_timestamp) {
  first_timestamps[camera_index] = first_timestamp;
  second_timestamps[camera_index] = second_timestamp;
}

void BearingVectorCorrespondencesOld::PushBvc(
    const opengv::bearingVector_t &first_bearing_vector,
    const opengv::bearingVector_t &second_bearing_vector,
    int camera_correspondence) {
  // Make sure that all bearing vectors pushed are normalized !!!
  first_bearing_vectors.push_back(first_bearing_vector.normalized());

  second_bearing_vectors.push_back(second_bearing_vector.normalized());

  first_correspondences.push_back(camera_correspondence);
  second_correspondences.push_back(camera_correspondence);
  number_of_correspondences++;
}

FeatureDetectorMatcher::FeatureDetectorMatcher(
    const CameraBundle &camera_bundle,
    const FeatureDetectorMatcherSettings &settings)
    : camera_bundle_(camera_bundle), settings_(settings) {
  decltype(cv::DescriptorMatcher::BRUTEFORCE_HAMMING) matcher_type;
  switch (settings.feature_type) {
    case FeatureType::kSift:
      feature_detector_ = cv::SIFT::create(
          settings.sift_settings.number_of_features,
          settings.sift_settings.number_of_octave_layers,
          settings.sift_settings.contrast_threshold,
          settings.sift_settings.edge_threshold, settings.sift_settings.sigma);
      matcher_type = cv::DescriptorMatcher::BRUTEFORCE;
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
      matcher_type =
          cv::DescriptorMatcher::BRUTEFORCE_HAMMING;  // for binary descriptors
      LOG(INFO) << "Feature detection and matching using ORB";
      break;

    default:
      LOG(FATAL) << "Unknown feature type";
      break;
  }

  descriptor_matcher_ = cv::DescriptorMatcher::create(matcher_type);
}

BearingVectorCorrespondencesOld FeatureDetectorMatcher::getBearingVectors(
    const std::vector<DatasetReader::FrameEntry> &first_frame_bundle,
    const std::vector<DatasetReader::FrameEntry> &second_frame_bundle) {
  // Initializing bearing vectors to be returned
  BearingVectorCorrespondencesOld bvcs(first_frame_bundle.size());

  for (int i = 0; i < first_frame_bundle.size(); i++) {
    // Push timestamps for this camera
    bvcs.SetCamTimestamp(i, first_frame_bundle[i].timestamp,
                         second_frame_bundle[i].timestamp);

    // Get camera model for the current camera
    Camera camera = camera_bundle_.camera(i);

    // IMPORTANT use the mask for detection!!!
    auto mask = camera.mask();

    // Get picture in grayscale
    cv::Mat1b frame_gray, second_frame_gray;
    cv::cvtColor(first_frame_bundle[i].frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(second_frame_bundle[i].frame, second_frame_gray,
                 cv::COLOR_BGR2GRAY);

    // Keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints, keypoints_next;
    cv::Mat desc, desc_next;  // d x N

    feature_detector_->detect(frame_gray, keypoints, mask);
    feature_detector_->detect(second_frame_gray, keypoints_next, mask);

    // For ORB: Remove overlapping detections
    // FOR SIFT: Clean an area of dense detections
    keypoints = NonMaximumSuppression(keypoints, 5);
    keypoints_next = NonMaximumSuppression(keypoints_next, 5);

    feature_detector_->compute(frame_gray, keypoints, desc);
    feature_detector_->compute(second_frame_gray, keypoints_next, desc_next);

    // Get matches between frames using the selected method
    std::vector<cv::DMatch> matches = CrossCheckMatches(
        RatioTestMatch(desc, desc_next), RatioTestMatch(desc_next, desc));

    std::cout << "Total matches: " << matches.size() << std::endl;

    // std::vector<cv::DMatch> pass_matches;

    // Get bearing vectors and correspondences from matches
    for (const auto &m : matches) {
      // keypoint from ith camera in frame_bundle
      cv::KeyPoint first_kp = keypoints[m.queryIdx];
      // keypoint from ith camera in second_frame_bundle
      cv::KeyPoint second_kp = keypoints_next[m.trainIdx];

      Eigen::Vector2d first_pt, second_pt;
      first_pt[0] = first_kp.pt.x;
      first_pt[1] = first_kp.pt.y;

      second_pt[0] = second_kp.pt.x;
      second_pt[1] = second_kp.pt.y;

      // Only add a correspondence between frames if the matched keypoint in
      // both frames passes the map(unmap()) check
      // => backproject-reproject error less than some number of px
      const double backproject_error = 1.0;
      if (MapUnmapDistance(first_pt, camera) < backproject_error &&
          MapUnmapDistance(second_pt, camera) < backproject_error) {
        Eigen::Vector3d first_bearing = camera.Unmap(first_pt).normalized();
        Eigen::Vector3d second_bearing = camera.Unmap(second_pt).normalized();

        // pass_matches.push_back(m);
        bvcs.PushBvc(first_bearing, second_bearing, i);
      }
    }
  }

  return bvcs;
}

std::vector<cv::DMatch> FeatureDetectorMatcher::RatioTestMatch(
    const cv::Mat &descriptors1, const cv::Mat &descriptors2) {
  CHECK_GE(settings_.min_ratio, 1.0);
  std::vector<std::vector<cv::DMatch>> matches;
  descriptor_matcher_->knnMatch(descriptors1, descriptors2, matches, 2);

  std::vector<cv::DMatch> good_matches;
  for (const auto &match : matches) {
    if (match.size() == 1) {
      good_matches.push_back(match[0]);
    } else if (match.size() == 2) {
      if (match[1].distance / match[0].distance > settings_.min_ratio) {
        good_matches.push_back(match[0]);
      }
    }
  }
  return good_matches;
}

std::vector<cv::DMatch> FeatureDetectorMatcher::CrossCheckMatches(
    const std::vector<cv::DMatch> &matches1,
    const std::vector<cv::DMatch> &matches2) {
  std::vector<cv::DMatch> symmetric_matches;
  for (const auto &m1 : matches1) {
    for (const auto &m2 : matches2) {
      if (m1.queryIdx == m2.trainIdx && m1.trainIdx == m2.queryIdx) {
        symmetric_matches.push_back(m1);
      }
    }
  }
  return symmetric_matches;
}

std::vector<cv::KeyPoint> FeatureDetectorMatcher::NonMaximumSuppression(
    const std::vector<cv::KeyPoint> &keypoints, float n_size) {
  std::vector<cv::KeyPoint> nms_keypoints;

  // naive implementation with two for loops
  for (auto kp : keypoints) {
    bool is_max = true;
    for (const auto &n_kp : keypoints) {
      float dist = std::max(std::abs(kp.pt.x - n_kp.pt.x),
                            std::abs(kp.pt.y - n_kp.pt.y));  // L_inf norm

      if (dist <= n_size) {
        if (kp.response < n_kp.response) {
          is_max = false;
          break;
        }
      }
    }
    if (is_max) {
      nms_keypoints.push_back(kp);
    }
  }
  return nms_keypoints;
}

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
  cv::imshow("Matched keypoints", debug_image);
  cv::waitKey(0);
}

void CheckMapUnmap(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints,
                   const Camera &camera) {
  std::vector<cv::KeyPoint> bad_keypoints;
  for (const auto &kp : keypoints) {
    double error = FeatureDetectorMatcher::MapUnmapDistance(
        Eigen::Vector2d(kp.pt.x, kp.pt.y), camera);

    if (error > 1.0) {
      bad_keypoints.push_back(kp);
    }
  }
  std::cout << "Total keypoints: " << keypoints.size() << std::endl;
  std::cout << "Bad keypoints: " << bad_keypoints.size() << std::endl;

  cv::Mat debug_image;
  cv::drawKeypoints(image, bad_keypoints, debug_image);
  cv::imshow("Test map(unmap())", debug_image);
  cv::waitKey(0);
}

void CheckCorrespondences(
    const BearingVectorCorrespondencesOld &correspondences,
    const CameraBundle &camera_bundle,
    const std::vector<DatasetReader::FrameEntry> &first_frame_bundle,
    const std::vector<DatasetReader::FrameEntry> &second_frame_bundle) {
  CHECK_EQ(correspondences.number_of_cameras, first_frame_bundle.size());
  CHECK_EQ(correspondences.number_of_cameras, second_frame_bundle.size());

  std::vector<cv::Mat> debug_images_frame(correspondences.number_of_cameras);
  std::vector<cv::Mat> debug_images_frame_next(
      correspondences.number_of_cameras);
  for (int c = 0; c < correspondences.number_of_cameras; c++) {
    debug_images_frame[c] = first_frame_bundle[c].frame.clone();
    debug_images_frame_next[c] = second_frame_bundle[c].frame.clone();
  }

  for (int i = 0; i < correspondences.number_of_correspondences; i++) {
    int cam = correspondences.first_correspondences[i];

    Eigen::Vector2d point_frame =
        camera_bundle.camera(cam).Map(correspondences.first_bearing_vectors[i]);
    cv::Point cv_point_frame(point_frame[0], point_frame[1]);

    Eigen::Vector2d point_second_frame = camera_bundle.camera(cam).Map(
        correspondences.second_bearing_vectors[i]);
    cv::Point cv_point_second_frame(point_second_frame[0],
                                    point_second_frame[1]);

    cv::circle(debug_images_frame[cam], cv_point_frame, 5,
               cv::Scalar(0, 0, 255));
    cv::circle(debug_images_frame_next[cam], cv_point_second_frame, 5,
               cv::Scalar(0, 0, 255));
  }

  for (int c = 0; c < correspondences.number_of_cameras; c++) {
    cv::Mat debug_image;
    cv::hconcat(debug_images_frame[c], debug_images_frame_next[c], debug_image);
    cv::imshow("Check bearing correspondences", debug_image);
    cv::waitKey(0);
  }
}

/* Deprecated functions */

void FeatureDetectorMatcher::DescriptorMatching(
    const cv::Mat &first_frame, const cv::Mat &second_frame,
    const cv::Mat &mask, cv::Ptr<cv::Feature2D> detector,
    cv::Ptr<cv::Feature2D> descriptor_extractor,
    std::vector<cv::KeyPoint> first_keypoints,
    std::vector<cv::KeyPoint> &second_keypoints,
    std::vector<cv::DMatch> &matches) {
  const double nms_region_size = 7.0;

  // Detect keypoints of the next frame
  detector->detect(second_frame, second_keypoints, mask);
  second_keypoints = NonMaximumSuppression(second_keypoints, nms_region_size);

  // Compute descriptors for both frames and use to find matches between frames
  cv::Mat desc, desc_next;  // d x N
  descriptor_extractor->compute(first_frame, first_keypoints, desc);
  descriptor_extractor->compute(second_frame, second_keypoints, desc_next);

  matches = CrossCheckMatches(RatioTestMatch(desc, desc_next),
                              RatioTestMatch(desc_next, desc));
}

void FeatureDetectorMatcher::KltMatching(
    const cv::Mat &first_frame, const cv::Mat &second_frame,
    const std::vector<cv::KeyPoint> &first_keypoints,
    std::vector<cv::KeyPoint> &second_keypoints,
    std::vector<cv::DMatch> &matches) {
  std::vector<cv::Point2f> prev_pts, second_pts;
  for (const auto &kp : first_keypoints) {
    prev_pts.push_back(kp.pt);
  }

  const double klt_win_size = 30.0;
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  const int klt_max_level = 4;

  cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                            klt_max_iter, klt_eps);
  std::vector<u_char> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(
      first_frame, second_frame, prev_pts, second_pts, status, error,
      cv::Size2i(klt_win_size, klt_win_size), klt_max_level, termcrit);

  for (int i = 0; i < prev_pts.size(); i++) {
    if (status[i]) {
      cv::KeyPoint kp = first_keypoints[i];
      kp.pt = second_pts[i];
      second_keypoints.push_back(kp);

      cv::DMatch m;
      m.queryIdx = i;
      m.trainIdx = second_keypoints.size() - 1;
      matches.push_back(m);

    } else {
      std::cout << "Optical Flow status was not good" << std::endl;
    }
  }
}

}  // namespace grpose
