#include "openGVInitialization/FeatureDetectorMatcher.h"

namespace mcam {

BearingVectorCorrespondences::BearingVectorCorrespondences(int numCameras)
    : numCameras(numCameras),
      numCorrespondences(0),
      currentTimestamps(numCameras),
      nextTimestamps(numCameras),
      bearingVectorsCurrent(),
      bearingVectorsNext(),
      correspondencesCurrent(),
      correspondencesNext() {}

void BearingVectorCorrespondences::setCamTimestamp(int cameraInd,
                                                   Timestamp currentTimestamp,
                                                   Timestamp nextTimestamp) {
  currentTimestamps[cameraInd] = currentTimestamp;
  nextTimestamps[cameraInd] = nextTimestamp;
}

void BearingVectorCorrespondences::pushBVC(
    opengv::bearingVector_t bearingVectorCurrent,
    opengv::bearingVector_t bearingVectorNext, int cameraCorrespondence) {
  // Make sure that all bearing vectors pushed are normalized !!!
  bearingVectorsCurrent.push_back(bearingVectorCurrent.normalized());

  bearingVectorsNext.push_back(bearingVectorNext.normalized());

  correspondencesCurrent.push_back(cameraCorrespondence);
  correspondencesNext.push_back(cameraCorrespondence);
  numCorrespondences++;
}

FeatureDetectorMatcher::FeatureDetectorMatcher(
    FeatureDetectorMatcherSettings settings, CameraBundle cameraBundle)
    : settings(settings), cameraBundle(cameraBundle) {
  decltype(cv::DescriptorMatcher::BRUTEFORCE_HAMMING) matcher_type;
  switch (settings.feature_type) {
    case FeatureType::SIFT:
      feat_detector =
          cv::SIFT::create(settings.sift.nfeatures, settings.sift.nOctaveLayers,
                           settings.sift.contrastThreshold,
                           settings.sift.edgeThreshold, settings.sift.sigma);
      matcher_type = cv::DescriptorMatcher::BRUTEFORCE;
      LOG(INFO) << "Feature detection and matching using SIFT";
      break;

    case FeatureType::ORB:
      feat_detector = cv::ORB::create(
          settings.orb.nfeatures, settings.orb.scaleFactor,
          settings.orb.nlevels, settings.orb.edgeThreshold,
          settings.orb.firstLevel, settings.orb.WTA_K, settings.orb.scoreType,
          settings.orb.patchSize, settings.orb.fastThreshold);
      matcher_type =
          cv::DescriptorMatcher::BRUTEFORCE_HAMMING;  // for binary descriptors
      LOG(INFO) << "Feature detection and matching using ORB";
      break;

    default:
      LOG(FATAL) << "Unknown feature type";
      break;
  }

  desc_matcher = cv::DescriptorMatcher::create(matcher_type);
}

BearingVectorCorrespondences FeatureDetectorMatcher::getBearingVectors(
    const std::vector<DatasetReader::FrameEntry> &frame_bundle,
    const std::vector<DatasetReader::FrameEntry> &next_frame_bundle) {
  // Initializing bearing vectors to be returned
  BearingVectorCorrespondences bvcs(frame_bundle.size());

  for (int i = 0; i < frame_bundle.size(); i++) {
    // Push timestamps for this camera
    bvcs.setCamTimestamp(i, frame_bundle[i].timestamp,
                         next_frame_bundle[i].timestamp);

    // Get camera model for the current camera
    CameraModel camera_model = cameraBundle.cam(i);

    // IMPORTANT use the mask for detection!!!
    auto mask = camera_model.mask();

    // Get picture in grayscale
    cv::Mat1b frame_gray, next_frame_gray;
    cv::cvtColor(frame_bundle[i].frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(next_frame_bundle[i].frame, next_frame_gray,
                 cv::COLOR_BGR2GRAY);

    // Keypoints and descriptors
    std::vector<cv::KeyPoint> keypoints, keypoints_next;
    cv::Mat desc, desc_next;  // d x N

    feat_detector->detect(frame_gray, keypoints, mask);
    feat_detector->detect(next_frame_gray, keypoints_next, mask);

    // Apply non-maximal suppression
    // For ORB: Remove overlapping detections
    // FOR SIFT: Clean an area of dense detections
    keypoints = nms(keypoints, 5);
    keypoints_next = nms(keypoints_next, 5);

    feat_detector->compute(frame_gray, keypoints, desc);
    feat_detector->compute(next_frame_gray, keypoints_next, desc_next);

    // Get matches between frames using the selected method
    std::vector<cv::DMatch> matches = crossCheckMatches(
        ratioTestMatch(desc, desc_next), ratioTestMatch(desc_next, desc));

    std::cout << "Total matches: " << matches.size() << std::endl;

    // std::vector<cv::DMatch> pass_matches;

    // Get bearing vectors and correspondences from matches
    for (const auto &m : matches) {
      cv::KeyPoint current_kp =
          keypoints[m.queryIdx];  // keypoint from ith camera in frame_bundle
      cv::KeyPoint next_kp =
          keypoints_next[m.trainIdx];  // keypoint from ith camera in
                                       // next_frame_bundle

      Eigen::Vector2d current_pt, next_pt;
      current_pt[0] = current_kp.pt.x;
      current_pt[1] = current_kp.pt.y;

      next_pt[0] = next_kp.pt.x;
      next_pt[1] = next_kp.pt.y;

      // Only add a correspondence between frames if the matched keypoint in
      // both frames passes the map(unmap()) check
      // => backproject-reproject error less than some number of px
      const double backproject_error = 1.0;
      if (mapUnmap(current_pt, camera_model) < backproject_error &&
          mapUnmap(next_pt, camera_model) < backproject_error) {
        Eigen::Vector3d current_bearing =
            camera_model.unmap(current_pt).normalized();
        Eigen::Vector3d next_bearing = camera_model.unmap(next_pt).normalized();

        // pass_matches.push_back(m);
        bvcs.pushBVC(current_bearing, next_bearing, i);
      }
    }
    // std::cout << "length of bvcs: " << bvcs.bearingVectorsCurrent.size()
    //           << std::endl;

    // /* FOR DEBUGGING */
    // drawKeypoints(frame_gray, keypoints);
    // drawKeypoints(next_frame_gray, keypoints_next);

    /* Draw the matched feature correspondences */
    // drawMatches(frame_gray, keypoints, next_frame_gray, keypoints_next,
    //             matches);

    // std::cout << "pass_matches size: " << pass_matches.size() << std::endl;
    // drawMatches(frame_gray, keypoints, next_frame_gray, keypoints_next,
    //             pass_matches);

    // checkMapUnmap(frame_gray, keypoints, camera_model);
  }

  return bvcs;
}

std::vector<cv::DMatch> FeatureDetectorMatcher::ratioTestMatch(
    const cv::Mat &desc1, const cv::Mat &desc2) {
  CHECK_GE(settings.min_ratio, 1.0);
  std::vector<std::vector<cv::DMatch>> matches;
  desc_matcher->knnMatch(desc1, desc2, matches, 2);

  std::vector<cv::DMatch> good_matches;
  for (const auto &match : matches) {
    if (match.size() == 1) {
      good_matches.push_back(match[0]);
    } else if (match.size() == 2) {
      if (match[1].distance / match[0].distance > settings.min_ratio) {
        good_matches.push_back(match[0]);
      }
    }
  }
  return good_matches;
}

std::vector<cv::DMatch> FeatureDetectorMatcher::crossCheckMatches(
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

std::vector<cv::KeyPoint> FeatureDetectorMatcher::nms(
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

/* DEBUG functions */

void drawKeypoints(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints) {
  cv::Mat debug_image;
  cv::drawKeypoints(image, keypoints, debug_image);
  cv::imshow("Detected keypoints", debug_image);
  cv::waitKey(0);
}

void drawMatches(const cv::Mat &image1,
                 const std::vector<cv::KeyPoint> &keypoints1,
                 const cv::Mat &image2,
                 const std::vector<cv::KeyPoint> &keypoints2,
                 const std::vector<cv::DMatch> &matches) {
  cv::Mat debug_image;
  cv::drawMatches(image1, keypoints1, image2, keypoints2, matches, debug_image);
  cv::imshow("Matched keypoints", debug_image);
  cv::waitKey(0);
}

void checkMapUnmap(const cv::Mat &image,
                   const std::vector<cv::KeyPoint> &keypoints,
                   const CameraModel &camera_model) {
  std::vector<cv::KeyPoint> bad_keypoints;
  for (const auto &kp : keypoints) {
    double error = FeatureDetectorMatcher::mapUnmap(
        Eigen::Vector2d(kp.pt.x, kp.pt.y), camera_model);

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

void checkCorrespondences(
    const BearingVectorCorrespondences &correspondences,
    const CameraBundle &camera_bundle,
    const std::vector<DatasetReader::FrameEntry> &frame_bundle,
    const std::vector<DatasetReader::FrameEntry> &next_frame_bundle) {
  CHECK_EQ(correspondences.numCameras, frame_bundle.size());
  CHECK_EQ(correspondences.numCameras, next_frame_bundle.size());

  std::vector<cv::Mat> debug_images_frame(correspondences.numCameras);
  std::vector<cv::Mat> debug_images_frame_next(correspondences.numCameras);
  for (int c = 0; c < correspondences.numCameras; c++) {
    debug_images_frame[c] = frame_bundle[c].frame.clone();
    debug_images_frame_next[c] = next_frame_bundle[c].frame.clone();
  }

  for (int i = 0; i < correspondences.numCorrespondences; i++) {
    int cam = correspondences.correspondencesCurrent[i];

    Eigen::Vector2d point_frame =
        camera_bundle.cam(cam).map(correspondences.bearingVectorsCurrent[i]);
    cv::Point cv_point_frame(point_frame[0], point_frame[1]);

    Eigen::Vector2d point_next_frame =
        camera_bundle.cam(cam).map(correspondences.bearingVectorsNext[i]);
    cv::Point cv_point_next_frame(point_next_frame[0], point_next_frame[1]);

    cv::circle(debug_images_frame[cam], cv_point_frame, 5,
               cv::Scalar(0, 0, 255));
    cv::circle(debug_images_frame_next[cam], cv_point_next_frame, 5,
               cv::Scalar(0, 0, 255));
  }

  for (int c = 0; c < correspondences.numCameras; c++) {
    cv::Mat debug_image;
    cv::hconcat(debug_images_frame[c], debug_images_frame_next[c], debug_image);
    cv::imshow("Check bearing correspondences", debug_image);
    cv::waitKey(0);
  }
}

/* Deprecated functions */

void FeatureDetectorMatcher::descriptorMatching(
    const cv::Mat &current_frame, const cv::Mat &next_frame,
    const cv::Mat &mask, cv::Ptr<cv::Feature2D> detector,
    cv::Ptr<cv::Feature2D> desc_extractor,
    std::vector<cv::KeyPoint> keypoints_current,
    std::vector<cv::KeyPoint> &keypoints_next,
    std::vector<cv::DMatch> &matches) {
  const double nms_region_size = 7.0;

  // Detect keypoints of the next frame
  detector->detect(next_frame, keypoints_next, mask);
  keypoints_next =
      nms(keypoints_next,
          nms_region_size);  // non-maximal suppression in neighbourhood

  // Compute descriptors for both frames and use to find matches between frames
  cv::Mat desc, desc_next;  // d x N
  desc_extractor->compute(current_frame, keypoints_current, desc);
  desc_extractor->compute(next_frame, keypoints_next, desc_next);

  matches = crossCheckMatches(ratioTestMatch(desc, desc_next),
                              ratioTestMatch(desc_next, desc));
}

void FeatureDetectorMatcher::kltMatching(
    const cv::Mat &current_frame, const cv::Mat &next_frame,
    const std::vector<cv::KeyPoint> &keypoints_current,
    std::vector<cv::KeyPoint> &keypoints_next,
    std::vector<cv::DMatch> &matches) {
  std::vector<cv::Point2f> prev_pts, next_pts;
  for (const auto &kp : keypoints_current) {
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
      current_frame, next_frame, prev_pts, next_pts, status, error,
      cv::Size2i(klt_win_size, klt_win_size), klt_max_level, termcrit);

  for (int i = 0; i < prev_pts.size(); i++) {
    if (status[i]) {
      cv::KeyPoint kp = keypoints_current[i];
      kp.pt = next_pts[i];
      keypoints_next.push_back(kp);

      cv::DMatch m;
      m.queryIdx = i;
      m.trainIdx = keypoints_next.size() - 1;
      matches.push_back(m);

    } else {
      std::cout << "Optical Flow status was not good" << std::endl;
    }
  }
}

}  // namespace mcam
