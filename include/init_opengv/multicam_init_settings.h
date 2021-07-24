#ifndef GRPOSE_INITOPENGV_MULTICAMINITSETTINGS_
#define GRPOSE_INITOPENGV_MULTICAMINITSETTINGS_

#include <opencv2/opencv.hpp>

namespace grpose {

/***********************************
 Settings for relative pose solvers
***********************************/
struct NonCentralRelativePoseSolverSettings {
  NonCentralRelativePoseSolverSettings(double focal_length,
                                       bool verbose = true);

  void UpdateRansacThreshold(double reproj_threshold, double focal_length);

  enum SolverAlgorithm {
    kSixPoint,                // Stewenius 6pt algorithm
    kGeneralizedEigensolver,  // Kneip Generalized Eigensolver
    kSeventeenPoint           // Li 17pt algorithm
  };
  static constexpr SolverAlgorithm default_algorithm = kSeventeenPoint;
  SolverAlgorithm algorithm = default_algorithm;

  // Used to compute ransac threshold in terms of the reprojection error
  // threshold
  static constexpr double default_ransac_reproj_threshold = 1.0;
  double ransac_reproj_threshold = default_ransac_reproj_threshold;

  double ransac_threshold;

  static constexpr double default_ransac_probability = 0.99;
  double ransac_probability = default_ransac_probability;

  static constexpr int default_ransac_max_iter = 3000;
  int ransac_max_iter = default_ransac_max_iter;

  enum RansacVerbosity { kQuiet = 0, kVerbose = 2 };
  static constexpr RansacVerbosity default_ransac_verbosity_level = kQuiet;
  RansacVerbosity ransac_verbosity_level = default_ransac_verbosity_level;

  static constexpr bool default_solver_verbose = true;
  bool solver_verbose = default_solver_verbose;
};

/*******************************************************
  Parameters for getting correspondences between frames
*******************************************************/

struct SiftSettings {
  int number_of_features = 0;  // 0 means to retain all features
  int number_of_octave_layers = 3;
  double contrast_threshold = 0.04;
  double edge_threshold = 10;
  double sigma = 1.6;
};

struct OrbSettings {
  int number_of_features = 1000;
  float scale_factor = 1.2f;
  int number_of_levels = 8;
  int edge_threshold = 51;
  int first_level = 0;
  int wta_k = 2;
  decltype(cv::ORB::HARRIS_SCORE) score_type = cv::ORB::HARRIS_SCORE;
  int patch_size = 51;
  int fast_threshold = 10;
};

struct FeatureDetectorMatcherSettings {
  // SIFT is slower but should give more stable keypoints and better matches
  // ORB is faster but gives less stable keypoints and worse matches
  enum FeatureType { kSift, kOrb };
  static constexpr FeatureType default_feature_type = kSift;
  FeatureType feature_type = default_feature_type;

  SiftSettings sift_settings;
  OrbSettings orb_settings;

  // Descriptor based tracker parameters
  static constexpr float default_min_ratio =
      1.3;  // May need to be different for SIFT and ORB!!!
            // ORB: 1.0
            // SIFT: 1.3
  float min_ratio = default_min_ratio;
};

}  // namespace grpose

#endif