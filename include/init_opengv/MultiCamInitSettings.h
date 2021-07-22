#ifndef GRPOSE_INITOPENGV_MULTICAMINITSETTINGS_
#define GRPOSE_INITOPENGV_MULTICAMINITSETTINGS_

#include <opencv2/opencv.hpp>

namespace grpose {

/***********************************
  Settings for relative pose solver
***********************************/
struct NonCentralRelativePoseSolverSettings {
  NonCentralRelativePoseSolverSettings(double focal_length,
                                       bool verbose = true);
  void updateRansacThreshold(double reproj_threshold, double focal_length);

  // Stewenius 6pt algorithm
  // Generalized Eigensolver
  // 17pt algorithm
  enum SolverAlgorithm { SIXPT, GE, SEVENTEENPT };
  static constexpr SolverAlgorithm default_algorithm = SEVENTEENPT;
  SolverAlgorithm algorithm = default_algorithm;

  // Computes the ransac threshold in terms of the reprojection error threshold
  static constexpr double default_ransac_reproj_threshold = 1.0;
  double ransac_reproj_threshold = default_ransac_reproj_threshold;
  double ransac_threshold;

  static constexpr double default_ransac_probability = 0.99;
  double ransac_probability = default_ransac_probability;

  static constexpr int default_ransac_max_iter = 3000;
  int ransac_max_iter = default_ransac_max_iter;

  enum RansacVerbosity { QUIET = 0, VERBOSE = 2 };
  static constexpr RansacVerbosity default_ransac_verbosity_level = QUIET;
  RansacVerbosity ransac_verbosity_level = default_ransac_verbosity_level;

  static constexpr bool default_solver_verbose = true;
  bool solver_verbose = default_solver_verbose;
};

/*******************************************************
  Parameters for getting correspondences between frames
*******************************************************/

struct SIFTSettings {
  const int nfeatures = 0;  // 0 means to retain all features
  const int nOctaveLayers = 3;
  const double contrastThreshold = 0.04;
  const double edgeThreshold = 10;
  const double sigma = 1.6;
};

struct ORBSettings {
  const int nfeatures = 1000;
  const float scaleFactor = 1.2f;
  const int nlevels = 8;
  const int edgeThreshold = 51;
  const int firstLevel = 0;
  const int WTA_K = 2;
  const decltype(cv::ORB::HARRIS_SCORE) scoreType = cv::ORB::HARRIS_SCORE;
  const int patchSize = 51;
  const int fastThreshold = 10;
};

struct FeatureDetectorMatcherSettings {
  // SIFT is slower but should give more stable keypoints and better matches
  // ORB is faster but gives less stable keypoints and worst matches since
  // binary descriptor
  enum FeatureType { SIFT, ORB };
  static constexpr FeatureType default_feature_type = SIFT;
  FeatureType feature_type = default_feature_type;

  SIFTSettings sift;
  ORBSettings orb;

  // Descriptor based tracker parameters
  static constexpr float default_min_ratio =
      1.3;  // May need to be different for SIFT and ORB!!!
            // ORB: 1.0
            // SIFT: 1.3
  float min_ratio = default_min_ratio;
};

}  // namespace grpose

#endif