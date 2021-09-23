#include <fstream>
#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/algorithm/string.hpp>

#include "grpose/bearing_vector_correspondences.h"
#include "grpose/opengv_adapter.h"
#include "grpose/opengv_solver.h"
#include "grpose/solver_6pt_poselib.h"
#include "grpose/solver_central_plus_scale.h"
#include "synthetic/car_like_scene.h"
#include "util/metrics.h"
#include "util/util.h"

DEFINE_int32(num_corresps, 200, "Number of correspondences to generate");
DEFINE_double(frac_cross, 0.1,
              "Fraction of cross-camera correspondences to generate");
DEFINE_int32(num_cross, 0,
             "Number of cross-camera correspondences for each solver run");
DEFINE_double(frac_first, 0.25,
              "Fraction of correspondences that the first camera gets at each "
              "solver run, [0, 1)");

DEFINE_double(min_motion_length, 3, "Minimal length of motion");
DEFINE_double(max_motion_length, 20, "Maximal length of motion");
DEFINE_int32(num_motion_lengths, 10, "Number of lengths of motion");

DEFINE_double(min_angle, 0, "Minimal turning angle of motion");
constexpr double kMaxAngle = 30.0 * M_PI / 180.0;
DEFINE_double(max_angle, kMaxAngle, "Maximal turning angle of motion");
DEFINE_int32(num_angles, 10, "Number of turning angles to check");

DEFINE_double(min_width, 15, "Minimal width of the scene");
DEFINE_double(max_width, 80, "Maximal width of the scene");
DEFINE_int32(num_widths, 10, "Number of widths of the scene");

DEFINE_double(min_angle_std, 0,
              "Minimal bearing vector direction standard deviation");
// With focal length f=1200pix, one pixel error corresponds to roughly 0.04
// degrees of angular deviation
constexpr double kMaxAngleStd = 0.12 * M_PI / 180.0;
DEFINE_double(max_angle_std, kMaxAngleStd,
              "Maximal bearing vector direction standard deviation");
DEFINE_int32(num_angle_std, 10,
             "Number of bearing vector direction standard deviations");

DEFINE_double(fix_motion_length, 10,
              "Fixed motion length for stability experiments");
DEFINE_double(fix_angle, 20, "Fixed turning angle for stability experiments");

DEFINE_int32(num_attempts, 10, "Number of solver runs per a set of parameters");

DEFINE_string(method_names, "6pt,8pt,17pt,c+s,6pt_poselib",
              "Names of the methods to be tested, separated by a comma. By "
              "default, all available methods are tested");

DEFINE_bool(run_motion_error, true,
            "Run the test w.r.t. motion length & angle");
DEFINE_bool(run_stability, true,
            "Run the test w.r.t. scene scale and direction errors");

using namespace grpose;

std::vector<std::string> GetMethodNames(const std::string &names) {
  std::vector<std::string> names_split;
  boost::split(names_split, names, boost::is_any_of(","));
  return names_split;
}

OpengvSolver::Algorithm NameToAlgorithm(const std::string &method_name) {
  if (method_name == "6pt")
    return OpengvSolver::Algorithm::kSixPoint;
  else if (method_name == "8pt")
    return OpengvSolver::Algorithm::kGeneralizedEigensolver;
  else if (method_name == "17pt")
    return OpengvSolver::Algorithm::kSeventeenPoint;
  else
    throw std::domain_error(fmt::format("Unknown method \"{}\"", method_name));
}

void RestructureIndices(const BearingVectorCorrespondences &correspondences,
                        std::vector<std::vector<int>> &indices_by_camera,
                        std::vector<int> &cross_camera_indices) {
  const int n = correspondences.NumberOfCorrespondences();
  const int c = correspondences.NumberOfCameras();
  indices_by_camera.resize(c);
  cross_camera_indices.clear();
  for (int i = 0; i < n; ++i) {
    const int ci1 = correspondences.camera_index(0, i);
    const int ci2 = correspondences.camera_index(1, i);
    if (ci1 == ci2)
      indices_by_camera[ci1].push_back(i);
    else
      cross_camera_indices.push_back(i);
  }
}

std::vector<int> SampleCorrespondences(
    const BearingVectorCorrespondences &correspondences, int number_needed,
    int number_cross_camera, double fraction_first_camera, std::mt19937 &mt) {
  const int n = correspondences.NumberOfCorrespondences();
  const int c = correspondences.NumberOfCameras();
  std::vector<std::vector<int>> indices_by_camera(c);
  std::vector<int> cross_camera_indices;
  RestructureIndices(correspondences, indices_by_camera, cross_camera_indices);
  const int number_camera0 =
      static_cast<int>(std::ceil(number_needed * fraction_first_camera));
  const int number_other_total =
      number_needed - number_camera0 - number_cross_camera;
  const int number_other = number_other_total / (c - 1);
  const int remainder_other = number_other_total % (c - 1);
  // TODO proper assert
  if (number_cross_camera > cross_camera_indices.size())
    throw std::runtime_error(fmt::format("SampleCorrespondences: {} > {}",
                                         number_cross_camera,
                                         cross_camera_indices.size()));

  std::vector<int> indices;
  std::sample(indices_by_camera[0].begin(), indices_by_camera[0].end(),
              std::back_inserter(indices), number_camera0, mt);
  std::sample(cross_camera_indices.begin(), cross_camera_indices.end(),
              std::back_inserter(indices), number_cross_camera, mt);
  for (int ic = 1; ic < c; ++ic) {
    const int current_needed =
        ic <= remainder_other ? number_other + 1 : number_other;
    CHECK_LE(current_needed, indices_by_camera[ic].size());
    std::sample(indices_by_camera[ic].begin(), indices_by_camera[ic].end(),
                std::back_inserter(indices), current_needed, mt);
  }

  CHECK_EQ(indices.size(), number_needed);
  std::shuffle(indices.begin(), indices.end(), mt);
  return indices;
}

void SampleNFromCamera(const std::vector<std::vector<int>> &indices_by_camera,
                       int n, std::mt19937 &mt, std::vector<int> &indices,
                       int &camera, int excluded_camera = -1) {
  const int c = indices_by_camera.size();

  std::vector<int> good_camera_indices;
  std::vector<int> good_camera_sizes;
  good_camera_indices.reserve(c);
  good_camera_sizes.reserve(c);
  for (int i = 0; i < indices_by_camera.size(); ++i)
    if (indices_by_camera[i].size() >= n && i != excluded_camera) {
      good_camera_indices.push_back(i);
      good_camera_sizes.push_back(indices_by_camera[i].size());
    }
  CHECK(!good_camera_indices.empty());
  std::discrete_distribution camera_d(good_camera_sizes.begin(),
                                      good_camera_sizes.end());
  camera = good_camera_indices[camera_d(mt)];

  std::sample(indices_by_camera[camera].begin(),
              indices_by_camera[camera].end(), std::back_inserter(indices), n,
              mt);
}

std::vector<int> SampleNPlus1Correspondence(
    const BearingVectorCorrespondences &correspondences, int n,
    bool use_cross_camera, std::mt19937 &mt) {
  const int total = correspondences.NumberOfCorrespondences();
  const int c = correspondences.NumberOfCameras();
  CHECK_GE(c, 2);
  std::vector<std::vector<int>> indices_by_camera(c);
  std::vector<int> cross_camera_indices;
  RestructureIndices(correspondences, indices_by_camera, cross_camera_indices);

  std::vector<int> indices;
  int camera_n = 0;
  SampleNFromCamera(indices_by_camera, n, mt, indices, camera_n);

  int index_1;
  if (use_cross_camera) {
    std::uniform_int_distribution<int> index_1_d(
        0, static_cast<int>(cross_camera_indices.size()) - 1);
    indices.push_back(cross_camera_indices[index_1_d(mt)]);
  } else {
    int camera_1 = 0;
    SampleNFromCamera(indices_by_camera, 1, mt, indices, camera_1, camera_n);
  }

  return indices;
}

bool EstimateFrame1FromFrame2(std::mt19937 &mt,
                              const synthetic::CarLikeScene &scene,
                              const std::string &method_name,
                              StdVectorA<SE3> &frame1_from_frame2,
                              double angle_std = 0.0) {
  std::shared_ptr correspondences =
      std::make_shared<BearingVectorCorrespondences>(
          scene.GetBearingVectorCorrespondences(FLAGS_num_corresps,
                                                FLAGS_frac_cross, mt()));
  correspondences->AddGaussianDirectionNoise(mt, angle_std);
  std::shared_ptr opengv_adapter = std::make_shared<OpengvAdapter>(
      correspondences, scene.GetBodyFromCameras());
  std::unique_ptr<NonCentralRelativePoseMinimalSolver> solver;
  std::vector<int> indices;
  if (method_name == "c+s") {
    solver.reset(new SolverCentralPlusScale(opengv_adapter));
    indices = SampleNPlus1Correspondence(
        *correspondences, solver->MinSampleSize() - 1, FLAGS_num_cross > 0, mt);
  } else if (method_name == "6pt_poselib") {
    solver.reset(
        new Solver6ptPoselib(correspondences, scene.GetBodyFromCameras()));
    indices = SampleCorrespondences(*correspondences, solver->MinSampleSize(),
                                    FLAGS_num_cross, FLAGS_frac_first, mt);
  } else {
    solver.reset(
        new OpengvSolver(opengv_adapter, NameToAlgorithm(method_name)));
    indices = SampleCorrespondences(*correspondences, solver->MinSampleSize(),
                                    FLAGS_num_cross, FLAGS_frac_first, mt);
  }
  return solver->Solve(indices, frame1_from_frame2);
}

SE3 BestRteMotion(const StdVectorA<SE3> &motions, const SE3 &true_motion) {
  CHECK(!motions.empty());
  SE3 best_motion;
  double min_rte = std::numeric_limits<double>::infinity();
  for (const SE3 &motion : motions) {
    const double rte = AngularTranslationError(true_motion, motion);
    if (rte < min_rte) {
      min_rte = rte;
      best_motion = motion;
    }
  }
  return best_motion;
}

void CalculateStabilityTables(synthetic::CarLikeScene scene, double min_width,
                              double max_width, int number_widths,
                              double min_angle_std, double max_angle_std,
                              int number_angle_std, int number_attempts,
                              const fs::path &output_filename) {
  const double width_step = (max_width - min_width) / (number_widths - 1);
  const double angle_std_step =
      (max_angle_std - min_angle_std) / (number_angle_std - 1);

  std::mt19937 mt;

  std::ofstream out_stream(output_filename);
  out_stream << "method_name,scene_width,scene_length,angle_std,"
                "experiment_num,ATE,RTE,ARE"
             << std::endl;

  scene.SetLength(FLAGS_fix_motion_length);
  scene.SetTurnAngle(FLAGS_fix_angle);

  for (const std::string &method_name : GetMethodNames(FLAGS_method_names)) {
    for (int il = 0; il < number_widths; ++il) {
      const double scene_width = min_width + il * width_step;
      const double scene_scale_change = scene_width / scene.width();
      const double scene_length = scene_scale_change * scene.length();
      scene.SetLength(scene_length);
      scene.SetWidth(scene_width);
      for (int ia = 0; ia < number_angle_std; ++ia) {
        const double angle_std = min_angle_std + ia * angle_std_step;

        fmt::print("w={:.2f}, h={:.2f} std={:.4f}\n", scene_width, scene_length,
                   angle_std);
        std::cout.flush();

        const SE3 true_frame1_from_frame2 =
            scene.GetWorldFromBody(0).inverse() * scene.GetWorldFromBody(1);
        for (int ie = 0; ie < number_attempts; ++ie) {
          StdVectorA<SE3> frame1_from_frame2;
          bool is_ok = EstimateFrame1FromFrame2(mt, scene, method_name,
                                                frame1_from_frame2, angle_std);
          double ate, rte, are;
          if (is_ok) {
            const SE3 best_frame1_from_frame2 =
                BestRteMotion(frame1_from_frame2, true_frame1_from_frame2);
            ate = AbsoluteTranslationError(true_frame1_from_frame2,
                                           best_frame1_from_frame2);
            rte = AngularTranslationError(true_frame1_from_frame2,
                                          best_frame1_from_frame2);
            are = AbsoluteRotationError(true_frame1_from_frame2,
                                        best_frame1_from_frame2);
          } else {
            ate = rte = are = std::numeric_limits<double>::infinity();
          }
          fmt::print(out_stream, "{:s},{:g},{:g},{:g},{:d},{:g},{:g},{:g}\n",
                     method_name, scene_width, scene_length, angle_std, ie, ate,
                     rte, are);
          out_stream.flush();
        }
      }
    }
  }
}

void CalculateErrorTables(synthetic::CarLikeScene scene,
                          double min_motion_length, double max_motion_length,
                          int number_motion_lengths, double min_angle,
                          double max_angle, int number_angles,
                          int number_attempts,
                          const fs::path &output_filename) {
  const double motion_length_step =
      (max_motion_length - min_motion_length) / (number_motion_lengths - 1);
  const double angle_step = (max_angle - min_angle) / (number_angles - 1);

  std::mt19937 mt;

  std::ofstream out_stream(output_filename);
  out_stream
      << "method_name,motion_length,turning_angle,experiment_num,ATE,RTE,ARE"
      << std::endl;
  for (const std::string &method_name : GetMethodNames(FLAGS_method_names)) {
    for (int il = 0; il < number_motion_lengths; ++il) {
      const double motion_length = min_motion_length + il * motion_length_step;
      scene.SetMotionLength(motion_length);
      for (int ia = 0; ia < number_angles; ++ia) {
        const double angle = min_angle + ia * angle_step;
        scene.SetTurnAngle(angle);

        fmt::print("len={:.2f}, ang={:.2f}\n", motion_length, angle);
        std::cout.flush();

        const SE3 true_frame1_from_frame2 =
            scene.GetWorldFromBody(0).inverse() * scene.GetWorldFromBody(1);
        for (int ie = 0; ie < number_attempts; ++ie) {
          StdVectorA<SE3> frame1_from_frame2;
          bool is_ok = EstimateFrame1FromFrame2(mt, scene, method_name,
                                                frame1_from_frame2);
          double ate, rte, are;
          if (is_ok) {
            const SE3 best_frame1_from_frame2 =
                BestRteMotion(frame1_from_frame2, true_frame1_from_frame2);

            ate = AbsoluteTranslationError(true_frame1_from_frame2,
                                           best_frame1_from_frame2);
            rte = AngularTranslationError(true_frame1_from_frame2,
                                          best_frame1_from_frame2);
            are = AbsoluteRotationError(true_frame1_from_frame2,
                                        best_frame1_from_frame2);
          } else {
            ate = rte = are = std::numeric_limits<double>::infinity();
          }
          fmt::print(out_stream, "{:s},{:g},{:g},{:d},{:g},{:g},{:g}\n",
                     method_name, motion_length, angle, ie, ate, rte, are);
        }
      }
    }
  }
}

int main(int argc, char *argv[]) {
  std::string usage = R"abacaba(Usage:   ./numeric_stability)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  fs::path output_directory =
      fs::path("output") / ("numeric_stability_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;

  synthetic::CarLikeScene car_like_scene;
  if (FLAGS_run_motion_error)
    CalculateErrorTables(car_like_scene, FLAGS_min_motion_length,
                         FLAGS_max_motion_length, FLAGS_num_motion_lengths,
                         FLAGS_min_angle, FLAGS_max_angle, FLAGS_num_angles,
                         FLAGS_num_attempts,
                         output_directory / "error_motion.csv");
  if (FLAGS_run_stability)
    CalculateStabilityTables(
        car_like_scene, FLAGS_min_width, FLAGS_max_width, FLAGS_num_widths,
        FLAGS_min_angle_std, FLAGS_max_angle_std, FLAGS_num_angle_std,
        FLAGS_num_attempts, output_directory / "error_stability.csv");

  return 0;
}
