#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "grpose/opengv_solver_bvc.h"
#include "synthetic/car_like_scene.h"
#include "util/metrics.h"
#include "util/util.h"

DEFINE_int32(num_corresps, 1000, "Number of correspondences to generate");
DEFINE_double(frac_cross, 0.1,
              "Fraction of cross-camera correspondences to generate among "
              "outlier correspondences");
DEFINE_double(outlier_frac_cross, 0.5,
              "Fraction of cross-camera correspondences to generate");

DEFINE_double(min_outlier_frac, 0.0, "Minimal fraction of outliers");
DEFINE_double(max_outlier_frac, 0.6, "Maximal fraction of outliers");
DEFINE_int32(num_outlier_frac, 7, "Number of fractions of outliers");

DEFINE_int32(ransac_max_iter, 20000, "Max iterations of RANSAC");
DEFINE_double(ransac_thres_factor, 3.0,
              "Error threshold for RANSAC will be calculated as f * s^2 where "
              "s is the angle standard deviation of the generated bearing "
              "vectors and f is the value of this flag");

DEFINE_int32(num_attempts, 10,
             "Number of experiment runs per a set of parameters");

// With focal length f=1200pix, one pixel error corresponds to roughly 0.04
// degrees of angular deviation
constexpr double kDefaultAngleStd = 0.06 * M_PI / 180.0;
DEFINE_double(angle_std, kDefaultAngleStd,
              "Bearing vector direction standard deviation");

DEFINE_double(fix_motion_length, 10,
              "Fixed motion length for stability experiments");
DEFINE_string(fix_angles, "20",
              "A set of turning angles for stability experiments, separated by "
              "commas. Angles are in degrees.");

DEFINE_string(
    opengv_method_names, "6pt,8pt,17pt",
    "Names of the minimal solvers to be used with OpenGV's RANSAC, "
    "separated by a comma. By default, all available methods are tested");

using namespace grpose;

namespace {

std::vector<double> GetDoubles(const std::string &names) {
  std::vector<std::string> split = SplitByComma(names);
  std::vector<double> result;
  for (const auto &s : split) result.push_back(std::stod(s));
  return result;
}

OpengvSolverBvcSettings::Algorithm NameToOpengvAlgorithm(
    const std::string &method_name) {
  if (method_name == "6pt")
    return OpengvSolverBvcSettings::Algorithm::SIXPT;
  else if (method_name == "8pt")
    return OpengvSolverBvcSettings::Algorithm::GE;
  else if (method_name == "17pt")
    return OpengvSolverBvcSettings::Algorithm::SEVENTEENPT;
  else
    throw std::domain_error(fmt::format("Unknown method \"{}\"", method_name));
}

void RunBenchmark(synthetic::CarLikeScene scene,
                  const fs::path &output_filename) {
  std::ofstream out_stream(output_filename);
  out_stream << "ransac_impl,min_solver,sample_size,outlier_frac,motion_length,"
                "turning_angle,success,qw,qx,qy,qz,tx,ty,tz,ate,are,rte,num_"
                "iter,experiment_num"
             << std::endl;

  scene.SetMotionLength(FLAGS_fix_motion_length);

  const double outlier_frac_step =
      (FLAGS_max_outlier_frac - FLAGS_min_outlier_frac) /
      (FLAGS_num_outlier_frac - 1);
  std::vector<double> angles = GetDoubles(FLAGS_fix_angles);
  std::mt19937 mt;
  for (double angle : angles) {
    std::cout << angle << std::endl;
    scene.SetTurnAngle(angle);
    for (const std::string &method_name :
         SplitByComma(FLAGS_opengv_method_names)) {
      for (int i_frac = 0; i_frac < FLAGS_num_outlier_frac; ++i_frac) {
        const double outlier_frac =
            FLAGS_min_outlier_frac + i_frac * outlier_frac_step;
        for (int it = 0; it < FLAGS_num_attempts; ++it) {
          const int number_outliers = FLAGS_num_corresps * outlier_frac;
          const int number_inliers = FLAGS_num_corresps - number_outliers;
          const auto algorithm = NameToOpengvAlgorithm(method_name);
          BearingVectorCorrespondences correspondences =
              scene.GetBearingVectorCorrespondences(number_inliers,
                                                    FLAGS_frac_cross, mt());
          correspondences.AddGaussianDirectionNoise(mt, FLAGS_angle_std);
          std::vector<bool> is_inlier = correspondences.MixIn(
              mt, scene.GetOutlierCorrespondences(
                      number_outliers, FLAGS_outlier_frac_cross, mt()));

          OpengvSolverBvcSettings settings;
          settings.algorithm = algorithm;
          settings.max_iterations = FLAGS_ransac_max_iter;
          // RANSAC in opengv uses cosine distance to check if the
          // correspondence is an inlier. This formula is based on approximation
          // cos x \approx 1 - x^2/2
          settings.threshold =
              FLAGS_ransac_thres_factor * FLAGS_angle_std * FLAGS_angle_std;
          OpengvSolverBvc solver(scene.GetBodyFromCameras(), settings);
          SE3 estimate_frame1_from_frame2;
          bool is_ok =
              solver.Solve(correspondences, estimate_frame1_from_frame2);
          const int number_iterations = solver.LastNumberOfIterations();
          const Quaternion q = estimate_frame1_from_frame2.unit_quaternion();
          const Vector3 t = estimate_frame1_from_frame2.translation();
          const SE3 true_frame1_from_frame2 =
              scene.GetWorldFromBody(0).inverse() * scene.GetWorldFromBody(1);
          const double ate = AbsoluteTranslationError(
              true_frame1_from_frame2, estimate_frame1_from_frame2);
          const double rte = AngularTranslationError(
              true_frame1_from_frame2, estimate_frame1_from_frame2);
          const double are = AbsoluteRotationError(true_frame1_from_frame2,
                                                   estimate_frame1_from_frame2);
          //        "ransac_impl,min_solver,sample_size,outlier_frac,motion_length,"
          //        "turning_angle,success,qw,qx,qy,qz,tx,ty,tz,ate,are,rte,num_iter"
          out_stream << fmt::format(
              "ransac_opengv,{:s},{:d},{:g},{:g},{:g},{:d},"
              "{:g},{:g},{:g},{:g},{:g},{:g},{:g},{:g},{:g},{:g},{:d},{:d}",
              method_name, solver.MinSampleSize(), outlier_frac,
              scene.motion_length(), scene.turn_angle(),
              static_cast<int>(is_ok), q.w(), q.x(), q.y(), q.z(), t[0], t[1],
              t[2], ate, are, rte, number_iterations, it);
          out_stream << std::endl;

          std::cout << fmt::format(
                           "Ran {} with {:.0}% outliers, angle={}. It took {} "
                           "iterations, metrics: {:.2}, {:.2}, {:.2}",
                           method_name, 100.0 * outlier_frac, angle,
                           number_iterations, ate, 180.0 / M_PI * rte,
                           180.0 / M_PI * are)
                    << std::endl;
        }
      }
    }
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  std::string usage = R"abacaba(Usage:   ./synthetic_ransac_benchmark)abacaba";
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  fs::path output_directory =
      fs::path("output") / ("synthetic_ransac_benchmark_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;

  synthetic::CarLikeScene car_like_scene;
  RunBenchmark(car_like_scene, output_directory / "data.csv");
  return 0;
}
