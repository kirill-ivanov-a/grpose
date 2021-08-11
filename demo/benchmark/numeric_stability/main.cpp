#include <fstream>
#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "grpose/bearing_vector_correspondences.h"
#include "grpose/opengv_adapter.h"
#include "grpose/opengv_solver.h"
#include "metrics.h"
#include "synthetic/car_like_scene.h"
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
DEFINE_double(max_angle, 30, "Maximal turning angle of motion");
DEFINE_int32(num_angles, 10, "Number of turning angles to check");

DEFINE_double(min_width, 20, "Minimal width of the scene");
DEFINE_double(max_width, 40, "Maximal width of the scene");
DEFINE_int32(num_widths, 10, "Number of widths of the scene");

DEFINE_double(min_angle_std, 0,
              "Minimal bearing vector direction standard deviation");
constexpr double kMaxAngleStd = 3.0 * M_PI / 180.0;
DEFINE_double(max_angle_std, kMaxAngleStd,
              "Maximal bearing vector direction standard deviation");
DEFINE_int32(num_angle_std, 10,
             "Number of bearing vector direction standard deviations");

DEFINE_int32(num_attempts, 10, "Number of solver runs per a set of parameters");

using namespace grpose;

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
    // TODO proper assert
    CHECK_LE(current_needed, indices_by_camera[ic].size());
    std::sample(indices_by_camera[ic].begin(), indices_by_camera[ic].end(),
                std::back_inserter(indices), current_needed, mt);
  }

  CHECK_EQ(indices.size(), number_needed);
  std::shuffle(indices.begin(), indices.end(), mt);
  return indices;
}

StdVectorA<SE3> EstimateFrame1FromFrame2(
    std::mt19937 &mt, const synthetic::CarLikeScene &scene,
    const OpengvSolver::Algorithm &algorithm, double angle_std = 0.0) {
  std::shared_ptr correspondences =
      std::make_shared<BearingVectorCorrespondences>(
          scene.GetBearingVectorCorrespondences(FLAGS_num_corresps));
  correspondences->AddGaussianDirectionNoise(mt, angle_std);
  std::shared_ptr opengv_adapter = std::make_shared<OpengvAdapter>(
      correspondences, scene.GetBodyFromCameras());
  std::unique_ptr<NonCentralRelativePoseSolver> solver(
      new OpengvSolver(opengv_adapter, algorithm));
  const int number_needed = solver->MinimalNeededCorrespondences();
  std::vector<int> indices = SampleCorrespondences(
      *correspondences, number_needed, FLAGS_num_cross, FLAGS_frac_first, mt);

  StdVectorA<SE3> frame1_from_frame2;
  solver->Solve(indices, frame1_from_frame2);
  return frame1_from_frame2;
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
  for (const std::string &method_name : {"6pt", "8pt", "17pt"}) {
    const OpengvSolver::Algorithm algorithm = NameToAlgorithm(method_name);
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
          StdVectorA<SE3> frame1_from_frame2 =
              EstimateFrame1FromFrame2(mt, scene, algorithm, angle_std);
          CHECK_EQ(frame1_from_frame2.size(), 1)
              << "Statistics collection does not support many solutions yet";

          const double ate = AbsoluteTranslationError(true_frame1_from_frame2,
                                                      frame1_from_frame2[0]);
          const double rte = AngularTranslationError(true_frame1_from_frame2,
                                                     frame1_from_frame2[0]);
          const double are = AbsoluteRotationError(true_frame1_from_frame2,
                                                   frame1_from_frame2[0]);
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
  for (const std::string &method_name : {"6pt", "8pt", "17pt"}) {
    const OpengvSolver::Algorithm algorithm = NameToAlgorithm(method_name);
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
          StdVectorA<SE3> frame1_from_frame2 =
              EstimateFrame1FromFrame2(mt, scene, algorithm);
          CHECK_EQ(frame1_from_frame2.size(), 1)
              << "Statistics collection does not support many solutions yet";

          const double ate = AbsoluteTranslationError(true_frame1_from_frame2,
                                                      frame1_from_frame2[0]);
          const double rte = AngularTranslationError(true_frame1_from_frame2,
                                                     frame1_from_frame2[0]);
          const double are = AbsoluteRotationError(true_frame1_from_frame2,
                                                   frame1_from_frame2[0]);
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
  CalculateErrorTables(car_like_scene, FLAGS_min_motion_length,
                       FLAGS_max_motion_length, FLAGS_num_motion_lengths,
                       FLAGS_min_angle, FLAGS_max_angle, FLAGS_num_angles,
                       FLAGS_num_attempts,
                       output_directory / "error_motion.csv");
  CalculateStabilityTables(
      car_like_scene, FLAGS_min_width, FLAGS_max_width, FLAGS_num_widths,
      FLAGS_min_angle_std, FLAGS_max_angle_std, FLAGS_num_angle_std,
      FLAGS_num_attempts, output_directory / "error_stability.csv");

  return 0;
}
