#include <iostream>
#include <random>

#include <central/central_bundle_adjuster.h>
#include <central/central_refiner.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "central/approximate_central_solution_sampler.h"
#include "central/central_opengv_solver_bvc.h"
#include "central/central_solver_using_features.h"
#include "dataset/autovision_reader.h"
#include "features/colmap_database.h"
#include "util/metrics.h"
#include "util/util.h"

DEFINE_string(
    frame_steps, "30,60",
    "Relative pose is run between frames #f and #(f + s) where values for "
    "s are taken from this flag. Integers are separated by commas.");
DEFINE_string(
    cameras, "0,1,2,3",
    "Indices of cameras tested, separated by comma. Indexing starts from 0.");
DEFINE_string(autovision_segments, "2",
              "Indices of segments of the AutoVision on which the experiment "
              "will be run, in [1,5]. Integers are separated by commas.");
DEFINE_int32(next_run_step, 1, "Step of the experiment (see code).");
DEFINE_int32(num_attempts, 10,
             "Number of experiment runs per a set of parameters.");

DEFINE_string(feature_matches_database, "",
              "Path to the database with feature matches in COLMAP format. If "
              "set, the matches from this database will be used instead of "
              "computing matches on the spot. Makes feature_types unused.");
DEFINE_string(feature_matches_database_root, "",
              "If feature_matcher_database is provided, this needs to be "
              "provided as well. It should hold the path to the directory with "
              "images, relative to which the colmap database was computed.");

DEFINE_string(optimization_types, "ba,sampson_3d,reproj_2d,dot_prod",
              "Types of refinement used, separated by commas.");
DEFINE_int32(max_ba_steps, 500, "Maximal number of bundle adjustment steps");
DEFINE_double(min_ba_tolerance, 1e-10,
              "Bundle adjustment terminates if relative decrease in cost is "
              "less than this.");
DEFINE_string(
    error_type, "reproj_2d",
    "Type of the error used to filter the input matches given the ground truth "
    "pose. Avaliable values: reproj_2d,angular,sampson_3d");
DEFINE_double(
    inlier_threshold, 4.0,
    "A threshold for a correspondence to be considered an inlier. Reasonable "
    "values depend on the error function chosen, see --error_type. If the "
    "error is angular, then this threshold is assumed to be in degrees.");

using namespace grpose;

namespace {

FeatureDetectorAndMatcherSettings::FeatureType FeatureTypeFromName(
    const std::string &name) {
  if (name == "sift")
    return FeatureDetectorAndMatcherSettings::kSift;
  else if (name == "orb")
    return FeatureDetectorAndMatcherSettings::kOrb;
  else
    throw std::domain_error("Unknown feature type " + name);
}

std::vector<int> GetInts(const std::string &names) {
  std::vector<std::string> split = SplitByComma(names);
  std::vector<int> result;
  for (const auto &s : split) result.push_back(std::stoi(s));
  return result;
}

std::string ToCsv(const SE3 &motion) {
  const Quaternion &q = motion.unit_quaternion();
  const Vector3 &t = motion.translation();
  return fmt::format("{:g},{:g},{:g},{:g},{:g},{:g},{:g}", q.w(), q.x(), q.y(),
                     q.z(), t.x(), t.y(), t.z());
}

void RunBenchmark(const fs::path &autovision_root,
                  const fs::path &autovision_config,
                  const fs::path &database_path, const fs::path &database_root,
                  const fs::path &output_filename) {
  std::ofstream out_stream(output_filename);
  out_stream << "type,segment_index,camera_index,first_frame_"
                "index,frame_step,gt_qw,gt_qx,gt_qy,"
                "gt_qz,gt_tx,gt_ty,gt_tz,qw,qx,qy,qz,tx,ty,tz,are,rte,"
                "num_inliers,num_iter,experiment_num"
             << std::endl;

  std::vector<std::string> optimization_types =
      SplitByComma(FLAGS_optimization_types);
  std::vector<int> autovision_segments = GetInts(FLAGS_autovision_segments);
  std::vector<int> frame_steps = GetInts(FLAGS_frame_steps);
  std::vector<int> camera_indices = GetInts(FLAGS_cameras);
  std::optional<ColmapDatabase> matches_database;

  std::mt19937 mt;

  for (int segment_index : autovision_segments) {
    std::shared_ptr<DatasetReader> dataset_reader =
        std::make_unique<AutovisionReader>(
            autovision_root / std::to_string(segment_index),
            autovision_root / "calib", autovision_config);
    if (matches_database)
      matches_database->SetDatasetReader(dataset_reader);
    else
      matches_database.emplace(database_path, database_root, dataset_reader);
    const CameraBundle camera_bundle = dataset_reader->GetCameraBundle();

    for (int frame_step : frame_steps) {
      for (int frame_index1 = 0;
           frame_index1 + frame_step < dataset_reader->NumberOfFrames();
           frame_index1 += FLAGS_next_run_step) {
        const int frame_index2 = frame_index1 + frame_step;
        const std::vector<DatasetReader::FrameEntry> frames[2] = {
            dataset_reader->Frame(frame_index1),
            dataset_reader->Frame(frame_index2)};
        for (int camera_index : camera_indices) {
          const Camera &camera = camera_bundle.camera(camera_index);
          cv::Mat1b camera_mask = camera.mask();
          const SE3 true_frame1_from_frame2 =
              camera_bundle.camera_from_body(camera_index) *
              dataset_reader->WorldFromFrame(frame_index1).inverse() *
              dataset_reader->WorldFromFrame(frame_index2) *
              camera_bundle.body_from_camera(camera_index);

          CentralPoint2dCorrespondences point_correspondences =
              matches_database->GetCentralCorrespondences(
                  frame_index1, camera_index, frame_index2, camera_index);
          point_correspondences.FilterByMasks(camera.mask(), camera.mask());
          CentralBearingVectorCorrespondences correspondences =
              point_correspondences.ToCentralBearingVectorCorrespondences(
                  camera, camera);
          ApproximateCentralSolutionSamplerSettings solution_sampler_settings;
          solution_sampler_settings.error_type = ToErrorType(FLAGS_error_type);
          solution_sampler_settings.threshold = FLAGS_inlier_threshold;
          ApproximateCentralSolutionSampler solution_sampler(
              camera, camera, point_correspondences, true_frame1_from_frame2,
              solution_sampler_settings);
          CentralPoint2dCorrespondences inlier_correspondences =
              solution_sampler.GetInlierCorrespondences();
          int num_inliers = inlier_correspondences.Size();

          for (int experiment_num = 0; experiment_num < FLAGS_num_attempts;
               ++experiment_num) {
            SE3 frame1_from_frame2_approx =
                solution_sampler.SampleFrame1FromFrame2(mt);

            const double rte_approx = AngularTranslationError(
                true_frame1_from_frame2, frame1_from_frame2_approx, true);
            const double are_approx = AbsoluteRotationError(
                true_frame1_from_frame2, frame1_from_frame2_approx, true);
            //"type,segment_index,camera_index,first_frame_"
            //"index,frame_step,gt_qw,gt_qx,gt_qy,"
            //"gt_qz,gt_tx,gt_ty,gt_tz,qw,qx,qy,qz,tx,ty,tz,are,rte,"
            //"num_inliers,num_iter,experiment_num"

            out_stream << fmt::format(
                "{:s},{:d},{:d},{:d},{:d},{:s},{:s},{:g},{:g},{:d},{:d},"
                "{:d}",
                "min_solver", segment_index, camera_index, frame_index1,
                frame_step, ToCsv(true_frame1_from_frame2),
                ToCsv(frame1_from_frame2_approx), are_approx, rte_approx,
                num_inliers, 0, experiment_num);
            out_stream << std::endl;

            for (const std::string &optimization_type : optimization_types) {
              SE3 frame1_from_frame2_refined;
              int num_iterations = 0;
              if (optimization_type == "ba") {
                CentralBundleAdjusterSettings settings;
                settings.solver_options.max_num_iterations = FLAGS_max_ba_steps;
                settings.solver_options.function_tolerance =
                    FLAGS_min_ba_tolerance;
                CentralBundleAdjuster bundle_adjuster(settings);
                auto adjustment_results =
                    bundle_adjuster.Refine(frame1_from_frame2_approx, camera,
                                           camera, inlier_correspondences);
                frame1_from_frame2_refined =
                    adjustment_results.frame1_from_frame2;
                num_iterations =
                    adjustment_results.solver_summary.iterations.size();
              } else {
                CentralRefinerSettings settings;
                settings.refinement_type = ToRefinementType(optimization_type);
                CentralRefiner refiner(settings);
                CentralRefiner::RefinementSummary summary;
                frame1_from_frame2_refined =
                    refiner.Refine(frame1_from_frame2_approx, camera, camera,
                                   inlier_correspondences, summary);
                num_iterations = summary.solver_summary.iterations.size();
              }

              const double rte_refined = AngularTranslationError(
                  true_frame1_from_frame2, frame1_from_frame2_refined, true);
              const double are_refined = AbsoluteRotationError(
                  true_frame1_from_frame2, frame1_from_frame2_refined, true);

              //"type,segment_index,camera_index,first_frame_"
              //"index,frame_step,gt_qw,gt_qx,gt_qy,"
              //"gt_qz,gt_tx,gt_ty,gt_tz,qw,qx,qy,qz,tx,ty,tz,are,rte,"
              //"num_inliers,num_iter,experiment_num"
              out_stream << fmt::format(
                  "{:s},{:d},{:d},{:d},{:d},{:s},{:s},{:g},{:g},{:d},{:d},"
                  "{:d}",
                  optimization_type, segment_index, camera_index, frame_index1,
                  frame_step, ToCsv(true_frame1_from_frame2),
                  ToCsv(frame1_from_frame2_refined), are_refined, rte_refined,
                  num_inliers, num_iterations, experiment_num);
              out_stream << std::endl;

              fmt::print(
                  "Ran {} on frames #{}&{}/{} on camera #{}, segment #{};\n"
                  " |t|={:.2} (estimated {:.2}), \n"
                  "rte={:.3} vs {:.3} deg, are={:.3} vs {:.3} deg\n"
                  "niter={} ninl={}/{}\n",
                  optimization_type, frame_index1, frame_index2,
                  dataset_reader->NumberOfFrames(), camera_index, segment_index,
                  true_frame1_from_frame2.translation().norm(),
                  frame1_from_frame2_refined.translation().norm(), rte_refined,
                  rte_approx, are_refined, are_approx, num_iterations,
                  num_inliers, correspondences.Size());
              std::cout << std::endl;
            }
          }
        }
      }
    }
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./central_refinement_benchmark autovision_root autovision_config database
      )abacaba";

  fs::path output_directory =
      fs::path("output") /
      ("central_refinement_benchmark_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;
  SaveArgv(output_directory / "argv.txt", argc, argv);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);
  CHECK_EQ(argc, 4) << usage;
  fs::path autovision_root = argv[1];
  fs::path autovision_config = argv[2];
  fs::path database_path = argv[3];

  fs::path database_root = !FLAGS_feature_matches_database_root.empty()
                               ? fs::path(FLAGS_feature_matches_database_root)
                               : autovision_root;

  RunBenchmark(autovision_root, autovision_config, database_path, database_root,
               output_directory / "data.csv");

  return 0;
}
