#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "central/central_opengv_solver_bvc.h"
#include "central/central_solver_using_features.h"
#include "dataset/autovision_reader.h"
#include "features/colmap_database.h"
#include "util/metrics.h"
#include "util/util.h"

DEFINE_int32(ransac_max_iter, 20000, "Max iterations of RANSAC");

// With focal length f=1200pix, one pixel error corresponds to roughly 0.04
// degrees of angular deviation
DEFINE_double(ransac_angle_thres, 0.3,
              "OpenGV's RANSAC threshold. It corresponds roughly to "
              "2*(1-cos(angle)) where angle is the angle of deviation of the "
              "bearing vector from its supposed direction.");

DEFINE_bool(draw_matches, false,
            "If set, matches used for each run are drawn.");

DEFINE_string(feature_types, "sift",
              "Feature types that are used. Possible values: sift,orb . Used "
              "values separated by commas.");
DEFINE_bool(use_nms, true, "Use non-maximal suppression of keypoints?");
DEFINE_bool(use_cross_check, true, "Use cross-check of matches?");
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
                  const fs::path &output_filename) {
  std::ofstream out_stream(output_filename);
  out_stream
      << "feature_name,segment_index,camera_index,first_frame_index,frame_step,"
         "success,gt_qw,gt_qx,gt_qy,"
         "gt_qz,gt_tx,gt_ty,gt_tz,qw,qx,qy,qz,tx,ty,tz,are,rte,num_"
         "ransac_iter,num_inliers,num_corresps,experiment_num"
      << std::endl;

  std::vector<std::string> feature_types = SplitByComma(FLAGS_feature_types);
  std::vector<int> autovision_segments = GetInts(FLAGS_autovision_segments);
  std::vector<int> frame_steps = GetInts(FLAGS_frame_steps);
  std::vector<int> camera_indices = GetInts(FLAGS_cameras);
  bool use_precomputed_features = !FLAGS_feature_matches_database.empty();
  if (use_precomputed_features) {
    CHECK(!FLAGS_feature_matches_database_root.empty())
        << "Expected meaningful --feature_matches_dataset_root !";
    feature_types = {"precomputed"};
  }

  for (const auto &feature_name : feature_types) {
    for (int segment_index : autovision_segments) {
      std::shared_ptr<DatasetReader> dataset_reader =
          std::make_unique<AutovisionReader>(
              autovision_root / std::to_string(segment_index),
              autovision_root / "calib", autovision_config);

      std::optional<FeatureDetectorAndMatcher> matcher;
      std::optional<ColmapDatabase> matches_database;
      if (use_precomputed_features) {
        matches_database.emplace(fs::path(FLAGS_feature_matches_database),
                                 fs::path(FLAGS_feature_matches_database_root),
                                 dataset_reader);
      } else {
        FeatureDetectorAndMatcherSettings matcher_settings;
        matcher_settings.feature_type = FeatureTypeFromName(feature_name);
        matcher_settings.debug_draw_matches = FLAGS_draw_matches;
        matcher_settings.use_non_maximal_suppression = FLAGS_use_nms;
        matcher_settings.cross_check_matches = FLAGS_use_cross_check;
        matcher.emplace(matcher_settings);
      }

      CentralOpengvSolverBvcSettings solver_settings;
      const double kThresInRadians = M_PI / 180.0 * FLAGS_ransac_angle_thres;
      solver_settings.threshold = kThresInRadians * kThresInRadians;
      solver_settings.max_iterations = fLI::FLAGS_ransac_max_iter;
      const std::shared_ptr<CentralSolverBvc> ransac_solver_opengv =
          std::make_shared<CentralOpengvSolverBvc>(solver_settings);

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

            for (int experiment_num = 0; experiment_num < FLAGS_num_attempts;
                 ++experiment_num) {
              SE3 frame1_from_frame2;

              CentralPoint2dCorrespondences point_correspondences;
              if (use_precomputed_features) {
                point_correspondences =
                    matches_database->GetCentralCorrespondences(
                        frame_index1, camera_index, frame_index2, camera_index);
              } else {
                cv::Mat3b frame1 =
                    dataset_reader->Frame(frame_index1)[camera_index].frame;
                cv::Mat3b frame2 =
                    dataset_reader->Frame(frame_index2)[camera_index].frame;
                point_correspondences = matcher->GetCorrespondences(
                    frame1, frame2, &camera_mask, &camera_mask);
              }
              point_correspondences.FilterByMasks(camera.mask(), camera.mask());
              CentralBearingVectorCorrespondences correspondences =
                  point_correspondences.ToCentralBearingVectorCorrespondences(
                      camera, camera);
              CentralSolverBvc::SolveInfo solve_info;
              bool is_ok = ransac_solver_opengv->Solve(
                  correspondences, frame1_from_frame2, &solve_info);
              const double rte = AngularTranslationError(
                  true_frame1_from_frame2, frame1_from_frame2);
              const double are = AbsoluteRotationError(true_frame1_from_frame2,
                                                       frame1_from_frame2);

              // segment_index,camera_index,first_frame_index,frame_step,success,gt_qw,gt_qx,gt_qy,
              // gt_qz,gt_tx,gt_ty,gt_tz,qw,qx,qy,qz,tx,ty,tz,are,rte,
              // num_ransac_iter,num_inliers,num_corresps,experiment_num
              out_stream << fmt::format(
                  "{:s},{:d},{:d},{:d},{:d},{:d},{:s},{:s},{:g},{:g},{:d},{:d},"
                  "{:d},{:d}",
                  feature_name, segment_index, camera_index, frame_index1,
                  frame_step, is_ok, ToCsv(true_frame1_from_frame2),
                  ToCsv(frame1_from_frame2), are, rte,
                  solve_info.number_of_iterations, solve_info.number_of_inliers,
                  solve_info.number_of_correspondences, experiment_num);
              out_stream << std::endl;

              fmt::print(
                  "Ran frame #{}/{} on camera #{}; |t|={}, rte={} deg, "
                  "are={:.3} deg niter={} inl: {}/{}",
                  frame_index1, dataset_reader->NumberOfFrames(), camera_index,
                  true_frame1_from_frame2.translation().norm(),
                  rte * 180.0 / M_PI, are * 180.0 / M_PI,
                  solve_info.number_of_iterations, solve_info.number_of_inliers,
                  correspondences.Size());
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
      R"abacaba(Usage:   ./central_ransac_benchmark autovision_root autovision_config
where autovision_root is the path to SP20 directory and autovision_config is the path to the configuration JSON file.
      )abacaba";

  fs::path output_directory =
      fs::path("output") / ("central_ransac_benchmark_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;
  SaveArgv(output_directory / "argv.txt", argc, argv);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);
  CHECK_EQ(argc, 3) << usage;

  RunBenchmark(argv[1], argv[2], output_directory / "data.csv");

  return 0;
}
