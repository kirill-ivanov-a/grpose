#include <iostream>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "central/approximate_central_solution_sampler.h"
#include "central/central_bundle_adjuster.h"
#include "central/central_refiner.h"
#include "dataset/autovision_reader.h"
#include "features/colmap_database.h"
#include "util/metrics.h"
#include "util/util.h"

DEFINE_bool(draw_inlier_matches, false,
            "If set, inlier matches used for each run are drawn.");

DEFINE_int32(
    frame1_ind, 0,
    "Index of the first matched frame in the dataset, starting from 0");
DEFINE_int32(camera1_ind, 0,
             "Index of the camera for the first matched frame in the dataset, "
             "starting from 0");
DEFINE_int32(
    frame2_ind, 30,
    "Index of the second matched frame in the dataset, starting from 0");
DEFINE_int32(camera2_ind, 0,
             "Index of the camera for the second matched frame in the dataset, "
             "starting from 0");

DEFINE_string(feature_matches_database, "",
              "Path to the database with feature matches in COLMAP format. If "
              "set, the matches from this database will be used instead of "
              "computing matches on the spot. Makes feature_types unused.");
DEFINE_string(
    feature_matches_database_root, "",
    "If this is set, it should hold the path to the directory "
    "with images, relative to which the colmap database was computed. "
    "Otherwise, it is assumed to be the same as the dataset root.");
DEFINE_string(
    error_type, "reproj_2d",
    "Type of the error used to filter the input matches given the ground truth "
    "pose. Avaliable values: reproj_2d,angular,sampson_3d");
DEFINE_string(refinement_type, "sampson_3d",
              "Type of the refinement error. Avaliable values: "
              "reproj_2d,cosine,sampson_3d");
DEFINE_double(
    inlier_threshold, 4.0,
    "A threshold for a correspondence to be considered an inlier. Reasonable "
    "values depend on the error function chosen, see --error_type. If the "
    "error is angular, then this threshold is assumed to be in degrees.");
DEFINE_int32(max_ba_steps, 10000, "Maximal number of bundle adjustment steps");
DEFINE_double(min_ba_tolerance, 1e-12,
              "Bundle adjustment terminates if relative decrease in cost is "
              "less than this.");
DEFINE_bool(ba_fix_points, false,
            "If set, bundle adjustment will not optimize point positions");
DEFINE_bool(ba_fix_pose, false,
            "If set, bundle adjustment will not optimize frame pose");
DEFINE_bool(run_all_pairs, false,
            "If set, runs all pairs of frames in the dataset for this camera, "
            "with difference between indices fixed to frame2_ind-frame1_ind, "
            "as set by corresponding flags. Camera indices are fixed.");

using namespace grpose;

namespace {

void SampleErrors(const std::shared_ptr<DatasetReader> &dataset_reader,
                  const ColmapDatabase &colmap_database,
                  const fs::path &output_directory) {
  std::vector<std::pair<int, int>> frame_pairs;
  if (FLAGS_run_all_pairs) {
    int diff = FLAGS_frame2_ind - FLAGS_frame1_ind;
    for (int fi1 = 0; fi1 + diff < dataset_reader->NumberOfFrames(); ++fi1) {
      int fi2 = fi1 + diff;
      frame_pairs.emplace_back(fi1, fi2);
    }
  } else {
    frame_pairs.emplace_back(FLAGS_frame1_ind, FLAGS_frame2_ind);
  }

  std::ofstream residual_ofs(output_directory / "residuals.csv");
  residual_ofs << "type,v1,v2,v3,v4" << std::endl;
  for (auto [frame1_ind, frame2_ind] : frame_pairs) {
    fmt::print("\n\nRUNNING ({}, {})\n", frame1_ind, frame2_ind);
    std::cout << std::endl;
    CentralPoint2dCorrespondences correspondences =
        colmap_database.GetCentralCorrespondences(
            frame1_ind, FLAGS_camera1_ind, frame2_ind, FLAGS_camera2_ind);

    CameraBundle camera_bundle = dataset_reader->GetCameraBundle();
    Camera camera1 = camera_bundle.camera(FLAGS_camera1_ind);
    Camera camera2 = camera_bundle.camera(FLAGS_camera2_ind);
    correspondences.FilterByMasks(camera1.mask(), camera2.mask());

    const SE3 frame1_from_frame2_true =
        camera_bundle.camera_from_body(FLAGS_camera1_ind) *
        dataset_reader->WorldFromFrame(frame1_ind).inverse() *
        dataset_reader->WorldFromFrame(frame2_ind) *
        camera_bundle.body_from_camera(FLAGS_camera2_ind);

    ApproximateCentralSolutionSamplerSettings settings;
    settings.error_type = ToErrorType(FLAGS_error_type);
    settings.threshold = FLAGS_inlier_threshold;
    ApproximateCentralSolutionSampler solution_sampler(
        camera1, camera2, correspondences, frame1_from_frame2_true, settings);
    CentralPoint2dCorrespondences inlier_correspondences =
        solution_sampler.GetInlierCorrespondences();
    LOG(INFO) << fmt::format("Retained {} / {} matches as inliers",
                             inlier_correspondences.Size(),
                             correspondences.Size());

    if (FLAGS_draw_inlier_matches) {
      cv::Mat3b frame1 =
          dataset_reader->Frame(frame1_ind)[FLAGS_camera1_ind].frame;
      cv::Mat3b frame2 =
          dataset_reader->Frame(frame2_ind)[FLAGS_camera2_ind].frame;
      cv::Mat3b inlier_matches =
          DrawMatches(inlier_correspondences, frame1, frame2);
      cv::Mat3b result;
      cv::resize(inlier_matches, result, cv::Size(), 0.5, 0.5);
      cv::imshow("Extracted matches", result);
      cv::waitKey();
    }

    std::mt19937 mt;
    SE3 frame1_from_frame2_approx = solution_sampler.SampleFrame1FromFrame2(mt);
    LOG(INFO) << fmt::format(
        "Errors of the sampled approximate: rte={} are={}",
        180.0 / M_PI *
            AngularTranslationError(frame1_from_frame2_true,
                                    frame1_from_frame2_approx),
        180.0 / M_PI *
            AbsoluteRotationError(frame1_from_frame2_true,
                                  frame1_from_frame2_approx));

    CentralBundleAdjusterSettings bundle_adjuster_settings;
    bundle_adjuster_settings.solver_options.max_num_iterations =
        FLAGS_max_ba_steps;
    bundle_adjuster_settings.solver_options.function_tolerance =
        FLAGS_min_ba_tolerance;
    bundle_adjuster_settings.fix_points = FLAGS_ba_fix_points;
    bundle_adjuster_settings.fix_pose = FLAGS_ba_fix_pose;
    //  bundle_adjuster_settings.solver_options.minimizer_progress_to_stdout =
    //  true;
    CentralBundleAdjuster bundle_adjuster(bundle_adjuster_settings);
    auto bundle_adjuster_results = bundle_adjuster.Refine(
        frame1_from_frame2_approx, camera1, camera2, inlier_correspondences);

    LOG(INFO) << bundle_adjuster_results.solver_summary.FullReport()
              << std::endl;

    LOG(INFO) << fmt::format(
        "Errors of the bundle adjustment: rte={} are={}",
        180.0 / M_PI *
            AngularTranslationError(frame1_from_frame2_true,
                                    bundle_adjuster_results.frame1_from_frame2),
        180.0 / M_PI *
            AbsoluteRotationError(frame1_from_frame2_true,
                                  bundle_adjuster_results.frame1_from_frame2));

    CentralRefinerSettings refiner_settings;
    refiner_settings.refinement_type = ToRefinementType(FLAGS_refinement_type);
    CentralRefiner refiner(refiner_settings);
    auto refiner_residuals =
        refiner.CalculateResiduals(bundle_adjuster_results.frame1_from_frame2,
                                   camera1, camera2, inlier_correspondences);

    CHECK_EQ(refiner_residuals.rows(),
             bundle_adjuster_results.residuals.rows());

    for (int i = 0; i < bundle_adjuster_results.residuals.rows(); ++i) {
      residual_ofs << "BA";
      for (int j = 0; j < bundle_adjuster_results.residuals.cols(); ++j)
        residual_ofs << "," << bundle_adjuster_results.residuals(i, j);
      residual_ofs << std::endl;
    }
    for (int i = 0; i < bundle_adjuster_results.residuals.rows(); ++i) {
      residual_ofs << "Sampson";
      for (int j = 0; j < refiner_residuals.cols(); ++j)
        residual_ofs << "," << refiner_residuals(i, j);
      residual_ofs << std::endl;
    }

    CentralRefiner::RefinementSummary refinement_summary;
    SE3 frame1_from_frame2_refined =
        refiner.Refine(frame1_from_frame2_approx, camera1, camera2,
                       inlier_correspondences, refinement_summary);

    LOG(INFO) << fmt::format(
        "Errors of the Sampson refinement: rte={} are={}",
        180.0 / M_PI *
            AngularTranslationError(frame1_from_frame2_true,
                                    frame1_from_frame2_refined),
        180.0 / M_PI *
            AbsoluteRotationError(frame1_from_frame2_true,
                                  frame1_from_frame2_refined));

    LOG(INFO) << refinement_summary.solver_summary.FullReport() << std::endl;
  }
}

}  // namespace

int main(int argc, char *argv[]) {
  std::string usage =
      R"abacaba(Usage:   ./sampson_error_comparison autovision_segment_dir autovision_calib_dir autovision_config database
where
  autovision_segment_dir is the path to an AutoVision segment directory
  autovision_calib_dir is the path to the calibration directory
  autovision_config is the path to the configuration JSON file
  database is the path to the COLMAP database containing matches
)abacaba";

  fs::path output_directory =
      fs::path("output") / ("sampson_error_comparison_" + CurrentTimeBrief());
  fs::create_directories(output_directory);
  std::cout << "output dir: " << output_directory.string() << std::endl;
  SaveArgv(output_directory / "argv.txt", argc, argv);

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  gflags::SetUsageMessage(usage);
  google::InitGoogleLogging(argv[0]);

  FLAGS_alsologtostderr = true;

  CHECK_EQ(argc, 5) << usage;
  fs::path autovision_segment_dir = argv[1];
  fs::path autovision_calib_dir = argv[2];
  fs::path autovision_config = argv[3];
  fs::path database_path = argv[4];

  fs::path database_root = FLAGS_feature_matches_database_root.empty()
                               ? autovision_segment_dir.parent_path()
                               : fs::path(FLAGS_feature_matches_database_root);

  std::shared_ptr<DatasetReader> dataset_reader =
      std::make_shared<AutovisionReader>(
          autovision_segment_dir, autovision_calib_dir, autovision_config);
  ColmapDatabase colmap_database(database_path, database_root, dataset_reader);

  SampleErrors(dataset_reader, colmap_database, output_directory);

  return 0;
}
