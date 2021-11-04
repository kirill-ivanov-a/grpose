#include "central/central_bundle_adjuster.h"

#include "util/ceres_util.h"
#include "util/geometry.h"

namespace grpose {

namespace {}

CentralBundleAdjuster::CentralBundleAdjuster(
    const CentralBundleAdjusterSettings& settings)
    : settings_(settings) {}

CentralBundleAdjuster::RefinementResults CentralBundleAdjuster::Refine(
    const SE3& frame1_from_frame2_estimate, const Camera& camera1,
    const Camera& camera2,
    const CentralPoint2dCorrespondences& correspondences) const {
  constexpr int kResidualSize = CentralBundleAdjusterSettings::kResidualSize;

  RefinementResults results;
  results.frame1_from_frame2 = frame1_from_frame2_estimate;
  SE3 frame2_from_frame1_estimate = frame1_from_frame2_estimate.inverse();
  results.points.resize(correspondences.Size(), 3);

  ceres::Problem problem;
  problem.AddParameterBlock(results.frame1_from_frame2.so3().data(),
                            SO3::num_parameters,
                            new SO3LocalParameterization());
  problem.AddParameterBlock(results.frame1_from_frame2.translation().data(), 3,
                            new ceres::HomogeneousVectorParameterization(3));
  if (settings_.fix_pose) {
    problem.SetParameterBlockConstant(results.frame1_from_frame2.so3().data());
    problem.SetParameterBlockConstant(results.frame1_from_frame2.translation().data());
  }

  std::vector<ceres::SizedCostFunction<4, 4, 3, 3>*> residual_functions;

  for (int i = 0; i < correspondences.Size(); ++i) {
    const Vector2 point1 = correspondences.point(0, i);
    const Vector2 point2 = correspondences.point(1, i);

    Vector3 point_in_frame1;
    Triangulate(frame2_from_frame1_estimate, camera1.Unmap(point1),
                camera2.Unmap(point2), point_in_frame1);
    results.points.row(i) = point_in_frame1.transpose();
    double* point_ptr = results.points.data() + 3 * i;
    problem.AddParameterBlock(point_ptr, 3);
      if (settings_.fix_points)
      problem.SetParameterBlockConstant(point_ptr);

    ceres::LossFunction* loss_function = nullptr;
    switch (settings_.loss_type) {
      case CentralBundleAdjusterSettings::kSquaredLoss:
        loss_function = nullptr;
        break;
      case CentralBundleAdjusterSettings::kHuberLoss:
        loss_function = new ceres::HuberLoss(settings_.pixel_outlier_threshold);
        break;
      case CentralBundleAdjusterSettings::kCauchyLoss:
        loss_function =
            new ceres::CauchyLoss(settings_.pixel_outlier_threshold);
        break;
      default:
        LOG(WARNING) << "Unknown loss function type " << settings_.loss_type;
        break;
    }

    residual_functions.push_back(
        new ceres::AutoDiffCostFunction<ProjectionResidual, kResidualSize, 4, 3,
                                        3>(
            new ProjectionResidual(&camera1, &camera2, point1, point2)));
    problem.AddResidualBlock(residual_functions.back(), loss_function,
                             results.frame1_from_frame2.so3().data(),
                             results.frame1_from_frame2.translation().data(),
                             point_ptr);
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> points_before =
      results.points;

  ceres::Solve(settings_.solver_options, &problem, &results.solver_summary);

  results.residuals.resize(residual_functions.size(), kResidualSize);
  for (int i = 0; i < residual_functions.size(); ++i) {
    double* parameter_ptrs[3] = {
        results.frame1_from_frame2.so3().data(),
        results.frame1_from_frame2.translation().data(),
        results.points.data() + 3 * i};
    residual_functions[i]->Evaluate(
        parameter_ptrs, results.residuals.data() + i * kResidualSize, nullptr);
  }

//  std::cout << "relative point dist = "
//            << (points_before - results.points)
//                   .rowwise()
//                   .norm()
//                   .cwiseQuotient(points_before.rowwise().norm()).transpose();

  return results;
}

ProjectionResidual::ProjectionResidual(const Camera* camera_1,
                                       const Camera* camera_2,
                                       const Vector2& point_1,
                                       const Vector2& point_2)
    : camera1_(camera_1),
      camera2_(camera_2),
      point1_(point_1),
      point2_(point_2) {}

}  // namespace grpose