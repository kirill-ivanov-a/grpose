#include "central/central_refiner.h"

#include "util/ceres_util.h"
#include "util/geometry.h"

namespace grpose {

CentralRefinerSettings::RefinementType ToRefinementType(
    const std::string& name) {
  using RefinementType = CentralRefinerSettings::RefinementType;
  if (name == "reproj_2d")
    return RefinementType::kSymmetricReprojection;
  else if (name == "dot_prod")
    return RefinementType::kDotProduct;
  else if (name == "sampson_3d")
    return RefinementType::kSampson3d;
  else if (name == "sampson_2d")
    return RefinementType::kSampsonOnPlane;
  else if (name == "algebraic")
    return RefinementType::kAlgebraic;
  else if (name == "sampson_pinhole")
    return RefinementType::kSampsonPinhole;
  else if (name == "symmetric_epipolar_pinhole")
    return RefinementType::kSymmetricEpipolarPinhole;
  else if (name == "symmetric_epipolar_cosine")
    return RefinementType::kSymmetricEpipolarCosine;
  else
    LOG(ERROR) << "Unknown error type " << name << std::endl;

  return RefinementType::kSampson3d;
}

namespace {

Matrix32 LocalSphereBasis(const Vector3& direction) {
  const Vector3 v1 = AnyOrthogonalUnitVector(direction);
  Matrix32 result;
  result << v1, direction.cross(v1);
  return result;
}

Matrix32 JLeft(const Camera* camera, const Vector3& direction) {
  Matrix32 local_basis = LocalSphereBasis(direction);
  DifferentiatedMapResult diff = camera->DifferentiateMap(direction);
  return local_basis * (diff.jacobian * local_basis).inverse();
}

}  // namespace

// NOTE: the user is responsible for the lifetime of these!!
std::vector<ceres::CostFunction*> CentralRefiner::GetCostFunctions(
    const SE3& frame1_from_frame2_estimate, const Camera& camera1,
    const Camera& camera2, const CentralPoint2dCorrespondences& correspondences,
    const CentralRefinerSettings& settings,
    ReLinearizationAngleMatrix& re_linearization_angles) {
  SE3 frame2_from_frame1_estimate = frame1_from_frame2_estimate.inverse();
  re_linearization_angles.resize(correspondences.Size(), 2);
  std::vector<ceres::CostFunction*> residual_functions;

  for (int i = 0; i < correspondences.Size(); ++i) {
    const Vector2 point1 = correspondences.point(0, i);
    const Vector2 point2 = correspondences.point(1, i);

    Vector3 linearization_direction1 = camera1.Unmap(point1);
    Vector3 linearization_direction2 = camera2.Unmap(point2);
    switch (settings.pre_triangulation_type) {
      case CentralRefinerSettings::kNoTriangulation:
        re_linearization_angles(i, 0) = 0.0;
        re_linearization_angles(i, 0) = 0.0;
        break;
      case CentralRefinerSettings::kTriangulation3d:
        Vector3 point_in_frame1;
        if (settings.refinement_type != CentralRefinerSettings::kSampson3d)
          LOG(WARNING) << "Pre-triangulation method is set, but it won't be "
                          "used (refinement_type="
                       << settings.refinement_type << ")";

        Triangulate(frame2_from_frame1_estimate, linearization_direction1,
                    linearization_direction2, point_in_frame1);
        linearization_direction1 = point_in_frame1.normalized();
        linearization_direction2 =
            (frame2_from_frame1_estimate * point_in_frame1).normalized();
        re_linearization_angles(i, 0) =
            Angle(camera1.Unmap(point1), linearization_direction1);
        re_linearization_angles(i, 1) =
            Angle(camera2.Unmap(point2), linearization_direction2);
    }

    const Vector3 direction1 = camera1.Unmap(point1);
    const Vector3 direction2 = camera2.Unmap(point2);

    switch (settings.refinement_type) {
      case CentralRefinerSettings::kSymmetricReprojection:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                SymmetricReprojectionResidual,
                SymmetricReprojectionResidual::kResidualSize, 4, 3>(
                new SymmetricReprojectionResidual(&camera1, &camera2, point1,
                                                  point2)));
        break;
      case CentralRefinerSettings::kDotProduct: {
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                DotProductResidual, DotProductResidual::kResidualSize, 4, 3>(
                new DotProductResidual(direction1, direction2)));
        break;
      }
      case CentralRefinerSettings::kSampson3d:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                Sampson3dResidual, Sampson3dResidual::kResidualSize, 4, 3>(
                new Sampson3dResidual(&camera1, &camera2, point1, point2,
                                      &linearization_direction1,
                                      &linearization_direction2)));
        break;
      case CentralRefinerSettings::kSampsonOnPlane:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                SampsonOnPlaneResidual, SampsonOnPlaneResidual::kResidualSize,
                4, 3>(new SampsonOnPlaneResidual(&camera1, &camera2, point1,
                                                 point2)));
        break;
      case CentralRefinerSettings::kAlgebraic:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                AlgebraicResidual, AlgebraicResidual::kResidualSize, 4, 3>(
                new AlgebraicResidual(direction1, direction2)));
        break;
      case CentralRefinerSettings::kSampsonPinhole:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                SampsonPinholeResidual, SampsonPinholeResidual::kResidualSize,
                4, 3>(new SampsonPinholeResidual(direction1, direction2)));
        break;
      case CentralRefinerSettings::kSymmetricEpipolarPinhole:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                SymmetricEpipolarPinholeResidual,
                SymmetricEpipolarPinholeResidual::kResidualSize, 4, 3>(
                new SymmetricEpipolarPinholeResidual(direction1, direction2)));
        break;
      case CentralRefinerSettings::kSymmetricEpipolarCosine:
        residual_functions.push_back(
            new ceres::AutoDiffCostFunction<
                SymmetricEpipolarCosineResidual,
                SymmetricEpipolarCosineResidual::kResidualSize, 4, 3>(
                new SymmetricEpipolarCosineResidual(direction1, direction2)));
        break;
      default:
        LOG(ERROR) << "Refinement type " << settings.refinement_type
                   << " not implemented!";
    }
  }

  return residual_functions;
}

CentralRefiner::ResidualMatrix CentralRefiner::EvaluateCostFunctions(
    const SE3& frame1_from_frame2,
    const std::vector<ceres::CostFunction*>& residual_functions) {
  ResidualMatrix residuals;
  if (!residual_functions.empty()) {
    int residual_size = residual_functions[0]->num_residuals();
    residuals.resize(residual_functions.size(), residual_size);
    const double* parameter_ptrs[2] = {frame1_from_frame2.so3().data(),
                                       frame1_from_frame2.translation().data()};
    for (int i = 0; i < residual_functions.size(); ++i) {
      CHECK_EQ(residual_size, residual_functions[i]->num_residuals());
      residual_functions[i]->Evaluate(
          parameter_ptrs, residuals.data() + i * residual_size, nullptr);
    }
  } else {
    residuals.resize(0, 0);
  }
  return residuals;
}

CentralRefiner::CentralRefiner(const CentralRefinerSettings& settings)
    : settings_(settings) {}

SE3 CentralRefiner::Refine(const SE3& frame1_from_frame2_estimate,
                           const Camera& camera1, const Camera& camera2,
                           const CentralPoint2dCorrespondences& correspondences,
                           RefinementSummary& summary) const {
  SE3 frame1_from_frame2 = frame1_from_frame2_estimate;

  std::vector<ceres::CostFunction*> residual_functions =
      GetCostFunctions(frame1_from_frame2, camera1, camera2, correspondences,
                       settings_, summary.re_linearization_angles);

  ceres::LossFunction* loss_function;
  switch (settings_.loss_type) {
    case CentralRefinerSettings::kSquaredLoss:
      loss_function = nullptr;
      break;
    case CentralRefinerSettings::kHuberLoss:
      loss_function = new ceres::HuberLoss(settings_.pixel_outlier_threshold);
      break;
    case CentralRefinerSettings::kCauchyLoss:
      loss_function = new ceres::CauchyLoss(settings_.pixel_outlier_threshold);
      break;
    default:
      LOG(WARNING) << "Unknown loss function type " << settings_.loss_type;
      break;
  }

  ceres::Problem problem;
  problem.AddParameterBlock(frame1_from_frame2.so3().data(),
                            SO3::num_parameters,
                            new SO3LocalParameterization());
  problem.AddParameterBlock(frame1_from_frame2.translation().data(), 3,
                            new ceres::HomogeneousVectorParameterization(3));
  for (ceres::CostFunction* residual_function : residual_functions) {
    problem.AddResidualBlock(residual_function, loss_function,
                             frame1_from_frame2.so3().data(),
                             frame1_from_frame2.translation().data());
  }

  ceres::Solve(settings_.solver_options, &problem, &summary.solver_summary);

  summary.residuals =
      EvaluateCostFunctions(frame1_from_frame2, residual_functions);

  return frame1_from_frame2;
}

CentralRefiner::ResidualMatrix CentralRefiner::CalculateResiduals(
    const SE3& frame1_from_frame2, const Camera& camera1, const Camera& camera2,
    const CentralPoint2dCorrespondences& correspondences) const {
  ReLinearizationAngleMatrix re_linearization_angles;
  std::vector<ceres::CostFunction*> residual_functions =
      GetCostFunctions(frame1_from_frame2, camera1, camera2, correspondences,
                       settings_, re_linearization_angles);
  return EvaluateCostFunctions(frame1_from_frame2, residual_functions);
}

SymmetricReprojectionResidual::SymmetricReprojectionResidual(
    const Camera* camera1, const Camera* camera2, const Vector2& point1,
    const Vector2& point2)
    : camera1_(camera1),
      camera2_(camera2),
      point1_(point1),
      point2_(point2),
      direction1_(camera1_->Unmap(point1_)),
      direction2_(camera2_->Unmap(point2_)) {
  CHECK(camera1_->IsOnImage(point1_));
  CHECK(camera2_->IsOnImage(point2_));
}

DotProductResidual::DotProductResidual(const Vector3& direction1,
                                       const Vector3& direction2)
    : direction1_(direction1), direction2_(direction2) {}

Sampson3dResidual::Sampson3dResidual(const Camera* camera1,
                                     const Camera* camera2,
                                     const Vector2& point1,
                                     const Vector2& point2,
                                     const Vector3* linearization_direction1,
                                     const Vector3* linearization_direction2)
    : camera1_(camera1),
      camera2_(camera2),
      point1_(point1),
      point2_(point2),
      linearization_point1_(linearization_direction1
                                ? camera1_->Map(*linearization_direction1)
                                : point1),
      linearization_point2_(linearization_direction2
                                ? camera2_->Map(*linearization_direction2)
                                : point2),
      direction1_(camera1_->Unmap(point1_)),
      direction2_(camera2_->Unmap(point2_)),
      linearization_direction1_(
          linearization_direction1 ? *linearization_direction1 : direction1_),
      linearization_direction2_(
          linearization_direction2 ? *linearization_direction2 : direction2_),
      J_left1(JLeft(camera1_, direction1_)),
      J_left2(JLeft(camera2_, direction2_)) {
  CHECK(camera1_->IsOnImage(point1_));
  CHECK(camera2_->IsOnImage(point2_));
  CHECK(linearization_direction1_.allFinite());
  CHECK(linearization_direction2_.allFinite());

  delta_point_ << point1_ - linearization_point1_,
      point2_ - linearization_point2_;
}

SampsonOnPlaneResidual::SampsonOnPlaneResidual(const Camera* camera1,
                                               const Camera* camera2,
                                               const Vector2& point1,
                                               const Vector2& point2)
    : camera1_(camera1),
      camera2_(camera2),
      normalized_point1_(camera1_->Unmap(point1).hnormalized()),
      normalized_point2_(camera2_->Unmap(point2).hnormalized()),
      Jphi1_(camera1_->DifferentiateNormalizedMap(normalized_point1_)),
      Jphi2_(camera2_->DifferentiateNormalizedMap(normalized_point2_)),
      Jinv1_(Jphi1_.inverse()),
      Jinv2_(Jphi2_.inverse()),
      Jinv1_Jinv1T_(Jinv1_ * Jinv1_.transpose()),
      Jinv2_Jinv2T_(Jinv2_ * Jinv2_.transpose()) {}

AlgebraicResidual::AlgebraicResidual(const Vector3& direction1,
                                     const Vector3& direction2)
    : direction1_(direction1), direction2_(direction2) {}

SampsonPinholeResidual::SampsonPinholeResidual(const Vector3& direction_1,
                                               const Vector3& direction_2)
    : direction1_(direction_1), direction2_(direction_2) {}

SymmetricEpipolarPinholeResidual::SymmetricEpipolarPinholeResidual(
    const Vector3& direction_1, const Vector3& direction_2)
    : direction1_(direction_1), direction2_(direction_2) {}

SymmetricEpipolarCosineResidual::SymmetricEpipolarCosineResidual(
    const Vector3& direction_1, const Vector3& direction_2)
    : direction1_(direction_1), direction2_(direction_2) {}

}  // namespace grpose