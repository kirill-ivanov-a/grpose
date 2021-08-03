#ifndef GRPOSE_CAMERA_CAMERA_MODEL_GENERIC_
#define GRPOSE_CAMERA_CAMERA_MODEL_GENERIC_

#include <fmt/format.h>

#include "camera/camera_model_pinhole.h"
#include "camera/camera_model_scaramuzza.h"
#include "camera/camera_model_unified.h"

namespace grpose {

template <typename DirectionDerived>
Eigen::Matrix<typename DirectionDerived::Scalar, 2, 1> CameraModelMap(
    CameraModelId model_id, const std::vector<double> &parameters,
    const Eigen::MatrixBase<DirectionDerived> &direction);

template <typename DirectionDerived>
DifferentiatedMapResult CameraModelDifferentiateMap(
    CameraModelId model_id, const std::vector<double> &parameters,
    const Eigen::MatrixBase<DirectionDerived> &direction);

template <typename PointDerived>
Vector3 CameraModelUnmap(CameraModelId model_id,
                         const std::vector<double> &parameters,
                         const Eigen::MatrixBase<PointDerived> &point);

template <typename PointDerived>
Vector3 CameraModelUnmapApproximate(
    CameraModelId model_id, const std::vector<double> &parameters,
    const Eigen::MatrixBase<PointDerived> &point);

template <typename PointDerived>
Vector3 CameraModelUnmapUnnormalized(
    CameraModelId model_id, const std::vector<double> &parameters,
    const Eigen::MatrixBase<PointDerived> &point);

// Implementation

namespace camera_model_generic_internal {

template <typename EigenMatrixType>
using Vector2t = Eigen::Matrix<typename EigenMatrixType::Scalar, 2, 1>;

template <typename EigenMatrixType>
using Vector3t = Eigen::Matrix<typename EigenMatrixType::Scalar, 3, 1>;

}  // namespace camera_model_generic_internal

#define GRPOSE_DEFINE_CAMERA_MODEL_METHOD(MethodName, InputType, OutputType) \
  template <typename InputType>                                              \
  OutputType CameraModel##MethodName(                                        \
      CameraModelId model_id, const std::vector<double> &parameters,         \
      const Eigen::MatrixBase<InputType> &__input) {                         \
    switch (model_id) {                                                      \
      case CameraModelPinhole::kModelId:                                     \
        return CameraModelPinhole::MethodName(__input, parameters);          \
      case CameraModelMultiFov::kModelId:                                    \
        return CameraModelMultiFov::MethodName(__input, parameters);         \
      case CameraModelUnified::kModelId:                                     \
        return CameraModelUnified::MethodName(__input, parameters);          \
      default:                                                               \
        throw std::domain_error(                                             \
            fmt::format("Unknown CameraModelId: {}", model_id));             \
    }                                                                        \
  }

GRPOSE_DEFINE_CAMERA_MODEL_METHOD(
    Map, PointDerived, camera_model_generic_internal::Vector2t<PointDerived>);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(DifferentiateMap, DirectionDerived,
                                  DifferentiatedMapResult);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(Unmap, PointDerived, Vector3);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(UnmapApproximate, PointDerived, Vector3);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(UnmapUnnormalized, PointDerived, Vector3);

}  // namespace grpose

#endif