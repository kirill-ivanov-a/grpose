#ifndef GRPOSE_CAMERA_CAMERA_MODEL_GENERIC_
#define GRPOSE_CAMERA_CAMERA_MODEL_GENERIC_

#include <fmt/format.h>

#include "camera/camera_model_pinhole.h"

namespace grpose {

template <typename T>
Eigen::Matrix<T, 2, 1> CameraModelMap(CameraModelId model_id,
                                      const std::vector<double> &parameters,
                                      const Eigen::Matrix<T, 3, 1> &direction);

bool CameraModelIsMappable(CameraModelId model_id,
                           const std::vector<double> &parameters,
                           const Vector3 &direction);

DifferentiatedMapResult CameraModelDifferentiateMap(
    CameraModelId model_id, const std::vector<double> &parameters,
    const Vector3 &direction);

Vector3 CameraModelUnmap(CameraModelId model_id,
                         const std::vector<double> &parameters,
                         const Vector2 &point);

Vector3 CameraModelUnmapApproximation(CameraModelId model_id,
                                      const std::vector<double> &parameters,
                                      const Vector2 &point);

Vector3 CameraModelUnmapUnnormalized(CameraModelId model_id,
                                     const std::vector<double> &parameters,
                                     const Vector2 &point);

namespace camera_model_generic_internal {

template <typename T>
using Vector2t = Eigen::Matrix<T, 2, 1>;

template <typename T>
using Vector3t = Eigen::Matrix<T, 3, 1>;

}  // namespace camera_model_generic_internal

#define GRPOSE_DEFINE_CAMERA_MODEL_METHOD(MethodName, InputType, OutputType) \
  OutputType CameraModel##MethodName(CameraModelId model_id,                 \
                                     const std::vector<double> &parameters,  \
                                     InputType __input) {                    \
    switch (model_id) {                                                      \
      case CameraModelPinhole::model_id:                                     \
        return CameraModelPinhole::MethodName(__input, parameters);          \
      default:                                                               \
        throw std::domain_error(                                             \
            fmt::format("Unknown CameraModelId: {}", model_id));             \
    }                                                                        \
  }

template <typename T>
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(
    Map, const camera_model_generic_internal::Vector3t<T> &,
    camera_model_generic_internal::Vector2t<T>);

}  // namespace grpose

#endif