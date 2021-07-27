#include "camera/camera_model_generic.h"

namespace grpose {

GRPOSE_DEFINE_CAMERA_MODEL_METHOD(IsMappable, const Vector3 &, bool);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(DifferentiateMap, const Vector3 &,
                                  DifferentiatedMapResult);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(Unmap, const Vector2 &, Vector3);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(UnmapApproximation, const Vector2 &, Vector3);
GRPOSE_DEFINE_CAMERA_MODEL_METHOD(UnmapUnnormalized, const Vector2 &, Vector3);

}  // namespace grpose