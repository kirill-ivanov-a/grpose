#include "grpose/opengv_adapter.h"

namespace grpose {

namespace {

opengv::translations_t ToTranslations(const StdVectorA<SE3> &transformations) {
  opengv::translations_t translations;
  translations.reserve(transformations.size());
  for (const SE3 &t : transformations) translations.push_back(t.translation());
  return translations;
}

opengv::rotations_t ToRotations(const StdVectorA<SE3> &transformations) {
  opengv::rotations_t rotations;
  rotations.reserve(transformations.size());
  for (const SE3 &t : transformations) rotations.push_back(t.rotationMatrix());
  return rotations;
}

}  // namespace

OpengvAdapter::OpengvAdapter(
    const std::shared_ptr<BearingVectorCorrespondences> &correspondences,
    const StdVectorA<SE3> &body_from_cameras)
    : correspondences_(correspondences),
      body_from_camera_translations_(ToTranslations(body_from_cameras)),
      body_from_camera_rotations_(ToRotations(body_from_cameras)),
      adapter_(correspondences_->bearing_vectors(0),
               correspondences_->bearing_vectors(1),
               correspondences_->camera_indices(0),
               correspondences_->camera_indices(1),
               body_from_camera_translations_, body_from_camera_rotations_) {}

}  // namespace grpose