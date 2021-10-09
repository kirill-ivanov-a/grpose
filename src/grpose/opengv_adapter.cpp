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

std::array<std::vector<std::shared_ptr<opengv::bearingVectors_t>>, 2>
ToMultiBearingVectors(const BearingVectorCorrespondences &correspondences) {
  std::array<std::vector<std::shared_ptr<opengv::bearingVectors_t>>, 2>
      multi_bearing_vectors;
  for (int fi = 0; fi < 2; ++fi) {
    multi_bearing_vectors[fi].reserve(correspondences.NumberOfCameras());
    for (int ci = 0; ci < correspondences.NumberOfCameras(); ++ci)
      multi_bearing_vectors[fi].emplace_back(new opengv::bearingVectors_t());
  }

  for (int i = 0; i < correspondences.Size(); ++i) {
    const int ci[2] = {correspondences.camera_index(0, i),
                       correspondences.camera_index(1, i)};
    // we discard cross-camera correspondences; OpenGV's "multi" formulation
    // can't handle them
    if (ci[0] == ci[1]) {
      for (int fi = 0; fi < 2; ++fi) {
        multi_bearing_vectors[fi][ci[fi]]->push_back(
            correspondences.bearing_vector(fi, i));
      }
    }
  }

  return multi_bearing_vectors;
}

}  // namespace

OpengvAdapter::OpengvInternalMultiAdapter OpengvAdapter::ConstructMultiAdapter(
    const BearingVectorCorrespondences &correspondences,
    const opengv::translations_t &body_from_camera_translations,
    const opengv::rotations_t &body_from_camera_rotations) {
  auto [bearing_vectors0, bearing_vectors1] =
      ToMultiBearingVectors(correspondences);
  return {bearing_vectors0, bearing_vectors1, body_from_camera_translations,
          body_from_camera_rotations};
}

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
               body_from_camera_translations_, body_from_camera_rotations_),
      multi_adapter_(ConstructMultiAdapter(*correspondences,
                                           body_from_camera_translations_,
                                           body_from_camera_rotations_)) {}

}  // namespace grpose