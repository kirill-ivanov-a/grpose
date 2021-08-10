#include "grpose/bearing_vector_correspondences.h"

namespace grpose {

void BearingVectorCorrespondences::AddCorrespondence(
    const Vector3 &bearing_vector1, const Vector3 &bearing_vector2,
    int camera_index1, int camera_index2) {
  bearing_vectors_[0].push_back(bearing_vector1);
  bearing_vectors_[1].push_back(bearing_vector2);
  camera_indices_[0].push_back(camera_index1);
  camera_indices_[1].push_back(camera_index2);
}

namespace {

std::pair<opengv::translations_t, opengv::rotations_t> ToOpengvVectors(
    const StdVectorA<SE3> &transformations) {
  opengv::translations_t translations;
  opengv::rotations_t rotations;
  translations.reserve(transformations.size());
  rotations.reserve(transformations.size());
  for (const SE3 &transformation : transformations) {
    translations.push_back(transformation.translation());
    rotations.push_back(transformation.rotationMatrix());
  }
  return {translations, rotations};
}

}  // namespace

BearingVectorCorrespondences::OpengvAdapter
BearingVectorCorrespondences::ToOpengvAdapter(
    const StdVectorA<SE3> &body_from_cameras) const {
  auto [camera_translations, camera_rotations] =
      ToOpengvVectors(body_from_cameras);
  return OpengvAdapter(bearing_vectors_[0], bearing_vectors_[1],
                       camera_indices_[0], camera_indices_[1],
                       camera_translations, camera_rotations);
}

}  // namespace grpose