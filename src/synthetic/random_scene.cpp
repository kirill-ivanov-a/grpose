#include "synthetic/random_scene.h"

#include <random>

#include "util/sampling.h"

namespace grpose::synthetic {

RandomScene::RandomScene(unsigned long random_seed) {
  std::mt19937 mt(random_seed);
  std::uniform_real_distribution<double> camera_t_norm_distr(
      min_camera_translation_norm_, max_camera_translation_norm_);
  std::uniform_real_distribution<double> t_norm_distr(min_translation_norm_,
                                                      max_translation_norm_);
  const double t_norm = t_norm_distr(mt);

  world_from_body_[0] = SE3();
  // not quite uniform, but we don't care at this point
  world_from_body_[1] = SE3(SO3::sampleUniform(mt),
                            SampleSphereUniform(mt, Vector3::Zero(), t_norm));

  body_from_cameras_.reserve(number_of_cameras_);
  for (int i = 0; i < number_of_cameras_; ++i) {
    const double camera_t_norm = camera_t_norm_distr(mt);
    body_from_cameras_.emplace_back(
        SO3::sampleUniform(mt),
        SampleSphereUniform(mt, Vector3::Zero(), camera_t_norm));
  }
}

BearingVectorCorrespondences RandomScene::GetBearingVectorCorrespondences(
    int number_of_correspondences, double cross_camera_fraction,
    unsigned long random_seed) const {
  const int number_cross =
      static_cast<int>(cross_camera_fraction * number_of_correspondences);
  const int number_same = number_of_correspondences - number_cross;

  std::mt19937 mt(random_seed);
  std::uniform_int_distribution<int> camera_index_distr(0, number_of_cameras_);
  std::uniform_real_distribution<double> point_coordinate_distr(
      -max_absolute_point_corrdinate_, max_absolute_point_corrdinate_);

  BearingVectorCorrespondences correspondences;
  for (int point_index = 0; point_index < number_same; ++point_index) {
    const int camera_index = camera_index_distr(mt);
    const Vector3 point(point_coordinate_distr(mt), point_coordinate_distr(mt),
                        point_coordinate_distr(mt));
    Vector3 bearing_vectors[2];
    for (int frame_index = 0; frame_index < 2; ++frame_index) {
      const SE3 camera_from_world =
          (world_from_body_[frame_index] * body_from_cameras_[camera_index])
              .inverse();
      bearing_vectors[frame_index] = (camera_from_world * point).normalized();
    }
    correspondences.AddCorrespondence(bearing_vectors[0], bearing_vectors[1],
                                      camera_index, camera_index);
  }

  std::vector<int> all_camera_indices(number_of_cameras_);
  for (int ci = 0; ci < number_of_cameras_; ++ci) all_camera_indices[ci] = ci;
  std::vector<int> sampled_camera_indices;
  sampled_camera_indices.reserve(2);
  for (int point_index = 0; point_index < number_cross; ++point_index) {
    std::sample(all_camera_indices.begin(), all_camera_indices.end(),
                std::back_inserter(sampled_camera_indices), 2, mt);
    const Vector3 point(point_coordinate_distr(mt), point_coordinate_distr(mt),
                        point_coordinate_distr(mt));
    Vector3 bearing_vectors[2];
    for (int frame_index = 0; frame_index < 2; ++frame_index) {
      const SE3 camera_from_world =
          (world_from_body_[frame_index] *
           body_from_cameras_[sampled_camera_indices[frame_index]])
              .inverse();
      bearing_vectors[frame_index] = (camera_from_world * point).normalized();
    }
    correspondences.AddCorrespondence(bearing_vectors[0], bearing_vectors[1],
                                      sampled_camera_indices[0],
                                      sampled_camera_indices[1]);

    sampled_camera_indices.clear();
  }

  return correspondences;
}

SE3 RandomScene::GetWorldFromBody(int frame_index) const {
  // TODO
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, 2);

  return world_from_body_[frame_index];
}

StdVectorA<SE3> RandomScene::GetBodyFromCameras() const {
  return body_from_cameras_;
}

}  // namespace grpose::synthetic
