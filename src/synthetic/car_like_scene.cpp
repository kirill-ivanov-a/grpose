#include "synthetic/car_like_scene.h"

namespace grpose::synthetic {

SE3 CarLikeScene::WorldFromSecondFrame(double lengthwise_motion,
                                       double turn_angle) {
  const double cos_angle = std::cos(turn_angle);
  const double widthwise_motion = lengthwise_motion * std::tan(turn_angle);
  return SE3(SO3::rotZ(-turn_angle),
             Vector3(widthwise_motion, lengthwise_motion, 0.0));
}

CarLikeScene::CarLikeScene()
    : world_from_body_{SE3(),
                       WorldFromSecondFrame(lengthwise_motion_, turn_angle_)} {}

SE3 CarLikeScene::GetWorldFromBody(int frame_index) const {
  if (frame_index < 0 || frame_index > 1)
    throw std::domain_error(fmt::format(
        "CarLikeScene::GetBodyToWorld: frame_index {} not in {0, 1}",
        frame_index));
  return world_from_body_[frame_index];
}

StdVectorA<SE3> CarLikeScene::GetBodyFromCameras() const {
  return body_from_cameras_;
}

BearingVectorCorrespondences CarLikeScene::GetBearingVectorCorrespondences(
    int number_of_correspondences, double angular_deviation,
    double cross_camera_fraction, unsigned long random_seed) const {
  BearingVectorCorrespondences correspondences;
  const int number_cross =
      static_cast<int>(cross_camera_fraction * number_of_correspondences);
  const int number_same = number_of_correspondences - number_cross;
  const int number_left =
      static_cast<int>(number_same * length_ / (length_ + width_)) / 2;
  const int number_front = number_same / 2 - number_left;
  const int number_corner = number_cross / 4;
  const int number_right = number_of_correspondences -
                           (number_left + 2 * number_front + 4 * number_corner);

  std::mt19937 mt(random_seed);
  std::uniform_real_distribution x_right(width_ / 2, width_ / 2 + depth_);
  std::uniform_real_distribution x_front(-width_ / 2, width_ / 2);
  std::uniform_real_distribution y_right(-length_ / 2, length_ / 2);
  std::uniform_real_distribution y_front(length_ / 2, length_ / 2 + depth_);
  std::uniform_real_distribution z(0.0, height_);

  // Adding same-camera correspondences
  for (int side : {-1, 1}) {
    const int side_camera_index =
        side == 1 ? kRightCameraIndex : kLeftCameraIndex;
    const int other_camera_index =
        side == 1 ? kFrontCameraIndex : kRearCameraIndex;
    const int number_side = side == 1 ? number_right : number_left;
    SE3 side_from_world[2];
    SE3 other_from_world[2];
    for (int frame_index : {0, 1}) {
      side_from_world[frame_index] = (world_from_body_[frame_index] *
                                      body_from_cameras_[side_camera_index])
                                         .inverse();
      other_from_world[frame_index] = (world_from_body_[frame_index] *
                                       body_from_cameras_[other_camera_index])
                                          .inverse();
    }

    // Right or left camera correspondences
    for (int i = 0; i < number_side; ++i) {
      const Vector3 point(side * x_right(mt), y_right(mt), z(mt));
      Vector3 bearing_vector[2];
      for (int frame_index : {0, 1})
        bearing_vector[frame_index] =
            (side_from_world[frame_index] * point).normalized();
      correspondences.AddCorrespondence(bearing_vector[0], bearing_vector[1],
                                        side_camera_index, side_camera_index);
    }

    // Front or rear camera correspondences
    for (int i = 0; i < number_front; ++i) {
      const Vector3 point(x_front(mt), side * y_front(mt), z(mt));
      Vector3 bearing_vector[2];
      for (int frame_index : {0, 1})
        bearing_vector[frame_index] =
            (other_from_world[frame_index] * point).normalized();
      correspondences.AddCorrespondence(bearing_vector[0], bearing_vector[1],
                                        other_camera_index, other_camera_index);
    }
  }

  // Adding cross-camera correspondences
  for (int side_camera_index : {kRightCameraIndex, kLeftCameraIndex})
    for (int other_camera_index : {kFrontCameraIndex, kRearCameraIndex}) {
      const double x_mult = side_camera_index == kRightCameraIndex ? 1.0 : -1.0;
      const double y_mult =
          other_camera_index == kFrontCameraIndex ? 1.0 : -1.0;
      const Vector3 point(x_mult * x_right(mt), y_mult * y_front(mt), z(mt));
      const bool is_front_0 = static_cast<bool>(mt() & 1);
      const int side_frame_index = is_front_0 ? 1 : 0;
      const int other_frame_index = is_front_0 ? 0 : 1;
      const SE3 side_from_world = (world_from_body_[side_frame_index] *
                                   body_from_cameras_[side_camera_index])
                                      .inverse();
      const SE3 other_from_world = (world_from_body_[other_frame_index] *
                                    body_from_cameras_[other_camera_index])
                                       .inverse();
      const Vector3 side_vector = (side_from_world * point).normalized();
      const Vector3 other_vector = (other_from_world * point).normalized();
      correspondences.AddCorrespondence(side_vector, other_vector,
                                        side_camera_index, other_camera_index);
    }

  return correspondences;
}

}  // namespace grpose::synthetic