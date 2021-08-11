#ifndef GRPOSE_SYNTHETIC_CAR_LIKE_SCENE_
#define GRPOSE_SYNTHETIC_CAR_LIKE_SCENE_

#include "synthetic/scene.h"

namespace grpose::synthetic {

class CarLikeScene : public Scene {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CarLikeScene();

  BearingVectorCorrespondences GetBearingVectorCorrespondences(
      int number_of_correspondences, double angular_deviation = 0.0,
      double cross_camera_fraction = 0.0,
      unsigned long random_seed = 42) const override;

  SE3 GetWorldFromBody(int frame_index) const override;
  StdVectorA<SE3> GetBodyFromCameras() const override;

  inline void SetWidth(double width);
  inline void SetLength(double length);
  inline void SetLengthwiseMotion(double lengthwise_motion);
  inline void SetTurnAngle(double angle);

 private:
  static SE3 WorldFromSecondFrame(double lengthwise_motion, double turn_angle);

  double width_ = 20.0;
  double length_ = 100.0;
  double depth_ = 5.0;
  double car_height_ = 1.0;
  double height_ = 20.0;
  double lengthwise_motion_ = 5.0;
  // radians
  double turn_angle_ = 0.0;

  static constexpr int kFrontCameraIndex = 0;
  static constexpr int kRightCameraIndex = 1;
  static constexpr int kLeftCameraIndex = 2;
  static constexpr int kRearCameraIndex = 3;

  // Camera configuration in MultiCam
  // clang-format off
  StdVectorA<SE3> body_from_cameras_ = {
      // front
      SE3((Matrix44() << 1,  0,  0, 0,
                         0,  0,  1, 0.016,
                         0, -1,  0, 0.001,
                         0,  0,  0, 1
          ).finished()),
      // right
      SE3((Matrix44() << 0,  0, 1, 0.01,
                        -1,  0, 0, 0,
                         0, -1, 0, 0.003,
                         0,  0, 0, 1
          ).finished()),
      // left
      SE3((Matrix44() << 0,  0, -1, -0.01,
                         1,  0,  0,  0,
                         0, -1,  0,  0.003,
                         0,  0,  0,  1
          ).finished()),
      // rear
      SE3((Matrix44() << -1,  0,  0,  0,
                          0,  0, -1, -0.03,
                          0, -1,  0,  0,
                          0,  0,  0,  1
          ).finished()),
  };

  // clang-format on
  SE3 world_from_body_[2];
};

// Implementation

inline void CarLikeScene::SetWidth(double width) {
  if (width < 0)
    throw std::domain_error(
        fmt::format("CarLikeScene::SetWidth: negative width = {}", width));
  width_ = width;
}

inline void CarLikeScene::SetLength(double length) {
  if (length < 0)
    throw std::domain_error(
        fmt::format("CarLikeScene::SetLength: negative length = {}", length));
  length_ = length;
}

inline void CarLikeScene::SetLengthwiseMotion(double lengthwise_motion) {
  lengthwise_motion_ = lengthwise_motion;
  world_from_body_[1] = WorldFromSecondFrame(lengthwise_motion_, turn_angle_);
}

inline void CarLikeScene::SetTurnAngle(double angle) {
  turn_angle_ = angle;
  world_from_body_[1] = WorldFromSecondFrame(lengthwise_motion_, turn_angle_);
}

}  // namespace grpose::synthetic

#endif