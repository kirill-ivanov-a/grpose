#ifndef GRPOSE_SYNTHETIC_CAR_LIKE_SCENE_
#define GRPOSE_SYNTHETIC_CAR_LIKE_SCENE_

#include "synthetic/scene.h"

namespace grpose::synthetic {

class CarLikeScene : public Scene {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CarLikeScene();

  BearingVectorCorrespondences GetBearingVectorCorrespondences(
      int number_of_correspondences, double cross_camera_fraction = 0.0,
      unsigned long random_seed = 42) const override;

  SE3 GetWorldFromBody(int frame_index) const override;
  StdVectorA<SE3> GetBodyFromCameras() const override;

  inline double width() const;
  inline void SetWidth(double width);
  inline double length() const;
  inline void SetLength(double length);
  inline double height() const;
  inline double depth() const;
  inline void SetMotionLength(double motion_length);
  inline void SetTurnAngle(double angle);

 private:
  static SE3 WorldFromFirstFrame(double car_height);
  static SE3 WorldFromSecondFrame(double motion_length, double turn_angle,
                                  double car_height);

  double width_ = 20.0;
  double length_ = 40.0;
  double depth_ = 5.0;
  double car_height_ = 1.0;
  double height_ = 20.0;
  double motion_length_ = 5.0;
  // radians
  double turn_angle_ = 0.0;

  static constexpr int kFrontCameraIndex = 0;
  static constexpr int kRightCameraIndex = 1;
  static constexpr int kLeftCameraIndex = 2;
  static constexpr int kRearCameraIndex = 3;

  // Camera configuration in MultiCam, scaled to a realistic car size
  // clang-format off
  StdVectorA<SE3> body_from_cameras_ = {
      // front
      SE3((Matrix44() << 1,  0,  0, 0,
                         0,  0,  1, 1.6,
                         0, -1,  0, 0.1,
                         0,  0,  0, 1
          ).finished()),
      // right
      SE3((Matrix44() << 0,  0, 1, 1,
                        -1,  0, 0, 0,
                         0, -1, 0, 0.3,
                         0,  0, 0, 1
          ).finished()),
      // left
      SE3((Matrix44() << 0,  0, -1, -1,
                         1,  0,  0,  0,
                         0, -1,  0,  0.3,
                         0,  0,  0,  1
          ).finished()),
      // rear
      SE3((Matrix44() << -1,  0,  0,  0,
                          0,  0, -1, -3,
                          0, -1,  0,  0,
                          0,  0,  0,  1
          ).finished()),
  };

  // clang-format on
  SE3 world_from_body_[2];
};

// Implementation

inline double CarLikeScene::width() const { return width_; }

inline void CarLikeScene::SetWidth(double width) {
  if (width < 0)
    throw std::domain_error(
        fmt::format("CarLikeScene::SetWidth: negative width = {}", width));
  width_ = width;
}

inline double CarLikeScene::length() const { return length_; }

inline void CarLikeScene::SetLength(double length) {
  if (length < 0)
    throw std::domain_error(
        fmt::format("CarLikeScene::SetLength: negative length = {}", length));
  length_ = length;
}

inline double CarLikeScene::height() const { return height_; }

inline double CarLikeScene::depth() const { return depth_; }

inline void CarLikeScene::SetMotionLength(double motion_length) {
  motion_length_ = motion_length;
  world_from_body_[1] =
      WorldFromSecondFrame(motion_length_, turn_angle_, car_height_);
}

inline void CarLikeScene::SetTurnAngle(double angle) {
  turn_angle_ = angle;
  world_from_body_[1] =
      WorldFromSecondFrame(motion_length_, turn_angle_, car_height_);
}

}  // namespace grpose::synthetic

#endif