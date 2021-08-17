#include "synthetic/car_like_scene.h"

#include <memory>

#include <gtest/gtest.h>

#include "util/geometry.h"

using namespace grpose;

// TODO if more scenes added, separate into specific scene vs general scene
class CarLikeSceneTest
    : public testing::TestWithParam<synthetic::CarLikeScene> {};

// returns points in world corordinate system
void Triangulate(const synthetic::Scene &scene,
                 const BearingVectorCorrespondences &correspondences,
                 std::vector<Vector3> &points, std::vector<double> &errors) {
  points.clear();
  errors.clear();
  points.reserve(correspondences.NumberOfCorrespondences());
  errors.reserve(correspondences.NumberOfCorrespondences());

  const SE3 frame1_from_frame0 =
      scene.GetWorldFromBody(1).inverse() * scene.GetWorldFromBody(0);
  const StdVectorA<SE3> body_from_cameras = scene.GetBodyFromCameras();

  for (int ci = 0; ci < correspondences.NumberOfCorrespondences(); ++ci) {
    const Vector3 &v0 = correspondences.bearing_vector(0, ci);
    const Vector3 &v1 = correspondences.bearing_vector(1, ci);
    const int c0 = correspondences.camera_index(0, ci);
    const int c1 = correspondences.camera_index(1, ci);

    const SE3 camera1_from_camera0 = body_from_cameras[c1].inverse() *
                                     frame1_from_frame0 * body_from_cameras[c0];
    const SE3 world_from_camera0 =
        scene.GetWorldFromBody(0) * body_from_cameras[c0];
    Vector3 point;
    double error = 0;
    Triangulate(camera1_from_camera0, v0, v1, point, &error);
    point = world_from_camera0 * point;
    points.push_back(point);
    errors.push_back(error);
  }
}

TEST_P(CarLikeSceneTest, Correspondences) {
  constexpr std::array numbers_of_correspondences = {0,  1,   2,    10,
                                                     17, 103, 11019};
  //  constexpr std::array numbers_of_correspondences = {4};
  constexpr std::array cross_camera_fractions = {0.0, 0.1, 1.0};
  //  constexpr std::array cross_camera_fractions = {1.0};
  constexpr double kMaxRelativePointError = 1e-10;

  const synthetic::CarLikeScene &scene = GetParam();

  for (int n : numbers_of_correspondences)
    for (double cc : cross_camera_fractions) {
      BearingVectorCorrespondences correspondences =
          scene.GetBearingVectorCorrespondences(n, cc);

      int number_same = 0, number_cross = 0;
      for (int i = 0; i < correspondences.NumberOfCorrespondences(); ++i) {
        if (correspondences.camera_index(0, i) ==
            correspondences.camera_index(1, i)) {
          number_same++;
        } else {
          number_cross++;
        }
      }
      EXPECT_EQ(number_same + number_cross, n);
      const int expected_cross = static_cast<int>(n * cc);
      const int expected_same = n - expected_cross;
      EXPECT_NEAR(number_same, expected_same, 3);
      EXPECT_NEAR(number_cross, expected_cross, 3);

      std::vector<Vector3> points;
      std::vector<double> errors;
      Triangulate(scene, correspondences, points, errors);
      ASSERT_EQ(points.size(), errors.size());
      ASSERT_EQ(points.size(), correspondences.NumberOfCorrespondences());

      for (int i = 0; i < points.size(); ++i)
        ASSERT_LT(errors[i] / points[i].norm(), kMaxRelativePointError)
            << fmt::format("i={} ci= {} {}, e={} p={}", i,
                           correspondences.camera_index(0, i),
                           correspondences.camera_index(1, i), errors[i],
                           points[i].transpose());

      // next follow implementation-specific tests
      const double height = scene.height();
      const double width = scene.width();
      const double length = scene.length();
      const double depth = scene.depth();

      for (int i = 0; i < points.size(); ++i) {
        const Vector3 &point = points[i];
        ASSERT_GE(point[2], 0.0) << "i=" << i;
        ASSERT_LE(point[2], height) << "i=" << i;

        int ci0 = correspondences.camera_index(0, i);
        int ci1 = correspondences.camera_index(1, i);
        if (ci0 == ci1) {
          const double abs_x = std::abs(point[0]);
          const double abs_y = std::abs(point[1]);
          ASSERT_LE(abs_x, width / 2 + depth);
          ASSERT_LE(abs_y, length / 2 + depth);

          ASSERT_TRUE(abs_x >= width / 2 || abs_y >= length / 2)
              << fmt::format("i={} abs_x={} width={} abs_y={} length={}", i,
                             abs_x, width, abs_y, length);
        }
      }
    }
}

TEST_P(CarLikeSceneTest, Setters) {
  synthetic::CarLikeScene scene = GetParam();
  scene.SetWidth(50.0);
  ASSERT_DOUBLE_EQ(scene.width(), 50.0);
  scene.SetLength(50.0);
  ASSERT_DOUBLE_EQ(scene.length(), 50.0);

  const double kMotionLength = 239.228;
  scene.SetMotionLength(kMotionLength);
  SE3 rel_motion =
      scene.GetWorldFromBody(0).inverse() * scene.GetWorldFromBody(1);
  ASSERT_NEAR(rel_motion.translation().norm(), kMotionLength, 1e-10);

  const double kTurnAngle = 23.9 * M_PI / 180.0;
  scene.SetTurnAngle(kTurnAngle);
  rel_motion = scene.GetWorldFromBody(0).inverse() * scene.GetWorldFromBody(1);
  ASSERT_NEAR(rel_motion.rotationMatrix()(0, 0), std::cos(kTurnAngle), 1e-10)
      << "\n"
      << rel_motion.rotationMatrix() << std::endl;
}

synthetic::CarLikeScene GetModifiedScene() {
  synthetic::CarLikeScene scene;
  scene.SetWidth(40.0);
  scene.SetLength(80.0);
  scene.SetMotionLength(15.0);
  scene.SetTurnAngle(20.0 * M_PI / 180.0);
  return scene;
}

INSTANTIATE_TEST_SUITE_P(SceneSuite, CarLikeSceneTest,
                         testing::Values(synthetic::CarLikeScene(),
                                         GetModifiedScene()));

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
