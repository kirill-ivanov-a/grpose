#include "metrics.h"

#include <glog/logging.h>
#include <gtest/gtest.h>

using namespace grpose;

static const double kEpsilon = 1e-6;

TEST(Metrics, AbsoluteTranslationError) {
  // Create some base trajectory and a reference trajectory which is a rigid
  // transformation of the base trajectory. Check that all errors are zero.
  {
    // Define some set of timestamps and poses to be in the groundtruth
    // trajectory.
    const Timestamp gt_timestamp_1{1234567891234567891};
    const SE3 gt_pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                        Vector3{1.0, 2.0, 3.0}};
    const Timestamp gt_timestamp_2{2345678912345678912};
    const SE3 gt_pose_2{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                        Vector3{2.0, 3.0, 1.0}};
    const Timestamp gt_timestamp_3{3456789123456789123};
    const SE3 gt_pose_3{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}},
                        Vector3{3.0, 1.0, 2.0}};
    const StdMapA<Timestamp, SE3> gt_trajectory{{{gt_timestamp_1, gt_pose_1},
                                                 {gt_timestamp_2, gt_pose_2},
                                                 {gt_timestamp_3, gt_pose_3}}};

    // Create an estimate trajectory which is just the groundtruth trajectory
    // but with some rigid transformation applied to each pose.
    std::mt19937 mt(239);
    const SE3 estw_from_gtw{SO3::sampleUniform(mt), Vector3{1.0, -1.0, 2.0}};
    const StdMapA<Timestamp, SE3> est_trajectory{
        {{gt_timestamp_1, estw_from_gtw * gt_pose_1},
         {gt_timestamp_2, estw_from_gtw * gt_pose_2},
         {gt_timestamp_3, estw_from_gtw * gt_pose_3}}};

    // Compute the ATE between the groundtruth and the estimate.
    const std::vector<double> errors =
        AbsoluteTranslationError(gt_trajectory, est_trajectory);

    // Expect that there are three error values, one for each pose.
    EXPECT_EQ(errors.size(), 3);

    // Expect that all error values are zero.
    for (const double error : errors) {
      EXPECT_NEAR(error, 0.0, kEpsilon);
    }
  }
}

TEST(Metrics, AbsoluteRotationError) {
  // Create some base trajectory and a reference trajectory which is a rigid
  // transformation of the base trajectory. Check that all errors are zero.
  {
    // Define some set of timestamps and poses to be in the groundtruth
    // trajectory.
    const Timestamp gt_timestamp_1{1234567891234567891};
    const SE3 gt_pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                        Vector3{1.0, 2.0, 3.0}};
    const Timestamp gt_timestamp_2{2345678912345678912};
    const SE3 gt_pose_2{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                        Vector3{2.0, 3.0, 1.0}};
    const Timestamp gt_timestamp_3{3456789123456789123};
    const SE3 gt_pose_3{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}},
                        Vector3{3.0, 1.0, 2.0}};
    const StdMapA<Timestamp, SE3> gt_trajectory{{{gt_timestamp_1, gt_pose_1},
                                                 {gt_timestamp_2, gt_pose_2},
                                                 {gt_timestamp_3, gt_pose_3}}};

    // Create an estimate trajectory which is just the groundtruth trajectory
    // but with some rigid transformation applied to each pose.
    std::mt19937 mt(228);
    const SE3 estw_from_gtw{SO3::sampleUniform(mt), Vector3{1.0, -1.0, 2.0}};
    const StdMapA<Timestamp, SE3> est_trajectory{
        {{gt_timestamp_1, estw_from_gtw * gt_pose_1},
         {gt_timestamp_2, estw_from_gtw * gt_pose_2},
         {gt_timestamp_3, estw_from_gtw * gt_pose_3}}};

    // Compute the ATE between the groundtruth and the estimate.
    const std::vector<double> errors =
        AbsoluteRotationError(gt_trajectory, est_trajectory);

    // Expect that there are three error values, one for each pose.
    EXPECT_EQ(errors.size(), 3);

    // Expect that all error values are zero.
    for (const double error : errors) {
      EXPECT_NEAR(error, 0.0, kEpsilon);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}
