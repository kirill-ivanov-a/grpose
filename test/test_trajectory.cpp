#include <glog/logging.h>
#include <gtest/gtest.h>

#include "metrics.h"
#include "trajectory.h"

using namespace grpose;

// The path from the current working directory to the directory of the
// executable that is being run. This allows for finding test files which are
// stored relative to the executable.
static fs::path kExecutableParentDpath;

TEST(Trajectory, FromFile) {
  // TODO No floating-point comparisons!
  // Define the timestamps and poses that are expected to exist inside of the
  // test file.
  const Timestamp timestamp_1{1234567891234567891};
  const SE3 pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}}, Vector3{1.0, 2.0, 3.0}};
  const Timestamp timestamp_2{2345678912345678912};
  const SE3 pose_2{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}}, Vector3{2.0, 3.0, 1.0}};

  // Read the timestamped poses from the file.
  const fs::path timestamped_poses_fpath =
      kExecutableParentDpath / "test-file_timestamped-poses.txt";
  Trajectory traj = Trajectory::FromFile(timestamped_poses_fpath);

  // Expect that two timestamped poses were loaded from file.
  EXPECT_EQ(traj.Size(), 2);

  // Expect that the first timestamped pose matches expectations.
  EXPECT_TRUE(traj.Contains(timestamp_1));

  EXPECT_EQ(traj.WorldFromFrameAt(timestamp_1).matrix(), pose_1.matrix());

  // Expect that the second timestamped pose matches expectations.
  EXPECT_TRUE(traj.Contains(timestamp_2));
  EXPECT_EQ(traj.WorldFromFrameAt(timestamp_2).matrix(), pose_2.matrix());
}

TEST(Trajectory, Construction) {
  {
    // Define a random set of timestamps and poses to be in the trajectory.
    const Timestamp timestamp_1{1234567891234567891};
    const SE3 pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                     Vector3{1.0, 2.0, 3.0}};
    const Timestamp timestamp_2{2345678912345678912};
    const SE3 pose_2{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                     Vector3{2.0, 3.0, 1.0}};
    const Timestamp timestamp_3{3456789123456789123};
    const SE3 pose_3{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}},
                     Vector3{3.0, 1.0, 2.0}};
    const StdMapA<Timestamp, SE3> timestamped_poses{
        {{timestamp_1, pose_1}, {timestamp_2, pose_2}, {timestamp_3, pose_3}}};
    const Trajectory trajectory{timestamped_poses};

    EXPECT_EQ(trajectory.Size(), 3);
  }
}

TEST(Trajectory, WorldFromFrameAt) {
  // Should throw an exception when trying to interpolate to an unbounded
  // timestamp. Shouldn't throw an exception if the timestamp is bounded.
  {
    const Timestamp timestamp_1{10};
    const SE3 pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}}, Vector3{0, 0, 0}};
    const Timestamp timestamp_2{20};
    const SE3 pose_2{SO3{Quaternion{0, 0, 0, 1}}, Vector3{1, 2, 3}};
    const StdMapA<Timestamp, SE3> timestamped_poses{
        {{timestamp_1, pose_1}, {timestamp_2, pose_2}}};
    const Trajectory traj{timestamped_poses};

    EXPECT_THROW(traj.WorldFromFrameAt(9), std::runtime_error);
    EXPECT_THROW(traj.WorldFromFrameAt(21), std::runtime_error);
    EXPECT_NO_THROW(traj.WorldFromFrameAt(15));
  }
}

TEST(Trajectory, AlignTo) {
  // Should throw an exception since the reference trajectory is not bounded by
  // the base trajectory.
  {
    // Define some set of timestamps and poses to be in the base trajectory.
    const Timestamp base_timestamp_1{1234567891234567891};
    const SE3 base_pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                          Vector3{1.0, 2.0, 3.0}};
    const Timestamp base_timestamp_2{2345678912345678912};
    const SE3 base_pose_2{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                          Vector3{2.0, 3.0, 1.0}};
    const Timestamp base_timestamp_3{3456789123456789123};
    const SE3 base_pose_3{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}},
                          Vector3{3.0, 1.0, 2.0}};
    const StdMapA<Timestamp, SE3> base_timestamped_poses{
        {{base_timestamp_1, base_pose_1},
         {base_timestamp_2, base_pose_2},
         {base_timestamp_3, base_pose_3}}};
    const Trajectory base_traj{base_timestamped_poses};

    // Create a reference trajectory which is just the base trajectory but with
    // some rigid transformation applied to each pose and the first timestamp
    // moved earlier.
    const SE3 ref_from_base{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}},
                            Vector3{1.0, -1.0, 2.0}};
    const StdMapA<Timestamp, SE3> ref_timestamped_poses{
        {{base_timestamp_1 - 1, ref_from_base * base_pose_1},
         {base_timestamp_2, ref_from_base * base_pose_2},
         {base_timestamp_3, ref_from_base * base_pose_3}}};
    const Trajectory ref_traj{ref_timestamped_poses};

    EXPECT_THROW(ref_traj.AlignTo(base_traj), std::runtime_error);
  }

  // Create some base trajectory and some reference trajectory which is the same
  // as the base trajectory but transformed by some rigid transform. After
  // alignment, the reference trajectory should equal the base trajectory.
  {
    // Define some set of timestamps and poses to be in the base trajectory.
    const Timestamp base_timestamp_1{1234567891234567891ll};
    const SE3 base_pose_1{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                          Vector3{1.0, 2.0, 3.0}};
    const Timestamp base_timestamp_2{2345678912345678912ll};
    const SE3 base_pose_2{SO3{Quaternion{1.0, 0.0, 0.0, 0.0}},
                          Vector3{2.0, 3.0, 1.0}};
    const Timestamp base_timestamp_3{3456789123456789123ll};
    const SE3 base_pose_3{SO3{Quaternion{0.5, 0.5, 0.5, 0.5}},
                          Vector3{3.0, 1.0, 2.0}};
    const StdMapA<Timestamp, SE3> base_timestamped_poses{
        {{base_timestamp_1, base_pose_1},
         {base_timestamp_2, base_pose_2},
         {base_timestamp_3, base_pose_3}}};
    const Trajectory base_traj{base_timestamped_poses};

    // Create a reference trajectory which is just the base trajectory but with
    // some rigid transformation applied to each pose.
    std::mt19937 mt(42);
    const SE3 basew_from_refw{SO3::sampleUniform(mt), Vector3{1.0, -1.0, 2.0}};
    // world_from_ref = world_from_base * base_from_ref
    const Trajectory ref_traj = base_traj.LeftTransform(basew_from_refw);
    const Trajectory aligned_traj = ref_traj.AlignTo(base_traj);

    std::vector<double> trans_errors =
        AbsoluteTranslationError(base_traj, aligned_traj);
    std::vector<double> rot_errors =
        AbsoluteTranslationError(base_traj, aligned_traj);

    for (int fi = 0; fi < trans_errors.size(); ++fi) {
      EXPECT_LE(trans_errors[fi], 1e-12);
      EXPECT_LE(rot_errors[fi], 1e-12);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Get the path to the parent directory of the exectuable.
  kExecutableParentDpath = fs::path(argv[0]).parent_path();

  return RUN_ALL_TESTS();
}
