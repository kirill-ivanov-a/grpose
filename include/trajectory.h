#ifndef GRPOSE_TRAJECTORY_
#define GRPOSE_TRAJECTORY_

#include "types.h"

namespace grpose {

class Trajectory {
 public:
  /**
   * @brief - Static factory function that creates a trajectory object from a
   * file.
   * @returns - The trajectory created from the file.
   */
  static Trajectory FromFile(const fs::path &filename);

  /**
   * @brief - Construct a trajectory.
   */
  Trajectory(const StdMapA<Timestamp, SE3> &timestamped_poses);

  /**
   * @returns - The number of timestamp-pose pairs in the trajectory.
   */
  int Size() const;

  /**
   * @returns - True if the specified timestamp is in the trajectory.
   */
  bool Contains(Timestamp t) const;

  /**
   * @returns - The timestamped poses.
   */
  const StdMapA<Timestamp, SE3> &TimestampedPoses() const;

  /**
   * @returns - The timestamps, in ascending order.
   */
  const std::vector<Timestamp> &Timestamps() const;

  /**
   * @returns - The first timestamp.
   */
  Timestamp FirstTimestamp() const;

  /**
   * @returns - The last timestamp.
   */
  Timestamp LastTimestamp() const;

  /**
   * @returns - The pose with the earliest timestamp.
   */
  const SE3 &WorldFromFirstFrame() const;

  /**
   * @param timestamp - The timestamp to check if it is bounded by this
   * trajectory.
   * @returns - True if the specified timestamp is between this trajectory's
   * first and last timestamps.
   */
  bool Bounds(Timestamp timestamp) const;

  /**
   * @param trajectory - The trajectory to check if it is bounded by this
   * trajectory.
   * @returns - True if all timestamps in the specified trajectory are between
   * this trajectory's first and last timestamps.
   */
  bool Bounds(Trajectory trajectory) const;

  /**
   * @param trajectory - The trajectory to check if it bounds this trajectory.
   * @returns - True if all timestamps in this trajectory are between the
   * specified trajectory's first and last timestamps.
   */
  bool BoundedBy(Trajectory trajectory) const;

  /**
   * @param timestamp - The timestamp to query the trajectory for.
   * @returns - The pose at the specified timestamp. If this trajectory does not
   * have a pose exactly at the specified timestamp, then the trajectory is
   * interpolated to the timestamp.
   */
  SE3 WorldFromFrameAt(Timestamp timestamp) const;

  /**
   * **WARNING**: Applying rightTransform does not produce a rigid
   * transformation of the trajectory!
   * @param old_from_new - The transformation to right-multiply with every pose.
   * @returns - A copy of this trajectory where all poses were right-multiplied
   * by old_from_new.
   */
  Trajectory RightTransform(const SE3 &old_from_new) const;

  /**
   * @param new_world_from_old_world - The transformation to left-multiply with
   * every pose.
   * @returns - A copy of this trajectory where all poses were left-multiplied
   * by old_from_new.
   */
  Trajectory LeftTransform(const SE3 &new_world_from_old_world) const;

  /**
   * @param base - The base trajectory to align to. Often this is some sort of
   * groundtruth trajectory.
   * @returns - A copy of the current trajectory with a rigid transform applied
   * to all of its poses such that its first pose is aligned with the first pose
   * of the base trajectory.
   */
  Trajectory AlignTo(const Trajectory &base) const;

  /**
   * **WARNING** double value comparison.
   * @param other - The other trajectory.
   * @returns - Whether or not this trajectory is equal to the other trajectory.
   */
  bool operator==(const Trajectory &other) const;

  /**
   * Writes the trajectory in the TUM format to a stream.
   * @param stream - The stream to write into.
   * @param trajectory - The trajectory to write.
   * @returns - The modified stream.
   */
  friend std::ostream &operator<<(std::ostream &stream,
                                  const Trajectory &trajectory);

  /**
   * Convenience wrapper around the output operator to write the trajectory in
   * the TUM format into a file.
   * @param filename - The name of the file to write into.
   */
  void SaveToFile(const fs::path &filename) const;

  double CumulativeTranslation(Timestamp from, Timestamp to) const;
  double CumulativeRotation(Timestamp from, Timestamp to) const;

 private:
  double CumulativeTranslationOrRotation(Timestamp from, Timestamp to,
                                         bool isTranslation) const;

  // A map from timestamps to poses.
  StdMapA<Timestamp, SE3> timestamped_poses_;

  // Timestamps, in ascending order.
  std::vector<Timestamp> timestamps_;

  // Poses, in ascending order by timestamp.
  StdVectorA<SE3> poses_;

  // Cumulative translation from 0-th to i-th frame
  std::vector<double> cum_translation_;

  // Cumulative rotation from 0-th to i-th frame
  std::vector<double> cum_rotation_;
};

}  // namespace grpose

#endif
