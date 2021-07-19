#include "trajectory.h"

#include <glog/logging.h>

#include "util.h"

namespace mcam {

Trajectory Trajectory::fromFile(const fs::path &filename) {
  // Open the specified file.
  assert(fs::is_regular_file(filename));
  std::ifstream ifs(filename, std::ios_base::in);

  StdMapA<Timestamp, SE3> timestamped_poses;
  std::string line;
  // If there is another line in the file, then read it.
  while (std::getline(ifs, line)) {
    // If the current line contains the comment character, then skip it.
    if (line.find("#") != std::string::npos) {
      continue;
    }

    // Check that the line is not comma-delimited.
    if (line.find(",") != std::string::npos) {
      std::stringstream error_msg;
      error_msg << "A comma was found in the trajectory file. Make sure it is "
                   "space-delimited, not "
                   "comma-delimited.\n  File path: "
                << filename << "\n  Line: " << line << std::endl;
      throw std::runtime_error(error_msg.str());
    }

    Timestamp t;
    double px, py, pz, qx, qy, qz, qw;

    // Read the timestamp, the position, and the rotation from the current line.
    std::istringstream line_ss{line};
    line_ss >> t >> px >> py >> pz >> qx >> qy >> qz >> qw;

    DLOG(INFO) << "t: " << t << ", px: " << px << ", py: " << py
               << ", pz: " << pz << ", qx: " << qx << ", qy: " << qy
               << ", qz: " << qz << ", qw: " << qw << std::endl;

    // Create an SE3 object.
    const Quaternion quat{qw, qx, qy, qz};
    const SO3 rotation{quat};
    const Vector3 translation{px, py, pz};
    const SE3 pose{rotation, translation};

    // Store the timestamp and pose into the map.
    timestamped_poses.emplace(std::make_pair(t, pose));
  }

  return Trajectory{timestamped_poses};
}

Trajectory::Trajectory(const StdMapA<Timestamp, SE3> &timestamped_poses)
    : timestamped_poses_(timestamped_poses) {
  for (const auto &[timestamp, pose] : timestampedPoses()) {
    timestamps_.emplace_back(timestamp);
    poses_.emplace_back(pose);
  }

  cum_translation_.resize(timestamps_.size());
  cum_rotation_.resize(timestamps_.size());
  if (!timestamps_.empty()) {
    cum_translation_[0] = 0.0;
    cum_rotation_[0] = 0.0;
    for (int i = 1; i < timestamps_.size(); ++i) {
      SE3 i_from_im1 = poses_[i].inverse() * poses_[i - 1];
      cum_translation_[i] =
          cum_translation_[i - 1] + i_from_im1.translation().norm();
      cum_rotation_[i] = cum_rotation_[i - 1] + i_from_im1.so3().log().norm();
    }
  }
}

int Trajectory::size() const { return poses_.size(); }

bool Trajectory::contains(Timestamp t) const {
  return timestampedPoses().count(t) > 0;
}

const StdMapA<Timestamp, SE3> &Trajectory::timestampedPoses() const {
  return timestamped_poses_;
}

const std::vector<Timestamp> &Trajectory::timestamps() const {
  return timestamps_;
}

Timestamp Trajectory::firstTimestamp() const { return timestamps().front(); }

Timestamp Trajectory::lastTimestamp() const { return timestamps().back(); }

const SE3 &Trajectory::worldFromFirstFrame() const { return poses_.front(); }

bool Trajectory::bounds(Timestamp t) const {
  return firstTimestamp() <= t && t <= lastTimestamp();
}

bool Trajectory::bounds(Trajectory traj) const {
  return firstTimestamp() <= traj.firstTimestamp() &&
         traj.lastTimestamp() <= lastTimestamp();
}

bool Trajectory::boundedBy(Trajectory traj) const {
  return traj.firstTimestamp() <= firstTimestamp() &&
         lastTimestamp() <= traj.lastTimestamp();
}

SE3 Trajectory::worldFromFrameAt(Timestamp t) const {
  // If the exact specified timestamp is in the trajectory, the simply return
  // the associated pose.
  if (timestampedPoses().count(t) > 0) {
    return timestampedPoses().at(t);
  }

  // The exact specified timestamp is not in the trajectory, so we will need to
  // interpolate between timestamps. Check that the trajectory bounds the
  // specified timestamp.
  if (!bounds(t)) {
    std::stringstream error_msg;
    error_msg << "This trajectory does not bound the specified timestamp, so "
                 "it cannot be "
                 "interpolated.\n  Specified timestamp: "
              << t << "\n  Trajectory first timestamp: " << firstTimestamp()
              << "\n  Trajectory last timestamp: " << lastTimestamp();
    throw std::runtime_error(error_msg.str());
  }

  // Find the high and low indices which differ by 1 and surround the specified
  // timestamp.
  const int high_idx =
      std::lower_bound(timestamps().begin(), timestamps().end(), t) -
      timestamps().begin();
  const int low_idx = high_idx - 1;
  assert(high_idx > 0 && high_idx < size());

  // Get the timestamps at the computed high and low indices.
  const Timestamp high_t = timestamps().at(high_idx);
  const Timestamp low_t = timestamps().at(low_idx);

  // low_from_high = low_from_world * world_from_high
  SE3 low_from_high =
      worldFromFrameAt(low_t).inverse() * worldFromFrameAt(high_t);

  // Compute what portion of the way from low_t to high_t is the specified
  // timestamp. Between 0 and 1.
  double t_frac = double(t - low_t) / (high_t - low_t);
  SE3 low_from_t = SE3::exp(t_frac * low_from_high.log());

  return worldFromFrameAt(low_t) *
         low_from_t;  // world_from_t = world_from_low * low_from_t
}

Trajectory Trajectory::rightTransform(const SE3 &old_from_new) const {
  StdMapA<Timestamp, SE3> new_timestamped_poses;
  for (const auto &[timestamp, world_from_old] : timestampedPoses()) {
    // world_from_new = world_from_old * old_from_new
    new_timestamped_poses.emplace(
        std::make_pair(timestamp, world_from_old * old_from_new));
  }

  return Trajectory{new_timestamped_poses};
}

Trajectory Trajectory::leftTransform(
    const SE3 &new_world_from_old_world) const {
  StdMapA<Timestamp, SE3> new_timestamped_poses;
  for (const auto &[timestamp, old_world_from_old] : timestampedPoses()) {
    new_timestamped_poses.emplace(std::make_pair(
        timestamp, new_world_from_old_world * old_world_from_old));
  }

  return Trajectory{new_timestamped_poses};
}

Trajectory Trajectory::alignTo(const Trajectory &base) const {
  // Check that this trajectory's first timestamp is bounded by the base
  // trajectory.
  if (!base.bounds(firstTimestamp())) {
    std::stringstream error_msg;
    error_msg << "This trajectory's first timestamp is not bounded by the base "
                 "trajectory, so this "
                 "trajectory cannot be properly aligned to the base "
                 "trajectory.\n  This "
                 "first timestamp: "
              << firstTimestamp()
              << "\n  This last timestamp: " << lastTimestamp()
              << "\n  Base first "
                 "timestamp: "
              << base.firstTimestamp()
              << "\n  Base last timestamp: " << base.lastTimestamp();
    throw std::runtime_error(error_msg.str());
  }

  const SE3 worldr_from_ref = worldFromFirstFrame();
  const SE3 worldb_from_base = base.worldFromFrameAt(firstTimestamp());
  const SE3 worldb_from_worldr = worldb_from_base * worldr_from_ref.inverse();

  return leftTransform(worldb_from_worldr);
}

bool Trajectory::operator==(const Trajectory &other) const {
  if (size() != other.size()) {
    return false;
  }

  for (const auto &[timestamp, pose] : timestampedPoses()) {
    if (other.contains(timestamp) == 0) {
      return false;
    }
    if (pose.translation() != other.worldFromFrameAt(timestamp).translation() ||
        pose.unit_quaternion().coeffs() !=
            other.worldFromFrameAt(timestamp).unit_quaternion().coeffs()) {
      return false;
    }
  }

  return true;
}

std::ostream &operator<<(std::ostream &stream, const Trajectory &trajectory) {
  for (Timestamp timestamp : trajectory.timestamps()) {
    const SE3 world_from_frame = trajectory.worldFromFrameAt(timestamp);
    const Vector3 t = world_from_frame.translation();
    const Quaternion q = world_from_frame.unit_quaternion();
    stream << timestamp << ' ' << t[0] << ' ' << t[1] << ' ' << t[2] << ' '
           << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  return stream;
}

void Trajectory::saveToFile(const fs::path &filename) const {
  std::ofstream stream(filename);
  stream.precision(15);
  stream << (*this);
}

double Trajectory::cumulativeTranslation(Timestamp from, Timestamp to) const {
  return cumulativeTranslationOrRotation(from, to, true);
}

double Trajectory::cumulativeRotation(Timestamp from, Timestamp to) const {
  return cumulativeTranslationOrRotation(from, to, false);
}

double Trajectory::cumulativeTranslationOrRotation(Timestamp from, Timestamp to,
                                                   bool isTranslation) const {
  if (from > to) throw std::runtime_error("Cumulative translation from > to");
  if (!bounds(from))
    throw std::runtime_error(
        "Cumulative translation or rotation \"from\"(=" + std::to_string(from) +
        ") not in the trajectory [" + std::to_string(firstTimestamp()) + ", " +
        std::to_string(lastTimestamp()) + "]");
  if (!bounds(to))
    throw std::runtime_error(
        "Cumulative translation or rotation \"to\" not in the trajectory");

  if (from == to) return 0.0;

  const int high_idx_from =
      std::lower_bound(timestamps().begin(), timestamps().end(), from) -
      timestamps().begin();
  const int low_idx_to =
      int(std::upper_bound(timestamps().begin(), timestamps().end(), to) -
          timestamps().begin()) -
      1;

  if (high_idx_from > low_idx_to) {
    const SE3 end_from_start =
        worldFromFrameAt(to).inverse() * worldFromFrameAt(from);
    if (isTranslation)
      return end_from_start.translation().norm();
    else
      return end_from_start.so3().log().norm();
  }

  const SE3 world_from_start = worldFromFrameAt(from);
  const SE3 world_from_end = worldFromFrameAt(to);
  const SE3 high_idx_from_start =
      poses_[high_idx_from].inverse() * world_from_start;
  const SE3 low_idx_from_end = poses_[low_idx_to].inverse() * world_from_end;
  if (isTranslation)
    return high_idx_from_start.translation().norm() +
           (cum_translation_[low_idx_to] - cum_translation_[high_idx_from]) +
           low_idx_from_end.translation().norm();
  else
    return high_idx_from_start.so3().log().norm() +
           (cum_rotation_[low_idx_to] - cum_rotation_[high_idx_from]) +
           low_idx_from_end.so3().log().norm();
}

}  // namespace mcam
