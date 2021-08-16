#include "util/trajectory.h"

#include <glog/logging.h>

#include "util/util.h"

namespace grpose {

Trajectory Trajectory::FromFile(const fs::path &filename) {
  // Open the specified file.
  CHECK(fs::is_regular_file(filename));
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
      // TODO cleanup fmt
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
  for (const auto &[timestamp, pose] : TimestampedPoses()) {
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

int Trajectory::Size() const { return poses_.size(); }

bool Trajectory::Contains(Timestamp t) const {
  return TimestampedPoses().count(t) > 0;
}

const StdMapA<Timestamp, SE3> &Trajectory::TimestampedPoses() const {
  return timestamped_poses_;
}

const std::vector<Timestamp> &Trajectory::Timestamps() const {
  return timestamps_;
}

Timestamp Trajectory::FirstTimestamp() const { return Timestamps().front(); }

Timestamp Trajectory::LastTimestamp() const { return Timestamps().back(); }

const SE3 &Trajectory::WorldFromFirstFrame() const { return poses_.front(); }

bool Trajectory::Bounds(Timestamp timestamp) const {
  return FirstTimestamp() <= timestamp && timestamp <= LastTimestamp();
}

bool Trajectory::Bounds(Trajectory trajectory) const {
  return FirstTimestamp() <= trajectory.FirstTimestamp() &&
         trajectory.LastTimestamp() <= LastTimestamp();
}

bool Trajectory::BoundedBy(Trajectory trajectory) const {
  return trajectory.FirstTimestamp() <= FirstTimestamp() &&
         LastTimestamp() <= trajectory.LastTimestamp();
}

SE3 Trajectory::WorldFromFrameAt(Timestamp timestamp) const {
  // If the exact specified timestamp is in the trajectory, the simply return
  // the associated pose.
  if (TimestampedPoses().count(timestamp) > 0) {
    return TimestampedPoses().at(timestamp);
  }

  // The exact specified timestamp is not in the trajectory, so we will need to
  // interpolate between timestamps. Check that the trajectory bounds the
  // specified timestamp.
  if (!Bounds(timestamp)) {
    std::stringstream error_msg;
    error_msg << "This trajectory does not bound the specified timestamp, so "
                 "it cannot be "
                 "interpolated.\n  Specified timestamp: "
              << timestamp
              << "\n  Trajectory first timestamp: " << FirstTimestamp()
              << "\n  Trajectory last timestamp: " << LastTimestamp();
    throw std::runtime_error(error_msg.str());
  }

  // Find the high and low indices which differ by 1 and surround the specified
  // timestamp.
  const int high_idx =
      std::lower_bound(Timestamps().begin(), Timestamps().end(), timestamp) -
      Timestamps().begin();
  const int low_idx = high_idx - 1;
  assert(high_idx > 0 && high_idx < Size());

  // Get the timestamps at the computed high and low indices.
  const Timestamp high_t = Timestamps().at(high_idx);
  const Timestamp low_t = Timestamps().at(low_idx);

  // low_from_high = low_from_world * world_from_high
  SE3 low_from_high =
      WorldFromFrameAt(low_t).inverse() * WorldFromFrameAt(high_t);

  // Compute what portion of the way from low_t to high_t is the specified
  // timestamp. Between 0 and 1.
  double t_frac = double(timestamp - low_t) / (high_t - low_t);
  SE3 low_from_t = SE3::exp(t_frac * low_from_high.log());

  return WorldFromFrameAt(low_t) *
         low_from_t;  // world_from_t = world_from_low * low_from_t
}

Trajectory Trajectory::RightTransform(const SE3 &old_from_new) const {
  StdMapA<Timestamp, SE3> new_timestamped_poses;
  for (const auto &[timestamp, world_from_old] : TimestampedPoses()) {
    // world_from_new = world_from_old * old_from_new
    new_timestamped_poses.emplace(
        std::make_pair(timestamp, world_from_old * old_from_new));
  }

  return Trajectory{new_timestamped_poses};
}

Trajectory Trajectory::LeftTransform(
    const SE3 &new_world_from_old_world) const {
  StdMapA<Timestamp, SE3> new_timestamped_poses;
  for (const auto &[timestamp, old_world_from_old] : TimestampedPoses()) {
    new_timestamped_poses.emplace(std::make_pair(
        timestamp, new_world_from_old_world * old_world_from_old));
  }

  return Trajectory{new_timestamped_poses};
}

Trajectory Trajectory::AlignTo(const Trajectory &base) const {
  // Check that this trajectory's first timestamp is bounded by the base
  // trajectory.
  if (!base.Bounds(FirstTimestamp())) {
    // TODO cleanup fmt
    std::stringstream error_msg;
    error_msg << "This trajectory's first timestamp is not bounded by the base "
                 "trajectory, so this "
                 "trajectory cannot be properly aligned to the base "
                 "trajectory.\n  This "
                 "first timestamp: "
              << FirstTimestamp()
              << "\n  This last timestamp: " << LastTimestamp()
              << "\n  Base first "
                 "timestamp: "
              << base.FirstTimestamp()
              << "\n  Base last timestamp: " << base.LastTimestamp();
    throw std::runtime_error(error_msg.str());
  }

  const SE3 worldr_from_ref = WorldFromFirstFrame();
  const SE3 worldb_from_base = base.WorldFromFrameAt(FirstTimestamp());
  const SE3 worldb_from_worldr = worldb_from_base * worldr_from_ref.inverse();

  return LeftTransform(worldb_from_worldr);
}

bool Trajectory::operator==(const Trajectory &other) const {
  if (Size() != other.Size()) {
    return false;
  }

  for (const auto &[timestamp, pose] : TimestampedPoses()) {
    if (other.Contains(timestamp) == 0) {
      return false;
    }
    if (pose.translation() != other.WorldFromFrameAt(timestamp).translation() ||
        pose.unit_quaternion().coeffs() !=
            other.WorldFromFrameAt(timestamp).unit_quaternion().coeffs()) {
      return false;
    }
  }

  return true;
}

std::ostream &operator<<(std::ostream &stream, const Trajectory &trajectory) {
  for (Timestamp timestamp : trajectory.Timestamps()) {
    const SE3 world_from_frame = trajectory.WorldFromFrameAt(timestamp);
    const Vector3 t = world_from_frame.translation();
    const Quaternion q = world_from_frame.unit_quaternion();
    stream << timestamp << ' ' << t[0] << ' ' << t[1] << ' ' << t[2] << ' '
           << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  return stream;
}

void Trajectory::SaveToFile(const fs::path &filename) const {
  std::ofstream stream(filename);
  stream.precision(15);
  stream << (*this);
}

double Trajectory::CumulativeTranslation(Timestamp from, Timestamp to) const {
  return CumulativeTranslationOrRotation(from, to, true);
}

double Trajectory::CumulativeRotation(Timestamp from, Timestamp to) const {
  return CumulativeTranslationOrRotation(from, to, false);
}

double Trajectory::CumulativeTranslationOrRotation(Timestamp from, Timestamp to,
                                                   bool isTranslation) const {
  if (from > to) throw std::runtime_error("Cumulative translation from > to");
  if (!Bounds(from))
    throw std::runtime_error(
        "Cumulative translation or rotation \"from\"(=" + std::to_string(from) +
        ") not in the trajectory [" + std::to_string(FirstTimestamp()) + ", " +
        std::to_string(LastTimestamp()) + "]");
  if (!Bounds(to))
    throw std::runtime_error(
        "Cumulative translation or rotation \"to\" not in the trajectory");

  if (from == to) return 0.0;

  const int high_idx_from =
      std::lower_bound(Timestamps().begin(), Timestamps().end(), from) -
      Timestamps().begin();
  const int low_idx_to =
      int(std::upper_bound(Timestamps().begin(), Timestamps().end(), to) -
          Timestamps().begin()) -
      1;

  if (high_idx_from > low_idx_to) {
    const SE3 end_from_start =
        WorldFromFrameAt(to).inverse() * WorldFromFrameAt(from);
    if (isTranslation)
      return end_from_start.translation().norm();
    else
      return end_from_start.so3().log().norm();
  }

  const SE3 world_from_start = WorldFromFrameAt(from);
  const SE3 world_from_end = WorldFromFrameAt(to);
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

}  // namespace grpose
