#include "dataset/robotcar_reader.h"
#include "util.h"

namespace grpose {

namespace {

SE3 Se3FromXyzrpy(double x, double y, double z, double roll, double pitch,
                  double yaw) {
  SO3 rot = SO3::rotZ(yaw) * SO3::rotY(pitch) * SO3::rotX(roll);
  return SE3(rot, Vector3(x, y, z));
}

SE3 ReadSe3FromExtrin(const fs::path &extrinsics_filename) {
  std::ifstream extrinsics_file(extrinsics_filename);
  // TODO cleanup throw
  CHECK(extrinsics_file.is_open())
      << "Could not open extrinsics file " << extrinsics_filename;
  double x, y, z, roll, pitch, yaw;
  extrinsics_file >> x >> y >> z >> roll >> pitch >> yaw;
  SE3 result = Se3FromXyzrpy(x, y, z, roll, pitch, yaw);
  return result;
}

SE3 ReadCameraFromBody(const fs::path &extrinsics_filename) {
  SE3 result = ReadSe3FromExtrin(extrinsics_filename);
  LOG(INFO) << "file: " << extrinsics_filename << "\nbody -> this:\n"
            << result.matrix();
  return result;
}

SE3 ReadLidarFromBody(const fs::path &extrinsics_filename) {
  SE3 body_from_lidar = ReadSe3FromExtrin(extrinsics_filename);
  LOG(INFO) << "file: " << extrinsics_filename << "\nthis -> body:\n"
            << body_from_lidar.matrix();
  return body_from_lidar.inverse();
}

SE3 ReadInsFromBody(const fs::path &extrinsics_filename) {
  SE3 body_from_ins = ReadSe3FromExtrin(extrinsics_filename);
  return body_from_ins.inverse();
}

void ReadTimestamps(const fs::path &timestamps_filename,
                    std::vector<Timestamp> &timestamps) {
  // TODO cleanup throw
  CHECK(fs::exists(timestamps_filename))
      << timestamps_filename.native() + " does not exist";
  std::ifstream timestamps_file(timestamps_filename);
  Timestamp timestamp;
  int one;
  while (timestamps_file >> timestamp >> one) {
    CHECK(timestamps.empty() || timestamps.back() < timestamp);
    timestamps.push_back(timestamp);
  }
}

void ReadOdometryPoses(const fs::path &odometry_filename,
                       bool fill_odometry_gaps,
                       std::vector<Timestamp> &timestamps,
                       StdVectorA<SE3> &odometry_first_from_body) {
  // TODO cleanup throw
  CHECK(fs::exists(odometry_filename))
      << odometry_filename.native() + " does not exist";

  Timestamp max_skip = -1;
  int skip_count = 0, back_skip_count = 0;
  // lbo = last but one
  SE3 last_from_lbo;

  std::ifstream odometry_file(odometry_filename);
  std::string header;
  std::getline(odometry_file, header);
  char comma;
  Timestamp source_timestamp, destination_timestamp;
  double x, y, z, roll, pitch, yaw;
  while (odometry_file >> source_timestamp >> comma >> destination_timestamp >>
         comma >> x >> comma >> y >> comma >> z >> comma >> roll >> comma >>
         pitch >> comma >> yaw) {
    if (timestamps.empty()) {
      timestamps.push_back(destination_timestamp);
      odometry_first_from_body.push_back(SE3());
    }

    SE3 last_from_source = Se3FromXyzrpy(x, y, z, roll, pitch, yaw);
    if (destination_timestamp != timestamps.back()) {
      Timestamp skip = (destination_timestamp - timestamps.back());
      if (skip < 0)
        back_skip_count++;
      else
        max_skip = std::max(skip, max_skip);
      skip_count++;

      if (fill_odometry_gaps && timestamps.size() >= 2) {
        double timestamp_fraction =
            static_cast<double>(skip) /
            static_cast<double>(timestamps.back() -
                                timestamps[timestamps.size() - 2]);
        SE3 source_from_destination_skipped =
            SE3::exp(timestamp_fraction * last_from_lbo.log());
        last_from_source = source_from_destination_skipped * last_from_source;
      }
    }
    last_from_lbo = last_from_source;
    SE3 first_from_source = odometry_first_from_body.back() * last_from_source;
    timestamps.push_back(source_timestamp);
    odometry_first_from_body.push_back(first_from_source);
  }

  LOG(INFO) << "the biggest skip in VO is "
            << static_cast<double>(max_skip) * 1.0e-6 << " seconds";
  LOG(INFO) << "there are " << skip_count << " skips in total";
}

void ReadRtk(const fs::path &rtk_filename, const SE3 &ins_from_body,
             bool correct_rtk, std::vector<Timestamp> &timestamps,
             StdVectorA<SE3> &rtkBodyToWorld) {
  std::ifstream rtk_file(rtk_filename);
  std::string cur_line;
  std::getline(rtk_file, cur_line);
  std::optional<SE3> first_from_world;
  while (std::getline(rtk_file, cur_line)) {
    Timestamp timestamp;
    double latitude, longitude, altitude;
    double northing, easting, down;
    double v_north, v_east, v_down;
    double roll, pitch, yaw;
    constexpr int kNeedToRead = 13;
    int num_read = sscanf(
        cur_line.c_str(),
        "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%*3c,%lf,%lf,%lf,%lf,%lf,%lf\n",
        &timestamp, &latitude, &longitude, &altitude, &northing, &easting,
        &down, &v_north, &v_east, &v_down, &roll, &pitch, &yaw);
    if (num_read != kNeedToRead) {
      LOG(WARNING) << "Read " << num_read << " instead of " << kNeedToRead
                   << " elements in RTK ground truth";
      break;
    }
    SE3 cur_world_from_body =
        correct_rtk
            ? Se3FromXyzrpy(-easting, northing, down, roll, pitch, yaw) *
                  ins_from_body
            : Se3FromXyzrpy(northing, easting, down, roll, pitch, yaw) *
                  ins_from_body;

    if (!first_from_world) {
      first_from_world.emplace(cur_world_from_body.inverse());
      cur_world_from_body = SE3();
    } else
      cur_world_from_body = first_from_world.value() * cur_world_from_body;

    timestamps.push_back(timestamp);
    rtkBodyToWorld.push_back(cur_world_from_body);
  }

  double time_covered = double(timestamps.back() - timestamps[0]) / 1.0e6;
  LOG(INFO) << "RTK data time covered = " << time_covered << " sec";
  LOG(INFO) << "RTK data num positions = " << timestamps.size()
            << "; avg time between positions = "
            << time_covered / timestamps.size();
}

void LogTimeInterval(const std::vector<Timestamp> &timestamps,
                     const std::string &name) {
  LOG(INFO) << name << ": [" << TimeOfDay(timestamps[0]) << " -- "
            << TimeOfDay(timestamps.back()) << "]"
            << " total items: " << timestamps.size();
}

std::string LogTimestamp(Timestamp timestamp) {
  return std::to_string(timestamp) + " (" + TimeOfDay(timestamp) + ")";
}

SE3 WorldFromBodyAtTimestampHelper(Timestamp timestamp,
                                   const StdVectorA<SE3> &bodyToWorld,
                                   const std::vector<Timestamp> &timestamps) {
  CHECK(timestamp >= timestamps[0] && timestamp <= timestamps.back())
      << "Interpolating outside of ground truth! "
      << "ts = " << LogTimestamp(timestamp) << ", bounds = ["
      << LogTimestamp(timestamps[0]) << ", " << LogTimestamp(timestamps.back())
      << "]";
  CHECK(timestamps.size() >= 2);
  if (timestamp == timestamps[0]) return bodyToWorld[0];

  int ind = std::lower_bound(timestamps.begin(), timestamps.end(), timestamp) -
            timestamps.begin();
  CHECK(ind > 0 && ind < bodyToWorld.size());
  SE3 low_from_high = bodyToWorld[ind - 1].inverse() * bodyToWorld[ind];
  double timestamp_fraction = double(timestamp - timestamps[ind - 1]) /
                              (timestamps[ind] - timestamps[ind - 1]);
  SE3 low_from_timestamp = SE3::exp(timestamp_fraction * low_from_high.log());
  return bodyToWorld[ind - 1] * low_from_timestamp;
}

// Here goes a large piece of code for timestamp synchronization between
// different cameras. The auxiliary functions might be moved to another place,
// but I don't know where yet.
constexpr int kInvalidPosition = -1;

Timestamp TimestampDistance(Timestamp a, Timestamp b) {
  return a < b ? b - a : a - b;
}

Timestamp TripletDistance(Timestamp x1, Timestamp x2, Timestamp x3) {
  return TimestampDistance(x1, x2) + TimestampDistance(x1, x3) +
         TimestampDistance(x2, x3);
}

std::vector<int> ClosestNotHigherIndices(const std::vector<Timestamp> &a,
                                         const std::vector<Timestamp> &b) {
  CHECK(!b.empty());
  std::vector<int> indices(a.size(), kInvalidPosition);
  int ind_b = 0, ind_a = 0;
  while (ind_b < b.size() && b[ind_b] > a[ind_a]) ind_a++;
  for (; ind_a < a.size(); ++ind_a) {
    if (ind_b + 1 == b.size() && a[ind_a] < b[ind_b]) break;
    while (ind_b + 1 < b.size() && b[ind_b + 1] <= a[ind_a]) ++ind_b;
    indices[ind_a] = ind_b;
  }
  return indices;
}

std::vector<int> ClosestNotLowerIndices(const std::vector<Timestamp> &a,
                                        const std::vector<Timestamp> &b) {
  CHECK(!b.empty());
  std::vector<int> indices(a.size(), kInvalidPosition);
  int ind_a = 0, ind_b = 0;
  while (ind_b < b.size() && b[ind_b] < a[ind_a]) ind_b++;
  for (; ind_a < a.size(); ++ind_a) {
    while (ind_b < b.size() && b[ind_b] < a[ind_a]) ++ind_b;
    if (ind_b == b.size()) break;
    indices[ind_a] = ind_b;
  }
  return indices;
}

std::vector<int> ClosestIndicesInB(const std::vector<Timestamp> &a,
                                   const std::vector<Timestamp> &b) {
  CHECK(!b.empty());
  std::vector<int> closest_to_b(a.size());
  int ind_b = 0;
  for (int ind_a = 0; ind_a < a.size(); ++ind_a) {
    while (ind_b + 1 < b.size() && b[ind_b + 1] < a[ind_a]) ++ind_b;
    if (ind_b + 1 == b.size()) {
      closest_to_b[ind_a] = ind_b;
      continue;
    }
    if (TimestampDistance(a[ind_a], b[ind_b]) <
        TimestampDistance(a[ind_a], b[ind_b + 1]))
      closest_to_b[ind_a] = ind_b;
    else
      closest_to_b[ind_a] = ++ind_b;
  }
  return closest_to_b;
}

void FilterBest(std::vector<Timestamp> &a, const std::vector<Timestamp> &b,
                const std::vector<Timestamp> &c, std::vector<int> &a_to_b,
                std::vector<int> &a_to_c) {
  std::vector<int> best_indices_a_in_b(b.size(), kInvalidPosition);
  std::vector<Timestamp> best_a_in_b(b.size());
  for (int ind_a = 0; ind_a < a.size(); ++ind_a) {
    Timestamp closeB = b[a_to_b[ind_a]], closeC = c[a_to_c[ind_a]];
    Timestamp curDist = TripletDistance(a[ind_a], closeB, closeC);
    if (best_indices_a_in_b[a_to_b[ind_a]] == kInvalidPosition ||
        curDist < best_a_in_b[a_to_b[ind_a]]) {
      best_a_in_b[a_to_b[ind_a]] = curDist;
      best_indices_a_in_b[a_to_b[ind_a]] = ind_a;
    }
  }

  std::vector<char> removed(a.size(), false);
  for (int ind_a = 0; ind_a < a.size(); ++ind_a) {
    if (best_indices_a_in_b[a_to_b[ind_a]] != ind_a) removed[ind_a] = true;
  }
  int new_ind_a = 0;
  for (int old_ind_a = 0; old_ind_a < a.size(); ++old_ind_a)
    if (!removed[old_ind_a]) {
      a[new_ind_a] = a[old_ind_a];
      a_to_b[new_ind_a] = a_to_b[old_ind_a];
      a_to_c[new_ind_a] = a_to_c[old_ind_a];
      ++new_ind_a;
    }
  a.resize(new_ind_a);
  a_to_b.resize(new_ind_a);
  a_to_c.resize(new_ind_a);
}

void FilterOutNoReference(std::vector<Timestamp> &v,
                          std::vector<int> &indices) {
  auto new_index_end = std::unique(indices.begin(), indices.end());
  CHECK_EQ(new_index_end - indices.begin(), indices.end() - indices.begin());
  int new_ind = 0;
  for (int i : indices) v[new_ind++] = v[i];
  v.resize(new_ind);
}

void CheckIndices(const std::vector<Timestamp> &a,
                  const std::vector<Timestamp> &b,
                  const std::vector<Timestamp> &c, int ind_a, int ind_b,
                  int ind_c, Timestamp &best_distance, int &best_ind_b,
                  int &best_ind_c) {
  if (ind_b == kInvalidPosition || ind_c == kInvalidPosition) return;
  Timestamp distance = TripletDistance(a[ind_a], b[ind_b], c[ind_c]);
  if (distance < best_distance) {
    best_distance = distance;
    best_ind_b = ind_b;
    best_ind_c = ind_c;
  }
}

void MostConsistentTriples(std::vector<Timestamp> &a, std::vector<Timestamp> &b,
                           std::vector<Timestamp> &c) {
  std::vector<int> lower_in_b = ClosestNotHigherIndices(a, b);
  std::vector<int> higher_in_b = ClosestNotLowerIndices(a, b);
  std::vector<int> lower_in_c = ClosestNotHigherIndices(a, c);
  std::vector<int> higher_in_c = ClosestNotLowerIndices(a, c);

  std::vector<int> a_to_b(a.size()), a_to_c(a.size());
  for (int ind_a = 0; ind_a < a.size(); ++ind_a) {
    Timestamp best_distance = std::numeric_limits<Timestamp>::max();
    int best_in_b = kInvalidPosition, best_in_c = kInvalidPosition;
    CheckIndices(a, b, c, ind_a, lower_in_b[ind_a], lower_in_c[ind_a],
                 best_distance, best_in_b, best_in_c);
    CheckIndices(a, b, c, ind_a, lower_in_b[ind_a], higher_in_c[ind_a],
                 best_distance, best_in_b, best_in_c);
    CheckIndices(a, b, c, ind_a, higher_in_b[ind_a], lower_in_c[ind_a],
                 best_distance, best_in_b, best_in_c);
    CheckIndices(a, b, c, ind_a, higher_in_b[ind_a], higher_in_c[ind_a],
                 best_distance, best_in_b, best_in_c);
    a_to_b[ind_a] = best_in_b;
    a_to_c[ind_a] = best_in_c;
  }

  FilterBest(a, b, c, a_to_b, a_to_c);
  FilterBest(a, c, b, a_to_c, a_to_b);
  FilterOutNoReference(b, a_to_b);
  FilterOutNoReference(c, a_to_c);
  CHECK_EQ(a.size(), b.size());
  CHECK_EQ(a.size(), c.size());
}

double AverageTripletDistance(const std::vector<Timestamp> &a,
                              const std::vector<Timestamp> &b,
                              const std::vector<Timestamp> &c) {
  double sum = 0;
  for (int i = 0; i < a.size(); ++i)
    sum += static_cast<double>(TripletDistance(a[i], b[i], c[i]));
  return sum / static_cast<double>(a.size());
}

}  // namespace

RobotcarReaderSettings::RobotcarReaderSettings() {
  // TODO can this be done via named initialization?
  camera_settings.unmap_polynomial_degree = 9;  // hand-tuned parameters
  camera_settings.unmap_polynomial_points = 60000;
}

bool RobotcarReader::IsRobotcar(const fs::path &segment_directory) {
  return fs::exists(segment_directory / "mono_left.timestamps") &&
         fs::exists(segment_directory / "mono_left") &&
         fs::exists(segment_directory / "mono_rear.timestamps") &&
         fs::exists(segment_directory / "mono_rear") &&
         fs::exists(segment_directory / "mono_right.timestamps") &&
         fs::exists(segment_directory / "mono_right") &&
         fs::exists(segment_directory / "lms_front.timestamps") &&
         fs::exists(segment_directory / "lms_front") &&
         fs::exists(segment_directory / "lms_rear.timestamps") &&
         fs::exists(segment_directory / "lms_rear") &&
         fs::exists(segment_directory / "ldmrs.timestamps") &&
         fs::exists(segment_directory / "ldmrs") &&
         fs::exists(segment_directory / "vo");
}

// clang-format off
const SE3 RobotcarReader::kImageFromCamera =
    SE3((Matrix44() <<
                    0, 0, 1, 0,
                    1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 0, 1).finished())
        .inverse();
// clang-format on

CameraBundle RobotcarReader::CreateCameraBundle(
    const fs::path &models_directory, const SE3 &left_from_body,
    const SE3 &rear_from_body, const SE3 &right_from_body, int w, int h,
    const CameraModelScaramuzzaSettings &camera_settings) {
  fs::path left_model = models_directory / "mono_left.txt";
  fs::path rear_model = models_directory / "mono_rear.txt";
  fs::path right_model = models_directory / "mono_right.txt";
  Camera cameras[RobotcarReader::kNumberOfCameras] = {
      Camera(w, h,
             CameraModelScaramuzza(w, h, left_model,
                                   CameraModelScaramuzza::kPolynomialUnmap,
                                   camera_settings)),
      Camera(w, h,
             CameraModelScaramuzza(w, h, rear_model,
                                   CameraModelScaramuzza::kPolynomialUnmap,
                                   camera_settings)),
      Camera(w, h,
             CameraModelScaramuzza(w, h, right_model,
                                   CameraModelScaramuzza::kPolynomialUnmap,
                                   camera_settings))};

  SE3 camera_from_body[RobotcarReader::kNumberOfCameras] = {
      left_from_body, rear_from_body, right_from_body};
  for (int i = 0; i < RobotcarReader::kNumberOfCameras; ++i)
    camera_from_body[i] =
        RobotcarReader::kImageFromCamera * camera_from_body[i];

  return CameraBundle(camera_from_body, cameras,
                      RobotcarReader::kNumberOfCameras);
}

RobotcarReader::RobotcarReader(const fs::path &segment_directory,
                               const fs::path &camera_models_directory,
                               const fs::path &extrinsics_directory,
                               const std::optional<fs::path> &rtk_directory,
                               const RobotcarReaderSettings &settings)
    : left_from_body_(
          ReadCameraFromBody(extrinsics_directory / "mono_left.txt")),
      rear_from_body_(
          ReadCameraFromBody(extrinsics_directory / "mono_rear.txt")),
      right_from_body_(
          ReadCameraFromBody(extrinsics_directory / "mono_right.txt")),
      lms_front_from_body_(
          ReadLidarFromBody(extrinsics_directory / "lms_front.txt")),
      lms_rear_from_body_(
          ReadLidarFromBody(extrinsics_directory / "lms_rear.txt")),
      ldmrs_from_body_(ReadLidarFromBody(extrinsics_directory / "ldmrs.txt")),
      ins_from_body_(ReadInsFromBody(extrinsics_directory / "ins.txt")),
      camera_bundle_(
          CreateCameraBundle(camera_models_directory, left_from_body_,
                             rear_from_body_, right_from_body_, kImageWidth,
                             kImageHeight, settings.camera_settings)),
      segment_directory_(segment_directory),
      left_directory_(segment_directory_ / fs::path("mono_left")),
      rear_directory_(segment_directory_ / fs::path("mono_rear")),
      right_directory_(segment_directory_ / fs::path("mono_right")),
      lms_front_directory_(segment_directory_ / fs::path("lms_front")),
      lms_rear_directory_(segment_directory_ / fs::path("lms_rear")),
      ldmrs_directory_(segment_directory_ / fs::path("ldmrs")),
      are_masks_provided_(false),
      settings_(settings) {
  CHECK(fs::is_directory(left_directory_));
  CHECK(fs::is_directory(rear_directory_));
  CHECK(fs::is_directory(right_directory_));
  CHECK(fs::is_directory(lms_front_directory_));
  CHECK(fs::is_directory(lms_rear_directory_));
  CHECK(fs::is_directory(ldmrs_directory_));

  ReadTimestamps(segment_directory_ / "mono_left.timestamps", left_timestamps_);
  ReadTimestamps(segment_directory_ / "mono_rear.timestamps", rear_timestamps_);
  ReadTimestamps(segment_directory_ / "mono_right.timestamps",
                 right_timestamps_);
  ReadTimestamps(segment_directory_ / "lms_front.timestamps",
                 lms_front_timestamps_);
  ReadTimestamps(segment_directory_ / "lms_rear.timestamps",
                 lms_rear_timestamps_);
  ReadTimestamps(segment_directory_ / "ldmrs.timestamps", ldmrs_timestamps_);

  bool is_rtk_found = false;
  if (rtk_directory) {
    fs::path rtk_filename =
        rtk_directory.value() / segment_directory_.filename() / "rtk.csv";
    if (fs::is_regular_file(rtk_filename)) {
      ReadRtk(rtk_filename, ins_from_body_, settings_.correct_rtk,
              ground_truth_pose_timestamps_, ground_truth_world_from_body_);
      is_rtk_found = true;
    }
  }

  ReadOdometryPoses(segment_directory_ / fs::path("vo") / fs::path("vo.csv"),
                    settings_.fill_odometry_gaps, odometry_pose_timestamps_,
                    odometry_world_from_body_);
  if (!is_rtk_found) {
    LOG(WARNING) << "No RTK ground truth for chunk \'" << segment_directory_
                 << "\' found";
    ReadOdometryPoses(segment_directory_ / fs::path("vo") / fs::path("vo.csv"),
                      settings_.fill_odometry_gaps,
                      ground_truth_pose_timestamps_,
                      ground_truth_world_from_body_);
  }

  LOG(INFO) << "Before timestamp synchronization:";
  LogTimeInterval(left_timestamps_, "left cam ");
  LogTimeInterval(rear_timestamps_, "rear cam ");
  LogTimeInterval(right_timestamps_, "right cam");

  SyncTimestamps();

  LOG(INFO) << "After timestamp synchronization:";
  LogTimeInterval(left_timestamps_, "left cam ");
  LogTimeInterval(rear_timestamps_, "rear cam ");
  LogTimeInterval(right_timestamps_, "right cam");
  LogTimeInterval(lms_rear_timestamps_, "LMS rear ");
  LogTimeInterval(lms_front_timestamps_, "LMS front");
  LogTimeInterval(ldmrs_timestamps_, "LDMRS    ");
  if (is_rtk_found) {
    LogTimeInterval(ground_truth_pose_timestamps_, "rtk      ");
    LogTimeInterval(odometry_pose_timestamps_, "vo       ");
  } else {
    LogTimeInterval(ground_truth_pose_timestamps_, "vo       ");
  }

  PrintOdometryAndGroundTruth(settings_.output_directory);
}

void RobotcarReader::PrintOdometryAndGroundTruth(
    const fs::path &output_directory) const {
  std::ofstream odometry_file(output_directory / "vo_traj.txt");
  std::ofstream ground_truth_file(output_directory / "gt_traj.txt");
  for (int i = 0; i < ground_truth_pose_timestamps_.size(); ++i) {
    if (ground_truth_pose_timestamps_[i] < odometry_pose_timestamps_[0] ||
        ground_truth_pose_timestamps_[i] >= odometry_pose_timestamps_.back())
      continue;
    PutInMatrixForm(ground_truth_file, ground_truth_world_from_body_[i]);
    PutInMatrixForm(odometry_file, WorldFromBodyAtTimestamp(
                                       ground_truth_pose_timestamps_[i], true));
  }
}

void RobotcarReader::ProvideMasks(const fs::path &masks_directory) {
  CHECK(fs::is_directory(masks_directory));
  fs::path left_mask_path = masks_directory / "mono_left.png";
  fs::path rear_mask_path = masks_directory / "mono_rear.png";
  fs::path right_mask_path = masks_directory / "mono_right.png";
  CHECK(fs::is_regular_file(left_mask_path));
  CHECK(fs::is_regular_file(rear_mask_path));
  CHECK(fs::is_regular_file(right_mask_path));
  cv::Mat3b left_mask = cv::imread(std::string(left_mask_path));
  cv::Mat3b rear_mask = cv::imread(std::string(rear_mask_path));
  cv::Mat3b right_mask = cv::imread(std::string(right_mask_path));
  camera_bundle_.camera(0).set_mask(ConvertBgrToGray(left_mask));
  camera_bundle_.camera(1).set_mask(ConvertBgrToGray(rear_mask));
  camera_bundle_.camera(2).set_mask(ConvertBgrToGray(right_mask));
  are_masks_provided_ = true;
}

int RobotcarReader::NumberOfFrames() const { return left_timestamps_.size(); }

int RobotcarReader::FirstTimestampToIndex(Timestamp timestamp) const {
  if (timestamp < left_timestamps_[0]) return 0;
  if (timestamp > left_timestamps_.back()) return NumberOfFrames();

  auto it = std::lower_bound(left_timestamps_.begin(), left_timestamps_.end(),
                             timestamp);
  if (it == left_timestamps_.end()) return NumberOfFrames();
  return static_cast<int>(it - left_timestamps_.begin());
}

std::vector<Timestamp> RobotcarReader::TimestampsFromIndex(
    int frame_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, NumberOfFrames());

  return std::vector{left_timestamps_[frame_index],
                     rear_timestamps_[frame_index],
                     right_timestamps_[frame_index]};
}

std::vector<fs::path> RobotcarReader::FrameFiles(int frame_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, NumberOfFrames());

  fs::path left_path =
      segment_directory_ / fs::path("mono_left") /
      fs::path(std::to_string(left_timestamps_[frame_index]) + ".png");
  fs::path rear_path =
      segment_directory_ / fs::path("mono_rear") /
      fs::path(std::to_string(rear_timestamps_[frame_index]) + ".png");
  fs::path right_path =
      segment_directory_ / fs::path("mono_right") /
      fs::path(std::to_string(right_timestamps_[frame_index]) + ".png");

  return {left_path, rear_path, right_path};
}

std::vector<RobotcarReader::FrameEntry> RobotcarReader::Frame(
    int frame_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, NumberOfFrames());
  std::vector<fs::path> filenames = FrameFiles(frame_index);
  fs::path left_path = filenames[0];
  fs::path rear_path = filenames[1];
  fs::path right_path = filenames[2];

  if (fs::is_regular_file(left_path) && fs::is_regular_file(rear_path) &&
      fs::is_regular_file(right_path)) {
    // Can fail if image is corrupted or not read correctly for whatever reason
    cv::Mat1b left_original =
        cv::imread(left_path.native(), cv::IMREAD_GRAYSCALE);
    cv::Mat1b rear_original =
        cv::imread(rear_path.native(), cv::IMREAD_GRAYSCALE);
    cv::Mat1b right_original =
        cv::imread(right_path.native(), cv::IMREAD_GRAYSCALE);

    std::vector<FrameEntry> result(kNumberOfCameras);
    cv::cvtColor(left_original, result[0].frame, cv::COLOR_BayerBG2BGR);
    result[0].timestamp = left_timestamps_[frame_index];
    cv::cvtColor(rear_original, result[1].frame, cv::COLOR_BayerBG2BGR);
    result[1].timestamp = rear_timestamps_[frame_index];
    cv::cvtColor(right_original, result[2].frame, cv::COLOR_BayerBG2BGR);
    result[2].timestamp = right_timestamps_[frame_index];

    return result;
  } else {
    throw std::runtime_error("Image paths for frame contain invalid paths");
  }
}

std::unique_ptr<FrameDepths> RobotcarReader::GetDepths(int frame_index) const {
  LOG(ERROR) << "RobotcarReader::depths not implemented yet!";
  return std::unique_ptr<FrameDepths>();
}

bool RobotcarReader::HasWorldFromFrame(int frame_index) const {
  if (frame_index < 0 || frame_index >= NumberOfFrames()) return false;
  Timestamp timestamp = TimestampsFromIndex(frame_index)[0];
  return timestamp >= ground_truth_pose_timestamps_[0] &&
         timestamp <= ground_truth_pose_timestamps_.back();
}

SE3 RobotcarReader::WorldFromFrame(int frame_index) const {
  CHECK_GE(frame_index, 0);
  CHECK_LT(frame_index, NumberOfFrames());

  Timestamp timestamp = TimestampsFromIndex(frame_index)[0];
  CHECK_GE(timestamp, ground_truth_pose_timestamps_[0]);
  CHECK_LT(timestamp, ground_truth_pose_timestamps_.back());
  return WorldFromBodyAtTimestamp(timestamp);
}

Trajectory RobotcarReader::GroundTruthTrajectory() const {
  StdMapA<Timestamp, SE3> timestamped_world_from_frame;
  for (int i = 0; i < ground_truth_pose_timestamps_.size(); ++i)
    timestamped_world_from_frame[ground_truth_pose_timestamps_[i]] =
        ground_truth_world_from_body_[i];
  return Trajectory{timestamped_world_from_frame};
}

SE3 RobotcarReader::WorldFromBodyAtTimestamp(Timestamp timestamp,
                                             bool use_odometry) const {
  if (use_odometry)
    return WorldFromBodyAtTimestampHelper(timestamp, odometry_world_from_body_,
                                          odometry_pose_timestamps_);
  else
    return WorldFromBodyAtTimestampHelper(timestamp,
                                          ground_truth_world_from_body_,
                                          ground_truth_pose_timestamps_);
}

void RobotcarReader::GetPointCloudHelper(
    const fs::path &scan_directory, const SE3 &body_from_sensor, Timestamp base,
    const std::vector<Timestamp> &timestamps, Timestamp from, Timestamp to,
    bool is_ldmrs, std::vector<Vector3> &cloud) const {
  int ind_from = std::lower_bound(timestamps.begin(), timestamps.end(), from) -
                 timestamps.begin();
  int ind_to = std::upper_bound(timestamps.begin(), timestamps.end(), to) -
               timestamps.begin();

  LOG(INFO) << "index bounds of the cloud: [" << ind_from << ", " << ind_to
            << "]";

  int current_percentage = 0;
  int total_scans = ind_to - ind_from;
  std::cout << "scanning the cloud from " << scan_directory << ": 0% ...";
  std::cout.flush();
  for (int i = ind_from; i < ind_to; ++i) {
    SE3 base_from_sensor = WorldFromBodyAtTimestamp(base).inverse() *
                           WorldFromBodyAtTimestamp(timestamps[i]) *
                           body_from_sensor;
    fs::path scan_filename =
        scan_directory / fs::path(std::to_string(timestamps[i]) + ".bin");
    std::vector<double> data = ReadDoublesFromBin(scan_filename);
    for (int j = 0; j + 2 < data.size(); j += 3) {
      double x, y, z, reflectance;
      if (is_ldmrs) {
        x = data[j];
        y = data[j + 1];
        z = data[j + 2];
      } else {
        x = data[j];
        y = data[j + 1];
        z = 0;
        reflectance = data[j + 2];
      }
      cloud.push_back(base_from_sensor * Vector3(x, y, z));
    }

    int new_percentage = (i + 1 - ind_from) * 100 / total_scans;
    if (new_percentage % 20 == 0 && new_percentage != current_percentage) {
      current_percentage = new_percentage;
      std::cout << " " << new_percentage << "%";
      std::cout.flush();
      if (new_percentage != 100)
        std::cout << " ...";
      else
        std::cout << std::endl;
    }
  }
}

std::vector<Vector3> RobotcarReader::GetLmsFrontCloud(Timestamp from,
                                                      Timestamp to,
                                                      Timestamp base) const {
  std::vector<Vector3> cloud;
  GetPointCloudHelper(lms_front_directory_, lms_front_from_body_.inverse(),
                      base, lms_front_timestamps_, from, to, false, cloud);
  return cloud;
}

std::vector<Vector3> RobotcarReader::GetLmsRearCloud(Timestamp from,
                                                     Timestamp to,
                                                     Timestamp base) const {
  std::vector<Vector3> cloud;
  GetPointCloudHelper(lms_rear_directory_, lms_rear_from_body_.inverse(), base,
                      lms_rear_timestamps_, from, to, false, cloud);
  return cloud;
}

std::vector<Vector3> RobotcarReader::GetLdmrsCloud(Timestamp from, Timestamp to,
                                                   Timestamp base) const {
  std::vector<Vector3> cloud;
  GetPointCloudHelper(ldmrs_directory_, ldmrs_from_body_.inverse(), base,
                      ldmrs_timestamps_, from, to, true, cloud);
  return cloud;
}

std::array<StdVectorA<std::pair<Vector2, double>>,
           RobotcarReader::kNumberOfCameras>
RobotcarReader::project(Timestamp from, Timestamp to, Timestamp base,
                        bool use_lms_front, bool use_lms_rear,
                        bool use_ldmrs) const {
  LOG(INFO) << "projection time window: [" << TimeOfDay(from) << ", "
            << TimeOfDay(to) << "]";
  std::vector<Vector3> lms_front_cloud;
  if (use_lms_front) lms_front_cloud = GetLmsFrontCloud(from, to, base);
  std::vector<Vector3> lms_rear_cloud;
  if (use_lms_rear) lms_rear_cloud = GetLmsRearCloud(from, to, base);
  std::vector<Vector3> ldmrs_cloud;
  if (use_ldmrs) ldmrs_cloud = GetLdmrsCloud(from, to, base);
  std::vector<Vector3> cloud;
  cloud.reserve(lms_front_cloud.size() + lms_rear_cloud.size() +
                ldmrs_cloud.size());
  cloud.insert(cloud.end(), lms_front_cloud.begin(), lms_front_cloud.end());
  cloud.insert(cloud.end(), lms_rear_cloud.begin(), lms_rear_cloud.end());
  cloud.insert(cloud.end(), ldmrs_cloud.begin(), ldmrs_cloud.end());
  return project(cloud);
}

std::array<StdVectorA<std::pair<Vector2, double>>,
           RobotcarReader::kNumberOfCameras>
RobotcarReader::project(const std::vector<Vector3> &cloud) const {
  std::array<StdVectorA<std::pair<Vector2, double>>,
             RobotcarReader::kNumberOfCameras>
      result;
  for (int ci = 0; ci < RobotcarReader::kNumberOfCameras; ++ci) {
    SE3 bodyToCam = camera_bundle_.camera_from_body(ci);
    for (const Vector3 &p : cloud) {
      Vector3 moved = bodyToCam * p;
      if (!camera_bundle_.camera(ci).IsMappable(moved)) continue;
      double depth = moved.norm();
      Vector2 projected = camera_bundle_.camera(ci).Map(moved);
      result[ci].push_back({projected, depth});
    }
  }
  return result;
}

Timestamp RobotcarReader::min_timestamp() const {
  return std::max({left_timestamps_[0], rear_timestamps_[0],
                   right_timestamps_[0], ground_truth_pose_timestamps_[0],
                   lms_front_timestamps_[0], lms_rear_timestamps_[0],
                   ldmrs_timestamps_[0]});
}

Timestamp RobotcarReader::max_timestamp() const {
  return std::min({left_timestamps_.back(), rear_timestamps_.back(),
                   right_timestamps_.back(),
                   ground_truth_pose_timestamps_.back(),
                   lms_front_timestamps_.back(), lms_rear_timestamps_.back(),
                   ldmrs_timestamps_.back()});
}

void RobotcarReader::SyncTimestamps() {
  int old_size = left_timestamps_.size();
  double old_average_triplet_distance = AverageTripletDistance(
      left_timestamps_, rear_timestamps_, right_timestamps_);
  MostConsistentTriples(left_timestamps_, rear_timestamps_, right_timestamps_);
  CHECK_EQ(left_timestamps_.size(), right_timestamps_.size());
  CHECK_EQ(left_timestamps_.size(), rear_timestamps_.size());
  int new_size = left_timestamps_.size();
  double new_average_triplet_distance = AverageTripletDistance(
      left_timestamps_, rear_timestamps_, right_timestamps_);
  LOG(INFO) << "triples filtered out for syncing timestamps: "
            << old_size - new_size;
  LOG(INFO) << "old avg tridist (s) = " << old_average_triplet_distance / 1.0e6;
  LOG(INFO) << "new avg tridist (s) = " << new_average_triplet_distance / 1.0e6;
}

}  // namespace grpose