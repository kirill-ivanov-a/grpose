#include "dataset/autovision_reader.h"

#include <map>

#include <fmt/ranges.h>
#include <boost/property_tree/json_parser.hpp>

#include "util/util.h"

namespace grpose {

namespace pt = boost::property_tree;

AutovisionReaderSettings::AutovisionReaderSettings(
    const fs::path &config_filename) {
  pt::ptree config_tree;
  pt::read_json(config_filename, config_tree);
  for (const pt::ptree::value_type &camera_node :
       config_tree.get_child("camera_names"))
    camera_names.push_back(camera_node.second.get_value<std::string>());
  LOG(INFO) << fmt::format("Using camera names: [{}]",
                           fmt::join(camera_names, ", "));
}

bool AutovisionReader::IsAutovision(const fs::path &segment_directory) {
  bool is_autovision = fs::exists(segment_directory / "gt_poses.txt") &&
                       fs::exists(segment_directory / "timestamps.txt");
  return is_autovision;
}

AutovisionReader::AutovisionReader(const fs::path &segment_directory,
                                   const fs::path &calibration_directory,
                                   const fs::path &config_filename)
    : settings_(config_filename),
      segment_directory_(segment_directory),
      camera_bundle_(
          CreateCameraBundle(calibration_directory, settings_.camera_names)) {
  CHECK(IsAutovision(segment_directory));
  ReadTimestamps(segment_directory / "timestamps.txt");
  ReadGroundTruth(segment_directory / "gt_poses.txt");
}

int AutovisionReader::NumberOfFrames() const { return frame_ids_.size(); }

int AutovisionReader::FirstTimestampToIndex(Timestamp timestamp) const {
  if (timestamp < frame_timestamps_[0]) return 0;
  if (timestamp > frame_timestamps_.back()) return NumberOfFrames();

  auto it = std::lower_bound(frame_timestamps_.begin(), frame_timestamps_.end(),
                             timestamp);
  return it - frame_timestamps_.begin();
}

std::vector<Timestamp> AutovisionReader::TimestampsFromIndex(
    int frame_index) const {
  if (frame_index < 0 || frame_index >= NumberOfFrames())
    throw std::out_of_range(
        fmt::format("AutovisionReader::TimestampsFromIndex: Index {:d} out of "
                    "range [0, {:d})",
                    frame_index, NumberOfFrames()));

  // All cameras have the same timestamp for autovision
  return std::vector<Timestamp>(camera_bundle_.NumberOfCameras(),
                                frame_timestamps_[frame_index]);
}

std::vector<fs::path> AutovisionReader::FrameFiles(int frame_index) const {
  if (frame_index < 0 || frame_index >= NumberOfFrames())
    throw std::out_of_range(fmt::format(
        "AutovisionReader::FrameFiles: Index {:d} out of range [0, {:d})",
        frame_index, NumberOfFrames()));

  std::string image_filename =
      fmt::format("{:08d}.jpg", frame_ids_[frame_index]);

  std::vector<fs::path> image_paths;
  for (const auto &cam_name : settings_.camera_names)
    image_paths.push_back(segment_directory_ / cam_name / image_filename);
  return image_paths;
}

std::vector<AutovisionReader::FrameEntry> AutovisionReader::Frame(
    int frame_index) const {
  if (frame_index < 0 || frame_index >= NumberOfFrames())
    throw std::out_of_range(fmt::format(
        "AutovisionReader::Frame: Index {:d} out of range [0, {:d})",
        frame_index, NumberOfFrames()));
  std::vector<fs::path> filenames = FrameFiles(frame_index);
  CHECK_EQ(filenames.size(), camera_bundle_.NumberOfCameras());

  std::vector<AutovisionReader::FrameEntry> result(
      camera_bundle_.NumberOfCameras());
  for (int c = 0; c < camera_bundle_.NumberOfCameras(); c++) {
    if (fs::is_regular_file(filenames[c])) {
      cv::Mat1b img = cv::imread(filenames[c].string(), cv::IMREAD_GRAYSCALE);
      cv::cvtColor(img, result[c].frame, cv::COLOR_GRAY2BGR);
      result[c].timestamp = frame_timestamps_[frame_index];
    } else {
      throw std::runtime_error(
          fmt::format("AutovisionReader::Frame: '{:s}' was not a regular file",
                      filenames[c]));
    }
  }
  return result;
}

CameraBundle AutovisionReader::GetCameraBundle() const {
  return camera_bundle_;
}

std::unique_ptr<FrameDepths> AutovisionReader::GetDepths(
    int frame_index) const {
  LOG(ERROR) << "AutovisionReader::GetDepths not implemented yet!";
  return std::unique_ptr<FrameDepths>();
}

bool AutovisionReader::HasWorldFromFrame(int frame_index) const {
  return frame_index >= 0 && frame_index < NumberOfFrames();
}

SE3 AutovisionReader::WorldFromFrame(int frame_index) const {
  if (frame_index < 0 || frame_index >= NumberOfFrames())
    throw std::out_of_range(fmt::format(
        "AutovisionReader::WorldFromFrame: Index {:d} out of range [0, {:d})",
        frame_index, NumberOfFrames()));
  return ground_truth_world_from_body_[frame_index];
}

Trajectory AutovisionReader::GroundTruthTrajectory() const {
  StdMapA<Timestamp, SE3> timestampedWorldFromFrame;
  for (int i = 0; i < frame_timestamps_.size(); ++i)
    timestampedWorldFromFrame[frame_timestamps_[i]] =
        ground_truth_world_from_body_[i];
  return Trajectory{timestampedWorldFromFrame};
}

void AutovisionReader::ReadTimestamps(const fs::path &timestamps_filename) {
  CHECK(fs::exists(timestamps_filename))
      << timestamps_filename.native() + " does not exist";
  std::ifstream ifs(timestamps_filename);
  std::string header_line;  // skip first line which is the header
  std::getline(ifs, header_line);

  int frame_id;
  Timestamp ts;
  while (ifs >> frame_id >> ts) {
    CHECK(frame_timestamps_.empty() || frame_timestamps_.back() < ts);
    frame_timestamps_.push_back(ts);
    frame_ids_.push_back(frame_id);
  }

  CHECK_GT(frame_ids_.size(), 0);
}

void AutovisionReader::ReadGroundTruth(const fs::path &ground_truth_filename) {
  CHECK(fs::exists(ground_truth_filename))
      << ground_truth_filename.native() + " does not exist";
  std::ifstream ifs(ground_truth_filename);
  std::string header_line;  // skip first line which is the header
  std::getline(ifs, header_line);

  Timestamp ts;
  double qw, qx, qy, qz, tx, ty, tz;
  int Q;  // no idea what this value is but it's in the GT file
  while (ifs >> ts >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> Q) {
    /*Ideally find frame index with the closest
     * timestamp but for the provided autovision chunks we know that
     * the ground truth and timestamp files are aligned
     */
    ground_truth_world_from_body_.push_back(
        SE3(Eigen::Quaternion(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz)));
  }
}

namespace {

MatrixXX ReadMatrix(const pt::ptree &matrix_tree) {
  const int rows = matrix_tree.get<int>("rows");
  const int cols = matrix_tree.get<int>("cols");
  CHECK_GT(rows, 0);
  CHECK_GT(cols, 0);

  MatrixXX result(rows, cols);
  int i = 0;
  for (const pt::ptree::value_type &value_node :
       matrix_tree.get_child("data")) {
    const double value = value_node.second.get_value<double>();
    result(i / cols, i % cols) = value;
    i++;
  }

  return result;
}

SE3 ReadSe3(const pt::ptree &se3_tree) {
  const MatrixXX quaternion_matrix = ReadMatrix(se3_tree.get_child("rotation"));
  const MatrixXX translation_matrix =
      ReadMatrix(se3_tree.get_child("translation"));
  CHECK_EQ(quaternion_matrix.rows(), 4);
  CHECK_EQ(quaternion_matrix.cols(), 1);
  CHECK_EQ(translation_matrix.rows(), 3);
  CHECK_EQ(translation_matrix.cols(), 1);

  return SE3(Quaternion(quaternion_matrix.data()), translation_matrix);
}

StdMapA<std::string, SE3> ReadBodyFromSensorJson(
    const fs::path &extrinsics_filename) {
  CHECK(fs::exists(extrinsics_filename))
      << extrinsics_filename.string() << " does not exist";

  pt::ptree extrinsics_tree;
  pt::read_json(extrinsics_filename, extrinsics_tree);

  StdMapA<std::string, SE3> body_from_camera_map;
  for (const pt::ptree::value_type &sensor_tree :
       extrinsics_tree.get_child("NSensorSystem.sensorMap")) {
    const std::string sensor_name = sensor_tree.second.get<std::string>("key");
    const SE3 body_from_sensor =
        ReadSe3(sensor_tree.second.get_child("value.transformation"));
    body_from_camera_map.emplace(sensor_name, body_from_sensor);
  }

  return body_from_camera_map;
}

std::pair<std::string, Camera> ReadNameAndCamera(const pt::ptree &camera_tree) {
  const std::string camera_name =
      camera_tree.get<std::string>("ptr_wrapper.data.Properties.Name");
  const int image_height =
      camera_tree.get<int>("ptr_wrapper.data.Properties.ImageHeight");
  const int image_width =
      camera_tree.get<int>("ptr_wrapper.data.Properties.ImageWidth");
  pt::ptree model_tree = camera_tree.get_child("ptr_wrapper.data.CameraModel");
  const std::string distortion_model_type =
      model_tree.get<std::string>("Distortion.DistortionModelType");
  const std::string projection_model_type =
      model_tree.get<std::string>("ProjectionModelType");
  CHECK_EQ(distortion_model_type, "radial_tangential");
  CHECK_EQ(projection_model_type, "unified");
  const double k1 = model_tree.get<double>("Distortion.k1");
  const double k2 = model_tree.get<double>("Distortion.k2");
  const double p1 = model_tree.get<double>("Distortion.p1");
  const double p2 = model_tree.get<double>("Distortion.p2");
  const double xi = model_tree.get<double>("xi");
  const double fu = model_tree.get<double>("fu");
  const double fv = model_tree.get<double>("fv");
  const double cu = model_tree.get<double>("cu");
  const double cv = model_tree.get<double>("cv");
  const std::vector<double> parameters = {fu, fv, cu, cv, k1, k2, p1, p2, xi};
  return {camera_name, Camera(image_width, image_height,
                              CameraModelId::kUnified, parameters)};
}

StdMapA<std::string, Camera> ReadIntrinsicsJson(
    const fs::path &intrinsics_filename) {
  CHECK(fs::exists(intrinsics_filename))
      << intrinsics_filename.string() << " does not exist";

  pt::ptree intrinsics_tree;
  pt::read_json(intrinsics_filename, intrinsics_tree);

  StdMapA<std::string, Camera> camera_map;
  for (const pt::ptree::value_type &camera_tree :
       intrinsics_tree.get_child("NCameraSystem.cameras")) {
    camera_map.emplace(ReadNameAndCamera(camera_tree.second));
  }

  return camera_map;
}

}  // namespace

CameraBundle AutovisionReader::CreateCameraBundle(
    const fs::path &calibration_directory,
    const std::vector<std::string> &camera_names) {
  CHECK(fs::is_directory(calibration_directory))
      << calibration_directory.string() << " doesn't exist!";

  fs::path extrinsics_filename =
      calibration_directory / "sensor_system_cal.json";
  fs::path intrinsics_filename =
      calibration_directory / "camera_system_cal.json";

  StdMapA<std::string, SE3> body_from_camera_map =
      ReadBodyFromSensorJson(extrinsics_filename);
  StdMapA<std::string, Camera> cameras_map =
      ReadIntrinsicsJson(intrinsics_filename);
  StdVectorA<SE3> camera_from_body;
  StdVectorA<Camera> cameras;
  for (const std::string &camera_name : camera_names) {
    auto pose_it = body_from_camera_map.find(camera_name);
    CHECK(pose_it != body_from_camera_map.end());
    // note that the extrinsics provide camera -> body, but the CameraBundle
    // constructor takes body -> camera
    camera_from_body.push_back(body_from_camera_map[camera_name].inverse());

    auto camera_it = cameras_map.find(camera_name);
    CHECK(camera_it != cameras_map.end());
    cameras.push_back(camera_it->second);
  }

  CameraBundle camera_bundle(camera_from_body.data(), cameras.data(),
                             camera_from_body.size());

  for (int i = 0; i < camera_names.size(); ++i) {
    fs::path mask_filename =
        calibration_directory / fmt::format("mask_{:s}.png", camera_names[i]);
    CHECK(fs::exists(mask_filename)) << mask_filename << " does not exist";
    cv::Mat3b mask = cv::imread(mask_filename);
    camera_bundle.camera(i).set_mask(ConvertBgrToGray(mask));
  }

  return camera_bundle;
}

}  // namespace grpose
